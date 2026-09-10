"""
core/slam/record.py

**What the localiser held, kept so it can be drawn afterwards.**

`slam.csv` records eighteen scalars per control step and answers every question
that is a number. It answers none of the questions that are a shape: does the
map look like lab42, does the estimate track the truth or wander off down a
corridor, what does the trajectory do at the instant a closure lands. Chunk 24b
exists for those, and this module is the half of it that lives in `core/`.

OPT-IN, AND INERT — THE SAME RULE `nodes/trace.py` FOLLOWS
-----------------------------------------------------------
`Tree.enable_trace()` is the precedent and it is a strong one: a traced run
produces a byte-identical `trace.csv`, because a recording that changed the run
would make every picture drawn from it evidence about a different episode.
`SlamLocaliser.enable_recording()` is the same promise. Nothing here draws from
the rng, nothing here feeds back into an estimate, and `core/tests/
test_slam_record.py` pins it by running one episode twice and diffing.

The cost is memory, and it is bounded by the map: one copy of every scan point,
in float64, plus three floats per graph node per *change* to the graph. A 240 s
held-out episode at 5 Hz is ~1100 scans of ~180 points, so ~3 MB.

WHY LOCAL POINTS AND A GRAPH HISTORY, RATHER THAN SNAPSHOTS OF THE WHOLE MAP
-----------------------------------------------------------------------------
Because it is the same structural fact `submap.py` is built on, and reusing it
buys the interesting frames for free.

A submap stores its points in its OWN frame; a closure moves the submap's
`origin` and the points come with it, unchanged, which is why relocating is
cheap. So the map at any instant is exactly

    points_in_world = transform(local_points_of_submap_k, graph_poses[k])

for whichever graph poses were current at that instant. Recording the points
once and the poses per change therefore reproduces the map at **every** step of
the episode, exactly, not just at the end — and that is what makes the map
visibly SNAP when a loop closes, which is the single most informative frame in
any SLAM animation and the one thing a final-state picture cannot show.

⚠️ Storing world-frame points per scan instead would be both larger and WRONG:
it would freeze each point at the pose the robot believed when it was measured,
so the drawn map would keep the pre-closure error forever while the trajectory
corrected around it.

THE TIME AXIS IS NOT IN HERE
----------------------------
Everything is keyed by **scan index**, never by `t`. The localiser is not handed
a clock — `update()` takes a `dt` — and inventing one here would mean a second,
independently-accumulated time base that could drift from the trace's. The join
is `slam.csv`'s own `scans` column: the first row where `scans == k` is the
control step scan `k` was taken at. `scripts/visualise_slam.py` does exactly
that and nothing else.

SIM-FREE. numpy only — no matplotlib, no PIL. Rendering is `scripts/`.
"""

import copy

import numpy as np

from core.slam import scan as scan_mod

# ---- below this line is data, not code.

# Why a graph snapshot was taken. Stored as a small int so the whole history is
# one array; the renderer shows solves differently from node additions, because
# a solve is the moment the map moves and a node addition is routine.
NODE_ADDED = 0
SOLVED = 1

# Edge kinds, as ints. `core/slam/graph.py` carries them as the strings
# ODOMETRY / CLOSURE; they are mapped here rather than stored as strings because
# an .npz of fixed-width numbers is a file anything can read, and because the
# renderer branches on exactly this distinction.
EDGE_ODOMETRY = 0
EDGE_CLOSURE = 1

# The keys `save_npz` writes. Listed once, so `load_npz` can say which one is
# missing rather than raising a KeyError from inside numpy.
ARRAYS = (
    "points", "point_submap", "point_scan",
    "edge_i", "edge_j", "edge_kind", "edge_scan", "edge_info",
    "hist_scan", "hist_n", "hist_reason", "hist_poses",
    "final_poses", "start_pose", "meta_values",
)

# ---- below this line is code, not data.


class MapRecorder:
    """Everything geometric the localiser built, in the order it built it.

    One episode, one recorder — `SlamLocaliser.reset()` makes a new one, for the
    same reason it rebuilds the graph: a submap surviving into the next episode
    is a correlation between episodes the four-condition ablation may not have.
    """

    def __init__(self, start_pose=(0.0, 0.0, 0.0), meta=None):
        self.start_pose = tuple(float(v) for v in start_pose)
        self.meta = dict(meta or {})
        self.scans = 0

        # Per scan-point-block, parallel lists. Kept as lists of arrays and
        # concatenated once in `save_npz`, because appending to a growing numpy
        # array is quadratic and a 240 s episode has ~1100 blocks.
        self._blocks = []            # (N, 2) float64, SUBMAP frame
        self._block_submap = []      # which submap the block landed in
        self._block_scan = []        # the scan index it was taken at

        self._edges = []             # (i, j, kind, scan, info_x, info_y, info_t)
        self._hist = []              # (scan, reason, poses (K, 3))
        self._final_poses = np.zeros((0, 3))

    # -- capture -----------------------------------------------------------

    def scan(self, submap_index, local_points):
        """One scan's points, already in the frame of submap `submap_index`.

        Taken in the submap frame rather than the world frame deliberately —
        see the module docstring. The localiser has them in that frame anyway,
        because that is what `Submap.add` stores, so this is a copy and not a
        transform.
        """
        pts = np.asarray(local_points, dtype=float).reshape(-1, 2)
        # ⚠️ Incremented BEFORE the block is tagged, so scan indices are
        # 1-based and read "the number of scans taken when this happened" --
        # which is exactly what `slam.csv`'s own `scans` column holds on the
        # step this scan was taken. The join between the two files is then a
        # plain equality and never an off-by-one.
        self.scans += 1
        self._blocks.append(pts.copy())
        self._block_submap.append(int(submap_index))
        self._block_scan.append(int(self.scans))

    def edge(self, i, j, kind, info):
        """One graph edge, tagged with the scan it appeared at.

        The scan index is what lets the animation reveal a closure at the moment
        it is accepted rather than drawing all of them from frame one.
        """
        self._edges.append((int(i), int(j),
                            EDGE_CLOSURE if kind == "closure" else EDGE_ODOMETRY,
                            int(self.scans),
                            float(info[0]), float(info[1]), float(info[2])))

    def graph(self, poses, reason):
        """A snapshot of every node pose. Called only when the graph CHANGED.

        Recording per change rather than per scan is what keeps the history
        small, and it loses nothing: between changes the poses are by definition
        identical, so `poses_at` forward-fills and is exact rather than
        interpolated.
        """
        self._hist.append((int(self.scans), int(reason),
                           np.asarray(poses, dtype=float).reshape(-1, 3).copy()))
        self._final_poses = self._hist[-1][2]

    # -- read back ---------------------------------------------------------

    @property
    def n_submaps(self):
        return self._final_poses.shape[0]

    def poses_at(self, scan):
        """The graph as it stood when scan `scan` had just been taken.

        `scan` is `slam.csv`'s `scans` column, 1-based; `poses_at(0)` is the
        anchor alone, snapshotted by `reset()` before any scan. Forward-filled
        from the last snapshot at or before `scan`, which is exact rather than
        interpolated: between snapshots the poses are by definition unchanged.
        """
        out = np.zeros((0, 3))
        for at, _reason, poses in self._hist:
            if at > scan:
                break
            out = poses
        return out

    def points_at(self, scan, poses=None):
        """Every point measured up to and including `scan`, in the WORLD frame.

        Placed with the graph as it stood at `scan` — so the same point moves
        between two calls with different `scan`, which is the correction a loop
        closure applies and exactly what the picture is for.
        """
        if poses is None:
            poses = self.poses_at(scan)
        if not len(poses) or not self._blocks:
            return np.zeros((0, 2))
        out = []
        for pts, k, at in zip(self._blocks, self._block_submap, self._block_scan):
            if at > scan or k >= len(poses) or not len(pts):
                continue
            out.append(scan_mod.transform(pts, tuple(poses[k])))
        if not out:
            return np.zeros((0, 2))
        return np.concatenate(out, axis=0)

    def closures(self):
        """(scan, i, j) for every closure edge, in the order they were accepted."""
        return [(e[3], e[0], e[1]) for e in self._edges
                if e[2] == EDGE_CLOSURE]

    def solve_scans(self):
        """The scan indices at which the graph was re-solved."""
        return [at for at, reason, _ in self._hist if reason == SOLVED]

    # -- persistence -------------------------------------------------------

    def save_npz(self, path):
        """One compressed .npz beside `slam.csv`.

        Compressed rather than raw: the points are the bulk and lidar endpoints
        against a room with six walls repeat heavily. `savez_compressed` is
        deterministic for the same arrays, so a recorded run is still diffable.
        """
        arrays = self._arrays()
        with open(path, "wb") as fh:
            np.savez_compressed(fh, **arrays)
        return path

    def _arrays(self):
        blocks = self._blocks or [np.zeros((0, 2))]
        points = np.concatenate(blocks, axis=0)
        point_submap = np.concatenate(
            [np.full(len(p), k, dtype=np.int32)
             for p, k in zip(blocks, self._block_submap or [0])])
        point_scan = np.concatenate(
            [np.full(len(p), s, dtype=np.int32)
             for p, s in zip(blocks, self._block_scan or [0])])

        edges = np.array(self._edges, dtype=float).reshape(-1, 7)

        # The history is ragged — the graph grows a node at a time — so it is
        # padded to the final node count and the true width carried alongside.
        # Padding rather than a ragged object array because an object array in
        # an .npz needs `allow_pickle`, and a file that needs `allow_pickle` to
        # read is a file this repo should not be writing.
        width = max([len(p) for _at, _r, p in self._hist] + [1])
        hist_poses = np.zeros((len(self._hist), width, 3))
        for row, (_at, _r, poses) in enumerate(self._hist):
            hist_poses[row, :len(poses)] = poses

        return {
            "points": points,
            "point_submap": point_submap,
            "point_scan": point_scan,
            "edge_i": edges[:, 0].astype(np.int32),
            "edge_j": edges[:, 1].astype(np.int32),
            "edge_kind": edges[:, 2].astype(np.int8),
            "edge_scan": edges[:, 3].astype(np.int32),
            "edge_info": edges[:, 4:7],
            "hist_scan": np.array([h[0] for h in self._hist], dtype=np.int32),
            "hist_reason": np.array([h[1] for h in self._hist], dtype=np.int8),
            "hist_n": np.array([len(h[2]) for h in self._hist], dtype=np.int32),
            "hist_poses": hist_poses,
            "final_poses": self._final_poses,
            "start_pose": np.array(self.start_pose, dtype=float),
            # Scalars travel as a (K, 2) array of name/value strings rather than
            # as pickled objects, for the `allow_pickle` reason above.
            "meta_values": np.array(
                [[str(k), str(v)] for k, v in sorted(self.meta.items())],
                dtype="<U40").reshape(-1, 2),
        }


class RecordedMap:
    """A `MapRecorder` read back from disk. Same read-back API, no capture.

    Separate from `MapRecorder` rather than a classmethod on it because the two
    have genuinely different jobs: one is fed by a live localiser and must be
    cheap to append to, the other is handed to a renderer and must be cheap to
    slice. Sharing the read-back methods through a mixin was considered and
    rejected as more indirection than three small methods are worth.
    """

    def __init__(self, data):
        missing = [k for k in ARRAYS if k not in data]
        if missing:
            raise ValueError(
                f"not a slam map file: missing {', '.join(missing)}. It was "
                f"probably written by an older core/slam/record.py")
        self.points = np.asarray(data["points"], dtype=float)
        self.point_submap = np.asarray(data["point_submap"])
        self.point_scan = np.asarray(data["point_scan"])
        self.edge_i = np.asarray(data["edge_i"])
        self.edge_j = np.asarray(data["edge_j"])
        self.edge_kind = np.asarray(data["edge_kind"])
        self.edge_scan = np.asarray(data["edge_scan"])
        self.edge_info = np.asarray(data["edge_info"], dtype=float)
        self.hist_scan = np.asarray(data["hist_scan"])
        self.hist_reason = np.asarray(data["hist_reason"])
        self.hist_n = np.asarray(data["hist_n"])
        self.hist_poses = np.asarray(data["hist_poses"], dtype=float)
        self.final_poses = np.asarray(data["final_poses"], dtype=float)
        self.start_pose = tuple(np.asarray(data["start_pose"], dtype=float))
        self.meta = {str(k): str(v) for k, v in np.asarray(data["meta_values"])}

    @classmethod
    def load(cls, path):
        with np.load(path, allow_pickle=False) as data:
            return cls(data)

    @property
    def n_submaps(self):
        return self.final_poses.shape[0]

    @property
    def n_scans(self):
        return int(self.point_scan.max()) if len(self.point_scan) else 0

    def poses_at(self, scan):
        rows = np.nonzero(self.hist_scan <= scan)[0]
        if not len(rows):
            return np.zeros((0, 3))
        row = rows[-1]
        return self.hist_poses[row, :self.hist_n[row]]

    def points_at(self, scan, poses=None):
        """World-frame points measured up to `scan`, placed by the graph at `scan`.

        Masked once and transformed per SUBMAP — not per scan block, which is
        what `MapRecorder` does. The renderer calls this once per animation
        frame against ~200 000 points, so the loop has to be over the ~50
        submaps rather than the ~1100 blocks; and it must still go through
        `scan_mod.transform`, because an open-coded rotation here would differ
        from the live pipeline's in the last bit and the two implementations
        would then disagree for no reason anyone could find.
        """
        if poses is None:
            poses = self.poses_at(scan)
        if not len(poses) or not len(self.points):
            return np.zeros((0, 2))
        keep = (self.point_scan <= scan) & (self.point_submap < len(poses))
        out = []
        for k in range(len(poses)):
            pts = self.points[keep & (self.point_submap == k)]
            if len(pts):
                out.append(scan_mod.transform(pts, tuple(poses[k])))
        if not out:
            return np.zeros((0, 2))
        return np.concatenate(out, axis=0)

    def decimate(self, stride):
        """A view of this map holding every `stride`-th point.

        For the animation, which redraws the whole cloud every frame: a 240 s
        episode measures ~200 000 points and the eye cannot tell that from
        40 000 against a 12 x 8 m room, but matplotlib can tell the difference
        in seconds per frame. A fixed stride rather than a random sample so the
        video is reproducible, and the graph, the edges and the history are
        SHARED rather than copied — only the point arrays are thinned, and
        nothing here is ever written back.
        """
        if stride <= 1:
            return self
        thin = copy.copy(self)
        thin.points = self.points[::stride]
        thin.point_submap = self.point_submap[::stride]
        thin.point_scan = self.point_scan[::stride]
        return thin

    def closures(self):
        keep = self.edge_kind == EDGE_CLOSURE
        return list(zip(self.edge_scan[keep].tolist(),
                        self.edge_i[keep].tolist(),
                        self.edge_j[keep].tolist()))

    def solve_scans(self):
        """The scan indices at which the graph was re-solved. The moments the
        map moves, and what the error plot marks."""
        return self.hist_scan[self.hist_reason == SOLVED].tolist()
