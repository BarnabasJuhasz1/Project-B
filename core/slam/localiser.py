"""
core/slam/localiser.py

**`SlamLocaliser` — the thing `Scenario.localisation` has been failing loudly
for since chunk 19.**

This is the assembly, and it holds almost no algorithm of its own. Every piece is
built and tested one file away:

    scan.py        a scan is endpoints in the robot frame          (21)
    likelihood.py  a blurred field, precomputed per submap         (21)
    matcher.py     correlative matching: scan + field -> a pose    (21)
    submap.py      N scans accumulated in one relocatable frame    (22)
    loop.py        closure proposals, and four gates refusing them (22)
    tracker.py     noisy odometry, corrected by matching           (22)
    graph.py       the least squares that applies a closure        (23)

WHAT THIS BUYS THE PROJECT
--------------------------
`core/drift.py` adds a seeded random walk to the true pose, so **no controller
can influence the recorded pose error** — drift is open-loop, `localisation_
failures` is bit-identical across controllers that behave nothing alike, and
`cards/store.py` refuses any prediction naming that metric. Here the estimate is
a consequence of what the robot did: drive somewhere featureless and the match
degrades; return somewhere you have been and a closure pulls the trajectory
back. That is the whole reason epic 03 exists.

TWO PLACES GROUND TRUTH COULD LEAK IN, AND WHAT STOPS IT
---------------------------------------------------------
**`update()` is handed the true pose.** It reaches exactly one place —
`Tracker._odometry`, which takes the *delta* between successive true poses,
corrupts it with `LocalisationSpec.odom_sigma_lin/ang`, and throws the absolute
value away. No absolute position, no correction, no ground truth about where the
robot is. It is the simulator standing in for wheel encoders and an IMU.

⚠️ **With `odom_sigma_lin == 0` it IS a free ride.** Perfect odometry means an
exact estimate, nothing for a closure to correct, and an epic whose central claim
is unmeasurable. A scenario asking for SLAM with zero odometry noise is a
misconfiguration wearing a valid dataclass, and `reset()` says so out loud rather
than producing a suspiciously excellent number.

**`error_estimate()` is a PROXY, and this docstring is where that is admitted.**
It is the accumulated dead-reckoning uncertainty since the last accepted closure:
rises monotonically while the robot drives, drops to zero when a closure lands.
That is the qualitative shape `IsLocalisationDegraded` needs and it is honest as
a predicted quantity. It is **not** the marginal covariance of the current pose
and it is **not** calibrated — chunk 23 chose the proxy over the marginal
deliberately (the marginal needs a sparse inverse and costs more than the solve),
and promoting it is a chunk of its own, to be done only if a card needs a
calibrated number.

SIM-FREE.
"""

import time

import numpy as np

from core.slam import scan as scan_mod
from core.slam.graph import (CLOSURE, CLOSURE_INFO, HUBER_DELTA, ITERATIONS,
                             ODOM_INFO, ODOMETRY, PoseGraph)
from core.slam.loop import CLOSURE_MAX_AMBIGUITY
from core.slam.record import NODE_ADDED, SOLVED, MapRecorder
from core.slam.submap import SCANS_PER_SUBMAP
from core.slam.tracker import Tracker

# ---- below this line is data, not code.

# Defaults for a LocalisationSpec that leaves a field at 0. Written here rather
# than in the dataclass because 0 has to keep meaning "unset" there -- the spec
# was frozen in chunk 19, before any of these numbers were known.
# ⚠️ NOT a tuned number and not a free choice — MEASURED, chunks 23 and 24a.
#
# One correlative match against a full submap costs ~44 ms (181 beams, ~2900
# points, on this machine). At the 50 Hz control rate that is 2.2 s of matching
# per simulated second and a 60 s episode does not finish -- chunk 23's finding,
# confirmed by chunk 24a at 7.3x real time.
#
# ⚠️ **5 Hz, MOVED FROM 10 BY CHUNK 24a**, which is the chunk chartered to own
# this number. Measured on the same trajectory at three lengths:
#
#     sim s |  5 Hz cost  final |  10 Hz cost  final
#      37.9 |      0.281  0.030 |       0.931  0.068
#      77.7 |      0.445  0.055 |       1.669  0.053
#     117.6 |      0.590  0.068 |       2.421  0.061
#
# 3.3-4.1x cheaper at indistinguishable accuracy, and it is the difference
# between an affordable evaluation sweep and an unaffordable one: the held-out
# scenarios carry time_limits of 120-240 s, where 10 Hz exceeds real time by
# 2.4x and 5 Hz does not reach 0.6x.
#
# The reason 10 Hz costs so much more for so little is worth stating, because it
# is not the matching: at three laps 10 Hz accepts 811 closure edges against
# 5 Hz's 148, and **the extra 663 buy nothing measurable**. Closure search is
# quadratic in the submap count and the scan rate sets that count, so the rate
# is a quadratic lever on the dominant cost and a linear one on matching.
#
# ⚠️ The one place 5 Hz is worse is a SHORT one-way episode: on a 14.5 s
# traverse it finishes too few submaps to propose a single closure and lands at
# 0.364 m against 10 Hz's 0.221. That is an artefact of the episode being too
# short for the closure machinery to start, not a property of the rate -- at
# 37.9 s, 5 Hz already takes 9 closures and beats 10 Hz outright. Every held-out
# scenario is 120 s or longer. Recorded as the residual risk of this choice.
#
# It may not move back to "every step", because that number is not reachable.
DEFAULT_SCAN_RATE = 5.0            # Hz. 0.0 would mean every control step.
DEFAULT_SUBMAP_SCANS = SCANS_PER_SUBMAP
DEFAULT_ITERATIONS = ITERATIONS
DEFAULT_HUBER_DELTA = HUBER_DELTA

# ---- below this line is code, not data.


def _closure_info(score, ambiguity):
    """Scale a closure edge's information by how good the match actually was.

    `matcher.py` says a barely-unique match deserves less weight than a sharp
    one, and this is where that is spent. Both factors are already bounded by
    `loop.py`'s gates -- score >= CLOSURE_MIN_SCORE and ambiguity <=
    CLOSURE_MAX_AMBIGUITY -- so the product lands in (0, score] and cannot
    reach zero, which would make the edge a no-op that still counted as a
    closure.
    """
    sharpness = max(0.0, 1.0 - float(ambiguity) / CLOSURE_MAX_AMBIGUITY)
    w = float(score) * sharpness
    return tuple(v * w for v in CLOSURE_INFO)


class SlamLocaliser:
    """Chunk 19's `Localiser` protocol, backed by the graph.

    One episode, one instance. `reset()` rebuilds everything, because a submap
    or an edge surviving into the next episode is a correlation between episodes
    that the four-condition ablation is not allowed to have.
    """

    def __init__(self, spec, rng):
        self._spec = spec
        self._rng = rng
        # Set by enable_recording() and NEVER by a spec field. Recording is a
        # property of this run of the episode, not of the scenario -- a
        # scenario that recorded its map would make the recording part of the
        # experiment's definition, and it is not: it is a picture of it.
        self._recording = False
        self.recorder = None
        self._reset_state()

    def enable_recording(self):
        """Keep the map and the graph history, for `scripts/visualise_slam.py`.

        ⚠️ **Inert, on `nodes/trace.py`'s precedent.** A recorded run and an
        unrecorded run of the same episode produce a byte-identical `slam.csv`
        and an identical pose stream; nothing captured here is read back into an
        estimate and nothing here touches the rng. `core/tests/
        test_slam_record.py` runs the episode twice and diffs, because
        "switching the visualiser on changed the run" is precisely the failure
        that would poison the evidence this exists to produce.

        Off by default because the cost is a copy of every scan point, which a
        1000-episode sweep has no use for and a demonstration run does.
        """
        self._recording = True
        if self._tracker is not None:            # already reset(); start now
            self._start_recorder()
        return self

    # -- protocol ----------------------------------------------------------

    def _reset_state(self):
        self._tracker = None
        self.recorder = None
        self._graph = PoseGraph(
            iterations=self._spec.solver_iterations or DEFAULT_ITERATIONS,
            huber_delta=self._spec.huber_delta or DEFAULT_HUBER_DELTA)
        self._pose = (0.0, 0.0, 0.0)
        self._since_scan = 0.0
        self._scans = 0
        self._closure_edges = 0
        self._solves = 0
        # Wall clock, kept OUT of diagnostics() -- see that method, and
        # `cost()` below for what chunk 24a reads.
        self.last_solve_seconds = 0.0
        self.total_solve_seconds = 0.0
        self.total_seconds = 0.0
        self._updates = 0

    def reset(self, scenario, rng):
        self._rng = rng
        self._reset_state()
        spec = self._spec

        if spec.odom_sigma_lin <= 0.0 and spec.odom_sigma_ang <= 0.0:
            raise ValueError(
                f"scenario {scenario.id!r}: LocalisationSpec(kind='slam') with "
                f"odom_sigma_lin=0 and odom_sigma_ang=0 gives the localiser "
                f"PERFECT dead reckoning. The estimate would then be exact, a "
                f"loop closure would have nothing to correct, and the reported "
                f"pose error would measure the simulator rather than SLAM. Set "
                f"odom_sigma_lin (metres per sqrt(second), same units as "
                f"DriftSpec.sigma_xy) -- or use kind='perfect' if a "
                f"ground-truth control is what was wanted.")

        start = tuple(float(v) for v in scenario.start)
        self._tracker = Tracker(
            start, rng,
            odom_sigma_lin=spec.odom_sigma_lin,
            odom_sigma_ang=spec.odom_sigma_ang,
            scans_per_submap=spec.submap_scans or DEFAULT_SUBMAP_SCANS)
        self._pose = start
        # Node 0 is submap 0's origin, and it is the anchor. The start pose is
        # the one pose a robot is entitled to know exactly: it is where the
        # episode is defined to begin, not something it has to estimate.
        self._graph.add_node(start)
        if self._recording:
            self._start_recorder()

    def _start_recorder(self):
        self.recorder = MapRecorder(
            self._pose,
            meta={"scan_rate": self._spec.scan_rate or DEFAULT_SCAN_RATE,
                  "submap_scans": self._spec.submap_scans or DEFAULT_SUBMAP_SCANS,
                  "odom_sigma_lin": self._spec.odom_sigma_lin,
                  "odom_sigma_ang": self._spec.odom_sigma_ang})
        self.recorder.graph(self._graph.poses, NODE_ADDED)

    def update(self, dt, true_pose, ranges, angles):
        """Timed, so that `cost()` can attribute an episode's wall clock.

        The wrapper exists because the phase accumulators cover matching,
        closure search and solving, and the REMAINDER -- likelihood fields,
        scan endpoints, submap accumulation, graph maintenance -- is the part
        nobody budgets for and is therefore the part worth measuring. `cost()`
        derives it by subtraction rather than by timing four more places.
        """
        t0 = time.perf_counter()
        try:
            return self._update(dt, true_pose, ranges, angles)
        finally:
            self.total_seconds += time.perf_counter() - t0
            self._updates += 1

    def _update(self, dt, true_pose, ranges, angles):
        if self._tracker is None:
            raise RuntimeError(
                "SlamLocaliser.update() before reset(); the localiser has no "
                "start pose and no graph anchor")

        self._since_scan += float(dt)
        rate = self._spec.scan_rate or DEFAULT_SCAN_RATE
        # The FIRST update always takes a scan, whatever the rate. It is the one
        # scan taken at a pose that is exactly known -- the scenario's start, and
        # the graph anchor -- so it is what registers submap 0's frame against
        # the truth. Deferring it to the first scan interval would hand submap 0
        # an origin that is already a dead-reckoning step stale, and every later
        # pose is expressed relative to that origin.
        if self._scans and rate > 0.0 and self._since_scan < 1.0 / rate:
            # Between scans the estimate is the last one. Deliberately NOT
            # dead-reckoned forward: that would need the true pose at a step
            # where no measurement is taken, and a stale estimate is an honest
            # consequence of a slow sensor. Chunk 24a sets the rate against
            # the measured cost; the default is DEFAULT_SCAN_RATE above, and a
            # spec asking for scan_rate=0 gets that default rather than every
            # step -- 0 means "unset" in a LocalisationSpec, per chunk 19. To
            # scan every control step, ask for the control rate explicitly.
            return self._pose

        step_dt, self._since_scan = self._since_scan, 0.0
        points = scan_mod.endpoints(ranges, angles)

        before = len(self._tracker.submaps)
        self._pose = self._tracker.update(step_dt, true_pose, points)
        finished_a_submap = len(self._tracker.submaps) > before
        self._scans += 1
        if self.recorder is not None:
            # The submap the scan actually landed in: the one that just
            # finished, if it did, otherwise the one still filling. Taken from
            # the submap itself rather than recomputed from the settled pose,
            # so the recording is what was stored and not a second
            # implementation of `Submap.add`.
            landed = (self._tracker.submaps[-1] if finished_a_submap
                      else self._tracker.current)
            self.recorder.scan(landed.index,
                               landed.last_scan() if len(points) else
                               np.zeros((0, 2)))
        self._sync_nodes()

        # ⚠️ CLOSURES ARE PROPOSED ONLY WHEN A SUBMAP FINISHES, not every scan.
        #
        # Chunk 23's own rule -- "the graph changes only when a submap finishes
        # or a closure is accepted" -- plus the cost: a proposal is a match with
        # a WIDER window against every nearby finished submap, so proposing per
        # scan multiplies the episode's dominant cost by the candidate count.
        # At 10 Hz and 20 scans per submap this is one proposal round every 2 s,
        # which is far finer than the drift it exists to catch.
        #
        # `Tracker.propose_closures` is unchanged and still callable per scan;
        # this is the LOCALISER's policy, so chunk 22's tests keep their meaning.
        if finished_a_submap:
            accepted = self._tracker.propose_closures(points)
            if accepted:
                self._add_closures(accepted)
                self._optimise()
        return self._pose

    def estimate(self):
        return self._pose

    def error_estimate(self):
        """PREDICTED radial uncertainty in metres. A proxy — see the module
        docstring, which is where that is admitted rather than buried."""
        return self._tracker.uncertainty if self._tracker else 0.0

    def diagnostics(self):
        """Flat scalars for the `slam.csv` sidecar.

        ⚠️ **No wall clock here, and that is a deliberate departure from chunk
        23's build list.** The chunk asked for "solve wall-clock" in
        `diagnostics()`, and the same chunk requires `slam.csv` to be
        byte-identical across two runs of one episode. Those cannot both hold: a
        timing is different every run by construction. The timing is kept on
        `last_solve_seconds` / `total_solve_seconds`, which is where chunk 24's
        cost budget reads it from, and out of the file determinism is checked on.
        """
        solve = self._graph.last_solve
        d = {
            "scans": self._scans,
            "submaps": len(self._graph),
            "graph_edges": len(self._graph.edges),
            "closure_edges": self._closure_edges,
            "solves": self._solves,
            "solve_iterations": solve.get("iterations", 0),
            "solve_chi2_before": round(solve.get("chi2_before", 0.0), 6),
            "solve_chi2_after": round(solve.get("chi2_after", 0.0), 6),
            "solve_singular": solve.get("singular", 0),
        }
        d.update(self._tracker.diagnostics() if self._tracker else {})
        return d

    def cost(self):
        """Wall clock by phase, in seconds. Chunk 24a's budget reads this.

        ⚠️ **Never merged into `diagnostics()`**, for the reason that method
        gives: `slam.csv` must be byte-identical across two runs of one episode
        and a timing is different every run. Cost and determinism are measured
        on the same pipeline and reported through different channels, which is
        the only way to have both.

        `other` is the remainder -- likelihood fields, scan endpoints, submap
        accumulation, graph maintenance -- and is derived by subtraction so
        that the phases and the total cannot disagree.
        """
        t = self._tracker
        match_s = t.match_seconds if t else 0.0
        closure_s = t.closure_seconds if t else 0.0
        total = self.total_seconds
        return {
            "total_seconds": total,
            "match_seconds": match_s,
            "closure_seconds": closure_s,
            "solve_seconds": self.total_solve_seconds,
            "other_seconds": total - match_s - closure_s
                             - self.total_solve_seconds,
            "updates": self._updates,          # every control step, scan or not
            "scans": self._scans,              # updates that took a scan
            "matches": t.matches if t else 0,
            "closure_rounds": t.closure_rounds if t else 0,
            "solves": self._solves,
        }

    # -- graph maintenance -------------------------------------------------

    def _sync_nodes(self):
        """Give every submap a node, and chain the new ones with odometry edges.

        Driven off the tracker's own submap list rather than a callback, because
        submap boundaries are a pure function of scan count (`submap.py`) and so
        this loop is too — a callback would be one more ordering to get right for
        no gain.
        """
        submaps = self._tracker.submaps + [self._tracker.current]
        while len(self._graph) < len(submaps):
            k = len(self._graph)
            prev, origin = submaps[k - 1].origin, submaps[k].origin
            self._graph.add_node(origin)
            self._graph.add_edge(k - 1, k, scan_mod.relative(prev, origin),
                                 info=ODOM_INFO)
            if self.recorder is not None:
                self.recorder.edge(k - 1, k, ODOMETRY, ODOM_INFO)
                self.recorder.graph(self._graph.poses, NODE_ADDED)

    def _add_closures(self, accepted):
        """Turn chunk 22's proposals into edges between two SUBMAP nodes.

        A proposal is expressed between the *robot* and an old submap, and the
        robot is not a node. The conversion, with P the robot's global pose:

            P = O_s (+) M          M  = proposal.pose_in_submap
            P = O_c (+) L          L  = the robot in the CURRENT submap's frame
            =>  relative(O_s, O_c) = M (+) L^-1

        Doing this in the submap frames rather than the global one matters: the
        global poses are exactly what is in dispute, so a measurement built from
        them would be a constraint derived from the thing it is meant to correct.
        """
        current = self._tracker.current
        c = current.index
        local = scan_mod.relative(current.origin, self._tracker.pose)
        inv_local = scan_mod.relative(local, (0.0, 0.0, 0.0))
        for proposal in accepted:
            s = proposal.submap_index
            if s == c:                              # pragma: no cover
                continue                            # gated by MIN_SUBMAP_GAP
            self._graph.add_edge(
                s, c, scan_mod.compose(proposal.pose_in_submap, inv_local),
                info=_closure_info(proposal.score, proposal.ambiguity),
                robust=True, kind=CLOSURE)
            self._closure_edges += 1
            if self.recorder is not None:
                self.recorder.edge(s, c, CLOSURE,
                                   self._graph.edges[-1].info)

    def _optimise(self):
        """Solve, then carry the submaps and the live pose with the result.

        The current pose is re-derived as `new_origin (+) local` using the local
        pose captured BEFORE the solve. Recomputing it from the corrected origin
        is the point: a closure that moves submap c by 20 cm must move the robot
        by 20 cm too, or the estimate and the map disagree and the next match
        starts from a guess the map does not support.
        """
        current = self._tracker.current
        local = scan_mod.relative(current.origin, self._tracker.pose)

        t0 = time.perf_counter()
        self._graph.solve()
        self.last_solve_seconds = time.perf_counter() - t0
        self.total_solve_seconds += self.last_solve_seconds
        self._solves += 1

        submaps = self._tracker.submaps + [current]
        for k, submap in enumerate(submaps):
            # Points live in the submap's own frame, so relocating an origin
            # carries the whole submap and keeps its cached likelihood field --
            # which is the reason `submap.py` stores them locally at all.
            submap.relocate(self._graph.poses[k])

        self._pose = scan_mod.compose(current.origin, local)
        self._tracker.pose = self._pose
        if self.recorder is not None:
            # AFTER the relocation, because this snapshot is what the map is
            # drawn against and the whole point of drawing it is to show the
            # map move when a loop closes.
            self.recorder.graph(self._graph.poses, SOLVED)
