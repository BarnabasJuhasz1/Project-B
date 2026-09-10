"""
core/slam/submap.py

**A submap: N consecutive scans accumulated in one frame, plus its field.**

Chunk 22's first piece. Chained scan-to-scan matching drifts — every match
carries a small error and the errors compound — and after a lap of lab42 the
estimate is metres out, which is an expensive way of reproducing
`core/drift.py`. Submaps are what a new scan matches *against*, and what a loop
closure closes *onto*.

WHY SUBMAPS AND NOT SCAN-TO-SCAN
--------------------------------
Two reasons, and the second is the one that matters:

1. A submap is denser than a single scan, so the likelihood field has fewer
   holes and matching is better conditioned.
2. **Scan-to-scan gives loop closure nothing stable to match against.** A single
   scan from a different heading looks different; a submap accumulated over
   several poses is closer to a local map, and a robot returning to the same
   place will match it from a heading it never occupied.

BOUNDARIES ARE A PURE FUNCTION OF SCAN COUNT
--------------------------------------------
A submap finishes after exactly `SCANS_PER_SUBMAP` scans. **Not** after a
distance travelled, not after a rotation threshold, not when the match score
drops. Those alternatives all make submap boundaries depend on the trajectory,
so two runs of the same episode that differ by one control step would cut their
submaps in different places and produce different graphs — and this project's
determinism guarantee is bitwise. A fixed count is the only rule under which two
identical runs are identical.

SIM-FREE.
"""

import numpy as np

from core.slam import scan as scan_mod
from core.slam.likelihood import build_field

# ---- below this line is data, not code.

# Scans per submap. The trade is coverage against staleness: too few and the
# field is sparse and the submap spans no useful area; too many and the submap
# accumulates its own internal drift, because scans late in it are placed by a
# pose estimate that has already moved. 20 scans at chunk 24's scan rate is a
# few seconds of travel, which is the right order for a 12 x 8 m room.
SCANS_PER_SUBMAP = 20

# ---- below this line is code, not data.


class Submap:
    """Scans accumulated in the frame of the pose the submap started at.

    `origin` is that pose in the global frame. Points are stored in the
    submap's OWN frame, not the global one, which is what makes a submap
    relocatable: chunk 23's optimiser moves `origin` and the points come with
    it, with no re-accumulation and no loss.
    """

    __slots__ = ("origin", "_points", "_n_scans", "_field", "index")

    def __init__(self, origin, index=0):
        self.origin = tuple(float(v) for v in origin)
        self.index = index
        self._points = []
        self._n_scans = 0
        self._field = None

    def __len__(self):
        return self._n_scans

    @property
    def finished(self):
        return self._n_scans >= SCANS_PER_SUBMAP

    @property
    def n_points(self):
        return sum(len(p) for p in self._points)

    def add(self, points, pose):
        """Add one scan. `points` are robot-frame; `pose` is global.

        The scan is stored relative to `origin`, so a later correction to
        `origin` carries the whole submap without touching the points.
        """
        if self.finished:
            raise ValueError(
                f"submap {self.index} already holds {self._n_scans} scans and "
                f"is finished; boundaries are a pure function of scan count so "
                f"that two identical runs cut them identically")
        local_pose = scan_mod.relative(self.origin, pose)
        self._points.append(scan_mod.transform(points, local_pose))
        self._n_scans += 1
        # Any addition invalidates the field. Rebuilt lazily rather than per
        # add, because building is the expensive half and a submap is added to
        # SCANS_PER_SUBMAP times and matched against many more.
        self._field = None
        return self

    def last_scan(self):
        """The most recently added scan, in the submap's own frame.

        Exists for `core/slam/record.py`, which must store exactly the points
        the submap stored rather than recompute them from the pose the tracker
        settled on — a recomputation would be a second implementation of `add`
        and could drift from it. Read-only, and the localiser calls it only when
        recording is enabled.
        """
        return self._points[-1] if self._points else np.zeros((0, 2))

    def points(self):
        """Every point, in the submap's own frame."""
        if not self._points:
            return np.zeros((0, 2))
        return np.concatenate(self._points, axis=0)

    def field(self):
        """The likelihood field, built once and cached."""
        if self._field is None:
            self._field = build_field(self.points())
        return self._field

    def global_points(self):
        """Every point in the global frame. For debugging and for tests; the
        matcher never wants this, because it matches in the submap's frame."""
        return scan_mod.transform(self.points(), self.origin)

    def relocate(self, origin):
        """Move the submap. Chunk 23's optimiser calls this.

        The field is NOT invalidated: it is expressed in the submap's own frame
        and the points did not move relative to the origin. That is the whole
        reason points are stored locally, and it is what makes re-optimising a
        graph cheap.
        """
        self.origin = tuple(float(v) for v in origin)
        return self
