"""
core/slam/likelihood.py

**The occupancy likelihood field a scan is scored against.**

Chunk 21's second piece. A field is built ONCE per submap and matched against
many times — that asymmetry is what makes the brute-force search in `matcher.py`
affordable at all, because matching then costs one table lookup per beam
endpoint instead of a nearest-neighbour query.

WHY BLURRED, AND NOT A HARD OCCUPANCY GRID
------------------------------------------
A hard grid scores 1 for a point in an occupied cell and 0 for a point one cell
away, so the objective is a plateau with cliffs and the search has no gradient to
follow — every candidate one cell off scores identically to one a metre off. The
blur turns "is this point on a wall" into "how close is this point to a wall",
which is what makes a coarse-to-fine search meaningful and what lets the score
degrade smoothly under chunk 20's range noise instead of falling off a cliff.

The kernel is a **fixed-width Gaussian with a fixed truncation radius**. Not
adaptive, not tuned per submap: an adaptive kernel would make the score
incomparable between submaps, and 22 thresholds on that score.

WHY A SEPARATE GRID FROM `core.geometry.occupancy_grid`
-------------------------------------------------------
`core.geometry`'s grid is **configuration space over the known floorplan** — a
cell is occupied if the robot *centre* there would touch a wall, computed from
`World.rects`, which is ground truth. Neither half is available to a localiser:
SLAM has no floorplan, and it needs workspace occupancy (where the walls are),
not C-space (where the robot may not go).

So this is a different grid for a different question, and it is built from
**scan endpoints only**. The *conventions* are borrowed deliberately —
`resolution`, an `origin` at the min corner, `(ny, nx)` row-major with y first —
so that anyone who has read `core/geometry.py:Grid` can read this, and so a
future chunk can overlay the two for debugging. Chunk 13 avoided a second
geometry convention for the same reason.

SIM-FREE.
"""

import numpy as np
from scipy.ndimage import gaussian_filter, maximum_filter

from core.geometry import GRID_RESOLUTION

# ---- below this line is data, not code.

# The field's cell size. Deliberately the same as `core.geometry.GRID_RESOLUTION`
# so that a SLAM map and the ground-truth grid are directly comparable when
# debugging, and so there is one number to reason about rather than two.
# It also sets the matcher's translational search resolution.
FIELD_RESOLUTION = GRID_RESOLUTION            # 0.05 m

# Gaussian width, in METRES, converted to cells below. 0.10 m is two cells at
# the resolution above: wide enough that a match one cell out still scores
# nearly as well (so the search has a gradient), narrow enough that a wall and a
# parallel wall 0.5 m away do not merge into one basin.
BLUR_SIGMA_M = 0.10

# Where the Gaussian is cut off, in standard deviations. FIXED, and part of the
# determinism story: scipy's default truncate is 4.0, and pinning it here means
# a scipy upgrade that changed the default could not silently change every
# match score in the project.
BLUR_TRUNCATE = 4.0

# Empty margin kept around the point cloud, in metres, so that a candidate pose
# near the edge of the window does not score high merely because its points fell
# off the field and were dropped. Must exceed the matcher's largest translational
# search window.
FIELD_PAD_M = 1.5

# Coarse-to-fine factor for the branch-and-bound upper bound in `matcher.py`.
# 4 means the coarse level is 0.20 m cells: few enough candidates to enumerate
# cheaply, fine enough that the bound is not so loose it prunes nothing.
COARSE_FACTOR = 4

# ---- below this line is code, not data.


class LikelihoodField:
    """A blurred occupancy field over one submap's points, plus its coarse
    upper-bound level.

    `coarse` is a **sliding-window maximum**, not a block maximum, and that
    distinction is load-bearing for correctness rather than for speed. The
    branch-and-bound in `matcher.py` needs `coarse[c] >= fine[c + d]` for every
    sub-offset `d` in `[0, COARSE_FACTOR)`. A block max only bounds cells inside
    its own block, so a sub-offset that crossed a block boundary would be bounded
    by the wrong block, the bound would not be admissible, and the search would
    prune the true optimum — producing a confidently wrong pose, which is exactly
    the failure mode chunk 21 rejected ICP to avoid.
    """

    __slots__ = ("fine", "coarse", "origin", "resolution", "n_points")

    def __init__(self, fine, coarse, origin, resolution, n_points):
        self.fine = fine
        self.coarse = coarse
        self.origin = origin              # (min_x, min_y) of cell (0, 0)
        self.resolution = resolution
        self.n_points = n_points

    @property
    def shape(self):
        return self.fine.shape            # (ny, nx)

    def world_to_cell(self, points):
        """(N, 2) world points -> (N, 2) integer cells. NOT clamped.

        Unclamped on purpose: the matcher has to know which points fell off the
        field so it can score them as zero rather than as whatever sat at the
        clamped edge cell. Clamping here would let a badly wrong pose pile every
        stray point onto one high-scoring border cell.
        """
        points = np.asarray(points, dtype=float)
        return np.floor((points - np.asarray(self.origin)) /
                        self.resolution).astype(np.int64)

    def score(self, points):
        """Mean field value at `points`. In [0, 1]; higher is a better fit.

        **Mean, not sum.** The point count varies between scans — chunk 20's
        dropout removes beams, and `scan.endpoints` drops `inf` — so a sum would
        make a sparse scan score lower than a dense one at the same true pose,
        and 22 thresholds on this number. A mean is comparable across scans.

        Points outside the field contribute 0, which is the honest reading:
        the map says nothing there, so nothing supports the hypothesis.
        """
        points = np.asarray(points, dtype=float)
        if points.size == 0:
            return 0.0
        cells = self.world_to_cell(points)
        ny, nx = self.fine.shape
        inside = ((cells[:, 0] >= 0) & (cells[:, 0] < nx) &
                  (cells[:, 1] >= 0) & (cells[:, 1] < ny))
        if not inside.any():
            return 0.0
        vals = self.fine[cells[inside, 1], cells[inside, 0]]
        return float(vals.sum() / len(points))


def build_field(points, resolution=FIELD_RESOLUTION, pad=FIELD_PAD_M,
                sigma_m=BLUR_SIGMA_M, coarse_factor=COARSE_FACTOR):
    """World-frame points -> a `LikelihoodField`.

    `points` are the accumulated endpoints of one submap, already placed in a
    common frame by the caller. Building the field is the expensive half of
    matching and happens once per submap; see the module docstring.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if len(points) == 0:
        raise ValueError("a likelihood field needs at least one point; an "
                         "empty submap is a caller bug, not a degenerate map")

    min_xy = points.min(axis=0) - pad
    max_xy = points.max(axis=0) + pad
    nx = int(np.ceil((max_xy[0] - min_xy[0]) / resolution))
    ny = int(np.ceil((max_xy[1] - min_xy[1]) / resolution))

    hits = np.zeros((ny, nx), dtype=float)
    cells = np.floor((points - min_xy) / resolution).astype(np.int64)
    np.clip(cells[:, 0], 0, nx - 1, out=cells[:, 0])
    np.clip(cells[:, 1], 0, ny - 1, out=cells[:, 1])
    # A cell hit by many beams is not more occupied than one hit by a single
    # beam -- it is closer to the sensor, or the sensor dwelt on it. Setting
    # rather than accumulating keeps the field a statement about geometry
    # instead of about the trajectory that observed it.
    hits[cells[:, 1], cells[:, 0]] = 1.0

    fine = gaussian_filter(hits, sigma=sigma_m / resolution,
                           truncate=BLUR_TRUNCATE, mode="constant", cval=0.0)
    peak = fine.max()
    if peak > 0:
        # Normalised so `score()` is in [0, 1] and comparable BETWEEN submaps,
        # which is what makes a single threshold in chunk 22 meaningful. Without
        # this, a submap observed from further away would score systematically
        # lower for reasons that are nothing to do with the match quality.
        fine = fine / peak

    coarse = maximum_filter(fine, size=coarse_factor, mode="constant", cval=0.0,
                            origin=_max_filter_origin(coarse_factor))
    return LikelihoodField(fine, coarse, (min_xy[0], min_xy[1]), resolution,
                           len(points))


def _max_filter_origin(size):
    """Shift `maximum_filter` from centred to FORWARD-looking.

    scipy centres its window. The bound the matcher needs is over the block
    *starting* at each cell — `coarse[c] = max(fine[c : c + size])` — so the
    window is shifted by half its width. For even sizes scipy's centre already
    sits half a cell forward, which is why this is `-(size // 2)` and not
    `-(size - 1) // 2`; getting it wrong makes the bound non-admissible in one
    direction only, which shows up as a rare, direction-dependent bad match and
    is very hard to find later.
    """
    return -(size // 2)
