"""
core/slam/matcher.py

**Correlative scan matching: given a scan and a map, where is the robot?**

Chunk 21's front end, and everything downstream is built on it. Chained, it is
odometry; matched against an older submap, it is loop closure (22); its output
edges are what the graph optimises (23).

WHY CORRELATIVE AND NOT ICP
---------------------------
Olson (2009), *Real-Time Correlative Scan Matching*. Brute-force search over a
discretised window of `(dx, dy, dtheta)`, scoring each candidate by how well the
transformed scan lands on a likelihood field, and taking the best cell. The
reason is determinism, and it is epic 03's stated rule that determinism wins over
accuracy wherever the two trade:

|                          | correlative        | ICP                          |
|--------------------------|--------------------|------------------------------|
| iterations               | fixed = window     | until a convergence tolerance |
| depends on initial guess | only via placement | strongly, and non-linearly    |
| random restarts / RANSAC | none               | usual, and a seeded rng       |
| failure mode             | a LOW SCORE        | confident local minimum       |

The last row is the disqualifying one. **A matcher that fails confidently
poisons the graph in chunk 23, and the failure surfaces as a mangled map three
chunks later.** A correlative matcher that cannot find a good pose returns a low
score, and a low score is something 22 can threshold on.

THE SCORE IS THE PRODUCT, NOT JUST THE POSE
-------------------------------------------
`match()` returns `MatchResult(pose, score, converged)` and the score is
**used**, not decorative: 22 thresholds on it to accept a loop closure and 23
weights on it. A front end that always returns an answer is a front end that
cannot say "I do not know", and that is the only thing standing between a bad
match and the pose graph.

TIE-BREAKING, WHICH IS A CORRECTNESS PROPERTY HERE
--------------------------------------------------
Equal scores happen constantly — a symmetric corridor, a blank wall, a scan of
nothing. **The lowest flat index wins**, always. `np.argmax` already does this,
and this module depends on it rather than merely benefiting from it: candidates
are enumerated in a fixed order (theta outermost, then dy, then dx, each
ascending), so "lowest index" is a statement about geometry and not about
whatever order an array happened to be built in. Tested with a deliberately
symmetric world.

SIM-FREE.
"""

import numpy as np

from core.slam import scan as scan_mod
from core.slam.likelihood import COARSE_FACTOR

# ---- below this line is data, not data-dependent code.

# Default search window, in metres and radians. A window must comfortably
# exceed the motion between two scans plus the error in the initial guess, or
# the true pose is outside the search and the matcher returns the best wrong
# answer inside it -- with a plausible score. Chunk 24 sets the scan rate, and
# these must be revisited together with it.
WINDOW_XY = 0.30          # +/- metres
WINDOW_THETA = 0.20       # +/- radians (~11.5 deg)

# Rotational search step. NOT derived from the translational resolution: the
# right step is the one that moves the FURTHEST point in a scan by about one
# cell, so it depends on scan range. 0.01 rad moves a point at 6 m by 6 cm,
# roughly one cell at FIELD_RESOLUTION -- the same granularity as the
# translational search, which is what keeps the two axes commensurable.
THETA_STEP = 0.01

# Below this score a match is not `converged`. ⚠️ It is a THRESHOLD ON A MEAN in
# [0, 1], so it moves if `likelihood.build_field`'s normalisation changes.
MIN_SCORE = 0.35

# ⚠️ A GOOD SCORE IS NOT ENOUGH, and chunk 21 as planned assumed it was.
#
# The plan said a symmetric world would "produce a low score, not a confident
# wrong answer". It does not. Measured on a featureless corridor: a scan taken
# 4 m from where the field was built matched at **score 0.618**, comfortably
# above MIN_SCORE, because its wall points landed on walls. The fit is genuinely
# good; it is just not unique.
#
# So there are TWO failure modes and one number cannot carry both:
#
#   NO MATCH        the scan does not fit the map anywhere -> a low score.
#   AMBIGUOUS MATCH many poses fit equally well -> a HIGH score at every one.
#
# The second is what a corridor, a blank wall or an empty room produces, and it
# is exactly what must never reach chunk 23's graph. It is detected from the
# SHAPE of the score surface rather than its peak: if many well-separated
# candidates score near the best, the data does not determine the pose. Olson
# (2009) derives a covariance from the same distribution; this is the cheap
# scalar version of it.
AMBIGUITY_BAND = 0.02     # score units: candidates this close to best "tie"
MAX_SPREAD = 0.25         # metres: how far apart tied candidates may sit

# ⚠️ AND THE SAME IS TRUE OF ROTATION, which the first implementation missed.
#
# Measured on a square trajectory in a rectangular room: matches reporting
# `score` 1.00 and translational `ambiguity` 0.00 were rotating the estimate by
# ~0.2 rad per step while the position stayed correct. The heading had spun
# 2 rad by the end of one lap.
#
# The cause is the corridor problem in the rotational axis. A room made of long
# straight walls has the property that rotating a scan about the sensor SLIDES
# its points along those walls, and a point sliding along a wall stays on the
# blurred ridge -- so the score barely moves. Rotation is genuinely weakly
# constrained by that geometry, and a translation-only spread cannot see it.
#
# Gated separately rather than folded into one number: the two have different
# units, different failure geometry, and chunk 23 may want to weight an edge
# anisotropically. Both are reported.
MAX_SPREAD_THETA = 0.05   # radians: how far apart tied headings may sit

# ---- below this line is code, not data.


class MatchResult:
    """A pose, how much the map supports it, and whether that is enough.

    `converged` is deliberately NOT "the optimiser terminated" — there is no
    optimiser and nothing to terminate. Naming it `converged` keeps the
    vocabulary chunk 23 expects while this docstring says what it really is.

    ⚠️ It is the AND of THREE conditions, not one: `score >= MIN_SCORE`, and
    `ambiguity <= MAX_SPREAD`, and `ambiguity_theta <= MAX_SPREAD_THETA` — see
    `_verdict`. An earlier version of this docstring said "it means score >=
    MIN_SCORE", which is the seductive half: a run whose matches are refused on
    ROTATIONAL spread then looks like a scoring failure and is diagnosed as the
    wrong thing. Chunk 24b hit exactly that reading off a plot.
    """

    __slots__ = ("pose", "score", "ambiguity", "ambiguity_theta", "converged",
                 "n_points", "evaluated")

    def __init__(self, pose, score, ambiguity, ambiguity_theta, converged,
                 n_points, evaluated):
        self.pose = pose
        self.score = score
        # Metres, and radians. How far apart the near-best candidates sit --
        # 0.0 when the peak is unique. Reported even when `converged` is True,
        # because chunk 23 weights an edge and a barely-unique match deserves
        # less weight than a sharp one.
        self.ambiguity = ambiguity
        self.ambiguity_theta = ambiguity_theta
        self.converged = converged
        self.n_points = n_points
        self.evaluated = evaluated       # candidates actually scored; chunk 24

    def __repr__(self):
        return (f"MatchResult(pose=({self.pose[0]:.3f}, {self.pose[1]:.3f}, "
                f"{self.pose[2]:.4f}), score={self.score:.3f}, "
                f"ambiguity={self.ambiguity:.3f}/{self.ambiguity_theta:.3f}, "
                f"converged={self.converged}, n_points={self.n_points}, "
                f"evaluated={self.evaluated})")


def _verdict(best_score, best_theta, near, min_score, max_spread,
             max_spread_theta):
    """Score and spread -> (ambiguity, ambiguity_theta, converged).

    `near` is every (x, y, theta) that scored within AMBIGUITY_BAND of the best.
    The translational spread is the bounding-box diagonal and the rotational
    spread is the range of headings, both cheap, deterministic, and monotone in
    the thing that matters: how far apart two equally-good explanations of the
    same scan are.

    Headings are compared as WRAPPED DIFFERENCES from the best, not as raw
    values, or a tie spanning +/-pi would report a spread of 2*pi and every
    match near the wrap boundary would be called ambiguous.
    """
    if not near:
        return 0.0, 0.0, False
    xs = [p[0] for p in near]
    ys = [p[1] for p in near]
    spread = float(np.hypot(max(xs) - min(xs), max(ys) - min(ys)))
    deltas = [scan_mod.wrap_angle(p[2] - best_theta) for p in near]
    spread_theta = float(max(deltas) - min(deltas))
    converged = bool(best_score >= min_score
                     and spread <= max_spread
                     and spread_theta <= max_spread_theta)
    return spread, spread_theta, converged


def match(points, field, initial_guess,
          window_xy=WINDOW_XY, window_theta=WINDOW_THETA,
          theta_step=THETA_STEP, min_score=MIN_SCORE,
          max_spread=MAX_SPREAD, max_spread_theta=MAX_SPREAD_THETA):
    """Robot-frame `points` against `field`, searching around `initial_guess`.

    Returns the best `(x, y, theta)` in the field's frame. The translational
    search is on the field's own cell grid, so the recovered pose is exact only
    to `field.resolution` — that is the documented accuracy of this front end,
    and chunk 23's graph is what refines it.

    THE SEARCH, in the order it happens:

    1. For each candidate theta, rotate the cloud ONCE. Rotation is the
       expensive part and it does not depend on the translation, so it is
       hoisted out of the inner loop.
    2. Score every translation on the COARSE field. Because `field.coarse` is a
       forward sliding-window maximum, each coarse score is an admissible UPPER
       BOUND on every fine translation inside its block.
    3. Walk the coarse candidates in descending bound order and expand only
       those whose bound still beats the best fine score found so far. Everything
       else is pruned without being scored.

    Step 3 is exact, not a heuristic: pruning on an admissible bound cannot
    discard the true optimum, so this returns the same answer a full brute force
    would — just faster. That equivalence is asserted by a test, because a
    branch-and-bound that quietly stops being exact is indistinguishable from one
    that works until the day it matters.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    n_points = len(points)
    if n_points == 0:
        # No measurement is not a match. Returning the initial guess with a
        # score of 0 is the honest answer, and `converged=False` keeps it out of
        # the graph.
        return MatchResult(tuple(float(v) for v in initial_guess), 0.0, 0.0,
                           0.0, False, 0, 0)

    gx, gy, gtheta = (float(initial_guess[0]), float(initial_guess[1]),
                      float(initial_guess[2]))
    res = field.resolution
    offsets = _offsets(window_xy, res)
    thetas = _thetas(gtheta, window_theta, theta_step)
    ny, nx = field.fine.shape

    # The coarse grid is the fine grid strided by COARSE_FACTOR, so the blocks
    # TILE the window instead of overlapping it. Overlapping blocks would score
    # the same fine offset several times -- harmless for the answer, since max
    # is idempotent, but it would inflate `evaluated`, and chunk 24 sets the
    # scan rate from that number.
    coarse_idx = np.arange(0, len(offsets), COARSE_FACTOR)

    best_score, best = -1.0, (gx, gy, gtheta)
    evaluated = 0
    # Every candidate within AMBIGUITY_BAND of the running best, so the spread
    # of near-best poses can be measured at the end. Over-collected on purpose:
    # `best_score` only rises, so an entry kept early may fall outside the band
    # later, and the final filter below is what makes the set exact.
    near = []

    for theta in thetas:
        # Rotation is the expensive part and does not depend on the
        # translation, so it is hoisted out of the inner loops.
        rotated = scan_mod.transform(points, (gx, gy, theta))
        base = field.world_to_cell(rotated)          # (N, 2) int, unclamped

        ox_c = offsets[coarse_idx]
        oy_c = offsets[coarse_idx]
        cx = base[:, 0][None, None, :] + ox_c[None, :, None]     # (1, nc, N)
        cy = base[:, 1][None, None, :] + oy_c[:, None, None]     # (nc, 1, N)
        inside = ((cx >= 0) & (cx < nx)) & ((cy >= 0) & (cy < ny))
        bounds = np.where(
            inside,
            field.coarse[np.where(inside, cy, 0), np.where(inside, cx, 0)],
            0.0).sum(axis=2) / n_points

        # Descending bound; ties broken by ASCENDING flat index. `kind="stable"`
        # is what makes that true -- it is the tie-break rule the module
        # docstring states, and it is why a symmetric world resolves the same
        # way every run.
        order = np.argsort(-bounds, axis=None, kind="stable")
        flat_bounds = bounds.ravel()
        n_c = len(coarse_idx)

        for flat in order:
            # Pruned with a MARGIN, not at the best score. Pruning at `<= best`
            # would discard blocks whose bound sits just under the peak, and
            # those are exactly the blocks that make a match ambiguous -- so the
            # ambiguity measure would systematically under-report on the
            # candidates it most needs to see.
            if flat_bounds[flat] <= best_score - AMBIGUITY_BAND:
                break                  # sorted: nothing after this can win
            jy, jx = divmod(int(flat), n_c)
            for iy in range(coarse_idx[jy],
                            min(coarse_idx[jy] + COARSE_FACTOR, len(offsets))):
                fy = base[:, 1] + offsets[iy]
                oky = (fy >= 0) & (fy < ny)
                if not oky.any():
                    continue
                for ix in range(coarse_idx[jx],
                                min(coarse_idx[jx] + COARSE_FACTOR,
                                    len(offsets))):
                    fx = base[:, 0] + offsets[ix]
                    ok = oky & (fx >= 0) & (fx < nx)
                    if not ok.any():
                        continue
                    score = float(field.fine[fy[ok], fx[ok]].sum() / n_points)
                    evaluated += 1
                    pose = (gx + offsets[ix] * res, gy + offsets[iy] * res,
                            scan_mod.wrap_angle(theta))
                    if score >= best_score - AMBIGUITY_BAND:
                        near.append((pose[0], pose[1], pose[2], score))
                    if score > best_score:
                        best_score = score
                        best = pose

    tied = [(x, y, th) for x, y, th, sc in near
            if sc >= best_score - AMBIGUITY_BAND]
    ambiguity, amb_theta, converged = _verdict(
        best_score, best[2], tied, min_score, max_spread, max_spread_theta)
    return MatchResult(best, best_score, ambiguity, amb_theta, converged,
                       n_points, evaluated)


def _offsets(window_xy, resolution):
    """Translational search offsets, in CELLS, symmetric about zero.

    One helper because `match` and `match_bruteforce` must search **exactly**
    the same set — the test that proves branch-and-bound is exact compares their
    results, and it would pass vacuously if they searched different windows.
    """
    half = int(round(window_xy / resolution))
    return np.arange(-half, half + 1, dtype=np.int64)


def _thetas(centre, window_theta, theta_step):
    """Rotational candidates. Same reason as `_offsets`."""
    n = int(round(window_theta / theta_step))
    return centre + np.arange(-n, n + 1, dtype=float) * theta_step


def match_bruteforce(points, field, initial_guess,
                     window_xy=WINDOW_XY, window_theta=WINDOW_THETA,
                     theta_step=THETA_STEP, min_score=MIN_SCORE,
                     max_spread=MAX_SPREAD,
                     max_spread_theta=MAX_SPREAD_THETA):
    """The same search with no pruning. **Reference implementation, not the
    production path** — it exists so a test can assert that branch-and-bound
    returns an identical result, which is the property that makes the pruning
    safe rather than merely fast.

    Deliberately written the slow, obvious way: if this and `match()` ever
    disagree, this one is right.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    n_points = len(points)
    if n_points == 0:
        return MatchResult(tuple(float(v) for v in initial_guess), 0.0, 0.0,
                           0.0, False, 0, 0)

    gx, gy, gtheta = (float(initial_guess[0]), float(initial_guess[1]),
                      float(initial_guess[2]))
    res = field.resolution
    offsets = _offsets(window_xy, res)
    ny, nx = field.fine.shape

    best_score, best, evaluated = -1.0, (gx, gy, gtheta), 0
    near = []
    for theta in _thetas(gtheta, window_theta, theta_step):
        rotated = scan_mod.transform(points, (gx, gy, theta))
        base = field.world_to_cell(rotated)
        for oy in offsets:
            for ox in offsets:
                fx = base[:, 0] + ox
                fy = base[:, 1] + oy
                ok = (fx >= 0) & (fx < nx) & (fy >= 0) & (fy < ny)
                if not ok.any():
                    continue
                score = float(field.fine[fy[ok], fx[ok]].sum() / n_points)
                evaluated += 1
                pose = (gx + ox * res, gy + oy * res,
                        scan_mod.wrap_angle(theta))
                near.append((pose[0], pose[1], pose[2], score))
                if score > best_score:
                    best_score = score
                    best = pose

    tied = [(x, y, th) for x, y, th, sc in near
            if sc >= best_score - AMBIGUITY_BAND]
    ambiguity, amb_theta, converged = _verdict(
        best_score, best[2], tied, min_score, max_spread, max_spread_theta)
    return MatchResult(best, best_score, ambiguity, amb_theta, converged,
                       n_points, evaluated)
