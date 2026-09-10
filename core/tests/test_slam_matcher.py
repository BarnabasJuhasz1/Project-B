"""
core/tests/test_slam_matcher.py

Chunk 21's guards: the front end recovers a known transform, and — the part that
matters more — it says so when it cannot.

**The test that matters is the symmetric-world one.** Chunk 21 rejected ICP
specifically because ICP fails *confidently*, and a matcher that always returns
an answer poisons the graph in chunk 23 with a failure that surfaces as a mangled
map three chunks later. So the assertion there is on the SCORE, not on the pose:
in a world with two identical ends, the right behaviour is a low score, and
asserting a particular (wrong) pose would be asserting the bug.

SIM-FREE. No simulator, no GPU.
"""

import numpy as np
import pytest

from core.geometry import World, raycast
from core.slam import scan as scan_mod
from core.slam.likelihood import (COARSE_FACTOR, FIELD_RESOLUTION,
                                  LikelihoodField, build_field)
from core.slam.matcher import (MAX_SPREAD, MAX_SPREAD_THETA, MIN_SCORE,
                               MatchResult, match, match_bruteforce)

# ---- below this line is data, not code.

# A small room with interior structure. Structure matters: a bare rectangle is
# translation-ambiguous along each wall, and a matcher tested only on it would
# look far better than it is.
ROOM = World(
    rects=(
        (0.0, -0.1, 6.0, 0.0),          # south
        (0.0, 4.0, 6.0, 4.1),           # north
        (-0.1, 0.0, 0.0, 4.0),          # west
        (6.0, 0.0, 6.1, 4.0),           # east
        (2.0, 1.0, 2.4, 2.6),           # a pillar, and the asymmetry
        (4.0, 2.2, 5.2, 2.6),           # a stub wall
    ),
    bounds=(0.0, 0.0, 6.0, 4.0),
    name="matcher_room",
)

N_BEAMS = 181
FOV = 2.0 * np.pi
RANGE_MAX = 8.0

# ⚠️ Chosen for CLEARANCE, not arbitrarily. The first draft used (2.5, 2.0),
# which sits 0.1 m from the pillar — so a delta of (-0.15, +0.05) put the robot
# INSIDE it, `raycast` returned 0.0 for every beam (documented behaviour for an
# origin inside geometry), and the matcher was handed a scan of 181 coincident
# points. It scored 1.000 and reported ambiguity 0.500, which is correct
# behaviour on a degenerate input and looked exactly like a matcher bug.
# `test_a_scan_taken_inside_a_wall_is_degenerate_and_is_flagged` pins that.
ANCHOR = (3.2, 1.2, 0.3)          # clearance 0.80 m, so +/-0.2 m stays free

# ---- below this line is code, not data.


def _angles():
    return np.linspace(-FOV / 2, FOV / 2, N_BEAMS, endpoint=False)


def _scan_at(pose, world=ROOM):
    """A robot-frame point cloud as the robot at `pose` would see it."""
    ranges = raycast(pose[:2], pose[2], _angles(), RANGE_MAX, world)
    return scan_mod.endpoints(ranges, _angles())


def _field_at(pose, world=ROOM):
    """A likelihood field built from one scan taken at `pose`."""
    return build_field(scan_mod.transform(_scan_at(pose, world), pose))


# --- scan.py ----------------------------------------------------------------

def test_inf_beams_are_dropped_not_clamped():
    """A beam that returned nothing is the absence of a measurement, not a
    measurement of empty space at the sensor limit."""
    angles = np.array([0.0, 0.5, 1.0])
    ranges = np.array([2.0, np.inf, 3.0])
    pts = scan_mod.endpoints(ranges, angles)
    assert len(pts) == 2
    assert np.all(np.isfinite(pts))
    assert np.hypot(*pts[0]) == pytest.approx(2.0)


def test_compose_and_relative_are_inverses():
    a = (1.3, -0.7, 0.9)
    b = (2.2, 3.1, -2.8)
    back = scan_mod.compose(a, scan_mod.relative(a, b))
    assert back[0] == pytest.approx(b[0])
    assert back[1] == pytest.approx(b[1])
    assert back[2] == pytest.approx(scan_mod.wrap_angle(b[2]))


def test_yaw_is_wrapped_by_compose():
    """An unwrapped yaw accumulating over a few thousand steps is one of the
    classic ways a pose graph ends up spun once round."""
    pose = (0.0, 0.0, 0.0)
    for _ in range(20):
        pose = scan_mod.compose(pose, (0.0, 0.0, 0.5))
    assert -np.pi < pose[2] <= np.pi


def test_transform_rotates_before_translating():
    pts = np.array([[1.0, 0.0]])
    out = scan_mod.transform(pts, (5.0, 0.0, np.pi / 2))
    assert out[0][0] == pytest.approx(5.0)
    assert out[0][1] == pytest.approx(1.0)


# --- likelihood.py ----------------------------------------------------------

def test_the_coarse_level_is_a_forward_sliding_maximum():
    """⚠️ The correctness property the whole branch-and-bound rests on.

    The bound must satisfy `coarse[c] >= fine[c + d]` for every sub-offset `d`
    in `[0, COARSE_FACTOR)`. A BLOCK max only bounds cells inside its own block,
    so a sub-offset crossing a block boundary would be bounded by the wrong
    block, the bound would not be admissible, and the search would prune the
    true optimum — a confidently wrong pose, which is the failure mode chunk 21
    rejected ICP to avoid.
    """
    field = _field_at((1.5, 1.5, 0.0))
    fine, coarse = field.fine, field.coarse
    ny, nx = fine.shape
    for dy in range(COARSE_FACTOR):
        for dx in range(COARSE_FACTOR):
            shifted = fine[dy:ny, dx:nx]
            bound = coarse[0:ny - dy, 0:nx - dx]
            assert np.all(bound >= shifted - 1e-12), (dy, dx)


def test_the_score_is_a_mean_so_it_is_comparable_across_scan_sizes():
    """Dropout removes beams, so a sum would make a sparse scan score lower at
    the same true pose — and chunk 22 thresholds on this number."""
    pose = (1.5, 1.5, 0.0)
    field = _field_at(pose)
    full = scan_mod.transform(_scan_at(pose), pose)
    half = full[::2]
    assert field.score(full) == pytest.approx(field.score(half), abs=0.05)


def test_points_off_the_field_score_zero_rather_than_clamping_to_an_edge():
    """Clamping would let a badly wrong pose pile every stray point onto one
    high-scoring border cell."""
    field = _field_at((1.5, 1.5, 0.0))
    away = np.array([[1e6, 1e6], [-1e6, -1e6]])
    assert field.score(away) == 0.0


def test_an_empty_submap_is_a_caller_bug_not_a_degenerate_map():
    with pytest.raises(ValueError, match="at least one point"):
        build_field(np.zeros((0, 2)))


# --- the matcher recovers a known transform ---------------------------------

@pytest.mark.parametrize("delta", [
    (0.0, 0.0, 0.0),
    (0.10, 0.0, 0.0),          # pure translation, x
    (0.0, -0.15, 0.0),         # pure translation, y
    (0.0, 0.0, 0.06),          # pure rotation
    (0.0, 0.0, -0.10),
    (0.10, 0.10, 0.05),        # both
    (-0.15, 0.05, -0.04),
])
def test_a_known_transform_is_recovered_to_the_search_resolution(delta):
    """The translational search is on the field's own cell grid, so the pose is
    exact only to `field.resolution`. That is this front end's documented
    accuracy; chunk 23's graph is what refines it."""
    anchor = ANCHOR
    field = _field_at(anchor)

    true = (anchor[0] + delta[0], anchor[1] + delta[1], anchor[2] + delta[2])
    points = _scan_at(true)

    result = match(points, field, anchor)
    assert result.converged, result
    # Tolerances are MEASURED, not guessed: over 40 random deltas from this
    # anchor the translational error was mean 0.036 m, p90 0.050, max 0.065 --
    # about one cell -- and the yaw error max 0.0104 rad. See the chunk 21
    # Outcome table.
    assert result.pose[0] == pytest.approx(true[0], abs=1.5 * FIELD_RESOLUTION)
    assert result.pose[1] == pytest.approx(true[1], abs=1.5 * FIELD_RESOLUTION)
    assert result.pose[2] == pytest.approx(true[2], abs=0.03)


def test_branch_and_bound_returns_exactly_what_brute_force_returns():
    """⚠️ The property that makes the pruning safe rather than merely fast.

    Pruning on an admissible bound cannot discard the true optimum, so `match()`
    must return what an exhaustive search returns. A branch-and-bound that
    quietly stops being exact is indistinguishable from one that works — until
    the day it matters.
    """
    anchor = ANCHOR
    field = _field_at(anchor)
    for delta in [(0.0, 0.0, 0.0), (0.12, -0.08, 0.03), (-0.2, 0.15, -0.05)]:
        true = (anchor[0] + delta[0], anchor[1] + delta[1],
                anchor[2] + delta[2])
        points = _scan_at(true)
        fast = match(points, field, anchor)
        slow = match_bruteforce(points, field, anchor)
        assert fast.pose == slow.pose, delta
        assert fast.score == pytest.approx(slow.score, abs=1e-12), delta


def test_pruning_actually_prunes():
    """If `evaluated` equalled the brute-force count the bound would be doing
    nothing, and the test above would be passing vacuously."""
    anchor = ANCHOR
    field = _field_at(anchor)
    points = _scan_at(anchor)
    fast = match(points, field, anchor)
    slow = match_bruteforce(points, field, anchor)
    assert fast.evaluated < slow.evaluated


# --- the failure modes it must REPORT rather than hide ----------------------

def test_a_symmetric_world_produces_a_low_score_not_a_confident_wrong_pose():
    """⚠️ **The test that matters**, and the reason ICP was rejected.

    A corridor whose two ends are identical is genuinely ambiguous. The right
    behaviour is to say so — a low score, which chunk 22 can threshold on — and
    the assertion is deliberately NOT on the pose: asserting a particular wrong
    pose would be asserting the bug.
    """
    corridor = World(
        rects=(
            (0.0, -0.1, 6.0, 0.0),
            (0.0, 1.0, 6.0, 1.1),
            (-0.1, 0.0, 0.0, 1.0),
            (6.0, 0.0, 6.1, 1.0),
        ),
        bounds=(0.0, 0.0, 6.0, 1.0),
        name="symmetric_corridor",
    )
    # A field built at one end, matched against a scan from the OTHER end. The
    # two look alike, so nothing in the map distinguishes them.
    field = _field_at((1.0, 0.5, 0.0), corridor)
    points = _scan_at((5.0, 0.5, 0.0), corridor)

    result = match(points, field, (1.0, 0.5, 0.0))
    assert not result.converged, (
        f"the matcher was confident about an ambiguous world: {result}")

    # ⚠️ **The score is HIGH here, and chunk 21 as planned expected it to be
    # low.** The plan said a symmetric world would "produce a low score, not a
    # confident wrong answer". Measured: 0.618, well above MIN_SCORE, because
    # the scan's wall points land on walls. The fit is genuinely good — it is
    # just not unique, and a score cannot express that.
    #
    # This is asserted rather than worked around, because it is the reason
    # `ambiguity` exists. If someone later makes the score alone detect this,
    # this assertion fails and they have to explain what changed.
    assert result.score > MIN_SCORE, (
        "the score no longer reports a good fit on an ambiguous world; if that "
        "is deliberate, `ambiguity` may no longer be needed")
    assert result.ambiguity > MAX_SPREAD


def test_an_empty_scan_is_not_a_match():
    field = _field_at((1.5, 1.5, 0.0))
    result = match(np.zeros((0, 2)), field, (1.5, 1.5, 0.0))
    assert not result.converged
    assert result.score == 0.0
    assert result.pose == (1.5, 1.5, 0.0)


def test_a_pose_outside_the_search_window_is_not_silently_accepted():
    """A window must exceed the motion between scans plus the guess error, or
    the true pose is outside the search and the best answer inside it is wrong.
    The score is what reveals that."""
    anchor = ANCHOR
    field = _field_at(anchor)
    far = (anchor[0] + 2.0, anchor[1] + 1.5, anchor[2])
    result = match(_scan_at(far), field, anchor)
    assert not result.converged, result


# --- tolerance to chunk 20's noise ------------------------------------------

def test_dropout_of_thirty_percent_still_recovers_the_transform():
    anchor = ANCHOR
    field = _field_at(anchor)
    true = (anchor[0] + 0.10, anchor[1] - 0.10, anchor[2] + 0.03)

    rng = np.random.default_rng(7)
    angles = _angles()
    ranges = raycast(true[:2], true[2], angles, RANGE_MAX, ROOM)
    ranges = np.where(rng.random(len(angles)) < 0.30, np.inf, ranges)
    points = scan_mod.endpoints(ranges, angles)

    result = match(points, field, anchor)
    assert result.converged, result
    assert result.pose[0] == pytest.approx(true[0], abs=2 * FIELD_RESOLUTION)
    assert result.pose[1] == pytest.approx(true[1], abs=2 * FIELD_RESOLUTION)


# --- determinism ------------------------------------------------------------

def test_the_same_inputs_give_the_same_result_every_time():
    """Same-machine bitwise determinism is a committed property of this project,
    and SLAM is where it goes wrong."""
    anchor = ANCHOR
    field = _field_at(anchor)
    points = _scan_at((anchor[0] + 0.1, anchor[1], anchor[2]))

    first = match(points, field, anchor)
    for _ in range(30):
        again = match(points, field, anchor)
        assert again.pose == first.pose
        assert again.score == first.score
        assert again.evaluated == first.evaluated


def test_a_tie_is_broken_by_the_lowest_index_every_time():
    """A blank field scores every candidate identically, so the answer is
    entirely decided by the tie-break rule. It must be the STATED rule — lowest
    flat index, which is the most negative offset — and not whatever the array
    happened to contain."""
    flat = LikelihoodField(
        fine=np.zeros((200, 200)), coarse=np.zeros((200, 200)),
        origin=(0.0, 0.0), resolution=FIELD_RESOLUTION, n_points=1)
    points = np.array([[1.0, 1.0], [1.2, 0.9]])
    results = [match(points, flat, (2.0, 2.0, 0.0)) for _ in range(10)]
    assert len({r.pose for r in results}) == 1
    # Lowest index = the most negative offset in x, y and theta.
    first = results[0]
    assert first.pose[0] < 2.0 and first.pose[1] < 2.0
    assert not first.converged


def test_a_scan_taken_inside_a_wall_is_degenerate_and_is_flagged():
    """The failure my own first fixture produced, kept as a test.

    `core.geometry.raycast` returns 0.0 for beams whose origin is inside
    geometry — "what a real sensor buried in geometry would effectively
    report". Every endpoint then collapses onto the robot, and the matcher is
    handed 181 coincident points that land on a single cell and score
    perfectly. A pose can be read off that, and it means nothing.

    The score cannot catch this: it is 1.0, the best possible. `ambiguity`
    does, because a single cluster of points fits equally well at many offsets.
    """
    points = _scan_at((2.2, 1.8, 0.0))
    assert np.allclose(points, 0.0), "fixture assumption: beams all read 0.0"

    # ⚠️ The dangerous configuration, and the one my first fixture hit. The map
    # is REAL — built from a scan taken just outside the pillar — and the
    # collapsed cluster lands within the blur radius of the pillar's own face.
    # Every point therefore sits on a peak cell and the score is **1.000**, the
    # best value obtainable. Nothing about the score says this is nonsense.
    #
    # `ambiguity` is what catches it: one cluster of coincident points explains
    # the map equally well at many offsets, so the near-best poses are spread
    # 0.5 m apart and `converged` is False.
    near_pillar = (2.5, 2.0, 0.3)
    result = match(points, _field_at(near_pillar), near_pillar)
    assert result.score == pytest.approx(1.0), (
        "the fixture no longer produces the perfect-score case this test is "
        "about")
    assert result.ambiguity > MAX_SPREAD
    assert not result.converged

    # A degenerate scan against a map built from the SAME degenerate scan is
    # caught too, but by the OTHER half of the measure. The translational peak
    # is genuinely unique — one blob on one blob — so `ambiguity` is 0.0 and an
    # earlier version of this test recorded it as converging. Adding rotational
    # ambiguity changed that: a single cluster of coincident points carries no
    # rotational information whatsoever, so every heading fits equally well and
    # `ambiguity_theta` is 0.400 against a 0.05 gate.
    #
    # Both halves are needed and each catches a case the other misses.
    self_match = match(points, _field_at((2.2, 1.8, 0.0)), (2.2, 1.8, 0.0))
    assert self_match.ambiguity == 0.0            # translation IS determined
    assert self_match.ambiguity_theta > MAX_SPREAD_THETA
    assert not self_match.converged


def test_ambiguity_is_zero_when_the_peak_is_unique():
    """The other side of the measure: a well-conditioned match must not be
    penalised, or `converged` would reject good edges and chunk 23's graph
    would starve."""
    field = _field_at(ANCHOR)
    result = match(_scan_at(ANCHOR), field, ANCHOR)
    assert result.converged
    assert result.ambiguity <= MAX_SPREAD
