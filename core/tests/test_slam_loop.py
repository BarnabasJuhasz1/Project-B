"""
core/tests/test_slam_loop.py

Chunk 22's guards: submaps cut identically every run, and a closure is refused
far more readily than it is accepted.

⚠️ **The asymmetry is the design.** Missing a true closure costs some accuracy;
accepting a false one costs the map — the graph folds two different places onto
each other and every pose after it is structurally wrong. So most of what is
asserted here is a refusal, and each gate is exercised in isolation with the
other two satisfied.

SIM-FREE. No simulator, no GPU.
"""

import numpy as np
import pytest

from core.geometry import World, raycast
from core.slam import loop, scan as scan_mod
from core.slam.submap import SCANS_PER_SUBMAP, Submap

# ---- below this line is data, not code.

# A room with enough structure that a match is well conditioned. Same shape as
# chunk 21's fixture, and for the same reason: a bare rectangle is
# translation-ambiguous along every wall.
ROOM = World(
    rects=(
        (0.0, -0.1, 8.0, 0.0),
        (0.0, 6.0, 8.0, 6.1),
        (-0.1, 0.0, 0.0, 6.0),
        (8.0, 0.0, 8.1, 6.0),
        (2.0, 1.5, 2.5, 3.5),           # pillar
        (5.0, 3.0, 6.5, 3.5),           # stub
        (3.5, 4.5, 4.0, 5.5),           # another asymmetry
    ),
    bounds=(0.0, 0.0, 8.0, 6.0),
    name="loop_room",
)

# The chunk-21 corridor: two ends that look alike. The false-closure hazard.
CORRIDOR = World(
    rects=(
        (0.0, -0.1, 8.0, 0.0),
        (0.0, 1.2, 8.0, 1.3),
        (-0.1, 0.0, 0.0, 1.2),
        (8.0, 0.0, 8.1, 1.2),
    ),
    bounds=(0.0, 0.0, 8.0, 1.2),
    name="symmetric_corridor",
)

N_BEAMS = 181
FOV = 2.0 * np.pi
RANGE_MAX = 10.0

# ---- below this line is code, not data.


def _angles():
    return np.linspace(-FOV / 2, FOV / 2, N_BEAMS, endpoint=False)


def _scan_at(pose, world=ROOM):
    return scan_mod.endpoints(
        raycast(pose[:2], pose[2], _angles(), RANGE_MAX, world), _angles())


def _submap_at(origin, index=0, world=ROOM, spread=0.05):
    """A finished submap built from scans taken around `origin`.

    The scans are nudged apart so the submap is a small local map rather than
    one scan repeated — which is what a real submap is, and what makes matching
    against it well conditioned.
    """
    sub = Submap(origin, index=index)
    for i in range(SCANS_PER_SUBMAP):
        pose = (origin[0] + spread * np.cos(i), origin[1] + spread * np.sin(i),
                origin[2])
        sub.add(_scan_at(pose, world), pose)
    return sub


# --- submaps ----------------------------------------------------------------

def test_boundaries_are_a_pure_function_of_scan_count():
    """Not distance, not rotation, not match score. Those all make submap
    boundaries depend on the trajectory, so two runs differing by one control
    step would cut differently and produce different graphs — and this
    project's determinism guarantee is bitwise."""
    sub = Submap((0.0, 0.0, 0.0))
    for i in range(SCANS_PER_SUBMAP):
        assert not sub.finished, i
        sub.add(_scan_at((1.0 + 0.3 * i, 1.0, 0.0)), (1.0 + 0.3 * i, 1.0, 0.0))
    assert sub.finished
    assert len(sub) == SCANS_PER_SUBMAP


def test_a_finished_submap_refuses_more_scans():
    sub = _submap_at((1.5, 1.0, 0.0))
    with pytest.raises(ValueError, match="already holds"):
        sub.add(_scan_at((1.5, 1.0, 0.0)), (1.5, 1.0, 0.0))


def test_points_are_stored_locally_so_relocation_is_free():
    """Chunk 23's optimiser moves origins. Points stored in the submap's own
    frame come with it, with no re-accumulation and no loss — which is what
    makes re-optimising a graph cheap."""
    sub = _submap_at((2.0, 2.0, 0.0))
    local_before = sub.points().copy()
    field_before = sub.field()

    sub.relocate((5.0, 4.0, 0.5))

    assert np.array_equal(sub.points(), local_before)
    # Same field object: it is expressed in the submap frame and nothing moved
    # relative to the origin.
    assert sub.field() is field_before
    # The global view DID move.
    assert not np.allclose(sub.global_points()[:5], local_before[:5])


def test_two_identical_scan_sequences_produce_identical_submaps():
    a = _submap_at((2.0, 2.0, 0.3), index=0)
    b = _submap_at((2.0, 2.0, 0.3), index=0)
    assert np.array_equal(a.points(), b.points())
    assert np.array_equal(a.field().fine, b.field().fine)


# --- candidate proposal -----------------------------------------------------

# ⚠️ `min_travel=0.0` appears in the three tests below and it is not an
# oversight. Their fixtures stack every submap on ONE origin, which is precisely
# the case chunk 22a's travel gate now suppresses — so with the default they
# would return nothing and would silently stop testing the index gate and the
# search radius, which is what they are about. The travel gate has its own
# tests further down.

def test_a_recent_submap_is_not_a_loop_it_is_the_trajectory():
    """Without the gap, every submap would 'close' against its immediate
    predecessor, which is odometry wearing a different name."""
    subs = [_submap_at((2.0, 2.0, 0.0), index=i) for i in range(4)]
    near = loop.candidates(subs, (2.0, 2.0, 0.0), current_index=4,
                           min_travel=0.0)
    assert [s.index for s in near] == [0, 1, 2]
    assert 3 not in [s.index for s in near]


def test_unfinished_submaps_are_never_candidates():
    partial = Submap((2.0, 2.0, 0.0), index=0)
    partial.add(_scan_at((2.0, 2.0, 0.0)), (2.0, 2.0, 0.0))
    assert loop.candidates([partial], (2.0, 2.0, 0.0), current_index=9,
                           min_travel=0.0) == []


def test_the_search_radius_grows_with_uncertainty_but_is_bounded():
    far = _submap_at((2.0, 2.0, 0.0), index=0)
    pose = (2.0 + loop.SEARCH_RADIUS + 1.0, 2.0, 0.0)
    assert loop.candidates([far], pose, current_index=9, min_travel=0.0) == []
    assert loop.candidates([far], pose, current_index=9, uncertainty=1.0,
                           min_travel=0.0)
    # Bounded: a badly drifted estimate must not propose every submap on the map.
    very_far = _submap_at((2.0, 2.0, 0.0), index=0)
    miles = (2.0 + loop.MAX_SEARCH_RADIUS + 1.0, 2.0, 0.0)
    assert loop.candidates([very_far], miles, current_index=9,
                           uncertainty=100.0, min_travel=0.0) == []


# --- the travel gate (chunk 22a) --------------------------------------------

def test_a_stationary_robot_proposes_no_closures_however_many_submaps_it_makes():
    """⚠️ THE REGRESSION THIS GATE EXISTS FOR.

    Chunk 24b measured it under physics: turning on the spot for 40 s finished
    nine submaps inside a 0.13 x 0.19 m box and closed 37 of them against each
    other, all four gates correctly satisfied. Every one valid, every one
    worthless. `MIN_SUBMAP_GAP` cannot see it, because a submap boundary is a
    scan count and a stationary robot still finishes submaps.
    """
    subs = [_submap_at((2.0, 2.0, 0.0), index=i) for i in range(9)]
    assert loop.candidates(subs, (2.0, 2.0, 0.0), current_index=9) == []


def test_a_real_revisit_is_still_a_candidate_even_from_the_exact_same_spot():
    """⚠️ THE THING THE OBVIOUS FIX WOULD HAVE BROKEN.

    Gating on the DISTANCE between the two submaps is the natural reading of
    "the closures were 10 cm apart" and it is exactly wrong: a revisit IS being
    in the same place. On chunk 24b's genuine closure set the median separation
    is 2.50 m and the minimum is 0.02 m. This is that case — a submap the robot
    returns to having driven a long way round — and it must survive.
    """
    old = _submap_at((2.0, 2.0, 0.0), index=0)
    away = [_submap_at((2.0 + 1.5 * k, 2.0, 0.0), index=k)
            for k in range(1, 4)]
    # Back to exactly where submap 0 sits, having travelled there and back.
    near = loop.candidates([old] + away, (2.0, 2.0, 0.0), current_index=9)
    assert 0 in [s.index for s in near]


def test_travel_is_measured_along_the_trajectory_not_across_it():
    subs = [_submap_at((2.0 + 1.5 * k, 2.0, 0.0), index=k) for k in range(3)]
    travel = loop.travel_to_pose(subs, (2.0, 2.0, 0.0))
    # Submap 2 sits 3.0 m from submap 0, but the robot went out to 5.0 and
    # came back, so the chain from submap 0 is 3.0 out plus 3.0 back.
    assert travel[-1] == pytest.approx(3.0)          # newest -> pose
    assert travel[0] == pytest.approx(6.0)           # oldest -> pose
    assert travel[0] > travel[1] > travel[2]


def test_travel_to_pose_on_an_empty_map_is_empty():
    assert loop.travel_to_pose([], (0.0, 0.0, 0.0)) == []


def test_a_suppressed_candidate_is_counted_and_is_not_a_rejection():
    """`loop.py` promises "the loop was there and we did not see it" stays
    distinguishable from "we saw it and rejected it". A filter running BEFORE
    the match would erase the first, so it gets its own counter — and it must
    not land in `rejected`, where it would look like a verdict on a scan."""
    log = loop.ClosureLog()
    subs = [_submap_at((2.0, 2.0, 0.0), index=i) for i in range(9)]
    assert loop.candidates(subs, (2.0, 2.0, 0.0), current_index=9,
                           log=log) == []
    # Indices 0..7 clear MIN_SUBMAP_GAP against current_index=9 and are then
    # suppressed for travel; index 8 never reaches the travel test.
    assert log.suppressed == 8
    assert log.considered == 0
    assert log.accepted == 0
    assert sum(log.rejected.values()) == 0
    assert log.as_dict()["closures_suppressed_travel"] == 8


def test_the_gate_is_two_metres_and_that_is_not_a_free_parameter():
    """It is `MIN_SUBMAP_GAP` in the currency that does not lie when the robot
    stands still: measured, ordinary driving covers 0.92-1.73 m per submap, so
    a gap of two ALREADY means about two metres whenever the robot is moving."""
    assert loop.MIN_SUBMAP_TRAVEL == 2.0
    assert loop.MIN_SUBMAP_GAP == 2


def test_candidates_come_back_in_a_fixed_order():
    """The first accepted closure is the one applied, so an unordered candidate
    set would make the result depend on iteration order."""
    subs = [_submap_at((2.0, 2.0, 0.0), index=i) for i in (5, 1, 3, 0)]
    got = loop.candidates(subs, (2.0, 2.0, 0.0), current_index=9)
    assert [s.index for s in got] == sorted(s.index for s in got)


# --- the three gates, each in isolation -------------------------------------

def test_a_true_revisit_is_accepted():
    """The positive control. If this fails, the gates are simply too tight and
    no closure would ever be accepted — which would look like 'SLAM does not
    help' in chunk 23's numbers."""
    origin = (2.0, 4.5, 0.0)
    sub = _submap_at(origin, index=0)
    # Come back to almost the same place, slightly off, as a drifted robot would.
    revisit = (origin[0] + 0.10, origin[1] - 0.08, origin[2] + 0.03)
    proposal = loop.propose(sub, _scan_at(revisit), revisit, uncertainty=0.25)
    assert proposal.accepted, proposal


def test_gate_1_refuses_a_scan_that_does_not_fit_at_all():
    sub = _submap_at((2.0, 4.5, 0.0), index=0)
    # A scan from a completely different part of the room.
    elsewhere = (6.8, 1.0, 1.2)
    proposal = loop.propose(sub, _scan_at(elsewhere), (2.0, 4.5, 0.0),
                            uncertainty=1.0)
    assert not proposal.accepted
    assert proposal.reason == "score"


def test_gate_1b_refuses_an_ambiguous_match_in_a_corridor():
    """⚠️ Chunk 21's finding, doing the work it was added for.

    A featureless corridor scores WELL from anywhere along itself — the wall
    points land on walls — so gate 1 alone would accept a closure between two
    ends that are metres apart. The ambiguity gate is what refuses it.
    """
    sub = _submap_at((1.2, 0.6, 0.0), index=0, world=CORRIDOR, spread=0.03)
    other_end = (6.5, 0.6, 0.0)
    proposal = loop.propose(sub, _scan_at(other_end, CORRIDOR), other_end,
                            uncertainty=5.0)
    assert not proposal.accepted, proposal
    assert proposal.reason in ("ambiguity", "score", "window_edge"), proposal


def test_gate_2_refuses_a_match_on_the_window_edge():
    """A match at the edge wanted to go further and was stopped by the window,
    not chosen by the data. Rejected rather than clamped: clamping would hand
    the graph a constraint the data does not support."""
    origin = (2.0, 4.5, 0.0)
    sub = _submap_at(origin, index=0)
    revisit = (origin[0] + 0.10, origin[1], origin[2])
    # The GUESS is `origin`, so the match has to travel the full 0.10 m — and
    # the window is exactly 0.10 m, so the answer sits on its edge. Passing
    # `revisit` as the guess instead would leave nothing to move and the gate
    # would never fire.
    proposal = loop.propose(sub, _scan_at(revisit), origin,
                            uncertainty=10.0, window_xy=0.10,
                            window_theta=0.05)
    assert not proposal.accepted
    assert proposal.reason == "window_edge"


def test_gate_3_refuses_a_correction_larger_than_the_believed_drift():
    """A closure proposing a large correction when the estimate says it has
    barely drifted is asserting the estimate was wrong by an order of
    magnitude. Far likelier the match is wrong."""
    origin = (2.0, 4.5, 0.0)
    sub = _submap_at(origin, index=0)
    # A real revisit offset by ~0.35 m, but the robot is CERTAIN of its pose,
    # so a correction that large is not credible.
    revisit = (origin[0] + 0.35, origin[1], origin[2])
    proposal = loop.propose(sub, _scan_at(revisit), origin, uncertainty=0.0)
    assert not proposal.accepted, proposal
    assert proposal.reason == "correction_too_large"


def test_the_same_match_is_accepted_once_the_uncertainty_justifies_it():
    """Gate 3 is about plausibility, not about distance. The identical
    geometry must pass once the robot admits it might be that lost."""
    origin = (2.0, 4.5, 0.0)
    sub = _submap_at(origin, index=0)
    revisit = (origin[0] + 0.35, origin[1], origin[2])
    tight = loop.propose(sub, _scan_at(revisit), origin, uncertainty=0.0)
    loose = loop.propose(sub, _scan_at(revisit), origin, uncertainty=0.5)
    assert not tight.accepted
    assert loose.accepted, loose


# --- lab42's own trap -------------------------------------------------------

def test_the_lab42_divider_does_not_produce_a_false_closure():
    """lab42's divider at x = 6 and the wall along y = 5 make two structurally
    similar corridors — a realistic false-closure hazard sitting in the
    benchmark scene. Better tested on purpose than met during evaluation."""
    from core.geometry import LAB42

    # Two places that look alike: either side of the divider, same local shape.
    a = (5.0, 2.5, 0.0)
    b = (7.0, 2.5, 0.0)
    sub = _submap_at(a, index=0, world=LAB42)
    proposal = loop.propose(sub, _scan_at(b, LAB42), b, uncertainty=2.0)
    assert not proposal.accepted, (
        f"the divider produced a false closure: {proposal}")


# --- the log ----------------------------------------------------------------

def test_every_rejection_is_counted_by_gate():
    """'The loop was there and we did not see it' and 'we saw it and rejected
    it' are different failures with different fixes, and nothing else can tell
    them apart."""
    log = loop.ClosureLog()
    origin = (2.0, 4.5, 0.0)
    sub = _submap_at(origin, index=0)

    log.record(loop.propose(sub, _scan_at((6.8, 1.0, 1.2)), origin,
                            uncertainty=1.0))                     # score
    log.record(loop.propose(sub, _scan_at((origin[0] + 0.35, origin[1],
                                           origin[2])), origin,
                            uncertainty=0.0))                     # correction
    revisit = (origin[0] + 0.08, origin[1], origin[2])
    log.record(loop.propose(sub, _scan_at(revisit), revisit, uncertainty=0.25))

    assert log.considered == 3
    assert log.accepted == 1
    assert log.rejected["score"] == 1
    assert log.rejected["correction_too_large"] == 1

    flat = log.as_dict()
    assert flat["closures_accepted"] == 1
    assert flat["closures_considered"] == 3
    # Every gate has a key even at zero, so `slam.csv` has a fixed column set.
    for reason in loop.REJECT_REASONS:
        assert f"closures_rejected_{reason}" in flat


def test_the_diagnostics_keys_are_fixed_width():
    """`slam.csv` is written from the first row's keys, so a counter appearing
    only after the first closure would misalign every later row."""
    assert set(loop.ClosureLog().as_dict()) == set(
        loop.ClosureLog().as_dict())
    fresh = loop.ClosureLog().as_dict()
    used = loop.ClosureLog()
    used.rejected["score"] = 3
    assert set(used.as_dict()) == set(fresh)


# --- the headline claim -----------------------------------------------------

def _drive_square(tracker, side=2.0, step=0.15, centre=(3.0, 3.0),
                  world=ROOM, accepted=None):
    """Walk a closed square, feeding the tracker a scan at every step.

    Returns the list of (true_pose, estimate) pairs. The square RETURNS TO ITS
    START, which is the whole point: a robot that revisits can correct, and one
    that does not cannot.
    """
    cx, cy = centre
    half = side / 2.0
    corners = [(cx - half, cy - half), (cx + half, cy - half),
               (cx + half, cy + half), (cx - half, cy + half),
               (cx - half, cy - half)]
    trail = []
    if accepted is None:
        accepted = []
    dt = 0.2
    for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
        heading = np.arctan2(y1 - y0, x1 - x0)
        n = max(1, int(np.hypot(x1 - x0, y1 - y0) / step))
        for i in range(n):
            t = i / n
            pose = (x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, heading)
            pts = _scan_at(pose, world)
            est = tracker.update(dt, pose, pts)
            # Proposals are collected AS THE ROBOT PASSES, not afterwards: the
            # tracker's pose moves, so proposing from the end-of-lap pose with a
            # mid-lap scan compares two different places.
            for proposal in tracker.propose_closures(pts):
                # The chained estimate AT THIS MOMENT is what the closure would
                # replace, so it has to be captured here and not read back
                # afterwards.
                accepted.append((pose, est, proposal))
            trail.append((pose, est))
    return trail


def _err(a, b):
    return float(np.hypot(a[0] - b[0], a[1] - b[1]))


def test_noisy_odometry_alone_accumulates_error():
    """The control case, and the thing that makes the mechanism attributable.

    Averaged over seeds, because a random walk WANDERS: on any single seed the
    error at step 60 is regularly below the error at step 5, and asserting on
    one seed makes a flaky test out of a real property.
    """
    from core.slam.tracker import Tracker

    early, late = [], []
    for seed in range(12):
        tracker = Tracker((2.0, 2.0, 0.0), np.random.default_rng(seed),
                          odom_sigma_lin=0.08, odom_sigma_ang=0.04)
        for i in range(60):
            pose = (2.0 + 0.05 * i, 2.0, 0.0)
            est = tracker.update(0.2, pose, np.zeros((0, 2)))
            if i == 5:
                early.append(_err(pose, est))
        late.append(_err(pose, est))
    assert np.mean(late) > np.mean(early), (np.mean(early), np.mean(late))


def test_a_thin_submap_is_not_matched_against():
    """⚠️ Measured, and it was catastrophic before the guard existed.

    At 1-3 scans a submap's field is one scan's outline, and the next scan is
    the same outline — so the matcher reported **score 1.00** while placing the
    heading 0.6 rad wrong, because any rotation that slides points along a wall
    fits perfectly. The estimate then diverged to 1.4 m over one lap, worse than
    doing nothing at all.
    """
    from core.slam.tracker import Tracker, MIN_SCANS_TO_MATCH

    tracker = Tracker((2.0, 2.0, 0.0), np.random.default_rng(2))
    for i in range(MIN_SCANS_TO_MATCH):
        pose = (2.0 + 0.01 * i, 2.0, 0.0)
        tracker.update(0.2, pose, _scan_at(pose))
        assert tracker.last_match is None, (
            f"matched against a submap holding {len(tracker.current)} scans")
    pose = (2.0 + 0.01 * MIN_SCANS_TO_MATCH, 2.0, 0.0)
    tracker.update(0.2, pose, _scan_at(pose))
    assert tracker.last_match is not None


def test_tracking_does_not_diverge_over_a_closed_lap():
    """⚠️ **What chunk 22 actually promises, and it is not error reduction.**

    This chunk *detects and proposes* closures; applying them is the graph
    optimisation in chunk 23, and the two are kept apart so 22's gates can be
    tested without a solver in the loop. So tracking alone lands in the same
    place as dead reckoning — measured over one lap: mean error 0.105 m with
    scan matching against 0.107 m without.

    That is the correct outcome for this chunk and it is the reason chunk 23
    exists. What matters here is that matching does not make things *worse*,
    which it did before the thin-submap and rotational-ambiguity fixes: 1.43 m
    against dead reckoning's 0.11 m.
    """
    from core.slam.tracker import Tracker

    matched = _drive_square(
        Tracker((2.0, 2.0, 0.0), np.random.default_rng(11),
                odom_sigma_lin=0.06, odom_sigma_ang=0.03))
    blind_tracker = Tracker((2.0, 2.0, 0.0), np.random.default_rng(11),
                            odom_sigma_lin=0.06, odom_sigma_ang=0.03)
    blind = [(pose, blind_tracker.update(0.2, pose, np.zeros((0, 2))))
             for pose, _ in matched]

    matched_mean = np.mean([_err(t, e) for t, e in matched])
    blind_mean = np.mean([_err(t, e) for t, e in blind])
    # Not "better" -- comparable. A regression here means matching is actively
    # harming the estimate, which is the failure this test was written after.
    assert matched_mean < 2.0 * blind_mean, (matched_mean, blind_mean)
    assert max(_err(t, e) for t, e in matched) < 0.6


def test_an_accepted_closure_points_closer_to_truth_than_the_chain():
    """⚠️ **Chunk 22's headline claim, scoped to what this chunk owns.**

    The Done-when says "a closed loop reduces error", but this chunk does not
    apply corrections. What it CAN show — and what chunk 23 then acts on — is
    that the pose implied by an accepted closure is closer to truth than the
    chained estimate it would replace. If that were not true, applying closures
    in chunk 23 would make the map worse and the epic would have no mechanism.
    """
    from core.slam.tracker import Tracker

    tracker = Tracker((2.0, 2.0, 0.0), np.random.default_rng(4),
                      odom_sigma_lin=0.07, odom_sigma_ang=0.03)
    _drive_square(tracker)
    accepted = []
    _drive_square(tracker, accepted=accepted)          # a second lap: revisiting

    improved = compared = 0
    for true_pose, chained, proposal in accepted:
        submap = next(s for s in tracker.submaps
                      if s.index == proposal.submap_index)
        implied = scan_mod.compose(submap.origin, proposal.pose_in_submap)
        compared += 1
        improved += _err(true_pose, implied) <= _err(true_pose, chained)

    assert compared > 0, "no closure was accepted on a second lap of a square"
    assert improved / compared > 0.5, (
        f"only {improved}/{compared} accepted closures pointed closer to truth "
        f"than the chained estimate; applying these in chunk 23 would make the "
        f"map worse")


def test_accepted_closures_are_geometrically_correct():
    """⚠️ The safety property, and it replaces a test whose premise was wrong.

    The first version asserted that a robot which never revisits accepts NO
    closures. It accepted 40. That is not a bug: **lab42 and these micro-worlds
    are smaller than the sensor range**, so a submap two indices back is still
    in view, overlaps the current scan, and matching against it is a genuine
    constraint. "Loop closure" is not distinguishable from extended tracking in
    a room this size, and pretending otherwise would mean tuning the gates until
    a false premise held.

    What must be true instead is that every ACCEPTED closure is a correct one.
    """
    from core.slam.tracker import Tracker

    tracker = Tracker((0.6, 0.6, 0.0), np.random.default_rng(5),
                      odom_sigma_lin=0.05, odom_sigma_ang=0.02)
    bad = []
    for i in range(120):
        x = 0.6 + 0.05 * i
        if x > 7.0:
            break
        pose = (x, 0.6, 0.0)
        pts = _scan_at(pose)
        tracker.update(0.2, pose, pts)
        for proposal in tracker.propose_closures(pts):
            submap = next(s for s in tracker.submaps
                          if s.index == proposal.submap_index)
            implied = scan_mod.compose(submap.origin, proposal.pose_in_submap)
            if _err(pose, implied) > 0.75:
                bad.append((pose, implied, proposal))
    assert not bad, f"{len(bad)} accepted closures were geometrically wrong: {bad[:2]}"


def test_the_tracker_is_deterministic():
    from core.slam.tracker import Tracker

    def run():
        t = Tracker((2.0, 2.0, 0.0), np.random.default_rng(9),
                    odom_sigma_lin=0.05, odom_sigma_ang=0.02)
        trail = _drive_square(t)
        return [e for _, e in trail], t.diagnostics()

    a_trail, a_diag = run()
    b_trail, b_diag = run()
    assert a_trail == b_trail
    assert a_diag == b_diag


def test_diagnostics_have_a_fixed_key_set_from_the_first_step():
    """`slam.csv` is written from the first row's keys, so a counter that only
    appeared after the first closure would misalign every later row."""
    from core.slam.tracker import Tracker

    t = Tracker((2.0, 2.0, 0.0), np.random.default_rng(1))
    first = set(t.diagnostics())
    _drive_square(t)
    assert set(t.diagnostics()) == first
