"""
core/tests/test_metrics.py

Synthetic traces, hand-built, one per behaviour. No simulator involved — which
is the point of scoring being a pure function of a recorded trace.

The debouncing tests are the ones that matter most. A metric that counts one
long wall scrape as 400 collisions is not wrong by a little; it dominates the
whole secondary metric table and makes conditions incomparable.
"""

import math

import numpy as np
import pytest

from core.geometry import shortest_path, shortest_path_length

from core.metrics import (
    COLLISION_CLEARANCE, EVENT_RELEASE, FALL_HEIGHT, GOAL_RADIUS,
    LOC_FAIL_THRESHOLD, NEAR_MISS_CLEARANCE, STUCK_WINDOW, TRACE_COLUMNS,
    EpisodeTrace, score,
)
from core.observation import Command
from core.scenario import load

DT = 0.02                 # 50 Hz control, the rate the runner will use
WALK = Command(0.5, 0.0, 0.0)
HALT = Command(0.0, 0.0, 0.0)


def _trace(poses, command=WALK, clearance=1.0, contact=False, est=None, dt=DT):
    """Build a trace from a list of (x, y) with sane defaults for everything
    the test under construction does not care about."""
    tr = EpisodeTrace()
    for i, (x, y) in enumerate(poses):
        c = clearance(i) if callable(clearance) else clearance
        k = contact(i) if callable(contact) else contact
        cmd = command(i) if callable(command) else command
        e = est(i) if callable(est) else (x, y, 0.0)
        tr.append(i * dt, (x, y, 0.55, 0.0), cmd, c, k, e)
    return tr


def _straight(start, goal, n):
    return [(start[0] + (goal[0] - start[0]) * i / (n - 1),
             start[1] + (goal[1] - start[1]) * i / (n - 1)) for i in range(n)]


# --- success ---------------------------------------------------------------

def test_clean_run_scores_success_and_high_spl():
    """A straight line cuts the corner the legal route has to go round, so this
    also checks SPL is capped at 1 rather than exceeding it."""
    sc = load("doorway_traverse")
    tr = _trace(_straight((1.5, 1.5), sc.goal, 400))
    r = score(tr, sc)
    assert r["goal_reached"] and r["termination"] == "goal"
    assert r["collisions"] == 0 and r["near_misses"] == 0
    assert 0.9 <= r["spl"] <= 1.0
    assert r["time_to_goal"] == pytest.approx(r["duration"])


def test_goal_credited_at_the_radius_not_at_the_exact_point():
    sc = load("doorway_traverse")
    just_inside = (sc.goal[0] - GOAL_RADIUS * 0.9, sc.goal[1])
    assert score(_trace(_straight((1.5, 1.5), just_inside, 400)), sc)["goal_reached"]


def test_path_length_stops_at_the_goal():
    """Wandering after arrival must not depress SPL."""
    sc = load("doorway_traverse")
    poses = _straight((1.5, 1.5), sc.goal, 400) + _straight(sc.goal, (7.0, 6.0), 200)
    r = score(_trace(poses), sc)
    assert r["goal_reached"]
    assert r["path_length"] == pytest.approx(
        score(_trace(_straight((1.5, 1.5), sc.goal, 400)), sc)["path_length"], abs=0.1)


# --- failure terminations --------------------------------------------------

def test_never_arriving_is_a_timeout():
    sc = load("doorway_traverse")
    r = score(_trace(_straight((1.5, 1.5), (4.0, 4.0), 300)), sc)
    assert not r["goal_reached"]
    assert r["termination"] == "timeout"
    assert r["spl"] == 0.0
    assert r["time_to_goal"] is None


def test_falling_terminates_and_forfeits_the_goal():
    """A robot that tumbles across the goal marker has not solved the task."""
    sc = load("doorway_traverse")
    tr = EpisodeTrace()
    for i, (x, y) in enumerate(_straight((1.5, 1.5), sc.goal, 400)):
        z = 0.55 if i < 200 else FALL_HEIGHT - 0.01
        tr.append(i * DT, (x, y, z, 0.0), WALK, 1.0, False, (x, y, 0.0))
    r = score(tr, sc)
    assert r["termination"] == "fell"
    assert not r["goal_reached"]
    assert r["control_steps"] == 201


# --- collisions and near misses --------------------------------------------

def test_one_long_scrape_is_one_collision():
    """The debouncing check. 200 consecutive steps in contact is one event."""
    sc = load("doorway_traverse")
    tr = _trace(_straight((1.5, 1.5), (4.0, 1.5), 400),
                clearance=lambda i: 0.0 if 100 <= i < 300 else 1.0)
    assert score(tr, sc)["collisions"] == 1


def test_chatter_across_the_threshold_is_still_one_collision():
    """Alternating either side of COLLISION_CLEARANCE is one scrape, not 100."""
    sc = load("doorway_traverse")
    tr = _trace(_straight((1.5, 1.5), (4.0, 1.5), 400),
                clearance=lambda i: (0.0 if i % 2 else 0.2) if 100 <= i < 300 else 1.0)
    assert score(tr, sc)["collisions"] == 1


def test_two_separated_scrapes_are_two_collisions():
    sc = load("doorway_traverse")
    tr = _trace(_straight((1.5, 1.5), (4.0, 1.5), 400),
                clearance=lambda i: 0.0 if (50 <= i < 100 or 300 <= i < 350) else 1.0)
    assert score(tr, sc)["collisions"] == 2


def test_physics_contact_alone_counts_as_a_collision():
    """Analytic clearance is a circumscribing-circle proxy; a real contact
    report beats it, so the two are OR-ed."""
    sc = load("doorway_traverse")
    tr = _trace(_straight((1.5, 1.5), (4.0, 1.5), 400),
                contact=lambda i: 100 <= i < 200)
    assert score(tr, sc)["collisions"] == 1


def test_a_near_miss_is_not_also_counted_as_a_collision():
    """Passing close without touching: one near miss, no collision."""
    sc = load("doorway_traverse")
    near = 0.5 * (COLLISION_CLEARANCE + NEAR_MISS_CLEARANCE)
    tr = _trace(_straight((1.5, 1.5), (4.0, 1.5), 400),
                clearance=lambda i: near if 100 <= i < 200 else 1.0)
    r = score(tr, sc)
    assert (r["near_misses"], r["collisions"]) == (1, 0)


def test_a_collision_is_not_double_counted_as_a_near_miss():
    """Otherwise the two numbers move together and say nothing independent."""
    sc = load("doorway_traverse")
    tr = _trace(_straight((1.5, 1.5), (4.0, 1.5), 400),
                clearance=lambda i: 0.0 if 100 <= i < 200 else 1.0)
    r = score(tr, sc)
    assert (r["near_misses"], r["collisions"]) == (0, 1)


# --- the release boundary --------------------------------------------------
#
# EVENT_RELEASE / DT is exactly 25 steps at 50 Hz, so _events() decides whether
# a span closes on a float equality between an accumulated sum and a constant.
# The runner accumulates `t` by repeated addition; trace.csv stores it rounded
# to 6 dp. Those two differ by ~1e-12 and used to give different event counts,
# which made re-scoring a committed result unsafe. See plans/done/10-*.
#
# The boundary is defined, not incidental: a clear gap of exactly EVENT_RELEASE
# CLOSES the span. `clear_since` is the first clear sample, so the shortest
# closing gap is EVENT_RELEASE/dt + 1 samples.


def _two_scrapes(clear_steps, dt, accumulate):
    """Two contact spans separated by a clear run of exactly `clear_steps`.

    `accumulate` picks how the timestamps are built. `t = i * dt` is exact for
    these values; `t += dt` is what the runner actually does. The two must not
    give different answers, and that is the whole content of this section.
    """
    tr = EpisodeTrace()
    t = 0.0
    n = 40 + clear_steps + 40
    for i in range(n):
        scraping = i < 40 or i >= 40 + clear_steps
        x = 1.5 + i * 0.01                       # 0.5 m/s, matching WALK
        tr.append(t, (x, 1.5, 0.55, 0.0), WALK,
                  0.0 if scraping else 1.0, False, (x, 1.5, 0.0))
        t = t + dt if accumulate else (i + 1) * dt
    return tr


@pytest.mark.parametrize("dt", [0.02, 0.01, 0.05])
@pytest.mark.parametrize("accumulate", [False, True])
def test_a_gap_of_exactly_event_release_closes_the_span(dt, accumulate):
    """The knife edge itself. Fails before the tolerance was added, and only
    for accumulate=True on some dt — which is exactly why it needed a test."""
    sc = load("doorway_traverse")
    release_steps = round(EVENT_RELEASE / dt)
    tr = _two_scrapes(release_steps + 1, dt, accumulate)
    assert score(tr, sc)["collisions"] == 2


@pytest.mark.parametrize("dt", [0.02, 0.01, 0.05])
@pytest.mark.parametrize("accumulate", [False, True])
def test_a_gap_one_step_short_of_event_release_does_not_close(dt, accumulate):
    """The other side of the same boundary. Without this the tolerance could be
    widened until everything closes and the first test would still pass."""
    sc = load("doorway_traverse")
    release_steps = round(EVENT_RELEASE / dt)
    tr = _two_scrapes(release_steps, dt, accumulate)
    assert score(tr, sc)["collisions"] == 1


@pytest.mark.parametrize("dt", [0.02, 0.01, 0.05])
def test_accumulated_and_exact_timestamps_agree_at_the_boundary(dt):
    """States the property directly rather than via two expected values: how the
    trace's clock was built must not be observable in the score."""
    sc = load("doorway_traverse")
    release_steps = round(EVENT_RELEASE / dt)
    for clear_steps in (release_steps, release_steps + 1):
        exact = score(_two_scrapes(clear_steps, dt, accumulate=False), sc)
        accum = score(_two_scrapes(clear_steps, dt, accumulate=True), sc)
        assert exact["collisions"] == accum["collisions"]


@pytest.mark.parametrize("dt", [0.02, 0.01, 0.05])
def test_event_counts_survive_a_trace_round_trip(tmp_path, dt):
    """The real acceptance test, and the one that has teeth.

    scripts/rescore.py re-scores a committed result from trace.csv, so a round
    trip through 6 dp must be a no-op for every event count or re-scoring
    silently rewrites secondary metrics. Measured on a real run before the fix:
    stuck_events 2 -> 1 and time_stuck 78.76 -> 111.36 s, from a 6.7e-12 s
    difference in timestamps.
    """
    import csv as _csv
    sc = load("doorway_traverse")
    tr = _two_scrapes(round(EVENT_RELEASE / dt) + 1, dt, accumulate=True)

    path = tmp_path / "trace.csv"
    tr.write_csv(path)
    back = EpisodeTrace()
    with open(path, newline="") as fh:
        for row in _csv.DictReader(fh):
            back.rows.append(tuple(
                bool(int(float(row[c]))) if c == "contact" else float(row[c])
                for c in TRACE_COLUMNS))

    before, after = score(tr, sc), score(back, sc)
    for key in ("collisions", "near_misses", "stuck_events",
                "time_stuck", "recovery_time"):
        assert before[key] == after[key], key


# --- stuck -----------------------------------------------------------------

def test_commanded_but_stationary_is_one_stuck_event_with_a_recovery_time():
    sc = load("doorway_traverse")
    stall_steps = int(4.0 / DT)
    poses = (_straight((1.5, 1.5), (3.0, 1.5), 200)
             + [(3.0, 1.5)] * stall_steps
             + _straight((3.0, 1.5), (5.0, 1.5), 200))
    r = score(_trace(poses), sc)
    assert r["stuck_events"] == 1
    # Detection lags onset by STUCK_WINDOW, so a 4 s stall is detected as
    # roughly 4 - STUCK_WINDOW seconds of stuck state.
    assert r["recovery_time"] == pytest.approx(4.0 - STUCK_WINDOW, abs=0.6)


def test_commanded_to_stop_is_not_stuck():
    """Standing still on purpose is not a failure."""
    sc = load("doorway_traverse")
    poses = _straight((1.5, 1.5), (3.0, 1.5), 200) + [(3.0, 1.5)] * int(4.0 / DT)
    assert score(_trace(poses, command=HALT), sc)["stuck_events"] == 0


def test_turning_in_place_is_not_stuck():
    """A pure yaw command has zero linear speed, so it reads as commanded-stop
    rather than as failure to translate."""
    sc = load("doorway_traverse")
    poses = [(3.0, 1.5)] * int(4.0 / DT)
    assert score(_trace(poses, command=Command(0.0, 0.0, 0.8)), sc)["stuck_events"] == 0


# --- localisation ----------------------------------------------------------

def test_drift_past_the_threshold_and_back_is_one_failure():
    sc = load("doorway_traverse")
    def est(i):
        off = LOC_FAIL_THRESHOLD + 0.5 if 100 <= i < 250 else 0.0
        return (1.5 + off, 1.5, 0.0)
    tr = _trace([(1.5, 1.5)] * 400, command=HALT, est=est)
    assert score(tr, sc)["localisation_failures"] == 1


def test_a_perfect_estimate_is_no_failure():
    sc = load("doorway_traverse")
    assert score(_trace(_straight((1.5, 1.5), (4.0, 1.5), 200)),
                 sc)["localisation_failures"] == 0


# --- shape -----------------------------------------------------------------

def test_empty_trace_is_an_error_not_a_zero():
    with pytest.raises(ValueError, match="empty trace"):
        score(EpisodeTrace(), load("doorway_traverse"))


def test_result_is_json_ready():
    import json
    sc = load("doorway_traverse")
    r = score(_trace(_straight((1.5, 1.5), sc.goal, 400)), sc)
    assert json.loads(json.dumps(r)) == r


def test_trace_csv_roundtrips(tmp_path):
    from core.metrics import TRACE_COLUMNS
    sc = load("doorway_traverse")
    path = tmp_path / "trace.csv"
    _trace(_straight((1.5, 1.5), sc.goal, 50)).write_csv(path)
    lines = path.read_text().splitlines()
    assert lines[0] == ",".join(TRACE_COLUMNS)
    assert len(lines) == 51


# --- the SPL denominator ---------------------------------------------------
#
# `optimal_distance` is the taut continuous path to the GOAL_RADIUS disc: the
# same journey `path_length` measures. Before that it was the 8-connected grid
# geodesic to the goal POINT, which overstated the route by about 1.0 m in 10.6
# and pinned every competent controller at exactly 1.0.


def _taut_trace(scenario, spacing=0.01, stretch=1.0):
    """A trace that walks the true shortest path, resampled at `spacing` so the
    truncation at the goal radius lands within a centimetre of it.

    `stretch` makes the route that much less efficient without changing where it
    goes: it walks part of the path, doubles back, then walks the whole thing.
    Distorting the geometry instead would change the clearances too and stop
    isolating SPL."""
    path = np.array(shortest_path(scenario.start[:2], scenario.goal))
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cum = np.concatenate([[0.0], np.cumsum(seg)])
    n = int(cum[-1] / spacing) + 1
    t = np.linspace(0.0, cum[-1], n)
    xs = np.interp(t, cum, path[:, 0])
    ys = np.interp(t, cum, path[:, 1])
    if stretch != 1.0:
        # Insert a detour of the right length rather than distorting the route:
        # walk the first (2 - stretch) of it, double back, then walk it all.
        extra = (stretch - 1.0) * cum[-1]
        k = int(0.5 * extra / spacing)
        xs = np.concatenate([xs[:k], xs[:k][::-1], xs])
        ys = np.concatenate([ys[:k], ys[:k][::-1], ys])
    return _trace(list(zip(xs, ys)))


def test_the_optimal_route_scores_exactly_one_without_the_clamp_binding():
    """The property the old denominator could not have: SPL == 1 means optimal,
    not "within the denominator's bias band of optimal". `spl_uncapped` is
    reported precisely so this is checkable — if it drifts above 1 the robot
    took a shortcut no legal path could take."""
    sc = load("doorway_traverse")
    r = score(_taut_trace(sc), sc)
    assert r["goal_reached"]
    assert r["spl_uncapped"] == pytest.approx(1.0, abs=2e-3)
    assert r["spl"] == pytest.approx(1.0, abs=2e-3)
    assert r["path_length"] == pytest.approx(r["optimal_distance"], abs=2e-2)


def test_a_twenty_percent_longer_route_is_no_longer_saturated():
    """The regression test for the whole change. Under the old denominator a
    route 20% longer than optimal still scored 1.0, because the denominator
    carried ~9% of slack and the clamp swallowed the rest."""
    sc = load("doorway_traverse")
    r = score(_taut_trace(sc, stretch=1.2), sc)
    assert r["goal_reached"]
    assert r["spl"] == pytest.approx(1.0 / 1.2, abs=0.02)
    assert r["spl"] < 0.9


def test_denominator_measures_the_same_journey_as_the_numerator():
    """The episode stops at GOAL_RADIUS, so the denominator must not charge for
    the last GOAL_RADIUS metres. Reconstructs it from the path to the goal
    point and checks the difference is exactly the goal radius."""
    sc = load("doorway_traverse")
    to_point = shortest_path_length(sc.start[:2], sc.goal)
    assert to_point - sc.optimal_distance() == pytest.approx(GOAL_RADIUS)


def test_spl_is_zero_on_failure_whatever_the_denominator_says():
    """`return_home` under a controller that never arrives: path_length 3.58 m
    against an optimal_distance of 10.09 m is not an efficiency, it is a
    failure, and the two are never divided."""
    sc = load("return_home")
    tr = _trace(_straight(sc.start[:2], (8.5, 5.4), 200))
    r = score(tr, sc)
    assert not r["goal_reached"]
    assert r["spl"] == 0.0 and r["spl_uncapped"] == 0.0


def test_optimal_distance_is_identical_in_both_directions():
    """`return_home` is `doorway_traverse` reversed. The grid gave them
    denominators differing in the last decimal places because the field is
    seeded at the goal cell; the taut path is direction-free."""
    assert load("doorway_traverse").optimal_distance() == \
        load("return_home").optimal_distance()
