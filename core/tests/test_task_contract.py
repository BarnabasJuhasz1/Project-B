"""
core/tests/test_task_contract.py

The one-time change to the frozen contract: Observation.goal is Optional, and a
scenario may withhold it until the robot has found the goal itself.

Two properties carry the whole design and both are tested here rather than
assumed. **Disclosure needs line of sight, not just range** — otherwise a wall
is no obstacle to knowing, and a search task becomes a distance-to-goal readout.
And **disclosure latches** — once told, always told, so "has the goal been
found" is monotonic and no node has to carry its own memory of it.

The third property is negative and just as load-bearing: every scenario that
predates this change still behaves exactly as it did. Every dev scenario
discloses from t=0, and a trace recorded before the `goal_known` column existed
still re-scores rather than being orphaned.
"""

import dataclasses

import pytest

from core.metrics import GOAL_RADIUS, TRACE_COLUMNS, EpisodeTrace, score
from core.observation import Command
from core.drift import make_drift
from core.scenario import (DETECT_RADIUS, DriftSpec, all_scenarios, load,
                           validate)
from core.tests.kinematic import rollout


def _search(goal, start=(4.0, 6.0, 0.0), time_limit=20.0):
    """A doorway_traverse variant that withholds its goal. Built with replace()
    rather than authored, so it cannot drift from the real scenario's sensor
    configuration — that is fixed across every condition."""
    return dataclasses.replace(
        load("doorway_traverse"), id="search_probe", family="search_unmapped",
        start=start, goal=goal, goal_known=False, time_limit=time_limit)


class _Walker:
    """Drives +x forever, whatever it sees.

    NOT NAVIGATION and not a controller. It exists to carry the robot past a
    goal so the latch can be observed, and it must never be registered in
    controllers/ or appear in a benchmark condition.
    """

    def reset(self, scenario, rng):
        pass

    def step(self, obs):
        return Command(0.8, 0.0, 0.0)


# --- the disclosure rule ---------------------------------------------------

def test_dev_scenarios_all_disclose_the_goal():
    """The reason every number in results/ is untouched by this change."""
    assert all(s.goal_known for s in all_scenarios())
    assert all(s.disclosed_at(s.start[:2]) for s in all_scenarios())


def test_a_withheld_goal_is_not_disclosed_at_the_start():
    sc = _search((8.0, 6.0))
    assert not sc.disclosed_at(sc.start[:2])


def test_disclosure_needs_line_of_sight_and_not_merely_range():
    """The divider is 1.1 m away and solid. If this passes with the sign on
    segment_clearance flipped, the whole search family is measuring distance."""
    sc = _search((6.6, 3.0))
    assert not sc.disclosed_at((5.5, 3.0))
    # ... and the same range, with nothing in the way, does disclose.
    assert _search((4.0, 3.0)).disclosed_at((5.5, 3.0))


def test_disclosure_needs_range_and_not_merely_line_of_sight():
    assert not _search((2.0, 3.0)).disclosed_at((5.5, 3.0))


def test_the_detect_radius_boundary_is_inclusive():
    """At exactly DETECT_RADIUS the goal IS disclosed. Stated so that a future
    edit to the comparison is a failing test rather than a silent redefinition
    of the search family."""
    sc = _search((6.0 + DETECT_RADIUS, 6.0))
    assert sc.disclosed_at((6.0, 6.0))


# --- the latch -------------------------------------------------------------

def _goal_known_column(trace):
    return [bool(v) for v in trace.column("goal_known")]


def test_disclosure_latches_and_never_reverts():
    """The goal is off to one side, so the walker approaches within
    DETECT_RADIUS, then carries on past and leaves that radius again. The
    contract says it stays disclosed."""
    sc = _search((7.0, 7.0))
    trace = rollout(sc, _Walker())
    flags = _goal_known_column(trace)

    assert flags[0] is False and any(flags), "never disclosed — bad test setup"
    first = flags.index(True)
    assert all(flags[first:]), "goal_known went back to False after disclosure"

    # And prove the latch is doing work rather than the robot simply staying in
    # range: by the end the rule alone would say "not disclosed".
    x, y = trace.column("x")[-1], trace.column("y")[-1]
    assert not sc.disclosed_at((x, y))
    assert flags[-1] is True


def test_the_latch_fires_on_the_first_step_the_rule_allows():
    """The runner and this rollout must disclose on the same step, or a node's
    unit scenario proves nothing about its behaviour in the benchmark."""
    sc = _search((7.0, 7.0))
    trace = rollout(sc, _Walker())
    flags = _goal_known_column(trace)
    first = flags.index(True)
    xs, ys = trace.column("x"), trace.column("y")

    assert sc.disclosed_at((xs[first], ys[first]))
    assert not any(sc.disclosed_at((xs[i], ys[i])) for i in range(first))


def test_a_controller_sees_none_until_the_latch_fires():
    """Reference controllers hold still on an undisclosed goal, deliberately —
    inventing exploration is what the generated conditions are for."""
    sc = _search((8.0, 6.0))
    from controllers.goal_seeker import GoalSeeker
    from controllers.reactive_avoid import ReactiveAvoid
    for controller in (GoalSeeker(), ReactiveAvoid()):
        trace = rollout(sc, controller)
        assert not any(_goal_known_column(trace))
        assert trace.column("vx").max() == pytest.approx(0.0)


# --- validation ------------------------------------------------------------

def test_detect_radius_must_exceed_the_goal_radius():
    sc = dataclasses.replace(_search((8.0, 6.0)), detect_radius=GOAL_RADIUS)
    with pytest.raises(ValueError, match="not greater than GOAL_RADIUS"):
        validate(sc)


def test_detect_radius_must_not_exceed_the_sensor_range():
    """Otherwise the goal arrives at a range the robot could not have resolved
    it at, which is ground truth entering Observation by the back door."""
    sc = dataclasses.replace(_search((8.0, 6.0)), detect_radius=99.0)
    with pytest.raises(ValueError, match="exceeds range_max"):
        validate(sc)


def test_a_drift_scenario_now_loads_and_actually_drifts():
    """The inverse of the gate 11a installed. Until chunk 13 this raised, so a
    `return_degraded` episode could not silently run with drift disabled and
    score `localisation_failures: 0`. The gate did its job; now the
    implementation exists and the assertion flips.

    `core.scenario._drift_ready()` is kept as a feature probe rather than
    deleted: delete `core/drift.py` and the refusal comes back.
    """
    from core.drift import DriftModel
    from core.scenario import _drift_ready
    assert _drift_ready()

    sc = validate(dataclasses.replace(
        load("doorway_traverse"),
        drift=DriftSpec(sigma_xy=0.12, sigma_yaw=0.06)))
    model = make_drift(sc)
    assert isinstance(model, DriftModel)
    for _ in range(500):
        model.advance(0.02)
    assert model.true_error() > 0.0
    assert model.error_estimate() > 0.0


def test_the_dev_scenarios_still_validate():
    for s in all_scenarios():
        validate(s)


# --- the trace format ------------------------------------------------------

def _write_legacy_csv(path, trace):
    """A trace as it was written before `goal_known` existed: same rows, one
    fewer column."""
    legacy = [c for c in TRACE_COLUMNS if c != "goal_known"]
    keep = [TRACE_COLUMNS.index(c) for c in legacy]
    with open(path, "w", newline="") as fh:
        fh.write(",".join(legacy) + "\n")
        for row in trace.rows:
            fh.write(",".join(
                f"{row[i]:.6f}" if isinstance(row[i], float) else str(int(row[i]))
                for i in keep) + "\n")


def test_a_trace_written_before_goal_known_still_scores_identically(tmp_path):
    """The acceptance test for adding a column at all. Without the backfill,
    every committed result becomes unre-scorable the moment TRACE_COLUMNS
    changes, and re-scoring is the property core/metrics.py opens by claiming."""
    import controllers
    sc = load("doorway_traverse")
    trace = rollout(sc, controllers.make("reactive_avoid"))

    legacy_path = tmp_path / "legacy.csv"
    _write_legacy_csv(legacy_path, trace)
    legacy = EpisodeTrace.read_csv(legacy_path)
    assert all(_goal_known_column(legacy)), "backfill must assume goal was known"

    current_path = tmp_path / "current.csv"
    trace.write_csv(current_path)
    current = EpisodeTrace.read_csv(current_path)

    assert score(legacy, sc) == score(current, sc)


def test_an_unknown_trace_column_is_an_error(tmp_path):
    path = tmp_path / "future.csv"
    path.write_text(",".join(TRACE_COLUMNS + ["invented_later"]) + "\n")
    with pytest.raises(ValueError, match="cannot be reconciled"):
        EpisodeTrace.read_csv(path)


def test_a_missing_column_with_no_backfill_is_an_error(tmp_path):
    path = tmp_path / "truncated.csv"
    path.write_text(",".join(c for c in TRACE_COLUMNS if c != "clearance") + "\n")
    with pytest.raises(ValueError, match="Missing with no backfill"):
        EpisodeTrace.read_csv(path)
