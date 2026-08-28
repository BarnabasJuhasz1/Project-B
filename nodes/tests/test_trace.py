"""
nodes/tests/test_trace.py

Node tracing: which node ticked, when, and which one drove.

The property that matters most is that **tracing changes nothing**. A
diagnostic that perturbs the thing it observes is worse than no diagnostic,
and a benchmark run must be byte-identical whether or not anyone was watching.
"""

import controllers
from core.metrics import score
from core.scenario import load
from core.tests.kinematic import rollout


def _traced(scenario_id, controller_name):
    controller = controllers.make(controller_name)
    log = controller.enable_trace()
    return rollout(load(scenario_id), controller), log


def test_tracing_does_not_change_the_episode():
    """The whole basis for trusting a traced run. Same scenario, same
    controller, one traced and one not — the traces must be identical row for
    row, not merely similar."""
    sc = load("held_search_past_corner")
    plain = rollout(sc, controllers.make("toy_search_tree"))
    traced, _ = _traced("held_search_past_corner", "toy_search_tree")
    assert plain.rows == traced.rows
    assert score(plain, sc) == score(traced, sc)


def test_tracing_is_off_by_default():
    """A benchmark run must not pay for a diagnostic."""
    tree = controllers.make("toy_search_tree")
    assert tree.tick_log is None
    rollout(load("blind_corner"), tree)
    assert tree.tick_log is None


def test_the_log_records_every_node_every_step_it_ticked():
    trace, log = _traced("held_nav_top_corridor", "toy_tree")
    summary = log.summary()
    assert set(summary) == {"toy_seq", "toy_is_goal_known", "toy_drive_to_goal"}
    # the sequence ticks both children every step on a disclosed-goal scenario
    assert summary["toy_is_goal_known"]["ticks"] == log.steps
    assert summary["toy_drive_to_goal"]["ticks"] == log.steps


def test_exactly_one_node_drives_per_step():
    """The Twist is the frozen interface and only one Action can decide it. If
    two ever recorded `drove` in the same step, the port contract's claim that
    only an Action emits — and that the last write wins — would be wrong."""
    _, log = _traced("held_search_past_corner", "toy_search_tree")
    by_step = {}
    for step, t, depth, node_id, kind, pre, status, drove in log.rows:
        if drove and kind == "action":
            by_step.setdefault(step, []).append(node_id)
    assert by_step, "no action ever drove"
    assert all(len(v) == 1 for v in by_step.values())


def test_the_search_tree_hands_over_when_the_goal_is_disclosed():
    """The trace's reason for existing: a scored result says the tree reached
    the goal, and says nothing about which branch was in charge when."""
    trace, log = _traced("held_search_past_corner", "toy_search_tree")
    summary = log.summary()

    sweep = summary["toy_sweep_for_goal"]
    drive = summary["toy_drive_to_goal"]
    assert sweep["drove"] > 0 and drive["drove"] > 0
    # the sweep runs first and stops; the drive starts later and finishes
    assert sweep["first_t"] < drive["first_t"]
    assert sweep["last_t"] <= drive["last_t"]
    # and they never overlap: exactly one branch owns the robot at a time
    assert sweep["drove"] + drive["drove"] == log.steps


def test_a_gated_node_is_recorded_as_gated_not_missing():
    """`toy_is_goal_known` fails while the goal is undisclosed, so the Sequence
    behind it never ticks. Recording that distinctly is what separates 'the
    precondition never fired' from 'the action was wrong' — two failures a
    scored result cannot tell apart."""
    _, log = _traced("held_search_past_corner", "toy_search_tree")
    cond = log.summary()["toy_is_goal_known"]
    assert cond["status"]["failure"] > 0 and cond["status"]["success"] > 0
    assert cond["ticks"] == log.steps


def test_the_timeline_renders(tmp_path):
    _, log = _traced("held_search_past_corner", "toy_search_tree")
    strip = log.timeline("toy_sweep_for_goal", 40)
    assert len(strip) == 40 and "#" in strip
    path = tmp_path / "nodes.csv"
    log.write_csv(path)
    assert path.read_text().splitlines()[0].startswith("step,t,depth,node_id")
