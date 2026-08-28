"""
generate/tests/test_admit.py

The admission decision, exercised on handwritten candidates.

**Handwritten on purpose.** These are the tests of the gate, not of the model:
each one is a candidate constructed to fail in exactly one way, so that a
rejection reason means what it says. A model-written candidate fails in whatever
way it happens to fail in that call, which is fine for a demonstration and
useless for a test.

Every candidate below is a FIXTURE and would never be a benchmark condition —
the same rule `nodes/toys.py` and `harness/tests/` already state about
themselves. Several of them do not navigate at all; that is the point.
"""

import textwrap

import pytest

from generate.admit import (ALLOWED_IMPORTS, Rejected, admit, check_source,
                            load_candidate, pass_rate)

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# The metadata a real generation call attaches. Complete, so that a test of some
# OTHER failure is not accidentally testing provenance.
META = {
    "node_id": "fixture_node",
    "condition": "a",
    "task": "navigate_to_goal",
    "card_id": None,
    "model_id": "claude-opus-5",
    "backend": "replay",
    "prompt_hash": "0" * 16,
    "temperature": None,
    "sampling_note": "the model does not accept a temperature parameter",
    "emitted_code": True,
}

# A candidate that works: it delegates to reactive_avoid... which it cannot
# import. So it hand-rolls the smallest thing that clears a box: turn away from
# the nearest return, otherwise drive at the goal.
GOOD = textwrap.dedent('''
    """A fixture node. NOT NAVIGATION and never a benchmark condition."""

    import math

    import numpy as np

    from core.observation import Command
    from nodes.contract import Action, Direction, Port, PortType, Status
    from harness.micro import MicroScenario, room

    PREDICTION = {
        "metric": "goal_reached",
        "baseline": "goal_seeker",
        "min_effect": 0.5,
        "cost_metric": "path_length",
    }


    class VeerFromObstacles(Action):
        """Drive at the goal, veering away from whatever is closest."""

        PORTS = (Port("goal_known", PortType.FLAG, Direction.IN),)
        PRECONDITION = "the goal has been disclosed"
        EFFECT = "steers away from the nearest return while approaching the goal"
        TAGS = ("avoid",)

        def precondition(self, obs, ports):
            return ports["goal_known"]

        def tick(self, obs, ports):
            x, y, yaw = obs.pose_estimate
            bearing = math.atan2(obs.goal[1] - y, obs.goal[0] - x) - yaw
            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi

            finite = np.isfinite(obs.ranges)
            if finite.any():
                i = int(np.argmin(np.where(finite, obs.ranges, np.inf)))
                near, at = float(obs.ranges[i]), float(obs.range_angles[i])
                if near < 1.2:
                    # Push the heading away from the obstruction, harder the
                    # closer it is. Sign of `at` says which way to swing.
                    bearing -= math.copysign(1.6 * (1.2 - near), at)

            bearing = (bearing + math.pi) % (2 * math.pi) - math.pi
            speed = 0.8 * max(0.0, math.cos(bearing))
            ports.drive(Command(speed, 0.0, 2.0 * bearing))
            return Status.RUNNING


    def micro(seed=0):
        return MicroScenario(
            id="fixture_corridor",
            world=room(6.0, 4.0, boxes=[(2.0, -0.1, 2.2, 2.5)]),
            start=(1.0, 1.0, 0.0), goal=(5.0, 3.0),
            time_limit=40.0, seed=seed)
''').lstrip()


def _swap(source, old, new):
    assert old in source, old
    return source.replace(old, new, 1)


# --- the static checks -----------------------------------------------------

def test_a_candidate_that_parses_and_declares_one_node_passes_the_static_check():
    assert check_source(GOOD) == "VeerFromObstacles"


def test_a_syntax_error_is_a_rejection_not_a_crash():
    with pytest.raises(Rejected) as exc:
        check_source("class Broken(Action)\n    pass\n")
    assert exc.value.reason == "SYNTAX_ERROR"


@pytest.mark.parametrize("line", [
    "import os",
    "import subprocess",
    "from core.scenario import load",
    "from core.geometry import occupancy_grid",
    "import controllers",
    "from pathlib import Path",
])
def test_an_import_outside_the_allowlist_is_refused(line):
    """The check the other guarantees hang off. `core.scenario` and
    `core.geometry` are the two that matter for the experiment: one is the
    scenario coupling §4 forbids, the other is ground truth."""
    with pytest.raises(Rejected) as exc:
        check_source(_swap(GOOD, "import math", line))
    assert exc.value.reason == "IMPORT_DENIED"


def test_the_sealed_split_is_refused_like_any_other_denied_import():
    """G3's seal, applied to code that does not exist when the AST guard in
    core/tests/test_held_out.py runs."""
    with pytest.raises(Rejected) as exc:
        check_source(_swap(GOOD, "import math",
                           "from core.scenarios_held_out import HELD_OUT_SCENARIOS"))
    assert exc.value.reason == "IMPORT_DENIED"


def test_an_import_hidden_inside_a_function_is_still_found():
    """Where a candidate would put one if it were trying not to be noticed."""
    hidden = _swap(GOOD, "    def tick(self, obs, ports):",
                   "    def tick(self, obs, ports):\n        import os")
    with pytest.raises(Rejected) as exc:
        check_source(hidden)
    assert exc.value.reason == "IMPORT_DENIED"


def test_the_allowlist_matches_dotted_components_not_characters():
    """`numpy.linalg` is allowed by `numpy`; `mathematics` is not by `math`."""
    from generate.admit import _allowed
    assert _allowed("numpy.linalg")
    assert _allowed("core.observation")
    assert not _allowed("mathematics")
    assert not _allowed("core.observation_hack")
    assert not _allowed("core")


def test_the_allowlist_is_the_documented_one():
    """Pinned, because widening it silently is how ground truth gets in."""
    assert ALLOWED_IMPORTS == frozenset({
        "math", "numpy", "core.observation", "nodes.contract", "harness.micro"})


def test_a_candidate_with_no_node_class_is_refused():
    with pytest.raises(Rejected) as exc:
        check_source("PREDICTION = {}\ndef micro(seed=0):\n    return None\n")
    assert exc.value.reason == "NO_NODE"


def test_two_node_classes_are_refused():
    """One card, one call, one node — otherwise a source span is attributable
    to neither and G1's fraction is a fraction of nothing."""
    two = GOOD + textwrap.dedent('''

        class AlsoANode(Action):
            PRECONDITION = "always"
            EFFECT = "a second node in one candidate"

            def tick(self, obs, ports):
                return Status.SUCCESS
        ''')
    with pytest.raises(Rejected) as exc:
        check_source(two)
    assert exc.value.reason == "MULTIPLE_NODES"


def test_a_candidate_that_raises_at_import_time_is_a_rejection():
    with pytest.raises(Rejected) as exc:
        load_candidate(_swap(GOOD, "PREDICTION = {", "raise RuntimeError('boom')\nPREDICTION = {"))
    assert exc.value.reason == "IMPORT_TIME_ERROR"


def test_a_missing_prediction_is_a_rejection():
    with pytest.raises(Rejected) as exc:
        load_candidate(_swap(GOOD, "PREDICTION = {", "NOT_A_PREDICTION = {"))
    assert exc.value.reason == "NO_PREDICTION"


def test_a_missing_micro_is_a_rejection():
    with pytest.raises(Rejected) as exc:
        load_candidate(_swap(GOOD, "def micro(seed=0):", "def not_micro(seed=0):"))
    assert exc.value.reason == "NO_MICRO"


# --- the harness half ------------------------------------------------------

def test_the_whole_path_admits_a_node_with_a_real_effect():
    """Card-free (condition A shaped): static checks, execution, tree wiring,
    micro-world validation and the unit scenario, with no human in the loop."""
    result = admit(META, GOOD)
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert result.unit.metric == "goal_reached"
    assert result.unit.baseline == 0.0, "the fixed baseline must actually fail"
    assert result.unit.effect >= result.unit.required


def test_the_same_node_with_its_mechanism_removed_is_not_admitted():
    """The control. Delete the veer and the node IS goal_seeker, so the effect
    vanishes and admission must vanish with it."""
    disabled = _swap(GOOD, "if near < 1.2:", "if False:")
    result = admit(META, disabled)
    assert not result.admitted
    assert result.reason == "UNIT_SCENARIO_FAILED"


def test_a_scenario_whose_baseline_already_succeeds_is_malformed():
    """chunk 13's most important check, reached through this path: an empty
    room is one goal_seeker solves, so the scenario tests nothing."""
    trivial = _swap(GOOD, 'boxes=[(2.0, -0.1, 2.2, 2.5)]', 'boxes=[]')
    result = admit(META, trivial)
    assert not result.admitted
    assert "MALFORMED" in result.detail


def test_a_malformed_micro_world_is_rejected_rather_than_scored():
    """A goal inside a box would otherwise produce a confident number about
    nothing."""
    bad = _swap(GOOD, "goal=(5.0, 3.0)", "goal=(2.1, 1.0)")
    result = admit(META, bad)
    assert not result.admitted
    assert result.reason in ("MALFORMED_MICRO", "UNIT_SCENARIO_FAILED")


def test_a_prediction_naming_an_unscoreable_metric_is_rejected():
    bad = _swap(GOOD, '"metric": "goal_reached"', '"metric": "elegance"')
    result = admit(META, bad)
    assert not result.admitted
    assert result.reason == "UNSCOREABLE_PREDICTION"


def test_a_prediction_naming_a_controller_independent_metric_is_rejected():
    """chunk 14's finding, enforced on the generation path: no controller can
    move localisation_failures, so a prediction about it is not falsifiable
    about the node."""
    bad = _swap(GOOD, '"metric": "goal_reached"',
                '"metric": "localisation_failures"')
    result = admit(META, bad)
    assert not result.admitted
    assert result.reason == "UNSCOREABLE_PREDICTION"


def test_a_condition_leaf_is_rejected_with_the_reason_that_says_why():
    """Not a silent pass and not a crash. A Condition emits no Twist, so it
    cannot move a navigation metric alone; the reason hands chunk 16 the
    problem stated precisely."""
    cond = GOOD.replace("class VeerFromObstacles(Action):",
                        "class VeerFromObstacles(Condition):")
    cond = _swap(cond, "from nodes.contract import Action,",
                 "from nodes.contract import Condition,")
    cond = _swap(cond, "        ports.drive(Command(speed, 0.0, 2.0 * bearing))\n", "")
    result = admit(META, cond)
    assert not result.admitted
    assert result.reason == "KIND_NOT_TESTABLE_ALONE"


def test_an_input_port_the_harness_cannot_supply_is_rejected():
    """Caught by Tree's construction-time wiring check, reported as a rejection
    rather than as a crash mid-episode."""
    bad = _swap(GOOD, 'Port("goal_known", PortType.FLAG, Direction.IN)',
                'Port("cue_gradient", PortType.CONFIDENCE, Direction.IN)')
    bad = _swap(bad, 'return ports["goal_known"]', 'return ports["cue_gradient"] > 0')
    result = admit(META, bad)
    assert not result.admitted
    assert result.reason == "UNSATISFIED_PORT"


def test_a_node_emitted_as_no_code_at_all_is_a_recorded_rejection():
    """The third outcome, found on the first run against the bad card: the model
    declined to emit anything. A finding about the card, and it belongs in G1's
    denominator — not an exception that throws it away."""
    result = admit({**META, "emitted_code": False}, None)
    assert not result.admitted
    assert result.reason == "NO_CANDIDATE_EMITTED"


# --- provenance ------------------------------------------------------------

@pytest.mark.parametrize("field", ["model_id", "prompt_hash"])
def test_a_candidate_missing_provenance_is_refused(field):
    """"A library that cannot say which model produced it cannot be
    regenerated." Chunk 12 left these optional and named this chunk as the one
    that would enforce them."""
    result = admit({**META, field: None}, GOOD)
    assert not result.admitted
    assert result.reason == "PROVENANCE_INCOMPLETE"


def test_temperature_must_be_recorded_even_when_it_is_none():
    result = admit({k: v for k, v in META.items() if k != "temperature"}, GOOD)
    assert not result.admitted
    assert result.reason == "PROVENANCE_INCOMPLETE"


def test_a_null_temperature_needs_a_reason():
    """None is ambiguous on its own: "the model does not accept the parameter"
    and "nobody recorded it" are different facts and only one is honest."""
    result = admit({**META, "sampling_note": ""}, GOOD)
    assert not result.admitted
    assert result.reason == "PROVENANCE_INCOMPLETE"


def test_pass_rate_counts_admissions_and_rejection_reasons(tmp_path):
    """G1's unit pass rate, computed from what is on disk rather than asserted."""
    import json
    for i, (admitted, reason, condition) in enumerate([
            (True, "admitted", "b"),
            (False, "IMPORT_DENIED", "b"),
            (False, "IMPORT_DENIED", "a"),
            (True, "admitted", "a")]):
        (tmp_path / f"c{i}.result.json").write_text(json.dumps({
            "node_id": f"n{i}", "admitted": admitted, "reason": reason,
            "detail": "", "metadata": {"condition": condition}}))

    rates = pass_rate(tmp_path)
    assert rates["a"]["pass_rate"] == 0.5
    assert rates["b"]["pass_rate"] == 0.5
    assert rates["b"]["reasons"] == {"IMPORT_DENIED": 1}


def test_a_fixture_card_is_excluded_from_the_reported_pass_rate(tmp_path,
                                                                cards):
    """G1's number must not contain the deliberately-bad card.

    The rule is mechanical: a `card_id` that does not resolve in the store was
    never part of the corpus, so it is a fixture. Excluded ids are RETURNED,
    not silently dropped.
    """
    import json
    rows = [("real", "limit_speed_dwa", True),
            ("fixture", "vague_adaptive_motion", True),
            ("real2", "reorient_geometry", False)]
    for i, (name, card_id, admitted) in enumerate(rows):
        (tmp_path / f"{name}.result.json").write_text(json.dumps({
            "node_id": name, "admitted": admitted,
            "reason": "admitted" if admitted else "UNIT_SCENARIO_FAILED",
            "detail": "", "metadata": {"condition": "b", "card_id": card_id}}))

    rates = pass_rate(tmp_path, cards=cards)
    assert rates["_excluded_fixtures"] == ["vague_adaptive_motion"]
    assert rates["b"]["admitted"] == 1 and rates["b"]["rejected"] == 1
    assert rates["b"]["pass_rate"] == 0.5, (
        "the fixture card is being counted in G1's pass rate")
