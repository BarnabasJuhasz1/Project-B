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

from generate.generator import PIPELINE_VERSION
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
    # Which pipeline staged it. Without this the fixture records default to
    # "v1-single-call" and `pass_rate` — which refuses to pool pipelines since
    # chunk 42 — skips every one of them, so the tests below would assert over
    # an empty bucket and pass for the wrong reason.
    "pipeline_version": PIPELINE_VERSION,
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


def test_a_node_whose_mechanism_does_nothing_is_ADMITTED_and_recorded_as_zero():
    """**The chunk-37 reversal, and the most important test in this file.**

    Delete the veer and the node IS `goal_seeker`: the mechanism contributes
    nothing. Under the old rule that was a rejection. It is now an ADMISSION
    with a recorded contribution of zero, because "this node does not help in
    the world its author designed" is a measurement, and a weak one — the same
    instrument admitted `ExtendedSurfaceDetour`, whose paper's mechanism can be
    ablated entirely without stopping it solving either dev scenario.
    """
    disabled = _swap(GOOD, "if near < 1.2:", "if False:")
    result = admit(META, disabled)
    assert result.admitted, f"{result.reason}: {result.detail}"
    contribution = result.metadata["contribution"]
    assert contribution["measured"] is True
    assert contribution["cleared_floor"] is False
    assert contribution["effect"] == 0.0


def test_a_scenario_whose_baseline_already_succeeds_is_recorded_not_rejected():
    """chunk 13's most important check, demoted to a measurement. An empty room
    is one `goal_seeker` solves, so the scenario tests nothing — which is a fact
    about the world the generator wrote, not about the node."""
    trivial = _swap(GOOD, 'boxes=[(2.0, -0.1, 2.2, 2.5)]', 'boxes=[]')
    result = admit(META, trivial)
    assert result.admitted
    contribution = result.metadata["contribution"]
    assert contribution["cleared_floor"] is False
    assert "MALFORMED" in contribution["reason"]


def test_a_malformed_micro_world_leaves_the_contribution_UNMEASURED():
    """A goal inside a box. The node is fine; its author's world is not.

    `measured: False` rather than an effect of zero, and the distinction is the
    point: two of condition D's four rejections were exactly this, filed under
    the same reason code as a node that ran and did not work. D's reported
    0.00 was half made of world-construction errors.
    """
    bad = _swap(GOOD, "goal=(5.0, 3.0)", "goal=(2.1, 1.0)")
    result = admit(META, bad)
    assert result.admitted
    assert result.metadata["contribution"]["measured"] is False
    assert "malformed" in result.metadata["contribution"]["reason"]


# --- the fault nobody anticipated is still counted --------------------------
#
# These pin `admit()`'s documented contract: it "never raises for a candidate's
# own faults". It did, once, and the cost was not a crash — it was a candidate
# that left G1's denominator without anyone noticing. `batch.run()` counts an
# escaping exception as `failed` and writes no `.result.json`, so the row was
# generated, never judged, and in no pass rate.


def test_a_micro_world_that_only_breaks_at_rollout_time_leaves_it_unmeasured():
    """The real fault, `b/navigate_to_goal/path_integration_ants`: `drift=0.6`,
    a bare float where a `DriftSpec` belongs.

    The node itself is sound — it passes validity, because the validity world is
    the HARNESS's and carries no drift. Only the candidate's own world is
    broken, so the contribution is unmeasured and the node is admitted.
    """
    bad = _swap(GOOD, "time_limit=40.0, seed=seed)",
                "time_limit=40.0, seed=seed, drift=0.6)")
    result = admit(META, bad)                    # must not raise
    assert result.admitted
    contribution = result.metadata["contribution"]
    assert contribution["measured"] is False
    assert "sigma_xy" in contribution["reason"]


def test_an_unmeasurable_contribution_keeps_its_traceback():
    """This clause cannot tell a candidate's fault from a harness bug hit while
    measuring one, so the record has to carry enough to let a reader decide. A
    bare message would not: `AttributeError: 'float' object has no attribute
    'sigma_xy'` names neither the candidate nor the frame that raised."""
    bad = _swap(GOOD, "time_limit=40.0, seed=seed)",
                "time_limit=40.0, seed=seed, drift=0.6)")
    reason = admit(META, bad).metadata["contribution"]["reason"]
    assert "Traceback (most recent call last)" in reason
    assert "core/drift.py" in reason


def test_a_node_that_raises_mid_rollout_fails_VALIDITY():
    """The gate that remains. A node that cannot be ticked is not a node, and
    this is the one thing chunk 37 still rejects on."""
    boom = _swap(GOOD, "        x, y, yaw = obs.pose_estimate",
                 "        raise ZeroDivisionError('mid-rollout')\n"
                 "        x, y, yaw = obs.pose_estimate")
    result = admit(META, boom)
    assert not result.admitted
    assert result.reason == "INVALID_NODE"
    assert "ZeroDivisionError" in result.detail
    assert result.metadata["validity"]["valid"] is False


def test_an_invalid_node_lands_in_g1s_denominator(tmp_path):
    """A rejection is still a rejection and still counts against the condition's
    rate — what changed is which faults are rejections."""
    import json
    boom = _swap(GOOD, "        x, y, yaw = obs.pose_estimate",
                 "        raise ZeroDivisionError('mid-rollout')\n"
                 "        x, y, yaw = obs.pose_estimate")
    result = admit(META, boom)
    (tmp_path / "raised.result.json").write_text(json.dumps(result.as_dict()))
    (tmp_path / "fine.result.json").write_text(json.dumps({
        "node_id": "n", "admitted": True, "reason": "admitted",
        "detail": "", "metadata": {"condition": "a",
                                   "pipeline_version": PIPELINE_VERSION}}))

    rates = pass_rate(tmp_path)
    assert rates["a"]["rejected"] == 1
    assert rates["a"]["reasons"] == {"INVALID_NODE": 1}
    assert rates["a"]["pass_rate"] == 0.5


def test_a_prediction_naming_an_unscoreable_metric_is_unmeasurable_not_rejected():
    bad = _swap(GOOD, '"metric": "goal_reached"', '"metric": "elegance"')
    result = admit(META, bad)
    assert result.admitted
    assert result.metadata["contribution"]["measured"] is False
    assert "UNSCOREABLE_PREDICTION" in result.metadata["contribution"]["reason"]


def test_a_prediction_naming_a_controller_independent_metric_is_unmeasurable():
    """chunk 14's finding, still recorded: no controller can move
    `localisation_failures`, so a prediction about it is not falsifiable about
    the node — which now makes the contribution unmeasurable rather than the
    node inadmissible."""
    bad = _swap(GOOD, '"metric": "goal_reached"',
                '"metric": "localisation_failures"')
    result = admit(META, bad)
    assert result.admitted
    assert result.metadata["contribution"]["measured"] is False


def test_a_condition_leaf_is_now_ADMITTED_on_validity():
    """`KIND_NOT_TESTABLE_ALONE` is gone.

    It was right about efficacy and wrong as a gate: only an Action moves a
    navigation metric alone, so gating on a moved metric made two of the
    description's three leaf kinds unreachable. The visible cost was that
    NOTHING IN THE LIBRARY READS A PORT — every producer had to be an Action
    that also drove, so every published output was orphaned.
    """
    cond = GOOD.replace("class VeerFromObstacles(Action):",
                        "class VeerFromObstacles(Condition):")
    cond = _swap(cond, "from nodes.contract import Action,",
                 "from nodes.contract import Condition,")
    cond = _swap(cond, "        ports.drive(Command(speed, 0.0, 2.0 * bearing))\n", "")
    result = admit(META, cond)
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert result.metadata["validity"]["valid"] is True


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
            "detail": "", "metadata": {"condition": condition,
                                       "pipeline_version": PIPELINE_VERSION}}))

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
            "detail": "", "metadata": {"condition": "b", "card_id": card_id,
                                   "pipeline_version": PIPELINE_VERSION}}))

    rates = pass_rate(tmp_path, cards=cards)
    assert rates["_excluded_fixtures"] == ["vague_adaptive_motion"]
    assert rates["b"]["admitted"] == 1 and rates["b"]["rejected"] == 1
    assert rates["b"]["pass_rate"] == 0.5, (
        "the fixture card is being counted in G1's pass rate")


# --- condition A: pointed at a source, never given its content --------------

def _pilot_card():
    from cards.store import CardStore
    return CardStore.load().get("limit_speed_dwa")


def test_a_citation_node_is_admitted_with_the_citation_origin():
    """Condition A names the reference it was pointed at, and nothing more.

    `card_id` carries WHICH paper — that is A's whole treatment and a node that
    cannot name its pointer is an INVENTED node with a misleading label.
    """
    from nodes.contract import Origin
    result = admit(dict(META, citation_id="limit_speed_dwa"), GOOD,
                   citation_of=_pilot_card())
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert result.metadata["origin"] == Origin.CITATION.value
    # The pointer lives in `citation_id`; `card_id` stays None because the card
    # itself never reached the model. Those two keys are the whole distinction.
    assert result.metadata["citation_id"] == "limit_speed_dwa"
    assert result.metadata["card_id"] is None


def test_a_citation_node_inherits_no_span_no_confidence_and_no_family():
    """**The leak this whole restructure has to prevent.**

    Span, extraction confidence and mechanism family are all products of a human
    reading the paper — condition B's treatment by definition. If any of them
    attached to a citation node, A would look traceable, G1's provenance
    numerator would count a paper nobody opened for it, and B-A would measure
    transcription instead of extraction. It would look identical in the numbers.
    """
    card = _pilot_card()
    result = admit(dict(META, citation_id=card.id), GOOD, citation_of=card)
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert not result.metadata.get("source_span")
    assert not result.metadata.get("confidence")
    assert not result.metadata.get("mechanism_family")
    assert result.metadata.get("is_traceable") in (False, None)


def test_a_citation_node_is_scored_against_its_own_prediction_not_the_cards():
    """A builds its own micro-world and states its own claim, exactly as an
    invented node does. Scoring it against the CARD's metric, baseline and
    minimum effect would hand condition A the extracted content through the back
    door — the one thing it is defined by not having."""
    card = _pilot_card()
    pointed = admit(dict(META, citation_id=card.id), GOOD, citation_of=card)
    alone = admit(META, GOOD)
    assert pointed.unit.metric == alone.unit.metric
    assert pointed.unit.baseline == alone.unit.baseline
    assert pointed.unit.required == alone.unit.required
    # And the card's own prediction is NOT what was applied.
    assert "prediction_restated_faithfully" not in pointed.metadata


def test_a_node_cannot_be_both_card_derived_and_citation_only():
    """Those are two conditions, and one node cannot be in both arms."""
    card = _pilot_card()
    result = admit(dict(META, card_id=card.id, citation_id=card.id), GOOD,
                   card=card, citation_of=card)
    assert not result.admitted
    assert result.reason == "PROVENANCE_INCOMPLETE"


def test_a_citation_candidate_from_a_fixture_card_is_excluded_too(tmp_path):
    """The fixture rule must cover BOTH columns.

    A's pointer lives in `citation_id`, so a rule that only inspected `card_id`
    would let a candidate generated from `vague_adaptive_motion` into G1's pass
    rate. Invisibly — fixture ids look exactly like real ones.
    """
    import json
    from generate.admit import pass_rate
    (tmp_path / "a__fixture__aaa.result.json").write_text(json.dumps({
        "admitted": True, "reason": "admitted",
        "metadata": {"condition": "a", "card_id": None,
                     "pipeline_version": PIPELINE_VERSION,
                     "citation_id": "vague_adaptive_motion"}}))
    out = pass_rate(staging=tmp_path)
    assert "a" not in out, out
    assert out["_excluded_fixtures"] == ["vague_adaptive_motion"]


def test_validity_uses_the_HARNESS_world_not_the_candidates():
    """The single most important detail of chunk 37.

    This candidate's `micro()` raises outright. Under the old rule that was a
    rejection, and it is how two of condition D's four candidates died — the
    harness refused the generator's world and recorded it as a node failure. The
    node is validated in a world `harness.micro` owns, so a broken world can no
    longer fail a node that runs perfectly well.
    """
    broken_world = _swap(GOOD, "def micro(seed=0):",
                         "def micro(seed=0):\n"
                         "    raise RuntimeError('this world is broken')")
    result = admit(META, broken_world)
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert result.metadata["validity"]["valid"] is True
    assert result.metadata["contribution"]["measured"] is False
    assert result.metadata["precondition_fired"]["ticks"] > 0, (
        "the node was never ticked, so 'valid' means nothing")


def test_a_decorator_is_admitted_and_is_given_a_child_to_govern():
    """The other half of what `KIND_NOT_TESTABLE_ALONE` was blocking. A
    Decorator cannot be constructed without a child, so the harness supplies a
    fixture one — NOT NAVIGATION, and it can never reach a condition (#3)."""
    dec = GOOD.replace("class VeerFromObstacles(Action):",
                       "class VeerFromObstacles(Decorator):")
    dec = _swap(dec, "from nodes.contract import Action,",
                "from nodes.contract import Decorator,")
    dec = _swap(dec, "        ports.drive(Command(speed, 0.0, 2.0 * bearing))\n", "")
    result = admit(META, dec)
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert result.metadata["validity"]["valid"] is True


def test_the_firing_rate_separates_never_fired_from_did_not_help():
    """Two findings the old single number could not tell apart. Measured on the
    real library: `LimitSpeedToOcclusionBoundary` passed its precondition 0
    times in 6000 ticks; `ReorientToGeometry` 6000 out of 6000."""
    never = _swap(GOOD, 'return ports["goal_known"]', "return False")
    result = admit(META, never)
    assert result.admitted
    fired = result.metadata["precondition_fired"]
    assert fired["ticks"] > 0
    assert fired["passed"] == 0
    assert fired["fraction"] == 0.0
    assert fired["drove"] == 0
