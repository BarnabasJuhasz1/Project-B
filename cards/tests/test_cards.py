"""
cards/tests/test_cards.py

The card store, and the two properties that make cards worth having.

**A Prediction must be falsifiable.** Not "well written" — decidable by
`harness.unit.run_unit` with no human in the loop. That means naming a metric
`core.metrics.score()` computes AND one a controller can actually move; both are
checked at load, so a card that cannot become a unit test never enters the store.

**Provenance must be earned.** G1 reports the fraction carrying a source span
with extraction confidence, and the description's framing is that a model states
a threshold "with equal confidence whether or not it is confabulating". An
unverified span is that confabulation, so it counts for nothing until a human has
opened the paper.
"""

import dataclasses

import pytest

import yaml

from cards.schema import Card, Confidence, Prediction, Source, Verification
from cards.store import CONTROLLER_INDEPENDENT, CardError, CardStore, metric_keys

STORE = CardStore.load()


def _yaml(card_id):
    from cards.store import CARDS_DIR
    return yaml.safe_load((CARDS_DIR / f"{card_id}.yaml").read_text())


# --- the store loads -------------------------------------------------------

def test_the_pilot_set_is_five_to_ten_cards():
    assert 5 <= len(STORE) <= 10


def test_every_card_names_a_metric_that_exists():
    for card in STORE:
        assert card.prediction.metric in metric_keys(), card.id
        if card.prediction.cost_metric:
            assert card.prediction.cost_metric in metric_keys(), card.id


def test_every_card_names_a_real_baseline():
    import controllers
    known = set(controllers.available()) | set(controllers.scaffolding())
    for card in STORE:
        assert card.prediction.baseline in known, card.id


def test_ids_match_filenames_and_are_unique():
    ids = [c.id for c in STORE]
    assert len(set(ids)) == len(ids)
    for card in STORE:
        _yaml(card.id)          # raises if the file is not named after the id


def test_at_least_three_are_testable_today():
    """The Done-when: three cards the harness can score now, without waiting on
    anything. All three tier-1 cards target a MEASURED failure of a committed
    baseline rather than whatever was most interesting to write."""
    today = [c for c in STORE if "tier1" in c.tags]
    assert len(today) >= 3


def test_the_pilot_covers_more_than_one_mechanism():
    """G1's coverage measure is a count of distinct admissible primitives, so a
    pilot set that is five cards on one mechanism would report a coverage of
    one however many cards it contained."""
    assert len({c.node_name for c in STORE}) >= 3


def test_one_mechanism_is_deliberately_carried_by_two_derivations():
    """limit_speed_dwa / limit_speed_ics, and path_integration_ants / _mammals.
    Same node_name, different Derivation. If the two produce nodes that score
    differently, the Derivation field carries information rather than just
    decorating the citation — which is a claim about the schema, testable inside
    the pilot."""
    from collections import Counter
    counts = Counter(c.node_name for c in STORE)
    paired = [n for n, k in counts.items() if k >= 2]
    assert len(paired) >= 2, counts


# --- falsifiability --------------------------------------------------------

def _card_dict(**overrides):
    data = _yaml("limit_speed_dwa")
    for key, value in overrides.items():
        if "." in key:
            outer, inner = key.split(".")
            data[outer] = dict(data[outer], **{inner: value})
        else:
            data[key] = value
    return data


def _load(data, where="probe.yaml"):
    from cards.store import _card_from_dict
    return _card_from_dict(data, where)


def test_a_prediction_naming_no_real_metric_is_rejected_at_load():
    with pytest.raises(CardError, match="not falsifiable"):
        _load(_card_dict(**{"prediction.metric": "elegance"}))


def test_a_prediction_on_a_metric_no_controller_can_move_is_rejected():
    """The finding that produced this rule: drift is injected open-loop and the
    runner records drift.estimate() straight into the trace, so
    localisation_failures is identical across controllers that behave nothing
    alike. A card predicting it would look rigorous and test nothing."""
    assert "localisation_failures" in metric_keys()
    with pytest.raises(CardError, match="cannot be moved by a controller"):
        _load(_card_dict(**{"prediction.metric": "localisation_failures"}))


def test_localisation_failures_really_is_controller_independent():
    """Asserted rather than believed, because the rule above rests on it and a
    change to the drift model could quietly invalidate it."""
    import math
    import controllers
    from core.metrics import score
    from core.scenario import DriftSpec, load
    from core.tests.kinematic import rollout

    sc = dataclasses.replace(load("doorway_traverse"),
                             drift=DriftSpec(sigma_xy=0.25, sigma_yaw=0.12))
    errors, failures = [], {}
    for name in ("goal_seeker", "scaffold_sweep"):
        trace = rollout(sc, controllers.make(name))
        result = score(trace, sc)
        failures[name] = result["localisation_failures"]
        errors.append(round(math.hypot(
            trace.column("est_x")[500] - trace.column("x")[500],
            trace.column("est_y")[500] - trace.column("y")[500]), 9))

    assert errors[0] == errors[1], "drift must not depend on the controller"
    assert failures["goal_seeker"] == failures["scaffold_sweep"]


def test_a_card_with_no_statement_is_rejected():
    with pytest.raises(CardError):
        _load(_card_dict(**{"prediction.statement": "  "}))


# --- provenance ------------------------------------------------------------

# The card that is still UNVERIFIED, and why. Kept as a named constant so the
# test below says WHICH card is outstanding rather than only how many.
UNVERIFIED_PILOT_CARDS = {"limit_speed_ics"}


def test_pilot_provenance_is_exactly_what_has_been_checked():
    """Pins the honest state, which is now 4 of 5 rather than 0 of 5.

    Four spans were extracted from PDFs actually on disk, by
    scripts/testing/fetch_and_verify.py, and checked against printed page
    numbers. limit_speed_ics is outstanding for a reason worth keeping visible:
    the PDF first obtained was the IROS 2003 conference paper, not the
    Advanced Robotics 18(10) journal version the card cites, and a span quoted
    from one may not be attributed to the other.

    This test replaces test_no_pilot_card_is_traceable_yet. It fails when the
    last card is verified, and retiring it then is part of finishing.
    """
    prov = STORE.provenance()
    assert set(prov["unverified"]) == UNVERIFIED_PILOT_CARDS
    assert prov["traceable_count"] == len(STORE) - len(UNVERIFIED_PILOT_CARDS)
    for card in STORE:
        expected = card.id not in UNVERIFIED_PILOT_CARDS
        assert card.is_traceable is expected, card.id


def test_every_verified_card_names_who_checked_it():
    """is_traceable only checks span + state. Provenance also means a person is
    accountable, and an empty verified_by would slip past the schema only if a
    card were constructed rather than loaded."""
    for card in STORE:
        if card.is_traceable:
            assert card.source.verified_by.strip(), card.id


def test_verification_requires_a_span_and_someone_accountable():
    with pytest.raises(ValueError, match="no span"):
        Source(citation="x", verified=Verification.VERIFIED)
    with pytest.raises(ValueError, match="no verified_by"):
        Source(citation="x", span="p. 24", verified=Verification.VERIFIED)
    ok = Source(citation="x", span="p. 24", verified=Verification.VERIFIED,
                verified_by="a human")
    assert ok.is_traceable


def test_a_span_alone_does_not_make_a_card_traceable():
    """The failure this schema is shaped to prevent: a plausible-looking span
    that nobody checked."""
    unchecked = Source(citation="x", span="p. 24")
    assert unchecked.span and not unchecked.is_traceable


def test_no_card_may_claim_novelty():
    """Novelty means published after the training cutoff. Every source a model
    can suggest is by construction inside its own training distribution, so a
    hand-curated pilot cannot supply that axis at all — it needs §5's ingestion
    daemon. Saying so is a finding; claiming it would be a lie."""
    for card in STORE:
        assert "novelty" not in card.supplies, card.id
    with pytest.raises(ValueError, match="claims NOVELTY"):
        _load(_card_dict(supplies=["novelty"]))


def test_every_card_says_which_advantage_it_supplies():
    for card in STORE:
        assert card.supplies, card.id
        assert set(card.supplies) <= set(Card.ADVANTAGES)


def test_confidence_is_the_three_point_scale():
    for card in STORE:
        assert isinstance(card.confidence, Confidence)
    counts = STORE.provenance()["by_confidence"]
    assert sum(counts.values()) == len(STORE)


# --- the mapping onto a node -----------------------------------------------

def test_a_prediction_becomes_a_unit_scenario():
    """§4's mapping, made mechanical: Prediction -> the unit test."""
    from harness.micro import MicroScenario, room
    micro = MicroScenario(id="probe", world=room(6.0, 4.0, boxes=[(2.0, -0.1, 2.2, 2.5)]),
                          start=(1.0, 1.0, 0.0), goal=(5.0, 3.0),
                          time_limit=30.0, seed=9001)
    card = STORE.get("reorient_geometry")
    unit = card.unit_scenario(micro)
    assert unit.metric == card.prediction.metric
    assert unit.baseline == card.prediction.baseline
    assert unit.node_id == card.node_name
    assert card.id in unit.note


def test_the_card_to_admission_path_runs_end_to_end():
    """A card's Prediction, turned into a unit scenario, decided by the harness,
    with no human in the loop. The candidate here is a reference controller
    standing in for the node chunk 15 will generate — the point is the pipe, not
    the node."""
    import controllers
    from harness.micro import MicroScenario, room
    from harness.unit import run_unit

    micro = MicroScenario(id="corridor", world=room(6.0, 4.0, boxes=[(2.0, -0.1, 2.2, 2.5)]),
                          start=(1.0, 1.0, 0.0), goal=(5.0, 3.0),
                          time_limit=30.0, seed=9001)
    card = STORE.get("reorient_geometry")          # goal_reached vs goal_seeker
    result = run_unit(lambda: controllers.make("reactive_avoid"),
                      card.unit_scenario(micro))
    assert result.admitted, result.reason


def test_a_card_declares_what_its_mechanism_costs():
    """A card that cannot say what it makes worse is a card that will be
    reported selectively. LimitSpeedToVisibleRange buys clearance with time."""
    for card in STORE:
        assert card.prediction.cost_metric, card.id
        assert card.prediction.cost_metric != card.prediction.metric


# --- retrieval -------------------------------------------------------------

def test_retrieval_by_tag_metric_and_node():
    assert {c.id for c in STORE.by_node("MaintainHomingVector")} == {
        "path_integration_ants", "path_integration_mammals"}
    assert STORE.by_metric("collisions")[0].id == "limit_speed_dwa"
    assert "drift" in STORE.tags()
    assert STORE.by_tag("tier1")


def test_an_unknown_card_names_the_known_ones():
    with pytest.raises(KeyError, match="no card"):
        STORE.get("nope")
