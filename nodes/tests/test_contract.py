"""
nodes/tests/test_contract.py

The contract, exercised end to end without a simulator.

The test that matters most is `test_a_tree_of_two_toy_leaves_is_a_controller`:
a Sequence of typed leaves drops into the same rollout loop the reference
controllers use and produces a scored trace, with **no change to the runner**.
That compatibility is the point of the whole contract — `core.observation.Controller`
was made two methods precisely so a generated behaviour tree would fit — and
without it every later chunk would need runner changes nobody pre-registered.
"""

import dataclasses

import pytest

from core.metrics import score
from core.observation import Command, Observation
from core.scenario import load
from core.tests.kinematic import rollout

import nodes
from nodes.contract import (Action, Condition, Direction, Kind, Node,
                            NodeMetadata, Origin, Confidence, Port,
                            PortAccessError, PortStore, PortType, PortView,
                            Status)
from nodes.library import Library, LibraryError
from nodes.toys import DriveToGoal, IsGoalKnown, toy_metadata
from nodes.tree import (Fallback, Sequence, Tree, WiringError, check_wiring,
                        tick_node)


def _toy_tree():
    return Tree(Sequence(toy_metadata("toy_seq"), [
        IsGoalKnown(toy_metadata("toy_is_goal_known")),
        DriveToGoal(toy_metadata("toy_drive_to_goal")),
    ]))


# --- a tree is a Controller ------------------------------------------------

def test_a_tree_of_two_toy_leaves_is_a_controller():
    """Runs through the same loop as the reference controllers, scored by the
    same metrics, with nothing in the runner changed."""
    sc = load("doorway_traverse")
    tree = _toy_tree()
    assert hasattr(tree, "reset") and hasattr(tree, "step")

    result = score(rollout(sc, tree), sc)
    assert result["control_steps"] > 0
    assert result["termination"] in ("goal", "timeout", "fell")
    # It has no sensing, so it drives into the divider exactly as goal_seeker
    # does. Asserted so that a fixture which silently starts succeeding — and
    # therefore stops testing the failure paths — is a failing test.
    assert not result["goal_reached"]
    assert result["collisions"] >= 1


def test_the_tree_emits_a_twist_and_only_actions_may():
    sc = load("doorway_traverse")
    tree = _toy_tree()
    tree.reset(sc, sc.rng())
    obs = _obs(sc, (1.5, 1.5, 0.0))
    assert isinstance(tree.step(obs), Command)

    store = PortStore()
    condition = IsGoalKnown(toy_metadata("c"))
    with pytest.raises(PortAccessError, match="Only an Action"):
        PortView(store, condition).drive(Command(1.0, 0.0, 0.0))


def _obs(scenario, pose, goal=...):
    import numpy as np
    angles = scenario.beam_angles()
    return Observation(
        t=0.0, ranges=np.full(angles.shape, np.inf), range_angles=angles,
        range_max=scenario.range_max, pose_estimate=pose,
        pose_error_estimate=0.0, contact=False,
        goal=scenario.goal if goal is ... else goal)


# --- the undisclosed goal --------------------------------------------------

def test_a_leaf_handles_an_undisclosed_goal_without_reading_a_stale_one():
    """chunk 11a's contract, from the node side. The Sequence stops at the
    condition, the action never ticks, and the tree emits STOP — rather than
    steering at wherever the goal happened to be last episode."""
    sc = load("doorway_traverse")
    tree = _toy_tree()
    tree.reset(sc, sc.rng())

    command = tree.step(_obs(sc, (1.5, 1.5, 0.0), goal=None))
    assert command.as_tuple() == (0.0, 0.0, 0.0)

    # and with a goal it does move, so the assertion above is about disclosure
    # and not about the tree being inert.
    assert tree.step(_obs(sc, (1.5, 1.5, 0.0))).as_tuple() != (0.0, 0.0, 0.0)


def test_the_condition_reports_failure_when_the_goal_is_withheld():
    sc = load("doorway_traverse")
    store = PortStore()
    node = IsGoalKnown(toy_metadata("c"))
    assert tick_node(node, _obs(sc, (1.5, 1.5, 0.0), goal=None), store) is Status.FAILURE
    assert store.values["goal_known"] is False


# --- ports are checked when the tree is built ------------------------------

class _WrongType(Action):
    PORTS = (Port("goal_known", PortType.SPEED, Direction.IN),)
    EFFECT = "reads a Flag as a Speed, which is the bug under test"

    def tick(self, obs, ports):
        return Status.SUCCESS


class _NoProducer(Action):
    PORTS = (Port("nobody_writes_this", PortType.BEARING, Direction.IN),)
    EFFECT = "reads a port nothing produces"

    def tick(self, obs, ports):
        return Status.SUCCESS


def test_a_type_mismatch_is_rejected_at_composition_time_not_tick_time():
    """There is no Observation anywhere in this test. The tree cannot be built,
    so it can never reach an episode — which matters because a wiring fault that
    surfaced mid-episode would score a timeout and read as a bad idea rather
    than a bad connection, and G3 counts solved scenarios."""
    bad = Sequence(toy_metadata("seq"), [
        IsGoalKnown(toy_metadata("c")),
        _WrongType(toy_metadata("a")),
    ])
    with pytest.raises(WiringError, match="Speed.*produced as Flag|Flag"):
        Tree(bad)


def test_an_unproduced_port_is_rejected_at_composition_time():
    with pytest.raises(WiringError, match="nothing earlier in the tree"):
        Tree(Sequence(toy_metadata("seq"), [_NoProducer(toy_metadata("a"))]))


def test_a_consumer_before_its_producer_is_rejected():
    """Tick order is part of wiring: you cannot read what has not been written."""
    with pytest.raises(WiringError, match="nothing earlier in the tree"):
        Tree(Sequence(toy_metadata("seq"), [
            DriveToGoal(toy_metadata("a")),
            IsGoalKnown(toy_metadata("c")),
        ]))


def test_a_node_cannot_touch_a_port_it_did_not_declare():
    """The 'explicit ports, not a blackboard' decision, enforced rather than
    documented. Without this, 'no coupling to a specific scenario' is a
    convention a generated node might or might not respect."""
    sc = load("doorway_traverse")
    store = PortStore()
    store.values["something_else"] = 1.0
    view = PortView(store, IsGoalKnown(toy_metadata("c")))
    with pytest.raises(PortAccessError, match="does not declare as an input"):
        view["something_else"]
    with pytest.raises(PortAccessError, match="does not declare as an output"):
        view["something_else"] = 2.0


# --- metadata --------------------------------------------------------------

class _NoEffect(Action):
    PORTS = ()
    EFFECT = ""

    def tick(self, obs, ports):
        return Status.SUCCESS


def test_a_node_without_metadata_is_rejected():
    with pytest.raises(ValueError, match="must be a NodeMetadata"):
        IsGoalKnown("toy_is_goal_known")


def test_a_node_without_an_effect_is_rejected():
    """The effect is what composition selects on and what the unit scenario
    tests. A node that cannot say what it changes cannot be admitted."""
    with pytest.raises(ValueError, match="EFFECT is empty"):
        _NoEffect(toy_metadata("a"))


def test_a_card_node_must_actually_be_traceable():
    """G1 reports the fraction carrying a source span WITH extraction
    confidence. A node claiming a card but carrying no span would inflate it."""
    with pytest.raises(ValueError, match="origin is CARD but"):
        NodeMetadata(node_id="n", origin=Origin.CARD, card_id="card-1")
    ok = NodeMetadata(node_id="n", origin=Origin.CARD, card_id="card-1",
                      source_span="Fox et al. 1997, p. 24",
                      confidence=Confidence.HIGH)
    assert ok.is_traceable


def test_origin_is_required_and_handwritten_is_not_traceable():
    with pytest.raises(ValueError, match="origin must be an Origin"):
        NodeMetadata(node_id="n", origin="card")
    assert not toy_metadata("t").is_traceable


def test_confidence_must_be_the_three_point_scale():
    with pytest.raises(ValueError, match="must be a Confidence"):
        NodeMetadata(node_id="n", origin=Origin.INVENTED, confidence=0.8)


# --- the library -----------------------------------------------------------

def _library():
    lib = Library()
    lib.add(IsGoalKnown, toy_metadata("toy_is_goal_known"))
    lib.add(DriveToGoal, toy_metadata("toy_drive_to_goal"))
    return lib


def test_the_snapshot_tag_is_stable_for_the_same_contents():
    assert _library().snapshot() == _library().snapshot()


def test_the_snapshot_tag_does_not_depend_on_admission_order():
    other = Library()
    other.add(DriveToGoal, toy_metadata("toy_drive_to_goal"))
    other.add(IsGoalKnown, toy_metadata("toy_is_goal_known"))
    assert other.snapshot() == _library().snapshot()


def test_the_snapshot_tag_changes_when_a_node_is_added():
    lib = _library()
    before = lib.snapshot()
    lib.add(_NoProducer, toy_metadata("extra"))
    assert lib.snapshot() != before
    assert before.startswith("lib-002-") and lib.snapshot().startswith("lib-003-")


def test_the_library_is_append_only():
    """#7. Adding never mutates or removes an existing entry, and an id is never
    reused — a revision is a new node, so both snapshot tags stay meaningful."""
    lib = _library()
    before_contents = lib.contents()
    before_ids = [e.node_id for e in lib.all()]

    with pytest.raises(LibraryError, match="append-only"):
        lib.add(IsGoalKnown, toy_metadata("toy_is_goal_known"))

    lib.add(_NoProducer, toy_metadata("extra"))
    after_ids = [e.node_id for e in lib.all()]
    assert after_ids[:len(before_ids)] == before_ids
    assert lib.contents()[:0] == []
    for old in before_contents:
        assert old in lib.contents(), "an existing entry changed"


def test_the_library_rejects_a_node_it_cannot_build():
    lib = Library()
    with pytest.raises(LibraryError, match="cannot be constructed"):
        lib.add(_NoEffect, toy_metadata("bad"))
    assert len(lib) == 0


def test_retrieval_by_port_type_and_tag():
    lib = _library()
    assert [e.node_id for e in lib.by_port(PortType.FLAG, Direction.OUT)] \
        == ["toy_is_goal_known"]
    assert [e.node_id for e in lib.by_port(PortType.FLAG, Direction.IN)] \
        == ["toy_drive_to_goal"]
    assert len(lib.by_port(PortType.FLAG)) == 2
    assert lib.by_port(PortType.REGION) == []
    assert [e.node_id for e in lib.by_tag("drive")] == ["toy_drive_to_goal"]
    assert "disclosure" in lib.tags()


def test_provenance_is_computed_not_asserted():
    """G1's numbers come out of the library, so 'the fraction carrying a source
    span' is never a figure someone typed into a report."""
    lib = _library()
    p = lib.provenance()
    assert p["node_count"] == 2 and p["traceable_count"] == 0
    assert p["by_origin"] == {"handwritten": 2}

    lib.add(_NoProducer, NodeMetadata(
        node_id="sourced", origin=Origin.CARD, card_id="card-1",
        source_span="Fox et al. 1997, p. 24", confidence=Confidence.MEDIUM))
    p = lib.provenance()
    assert (p["node_count"], p["traceable_count"]) == (3, 1)
    assert p["traceable_fraction"] == pytest.approx(1 / 3)


def test_an_entry_builds_a_working_node():
    entry = _library().get("toy_is_goal_known")
    assert isinstance(entry.build(), IsGoalKnown)


# --- composites ------------------------------------------------------------

def test_fallback_takes_the_first_child_that_does_not_fail():
    sc = load("doorway_traverse")
    tree = Tree(Fallback(toy_metadata("fb"), [
        IsGoalKnown(toy_metadata("c")),
        IsGoalKnown(toy_metadata("c2")),
    ]))
    tree.reset(sc, sc.rng())
    store = PortStore()
    assert tick_node(tree.root, _obs(sc, (1.5, 1.5, 0.0)), store) is Status.SUCCESS


def test_an_empty_composite_is_rejected():
    with pytest.raises(WiringError, match="no children"):
        Sequence(toy_metadata("seq"), [])


def test_the_snapshot_tag_reaches_the_manifest():
    """Non-negotiable #8: every reported result names the frozen library
    snapshot it was produced from. `manifest["library_snapshot"]` has been None
    since the first run; this is the chunk that gives it something to hold."""
    from core.manifest import capture
    tag = _library().snapshot()
    manifest = capture(scene="scenes/lab42_v1.usd", library_snapshot=tag)
    assert manifest["library_snapshot"] == tag

    tree = _toy_tree()
    assert tree.snapshot is None
    assert Tree(tree.root, snapshot=tag).snapshot == tag


def test_the_registry_resolves_node_classes():
    assert nodes.available() == ["toy_drive_to_goal", "toy_is_goal_known"]
    assert nodes.node_class("toy_is_goal_known") is IsGoalKnown
    with pytest.raises(KeyError, match="unknown node"):
        nodes.node_class("nope")
