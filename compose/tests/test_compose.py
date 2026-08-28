"""
compose/tests/test_compose.py

Plan -> tree, and the properties that make G3's "recombination alone, no new
primitives" a mechanical claim rather than a promise.

Every model-dependent test here replays a committed transcript. None calls a
model.
"""

import json

import pytest

from compose.compose import (Composition, CompositionError, RECORDS, build,
                             compose, load_composition, parse_plan, render,
                             repair)
from compose.tests.conftest import (NAVIGATE_ID, REPAIR_ID, SEARCH_ID,
                                    SPLIT_PLAN_ID, STARVED_ID)
from nodes.tree import Tree


def plan_of(composition_id):
    record = json.loads(
        (RECORDS / f"{composition_id}.json").read_text(encoding="utf-8"))
    return record["plan"]


# --- the vocabulary is the retrieved set -----------------------------------

def test_a_plan_naming_an_unretrieved_node_is_rejected(found):
    """The whole basis of 'no new primitives'. A model that wanted to invent a
    node has nowhere to put it: the only thing a plan can contain is an id, and
    an id that was not retrieved does not resolve."""
    with pytest.raises(CompositionError) as exc:
        build({"tree": {"type": "leaf", "node": "InventedFrontierExplorer"}},
              found)
    assert exc.value.reason == "UNRETRIEVED_NODE"


def test_a_plan_cannot_smuggle_code(found):
    """There is no code path from a plan to an exec(). The plan is JSON and the
    builder only ever looks node ids up in a dict — asserted here so that a
    later refactor introducing one fails a test rather than a review."""
    with pytest.raises(CompositionError):
        build({"tree": {"type": "leaf", "node": "class X(Action): pass"}}, found)


def test_a_tree_with_no_action_is_rejected(found):
    with pytest.raises(CompositionError) as exc:
        build({"tree": {"type": "leaf", "node": "toy_is_goal_known"}}, found)
    assert exc.value.reason == "NO_ACTION"


def test_bad_wiring_fails_at_composition_not_mid_episode(found):
    """Chunk 12's decision, cashed in. A tree that failed mid-episode would
    score a timeout and look like a bad idea rather than a bad wiring, and G3
    counts solved scenarios."""
    with pytest.raises(CompositionError) as exc:
        build({"tree": {"type": "leaf", "node": "toy_avoid_obstacles"}}, found)
    assert exc.value.reason == "WIRING_FAILED"
    assert "goal_known" in exc.value.detail


def test_a_leaf_cannot_be_used_as_a_decorator(found):
    with pytest.raises(CompositionError) as exc:
        build({"tree": {"type": "decorator", "node": "toy_avoid_obstacles",
                        "child": {"type": "leaf", "node": "toy_is_goal_known"}}},
              found)
    assert exc.value.reason == "KIND_MISMATCH"


@pytest.mark.parametrize("plan, reason", [
    ({"tree": {"type": "sequence"}}, "MALFORMED_PLAN"),
    ({"tree": {"type": "sequence", "children": []}}, "MALFORMED_PLAN"),
    ({"tree": {"type": "selector", "children": []}}, "MALFORMED_PLAN"),
    ({"tree": "toy_avoid_obstacles"}, "MALFORMED_PLAN"),
])
def test_a_malformed_plan_is_a_reason_code_not_a_crash(plan, reason, found):
    with pytest.raises(CompositionError) as exc:
        build(plan, found)
    assert exc.value.reason == reason


def test_no_fenced_json_is_a_recorded_outcome():
    """The composition counterpart of NO_CANDIDATE_EMITTED. A model that
    declines belongs in the denominator, not in a traceback."""
    with pytest.raises(CompositionError) as exc:
        parse_plan("I cannot compose this from the nodes available.")
    assert exc.value.reason == "NO_PLAN_EMITTED"


def test_two_fenced_blocks_are_refused():
    """Picking one would be the harness choosing the composition."""
    with pytest.raises(CompositionError) as exc:
        parse_plan("```json\n{}\n```\ntext\n```json\n{}\n```\n")
    assert exc.value.reason == "MULTIPLE_PLANS"


# --- composite ids ----------------------------------------------------------

def test_composite_ids_are_deterministic(found):
    plan = plan_of(NAVIGATE_ID)
    first = [n.metadata.node_id for n in build(plan, found).nodes()]
    second = [n.metadata.node_id for n in build(plan, found).nodes()]
    assert first == second


def test_a_composite_carries_no_provenance(found):
    """A composite emits nothing of its own, so it names no card and no source.
    Where the CONTROL FLOW came from is recorded in the composition record, with
    the prompt hash that produced it."""
    from nodes.contract import Origin
    from nodes.tree import Composite
    for node in build(plan_of(NAVIGATE_ID), found).nodes():
        if isinstance(node, Composite):
            assert node.metadata.origin is Origin.HANDWRITTEN
            assert node.metadata.card_id is None
            assert node.metadata.source_span is None


# --- the replayed compositions ----------------------------------------------

def test_composing_from_a_committed_transcript_reproduces_the_record(
        library, spec, found, replay, tmp_path):
    """Reproducible, not repeatable — the property chunk 15 established and
    `ReplayBackend` exists to provide."""
    result = compose(spec, library, replay, retrieval=found,
                     include_toys=True, records=tmp_path,
                     transcripts=tmp_path)
    assert result.ok, f"{result.reason}: {result.detail}"
    assert result.composition_id == NAVIGATE_ID
    committed = json.loads(
        (RECORDS / f"{NAVIGATE_ID}.json").read_text(encoding="utf-8"))
    assert result.as_dict() == committed


def test_a_recorded_rejection_is_kept_with_its_reason():
    """G3 needs a denominator, so a composition that failed is recorded rather
    than retried away. This one split a single object across two fenced blocks,
    which the output contract refuses for the same reason `extract_code` refuses
    two python blocks: merging them would be the harness composing."""
    record = json.loads(
        (RECORDS / f"{SPLIT_PLAN_ID}.json").read_text(encoding="utf-8"))
    assert record["ok"] is False
    assert record["reason"] == "MULTIPLE_PLANS"
    assert record["plan"] is None and record["tree_nodes"] is None


def test_a_composition_can_succeed_and_still_not_work():
    """The finding this chunk turned up, kept as a committed pair.

    Both records composed, against the same library and the same retrieval. One
    reached the goal; the other put a driving Action ahead of another inside a
    Sequence, where RUNNING short-circuits, and starved it. Nothing in a port
    contract says how often a precondition fires, so nothing available to
    composition could have told the two apart — which is why the working
    demonstration is not evidence on its own, and why composition rate is never
    a proxy for goal-reach rate (PRE-REGISTRATION.md §8.3).

    **The starved tree is no longer executable, and the reason is specific.**
    The Action it starves on IS `gen_b_1f2fe8016b1441dc`, the generation of
    `limit_speed_dwa` that was superseded when the card was verified. Rebuilding
    it would need a library that no longer loads, and `load_composition` refuses
    that by design (#8). So the half of the pair that still runs is checked by
    running it, and the half that does not is read off its record — structurally,
    from the plan, not from the rationale the model wrote about the plan.
    """
    from core.metrics import score
    from core.scenario import load
    from core.tests.kinematic import rollout
    import controllers

    scenario = load("doorway_traverse")
    good = score(rollout(scenario, controllers.make(f"composed:{NAVIGATE_ID}")),
                 scenario)
    assert good["goal_reached"] is True

    for composition_id in (NAVIGATE_ID, STARVED_ID):
        record = json.loads(
            (RECORDS / f"{composition_id}.json").read_text(encoding="utf-8"))
        assert record["ok"] is True, (
            f"{composition_id} is recorded as a failed composition. The pair is "
            f"only a finding because BOTH succeeded.")

    starved = json.loads(
        (RECORDS / f"{STARVED_ID}.json").read_text(encoding="utf-8"))
    # `retrieval.considered` carries each offered node's kind, so the plan can be
    # read for the shape that starves without loading a single node class.
    kinds = {c["node_id"]: c["kind"] for c in starved["retrieval"]["considered"]}

    def starving_sequences(node):
        """Sequences whose first child is an Action and which hold another."""
        if not isinstance(node, dict):
            return
        children = node.get("children") or []
        if node.get("type") == "sequence" and len(children) > 1:
            leaves = [c.get("node") for c in children if c.get("type") == "leaf"]
            if len(leaves) > 1 and all(kinds.get(n) == "action" for n in leaves):
                yield leaves
        for child in children:
            yield from starving_sequences(child)

    starved_pairs = list(starving_sequences(starved["plan"]["tree"]))
    assert starved_pairs, (
        "the archived plan no longer contains an Action sequenced ahead of "
        "another Action, which is the shape that starved it. If the record "
        "changed, the finding it is evidence for changed with it.")
    assert "gen_b_1f2fe8016b1441dc" in starved_pairs[0]


def test_every_committed_composition_names_its_library_snapshot():
    """Non-negotiable #8. A composition is not a reported result yet, but the
    tag is what makes one attributable when it becomes one."""
    for path in RECORDS.glob("*.json"):
        record = json.loads(path.read_text(encoding="utf-8"))
        assert record["library_snapshot"].startswith("lib-")


def test_every_committed_composition_created_no_primitive():
    for path in RECORDS.glob("*.json"):
        record = json.loads(path.read_text(encoding="utf-8"))
        assert record["created_primitives"] == []


def test_the_log_separates_a_repair_from_an_initial_composition():
    """Chunk 16 requires the two to be separately logged, and requires a repair
    to rewire rather than create. It cannot create — the vocabulary is the
    previous retrieval — so the log's job is to say which kind it was."""
    initial = json.loads(
        (RECORDS / f"{SEARCH_ID}.json").read_text(encoding="utf-8"))
    repaired = json.loads(
        (RECORDS / f"{REPAIR_ID}.json").read_text(encoding="utf-8"))
    assert initial["kind"] == "initial" and initial["repair_of"] is None
    assert repaired["kind"] == "repair"
    assert repaired["repair_of"] == SEARCH_ID
    # Rewiring, not creating: the same library, the same offered set.
    assert repaired["library_snapshot"] == initial["library_snapshot"]
    assert repaired["retrieval"]["retrieved"] == initial["retrieval"]["retrieved"]
    leaves = {n for n in repaired["tree_nodes"]
              if not n.startswith("composed_")}
    assert leaves <= set(initial["retrieval"]["retrieved"])


def test_repair_replays_from_the_same_retrieval(library, replay, tmp_path):
    """The committed repair, reproduced. `repair()` is handed the PREVIOUS
    composition's retrieval, so the only thing that changed between the two
    prompts is the failure signature — which is what makes the pair readable as
    a repair rather than as two attempts."""
    from compose.retrieve import Retrieval
    from compose.spec import TaskSpec

    initial = json.loads(
        (RECORDS / f"{SEARCH_ID}.json").read_text(encoding="utf-8"))
    repaired = json.loads(
        (RECORDS / f"{REPAIR_ID}.json").read_text(encoding="utf-8"))
    signature = repaired["spec"]["failure_signature"]

    previous = Composition(
        composition_id=SEARCH_ID, spec=TaskSpec(**initial["spec"]),
        retrieval=Retrieval(
            entries=[library.get(n) for n in initial["retrieval"]["retrieved"]],
            query_tags=tuple(initial["retrieval"]["query_tags"]),
            library_size=initial["retrieval"]["library_size"]),
        snapshot=initial["library_snapshot"], prompt="", response=None,
        include_toys=True)

    result = repair(previous, signature, replay, library=library,
                    records=tmp_path, transcripts=tmp_path)
    assert signature in result.prompt
    assert result.kind == "repair" and result.repair_of == SEARCH_ID
    assert result.composition_id == REPAIR_ID
    assert result.as_dict() == repaired


# --- rebuilding -------------------------------------------------------------

def test_a_recorded_composition_rebuilds_into_the_same_tree():
    tree = load_composition(NAVIGATE_ID)
    assert isinstance(tree, Tree)
    record = json.loads(
        (RECORDS / f"{NAVIGATE_ID}.json").read_text(encoding="utf-8"))
    assert [n.metadata.node_id for n in tree.nodes()] == record["tree_nodes"]
    assert tree.snapshot == record["library_snapshot"]


def test_rebuilding_against_a_grown_library_is_refused(tmp_path):
    """#7 makes the library append-only, so a snapshot mismatch means it grew.
    Running an old plan under a new tag would make #8's claim a lie."""
    record = json.loads(
        (RECORDS / f"{NAVIGATE_ID}.json").read_text(encoding="utf-8"))
    record["library_snapshot"] = "lib-999-deadbeefcafe"
    (tmp_path / f"{NAVIGATE_ID}.json").write_text(json.dumps(record))
    with pytest.raises(CompositionError) as exc:
        load_composition(NAVIGATE_ID, records=tmp_path)
    assert exc.value.reason == "SNAPSHOT_MISMATCH"


def test_a_failed_composition_never_becomes_a_controller(tmp_path):
    (tmp_path / "flop.json").write_text(json.dumps(
        {"ok": False, "reason": "NO_PLAN_EMITTED"}))
    with pytest.raises(CompositionError) as exc:
        load_composition("flop", records=tmp_path)
    assert exc.value.reason == "COMPOSITION_FAILED"


# --- the runner -------------------------------------------------------------

def test_a_composed_tree_resolves_through_the_controller_registry():
    """No runner change and nothing registered by hand: `run_episode.py` calls
    `controllers.make(--controller)` and a Tree is already a Controller."""
    import controllers
    tree = controllers.make(f"composed:{NAVIGATE_ID}")
    assert isinstance(tree, Tree)
    assert f"composed:{NAVIGATE_ID}" in controllers.compositions()


def test_a_composition_is_never_in_the_safe_listing():
    """`available()` is what a batch runner sweeps. A composed tree is a
    condition only once someone names it and its snapshot."""
    import controllers
    assert not any(n.startswith("composed:") for n in controllers.available())


def test_the_composed_tree_reaches_the_goal_on_doorway_traverse():
    """Chunk 16's headline check, on the sim-free rollout.

    NOT A BENCHMARK NUMBER — `core/tests/kinematic.py` is an idealised unicycle
    with the frozen locomotion policy absent entirely, and it says so. What it
    establishes is that the composition path produces a tree that works, rather
    than one that merely constructs; the reported number comes from
    `run_episode.py` inside the pinned container.
    """
    import controllers
    from core.metrics import score
    from core.scenario import load
    from core.tests.kinematic import rollout

    scenario = load("doorway_traverse")
    result = score(rollout(scenario, controllers.make(f"composed:{NAVIGATE_ID}")),
                   scenario)
    assert result["goal_reached"] is True
    assert result["spl"] > 0.9
