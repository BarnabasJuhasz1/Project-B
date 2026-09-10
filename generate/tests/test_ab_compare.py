"""
generate/tests/test_ab_compare.py

The A-vs-B pairing rule, pinned.

`scripts/ab_compare.py` reports the first number this project has that compares
condition A with condition B, and the number is only meaningful if the SET it is
computed over is the right one. Three things can quietly corrupt it and none of
them would raise:

- a **test fixture** card (`vague_adaptive_motion`) drifting into the pairing,
  which would put a deliberately-bad card into a headline comparison;
- a **superseded** generation being counted alongside its replacement, which
  reports one mechanism as two;
- a **retired** record — the four naive-arm nodes — being pooled with the
  current condition A, which computes one number over two different treatments
  and labels it with A's letter.

Those are the same three exclusions `generate/admit.py:pass_rate` applies, and
these tests exist so the two cannot drift apart: a pairing over a different set
than the pass rate would describe a different library than the number beside it.

**No model is called.** Everything here reads committed staging records.
"""

import importlib.util
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent.parent


def _load_script():
    """Import `scripts/ab_compare.py` as a module.

    Loaded by path because `scripts/` is not a package — the same reason
    `scripts/` holds entry points and `generate/` holds the library they call.
    """
    path = REPO_ROOT / "scripts" / "ab_compare.py"
    spec = importlib.util.spec_from_file_location("ab_compare", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["ab_compare"] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def ab():
    return _load_script()


@pytest.fixture(scope="module")
def records(ab, cards):
    return ab.staged_records(cards=cards)


@pytest.fixture(scope="module")
def paired(ab, records):
    return ab.pairs(records[0])


# The five pilot cards, and the fact that makes this chunk possible: each one
# now carries BOTH arms. Written out rather than derived, so that a card
# appearing or disappearing from the pairing is a test failure and not a
# silently different headline number.
PILOT_CARDS = {
    "limit_speed_dwa", "limit_speed_ics", "path_integration_ants",
    "path_integration_mammals", "reorient_geometry",
}


def test_the_pairing_is_exactly_the_five_pilot_cards(paired):
    assert set(paired) == PILOT_CARDS


def test_every_pair_has_one_a_node_and_one_b_node(paired):
    for ref_id, arms in paired.items():
        assert set(arms) == {"a", "b"}, ref_id
        # A is pointed at the paper; B is handed the extracted card. The two
        # differ by exactly one input and `is_card` is where that is recorded.
        assert arms["a"]["is_card"] is False, ref_id
        assert arms["b"]["is_card"] is True, ref_id


def test_both_arms_of_a_pair_name_the_same_paper(paired):
    """The whole design. If the two arms came from different cards this would
    be an unpaired comparison of two samples wearing a paired label."""
    for ref_id, arms in paired.items():
        assert arms["a"]["ref_id"] == ref_id
        assert arms["b"]["ref_id"] == ref_id


def test_the_retired_naive_arm_is_excluded(ab, records):
    """The four `a__<task>__<hash>` records were generated under a definition of
    condition A that no longer exists — A received no literature at all. They
    stay on disk (the denominator may not be edited after the fact) and they
    must not reach a comparison labelled with A's current meaning."""
    _, excluded = records
    retired = {node_id for node_id, why in excluded if why == "retired"}
    assert len(retired) == 4, retired
    kept = {rec["node_id"] for rec in records[0]}
    assert not (retired & kept)


def test_the_fixture_card_is_excluded(ab, records):
    """`vague_adaptive_motion` is the deliberately-bad card. It was ADMITTED —
    that is `generate/FINDINGS.md` §1, the harness cannot detect card
    infidelity — so nothing but this exclusion keeps it out."""
    _, excluded = records
    assert any(why == "fixture" for _, why in excluded)
    for ref_id in _load_refs(records[0]):
        assert ref_id != "vague_adaptive_motion"


def test_the_superseded_generation_is_excluded(ab, records):
    """`gen_b_1f2fe8016b1441dc` is the pre-verification generation of
    `limit_speed_dwa`'s B node. Counting it beside `gen_b_d6ae93648c7b4f25`
    would put one mechanism in the comparison twice."""
    _, excluded = records
    superseded = {node_id for node_id, why in excluded if why == "superseded"}
    assert "gen_b_1f2fe8016b1441dc" in superseded
    kept = {rec["node_id"] for rec in records[0]}
    assert "gen_b_1f2fe8016b1441dc" not in kept
    assert "gen_b_d6ae93648c7b4f25" in kept


def _load_refs(records):
    return {rec["ref_id"] for rec in records if rec["ref_id"]}


def test_the_pairing_agrees_with_the_pass_rate_denominator(ab, records, cards):
    """The set this comparison runs over must be the set G1's pass rate
    describes. They are computed by different code and this is what stops them
    diverging."""
    from generate.admit import pass_rate

    # v1 EXPLICITLY. Chunk 39's pairing is over the one-call records, and
    # `pass_rate` refuses to pool pipelines since chunk 42 — so a number about
    # those records has to ask for them by name.
    rates = pass_rate(ab.STAGING, cards=cards,
                      pipeline_version="v1-single-call")
    kept = records[0]
    for arm in ("a", "b"):
        bucket = rates[arm]
        counted = bucket["admitted"] + bucket["rejected"]
        assert sum(1 for r in kept if r["condition"] == arm) == bucket["admitted"]
        assert bucket["admitted"] <= counted


def test_both_framings_build_for_every_paired_node(ab, paired, cards):
    """Ten nodes, two trees each, all constructed. A tree that does not build is
    not a measurement, and `Tree` type-checks its wiring at CONSTRUCTION — so
    this is where an unsatisfied port surfaces, not mid-rollout."""
    built = 0
    for ref_id, arms in paired.items():
        for arm, rec in sorted(arms.items()):
            node_cls, metadata = ab.load_node(rec, cards=cards)
            solo = ab.solo_factory(node_cls, metadata)()
            carried = ab.carried_factory(node_cls, metadata)()
            from harness.ablate import ablatable
            assert metadata.node_id in ablatable(solo), (ref_id, arm)
            assert metadata.node_id in ablatable(carried), (ref_id, arm)
            built += 1
    assert built == 10


def test_the_carrier_is_present_in_the_carried_framing_and_absent_from_solo(
        ab, paired, cards):
    """The carrier must be identical in both arms — that is what makes it cancel
    out of a paired difference — and it must not be in the solo tree, or solo
    would stop measuring the node alone."""
    from harness.ablate import ablatable

    for ref_id, arms in paired.items():
        for arm, rec in sorted(arms.items()):
            node_cls, metadata = ab.load_node(rec, cards=cards)
            assert ab.CARRIER_ID in ablatable(
                ab.carried_factory(node_cls, metadata)()), (ref_id, arm)
            assert ab.CARRIER_ID not in ablatable(
                ab.solo_factory(node_cls, metadata)()), (ref_id, arm)


def test_no_generated_node_declares_an_in_port(ab, paired, cards):
    """⚠️ THE FINDING CHUNK 40 EXISTS FOR, pinned here so it cannot be fixed
    silently. Every generated node publishes outputs and reads none, so B's
    mechanisms cannot be wired to anything and cannot act as modifiers. If this
    test ever fails, the fair comparison has become possible and the write-up's
    caveat is out of date."""
    from nodes.contract import Direction

    for ref_id, arms in paired.items():
        for arm, rec in sorted(arms.items()):
            node_cls, _ = ab.load_node(rec, cards=cards)
            ins = [p.name for p in node_cls.PORTS
                   if p.direction is Direction.IN]
            assert ins == [], (ref_id, arm, ins)


def test_the_sealed_split_is_unreachable_from_this_script(ab):
    """There is no flag pointing this at the held-out scenarios, and there must
    not be: the measurement feeds back into which nodes get built, which is
    exactly the leak G3 exists to prevent."""
    source = (REPO_ROOT / "scripts" / "ab_compare.py").read_text(
        encoding="utf-8")
    assert "scenarios_held_out" not in source
    assert "HELD_OUT" not in source
    assert "DEV_SCENARIOS" in source


def test_every_generated_node_both_drives_and_publishes_a_port(ab, cards):
    """⚠️ THE TWO-EFFECT VIOLATION, COUNTED. `nodes/contract.py` requires one
    precondition and one effect; `compose/FINDINGS.md` §2 recorded a single node
    breaking it and asked chunk 17 to decide whether an Action that drives may
    also publish a port.

    It is not one node. It is EVERY generated node, and the count is asserted
    rather than described so that chunk 40 landing a modifier kind shows up here
    as a failure instead of as a stale paragraph.

    The `.drive(` check is textual on purpose: it asks what the model WROTE. A
    node whose precondition never passes still drives in every sense that
    matters to the contract, and a runtime probe would score it as innocent for
    the wrong reason — which is exactly how `LimitSpeedToOcclusionBoundary`
    (0 m travelled, precondition never true) would have escaped.
    """
    from pathlib import Path

    from nodes.contract import Direction

    # The script's own STAGING, not a second copy of the path. It points at the
    # ARCHIVED v1 records since 2026-09-08, and a test that recomputed the
    # location would silently read a different set than the code under test.
    staging = ab.STAGING
    records, _ = ab.staged_records(cards=cards)
    generated = [r for r in records if r["node_id"].startswith("gen_")]
    assert generated, "no generated nodes on disk; the corpus run has not happened"

    offenders = []
    for rec in generated:
        source = (staging / f"{rec['stem']}.py").read_text(encoding="utf-8")
        node_cls, _ = ab.load_node(rec, cards=cards)
        publishes = [p.name for p in node_cls.PORTS
                     if p.direction is Direction.OUT]
        if ".drive(" in source and publishes:
            offenders.append(node_cls.__name__)

    assert len(offenders) == len(generated) == 14, (
        f"{len(offenders)} of {len(generated)} generated nodes drive AND "
        f"publish. If this dropped, a modifier kind landed (chunk 40) and "
        f"out/evidence/nodes/ab-comparison/README.md is out of date.")
