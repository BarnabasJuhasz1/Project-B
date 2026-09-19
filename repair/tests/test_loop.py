"""repair/loop.py — the loop. ⚠️ No test here calls a model."""
from __future__ import annotations

import inspect
import json
import re

import pytest

from repair import loop as L
from repair.verdict import Measurement, WorldOutcome

NODE = "gen_b_44d12df4d25b437f"


def _measurement(node_id="n", outcomes=("broke",) * 5, fire=0.9, changed=0.9):
    rows = tuple(WorldOutcome(seed=2000 + i, scenario_id=f"s{i}",
                              base_reached=True, pair_reached=False,
                              outcome=o, fire=fire, changed=changed)
                 for i, o in enumerate(outcomes))
    return Measurement(node_id=node_id, tier="maze", worlds=rows, metrics={},
                       fire=fire, changed=changed)


# --------------------------------------------------------------------------- #
# ⚠️ the two rules that carry the design


def test_inert_is_not_a_terminal_exit():
    """⚠️ The cheapest way to stop a node regressing is to make it decline
    everywhere. A loop rewarded for "no longer regresses" finds this
    immediately, and every node converges on doing nothing while the table
    fills with clean rows."""
    assert L._terminal_for("INERT") is None
    assert L._terminal_for("HELPED") is L.Exit.HELPED


@pytest.mark.parametrize("label", ["INERT", "DEGENERATE", "REGRESSED",
                                   "MISJUDGED", "ACCEPT"])
def test_only_helped_terminates_on_the_verdict_alone(label):
    """Everything else needs the model's judgement or another round."""
    assert L._terminal_for(label) is None


def test_all_five_exits_exist_and_none_is_an_error():
    assert {e.value for e in L.Exit} == {
        "HELPED", "ACCEPT", "UNTESTABLE", "NOT_TRANSFERABLE", "UNREPAIRED"}


# --------------------------------------------------------------------------- #
# oscillation


def test_the_signature_is_about_behaviour_not_source():
    a = _measurement(outcomes=("broke", "broke", "both_ok"))
    b = _measurement(outcomes=("broke", "broke", "both_ok"))
    c = _measurement(outcomes=("broke", "fixed", "both_ok"))
    assert L.signature_of(a) == L.signature_of(b)
    assert L.signature_of(a) != L.signature_of(c)


def test_a_changed_fire_rate_changes_the_signature():
    assert (L.signature_of(_measurement(fire=0.9)) !=
            L.signature_of(_measurement(fire=0.1)))


def test_a_repeated_signature_ends_the_lineage(monkeypatch):
    """⚠️ A repair can land back on something behaviourally identical to a
    round already tried. Without this the loop spends the rest of its budget
    rediscovering it."""
    m = _measurement()
    monkeypatch.setattr(L, "_measure", lambda *a, **k: m)
    monkeypatch.setattr("repair.verdict.claim_for",
                        lambda *a, **k: __import__("repair.verdict", fromlist=["Claim"]
                                                   ).Claim("goal_reached", 0.1,
                                                           "higher", "card"))
    seen = {L.signature_of(m)}
    rec, done, _ = L.run_round("n", 2, 4, seen=seen)
    assert done is L.Exit.UNREPAIRED
    assert "repeats an earlier round" in rec.outcome


# --------------------------------------------------------------------------- #
# ⚠️ round-numbered staging — plan_hash collides


def test_rounds_get_separate_directories():
    """`plan_hash` is a pure function of (condition, task, card) and the card is
    NOT regenerated, so every round produces the IDENTICAL stem. Two rounds
    cannot share a row key — the reason chunk 52 moved the v2 records aside."""
    assert L.round_dir(NODE, 1) != L.round_dir(NODE, 2)
    assert L.round_dir(NODE, 1).name == "round-01"


def test_rounds_stage_outside_the_experiments_record():
    """`generate/staging/` is the experiment's record. A repair round is a
    candidate, not a library member, until it earns it."""
    from generate.generator import STAGING
    assert str(STAGING) not in str(L.round_dir(NODE, 1))


# --------------------------------------------------------------------------- #
# the repair prompt


def test_render_repair_cannot_be_told_which_arm_it_is():
    params = set(inspect.signature(L.render_repair).parameters)
    for banned in ("condition", "arm", "treatment", "origin"):
        assert banned not in params


def test_the_repair_prompt_splices_the_frozen_node_spec():
    """⚠️ SPLICED, NOT COPIED. The contract has one definition, so a change to
    `generate/prompts/modifier.md` reaches repairs automatically."""
    text = L.REPAIR_PROMPT.read_text(encoding="utf-8")
    assert "{{NODE_PROMPT}}" in text
    body = text.replace("{{NODE_PROMPT}}", "")
    assert "The one hard rule" not in body, "the contract was copied, not spliced"


def test_the_repair_prompt_advertises_the_bases_ports():
    """⚠️ THE FIX. `generate/prompts/modifier.md` never mentions these, so NO
    node in the library could read them — the arbitration failure that motivated
    the layered pipeline is a PROMPT OMISSION, not a model failure."""
    text = L.REPAIR_PROMPT.read_text(encoding="utf-8")
    for port in ("nav_heading", "nav_stuck", "nav_progress"):
        assert port in text, port
    assert "nav_heading" not in (
        __import__("pathlib").Path("generate/prompts/modifier.md")
        .read_text(encoding="utf-8")), (
        "modifier.md now mentions nav_heading — this test records that it did "
        "NOT, which is the finding; update the finding, do not delete the test")


def test_the_advertised_port_types_match_the_base():
    """⚠️ A claim in a prompt the code does not check is the failure mode
    chunk 56 hit twice. A wrong PortType here makes every repaired node fail to
    build."""
    from generate import base as basemod
    declared = {p.name: p.type.name for p in basemod.load().node_cls.PORTS}
    text = L.REPAIR_PROMPT.read_text(encoding="utf-8")
    found = re.findall(r"\| `(nav_\w+)` \| `(\w+)` \|", text)
    assert found, "the port table is not being parsed; the test is vacuous"
    for name, typ in found:
        assert declared.get(name) == typ, (name, typ, declared.get(name))


def test_the_repair_prompt_says_the_card_is_not_in_question():
    """The card is the independent variable and is reused byte for byte."""
    text = L.REPAIR_PROMPT.read_text(encoding="utf-8").lower()
    assert "the card is not in question" in text


# --------------------------------------------------------------------------- #
# budget, and writing as it goes


def test_the_budget_is_declared_and_fixed():
    """⚠️ PRE-REGISTRATION MATERIAL. Without a fixed, identical budget, B−A
    measures how long we kept going."""
    assert L.DEFAULT_ROUNDS == 3


def test_a_dry_run_calls_nothing_and_still_classifies(tmp_path, monkeypatch):
    """⚠️ `WORK` is redirected. Without it this test writes into the REAL
    `out/repair/<node>/loop.json` and clobbers a live run's record — which it
    did, mid-run, and the stale file then read as the live result."""
    monkeypatch.setattr(L, "WORK", tmp_path)
    r = L.run(NODE, rounds=3, dry_run=True)
    assert len(r.rounds) == 1
    assert r.rounds[0].verdict_label == "DEGENERATE"
    assert r.rounds[0].action == "", "a dry run must not reach the diagnosis"


def test_every_round_is_written_before_the_next_one_starts(tmp_path, monkeypatch):
    """⚠️ The sweep rehearsal's own worst bug: five episodes exited 0, wrote
    nothing, and were recorded as ok.

    ⚠️ This used `dry_run=True` for convenience, which stopped testing anything
    once a dry run correctly stopped writing. It now drives a REAL round that
    terminates on its verdict, so no model is called and the write path under
    test is the one a real run takes."""
    from repair.verdict import Label, Verdict

    monkeypatch.setattr(L, "WORK", tmp_path)
    monkeypatch.setattr(L, "_measure",
                        lambda nid, tier="maze", **kw: _measurement(nid))
    monkeypatch.setattr("repair.verdict.classify",
                        lambda m, c: Verdict(node_id=NODE, label=Label.HELPED,
                                             reason="cleared its claim", n=5,
                                             tier="maze"))

    L.run(NODE, rounds=1)

    written = json.loads((tmp_path / NODE / "loop.json").read_text(encoding="utf-8"))
    assert written["rounds"], "the round was not written"
    assert written["rounds"][0]["verdict_label"] == "HELPED"
    assert written["exit"] == "HELPED"


def test_not_transferable_cannot_be_reached_on_round_one(monkeypatch):
    """⚠️ A loop that can give up immediately will report "the paper does not
    transfer" for a node it never tried to fix. It requires at least one real
    repair attempt first."""
    import repair.diagnose as DG
    from repair.verdict import Claim

    m = _measurement()
    monkeypatch.setattr(L, "_measure", lambda *a, **k: m)
    monkeypatch.setattr("repair.verdict.claim_for",
                        lambda *a, **k: Claim("goal_reached", 0.1, "higher", "card"))

    class _Bundle:
        handle = "node-x"

        def render(self):
            return "b"

        def leaks(self):
            return ()

    monkeypatch.setattr("repair.evidence.bundle", lambda *a, **k: _Bundle())
    monkeypatch.setattr("repair.evidence.worst_case_seed", lambda *a, **k: 2004)
    monkeypatch.setattr(DG, "diagnose", lambda *a, **k: DG.Diagnosis(
        handle="node-x", action=DG.Action.NOT_TRANSFERABLE, cause="c",
        action_reason="r", confidence="high"))
    monkeypatch.setattr(L, "regenerate",
                        lambda *a, **k: ("n__r01", L.round_dir("n", 1)))

    rec, done, _ = L.run_round("n", 1, 4, seen=set())
    assert done is not L.Exit.NOT_TRANSFERABLE
    assert "REFUSED" in rec.outcome


def test_the_task_is_read_from_metadata_not_parsed_from_the_stem():
    """⚠️ THE FIRST LIVE RUN DIED ON THIS. The stem is
    `<arm>__<CARD id>__<plan_hash>` and its middle segment is the CARD, not the
    task — `b__path_integration_ants__44d12…` has task `navigate_to_goal`.
    Parsing the stem produced:

        no task 'path_integration_ants'. Known: ['navigate_to_goal', ...]

    Two different names that both look like a task is a trap the filename
    invites."""
    import json as _json
    from pathlib import Path as _P
    from generate.generator import STAGING, TASKS
    from repair.evidence import _stem_for

    stem = _stem_for(NODE)
    middle = stem.split("__")[1]
    meta = _json.loads((_P(STAGING) / f"{stem}.json").read_text(encoding="utf-8"))

    assert middle == meta["card_id"], "the middle segment is the card id"
    assert middle not in TASKS, "and it is NOT a task — that was the bug"
    assert meta["task"] in TASKS
    assert meta["task"] != middle


def test_a_repaired_node_is_not_looked_up_in_the_growth_csv(monkeypatch):
    """⚠️ THE SECOND LIVE RUN DIED ON THIS, after a SUCCESSFUL regeneration:

        KeyError: 'gen_b_…__r01 has no rows in …/maze-worlds.csv'

    `load_measurement` reads a committed CSV; a repaired node has no rows in it
    and never will. It has to be rolled out."""
    called = {}
    # ⚠️ `entry_for` is patched too, and it was not before. This test used to
    # pass only BECAUSE of the shim bug: `entry_for` on a round that does not
    # exist returned `ContractPorts` instead of raising, so the stub was never
    # missed. A test that a bug was holding up is worth saying out loud.
    monkeypatch.setattr(L, "entry_for", lambda nid: ("entry", nid))
    monkeypatch.setattr(L, "load_round", lambda p, n: ("entry", p, n))
    monkeypatch.setattr(L, "measure_live",
                        lambda e, nid, seeds, tier="maze", drift=None:
                        called.setdefault("live", (nid, list(seeds), drift)))
    L._measure("gen_b_abc__r01")
    assert called["live"][0] == "gen_b_abc__r01"
    assert called["live"][1] == list(L.GROWTH_SEEDS), (
        "a repaired node must be scored on the SAME worlds as its predecessor")


def test_an_unrepaired_node_still_reads_the_committed_csv():
    m = L._measure(NODE)
    assert m.n == 20 and m.node_id == NODE


def test_a_repaired_candidate_keeps_its_parents_provenance():
    """⚠️ THE THIRD LIVE RUN DIED ON THIS, after a successful regeneration AND
    a successful re-measurement:

        PROVENANCE_INCOMPLETE: temperature was not recorded

    A hand-built metadata dict loses required fields one at a time. Inheriting
    the parent's keeps `temperature`, `sampling_note`, `card_id`, `ref_id`,
    `treatment` and `origin` correct by construction — and they ARE still
    correct: a repair re-makes the NODE call, and the card call that produced
    them is untouched."""
    import json as _json
    from pathlib import Path as _P
    from generate.generator import STAGING
    from repair.evidence import _stem_for

    parent = _json.loads(
        (_P(STAGING) / f"{_stem_for(NODE)}.json").read_text(encoding="utf-8"))
    for field in ("temperature", "sampling_note", "model_id", "prompt_hash",
                  "card_id", "treatment", "condition", "task"):
        assert field in parent, f"{field} is not on the parent to inherit"

    src = inspect.getsource(L.regenerate)
    assert "dict(meta_parent)" in src, "metadata must be inherited, not rebuilt"
    for overridden in ("node_id", "parent_node_id", "repair_round",
                       "prompt_hash", "model_id"):
        assert f'"{overridden}"' in src


def test_one_resolver_handles_library_members_and_repair_rounds():
    """⚠️ THE FOURTH LIVE FAILURE, AND THE SAME ROOT CAUSE AS THE SECOND.

    Four separate components each assumed a node id names a member of the
    committed library. A repaired node never is one:

        run 2: _measure          -> KeyError: no rows in maze-worlds.csv
        run 4: evidence.case_block -> KeyError: no node '…__r01' in the library

    Patching the fourth call site would only have waited for the fifth. Every
    component that needs to RUN a node now goes through one resolver."""
    from repair import evidence as E

    assert E.entry_for(NODE) is not None
    # ⚠️ Scoped to case_block. `entry_for` itself calls `load_library().get`,
    # which is the point — what must not happen is another component doing it.
    assert "load_library().get(node_id)" not in inspect.getsource(E.case_block), (
        "case_block is resolving nodes itself again instead of using entry_for")
    assert L.entry_for is E.entry_for, "the loop must share the resolver"


def test_a_repaired_node_shares_its_parents_stem():
    """The card is not regenerated, so `plan_hash` and the stem are unchanged."""
    from repair.evidence import _stem_for
    assert _stem_for(NODE + "__r01") == _stem_for(NODE)


# --------------------------------------------------------------------------- #
# ⚠️ THE FIFTH LIVE FAILURE — the lineage root


def test_a_round_is_named_off_the_lineage_root_not_off_its_parent():
    """⚠️ THE FIRST FULL 3-ROUND RUN DIED HERE, after four model calls.

    Round 2 is handed round 1's id, so naming the new node
    `f"{node_id}__r{number:02d}"` compounds: `gen_b_…__r01__r02`. Round 3 then
    measures it, `entry_for` partitions on the FIRST `__r`, and the run ends in

        ValueError: invalid literal for int() with base 10: '01__r02'

    A lineage has one root and `repair_round` already carries the depth."""
    from repair.evidence import lineage_root

    assert lineage_root(NODE) == NODE
    assert lineage_root(NODE + "__r01") == NODE
    assert lineage_root(NODE + "__r01__r02") == NODE, (
        "a compounded id must still resolve to one root")


def test_every_round_of_a_lineage_stages_under_one_root():
    """The round DIRECTORY has the same rule as the id — otherwise round 2 of
    `…__r01` lands in a second top-level folder and the lineage is split across
    two places on disk. That happened in the same run."""
    assert L.round_dir(L.lineage_root(NODE + "__r01"), 2) == L.round_dir(NODE, 2)
    assert L.round_dir(NODE, 2).parent.name == NODE


def test_entry_for_rejects_a_compounded_id_readably():
    """The root cause is fixed in `regenerate`; this is what says so out loud
    if anything reintroduces it. ⚠️ A naming bug must not surface as an
    `int()` traceback three rounds into a paid run."""
    from repair import evidence as E

    with pytest.raises(ValueError, match="not a repair-round id"):
        E.entry_for(NODE + "__r01__rXX")


def test_a_finished_round_resumes_without_a_model_call(tmp_path, monkeypatch):
    """⚠️ Done-when: "a killed run resumes without repeating a model call."

    And the sweep rehearsal's own worst bug applies — five episodes exited 0,
    wrote nothing and were recorded as `ok` — so the marker is the RESULT FILE,
    written last, never the directory or the `.py`."""
    from repair import evidence as E

    monkeypatch.setattr(E, "WORK", tmp_path)

    def _boom(*a, **k):                      # any model call is a failure here
        raise AssertionError("regenerate made a model call on a finished round")

    monkeypatch.setattr("generate.backends.ClaudeCliBackend", _boom)

    stem = E._stem_for(NODE)
    out = E.round_dir(NODE, 1)
    out.mkdir(parents=True)
    (out / f"{stem}.py").write_text("# staged\n")

    # no result file yet -> an INTERRUPTED call, which must NOT be resumed
    with pytest.raises(AssertionError):
        L.regenerate(NODE, "because", 1)

    (out / f"{stem}.result.json").write_text("{}")
    new_id, staged = L.regenerate(NODE, "because", 1)
    assert new_id == f"{NODE}__r01"
    assert staged == out


def test_the_resolver_returns_the_REPAIR_not_the_port_shim(staged_round):
    """⚠️ THE FIFTH LIVE FAILURE, AND THE ONLY ONE THAT NEVER RAISED.

    `ContractPorts` is always in a loaded library — the harness's own port
    shim, exempt by design — and it sorts FIRST. `entries[0]` therefore
    returned the shim for every repaired node, so the loop measured the shim
    instead of the repair. The shim does nothing, so:

        fire 0.0% -> INERT -> two rounds with the IDENTICAL signature
        -> the oscillation guard -> UNREPAIRED

    A full 3-round run, every round green, every verdict after round 1 an
    artefact. Nothing in the loop could have noticed: an artefact that looks
    exactly like a plausible result is the failure mode this test exists for."""
    from compose.library import CONTRACT_PORTS_ID
    from repair.evidence import entry_for

    e = entry_for(staged_round)
    assert e.node_cls.__name__ != "ContractPorts"
    assert getattr(e, "node_id", None) != CONTRACT_PORTS_ID
    # the repair is a real node with a precondition of its own
    assert hasattr(e.node_cls, "precondition")


def test_an_ambiguous_round_raises_rather_than_guessing():
    """A resolver that GUESSES which entry was meant will eventually guess
    wrong silently — which is exactly what just happened. A repair round is one
    node call and must stage exactly one node."""
    import inspect as _i
    from repair import evidence as E
    src = _i.getsource(E.entry_for)
    assert "must stage exactly one" in src, (
        "entry_for must refuse an ambiguous round instead of taking entries[0]")


def test_a_dry_run_never_writes_over_a_real_record(tmp_path, monkeypatch):
    """⚠️ `--dry-run` is documented as "the plan, calls nothing", and a reader
    reasonably takes that to mean it changes nothing either. It did not:
    `_write` clobbered `loop.json`, so running the advertised dry-run command
    DESTROYED the completed run's record.

    Found 2026-09-17 by running the documented command immediately after the
    run it destroyed. Calling nothing and writing nothing are different
    promises and the docstring only made one of them."""
    from repair import evidence as E

    monkeypatch.setattr(E, "WORK", tmp_path)
    monkeypatch.setattr(L, "WORK", tmp_path)
    monkeypatch.setattr(L, "_measure",
                        lambda nid, tier="maze", **kw: _measurement(nid))
    monkeypatch.setattr(L, "signature_of", lambda m: "sig")

    real = tmp_path / NODE
    real.mkdir(parents=True)
    (real / "loop.json").write_text('{"exit": "UNREPAIRED", "rounds": [1, 2, 3]}')

    L.run(NODE, rounds=3, dry_run=True)

    kept = json.loads((real / "loop.json").read_text())
    assert kept["exit"] == "UNREPAIRED", "a dry run overwrote a completed record"
    assert kept["rounds"] == [1, 2, 3]


# --------------------------------------------------------------------------- #
# ⚠️ THE SEVENTH BUG — a verdict with no data behind it


def test_measure_live_does_not_ship_an_empty_metrics_dict():
    """⚠️ `metrics={}` made three of the five verdicts UNREACHABLE for anything
    the loop repaired.

    `pair_of(PRIMARY)` returned `(None, None)`, so `primary_worse` was always
    False, and `DEGENERATE`, `REGRESSED` and `HELPED` all depend on metrics
    that were not there. A repaired node could only ever come back `INERT` or
    `ACCEPT` — which is the real reason three exits were never produced, not
    the round budget."""
    import inspect as _i
    # ⚠️ Comments stripped first: the fix's own comment QUOTES the old code, so
    # a plain substring check would fail on the explanation of the bug.
    src = "\n".join(line.split("#")[0]
                    for line in _i.getsource(L.measure_live).splitlines())
    assert "metrics={}" not in src, "measure_live is shipping an empty metrics dict again"
    assert "aggregate_metrics" in src


def test_aggregate_metrics_reuses_measure_growths_arithmetic():
    """A repaired node and a committed one must be scored by the same
    arithmetic or their verdicts are not comparable — and the verdict rule is
    what decides whether a repair is kept."""
    from repair.evidence import _growth_module, aggregate_metrics

    g = _growth_module()
    base = {1: {"goal_reached": 1.0, "path_length": 10.0},
            2: {"goal_reached": 1.0, "path_length": 20.0}}
    pair = {1: {"goal_reached": 0.0, "path_length": 5.0},
            2: {"goal_reached": 0.0, "path_length": 5.0}}

    m = aggregate_metrics(base, pair)
    assert m["goal_reached"] == (1.0, 0.0)
    assert m["path_length"] == (15.0, 5.0)
    # every metric measure_growth reports is present, so none reads as absent
    assert {k for k, _ in g.REPORTED_METRICS} <= set(m)


def test_a_none_valued_metric_survives_aggregation_as_none():
    """`time_to_goal` is None when the goal was never reached. Averaging only
    the episodes where it happened is deliberate — see `measure_growth._mean`
    — and a metric that never happened must come back None, not 0.0, or a
    node that never arrives would look instant."""
    from repair.evidence import aggregate_metrics

    m = aggregate_metrics({1: {"time_to_goal": None}}, {1: {"time_to_goal": None}})
    assert m["time_to_goal"] == (None, None)


def test_a_node_that_loses_worlds_is_not_called_harmless(monkeypatch):
    """⚠️ THE CASE THAT EXPOSED IT. `gen_b_…__r01` was classified

        ACCEPT — "acted, harmed nothing measurable, and did not clear its
                  own claim"

    while losing FOUR of the ten worlds the base solves. The verdict was not
    wrong about the data; it had no data."""
    from repair.verdict import Claim, Label, Measurement, WorldOutcome, classify

    worlds = tuple(
        WorldOutcome(seed=2000 + i, scenario_id=f"s{i}", base_reached=True,
                     pair_reached=(i >= 4), outcome=("broke" if i < 4 else "both_ok"),
                     fire=0.2, changed=0.13)
        for i in range(10))
    m = Measurement(node_id="n__r01", tier="maze", worlds=worlds,
                    metrics={"goal_reached": (1.0, 0.6), "path_length": (25.0, 18.0)},
                    fire=0.2, changed=0.13)

    v = classify(m, Claim("goal_reached", 0.1, "higher", "card"))
    assert v.label is not Label.ACCEPT, (
        "a node that loses 4 of 10 worlds the base wins is not harmless")
    assert v.label in (Label.DEGENERATE, Label.REGRESSED)


def test_a_repaired_node_inherits_its_parents_claim():
    """⚠️ WITHOUT THIS, `HELPED` IS UNREACHABLE FOR EVERYTHING THE LOOP FIXES.

    `_stem_for` split the raw id on "_", so `gen_b_…__r01` gave `digest="r01"`,
    matched nothing in `generate/staging/`, and `claim_for` returned
    `Claim(metric=None, source="missing")`. `HELPED` and `MISJUDGED` both
    require a claim — so the only verdict that terminates the loop on SUCCESS
    could never be produced for a repaired node.

    Inheriting is correct by construction, not a convenience: the card is not
    regenerated and is reused byte for byte, so a repair's claim IS its
    parent's claim."""
    from repair.verdict import claim_for

    parent = claim_for(NODE)
    assert parent.metric == "goal_reached" and parent.source == "card"
    for suffix in ("__r01", "__r02"):
        child = claim_for(NODE + suffix)
        assert child == parent, f"{suffix} did not inherit the card's claim"


# ---------------------------------------------------------------------------
# the drift axis — 2026-09-18


def test_round_numbering_continues_from_a_repair_round():
    """⚠️ Re-entering a lineage must not restage over its only valid round.

    `run("…__r01")` means "carry on from the node round 1 produced". Without
    this, `regenerate` names the next node `…__r01` again and stages it into
    `round-01`, destroying the round the re-run exists to build on.
    """
    from repair.loop import first_round_number

    assert first_round_number("gen_b_44d12df4d25b437f") == 1
    assert first_round_number("gen_b_44d12df4d25b437f__r01") == 2
    assert first_round_number("gen_b_44d12df4d25b437f__r09") == 10
    with pytest.raises(ValueError):
        first_round_number("gen_b_44d12df4d25b437f__rXX")


def test_a_drifted_measurement_never_reads_the_undrifted_csv(monkeypatch):
    """⚠️ THE MISTAKE THIS EPIC EXISTS BECAUSE OF, ONE LEVEL UP.

    `core.worldgen.to_scenario()` never sets `drift`, so every row in the
    committed `<tier>-worlds.csv` was measured with a PERFECT POSE ESTIMATE.
    `_measure` short-circuits to that CSV for any id without `__r` — so asking
    for drift on the ORIGINAL node would return the clean number wearing a
    drifted label, which is precisely "a node judged on a suite that does not
    contain the condition its card names".
    """
    from repair import loop

    called = {}

    def fake_load_measurement(node_id, **kw):
        called["csv"] = True
        raise AssertionError("the undrifted CSV was read for a drifted run")

    def fake_measure_live(entry, node_id, seeds, tier="maze", drift=None):
        called["live"] = drift
        return "measurement"

    monkeypatch.setattr("repair.verdict.load_measurement", fake_load_measurement)
    monkeypatch.setattr(loop, "measure_live", fake_measure_live)
    monkeypatch.setattr(loop, "entry_for", lambda node_id: "entry")

    # the ORIGINAL node — no `__r` — is the case that would have slipped through
    assert loop._measure("gen_b_44d12df4d25b437f", drift=0.10) == "measurement"
    assert called == {"live": 0.10}


def test_an_undrifted_measurement_still_reads_the_csv(monkeypatch):
    """The short-circuit is kept where it is valid — it is what makes round 1
    of a clean run cost nothing."""
    from repair import loop

    monkeypatch.setattr("repair.verdict.load_measurement",
                        lambda node_id, **kw: "from-csv")
    assert loop._measure("gen_b_44d12df4d25b437f") == "from-csv"


def test_a_lineage_may_not_mix_axes(tmp_path, monkeypatch):
    """⚠️ `round_dir` keys off the node id ALONE, so a drifted run and a clean
    one would interleave in one directory under one `loop.json` with nothing
    recording which round saw which hazard. Changing the axis is deliberate and
    means archiving the previous run first."""
    from repair import loop

    monkeypatch.setattr(loop, "WORK", tmp_path)
    root = tmp_path / "gen_b_44d12df4d25b437f"
    root.mkdir()
    (root / "loop.json").write_text(
        json.dumps({"node_id": "gen_b_44d12df4d25b437f", "drift": None,
                    "rounds": []}), encoding="utf-8")

    found, previous = loop.recorded_drift("gen_b_44d12df4d25b437f__r01")
    assert found and previous is None

    with pytest.raises(ValueError, match="may not mix"):
        loop.run("gen_b_44d12df4d25b437f__r01", drift=0.10, dry_run=True)


def test_the_axis_is_recorded_per_round_not_only_per_run():
    """Round 1 of this lineage was measured clean and later rounds need not be,
    so a run-level field alone would mislabel it."""
    from repair.loop import Round

    assert Round(number=1, node_id="x").drift is None
    assert Round(number=2, node_id="x", drift=0.10).as_dict()["drift"] == 0.10


def test_a_re_entered_lineage_records_where_it_picked_up():
    """⚠️ A run entered at `…__r01` numbers from 2, so its `rounds` list does
    not describe round 1 — while `round-01/` is still in the lineage directory.
    Without this the live record silently omits the round everything after it
    was built on."""
    from repair.loop import LoopResult

    assert LoopResult("n", None, "", continues_from=None).continues_from is None
    r = LoopResult("gen_b_abc__r01", None, "", continues_from="gen_b_abc__r01")
    assert r.continues_from == "gen_b_abc__r01"


def test_a_probe_round_writes_its_whole_search_down(tmp_path, monkeypatch):
    """⚠️ `UNTESTABLE` IS DEFINED AS "the search is recorded". The first real
    one recorded `note[:80]`, which truncated mid-word and lost all six trials
    — leaving a terminal verdict with nothing to check it against, in an epic
    that has already shipped two verdicts that were measurement artefacts."""
    from repair import loop as L
    from repair import probe as P

    monkeypatch.setattr(L, "WORK", tmp_path)
    t = P.Trial(P.Candidate("maze", 4000, drift=0.05), False,
                "base cannot solve this world even without the axis",
                0.9, 0.9, False, False, 1.0)
    result = P.ProbeResult("n__r01", None, (t,), ("drift",), 1.0, "UNTESTABLE.")

    path = L._write_probe_log("n__r01", 2, result)

    assert path.name == "probe-round-02.log"
    body = path.read_text(encoding="utf-8")
    assert "gen-maze-04000+drift0.05" in body, "the world that was tried"
    assert "even without the axis" in body, "and WHY it was passed over"
