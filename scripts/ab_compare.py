"""
scripts/ab_compare.py

**A vs B, on the same five papers.** The paired comparison chunk 39 exists for.

    $P scripts/ab_compare.py
    $P scripts/ab_compare.py --seeds 0,1,2
    $P scripts/ab_compare.py --json
    $P scripts/ab_compare.py --out out/evidence/nodes/ab-comparison

Condition A is handed a paper's CITATION and nothing else; condition B is handed
the same citation AND the human-extracted card. Every other input is identical,
so a per-card difference is what EXTRACTION bought. Five cards now carry both
arms, which is what makes this a paired design rather than two samples.

⚠️ READ THE TRAP BEFORE READING THE TABLE
------------------------------------------
**A produces whole navigators; B produces single mechanisms.** That is not a
finding about quality, it is a finding about what each prompt licenses. The
card's Derivation pins B to one mechanism and B implements exactly that; a
citation alone leaves the model free to write a complete controller wearing the
paper's name. So the SOLO framing measures BREADTH AS MUCH AS MERIT, and A wins
it by construction.

`A's advantage is partly A doing more than its source licenses`, and that cannot
be checked, because a citation node can be diffed against nothing
(`generate/FINDINGS.md` §1).

Two framings are therefore printed and **neither may be quoted alone**:

  solo     Sequence[ContractPorts, NODE]                    — the admission tree.
           Answers "is this node a complete controller?"
  carried  Sequence[ContractPorts, Fallback[NODE, AvoidObstacles]]
           An identical fixed carrier in both arms, so the carrier cancels.
           Answers "what does this node add to a fixed baseline?"

⚠️ **Carried is fairer and still not fair.** Every generated node is an Action
whose precondition usually holds, so it returns RUNNING, the `Fallback`
short-circuits, and the carrier never ticks — the node REPLACES the carrier
rather than augmenting it. The firing rates are printed beside every number for
exactly this reason: `carrier_drove` near zero means the comparison degenerated
back into solo. The genuinely fair framing needs chunk 40 — **no generated node
declares a single IN port**, so B's mechanisms cannot be wired as modifiers to
anything.

⚠️ **NOT A REPORTABLE NUMBER, AND NOT A G2 NUMBER.** It runs
`core/tests/kinematic.py`: real lab42 geometry, real sensing, real drift, real
scoring — and an idealised unicycle with the frozen locomotion policy absent.
Physics confirmation is chunk 36. `core/stats.py:decide()` is G2's pre-registered
rule over the sealed split and is deliberately NOT called here; five paired card
differences are reported as five signed differences, and **a bootstrap CI over
n=5 cards must not be computed and presented as a result.**

⚠️ **THE DEV SCENARIOS ONLY.** This measurement feeds back into which nodes get
built, so running it against the sealed split is precisely the leak G3 exists to
prevent. There is no flag to point it at the held-out split.

SIM-FREE.
"""

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))



def _parse_args(argv=None):
    """Parsed inside `main`, NOT at module level.

    The repo's convention puts argparse above the imports so a malformed flag
    is rejected before `SimulationApp` spends 20 s booting. That reason does not
    apply here — this script is SIM-FREE and every heavy import is already lazy
    — and module-level parsing would make the module unimportable from a test,
    which matters more: the pairing rule below is the thing worth pinning.
    """
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--seeds", default="0",
                        help="comma-separated seeds (default: 0)")
    parser.add_argument("--json", action="store_true",
                        help="machine-readable, to stdout")
    parser.add_argument("--out", default=None, metavar="DIR",
                        help="also write table.txt and comparison.json into DIR")
    return parser.parse_args(argv)


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# The ARCHIVED one-call records. This script's committed evidence, its paired
# table and every number in out/evidence/nodes/ab-comparison/README.md are about
# those, which is what DEFAULT_PIPELINE below already said; after the 2026-09-08
# archive move it is also where they live. Pass `staging=` to point it at
# another set — but a pairing across two pipelines is not a paired design.
STAGING = REPO_ROOT / "generate" / "archive" / "v1-single-call" / "staging"

# The two arms this script pairs. C and D are not in it: D is unpaired (it is
# handed no reference at all, so there is no card to pair ON) and C is not built.
ARMS = ("a", "b")

# The full metric vector, reported together. A single metric is how "B loses"
# and "B travels zero metres because it is a speed limiter with nothing to
# limit" become indistinguishable.
METRICS = ("goal_reached", "spl", "collisions", "near_misses",
           "stuck_events", "time_to_goal", "path_length")

# Metrics where a smaller number is the better one. Only used to orient the
# printed sign, never to compute a score — there is no composite score here on
# purpose, because weighting the vector is a decision nobody has made.
LOWER_IS_BETTER = ("collisions", "near_misses", "stuck_events",
                   "time_to_goal", "path_length")

CARRIER_ID = "toy_avoid_obstacles"


# Which pipeline this comparison is ABOUT. Chunk 39's paired A/B table, the
# committed evidence in out/evidence/nodes/ab-comparison/, and every number in
# its README were computed over the ONE-CALL records — so that is the default,
# stated rather than inherited from whatever happens to be on disk. Chunk 42
# added a second pipeline; pooling the two would pair a v1 A node against a v2 B
# node and call the difference an arm effect.
DEFAULT_PIPELINE = "v1-single-call"


def staged_records(staging=None, cards=None, pipeline_version=DEFAULT_PIPELINE):
    """Every staging record that counts, with the SAME exclusions `pass_rate` uses.

    Duplicating the rule rather than importing it is deliberate: `pass_rate`
    returns counts, not records. The four exclusions are quoted from it so a
    reader can diff them — a fixture card, a superseded generation, a retired
    condition definition and a record from another generation pipeline are each
    excluded for a reason that applies identically here, and a pairing built
    over a different set than the pass rate would describe a different library
    than the number beside it.

    `pipeline_version=None` disables the pipeline filter. There is no honest use
    for that in a paired comparison; it exists so a caller can COUNT what is on
    disk without being told a story about it.
    """
    staging = Path(staging or STAGING)
    if cards is None:
        from cards.store import CardStore
        cards = CardStore.load()

    by_node_stem = {}
    for path in staging.glob("*.json"):
        if path.name.endswith(".result.json"):
            continue
        meta = json.loads(path.read_text(encoding="utf-8"))
        by_node_stem[meta["node_id"]] = path.name[: -len(".json")]

    out, excluded = [], []
    for path in sorted(staging.glob("*.result.json")):
        result = json.loads(path.read_text(encoding="utf-8"))
        meta = result["metadata"]
        ref_id = meta.get("card_id") or meta.get("citation_id")
        # Generated from a card that is not in the corpus: a test fixture.
        if ref_id and ref_id not in cards:
            excluded.append((meta.get("node_id"), "fixture"))
            continue
        # The same prompt run again; counting both is one mechanism twice.
        if meta.get("superseded_by"):
            excluded.append((meta.get("node_id"), "superseded"))
            continue
        # Generated under a condition definition that no longer exists.
        if meta.get("retired_reason"):
            excluded.append((meta.get("node_id"), "retired"))
            continue
        # Generated by a DIFFERENT PIPELINE. A record with no version predates
        # the field, which makes it v1 — the absence is the answer, not missing
        # data. `generate/admit.py:pass_rate` skips the same records on the same
        # rule, which is what keeps this pairing and that denominator equal.
        if (pipeline_version is not None
                and meta.get("pipeline_version", "v1-single-call")
                != pipeline_version):
            excluded.append((meta.get("node_id"), "other pipeline"))
            continue
        # Chunk 37's verdict where one exists, the original otherwise — the
        # same order `pass_rate` and `compose.library.admitted_stems` read in.
        if not result.get("admitted_v2", result["admitted"]):
            excluded.append((meta.get("node_id"), "not admitted"))
            continue
        node_id = meta["node_id"]
        out.append({
            "node_id": node_id,
            "condition": meta.get("condition", "?"),
            "ref_id": ref_id,
            "is_card": bool(meta.get("card_id")),
            "stem": by_node_stem[node_id],
            "meta": meta,
        })
    return out, excluded


def pairs(records):
    """card id -> {"a": record, "b": record}, for cards carrying BOTH arms.

    A card with only one arm is dropped rather than half-reported: the whole
    value of this design is that the two nodes come from the same paper, and an
    unpaired row silently turns a paired comparison into an unpaired one.
    """
    by_ref = {}
    for rec in records:
        if rec["condition"] not in ARMS or not rec["ref_id"]:
            continue
        by_ref.setdefault(rec["ref_id"], {})[rec["condition"]] = rec
    return {ref: arms for ref, arms in sorted(by_ref.items())
            if all(arm in arms for arm in ARMS)}


def load_node(rec, staging=None, cards=None):
    """(node_cls, metadata) for one staged record, rebuilt exactly as the library does."""
    from generate import admit as admit_mod

    staging = Path(staging or STAGING)
    if cards is None:
        from cards.store import CardStore
        cards = CardStore.load()

    source = (staging / f"{rec['stem']}.py").read_text(encoding="utf-8")
    node_cls, _, _, _ = admit_mod.load_candidate(source, rec["node_id"])
    # `card` for B, `citation_of` for A — never both, and the distinction is
    # what keeps a citation node from inheriting the span and the confidence
    # that only reading the paper produces.
    card = citation_of = None
    if rec["ref_id"]:
        if rec["is_card"]:
            card = cards.get(rec["ref_id"])
        else:
            citation_of = cards.get(rec["ref_id"])
    metadata = admit_mod._node_metadata(rec["meta"], rec["node_id"],
                                        card, citation_of=citation_of)
    return node_cls, metadata


def solo_factory(node_cls, metadata):
    """The admission tree, unchanged. `generate.admit.build_tree` IS the framing."""
    from generate.admit import build_tree
    return lambda: build_tree(node_cls, metadata)


def carried_factory(node_cls, metadata):
    """The node in front of a fixed carrier that can actually navigate.

    `AvoidObstacles` is a HANDWRITTEN FIXTURE and NOT NAVIGATION for benchmark
    purposes (#3) — it can never appear in a benchmark condition. It is here
    because it is identical in both arms and therefore cancels out of a paired
    difference, which is the only property this framing needs from it.
    """
    from generate.admit import ContractPorts
    from nodes.contract import Kind
    from nodes.toys import AvoidObstacles, toy_metadata
    from nodes.tree import Fallback, Sequence, Tree

    def build():
        carrier = AvoidObstacles(toy_metadata(CARRIER_ID))
        if node_cls.KIND is Kind.DECORATOR:
            # A Decorator has no meaning without a child, so it governs the
            # carrier directly and the Fallback's second branch is a fresh
            # carrier it can decline to. Note this is a DIFFERENT framing from
            # the Action one and the two are not comparable to each other; it
            # is here so the script does not silently skip a kind.
            leaf = node_cls(metadata, AvoidObstacles(toy_metadata(CARRIER_ID)))
        elif node_cls.KIND is Kind.CONDITION:
            raise NotImplementedError(
                f"{metadata.node_id} is a Condition. A Condition returning "
                f"SUCCESS inside this Fallback would SUPPRESS the carrier "
                f"rather than gate it, which measures the opposite of what is "
                f"intended. The right framing for a Condition is a Sequence "
                f"gate, and choosing between the two is chunk 40's decision, "
                f"not this script's. No generated node is a Condition today.")
        else:
            leaf = node_cls(metadata)
        return Tree(Sequence(toy_metadata("ab_scaffold"), [
            ContractPorts(toy_metadata("ab_ports")),
            Fallback(toy_metadata("ab_carrier"), [leaf, carrier]),
        ]))
    return build


def carrier_only_factory():
    """The carrier with NO generated node in front of it.

    ⚠️ **THE REFERENCE ROW, AND WITHOUT IT THE ABLATION IS UNREADABLE.** Every
    leave-one-out delta below is `intact - ablated`, and the ablated tree IS
    this tree: a stand-in that always fails hands control straight to the
    carrier. So a delta of zero means "the same as this row", not "no effect",
    and a negative delta means the generated node made a handwritten fixture
    WORSE. Printing the reference makes both readings available instead of
    inferable.

    `AvoidObstacles` is scaffolding (#3) and can never be a benchmark condition.
    That is precisely what makes it a fair yardstick here: it is identical in
    both arms and neither arm's prompt has ever seen it.
    """
    from generate.admit import ContractPorts
    from nodes.toys import AvoidObstacles, toy_metadata
    from nodes.tree import Sequence, Tree

    return lambda: Tree(Sequence(toy_metadata("ab_scaffold"), [
        ContractPorts(toy_metadata("ab_ports")),
        AvoidObstacles(toy_metadata(CARRIER_ID)),
    ]))


def _aggregate(rows, node_id):
    """Mean of each metric over scenario x seed, plus the firing rates.

    `time_to_goal` is None on an episode that did not reach the goal, and it is
    averaged over the episodes that DID rather than treated as zero — a zero
    would make a controller that never arrives look infinitely fast.
    """
    agg = {}
    for metric in METRICS:
        values = [r["score"].get(metric) for r in rows]
        present = [float(v) for v in values if v is not None]
        agg[metric] = (sum(present) / len(present)) if present else None
        if metric == "time_to_goal":
            agg["time_to_goal_n"] = len(present)

    ticks = passed = drove = 0
    carrier_ticks = carrier_drove = 0
    for row in rows:
        seen = row["fired"].get(node_id, {})
        ticks += seen.get("ticks", 0)
        passed += seen.get("precondition_passed", 0)
        drove += seen.get("drove", 0)
        carrier = row["fired"].get(CARRIER_ID, {})
        carrier_ticks += carrier.get("ticks", 0)
        carrier_drove += carrier.get("drove", 0)

    agg["ticks"] = ticks
    agg["precondition_passed"] = passed
    agg["drove"] = drove
    # THE NUMBER THAT SEPARATES "did not help" FROM "never got the chance".
    agg["fired_fraction"] = (passed / ticks) if ticks else 0.0
    agg["drove_fraction"] = (drove / ticks) if ticks else 0.0
    agg["carrier_ticks"] = carrier_ticks
    agg["carrier_drove"] = carrier_drove
    # Near zero means the Fallback short-circuited every tick and the node
    # REPLACED the carrier instead of augmenting it — the framing degenerated.
    agg["carrier_drove_fraction"] = (carrier_drove / ticks) if ticks else 0.0
    agg["per_episode"] = [
        {"scenario": r["scenario"], "seed": r["seed"],
         **{m: r["score"].get(m) for m in METRICS}} for r in rows]
    return agg


def measure(rec, scenarios, seeds, staging=None, cards=None):
    """Both framings for one node, plus its ablation contribution when carried."""
    from harness.ablate import ablate, score_tree

    node_cls, metadata = load_node(rec, staging, cards)
    node_id = rec["node_id"]
    out = {"node_id": node_id, "class": node_cls.__name__,
           "kind": node_cls.KIND.name, "condition": rec["condition"],
           "ref_id": rec["ref_id"], "origin": metadata.origin.value,
           "ports": [f"{p.name}:{p.direction.name}" for p in node_cls.PORTS]}

    solo = solo_factory(node_cls, metadata)
    out["solo"] = _aggregate(score_tree(solo, scenarios, seeds), node_id)

    try:
        carried = carried_factory(node_cls, metadata)
        intact = score_tree(carried, scenarios, seeds)
        out["carried"] = _aggregate(intact, node_id)
        # Leave-one-out against the SAME carrier: what the node adds once
        # something else in the tree can already drive.
        #
        # `harness.ablate.contribution` is deliberately NOT called here, and the
        # reason is not speed. It re-scores the intact tree once per metric, so
        # the baseline behind the ablation would be a SECOND rollout of the same
        # tree rather than the rows printed in the carried table above. They
        # would agree today — the rollout is deterministic — and a table whose
        # two halves are computed from different runs is one refactor away from
        # disagreeing silently. `ablate()` itself, which is the load-bearing
        # part (replace, never delete), is used unchanged.
        ablated = score_tree(lambda: ablate(carried(), node_id),
                             scenarios, seeds)
        out["contribution"] = _contribution(intact, ablated, node_id)
    except NotImplementedError as exc:
        out["carried"] = None
        out["carried_skipped"] = str(exc)
    return out


def _contribution(intact, ablated, node_id):
    """Intact minus ablated, per metric, over the same scenario x seed cells.

    `never_fired_in` is carried beside the effect because "did not help" and
    "never got the chance" are different findings and the admission gate this
    replaced reported them as one number.
    """
    baseline = {(r["scenario"], r["seed"]): r for r in intact}
    out = {}
    for metric in ("goal_reached", "spl"):
        deltas, never_fired = [], 0
        for row in ablated:
            key = (row["scenario"], row["seed"])
            full = baseline[key]["score"].get(metric)
            gone = row["score"].get(metric)
            full = 0.0 if full is None else float(full)
            gone = 0.0 if gone is None else float(gone)
            deltas.append(full - gone)
            if baseline[key]["fired"].get(node_id, {}).get(
                    "precondition_passed", 0) == 0:
                never_fired += 1
        out[metric] = {
            "metric": metric,
            "mean_delta": sum(deltas) / len(deltas) if deltas else 0.0,
            "n": len(deltas),
            "helped_in": sum(1 for d in deltas if d > 0),
            "hurt_in": sum(1 for d in deltas if d < 0),
            "never_fired_in": never_fired,
            "deltas": deltas,
        }
    return out


def _diff(a_value, b_value):
    if a_value is None or b_value is None:
        return None
    return float(a_value) - float(b_value)


def build(seeds, staging=None, cards=None):
    from core.scenario import DEV_SCENARIOS

    scenarios = list(DEV_SCENARIOS)
    records, excluded = staged_records(staging, cards)
    paired = pairs(records)

    from harness.ablate import score_tree
    reference = _aggregate(
        score_tree(carrier_only_factory(), scenarios, seeds), CARRIER_ID)
    reference.pop("per_episode", None)

    cells = {}
    for ref_id, arms in paired.items():
        cells[ref_id] = {arm: measure(rec, scenarios, seeds, staging, cards)
                         for arm, rec in arms.items()}

    # THE PAIRED STATISTIC: five signed differences, A minus B, per framing.
    # Reported as five numbers and their signs. NOT bootstrapped: n=5 cards, and
    # a CI over five paired differences presented next to G2's language is how a
    # development measurement gets read as the pre-registered one.
    differences = {}
    for framing in ("solo", "carried"):
        differences[framing] = {}
        for ref_id, arms in cells.items():
            a, b = arms["a"].get(framing), arms["b"].get(framing)
            if a is None or b is None:
                differences[framing][ref_id] = None
                continue
            differences[framing][ref_id] = {
                m: _diff(a.get(m), b.get(m)) for m in METRICS}

    return {
        "generated": "scripts/ab_compare.py",
        "scenarios": [s.id for s in scenarios],
        "seeds": list(seeds),
        "split": "dev",
        "metrics": list(METRICS),
        "arms": {"a": "citation only", "b": "citation + extracted card"},
        "n_cards": len(paired),
        # The yardstick both arms are ablated back to. See carrier_only_factory.
        "carrier_only": reference,
        "excluded": [{"node_id": n, "why": w} for n, w in excluded],
        "cells": cells,
        "differences": differences,
        "caveats": [
            "kinematic rollout, not physics — NOT a reportable number",
            "dev scenarios only; the sealed split is never used here",
            "solo measures breadth as much as merit: A writes whole "
            "navigators, B writes single mechanisms",
            "carried degenerates wherever carrier_drove is ~0: the node "
            "replaced the carrier rather than augmenting it",
            "no generated node declares an IN port, so B's mechanisms cannot "
            "be wired as modifiers — chunk 40",
            "not a G2 number; core/stats.py:decide() is not called",
        ],
    }


def _fmt(value, width=7, places=3):
    if value is None:
        return f"{'--':>{width}}"
    return f"{value:>{width}.{places}f}"


def render(data):
    lines = []
    add = lines.append
    add("[ab_compare] A (citation only) vs B (citation + extracted card)")
    add(f"[ab_compare] scenarios={data['scenarios']}  seeds={data['seeds']}  "
        f"split={data['split']}  cards={data['n_cards']}")
    add("")
    add("  ⚠️ kinematic, dev scenarios only — NOT a reportable number, NOT a G2 number")
    add("  ⚠️ solo measures BREADTH as much as merit: A writes whole navigators,")
    add("     B writes single mechanisms. Read the docstring before the table.")
    add("")

    for framing in ("solo", "carried"):
        add(f"=== {framing} " + "=" * (68 - len(framing)))
        header = (f"  {'card':26} {'arm':3} {'node':30} {'reach':>6} {'spl':>7} "
                  f"{'coll':>5} {'near':>5} {'stuck':>6} {'path':>7} "
                  f"{'fired':>6} {'drove':>6} {'carry':>6}")
        add(header)
        for ref_id, arms in data["cells"].items():
            for arm in ARMS:
                cell = arms[arm]
                agg = cell.get(framing)
                if agg is None:
                    add(f"  {ref_id:26} {arm.upper():3} {cell['class']:30} "
                        f"  SKIPPED: {cell.get('carried_skipped','')[:40]}")
                    continue
                add(f"  {ref_id:26} {arm.upper():3} {cell['class']:30} "
                    f"{_fmt(agg['goal_reached'], 6, 2)} {_fmt(agg['spl'])} "
                    f"{_fmt(agg['collisions'], 5, 1)} {_fmt(agg['near_misses'], 5, 1)} "
                    f"{_fmt(agg['stuck_events'], 6, 1)} {_fmt(agg['path_length'], 7, 2)} "
                    f"{_fmt(agg['fired_fraction'], 6, 2)} "
                    f"{_fmt(agg['drove_fraction'], 6, 2)} "
                    f"{_fmt(agg['carrier_drove_fraction'], 6, 2)}")
            diff = data["differences"][framing].get(ref_id)
            if diff:
                add(f"  {'':26} {'A-B':>3} {'':30} "
                    f"{_fmt(diff['goal_reached'], 6, 2)} {_fmt(diff['spl'])} "
                    f"{_fmt(diff['collisions'], 5, 1)} {_fmt(diff['near_misses'], 5, 1)} "
                    f"{_fmt(diff['stuck_events'], 6, 1)} {_fmt(diff['path_length'], 7, 2)}")
            add("")

    add("=== the paired statistic: five signed differences, A minus B ===")
    add("  Reported as five numbers. NOT bootstrapped — n=5 cards, and a CI here")
    add("  would be read as the pre-registered G2 interval, which it is not.")
    for framing in ("solo", "carried"):
        add(f"\n  {framing}:")
        for metric in ("goal_reached", "spl"):
            values = [d[metric] for d in data["differences"][framing].values()
                      if d and d.get(metric) is not None]
            signs = "".join("+" if v > 0 else ("-" if v < 0 else "0")
                            for v in values)
            pretty = ", ".join(f"{v:+.3f}" for v in values)
            add(f"    {metric:14} [{pretty}]  signs={signs}  "
                f"A>B in {sum(1 for v in values if v > 0)}/{len(values)}")

    add("")
    add("=== the reference row: the carrier ALONE, no generated node ===")
    ref = data["carrier_only"]
    add("  Sequence[ContractPorts, AvoidObstacles] — a HANDWRITTEN FIXTURE (#3),")
    add("  identical in both arms, which neither prompt has ever seen.")
    add(f"  {'':30} {'reach':>6} {'spl':>7} {'coll':>5} {'near':>5} "
        f"{'stuck':>6} {'path':>7}")
    add(f"  {'carrier only':30} {_fmt(ref['goal_reached'], 6, 2)} "
        f"{_fmt(ref['spl'])} {_fmt(ref['collisions'], 5, 1)} "
        f"{_fmt(ref['near_misses'], 5, 1)} {_fmt(ref['stuck_events'], 6, 1)} "
        f"{_fmt(ref['path_length'], 7, 2)}")
    add("  ⚠️ This is also the tree every leave-one-out below is ablated BACK to,")
    add("     so a delta of 0.000 means 'no better than the fixture' and a")
    add("     NEGATIVE delta means the generated node made the fixture WORSE.")

    add("")
    add("=== does the node itself help, once something else can drive? ===")
    add("  Leave-one-out inside the CARRIED tree: the node swapped for a")
    add("  stand-in of the same kind that always fails, so the tree's shape and")
    add("  control flow are unchanged and only the node's content is removed.")
    add("  ⚠️ `fired` 0.00 means the node NEVER TRIGGERED — a zero effect there is")
    add("  'never got the chance', not 'did not help', and the row beside it is")
    add("  measuring the carrier.")
    add(f"  {'card':26} {'arm':3} {'node':30} {'d_reach':>8} {'d_spl':>8} "
        f"{'helped':>7} {'hurt':>5} {'never fired':>12}")
    for ref_id, arms in data["cells"].items():
        for arm in ARMS:
            cell = arms[arm]
            contrib = cell.get("contribution")
            if not contrib:
                continue
            reach, spl = contrib["goal_reached"], contrib["spl"]
            add(f"  {ref_id:26} {arm.upper():3} {cell['class']:30} "
                f"{reach['mean_delta']:>+8.3f} {spl['mean_delta']:>+8.3f} "
                f"{reach['helped_in']:>3}/{reach['n']:<3} {reach['hurt_in']:>5} "
                f"{reach['never_fired_in']:>7}/{reach['n']:<4}")
        add("")

    add("  caveats:")
    for caveat in data["caveats"]:
        add(f"    - {caveat}")
    return "\n".join(lines)


def main(argv=None):
    args = _parse_args(argv)
    seeds = tuple(int(s) for s in args.seeds.split(",") if s.strip())
    data = build(seeds)

    if args.json:
        print(json.dumps(data, indent=2, sort_keys=True, default=str))
    else:
        print(render(data))

    if args.out:
        out_dir = Path(args.out)
        if not out_dir.is_absolute():
            out_dir = REPO_ROOT / out_dir
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / "table.txt").write_text(render(data) + "\n", encoding="utf-8")
        (out_dir / "comparison.json").write_text(
            json.dumps(data, indent=2, sort_keys=True, default=str) + "\n",
            encoding="utf-8")
        print(f"\n[ab_compare] wrote {out_dir}/table.txt and comparison.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
