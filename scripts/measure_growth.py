#!/usr/bin/env python
"""
scripts/measure_growth.py

**Does adding this node to the base help?** Chunk 44's growth loop, one round.

For each admitted library node, builds

    Layer[ ContractPorts, BASE, node ]

and scores it against the BASE ALONE on **identical worlds**.

⚠️ **THE TREE SHAPE CHANGED ON 2026-09-14 (chunk 52) AND THE OLD ONE IS WHY.**
Round 1 used `Fallback[specialist, BASE]`, which gives the specialist first
refusal every tick and silences the base whenever it fires. Measured over 20
maze worlds, 11 nodes: **not one improved the tree, `fixed` was 0 in every row,
and B−A was exactly zero.** Fire rates were bimodal with nothing between — five
nodes at 0.0%, six at 87.6-100% — and at 100% the base is never ticked at all,
so the "pair" is just the specialist alone.

Under a `Layer` every child ticks. The base proposes a whole command every tick
and the node revises only the Twist fields it declared in `WRITES`. A node whose
precondition fails leaves the base's command **byte-identical**, so an unchanged
episode is now provably an episode the node did not touch — which is what makes
`fixed` and `broke` attributable at all.

`changed` is the new column and it is the one that separates the two failure
modes round 1 could not: a node with `fire` high and `changed` zero is firing and
doing nothing, which reads identically to a node that never fires if you only
look at the rate.

⚠️ THE SUMMARY CSV IS AN AGGREGATE. `fixed = 2` says two worlds flipped and never
which two, so attributing a result to a geometry meant re-running. Since
2026-09-15 a second file, `<stem>-worlds.csv`, carries ONE ROW PER (node x world)
— seed, scenario id, both booleans, the outcome as one of
fixed/broke/both_ok/both_fail, that world's per-world fire/changed, and its
MEASURED difficulty. A world is a pure function of (tier, seed), so those two
name the environment completely.

⚠️ NOT A BENCHMARK PATH. Kinematic rollout, no locomotion policy, no physics.
`core/tests/kinematic.py` says it first. And the 2026-09-12 calibration measured
kinematic-vs-physics agreement as excellent on `goal_reached` (6/6) but poor on
path shape for FAILING runs — so read the success column, not the path column.

⚠️ Paired on seeds. Every controller sees the same worlds in the same order, so
a difference is a difference and not a resampling artefact.

Examples
--------
    P=~/miniforge3/envs/env_isaacsim/bin/python

    $P scripts/measure_growth.py --tier maze --n 20
    $P scripts/measure_growth.py --tier maze --tier cluttered --n 20
    $P scripts/measure_growth.py --tier maze --n 20 --node gen_b_c2c95bcb965e342e

    # a REPAIR ROUND, measured by this script rather than by the repair loop
    $P scripts/measure_growth.py --tier maze --n 20 --metrics \
         --staging out/repair/gen_b_44d12df4d25b437f/round-01 \
         --out out/growth/repair-r01.csv
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Base vs base+node, paired. NOT a benchmark path.")
    p.add_argument("--tier", action="append", dest="tiers", default=None,
                   help="repeatable; default is the growth tier, maze")
    p.add_argument("--n", type=int, default=20, help="seeds per tier")
    p.add_argument("--start-seed", type=int, default=2000)
    p.add_argument("--time-limit", type=float, default=120.0)
    p.add_argument("--node", action="append", dest="nodes", default=None,
                   help="repeatable; default is every admitted node")
    p.add_argument("--staging", action="append", dest="staging", default=None,
                   help="repeatable; read the library from THESE directories "
                        "instead of generate/staging/. ⚠️ This is how a REPAIR "
                        "ROUND is measured from outside the repair loop: "
                        "repair/loop.py stages each round under "
                        "out/repair/<lineage>/round-NN/ and deliberately never "
                        "writes into generate/staging/, so a repaired node is "
                        "invisible to load_library() and to this script's "
                        "default. It carries its own node_id in its record, so "
                        "the directory is all that is needed. Nothing is "
                        "copied and no admission is re-decided.")
    p.add_argument("--metrics", action="store_true",
                   help="also score the FULL metric set — collisions, "
                        "min_clearance, time_to_goal, spl, ... — not just "
                        "goal_reached. ⚠️ 6 of the 16 committed nodes predict "
                        "`collisions`, not `goal_reached`, so without this "
                        "they are being judged on a claim they never made. "
                        "Costs one ~22 s SPL denominator per world, ONCE: "
                        "core.geometry caches it and every later controller on "
                        "that world reuses it.")
    p.add_argument("--out", default=None,
                   help="results file, written INCREMENTALLY. Default "
                        "out/growth/<tier>.csv. Two more land beside it: "
                        "<stem>-worlds.csv, one row per (node x world) so "
                        "`fixed` names WHICH worlds; and, with --metrics, "
                        "<stem>-metrics.csv")
    return p.parse_args(argv)


# Everything above this line is code; below it is data, not code.
# ---------------------------------------------------------------------------

# The growth tier, from the chunk 47 calibration: the base scores 40% on `maze`,
# which is inside the 30-70% band. `cluttered` (70%) is the secondary and `open`
# (90%) is a regression floor, not a place to measure improvement.
DEFAULT_TIERS = ("maze",)

# The metrics reported by --metrics, and which way is better.
#
# ⚠️ WHY THIS EXISTS AT ALL. Round 2 measured `goal_reached` and nothing else,
# and then read six "+0%" rows as "this node does nothing". Six of the sixteen
# committed nodes predict **`collisions`** — the entire speed-limiter family —
# and one predicts `min_clearance`. Judging a speed cap on goal-reach is judging
# it on a claim it never made: a cap that prevents two collisions and costs
# nothing in arrivals scores +0% and looks inert.
#
# `spl` is included because the denominator has to be paid for anyway, and
# `localisation_failures` is not: `cards.store.CONTROLLER_INDEPENDENT` says no
# controller can move it, so a delta in it would be noise wearing a name.
# ⚠️ A COUNT METRIC IS NOT COMPARABLE BETWEEN CONTROLLERS THAT TRAVEL DIFFERENT
# DISTANCES, and this bit on the first run that measured it. `collisions` is
# lower-is-better, so **a robot that stops scores perfectly**: one node took
# `collisions` from 0.40 to 0.30 and looked like the library's second win — it
# had walked 6.2 m instead of 30.0 m and reached the goal in 0 of 20 worlds.
# Per metre it collides 3.6x MORE than the base, and the "win" reverses.
#
# So every count metric is ALSO reported per metre travelled, and the per-metre
# figure is the one to read whenever the arrival rate moved. Neither replaces
# the other: per-metre alone would flatter a node that crawls safely and never
# arrives, which is why `goal_reached` stays the primary.
PER_METRE = ("collisions", "near_misses", "stuck_events")

REPORTED_METRICS = (
    ("goal_reached", "higher"),
    ("collisions", "lower"),
    ("near_misses", "lower"),
    ("min_clearance", "higher"),
    ("stuck_events", "lower"),
    ("time_to_goal", "lower"),
    ("path_length", "lower"),
    ("spl", "higher"),
)


def entry_metric(entry):
    """The metric THIS node's own card claims to move, or None.

    Read off the staged module's `PREDICTION`, which is the card's claim
    restated — so the row can say whether the node moved the thing it said it
    would, rather than only whether it moved the thing the loop happens to
    measure.
    """
    import re
    from generate.generator import STAGING
    for path in Path(STAGING).glob("*.py"):
        if f"gen_{path.name.split('__')[0]}_" not in entry.metadata.node_id:
            continue
        if entry.metadata.node_id.split("_")[-1] not in path.stem:
            continue
        found = re.search(r'"metric":\s*"([a-z_]+)"',
                          path.read_text(encoding="utf-8"))
        return found.group(1) if found else None
    return None


def _mean(rows, key):
    """Mean of one metric over the scenarios, with None meaning 'did not happen'.

    `time_to_goal` is None when the goal was never reached and `recovery_time`
    when nothing was ever stuck. Averaging over only the episodes where the
    event happened would make a node that reaches the goal twice as often look
    slower, so the episodes where it did not happen are excluded and the COUNT
    is reported beside the mean.
    """
    import math as _m
    vals = [r[key] for r in rows.values() if r.get(key) is not None]
    if not vals:
        return None, 0
    return float(sum(vals)) / len(vals), len(vals)


def _report_metrics(csv, tier, node_id, arm, base_scores, pair_scores, claimed):
    """One block per node: base vs pair on every reported metric."""
    base_m, _ = _mean(base_scores, "path_length")
    pair_m, _ = _mean(pair_scores, "path_length")

    for metric, direction in REPORTED_METRICS:
        b, nb = _mean(base_scores, metric)
        p, np_ = _mean(pair_scores, metric)
        if b is None or p is None:
            csv.write(f"{tier},{node_id},{arm},{metric},,,,{direction},"
                      f"{metric == claimed},{nb},{np_}\n")
            continue
        delta = p - b
        better = (delta > 0) if direction == "higher" else (delta < 0)
        mark = ""
        if abs(delta) > 1e-9:
            mark = "  BETTER" if better else "  WORSE"
        if metric == claimed:
            mark += "   <-- THE NODE'S OWN CLAIM"
        print(f"[growth]     {metric:16} base {b:8.3f}  pair {p:8.3f}  "
              f"{delta:+8.3f}{mark}", flush=True)
        csv.write(f"{tier},{node_id},{arm},{metric},{b:.4f},{p:.4f},"
                  f"{delta:+.4f},{direction},{metric == claimed},{nb},{np_}\n")

        # The same count, per metre travelled. See PER_METRE above: without it
        # a node that stops the robot reads as a safety improvement.
        if metric in PER_METRE and base_m and pair_m:
            rb, rp = b / base_m, p / pair_m
            flips = ((delta < 0) != (rp - rb < 0)) and abs(delta) > 1e-9
            print(f"[growth]     {metric + '/m':16} base {rb:8.4f}  "
                  f"{'pair':>4} {rp:8.4f}  {rp - rb:+8.4f}"
                  f"{'   <-- REVERSES the raw count' if flips else ''}",
                  flush=True)
            csv.write(f"{tier},{node_id},{arm},{metric}_per_metre,{rb:.6f},"
                      f"{rp:.6f},{rp - rb:+.6f},{direction},False,{nb},{np_}\n")


def _build(specialist_cls, specialist_meta, base):
    """Layer[ContractPorts, BASE, node] — the growth tree.

    The node goes AFTER the base, which is the whole of the ordering rule: the
    base proposes, later layers revise. A node placed before it would be revising
    nothing, and `PortView` raises rather than letting that pass silently.

    A node that still drives a whole command (a v2-shaped Action, or a v3 one
    that did not narrow its WRITES) is not special-cased. It sits in the same
    slot and overwrites the base's command outright — which IS what such a node
    does, and measuring it honestly is more useful than excluding it.
    """
    from generate.admit import ContractPorts
    from nodes.tree import Layer, Tree
    from nodes.toys import toy_metadata

    return Tree(Layer(toy_metadata("growth_layer"), [
        ContractPorts(toy_metadata("growth_ports")),
        base.node_cls(base.metadata),
        specialist_cls(specialist_meta),
    ]))


def main(argv=None):
    args = _parse_args(argv)

    import math
    from compose.library import CONTRACT_PORTS_ID, load_library
    from core.metrics import GOAL_RADIUS
    from core.tests.kinematic import rollout
    from core.worldgen import clear_geometry_caches, generate, to_scenario
    from generate import base as basemod
    from generate.admit import build_tree
    from nodes.tree import WiringError

    tiers = args.tiers or list(DEFAULT_TIERS)
    seeds = list(range(args.start_seed, args.start_seed + args.n))
    base = basemod.load()

    # ⚠️ `staging=None` is load_library's own default and means the live
    # staging dir — passing args.staging through unconditionally keeps one code
    # path, so the flag cannot drift from the default it overrides.
    entries = [e for e in load_library(staging=args.staging).all()
               if e.metadata.node_id != CONTRACT_PORTS_ID]
    if args.nodes:
        entries = [e for e in entries if e.metadata.node_id in set(args.nodes)]
    entries.sort(key=lambda e: (str(getattr(e.metadata.origin, "value",
                                            e.metadata.origin)),
                                e.metadata.node_id))

    out_dir = Path(args.out).parent if args.out else REPO_ROOT / "out" / "growth"
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[growth] base={basemod.BASE_NODE_ID}  {len(entries)} node(s)  "
          f"tiers={tiers}  n={len(seeds)}", flush=True)
    print(f"[growth] ⚠ kinematic, paired on seeds, NOT a benchmark path",
          flush=True)
    print(f"[growth] results are written INCREMENTALLY to {out_dir} — a run "
          f"that is killed keeps everything it had finished.", flush=True)

    def scored(tree, scenario, watch=None):
        """The FULL metric set for one rollout, through the real scorer.

        `core.metrics.score`, never a reimplementation: a number this project
        reports has to come from the committed scorer, and reproducing six of
        its eighteen keys here would be a second definition of `collisions` that
        nothing keeps in step with the first.
        """
        from core.metrics import score
        log = tree.enable_trace() if watch else None
        trace = rollout(scenario, tree)
        seen = log.summary().get(watch, {}) if log is not None else {}
        return score(trace, scenario), seen

    def reached(tree, scenario, watch=None):
        """Did it arrive — and, for the added node, did it ever do anything?

        `watch` turns tracing on and reports that node's ticks. Tracing is inert
        by construction (`nodes/trace.py`): the Twist emitted and the trace
        written are byte-identical either way, so the success column is not
        affected by measuring the fire rate beside it.
        """
        log = tree.enable_trace() if watch else None
        trace = rollout(scenario, tree)
        x, y = trace.column("x")[-1], trace.column("y")[-1]
        hit = math.dist((x, y), scenario.goal) <= GOAL_RADIUS
        seen = log.summary().get(watch, {}) if log is not None else {}
        return hit, seen

    for tier in tiers:
        # Built once and reused by every controller: paired comparison, and the
        # worlds are the expensive part.
        # The GeneratedWorld is kept, not just the Scenario: `difficulty` lives
        # on the world and the per-world rows carry it, so a row can be read as
        # "this node fails on THAT geometry" without regenerating anything.
        generated = [generate(tier, s) for s in seeds]
        scenarios = [to_scenario(g, time_limit=args.time_limit)
                     for g in generated]
        difficulty = {sc.id: g.difficulty
                      for g, sc in zip(generated, scenarios)}

        if args.metrics:
            # ⚠️ ONCE, HERE, AND NEVER AGAIN. `optimal_distance` is the SPL
            # denominator and it is a ~22 s visibility-graph solve on a
            # generated world — quadratic in obstacle count, epic 05 chunk 45.
            # core.geometry caches it on the world, so paying it here makes
            # `score()` free for all 17 controllers that follow. Doing it lazily
            # inside the loop would pay it once anyway; doing it here means the
            # cost is visible rather than looking like a slow first node.
            #
            # ⚠️ And it is why `clear_geometry_caches()` must stay at the END of
            # the tier loop and never move inside it.
            import time
            t0 = time.time()
            print(f"[growth] --metrics: precomputing the SPL denominator for "
                  f"{len(scenarios)} world(s); ~22 s each, then cached",
                  flush=True)
            for i, sc in enumerate(scenarios, 1):
                d = sc.optimal_distance()
                print(f"[growth]   {sc.id:28} optimal={d:6.2f} m  "
                      f"({time.time() - t0:5.0f}s elapsed)", flush=True)

        csv_path = Path(args.out) if args.out else out_dir / f"{tier}.csv"
        csv = csv_path.open("w", encoding="utf-8", buffering=1)   # line-buffered
        csv.write("tier,node_id,arm,rate,delta,fixed,broke,fire,changed,n\n")

        # ⚠️ THE SUMMARY FILE IS AN AGGREGATE: `fixed = 2` says two worlds
        # flipped and never WHICH two, so attributing a result to a geometry
        # meant re-running. This file is the missing half — one row per
        # (node x world), with the base's own rows first.
        worlds_path = csv_path.with_name(csv_path.stem + "-worlds.csv")
        worlds_csv = worlds_path.open("w", encoding="utf-8", buffering=1)
        worlds_csv.write("tier,seed,scenario_id,node_id,arm,base_reached,"
                         "pair_reached,outcome,fire,changed,ticks,geodesic,"
                         "tortuosity,min_corridor,claim_metric,base_claim,"
                         "pair_claim\n")

        def world_row(sc, node_id, arm, base_hit, pair_hit=None, outcome="",
                      fire="", changed="", ticks="", claim="",
                      base_claim="", pair_claim=""):
            d = difficulty[sc.id]
            worlds_csv.write(
                f"{tier},{sc.seed},{sc.id},{node_id},{arm},"
                f"{int(base_hit)},"
                f"{'' if pair_hit is None else int(pair_hit)},{outcome},"
                f"{fire},{changed},{ticks},{d.geodesic_length:.2f},"
                f"{d.tortuosity:.2f},{d.min_corridor:.2f},"
                f"{claim},{base_claim},{pair_claim}\n")

        metrics_csv = None
        if args.metrics:
            metrics_path = csv_path.with_name(csv_path.stem + "-metrics.csv")
            metrics_csv = metrics_path.open("w", encoding="utf-8", buffering=1)
            metrics_csv.write("tier,node_id,arm,metric,base,pair,delta,"
                              "direction,is_own_claim,n_base,n_pair\n")

        base_scores = {}
        if args.metrics:
            for s in scenarios:
                base_scores[s.id] = scored(
                    build_tree(base.node_cls, base.metadata), s)[0]
            base_hits = {k: bool(v["goal_reached"])
                         for k, v in base_scores.items()}
        else:
            base_hits = {
                s.id: reached(build_tree(base.node_cls, base.metadata), s)[0]
                for s in scenarios}
        base_rate = sum(base_hits.values()) / len(scenarios)
        print(f"\n[growth] === {tier} ===   BASE ALONE: "
              f"{sum(base_hits.values())}/{len(scenarios)} = {base_rate:.0%}",
              flush=True)
        csv.write(f"{tier},BASE_ALONE,-,{base_rate:.4f},0,0,0,,,"
                  f"{len(scenarios)}\n")
        for sc in scenarios:
            world_row(sc, "BASE_ALONE", "-", base_hits[sc.id])
        print(f"[growth] {'node':30} {'arm':4} {'rate':>7} {'delta':>7}  "
              f"{'fixed':>5} {'broke':>5} {'fire':>6} {'chg':>6}", flush=True)

        for e in entries:
            meta = e.metadata
            # ⚠️ `invented` was MISSING here until 2026-09-14, so D's row —
            # the only row in the whole table with a non-zero `fixed` — printed
            # as "?". The arms are A/B/D and the map has to carry all three.
            arm = {"citation": "A", "card": "B", "invented": "D"}.get(
                str(getattr(meta.origin, "value", meta.origin)), "?")
            try:
                hits, ticks, fired, changed = {}, 0, 0, 0
                pair_scores = {}
                # Buffered, not written yet: a node that raises partway through
                # writes NO rows, so this file never disagrees with the summary
                # about which nodes were measured.
                per_world = []
                for s in scenarios:
                    if args.metrics:
                        row, seen = scored(_build(e.node_cls, meta, base), s,
                                           watch=meta.node_id)
                        pair_scores[s.id] = row
                        hit = bool(row["goal_reached"])
                    else:
                        hit, seen = reached(_build(e.node_cls, meta, base), s,
                                            watch=meta.node_id)
                    hits[s.id] = hit
                    w_ticks = seen.get("ticks", 0)
                    per_world.append((s, hit, w_ticks,
                                      seen.get("precondition_passed", 0),
                                      seen.get("drove", 0)))
                    ticks += w_ticks
                    fired += seen.get("precondition_passed", 0)
                    # `drove` is recorded as "the command object changed while
                    # this node ticked". For a driver that means it wrote one;
                    # for a modifier it means its adjustment actually BIT — a cap
                    # that was already satisfied leaves the command untouched and
                    # is correctly counted as having done nothing.
                    changed += seen.get("drove", 0)
            except WiringError as exc:
                # A node that reads a port only another specialist publishes
                # cannot sit in a two-leaf tree. That is a fact about the node,
                # recorded rather than skipped silently.
                print(f"[growth] {meta.node_id:30} {arm:4} "
                      f"UNSATISFIED_PORT: {str(exc)[:60]}", flush=True)
                csv.write(f"{tier},{meta.node_id},{arm},,,,,,,UNSATISFIED_PORT\n")
                continue
            except Exception as exc:                       # noqa: BLE001
                print(f"[growth] {meta.node_id:30} {arm:4} "
                      f"ERROR {type(exc).__name__}: {str(exc)[:50]}", flush=True)
                csv.write(f"{tier},{meta.node_id},{arm},,,,,,,ERROR\n")
                continue

            rate = sum(hits.values()) / len(scenarios)
            fixed = sum(1 for k in hits if hits[k] and not base_hits[k])
            broke = sum(1 for k in hits if base_hits[k] and not hits[k])
            fire = fired / ticks if ticks else 0.0
            chg = changed / ticks if ticks else 0.0
            flag = "  <-- IMPROVES" if rate > base_rate else (
                   "  <-- REGRESSES" if rate < base_rate else "")
            if fire > 0.0 and chg == 0.0:
                flag += "  (fires, changes nothing)"
            print(f"[growth] {meta.node_id:30} {arm:4} {rate:6.0%} "
                  f"{rate - base_rate:+6.0%}  {fixed:5} {broke:5} "
                  f"{fire:5.0%} {chg:5.0%}{flag}", flush=True)
            csv.write(f"{tier},{meta.node_id},{arm},{rate:.4f},"
                      f"{rate - base_rate:+.4f},{fixed},{broke},"
                      f"{fire:.4f},{chg:.4f},{len(scenarios)}\n")

            # ⚠️ THIS is what makes `fixed` and `broke` attributable to a world.
            # `outcome` names the four cells of the paired 2x2 explicitly rather
            # than leaving the reader to recompute them from two booleans.
            claim = entry_metric(e) or ""
            for sc, hit, w_ticks, w_fired, w_changed in per_world:
                was = base_hits[sc.id]
                outcome = ("fixed" if hit and not was else
                           "broke" if was and not hit else
                           "both_ok" if hit else "both_fail")
                bc = pc = ""
                if claim and pair_scores:
                    # The node's OWN claimed metric, per world. None (e.g.
                    # time_to_goal on a failure) is written as empty, never 0 —
                    # "did not arrive" is not "arrived instantly".
                    bv = base_scores.get(sc.id, {}).get(claim)
                    pv = pair_scores.get(sc.id, {}).get(claim)
                    bc = "" if bv is None else f"{float(bv):.4f}"
                    pc = "" if pv is None else f"{float(pv):.4f}"
                world_row(sc, meta.node_id, arm, was, hit, outcome,
                          f"{w_fired / w_ticks:.4f}" if w_ticks else "",
                          f"{w_changed / w_ticks:.4f}" if w_ticks else "",
                          w_ticks, claim, bc, pc)

            if args.metrics:
                _report_metrics(metrics_csv, tier, meta.node_id, arm,
                                base_scores, pair_scores, entry_metric(e))

        csv.close()
        worlds_csv.close()
        print(f"[growth] wrote {worlds_path}  (one row per node x world)",
              flush=True)
        if metrics_csv is not None:
            metrics_csv.close()
            print(f"[growth] wrote {metrics_path}", flush=True)
        print(f"[growth] wrote {csv_path}", flush=True)
        clear_geometry_caches()

    print("\n[growth] `fixed` = base failed and the pair succeeded; "
          "`broke` = the reverse. A node can be net-zero and still be doing "
          "something — read those two columns, not just the delta.")
    print("[growth] `fire` = ticks whose precondition passed; `changed` = ticks "
          "where the command actually differed afterwards. fire>0 with "
          "changed=0 is a node that runs and does nothing, which is a different "
          "finding from one that never runs.")
    print("[growth] <name>-worlds.csv carries ONE ROW PER (node x world) with "
          "the seed, the outcome (fixed/broke/both_ok/both_fail) and that "
          "world's measured difficulty — so `fixed` names WHICH worlds, not "
          "just how many.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
