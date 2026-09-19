#!/usr/bin/env python
"""
scripts/measure_under_drift.py

**DOES THIS NODE HELP ON THE AXIS ITS CARD ACTUALLY PREDICATES ON?**

`core.worldgen.to_scenario()` never sets `drift`, so every world in the growth
suite hands the controller a **perfect pose estimate**. A card whose prediction
opens *"On scenarios where pose_estimate drifts"* has therefore never been
measured under the one condition it names. This runs the same paired comparison
`measure_growth.py` runs — `Layer[ports, BASE, node]` against the base alone —
with the hazard switched **on**, on the **same seeds**.

⚠️ **SAME SEEDS, ONE VARIABLE. THIS IS NOT THE PROBE.** `repair/probe.py`
SEARCHES for a world where a node could help, which is the loop's biggest
overfitting hazard and is guarded accordingly. This does not search: the seed
set is fixed in advance, every seed is reported, and the only thing that moves
is `drift`. There is no world-selection freedom, so there is nothing to overfit
to.

⚠️ **NOT A BENCHMARK PATH.** Kinematic rollouts, no frozen locomotion policy,
so no number here is reportable. It is the fast loop that says whether spending
Isaac Sim on this node is justified.

⚠️ **A DRIFTED WORLD IS A DIFFERENT SCENARIO.** `Candidate.label` renames it
`gen-<tier>-<seed>+drift<sigma>`, the same discipline `--goal-hidden` and
`+cap60s` follow, so a drifted result can never overwrite its clean twin.

The pairing is base-under-drift versus node-under-drift. Comparing a drifted
node against a clean base would credit the node with the hazard.

Examples
--------
    P=~/miniforge3/envs/env_isaacsim/bin/python

    $P scripts/measure_under_drift.py --node gen_b_44d12df4d25b437f__r01 --dry-run
    $P scripts/measure_under_drift.py --node gen_b_44d12df4d25b437f__r01
    $P scripts/measure_under_drift.py --node gen_b_44d12df4d25b437f__r01 \
         --drift 0.20 --seeds 9000-9019          # the CONFIRMATION range
"""

import argparse
import csv
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# The growth suite's own seeds. Every other node in the library was scored on
# exactly these, which is what makes this number comparable with that table.
DEFAULT_SEEDS = range(2000, 2020)

# ⚠️ FROM `repair/probe.py`, NOT REDECLARED. `core/drift.py`'s sigma is RADIAL
# m/sqrt(s), so realised error after 120 s is sigma*sqrt(120) = 0.55 / 1.10 /
# 2.19 m. 0.10 is the corridor-width rung and is the default here.
DEFAULT_SIGMA = 0.10

CSV_COLUMNS = ("seed", "condition", "scenario_id", "base_reached",
               "node_reached", "outcome", "fire", "changed", "seconds")


# ---------------------------------------------------------------------------


def _parse_seeds(text):
    """`2000-2019` or `2004,2007` -> a list. Validated before anything loads."""
    if "-" in text and "," not in text:
        lo, _, hi = text.partition("-")
        return list(range(int(lo), int(hi) + 1))
    return [int(s) for s in text.split(",") if s.strip()]


def _parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[2])
    p.add_argument("--node", required=True)
    p.add_argument("--tier", default="maze")
    p.add_argument("--seeds", type=_parse_seeds, default=list(DEFAULT_SEEDS))
    p.add_argument("--drift", type=float, default=DEFAULT_SIGMA,
                   help="sigma_xy in m/sqrt(s); sigma_yaw is half of it")
    p.add_argument("--no-clean", action="store_true",
                   help="skip the undrifted twin (it is the control — keep it "
                        "unless it has already been measured by this script)")
    p.add_argument("--out", default=None)
    p.add_argument("--dry-run", action="store_true",
                   help="print the plan and roll out nothing")
    return p.parse_args(argv)


def _row(cand, condition, base_reached, node_reached, fire, changed, secs):
    outcome = ("fixed" if node_reached and not base_reached else
               "broke" if base_reached and not node_reached else
               "both_ok" if base_reached else "both_fail")
    return {"seed": cand.seed, "condition": condition,
            "scenario_id": cand.label, "base_reached": int(base_reached),
            "node_reached": int(node_reached), "outcome": outcome,
            "fire": round(fire, 4), "changed": round(changed, 4),
            "seconds": round(secs, 1)}


def _summarise(rows, condition):
    sel = [r for r in rows if r["condition"] == condition]
    if not sel:
        return None
    n = len(sel)
    counts = {k: sum(1 for r in sel if r["outcome"] == k)
              for k in ("fixed", "broke", "both_ok", "both_fail")}
    return {
        "condition": condition, "n": n,
        "base": sum(r["base_reached"] for r in sel),
        "node": sum(r["node_reached"] for r in sel),
        "fire": sum(r["fire"] for r in sel) / n,
        "changed": sum(r["changed"] for r in sel) / n,
        **counts,
    }


def main(argv=None):
    args = _parse_args(argv)

    conditions = [("drift", args.drift)]
    if not args.no_clean:
        conditions.insert(0, ("clean", None))

    print(f"[measure_drift] node   {args.node}")
    print(f"[measure_drift] tier   {args.tier}  seeds {args.seeds[0]}-"
          f"{args.seeds[-1]} (n={len(args.seeds)})")
    print(f"[measure_drift] sigma  {args.drift}  "
          f"(~{args.drift * (120 ** 0.5):.2f} m realised error over 120 s)")
    print(f"[measure_drift] pairs  {len(args.seeds) * len(conditions)} "
          f"node rollouts + the same number of base rollouts, minus cache hits")

    if args.dry_run:
        print("[measure_drift] DRY RUN — rolling out nothing")
        return 0

    from repair.evidence import entry_for
    from repair.probe import (Candidate, _load_cache, _runner, _save_cache,
                              base_result)

    entry = entry_for(args.node)
    runner = _runner()
    dbg, build, base, _ = runner
    cache = _load_cache()

    out = Path(args.out or REPO_ROOT / "out" / "drift" /
               f"{args.node}-{args.drift:.2f}.csv")
    out.parent.mkdir(parents=True, exist_ok=True)

    rows = []
    # ⚠️ WRITTEN INCREMENTALLY, so a killed run keeps what it finished — the
    # same rule `measure_growth.py` follows.
    with out.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        for condition, sigma in conditions:
            for seed in args.seeds:
                t0 = time.time()
                cand = Candidate(args.tier, int(seed), drift=sigma)
                base_reached, hit = base_result(cand, cache, runner)
                _save_cache(cache)

                tree = build(entry.node_cls, entry.metadata, base)
                _, score, _, summary = dbg._rollout(tree, cand.scenario(),
                                                    args.node)
                ticks = max(1, int(summary.get("ticks", 1)))
                row = _row(cand, condition, base_reached,
                           bool(score["goal_reached"]),
                           float(summary.get("precondition_passed", 0)) / ticks,
                           float(summary.get("drove", 0)) / ticks,
                           time.time() - t0)
                writer.writerow(row)
                fh.flush()
                rows.append(row)
                print(f"[measure_drift] {row['scenario_id']:<28} "
                      f"base={'Y' if base_reached else 'n'}"
                      f"{'*' if hit else ' '} node="
                      f"{'Y' if row['node_reached'] else 'n'} "
                      f"{row['outcome']:<10} fire={row['fire']:.2f} "
                      f"{row['seconds']:.0f}s")

    print()
    head = (f"{'condition':<10}{'n':>4}{'base':>6}{'node':>6}{'fixed':>7}"
            f"{'broke':>7}{'both_ok':>9}{'both_fail':>11}{'fire':>7}{'changed':>9}")
    print(head)
    print("-" * len(head))
    for condition, _ in conditions:
        s = _summarise(rows, condition)
        if s:
            print(f"{s['condition']:<10}{s['n']:>4}{s['base']:>6}{s['node']:>6}"
                  f"{s['fixed']:>7}{s['broke']:>7}{s['both_ok']:>9}"
                  f"{s['both_fail']:>11}{s['fire']:>7.2f}{s['changed']:>9.2f}")

    print()
    print(f"[measure_drift] wrote {out}")
    print("[measure_drift] ⚠️ KINEMATIC. Not a benchmark number.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
