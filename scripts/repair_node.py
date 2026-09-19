#!/usr/bin/env python
"""
scripts/repair_node.py

**THE REPAIR LOOP ENTRY POINT.** One node, one arm, one task, to a terminal
verdict, with every round on disk.

⚠️ Read `REPAIR-LOOP.md` before using it: the verdict taxonomy, the repair
action space, the arm-blindness decision and the traps are all there, and this
script assumes them.

⚠️ **THIS SPENDS MONEY.** One diagnosis call plus one node call per round, at a
default of 3 rounds. `--dry-run` prints the plan and calls nothing.

Examples
--------
    P=~/miniforge3/envs/env_isaacsim/bin/python

    $P scripts/repair_node.py --node gen_b_44d12df4d25b437f --dry-run
    $P scripts/repair_node.py --node gen_b_44d12df4d25b437f --rounds 3
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[2])
    p.add_argument("--node", required=True)
    p.add_argument("--tier", default="maze")
    p.add_argument("--rounds", type=int, default=None,
                   help="per-node budget. ⚠️ PRE-REGISTRATION MATERIAL: fixed "
                        "in advance and identical across arms.")
    p.add_argument("--probe-budget", type=int, default=6)
    p.add_argument("--drift", type=float, default=None, metavar="SIGMA",
                   help="put the drift hazard ON the repair set, sigma_xy in "
                        "m/sqrt(s) (sigma_yaw is half). ⚠️ The committed growth "
                        "CSV is UNDRIFTED, so this measures every node live, "
                        "the original included. A card whose prediction opens "
                        "\"on scenarios where pose_estimate drifts\" has never "
                        "been judged on that axis — see REPAIR-LOOP.md. "
                        "0.05/0.10/0.20 are probe.DRIFT_LADDER's rungs: a body "
                        "length, a corridor width, a doorway over 120 s. ⚠️ A "
                        "lineage may not mix axes.")
    p.add_argument("--dry-run", action="store_true",
                   help="measure and classify; stop before any model call")
    a = p.parse_args(argv)

    from repair.loop import DEFAULT_ROUNDS, first_round_number, run

    rounds = DEFAULT_ROUNDS if a.rounds is None else a.rounds
    start = first_round_number(a.node)
    print(f"[repair] node   {a.node}")
    print(f"[repair] rounds {start}-{start + rounds - 1}  "
          f"(budget {rounds}, ~{2 * rounds} model calls)"
          if not a.dry_run else "[repair] DRY RUN — calls nothing")
    if a.drift is None:
        print("[repair] axis   CLEAN — ⚠️ pose_estimate is PERFECT on every "
              "world in the growth suite")
    else:
        print(f"[repair] axis   drift sigma_xy={a.drift} "
              f"(~{a.drift * (120 ** 0.5):.2f} m realised error over 120 s), "
              f"sigma_yaw={a.drift / 2}")
        print("[repair] ⚠️ every node is measured LIVE: the committed growth "
              "CSV is undrifted and cannot answer this")

    result = run(a.node, rounds=rounds, tier=a.tier,
                 probe_budget=a.probe_budget, dry_run=a.dry_run,
                 drift=a.drift)
    print()
    print(result.log())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
