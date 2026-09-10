"""
scripts/node_contribution.py

**Does this node help?** Leave-one-out ablation of a tree, over the dev
scenarios.

    $P scripts/node_contribution.py --composed <composition_id>
    $P scripts/node_contribution.py --composed <id> --metric spl --seeds 0,1,2
    $P scripts/node_contribution.py --composed <id> --json

This is what replaced chunk 37's admission gate. The gate asked whether a node
worked ALONE in a world the generator designed for itself; this asks whether a
TREE containing it does better than the same tree without it, across scenarios
the generator did not design.

READ THE DISTRIBUTION, NOT THE MEAN. A node that changes nothing on most
scenarios and rescues one is the case this exists for. `never_fired_in` is
printed beside the effect because "did not help" and "never got the chance" are
different findings, and the single admission number conflated them.

⚠️ SIM-FREE, and therefore NOT REPORTABLE. It runs `core/tests/kinematic.py`:
real lab42 geometry, real sensing, real drift, real scoring — and an idealised
unicycle with the frozen locomotion policy absent. Representative world,
unrepresentative physics. Physics confirmation is chunk 36.

⚠️ THE DEV SCENARIOS ONLY. This measurement feeds back into which nodes get
built, so running it against the sealed split is the leak G3 exists to prevent.
"""

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--composed", required=True,
                    help="composition id (without the composed: prefix)")
parser.add_argument("--metric", default="goal_reached",
                    help="metric to difference (default: goal_reached)")
parser.add_argument("--seeds", default="0",
                    help="comma-separated seeds (default: 0)")
parser.add_argument("--json", action="store_true", help="machine-readable")
ARGS = parser.parse_args()

import controllers                                           # noqa: E402
from core.scenario import DEV_SCENARIOS                       # noqa: E402
from harness.ablate import ablatable, contribution            # noqa: E402


def main():
    seeds = tuple(int(s) for s in ARGS.seeds.split(",") if s.strip())
    name = ARGS.composed
    if not name.startswith("composed:"):
        name = "composed:" + name
    factory = lambda: controllers.make(name)

    scenarios = list(DEV_SCENARIOS)
    out = contribution(factory, scenarios, seeds=seeds, metric=ARGS.metric)

    if ARGS.json:
        print(json.dumps(out, indent=2, sort_keys=True))
        return 0

    print(f"[contribution] {ARGS.composed}")
    print(f"[contribution] metric={ARGS.metric}  scenarios="
          f"{[s.id for s in scenarios]}  seeds={list(seeds)}")
    print(f"[contribution] leaves: {ablatable(factory())}")
    print("\n  ⚠️ kinematic, dev scenarios only — NOT a reportable number\n")
    print(f"  {'node':30} {'mean':>7} {'helped':>7} {'hurt':>5} {'never fired':>12}")
    for node_id, row in sorted(out.items(),
                               key=lambda kv: -kv[1]["mean_delta"]):
        print(f"  {node_id:30} {row['mean_delta']:+7.3f} "
              f"{row['helped_in']:>3}/{row['n']:<3} {row['hurt_in']:>5} "
              f"{row['never_fired_in']:>7}/{row['n']:<4}")
    print("\n  per scenario:")
    for node_id, row in sorted(out.items()):
        print(f"    {node_id}")
        for key, cell in row["per_scenario"].items():
            flag = "  (never fired)" if cell["fired"] == 0 else ""
            print(f"      {key:26} intact={cell['intact']:>6.3f} "
                  f"ablated={cell['ablated']:>6.3f} "
                  f"delta={cell['delta']:+.3f}{flag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
