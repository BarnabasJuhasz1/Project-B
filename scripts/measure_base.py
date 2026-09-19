#!/usr/bin/env python
"""
scripts/measure_base.py

**Is the environment hard enough?** Chunk 47's calibration loop.

The design decision this serves, in the user's words:

    I want the worlds to be too hard for the base node, because then we can
    actually see the other nodes helping. I want the base as strong as possible,
    but still failing because the environment is difficult. **If it succeeds
    100% of the time, that does not mean the base is too strong — it means the
    environment is too easy.**

So difficulty is **the base node's failure rate**, and the free variable is the
WORLD. This measures the rate; hardening the tier is the response to a rate that
is too low, and weakening the base never is.

`core/worldgen.py`'s geometry statistics (tortuosity, free fraction, bottleneck,
obstacle count) are cheap proxies for the same thing. Where they and this
disagree, **this is right and the proxies are wrong.**

⚠️ NOT A BENCHMARK PATH. Kinematic rollout: no locomotion policy, no physics, no
contact dynamics (`core/tests/kinematic.py` says it first). Calibration decides
what to build, not what to report.

⚠️ It reports `goal_reached` and NOTHING that needs the SPL denominator.
`optimal_distance()` costs ~43 s on a generated world because the visibility
graph is quadratic in obstacle count, and a calibration sweep over hundreds of
worlds cannot pay it. Deliberate: see the chunk 45 outcome in epic 05.

⚠️ The suite is tuned until the base fails, which is **adversarial selection of
the environment against the base**. Absolute success rates from a suite
calibrated this way may never be reported as "how well the system navigates".
And the tuning target is the SHARED BASE ONLY, never an arm's nodes.

Examples
--------
    P=~/miniforge3/envs/env_isaacsim/bin/python

    $P scripts/measure_base.py --controller reactive_avoid --n 20
    $P scripts/measure_base.py --controller node:<base_id> --tier maze --n 40
    $P scripts/measure_base.py --controller reactive_avoid --n 20 --failures
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Base-node failure rate per tier. NOT a benchmark path.")
    p.add_argument("--controller", required=True,
                   help="any name run_episode.py accepts: a registry name, "
                        "node:<id>, composed:<id>")
    p.add_argument("--tier", action="append", dest="tiers",
                   help="repeatable; default is every tier")
    p.add_argument("--n", type=int, default=20, help="seeds per tier")
    p.add_argument("--start-seed", type=int, default=2000)
    p.add_argument("--time-limit", type=float, default=120.0)
    p.add_argument("--failures", action="store_true",
                   help="list the seeds that failed, which is what a "
                        "failure-driven generation round needs as its query")
    return p.parse_args(argv)


# Everything above this line is code; below it is data, not code.
# ---------------------------------------------------------------------------

# The band a growth tier has to land in, from plans/done/47-strongest-base-node.md.
# Below it the loop has too few successes to avoid regressing; above it there is
# too little headroom for an added node to show anything. A judgement, recorded
# so it is not quietly changed later.
TARGET_LOW, TARGET_HIGH = 0.30, 0.70


def _verdict(rate):
    """What the measured success rate MEANS, in the framing above."""
    if rate >= 0.995:
        return "ENVIRONMENT TOO EASY — harden the tier, do not weaken the base"
    if rate <= 0.005:
        return "ENVIRONMENT TOO HARD — nothing to climb from; soften the tier"
    if TARGET_LOW <= rate <= TARGET_HIGH:
        return "IN BAND — usable as a growth tier"
    if rate > TARGET_HIGH:
        return "too easy for a growth tier (fine as a regression floor)"
    return "too hard for a growth tier (few successes to protect)"


def main(argv=None):
    args = _parse_args(argv)

    import math
    import controllers
    from core.metrics import GOAL_RADIUS
    from core.tests.kinematic import rollout
    from core.worldgen import TIERS, clear_geometry_caches, generate, to_scenario

    tiers = args.tiers or list(TIERS)
    seeds = list(range(args.start_seed, args.start_seed + args.n))

    print(f"[measure_base] controller={args.controller}  "
          f"{len(tiers)} tier(s) x {len(seeds)} seed(s)")
    print(f"[measure_base] ⚠ kinematic, not a benchmark path; "
          f"target band {TARGET_LOW:.0%}-{TARGET_HIGH:.0%}")

    rows = []
    for tier in tiers:
        reached, failed = 0, []
        for seed in seeds:
            g = generate(tier, seed)
            scenario = to_scenario(g, time_limit=args.time_limit)
            trace = rollout(scenario, controllers.make(args.controller))
            # goal_reached WITHOUT score(): the SPL denominator is the 43 s call
            # and calibration does not need it. Same rule metrics.score() uses.
            x, y = trace.column("x")[-1], trace.column("y")[-1]
            if math.dist((x, y), scenario.goal) <= GOAL_RADIUS:
                reached += 1
            else:
                failed.append(seed)
        rate = reached / len(seeds)
        rows.append((tier, rate, failed))
        print(f"[measure_base] {tier:10} success {reached:3}/{len(seeds):<3} "
              f"= {rate:5.1%}   {_verdict(rate)}")
        if args.failures and failed:
            print(f"[measure_base]   failed seeds: {failed}")
        clear_geometry_caches()

    print()
    print("[measure_base] difficulty IS this table. The geometry statistics in "
          "core/worldgen.py are proxies for it.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
