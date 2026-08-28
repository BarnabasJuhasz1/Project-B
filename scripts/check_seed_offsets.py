"""
scripts/check_seed_offsets.py

Does the drift manipulation actually take, at every pre-registered replicate seed?

`core.stats.SEED_OFFSETS` is defined by a rule: the first three multiples of
10000 at which **every** drifted held-out scenario's pose error crosses
`LOC_FAIL_THRESHOLD` inside its own time limit. This script is that rule, run.
It exists because the rule is the kind of claim that is easy to state and easy
to leave unchecked, and because PRE-REGISTRATION.md quotes its output.

Why an offset can fail at all: drift is a seeded random walk, so a realisation
can wander back toward the truth and never cross. A drifted scenario whose
error never crosses the threshold is a scenario where drift is nominally on and
functionally absent -- the treatment did not get applied, and the episode
silently becomes a fourth `navigate_to_goal`.

Why choosing offsets this way is not seed-shopping: **drift is injected
open-loop and is bit-identical across conditions.** `run_episode.py` records
`drift.estimate()` straight into the trace and `Scenario.drift_rng()` is a
separate stream from the controller's, so no controller can influence the
realisation and no choice of offset can favour any condition. See
`plans/done/14-first-cards.md` and `cards/store.py`'s CONTROLLER_INDEPENDENT.

    python scripts/check_seed_offsets.py             # check the registered offsets
    python scripts/check_seed_offsets.py --scan 10   # show the first 10 candidates

Prints SHAPE ONLY -- counts and fractions, never a scenario id, start or goal --
so its output is safe to paste into a commit message. Same discipline as
`scripts/seal_split.py`, and for the same reason: the sealed split must not
reach a model's context.

Sim-free. No simulator, no GPU.
"""

import argparse
import dataclasses
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.drift import make_drift                     # noqa: E402
from core.metrics import LOC_FAIL_THRESHOLD           # noqa: E402
from core.scenario import _held_out                   # noqa: E402
from core.stats import SEED_OFFSETS                   # noqa: E402


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# Integration step for the random walk. Matches the runner's control step, which
# is the rate the walk is actually advanced at in an episode -- a coarser step
# would integrate a different process.
DT = 0.02

# The granularity SEED_OFFSETS is drawn from. Large enough that two replicates
# of the same scenario cannot collide with each other or with any seed already
# in use (dev 1001-1003, held-out 2000-2011).
OFFSET_STRIDE = 10000


# ---------------------------------------------------------------------------


def crossings(offset, drifted):
    """Fraction of each scenario's time limit at which the error first crosses.

    None where it never crosses inside the limit.
    """
    out = []
    for s in drifted:
        sc = dataclasses.replace(s, seed=s.seed + offset)
        drift = make_drift(sc)
        t, first = 0.0, None
        while t < sc.time_limit:
            drift.advance(DT)
            t += DT
            if drift.true_error() > LOC_FAIL_THRESHOLD:
                first = t
                break
        out.append(None if first is None else first / sc.time_limit)
    return out


def report(offset, fr):
    ok = [f for f in fr if f is not None]
    status = "ok  " if len(ok) == len(fr) else "MISS"
    span = (f"earliest {min(ok):>4.0%}, latest {max(ok):>4.0%} of the limit"
            if ok else "never crosses")
    return f"  offset {offset:>6}  {status}  {len(ok)}/{len(fr)} crossed, {span}"


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan", type=int, default=0, metavar="K",
                    help="show the first K candidate offsets rather than only "
                         "the registered ones, to show how the rule selected")
    args = ap.parse_args()

    drifted = [s for s in _held_out() if s.drift is not None]
    print(f"[seeds] {len(drifted)} drifted held-out scenarios, "
          f"threshold {LOC_FAIL_THRESHOLD} m, dt {DT} s")

    if args.scan:
        print(f"[seeds] scanning the first {args.scan} multiples of {OFFSET_STRIDE}:")
        chosen = []
        for k in range(args.scan):
            off = k * OFFSET_STRIDE
            fr = crossings(off, drifted)
            print(report(off, fr))
            if all(f is not None for f in fr) and len(chosen) < len(SEED_OFFSETS):
                chosen.append(off)
        print(f"[seeds] the rule selects {tuple(chosen)}; "
              f"core.stats.SEED_OFFSETS is {SEED_OFFSETS}")
        return 0 if tuple(chosen) == tuple(SEED_OFFSETS) else 1

    print(f"[seeds] registered offsets {SEED_OFFSETS}:")
    bad = 0
    for off in SEED_OFFSETS:
        fr = crossings(off, drifted)
        print(report(off, fr))
        bad += sum(1 for f in fr if f is None)
    if bad:
        print(f"[seeds] FAIL: {bad} drifted episodes where the manipulation "
              f"does not take")
        return 1
    print("[seeds] ok: drift bites in every drifted episode at every replicate")
    return 0


if __name__ == "__main__":
    sys.exit(main())
