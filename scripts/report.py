"""
scripts/report.py

The scored sweep, read as the pre-registration says to read it.

    python scripts/report.py                                   # held-out, the headline
    python scripts/report.py --split dev                       # descriptive
    python scripts/report.py --spec runs/specs/rehearsal.json --split dev \
        --results-dir out/rehearsal --conditions A B

Refuses an incomplete set. That is the point of it: non-negotiable #10 requires
identical episode counts across conditions, so a report over "whatever finished"
would be a comparison between two different benchmarks, and it would look
exactly like a valid one.

Sim-free -- reads committed JSON and calls `core.stats.decide()`. Nothing here
re-derives the decision rule; `core/stats.py` IS the rule and this prints it.
"""

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from runs.aggregate import Incomplete, collect, report        # noqa: E402
from runs.design import DEV, HELD_OUT, sweep_design           # noqa: E402
from runs.spec import DEFAULT_SPEC, load_spec                 # noqa: E402

parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--spec", default=DEFAULT_SPEC)
parser.add_argument("--split", choices=[HELD_OUT, DEV], default=HELD_OUT,
                    help="held_out is the headline; dev is descriptive and is "
                         "labelled as the set the prompts were iterated against")
parser.add_argument("--results-dir", default="results")
parser.add_argument("--conditions", nargs=2, default=["B", "D"],
                    metavar=("B", "D"),
                    help="which pair the decision rule is applied to. B D is "
                         "the headline contrast and the default; B A is "
                         "confounded with principle articulation and is "
                         "reported as such (PRE-REGISTRATION.md §1)")
args = parser.parse_args()


def main():
    spec = load_spec(args.spec)
    plan = sweep_design(spec, splits=(args.split,))
    try:
        collected = collect(spec, plan, results_dir=args.results_dir,
                            split=args.split)
    except Incomplete as exc:
        print(f"[report] refusing to report.\n{exc}", file=sys.stderr)
        return 2
    b, d = args.conditions
    have = set(collected.conditions)
    if not {b, d} <= have:
        print(f"[report] the decision rule needs conditions {b} and {d}; the "
              f"spec has {sorted(have)}. PRE-REGISTRATION.md §9: a missing "
              f"condition is reported as missing, never substituted for.",
              file=sys.stderr)
        # Not an error -- the per-condition table below is still worth printing.
        print(report(collected, b=b, d=d))
        return 1
    print(report(collected, b=b, d=d))
    return 0


if __name__ == "__main__":
    sys.exit(main())
