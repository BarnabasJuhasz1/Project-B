"""
scripts/run_sweep.py

THE EVALUATION ENTRY POINT. One spec, every episode, resumable.

    python scripts/run_sweep.py --dry-run
    python scripts/run_sweep.py --spec runs/specs/rehearsal.json --dry-run
    python scripts/run_sweep.py --spec runs/specs/rehearsal.json \
        --split dev --results-dir out/rehearsal --allow-unreportable
    python scripts/run_sweep.py --spec runs/specs/evaluation.json

Runs `scripts/run_episode.py` once per episode, in its own process, with
`--no-record`. Resumes from whatever is already in `--results-dir`, so an
interrupted sweep is re-run with the same command and picks up where it stopped.

**This does not boot the simulator itself.** Everything up to the first
subprocess is sim-free, which is why `--dry-run` costs milliseconds and why the
pre-flight refusal happens before 180 twenty-second boots rather than after.

Held-out scenario ids are **redacted** in the log unless `--reveal` is passed.
A sweep log is exactly the kind of output that gets pasted into a chat window,
and the sealed split reaching a model's context is the one failure sealing
cannot survive. Pass `--reveal` when you are debugging one episode and the
output is not going anywhere.

Not a benchmark unless the spec says so: `reportable: false` and the pre-flight
between them make it hard to produce a citable number by accident, and easy to
produce an honest rehearsal on purpose.
"""

import argparse
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

# argparse first, and everything it touches is sim-free -- same ordering rule as
# run_episode.py, for the same reason: a malformed flag must be rejected before
# anything expensive happens. Here the expensive thing is 180 simulator boots.
from runs.design import DEV, HELD_OUT, sweep_design          # noqa: E402
from runs.spec import DEFAULT_SPEC, load_spec                 # noqa: E402
from runs.sweep import (Unreportable, episode_command,        # noqa: E402
                        run_sweep, describe)

parser = argparse.ArgumentParser(
    description=__doc__,
    formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--spec", default=DEFAULT_SPEC,
                    help=f"condition -> controller binding (default {DEFAULT_SPEC})")
parser.add_argument("--split", choices=[HELD_OUT, DEV, "both"], default="both",
                    help="which scenarios. 'both' is what the pre-registration "
                         "specifies; 'dev' is for rehearsals")
parser.add_argument("--results-dir", default="results",
                    help="where scored results go AND where resume reads from. "
                         "Point a rehearsal somewhere under out/")
parser.add_argument("--dry-run", action="store_true",
                    help="print the plan and the exact per-episode command, "
                         "run nothing, boot nothing")
parser.add_argument("--allow-unreportable", action="store_true",
                    help="proceed even though the output may not be reported "
                         "(not in the container, dirty tree, no prereg tag, no "
                         "library snapshot). For rehearsals only")
parser.add_argument("--reveal", action="store_true",
                    help="print held-out scenario ids in full. Off by default; "
                         "see the module docstring")
parser.add_argument("--limit", type=int, default=None, metavar="N",
                    help="run only the first N episodes of the plan. For "
                         "smoke-testing the machinery, never for an "
                         "evaluation -- a truncated plan is unbalanced and "
                         "scripts/report.py will refuse it, which is correct")
parser.add_argument("--name", default=None,
                    help="name for the sweep log directory under out/sweeps/")
parser.add_argument("--python", default=None,
                    help="interpreter for the per-episode subprocess "
                         "(default: the one running this)")
args = parser.parse_args()


def main():
    spec = load_spec(args.spec)
    splits = (HELD_OUT, DEV) if args.split == "both" else (args.split,)
    plan = sweep_design(spec, splits=splits)

    print(f"[sweep] spec {spec.name!r}, "
          f"{'REPORTABLE' if spec.reportable else 'rehearsal (not reportable)'}")
    for c in spec.conditions:
        # The spec's own note wins over the canonical meaning. On a rehearsal
        # the labels are placeholders, and printing "A: naive LLM" next to
        # `goal_seeker` would be the log telling a comfortable lie.
        gloss = c.note or (c.meaning if spec.reportable else "")
        print(f"[sweep]   {c.label}: {c.controller}"
              + (f"  snapshot={c.library_snapshot}" if c.library_snapshot else "")
              + (f"  -- {gloss}" if gloss else ""))
    if args.limit:
        from runs.design import SweepPlan
        plan = SweepPlan(plan.episodes[:args.limit])
        print(f"[sweep] --limit {args.limit}: SMOKE TEST. The plan is "
              f"truncated and therefore unbalanced; report.py will refuse it.")
    print(f"[sweep] {len(plan)} episodes, per condition {plan.per_condition()}, "
          f"per split {plan.per_split()}")
    if not plan.is_balanced() and not args.limit:
        parser.error("the plan is not balanced across conditions, which "
                     "violates non-negotiable #10. This is a bug in "
                     "runs/design.py, not something to override.")
    print(f"[sweep] balanced across conditions; worst case "
          f"{plan.worst_case_sim_seconds():.0f}s of sim "
          f"({plan.worst_case_sim_seconds() / 3600:.1f}h), before the headless "
          f"ratio and the per-episode simulator boot")

    if args.dry_run:
        print("[sweep] --dry-run: nothing will be run. First three commands:")
        for episode in plan.episodes[:3]:
            cmd = episode_command(episode, spec, args.results_dir,
                                  python=args.python)
            shown = [os.path.relpath(c, REPO_ROOT) if c.startswith(REPO_ROOT)
                     else c for c in cmd]
            if not args.reveal and episode.split == HELD_OUT:
                shown[shown.index(episode.scenario)] = "<held-out id redacted>"
            print("           " + " ".join(shown))
        print(f"[sweep] ... and {len(plan) - 3} more. "
              f"{'Pass --reveal to print held-out ids.' if not args.reveal else ''}")
        return 0

    try:
        log = run_sweep(spec, plan, results_dir=args.results_dir,
                        allow_unreportable=args.allow_unreportable,
                        python=args.python, reveal=args.reveal,
                        name=args.name)
    except Unreportable as exc:
        print(f"[sweep] refusing to run.\n{exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    counts = log.counts()
    # A sweep with any failure is a sweep whose aggregation will refuse, so the
    # exit code says so rather than making someone read the log to find out.
    return 0 if not (counts.get("failed") or counts.get("timed_out")) else 1


if __name__ == "__main__":
    sys.exit(main())
