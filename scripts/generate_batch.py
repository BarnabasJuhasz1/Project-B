"""
scripts/generate_batch.py

Generation at scale: enumerate every (condition, task, card) combination the
pipeline can reach, generate the ones that are not already staged, and record
what happened to each. `scripts/generate_node.py` is one call; this is the sweep.

    P=~/miniforge3/envs/env_isaacsim/bin/python

    # the plan. Calls nothing, costs milliseconds, prints the full grid.
    $P scripts/generate_batch.py --dry-run

    # one condition, replaying committed transcripts: no model, no key, no network
    $P scripts/generate_batch.py --condition b --backend replay

    # the real thing, for one condition, capped
    $P scripts/generate_batch.py --condition d --limit 3

    # what G1 reports, afterwards
    $P scripts/generate_node.py --pass-rate

WHY IT REFUSES MORE THAN IT RUNS
--------------------------------
Three properties, each of which is a defect this script exists to make
impossible rather than a convenience:

1. **Resumable from `generate/staging/`, never from a state file.** The staged
   record IS the state. A separate progress file can disagree with the artefacts
   it describes, and chunk 28 found exactly that failure in the episode sweep —
   five episodes recorded `ok` that had written nothing.

2. **A rejection is never retried.** Re-running a combination until it is
   admitted produces a pass rate with no denominator, which is the one number
   G1 asks for. A staged combination is SKIPPED whatever its outcome; `--force`
   is the only way past, and it says so in the log.

3. **It refuses before the first model call, not during.** The grid, the
   collisions and the skips are computed up front, so `--dry-run` is the same
   code path as the real run minus the call.

⚠️ Generation EXECUTES MODEL-WRITTEN PYTHON through `generate/admit.py`, with an
import allowlist and no sandbox. That is admit.py's decision, documented there;
this script only calls it more often.

SIM-FREE. The unit scenario is a micro-world rolled out kinematically, so
nothing here needs Isaac Sim, a GPU or a scene — and an admitted node may still
fail under physics.
"""

import argparse
import json
import os
import sys
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def _condition(value):
    from generate.generator import CONDITIONS
    value = value.lower()
    if value not in CONDITIONS:
        raise argparse.ArgumentTypeError(
            f"{value!r} is not a generated condition. {list(CONDITIONS)} — C is "
            f"an off-the-shelf Nav2 stack and is not generated at all.")
    return value


def _task(value):
    from generate.generator import TASKS
    if value not in TASKS:
        raise argparse.ArgumentTypeError(
            f"{value!r} is not a task family. Known: {sorted(TASKS)}")
    return value


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Generate every reachable node for one or more conditions.")
    parser.add_argument("--condition", type=_condition, action="append",
                        dest="conditions", default=None,
                        help="repeatable; default is every generated condition")
    parser.add_argument("--task", type=_task, action="append", dest="tasks",
                        default=None, help="repeatable; default is every task")
    parser.add_argument("--card", action="append", dest="card_ids", default=None,
                        help="repeatable; condition B only. Default: every card")
    parser.add_argument("--backend", default="replay",
                        choices=("claude-cli", "anthropic-sdk", "replay"),
                        help="DEFAULT IS REPLAY. A batch against a live model "
                             "costs money and is not reproducible; ask for it "
                             "explicitly.")
    parser.add_argument("--model", default=None, help="model id override")
    parser.add_argument("--seeds", type=int, nargs="*", default=None)
    parser.add_argument("--limit", type=int, default=None,
                        help="stop after this many generations. The budget lever.")
    parser.add_argument("--force", action="store_true",
                        help="re-run combinations that are already staged. "
                             "\u26a0 This overwrites a recorded outcome and makes "
                             "the pass rate a number with no denominator.")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the grid and what would run; call nothing")
    args = parser.parse_args(argv)

    from cards.store import CardStore
    from generate import backends, batch

    cards = CardStore.load()
    if args.card_ids:
        known = {c.id for c in cards}
        unknown = sorted(set(args.card_ids) - known)
        if unknown:
            parser.error(f"unknown card(s) {unknown}. Known: {sorted(known)}")

    rows = batch.plan(args.conditions, args.tasks, args.card_ids, cards=cards)

    clashes = batch.collisions(rows)
    if clashes:
        for first, second in clashes:
            print(f"[batch] STEM COLLISION: {first['stem']} is produced by "
                  f"{first['condition']}/{first['task']}/{first['card_id']} "
                  f"and {second['condition']}/{second['task']}/"
                  f"{second['card_id']}", file=sys.stderr)
        print("[batch] refusing: one generation would overwrite another.",
              file=sys.stderr)
        return 2

    todo = batch.pending(rows, force=args.force, limit=args.limit)
    todo_stems = {r["stem"] for r in todo}

    print(f"[batch] grid: {len(rows)} combinations "
          f"({len(rows) - len(todo_stems)} skipped, {len(todo)} to run)")
    for row in rows:
        print(("->" if row["stem"] in todo_stems else "  ") + batch.describe(row))

    if args.dry_run:
        print("[batch] --dry-run: nothing called.")
        return 0
    if not todo:
        print("[batch] nothing to do. Every combination is already staged; "
              "--force would overwrite the record.")
        return 0

    kwargs = ({} if args.backend == "replay"
              else {"model_id": args.model or backends.DEFAULT_MODEL})
    backend = backends.make(args.backend, **kwargs)

    def say(kind, i, total, row, result=None, error=None):
        label = (f"[{i}/{total}] {row['condition']}/{row['task']}"
                 f"/{row['card_id'] or '-'}")
        if kind == "failed":
            print(f"{label} FAILED: {type(error).__name__}: {error}",
                  file=sys.stderr)
        elif kind == "admitted":
            print(f"{label} ADMITTED {result.node_id}")
        else:
            print(f"{label} rejected \u2014 {result.reason}")

    summary = batch.run(todo, backend, cards=cards, seeds=args.seeds,
                        record=(args.backend != "replay"), on_event=say)

    print(f"[batch] {summary['admitted']} admitted, {summary['rejected']} "
          f"rejected, {summary['failed']} failed in "
          f"{summary['elapsed_s']:.0f}s")
    print("[batch] pass rate: scripts/generate_node.py --pass-rate")
    return 0 if summary["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
