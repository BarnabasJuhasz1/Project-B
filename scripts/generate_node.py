"""
scripts/generate_node.py

One generation call: prompt a model for one behaviour-tree leaf, stage it, and
let the harness decide whether it is admissible. No human in the loop.

    P=~/miniforge3/envs/env_isaacsim/bin/python

    # condition B: one card, one node
    $P scripts/generate_node.py --condition b --card limit_speed_dwa

    # condition A and condition D: no card, the task specification is the input
    $P scripts/generate_node.py --condition a --task navigate_to_goal
    $P scripts/generate_node.py --condition d --task search_for_goal

    # re-run a recorded generation with no model, no key and no network
    $P scripts/generate_node.py --condition b --card limit_speed_dwa --backend replay

    # what G1 reports: unit pass rate per condition, with rejection reasons
    $P scripts/generate_node.py --pass-rate

SIM-FREE. No Isaac Sim, no GPU, no scene: the unit scenario is a micro-world
rolled out kinematically. **A node admitted here may still fail under physics** —
`harness/micro.py` says so at length, and the admission gate is a necessary
condition, never a sufficient one.

`--backend replay` is the reproducible path and the one a container run should
use: it replays the committed transcript for this exact prompt instead of
calling a model, so a library can be rebuilt byte-for-byte from files. See
`generate/backends.py` on why that replaces temperature-0 determinism rather
than supplementing it.
"""

import argparse
import json
import os
import sys

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
        description="Generate one behaviour-tree leaf and judge it.")
    parser.add_argument("--condition", type=_condition,
                        help="a, b or d. B takes a card; A and D do not.")
    parser.add_argument("--card", default=None,
                        help="card id, for condition B. One card per call (§4).")
    parser.add_argument("--card-file", default=None,
                        help="load a card by PATH instead of by id. For the "
                             "deliberately-bad fixture, which must not live in "
                             "cards/ where G1 would count it.")
    parser.add_argument("--task", type=_task, default="navigate_to_goal",
                        help="the task family the node must be useful for")
    parser.add_argument("--backend", default="claude-cli",
                        choices=("claude-cli", "anthropic-sdk", "replay"))
    parser.add_argument("--model", default=None, help="model id override")
    parser.add_argument("--node-id", default=None)
    parser.add_argument("--seeds", type=int, nargs="*", default=None,
                        help="unit-scenario seeds; only matters when something "
                             "in the rollout is actually stochastic")
    parser.add_argument("--pass-rate", action="store_true",
                        help="report G1's unit pass rate per condition and exit")
    args = parser.parse_args(argv)

    from generate import backends
    from generate.admit import admit, pass_rate
    from generate.generator import generate

    if args.pass_rate:
        print(json.dumps(pass_rate(), indent=2, sort_keys=True))
        return 0

    if args.condition is None:
        parser.error("--condition is required (or use --pass-rate)")

    if args.card and args.card_file:
        parser.error("--card and --card-file name the same thing two ways")

    card = None
    if args.card or args.card_file:
        from cards.store import CardStore, load_card
        card = (load_card(args.card_file) if args.card_file
                else CardStore.load().get(args.card))
        if args.condition != "b":
            parser.error(
                f"--card with condition {args.condition}: only B is given a "
                f"card. A card reaching A or D would void the ablation.")
    elif args.condition == "b":
        parser.error("condition b needs --card: the card IS the independent "
                     "variable")

    kwargs = {} if args.backend == "replay" else {"model_id": args.model
                                                  or backends.DEFAULT_MODEL}
    backend = backends.make(args.backend, **kwargs)

    print(f"[generate] condition={args.condition} task={args.task} "
          f"card={args.card or '-'} backend={args.backend}")

    candidate = generate(args.condition, args.task, backend, card=card,
                         node_id=args.node_id,
                         record=(args.backend != "replay"))
    if candidate.source is None:
        print(f"[generate] staged {candidate.node_id} — NO CODE EMITTED, "
              f"{len(candidate.response.text)} chars of prose "
              f"(prompt {candidate.prompt_hash})")
    else:
        print(f"[generate] staged {candidate.node_id} "
              f"({len(candidate.source)} chars, prompt {candidate.prompt_hash})")

    result = admit(candidate.metadata(), candidate.source, card=card,
                   seeds=args.seeds)
    stem = candidate.stem
    from generate.generator import STAGING
    (STAGING / f"{stem}.result.json").write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8")

    if result.admitted:
        u = result.unit
        print(f"[generate] ADMITTED — {u.metric}: candidate {u.candidate:.3f} "
              f"vs baseline {u.baseline:.3f}, effect {u.effect:.3f} "
              f"(required {u.required:.3f})")
    else:
        print(f"[generate] REJECTED — {result.reason}")
        print(f"[generate]   {result.detail}")
    print(f"[generate] record: generate/staging/{stem}.result.json")
    return 0 if result.admitted else 1


if __name__ == "__main__":
    raise SystemExit(main())
