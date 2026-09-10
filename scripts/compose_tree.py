"""
scripts/compose_tree.py

Compose a behaviour tree for one scenario, from the frozen node library.

Sim-free: no `isaacsim`, no GPU, no scene. It reads the library out of
`generate/staging/`, retrieves a working set, calls a model for a plan, builds
the tree and writes a composition record. The tree it produces is then run the
ordinary way, with no runner change and nothing registered by hand:

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P scripts/compose_tree.py --scenario doorway_traverse --backend replay
    $P scripts/compose_tree.py --scenario doorway_traverse --include-toys
    $P scripts/run_episode.py --scenario doorway_traverse \
        --controller composed:navigate_to_goal__goal_known__no_drift__<hash>

    # what is composable right now, and what the library holds
    $P -c "import controllers; print(controllers.compositions())"

NON-NEGOTIABLE #6: this path never sees a card. It renders port contracts and a
task specification, and `compose/tests/test_no_papers.py` greps the rendered
prompt against every card in the store to prove it.

⚠️ Building the library EXECUTES MODEL-WRITTEN PYTHON with no sandbox — the same
warning `generate/admit.py` carries. See compose/library.py.
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from compose.compose import compose                      # noqa: E402
from compose.library import load_library                 # noqa: E402
from compose import retrieve as retrieve_module          # noqa: E402
from compose.retrieve import retrieve                    # noqa: E402
from compose.spec import TaskSpec                        # noqa: E402
from core import scenario as scenario_defaults           # noqa: E402
from core.scenario import load as load_scenario          # noqa: E402
from generate import backends                            # noqa: E402


def _repair(args, parser):
    """Re-compose one recorded composition against the same retrieval."""
    from compose.compose import RECORDS, Composition, repair
    import json

    record = json.loads(
        (RECORDS / f"{args.repair_of}.json").read_text(encoding="utf-8"))
    record_conditions = record.get("conditions")
    library = load_library(include_toys=record.get("include_toys", False),
                           conditions=record_conditions)
    if library.snapshot() != record["library_snapshot"]:
        parser.error(
            f"the record names {record['library_snapshot']} and the library "
            f"loads as {library.snapshot()}. A repair must rewire the SAME "
            f"library, or it is not a repair.")

    retrieval = retrieve_module.Retrieval(
        entries=[library.get(n) for n in record["retrieval"]["retrieved"]],
        query_tags=tuple(record["retrieval"]["query_tags"]),
        library_size=record["retrieval"]["library_size"])
    previous = Composition(
        composition_id=args.repair_of,
        spec=TaskSpec(**record["spec"]), retrieval=retrieval,
        snapshot=record["library_snapshot"], prompt="", response=None,
        include_toys=record.get("include_toys", False),
        conditions=tuple(record_conditions) if record_conditions else None)

    backend = backends.make(args.backend, model_id=args.model)
    result = repair(previous, args.failure, backend, library=library)
    print(f"[compose] {result.composition_id}: {result.reason}"
          + (f" — {result.detail}" if result.detail else ""))
    print(f"[compose] repair of {args.repair_of}, rewired from "
          f"{list(retrieval.node_ids)} — no primitive created")
    if result.ok:
        print(f"[compose] {result.tree!r}")
        print(f"[compose] rationale: {result.plan.get('rationale', '')}")
        print(f"[compose] run it: --controller composed:{result.composition_id}")
    return 0 if result.ok else 1


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Compose a behaviour tree from the frozen node library.")
    parser.add_argument("--scenario", help="scenario to build a TaskSpec from")
    # The axes, for composing against a task FAMILY rather than an episode.
    # This is how a held-out family is composed for without naming a held-out
    # scenario: a TaskSpec is the 2 x 2 cell plus the sensor configuration, and
    # chunk 11's split has three scenarios per cell.
    parser.add_argument("--family", help="compose from the axes, not a scenario")
    parser.add_argument("--goal-hidden", action="store_true",
                        help="with --family: the goal is not disclosed at t=0")
    parser.add_argument("--drift", action="store_true",
                        help="with --family: the pose estimate degrades")
    parser.add_argument(
        "--backend", default="claude-cli",
        choices=("claude-cli", "anthropic-sdk", "replay"),
        help="replay reproduces a committed transcript with no model call")
    parser.add_argument("--model", default=backends.DEFAULT_MODEL)
    parser.add_argument(
        "--condition", action="append", dest="conditions", default=None,
        metavar="LETTER",
        help="compose over ONE ARM's nodes only, repeatable. This is the "
             "partition G2's ablation is made of: A and B share a task family "
             "and a card set, so an unpartitioned library lets condition A "
             "retrieve a node condition B wrote and B-A stops measuring "
             "anything. Omit for the whole library, which is right for "
             "exercising the path and wrong for a condition.")
    parser.add_argument(
        "--include-toys", action="store_true",
        help="add the handwritten fixture leaves to the library. NOT "
             "navigation and never a benchmark condition (#3) — this exists so "
             "the composition path can be exercised while the generated "
             "library is short.")
    parser.add_argument(
        "--repair-of", metavar="COMPOSITION_ID",
        help="re-compose after a failure, from the SAME library and the SAME "
             "retrieval. Rewiring only: the vocabulary is the previous "
             "retrieval's node ids, so a repair cannot create a primitive, "
             "which is what G3 needs.")
    parser.add_argument(
        "--failure", metavar="SIGNATURE",
        help="with --repair-of: what went wrong. Describes the failure, never "
             "the scenario.")
    parser.add_argument(
        "--dry-run", action="store_true",
        help="render the prompt and the retrieval and stop, without calling a "
             "model")
    parser.add_argument(
        "--retrieval", action="store_true",
        help="print what retrieval would offer, and stop")
    args = parser.parse_args(argv)

    if args.repair_of:
        if not args.failure:
            parser.error("--repair-of needs --failure: a repair with no "
                         "failure signature is just a second attempt, and the "
                         "log has to be able to tell them apart")
        return _repair(args, parser)

    conditions = tuple(args.conditions) if args.conditions else None
    library = load_library(include_toys=args.include_toys,
                           conditions=conditions)
    arms = "+".join(conditions) if conditions else "all arms"
    print(f"[compose] library {library.snapshot()} — {len(library)} nodes "
          f"({arms})")
    if conditions and len(library) <= 1:
        # Only ContractPorts survived: the arm has no admitted node. Composition
        # would still "succeed" and return a tree that drives nothing, which is
        # compose/FINDINGS.md's first finding arriving by a new route.
        parser.error(
            f"condition {arms} has no admitted nodes — the library is "
            f"{len(library)} entries and the only one is the harness's port "
            f"publisher. Composing would produce a tree that cannot drive. "
            f"Generate for this arm first.")

    if args.scenario and args.family:
        parser.error("--scenario and --family are two ways to build the same "
                     "TaskSpec; pass one")

    if not (args.scenario or args.family):
        for entry in library.all():
            print(f"[compose]   {entry.node_id} "
                  f"({entry.node_cls.KIND.value})")
        return 0

    if args.scenario:
        spec = TaskSpec.from_scenario(load_scenario(args.scenario))
    else:
        spec = TaskSpec(
            family=args.family, goal_known=not args.goal_hidden,
            pose_drifts=args.drift, n_beams=scenario_defaults.N_BEAMS, fov=scenario_defaults.FOV,
            range_max=scenario_defaults.RANGE_MAX,
            detect_radius=scenario_defaults.DETECT_RADIUS,
            max_lin=scenario_defaults.MAX_LIN,
            max_ang=scenario_defaults.MAX_ANG)
    found = retrieve(library, spec)
    print(f"[compose] task {spec.family} {list(spec.axes)}")
    for node_id in found.node_ids:
        print(f"[compose]   retrieved {node_id} — {found.reasons[node_id]}")
    if found.orphan_ports():
        print(f"[compose]   ORPHAN PORTS {found.orphan_ports()} — written by a "
              f"retrieved node, read by none of them; whichever node writes "
              f"one is inert in any tree built from this set")
    if found.unsatisfiable:
        print(f"[compose]   UNSATISFIABLE PORTS {found.unsatisfiable} — the "
              f"library lacks a producer, which is a library gap and not a "
              f"retrieval miss")
    if args.retrieval:
        return 0

    if args.dry_run:
        from compose.compose import render
        print(render(spec, found))
        return 0

    backend = backends.make(args.backend, model_id=args.model)
    result = compose(spec, library, backend, retrieval=found,
                     include_toys=args.include_toys, conditions=conditions)
    print(f"[compose] {result.composition_id}: {result.reason}"
          + (f" — {result.detail}" if result.detail else ""))
    if result.ok:
        print(f"[compose] {result.tree!r}")
        print(f"[compose] rationale: {result.plan.get('rationale', '')}")
        print(f"[compose] run it: --controller composed:{result.composition_id}")
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
