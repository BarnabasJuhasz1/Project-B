"""
scripts/probe_citations.py

**The citations probe: does human extraction earn its place?**

Hand a model a citation and nothing else, take back the principle card it
reconstructs from memory, and diff it against the card a human wrote after
reading the paper.

    P=~/miniforge3/envs/env_isaacsim/bin/python

    # the plan: which citations, which have an answer key. Calls nothing.
    $P scripts/probe_citations.py --dry-run

    # one citation
    $P scripts/probe_citations.py --card limit_speed_dwa

    # everything with a verified span to check against
    $P scripts/probe_citations.py --verified-only

    # rewrite cards/probe/FINDINGS.md from what is already on disk
    $P scripts/probe_citations.py --report

WHAT IT MEASURES, AND WHAT IT DOES NOT
--------------------------------------
It measures **reconstruction fidelity**, which needs no simulator and no
episodes: whether the model, pointed at a paper, produces the mechanism, the
trigger, the derivation and — the part that matters — the *numbers* a human
found there. `cards/VERIFICATION.md` records which spans have been checked
against the document, so four of the five citations come with an answer key.

It does **not** measure whether an E-node navigates better than a D-node. That
would need a fifth condition, and `runs/spec.py` refuses one: #10's
identical-episode-count argument is stated over exactly four, so a fifth costs
an amendment to a tagged pre-registration plus a full extra sweep. This probe is
the part of that question worth having for zero episodes; whether the rest is
worth 45 is a decision to take WITH this evidence rather than without it.

⚠️ A reconstructed card is **never** a real card. Everything written here lands
in `cards/probe/`, which `cards.store` refuses to load.

SIM-FREE.
"""

import argparse
import json
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Reconstruct principle cards from citations alone.")
    parser.add_argument("--card", action="append", dest="card_ids", default=None,
                        help="repeatable; default is every card")
    parser.add_argument("--verified-only", action="store_true",
                        help="only citations whose span a human has checked — "
                             "the ones with an answer key")
    parser.add_argument("--backend", default="claude-cli",
                        choices=("claude-cli", "anthropic-sdk", "replay"))
    parser.add_argument("--model", default=None, help="model id override")
    parser.add_argument("--force", action="store_true",
                        help="re-probe a citation already reconstructed. "
                             "⚠ Overwrites the recorded reconstruction.")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the plan and call nothing")
    parser.add_argument("--report", action="store_true",
                        help="rewrite FINDINGS.md from what is on disk; "
                             "calls nothing")
    args = parser.parse_args(argv)

    from cards.store import CardStore
    from generate import backends, probe
    from generate.probe import PROBE_DIR

    cards = CardStore.load()
    if args.card_ids:
        known = {c.id for c in cards}
        unknown = sorted(set(args.card_ids) - known)
        if unknown:
            parser.error(f"unknown card(s) {unknown}. Known: {sorted(known)}")

    selected = [c for c in cards
                if (not args.card_ids or c.id in args.card_ids)
                and (not args.verified_only or c.is_traceable)]

    if args.report:
        results = probe.load_results(PROBE_DIR)
        if not results:
            print("[probe] nothing on disk yet.", file=sys.stderr)
            return 1
        path = probe.write_findings(results, PROBE_DIR)
        print(f"[probe] wrote {path}")
        return 0

    print(f"[probe] {len(selected)} citation(s), backend={args.backend}")
    for card in selected:
        done = (PROBE_DIR / f"{card.id}.yaml").exists()
        key = "answer key" if card.is_traceable else "NO ANSWER KEY"
        state = "done" if done and not args.force else "to run"
        print(f"  {card.id:<26} {key:<14} {state}")
        print(f"    {card.source.citation.strip().splitlines()[0]}")

    if args.dry_run:
        print("[probe] --dry-run: nothing called.")
        return 0

    todo = [c for c in selected
            if args.force or not (PROBE_DIR / f"{c.id}.yaml").exists()]
    if not todo:
        print("[probe] nothing to do; --force would overwrite.")
        return 0

    kwargs = ({} if args.backend == "replay"
              else {"model_id": args.model or backends.DEFAULT_MODEL})
    backend = backends.make(args.backend, **kwargs)

    results, failed = [], 0
    for i, card in enumerate(todo, 1):
        label = f"[{i}/{len(todo)}] {card.id}"
        try:
            result, reconstructed = probe.probe(
                card, backend, record=(args.backend != "replay"))
        except Exception as exc:                       # noqa: BLE001
            failed += 1
            print(f"{label} FAILED: {type(exc).__name__}: {exc}",
                  file=sys.stderr)
            continue
        probe.write_result(result, reconstructed, PROBE_DIR)
        results.append(result)
        if not result["reconstructed"]:
            print(f"{label} no card returned — {result.get('note', '')}")
            continue
        print(f"{label} {result['exact_match_count']}/"
              f"{result['exact_field_count']} exact fields, "
              f"span {'offered' if result['span']['model_offered_one'] else 'empty'}"
              f"{'' if result['span']['has_answer_key'] else ' (no key)'}")

    on_disk = probe.load_results(PROBE_DIR)
    path = probe.write_findings(on_disk, PROBE_DIR)
    print(f"[probe] {len(results)} probed, {failed} failed. {path}")
    print("[probe] ⚠ a reconstructed span is a GUESS until someone opens "
          "the paper. FINDINGS.md says which.")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
