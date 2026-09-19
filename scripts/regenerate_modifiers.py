#!/usr/bin/env python
"""
scripts/regenerate_modifiers.py

**Re-make the NODE call, and only the node call.** Chunk 52's regeneration.

The layered command pipeline changed what a generated node *is*: a leaf that
drives and silences everything below it became a modifier that revises the
shared base's command. Every v2 candidate was written against the old contract,
so every one of them has to be written again.

**The cards are NOT regenerated.** They are reused byte for byte from
`generate/cards/<condition>/`, and that is the point rather than a saving: the
card call is the independent variable, and rewriting the cards at the same time
as the execution model would leave any difference attributable to neither. So
this script walks the committed cards, renders `generate/prompts/modifier.md`
against each, makes ONE model call per card, stages the candidate and admits it.

    cards written by the arms   →  UNCHANGED, reused
    the node call               →  re-made, against the modifier prompt

⚠️ **ONE MODEL CALL PER CARD, AND THEY ARE NOT FREE.** `--dry-run` prints the
plan and calls nothing. The backend carries `backends.CLI_BUDGET_USD` as a
runaway backstop — the D arm once spent $3.71 of thinking tokens and returned no
code, which is the failure this budget exists to bound.

⚠️ Resumes from `generate/staging/`, and **never retries a rejection**. A
rejection is an outcome with a denominator, exactly as in `generate/batch.py`.

Examples
--------
    P=~/miniforge3/envs/env_isaacsim/bin/python

    $P scripts/regenerate_modifiers.py --dry-run
    $P scripts/regenerate_modifiers.py --condition a
    $P scripts/regenerate_modifiers.py --card b__reorient_geometry__dc50c33027c308a3
    $P scripts/regenerate_modifiers.py                      # the whole library
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Re-make the node call for every committed card.")
    p.add_argument("--condition", action="append", dest="conditions",
                   help="repeatable; default is every arm that has cards")
    p.add_argument("--card", action="append", dest="stems",
                   help="repeatable; a committed card's stem, e.g. "
                        "b__reorient_geometry__dc50c33027c308a3")
    p.add_argument("--backend", default="claude-cli",
                   help="claude-cli (default) or replay")
    p.add_argument("--dry-run", action="store_true",
                   help="print the plan and call nothing")
    p.add_argument("--retry-failed", action="store_true",
                   help="also re-attempt rows whose last call FAILED. A call "
                        "failure is not a rejection — generator.py records one "
                        "precisely so the row stays pending rather than looking "
                        "like a call nobody made — but retrying costs money, so "
                        "it is an explicit decision rather than the default. "
                        "⚠️ A second failure overwrites the first record.")
    p.add_argument("--limit", type=int, default=None,
                   help="stop after this many calls. A cost lever.")
    return p.parse_args(argv)


# Everything above this line is code; below it is data, not code.
# ---------------------------------------------------------------------------

# What a row can be, before anything is called.
PENDING = "pending"          # no candidate staged; this one costs a call
DONE = "done"                # already staged under the current pipeline
FAILED = "failed"            # a recorded call failure; re-running it costs again


def _rows(cards_dir, staging, conditions=None, stems=None):
    """Every committed card, with what is already on disk for it."""
    from generate import cardgen

    rows = []
    for condition_dir in sorted(cards_dir.iterdir()):
        if not condition_dir.is_dir():
            continue
        condition = condition_dir.name
        if conditions and condition not in conditions:
            continue
        for path in sorted(condition_dir.glob("*.yaml")):
            stem = path.stem
            if stems and stem not in stems:
                continue
            generated = cardgen.load_generated(path)
            if (staging / f"{stem}.py").exists() or \
                    (staging / f"{stem}.refusal.md").exists():
                state = DONE
            elif (staging / f"{stem}.error.json").exists():
                state = FAILED
            else:
                state = PENDING
            rows.append((stem, condition, generated, state))
    return rows


def main(argv=None):
    args = _parse_args(argv)

    from generate import backends, cardgen
    from generate.admit import admit_staged
    from generate.generator import (PIPELINE_VERSION, NODE_PROMPT, STAGING,
                                    node_call, render_node)

    staging = Path(STAGING)
    staging.mkdir(parents=True, exist_ok=True)
    rows = _rows(Path(cardgen.CARDS_DIR), staging,
                 conditions=args.conditions, stems=args.stems)

    print(f"[regen] pipeline={PIPELINE_VERSION}  prompt={NODE_PROMPT.name}")
    print(f"[regen] {len(rows)} committed card(s); the CARD call is not re-made")
    print(f"[regen] {'stem':50} {'arm':4} {'task':22} state")
    for stem, condition, generated, state in rows:
        print(f"[regen] {stem:50} {condition:4} {generated.task:22} {state}")

    wanted = {PENDING} | ({FAILED} if args.retry_failed else set())
    todo = [r for r in rows if r[3] in wanted]
    if args.limit is not None:
        todo = todo[:args.limit]
    print(f"\n[regen] {len(todo)} call(s) to make")

    if args.dry_run:
        print("[regen] --dry-run: nothing called")
        return 0
    if not todo:
        print("[regen] nothing to do")
        return 0

    backend = backends.make(args.backend)
    admitted = rejected = failed = 0
    for stem, condition, generated, _state in todo:
        # The corpus entry this arm's CARD call was pointed at. A node built
        # from a citation and one built from the paper are different provenance
        # and `_node_metadata` refuses to merge them, so the distinction has to
        # survive the round trip through disk.
        treatment = cardgen.treatment(condition)
        ref = generated.ref_id
        card_id = ref if treatment == "paper" else None
        citation_id = ref if treatment == "citation" else None

        prompt = render_node(generated.card, generated.task)
        print(f"\n[regen] === {stem} ===  {len(prompt)} prompt chars")
        try:
            candidate = node_call(condition, generated.task, generated, backend,
                                  card_id=card_id, citation_id=citation_id,
                                  staging=staging)
        except backends.BackendError as exc:
            failed += 1
            print(f"[regen]   CALL FAILED: {exc}")
            print(f"[regen]   recorded as {stem}.error.json — the row stays "
                  f"pending and is NOT retried automatically")
            continue

        print(f"[regen]   staged {candidate.node_id} "
              f"({'code' if candidate.source else 'REFUSAL'})")
        result = admit_staged(stem, staging=staging)
        if result.admitted:
            admitted += 1
        else:
            rejected += 1
        print(f"[regen]   {result!r}  {result.detail[:160]}")
        fired = result.metadata.get("precondition_fired") or {}
        if fired:
            print(f"[regen]   ticks={fired.get('ticks')} "
                  f"fired={fired.get('fraction')} "
                  f"changed_command={fired.get('drove')}")

    print(f"\n[regen] {admitted} admitted, {rejected} rejected, "
          f"{failed} call failures")
    print("[regen] contribution is NOT measured here — run "
          "scripts/measure_growth.py, which is where `fixed` comes from.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
