"""
generate/batch.py

Generation at scale. `generate/generator.py` makes one call; this enumerates
every call the pipeline can reach and drives them.

The grid is a **pure function of the frozen inputs** — `CONDITIONS`, `TASKS` and
the card store — in the same way `runs/design.py` derives the episode set from
the pre-registered constants. That is what lets `--dry-run` be the real code
path minus the model call, and what lets the plan be diffed between runs.

WHY IT SKIPS RATHER THAN RETRIES
--------------------------------
`generate/staging/` IS the state. There is no progress file, because a progress
file can disagree with the artefacts it describes — chunk 28 found precisely
that in the episode sweep, where five episodes were recorded `ok` having written
nothing. A combination with a staged record is skipped whatever its outcome:
**re-running a rejection until it is admitted produces a pass rate with no
denominator**, and the pass rate is what G1 reports.

The ceiling this enumerates is not a budget. `render()` is a pure function of
`(condition, task, card)` and the node id IS the prompt hash, so the reachable
library is |TASKS| for A and D and |cards| x |TASKS| for B — asserted in
`generate/tests/test_ceiling.py`, and the reason plans/todo/29 exists.

SIM-FREE.
"""

import json
import time

from cards.store import CardStore
from generate import backends
from generate.generator import CONDITIONS, STAGING, TASKS, render


class BatchError(ValueError):
    pass


def plan(conditions=None, tasks=None, card_ids=None, cards=None, staging=None):
    """The full grid, as data, before anything is called.

    One row per reachable combination: the condition, the task, the card (B
    only), the prompt hash — which is both the node id and the staging stem —
    and what is already on disk for it. Deterministic order, so two plans are
    diffable.
    """
    staging = staging or STAGING
    cards = cards if cards is not None else CardStore.load()
    conditions = list(conditions or CONDITIONS)
    tasks = list(tasks or TASKS)
    selected = [c for c in cards if not card_ids or c.id in card_ids]

    rows = []
    for condition in conditions:
        for task in tasks:
            for card in (selected if condition == "b" else [None]):
                prompt_hash = backends.prompt_hash(render(condition, task, card))
                stem = f"{condition}__{card.id if card else task}__{prompt_hash}"
                outcome = _read_outcome(staging / f"{stem}.result.json")
                rows.append({
                    "condition": condition,
                    "task": task,
                    "card_id": card.id if card else None,
                    "prompt_hash": prompt_hash,
                    "node_id": f"gen_{condition}_{prompt_hash}",
                    "stem": stem,
                    "staged": (staging / f"{stem}.json").exists(),
                    "judged": outcome is not None,
                    "admitted": outcome.get("admitted") if outcome else None,
                    "reason": outcome.get("reason") if outcome else None,
                })
    return rows


def _read_outcome(path):
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        # A record that cannot be read is not a combination that can be
        # re-run: it is already staged, and overwriting it would destroy
        # whatever G1's denominator was supposed to contain.
        return {"admitted": None, "reason": "UNREADABLE_RECORD"}


def collisions(rows):
    """Combinations sharing a stem — one generation would silently overwrite
    another. Impossible with today's inputs, and checked rather than assumed,
    because the failure is invisible: the second write just wins."""
    seen, clashes = {}, []
    for row in rows:
        if row["stem"] in seen:
            clashes.append((seen[row["stem"]], row))
        seen[row["stem"]] = row
    return clashes


def pending(rows, force=False, limit=None):
    """What would actually run. `force` overwrites recorded outcomes, which is
    why it is not the default and why the caller is expected to say so."""
    todo = [r for r in rows if force or not r["staged"]]
    return todo[:limit] if limit is not None else todo


def describe(row):
    if row["judged"]:
        state = "ADMITTED" if row["admitted"] else f"rejected/{row['reason']}"
    elif row["staged"]:
        state = "staged, unjudged"
    else:
        state = "PENDING"
    return (f"  {row['condition']}  {row['task']:<22} "
            f"{row['card_id'] or '-':<26} {row['prompt_hash']}  {state}")


def run(rows, backend, cards=None, seeds=None, record=True, staging=None,
        on_event=None):
    """Generate and judge each row. Returns a summary dict.

    A row that raises is counted and skipped, never fatal: the remaining
    combinations are independent of it, and abandoning them would leave an
    unbalanced grid for nothing.
    """
    from generate.admit import admit
    from generate.generator import generate

    staging = staging or STAGING
    cards = cards if cards is not None else CardStore.load()
    say = on_event or (lambda *a, **k: None)

    started = time.time()
    summary = {"admitted": 0, "rejected": 0, "failed": 0, "results": []}
    for i, row in enumerate(rows, 1):
        card = cards.get(row["card_id"]) if row["card_id"] else None
        try:
            candidate = generate(row["condition"], row["task"], backend,
                                 card=card, record=record)
            result = admit(candidate.metadata(), candidate.source, card=card,
                           seeds=seeds)
            (staging / f"{candidate.stem}.result.json").write_text(
                json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
                encoding="utf-8")
        except Exception as exc:                       # noqa: BLE001
            summary["failed"] += 1
            summary["results"].append(dict(row, outcome="failed",
                                           error=f"{type(exc).__name__}: {exc}"))
            say("failed", i, len(rows), row, error=exc)
            continue
        key = "admitted" if result.admitted else "rejected"
        summary[key] += 1
        summary["results"].append(dict(row, outcome=key,
                                       reason=result.reason,
                                       node_id=result.node_id))
        say(key, i, len(rows), row, result=result)
    summary["elapsed_s"] = time.time() - started
    return summary
