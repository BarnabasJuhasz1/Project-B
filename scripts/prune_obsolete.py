#!/usr/bin/env python
"""
scripts/prune_obsolete.py

**`DELETION-CANDIDATES.md` as code.** Chunk 52.

The layered command pipeline replaced the execution model, and a handful of
files became dead weight the moment it did. This deletes them — but only once
the new structure has been *shown* to work, because "we replaced it" and "the
replacement is better" are different claims and only the second one justifies
throwing anything away.

    P=~/miniforge3/envs/env_isaacsim/bin/python

    $P scripts/prune_obsolete.py            # what would go, and why. Deletes nothing.
    $P scripts/prune_obsolete.py --apply    # only if the gate below passes
    $P scripts/prune_obsolete.py --gate     # just the gate, nothing else

THE GATE
--------
1. `generate/staging/` holds at least one ADMITTED candidate stamped
   `pipeline_version: v3-layered`.
2. Some `out/growth/*.csv` shows **`fixed` > 0** for at least one node.

`fixed` was 0 in every row of growth round 1, across eleven nodes and both arms.
It is the number chunk 52 exists to move, so it is the number that decides
whether the thing it replaced may be deleted.

⚠️ Dry run by default, exactly like `scripts/tidy_out.py`, and for the same
reason: a script that deletes by default is a script that deletes something
somebody wanted.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Delete what chunk 52 obsoleted. Dry run by default.")
    p.add_argument("--apply", action="store_true",
                   help="actually delete. Refused unless the gate passes.")
    p.add_argument("--gate", action="store_true",
                   help="report the confirmation gate and stop")
    p.add_argument("--force", action="store_true",
                   help="delete even though the gate has not passed. Says so "
                        "loudly, and the reason belongs in the commit message.")
    return p.parse_args(argv)


# Everything above this line is code; below it is data, not code.
# ---------------------------------------------------------------------------

# TIER 1 — superseded by a recorded decision, read by nothing.
#
# Each entry is (path, why). The `why` is not decoration: it is what a reader of
# the deletion commit gets instead of the file, and a path with no reason beside
# it is a path somebody will restore.
TIER1 = [
    ("harness/ablate.py",
     "removal-based ablation. The user's decision, 2026-09-13: contribution is "
     "measured by ADDING a node to the tree, not by removing one. "
     "scripts/measure_growth.py is the addition-based measurement."),
    ("scripts/node_contribution.py",
     "the CLI over harness/ablate.py — leave-one-out, over lab42's dev "
     "scenarios, which stopped being the benchmark on 2026-09-13."),
    ("harness/tests/test_ablate.py",
     "tests for harness/ablate.py; they fail the moment it goes."),
    ("scripts/rejudge.py",
     "a one-shot migration that re-judged v2 staging under chunk 37's rule. It "
     "has been run, its output is committed, and that staging is now under "
     "generate/archive/v2-exclusive-control/."),
]

# TIER 2 — kept deliberately. Listed so a tidy-up does not take them.
TIER2 = [
    ("generate/prompts/node.md",
     "every v2 node transcript hashes the prompt rendered from this file, and "
     "generate/base.py splices its contract section — the base is FROZEN and "
     "was written against this exact text."),
    ("compose/prompts/compose.md",
     "every committed composition transcript. Superseded by compose_layered.md "
     "on 2026-09-14; editing it in place orphaned two of them within the "
     "minute, which is how it ended up on this list."),
    ("generate/prompts/condition_a.md", "every v1 transcript."),
    ("generate/prompts/condition_b.md", "every v1 transcript."),
    ("generate/prompts/condition_d.md", "every v1 transcript."),
    ("generate/archive/v1-single-call",
     "8 committed compositions, 7 committed episode results, chunk 39's A/B "
     "table."),
    ("generate/archive/v2-exclusive-control",
     "growth round 1, and every finding in generate/FINDINGS.md drawn from it."),
]

# TIER 3 — narrow later, and the two conditions that decide when.
TIER3_NOTE = (
    "harness/unit.py's run_unit/UnitScenario and cards/schema.py's "
    "unit_scenario() implement the micro-world contribution measurement. "
    "Chunk 52's prompt does not ask for a micro(), but harness/gate.py still "
    "calls run_unit for #9's regression suite. Revisit when gate.py has been "
    "rewritten against the growth suite AND no staged candidate defines a "
    "micro().")

STAGING = REPO_ROOT / "generate" / "staging"
GROWTH = REPO_ROOT / "out" / "growth"
CURRENT_PIPELINE = "v3-layered"


def gate():
    """(passed, [(check, ok, detail), ...]). Measured, never remembered."""
    checks = []

    admitted = []
    for path in sorted(STAGING.glob("*.result.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        meta = record.get("metadata") or {}
        if record.get("admitted") and meta.get("pipeline_version") == CURRENT_PIPELINE:
            admitted.append(record["node_id"])
    checks.append((
        f"an admitted {CURRENT_PIPELINE} candidate exists",
        bool(admitted),
        f"{len(admitted)} admitted" if admitted
        else f"none in {STAGING.relative_to(REPO_ROOT)}"))

    fixers = []
    for path in sorted(GROWTH.glob("*.csv")):
        with path.open(encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                try:
                    if int(row.get("fixed") or 0) > 0:
                        fixers.append(f"{row['node_id']} (+{row['fixed']})")
                except ValueError:
                    continue
    checks.append((
        "some node has fixed > 0 in a growth run",
        bool(fixers),
        ", ".join(fixers) if fixers
        else "fixed is 0 everywhere — the same result round 1 gave, so the "
             "new structure has not been shown to beat the old one"))

    return all(ok for _, ok, _ in checks), checks


def _exists(rel):
    path = REPO_ROOT / rel
    return path, path.exists()


def main(argv=None):
    args = _parse_args(argv)

    passed, checks = gate()
    print("[prune] the confirmation gate")
    for name, ok, detail in checks:
        print(f"[prune]   [{'PASS' if ok else 'FAIL'}] {name}")
        print(f"[prune]          {detail}")
    print(f"[prune]   => {'PASSED' if passed else 'NOT PASSED'}")
    if args.gate:
        return 0 if passed else 1

    print("\n[prune] TIER 1 — delete once the gate passes")
    live = []
    for rel, why in TIER1:
        path, exists = _exists(rel)
        print(f"[prune]   {'[ ]' if not exists else '[x]'} {rel}")
        print(f"[prune]       {why}")
        if exists:
            live.append(path)

    print("\n[prune] TIER 2 — KEPT. Evidence, not clutter.")
    for rel, why in TIER2:
        _path, exists = _exists(rel)
        print(f"[prune]   {'present' if exists else 'MISSING':8} {rel}")
        print(f"[prune]       {why}")

    print(f"\n[prune] TIER 3 — narrow later.\n[prune]   {TIER3_NOTE}")
    micro = [p.name for p in STAGING.glob("*.py")
             if "def micro(" in p.read_text(encoding="utf-8")]
    print(f"[prune]   staged candidates defining micro(): "
          f"{micro if micro else 'none'}")

    if not args.apply:
        print(f"\n[prune] dry run. {len(live)} path(s) would be deleted; "
              f"nothing was. Pass --apply.")
        return 0

    if not passed and not args.force:
        print("\n[prune] ⚠ REFUSED. The gate has not passed, so the structure "
              "that would justify these deletions has not been shown to work. "
              "Fix the gate, or pass --force and say why in the commit.")
        return 1
    if not passed:
        print("\n[prune] ⚠ --force: deleting although the gate has NOT passed.")

    import shutil
    for path in live:
        if path.is_dir():
            shutil.rmtree(path)
        else:
            path.unlink()
        print(f"[prune] deleted {path.relative_to(REPO_ROOT)}")
    print(f"[prune] {len(live)} path(s) deleted. Tier 2 untouched.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
