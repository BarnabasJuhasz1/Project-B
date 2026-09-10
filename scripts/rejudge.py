"""
scripts/rejudge.py

Re-judge every staged candidate under chunk 37's rule, WITHOUT rewriting what
the old rule said.

    $P scripts/rejudge.py                 # the diff, changes nothing
    $P scripts/rejudge.py --apply         # write admitted_v2 back
    $P scripts/rejudge.py --apply --staging generate/staging

WHY BOTH VERDICTS ARE KEPT
---------------------------
Chunk 37 redefines admissible from "passes its unit scenario in isolation" to
"runs". That is a redefinition, not an improvement, and the expected effect is
that condition D goes from 0 admitted to as many as 4 — because two of its four
rejections were the harness refusing the GENERATOR'S WORLD rather than judging a
node.

A redefinition that silently replaces its own denominator is indistinguishable
from a result. So `admitted` and `reason` are never touched; `admitted_v2` and
`reason_v2` are added beside them, and `generate/admit.py:pass_rate` reports
both. The write-up must say which rule any number was produced under.

NO MODEL CALLS. Every input is a committed file, so this is deterministic and
idempotent: running it twice writes the same bytes.

⚠️ It re-executes model-written Python, exactly as `generate/admit.py` and
`compose/library.py` do, for the same reason and with the same absence of a
sandbox.

SIM-FREE.
"""

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT))

# argparse before the heavy imports, so a malformed flag costs nothing.
parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--staging", default=None,
                    help="staging directory (default: generate/staging)")
parser.add_argument("--apply", action="store_true",
                    help="write admitted_v2 back; without it nothing changes")
parser.add_argument("--json", action="store_true", help="machine-readable diff")
ARGS = parser.parse_args()

from cards.store import CardStore                            # noqa: E402
from generate.admit import admit                             # noqa: E402
from generate.generator import STAGING                       # noqa: E402


def rejudge(staging, apply=False):
    """Every staged candidate, judged again. Returns one row per record."""
    staging = Path(staging or STAGING)
    cards = CardStore.load()
    rows = []

    for path in sorted(staging.glob("*.result.json")):
        stem = path.name[: -len(".result.json")]
        record = json.loads(path.read_text(encoding="utf-8"))
        meta = json.loads((staging / f"{stem}.json").read_text(encoding="utf-8"))

        source = None
        py = staging / f"{stem}.py"
        if py.exists():
            source = py.read_text(encoding="utf-8")

        card = citation_of = None
        ref = meta.get("card_id") or meta.get("citation_id")
        if ref and ref in cards:
            if meta.get("card_id"):
                card = cards.get(ref)
            else:
                citation_of = cards.get(ref)

        result = admit(meta, source, card=card, citation_of=citation_of)

        rows.append({
            "stem": stem,
            "condition": meta.get("condition", "?"),
            "ref": ref or "-",
            "v1_admitted": bool(record.get("admitted")),
            "v1_reason": record.get("reason", ""),
            "v2_admitted": bool(result.admitted),
            "v2_reason": result.reason,
            "contribution": result.metadata.get("contribution", {}),
            # ⚠️ Read from the RECORD, not the candidate sidecar. `retired_reason`
            # and `superseded_by` are annotations added to the result after the
            # fact — the candidate .json predates them — so checking the sidecar
            # silently counts four retired naive-arm records in G1.
            "excluded": bool(record.get("metadata", {}).get("retired_reason")
                             or record.get("metadata", {}).get("superseded_by")
                             or (ref and ref not in cards)),
            "changed": bool(record.get("admitted")) != bool(result.admitted),
        })

        if apply:
            # ADDITIVE ONLY. `admitted` and `reason` are the old rule's verdict
            # and are evidence; overwriting them would edit a denominator after
            # the fact, which is the one thing generate/staging exists to
            # prevent.
            record["admitted_v2"] = bool(result.admitted)
            record["reason_v2"] = result.reason
            record["detail_v2"] = result.detail
            record.setdefault("metadata", {})
            for key in ("validity", "contribution", "precondition_fired"):
                if key in result.metadata:
                    record["metadata"][key] = result.metadata[key]
            path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n",
                            encoding="utf-8")

    return rows


def main():
    rows = rejudge(ARGS.staging, apply=ARGS.apply)
    if ARGS.json:
        print(json.dumps(rows, indent=2, sort_keys=True))
        return 0

    print(f"[rejudge] {len(rows)} staged records, "
          f"{'APPLIED' if ARGS.apply else 'dry run — nothing written'}")
    print()
    print(f"  {'':2} {'c':2} {'reference':26} {'v1':>4} -> {'v2':<4} reason")
    for row in sorted(rows, key=lambda r: (r["condition"], r["ref"])):
        mark = "!" if row["changed"] else " "
        excl = "x" if row["excluded"] else " "
        v1 = "PASS" if row["v1_admitted"] else "fail"
        v2 = "PASS" if row["v2_admitted"] else "fail"
        print(f"  {mark}{excl} {row['condition']:2} {row['ref']:26} "
              f"{v1:>4} -> {v2:<4} {row['v2_reason']}")
    print("\n  ! = the verdict changed   x = excluded from G1 "
          "(retired / superseded / fixture)")

    counted = [r for r in rows if not r["excluded"]]
    print(f"\n[rejudge] counted records: {len(counted)}")
    for rule in ("v1", "v2"):
        n = sum(1 for r in counted if r[f"{rule}_admitted"])
        print(f"    {rule}: {n}/{len(counted)} admitted "
              f"({n / len(counted):.2f})" if counted else f"    {rule}: none")
    changed = [r for r in counted if r["changed"]]
    print(f"[rejudge] {len(changed)} counted verdicts changed")
    if not ARGS.apply and changed:
        print("[rejudge] re-run with --apply to record them")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
