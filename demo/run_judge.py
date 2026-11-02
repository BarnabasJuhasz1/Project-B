"""demo/run_judge.py — Exp 4's SECOND RATER: demo/JUDGE_PROMPT.md, once per blinded file.

One `claude -p` call per blinded controller, each in a fresh empty scratch
directory, with every tool disabled (`--tools ""`), no MCP servers, no setting
sources (so no CLAUDE.md is discovered) and a fixed system prompt. The prompt
is the judge template with {{RUBRIC}} = demo/CODING.md, {{CODE}} = the blinded
file and {{BLIND_ID}} = its id, byte for byte. The judge therefore sees the
rubric and one blinded file and nothing else: not the key, not the README, not
the unblinded arms, not rater 1's labels, not the features.

Every call is recorded VERBATIM under demo/out/v2/judge/<set>/<blind_id>.json
(prompt sha256, argv minus the prompt, CLI version, the CLI's JSON payload, the
parsed row). A file that already has a record is never re-called: the judge is
run ONCE per file, and is never re-run until it agrees.

⚠️ PAID. `--max-total-usd` is a hard cap: before each call the script stops if
the spend so far plus the largest single call seen so far (or `--per-call-usd`
before the first) could exceed it. `--max-budget-usd` is also passed to the
CLI per call as a runaway backstop.

    P="env -u PYTHONHOME -u PYTHONPATH $HOME/miniforge3/envs/env_isaacsim/bin/python"
    $P demo/run_judge.py --set pilot --dry-run          # renders, hashes, calls nothing
    $P demo/run_judge.py --set pilot --limit 1          # one call, to measure its cost
    $P demo/run_judge.py --set pilot                    # the rest, under the cap
    $P demo/run_judge.py --set pilot --write-only       # rebuild the rater-2 CSV from the records
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import pathlib
import re
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
V2 = HERE / "out" / "v2"
TEMPLATE = HERE / "JUDGE_PROMPT.md"
RUBRIC = HERE / "CODING.md"

# ---- below this line is data, not code ------------------------------------

MODEL = "claude-sonnet-5-5"
SYSTEM_PROMPT = ("You are a careful reviewer applying a fixed coding rubric to "
                 "source code. Follow the instructions in the message exactly "
                 "and reply with a single JSON object.")
COLUMNS = ["blind_id", "rater", "planner_type", "map", "velocity_selection",
           "stuck_handling", "claims_dstar", "dstar_variant",
           "ds_open_keyed_on_k", "ds_h_and_k", "ds_raise_lower",
           "ds_modify_cost", "ds_stop_rule", "ds_backpointers", "notes"]
ALLOWED = {
    "planner_type": {"full_every_step", "full_periodic", "incremental_only",
                     "incremental_with_fallback", "no_global_planner", "unclassifiable"},
    "map": {"persistent_grid", "none", "unclassifiable"},
    "velocity_selection": {"rollout_scored", "path_following", "reactive", "unclassifiable"},
    "stuck_handling": {"explicit_recovery", "timeout", "none", "unclassifiable"},
    "claims_dstar": {"yes", "no"},
    "dstar_variant": {"stentz1994", "dstar_lite", "other", "na"},
    **{c: {"yes", "partial", "no", "na"} for c in
       ("ds_open_keyed_on_k", "ds_h_and_k", "ds_raise_lower",
        "ds_modify_cost", "ds_stop_rule", "ds_backpointers")},
}


def template() -> str:
    """The judge template with its leading maintainer comment stripped."""
    t = TEMPLATE.read_text()
    return re.sub(r"\A\s*<!--.*?-->\s*", "", t, count=1, flags=re.S)


def render(blind_id: str, code: str) -> str:
    return (template().replace("{{RUBRIC}}", RUBRIC.read_text())
            .replace("{{CODE}}", code).replace("{{BLIND_ID}}", blind_id))


def parse(text: str) -> dict:
    m = re.search(r"\{.*\}", text, flags=re.S)
    if not m:
        raise ValueError("no JSON object in the reply")
    obj = json.loads(m.group(0))
    bad = {k: obj.get(k) for k, ok in ALLOWED.items() if obj.get(k) not in ok}
    if bad:
        raise ValueError(f"codes outside the rubric: {bad}")
    return obj


def call(prompt: str, model: str, per_call_cap: float) -> tuple[list, dict]:
    argv = ["claude", "-p", "--model", model, "--tools", "", "--strict-mcp-config",
            "--setting-sources", "", "--no-session-persistence",
            "--system-prompt", SYSTEM_PROMPT, "--output-format", "json",
            "--max-budget-usd", str(per_call_cap)]
    with tempfile.TemporaryDirectory(prefix="judge-clean-") as clean:
        proc = subprocess.run(argv, cwd=clean, input=prompt, capture_output=True,
                              text=True, timeout=900)
    try:
        payload = json.loads(proc.stdout)
    except json.JSONDecodeError:
        payload = {"is_error": True, "stdout": proc.stdout[:2000],
                   "stderr": proc.stderr[:2000], "returncode": proc.returncode}
    return argv, payload


def cli_version() -> str:
    return subprocess.run(["claude", "--version"], capture_output=True,
                          text=True).stdout.strip()


def write_csv(records: list[dict], out: pathlib.Path, rater: str) -> int:
    rows = []
    for rec in records:
        p = rec.get("parsed")
        if not p:
            continue
        row = {c: str(p.get(c, "")) for c in COLUMNS}
        row["blind_id"], row["rater"] = rec["blind_id"], rater
        row["notes"] = " ".join(row["notes"].split())
        rows.append(row)
    rows.sort(key=lambda r: r["blind_id"])
    with out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    return len(rows)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--set", required=True)
    ap.add_argument("--model", default=MODEL)
    ap.add_argument("--limit", type=int, default=None)
    ap.add_argument("--max-total-usd", type=float, default=5.0)
    ap.add_argument("--per-call-usd", type=float, default=0.60,
                    help="CLI runaway backstop per call, and the cost assumed before the first")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--write-only", action="store_true")
    a = ap.parse_args(argv)

    blind = V2 / "blind" / a.set
    recdir = V2 / "judge" / a.set
    recdir.mkdir(parents=True, exist_ok=True)
    files = sorted(blind.glob("*.py"))
    if not files:
        print(f"[judge] no blinded files under {blind}", file=sys.stderr)
        return 1
    records = [json.loads(p.read_text()) for p in sorted(recdir.glob("*.json"))]
    spent = sum(r.get("cost_usd") or 0.0 for r in records)
    worst = max([r.get("cost_usd") or 0.0 for r in records] or [a.per_call_usd])
    rater = f"rater2_{a.model}"
    out = V2 / f"code_labels_{a.set}_rater2.csv"

    if not a.write_only:
        done = {r["blind_id"] for r in records}
        todo = [p for p in files if p.stem not in done][: a.limit]
        version = cli_version()
        for p in todo:
            prompt = render(p.stem, p.read_text())
            sha = hashlib.sha256(prompt.encode()).hexdigest()
            if a.dry_run:
                print(f"[judge] {p.stem} prompt {len(prompt)} chars sha {sha[:16]}")
                continue
            if spent + worst > a.max_total_usd:
                print(f"[judge] STOP: spent ${spent:.2f}, next call could cost "
                      f"${worst:.2f}, cap ${a.max_total_usd:.2f}")
                break
            argv_, payload = call(prompt, a.model, a.per_call_usd)
            cost = payload.get("total_cost_usd")
            rec = {"blind_id": p.stem, "model": a.model, "cli_version": version,
                   "prompt_sha256": sha, "argv": argv_, "cost_usd": cost,
                   "payload": payload, "parsed": None, "parse_error": None}
            try:
                rec["parsed"] = parse(payload.get("result") or "")
            except (ValueError, json.JSONDecodeError) as exc:
                rec["parse_error"] = str(exc)
            (recdir / f"{p.stem}.json").write_text(json.dumps(rec, indent=1) + "\n")
            records.append(rec)
            spent += cost or 0.0
            # the largest call SEEN so far; a call with no reported cost
            # counts at the backstop, never as free
            worst = max((r.get("cost_usd") or a.per_call_usd) for r in records)
            print(f"[judge] {p.stem} ${cost} total ${spent:.2f} "
                  f"{'OK' if rec['parsed'] else 'PARSE ERROR: ' + rec['parse_error']}")
    if a.dry_run:
        return 0
    n = write_csv(records, out, rater)
    print(f"[judge] {n} rows -> {out}  (spent ${spent:.2f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
