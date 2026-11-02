"""
demo/controls/calibrate.py

THE CALIBRATION COUNT for armX (MANIFEST.md, "Needs Barna's approval";
approved 2026-11-02, "yes to 1-4"). One call per named arm, with the EXACT
prompt `render(arm)` and the EXACT backend `generate.py` uses
(`ClaudeCliBackend(timeout=1800)`: same model, flags, system prompt and
budget cap), so the CLI's reported input tokens are the arm's real count under
the installed CLI version.

⚠️ THESE CALLS ARE NOT SAMPLES, decided before they were made. The transcript
goes to `demo/transcripts/calibration/` -- a subdirectory nothing globs (the
analysis reads `demo/transcripts/*.json` only) -- and NO `arms/<arm>/k*.py` is
ever written. The controller the model returns is kept inside the transcript
and is in no denominator.

    python demo/controls/calibrate.py --arm armB --arm armX     # 2 paid calls
    python demo/controls/calibrate.py --arm armB --dry-run      # calls nothing

Costs real money (~$1.5 per call). Never run it without approval.
"""
import argparse
import hashlib
import json
import pathlib
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from demo.render import render, check, ARMS                       # noqa: E402

OUT = pathlib.Path(__file__).resolve().parents[1] / "transcripts" / "calibration"
TIMEOUT_S = 1800            # generate.py's default; see its comment on why


def usage_tokens(raw):
    u = (raw or {}).get("usage") or {}
    return sum(u.get(k) or 0 for k in ("input_tokens", "cache_creation_input_tokens",
                                       "cache_read_input_tokens"))


def calibrate(arm, backend, cli_version):
    prompt = render(arm)
    digest = hashlib.sha256(prompt.encode()).hexdigest()[:16]
    t0 = time.time()
    record = {"arm": arm, "purpose": "calibration -- NOT a sample, in no denominator",
              "prompt_sha256_16": digest, "prompt_chars": len(prompt),
              "cli_version": cli_version}
    try:
        r = backend.complete(prompt)
        raw = r.raw
        record.update(model=r.model_id, backend=r.backend, raw=raw, response=r.text)
    except Exception as exc:                 # a failed call still reports usage
        raw = getattr(exc, "payload", None) or {}
        record.update(error=repr(exc), raw=raw)
    record["elapsed_s"] = round(time.time() - t0, 1)
    record["input_tokens_total"] = usage_tokens(raw)
    record["num_turns"] = (raw or {}).get("num_turns")
    record["total_cost_usd"] = (raw or {}).get("total_cost_usd")
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"calibration__{arm}__{digest}.json"
    path.write_text(json.dumps(record, indent=2), encoding="utf-8")
    return path, record


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=ARMS, action="append", required=True)
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    ok, detail = check()
    print(f"[cal] arm equivalence: {'OK' if ok else 'FAIL'} -- {detail}")
    if not ok:
        raise SystemExit("[cal] the arms differ outside the block; refusing")
    cli = subprocess.run(["claude", "--version"], capture_output=True,
                         text=True).stdout.strip()
    for arm in a.arm:
        p = render(arm)
        print(f"[cal] {arm}: {len(p):,} chars, sha256 "
              f"{hashlib.sha256(p.encode()).hexdigest()[:16]}, CLI {cli}")
    if a.dry_run:
        print(f"[cal] dry run -- would make {len(a.arm)} calls, made 0")
        return

    from generate.backends import ClaudeCliBackend
    backend = ClaudeCliBackend(timeout=TIMEOUT_S)
    with ThreadPoolExecutor(max_workers=len(a.arm)) as pool:
        for path, rec in pool.map(lambda arm: calibrate(arm, backend, cli), a.arm):
            print(f"[cal] {rec['arm']}: {rec['input_tokens_total']} input tokens, "
                  f"{rec['num_turns']} turn(s), ${rec['total_cost_usd']}, "
                  f"{rec['elapsed_s']} s" + (f", ERROR {rec['error'][:200]}"
                                             if "error" in rec else "")
                  + f" -> {path}", flush=True)


if __name__ == "__main__":
    main()
