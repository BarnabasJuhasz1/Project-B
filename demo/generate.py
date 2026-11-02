"""
demo/generate.py

ONE MODEL CALL PER CONTROLLER. That is the whole generation pipeline.

    arm0   the task, and nothing else
    armB   the identical task, plus Stentz (1994) in full

Both arms go through `demo/render.py`, which is the only place they differ and
which asserts they differ nowhere else. The backend is the existing
`generate.backends.ClaudeCliBackend`, reused for one reason: it runs each call
in a scratch directory with `--setting-sources ''`, so this project's CLAUDE.md
-- which describes the experiment, the arms and the failure being probed --
never reaches the model. A fresh session per call also means arm0 cannot see
what armB was given.

⚠️ **EVERY CALL IS RECORDED**, prompt hash and all, under `demo/transcripts/`.
There is no temperature-0 determinism on any current model, so the transcript IS
the reproducibility story -- the same position `generate/FINDINGS.md` reached.

⚠️ **IT NEVER RE-CALLS A SAMPLE THAT EXISTS.** k is a denominator: silently
redrawing a sample that came out badly is how a k of 3 becomes a best-of-many.
Delete the file by hand if you actually want it redrawn.

    $ python demo/generate.py --dry-run          # prompt sizes, calls nothing
    $ python demo/generate.py --arm arm0 --k 3
    $ python demo/generate.py --k 3              # both arms
"""
import sys, pathlib, json, hashlib, re, argparse, time
from concurrent.futures import ThreadPoolExecutor
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from demo.render import render, check, ARMS

HERE = pathlib.Path(__file__).resolve().parent
ARMS_DIR = HERE / "arms"
TRANSCRIPTS = HERE / "transcripts"

CODE_FENCE = re.compile(r"```(?:python|py)?\n(?P<code>.*?)```", re.S)


def extract_code(text):
    blocks = CODE_FENCE.findall(text)
    if not blocks:
        return None
    # The longest block: models sometimes precede the file with a short sketch.
    return max(blocks, key=len)


def generate_one(arm, k, backend, force=False):
    """One sample. ⚠️ A FAILED CALL IS RECORDED AND RETURNED, NEVER RAISED --
    one timeout must not take the other five samples with it, and a sample that
    failed still belongs in the denominator."""
    out = ARMS_DIR / arm / f"k{k}.py"
    if out.exists() and not force:
        return out, "exists"
    prompt = render(arm)
    digest = hashlib.sha256(prompt.encode()).hexdigest()[:16]
    t0 = time.time()
    try:
        response = backend.complete(prompt)
    except Exception as exc:
        elapsed = round(time.time() - t0, 1)
        TRANSCRIPTS.mkdir(parents=True, exist_ok=True)
        (TRANSCRIPTS / f"{arm}__k{k}__FAILED.json").write_text(json.dumps({
            "arm": arm, "k": k, "prompt_sha256_16": digest,
            "prompt_chars": len(prompt), "elapsed_s": elapsed,
            "error": repr(exc),
            "payload": getattr(exc, "payload", None),
        }, indent=2), encoding="utf-8")
        return None, f"FAILED after {elapsed}s: {exc}"
    elapsed = round(time.time() - t0, 1)

    TRANSCRIPTS.mkdir(parents=True, exist_ok=True)
    (TRANSCRIPTS / f"{arm}__k{k}__{digest}.json").write_text(json.dumps({
        "arm": arm, "k": k, "prompt_sha256_16": digest,
        "prompt_chars": len(prompt), "model": response.model_id,
        "backend": response.backend, "sampling_note": response.sampling_note,
        "elapsed_s": elapsed, "raw": response.raw,
        "response": response.text,
    }, indent=2), encoding="utf-8")

    code = extract_code(response.text)
    if code is None:
        return None, "no fenced python block (transcript kept)"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(code, encoding="utf-8")
    cost = (response.raw or {}).get("total_cost_usd")
    return out, f"generated in {elapsed}s" + (f", ${cost:.2f}" if cost else "")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=ARMS, action="append")
    ap.add_argument("--k", type=int, default=3)
    ap.add_argument("--dry-run", action="store_true")
    # ⚠️ 600 s (the backend default) was NOT enough: arm0 k0 -- the SHORT
    # prompt -- timed out on the first attempt, so this is thinking time and
    # not prompt size. generate/backends.py already records the same hazard.
    ap.add_argument("--timeout", type=int, default=1800)
    ap.add_argument("--jobs", type=int, default=6,
                    help="calls in flight at once. Each is a separate CLI "
                         "session, so they cannot leak into one another.")
    ap.add_argument("--force", action="store_true",
                    help="redraw a sample that already exists. k is a "
                         "denominator; think before using this.")
    a = ap.parse_args(argv)
    arms = a.arm or list(ARMS)

    ok, detail = check()
    print(f"[gen] arm equivalence: {'OK' if ok else 'FAIL'} — {detail}")
    if not ok:
        raise SystemExit("the arms differ outside the paper block; refusing to run")

    for arm in arms:
        print(f"[gen] {arm}: prompt {len(render(arm)):,} chars")
    if a.dry_run:
        print(f"[gen] dry run — would make {len(arms) * a.k} calls, made 0")
        return

    from generate.backends import ClaudeCliBackend
    backend = ClaudeCliBackend(timeout=a.timeout)
    jobs = [(arm, k) for arm in arms for k in range(a.k)]
    print(f"[gen] {len(jobs)} calls, {a.jobs} at a time, timeout {a.timeout}s")

    def work(job):
        arm, k = job
        path, note = generate_one(arm, k, backend, force=a.force)
        return arm, k, path, note

    failures = 0
    with ThreadPoolExecutor(max_workers=a.jobs) as pool:
        for arm, k, path, note in pool.map(work, jobs):
            print(f"[gen] {arm} k{k}: {note}" + (f" -> {path}" if path else ""),
                  flush=True)
            failures += path is None
    print(f"[gen] done. {len(jobs) - failures}/{len(jobs)} produced a controller.")


if __name__ == "__main__":
    main()
