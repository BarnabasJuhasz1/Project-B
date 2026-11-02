"""demo/blind_controllers.py — the blinded copy a rater codes (Exp 4, §4.2).

Each controller `<arm>/k<n>.py` is copied to `<blind_id>.py` with its HEADER
COMMENT STRIPPED — the leading `#` block and/or module docstring, cut by
`code_features.split_header`. The prompt tells armB (and armX) to "add a short
comment at the top of your file naming what you took from the paper", so the
header is where the arm announces itself. Nothing below the header is changed:
blinded body == original body, byte for byte, and a test pins that.

The KEY (blind_id -> arm, k, hashes) is written to a SEPARATE directory and is
joined to the labels only after coding is finished. Blind ids are random
(`secrets`), so neither the id nor the file order says anything about the arm;
the seed that drew them is recorded in the key, not beside the blinded files.

⚠️ RESIDUAL LEAK, DECLARED: identifiers and comments in the BODY are kept, and
they can still name the arm's source (`PROCESS-STATE`, `rhs`, "Sec. 2.3"). The
D* faithfulness checklist cannot be coded without reading that vocabulary, so
it is not stripped. `--strip-comments` additionally removes every `#` comment
(identifiers and docstrings stay); it is OFF by default because §4.2 asks for
the header only, and the summary line reports how many blinded files still
carry D* vocabulary either way — as a COUNT, never per id, so running this does
not itself unblind anyone.

⚠️ Refuses to overwrite an existing blinded set: re-drawing ids after a rater
has started would orphan their labels. `--force` exists for an unstarted set.

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P demo/blind_controllers.py --set pilot                 # demo/arms -> demo/out/v2/blind/pilot/
    $P demo/blind_controllers.py --set pilot --arms DIR --arm arm0 --arm armB
    ls demo/out/v2/blind/pilot/                              # what the rater sees
    # the key is in demo/out/v2/blind_keys/pilot_key.csv — do NOT open it while coding
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import io
import pathlib
import random
import re
import secrets
import sys
import tokenize

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from demo.code_features import DSTAR_VOCAB, discover, split_header  # noqa: E402

DEFAULT_ARMS = HERE / "arms"
BLIND_ROOT = HERE / "out" / "v2" / "blind"
KEY_ROOT = HERE / "out" / "v2" / "blind_keys"
KEY_COLUMNS = ["blind_id", "arm", "k", "source_sha256", "blinded_sha256",
               "header_lines_stripped", "strip_comments", "seed"]


def strip_comments(src: str) -> str:
    """Remove every `#` comment token, keep everything else (incl. docstrings).
    A line left holding only whitespace is dropped entirely."""
    lines = src.splitlines(keepends=True)
    cuts: dict[int, int] = {}
    for tok in tokenize.generate_tokens(io.StringIO(src).readline):
        if tok.type == tokenize.COMMENT:
            cuts[tok.start[0]] = tok.start[1]
    out = []
    for i, ln in enumerate(lines, 1):
        if i in cuts:
            kept = ln[:cuts[i]].rstrip()
            if not kept:
                continue
            ln = kept + "\n"
        out.append(ln)
    return "".join(out)


def blind_text(src: str, comments: bool = False) -> tuple[str, int]:
    """(blinded source, header lines removed)."""
    header, body = split_header(src)
    if comments:
        body = strip_comments(body)
    return body, len(header.splitlines())


def draw_ids(n: int, seed: int) -> list[str]:
    rng = random.Random(seed)
    ids: set[str] = set()
    while len(ids) < n:
        ids.add(f"c{rng.getrandbits(32):08x}")
    out = sorted(ids)
    rng.shuffle(out)
    return out


def blind(items, out_dir: pathlib.Path, key_path: pathlib.Path, *,
          seed: int, comments: bool = False, force: bool = False) -> list[dict]:
    if out_dir.exists() and any(out_dir.iterdir()) and not force:
        raise SystemExit(f"[blind] {out_dir} already holds a blinded set; "
                         "refusing to re-draw ids (use --force only if nobody has coded it)")
    if key_path.resolve().is_relative_to(out_dir.resolve()):
        raise SystemExit("[blind] the key must not live inside the blinded directory")
    out_dir.mkdir(parents=True, exist_ok=True)
    for old in out_dir.glob("*.py"):
        old.unlink()
    rng = random.Random(seed)
    order = list(items)
    rng.shuffle(order)
    ids = draw_ids(len(order), seed)
    rows = []
    for bid, (arm, k, path) in zip(ids, order):
        src = path.read_text()
        text, nhead = blind_text(src, comments)
        (out_dir / f"{bid}.py").write_text(text)
        rows.append({
            "blind_id": bid, "arm": arm, "k": k,
            "source_sha256": hashlib.sha256(src.encode()).hexdigest(),
            "blinded_sha256": hashlib.sha256(text.encode()).hexdigest(),
            "header_lines_stripped": nhead, "strip_comments": int(comments),
            "seed": seed,
        })
    rows.sort(key=lambda r: r["blind_id"])
    key_path.parent.mkdir(parents=True, exist_ok=True)
    with key_path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=KEY_COLUMNS, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    return rows


def leak_count(out_dir: pathlib.Path) -> int:
    pat = re.compile("|".join(f"(?:{p})" for p in DSTAR_VOCAB.values()))
    return sum(bool(pat.search(p.read_text())) for p in out_dir.glob("*.py"))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--set", required=True, help="name of the blinded set, e.g. pilot")
    ap.add_argument("--arms", type=pathlib.Path, default=DEFAULT_ARMS)
    ap.add_argument("--arm", action="append", help="restrict to these arms (repeatable)")
    ap.add_argument("--seed", type=int, default=None,
                    help="id seed; default is drawn from secrets and recorded in the key")
    ap.add_argument("--strip-comments", action="store_true")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args(argv)
    if not re.fullmatch(r"[\w-]+", a.set):
        ap.error("--set must be a plain name")
    items = [it for it in discover(a.arms) if not a.arm or it[0] in a.arm]
    if not items:
        print(f"[blind] no controllers under {a.arms}", file=sys.stderr)
        return 1
    seed = a.seed if a.seed is not None else secrets.randbits(63)
    out_dir, key = BLIND_ROOT / a.set, KEY_ROOT / f"{a.set}_key.csv"
    rows = blind(items, out_dir, key, seed=seed, comments=a.strip_comments, force=a.force)
    print(f"[blind] {len(rows)} controllers -> {out_dir}")
    print(f"[blind] key -> {key}  (do not open while coding)")
    print(f"[blind] {leak_count(out_dir)} of {len(rows)} blinded files still carry "
          "D* vocabulary in the body (declared residual leak)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
