"""
demo/controls/build_feng2015.py

REGENERATES armX's DOCUMENT FROM THE PDF. The text itself is not committed
(licence unclear, Mindfire plan R10); this script plus the hashes in
`manifest.json` are what is released instead.

armX is the length-matched OFF-TOPIC control (Mindfire plan §2.2): the same
prompt block armB gets, holding a published paper about acoustic gas
thermometry instead of D*. It controls for "a long document in the prompt",
NOT for "a relevant-looking paper" -- say so wherever armX is reported.

The extraction is the rule that reproduces `demo/paper.txt` byte for byte:

    pdftotext -layout demo/paper.pdf -        # == paper.txt, checked 2026-11-02

applied to Feng et al. (2015), then `.strip()` (what `render.paper_text()`
does to paper.txt), then truncated to the first `CUT_LINES` lines. The cut sits
on a paragraph boundary chosen by `estimate_tokens.py` so that render("armX")
matches render("armB") in tokens. No marker is added at the cut: the block's
instruction text must stay identical across arms.

⚠️ Feng is single-column, so `-layout` does not interleave columns the way it
does for D*'s two-column text. That residual format difference is DISCLOSED
(plan R7), never faked by scrambling Feng.

    python demo/controls/build_feng2015.py                 # write + verify hashes
    python demo/controls/build_feng2015.py --check         # verify only, write nothing
    python demo/controls/build_feng2015.py --pdf PATH      # PDF somewhere else

SIM-FREE. Calls no model. Needs `pdftotext` (poppler) on PATH.
"""
import argparse
import hashlib
import json
import pathlib
import re
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE / "feng2015.txt"
MANIFEST = HERE / "manifest.json"

# ---- below this line is data, not code

PDF_NAME = "feng-2015-boltzmann-differential-cylindrical-arxiv-1501.02519.pdf"
PDF_SHA256 = "da7a29778e0e10557cadeec545709dcd368cd5fadac722a06f972d855e219cdf"
SOURCE_URL = "https://arxiv.org/abs/1501.02519"
EXTRACT_CMD = ["pdftotext", "-layout"]          # + PDF + "-" (stdout)

# Where the PDF is looked for, relative to this file. Project_B/corpus_pdfs/
# sits outside the git repo, and a worktree sits one level deeper than Repo/.
PDF_CANDIDATES = [
    HERE.parents[2] / "corpus_pdfs" / "controls" / PDF_NAME,   # Repo/demo/controls
    HERE.parents[3] / "corpus_pdfs" / "controls" / PDF_NAME,   # worktrees/<x>/demo/controls
]

# ---- below this line is code


def load_manifest():
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def find_pdf(path=None):
    for p in ([pathlib.Path(path)] if path else PDF_CANDIDATES):
        if p.is_file():
            return p
    raise FileNotFoundError(
        f"{PDF_NAME} not found (tried {[str(p) for p in PDF_CANDIDATES]}). "
        f"Download it from {SOURCE_URL} and pass --pdf.")


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def extract(pdf):
    """The full extraction, `.strip()`ed -- the same cleaning paper.txt gets."""
    data = pathlib.Path(pdf).read_bytes()
    if sha256(data) != PDF_SHA256:
        raise ValueError(f"{pdf}: SHA-256 {sha256(data)} is not the recorded "
                         f"{PDF_SHA256}. A different file would be a different control.")
    out = subprocess.run(EXTRACT_CMD + [str(pdf), "-"], check=True,
                         capture_output=True).stdout
    return out.decode("utf-8").strip()


_SECTION = re.compile(r"^\s*\d+(\.\d+)*\.?\s+[A-Z]")


def paragraph_ends(lines):
    """Indices i such that cutting after lines[i] ends on a paragraph boundary.

    A boundary is a line that ends a sentence ('.'), followed -- after blank
    lines or a page break -- by a line that opens a new paragraph (indented by
    3+ spaces) or a numbered section heading. pdftotext double-spaces this
    preprint, so "followed by a blank line" alone would accept every line.
    """
    ends = []
    for i, line in enumerate(lines):
        if not line.rstrip().endswith("."):
            continue
        j = i + 1
        while j < len(lines) and not lines[j].replace("\f", "").strip():
            j += 1
        if j >= len(lines):
            continue
        nxt = lines[j].replace("\f", "")
        if (len(nxt) - len(nxt.lstrip(" ")) >= 3) or _SECTION.match(nxt):
            ends.append(i)
    return ends


def truncate(text, cut_lines):
    """The first `cut_lines` lines, `.strip()`ed. render() strips again anyway."""
    return "\n".join(text.split("\n")[:cut_lines]).strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[1])
    ap.add_argument("--pdf", help="path to the Feng 2015 PDF")
    ap.add_argument("--check", action="store_true", help="verify, write nothing")
    a = ap.parse_args()

    m = load_manifest()
    full = extract(find_pdf(a.pdf))
    if sha256(full.encode()) != m["full_extraction_sha256"]:
        sys.exit(f"[feng] FAIL: full extraction hash {sha256(full.encode())} != "
                 f"{m['full_extraction_sha256']} -- a different pdftotext build? "
                 f"Recorded with {m['pdftotext_version']}.")
    cut = m["cut_lines"]
    if cut - 1 not in paragraph_ends(full.split("\n")):
        sys.exit(f"[feng] FAIL: cut_lines={cut} is not a paragraph boundary")
    text = truncate(full, cut)
    got = sha256(text.encode())
    if got != m["truncated_sha256"]:
        sys.exit(f"[feng] FAIL: truncated text hash {got} != {m['truncated_sha256']}")
    if not a.check:
        OUT.write_text(text + "\n", encoding="utf-8")
    print(f"[feng] OK   {len(text):,} chars, first {cut} of "
          f"{len(full.split(chr(10)))} lines, sha256 {got[:16]}"
          + ("" if a.check else f" -> {OUT}"))


if __name__ == "__main__":
    main()
