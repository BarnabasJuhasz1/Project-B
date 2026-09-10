"""
generate/corpus.py

The source documents themselves, as text — condition B's card-writing input.

B used to be handed a human's extraction of a paper. It is now handed **the
paper**, which is what `ProjectB_pipeline.drawio` has always shown ("B - LLM
Writing Card (with literature context)") and what chunk 42 builds. This module
is the only place the pipeline reads a PDF.

⚠️ **THE CORPUS IS AN INPUT AND IS NEVER COMMITTED.** `out/corpus/` is
gitignored on purpose: the PNAS copy carries the downloader's IP address, and
the rest are publisher PDFs that are ours to read and not ours to redistribute.
So a machine with no corpus cannot re-run B's card call, and that is a stated
property rather than a bug — the same status `scenes/lab42_v1.usd` has, except
that this one is not ours to ship.

**Everything downstream of the card is still reproducible from committed files.**
The generated cards are committed, the node call is a pure function of one, and
`ReplayBackend` needs no corpus at all. What needs the PDF is exactly one step:
turning a paper into a card again from scratch.

WHY pdftotext AND NOT pdfplumber
--------------------------------
Not a preference. Older TeX-produced papers place every glyph with its own
positioning operator and pdfplumber reports a space between each one — the DWA
paper extracts as "t h e c o n s t r u c t i o n o f l o c a l w o r l d
m o d e l s". `scripts/testing/fetch_and_verify.py` measured all three
extractors on that exact PDF and this module inherits its finding: poppler is
clean, pypdfium2 drops inter-word spaces, pdfplumber is unusable. pdfplumber is
kept only as the fallback for a machine with no poppler, and the extractor that
was actually used is recorded on every extraction, because a card written from
mangled text is a card about nothing and the extractor name is how anyone would
ever find that out.

SIM-FREE. No simulator, no GPU, no network.
"""

import hashlib
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
CORPUS_DIR = REPO_ROOT / "out" / "corpus"

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# The most text that may reach a prompt, in characters. Frozen, and it is a
# GUARD rather than a budget: the five pilot papers extract to 17k-87k
# characters, so on this corpus truncation never fires and the constant does
# nothing. It exists so that a future paper with a 200-page appendix produces a
# recorded, visible truncation instead of a prompt nobody meant to send.
CHAR_BUDGET = 200_000

# What a truncated extraction says, in the prompt, where the model can see it.
# A silent truncation is a card written from half a paper by a model that
# believed it had the whole one.
TRUNCATION_MARKER = (
    "\n\n[... TRUNCATED: this document exceeded the {budget} character budget "
    "and {dropped} characters were dropped from the end. You are reading the "
    "first {kept} characters only. Say so in your card's confidence.]\n")

# What replaces the paper text in a COMMITTED transcript. See `redaction()`.
REDACTION_MARKER = (
    "[PAPER TEXT REDACTED FROM THIS TRANSCRIPT — {chars} characters, "
    "sha256(pdf)={sha256}, extractor={extractor}. The corpus is an input and is "
    "never committed (out/corpus/ is gitignored). The prompt_hash recorded "
    "beside this is the hash of the REAL prompt, so a machine holding the PDF "
    "re-renders it byte-identically and replay finds this transcript.]")


class CorpusError(FileNotFoundError):
    """The paper is not on this machine. Distinguished from every other failure
    because it is the expected state on a machine that never fetched the corpus,
    and the caller should say so rather than crash."""


@dataclass(frozen=True)
class Paper:
    """One source document, extracted. `sha256` is of the PDF BYTES, not of the
    text: it identifies the document, and two extractors disagreeing about the
    same document is a fact worth being able to see."""

    card_id: str
    text: str
    sha256: str
    extractor: str
    char_count: int          # of the text actually handed to the model
    truncated: bool
    path: Path

    def redaction(self):
        """What stands in for this paper in a committed transcript."""
        return REDACTION_MARKER.format(chars=self.char_count,
                                       sha256=self.sha256[:16],
                                       extractor=self.extractor)


def paper_path(card_id, directory=None):
    """Where a card's source document lives. Named after the CARD, not after the
    publisher's filename, because the card id is what the pipeline keys on."""
    return Path(directory or CORPUS_DIR) / f"{card_id}.pdf"


def available(card_id, directory=None):
    return paper_path(card_id, directory).is_file()


def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def _extract_pdftotext(path):
    out = subprocess.run(["pdftotext", "-q", str(path), "-"],
                         capture_output=True, timeout=180)
    if out.returncode != 0:
        raise RuntimeError(f"pdftotext exited {out.returncode}")
    return out.stdout.decode("utf-8", "replace")


def _extract_pdfplumber(path):
    import pdfplumber
    with pdfplumber.open(path) as pdf:
        return "\n".join((page.extract_text() or "") for page in pdf.pages)


def load(card_id, directory=None, budget=CHAR_BUDGET):
    """One paper, as the text that will reach the prompt.

    Cached beside the PDF under the PDF's own hash, so a re-extraction is free
    and a REPLACED pdf cannot be served from a stale cache — the cache key is
    the document, not the name.
    """
    path = paper_path(card_id, directory)
    if not path.is_file():
        raise CorpusError(
            f"no source document for {card_id!r} at {path}. Condition B is "
            f"handed the paper itself, so its card call cannot run on a machine "
            f"that has not fetched the corpus. See cards/manifest_pilot.csv for "
            f"the URLs and scripts/testing/fetch_and_verify.py for the fetcher; "
            f"out/corpus/ is gitignored and is never committed.")

    digest = _sha256(path)
    cache = path.with_suffix(f".{digest[:12]}.txt")
    if cache.is_file():
        text, extractor = cache.read_text(encoding="utf-8"), "cache"
    elif shutil.which("pdftotext"):
        text, extractor = _extract_pdftotext(path), "pdftotext"
        cache.write_text(text, encoding="utf-8")
    else:
        # Kept, but loudly second-best. See the module docstring.
        text, extractor = _extract_pdfplumber(path), "pdfplumber"
        cache.write_text(text, encoding="utf-8")

    text = text.replace("\f", "\n\n")          # form feeds are page breaks
    truncated = len(text) > budget
    if truncated:
        dropped = len(text) - budget
        text = text[:budget] + TRUNCATION_MARKER.format(
            budget=budget, dropped=dropped, kept=budget)
    return Paper(card_id=card_id, text=text, sha256=digest, extractor=extractor,
                 char_count=len(text), truncated=truncated, path=path)
