"""
demo/render.py

THE ONE PLACE THE ARMS DIFFER, and it is a delimited block.

`arm0` gets the task and nothing else. `armB` gets the identical task plus one
paper, inside `PAPER BLOCK`. `armX` (added 2026-11-02, Mindfire plan §2.2) gets
the SAME block holding a length-matched OFF-TOPIC paper -- Feng et al. 2015 on
acoustic gas thermometry -- so armB - armX is the paper's CONTENT and armX -
arm0 is the cost of a long document. Strip the block from any rendering and all
three are **byte-identical** -- `check()` asserts it, and `demo/test_demo.py`
fails if it ever stops being true.

⚠️ armX controls for length and distraction, NOT for "a relevant-looking paper
pulls the model off course" (plan R8). Its text is not committed (R10):
`demo/controls/build_feng2015.py` regenerates it and `controls/manifest.json`
pins its hash.

That is the whole basis for the comparison. If the arms could differ anywhere
else -- a word of extra instruction, a reordered section, a different framing of
the task -- a difference in their controllers would not be attributable to the
paper. This is the same discipline `generate/prompts/card_{a,b,d}.md` uses, kept
because it is the only part of the old pipeline that was load-bearing here.

SIM-FREE. Calls no model.
"""
import pathlib
import re

HERE = pathlib.Path(__file__).resolve().parent

# The leading \n* matters: without it, stripping the block leaves behind the
# blank line that separated it from the task, and the two arms differ by one
# newline -- which is a real difference and would rightly fail check().
BLOCK = re.compile(
    r"\n*<!-- ---- PAPER BLOCK BEGIN ---- -->\n(?P<block>.*?)"
    r"<!-- ---- PAPER BLOCK END ---- -->\n", re.S)

ARMS = ("arm0", "armB", "armX")

# Which document fills the block. arm0 has none: its block is REMOVED, not
# emptied. Both documents are `pdftotext -layout <pdf> -`, then `.strip()`.
DOCUMENTS = {
    "armB": HERE / "paper.txt",
    "armX": HERE / "controls" / "feng2015.txt",
}


def paper_text(path=None):
    p = pathlib.Path(path) if path else DOCUMENTS["armB"]
    if not p.is_file():
        how = ("run `python demo/controls/build_feng2015.py` to regenerate it"
               if p == DOCUMENTS["armX"] else "see demo/README.md for how the "
               "text is produced")
        raise FileNotFoundError(
            f"{p} is missing. The arms with a paper block cannot run without "
            f"it; arm0 can. To fix: {how}.")
    text = p.read_text(encoding="utf-8", errors="replace").strip()
    if len(text) < 5000:
        raise ValueError(
            f"{p} holds {len(text)} characters. That is too short to be a "
            f"paper -- a scanned PDF with no text layer extracts to nothing, "
            f"and handing armB an empty block would silently make it arm0.")
    return text


def render(arm, paper=None):
    """The exact text sent to the model. Hashed and recorded by generate.py.

    `paper` overrides the arm's document (tests use a placeholder); it is
    ignored for arm0, which has no block.
    """
    if arm not in ARMS:
        raise KeyError(f"no arm {arm!r}. Known: {ARMS}")
    template = (HERE / "prompt.md").read_text(encoding="utf-8")
    task = (HERE / "task.md").read_text(encoding="utf-8")
    if arm == "arm0":
        template = BLOCK.sub("", template)          # the block is REMOVED
    else:
        template = template.replace(
            "{{PAPER}}", paper or paper_text(DOCUMENTS[arm]))
    return template.replace("{{TASK}}", task).strip() + "\n"


def shared_body(arm, paper=None):
    """A rendering with the paper block removed. Must be equal across arms.

    `.strip()` because render() already normalises its own trailing whitespace,
    so a trailing newline left behind by removing the block is an artefact of
    the removal rather than a difference between the two prompts.
    """
    return BLOCK.sub("", render(arm, paper=paper)).strip()


def check(paper=None, arms=ARMS):
    """Returns (ok, detail). The property the comparison rests on: every arm's
    rendering, block stripped, is byte-identical to arm0's."""
    a = shared_body("arm0", paper=paper)
    for arm in arms:
        if arm == "arm0":
            continue
        b = shared_body(arm, paper=paper)
        if a != b:
            import difflib
            diff = "\n".join(list(difflib.unified_diff(
                a.splitlines(), b.splitlines(), "arm0", arm, lineterm=""))[:40])
            return False, diff
    return True, (f"{', '.join(arms)} identical outside the paper block "
                  f"({len(a)} chars)")


if __name__ == "__main__":
    import sys
    paper = "PLACEHOLDER " * 1000 if "--no-paper" in sys.argv else None
    ok, detail = check(paper=paper)
    print(("OK   " if ok else "FAIL ") + detail)
    if ok and "--show" in sys.argv:
        print("\n" + "=" * 70 + "\narm0 prompt:\n" + "=" * 70)
        print(render("arm0"))
