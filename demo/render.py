"""
demo/render.py

THE ONE PLACE THE TWO ARMS DIFFER, and it is a delimited block.

`arm0` gets the task and nothing else. `armB` gets the identical task plus one
paper, inside `PAPER BLOCK`. Strip that block from either rendering and the two
are **byte-identical** -- `check()` asserts it, and `demo/test_demo.py` fails if
it ever stops being true.

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

ARMS = ("arm0", "armB")


def paper_text(path=None):
    p = pathlib.Path(path) if path else HERE / "paper.txt"
    if not p.is_file():
        raise FileNotFoundError(
            f"{p} is missing. armB cannot run without the paper; arm0 can. "
            f"See demo/README.md for how the text is produced.")
    text = p.read_text(encoding="utf-8", errors="replace").strip()
    if len(text) < 5000:
        raise ValueError(
            f"{p} holds {len(text)} characters. That is too short to be a "
            f"paper -- a scanned PDF with no text layer extracts to nothing, "
            f"and handing armB an empty block would silently make it arm0.")
    return text


def render(arm, paper=None):
    """The exact text sent to the model. Hashed and recorded by generate.py."""
    if arm not in ARMS:
        raise KeyError(f"no arm {arm!r}. Known: {ARMS}")
    template = (HERE / "prompt.md").read_text(encoding="utf-8")
    task = (HERE / "task.md").read_text(encoding="utf-8")
    if arm == "arm0":
        template = BLOCK.sub("", template)          # the block is REMOVED
    else:
        template = template.replace("{{PAPER}}", paper or paper_text())
    return template.replace("{{TASK}}", task).strip() + "\n"


def shared_body(arm, paper=None):
    """A rendering with the paper block removed. Must be equal across arms.

    `.strip()` because render() already normalises its own trailing whitespace,
    so a trailing newline left behind by removing the block is an artefact of
    the removal rather than a difference between the two prompts.
    """
    return BLOCK.sub("", render(arm, paper=paper)).strip()


def check(paper=None):
    """Returns (ok, detail). The property the comparison rests on."""
    a = shared_body("arm0", paper=paper)
    b = shared_body("armB", paper=paper)
    if a == b:
        return True, f"identical outside the paper block ({len(a)} chars)"
    import difflib
    diff = "\n".join(list(difflib.unified_diff(
        a.splitlines(), b.splitlines(), "arm0", "armB", lineterm=""))[:40])
    return False, diff


if __name__ == "__main__":
    import sys
    paper = "PLACEHOLDER " * 1000 if "--no-paper" in sys.argv else None
    ok, detail = check(paper=paper)
    print(("OK   " if ok else "FAIL ") + detail)
    if ok and "--show" in sys.argv:
        print("\n" + "=" * 70 + "\narm0 prompt:\n" + "=" * 70)
        print(render("arm0"))
