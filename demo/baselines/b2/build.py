"""
demo/baselines/b2/build.py

Builds the Exp B2 baselines from the Exp B ones: the same file, with the shared
block replaced by `b2/_shared.py` and a B2 note under the docstring's first
line. The planner below the block is copied byte for byte, so B2 differs from
Exp B ONLY in the shared follower. The Exp B files are read, never written.

    python demo/baselines/b2/build.py           # write b2/dstar_lite.py, b2/astar_replan.py
    python demo/baselines/b2/build.py --check   # exit 1 if either is stale

SIM-FREE.
"""
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent.parent))

from demo.baselines import assemble

SHARED = HERE / "_shared.py"
SOURCES = {HERE / "dstar_lite.py": HERE.parent / "dstar_lite.py",
           HERE / "astar_replan.py": HERE.parent / "astar_replan.py"}
NOTE = ("\n⚠️ EXP B2 VERSION (prereg/expB2.md). Built by demo/baselines/b2/build.py\n"
        "from the Exp B file of the same name: identical except the SHARED block,\n"
        "whose follower turns on an arc at <= 0.8 rad/s instead of spinning in\n"
        "place at 1.0 rad/s, and un-sticks the gait when a commanded turn stalls.\n")


def built(source):
    text = source.read_text()
    want = assemble.BLOCK.search(SHARED.read_text()).group(0)
    text = assemble.BLOCK.sub(lambda _m: want, text, count=1)
    head, rest = text.split("\n", 2)[1], text.split("\n", 2)[2]
    return '"""\n' + head.replace("demo/baselines/", "demo/baselines/b2/") + "\n" + NOTE + rest


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    bad = 0
    for target, source in SOURCES.items():
        want = built(source)
        if target.exists() and target.read_text() == want:
            print(f"[b2] {target.name}: up to date")
        elif "--check" in argv:
            print(f"[b2] {target.name}: STALE")
            bad += 1
        else:
            target.write_text(want)
            print(f"[b2] {target.name}: written")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
