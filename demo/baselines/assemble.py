"""
demo/baselines/assemble.py

Copies the shared block (`_shared.py`) into both hand-written baselines,
between their `# ==== SHARED BEGIN` and `# ==== SHARED END` lines.

Each baseline has to be ONE self-contained file, because that is the rule every
generated controller was held to (demo/task.md) and `demo/rollout.py`'s import
gate refuses `import demo.anything`. So the shared map, follower and recovery
code is pasted, not imported, and this script is the only way it is pasted.
`test_baselines.py::test_shared_block_is_identical` fails if a copy drifts.

    python demo/baselines/assemble.py           # rewrite both files
    python demo/baselines/assemble.py --check   # exit 1 if either differs

SIM-FREE.
"""
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
SHARED = HERE / "_shared.py"
TARGETS = (HERE / "dstar_lite.py", HERE / "astar_replan.py")
BLOCK = re.compile(r"# ==== SHARED BEGIN.*?# ==== SHARED END[^\n]*\n", re.S)


def shared_block():
    return BLOCK.search(SHARED.read_text()).group(0)


def block_of(path):
    m = BLOCK.search(pathlib.Path(path).read_text())
    return m.group(0) if m else None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    want = shared_block()
    bad = 0
    for t in TARGETS:
        text = t.read_text()
        if block_of(t) == want:
            print(f"[assemble] {t.name}: identical")
            continue
        if "--check" in argv:
            print(f"[assemble] {t.name}: DIFFERS from _shared.py")
            bad += 1
            continue
        t.write_text(BLOCK.sub(lambda _m: want, text, count=1))
        print(f"[assemble] {t.name}: rewritten")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
