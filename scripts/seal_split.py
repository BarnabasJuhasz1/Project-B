"""
scripts/seal_split.py

Print the SHA-256 of the sealed held-out split, and enough of its shape to put
in a commit message.

Sim-free: imports core/ only. Run it on the sealing commit, paste the hash into
the Outcome and into results/README.md, and tag that commit `split-v1`. The hash
also lands in every run's manifest as `held_out_sha256`, so a result can prove
which split it scored against — and a dev result carrying it is evidence that
the split already existed, with this content, while development was happening.

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P scripts/seal_split.py

THIS SCRIPT PRINTS THE SPLIT'S SHAPE, NOT ITS CONTENT. It deliberately does not
print scenario ids, starts or goals: its output is meant to be safe to paste
into a commit message, an issue or a chat window, and pasting the split itself
anywhere an LLM can see it is the one failure mode sealing cannot survive.
"""

import os
import sys
from collections import Counter

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from core.manifest import sha256_file                # noqa: E402
from core.scenarios_held_out import (                # noqa: E402
    HELD_OUT_FAMILIES, HELD_OUT_SCENARIOS)

SPLIT_PATH = os.path.join(REPO_ROOT, "core", "scenarios_held_out.py")


def main():
    digest = sha256_file(SPLIT_PATH)
    if digest is None:
        sys.exit(f"[seal_split] not found: {SPLIT_PATH}")

    axes = Counter((s.goal_known, s.drift is not None) for s in HELD_OUT_SCENARIOS)
    families = Counter(s.family for s in HELD_OUT_SCENARIOS)

    print(f"[seal_split] core/scenarios_held_out.py")
    print(f"[seal_split] sha256 {digest}")
    print(f"[seal_split] {len(HELD_OUT_SCENARIOS)} scenarios, "
          f"{len(HELD_OUT_FAMILIES)} families")
    for family in HELD_OUT_FAMILIES:
        print(f"[seal_split]     {family:18s} {families[family]}")
    print(f"[seal_split] 2x2 (goal disclosed, drift on):")
    for known in (True, False):
        for drift in (False, True):
            print(f"[seal_split]     goal_known={str(known):5s} "
                  f"drift={str(drift):5s} -> {axes[(known, drift)]}")
    print(f"[seal_split] seeds {min(s.seed for s in HELD_OUT_SCENARIOS)}"
          f"-{max(s.seed for s in HELD_OUT_SCENARIOS)}")
    print(f"[seal_split] total time limit {sum(s.time_limit for s in HELD_OUT_SCENARIOS):.0f} s "
          f"per condition per seed, if every episode runs to timeout")
    print(f"[seal_split] tag the sealing commit: git tag split-v1")


if __name__ == "__main__":
    main()
