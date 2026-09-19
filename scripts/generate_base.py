#!/usr/bin/env python
"""
scripts/generate_base.py

**Generate the shared base node.** Chunk 47. ONE model call.

⚠️ NOT A CONDITION. The base is the substrate every arm's tree is grown on,
written by a neutral call with no card, no citation and no corpus. It lands in
`generate/base/`, never `generate/staging/`, so it is counted in no G1 number
and cannot be retrieved by composition as an arm's node.

⚠️ FROZEN once generated. Every growth-loop number is relative to it, so
regenerating is a different experiment rather than a better base. The script
refuses to overwrite without --force.

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P scripts/generate_base.py --dry-run          # print the prompt, call nothing
    $P scripts/generate_base.py                    # ONE call
    $P scripts/generate_base.py --backend replay   # from the committed transcript
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def main(argv=None):
    p = argparse.ArgumentParser(description="Generate the shared base node.")
    p.add_argument("--backend", default="claude-cli",
                   choices=("claude-cli", "anthropic-sdk", "replay"))
    p.add_argument("--model", default=None)
    p.add_argument("--timeout", type=int, default=None)
    p.add_argument("--dry-run", action="store_true",
                   help="print the rendered prompt and exit; calls nothing")
    p.add_argument("--force", action="store_true",
                   help="overwrite an existing base. ⚠ This invalidates every "
                        "growth-loop number measured against the old one.")
    args = p.parse_args(argv)

    from generate import backends, base

    prompt = base.render()
    if args.dry_run:
        print(prompt)
        print(f"\n[base] {len(prompt)} chars, hash {backends.prompt_hash(prompt)}")
        print("[base] --dry-run: nothing called.")
        return 0

    existing = base.BASE_DIR / f"{base.BASE_NODE_ID}.py"
    if existing.exists() and not args.force:
        print(f"[base] a base already exists at {existing}")
        print("[base] it is FROZEN — every growth-loop number is relative to it.")
        print("[base] pass --force only if you intend a different experiment.")
        return 1

    kwargs = {}
    if args.model:
        kwargs["model_id"] = args.model
    if args.timeout and args.backend == "claude-cli":
        kwargs["timeout"] = args.timeout
    backend = {"claude-cli": backends.ClaudeCliBackend,
               "anthropic-sdk": backends.AnthropicBackend,
               "replay": backends.ReplayBackend}[args.backend](**kwargs)

    print(f"[base] backend={args.backend} prompt={len(prompt)} chars "
          f"hash={backends.prompt_hash(prompt)}")
    candidate = base.generate(backend)
    print(f"[base] wrote {base.BASE_DIR / (base.BASE_NODE_ID + '.py')}")
    print(f"[base] class={candidate.node_cls.__name__} id={base.BASE_NODE_ID}")
    print("[base] it BUILDS and TICKS. How good it is, is "
          "`scripts/measure_base.py`'s question.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
