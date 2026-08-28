#!/usr/bin/env python3
"""
tidy_out.py

out/ is gitignored scratch and it fills up. Before the first sweep of this
tidy it held 39 run directories and 99 MB, most of them superseded re-runs of
the same six episodes, plus three stray mp4s predating the run-directory
layout entirely. This sweeps it.

    python scripts/tidy_out.py            # what it WOULD do, deletes nothing
    python scripts/tidy_out.py --apply    # do it

It is a script rather than an `rm -rf` because deciding what is safe to delete
has to be the same rule every time, and one of the things in there is the only
copy of a trace a committed result was scored from.

THE RULE. A directory under out/runs/ is KEPT if some result still points at
it, and deleted otherwise. Three kinds of result count:

  results/*.json                   the committed projections. NON-NEGOTIABLE:
                                   scripts/rescore.py re-scores from these
                                   traces, and core/tests/test_results_integrity
                                   .py fails if the two ever disagree. Deleting
                                   one of these makes a committed number
                                   unreproducible without booting the simulator.
  out/sweeps/**/results/*.json     a sweep campaign's scored episodes.

A directory is also kept if something in out/evidence/ SYMLINKS to it. A
curated run that was copied into out/evidence/ does NOT keep its source alive:
the copy is self-contained, and pinning the original would leave two copies of
the same 3 MB video forever. Evidence links, rather than copies, exactly when
the run backs a committed result and therefore cannot be moved.

A result written before run directories carried the invocation timestamp has
`run_dir: null`; its trace is at the flat legacy path, and that is kept too.

WHAT THIS NEVER TOUCHES. out/evidence/, out/sweeps/ and out/corpus/ are the
folders a human put something in deliberately — curated runs, campaign logs and
the source PDFs the cards cite. This sweeps out/runs/ and loose files at the top
of out/, and nothing else. See out/README.md for what each folder is for.
"""

import argparse
import json
import os
import shutil
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(REPO_ROOT, "out")

# ---- below this line is data, not code.

# Swept. Everything else directly under out/ is left alone.
RUNS_DIR = os.path.join(OUT, "runs")

# Never swept, and the reason each one is exempt.
PROTECTED = {
    "evidence": "curated runs — the point of keeping anything",
    "sweeps": "campaign logs and their scored episodes",
    "corpus": "source PDFs the cards cite; inputs, not outputs",
    "runs": "swept per-directory by the rule above, not as a whole",
    "README.md": "the map",
}

# Where to look for results that pin a run directory.
RESULT_ROOTS = (
    os.path.join(REPO_ROOT, "results"),
    os.path.join(OUT, "sweeps"),
)

# Scanned for symlinks into out/runs/, not for results.
EVIDENCE_DIR = os.path.join(OUT, "evidence")


def human(n):
    for unit in ("B", "K", "M", "G"):
        if n < 1024 or unit == "G":
            return f"{n:.0f}{unit}" if unit == "B" else f"{n:.1f}{unit}"
        n /= 1024.0


def size_of(path):
    if os.path.islink(path) or os.path.isfile(path):
        return os.path.getsize(path) if not os.path.islink(path) else 0
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            full = os.path.join(root, name)
            if not os.path.islink(full):
                total += os.path.getsize(full)
    return total


def _payloads():
    """Every result-shaped json under RESULT_ROOTS, with where it was found.

    A manifest.json alone is enough: out/evidence/ holds copies of run
    directories, and a copied run directory has a manifest but its result.json
    is the same file under another name.
    """
    for root in RESULT_ROOTS:
        if not os.path.isdir(root):
            continue
        for dirpath, _, files in os.walk(root):
            for name in files:
                if not name.endswith(".json"):
                    continue
                path = os.path.join(dirpath, name)
                try:
                    with open(path) as fh:
                        doc = json.load(fh)
                except (ValueError, OSError):
                    continue
                manifest = doc.get("manifest", doc)
                if isinstance(manifest, dict) and "scenario_id" in manifest:
                    yield os.path.relpath(path, REPO_ROOT), manifest


def referenced():
    """run-directory name -> what pins it."""
    keep = {}
    for dirpath, dirnames, files in os.walk(EVIDENCE_DIR):
        for name in dirnames + files:
            link = os.path.join(dirpath, name)
            if not os.path.islink(link):
                continue
            target = os.path.realpath(link)
            if os.path.dirname(target) == os.path.realpath(RUNS_DIR):
                keep.setdefault(os.path.basename(target), []).append(
                    os.path.relpath(link, REPO_ROOT) + " (symlink)")
    for where, manifest in _payloads():
        run_dir = manifest.get("run_dir")
        if run_dir:
            name = os.path.basename(os.path.normpath(run_dir))
        else:
            # Pre-timestamp layout: the directory was named after the episode.
            # core/tests/test_results_integrity.py still checks this path, so
            # it is load-bearing rather than merely old.
            from core.manifest import run_id
            name = run_id(manifest["scenario_id"], manifest["controller"],
                          manifest["seed"])
        keep.setdefault(name, []).append(where)
    return keep


def main():
    parser = argparse.ArgumentParser(
        description="sweep out/ of run directories nothing points at")
    parser.add_argument("--apply", action="store_true",
                        help="actually delete. Without it this only reports")
    args = parser.parse_args()

    sys.path.insert(0, REPO_ROOT)
    if not os.path.isdir(OUT):
        sys.exit("[tidy] no out/ directory; nothing to do")

    pinned = referenced()
    doomed, kept_bytes, freed_bytes = [], 0, 0

    print("[tidy] out/runs/")
    if os.path.isdir(RUNS_DIR):
        for name in sorted(os.listdir(RUNS_DIR)):
            path = os.path.join(RUNS_DIR, name)
            if not os.path.isdir(path):
                continue
            nbytes = size_of(path)
            holders = pinned.get(name, [])
            if holders:
                kept_bytes += nbytes
                print(f"  keep   {human(nbytes):>7}  {name[:78]}")
                print(f"         {'':>7}  <- {holders[0]}"
                      + (f" (+{len(holders) - 1} more)" if len(holders) > 1 else ""))
            else:
                freed_bytes += nbytes
                doomed.append(path)
                print(f"  DELETE {human(nbytes):>7}  {name[:78]}")

    # Loose files at the top of out/ predate the run-directory layout: a bare
    # mp4 with no trace, no manifest and no result is not evidence of anything.
    print("[tidy] loose files at the top of out/")
    for name in sorted(os.listdir(OUT)):
        if name in PROTECTED:
            continue
        path = os.path.join(OUT, name)
        nbytes = size_of(path)
        freed_bytes += nbytes
        doomed.append(path)
        kind = "DELETE" if os.path.isfile(path) else "DELETE (dir)"
        print(f"  {kind:12s} {human(nbytes):>7}  {name}")

    print(f"[tidy] keep {human(kept_bytes)} in {len(pinned)} pinned "
          f"directories, free {human(freed_bytes)} across {len(doomed)} paths")
    if not doomed:
        print("[tidy] nothing to do")
        return
    if not args.apply:
        print("[tidy] dry run — pass --apply to delete")
        return
    for path in doomed:
        if os.path.isdir(path) and not os.path.islink(path):
            shutil.rmtree(path)
        else:
            os.remove(path)
    print(f"[tidy] deleted {len(doomed)} paths, freed {human(freed_bytes)}")


if __name__ == "__main__":
    main()
