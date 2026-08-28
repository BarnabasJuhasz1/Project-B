#!/usr/bin/env bash
# check_determinism.sh
#
# Run one episode twice and diff the traces. Same machine, same scene bytes,
# same seed: the two runs must be byte-identical.
#
#     scripts/check_determinism.sh [scenario] [controller]
#
# WHAT THIS DOES NOT CLAIM: GPU physics is not guaranteed bitwise deterministic
# across drivers or hardware. This is same-machine repeatability. Across
# machines the claim is statistical equivalence, and the manifest records GPU,
# driver, Isaac version, container image and USD hash so two runs can always be
# told apart. See plans/done/08-determinism.md.
set -euo pipefail

SCENARIO="${1:-doorway_traverse}"
CONTROLLER="${2:-reactive_avoid}"
PYTHON="${PYTHON:-$HOME/miniforge3/envs/env_isaacsim/bin/python}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# The scenario prefix is globbed rather than fixed: run_episode.py caps an
# episode at 60 sim-seconds by default and renames the scenario when the cap
# bites, so `doorway_traverse` writes `doorway_traverse+cap60s__...`. Both
# runs below get the same cap, so repeatability is still what is measured.
WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# --no-record: this runs the episode twice and only wants the trace. Recording
# costs ~0.9 s of wall clock per frame and cannot change the trace, because
# rendering never advances physics.
#
# --results-dir points at a temp directory so a repeatability check never
# rewrites a committed result. It used to leave the run directory pointing at
# the wrong invocation all the same, since run directories were keyed on the
# episode rather than on the invocation; they now carry the start time, so the
# two runs below land in their own directories and clobber nothing.
for attempt in 1 2; do
    echo "[determinism] run $attempt of 2: $SCENARIO / $CONTROLLER"
    "$PYTHON" scripts/run_episode.py --scenario "$SCENARIO" \
        --controller "$CONTROLLER" --no-record \
        --results-dir "$WORK/results" > "$WORK/log$attempt" 2>&1 \
        || { tail -40 "$WORK/log$attempt"; exit 1; }
    # Ask the run which directory it wrote, rather than guessing from a glob:
    # a glob sorted either way picks the wrong one of two runs half the time,
    # and comparing a trace against itself passes for the wrong reason.
    RUN_DIR="$("$PYTHON" -c "import json,sys; print(json.load(open(sys.argv[1]))['manifest']['run_dir'])" \
               "$WORK/results/${SCENARIO}"*"__${CONTROLLER}__seed"*.json)"
    cp "$RUN_DIR/trace.csv" "$WORK/trace$attempt.csv"
done

if diff -q "$WORK/trace1.csv" "$WORK/trace2.csv" > /dev/null; then
    echo "[determinism] PASS: $(wc -l < "$WORK/trace1.csv") lines identical"
else
    echo "[determinism] FAIL: traces differ"
    diff "$WORK/trace1.csv" "$WORK/trace2.csv" | head -20
    exit 1
fi
