#!/usr/bin/env bash
# check_determinism.sh
#
# Run one episode twice and diff the traces. Same machine, same scene bytes,
# same seed: the two runs must be byte-identical.
#
#     scripts/check_determinism.sh [scenario] [controller] [run_episode args...]
#
# Anything after the controller is passed through to run_episode.py, which is
# how a localiser gets checked -- chunk 24's gate:
#
#     scripts/check_determinism.sh doorway_traverse reactive_avoid \
#         --sensor-noise 1.0 --localisation slam
#
# BOTH sidecars are diffed. `slam.csv` is the localiser's own state stream, and
# it is the file that would show a SLAM pipeline being "almost always
# identical" -- a trace can stay byte-identical while the estimate behind it
# wobbles, because the trace records where the robot WENT and slam.csv records
# what it BELIEVED. Checking only the trace would pass a localiser that is not
# reproducible. See plans/todo/24-slam-determinism.md.
#
# WHAT THIS DOES NOT CLAIM: GPU physics is not guaranteed bitwise deterministic
# across drivers or hardware. This is same-machine repeatability. Across
# machines the claim is statistical equivalence, and the manifest records GPU,
# driver, Isaac version, container image and USD hash so two runs can always be
# told apart. See plans/done/08-determinism.md.
set -euo pipefail

SCENARIO="${1:-doorway_traverse}"
CONTROLLER="${2:-reactive_avoid}"
shift 2 2>/dev/null || true
EXTRA=("$@")                      # passed straight through to run_episode.py
PYTHON="${PYTHON:-$HOME/miniforge3/envs/env_isaacsim/bin/python}"

# ⚠️ Threading is NOT pinned here, deliberately. Chunk 24a measured the SLAM
# pipeline byte-identical at 1 thread and at 22, because nothing in it reaches
# a BLAS threading threshold: spsolve is SuperLU, and the only dense products
# are 3x3 blocks and an (N,2)@(2,2). Pinning the threads inside this script
# would make it pass for a reason it has not established. If that ever changes,
# this check is where it surfaces -- which is the point of not hiding it.

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
        --results-dir "$WORK/results" ${EXTRA[@]+"${EXTRA[@]}"} \
        > "$WORK/log$attempt" 2>&1 \
        || { tail -40 "$WORK/log$attempt"; exit 1; }
    # Ask the run which directory it wrote, rather than guessing from a glob:
    # a glob sorted either way picks the wrong one of two runs half the time,
    # and comparing a trace against itself passes for the wrong reason.
    RUN_DIR="$("$PYTHON" -c "import json,sys; print(json.load(open(sys.argv[1]))['manifest']['run_dir'])" \
               "$WORK/results/${SCENARIO}"*"__${CONTROLLER}__seed"*.json)"
    cp "$RUN_DIR/trace.csv" "$WORK/trace$attempt.csv"
    # Only present when the scenario carries a localiser. Copied when it
    # exists so that the two attempts can be compared for PRESENCE as well as
    # content -- one run producing a sidecar and the other not is a failure,
    # not a reason to skip the check.
    if [ -f "$RUN_DIR/slam.csv" ]; then
        cp "$RUN_DIR/slam.csv" "$WORK/slam$attempt.csv"
    fi
done

status=0

if diff -q "$WORK/trace1.csv" "$WORK/trace2.csv" > /dev/null; then
    echo "[determinism] PASS trace.csv: $(wc -l < "$WORK/trace1.csv") lines identical"
else
    echo "[determinism] FAIL: traces differ"
    diff "$WORK/trace1.csv" "$WORK/trace2.csv" | head -20
    status=1
fi

if [ -f "$WORK/slam1.csv" ] || [ -f "$WORK/slam2.csv" ]; then
    if [ ! -f "$WORK/slam1.csv" ] || [ ! -f "$WORK/slam2.csv" ]; then
        echo "[determinism] FAIL: one run wrote slam.csv and the other did not"
        status=1
    elif diff -q "$WORK/slam1.csv" "$WORK/slam2.csv" > /dev/null; then
        echo "[determinism] PASS slam.csv: $(wc -l < "$WORK/slam1.csv") lines identical"
    else
        echo "[determinism] FAIL: localiser state differs"
        diff "$WORK/slam1.csv" "$WORK/slam2.csv" | head -20
        status=1
    fi
else
    echo "[determinism] slam.csv: absent, no localiser on this scenario"
fi

exit $status
