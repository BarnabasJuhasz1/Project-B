"""
scripts/rescore.py

Re-score a committed result from the trace it was produced from, without a
simulator and without re-running the episode.

Scoring is a pure function of a recorded trace (core/metrics.py), which is what
makes this possible: when a metric constant or the SPL denominator moves, the
old episodes do not have to be thrown away or re-simulated to be comparable
with the new ones. That property is worth an actual entry point, otherwise it
is only ever asserted.

Sim-free: imports core/ only, so no SimulationApp, no GPU, no 20 s boot. It is
NOT on the benchmark path — it produces no new episodes and cannot turn a
non-reportable run into a reportable one.

PROVENANCE. The manifest's git_commit describes the code that flew the robot.
After a re-score the numbers no longer come from that commit, so the manifest
grows `rescored_utc`, `rescored_git_commit` and `rescored_from`. A result
carrying those was scored by later code than it was run by, and anything
comparing it against a freshly-run episode has to know that.

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P scripts/rescore.py --dry-run                  # show the diff, write nothing
    $P scripts/rescore.py                            # rewrite every result file
    $P scripts/rescore.py --only doorway_traverse__reactive_avoid__seed1001

    # which traces are still on disk (out/runs/ is gitignored and regenerable)
    ls out/runs/*/trace.csv
"""

import argparse
import json
import os
import sys
from datetime import datetime, timezone

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)

from core.manifest import _run                      # noqa: E402
from core.metrics import EpisodeTrace, score         # noqa: E402
from core.scenario import load                      # noqa: E402


def log(msg):
    print(f"[rescore] {msg}")


def read_trace(path):
    """trace.csv -> EpisodeTrace. The inverse of EpisodeTrace.write_csv().

    Values were written rounded to 6 dp, so a re-score is reproducible from the
    file but not bitwise identical to scoring the in-memory trace.

    That difference used to be dangerous rather than negligible. An earlier
    version of this docstring claimed 6 dp is "well below every threshold in
    core/metrics.py"; it is below every threshold in MAGNITUDE, but two
    comparisons in core/metrics.py sat on exact ties — EVENT_RELEASE is an
    integer multiple of the control step, and so is STUCK_WINDOW — so the last
    bits decided the answer and a round trip could move stuck_events by one and
    time_stuck by 41%. Both now carry EVENT_RELEASE_EPS. The claim is true as
    written only because that was fixed; see plans/done/10-*.

    The parsing itself lives on EpisodeTrace, beside write_csv, so the reader
    and the writer cannot drift apart. It matches columns by NAME and fills a
    column that did not exist when the trace was written from
    TRACE_BACKFILL — which is what lets a result committed before `goal_known`
    existed still be re-scored rather than orphaned.
    """
    return EpisodeTrace.read_csv(path)


def trace_for(payload, run_id):
    """The trace belonging to THIS result, or None.

    `manifest["run_dir"]` names one invocation and is the only trustworthy
    answer: run directories used to be keyed on the episode alone, so a later
    run of the same scenario/controller/seed overwrote the trace a committed
    result pointed at. The legacy fallback is kept for results written before
    that changed, and it is checked against `started_utc` rather than trusted.
    """
    run_dir = payload["manifest"].get("run_dir")
    if run_dir:
        return os.path.join(REPO_ROOT, run_dir, "trace.csv")

    legacy = os.path.join(REPO_ROOT, "out", "runs", run_id)
    manifest_path = os.path.join(legacy, "manifest.json")
    if not os.path.exists(manifest_path):
        return None
    with open(manifest_path) as fh:
        on_disk = json.load(fh)
    if on_disk.get("started_utc") != payload["manifest"].get("started_utc"):
        log(f"REFUSE {run_id}: out/runs/{run_id}/ holds a different invocation "
            f"({on_disk.get('started_utc')}) from the committed result "
            f"({payload['manifest'].get('started_utc')}). Re-run the episode "
            f"rather than re-scoring it against someone else's trace.")
        return None
    return os.path.join(legacy, "trace.csv")


def rescore(result_path, dry_run=False):
    """Returns (old_result, new_result), or None if the trace is gone."""
    run_id = os.path.splitext(os.path.basename(result_path))[0]

    with open(result_path) as fh:
        payload = json.load(fh)

    trace_path = trace_for(payload, run_id)
    if trace_path is None or not os.path.exists(trace_path):
        log(f"SKIP {run_id}: no trace on disk — out/ is gitignored, so "
            f"re-run the episode instead")
        return None

    old = payload["result"]
    new = score(read_trace(trace_path), load(payload["manifest"]["scenario_id"]))

    if not dry_run:
        payload["result"] = new
        payload["manifest"].update({
            "rescored_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
            "rescored_git_commit": _run(["git", "rev-parse", "HEAD"]),
            "rescored_from": os.path.relpath(trace_path, REPO_ROOT),
        })
        for path in (result_path, os.path.join(os.path.dirname(trace_path),
                                               "result.json")):
            if path == result_path or os.path.exists(path):
                with open(path, "w") as fh:
                    json.dump(payload, fh, indent=2, sort_keys=True)
                    fh.write("\n")
    return old, new


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results-dir", default="results")
    ap.add_argument("--only", help="one result stem, without .json")
    ap.add_argument("--dry-run", action="store_true",
                    help="print the diff without touching any file")
    args = ap.parse_args()

    results_dir = os.path.join(REPO_ROOT, args.results_dir)
    stems = sorted(f for f in os.listdir(results_dir) if f.endswith(".json"))
    if args.only:
        stems = [f"{args.only}.json"]

    changed = 0
    for stem in stems:
        outcome = rescore(os.path.join(results_dir, stem), args.dry_run)
        if outcome is None:
            continue
        old, new = outcome
        keys = sorted(set(old) | set(new))
        diffs = [(k, old.get(k), new.get(k)) for k in keys if old.get(k) != new.get(k)]
        if not diffs:
            log(f"{stem[:-5]}: unchanged")
            continue
        changed += 1
        log(f"{stem[:-5]}:")
        for k, a, b in diffs:
            log(f"    {k:20s} {a!r:>22}  ->  {b!r}")

    log(f"{changed} of {len(stems)} result file(s) changed"
        f"{' (dry run, nothing written)' if args.dry_run else ''}")


if __name__ == "__main__":
    main()
