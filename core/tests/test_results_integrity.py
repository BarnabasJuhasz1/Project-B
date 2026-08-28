"""
core/tests/test_results_integrity.py

results/ and out/runs/ must never disagree about what an episode did.

They did. `results/doorway_traverse__reactive_avoid__seed1001.json` described a
run started at 19:54:19 while `out/runs/doorway_traverse__reactive_avoid__seed1001/`
held the trace of a different invocation started at 20:03:28. The cause was
structural rather than careless: run directories were named after the EPISODE
(scenario, controller, seed), so any re-run replaced the trace belonging to a
committed result. `scripts/check_determinism.sh` does exactly that twice, and
it sends its own results to a temp directory, so it left no trace of its visit
except the overwritten one.

Run directories now carry the invocation's start time and the manifest records
`run_dir`, which makes the correspondence checkable. This is the check.

out/ is gitignored and regenerable, so a run directory that is simply absent is
not a failure — a fresh clone has none of them. A run directory that is present
and says something different is.
"""

import json
import os

import pytest

from core.manifest import run_id

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RESULTS_DIR = os.path.join(REPO_ROOT, "results")


def _results():
    if not os.path.isdir(RESULTS_DIR):
        return []
    return sorted(f for f in os.listdir(RESULTS_DIR) if f.endswith(".json"))


def _load(name):
    with open(os.path.join(RESULTS_DIR, name)) as fh:
        return json.load(fh)


@pytest.mark.parametrize("name", _results())
def test_result_is_named_after_its_scenario_controller_and_seed(name):
    """The filename is the episode's identity and is what a batch runner globs
    for. A result whose filename disagrees with its own manifest is unfindable
    by anything that trusts either one."""
    payload = _load(name)
    m = payload["manifest"]
    expected = run_id(m["scenario_id"], m["controller"], m["seed"]) + ".json"
    assert name == expected


@pytest.mark.parametrize("name", _results())
def test_result_agrees_with_the_run_directory_it_came_from(name):
    """The run directory is the record; results/ is a committed projection of
    it. If both exist they must be the same episode, scored the same way."""
    payload = _load(name)
    manifest = payload["manifest"]
    run_dir = manifest.get("run_dir")

    if run_dir is None:
        # Written before run directories carried the invocation's start time.
        # The legacy location may hold a DIFFERENT invocation, which is the bug
        # this module exists for, so check identity rather than assuming it.
        legacy = os.path.join(REPO_ROOT, "out", "runs",
                              os.path.splitext(name)[0], "manifest.json")
        if not os.path.exists(legacy):
            pytest.skip("no run directory on disk; out/ is gitignored")
        with open(legacy) as fh:
            on_disk = json.load(fh)
        assert on_disk["started_utc"] == manifest["started_utc"], (
            f"{name} describes a run started {manifest['started_utc']} but "
            f"out/runs/{os.path.splitext(name)[0]}/ holds one started "
            f"{on_disk['started_utc']}. Re-run the episode; do not re-score it "
            f"against a trace that belongs to a different invocation.")
        return

    result_json = os.path.join(REPO_ROOT, run_dir, "result.json")
    if not os.path.exists(result_json):
        pytest.skip(f"{run_dir} not on disk; out/ is gitignored")
    with open(result_json) as fh:
        assert json.load(fh) == payload, (
            f"{name} and {run_dir}/result.json have drifted apart. "
            f"scripts/rescore.py rewrites both together; anything else that "
            f"edits one must edit the other.")


@pytest.mark.parametrize("name", _results())
def test_a_rescored_result_says_so(name):
    """A re-scored result was scored by later code than it was run by. All
    three provenance fields go together — one on its own is worse than none,
    because it looks like a complete record."""
    manifest = _load(name)["manifest"]
    stamped = [k for k in ("rescored_utc", "rescored_git_commit", "rescored_from")
               if manifest.get(k) is not None]
    assert len(stamped) in (0, 3), f"{name}: partial re-score provenance {stamped}"
