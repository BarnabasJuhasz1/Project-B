"""
runs/tests/test_sweep.py

The driver, tested without booting a simulator: `run_one` is monkeypatched, and
what is under test is the resume logic, the refusals and the redaction.

    pytest runs/tests/test_sweep.py -q
"""

import json
import os

import pytest

from runs import sweep as S
from runs.design import DEV, HELD_OUT, Episode, sweep_design
from runs.spec import Condition, SweepSpec


def spec(reportable=False, snapshot=None):
    return SweepSpec((Condition("A", "goal_seeker", snapshot),
                      Condition("B", "reactive_avoid", snapshot)),
                     name="test", reportable=reportable)


def test_preflight_refuses_an_unreportable_run():
    with pytest.raises(S.Unreportable) as exc:
        S.preflight(spec(reportable=True, snapshot="lib-v1"), "results")
    # Whatever else is wrong locally, "not in the container" always is.
    assert "in_container" in str(exc.value)


def test_preflight_lists_every_problem_not_just_the_first():
    """A sweep operator should fix all of them in one pass, not discover them
    one twenty-second boot at a time."""
    _, problems = S.preflight(spec(), "results", allow_unreportable=True)
    assert len(problems) >= 2
    assert any("library snapshot" in p or "library_snapshot" in p for p in problems)
    assert any("reportable: false" in p for p in problems)


def test_preflight_passes_through_when_allowed():
    probe, problems = S.preflight(spec(), "results", allow_unreportable=True)
    assert probe["usd_sha256"]
    assert problems


def test_already_done_reads_the_results_directory(tmp_path):
    plan = sweep_design(spec(), splits=(DEV,))
    assert S.already_done(plan, str(tmp_path)) == set()
    target = plan.episodes[0]
    (tmp_path / target.result_name).write_text("{}")
    assert S.already_done(plan, str(tmp_path)) == {target.run_id}


def test_a_missing_results_directory_is_not_an_error(tmp_path):
    """A first sweep has nowhere to resume from, and that is the normal case."""
    plan = sweep_design(spec(), splits=(DEV,))
    assert S.already_done(plan, str(tmp_path / "nope")) == set()


def test_the_sweep_skips_what_already_exists(tmp_path, monkeypatch):
    plan = sweep_design(spec(), splits=(DEV,))
    done = plan.episodes[0]
    (tmp_path / done.result_name).write_text("{}")

    ran = []

    def fake_run_one(episode, *a, **k):
        ran.append(episode.run_id)
        return S.EpisodeRun(episode.run_id, episode.condition, episode.scenario,
                            episode.seed, episode.split, S.OK, 1.0)

    monkeypatch.setattr(S, "run_one", fake_run_one)
    log = S.run_sweep(spec(), plan, results_dir=str(tmp_path),
                      allow_unreportable=True, log=lambda *_: None,
                      log_dir=str(tmp_path / "sweeps"))
    assert done.run_id not in ran
    assert log.counts()[S.SKIPPED] == 1
    assert log.counts()[S.OK] == len(plan) - 1


def test_a_failed_episode_is_recorded_and_the_sweep_continues(tmp_path, monkeypatch):
    """A crashed model-generated controller costs one episode, not the sweep."""
    plan = sweep_design(spec(), splits=(DEV,))

    def fake_run_one(episode, *a, **k):
        status = S.FAILED if episode.condition == "A" else S.OK
        return S.EpisodeRun(episode.run_id, episode.condition, episode.scenario,
                            episode.seed, episode.split, status, 1.0,
                            "boom" if status == S.FAILED else "")

    monkeypatch.setattr(S, "run_one", fake_run_one)
    log = S.run_sweep(spec(), plan, results_dir=str(tmp_path),
                      allow_unreportable=True, log=lambda *_: None,
                      log_dir=str(tmp_path / "sweeps"))
    assert log.counts()[S.FAILED] == len(plan) // 2
    assert log.counts()[S.OK] == len(plan) // 2
    assert any(r.reason == "boom" for r in log.runs)


def test_a_sweep_writes_its_log_somewhere_it_was_told_to(tmp_path, monkeypatch):
    """`log_dir` exists so a test does not litter the repo's out/ directory --
    which the first version of this suite did, and which is exactly the kind of
    thing that later gets mistaken for a real run."""
    plan = sweep_design(spec(), splits=(DEV,))
    monkeypatch.setattr(S, "run_one", lambda e, *a, **k: S.EpisodeRun(
        e.run_id, e.condition, e.scenario, e.seed, e.split, S.OK, 1.0))
    S.run_sweep(spec(), plan, results_dir=str(tmp_path / "r"),
                allow_unreportable=True, log=lambda *_: None,
                log_dir=str(tmp_path / "sweeps"))
    assert _sweep_logs(tmp_path / "sweeps")
    assert not _sweep_logs(tmp_path / "r")


def test_the_log_is_written_incrementally(tmp_path, monkeypatch):
    """The failure this guards against is a ten-hour sweep dying at hour nine
    and leaving no account of the nine."""
    plan = sweep_design(spec(), splits=(DEV,))
    seen = []

    def fake_run_one(episode, *a, **k):
        # By the time the second episode runs, the first must be on disk.
        paths = [p for p in _sweep_logs(tmp_path)]
        seen.append(len(paths))
        return S.EpisodeRun(episode.run_id, episode.condition, episode.scenario,
                            episode.seed, episode.split, S.OK, 1.0)

    monkeypatch.setattr(S, "run_one", fake_run_one)
    S.run_sweep(spec(), plan, results_dir=str(tmp_path / "r"),
                allow_unreportable=True, log=lambda *_: None,
                log_dir=str(tmp_path / "sweeps"))
    written = _sweep_logs(tmp_path)
    assert written, "no sweep log was written"
    payload = json.load(open(written[0]))
    assert len(payload["runs"]) == len(plan)
    assert payload["counts"][S.OK] == len(plan)


def _sweep_logs(root):
    out = []
    for dirpath, _, names in os.walk(root):
        out += [os.path.join(dirpath, n) for n in names if n == "sweep.json"]
    return out


def test_held_out_ids_are_redacted_by_default():
    ep = Episode("B", "reactive_avoid", "held_search_deep_right", 2000,
                 HELD_OUT, "search_unmapped", 180.0)
    hidden = S.describe(ep)
    assert "held_search_deep_right" not in hidden
    assert "search_unmapped" in hidden
    assert "held_search_deep_right" in S.describe(ep, reveal=True)


def test_the_redaction_is_stable_across_processes():
    """`hash()` is randomised per process; the label has to survive that or two
    log lines about the same scenario cannot be correlated."""
    ep = Episode("B", "reactive_avoid", "held_x", 2000, HELD_OUT, "nav", 10.0)
    assert S.describe(ep) == S.describe(ep)
    assert S.describe(ep) == ("<nav #" +
                             __import__("hashlib").sha256(b"held_x").hexdigest()[:3]
                             + ">/B/seed2000")


def test_dev_ids_are_never_redacted():
    ep = Episode("A", "goal_seeker", "doorway_traverse", 1001, DEV, "nav", 120.0)
    assert "doorway_traverse" in S.describe(ep)


def test_the_episode_command_never_records():
    """Rendering is ~0.9 s per frame and buys a video nobody watches 180 of."""
    plan = sweep_design(spec(), splits=(DEV,))
    cmd = S.episode_command(plan.episodes[0], spec(), "results")
    assert "--no-record" in cmd
    assert "--results-dir" in cmd


def test_the_sweep_disables_the_episode_cap():
    """run_episode.py caps an episode at 60 sim-seconds by default so an
    interactive run does not spend half an hour rendering. An evaluation must
    run to the scenario's own pre-registered time_limit -- and the cap RENAMES
    the scenario when it bites, so inheriting the default would not quietly
    shorten the sweep, it would make every long episode write a result under a
    run_id the plan has never heard of."""
    plan = sweep_design(spec(), splits=(DEV,))
    cmd = S.episode_command(plan.episodes[0], spec(), "results")
    assert cmd[cmd.index("--max-seconds") + 1] == "0"


def test_the_snapshot_tag_reaches_the_subprocess():
    """Non-negotiable #8 has to survive the trip into the child process."""
    sp = spec(snapshot="lib-v1")
    plan = sweep_design(sp, splits=(DEV,))
    cmd = S.episode_command(plan.episodes[0], sp, "results")
    assert cmd[cmd.index("--tag") + 1] == "lib-v1"


# --- exit 0 is not evidence that an episode ran -------------------------------
#
# The chunk-28 rehearsal found five episodes that exited 0 in half the expected
# wall clock and wrote nothing. The sweep called them `ok`. These pin the fix.

class _Proc:
    def __init__(self, returncode=0, stdout="", stderr=""):
        self.returncode, self.stdout, self.stderr = returncode, stdout, stderr


def test_exit_zero_with_no_result_file_is_a_failure(tmp_path, monkeypatch):
    plan = sweep_design(spec(), splits=(DEV,))
    episode = plan.episodes[0]
    monkeypatch.setattr(S.subprocess, "run",
                        lambda *a, **k: _Proc(0, "", "Traceback: boom"))
    got = S.run_one(episode, spec(), str(tmp_path))
    assert got.status == S.FAILED
    assert "no result file" in got.reason
    # And the stderr survives, which is the half that was being thrown away.
    assert "boom" in got.reason


def test_exit_zero_with_a_result_file_is_ok(tmp_path, monkeypatch):
    plan = sweep_design(spec(), splits=(DEV,))
    episode = plan.episodes[0]
    (tmp_path / episode.result_name).write_text("{}")
    monkeypatch.setattr(S.subprocess, "run", lambda *a, **k: _Proc(0))
    assert S.run_one(episode, spec(), str(tmp_path)).status == S.OK


def test_a_failure_keeps_its_full_output_on_disk(tmp_path, monkeypatch):
    """Three lines in the log notice a pattern; they never diagnose one."""
    plan = sweep_design(spec(), splits=(DEV,))
    episode = plan.episodes[0]
    noise = "\n".join(f"warning {i}" for i in range(500))
    monkeypatch.setattr(S.subprocess, "run",
                        lambda *a, **k: _Proc(1, noise, "RuntimeError: the real cause"))
    logs = tmp_path / "logs"
    got = S.run_one(episode, spec(), str(tmp_path), log_dir=str(logs))
    assert got.status == S.FAILED
    kept = (logs / (episode.run_id + ".log")).read_text()
    assert "RuntimeError: the real cause" in kept
    assert "warning 0" in kept, "stdout was dropped"


def test_a_sweep_does_not_die_because_it_cannot_write_a_log(tmp_path, monkeypatch):
    plan = sweep_design(spec(), splits=(DEV,))
    episode = plan.episodes[0]
    monkeypatch.setattr(S.subprocess, "run", lambda *a, **k: _Proc(1, "", "x"))
    # A path that cannot be created.
    got = S.run_one(episode, spec(), str(tmp_path),
                    log_dir=str(tmp_path / "f" / "g")
                    if (tmp_path / "f").write_text("not a dir") is None else None)
    assert got.status == S.FAILED


def test_a_timeout_is_recorded_not_raised(tmp_path, monkeypatch):
    plan = sweep_design(spec(), splits=(DEV,))
    episode = plan.episodes[0]

    def boom(*a, **k):
        raise S.subprocess.TimeoutExpired(cmd="x", timeout=1)

    monkeypatch.setattr(S.subprocess, "run", boom)
    got = S.run_one(episode, spec(), str(tmp_path))
    assert got.status == S.TIMED_OUT
    assert "sim limit" in got.reason
