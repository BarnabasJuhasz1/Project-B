"""
runs/sweep.py

Runs the episode set `runs/design.py` describes, one subprocess at a time.

    from runs.sweep import run_sweep
    log = run_sweep(spec, plan, results_dir="results")

What this does that a shell loop does not:

- **Resumes** from `results/`, not from a state file. An episode is done if its
  committed result exists. Two sources of truth about what has run would
  disagree exactly once, at hour eight.
- **Refuses to produce a number that may not be reported.** `in_container`,
  `git_dirty`, the pre-registration tag and the library snapshot are checked
  BEFORE the first simulator boot, not discovered afterwards.
- **Records a failure instead of raising or retrying.** A crashed episode is an
  outcome with a reason; retrying until it works has no denominator, which is
  the same argument `generate/` and `compose/` already make about refusals.
- **Survives an interrupt.** Ctrl-C writes the log and exits, and the next
  invocation picks up exactly where it stopped.

One subprocess per episode, deliberately -- see `plans/04-epic-evaluation.md`.
The short version: a model-generated controller can take the process down,
`World` is a singleton with known re-entrancy gotchas, and the bitwise
determinism claim was measured through subprocesses.

**Held-out scenario ids are not printed** unless `--reveal` is passed. Same
discipline as `scripts/seal_split.py` and `run_episode.py --help`: a sweep log
is the kind of thing that gets pasted into a chat window, and the split reaching
a model's context is the one failure sealing cannot survive.
"""

import datetime
import hashlib
import json
import os
import subprocess
import sys
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional

from core.manifest import capture
from core.scenario import DEFAULT_SCENE
from runs.design import DEV, HELD_OUT

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# How long an episode may take before the sweep gives up on it, as a multiple of
# its own sim time limit plus a fixed allowance.
#
# The multiple covers the headless ratio (measured 1.32-1.34x on the authoring
# GPU) with room for a slower machine; the constant covers the simulator boot,
# which is ~20 s and is per-episode because every episode is its own process.
# Generous on purpose: a timeout that fires on a slow-but-working episode
# converts a real result into a recorded failure, which is worse than waiting.
TIMEOUT_RATIO = 4.0
TIMEOUT_FIXED_S = 300.0

# Where a sweep's own record goes. Under out/, which is gitignored: the log is
# regenerable and large, the scored results are neither.
SWEEP_DIR = os.path.join("out", "sweeps")

# Episode outcomes. A sweep produces one of these per episode and never an
# exception, because an exception loses the other 179.
OK = "ok"
SKIPPED = "skipped"          # already had a committed result
FAILED = "failed"            # non-zero exit, or exit 0 with no result written
TIMED_OUT = "timed_out"
INTERRUPTED = "interrupted"

# A subprocess exiting 0 is NOT evidence that an episode ran.
#
# Measured, on the chunk-28 rehearsal: five `return_home` episodes exited 0, in
# roughly half the wall clock the same episode takes standalone, and wrote no
# result file and no run directory. The sweep recorded all five as `ok` and
# threw their stderr away, because the stderr tail was only kept on a non-zero
# exit. The sweep log said 16 ok; the disk held 11.
#
# `SimulationApp` installs its own shutdown path, so a failure late in a run can
# still leave the process exiting 0. The exit code is therefore a necessary
# condition and the RESULT FILE is the sufficient one, and the stderr is kept
# whichever way it goes -- "condition D failed on everything" and "condition D
# failed on everything with the same AttributeError" are different findings.
VERIFY_RESULT_WRITTEN = True

# Substrings of a scenario id that mark it as belonging to the sealed split, for
# the redaction in `describe()`. Derived from the split's own naming convention
# rather than by importing the ids, so this file never holds one.
HELD_OUT_PREFIX = "held_"


# ---------------------------------------------------------------------------


def _log(message):
    """The default logger, and it FLUSHES.

    Plain `print` is block-buffered when stdout is a pipe, which is how a
    ten-hour sweep is always run -- redirected to a file somebody tails. The
    first eighteen minutes of the rehearsal produced an empty log file for
    exactly this reason. A progress line that arrives an hour late is not
    progress.
    """
    print(message, flush=True)


def describe(episode, reveal=False):
    """One episode, for a log line. Redacts held-out ids unless asked.

    A dev id is printed in full: those are in `--help` already. A held-out id
    becomes `<family #nnn>`, where nnn is a stable digest of the id -- enough to
    correlate two log lines about the same scenario, not enough to reconstruct
    the split. The family is not redacted: `results/README.md` names all four.

    `hashlib`, not `hash()`. Python randomises string hashing per process, so
    the built-in would give a scenario a different label on every invocation and
    the correlation the redaction exists to preserve would be lost.
    """
    if reveal or episode.split != HELD_OUT:
        return f"{episode.scenario}/{episode.condition}/seed{episode.seed}"
    tag = hashlib.sha256(episode.scenario.encode()).hexdigest()[:3]
    return (f"<{episode.family} #{tag}>"
            f"/{episode.condition}/seed{episode.seed}")


@dataclass
class EpisodeRun:
    run_id: str
    condition: str
    scenario: str
    seed: int
    split: str
    status: str
    wall_seconds: float = 0.0
    reason: str = ""


@dataclass
class SweepLog:
    """A sweep's own record. Written after every episode, not at the end.

    Written incrementally on purpose: the failure mode this guards against is a
    ten-hour sweep dying at hour nine and leaving no account of the nine.
    """

    name: str
    started_utc: str
    spec: str
    results_dir: str
    reportable: bool
    runs: List[EpisodeRun] = field(default_factory=list)
    path: Optional[str] = None

    def counts(self):
        out = {}
        for r in self.runs:
            out[r.status] = out.get(r.status, 0) + 1
        return out

    def wall_seconds(self):
        return sum(r.wall_seconds for r in self.runs)

    def write(self):
        if self.path is None:
            return
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        payload = {
            "name": self.name, "started_utc": self.started_utc,
            "spec": self.spec, "results_dir": self.results_dir,
            "reportable": self.reportable, "counts": self.counts(),
            "wall_seconds": round(self.wall_seconds(), 1),
            "runs": [asdict(r) for r in self.runs],
        }
        with open(self.path, "w") as fh:
            json.dump(payload, fh, indent=2, sort_keys=True)


class Unreportable(Exception):
    """Raised by preflight. Not a warning: `results/README.md` says these
    results may not be reported, and a sweep that produced them anyway would
    fill the committed directory with files nobody may cite."""


def preflight(spec, results_dir, allow_unreportable=False):
    """Everything that makes a sweep's output un-citable, checked before the
    first 20 s simulator boot rather than after 180 of them.

    Returns the probe manifest so the caller can log what it saw.
    """
    probe = capture(scene=DEFAULT_SCENE)
    problems = []

    if not probe["in_container"]:
        problems.append(
            "in_container is false. Every reported number must come from a "
            "headless run inside nvcr.io/nvidia/isaac-sim:6.0.1 against the "
            "same scene assets (results/README.md).")
    if probe["git_dirty"]:
        problems.append(
            "git_dirty is true, so no result could be traced to a commit.")
    if probe["git_dirty"] is None:
        problems.append("not a git repository, so there is no commit to trace to.")
    if not probe["prereg_tag"]:
        problems.append(
            "prereg_tag is null. PRE-REGISTRATION.md must be committed and "
            "tagged prereg-v1 BEFORE the first evaluation run -- a "
            "pre-registration tagged afterwards is not one.")
    for condition in spec.conditions:
        if not condition.library_snapshot:
            problems.append(
                f"condition {condition.label} names no library snapshot (#8).")

    if not spec.reportable:
        problems.append(
            f"the spec {spec.name!r} declares reportable: false, so its output "
            f"is a rehearsal and not an evaluation.")

    if problems and not allow_unreportable:
        raise Unreportable(
            "this sweep would produce results that may not be reported:\n  - "
            + "\n  - ".join(problems)
            + "\n\nPass --allow-unreportable for a rehearsal, and send its "
              "output somewhere that is not results/ with --results-dir.")
    return probe, problems


def already_done(plan, results_dir):
    """Run ids whose committed result is already on disk.

    Existence is the whole check. A result file that exists but is corrupt will
    fail `core/tests/test_results_integrity.py`, which is where that belongs --
    validating it here would be a second, weaker copy of that test.
    """
    full = results_dir if os.path.isabs(results_dir) \
        else os.path.join(REPO_ROOT, results_dir)
    if not os.path.isdir(full):
        return set()
    have = {n[:-len(".json")] for n in os.listdir(full) if n.endswith(".json")}
    return {e.run_id for e in plan.episodes if e.run_id in have}


def episode_command(episode, spec, results_dir, python=None):
    """The exact subprocess a sweep runs. Public so `--dry-run` can print it and
    a human can paste one line and reproduce a single episode by hand."""
    condition = spec.condition(episode.condition)
    cmd = [python or sys.executable,
           os.path.join(REPO_ROOT, "scripts", "run_episode.py"),
           "--scenario", episode.scenario,
           "--controller", episode.controller,
           "--seed", str(episode.seed),
           # Never record. Rendering is ~0.9 s/frame and buys a video nobody
           # watches for 180 episodes; the trace is byte-identical either way.
           "--no-record",
           # Uncapped. run_episode.py caps an episode at 60 sim-seconds by
           # default so an interactive run does not spend half an hour
           # rendering, and it RENAMES the scenario when the cap bites. An
           # evaluation must run to the scenario's own pre-registered
           # time_limit, and a renamed scenario would not match the plan's
           # run_id at all - so the sweep says so rather than inheriting a
           # default that is about wall clock.
           "--max-seconds", "0",
           "--results-dir", results_dir]
    if condition.library_snapshot:
        cmd += ["--tag", condition.library_snapshot]
    return cmd


def run_one(episode, spec, results_dir, python=None, env=None, log_dir=None):
    """One episode, in its own process. Returns an EpisodeRun, never raises.

    A non-zero exit is an outcome. The last few lines of stderr go into
    `reason`, because "condition D failed on everything" and "condition D failed
    on everything with the same AttributeError" are very different findings and
    only the second is actionable.
    """
    cmd = episode_command(episode, spec, results_dir, python=python)
    timeout = episode.time_limit * TIMEOUT_RATIO + TIMEOUT_FIXED_S
    started = time.monotonic()

    def outcome(status, wall, reason=""):
        return EpisodeRun(episode.run_id, episode.condition, episode.scenario,
                          episode.seed, episode.split, status, wall, reason)

    try:
        proc = subprocess.run(cmd, cwd=REPO_ROOT, capture_output=True,
                              text=True, timeout=timeout, env=env)
    except subprocess.TimeoutExpired:
        return outcome(TIMED_OUT, time.monotonic() - started,
                       f"no exit after {timeout:.0f}s wall "
                       f"({episode.time_limit:.0f}s sim limit)")
    wall = time.monotonic() - started

    def tail_of(text, n=3):
        lines = [l for l in (text or "").strip().splitlines() if l.strip()]
        return " | ".join(lines[-n:])[:500]

    if proc.returncode != 0:
        reason = (tail_of(proc.stderr) or tail_of(proc.stdout)
                  or f"exit {proc.returncode}")
        _keep_output(log_dir, episode, proc)
        return outcome(FAILED, wall, reason)

    # Exit 0 is not enough. See VERIFY_RESULT_WRITTEN.
    if VERIFY_RESULT_WRITTEN and not _result_exists(results_dir, episode):
        _keep_output(log_dir, episode, proc)
        return outcome(FAILED, wall,
                       "exit 0 but no result file was written. "
                       + (tail_of(proc.stderr) or tail_of(proc.stdout)
                          or "no output captured"))
    return outcome(OK, wall)


def _result_exists(results_dir, episode):
    full = results_dir if os.path.isabs(results_dir) \
        else os.path.join(REPO_ROOT, results_dir)
    return os.path.exists(os.path.join(full, episode.result_name))


def _keep_output(log_dir, episode, proc):
    """The FULL stdout and stderr of a failed episode, on disk.

    Three lines in the log are enough to notice a pattern and never enough to
    diagnose one, and a simulator traceback is a hundred lines under a thousand
    lines of Omniverse warnings. This is the artefact you actually want at
    hour nine of a ten-hour sweep.
    """
    if not log_dir:
        return
    try:
        os.makedirs(log_dir, exist_ok=True)
        with open(os.path.join(log_dir, episode.run_id + ".log"), "w") as fh:
            fh.write(f"$ exit {proc.returncode}\n\n--- stdout ---\n")
            fh.write(proc.stdout or "")
            fh.write("\n--- stderr ---\n")
            fh.write(proc.stderr or "")
    except OSError:
        # A sweep must not die because it could not write a log about a death.
        pass


def run_sweep(spec, plan, results_dir="results", allow_unreportable=False,
              python=None, reveal=False, log=_log, name=None, log_dir=None):
    """The sweep. Returns a SweepLog whichever way it ends.

    Scenario-major order comes from the plan, not from here -- see
    `runs.design.sweep_design`.
    """
    probe, problems = preflight(spec, results_dir, allow_unreportable)
    for p in problems:
        log(f"[sweep] UNREPORTABLE: {p}")

    started = datetime.datetime.now(datetime.timezone.utc)
    stamp = started.strftime("%Y%m%d-%H%M%S")
    name = name or spec.name or "sweep"
    sweep_log = SweepLog(
        name=name, started_utc=started.isoformat(timespec="seconds"),
        spec=spec.name, results_dir=results_dir,
        reportable=spec.reportable and not problems,
        path=os.path.join(log_dir or os.path.join(REPO_ROOT, SWEEP_DIR),
                          f"{name}__{stamp}", "sweep.json"))

    done = already_done(plan, results_dir)
    todo = [e for e in plan.episodes if e.run_id not in done]
    log(f"[sweep] {len(plan)} episodes, {len(done)} already scored, "
        f"{len(todo)} to run")
    log(f"[sweep] worst case {plan.worst_case_sim_seconds():.0f}s of sim; "
        f"log -> {os.path.relpath(sweep_log.path, REPO_ROOT)}")

    for episode in plan.episodes:
        if episode.run_id in done:
            sweep_log.runs.append(EpisodeRun(
                episode.run_id, episode.condition, episode.scenario,
                episode.seed, episode.split, SKIPPED))
            continue
        try:
            result = run_one(episode, spec, results_dir, python=python,
                             log_dir=os.path.dirname(sweep_log.path))
        except KeyboardInterrupt:
            # Not swallowed: recorded, written, and re-raised so the caller
            # decides. Resume works because results/ already holds whatever
            # finished, so nothing is lost by stopping here.
            sweep_log.runs.append(EpisodeRun(
                episode.run_id, episode.condition, episode.scenario,
                episode.seed, episode.split, INTERRUPTED))
            sweep_log.write()
            log(f"[sweep] interrupted; {sweep_log.counts()}. Re-run to resume.")
            raise
        sweep_log.runs.append(result)
        sweep_log.write()
        mark = "ok " if result.status == OK else result.status.upper()
        log(f"[sweep] {mark} {describe(episode, reveal)} "
            f"{result.wall_seconds:.1f}s"
            + (f" -- {result.reason}" if result.reason else ""))

    sweep_log.write()
    counts = sweep_log.counts()
    log(f"[sweep] done: {counts}, {sweep_log.wall_seconds() / 60:.1f} min wall")
    ran = [r for r in sweep_log.runs if r.status == OK]
    if ran:
        # The measurement PRE-REGISTRATION.md §3 asks for: re-measure the
        # headless ratio on the benchmark machine rather than trusting a laptop.
        sim = sum(e.time_limit for e in plan.episodes
                  if e.run_id in {r.run_id for r in ran})
        log(f"[sweep] {sweep_log.wall_seconds() / len(ran):.1f}s wall per "
            f"episode; worst-case wall/sim ratio "
            f"{sweep_log.wall_seconds() / sim:.2f} (episodes that ended early "
            f"pull this below the true per-second cost)")
    return sweep_log
