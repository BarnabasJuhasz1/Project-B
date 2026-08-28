# 28 — The batch runner and aggregation

**Epic:** [04](../04-epic-evaluation.md) · **Depends on:** 17 · **Needs Isaac Sim:** no
**Blocks:** 29, 30, 31, 32 — everything in Week 5 runs through this.

## Why

`PRE-REGISTRATION.md` fixes an episode set: four conditions × (12 held-out + 3 dev) scenarios
× 3 seeds = 180 episodes. `core/stats.py` says what to do with them once they are scored.
Nothing turns the first into the second, and `CLAUDE.md` has named `runs/` as the one planned
addition since epic 00 closed.

The reason this is a chunk and not a shell loop is in the epic's design section: ten hours,
model-generated controllers that can crash the process, and a comparison that is void unless
every condition ran exactly the same episodes.

## Decided

**The design is data, not a loop.** `runs/design.py` derives the episode set from
`PRE-REGISTRATION.md`'s own constants — `core.stats.seeds_for()`, the sealed split, the dev
set — rather than from a list somebody typed. #10 then holds because the same generator
produced every condition's episodes, not because someone checked.

**A condition is a (label, controller) binding, committed.** `composed:<hash>` is neither
readable nor stable, and the evaluation needs to say *which artefact was condition B* in a way
a reader can audit. The binding lives in a committed JSON spec; the sweep reads it and passes
the controller name through to `run_episode.py` unchanged.

**Rejected: one process, many episodes.** See the epic. The determinism claim was measured
through subprocesses and a generated controller may take the process down.

**Rejected: a `--retry` flag.** A crashed episode is recorded with its reason and the sweep
moves on. Retrying until it works has no denominator, which is the same argument
`generate/` and `compose/` already make about refusals.

## Build

- `runs/design.py` — `Condition`, `Episode`, `sweep_design()`. Sim-free. The episode set as a
  pure function of the pre-registered constants. Balanced by construction, and a test that
  asserts it.
- `runs/spec.py` — the committed condition→controller binding, loaded and validated. Refuses
  a scaffolding controller (#3) and refuses a duplicate binding.
- `runs/sweep.py` — the driver. Subprocess per episode, resume from `results/`, per-episode
  cost accounting, a recorded failure rather than an exception, and a pre-flight refusal for
  anything that could not be reported.
- `runs/aggregate.py` — committed results → `core.stats.decide()` plus the four named
  secondaries, per family where the pre-registration says per family.
- `scripts/run_sweep.py` and `scripts/report.py` — the entry points, argparse-first.
- `core/manifest.py` — one `run_id()` helper, so `run_episode.py`, the integrity test and the
  sweep cannot disagree about what an episode is called. A composed controller's name
  contains a colon; the helper is where that is dealt with, once.
- `runs/tests/` — the design is balanced, resume skips exactly what exists, an unbalanced
  aggregation raises, a scaffolding binding is refused, a failed episode is recorded.

## Done when

- `python scripts/run_sweep.py --dry-run` prints the full 180-episode plan without booting
  anything, and the count per condition is identical.
- A rehearsal sweep over the dev set with the reference controllers completes, resumes
  correctly when interrupted, and writes nothing into `results/`.
- `python scripts/report.py` on an incomplete set **refuses**, naming what is missing.
- The pre-flight refuses `in_container: false` without the opt-out.
- The full suite stays green and `rescore.py --dry-run` still reports 0 of 6 changed.

## Not in scope

Running the real sweep. Growing the library. Condition C. Anything that needs the container.

---

## Outcome

**Done 2026-08-26.** 437 → **488 tests**, ~106 s, no simulator. `results/` untouched —
`rescore.py --dry-run` reports 0 of 6 changed, and the six committed files still carry their
Aug 25 timestamps.

Built `runs/design.py`, `runs/spec.py`, `runs/sweep.py`, `runs/aggregate.py`,
`runs/specs/rehearsal.json`, `scripts/run_sweep.py`, `scripts/report.py`, 51 tests in
`runs/tests/`, and a `run_id()` helper in `core/manifest.py`. `runs/` is now covered by the
sim-free import guard.

### The rehearsal found a defect in the sweep itself, which is what a rehearsal is for

The 18-episode dev rehearsal reported **16 ok**. The disk held **11 results**. Five
`return_home` episodes exited **0**, in roughly half the wall clock the same episode takes
standalone, and wrote no result file and no run directory.

Two bugs, and the second is the one that matters:

1. The episodes themselves failed transiently — re-running the identical command by hand
   produced a correct 211 s episode, and a resumed sweep completed all five. The machine was
   running the test suite concurrently, so contention is the likely cause. Not chased further:
   it is a property of a laptop, not of the code.
2. **The sweep believed the exit code and threw the evidence away.** `run_one` kept the stderr
   tail only on a non-zero exit, so an episode that failed *while exiting 0* was recorded as a
   success with no diagnostic at all. `SimulationApp` installs its own shutdown path, so this
   is not exotic — it is the normal way a late failure presents.

Fixed: exit 0 is now a **necessary** condition and the result file is the **sufficient** one,
the stderr is kept either way, and the full stdout+stderr of any non-OK episode is written to
`<sweep log dir>/<run_id>.log` — because three lines are enough to notice a pattern and never
enough to diagnose one. Five tests pin it, including one asserting that a 500-line stdout
survives.

Had this shipped, a real sweep would have reported a complete four-condition table built on
however many episodes silently did not run, and `report.py` would not have caught it either:
the missing files would have made the set unbalanced, so it would have refused — but only
after the ten hours were spent, with no log saying why.

### The pre-registration's central claim now has physics behind it

Chunk 17 argued that replicate seeds are degenerate on undrifted scenarios, measured on the
kinematic rollout. The rehearsal measured it **in the simulator**: all six
(scenario × controller) pairs produced **identical** `goal_reached`, `path_length`, `duration`,
`collisions`, `stuck_events`, `spl`, `min_clearance` and `time_stuck` across all three
replicate seeds. Eighteen real episodes, no exceptions. The resampling-unit decision in
`core/stats.py` is now backed by the apparatus it describes rather than by an idealised model
of it.

### Costs, and one assumption that was wrong

| | assumed | measured |
|---|---|---|
| simulator boot per episode | ~20 s | **32.4 s median** (24.4–39.5, n = 17) |
| wall/sim ratio inside the runner | 1.34 | **1.26 median** (1.17–1.58) |
| full four-condition sweep, worst case | 9.8 h | **10.9 h** |

The boot is 1.6 h across 180 episodes — comparable to a whole condition's worth of episode
time, and the price of subprocess isolation. `PRE-REGISTRATION.md` §3 was updated with the
measured figures; that is an **edit, not an amendment**, and it is legitimate only because
`prereg-v1` does not exist yet. Once it is tagged, the same change would be a dated append.

### Two smaller things the build turned up

**A composed controller's run id contained a colon.** `composed:<hash>` went straight into
`results/<run_id>.json`, and a colon is illegal on NTFS, was the path separator on classic
macOS, and makes `host:path` ambiguous to scp. No existing result had one because no composed
tree has ever run. `core.manifest.run_id()` is now the single definition, shared by
`run_episode.py`, the sweep and `test_results_integrity.py`; the manifest keeps the real
controller name, only the filename is substituted.

**`Verdict.summary()` hardcoded "B" and "D".** Applied to B−A it printed the A condition as
"D". The labels now ride on the verdict, which matters because B−A is the confounded contrast
and a line that mislabels it hides exactly the thing the write-up has to be careful about.

Also: the default logger now flushes. The first eighteen minutes of the rehearsal produced an
empty log file, because `print` is block-buffered to a pipe and a ten-hour sweep is always run
redirected. And `run_sweep` takes a `log_dir`, because the first version of the test suite wrote
sweep logs into the repo's real `out/sweeps/` — which is precisely the kind of artefact that
later gets mistaken for a real run.

### What was NOT done, deliberately

**No condition has run.** The rehearsal binds labels A and B to `goal_seeker` and
`reactive_avoid`, which `results/README.md` forbids reporting and which `reportable: false`
keeps out of `results/`. The spec's note says so and the log prints the note rather than the
canonical condition meaning, so no output of this chunk can be mistaken for an evaluation.

The blocker is chunk 29, not this chunk: the library is one node, and conditions A and D have
zero admitted nodes each. See `plans/04-epic-evaluation.md`.

### For chunk 31

`scripts/run_sweep.py` refuses by default and lists **every** reason at once, so the container
run's first act should be `--dry-run` followed by an un-flagged invocation to see the full
refusal list. Expect `in_container`, `git_dirty`, `prereg_tag` and two `library_snapshot`
complaints to clear together, and nothing else.
