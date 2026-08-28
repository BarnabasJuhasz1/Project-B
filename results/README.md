# Results

One JSON file per episode, committed. `<scenario>__<controller>__seed<N>.json`, two top-level
keys:

- `result` — the scored episode, straight from `core.metrics.score()`
- `manifest` — provenance, straight from `core.manifest.capture()`

A manifest carrying `rescored_utc`, `rescored_git_commit` and `rescored_from` was **scored by
later code than it was run by**: `scripts/rescore.py` re-ran `score()` over the recorded trace
without re-simulating. `git_commit` still names the code that flew the robot. Anything comparing
such a result against a freshly-run episode has to know that the two halves came from different
commits.

## Where a run actually lives

The run directory is the record. `results/` is a committed projection of it.

```
out/runs/<run_id>__<utc>/trace.csv      per-control-step trace
out/runs/<run_id>__<utc>/manifest.json  provenance
out/runs/<run_id>__<utc>/result.json    identical to the file in results/
out/runs/<run_id>__<utc>/episode.mp4    two-pane video, recorded by default
results/<run_id>.json                   the committed copy
```

All of `out/` is **gitignored** — traces and video are large and regenerable, the scored result
and the manifest are not.

`<run_id>` is `<scenario>__<controller>__seed<N>`: the identity of the **episode**, and the name
of the committed result. The run directory appends the start time, making it the identity of one
**invocation**. `manifest["run_dir"]` names the directory a committed result came from, and
`core/tests/test_results_integrity.py` fails if the two ever disagree.

That distinction is not bookkeeping for its own sake. Run directories used to be named after the
episode alone, so any re-run replaced the trace belonging to a committed result.
`scripts/check_determinism.sh` does exactly that twice, and it redirects its own results to a temp
directory, so it left no evidence of the visit except the overwritten trace. It happened:
`doorway_traverse__reactive_avoid__seed1001.json` described a run started at 19:54:19 while
`out/runs/doorway_traverse__reactive_avoid__seed1001/` held a different invocation started at
20:03:28.

## Video

Every run records to `episode.mp4` in its own run directory unless `--no-record` is passed.

Rendering dominates wall clock: **~0.9 s per frame**, and measurement says that is fixed overhead
rather than pixels — 0.87 s/frame at 1280×720 against 0.97 s/frame at 960×540. `--fps` is the only
real lever, which is why the default is 15 rather than 30.

```
15 s episode   21 s headless   240 s at 15 fps   425 s at 30 fps
120 s episode                  ~30 min at 15 fps
```

Rendering never advances physics, so **the trace is byte-identical either way** — the 2026-08-24
re-run of `doorway_traverse__reactive_avoid` reproduced `path_length` to the last bit from a trace
recorded two days and one refactor earlier. What the time buys is the video, nothing else. Use
`--no-record` for seed sweeps, batch runs and anything scored in bulk; the numbers are unaffected.

Reproduce any file here with:

```bash
python scripts/run_episode.py --scenario <scenario> --controller <controller> --seed <N>
```

Reproduce the whole pre-registered set — resumable, one subprocess per episode, refusing before
the first simulator boot if the output could not be reported:

```bash
python scripts/run_sweep.py --dry-run      # the plan, boots nothing
python scripts/run_sweep.py                # the sweep
python scripts/report.py --split held_out  # the pre-registered reading
```

`scripts/report.py` **refuses an incomplete set**, naming which condition is short. That is not
strictness for its own sake: non-negotiable #10 requires identical episode counts, so a report
over whatever finished would compare two different benchmarks and would look exactly like a valid
one. Re-run the sweep — it resumes from this directory.

Re-score one without re-simulating, when a metric constant moves and the trace is still on disk:

```bash
python scripts/rescore.py --dry-run          # show the diff, write nothing
```

## The SPL denominator

`optimal_distance` is the **taut continuous shortest path from the start to the `GOAL_RADIUS`
disc** — `core.geometry.shortest_path_length(..., stop_radius=GOAL_RADIUS)`. It is not the
8-connected grid geodesic, and it is not measured to the goal point. Both of those were true
until 2026-08-24 and between them put about 1.0 m of slack into a ~10.6 m denominator, which
pinned `spl` at exactly 1.0 for every controller within roughly 9% of optimal. See
`plans/done/09-spl-denominator.md`.

`spl_uncapped` is reported alongside `spl`, which is `min(1, spl_uncapped)`. With an exact
denominator the clamp should never bind, so **`spl_uncapped > 1` is a fault signal**, not a
rounding artefact: it means the robot took a shortcut no legal path could take, and there should
be a collision in the same episode to match.

`core.geometry.geodesic_distance` still exists and still decides whether a scenario is solvable
at all — the grid field is exhaustive over the room, so an unreachable goal is `inf` and the
scenario fails to load. It is no longer used for scoring.

## How far the reproducibility claim goes

**Same machine: bitwise.** Two runs of the same scenario, controller and seed on the same machine
produce byte-identical `trace.csv`. Checked by `scripts/check_determinism.sh`, which is the
runnable form of that claim.

**Across machines: statistical equivalence, not bitwise.** GPU physics is not guaranteed bitwise
deterministic across drivers, GPU models or CUDA versions. Anyone comparing conditions across
machines is comparing distributions, not traces. This is a property of the simulator, not
something the code here can fix.

What makes that honest rather than hand-waving is that every manifest records enough to tell two
runs apart: `gpu_name`, `driver_version`, `isaacsim_version`, `container_image`, `in_container`,
`python_version`, `platform`, `git_commit`, `git_dirty`, and `usd_sha256` — the hash of the scene
bytes actually opened, not of whatever `build_scene.py` would produce today.

## What may not be reported

- **`in_container: false`.** Local conda runs are for authoring and debugging. Every reported
  number must come from a headless run inside `nvcr.io/nvidia/isaac-sim:6.0.1`, against the same
  scene assets. `run_episode.py` prints a warning when it is not in the container.
- **`git_dirty: true`.** The result cannot be traced to a commit.
- **Anything produced by `goal_seeker` or `reactive_avoid` as a benchmark condition.** They are
  harness-validation references with no literature input and no generation step. See their
  docstrings.
- **Anything with `library_snapshot: null`, once the library exists.** Non-negotiable #8: every
  reported result names the frozen snapshot tag it came from. The field is in the schema from the
  first run so no result file ever exists without a slot for it.
- **Anything with `prereg_tag: null`, or a `prereg_digest` that does not match the tagged
  `PRE-REGISTRATION.md`.** A result scored under constants nobody registered is not a pre-registered
  result. Both fields are in the schema from before the tag exists, for the same reason as
  `library_snapshot`.

## Pre-registered — `PRE-REGISTRATION.md`, tag `prereg-v1`

**Settled.** The metric constants in `core/metrics.py`, G2's decision threshold X, the seed set,
the episode count and the secondary metric list are all fixed in `PRE-REGISTRATION.md` at the repo
root, and `core/tests/test_preregistration.py` fails if a frozen constant moves without the
document moving with it. Every manifest now carries `prereg_tag` and `prereg_digest` alongside
`library_snapshot` and `held_out_sha256`, so a result proves which pre-registration it ran under —
the tag says which commit was in force, the digest says what the constants actually were, and only
the second is immune to a tag being moved.

The headline numbers, so they are findable without opening the document:

- **X = 0.30**, primary metric goal-reach rate, scored over the twelve held-out scenarios.
- **3 seeds per scenario**, at offsets `(0, 10000, 30000)` from each scenario's own sealed seed.
  36 held-out + 9 dev episodes per condition, 180 in total.
- **Resampling unit is the SCENARIO, not the episode**, 10 000 percentile bootstrap resamples,
  `default_rng(20260826)`. Episode-level resampling would be wrong by roughly √3 here, because on
  the six undrifted held-out scenarios the replicate seeds produce a **bit-identical episode** —
  `Scenario.seed` reaches only `rng()` and `drift_rng()`.
- **The rule is conservative and says so in advance**: `scripts/power_analysis.py` puts it at
  ~0.16 power against a true 30-point gap and ~0.75 against a 50-point one. A rule that does not
  fire may **not** be reported as "the corpus does not help".
- Named secondaries: `spl` (per family, never pooled), `collisions`, `stuck_events`,
  `time_to_goal`. Everything else `score()` returns is descriptive only.

Three items that remain open, all of them stated in the document rather than resolved:

- ~~**Event counts sit on a knife edge.**~~ **FIXED** — `plans/done/10-event-debounce-tolerance.md`.
  `EVENT_RELEASE = 0.5` s is exactly 25 steps at 50 Hz and `_events()` compared against it with no
  tolerance, so whether a span closed came down to the last bits of an accumulated float. Both
  `_events()` and `_stuck_spans()` now carry `EVENT_RELEASE_EPS = 1e-9`, and the boundary direction
  is defined in writing: **a clear gap of exactly `EVENT_RELEASE` closes the span.**

  The audit that chunk asked for found a **second** tie of the same kind that the plan had not
  predicted: `_stuck_spans()` locates its trailing window with
  `searchsorted(t, t[i] - STUCK_WINDOW)`, and `STUCK_WINDOW = 2.0` s is likewise an exact multiple
  of the control step — so the window came up one step short and the sample was silently dropped
  from the stuck signal on **26% of samples at 50 Hz**. Not a rare tie; a systematic one.

  **Every stuck figure in this directory moved as a result**, and toward the documented behaviour
  rather than away from it:

  | run | `stuck_events` | `time_stuck` | `recovery_time` |
  |---|---|---|---|
  | `doorway_traverse__goal_seeker` | 2 → **1** | 78.76 → **111.36** | 39.38 → **111.36** |
  | `blind_corner__goal_seeker` | 4 → **4** | 3.50 → **16.20** | 0.87 → **4.05** |
  | `return_home__goal_seeker` | 1 → **1** | 110.66 (unchanged) | 110.66 (unchanged) |

  `goal_seeker` is documented as wedged for ~110 s of a 120 s episode; 111.36 s of a 119.98 s run
  describes that, and 78.76 s did not. `collisions` and `near_misses` were unaffected on all six
  runs. Re-scoring is now idempotent — a second `--dry-run` reports 0 of 6 changed.

- **`FALL_HEIGHT` is a height, and a fall is a tilt.** Base z varies by ~40% across a gait cycle
  on this asset (0.128–0.213 m measured). 0.08 is safely below that, but base roll/pitch would be
  a far more robust criterion. It needs orientation on the trace, and 11a closed the field list —
  so it is **deferred with its caveat attached**, not fixed.
- **`contact` is analytic, not physical.** Both `Observation.contact` and the trace column are
  `clearance(x, y) - ROBOT_RADIUS <= 0`, with `ROBOT_RADIUS = 0.30` circumscribing a 0.65 × 0.31 m
  body. No physics contact sensor is wired to the Go2 yet, so the collision count is a geometric
  proxy. **Settled as deferred**: `collisions` IS a named secondary, and the caveat is
  pre-registered to travel with every figure rather than the metric being withheld.

## The trace gained a column

`TRACE_COLUMNS` now ends with **`goal_known`** — whether the goal had been disclosed to the
controller at that step. Every trace recorded before it existed is still re-scorable:
`EpisodeTrace.read_csv()` matches columns **by name** and fills a missing one from
`TRACE_BACKFILL`, which maps `goal_known` to `True` because every episode recorded before the
goal could be withheld ran on a scenario that disclosed it from t=0. An unknown column, or a
missing one with no backfill, is still a hard error.

Verified rather than assumed: re-running `blind_corner__reactive_avoid__seed1002` on the changed
code produced a trace **byte-identical in all 343 rows** to the committed one once the new column
is stripped, and `scripts/rescore.py --dry-run` reports 0 of 6 result files changed.

## The held-out split is sealed

`core/scenarios_held_out.py`, **SHA-256 `00b3312b0755cc94d252ccb2eb6a096cf0867aae19ef61c6c71cb31572870a63`**,
recorded in `core/tests/test_held_out.py` so a change to the split is a failing test rather than a
silent re-definition of what G3 measured, and stamped into every run's manifest as
`held_out_sha256`. Twelve scenarios, three in each cell of a 2 × 2:

|  | goal disclosed | goal withheld |
|---|---|---|
| **no drift** | `navigate_to_goal` (3) | `search_unmapped` (3) |
| **drift on** | `return_degraded` (3) | `search_degraded` (3) |

The axes are crossed rather than confounded, so each mechanism family's contribution is separable
without a further ablation. `search_degraded` is the family carrying G3's claim: it is the only one
unsolvable by any single primitive, since the robot must explore for an undisclosed goal *while*
its pose estimate decays. Seeds are `2000 + position`, stated as a rule so a seed added or moved
after someone saw the results is visible by inspection. Six of the twelve carry a `DriftSpec` and
**refuse to load** until chunk 13 implements the drift model.

Worst-case cost, for chunk 17's episode-count arithmetic: **1920 s of sim time per condition per
seed** if every episode runs to timeout, or roughly 45 min wall clock at the measured 1.4× headless
ratio. Four conditions and a seed set multiply that.

**What sealing does not buy.** Nothing under `controllers/`, `nodes/`, `generate/` or `compose/`
can import the split — checked by AST — and `core.scenario` reaches it only from inside a function,
so importing `core.scenario` does not drag it in. `--help` on `run_episode.py` does not print the
ids, and `scripts/seal_split.py` prints the split's shape but never its content. **None of that
protects against the split having been in an LLM's context in an earlier session.** Generation is
LLM-driven and its context is not a sandbox. The mitigation is procedural and no test can see it:
generation sessions must never have had the split in context, and it must never be pasted into a
prompt. The session that authored the split is itself contaminated for generation.

⚠️ **`git tag split-v1` has not been applied** — `plans/`, `core/` and `results/` are still
untracked, so there is no sealing commit to tag yet. Tag it on the commit that first tracks
`core/scenarios_held_out.py`, and check the hash above still matches.

## `localisation_failures` is not a performance metric

Measured, not argued. Drift is injected open-loop — `run_episode.py` records
`drift.estimate()` straight into the trace — so **no controller can influence the recorded pose
error**. On a drifted `doorway_traverse` the localisation error at step 500 is *bit-identical*
across `goal_seeker` and `scaffold_sweep`, controllers that behave nothing alike, and both score
exactly 2 failures because both run the full 6000 steps. `reactive_avoid` scores 0 only because it
finished in 798.

A controller can lower this number **only by finishing sooner**. So it measures the drift model and
the episode duration, not localisation competence.

It is still worth recording, as a **manipulation check** — did drift actually bite? — and it must
not be reported as a performance secondary, or "condition B localises better" will be read out of a
number that cannot show it. **`PRE-REGISTRATION.md` §5 states it is the former**, which is what
chunk 17 owed.
`cards/store.py` enforces the consequence at load: a card predicting it is rejected as not
falsifiable about the node, alongside `optimal_distance`, `scenario` and `seed`.

## Decided for this cycle, not yet built

Recorded here because each one binds what may be reported, and because the plan files they came
from will move into `plans/done/` where they are easier to miss. All of them are now collected in
`PRE-REGISTRATION.md`, committed and tagged `prereg-v1` **before the first evaluation run** —
`plans/done/17-pre-registration.md`.

- **The held-out split varies routes *and* task families** (option B), twelve scenarios, three
  each across a 2 × 2 of goal-disclosure × drift: `navigate_to_goal`, `search_unmapped`,
  `return_degraded`, `search_degraded`. A `resume_after_bump` family was considered and dropped —
  it needs an impulse applied in the simulator, and a topple terminates the episode as `fell`, so
  tuning it interacts with the scored metrics.
- **`Observation.goal` becomes `Optional`.** `None` means the goal has not been disclosed to the
  controller; it latches to a real coordinate on the first step where the goal is within
  `DETECT_RADIUS = 2.0` m **and** in line of sight, and never returns to `None`. `Scenario.goal`
  is unchanged and still scores every episode — ground truth still never enters `Observation`.
- **Localisation drift is a seeded random walk** with a magnitude parameter, off by default. Every
  number in this directory was produced with drift off and must be unchanged by its introduction.
- **Condition A is given the node contract.** The description's "writes the stack cold" is
  ambiguous and this is an interpretation of it; the consequence is that this cycle **cannot report
  what the decomposition alone buys**, which would need a fifth condition.
  ⚠️ **B−A does NOT isolate the corpus** — the earlier claim here is withdrawn. The condition
  prompts ask D to articulate a principle before implementing and do not ask A to, so B−A confounds
  the corpus with the act of principle articulation. **Only B−D isolates the corpus**, which is
  consistent with the plan's own ranking. `PRE-REGISTRATION.md` §1 is the resolution.

## Out of scope this cycle

Both are stated here rather than omitted, because a reader of §4 will look for them.

- **Legibility, and therefore `DeviateToDisambiguate`.** Dragan, Lee & Srinivasa (2013) measure an
  *observer's* inference about the robot's intent. None of `score()`'s eighteen keys is a property
  of anyone's belief, and a legible path is by construction longer than the optimal one — so the
  node would score negatively on SPL and the admission gate would reject it for a reason that has
  nothing to do with whether the mechanism works. **No legibility metric is defined, and none is
  reported.** Marked for a later cycle: adding one needs a candidate-goal set per scenario, an
  observer model over `core.geometry`'s geodesic costs, a scalar metric and a new frozen constant
  for observer rationality, all pre-registered before anything is reported.
- **Area-restricted search, and therefore `RestrictSearchArea`** — which is the description's own
  running example. ARS concentrates search around an encounter *rate*; the contract above yields a
  single goal detected once, so there is no rate and no gradient to climb. Making it testable
  needs a cue field or multiple targets, and neither was added, deliberately, to keep the frozen
  contract to a single change. The search family still tests coverage, path integration and
  geometric reorientation — three separable mechanisms.
- **`RequireSensorAgreement`** is structurally untestable rather than merely deferred: there is
  one sensing modality, a 2D range scan, so there is nothing to cross-check against.
