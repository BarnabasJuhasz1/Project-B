# Pre-registration

**Status: committed and tagged `prereg-v1` before the first evaluation run.**
Written 2026-08-26, during Week 1, at the end of epic 02 and before any of Week 5.

This document fixes, in advance, everything about the Week 5 evaluation that could
otherwise be chosen after seeing the numbers: the primary metric, the decision threshold,
the scenario and seed set, the episode count, the named secondary metrics, every scoring
constant, and the resampling procedure behind the confidence intervals.

The entire evidential value of a pre-registration is that it **demonstrably predates the
first run**. That is why it is a tagged commit rather than a section of the write-up, why
`core/tests/test_preregistration.py` fails when a frozen constant moves without this file
moving with it, and why every result manifest records the digest below.

**Amendments are dated appends, never edits.** If something here turns out to be wrong or
unworkable, add a new entry under [Amendments](#amendments) with the date, what changed and
why, and leave the original text standing. The write-up reports both versions. An amendment
made after the first evaluation run is reported as such, and any analysis affected by it is
reported both ways.

---

## 1. What is being tested

The hypothesis is that **injecting principles extracted from research literature into the
code-generation step produces materially better navigation code than a frontier model
produces alone**. No model is trained. Improvement, if it exists, comes from program
synthesis under selection over a frozen locomotion policy.

Four conditions, identical frozen locomotion policy, identical scene, identical scenario
set, identical seed set, identical episode count (non-negotiable #10):

| | Condition | Status at the time of writing |
|---|---|---|
| **A** | Naive LLM, given the node contract, no literature | generator built, 1 real call, node rejected |
| **B** | Literature-informed LLM, card store available | generator built, 2 real calls, 1 node admitted |
| **C** | Human baseline, off-the-shelf tuned Nav2 stack | **not built** — see §9 |
| **D** | Invented principles, same model, no corpus | generator built, 1 real call, node rejected |

### Which contrast carries which claim

- **B − D is the headline and the only clean contrast.** It separates genuine information
  gain from the model restating what it already knew.
- **B − A is confounded and is reported as such.** The condition prompts ask D to articulate
  a principle before implementing and do not ask A to, because that is what makes D a
  different condition from A rather than a rewording of it. B − A therefore confounds the
  corpus with the act of principle articulation. `CLAUDE.md` previously claimed B − A
  isolates the corpus alone; **that claim is withdrawn here**, which is the resolution chunk
  17 owed. See `generate/FINDINGS.md` and `plans/done/15-generator.md`.
- **B − C is the applied comparison** and depends on C existing.
- This cycle **cannot report what the decomposition into typed leaves alone buys**. That
  needs a fifth condition with no node contract, and #10 forbids adding one at a different
  episode count. Recorded as a known limitation.

---

## 2. Primary metric and the decision rule

**Primary metric: goal-reach rate** — the fraction of episodes with
`result["goal_reached"] == True`, straight from `core.metrics.score()`. Nothing else enters
the primary rule. There is no composite score, no re-weighting, and no alternative
definition of success held in reserve.

**Scored over: the twelve sealed held-out scenarios.** The dev set is scored and reported
too, separately and descriptively, and never as the headline (§4).

### The rule

> **B's goal-reach rate exceeds D's by at least X = 0.30 (30 points), and the two 95%
> cluster-bootstrap confidence intervals do not overlap.**

Implemented in `core.stats.decide()`, which is the rule — this prose is a description of
that function, not a second source of truth. The same rule is applied to B − A and, if C
exists, to B − C; only B − D carries the headline claim.

### The bootstrap, concretely

Reimplementable from this alone, and implemented in `core.stats`:

| | |
|---|---|
| **Resampling unit** | the **scenario**, not the episode. 12 clusters. |
| **Procedure** | reduce each scenario to its mean success over its 3 seeds; resample the 12 scenario means with replacement; take the mean. |
| **Pairing** | one index draw per resample, applied to **both** conditions. B and D ran the same scenarios; resampling them apart would model an experiment nobody performed. |
| **Resamples** | 10 000 |
| **Interval** | percentile, 95%. Not BCa: with twelve clusters the bias-correction and acceleration terms are themselves estimated from twelve numbers, and a correction estimated that badly is noise dressed as rigour. |
| **RNG** | `numpy.random.default_rng(20260826)`, fixed here, before any result exists |

### Why the resampling unit is the scenario, and why it costs so much

Episode-level resampling is wrong here, and not marginally.

1. Episodes are **clustered** within scenarios. Success on one seed of a scenario says a
   great deal about its other seeds and rather little about a different scenario. Treating
   them as independent understates every interval.
2. On the six **undrifted** held-out scenarios the seeds are not merely correlated, they are
   **identical**. `Scenario.seed` reaches exactly two places — `rng()`, handed to the
   controller, and `drift_rng()`. A controller that does not draw from its rng, run on a
   scenario with no drift, produces a bit-identical episode at every seed. Measured on
   `reactive_avoid`, `goal_seeker` and `toy_search_tree`: identical trace hash across seeds
   1002 / 7 / 999 with drift off, three different hashes with drift on. Episode-level
   resampling would count one episode three times and shrink the interval by √3 for free.

The price is that the effective sample size is the **number of scenarios**, and the sealed
split has twelve. That is the binding constraint on this experiment's power. It is recorded
rather than worked around, because the alternative is an interval that is narrow and wrong.

### What this rule cannot do

`plans/done/17-pre-registration.md` carried a table of "smallest detectable X" derived from
`1.96/√N` for two independent proportions at the episode level. **That table does not apply
to this design and is optimistic.** `scripts/power_analysis.py` simulates the actual
estimator — scenario-level clustering, degenerate seeds where the design makes them
degenerate, and `core.stats.decide()` itself. Power at 12 scenarios, 3 seeds, D's rate 0.40,
1000 replications per cell, moderate between-scenario heterogeneity:

| true B − D gap | rule fires (disjoint intervals) | paired difference CI excludes 0 |
|---|---|---|
| 0.10 | 0.01 | 0.07 |
| 0.20 | 0.04 | 0.23 |
| 0.30 | 0.16 | 0.51 |
| 0.40 | 0.40 | 0.74 |
| 0.50 | 0.75 | 0.93 |

Reproduce with `python scripts/power_analysis.py --reps 1000`.

Three things follow, and all three are pre-registered rather than left to be discovered:

- **The rule is conservative and is expected not to fire on a real 20-point advantage.**
  A non-firing rule therefore **may not be reported as "the corpus does not help."** It may
  be reported as "the pre-registered rule did not fire," alongside the observed gap and both
  intervals.
- **X = 0.30 is not what limits the rule; the interval criterion is.** X's job is to be a
  floor on what is worth claiming that does not bind before the intervals do. 0.30 is the
  largest value with that property.
- **Adding seeds cannot fix this.** Going from 3 replicates to 10 moves power by a few points
  (0.75 → 0.82 at a true gap of 0.50) and multiplies wall clock by 3.3. Adding *scenarios*
  would fix it, and the split is sealed at `00b3312b…` and is never re-sealed.

### The paired secondary, and how the four outcomes are read

The paired difference — mean over scenarios of (B success − D success), with a 95% percentile
CI from the same resampled indices — is computed and reported **always**, whichever way the
primary rule comes out. It is a secondary, not a tie-breaker. Its interpretation is fixed here:

| primary fires | paired CI excludes 0 | reported as |
|---|---|---|
| yes | yes | **B beats D.** The headline claim. |
| no | yes | A consistent per-scenario advantage of the observed size, **not resolvable by the pre-registered rule at twelve scenarios.** Reported as suggestive and underpowered. Explicitly **not** the headline. |
| no | no | Null. No evidence the corpus helps at this scale. |
| yes | no | Should not occur. If it does, something is wrong with the analysis; report it and investigate before claiming anything. |

Deciding which of the two to believe after seeing them is the exact move a pre-registration
exists to forbid, which is why the table above is written before any of them exist.

---

## 3. The episode set

### Scenarios

- **Held-out (the decision rule's set): the twelve scenarios in `core/scenarios_held_out.py`**,
  sha256 `00b3312b0755cc94d252ccb2eb6a096cf0867aae19ef61c6c71cb31572870a63`. Three each in
  each cell of a 2 × 2 of goal-disclosure × drift: `navigate_to_goal`, `search_unmapped`,
  `return_degraded`, `search_degraded`.
- **Dev (descriptive only): the three scenarios in `core.scenario.DEV_SCENARIOS`** —
  `doorway_traverse`, `blind_corner`, `return_home`.

### Seeds

Replicate *k* of a scenario runs at `scenario.seed + SEED_OFFSETS[k]`, with

```
SEED_OFFSETS = (0, 10000, 30000)
```

stated as a rule so that a seed added or moved after someone saw the results is visible by
inspection. Replicate 0 leaves `Scenario.seed` alone, so the first replicate of every held-out
scenario **is** the episode chunk 11 sealed and chunk 13 verified.

**20000 is skipped, by a stated rule rather than a preference.** The offsets are the first
three multiples of 10000 at which every drifted held-out scenario's pose error crosses
`LOC_FAIL_THRESHOLD` inside its own time limit. At 20000 one of the six never crosses, so
drift would be nominally on and functionally absent for that episode — a manipulation that
did not take. This selection cannot favour a condition: drift is injected open-loop into a
separate RNG stream and is bit-identical across conditions, which is exactly the property
that makes `localisation_failures` a manipulation check rather than a performance metric
(§5). Reproduce with `python scripts/check_seed_offsets.py --scan 5`.

### Counts

| | scenarios | seeds | episodes per condition | episodes, 4 conditions |
|---|---|---|---|---|
| held-out | 12 | 3 | **36** | 144 |
| dev | 3 | 3 | 9 | 36 |
| | | | **45** | **180** |

Identical across all four conditions. A condition that crashes on some scenarios is **not**
scored over the remainder — `core.stats.decide()` raises rather than silently comparing two
different benchmarks.

### Cost

Measured, not assumed, over the 18-episode dev rehearsal in `runs/` (chunk 28), headless with
`--no-record` on the local RTX 4060:

| | measured |
|---|---|
| wall/sim ratio **inside** the runner | median **1.26** (range 1.17–1.58; short episodes amortise startup worse) |
| **simulator boot per episode** | median **32.4 s** (range 24.4–39.5, n = 17) |
| held-out time limits | 1920 s per condition per seed if every episode times out |

The boot is per episode because every episode is its own process, and it is **not negligible**:
across 180 episodes it is **1.6 h**, comparable to a whole condition's worth of episode time. It
was assumed to be ~20 s before it was measured. It buys crash isolation from model-generated
controllers and keeps the bitwise determinism claim in the configuration it was measured in
(`scripts/check_determinism.sh` runs subprocesses), and that trade is accepted deliberately —
see `plans/04-epic-evaluation.md`.

Worst case for a full four-condition sweep: 26 460 s ≈ 7.3 h of sim, × 1.26 ≈ 9.3 h, plus 1.6 h
of boot ≈ **10.9 h of wall clock**. Real sweeps will be well under that, since a successful
traverse ends in seconds. Every reported number comes from
`nvcr.io/nvidia/isaac-sim:6.0.1`, so this is container time.

**Re-measure both figures on the benchmark machine before the first sweep** and record them; the
figures above are from a laptop GPU that is explicitly not the benchmark target, taken with other
work running on the same machine. `scripts/run_sweep.py` prints them at the end of every sweep.

---

## 4. Which scenarios G2 reports over

Both sets are run and reported, **separately, never pooled**:

- **Held-out is the headline.** The decision rule in §2 applies to the held-out set alone.
- **Dev is reported descriptively**, labelled as the set the prompts were iterated against.
  No decision rule is applied to it, and no claim rests on it.

Pooling them is forbidden here because they answer different questions — dev measures the
pipeline where development happened, held-out measures transfer — and because "we report the
one that worked" is the exact failure this document exists to prevent.

**G2 and G3 are computed from the same held-out runs and are different quantities.** G2 is the
between-condition comparison of goal-reach rate. G3 is the fraction of held-out scenarios
solved by recombination alone from a library frozen at a tagged snapshot, reported per
condition. One set of runs, two readings, both stated in advance.

---

## 5. Metrics

### Primary

`goal_reached`, as above.

### Named secondaries — four, fixed here

Reporting all eighteen keys of `score()` and highlighting the significant ones is multiple
comparisons by another name. These four are named in advance; each is reported for every
condition whether or not it favours B.

| secondary | reported how | caveat that travels with it |
|---|---|---|
| `spl` | **per family, never pooled** | the denominator is the omniscient taut path, so SPL is near-meaningless for `search_unmapped` and `search_degraded`; pooling across four families produces a number that means nothing in particular |
| `collisions` | count per episode | `contact` is an **analytic proxy** — `clearance − ROBOT_RADIUS ≤ 0` with a 0.30 m circle round a 0.65 × 0.31 m body. No physics contact sensor is wired to the Go2. |
| `stuck_events` | count, with `time_stuck` | sat on a float knife edge until chunk 10; sound since, and `rescore.py` is idempotent |
| `time_to_goal` | seconds, successes only | undefined on failures, so it is conditioned on success and must be read next to the success rate |

`spl_uncapped` is reported alongside `spl`. **`spl_uncapped > 1` is a fault signal**, not a
rounding artefact: the robot took a shortcut no legal path could take, and there should be a
collision in the same episode.

### Descriptive only

Everything else `score()` returns — `near_misses`, `recovery_time`, `time_stuck`,
`min_clearance`, `path_length`, `optimal_distance`, `duration`, `control_steps`,
`termination` — is reported as a table, marked descriptive, and **no claim rests on it**. In
particular no significance test is run on any of them.

### `localisation_failures` is a manipulation check, not a performance metric

Declared here because chunk 14 required it to be declared. Drift is injected open-loop:
`run_episode.py` records `drift.estimate()` straight into the trace and `drift_rng()` is a
separate stream from the controller's, so **no controller can influence the recorded pose
error**. Measured: on a drifted `doorway_traverse` the localisation error at step 500 is
bit-identical across `goal_seeker` and `scaffold_sweep`, and both score exactly 2 failures
because both run the full 6000 steps; `reactive_avoid` scores 0 only because it finished in
798. A controller can lower it **only by finishing sooner**.

So it is reported to answer *did drift actually bite*, and it is **never** reported as a
performance secondary. `cards/store.py` refuses a card predicting it, and
`test_localisation_failures_really_is_controller_independent` pins the property.

### `termination == "fell"` carries a caveat

`FALL_HEIGHT = 0.08` is a **height**, and a fall is really a tilt. Base z varies by ~40% across
a gait cycle on this asset (0.128–0.213 m measured), so 0.08 m is safely below the walking
minimum and unambiguously a collapsed posture — but roll/pitch would be a far more robust
criterion and is not on the trace. Fall counts are reported with this sentence attached.

---

## 6. G1 and G3

**G1 — traceable primitive library and coverage count.** Reported per condition, for **every**
condition, whether or not the performance difference materialises:

- node count admitted into the frozen library snapshot,
- unit pass rate, computed from `generate/staging/` — **every** candidate is the denominator,
  admitted or rejected, and rejections are committed for that reason,
- fraction of nodes carrying a **verified** source span, from `CardStore.provenance()`.

⚠️ **The provenance fraction is 0.0 today and will be reported as 0.0 unless
`cards/VERIFICATION.md` is worked through first.** There is no corpus on the authoring
machine, so no span has been checked against a paper. An unverified span is never counted as
traceable — that is the honest state, and reporting a generated span as provenance would make
G1's provenance number a measure of fluency.

⚠️ **The library is one admitted node plus four handwritten fixture leaves that can never be
a condition**, against G1's target of ≥20. If the library is still this size at Week 5, G1's
coverage count is reported as what it is and the shortfall is stated, not padded.

**G3 — compositional generalisation.** Fraction of held-out scenarios solved by recombination
alone, no new primitives, reported for the literature-informed and the naive library
separately. "No new primitives" is structural, not a rule anyone has to keep: composition
returns a JSON tree over node ids and never Python, and the library is frozen at a tagged
snapshot before any held-out scenario is composed for.

**Every reported result names the frozen library snapshot tag** it was produced from
(non-negotiable #8) and the pre-registration digest below.

---

## 7. The frozen constants

Every constant whose value changes what a reported number **means**. Rationale for each lives
next to it in the source, under that file's `data, not code` divider; this block is the
record that they held these values before the first run.

`core/tests/test_preregistration.py` compares this block against the live values, name by
name, and fails on any disagreement. **Changing a constant without editing this file is a
failing test.** Bumping the digest alone does not satisfy it — the values are checked
individually, so the new number has to be written here where a reader will see it.

```prereg-constants
core.geometry.GRID_RESOLUTION = 0.05
core.geometry.ROBOT_RADIUS = 0.3
core.geometry.ARC_STEP_DEG = 2.0
core.metrics.GOAL_RADIUS = 0.5
core.metrics.FALL_HEIGHT = 0.08
core.metrics.COLLISION_CLEARANCE = 0.05
core.metrics.NEAR_MISS_CLEARANCE = 0.25
core.metrics.EVENT_RELEASE = 0.5
core.metrics.EVENT_RELEASE_EPS = 1e-09
core.metrics.STUCK_WINDOW = 2.0
core.metrics.STUCK_DISPLACEMENT = 0.1
core.metrics.STUCK_COMMAND_EPS = 0.05
core.metrics.LOC_FAIL_THRESHOLD = 1.0
core.scenario.N_BEAMS = 181
core.scenario.FOV = 6.283185307179586
core.scenario.RANGE_MAX = 6.0
core.scenario.MAX_LIN = 0.8
core.scenario.MAX_ANG = 1.0
core.scenario.DETECT_RADIUS = 2.0
core.scenario.DEFAULT_SCENE = 'scenes/lab42_v1.usd'
harness.unit.MIN_EFFECT_RATE = 0.5
harness.unit.MIN_EFFECT_REL = 0.1
core.stats.RESAMPLING_UNIT = 'scenario'
core.stats.N_RESAMPLES = 10000
core.stats.CI_METHOD = 'percentile'
core.stats.CI_LEVEL = 0.95
core.stats.BOOTSTRAP_SEED = 20260826
core.stats.X = 0.3
core.stats.SEED_OFFSETS = (0, 10000, 30000)
core/scenarios_held_out.py = 'sha256:00b3312b0755cc94d252ccb2eb6a096cf0867aae19ef61c6c71cb31572870a63'
scenes/lab42_v1.usd = 'sha256:01d083b1e44837d968107964c8c9f5c341372bfefd755d8512145f19452a28b1'
```

prereg-digest: `9f9bfb61d96edc41d9af71a04b2a337a33595674d463d26b497e184c402bb330`

A one-line summary of the load-bearing choices, for a reader who will not open the source:

- **`GOAL_RADIUS = 0.5`** — one body length. Also the disc the SPL denominator is measured to,
  so numerator and denominator cannot drift apart.
- **`DETECT_RADIUS = 2.0`** — bounded on both sides by `validate()`. Below `GOAL_RADIUS`,
  detection coincides with arrival and search tests nothing; above `RANGE_MAX`, the goal
  arrives at a range the sensor could not have resolved it at, which is ground truth entering
  `Observation` through the back door.
- **`EVENT_RELEASE = 0.5` with `EVENT_RELEASE_EPS = 1e-9`** — a clear gap of *exactly*
  `EVENT_RELEASE` closes a span. The epsilon exists to make an equality decidable, not to be
  tuned: without it the same episode scored differently depending on whether timestamps were
  accumulated or read back from disk.
- **`MIN_EFFECT_RATE = 0.5` / `MIN_EFFECT_REL = 0.10`** — calibrated on the two reference
  controllers, whose capabilities are known and neither of which is a generated node with an
  interest in the answer. A unit scenario may **raise** its own floor and never lower it.
- **`N_BEAMS`, `FOV`, `RANGE_MAX`, `MAX_LIN`, `MAX_ANG`** — the sensor suite and actuation
  limits belong to the task, not the controller. If a condition could move one, the ablation
  would be measuring the sensor.

The two hashed artefacts are frozen by content: swapping the scene or re-sealing the split is
as loud a failure as moving a threshold.

---

## 8. Threats to validity, declared in advance

Each of these was found during epic 02 and is written here rather than in the discussion
section, so that it cannot be presented as a nuance discovered after the numbers.

1. **The admission harness cannot detect card infidelity.** A deliberately contentless card
   (`generate/tests/fixtures/vague_adaptive_motion.yaml`) was **admitted**: the model filled
   the card's vague mechanism with a real one of its own and the real one passed on its own
   merits. `harness/unit.py` tests whether the *node* works, not whether the node implements
   the *card*, and only the first is mechanical.
   **Decision: this is a stated limitation, absorbed by B − D, not a pre-registered fidelity
   check.** A fidelity check needs a human judge scoring node-against-card, which is a rater
   study this cycle has no budget for, and a badly-run one would be worse than none. B − D
   is the control that absorbs it: if B's nodes are partly model-invented mechanisms wearing
   card names, then B and D are drawing on the same source and B − D shrinks toward zero. The
   contrast is therefore conservative with respect to this threat, which is the reason the
   plan ranks it first.
2. **The generator designs its own micro-world, so the effect floor can always be cleared.**
   Requiring the baseline to actually fail incentivises an adversarial world. **A cleared
   effect floor is not evidence the mechanism generalises**, and the write-up says so.
3. **A composition can succeed and produce a tree that does not work.** A committed pair
   demonstrates it: same library, same retrieval, one reaches the goal at SPL 0.96 and one
   starves a node behind a RUNNING sibling in a `Sequence` and times out. Nothing in a port
   contract says how often a precondition fires, and that is what decides which of the two
   you get. **Composition success rate and goal-reach rate are reported separately** and the
   first is never used as a proxy for the second.
4. **The one admitted node has two effects, which `nodes/contract.py` forbids.** It writes
   `admissible_speed` *and* drives toward the goal. Neither the harness nor composition can
   detect this. **Decision: an Action that drives may not also publish a port** — that is a
   second effect, and it is what makes a unit scenario unable to isolate one claimed effect.
   Enforcement needs an admission check the harness does not have; building it re-opens
   admission for every existing node, so it is a named follow-up chunk and not chunk 17.
   **The existing violation is reported as a known defect, not silently fixed**, and any
   result involving that node carries the note.
5. **Generation is reproducible, not repeatable.** `temperature` was removed from the
   Messages API on every current model, so "temperature 0 gives a byte-identical candidate"
   is unavailable. The same prompt produced different outcomes twice, demonstrably. Replaced
   by committed transcripts and `ReplayBackend`, which rebuilds a library byte-for-byte with
   no model, no key and no network — a stronger property than temperature 0 would have given.
   **Pre-registered consequence: every reported condition is produced by `--backend replay`
   from committed transcripts**, so a reported number is reproducible even though a fresh
   generation call would not reproduce it.
6. **A refusal or a malformed response is an outcome, not a retry.** Nothing is re-rolled
   until it is liked. G1's pass rate and G3's fraction both need a denominator, and a
   pipeline that quietly retries has none. Already the practice; recorded here so it is a
   commitment.
7. **Across machines, reproducibility is statistical, not bitwise.** GPU physics is not
   guaranteed bitwise deterministic across drivers, GPU models or CUDA versions. Anyone
   comparing conditions across machines compares distributions. Every manifest records
   `gpu_name`, `driver_version`, `isaacsim_version`, `container_image`, `in_container`,
   `python_version`, `platform`, `git_commit`, `git_dirty` and `usd_sha256`, so two runs can
   always be told apart. **All four conditions of a given sweep run on the same machine.**
8. **No composed tree has run in the simulator yet.** Every number produced so far comes from
   `core/tests/kinematic.py`, an idealised unicycle with the frozen locomotion policy absent
   entirely. Nothing from it is a reported result.

---

## 9. What is not reported, and why

- **`in_container: false`.** Local conda runs are authoring and debugging only. Every reported
  number comes from a headless run inside `nvcr.io/nvidia/isaac-sim:6.0.1` against the same
  scene assets.
- **`git_dirty: true`.** The result cannot be traced to a commit.
- **Anything produced by `goal_seeker`, `reactive_avoid`, `scaffold_sweep`, `toy_tree` or
  `toy_search_tree` as a benchmark condition.** They are harness-validation references and
  scaffolding with no literature input and no generation step (non-negotiable #3).
- **Anything with `library_snapshot: null`** (non-negotiable #8).
- **Any legibility metric.** Out of scope this cycle, and therefore `DeviateToDisambiguate`
  is out too. Its dependent variable is an *observer's* inference about intent and none of
  `score()`'s eighteen keys is a property of anyone's belief; a legible path is by
  construction longer than the optimal one, so the node would score negatively on SPL and the
  gate would reject it for a reason unrelated to whether the mechanism works. Out on cost,
  not on merit.
- **Area-restricted search, and therefore `RestrictSearchArea`** — the description's own
  running example. ARS concentrates search around an encounter *rate*; the frozen contract
  yields a single goal detected once, so there is no rate and no gradient. Making it testable
  needs a cue field or multiple targets, and neither was added, deliberately. *The running
  example turned out not to be measurable under our own contract* is itself a finding about
  the pipeline and is reported as one.
- **`RequireSensorAgreement`** — structurally untestable rather than deferred. One sensing
  modality, nothing to cross-check against.
- **Condition C, if the Nav2 stack is not built in time.** It is not built at the time of
  writing. If it does not exist for the sweep, **B − C is not reported at all** and its
  absence is stated; the missing condition is never substituted for by a reference controller.
  B − D and B − A are unaffected, because §3's counts are per condition.

---

## 10. Deferred, with reasons

Items that `CLAUDE.md` or `results/README.md` marked open and that this document does **not**
resolve. Each is deferred deliberately, with the reason, rather than left ambiguous.

| Item | Why it is deferred |
|---|---|
| A tilt-based fall criterion in place of `FALL_HEIGHT` | needs roll/pitch on the trace, which is a change to `TRACE_COLUMNS`, and 11a declared the field list closed. Reported with its caveat instead. |
| A physics contact sensor in place of the analytic `contact` proxy | needs sim-side work on the Go2 articulation; `collisions` is reported with its caveat instead. |
| Real SLAM in place of `core/drift.py`'s open-loop random walk | epic `03-epic-slam.md`, chunks 19–27, deliberately **not next**. It must not land between now and the first evaluation run, or the drift model that these seed offsets and the sealed split's magnitudes were calibrated against changes underneath them. |
| An admission check for effect multiplicity | §8.4. Re-opens admission for every existing node. |
| A card-fidelity rater study | §8.1. |
| Verifying the five card source spans | needs a corpus, which is not on the authoring machine. Until then G1's provenance fraction is 0.0 and is reported as 0.0. |
| A fifth condition isolating the node contract | #10 requires identical episode counts; adding one is a separate cycle. |
| `git tag split-v1` on the held-out seal | there was no sealing commit to tag when chunk 11 ran, because the tree was untracked. Apply it to the commit that first tracks `core/scenarios_held_out.py` and re-check the hash then. |

---

## Amendments

Three, all dated 2026-08-31. Add dated entries below; never edit above this line.

- **G1's reported coverage target** is a common ceiling every condition reaches on
  equal terms, not ≥20. Supersedes §6's target. Nothing in G2 moves.
- **The provenance fraction is 0.80**, not the 0.0 §6 states. The corpus arrived and
  four of five spans were verified.
- **`git tag split-v1` is applied**, closing that §10 deferral.

### 2026-08-31 — G1's reported coverage target: a common ceiling, not ≥20

**What changed.** §6 states G1's target as ≥20 admissible nodes per condition and warns that
the library is one node. That warning understated the problem: **the shortfall is structural,
not a budget shortfall**, and no amount of generation budget removes it.

`generate/generator.py:render()` is a pure function of `(condition, task, card)` — no nonce,
no sample index, no seed — and `Candidate.node_id` **is** the prompt hash. Two calls with the
same inputs therefore produce the same node id and the same staging stem, and the second
overwrites the first. With `TASKS` naming three families and `cards/` holding five cards, the
number of *distinct* nodes reachable without changing anything is:

| Condition | Distinct prompts | Ceiling if every one is admitted |
|---|---|---|
| A | 3 tasks | **3** |
| B | 5 cards × 3 tasks | **15** |
| D | 3 tasks | **3** |

**A and D cannot reach 20 by spending more calls.** The binding constraint is the
prompt-identity design, not the corpus. This is asserted by `generate/tests/test_ceiling.py`
rather than left as prose, and a condition added to `CONDITIONS` without being costed fails
that test.

**The decision.** G1 is reported against **the largest count every condition reaches on equal
terms**, not against 20. Concretely: 3 today, and **4** if chunk 29 takes the fourth cell of
11a's frozen `goal_known` × `pose_drifts` contract (hidden goal under drift), which `TASKS`
currently leaves unused. The reported number is whatever that common ceiling turns out to be,
stated **against the target of 20 with the arithmetic above as the reason**. The shortfall is
reported, never padded — which is what §6 already said, now with a number attached to why.

**Why this and not the alternatives**, both of which were considered and rejected:

- *Actually reach ≥20.* Needs new task families well beyond the free fourth cell, or a
  declared sample index in the prompt. The latter changes the prompts, orphaning every
  committed transcript — the exact ground on which chunk 15 rejected a prompt change — and it
  changes the elicitation: asking one model for *n* distinct mechanisms is a different
  measurement from asking it once, so the resulting coverage count is no longer the quantity
  the pilot measured. Rejected on scope and on measurement validity.
- *Grow B with more cards and cap A and D.* Rejected explicitly rather than silently. Having
  cards **is B's treatment**, so enumerating over them lifts B alone and makes library size a
  second, uncontrolled difference between conditions. A larger library composes better for
  reasons that have nothing to do with the corpus and would look identical in the numbers.
  That is a worse confound than the B−A confound §1 already accepts.

**The rule this establishes, which binds anything later:** *any axis used to enumerate more
nodes within a condition must be condition-neutral* — available in identical form to A, B and
D. Cards fail that test. Task cells derived from the frozen contract pass it. Failure
signatures from scored episodes pass it by construction, since the same signature is handed to
all three.

⚠️ **A task family invented by reading the held-out split is leakage and voids G3.** Chunk 11's
procedural rule applies to the fourth cell: authored in a session that has never had
`core/scenarios_held_out.py` in context.

**What this does not change.** Nothing in §2, §3, §4 or §7. X stays 0.30, the seed set, the
episode counts and the frozen constants are untouched, and the digest is unaffected. #10 holds:
identical inputs per condition, identical episode counts. This amendment changes **what number
G1 is reported against**, and nothing about G2.

### 2026-08-31 — §6's provenance fraction of 0.0 is superseded: it is 0.80

§6 states, twice, that the provenance fraction "is 0.0 today and will be reported as 0.0
unless `cards/VERIFICATION.md` is worked through first", on the grounds that there was no
corpus on the authoring machine. **That work has since been done.** A corpus was obtained,
and on 2026-08-27 four of the five card spans were verified against the PDFs actually on disk,
mechanically extracted by `scripts/testing/fetch_and_verify.py`, with each page number checked
against the **printed** page rather than the PDF page index.

`CardStore.provenance()` now returns `traceable_fraction: 0.8` — 5 cards, 4 traceable, 3
distinct mechanisms. Three cards had their Confidence **raised** on verification
(`reorient_geometry` low → high, `path_integration_ants` medium → high,
`path_integration_mammals` low → high) and one held at medium (`limit_speed_dwa`: the closed
form is the paper's, but the card measures distance along the intended heading where the paper
measures it along the corresponding curvature — one substitution that is ours).

`limit_speed_ics` remains **unverified on purpose**: the PDF obtained is the IROS 2003
conference paper, and the card cites the *Advanced Robotics* 18(10) journal version. Different
documents, different pagination. It is listed as outstanding with that reason rather than
verified against the wrong edition, which is the behaviour §6 asks for.

The principle §6 states is unchanged and is what produced this number: **an unverified span is
never counted as traceable.** Only the value moves — 0.0 → 0.80 — and it moves because the
work was done, not because the criterion was loosened. §10's deferred entry "Verifying the
five card source spans" is closed by this amendment for four of five.

### 2026-08-31 — §10's deferred `git tag split-v1` is closed

§10 defers `git tag split-v1` on the ground that "there was no sealing commit to tag when
chunk 11 ran, because the tree was untracked". The tree is now tracked: `a84cd46` is the
commit that first tracks `core/scenarios_held_out.py`, and **both `split-v1` and `prereg-v1`
are annotated tags pointing at it**. The sealed-split hash `00b3312b…` was re-checked at that
commit and is unchanged. The seal is never re-sealed.

### 2026-09-06 — the conditions are restructured: A becomes citation-only, the naive arm is retired

**This amendment changes what the letters mean.** It is an append, not an edit, and the
superseded definitions stay above it exactly as written, because every committed transcript,
staged candidate and result was produced under them.

**What changed.** Condition A was "naive LLM — no literature input, but given the node contract".
It is now **the citation arm**: A is handed the `source.citation` of a card — authors, title,
venue, year — and **none** of the extracted content. B is handed the same reference *with* the
card. The pure-naive arm, which received no literature at all, is **retired**; it is not
reassigned to another letter and it will not be run.

**Why.** §1 already records that B−A confounds the corpus with the act of principle
articulation, and that only B−D isolates the corpus. That confound is now removed at the source
rather than absorbed: A and B are pointed at the same paper and differ by exactly one thing, the
paper's content, so

> **B−A is the measure of what EXTRACTION buys** — the same paper, read by a human or not.

This is the comparison `generate/probe.py` already names as `B − E` and answers only at the level
of a reconstructed card. Promoting it to a condition answers it at the level of nodes and
episodes, which is the level G1 and G2 report at.

**Consequences that are pre-registration material:**

1. **A's ceiling changes.** A now enumerates over the card store, so its reachable node count is
   `|cards| x |TASKS|` = 20, the same as B's, where it was `|TASKS|` = 4. The 2026-09-05 append
   recording chunk 29's Option 2 stands, but **it now binds only the condition with no corpus
   axis**. `generate/tests/test_ceiling.py` carries the numbers: (a 20, b 20, d 4).
2. **G1's provenance fraction is protected by a new origin.** `nodes.contract.Origin.CITATION`
   records a node that was pointed at a source and never given its content. Such a node may
   carry **no source span and no extraction confidence** — both are products of a human reading
   the paper — and `NodeMetadata` raises if either is set. **A citation node is therefore never
   traceable**, and it enters G1's denominator and not its numerator. This is the honest state
   and it is enforced, not conventional.
3. **A is scored against its own prediction, never the card's.** Condition B's unit scenario is
   taken from the card's `Prediction`. A's is not: it builds its own micro-world and states its
   own claim, as an invented node does. Handing A the card's metric, baseline and minimum effect
   would give it the extracted content through the back door and B−A would measure nothing.
4. **Four records are retired from the counts.** The four condition-A candidates generated on
   2026-09-05 under the old definition (one admitted, three rejected) were produced by prompts
   that are no longer reachable. They remain on disk with a `retired_reason`, are reported under
   `_retired` by `generate/admit.py:pass_rate`, and are **excluded from A's pass rate and from
   the library** — pooling them with the new A would compute one number over two different
   treatments. The library snapshot consequently returns to `lib-006-23c57a77269e`, the tag the
   committed compositions name.
5. **Condition D is unchanged by this amendment and is still `d`.** Its planned successor —
   condition **C**, the same no-corpus arm given failure context from scored episodes — is
   specified in `plans/todo/34-condition-c-failure-context.md` and is **not built**. The
   off-the-shelf Nav2 human baseline that previously held the letter C is **retired and will not
   be run this cycle**; the letter is being reused, and the write-up must say so plainly rather
   than let a reader assume the baseline exists.

**What this amendment does NOT change.** G2's decision rule, X = 0.30, the seed set, the episode
count, the sealed split, the secondary metric list and the bootstrap procedure are all untouched.
The rule is still stated over B and the no-corpus arm; when that arm becomes C, the rule's
referent changes name and not content, and that renaming will be its own dated append.

### 2026-09-06 — the corpus is deliberately in-distribution, and a second unseen corpus is planned

Declared in advance rather than conceded afterwards, because it governs what the headline
comparison is allowed to claim.

**The five pilot cards are drawn from work the model can be expected to know.** This is a design
choice, not a limitation discovered late. Confirmed with the first citation-arm run on
2026-09-06: condition A was handed each reference alone and **recognised all four it was asked
about**, naming Müller & Wehner (1988), Etienne & Jeffery (2004), Cheng (1986) and Fox, Burgard &
Thrun (1997) correctly and unprompted. Recall is therefore not the variable under test on this
corpus — it is held high and constant on purpose.

**What that buys.** With knowledge of the paper held constant across A and B, the only thing
separating them is whether a human turned the finding into a rule for this robot. So on this
corpus:

> **B−A is the measure of EXTRACTION** — what the Derivation step buys, given a model that
> already knows the source.

`limit_speed_dwa` is the demonstration and it is committed: A reconstructed the Dynamic Window
Approach faithfully — velocity-space search, the dynamic window, the admissibility bound — and
was **rejected**; B, given the card's derivation of the same finding onto a 181-beam scan, was
**admitted**. Knowledge was not the binding constraint. That is a result about extraction that an
out-of-distribution corpus could not have produced, because there the two arms would differ by
knowledge as well.

**What it does NOT buy, and what is deferred.** CLAUDE.md's first claim for a corpus is
**novelty** — "findings published after the training cutoff". **This cycle does not test that
claim and may not report on it.** A pilot corpus the model knows can say nothing about what a
corpus supplies that a model cannot supply from itself. The claim is deferred to a **second
corpus of unseen, post-cutoff papers**, planned in `plans/todo/35-unseen-corpus.md` and not built.

The two corpora measure different quantities and neither number may be reported as the other:

| corpus | holds constant | measures | status |
|---|---|---|---|
| pilot, in-distribution | the model knows the paper | **extraction** (B−A) | running |
| unseen, post-cutoff | the model cannot know the paper | **novelty** | planned, not built |

**This makes the citation arm load-bearing beyond B−A.** Because A is handed the reference alone,
its transcripts record whether the model *claims* to recognise each source. "In-distribution" is
therefore an observation with a per-card answer, recorded at generation time, rather than an
assumption about the corpus — and the same instrument reads the unseen corpus when it exists. A
claimed recognition of a post-cutoff paper is itself a finding, and it is the confabulation the
provenance axis exists to detect.

**Unchanged by this amendment:** G2's decision rule, X = 0.30, the seed set, the episode count,
the sealed split, the secondary metric list and the bootstrap procedure.

### 2026-09-08 — generation becomes two calls: every arm writes its own card first

**This amendment changes where the independent variable lives.** It is an append, not an edit,
and every superseded definition above stands exactly as written, because every committed
transcript, staged candidate, library snapshot and composition was produced under them.

**What changed.** Generation was one call: a condition's prompt carried a human's extracted card
(B), a citation (A) or nothing (D), and the model wrote a leaf node straight from it. It is now
**two calls**:

1. **the card call** — `generate/prompts/card_{a,b,d}.md`, three files differing **only** inside
   a delimited `SOURCE BLOCK`. Each arm writes a principle card in the frozen six-field schema,
   parsed by `cards.store.card_from_dict`, the real loader.
2. **the node call** — `generate/prompts/node.md`, **one file for every condition**, fed that
   card and nothing that identifies which arm wrote it.

This is the pipeline `ProjectB_pipeline.drawio` has shown since 2026-09-02 — three
`LLM Writing Card` boxes feeding one `Node` step — and which the code had never implemented.

**What each arm's card call is given.** A: the `source.citation` and nothing else. B: **the
source document, in full**, extracted from the PDF. D: nothing.

**Why.** Two reasons, and the second is the stronger one.

- **B is now reading the paper rather than a human's summary of it.** Under the old structure B
  was handed an extraction and D was handed nothing, so B−D confounded "has a corpus" with "has
  somebody's derivation already done". B now does its own derivation from the document, which is
  what the corpus actually supplies.
- **The arms can no longer differ anywhere but the card.** The old guarantee was a diff test over
  three node prompts: real, but a guarantee about files. After the card call the pipeline has one
  node prompt and no condition parameter — `generate/tests/test_prompts.py` asserts
  `render_node`'s signature is `(card, task)` — so the arms are indistinguishable downstream by
  construction rather than by inspection.

**Consequences that are pre-registration material:**

1. **The ceiling is unchanged: A 20, B 20, D 4.** All three card prompts are task-conditioned —
   the arm with no source needs a problem to invent against, and a prompt carrying `{{TASK}}` for
   one arm and not another would differ outside the source block. So A and B write one card per
   (paper × task family) and the grid keeps its shape. `generate/tests/test_ceiling.py` carries
   the numbers and they did not move.

2. **⚠️ G1's NODE-LEVEL provenance fraction becomes 0.0, and this is the honest state.** A node
   is now implemented from a card the *model* wrote, so its span is a claim until somebody checks
   it. `NodeMetadata` gains `span_verified`; `is_traceable` requires it; nothing in the generation
   path sets it. Carrying the human card's verified span onto a node built from a model-written
   one would hand the arm provenance it did not earn — the failure `Origin.CITATION` was invented
   to prevent, one step further down.

   **The CORPUS-level fraction is untouched at 0.80.** `CardStore.provenance()` is a fact about
   `cards/*.yaml` and was never a fact about the nodes. **The two numbers are reported separately
   and neither may be reported as the other.** Verifying generated spans against
   `cards/VERIFICATION.md` — B has an answer key, having read the document — is a follow-up and
   is not claimed here.

3. **A's claimed spans become a measurement rather than a leak.** `NodeMetadata`'s CITATION guard
   forbade a span outright. It now permits a *claimed* one and forbids `span_verified`. A is
   pointed at a paper it cannot read, so a span in A's own card is precisely the confabulation the
   provenance axis exists to detect, and refusing to store it threw the finding away.

4. **Nothing is retired by hand, and nothing is pooled.** Candidate metadata carries
   `pipeline_version`; `generate/admit.py:pass_rate` buckets on it and **refuses to pool**,
   naming skipped records under `_superseded_pipeline`. The v1 records — 14 admitted nodes, the
   `lib-019-…` snapshot, chunk 39's A/B table, every committed composition — stay on disk,
   stay loadable, and stay readable with
   `pass_rate(pipeline_version="v1-single-call")`. They keep the provenance they actually had:
   a v1 CARD node really was built from a human-verified extraction, and rewriting it to the new
   rule would restate history.

5. **B's card call is reproducible only where the corpus is.** `out/corpus/` is gitignored and
   never committed — the PNAS copy carries the downloader's IP address — and B's card prompt is a
   whole paper, so the committed transcript stores it **redacted**, with the PDF's `sha256`, the
   extractor and the character count in place of the text. The recorded hash is of the **real**
   prompt, so a machine holding the PDF re-renders it byte-identically and replay resolves.
   Everything downstream is reproducible from committed files with no corpus at all: the
   generated cards are committed and the node call replays from them.

   This is the same status the scene assets have — a named, hashed input — except that these are
   not ours to ship. **It is stated in the write-up, not discovered by a reader.**

6. **The superseded prompts are kept and frozen.** `condition_{a,b,d}.md` are never edited and
   never rendered; `generator.render_v1()` exists solely so a committed v1 transcript can still be
   checked against the prompt it was taken against, and a test verifies it reproduces
   `d6ae93648c7b4f25` exactly.

**What this amendment does NOT change.** G2's decision rule, X = 0.30, the seed set, the episode
count, the sealed split, the secondary metric list, the bootstrap procedure, the condition letters
and what they mean, and the two-corpus split of the 2026-09-06 amendment. The card call is a
change to *how each arm's treatment is administered*, not to what the treatments are: A is still
pointed at a reference and denied its content, B is still the arm with the literature, D still has
no corpus.

### 2026-09-13 — the benchmark environment is replaced: lab42 gives way to a randomised suite

**What changed, and on whose authority.** The user decided on 2026-09-13 that a new, larger,
randomised environment suite **replaces lab42 as the benchmark**. Planned in
`plans/05-epic-environment-suite.md`; nothing is built yet, and this amendment records the
decision, not a result.

**Non-negotiable #10 is not weakened.** "One unmodified benchmark, one environment" still holds:
there is still exactly one benchmark and every condition runs on it with identical episode
counts. It is a different one.

**Why.** Two reasons, and the second is the one that matters for this document:

1. Chunk 44 moves a node's contribution test from a generated micro-world to a tree measured on
   a shared suite. Three dev scenarios, all solved by `reactive_avoid`, cannot drive that loop.
2. ⚠️ **It is the only available fix for the power problem this document already declares.**
   §3 records that `Scenario.seed` reaches only `rng()` and `drift_rng()`, so on an undrifted
   scenario every replicate seed is the *same episode*; that is why the bootstrap resamples
   scenarios, why the effective N is 12 rather than 36, and why `scripts/power_analysis.py` puts
   the rule at ~0.16 power against a true 30-point gap. Procedural randomisation makes replicate
   seeds genuine samples. No increase in episode count on lab42 achieves this.

**What this amendment DOES settle.**

- The benchmark environment is the new suite. lab42 is retired *as a scoring environment*.
- lab42 is **not deleted**. It remains the development and debugging scene, the world the
  sim-free fast loop (`core/tests/kinematic.py`, `harness/ablate.py`,
  `scripts/plot_kinematic.py`) runs in, and the only way to reproduce historical results.
- **The 20 committed `results/*.json` are historical**, produced on a superseded benchmark.
  They are kept and never deleted, per repo convention, and **must not be pooled with any
  number produced on the new suite.**
- **lab42's sealed split is retired for benchmarking and is NEVER re-sealed.**
  `core/scenarios_held_out.py`, `git tag split-v1` and the `00b3312b…` seal stand exactly as
  they are. The new suite requires a **new** held-out split, authored and sealed separately
  under chunk 11's procedural rule — in a session that has never had the held-out file in
  context and has not generated. This session has generated and therefore may not seal it.

**What this amendment does NOT settle, and what may NOT be reported until it does.**

- **G2's X = 0.30 and the entire power analysis were derived against lab42's twelve scenarios
  and do not transfer.** Both must be re-derived on the new suite, in a further dated
  amendment, **before the first evaluation run on it**. Until then there is no decision rule
  for the new benchmark and no G2 number may be reported from it.
- **The resampling unit is reopened.** `core/stats.py` resamples scenarios *because* seeds are
  degenerate. If randomisation makes them non-degenerate, that justification lapses and the
  bootstrap's unit must be re-derived rather than carried over by default.
- The scenario count, the episode count, the seed offsets, the new split's composition, and
  SPL's denominator under moving obstacles are all open. SPL is a named secondary in §5; with
  moving obstacles "the optimal path" is not well defined, and that is a declared open question
  rather than an implementation detail.
- `core/preregistration.py`'s frozen-constant registry will fail `verify()` once the scene
  constants move. That is the registry working as designed. The updated constants are part of
  the amendment that closes the items above, not a workaround applied ahead of it.

**Threat introduced, declared here in advance.** Replacing the benchmark after the pipeline has
been built on the old one creates an obvious researcher degree of freedom: the new environment
could be shaped, consciously or not, to suit the nodes that already exist. Two commitments
against it — the suite is generated **procedurally from a seed** rather than hand-authored
scenario by scenario, and the held-out split is sealed **before** any node is generated against
the new suite, under the same procedural rule that governed the first one.

---

### 2026-09-14 — the arbitration rule: how a tree combines what its nodes propose

**Why this is in this document at all.** Chunk 52 replaced exclusive control with a **layered
command pipeline**: every node in a `Layer` ticks, the shared base proposes a whole Twist, and
later nodes revise individual fields of it. That introduces something this project did not have
before — **a rule for combining proposals that came from neither arm's corpus.** Whoever writes
that rule is injecting design into every tree in every condition, and B−A would then partly
measure *our* arbiter rather than the corpus. It is declared here, in advance, for the same
reason X and the seed set are.

**The rule, in full. There is no more of it than this.**

| combinator | semantics |
|---|---|
| `ports.adjust(field=value)` | per-field **last writer wins** |
| `ports.cap(field=value)` | per-field **`min(|pending|, |value|)`, sign preserved** |

- **Priority is position in the `Layer`, later is higher.** Nothing else orders anything.
- A node declares `WRITES`, a subset of `(vx, vy, wz)`, and writing outside it **raises**.
  It is enforced by `nodes/contract.py:PortView`, not by review.
- A node whose precondition fails leaves the pending command **byte-identical** — asserted by
  `nodes/tests/test_layer.py`, by object identity rather than by float comparison.
- **No weights, no scores, no blending, no tuning, and no per-arm anything.** The rule is
  identical in A, B, C and D, and it is stated in the write-up rather than left implicit.

**Frozen as of this amendment.** Changing either combinator, or the ordering rule, is a further
dated append. It is not a tuning knob and must never be treated as one.

⚠️ **Layer ORDER is the same hazard and gets the same treatment.** With priority by position,
the composition's ordering decision is load-bearing. It is produced by the LLM composer under a
prompt **identical across arms** (`compose/prompts/compose.md`, non-negotiable #6 — port
contracts and a task spec, never the papers) and is **never hand-tuned per arm**. A tree whose
node order was adjusted by a human is not reportable.

**What this amendment does NOT settle.**

- Whether the base itself should be decomposed into smaller nodes. Considered and deferred:
  changing the execution model and splitting a 538-line node in one step leaves no way to
  attribute a result to either. Revisit as its own chunk.
- The arbitration rule for a field that two nodes cap *and* adjust in the same tick. Today the
  answer falls out of position (the later call wins, whichever it was) and no committed node
  does it. If one ever does, that is a case to declare rather than to resolve silently.

**Threat introduced, declared in advance.** The v3 library is regenerated against a **new node
prompt** (`generate/prompts/modifier.md`), so v2 and v3 nodes are not comparable and
`admit.pass_rate` refuses to pool them — the same discipline chunk 42 applied to v1. The
**card** call is unchanged and every committed card is reused byte for byte, which is
deliberate: the independent variable must not move at the same time as the execution model, or
any difference is attributable to neither. The eleven v2 candidates keep their pass-rate
denominators and are kept intact under `generate/archive/v2-exclusive-control/`.

---

### 2026-09-15 — a question raised against §5's named secondaries, and deliberately NOT settled

**This append settles nothing. It records a problem found in the secondaries before the first
evaluation run, so that whoever settles it cannot do so after seeing the numbers.**

§5 names `collisions` and `near_misses` among the secondary metrics. Both are
**lower-is-better counts**, and a count of that shape has a failure mode that showed up the
first time it was actually measured across controllers:

> **A robot that does not move scores perfectly.**

Measured 2026-09-15, growth suite, 20 `maze` worlds, kinematic. One generated node took
`collisions` from 0.40 to 0.30 and read as an improvement. It had walked **6.2 m instead of
30.0 m** and reached the goal in **0 of 20 worlds**. Per metre travelled it collides
**0.0481 against the base's 0.0133 — 3.6x worse.** The raw count and the normalised count have
**opposite signs**, and the raw count is the one §5 currently names.

`scripts/measure_growth.py --metrics` now reports both, and flags a disagreement. That is a
diagnostic tool and it binds nothing.

**What must be decided, in a further dated amendment, BEFORE the first evaluation run:**

- Whether §5's `collisions` and `near_misses` are reported raw, per metre, per episode-second,
  or conditioned on `goal_reached`.
- Whether `stuck_events`, `time_stuck` and `recovery_time` — the same shape — take the same
  treatment.
- Whether a secondary may be reported at all for a condition whose goal-reach rate differs
  materially from the comparator's, given that every one of these counts is confounded by how
  far the robot got.

⚠️ **None of this touches G2.** The primary metric is `goal_reached`, a rate over scenarios, and
it has none of this problem. X = 0.30 and `core/stats.py:decide()` are untouched by this append.

---

### 2026-09-21 — the architecture pivots: a whole-controller arm, and a `p` control

⚠️ **This is a DATED APPEND and nothing above it is edited.** `prereg-v1` remains tagged at
`a84cd46`, and §7's frozen constants and their digest are **untouched** — this append freezes no
new constant and changes no existing one.

**What changed, as a fact this document must record.** On 2026-09-21, on the user's explicit
decision, the **behaviour-tree execution model was dropped**. A generated artefact is no longer a
leaf node composed into a tree. A model is handed the frozen base navigator's source
(`generate/base/base_navigator.py`) together with an arm's cards, and returns **a whole
`core.observation.Controller` and a diff**. There is no `Layer`, no `WRITES`, no arbitration, no
layer order, no retrieval and no composition on the live path. The epic is
`plans/07-epic-whole-controller.md`.

**Why it is recorded here.** This document describes an experiment, and as of 2026-09-21 the
experiment it describes is not the one being run. Leaving that implicit would be the failure mode
the append-never-edit rule exists to prevent.

**The evidence that motivated it**, all of it collected under this document's own rules:

- **B−A came out exactly zero three times**, under exclusive control, under layered modifiers,
  and under layered modifiers **with the drift hazard switched on**. The 2026-09-14 arbitration
  amendment's design was implemented and measured; the 2026-09-16 drift defect was repaired and
  re-measured (`out/growth/maze+drift0.10.csv`). The null survived the repair.
- The library splits **bimodally** — 4 nodes never fire, 6 fire on ~100% of ticks and overwrite
  the base entirely, all 6 regress. Sixteen papers do not produce a bimodal distribution.
- **Zero of sixteen nodes improved their own claimed metric while still doing the task.**

⚠️ **THE 2026-09-14 ARBITRATION AMENDMENT IS SUPERSEDED IN EFFECT, NOT RETRACTED.** Two
combinators, no weights, nothing tunable — that rule governed how a tree combined what its nodes
proposed. There is no tree, so there is nothing for it to govern. It stays on the record because
every v3 record was produced under it.

## What this append DOES declare

**A control arm, `p`.** The base edited with **no card and no new information**. ⚠️ The
hypothesis names *"an LLM asked plainly"* and this project has never had that condition — the
naive arm was retired (2026-09-06 amendment) and D is not it, because D articulates a principle
before implementing. ⚠️ **Asking a model to improve code is itself a treatment**, so `p` is an
*edited* base and not the frozen one; **B − P is the corpus effect and B − BASE is not.**
⚠️ The letter is `p` and never `n`: `n` means sample size throughout this document.

**`generate.cardgen.CONDITIONS` stays `("a","b","d")`.** `p` lives only in epic 07's registry. It
is a control for the edit call, not a fourth card condition.

**Non-negotiable #4 is SUSPENDED for this arm only.** "The generator emits leaf nodes, never a
monolithic stack" is what this arm stops doing. Suspended, not repealed.

**The cards do not move.** Every card under `generate/cards/{a,b}/` is reused byte for byte, and
**this arm makes zero card calls** — the same discipline the 2026-09-14 amendment followed. The
independent variable and the execution model must never move in the same step.

**The unit: 3 arms × 4 task families × k = 3 = 36 controllers.** ⚠️ **k exists because one
generation is one draw from a stochastic process** — §8's temperature-0 finding, applied to a
larger artefact. Editing suppresses most of the variance that writing from scratch would carry;
it does not remove it.

**The dossier rule:** all of an arm's cards, every time, **no retrieval**. Retrieval is an
apparatus surface and this cycle is what apparatus surfaces cost.

## What this append does NOT settle, and what must land before the first scoring run

⚠️ **No number from this arm may be reported until a further dated amendment closes these.**
They are named now so the first scoring run does not discover them.

1. **The analysis unit and the bootstrap cluster.** There are two nested factors, controller
   sample and world. The proposal, to be confirmed: *an arm's score on a world is the mean over
   its k controllers; the bootstrap resamples worlds.* That is **n = 20, not n = 60**, and only
   one of those is honest — the k samples of a row are repeated measures of one generation
   process, not independent observations of a world.
2. **Best-of-k.** It answers a real question — does the corpus expand what is *reachable* — but
   it rises mechanically with k. It is a **secondary, declared in advance or not reported**.
   ⚠️ Scoring k controllers, picking the best, and reporting it as the arm's score would
   invalidate the result.
3. **X, and whether G2's decision rule applies at all.** X = 0.30 was derived on lab42's twelve
   scenarios and the 2026-09-13 amendment already records that it does not transfer to the
   randomised suite. It has not been re-derived, and nothing in this append re-derives it.
4. **The count-metric question raised on 2026-09-15** is still open and still binds here.

## Two things are given up, and the write-up states them rather than omitting them

- ⚠️ **G3 is not measurable from this arm.** Compositional generalisation needs primitives to
  recombine and a whole controller has none. G3 stands on the node library or it does not stand.
  **Generalisation across world sets may be reported, but never as G3.**
- ⚠️ **§6's G1 library framing narrows to the card level for this arm.** The coverage count
  (distinct mechanism *families*) and the provenance fraction were always properties of the
  **cards** — the 2026-08-31 amendment says so — and that is where they are reported.

## The threat this design adds, declared in advance

⚠️ **ANCHORING, and its direction is toward null.** A model handed working code makes
conservative edits. If a card implies a structurally different navigator, editing will not produce
it. **A null result from this arm means "cards did not improve this navigator", never "literature
does not help"**, and that distinction is stated from the start rather than discovered afterwards.
It joins §8's declared threats.

⚠️ **And §8's existing card-infidelity limitation is unchanged.** A controller can be handed a
card and ignore it. The diff makes that **visible for the first time**, but reading a diff is a
human judgement, not an automatic check, and the limitation is absorbed by `p` rather than solved.

⚠️ **No number from this arm pools with any node-library number**, under any pipeline version.
