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

None. Add dated entries below; never edit above this line.
