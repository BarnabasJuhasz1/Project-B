# Plans

Each file is one independently executable chunk of work. The folder a file sits in **is** its
status — move the file, don't edit a status field.

```
plans/
  00-epic-*.md   the umbrella document: context, rationale, full design
  todo/          not started
  doing/         in progress — keep this to one file at a time
  done/          finished, with an Outcome section appended
```

Workflow: `git mv plans/todo/NN-x.md plans/doing/` to start, `git mv plans/doing/NN-x.md plans/done/`
when the Done-when checks pass. Append an **Outcome** section before moving to `done/` recording
what actually happened, anything that differed from the plan, and anything discovered.

## Epic 00 — the reproducible scored episode  ✅ complete, follow-ups open

Week 1's last open item: *"one scripted episode running headless and reproducibly."*
Rationale, the controller contract, and the metric definitions live in
[`00-epic-reproducible-episode.md`](00-epic-reproducible-episode.md). Chunks reference it
rather than repeating it.

| # | Chunk | Depends on | Needs Isaac Sim | Status |
|---|---|---|---|---|
| 01 | [Extract the scene spec](done/01-scene-spec.md) | — | no | done |
| 02 | [Geometry: grid, raycast, geodesic](done/02-geometry.md) | 01 | no | done |
| 03 | [Contracts: Observation, Scenario](done/03-contracts.md) | 02 | no | done |
| 04 | [Metrics](done/04-metrics.md) | 02, 03 | no | done |
| 05 | [Reference controllers](done/05-controllers.md) | 03 | no | done |
| 06 | [Manifest](done/06-manifest.md) | 01 | no | done |
| 07 | [Episode runner](done/07-runner.md) | 01–06 | **yes** | done |
| 08 | [Determinism & end-to-end](done/08-determinism.md) | 07 | **yes** | done |
| 09 | [The SPL denominator saturates](done/09-spl-denominator.md) | 02, 04 | no | done |
| 10 | [Event counts sit on a float knife edge](done/10-event-debounce-tolerance.md) | 04 | no | done |

Chunks 09 and 10 were found after the epic closed. 09 came from reading the committed result
files — `path_length` came out *below* `optimal_distance`, and SPL saturated at 1.0 for any
controller within ~9% of optimal; it is done, and three identical 1.0 scores are now three
distinguishable ones. 10 came out of executing 09: re-scoring the same traces showed event counts
flipping on a 6.7e-12 s difference in timestamps. **Both are now done.** 10 fixed it with option
A, defined the boundary direction in writing — a clear gap of exactly `EVENT_RELEASE` closes the
span — and its audit turned up a *second* tie of the same kind in `_stuck_spans()` that the plan
had predicted would not exist: `STUCK_WINDOW` is also an exact multiple of the control step, and
the trailing window was mislocated on 26% of samples at 50 Hz. Every stuck figure in `results/`
moved as a result. Re-scoring is now idempotent.

Both are **pre-registration material**. They move numbers in committed result files, and both
decisions are collected in [`PRE-REGISTRATION.md`](../PRE-REGISTRATION.md) — chunk
[17](done/17-pre-registration.md).

Each done file carries an **Outcome** section. Read it before trusting the chunk body — several
chunks turned up things the plan had wrong (a test point inside a wall, a fall threshold inside
the normal gait envelope, a potential field with a permanent local minimum).

Chunks 01–06 are sim-free: they import in milliseconds, test under plain `pytest`, and need no
GPU. Only 07 and 08 boot the simulator. That ordering is deliberate — six of eight chunks can be
written and verified without paying a 20 s startup per iteration.

## Conventions

- One chunk = one sitting. If a chunk grows past that, split it and renumber.
- Every chunk carries a **Done when** section with checks that are actually runnable.
- New epics get the next free `NN-epic-*.md` and their own chunk range.
- A chunk that has to be **inserted before** an existing one takes a letter suffix — `11a` runs
  before `11` — rather than renumbering the board. Renumbering breaks every cross-reference in
  every `done/` Outcome, and those are the files the README tells you to trust.
- A chunk whose decisions move committed numbers or bind the evaluation carries a **Decided**
  section stating the choice *and the rejected alternatives*. Once decided it is collected in
  [`PRE-REGISTRATION.md`](../PRE-REGISTRATION.md) and not re-opened. **After `prereg-v1` is
  tagged, a change to anything that document freezes is a DATED APPEND to its Amendments
  section, never an edit** — and `core/tests/test_preregistration.py` fails until the document
  and the code agree again.


## Epic 02 — the traceable node library  ✅ complete, one tag outstanding

Weeks 2–4: the node contract, the library, the unit harness, the first hand-curated cards, the
generator and composition. Rationale and the three gaps that block whole card families live in
[`02-epic-node-library.md`](02-epic-node-library.md).

| # | Chunk | Depends on | Needs Isaac Sim | Status |
|---|---|---|---|---|
| 11a | [The task contract](done/11a-task-contract.md) | — | no | done |
| 11 | [Seal the held-out split](done/11-held-out-split.md) | 11a | no | **done — tagged `split-v1`** |
| 12 | [The node contract and the library](done/12-node-contract.md) | 11a | no | done |
| 13 | [Unit-scenario harness and promotion gate](done/13-unit-harness.md) | 11a, 12 | no | done |
| 14 | [The first hand-curated cards](done/14-first-cards.md) | 12 | no | done — spans unverified |
| 15 | [The generator](done/15-generator.md) | 12, 13, 14 | no | **done — see FINDINGS.md** |
| 16 | [Composition](done/16-composition.md) | 12, 15 | no | **done** |
| 17 | [Pre-registration](done/17-pre-registration.md) | 10, 11a, 11 | no | **done — tagged `prereg-v1`** |
| 18 | [Node tracing](done/18-node-tracing.md) | 12 | no | done |

**Chunk 11a is done**, and it was the single change to the frozen `Observation` and `Scenario`
contracts — `goal` is now Optional, `Scenario` carries `goal_known` / `detect_radius` / `drift`,
and `Scenario.disclosed_at()` is the one place the detection rule lives. Nothing after it touches
the field list. Its Outcome records one gap worth knowing: the runner's latch line has no test
until chunk 11 authors a `goal_known=False` scenario.

**Chunk 11 is done — the gate is open.** Twelve scenarios in a 2 × 2 of goal-disclosure × drift,
sealed at `00b3312b…`, hash recorded in the tests and in every manifest. Its other half is
procedural and untestable, and now binding: **15 and 16 must run in a session that has never had
`core/scenarios_held_out.py` in context.** The session that authored it is contaminated. Two
open actions in its Outcome: `git tag split-v1` once the files are tracked, and six scenarios
that refuse to load until chunk 13.

**Chunk 15 is done, and it produced four findings rather than a library.**
`Repo/generate/FINDINGS.md` is required reading before chunk 16 or 17. The two that bind:
**the admission harness cannot detect card infidelity** — the deliberately-bad card was
ADMITTED, because the harness tests whether the node works and not whether it implements the
card, and a model handed a contentless mechanism invents a real one; and **temperature-0
determinism is unavailable on any current model**, the parameter having been removed from the
Messages API, so reproducibility comes from committed transcripts and `ReplayBackend` instead.
Generation is reproducible but **not repeatable**: the same prompt produced a refusal on one call
and an admitted node on the next, and both transcripts are committed. Only one node has been
admitted so far — four calls is a pilot of the pipeline, not the library G1 needs.

**Chunk 16 is done, and `Repo/compose/FINDINGS.md` is required reading before chunk 17.** The
decision it turned on: **composition returns a JSON tree over node ids, never Python**, which
makes G3's "no new primitives" structural rather than a rule the model is asked to respect — and
makes the repair loop rewire-only for free, since a repair is handed the previous retrieval and
has no other vocabulary. Two findings bind: **a composition can succeed and produce a tree that
does not work** (a committed pair, same library and same retrieval, one reaching the goal and one
starving its driving action inside a Sequence — nothing in a port contract says how often a
precondition fires), and **the admitted condition-B node has two effects**, which
`nodes/contract.py` forbids and neither the harness nor composition can detect. Two proposed
TaskSpec fields were dropped after measurement: `time_limit` discriminates within three of the
four held-out families, and the start region cannot be blurred without becoming useless.

**Chunk 17 is done, and it found that the plan's own arithmetic was wrong in the unsafe
direction.** `PRE-REGISTRATION.md` fixes X = 0.30, 3 seeds per scenario at offsets
`(0, 10000, 30000)`, 36 held-out + 9 dev episodes per condition, and four named secondaries. The
finding: **the episode count is nearly irrelevant to the power and the scenario count is
everything.** `Scenario.seed` reaches only `rng()` and `drift_rng()`, so on the six undrifted
held-out scenarios a controller that ignores its rng produces a *bit-identical episode at every
seed* — measured on three controllers. The bootstrap therefore resamples **scenarios**, the
effective N is 12, and `scripts/power_analysis.py` puts the rule at ~0.16 power against a true
30-point gap. That consequence is **pre-registered**: a rule that does not fire may not be
reported as "the corpus does not help". One Done-when clause is still open and it is the one that
blocks Week 5 — **`git tag prereg-v1`**, which needs the files tracked first. **Both are now
done**: the tree was committed at `a84cd46` and `split-v1` and `prereg-v1` both point at it.

The three findings recorded in the epic — **localisation is not simulated**, **there is no search
task**, **legibility has no metric** — are now all decided: a seeded random-walk drift model
(13); `Observation.goal` becomes `Optional` with latched line-of-sight detection (11a); and
legibility is **out of scope this cycle**, marked for revisit. The price of the second, recorded
in the epic, is that area-restricted search — the description's own running example — is not
testable without a cue field.

`01-reproducible-episode.md` at the top level is epic 00's superseded predecessor, kept for its
context section. Read the epic instead.

## Epic 04 — the evaluation sweep  🟨 in progress

Weeks 5–6: the batch runner, the missing fourth condition, the container run, and G3 over the
held-out split. Rationale, the honest dependency picture and the design decisions that belong to
the epic rather than to one chunk live in [`04-epic-evaluation.md`](04-epic-evaluation.md).

| # | Chunk | Depends on | Needs Isaac Sim | Status |
|---|---|---|---|---|
| 28 | [The batch runner and aggregation](done/28-batch-runner.md) | 17 | no | **done** |
| 29 | [Grow the libraries to a reportable size](doing/29-grow-libraries.md) | 15, 28 | no | **doing — machinery built; blocked on the scale decision and the corpus run** |
| 30 | Condition C — the Nav2 baseline | 28 | **yes** | not written |
| 31 | The container run | 28, 29 | **yes** | not written |
| 32 | G3 — composition over the held-out split | 16, 29, 31 | **yes** | not written |
| 33 | [The citations probe](doing/33-citation-probe.md) | 14, 15 | no | **doing — machinery built; five model calls outstanding** |
| 34 | [Condition C — failure context](todo/34-condition-c-failure-context.md) | 28 | no | specified, not built |
| 35 | [The unseen post-cutoff corpus](todo/35-unseen-corpus.md) | 14 | no | specified, not built |
| 36 | [A node under physics + the A/B video pair](done/36-node-physics-evidence.md) | 37 | **yes** | **done — every kinematic verdict survived; one failure MECHANISM did not** |
| 37 | [Admission becomes a validity gate](doing/37-admission-becomes-validity.md) | 29 | no | **doing — code done; CLAUDE.md #5 and a PRE-REGISTRATION append AWAIT SIGN-OFF** |
| 38 | [Node contribution: leave-one-out ablation](doing/38-node-contribution-ablation.md) | 37 | no | **doing — built and working; not yet useful on the real library, see 40** |
| 39 | [The first comparable A vs B numbers](done/39-ab-node-comparison.md) | 37, 38 | no | **done — A > B on 5/5 cards, and NEITHER arm beats a handwritten toy** |
| 40 | [The contract cannot express a modifier](todo/40-modifier-kind.md) | 38, 39 | no | **NEXT — 39 measured the blocker: 14/14 nodes drive AND publish, 0 read a port** |
| 41 | [The harness and the runner do not share a timebase](todo/41-timebase-mismatch.md) | — | one run | **written — a 3.78e-12 s difference silently disables a node's mechanism** |
| 42 | [Card-first generation: every arm writes its own card](doing/42-card-first-generation.md) | — | none yet | **BUILT, unrun — two calls per row; B now reads the PAPER; one shared node prompt; v1 records no longer pooled** |

**Where this is going, 2026-09-07.** Chunk 37 redefined admission from "passes its unit scenario
in isolation" to "runs", after the isolated test was measured to be a weak instrument: an A node
still solves both dev scenarios with its paper's mechanism ablated, while *failing its own unit
scenario* under the same ablation. One consequence is that **all five pilot cards now have both an
A node and a B node** — ten nodes, one task family, five shared papers, nothing left to generate.
**39 turned that into the project's first A-vs-B number** and is done.

⚠️ **39's answer was not the one it went looking for.** A beats B on 5/5 cards, but the
carrier-only reference row — added after the first run, because without it the ablation deltas
were unreadable — shows a **handwritten fixture solving all three dev scenarios at SPL 0.982**.
Against that yardstick every A node is inert (ΔSPL ≤ 0 in four of five) and four of five B nodes
*destroy* the carrier by starving it. **Nine of ten generated nodes fail to beat scaffolding.**
The one-line reading is "A beats B, and neither arm beats a toy" — not "the corpus does not
help", which the evidence does not support.

That makes **40 the next chunk, not an optional one**: 39 measured the blocker rather than
arguing it. No generated node reads a port and 14 of 14 both drive and publish one, so a B
mechanism can only drive (starving whatever else would have) or publish into the void. Both are
in the table. `8 of 10 carrier_drove = 0.00` is 40's acceptance target.

**36 is now done too**, and it is the project's first physics. `node:<node_id>` runs one admitted
leaf in the admission tree under `run_episode.py`; seven episodes and three videos are in
`out/evidence/nodes/reorient-geometry-a-vs-b/`. **All four kinematic verdicts survived the
locomotion policy** — A reaches the goal at SPL 0.975 / 0.931, both B nodes fail — but one
failure *mechanism* did not: `ReorientToGeometry` slides 36 m along walls on the unicycle and
**pins itself against the divider for 49.6 of 60 s** under physics, so `stuck_events` and
`recovery_time` are zero in every kinematic row and non-zero here. A failure characterised on the
unicycle is not the failure the robot has, which bears on `harness/ablate.py` and every admission
decision. It does not change 40's priority — it films the two failure shapes 40 exists to fix.

⚠️ **36's diagnosis opened 41, and 41 is not cosmetic.** Asked *why* B fails, the answer is three
causes and none is "the mechanism is wrong". `ReorientToGeometry`'s recovery **fired 0 times in
3000 physics steps** while firing 8 times on the same scenario kinematically, because
`core/tests/kinematic.py` computes `t = step * dt` and `run_episode.py` accumulates `t += dt`.
The difference is **3.78e-12 s** and the node's 1.2 s window sits exactly on that tie — chunk 10's
float knife edge again, this time in the contract between the harness and the runner rather than
in the scorer. **Every admission decision, every `harness/ablate.py` contribution and every
`ab_compare.py` row is computed on one timebase; the G2 sweep runs on the other.** 41 also touches
three pre-registered metrics, so its fix carries a dated append that is the user's call.

**28 is done; 29, 33, 37 and 38 are written; 30–32 are still blocked.** They are named so the shape is
visible, and each gets a plan file when its blockers clear, because a plan written against a
blocked dependency is a guess.

**Chunk 29 turned up a ceiling nobody had costed.** `generate/generator.py:render()` is a pure
function of `(condition, task, card)` with no nonce, and the node id *is* the prompt hash — so
the reachable library is 3 nodes for A, 3 for D and 15 for B, and **A and D cannot reach G1's 20
by spending more calls**. The binding constraint is the prompt-identity design, not the corpus.
A second, softer finding: B's 15 is a count of *prompts*, and the five cards name only three
distinct mechanisms — **deliberately**, since chunk 14 spent two cards pairing derivations to
test the schema. Unlike A's and D's, B's 3 is not structural. But `node_name` is a hand-typed
field nothing validates, so G1's "distinct admissible primitives" headline is set by a naming
convention rather than measured; the node count and the mechanism count are different numbers
and the plan now requires both. The plan
states the levers that lift it, the **condition-neutrality rule** any of them has to obey (an
enumeration axis available to A, B and D alike — which is exactly why cards cannot be one), and
the decision that has to be taken before any generation runs.

**Chunk 28's rehearsal found a defect in the sweep itself, which is what a rehearsal is for.**
Five of eighteen episodes exited **0**, wrote no result, and were recorded as `ok` — the driver
believed the exit code and kept the stderr only on a non-zero exit. Exit 0 is now a necessary
condition and the result file the sufficient one. It also confirmed chunk 17's central claim
under physics rather than kinematics: **all six (scenario × controller) pairs were identical
across all three replicate seeds**, on every scored key.

⚠️ **Chunk 29 is the one that decides whether this cycle has a result at all.** Four generation
calls produced one admitted node; G1 asks for ≥20 *per condition*, and conditions A and D have
**zero** each. A condition with an empty library composes nothing. Either generation runs at a
scale nobody has attempted yet, or the write-up reports a coverage count of one against a target
of twenty and says why. That is a project-level decision, and it is flagged in the epic rather
than discovered in Week 5.

## Epic 03 — localisation as a real estimate  🟨 chunk 19 only; 20–27 deliberately not next

A second cycle: replace `core/drift.py`'s open-loop random walk with a real 2D lidar SLAM
pipeline, so that **the pose estimate becomes a consequence of what the robot did**. Rationale,
the four decisions and the three named risks live in [`03-epic-slam.md`](03-epic-slam.md).

| # | Chunk | Depends on | Needs Isaac Sim | Status |
|---|---|---|---|---|
| 19 | [The localisation contract](doing/19-localisation-contract.md) | — | no | **complete — byte-identical under physics; held in `doing/` until the epic opens** |
| 20 | [The sensor noise model](doing/20-sensor-noise.md) | 19 | no | **complete — byte-identical off, deterministic on; held in `doing/`** |
| 21 | [Scan matching: the front end](doing/21-scan-matching.md) | 19, 20 | no | **complete — 25 tests; ambiguity added, see Outcome** |
| 22 | [Submaps and loop closure](doing/22-loop-closure.md) | 21 | no | **complete — 24 tests; three findings, see Outcome** |
| 23 | [Pose graph: the back end](doing/23-pose-graph.md) | 22 | no | **complete — 40 tests; SLAM beats the walk, see Outcome** |
| 22a | [The closure travel gate](doing/22a-closure-travel-gate.md) | 22 | no | **complete — a stationary robot no longer manufactures closures; held in `doing/`** |
| 24a | [The cost model and the scan rate](doing/24a-slam-cost-model.md) | 23 | no | **complete — rate moved to 5 Hz without a GPU; held in `doing/`** |
| 24b | [Seeing the SLAM run](doing/24b-slam-visualisation.md) | 23, 24a | no | **complete — the map, the graph and the error, drawn; held in `doing/`** |
| 24 | [Determinism and cost gate](todo/24-slam-determinism.md) | 23, 24a | **yes** | todo — **now the physics half only** |
| 25 | [Recalibration: what moves](todo/25-slam-recalibration.md) | 24 | no | todo |
| 26 | [The cards SLAM unlocks](todo/26-slam-cards.md) | 25 | no | todo |
| 27 | [Camera VO](todo/27-camera-vo.md) | 24 | **yes** | **deferred — a recorded decision, not scheduled work** |

**Why it earns its place.** `localisation_failures` is currently controller-INDEPENDENT — chunk
14 measured it — so it is a manipulation check rather than a metric, and three of the
description's own example leaves are untestable or testable only degraded. SLAM makes the metric
a real dependent variable, and it is the only way `path_integration_mammals`' claim (path
integration is *arbitrated* against other cues, not used alone) becomes falsifiable as written.

**The contract does not change.** SLAM writes into `pose_estimate` and `pose_error_estimate`,
the two fields that already exist, and no node, controller, condition or metric is rewritten.
That is chunk [11a](done/11a-task-contract.md)'s decision paying off: `Observation` carries a
belief and never ground truth, precisely so the thing producing that belief could be swapped
later.

**19 is a gate** and plays 11a's role — the one change to the `Scenario` contract, declaring
fields that 20 and 23 implement without touching the field list. **24 is a gate**: a SLAM
pipeline that is not bitwise reproducible cannot produce a reported number in this project.

⚠️ **Scheduling: this epic must NOT land between chunk [17](done/17-pre-registration.md) and the
first evaluation run** — which is *now*. A pre-registration is written against a measurement
apparatus; introducing SLAM changes what `localisation_failures` measures and what goal-reach
rate is achievable on every drifted scenario, which would make the pre-registered rule a rule
about a different experiment. Land 20–27 after cycle 1 is run and written up. Chunk 25 collects
cycle 2's pre-registration material.

**Chunk 19 is the exception, and it was taken on 2026-08-31.** It is the socket, not the plug:
`drift` is untouched, `TRACE_COLUMNS` is unchanged, `localisation` and `sensor_noise` fail loudly
until 23 and 20 exist, and nothing that exists sets either field — so the default path is the
drift model it always was, wrapped. Byte-identity of the existing path is the chunk's own
acceptance criterion and is asserted against a **copy of the pre-chunk-19 rollout**, with drift
and without. Nothing moved underneath `PRE-REGISTRATION.md`: `FROZEN` holds module-level
constants and `FROZEN_FILES` hashes the sealed split and the USD, and neither is a field list —
`core.preregistration.verify()` is now called by a test that says so.

**Chunk 20 was taken on 2026-08-31 too, and the warning above it was overstated.** Sensor noise
changes `Observation.ranges` only *for a scenario that carries a `SensorNoiseSpec`*, and chunk 20's
own Decided section makes noise **opt-in per scenario and off by default** with byte-identity as a
Done-when — the same property that made 19 safe. `NoNoise.scan()` calls `raycast` and returns it,
so the noiseless path is not a branch that looks equivalent to the old one, it **is** the old one.
No scenario in the repo carries the field, and none may until epic 03 has its own
pre-registration.

**Chunk 24a was taken on 2026-09-03/04, and it is the half of 24 that never needed the GPU.**
`SlamLocaliser.update()` takes a dt, a pose and an array of ranges and cannot tell a physics step
from a kinematic one, so **its cost is a pure function of what it is handed** and the scan rate can
be decided on a laptop. It was: **5 Hz**, moving `DEFAULT_SCAN_RATE` from 10. Chunk 24 keeps
determinism-under-physics and the containerised table, and nothing else.

⚠️ **The rate answer was 10 Hz first, and getting it wrong is the more useful half.** One 14.5 s
traverse says 10 Hz: at 5 Hz that trajectory proposes *zero* closures and looks like a false
economy. The same trajectory family at three lengths says the opposite — **5 Hz is 3.3-4.1x cheaper
at indistinguishable accuracy**, and better at the shortest length measured (0.030 m vs 0.068).
The zero was about 14.5 seconds, not about 5 Hz, and every held-out scenario is 120-240 s. The
lesson generalises past the number: **a scan rate cannot be chosen from one trajectory**, because
the rate interacts with episode length through the submap count and the submap count is what the
closure machinery needs to start. ⚠️ 5 Hz is also the rate the delivered-rate defect below bites
hardest on — it delivers **4.53 Hz**, so cycle 2's pre-registration must say 4.53 or fix the
accumulator first.

⚠️ **24a's third finding is the one that matters most, and it was not in anyone's plan: SLAM's
cost is SUPERLINEAR in episode length.** A 4.2x longer episode costs 3.31x more per simulated
second — matching is linear, **closure search is quadratic**, because a proposal round matches
against every finished submap within `SEARCH_RADIUS` and lab42 is smaller than the sensor range.
The held-out scenarios run 120–240 s and both measured trajectories are shorter, so the budget
extrapolated from them under-reads in the unsafe direction; and the same rate costs 0.35x real time
one-way against 3.12x on a trajectory that revisits, so **the sweep's wall clock now depends on
what the controller does.** **The rate absorbed it** — the rate is a linear lever on matching but a *quadratic* one on closure
search, so 10 -> 5 Hz saved 3.3-4.1x and made the sweep affordable (~4.3 SLAM hours against ~17.8,
before chunk 25's mandatory control doubles both). ⚠️ **It will not absorb it twice**: 2 Hz is
already below any real 2D lidar. The real fix — a bound on the candidate set per proposal round —
is a design change needing its own chunk, and is the largest open design item in epic 03.

Two further findings depart from chunk 24's plan. **The proposed `OMP_NUM_THREADS == 1` runtime assertion
is replaced by a measurement**: 1 thread and 22 are byte-identical, because nothing in `core/slam`
reaches a BLAS threading threshold — `spsolve` is SuperLU and the only dense products are 3x3
blocks. An assertion whose premise is measured false is not cheap insurance, so what enforces it now
is a test contrasting the two thread counts, and `check_determinism.sh` deliberately does *not* pin
threads. And **the requested scan rate is not the delivered one** — 20 Hz delivers 16.7 by
quantisation, 5 Hz delivers 4.53 by float accumulation. Deterministic, so reproducibility is fine;
but the rate is pre-registration material, so a document saying "5 Hz" would name an apparatus that
never ran. Reported, not fixed: the fix moves every committed SLAM number and belongs to 25. The
chosen 5 Hz is the rate it bites HARDEST on, delivering 4.53 — so cycle 2's pre-registration must
say 4.53 or fix the accumulator first.

**Chunk 24b was taken on 2026-09-04 and it is a looking chunk, not a building one.** Chunks 21–23
were each verified by a number in a test or a number in `slam.csv`, which answers every question
that is a scalar and none of the questions that are a shape. `core/slam/record.py` keeps the map
and the pose-graph history — **opt-in and inert, on `nodes/trace.py`'s precedent**, pinned by a
test that runs one episode twice and diffs — and `scripts/visualise_slam.py` draws four artefacts
from a run directory with no simulator and no GPU. `scripts/compare_localisers.py --save-run`
writes one, so the pictures are reproducible on a laptop; `run_episode.py --slam-map` writes one
from a physics episode.

⚠️ **Five findings, and the first two bind chunk 26 directly.**

**1. `error_estimate()` reports LOW uncertainty during the episode where localisation fails.** Not
a scale error — it disagrees in *opposite directions* on the two runs: 3.6x too high on a clean
replay, **0.4x too low** on the physics episode that failed, where the realised error was over
`LOC_FAIL_THRESHOLD` on 24.5% of steps and the proxy reached it on **zero**.
`IsLocalisationDegraded` reads that field, and on the one episode in this repo where localisation
actually degraded it would have said it had not.

**2. A stationary robot manufactures loop closures and nothing stops it.** Spinning in place for
40 s, the robot finished **nine submaps inside a 0.13 x 0.19 m box** and closed them against each
other: 37 proposals, 37 accepted, 0 rejected, **median separation 0.105 m**, exactly one edge in
the episode spanning over a metre. `loop.py`'s four gates all judge a match's QUALITY and all four
are satisfied; nothing gates on the submaps being anywhere apart. This is also why finding 1
happens — the proxy resets on every accepted closure. ⚠️ It compounds 24a's superlinear cost in
the worst way: the cheapest possible behaviour produces the most expensive possible search.

**3. `slam.csv` could not explain its own verdict.** 80% of the episode read "not converged" beside
a healthy score and a small translational ambiguity, because `converged` is an AND of three
conditions and the file carried two. `match_ambiguity_theta` is now a column, with a regression
test; `MatchResult`'s docstring, which claimed `converged` meant `score >= MIN_SCORE`, is fixed.

**4. The heading error during a turn is scan staleness, not a bad match** — the match is *better*
while turning. It is the held estimate going stale at the delivered 4.55 Hz, which independently
confirms 24a's delivered-rate finding from the trace rather than from the code.

**5. The first episode in this repo where a localiser changed a primary metric.** A
`PerfectLocaliser` control with the sensor noise held on reaches the goal in 15.36 s at SPL 0.950;
the SLAM run times out at SPL 0. The robot believed itself **0.017 m** from the goal while
**1.015 m** away — it stopped because, by its own reckoning, it had arrived. (The runner scores
the TRUE distance, so it was not recorded as a success; that separation is what it is for.)

**Finding 2 was then fixed as chunk [22a](doing/22a-closure-travel-gate.md)**, on request.
`loop.py` now gates on **metres travelled** as well as submap index — `MIN_SUBMAP_TRAVEL = 2.0`,
which is `MIN_SUBMAP_GAP = 2` restated in a unit that does not lie when the robot stands still.
The episode drops from 37 accepted closures to 1 with 36 suppressed before matching; the two-lap
loop is **byte-identical and suppresses nothing**. ⚠️ The obvious fix — gating Euclidean separation
— would have broken loop closure outright, since genuine closures have a **minimum** separation of
0.02 m (a revisit *is* being in the same place); that alternative was measured and rejected before
implementing.

⚠️ **Fixing 2 did not fix 1, and finding out why was worth more than being right.** The proxy
nearly doubled (peak 0.557 → 0.740 m) and still never reaches 1.0 m. It is
`σ_odom · √(time since the last closure)`, so at σ = 0.12 it **cannot** reach 1.0 m until 69 s have
passed without a closure. The defect is a ceiling set by σ_odom, not only the resets — a
calibration question for 25, now with a formula.

A separation alarm was raised and **retracted**: a single seed showed odometry beating SLAM; over
five seeds SLAM wins on the mean (0.876 vs 1.302) with half the variance. ⚠️ Not retracted: the
**drift model** still beats SLAM on a one-way traverse (0.468 vs 0.876) while losing 11x on the
loop, so **SLAM's advantage over the model it replaces is contingent on revisiting** — which makes
"do the held-out scenarios induce revisiting?" a chunk 25 question that decides whether epic 03
delivers.

One correction also landed: **24a's `+control` column doubles the SLAM hours and should not** — a
`PerfectLocaliser` control adds ~3% of the SLAM compute, not 100%. Conservative, so no verdict
flips, but it would have over-provisioned a GPU rental 2x.

⚠️ **Chunk 21 is the real boundary.** Scan matching is the first chunk that produces a *number*
rather than a capability — a pose estimate that competes with `core/drift.py`'s — and 23 is where
`localisation_failures` stops meaning what `PRE-REGISTRATION.md` was written against. Do not start
21 before the evaluation run.

⚠️ **The sealed split at `00b3312b…` is not re-sealed.** G3 measures a frozen library against a
frozen split; retrofitting SLAM onto those twelve scenarios would void it retroactively. Chunk 26
authors a **new** split, sealed separately, under chunk 11's procedural rule.

