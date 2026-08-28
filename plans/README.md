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
| 29 | [Grow the libraries to a reportable size](todo/29-grow-libraries.md) | 15, 28 | no | **todo — carries an undecided project-level question** |
| 30 | Condition C — the Nav2 baseline | 28 | **yes** | not written |
| 31 | The container run | 28, 29 | **yes** | not written |
| 32 | G3 — composition over the held-out split | 16, 29, 31 | **yes** | not written |

**28 is done and 29 is now written; 30–32 are still blocked.** They are named so the shape is
visible, and each gets a plan file when its blockers clear, because a plan written against a
blocked dependency is a guess.

**Chunk 29 turned up a ceiling nobody had costed.** `generate/generator.py:render()` is a pure
function of `(condition, task, card)` with no nonce, and the node id *is* the prompt hash — so
the reachable library is 3 nodes for A, 3 for D and 15 for B, and **A and D cannot reach G1's 20
by spending more calls**. The binding constraint is the prompt-identity design, not the corpus.
The plan states the three levers that lift it and the decision that has to be taken before any
generation runs.

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

## Epic 03 — localisation as a real estimate  ⬜ not started, and deliberately not next

A second cycle: replace `core/drift.py`'s open-loop random walk with a real 2D lidar SLAM
pipeline, so that **the pose estimate becomes a consequence of what the robot did**. Rationale,
the four decisions and the three named risks live in [`03-epic-slam.md`](03-epic-slam.md).

| # | Chunk | Depends on | Needs Isaac Sim | Status |
|---|---|---|---|---|
| 19 | [The localisation contract](todo/19-localisation-contract.md) | — | no | todo |
| 20 | [The sensor noise model](todo/20-sensor-noise.md) | 19 | no | todo |
| 21 | [Scan matching: the front end](todo/21-scan-matching.md) | 19, 20 | no | todo |
| 22 | [Submaps and loop closure](todo/22-loop-closure.md) | 21 | no | todo |
| 23 | [Pose graph: the back end](todo/23-pose-graph.md) | 22 | no | todo |
| 24 | [Determinism and cost gate](todo/24-slam-determinism.md) | 23 | **yes** | todo |
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
first evaluation run** — which is *now*, since 17 is done and untagged. A pre-registration is written against a measurement apparatus; introducing
SLAM changes what `localisation_failures` measures and what goal-reach rate is achievable on every
drifted scenario, which would make the pre-registered rule a rule about a different experiment.
Land it after cycle 1 is run and written up. Chunk 25 collects cycle 2's pre-registration material.

⚠️ **The sealed split at `00b3312b…` is not re-sealed.** G3 measures a frozen library against a
frozen split; retrofitting SLAM onto those twelve scenarios would void it retroactively. Chunk 26
authors a **new** split, sealed separately, under chunk 11's procedural rule.

