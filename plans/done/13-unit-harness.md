# 13 — Unit-scenario harness and promotion gate

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** [11a](../done/11a-task-contract.md), 12 · **Needs Isaac Sim:** no
**Blocks:** running `return_degraded` and `search_degraded`, half the sealed split.

## Why

Non-negotiable #5: **a node is admissible only if it passes its unit scenario in isolation.** The
unit scenario is a purpose-built micro-world that isolates one node's claimed effect and tests it
against a fixed baseline, pass or fail, decided automatically. G1 reports the pass rate, so this
harness *is* the measurement, not a convenience.

The card schema makes the test almost mechanical: **Prediction becomes the unit test.** A card
whose Prediction is not falsifiable produces a node that cannot be admitted, which is the
filter working rather than failing.

## The gap that gates a third of the cards

`run_episode.observe()` sets `pose_estimate` to the true pose. So `localisation_failures` is
structurally 0 in every result so far, and `IsLocalisationDegraded`, `MaintainHomingVector` and
`RequireSensorAgreement` **cannot be tested at all**. A drift model is not an optional extra
here; it is what makes a whole family of cards admissible or not. Build it in this chunk.

## Decided

**Where the micro-world comes from.** `core/geometry.py` builds occupancy, raycast and clearance
from a list of boxes, and `core/tests/kinematic.py` rolls out a unicycle — so a micro-world can
be a handful of boxes plus a start and a goal, scored by `core/metrics.py`, with **no simulator**.
Recommend that: unit scenarios that need a GPU cannot be run per-candidate at generation time.
State the limitation plainly — a node that passes kinematically may still fail under physics, so
the unit scenario is a *necessary* not sufficient condition for admission.

**Who writes the unit scenario.** §4 says Prediction becomes the unit test, which implies the
generator emits it alongside the node. A generator that writes both the node and its test can
write a test the node trivially passes. Recommend: the generator proposes, and the harness
enforces structural requirements it cannot dodge — there must be a fixed baseline, the baseline
must actually fail, and the effect size must exceed a threshold fixed in advance.

**The drift model. Decided: a seeded random walk with a magnitude parameter.** Rejected
alternatives: additive Gaussian on pose (zero-mean, so it never accumulates past
`LOC_FAIL_THRESHOLD` and the family would be unfailable); a discrete kidnapping event (a step
change is a different phenomenon and it makes `pose_error_estimate` uninformative between jumps);
degradation tied to feature-poor regions (needs a feature model this scene does not have). The
random walk is reproducible from the scenario seed, it makes `LOC_FAIL_THRESHOLD = 1.0` reachable
within a 120 s episode, and it turns `pose_error_estimate` — already in `Observation`, always 0.0
today — into a real signal a node can condition on.

`DriftSpec` is **declared** by [11a](../done/11a-task-contract.md), so this chunk implements a type that
already exists and is already referenced by sealed scenarios. Do not change its field list; if a
field is missing that is a finding for the Outcome. Until this chunk lands, a scenario carrying
`drift` fails loudly on load, per 11a's decision 4 — check that it does before implementing, or
the guard is not doing anything.

**The effect-size threshold for admission is pre-registration material.** It has to be fixed
before any candidate is judged, or the first borderline node sets it retroactively. Fix it in this
chunk's Outcome and hand it to chunk [17](../done/17-pre-registration.md). Fixing it blind is genuinely
hard; the honest procedure is to calibrate it on the two *reference* controllers, whose
capabilities are known and neither of which is a generated node, and then never touch it again.

## Build

- `harness/micro.py` — `MicroScenario`: boxes, start, goal, time limit, seed, and the ports the
  node under test expects. Built on `core.geometry`, scored by `core.metrics`.
- `harness/unit.py` — `run_unit(node, micro, baseline) -> UnitResult`. Runs the node and the
  fixed baseline over the same micro-world and seed, returns pass/fail plus the effect size.
- `harness/gate.py` — promotion. Non-negotiable #9: an addition is promoted only if it **beats
  the incumbent on a frozen regression suite**, for both capability- and failure-driven additions.
  The regression suite is every previously admitted node's unit scenario; adding a node must not
  regress any of them.
- `core/observation.py` / `run_episode.py` — the drift model behind `--drift`, feeding
  `pose_estimate` and `pose_error_estimate`. Off by default so every result committed so far
  stays reproducible; a scenario field, not a flag, once it is used in earnest.

## Done when

- A toy node with a real effect passes; the same node with its effect disabled fails. Both
  automatic, no human reading the output.
- A unit scenario whose baseline already succeeds is **rejected as malformed** — that is the
  generator-marks-its-own-homework failure, and it needs a test.
- The regression suite runs every admitted node and fails on any regression.
- With `--drift`, `localisation_failures` on a dev scenario becomes non-zero and
  `pose_error_estimate` tracks the true error well enough to condition on. Without it, every
  number in `results/` is unchanged — assert that with `scripts/rescore.py --dry-run`, do not
  assume it.
- The six `return_degraded` and `search_degraded` scenarios sealed by chunk
  [11](../done/11-held-out-split.md) now load instead of raising, and each accumulates enough drift within
  its time limit to cross `LOC_FAIL_THRESHOLD` at least once under a baseline that ignores it.
  A degraded family the baseline passes anyway tests nothing.
- `harness/` is sim-free, enforced by the import guard.

## Not in scope

Generating any real node, and re-running the committed results under drift.

---

## Outcome

**Done 2026-08-25.** `core/tests/` + `nodes/tests/` + `harness/tests/` = **263 passed**, no
simulator, ~50 s. `results/` untouched, verified after every stage.

Built: `core/drift.py`, `harness/micro.py`, `harness/unit.py`, `harness/gate.py`, plus a
parameterisation of `core/geometry.py` the plan did not anticipate needing.

### The plan missed a prerequisite: geometry was hardwired to lab42

`harness/micro.py` was specified as "a handful of boxes plus a start and a goal, built on
`core.geometry`". But every function in `core/geometry.py` read `core.scene_spec.wall_rects()`
directly, in five places, so the module could only ever answer questions about lab42.

Reimplementing the primitives inside `harness/` was the tempting shortcut and would have been
fatal: a node that passes its unit scenario would then have been tested against *different
geometry code* from the benchmark, and a passing unit scenario would say nothing about a run. So
`core/geometry.py` grew a `World` value object and a `world=None` parameter meaning LAB42 —
purely additive, every existing call site unchanged in meaning.

**The caches were the real hazard.** `_GRID_CACHE` keyed on `(resolution, inflate)`, `_VISGRAPH_CACHE`
on `(radius, step_deg)`. Two different worlds at the same resolution would have **silently shared a
grid**, so building one micro-world would have changed what `core.scenario.validate()` considers
solvable and what the SPL denominator is. Every cache is now keyed on the world, and
`test_a_micro_world_does_not_disturb_lab42` asserts lab42's grid is the same object, cell for cell,
after a micro-world has been built.

Acceptance: 263 tests green and `rescore --dry-run` reporting 0 of 6 changed — the refactor is
behaviour-preserving on lab42 by construction and by measurement.

### Drift: two things that were easy to get wrong, and one that was not in the plan at all

**sigma is radial.** The magnitudes were sealed with the split *before* this model existed, with
the arithmetic "crosses LOC_FAIL_THRESHOLD at (threshold/sigma)^2". Drawing each axis with std
`sigma*sqrt(dt)` gives radial RMS `sigma*sqrt(2t)` and crosses ~41% early, which would have
invalidated all six degraded scenarios. Each axis is drawn with `sigma*sqrt(dt/2)` instead.
Measured over 400 seeds: RMS radial error at the predicted crossing time is 0.996 m for
DRIFT_MODERATE and 1.002 m for DRIFT_SLOW, against a target of 1.000. **The sealed constants and
the implementation agree**, which is the reconciliation chunk 11 promised would happen here.

All six sealed degraded scenarios cross the threshold inside their time limits at their own seeds —
first crossing between 13.6 s and 142.4 s, 14% to 81% of the episode degraded. Now asserted
permanently in `core/tests/test_drift.py` rather than checked once.

**`pose_error_estimate` is predicted, not realised.** A robot knows its noise model, not its own
realisation. It is `sigma*sqrt(t)` — deterministic, identical across seeds — and the realised error
wanders around it. Had it carried the true error, `IsLocalisationDegraded` would be a node that
reads the answer and ground truth would have re-entered `Observation` by the back door.

**Not in the plan: drift needs its own random stream.** `Scenario.rng()` is handed to
`controller.reset()`, and the four conditions draw different amounts of randomness from it. If
drift shared that stream, condition A and condition B would face **different drift on the same
episode** and the ablation would be comparing two different worlds — a direct violation of #10 that
nothing would have surfaced. `Scenario.drift_rng()` is a separate stream, with a test that drawing
1000 samples from the controller stream cannot move it.

`NoDrift` is a null object rather than an `if drift is not None` branch, so the no-drift path is
literally the path it always was. That is why every committed number is unchanged by construction
and not merely by inspection.

### The homework-marking guard is the most valuable thing here

§4 makes the generator emit the node *and* its test, so it can write a test its node trivially
passes. `run_unit` enforces three things it cannot dodge: a named fixed baseline; **the baseline
must actually fail**; and an effect floor fixed in advance that a scenario may raise but never
lower. A scenario whose baseline already succeeds is rejected as MALFORMED even when the candidate
is genuinely good — demonstrated in `test_a_scenario_whose_baseline_already_succeeds_is_malformed`.

`MIN_EFFECT_RATE = 0.5` / `MIN_EFFECT_REL = 0.10` are **pre-registration material** and were
calibrated on the two reference controllers, deliberately: their capabilities are known and neither
is a generated node with an interest in the answer. On a corridor micro-world with one box,
`reactive_avoid` reaches the goal and `goal_seeker` does not — a rate effect of 1.0, of which the
floor is half. For chunk 17.

### Two things measured that changed the design

**Five seeds were five identical episodes.** The default `seeds=(0,1,2,3,4)` made the suite take
98 s and implied an average that did not exist: a deterministic micro-world driven by a
deterministic node gives the same rollout for every seed. Default is now one seed, with a note that
more only buys something under drift or a stochastic node. Suite time halved to 49 s.

**The first fixture was a bad gap follower.** `SteerPastObstacle` was hand-rolled, oscillated
against the box and scored 0, which looked like a harness bug. It now delegates to
`reactive_avoid` when its mechanism is on and `goal_seeker` when off — so "the same node with its
effect disabled" is literally a swap of the mechanism with ports, precondition, metadata and tree
all held constant, and the test is about the harness rather than about my ability to write a
controller.

### Verified under physics

`--drift 1.0` on `blind_corner`: **`localisation_failures: 3`** — non-zero for the first time in
this project. The robot still reached the goal but took 18.34 s against 7.00 s undrifted, wandering
under a bad estimate. Gap 1 of epic 02's three is closed. A first attempt at `--drift 0.3` returned
0 failures and was *not* a bug: that episode ended at 7.0 s and the threshold is crossed at ~11 s.
Checking the trace (error 0.06 -> 0.45 m) is what distinguished "too short" from "not applied", and
is worth doing before believing a zero.

### How strong the regression suite is, honestly

Weak today, and the code says so. The library holds independent leaves, so promoting one cannot
change another's result except through a bug — shared module state, a poisoned cache, a
monkeypatch. Those are real and this catches them. It becomes the interesting check when decorators
arrive, because a `RateLimitedRetry` wrapped around an existing action genuinely changes that
action's unit result. Built now so it is already running and already green.

On a regression the candidate is **rolled back** and the incumbent wins. `Gate._rollback` reaches
into `Library`'s internals in exactly one place rather than giving `Library` a public `remove()`,
which #7 forbids: a candidate that failed the suite was never admitted, and leaving it would
corrupt both the snapshot tag and G1's node count.

### Contract note

`Scenario` gained a `world` **property**, not a field. Chunk 11a declared the field list closed and
this stores, seals and hashes nothing — it exists so the rollout can ask every scenario the same
question and so `MicroScenario`, which is not a benchmark scenario, can answer differently.

### For chunk 15

`run_unit` returns a `UnitResult` carrying the reason on rejection, which is the denominator G1's
pass rate needs and the artefact the plan calls the most informative thing the pipeline produces.
A rejection is an outcome, not an error.

### For chunk 17

`MIN_EFFECT_RATE`, `MIN_EFFECT_REL`, and the fact that a unit scenario may raise the floor but
never lower it.
