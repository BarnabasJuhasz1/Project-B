# 25 — Recalibration: what moves, and what it costs

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 24 · **Needs Isaac Sim:** no

## Why

SLAM changes what several committed numbers *mean*. This chunk is where that is worked out
deliberately, in one place, rather than discovered halfway through cycle 2's evaluation.

It is the counterpart of chunk [17](../done/17-pre-registration.md) for this epic, and like 17 it
**blocks the first evaluation run that uses SLAM**.

## Decided

**`localisation_failures` becomes a performance metric, and that is a change to a written rule.**

`cards/store.py` currently *rejects at load* any card predicting it, and
`test_localisation_failures_really_is_controller_independent` asserts the property. Both were
right: under an open-loop random walk the metric is bit-identical across controllers that behave
nothing alike, and reporting it would let "condition B localises better" be read out of a number
that cannot show it.

Under SLAM it **can** show it. So:

- `CONTROLLER_INDEPENDENT` drops `localisation_failures` **only when the scenario carries a
  `localisation` spec**, not unconditionally. A drifted scenario keeps the old rule, because the
  old rule is still true of it. This is a per-scenario property, and the card store has to learn
  the distinction rather than flip a global.
- `test_localisation_failures_really_is_controller_independent` is **not deleted**. It is
  narrowed to drifted scenarios and a new test asserts the opposite on SLAM scenarios: two
  controllers that behave differently must produce *different* localisation error. If that new
  test does not pass, SLAM has not actually made the metric controller-dependent and no card may
  predict it yet.

**`LOC_FAIL_THRESHOLD` is re-derived, not reused.** It is 1.0 m, chosen against the random walk so
that DRIFT_MODERATE crosses it at ~69 s. SLAM's error distribution has nothing to do with that
arithmetic — it is bounded and jumpy rather than diffusive, with sawtooth drops at loop closures.
Re-derive from the chunk-24 measurements and **record the rejected alternatives**, per the
`Decided` convention.

**Committed results are not re-scored, and results/README.md says why.** Every file in `results/`
was produced under `NoDrift` or `DriftModel` and means what it meant. Re-scoring them under a
localiser they never ran with would be inventing data. Instead the **manifest must name the
localiser and its parameters**, the way it names the library snapshot (#8), and older manifests
are read as `NoDrift` by absence.

*Rejected:* a compatibility shim that back-fills a localiser field into old manifests. It would
make two genuinely different runs look like the same kind of run.

**Reporting the localisation contribution is mandatory, not optional.** The epic's named
confounding risk is that a goal-reach difference is really a localisation difference. Every
SLAM condition is therefore run twice — once with `PerfectLocaliser` on the same scenario and
seed — and **the delta is reported alongside the headline number**. A SLAM result published
without its ground-truth control is not interpretable and should not pass review.

**Cycle 2 gets its own pre-registration, and cycle 1's stands.** The X, the seed set, the episode
count and the secondary metric list are all re-opened *for cycle 2 only*, because the apparatus
changed. Cycle 1's numbers keep their pre-registration and stay comparable to each other.

## Build

- `core/metrics.py` — re-derived `LOC_FAIL_THRESHOLD`, with the derivation in a comment as
  `GOAL_RADIUS` and `FALL_HEIGHT` already carry theirs.
- `cards/store.py` — `CONTROLLER_INDEPENDENT` becomes scenario-aware.
- `cards/tests/` — narrow the old test, add its SLAM-scenario opposite.
- `core/manifest.py` — record localiser name, spec, rate and sensor-noise spec.
- `results/README.md` — a section on which localiser produced which results and why they are not
  re-scored.
- `SLAM-PRE-REGISTRATION.md` — cycle 2's, committed and tagged before the first SLAM evaluation.

## Done when

- A SLAM scenario shows `localisation_failures` differing between `goal_seeker` and
  `reactive_avoid` on the same seed — the property that licenses the rule change, demonstrated
  before the rule changes.
- A drifted scenario still shows them identical, and the narrowed test asserts it.
- A card predicting `localisation_failures` loads for a SLAM scenario and is still rejected for a
  drifted one.
- `core/tests/test_results_integrity.py` passes — `results/` and `out/runs/` have not drifted.
- Every SLAM manifest names the localiser, its rate, and the sensor-noise spec.
- `SLAM-PRE-REGISTRATION.md` is committed and tagged, and states X, the seed set, the episode
  count, the secondary metric list, and the mandatory `PerfectLocaliser` control.

## Not in scope

Running cycle 2. This chunk makes it interpretable; chunk 26 supplies the cards it should be run
with.
