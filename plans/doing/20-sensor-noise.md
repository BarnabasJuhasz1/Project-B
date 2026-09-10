# 20 — The sensor noise model

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 19 · **Needs Isaac Sim:** no

## Why

**`core.geometry.raycast` is exact.** It returns the analytic distance to the nearest rectangle,
with no noise, no dropouts, no mixed pixels and no incidence-angle effects. Scan matching against
exact data is not SLAM; it is a lookup that will report centimetre accuracy and mean nothing.

If this chunk is skipped, every number chunks 21–23 produce is an overstatement, and the
overstatement is invisible — the pipeline will look like it works beautifully.

## Decided

**Noise is opt-in per scenario, and off by default.** `Observation.ranges` is read by every
controller and every node that exists. Turning noise on globally would change `reactive_avoid`'s
behaviour, change every committed result, and make cycle 1 and cycle 2 incomparable for a reason
that has nothing to do with the thing under study.

*Rejected:* noise always on, "because a real sensor is always noisy". True, and it would silently
invalidate `results/` and the sealed split in one commit.

**A separate rng stream, exactly as drift has one.** `Scenario.sensor_rng()` alongside `rng()`
and `drift_rng()`, seeded from `[seed, 0x5E7150]`. The reason is the one `core/scenario.py`
already gives for `drift_rng`: **noise must be identical across conditions however much
randomness a controller draws.** If noise came off the shared stream, a condition that sampled
one extra number would get a different scan sequence, and the ablation would be comparing
different worlds.

**Four effects, and no more.** Each is one line to describe and one line to test:

| Effect | Model | Why it is in |
|---|---|---|
| Range noise | Gaussian, σ proportional to range | the dominant error, and what sets the matcher's accuracy floor |
| Dropout | per-beam Bernoulli → `inf` | the matcher must tolerate missing data |
| Spurious max-range | Bernoulli → `inf` on a beam that *did* hit | `inf` already means "nothing there"; this makes that ambiguous, which it is on a real sensor |
| Angular jitter | Gaussian on the beam bearing before casting | stops the matcher exploiting a perfectly regular beam grid |

*Rejected:* mixed pixels at depth discontinuities, incidence-angle dropout, multi-echo. All real,
all more code, and none of them changes what this epic is testing. Recorded here so the omission
is a decision.

⚠️ **`inf` is load-bearing and must stay meaningful.** `core/geometry.py` returns `inf` rather
than `max_range` specifically so a controller can tell "nothing out there" from "a wall exactly at
the sensor limit", and `LimitSpeedToVisibleRange` turns on precisely that difference. Dropout and
spurious-max-range both produce `inf`, so a noisy scenario **weakens a distinction a card
depends on**. That is realistic and it is a real effect on that card's node — say so in the
write-up rather than tuning the noise until the card survives.

## Build

- `core/sensor_noise.py` — `SensorNoise` applying the four effects to a clean scan, and a
  `NoNoise` null object so the noiseless path is not a branch (the `NoDrift` pattern).
- `core/scenario.py` — implement `SensorNoiseSpec` declared by 19; add `sensor_rng()`.
- `scripts/run_episode.py` and `core/tests/kinematic.py` — apply noise between `raycast()` and
  `Observation`, in exactly one place each.
- A calibration note in the module docstring: what σ was chosen and against what. A 2D lidar at
  this scale is roughly ±3 cm at 6 m; state the figure and that it is a stand-in, not a datasheet.

## Done when

- A scenario without `sensor_noise` produces a **byte-identical** trace to today's, checked
  against a committed result.
- Two runs of a noisy scenario at the same seed are byte-identical; two different seeds are not.
- Noise is **independent of the controller**: the same scenario and seed produce the identical
  scan sequence under `goal_seeker` and under `reactive_avoid`, at every step where the pose
  matches. This is the `drift_rng` property and it is the one that protects the ablation.
- Dropout and spurious returns produce `inf`, and a test asserts `LimitSpeedToVisibleRange`'s
  free-distance calculation does not crash on an all-`inf` scan.
- `reactive_avoid` still reaches the goal on `doorway_traverse` with moderate noise on — if the
  reference controller cannot cope, the noise model is too aggressive to attribute anything to.
- The chosen σ is recorded in the manifest, not only in the spec.

## Not in scope

Camera noise (chunk 27). Modelling the Go2's actual lidar — this is a plausible sensor, not that
sensor, and the write-up says so.

---

## Status 2026-08-31 — built, 609 sim-free tests green, two deviations

`core/sensor_noise.py`, both call sites, a `--sensor-noise` flag, the spec in the manifest, and
16 tests. **609 sim-free tests pass** (592 before this chunk).

### Why this was safe to take before the evaluation run

Epic 03's scheduling constraint binds anything that changes what a controller sees or what a
metric measures. The warning that had been written on the board — *"chunk 20 is where the
constraint starts to bite, because sensor noise changes `Observation.ranges`"* — **was
overstated, and this chunk's own Decided section is why**: noise is opt-in per scenario and off by
default, and *"a scenario without `sensor_noise` produces a byte-identical trace"* is a Done-when
here, not an aspiration. `NoNoise.scan()` calls `raycast` and returns it, so the noiseless path is
not a branch that looks equivalent to the old one — it **is** the old one. No scenario in the repo
carries the field.

**Chunk 21 is the real boundary.** It is the first chunk that produces a *number* rather than a
capability: a pose estimate that competes with `core/drift.py`'s.

### Deviation 1 — the noise model owns the raycast

The plan said to *"apply noise between `raycast()` and `Observation`, in exactly one place each"*.
That cannot hold: **angular jitter is applied before casting** — the plan says so itself, and it is
what stops the matcher exploiting a perfectly regular beam grid — so the model straddles the call
and cannot live after it. Splitting it would put two lines at every call site.

So `SensorNoise.scan()` owns the cast, and `NoNoise.scan()` is a one-line passthrough. Each call
site is exactly one line, which is what "exactly one place each" was protecting.

### Deviation 2 — `--sensor-noise` RENAMES the scenario, following `--goal-hidden`, not `--drift`

`run_episode.py` already draws this line and states it in the `--goal-hidden` help: *"drift adds a
noise process to an otherwise identical episode, whereas hiding the goal changes what the
controller can see, so it is a different scenario and must not be scored under the same run id"*.

Sensor noise is on the **`--goal-hidden` side of that line**, and it is not a close call: drift
perturbs the pose *estimate* and leaves the scan alone, whereas this changes `Observation.ranges`
itself. So a noisy run is `<id>+noise` and can never overwrite a clean committed result. The plan
did not raise the question; leaving the default would have meant a `--sensor-noise` run silently
overwriting `results/doorway_traverse__reactive_avoid__seed1001.json`.

### One design choice the plan did not call for

**All four noise arrays are drawn every step, even at a coefficient of 0.** The cost is wasted
draws; the benefit is that the effects are *independently controllable* — turning dropout on does
not shift the range-noise realisation, so chunk 21 can bisect which effect broke the matcher
instead of facing the interaction of four. A test pins it.

Two clamps are applied and are deliberately **not** a fifth effect: a noisy range is clipped at 0
(a negative range is not a reading) and a range pushed past `max_range` becomes `inf` (a number
larger than max range is a reading the sensor cannot produce, and `inf` is how `raycast` already
spells "nothing in range").

### The `inf` cost is asserted, not just described

The plan warns that dropout and spurious returns both produce `inf`, so a noisy scenario weakens
the distinction `LimitSpeedToVisibleRange` — the one admitted condition-B node — depends on.
`test_noise_makes_inf_ambiguous_which_is_the_recorded_cost` **asserts that beams which hit are
being reported as `inf`**, so the cost is measured rather than assumed, and it fails loudly if
someone later tunes the coefficients until the card survives.

### Verified under physics, not only sim-free

| Check | Result |
|---|---|
| Noise off → byte-identical trace | **774 lines identical** to `doorway_traverse__reactive_avoid__seed1001__20260826-173511` |
| Noisy run, same seed, twice | **770 lines identical** |
| `reactive_avoid` under `--sensor-noise 1.0` | **reaches the goal** |
| The rename | `results/doorway_traverse+noise+cap60s__…json` — the clean result is untouched |
| σ in the manifest | `sensor_noise: {sigma_range_frac: 0.005, dropout: 0.02, spurious_max_range: 0.01, sigma_angle: 0.002}` |

Clean vs noisy on the same scenario, controller and seed:

| | clean | noisy |
|---|---|---|
| `goal_reached` | True | True |
| `time_to_goal` | 15.44 | 15.36 |
| `spl` | 0.9495 | 0.9500 |
| `min_clearance` | −0.0689 | **−0.0719** |
| `collisions` | 1 | 1 |

The calibration reads correctly: the noise is real — it costs ~3 mm of clearance on the tightest
pass — and it is **not** so aggressive that the reference controller stops working, which is the
condition for attributing anything to it later. (The negative `min_clearance` and the single
collision are pre-existing on this scenario and are not caused by noise; the clean run has them
too.)

### Done when — all green

- ✅ Byte-identical trace without `sensor_noise`, under physics and under the kinematic rollout.
- ✅ Same seed byte-identical; different seeds not (sim-free), and byte-identical twice under
  physics.
- ✅ Noise independent of the controller — two controllers that behave nothing alike see the
  identical scan after the controller stream is drawn 997 times.
- ✅ Dropout and spurious returns produce `inf`; an all-`inf` scan does not crash either
  reference controller.
- ✅ `reactive_avoid` still reaches the goal under moderate noise.
- ✅ The chosen σ is in the manifest, as **values** rather than a flag — a result saying only
  "noise: on" could not be compared with one run at a different σ. `localisation` and `drift` are
  recorded the same way; `None` on all three is the state every committed result is in.

### Not done, and deliberately

`compose/spec.py:TaskSpec` carries the sensor's **geometry** (beams, FOV, range) and not its
noise. Adding a noise axis would change the composition prompt and orphan the five committed
compositions — the argument chunk 15 used to reject a prompt change, and chunk 16 to keep the
retrieval log stable. Whether a composer should be told the sensor is noisy is a **chunk 26**
question, to be taken with epic 03's own pre-registration.

**Not moved to `done/`** — same reason as 19: the chunk is complete, but epic 03 does not open
until cycle 1's evaluation is written up.
