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
