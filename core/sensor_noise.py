"""
core/sensor_noise.py

**Per-beam lidar noise. Off by default, opt-in per scenario.**

Chunk 20 of epic 03. `SensorNoiseSpec` was DECLARED in `core/scenario.py` by
chunk 19, ahead of this module, so that chunk 19 could be the single change to
the `Scenario` contract; this module implements those fields and adds none.
Until this file existed, a scenario carrying a SensorNoiseSpec refused to load.

WHY THIS EXISTS
---------------
**`core.geometry.raycast` is exact.** It returns the analytic distance to the
nearest rectangle, with no noise, no dropouts, no mixed pixels and no
incidence-angle effects. Scan matching against exact data is not SLAM; it is a
lookup that reports centimetre accuracy and means nothing. Without this module
every number chunks 21-23 produce is an overstatement, **and the overstatement
is invisible** — the pipeline would look like it works beautifully.

OFF BY DEFAULT, AND WHY THAT IS NOT TIMIDITY
--------------------------------------------
`Observation.ranges` is read by every controller and every node that exists.
Turning noise on globally would change `reactive_avoid`'s behaviour, change every
committed result in `results/`, and make cycle 1 and cycle 2 incomparable for a
reason that has nothing to do with the thing under study. So `NoNoise` is the
null object and it does not merely *behave* like the old path — it calls
`raycast` and returns the result, so the noiseless path IS the old path and
byte-identity is structural rather than tested.

⚠️ `inf` IS LOAD-BEARING AND THIS MODULE WEAKENS IT
---------------------------------------------------
`core/geometry.py` returns `inf` rather than `max_range` specifically so a
controller can tell "nothing out there" from "a wall exactly at the sensor
limit", and `LimitSpeedToVisibleRange` — the one admitted condition-B node —
turns on exactly that difference. Both `dropout` and `spurious_max_range`
produce `inf`, so **a noisy scenario weakens a distinction that card depends
on.** That is realistic and it is a real effect on that card's node. It goes in
the write-up; it does not get tuned away until the card survives.

CALIBRATION, AND WHAT IT IS NOT
-------------------------------
A 2D lidar at this scale is roughly +/-3 cm at 6 m, so `sigma_range_frac`
defaults to 0.005 (0.5% of range) in `NOISE_MODERATE` below. **This is a
plausible sensor, not the Go2's sensor.** No datasheet was consulted and none is
claimed; the figure is a stand-in chosen to be the right order of magnitude, and
the write-up says so. What matters for this epic is that the matcher faces
*some* realistic error floor, not that the floor is this particular number.

SIM-FREE. Imports no isaacsim/omni/torch/pxr/carb, enforced by
core/tests/test_import_guard.py.
"""

import numpy as np

from core.geometry import raycast
from core.scenario import SensorNoiseSpec

# ---- below this line is data, not code.

# The calibration point named in the docstring, as a spec. Used by the
# `--sensor-noise` debugging flag and by tests; no scenario in the repo carries
# it, and none may until epic 03's own pre-registration exists.
NOISE_MODERATE = SensorNoiseSpec(
    sigma_range_frac=0.005,      # ~3 cm at 6 m
    dropout=0.02,                # 2% of beams return nothing
    spurious_max_range=0.01,     # 1% of hits are reported as "nothing there"
    sigma_angle=0.002,           # ~0.11 deg of bearing jitter
)

# ---- below this line is code, not data.


class SensorNoise:
    """The four effects, applied to one scan.

    Seeded from `Scenario.sensor_rng()` — a SEPARATE stream from the
    controller's `rng()`, from `drift_rng()` and from `localisation_rng()`. That
    separation is the one that protects the ablation: noise must be identical
    across conditions however much randomness a controller draws. If noise came
    off the shared stream, a condition that happened to sample one extra number
    would face a different scan sequence, and the four conditions would be
    compared on different worlds.
    """

    def __init__(self, spec, rng):
        if spec is None:
            raise ValueError("SensorNoise needs a SensorNoiseSpec; None means "
                             "no noise")
        for name in ("sigma_range_frac", "dropout", "spurious_max_range",
                     "sigma_angle"):
            if getattr(spec, name) < 0:
                raise ValueError(f"sensor noise {name} must be non-negative: "
                                 f"{spec}")
        for name in ("dropout", "spurious_max_range"):
            if getattr(spec, name) > 1:
                raise ValueError(f"sensor noise {name} is a probability and "
                                 f"must be <= 1: {spec}")
        self._spec = spec
        self._rng = rng

    def scan(self, origin_xy, yaw, angles, max_range, world=None):
        """One noisy scan. Owns the raycast rather than wrapping its output.

        ⚠️ **Deviation from chunk 20 as written**, which said to "apply noise
        between `raycast()` and `Observation`". Angular jitter is applied
        *before* casting — that is what stops the matcher exploiting a perfectly
        regular beam grid — so it cannot live after the cast, and splitting the
        model across both sides of the call would put two lines at every call
        site instead of one. Owning the cast keeps the call sites to exactly one
        line each, which is what the plan actually wanted.

        FOUR DRAWS, ALWAYS, IN THIS ORDER: jitter, range, spurious, dropout.
        Every array is drawn every step even when its coefficient is 0, so the
        effects are **independently controllable** — turning dropout on does not
        shift the range-noise realisation, and a scan sequence can be attributed
        to one effect rather than to the interaction of four. The cost is some
        wasted draws; the benefit is that chunk 21 can bisect which effect broke
        the matcher.
        """
        angles = np.atleast_1d(np.asarray(angles, dtype=float))
        n = angles.shape[0]
        spec = self._spec

        jitter = self._rng.normal(0.0, 1.0, size=n)
        range_noise = self._rng.normal(0.0, 1.0, size=n)
        spurious = self._rng.random(n)
        dropped = self._rng.random(n)

        ranges = raycast(origin_xy, yaw,
                         angles + jitter * spec.sigma_angle,
                         max_range, world)
        ranges = np.array(ranges, dtype=float, copy=True)

        hit = np.isfinite(ranges)

        # Range noise is proportional to range, and applies only where there is
        # a range to perturb: `inf` means the beam hit nothing, and scaling
        # infinity is not a measurement.
        ranges[hit] += range_noise[hit] * spec.sigma_range_frac * ranges[hit]

        # A negative range is not a reading. Clipping at 0 is not a fifth
        # effect; it is keeping the output well-formed, and it can only bite on
        # a beam already inside geometry.
        np.clip(ranges, 0.0, None, out=ranges)

        # A reading pushed past the sensor limit by noise is a beam that saw
        # nothing in range, which `raycast` spells `inf`. Also not a fifth
        # effect: it is what the FIRST one means at the boundary, and returning
        # a number larger than max_range would be a reading the sensor cannot
        # produce.
        ranges[hit & (ranges > max_range)] = np.inf

        # A beam that DID hit, reported as nothing there. This is what makes
        # `inf` ambiguous -- which it is on a real sensor, and which is the
        # honest cost recorded in the module docstring.
        ranges[hit & (spurious < spec.spurious_max_range)] = np.inf

        # No echo at all, applied last and to every beam: a dropout does not
        # care whether the beam hit something.
        ranges[dropped < spec.dropout] = np.inf
        return ranges


class NoNoise:
    """The null object, so callers never branch on `scenario.sensor_noise`.

    `scan()` calls `raycast` and returns it. There is no `if noise:` at a call
    site to get wrong, and the noiseless path is not a branch that merely looks
    equivalent to the old one — it IS the old one. Every episode committed so
    far ran through this shape, and every one of them must keep scoring
    identically. Same pattern, same reason, as `drift.NoDrift`.
    """

    def scan(self, origin_xy, yaw, angles, max_range, world=None):
        return raycast(origin_xy, yaw, angles, max_range, world)


def make_sensor_noise(scenario):
    """The noise model for this scenario, or NoNoise. One place, because the
    runner and `core/tests/kinematic.py` must build it identically — a node
    whose unit scenario sees a different sensor from the benchmark is a node
    whose unit scenario proves nothing. The argument `make_drift` and
    `make_localiser` both already make."""
    if getattr(scenario, "sensor_noise", None) is None:
        return NoNoise()
    return SensorNoise(scenario.sensor_noise, scenario.sensor_rng())
