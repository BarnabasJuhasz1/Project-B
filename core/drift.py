"""
core/drift.py

Localisation drift: a seeded random walk on the pose estimate.

WHY THIS EXISTS. `run_episode.observe()` set `pose_estimate` to the true pose,
so `localisation_failures` was structurally 0 in every result ever committed and
`IsLocalisationDegraded` and `MaintainHomingVector` could not pass or fail a unit
scenario at all. That is gap 1 of the three epic 02 opens by naming, and it gates
roughly a third of the interesting cards.

`DriftSpec` was DECLARED in `core/scenario.py` by chunk 11a, ahead of this
module, so that the sealed held-out split could reference it. Its field list is
fixed: this module implements those fields and does not add to them. Until this
file existed, a scenario carrying a DriftSpec refused to load — deliberately,
because a `return_degraded` episode silently running with drift disabled would
score `localisation_failures: 0` and look perfectly fine.

TWO THINGS THAT ARE EASY TO GET WRONG
-------------------------------------
**`pose_error_estimate` is the model's PREDICTED uncertainty, not the realised
error.** A robot knows its noise model; it does not know its own realisation. If
this field carried the true error it would be ground truth entering
`Observation` through the back door, and `IsLocalisationDegraded` would be a
node that reads the answer. So it is `sigma_xy * sqrt(t)` — a deterministic
function of elapsed time, identical across seeds — and the realised error
wanders around it.

**sigma is RADIAL, not per-axis.** `DriftSpec.sigma_xy` is metres per root-second
of RMS *distance* error, which is what makes the arithmetic sealed with the split
correct: error reaches LOC_FAIL_THRESHOLD at t = (threshold / sigma)^2, so
DRIFT_MODERATE (0.12) crosses 1.0 m at ~69 s. Drawing each axis with std
`sigma*sqrt(dt)` would give radial RMS `sigma*sqrt(2t)` and cross ~40% early, so
each axis is drawn with `sigma * sqrt(dt / 2)` and the two variances sum to
`sigma^2 * t`.
"""

import math

import numpy as np


class DriftModel:
    """A random walk on (x, y, yaw), advanced once per control step.

    Seeded from the scenario's rng, so drift is part of the episode rather than
    noise on top of it: the same scenario and seed produce the same drift in
    every condition, which is what non-negotiable #10 requires of anything that
    differs between conditions.
    """

    def __init__(self, spec, rng):
        if spec is None:
            raise ValueError("DriftModel needs a DriftSpec; None means no drift")
        if spec.sigma_xy < 0 or spec.sigma_yaw < 0:
            raise ValueError(f"drift sigmas must be non-negative: {spec}")
        self._spec = spec
        self._rng = rng
        self._offset = np.zeros(2)
        self._yaw_offset = 0.0
        self._t = 0.0

    def reset(self):
        self._offset = np.zeros(2)
        self._yaw_offset = 0.0
        self._t = 0.0

    def advance(self, dt):
        """Take one step of the walk. Call once per control step, before observing."""
        if dt <= 0:
            raise ValueError(f"drift step needs a positive dt, got {dt}")
        # Per-axis std of sigma*sqrt(dt/2) makes the two variances sum to
        # sigma^2*dt, so the RADIAL error grows as sigma*sqrt(t). See the module
        # docstring: the alternative reading crosses the failure threshold 40%
        # early and would silently invalidate the magnitudes sealed with the
        # held-out split.
        self._offset += self._rng.normal(
            0.0, self._spec.sigma_xy * math.sqrt(dt / 2.0), size=2)
        self._yaw_offset += float(self._rng.normal(
            0.0, self._spec.sigma_yaw * math.sqrt(dt)))
        self._t += dt

    def estimate(self, true_pose):
        """The belief: true pose plus the accumulated offset.

        `true_pose` is (x, y, yaw). The runner holds the truth and passes it
        here; what comes back is the only pose a controller ever sees.
        """
        x, y, yaw = true_pose
        ex = float(x + self._offset[0])
        ey = float(y + self._offset[1])
        eyaw = (yaw + self._yaw_offset + math.pi) % (2 * math.pi) - math.pi
        return (ex, ey, eyaw)

    def error_estimate(self):
        """What the robot BELIEVES its position error to be: the RMS the noise
        model predicts at this elapsed time. Deterministic, seed-independent,
        and deliberately not the realised error — see the module docstring."""
        return self._spec.sigma_xy * math.sqrt(self._t)

    def true_error(self):
        """The realised radial error. GROUND TRUTH — for tests and for scoring,
        never for anything a controller can reach."""
        return float(np.hypot(*self._offset))

    @property
    def elapsed(self):
        return self._t


class NoDrift:
    """The null object, so callers never branch on `scenario.drift is None`.

    Every episode committed so far ran with a perfect pose estimate, and every
    one of them must keep scoring identically. Making "no drift" a real object
    with identity behaviour is what guarantees that: there is no `if drift:` in
    the runner to get wrong, and the no-drift path is the same code path it
    always was rather than a branch that merely looks equivalent.
    """

    def reset(self):
        pass

    def advance(self, dt):
        pass

    def estimate(self, true_pose):
        return (float(true_pose[0]), float(true_pose[1]), float(true_pose[2]))

    def error_estimate(self):
        return 0.0

    def true_error(self):
        return 0.0

    @property
    def elapsed(self):
        return 0.0


def make_drift(scenario):
    """The drift model for this scenario, or NoDrift. One place, because the
    runner and core/tests/kinematic.py must build it identically — a node whose
    unit scenario drifts differently from the benchmark is a node whose unit
    scenario proves nothing."""
    if getattr(scenario, "drift", None) is None:
        return NoDrift()
    return DriftModel(scenario.drift, scenario.drift_rng())
