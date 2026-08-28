"""
core/tests/test_drift.py

The drift model, which closes gap 1 of the three epic 02 opens by naming:
`pose_estimate` was the true pose, so `localisation_failures` was structurally 0
in every committed result and a third of the interesting cards were untestable.

Two claims carry the design and neither is obvious from reading the code.

**sigma is radial.** `DriftSpec.sigma_xy` is metres per root-second of RMS
*distance* error. That is what makes the magnitudes sealed with the held-out
split correct — and those cannot be changed now, so the implementation has to
match them rather than the other way round.

**`pose_error_estimate` is predicted, not realised.** A robot knows its noise
model, not its own realisation. If this field carried the true error then
`IsLocalisationDegraded` would be a node that reads the answer, and ground truth
would have entered `Observation` through the back door.
"""

import math

import numpy as np
import pytest

from core.drift import DriftModel, NoDrift, make_drift
from core.metrics import LOC_FAIL_THRESHOLD
from core.scenario import DriftSpec, all_scenarios, load

SPEC = DriftSpec(sigma_xy=0.12, sigma_yaw=0.06)
DT = 0.02


def _walk(spec, seed, seconds, dt=DT):
    m = DriftModel(spec, np.random.default_rng(seed))
    for _ in range(int(seconds / dt)):
        m.advance(dt)
    return m


# --- the units claim -------------------------------------------------------

@pytest.mark.parametrize("seconds", [25.0, 69.4, 150.0])
def test_radial_rms_error_grows_as_sigma_root_t(seconds):
    """The claim the sealed drift magnitudes rest on. Drawing each axis with
    std sigma*sqrt(dt) instead of sigma*sqrt(dt/2) would put this ~41% high and
    would silently invalidate every degraded scenario in the split."""
    errs = [_walk(SPEC, seed, seconds).true_error() for seed in range(400)]
    rms = math.sqrt(float(np.mean(np.square(errs))))
    assert rms == pytest.approx(SPEC.sigma_xy * math.sqrt(seconds), rel=0.08)


def test_the_failure_threshold_is_crossed_when_the_arithmetic_says():
    """t = (threshold / sigma)^2, which is the number written next to the sealed
    constants in core/scenarios_held_out.py."""
    predicted = (LOC_FAIL_THRESHOLD / SPEC.sigma_xy) ** 2
    errs = [_walk(SPEC, seed, predicted).true_error() for seed in range(400)]
    rms = math.sqrt(float(np.mean(np.square(errs))))
    assert rms == pytest.approx(LOC_FAIL_THRESHOLD, rel=0.08)


# --- the estimate is a belief, not the answer ------------------------------

def test_the_error_estimate_is_predicted_not_realised():
    """Identical across seeds, because it is a function of the noise model and
    elapsed time only. If this ever tracked the realised error, a node could
    read ground truth out of Observation."""
    estimates = {_walk(SPEC, seed, 40.0).error_estimate() for seed in range(20)}
    assert len(estimates) == 1
    assert estimates.pop() == pytest.approx(SPEC.sigma_xy * math.sqrt(40.0))

    realised = {round(_walk(SPEC, seed, 40.0).true_error(), 6) for seed in range(20)}
    assert len(realised) > 1, "the realised error must vary with the seed"


def test_the_estimate_is_the_true_pose_plus_the_offset():
    m = _walk(SPEC, 7, 30.0)
    ex, ey, _ = m.estimate((3.0, 4.0, 0.0))
    assert math.hypot(ex - 3.0, ey - 4.0) == pytest.approx(m.true_error())


def test_yaw_estimate_stays_wrapped():
    m = _walk(DriftSpec(sigma_xy=0.0, sigma_yaw=2.0), 3, 100.0)
    _, _, yaw = m.estimate((0.0, 0.0, 0.0))
    assert -math.pi <= yaw <= math.pi


# --- reproducibility -------------------------------------------------------

def test_drift_is_reproducible_from_the_scenario_seed():
    sc = next(s for s in all_scenarios("held_out") if s.drift is not None)
    a = [make_drift(sc) for _ in range(2)]
    for m in a:
        for _ in range(500):
            m.advance(DT)
    assert a[0].true_error() == a[1].true_error()


def test_drift_uses_a_separate_stream_from_the_controller():
    """Load-bearing for #10. The four conditions draw different amounts of
    randomness from `scenario.rng()`; if drift shared that stream, condition A
    and condition B would face different drift on the same episode and the
    ablation would compare two different worlds."""
    sc = load("doorway_traverse")
    assert sc.rng().bit_generator.state != sc.drift_rng().bit_generator.state

    # drawing from the controller stream cannot move the drift stream
    before = make_drift(sc.__class__(**{**sc.__dict__, "drift": SPEC}))
    rng = sc.rng()
    rng.normal(size=1000)
    after = make_drift(sc.__class__(**{**sc.__dict__, "drift": SPEC}))
    for m in (before, after):
        for _ in range(200):
            m.advance(DT)
    assert before.true_error() == after.true_error()


# --- the null object -------------------------------------------------------

def test_no_drift_is_exactly_the_identity():
    """Every result committed so far ran with a perfect pose estimate and must
    keep scoring identically. NoDrift is why that is true by construction rather
    than by a branch someone might get wrong."""
    nd = make_drift(load("doorway_traverse"))
    assert isinstance(nd, NoDrift)
    for _ in range(100):
        nd.advance(DT)
    assert nd.estimate((1.5, 2.5, 0.75)) == (1.5, 2.5, 0.75)
    assert nd.error_estimate() == 0.0
    assert nd.true_error() == 0.0


def test_every_dev_scenario_is_drift_free():
    assert all(s.drift is None for s in all_scenarios())


def test_a_negative_sigma_is_rejected():
    with pytest.raises(ValueError, match="non-negative"):
        DriftModel(DriftSpec(sigma_xy=-1.0, sigma_yaw=0.0), np.random.default_rng(0))


# --- the sealed scenarios actually degrade ---------------------------------

@pytest.mark.parametrize(
    "scenario", [s for s in all_scenarios("held_out") if s.drift is not None],
    ids=lambda s: s.id)
def test_each_degraded_scenario_crosses_the_threshold_in_time(scenario):
    """chunk 13's acceptance test, and chunk 11's promise. A degraded family a
    baseline passes anyway tests nothing.

    The magnitudes were sealed BEFORE this model existed, so this is where the
    two are reconciled. If it ever fails, the finding is about the drift model —
    a split tuned after sealing is not a split.
    """
    model = make_drift(scenario)
    crossed = False
    for _ in range(int(scenario.time_limit / DT)):
        model.advance(DT)
        if model.true_error() > LOC_FAIL_THRESHOLD:
            crossed = True
            break
    assert crossed, (
        f"{scenario.id}: never crossed {LOC_FAIL_THRESHOLD} m in "
        f"{scenario.time_limit} s at sigma_xy={scenario.drift.sigma_xy}")
