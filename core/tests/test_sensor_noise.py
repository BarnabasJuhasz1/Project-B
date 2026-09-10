"""
core/tests/test_sensor_noise.py

Chunk 20's guards: the four effects, and the three properties that make a noisy
scenario interpretable.

The load-bearing ones are not the effects. They are:

  1. **noise off changes nothing** — byte-identity of the existing path;
  2. **noise is independent of the controller** — the `drift_rng` property, and
     the one that protects the ablation;
  3. **`inf` stays meaningful, and is now ambiguous** — the honest cost to
     `LimitSpeedToVisibleRange`, asserted rather than described.

SIM-FREE. No simulator, no GPU.
"""

import dataclasses

import numpy as np
import pytest

from core.geometry import raycast
from core.scenario import SensorNoiseSpec, load
from core.sensor_noise import (NOISE_MODERATE, NoNoise, SensorNoise,
                               make_sensor_noise)
from core.tests.kinematic import rollout

import controllers


def _dev(scenario_id="doorway_traverse"):
    return load(scenario_id)


def _noisy(scenario_id="doorway_traverse", **overrides):
    spec = dataclasses.replace(NOISE_MODERATE, **overrides) if overrides \
        else NOISE_MODERATE
    return dataclasses.replace(_dev(scenario_id), sensor_noise=spec)


# --- noise off changes nothing ---------------------------------------------

def test_no_noise_is_literally_raycast():
    """Not "behaves like" — IS. Byte-identity of the noiseless path is
    structural, which is why `NoNoise.scan` has one line in it."""
    scenario = _dev()
    angles = scenario.beam_angles()
    got = NoNoise().scan((1.5, 1.5), 0.0, angles, scenario.range_max)
    want = raycast((1.5, 1.5), 0.0, angles, scenario.range_max)
    assert np.array_equal(got, want, equal_nan=True)


def test_a_scenario_without_the_field_resolves_to_no_noise():
    assert isinstance(make_sensor_noise(_dev()), NoNoise)


def test_the_rollout_is_unchanged_when_no_noise_is_configured():
    """Every node in the library was admitted through this rollout, so a change
    here silently re-decides admissions that are already committed."""
    from core.metrics import score

    for scenario_id in ("doorway_traverse", "blind_corner"):
        scenario = _dev(scenario_id)
        for name in ("goal_seeker", "reactive_avoid"):
            a = rollout(scenario, controllers.make(name))
            b = rollout(scenario, controllers.make(name))
            assert a.rows == b.rows, (scenario_id, name)
            assert score(a, scenario) == score(b, scenario)


# --- determinism, and independence from the controller ----------------------

def test_the_same_seed_gives_the_same_scan_and_a_different_seed_does_not():
    scenario = _noisy()
    angles = scenario.beam_angles()

    def scan_once(seed):
        sc = dataclasses.replace(scenario, seed=seed)
        return SensorNoise(sc.sensor_noise, sc.sensor_rng()).scan(
            (1.5, 1.5), 0.0, angles, sc.range_max)

    a, b = scan_once(1001), scan_once(1001)
    assert np.array_equal(a, b, equal_nan=True)
    assert not np.array_equal(a, scan_once(2002), equal_nan=True)


def test_noise_is_independent_of_the_controller():
    """**The property that protects the ablation.**

    Noise must be identical across conditions however much randomness a
    controller draws. If it came off the shared `rng()` stream, a condition that
    happened to sample one extra number would face a different scan sequence,
    and the four conditions would be compared on different worlds.

    Asserted the way chunk 14 asserted it for drift: two controllers that behave
    nothing alike must see the identical scan at every step where the pose
    matches — here, the first step, where it provably does.
    """
    scenario = _noisy()
    angles = scenario.beam_angles()
    seen = []
    for name in ("goal_seeker", "reactive_avoid"):
        controller = controllers.make(name)
        controller.reset(scenario, scenario.rng())
        # Draw from the controller's stream as hard as it likes; the sensor
        # stream is untouched by it.
        scenario.rng().normal(size=997)
        sensor = make_sensor_noise(scenario)
        seen.append(sensor.scan(scenario.start[:2], scenario.start[2],
                                angles, scenario.range_max))
    assert np.array_equal(seen[0], seen[1], equal_nan=True)


def test_the_four_effects_are_independently_controllable():
    """Every array is drawn every step even at a coefficient of 0, so turning
    one effect on does not shift another's realisation. That is what lets chunk
    21 bisect which effect broke the matcher."""
    scenario = _dev()
    angles = scenario.beam_angles()

    def ranges(**overrides):
        spec = dataclasses.replace(
            SensorNoiseSpec(), sigma_range_frac=0.005, **overrides)
        return SensorNoise(spec, scenario.sensor_rng()).scan(
            (1.5, 1.5), 0.0, angles, scenario.range_max)

    base = ranges()
    with_dropout = ranges(dropout=0.05)
    # Where no beam was dropped, the range is the SAME number it was without
    # dropout configured at all -- not merely a similar one.
    kept = np.isfinite(base) & np.isfinite(with_dropout)
    assert kept.sum() > 0
    assert np.allclose(base[kept], with_dropout[kept], rtol=0, atol=0)


# --- the four effects -------------------------------------------------------

def test_range_noise_scales_with_range():
    """The dominant error, and what sets the matcher's accuracy floor."""
    scenario = _dev()
    angles = np.array([0.0])
    spec = SensorNoiseSpec(sigma_range_frac=0.05)
    errors = {}
    for origin, label in (((1.5, 1.5), "near"), ((0.3, 1.5), "far")):
        clean = raycast(origin, 0.0, angles, scenario.range_max)[0]
        samples = np.array([
            SensorNoise(spec, np.random.default_rng(s)).scan(
                origin, 0.0, angles, scenario.range_max)[0]
            for s in range(300)])
        samples = samples[np.isfinite(samples)]
        errors[label] = (clean, np.std(samples - clean))
    # A longer clean range must carry a proportionally larger spread.
    assert errors["far"][0] > errors["near"][0]
    assert errors["far"][1] > errors["near"][1]


def test_dropout_and_spurious_returns_both_produce_inf():
    scenario = _dev()
    angles = scenario.beam_angles()
    clean = raycast((1.5, 1.5), 0.0, angles, scenario.range_max)

    dropped = SensorNoise(SensorNoiseSpec(dropout=1.0),
                          scenario.sensor_rng()).scan(
        (1.5, 1.5), 0.0, angles, scenario.range_max)
    assert np.all(np.isinf(dropped))

    spurious = SensorNoise(SensorNoiseSpec(spurious_max_range=1.0),
                           scenario.sensor_rng()).scan(
        (1.5, 1.5), 0.0, angles, scenario.range_max)
    # Every beam that HIT is now reported as nothing there; beams that already
    # missed are unchanged.
    assert np.all(np.isinf(spurious[np.isfinite(clean)]))


def test_angular_jitter_perturbs_the_scan_without_any_range_noise():
    """Applied BEFORE casting, which is what stops the matcher exploiting a
    perfectly regular beam grid — and is why the noise model owns the raycast
    rather than wrapping its output."""
    scenario = _dev()
    angles = scenario.beam_angles()
    clean = raycast((1.5, 1.5), 0.3, angles, scenario.range_max)
    jittered = SensorNoise(SensorNoiseSpec(sigma_angle=0.02),
                           scenario.sensor_rng()).scan(
        (1.5, 1.5), 0.3, angles, scenario.range_max)
    both = np.isfinite(clean) & np.isfinite(jittered)
    assert not np.allclose(clean[both], jittered[both])


def test_a_noisy_range_never_exceeds_max_range_or_goes_negative():
    """`inf` means "nothing in range". A number larger than max_range would be a
    reading the sensor cannot produce, and a negative range is not a reading."""
    scenario = _dev()
    angles = scenario.beam_angles()
    for seed in range(20):
        ranges = SensorNoise(
            dataclasses.replace(NOISE_MODERATE, sigma_range_frac=0.4),
            np.random.default_rng(seed)).scan(
                (1.5, 1.5), 0.0, angles, scenario.range_max)
        finite = ranges[np.isfinite(ranges)]
        assert np.all(finite >= 0.0)
        assert np.all(finite <= scenario.range_max)


# --- the honest cost to LimitSpeedToVisibleRange ----------------------------

def test_noise_makes_inf_ambiguous_which_is_the_recorded_cost():
    """⚠️ Asserted, not merely described.

    `core/geometry.py` returns `inf` rather than `max_range` so a controller can
    tell "nothing out there" from "a wall at the sensor limit", and
    `LimitSpeedToVisibleRange` — the one admitted condition-B node — turns on
    exactly that difference. Dropout and spurious returns both produce `inf` on
    beams that DID hit something, so under noise `inf` no longer means what the
    card assumed. That is a real effect on that card's node and it belongs in
    the write-up, not in a tuned-down coefficient.
    """
    scenario = _dev()
    angles = scenario.beam_angles()
    clean = raycast((1.5, 1.5), 0.0, angles, scenario.range_max)
    noisy = SensorNoise(NOISE_MODERATE, scenario.sensor_rng()).scan(
        (1.5, 1.5), 0.0, angles, scenario.range_max)
    lied_about = np.isfinite(clean) & np.isinf(noisy)
    assert lied_about.sum() > 0, (
        "no beam that hit was reported as inf; the ambiguity this test exists "
        "to document is not being produced")


def test_an_all_inf_scan_does_not_crash_a_controller():
    """Total dropout is the degenerate case every node has to survive."""
    from core.observation import Observation

    scenario = _dev()
    angles = scenario.beam_angles()
    blind = np.full(angles.shape, np.inf)
    obs = Observation(
        t=0.0, ranges=blind, range_angles=angles,
        range_max=scenario.range_max, pose_estimate=(1.5, 1.5, 0.0),
        pose_error_estimate=0.0, goal=scenario.goal, contact=False)
    for name in ("goal_seeker", "reactive_avoid"):
        controller = controllers.make(name)
        controller.reset(scenario, scenario.rng())
        controller.step(obs)          # must not raise


# --- the noise model must not be so aggressive that nothing works -----------

def test_the_reference_controller_still_solves_a_scenario_under_moderate_noise():
    """If `reactive_avoid` cannot cope, the noise model is too aggressive to
    attribute anything to — chunk 20's own calibration check, run under the
    kinematic rollout rather than physics."""
    from core.metrics import score

    scenario = _noisy()
    scored = score(rollout(scenario, controllers.make("reactive_avoid")),
                   scenario)
    assert scored["goal_reached"], scored


# --- validation -------------------------------------------------------------

def test_negative_and_out_of_range_coefficients_are_rejected():
    scenario = _dev()
    with pytest.raises(ValueError, match="non-negative"):
        SensorNoise(SensorNoiseSpec(sigma_range_frac=-0.1),
                    scenario.sensor_rng())
    with pytest.raises(ValueError, match="must be <= 1"):
        SensorNoise(SensorNoiseSpec(dropout=1.5), scenario.sensor_rng())


def test_the_calibration_figure_is_the_one_the_docstring_claims():
    """~3 cm at 6 m. If this moves, the module docstring is wrong and the
    write-up's "plausible sensor" sentence is unsupported."""
    assert NOISE_MODERATE.sigma_range_frac * 6.0 == pytest.approx(0.03)


def test_a_noisy_run_is_renamed_so_it_cannot_overwrite_a_clean_result():
    """`--sensor-noise` renames to `<id>+noise`, the `--goal-hidden` discipline
    and deliberately NOT the `--drift` one: drift perturbs the pose ESTIMATE and
    leaves what the robot can see alone, whereas this changes
    `Observation.ranges`, which every controller and every node reads."""
    source = (pathlib := __import__("pathlib")).Path(
        __file__).resolve().parents[2] / "scripts" / "run_episode.py"
    text = source.read_text(encoding="utf-8")
    assert 'id=f"{scenario.id}+noise"' in text
