"""
core/tests/test_localisation.py

Chunk 19's guards: the localisation socket, and the proof it changed nothing.

**This chunk adds capability and changes no behaviour**, so most of what is
asserted here is an absence — the drift path produces what it always produced,
the trace is unchanged, and a scenario that asks for something unimplemented
fails loudly rather than quietly running with a perfect pose.

SIM-FREE. No simulator, no GPU.
"""

import math

import numpy as np
import pytest

from core.drift import DriftModel, NoDrift, make_drift
from core.localisation import (DriftLocaliser, Localiser, PerfectLocaliser,
                               make_localiser)
from core.scenario import (KINDS, PERFECT, SLAM, DriftSpec, LocalisationSpec,
                           SensorNoiseSpec, load, validate)
from core.tests.kinematic import rollout

import controllers


def _dev(scenario_id="doorway_traverse"):
    return load(scenario_id)


# --- the protocol -----------------------------------------------------------

def test_the_shipped_localisers_satisfy_the_protocol():
    for obj in (PerfectLocaliser(), DriftLocaliser(NoDrift())):
        assert isinstance(obj, Localiser), obj


def test_error_estimate_is_predicted_uncertainty_never_realised_error():
    """The single easiest way for ground truth to re-enter Observation.

    A localiser knows its noise model; it does not know its own realisation. If
    `error_estimate()` returned the realised error, `IsLocalisationDegraded`
    would be a node that reads the answer rather than a node that infers it.
    """
    spec = DriftSpec(sigma_xy=0.12, sigma_yaw=0.06)
    scenario = _dev()
    drift = DriftModel(spec, scenario.drift_rng())
    loc = DriftLocaliser(drift)
    loc.reset(scenario, scenario.localisation_rng())

    for step in range(200):
        loc.update(0.02, (1.0 + step * 0.01, 2.0, 0.0), None, None)

    predicted, realised = loc.error_estimate(), drift.true_error()
    assert predicted > 0.0 and realised > 0.0
    # Provably different, not merely usually different: the prediction is a
    # deterministic function of elapsed time and the realisation is a draw.
    assert predicted != realised
    assert predicted == pytest.approx(spec.sigma_xy * math.sqrt(200 * 0.02))


def test_the_predicted_error_is_identical_across_seeds():
    """It is `sigma * sqrt(t)` and nothing else — so it carries no information
    about the realisation, which is the property the test above depends on."""
    spec = DriftSpec(sigma_xy=0.12, sigma_yaw=0.06)
    predictions = []
    for seed in (1, 2, 3):
        loc = DriftLocaliser(DriftModel(spec, np.random.default_rng(seed)))
        for _ in range(50):
            loc.update(0.02, (0.0, 0.0, 0.0), None, None)
        predictions.append(loc.error_estimate())
    assert len(set(predictions)) == 1


# --- the default path is the path it always was -----------------------------

def test_a_scenario_with_neither_field_resolves_to_the_drift_model():
    """Byte-identity here is STRUCTURAL: it is the same object, wrapped."""
    loc = make_localiser(_dev())
    assert isinstance(loc, DriftLocaliser)
    assert isinstance(loc.drift, NoDrift)


def test_the_adapter_reproduces_the_drift_model_step_for_step():
    """The mapping the runner used to do inline: advance(dt), then
    estimate(pose). If these ever diverge, every drifted result moves."""
    spec = DriftSpec(sigma_xy=0.12, sigma_yaw=0.06)
    scenario = _dev()

    bare = DriftModel(spec, scenario.drift_rng())
    wrapped = DriftLocaliser(DriftModel(spec, scenario.drift_rng()))

    for step in range(100):
        pose = (1.0 + 0.01 * step, 2.0 - 0.005 * step, 0.1 * step)
        bare.advance(0.02)
        wrapped.update(0.02, pose, None, None)
        assert wrapped.estimate() == bare.estimate(pose)
        assert wrapped.error_estimate() == bare.error_estimate()


def _rollout_the_old_way(scenario, controller, dt=0.02):
    """`core/tests/kinematic.py` EXACTLY as it stood before chunk 19.

    Copied rather than imported, deliberately: the point is to compare the new
    code path against the old one, and importing the new one would compare it
    with itself. If chunk 19's substitution is behaviour-preserving, this
    produces the same trace, step for step, forever.
    """
    import math as _math

    from core.drift import make_drift
    from core.geometry import ROBOT_RADIUS, clearance, raycast
    from core.metrics import EpisodeTrace
    from core.observation import Observation, clamp

    BLOCK_MARGIN = 0.55
    controller.reset(scenario, scenario.rng())
    angles = scenario.beam_angles()
    x, y, yaw = scenario.start
    trace = EpisodeTrace()
    goal_known = scenario.goal_known
    drift = make_drift(scenario)
    world = scenario.world

    for step in range(int(scenario.time_limit / dt)):
        t = step * dt
        gap = float(clearance((x, y), world)) - ROBOT_RADIUS
        goal_known = goal_known or scenario.disclosed_at((x, y))
        drift.advance(dt)
        obs = Observation(
            t=t,
            ranges=raycast((x, y), yaw, angles, scenario.range_max, world),
            range_angles=angles,
            range_max=scenario.range_max,
            pose_estimate=drift.estimate((x, y, yaw)),
            pose_error_estimate=drift.error_estimate(),
            goal=scenario.goal if goal_known else None,
            contact=gap <= 0.0,
        )
        command = clamp(controller.step(obs), scenario.max_lin, scenario.max_ang)
        trace.append(t, (x, y, 0.55, yaw), command, gap, gap <= 0.0,
                     drift.estimate((x, y, yaw)), goal_known)

        nx = x + (command.vx * _math.cos(yaw) - command.vy * _math.sin(yaw)) * dt
        ny = y + (command.vx * _math.sin(yaw) + command.vy * _math.cos(yaw)) * dt
        if float(clearance((nx, ny), world)) > ROBOT_RADIUS * BLOCK_MARGIN:
            x, y = nx, ny
        yaw = (yaw + command.wz * dt + _math.pi) % (2 * _math.pi) - _math.pi
        if _math.hypot(x - scenario.goal[0], y - scenario.goal[1]) <= 0.4:
            break
    return trace


def test_the_rollout_is_bit_identical_to_the_pre_chunk_19_code_path():
    """The sim-free half of chunk 19's byte-identity criterion.

    `core/tests/kinematic.py` now builds a localiser instead of a drift model,
    and casts the scan one line earlier. Neither may move a single number: every
    node in the library was admitted through this rollout, so a change here
    silently re-decides admissions that are already committed.

    Checked WITH drift as well as without — the drifted path is the one where
    an ordering mistake between `advance(dt)` and `estimate(pose)` would show.
    """
    import dataclasses

    from core.metrics import score

    cases = []
    for scenario_id in ("doorway_traverse", "blind_corner"):
        base = _dev(scenario_id)
        cases.append(base)
        cases.append(dataclasses.replace(
            base, drift=DriftSpec(sigma_xy=0.12, sigma_yaw=0.06)))

    for scenario in cases:
        for name in ("goal_seeker", "reactive_avoid"):
            new_trace = rollout(scenario, controllers.make(name))
            old_trace = _rollout_the_old_way(scenario, controllers.make(name))
            label = (scenario.id, name, scenario.drift is not None)

            assert len(new_trace) == len(old_trace), label
            assert new_trace.rows == old_trace.rows, label
            assert score(new_trace, scenario) == score(old_trace, scenario), label


def test_perfect_localiser_and_no_drift_agree_exactly():
    """`PerfectLocaliser` is `NoDrift` reachable through the new field. If the
    two ever differ, the ground-truth control is not a control."""
    perfect, none = PerfectLocaliser(), DriftLocaliser(NoDrift())
    scenario = _dev()
    for loc in (perfect, none):
        loc.reset(scenario, scenario.localisation_rng())

    for step in range(50):
        pose = (1.0 + 0.01 * step, 2.0, 0.3 * step - math.pi)
        perfect.update(0.02, pose, None, None)
        none.update(0.02, pose, None, None)
        assert perfect.estimate() == none.estimate()
        assert perfect.error_estimate() == none.error_estimate()


def test_a_control_run_writes_no_sidecar():
    """An empty `diagnostics()` is what suppresses `slam.csv`, so a run
    directory produced today matches one produced before chunk 19."""
    assert PerfectLocaliser().diagnostics() == {}
    assert DriftLocaliser(NoDrift()).diagnostics() == {}
    assert DriftLocaliser(
        DriftModel(DriftSpec(0.1, 0.05), np.random.default_rng(0))
    ).diagnostics() == {}


# --- the contract change ----------------------------------------------------

def test_drift_and_localisation_are_mutually_exclusive():
    import dataclasses
    scenario = dataclasses.replace(
        _dev(),
        drift=DriftSpec(sigma_xy=0.12, sigma_yaw=0.06),
        localisation=LocalisationSpec(kind=PERFECT))
    with pytest.raises(ValueError, match="mutually exclusive"):
        validate(scenario)


def test_an_unknown_localisation_kind_is_rejected_at_validate():
    import dataclasses
    scenario = dataclasses.replace(
        _dev(), localisation=LocalisationSpec(kind="vibes"))
    with pytest.raises(ValueError, match="unknown localisation kind"):
        validate(scenario)


def test_the_slam_guard_is_retired_now_that_chunk_23_landed():
    """Chunk 19's `_drift_ready()` pattern, and its retirement.

    While `core/slam/localiser.py` did not exist, a scenario asking for SLAM had
    to fail LOUDLY: the alternative was an episode running with a silently
    perfect pose and scoring `localisation_failures: 0`, which looks exactly
    like success. The module exists now, so the probe could only ever return
    True, and a probe that cannot fail is a comment pretending to be code.

    ⚠️ **The loud failure did not go away, it moved and got sharper** — see the
    companion test below. That is what makes this retirement safe rather than a
    guard quietly deleted.
    """
    import dataclasses
    from core.slam.localiser import SlamLocaliser
    scenario = dataclasses.replace(
        _dev(), localisation=LocalisationSpec(
            kind=SLAM, odom_sigma_lin=0.12, odom_sigma_ang=0.060))
    # Well-formed, so validate() accepts it: whether it can RUN is the runner's
    # question, and validate() must not drag a SLAM stack in at import.
    validate(scenario)
    assert isinstance(make_localiser(scenario), SlamLocaliser)


def test_slam_with_perfect_odometry_is_still_refused():
    """Where chunk 19's loud failure went.

    The failure that mattered was never "the module is missing" — it was "SLAM
    is configured into being a free ride and reports a number that looks like
    success". Zero odometry noise is the remaining way to do that: the estimate
    would be exact, a loop closure would have nothing to correct, and the
    reported pose error would measure the simulator rather than SLAM.
    """
    import dataclasses
    scenario = dataclasses.replace(
        _dev(), localisation=LocalisationSpec(kind=SLAM))
    validate(scenario)
    localiser = make_localiser(scenario)       # constructing is fine
    with pytest.raises(ValueError, match="PERFECT dead reckoning"):
        localiser.reset(scenario, scenario.localisation_rng())


def test_the_sensor_noise_guard_is_retired_now_that_chunk_20_landed():
    """Chunk 19 declared `SensorNoiseSpec` and made a scenario carrying one fail
    loudly, so that chunks 21-23 could not silently match against
    `core.geometry.raycast`'s EXACT ranges and report centimetre accuracy.

    Chunk 20 implemented it, so the guard has done its job and the scenario now
    loads. The guard itself is still asserted below — it is the mechanism, and
    `localisation` still depends on the same pattern until chunk 23.
    """
    import dataclasses
    scenario = dataclasses.replace(
        _dev(), sensor_noise=SensorNoiseSpec(sigma_range_frac=0.005))
    validate(scenario)          # no longer raises

    from core.scenario import _sensor_noise_ready
    assert _sensor_noise_ready()


def test_the_readiness_probe_is_what_gates_a_declared_field():
    """The mechanism, independent of whether any given chunk has landed.

    A declared-but-unimplemented field must be a hard error and never a silent
    no-op — a scenario running with its noise or its localiser quietly disabled
    scores like a clean run and looks exactly like success.
    """
    import core.scenario as sc

    real = sc._sensor_noise_ready
    sc._sensor_noise_ready = lambda: False
    try:
        import dataclasses
        scenario = dataclasses.replace(
            _dev(), sensor_noise=SensorNoiseSpec(sigma_range_frac=0.005))
        with pytest.raises(ValueError, match="does not exist yet"):
            validate(scenario)
    finally:
        sc._sensor_noise_ready = real


def test_perfect_is_reachable_through_the_new_field_today():
    import dataclasses
    scenario = dataclasses.replace(
        _dev(), localisation=LocalisationSpec(kind=PERFECT))
    assert isinstance(make_localiser(validate(scenario)), PerfectLocaliser)


def test_the_kind_vocabulary_is_closed():
    assert KINDS == {PERFECT, SLAM}


# --- what chunk 19 must NOT have moved --------------------------------------

def test_drift_keeps_its_own_api_untouched():
    """`drift` is not renamed, not deprecated and not touched. The sealed split,
    every committed result and test_drift.py all reference it, so the adapter
    exists precisely so these signatures do not have to change."""
    import inspect
    assert inspect.signature(DriftModel.estimate).parameters.keys() == {
        "self", "true_pose"}
    assert inspect.signature(DriftModel.advance).parameters.keys() == {
        "self", "dt"}
    assert make_drift(_dev()).__class__ is NoDrift


def test_the_trace_columns_are_unchanged():
    """Scoring is a pure function of the trace and `scripts/rescore.py` depends
    on it, which is why diagnostics go to a sidecar and never into the trace."""
    from core.metrics import TRACE_COLUMNS
    assert "slam" not in " ".join(TRACE_COLUMNS).lower()
    assert "localis" not in " ".join(TRACE_COLUMNS).lower()


def test_the_frozen_digest_did_not_move():
    """Adding fields to `Scenario` must not disturb the pre-registration.
    FROZEN holds module-level constants and FROZEN_FILES hashes the sealed split
    and the USD — neither is a field list, and this test says so out loud."""
    from core.preregistration import verify
    verify()


def test_the_four_rng_streams_are_distinct():
    """Each stream exists so one source of randomness cannot shift another.
    Sharing any two would let a condition that draws differently face a
    different world, which is what #10 forbids."""
    scenario = _dev()
    draws = [tuple(stream().normal(size=4))
             for stream in (scenario.rng, scenario.drift_rng,
                            scenario.sensor_rng, scenario.localisation_rng)]
    assert len(set(draws)) == 4


def test_micro_scenarios_carry_the_same_streams():
    """A unit scenario that localised differently from the benchmark would be a
    unit scenario that proves nothing."""
    from harness.micro import MicroScenario, room
    micro = MicroScenario(id="unit", world=room(4.0, 4.0),
                          start=(1.0, 1.0, 0.0), goal=(3.0, 3.0),
                          time_limit=10.0, seed=7)
    scenario = _dev()
    for name in ("rng", "drift_rng", "sensor_rng", "localisation_rng"):
        assert hasattr(micro, name), name
    # Same salt, so the same seed gives the same stream in both.
    import dataclasses
    assert (list(micro.localisation_rng().normal(size=3))
            == list(dataclasses.replace(scenario, seed=7)
                    .localisation_rng().normal(size=3)))
