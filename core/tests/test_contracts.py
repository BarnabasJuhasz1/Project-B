"""
core/tests/test_contracts.py

The scenario validator earns its keep here. Every case below is a failure that
would otherwise surface as a confusing zero score partway through a batch run,
long after the thing that caused it.
"""

import dataclasses
import math

import numpy as np
import pytest

from core.observation import Command, Controller, Observation, STOP, clamp
from core.scenario import DEV_SCENARIOS, all_scenarios, load, validate


def _variant(**changes):
    return dataclasses.replace(load("doorway_traverse"), **changes)


# --- scenarios -------------------------------------------------------------

@pytest.mark.parametrize("scenario", DEV_SCENARIOS, ids=lambda s: s.id)
def test_dev_scenarios_load(scenario):
    assert load(scenario.id) is not None


@pytest.mark.parametrize("scenario", DEV_SCENARIOS, ids=lambda s: s.id)
def test_dev_scenarios_are_solvable_within_the_time_limit(scenario):
    """The optimal traverse must fit in the limit with room to spare, or the
    scenario measures the speed cap rather than the navigation."""
    optimal = scenario.optimal_distance() / scenario.max_lin
    assert optimal < scenario.time_limit / 3.0, (
        f"{scenario.id}: {optimal:.1f}s optimal against a "
        f"{scenario.time_limit:.0f}s limit is too tight")


def test_goal_inside_a_wall_raises():
    with pytest.raises(ValueError, match="inside a wall"):
        validate(_variant(id="wall_goal", goal=(6.0, 2.0)))


def test_goal_in_the_sealed_room_raises_as_unreachable():
    """lab42's divider (x=6, y in [0,5]) and blind-corner wall (y=5,
    x in [6,12]) close off x in [6.075, 11.925], y in [0.075, 4.925] against the
    south and east perimeters. No door. The message must say unreachable — a
    bare 'invalid' would send someone hunting for a typo in the coordinates."""
    with pytest.raises(ValueError, match="UNREACHABLE"):
        validate(_variant(id="sealed", goal=(9.0, 2.5)))


def test_goal_outside_the_room_raises():
    with pytest.raises(ValueError, match="outside the room bounds"):
        validate(_variant(id="outside", goal=(20.0, 2.0)))


def test_unknown_scenario_lists_the_known_ones():
    with pytest.raises(KeyError, match="doorway_traverse"):
        load("no_such_scenario")


def test_seeds_are_distinct():
    seeds = [s.seed for s in all_scenarios()]
    assert len(set(seeds)) == len(seeds)


# --- sensor config ---------------------------------------------------------

def test_beam_angles_do_not_duplicate_on_a_full_circle():
    angles = load("doorway_traverse").beam_angles()
    assert len(angles) == 181
    assert not np.isclose(angles[0] % (2 * math.pi), angles[-1] % (2 * math.pi))


def test_sensor_config_is_identical_across_scenarios():
    """It is part of the task, not the controller. A condition must not be able
    to win by asking for a better sensor."""
    configs = {(s.n_beams, s.fov, s.range_max, s.max_lin, s.max_ang)
               for s in all_scenarios()}
    assert len(configs) == 1


def test_rng_is_reproducible_from_the_seed():
    a = load("doorway_traverse").rng().random(5)
    b = load("doorway_traverse").rng().random(5)
    assert np.array_equal(a, b)


# --- controller contract ---------------------------------------------------

def test_command_is_saturated_symmetrically():
    assert clamp(Command(9.0, -9.0, 9.0), 0.8, 1.0) == Command(0.8, -0.8, 1.0)
    assert clamp(Command(0.1, 0.0, -0.2), 0.8, 1.0) == Command(0.1, 0.0, -0.2)


def test_stop_is_zero():
    assert STOP.as_tuple() == (0.0, 0.0, 0.0)


def test_observation_is_frozen():
    """A controller must not be able to edit what it was handed and have that
    leak into the trace."""
    obs = Observation(
        t=0.0, ranges=np.array([1.0]), range_angles=np.array([0.0]),
        range_max=6.0, pose_estimate=(0.0, 0.0, 0.0), pose_error_estimate=0.0,
        goal=(1.0, 1.0), contact=False,
    )
    with pytest.raises(dataclasses.FrozenInstanceError):
        obs.t = 1.0


def test_observation_carries_no_ground_truth():
    """Ground truth lives on the runner and is used only for scoring. If a true
    pose ever appears here, the localisation-failure metric stops meaning
    anything and IsLocalisationDegraded becomes untestable."""
    names = {f.name for f in dataclasses.fields(Observation)}
    assert not {n for n in names if "true" in n or "gt" in n}
    assert "pose_estimate" in names


def test_controller_protocol_requires_reset():
    """reset() is the structural fix for drive_go2.py:135, where the waypoint
    index lives in a mutable default and survives into the next episode."""
    assert "reset" in Controller.__protocol_attrs__
    assert "step" in Controller.__protocol_attrs__
