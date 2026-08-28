"""
core/tests/test_controllers.py

Sim-free. Hand-built Observations for the behavioural claims, and the idealised
unicycle in core/tests/kinematic.py for the end-to-end ones.

The reset() tests are the reason reset() is in the protocol. demo_waypoints
(drive_go2.py:135) keeps its route index in a mutable default argument, so a
second episode in the same process starts mid-route. A batch runner over a fixed
seed set does exactly that, and the symptom would be a handful of quietly
mis-scored episodes rather than an error.
"""

import ast
import math
import pathlib

import numpy as np
import pytest

import controllers
from core.observation import Command, Observation
from core.scenario import load
from core.tests.kinematic import command_sequence, rollout
from core.metrics import score

CONTROLLER_DIR = pathlib.Path(controllers.__file__).parent
NAMES = controllers.available()


def _obs(scenario, pose, goal=None, ranges=None):
    angles = scenario.beam_angles()
    if ranges is None:
        ranges = np.full(angles.shape, np.inf)
    return Observation(
        t=0.0, ranges=ranges, range_angles=angles, range_max=scenario.range_max,
        pose_estimate=pose, pose_error_estimate=0.0,
        goal=goal or scenario.goal, contact=False,
    )


# --- registry --------------------------------------------------------------

def test_registry_lists_and_builds_both():
    assert NAMES == ["goal_seeker", "reactive_avoid"]
    for name in NAMES:
        assert hasattr(controllers.make(name), "step")


def test_scaffolding_is_resolvable_but_never_listed_as_a_condition():
    """Non-negotiable #3, enforced rather than remembered.

    Scaffolding is registered so `run_episode.py --controller` can reach it, and
    excluded from available() so a batch runner iterating "every controller"
    cannot sweep it into an evaluation. Same shape as all_scenarios() excluding
    the sealed split, and the same failure it prevents.
    """
    scaffold = controllers.scaffolding()
    assert scaffold, "the safe-list is only meaningful if something is on it"
    assert not set(scaffold) & set(controllers.available())
    for name in scaffold:
        assert hasattr(controllers.make(name), "step"), name
    assert set(controllers._REGISTRY) == set(scaffold) | set(controllers.available())


@pytest.mark.parametrize("name", controllers.scaffolding())
def test_scaffolding_says_so_in_its_docstring(name):
    """Every scaffold in this repo announces itself; see walk_forward.py,
    goal_seeker.py and drive_go2.py's demo sources.

    Checked on the REGISTRY TARGET, not on the instance. `toy_tree` is a factory
    returning a nodes.tree.Tree, so an instance-based check reads Tree's
    docstring — a generic one that would pass while the warning was missing from
    the thing a reader actually opens.
    """
    import importlib
    module_name, _, attr = controllers._REGISTRY[name].partition(":")
    module = importlib.import_module(module_name)
    doc = (module.__doc__ or "") + (getattr(module, attr).__doc__ or "")
    assert "NOT NAVIGATION" in doc.upper()
    assert "benchmark condition" in doc


def test_the_sweep_scaffold_never_reads_the_goal():
    """It exists to make the robot move so the disclosure latch can be observed
    under physics. If it ever read obs.goal it would become a search baseline
    nobody pre-registered, and the search family would silently acquire a
    fourth condition."""
    source = (CONTROLLER_DIR / "scaffold_sweep.py").read_text()
    tree = ast.parse(source)
    reads = [n for n in ast.walk(tree)
             if isinstance(n, ast.Attribute) and n.attr == "goal"]
    assert not reads, "scaffold_sweep read obs.goal; it must never navigate"


def test_unknown_controller_names_the_known_ones():
    with pytest.raises(KeyError, match="goal_seeker"):
        controllers.make("nope")


# --- goal_seeker -----------------------------------------------------------

def test_goal_seeker_turns_when_the_goal_is_behind():
    sc = load("doorway_traverse")
    c = controllers.make("goal_seeker")
    c.reset(sc, sc.rng())
    cmd = c.step(_obs(sc, (5.0, 5.0, 0.0), goal=(0.0, 5.0)))
    assert cmd.vx == pytest.approx(0.0)
    assert abs(cmd.wz) > 1.0


def test_goal_seeker_walks_when_the_goal_is_ahead():
    sc = load("doorway_traverse")
    c = controllers.make("goal_seeker")
    c.reset(sc, sc.rng())
    cmd = c.step(_obs(sc, (5.0, 5.0, 0.0), goal=(9.0, 5.0)))
    assert cmd.vx > 0.5
    assert abs(cmd.wz) < 0.1


def test_goal_seeker_ignores_ranges_entirely():
    """It is the no-sensing reference. If it ever starts reacting to beams it
    has stopped being the thing the failure metrics are validated against."""
    sc = load("doorway_traverse")
    c = controllers.make("goal_seeker")
    c.reset(sc, sc.rng())
    ahead = _obs(sc, (5.0, 5.0, 0.0), goal=(9.0, 5.0))
    blocked = _obs(sc, (5.0, 5.0, 0.0), goal=(9.0, 5.0),
                   ranges=np.full(sc.beam_angles().shape, 0.2))
    assert c.step(ahead).as_tuple() == c.step(blocked).as_tuple()


# --- reactive_avoid --------------------------------------------------------

def test_reactive_avoid_does_not_charge_a_wall_dead_ahead():
    sc = load("doorway_traverse")
    c = controllers.make("reactive_avoid")
    c.reset(sc, sc.rng())
    angles = sc.beam_angles()
    ranges = np.full(angles.shape, np.inf)
    ranges[np.abs(angles) < math.radians(40)] = 0.4      # wall across the front
    cmd = c.step(_obs(sc, (5.0, 5.0, 0.0), goal=(9.0, 5.0), ranges=ranges))
    assert cmd.vx < sc.max_lin * 0.5


def test_reactive_avoid_steers_towards_the_gap():
    """Blocked ahead, open to the left: it should turn left, not stop.

    The gap has to be wider than 2*CONE, or the robot correctly refuses it: a
    direction is only admissible if the whole cone the body sweeps is clear."""
    sc = load("doorway_traverse")
    c = controllers.make("reactive_avoid")
    c.reset(sc, sc.rng())
    angles = sc.beam_angles()
    ranges = np.full(angles.shape, 0.5)
    ranges[(angles > math.radians(30)) & (angles < math.radians(120))] = 5.0
    cmd = c.step(_obs(sc, (5.0, 5.0, 0.0), goal=(9.0, 5.0), ranges=ranges))
    assert cmd.wz > 0.3


def test_reactive_avoid_goes_straight_at_an_unobstructed_goal():
    sc = load("doorway_traverse")
    c = controllers.make("reactive_avoid")
    c.reset(sc, sc.rng())
    cmd = c.step(_obs(sc, (5.0, 5.0, 0.0), goal=(9.0, 5.0)))
    assert cmd.vx > 0.5 and abs(cmd.wz) < 0.1


# --- reset() ---------------------------------------------------------------

@pytest.mark.parametrize("name", NAMES)
@pytest.mark.parametrize("scenario_id", ["doorway_traverse", "blind_corner"])
def test_reset_makes_a_second_episode_identical_to_the_first(name, scenario_id):
    """The direct guard against drive_go2.py:135. Run a scripted sequence,
    reset, run it again; the two command sequences must match exactly."""
    sc = load(scenario_id)
    c = controllers.make(name)
    first = command_sequence(sc, c)
    second = command_sequence(sc, c)
    assert first == second


@pytest.mark.parametrize("name", NAMES)
def test_no_module_level_mutable_default_arguments(name):
    """The shape of the bug, not just its symptom: a def whose default is a
    list, dict or set is state that outlives the call."""
    module = controllers._REGISTRY[name].split(":")[0].split(".")[-1]
    tree = ast.parse((CONTROLLER_DIR / f"{module}.py").read_text())
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            for default in node.args.defaults + node.args.kw_defaults:
                assert not isinstance(default, (ast.List, ast.Dict, ast.Set)), (
                    f"{module}.{node.name} has a mutable default argument")


@pytest.mark.parametrize("name", NAMES)
def test_no_global_random(name):
    """Randomness must come from the rng handed to reset(), which is seeded from
    the scenario. A global draw makes a run unreproducible for reasons that do
    not appear anywhere in the manifest."""
    module = controllers._REGISTRY[name].split(":")[0].split(".")[-1]
    source = (CONTROLLER_DIR / f"{module}.py").read_text()
    assert "np.random." not in source
    assert "import random" not in source


# --- the behavioural claims in the docstrings ------------------------------

@pytest.mark.parametrize("scenario_id", ["doorway_traverse", "blind_corner",
                                         "return_home"])
def test_goal_seeker_fails_as_advertised(scenario_id):
    """It has no obstacle awareness, so it must not solve anything. If it starts
    succeeding, the failure half of the scoring code is no longer exercised."""
    sc = load(scenario_id)
    r = score(rollout(sc, controllers.make("goal_seeker")), sc)
    assert not r["goal_reached"]
    assert r["collisions"] >= 1


@pytest.mark.parametrize("scenario_id", ["doorway_traverse", "blind_corner",
                                         "return_home"])
def test_reactive_avoid_solves_the_dev_set(scenario_id):
    """Idealised kinematics, so this is a claim about the controller and the
    scenario, not about the Go2. It is what makes the dev set demonstrably
    solvable before any of it is run in the simulator."""
    sc = load(scenario_id)
    r = score(rollout(sc, controllers.make("reactive_avoid")), sc)
    assert r["goal_reached"], r
    assert r["termination"] == "goal"
    assert 0.0 < r["spl"] <= 1.0


def test_reactive_avoid_refuses_a_gap_narrower_than_the_body():
    """A 20 deg slot is a ray's-eye illusion, not a doorway. The cone check is
    what stops the robot walking at it."""
    sc = load("doorway_traverse")
    c = controllers.make("reactive_avoid")
    c.reset(sc, sc.rng())
    angles = sc.beam_angles()
    ranges = np.full(angles.shape, 0.5)
    ranges[np.abs(angles - math.radians(60)) < math.radians(10)] = 5.0
    cmd = c.step(_obs(sc, (5.0, 5.0, 0.0), goal=(9.0, 5.0), ranges=ranges))
    assert cmd.vx == 0.0


def test_boxed_in_turns_towards_the_goal_not_by_beam_order():
    """With the front uniformly blocked every direction is equally open, so the
    tie-break decides. It must be the goal, not the order of the beam array."""
    sc = load("doorway_traverse")
    c = controllers.make("reactive_avoid")
    c.reset(sc, sc.rng())
    ranges = np.full(sc.beam_angles().shape, 0.4)
    left = c.step(_obs(sc, (5.0, 5.0, 0.0), goal=(5.0, 9.0), ranges=ranges))
    right = c.step(_obs(sc, (5.0, 5.0, 0.0), goal=(5.0, 1.0), ranges=ranges))
    assert left.wz > 0.0 and right.wz < 0.0
