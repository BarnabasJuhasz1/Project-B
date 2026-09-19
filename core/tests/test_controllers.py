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


# --- the `node:` form (chunk 36) -------------------------------------------
#
# `node:<id>` is how one admitted leaf reaches `run_episode.py` under physics.
# The value of the result depends entirely on the tree being the SAME tree the
# admission decision was taken on, so that is what these pin.

def test_node_prefix_resolves_to_the_admission_tree():
    """Not merely "a Tree" — the same leaves `generate.admit.build_tree` builds.

    Two constructions of "the admission tree" would drift apart silently, and
    only a wiring failure would ever say so. This is the assertion that makes
    the physics result a claim about the admitted node.
    """
    from compose.compose import RECONSTRUCTION_STAGING
    from compose.library import load_library
    from generate.admit import build_tree
    from nodes.tree import Tree

    # A v1 node, archived on 2026-09-08, and deliberately still the subject:
    # `results/` commits seven episodes whose controller is a `node:` id like
    # this one, and the point of the test is that a committed result stays
    # re-scorable. It reads through the SAME widened path `controllers.make`
    # uses — resolving it here by a narrower one would compare two different
    # libraries and call the difference a drift.
    node_id = "gen_a_69724a1f2c31010e"
    tree = controllers.make(controllers.NODE_PREFIX + node_id)
    assert isinstance(tree, Tree)

    entry = load_library(staging=RECONSTRUCTION_STAGING,
                         only=[node_id]).get(node_id)
    expected = build_tree(entry.node_cls, entry.metadata)
    assert _leaf_ids(tree) == _leaf_ids(expected)
    assert node_id in _leaf_ids(tree)


def _leaf_ids(tree):
    """Node ids of every leaf in a tree, in tick order."""
    out = []

    def walk(node):
        children = getattr(node, "children", None)
        if children:
            for child in children:
                walk(child)
        else:
            out.append(node.metadata.node_id)

    walk(tree.root)
    return out


def test_every_library_node_resolves_and_is_a_controller():
    """`library_nodes()` is the listing; each entry must actually make.

    `Controller` is a plain Protocol and not runtime-checkable, so the check is
    structural: reset() and step() are what `run_episode.py` calls.
    """
    names = controllers.library_nodes()
    assert names, "the library is empty; this test is about a non-empty one"
    for name in names:
        assert name.startswith(controllers.NODE_PREFIX)
        tree = controllers.make(name)
        assert callable(tree.reset) and callable(tree.step)


def test_library_nodes_are_not_in_available():
    """A single leaf is evidence about that leaf, never a benchmark condition.
    Nothing that sweeps `available()` may pick one up."""
    assert not set(controllers.library_nodes()) & set(controllers.available())


def test_the_port_publisher_is_not_offered_as_a_node():
    """`ContractPorts` is the harness's own publisher. A tree containing only
    it drives nothing, so naming it is a mistake worth an error."""
    from compose.library import CONTRACT_PORTS_ID

    assert controllers.NODE_PREFIX + CONTRACT_PORTS_ID \
        not in controllers.library_nodes()
    with pytest.raises(KeyError):
        controllers.make(controllers.NODE_PREFIX + CONTRACT_PORTS_ID)


@pytest.mark.parametrize("node_id", [
    "nope",                       # never existed
    "gen_b_1f2fe8016b1441dc",     # admitted, then superseded by a re-record
    "gen_a_4b6fe714f7154470",     # retired with the naive arm
])
def test_a_node_outside_the_live_library_is_refused(node_id):
    """The library is one definition, not two: `node:` reads the same records
    `pass_rate` and `compose.library` do, so a node that is not in the library
    today cannot be run today."""
    with pytest.raises(KeyError):
        controllers.make(controllers.NODE_PREFIX + node_id)


# ---------------------------------------------------------------------------
# chunk 53 — the growth pairing, resolvable under physics


def test_layer_prefix_returns_the_growth_pairing():
    """`layer:<id>` IS `measure_growth._build`, leaf for leaf, in tick order.

    The whole value of a physics run on the growth pairing is that the thing
    under physics is the thing the growth row measured. A second construction
    would drift and only a wiring failure would say so.
    """
    import importlib.util
    import os

    import controllers
    from compose.compose import RECONSTRUCTION_STAGING
    from compose.library import CONTRACT_PORTS_ID, load_library
    from generate.base import load as load_base

    node_id = next(
        e.node_id for e in load_library(staging=RECONSTRUCTION_STAGING).all()
        if e.node_id != CONTRACT_PORTS_ID)

    spec = importlib.util.spec_from_file_location(
        "_mg_for_test",
        os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__)))), "scripts", "measure_growth.py"))
    growth = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(growth)

    entry = load_library(staging=RECONSTRUCTION_STAGING,
                         only=[node_id]).get(node_id)
    expected = growth._build(entry.node_cls, entry.metadata, load_base())
    actual = controllers.make(controllers.LAYER_PREFIX + node_id)

    assert ([type(c).__name__ for c in actual.root.children]
            == [type(c).__name__ for c in expected.root.children])
    # Order is priority: ports publish, the base proposes, the node revises.
    assert type(actual.root.children[-1]).__name__ != "BaseNavigator"


def test_layer_prefix_refuses_what_node_prefix_refuses():
    """One definition of "in the library", shared by both prefixes."""
    import controllers
    from compose.library import CONTRACT_PORTS_ID

    for bad in ("definitely_not_a_node", CONTRACT_PORTS_ID):
        with pytest.raises(KeyError):
            controllers.make(controllers.LAYER_PREFIX + bad)
        with pytest.raises(KeyError):
            controllers.make(controllers.NODE_PREFIX + bad)


def test_layer_prefix_is_not_listed_as_a_condition():
    """#3: it is the substrate plus one leaf, so it stays out of available()."""
    import controllers

    assert not any(n.startswith(controllers.LAYER_PREFIX)
                   for n in controllers.available())


# ---------------------------------------------------------------------------
# a REPAIR ROUND, resolvable under physics — the confirmation path


def _a_repair_round():
    """The first `<lineage>/round-NN` on disk, as (node_id, round_dir), or None.

    Skips rather than fails when nothing has been repaired yet: a repair round
    is a RUN OUTPUT under out/, not a committed record, so it is present on a
    machine that has run the loop and absent on one that has not.
    """
    import re
    from pathlib import Path

    root = Path(__file__).resolve().parents[2] / "out" / "repair"
    for lineage in sorted(p for p in root.glob("*") if p.is_dir()):
        if lineage.name == "archive":
            continue
        for rd in sorted(lineage.glob("round-*")):
            m = re.fullmatch(r"round-(\d+)", rd.name)
            if m and any(rd.glob("*.result.json")):
                return f"{lineage.name}__r{int(m.group(1)):02d}", rd
    return None


def test_a_repair_round_resolves_to_the_repair_and_never_to_the_shim():
    """⚠️ THE BUG THIS EXISTS FOR TOOK A WHOLE LOOP RUN DOWN SILENTLY.

    `ContractPorts` is always in a loaded library and sorts first, so a
    resolver that takes `entries[0]` returns the harness's own port shim for
    every repaired node. The shim does nothing, which reads as INERT, which
    reads as a plausible verdict — three rounds of it looked entirely green.
    See `repair/evidence.py:entry_for`.

    `layer:<id>__rNN` is the only way to put a repaired node under physics, so
    a confirmation run is worth exactly as much as this resolution is.
    """
    import controllers

    found = _a_repair_round()
    if found is None:
        pytest.skip("no repair round on this machine; run scripts/repair_node.py")
    node_id, _ = found

    tree = controllers.make(controllers.LAYER_PREFIX + node_id)
    names = [type(c).__name__ for c in tree.root.children]
    assert names[0] == "ContractPorts"          # the shim publishes, first
    assert names[-1] != "ContractPorts"         # and is never the thing measured
    assert len(names) == 3                      # ports, base, the repair


def test_a_repair_round_id_that_names_no_round_raises():
    """A resolver that GUESSES which entry was meant will guess wrong silently
    — the whole lesson of the shim bug. An unreachable round fails loudly, and
    fails the same way an unknown registry name does."""
    import controllers

    for bad in ("gen_b_44d12df4d25b437f__r99",   # lineage exists, round does not
                "no_such_lineage__r01",          # lineage does not exist
                "gen_b_44d12df4d25b437f__rXX"):  # not a round id at all
        with pytest.raises(KeyError):
            controllers.make(controllers.LAYER_PREFIX + bad)


def test_a_repair_round_is_not_listed_as_a_library_node():
    """It is a run output, not an admitted library member. `library_nodes()` is
    the safe listing and naming one is the explicit act — the same split
    `available()` already makes for `layer:`."""
    import controllers

    found = _a_repair_round()
    if found is None:
        pytest.skip("no repair round on this machine; run scripts/repair_node.py")
    node_id, _ = found

    assert controllers.NODE_PREFIX + node_id not in controllers.library_nodes()
