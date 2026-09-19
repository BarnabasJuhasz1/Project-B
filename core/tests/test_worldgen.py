"""
core/tests/test_worldgen.py

Chunk 45. What a generated world has to be true of, asserted rather than
eyeballed.

The properties that matter are not "does it look like a room" but: is it the
SAME room for the same seed, is it solvable, can the robot physically fit
through it, and do the tiers differ in the way they claim to. The last one is
the interesting test — it is the one that failed first and produced the `detour`
tier parameter.
"""

import math

import numpy as np
import pytest

from core.geometry import (ROBOT_RADIUS, clearance, geodesic_distance,
                           occupancy_grid)
from core.worldgen import (ENDPOINT_CLEARANCE, MIN_EUCLIDEAN, ROOM, TIERS,
                           GeneratedWorld, batch, generate, measure,
                           tier_report, WorldgenError)

SEEDS = range(400, 412)
REPO = __import__('pathlib').Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("tier", sorted(TIERS))
def test_same_seed_same_world(tier):
    """Reproducible from (tier, seed) alone — the whole basis for a procedural
    suite being auditable."""
    a, b = generate(tier, 99), generate(tier, 99)
    assert a.world.rects == b.world.rects
    assert a.start == b.start and a.goal == b.goal
    assert a.world.name == b.world.name


@pytest.mark.parametrize("tier", sorted(TIERS))
def test_same_seed_same_world_ACROSS_PROCESSES(tier):
    """⚠️ THE TEST THAT WAS MISSING, AND THE BUG IT WOULD HAVE CAUGHT.

    `test_same_seed_same_world` above compares two calls in ONE process and
    passed throughout, while `generate(tier, seed)` was returning a DIFFERENT
    WORLD IN EVERY PROCESS — 21, 24 and 31 rects on three consecutive runs of
    the same call. The cause was `abs(hash(tier))` in the seed: Python
    randomises str hashing per process unless PYTHONHASHSEED is fixed.

    It silently invalidated the module's central claim, chunk 47's base
    calibration and a whole growth-round measurement. A same-process test cannot
    see it, so this one spawns real interpreters.
    """
    import subprocess
    import sys as _sys
    code = ("from core.worldgen import generate;"
            f"g=generate({tier!r}, 99);"
            "print(len(g.world.rects), g.start, g.goal)")
    outs = {subprocess.run([_sys.executable, "-c", code], cwd=str(REPO),
                           capture_output=True, text=True, check=True).stdout
            for _ in range(3)}
    assert len(outs) == 1, (
        f"{tier} is not reproducible across processes: {outs}")


@pytest.mark.parametrize("tier", sorted(TIERS))
def test_different_seeds_differ(tier):
    """A distribution, not a constant dressed up as one."""
    worlds = {generate(tier, s).world.rects for s in SEEDS}
    assert len(worlds) >= len(list(SEEDS)) - 1


@pytest.mark.parametrize("tier", sorted(TIERS))
def test_solvable_by_the_same_check_scenario_validation_uses(tier):
    """`core.scenario.validate` proves solvability with the geodesic field. A
    world that passes here must pass there, so this uses the same function
    rather than trusting worldgen's own component labelling."""
    for seed in SEEDS:
        g = generate(tier, seed)
        d = geodesic_distance(g.start, g.goal, world=g.world)
        assert math.isfinite(d), f"{tier}/{seed} generated an unsolvable world"


@pytest.mark.parametrize("tier", sorted(TIERS))
def test_endpoints_are_clear_and_far_apart(tier):
    for seed in SEEDS:
        g = generate(tier, seed)
        for p in (g.start, g.goal):
            assert float(clearance(np.array(p), g.world)) >= ENDPOINT_CLEARANCE
        assert math.dist(g.start, g.goal) >= MIN_EUCLIDEAN


@pytest.mark.parametrize("tier", sorted(TIERS))
def test_route_admits_the_robot(tier):
    """The bottleneck is the largest body that still fits. Below ROBOT_RADIUS
    the route does not exist for THIS robot, and the world should never have
    been returned."""
    for seed in SEEDS:
        g = generate(tier, seed)
        assert g.difficulty.min_corridor >= ROBOT_RADIUS - 1e-9


@pytest.mark.parametrize("tier", sorted(TIERS))
def test_geometry_stays_inside_the_room(tier):
    x0, y0, x1, y1 = ROOM
    for seed in SEEDS:
        g = generate(tier, seed)
        for (rx0, ry0, rx1, ry1) in g.world.rects:
            assert rx0 >= x0 - 1e-9 and ry0 >= y0 - 1e-9
            assert rx1 <= x1 + 1e-9 and ry1 <= y1 + 1e-9
            assert rx1 > rx0 and ry1 > ry0


def test_unknown_tier_raises():
    with pytest.raises(KeyError):
        generate("impossible", 1)


def test_tiers_are_ordered_by_measured_difficulty():
    """⚠️ THE TEST THAT FAILED TWICE, AND THE SECOND FAILURE WAS REAL.

    A tier is generation parameters, not a label, so "harder" has to be a
    measured fact. This assertion has falsified the tiers twice:

    1. The first TIERS scored tortuosity 1.30+-0.25 for `cluttered` and
       1.28+-0.19 for `maze` over 25 seeds — indistinguishable, because more
       partitions make rooms DENSER, not routes LONGER. The `detour` tier
       parameter was added in response and separated them at n=25.
    2. It failed again at n=12, and tuning `detour` over 2/3/4 moved
       `cluttered` only from 1.73+-0.40 to 1.87+-0.44 against `maze`'s
       2.16+-0.45 (n=25 each). The distributions genuinely overlap —
       cluttered reaches 2.90, maze descends to 1.23.

    **So tortuosity is NOT asserted between `cluttered` and `maze`.** It is a
    real difference in the means and not a usable ordering per-world. What the
    tiers do separate, strictly and with small variance, is DENSITY and
    TIGHTNESS. That is the honest claim and it is what is pinned here.
    """
    stats = {}
    for tier in ("open", "cluttered", "maze"):
        ws = batch(tier, range(600, 612))
        stats[tier] = dict(
            tort=float(np.mean([w.difficulty.tortuosity for w in ws])),
            free=float(np.mean([w.difficulty.free_fraction for w in ws])),
            corridor=float(np.mean([w.difficulty.min_corridor for w in ws])),
            obstacles=float(np.mean([w.difficulty.n_obstacles for w in ws])),
        )
    o, c, m = stats["open"], stats["cluttered"], stats["maze"]

    # These three order strictly, and are what the tiers actually mean.
    assert o["free"] > c["free"] > m["free"], stats
    assert o["corridor"] > c["corridor"] > m["corridor"], stats
    assert o["obstacles"] < c["obstacles"] < m["obstacles"], stats

    # Tortuosity separates `open` from both, and nothing finer than that.
    assert o["tort"] < c["tort"], stats
    assert o["tort"] < m["tort"], stats


# ---------------------------------------------------------------------------
# chunk 53 — a generated world resolves as a scenario, BY NAME ONLY


def test_load_resolves_a_generated_scenario_id():
    from core.scenario import load

    scenario = load("gen-maze-02004")
    assert scenario.id == "gen-maze-02004"
    assert scenario.seed == 2004
    assert scenario.scene == "scenes/generated/maze-02004.usd"
    # validate() ran: it is what computes the SPL denominator, and a run whose
    # denominator was never computed cannot be scored.
    assert math.isfinite(scenario.optimal_distance())


def test_a_generated_scenario_is_never_enumerated():
    """⚠️ THE LOAD-BEARING ONE. A generated world is in no split, carries no
    sealed seed and appears in no PRE-REGISTRATION constant, so a runner that
    iterates "everything" must never evaluate against one. Same rule the sealed
    split follows.
    """
    from core.scenario import all_scenarios

    for split in (None, "dev", "held_out"):
        assert not any(s.id.startswith("gen-")
                       for s in all_scenarios(split))


def test_an_unknown_generated_tier_is_refused():
    from core.scenario import load

    with pytest.raises(KeyError, match="unknown tier"):
        load("gen-nosuchtier-00001")


def test_a_non_generated_id_still_raises_the_original_error():
    from core.scenario import load

    with pytest.raises(KeyError, match="unknown scenario"):
        load("not-a-scenario-at-all")


def test_runner_threads_the_scenario_world_into_geometry():
    """⚠️ REGRESSION GUARD, chunk 53. `core.geometry`'s `world` argument
    defaults to LAB42, so a call site that omits it measures a DIFFERENT ROOM
    than the robot is in. `scripts/run_episode.py:observe` omitted it on both
    `clearance` and `sensor.scan` until 2026-09-15; the defect was unreachable
    until a generated world could be run under physics, and then it silently
    produced a scan of lab42 while the robot walked a maze.

    Checked by AST rather than by running the simulator: the property is "this
    call passes the world", which is syntactic, and a physics run to assert it
    would cost half an hour.
    """
    import ast
    import pathlib

    src = (pathlib.Path(__file__).resolve().parents[2]
           / "scripts" / "run_episode.py").read_text(encoding="utf-8")
    tree = ast.parse(src)

    observe = next(n for n in ast.walk(tree)
                   if isinstance(n, ast.FunctionDef) and n.name == "observe")
    calls = {}
    for node in ast.walk(observe):
        if not isinstance(node, ast.Call):
            continue
        name = getattr(node.func, "attr", getattr(node.func, "id", None))
        if name in ("clearance", "scan"):
            calls.setdefault(name, []).append(node)

    assert "clearance" in calls and "scan" in calls, (
        "observe() no longer calls clearance and scan — this guard is stale "
        "and must be rewritten, not deleted")

    for name, nodes in calls.items():
        for node in nodes:
            passed = [getattr(a, "id", None) for a in node.args]
            passed += [getattr(k.value, "id", None) for k in node.keywords]
            assert "world" in passed, (
                f"run_episode.observe() calls {name}() without the scenario's "
                f"world. core.geometry defaults to LAB42, so this measures the "
                f"wrong room on any generated world.")
