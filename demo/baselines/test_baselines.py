"""
demo/baselines/test_baselines.py

The two hand-written reference baselines (Mindfire Exp B): are they correct,
are they held to the generated controllers' rules, do they differ only in the
planner, and are they kept out of every arm.

    python -m pytest demo/baselines/test_baselines.py -q

SIM-FREE. The two whole-patrol kinematic rollouts take ~15 s each.
"""
import heapq
import pathlib
import random
import re
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))

from demo.baselines import assemble
from demo.rollout import check_imports, load_controller, rollout

BASELINES = {"dstar_lite": HERE / "dstar_lite.py",
             "astar_replan": HERE / "astar_replan.py"}
INF = float("inf")


def namespace(name, margin=None):
    """Exec a baseline file and return its globals. `margin` shrinks the grid
    so the planner tests run on a few hundred cells."""
    ns = {"__name__": f"baseline_{name}"}
    exec(compile(BASELINES[name].read_text(), str(BASELINES[name]), "exec"), ns)
    if margin is not None:
        ns["MARGIN"] = margin
    return ns


def small_grid(ns, side=1.4):
    return ns["_Grid"]((0.0, 0.0), (side, side), 6.0)


def dijkstra(ns, grid, cost, start, goal):
    """Ground truth: the cheapest start -> goal cost on the same graph."""
    ec = ns["edge_cost"]
    dist, heap = {start: 0.0}, [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        if u == goal:
            return d
        for (v, length, ca, cb) in grid.nbrs[u]:
            nd = d + ec(cost, u, v, length, ca, cb)
            if nd < dist.get(v, INF):
                dist[v] = nd
                heapq.heappush(heap, (nd, v))
    return INF


def random_cost(rng, grid, goal, start):
    cost = np.array([rng.choice((1.0, 1.0, 1.0, 2.0, 5.0, INF))
                     for _ in range(grid.n)])
    cost[goal] = cost[start] = 1.0
    return cost


def close(a, b):
    return (a == INF and b == INF) or abs(a - b) < 1e-6


# ---- the rules every generated controller is held to


@pytest.mark.parametrize("name", sorted(BASELINES))
def test_passes_the_same_import_gate_as_the_generated_controllers(name):
    assert check_imports(BASELINES[name].read_text(), where=name) == []
    assert isinstance(load_controller(BASELINES[name]), type)


@pytest.mark.parametrize("name", sorted(BASELINES))
def test_no_wall_clock_in_control(name):
    """`time` is imported for the plan_ms statistic only. The behaviour must
    not depend on it, or machine load could move a trajectory (plan R5)."""
    src = BASELINES[name].read_text()
    uses = [ln for ln in src.splitlines() if re.search(r"\btime\.\w", ln)
            and not ln.lstrip().startswith("#")]
    assert all("perf_counter" in ln and ("t0" in ln or "plan_ms" in ln)
               for ln in uses), uses


def test_shared_block_is_identical():
    """The baselines differ ONLY in the planner."""
    want = assemble.shared_block()
    for path in assemble.TARGETS:
        assert assemble.block_of(path) == want, f"{path.name} drifted; run assemble.py"


# ---- correctness of the planners against Dijkstra


def test_astar_matches_dijkstra_on_random_grids():
    ns = namespace("astar_replan", margin=0.0)
    grid, rng = small_grid(ns), random.Random(11)
    for _ in range(150):
        goal, start = rng.randrange(grid.n), rng.randrange(grid.n)
        cost = random_cost(rng, grid, goal, start)
        path = ns["AStar"](grid, goal).plan(cost, cost.tolist(), start)
        got = ns["path_cost"](cost.tolist(), grid, path) if path else INF
        assert close(got, dijkstra(ns, grid, cost.tolist(), start, goal))


def test_dstar_lite_matches_dijkstra_under_random_changes_and_motion():
    """The incremental tree stays optimal: after every repair, g(start) and
    the extracted path both equal a from-scratch Dijkstra, over random cost
    changes and random robot motion. This is the test that found the 1-ulp
    key tie (dstar_lite.py, DStarLite.key)."""
    ns = namespace("dstar_lite", margin=0.0)
    grid, rng = small_grid(ns), random.Random(7)
    checks = 0
    for _ in range(300):
        goal, start = rng.randrange(grid.n), rng.randrange(grid.n)
        cost = random_cost(rng, grid, goal, start)
        d = ns["DStarLite"](grid, goal)
        for _step in range(8):
            path = d.plan(cost, cost.tolist(), start)
            want = dijkstra(ns, grid, cost.tolist(), start, goal)
            assert close(d.g[start], want)
            got = ns["path_cost"](cost.tolist(), grid, path) if path else INF
            assert close(got, want)
            checks += 1
            moves = [v for (v, *_r) in grid.nbrs[start] if cost[v] < INF]
            if moves:
                start = rng.choice(moves)
            cost = cost.copy()
            for _ in range(rng.randrange(1, 6)):
                c = rng.randrange(grid.n)
                if c not in (goal, start):
                    cost[c] = rng.choice((1.0, 3.0, INF))
        assert d.inits == 1
    assert checks == 2400


def corridor(ns):
    """Two routes from the left to the right of a 2.6 x 1.4 m box: a short one
    through a gap in a central wall, and a long one round its end."""
    grid = ns["_Grid"]((0.0, 0.0), (2.6, 1.4), 6.0)
    cost = np.ones(grid.n)
    wall_x = grid.nx // 2
    for iy in range(grid.ny - 2):                 # wall, open at the top 2 rows
        cost[iy * grid.nx + wall_x] = INF
    gap = [iy * grid.nx + wall_x for iy in range(3, 6)]
    for c in gap:                                 # the short route's door
        cost[c] = 1.0
    start = grid.cell(0.2, 0.4)
    goal = grid.cell(2.4, 0.4)
    return grid, cost, start, goal, gap


def test_blocked_route_dstar_lite_repairs_without_reinitialising():
    """THE BLOCKED-ROUTE REPLANNING TEST, planner level. The door on the short
    route shuts after the first plan. D* Lite must repair its existing tree --
    one initialisation, never a second -- onto the long route, at the cost a
    from-scratch search finds, and must repair for less than a fresh search."""
    ns = namespace("dstar_lite")
    grid, cost, start, goal, gap = corridor(ns)
    d = ns["DStarLite"](grid, goal)
    first = d.plan(cost, cost.tolist(), start)
    assert set(first) & set(gap), "the open door is the short route"
    fresh_expansions = d.last_expanded
    short = d.g[start]

    shut = cost.copy()
    shut[gap] = INF
    repaired = d.plan(shut, shut.tolist(), start)
    assert d.inits == 1
    assert not set(repaired) & set(gap)
    assert repaired[0] == start and repaired[-1] == goal
    want = dijkstra(ns, grid, shut.tolist(), start, goal)
    assert close(d.g[start], want) and want > short
    assert close(ns["path_cost"](shut.tolist(), grid, repaired), want)
    assert d.last_updated > 0
    assert d.last_expanded < fresh_expansions * 3   # a repair, not a rebuild

    reopened = cost.copy()
    back = d.plan(reopened, reopened.tolist(), start)
    assert close(d.g[start], short) and set(back) & set(gap)
    assert d.inits == 1


def test_blocked_route_astar_replans_from_scratch():
    ns = namespace("astar_replan")
    grid, cost, start, goal, gap = corridor(ns)
    a = ns["AStar"](grid, goal)
    first = a.plan(cost, cost.tolist(), start)
    shut = cost.copy()
    shut[gap] = INF
    second = a.plan(shut, shut.tolist(), start)
    assert set(first) & set(gap) and not set(second) & set(gap)
    assert a.searches == 2
    assert close(ns["path_cost"](shut.tolist(), grid, second),
                 dijkstra(ns, grid, shut.tolist(), start, goal))


# ---- the kinematic patrol: the door shuts before leg 2


@pytest.mark.parametrize("name", sorted(BASELINES))
def test_kinematic_patrol_completes_all_four_trips(name):
    """The controller-level blocked-route test: demo/rollout.py, rep 0, the
    exact harness the generated controllers are scored on."""
    ctrl = load_controller(BASELINES[name])()
    r = rollout(ctrl, seed=0)
    assert r["crashed"] is None and ctrl.stats["errors"] == 0
    assert r["legs_completed"] == 4, r
    assert len(ctrl.planners) == 2                    # one per goal
    if name == "dstar_lite":
        # the GOAL tree built on leg 0 is the one repaired on leg 2
        assert all(p.inits == 1 for p in ctrl.planners.values())
    else:
        assert sum(p.searches for p in ctrl.planners.values()) == \
            ctrl.stats["plans"]


# ---- kept apart from the arms


def test_baselines_are_in_no_arm():
    from demo import physics_sweep
    from demo.render import ARMS
    assert not {"dstar_lite", "astar_replan", "baselines"} & set(ARMS)
    paths = [str(p) for _a, _k, p in physics_sweep.discover(ARMS)]
    assert not any("baselines" in p for p in paths)
    for arm_file in (REPO / "demo" / "arms").rglob("*.py"):
        assert "HAND-WRITTEN REFERENCE BASELINE" not in arm_file.read_text()


def test_runner_plan_schedule_matches_the_controllers():
    """run_expB.py reads planning time off the physics trace by reproducing
    the shared block's schedule; the two constants must agree."""
    from demo.baselines import run_expB
    assert run_expB.PLAN_EVERY == namespace("dstar_lite")["PLAN_EVERY"]
    rows = [{"leg": 0}] * 25 + [{"leg": 1}] * 12
    assert run_expB.plan_step_rows(rows) == [0, 10, 20, 25, 35]
