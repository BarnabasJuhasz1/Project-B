"""
demo/baselines/b2/test_b2.py

Exp B2 (prereg/expB2.md): the B2 baselines differ from Exp B's ONLY in the
shared follower, are still held to the generated controllers' rules, still
complete the kinematic patrol, never command a full-rate turn in place, and
un-stick a gait that stops turning.

    python -m pytest demo/baselines/b2/test_b2.py -q

SIM-FREE. The two kinematic rollouts take ~15 s each.
"""
import ast
import math
import pathlib
import re
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent.parent.parent
sys.path.insert(0, str(REPO))

from core.geometry import raycast
from core.observation import Observation
from demo.baselines import assemble
from demo.baselines.b2 import build
from demo.rollout import PatrolScenario, check_imports, load_controller, rollout
from demo.world import GOAL, HOME, OPEN_WORLD

NAMES = ("dstar_lite", "astar_replan")


def b1(name):
    return HERE.parent / f"{name}.py"


def b2(name):
    return HERE / f"{name}.py"


def top_level(source):
    """name -> ast dump of every top-level def/class (methods for _Base)."""
    out = {}
    for node in ast.parse(source).body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)):
            out[node.name] = ast.dump(node)
            if node.name == "_Base":
                for m in node.body:
                    if isinstance(m, ast.FunctionDef):
                        out[f"_Base.{m.name}"] = ast.dump(m)
    return out


def test_built_files_are_up_to_date():
    assert build.main(["--check"]) == 0


@pytest.mark.parametrize("name", NAMES)
def test_planner_is_byte_identical_to_exp_b(name):
    after = lambda p: p.read_text().split("# ==== SHARED END", 1)[1]
    assert after(b2(name)) == after(b1(name))


def test_only_the_follower_changed():
    one = top_level(assemble.block_of(b1("dstar_lite")))
    two = top_level(assemble.block_of(b2("dstar_lite")))
    for unchanged in ("_Grid", "edge_cost", "path_cost", "_wrap",
                      "_Base._plan", "_Base._lookahead", "_Base.step",
                      "_Base._reset_planner"):
        assert one[unchanged] == two[unchanged], unchanged
    assert one["_Base._step"] != two["_Base._step"]
    assert set(two) - set(one) == {"_Base._turned", "_Base._stalled"}


@pytest.mark.parametrize("name", NAMES)
def test_same_rules_as_the_generated_controllers(name):
    src = b2(name).read_text()
    assert check_imports(src, where=name) == []
    uses = [ln for ln in src.splitlines() if re.search(r"\btime\.\w", ln)
            and not ln.lstrip().startswith("#")]
    assert all("perf_counter" in ln for ln in uses), uses


@pytest.mark.parametrize("name", NAMES)
def test_kinematic_patrol_and_no_full_rate_spin(name):
    ctrl = load_controller(b2(name))()
    sent = []
    step = ctrl.step
    ctrl.step = lambda obs: sent.append(step(obs)) or sent[-1]
    r = rollout(ctrl, seed=0)
    assert r["legs_completed"] == 4 and ctrl.stats["errors"] == 0, r
    assert ctrl.stats["unsticks"] == 0          # a unicycle never stalls
    assert max(abs(c.wz) for c in sent) <= 0.8 + 1e-9
    if name == "dstar_lite":
        assert all(p.inits == 1 for p in ctrl.planners.values())


@pytest.mark.parametrize("name", NAMES)
def test_watchdog_unsticks_a_gait_that_will_not_turn(name):
    """A frozen pose with the goal behind the robot: the follower asks for a
    turn, the pose never changes, and within ~0.5 s the watchdog must switch
    to a straight unstick command instead of repeating the turn forever (the
    Exp B failure)."""
    ctrl = load_controller(b2(name))()
    scen = PatrolScenario(seed=0)
    angles = scen.beam_angles()
    ctrl.reset(scen, scen.rng())
    yaw = math.atan2(GOAL[1] - HOME[1], GOAL[0] - HOME[0]) + math.pi
    cmds = []
    for i in range(60):
        obs = Observation(t=i * 0.02,
                          ranges=raycast(HOME, yaw, angles, 6.0, OPEN_WORLD),
                          range_angles=angles, range_max=6.0,
                          pose_estimate=(HOME[0], HOME[1], yaw),
                          pose_error_estimate=0.0, goal=GOAL, contact=False)
        cmds.append(ctrl.step(obs))
    assert abs(cmds[0].wz) > 0.5
    first = next(i for i, c in enumerate(cmds) if abs(c.wz) < 1e-9)
    assert 20 <= first <= 30, first
    assert cmds[first].vx != 0.0 and ctrl.stats["unsticks"] >= 1


def test_runner_knows_b2_and_checks_ac():
    from demo.baselines import run_expB
    files, prereg, out = run_expB.VARIANTS["b2"]
    assert [p for _n, p in files] == [b2("dstar_lite"), b2("astar_replan")]
    assert prereg == "prereg/expB2.md" and out.name == "expB2"
    assert run_expB.VARIANTS["b1"][1] == "prereg/expB.md"
    assert isinstance(run_expB.ac_online(), bool)
