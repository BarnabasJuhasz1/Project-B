"""
demo/test_expF.py

Mindfire Exp F's invariants: the second maze's guarantees, that selecting it
changes nothing about the first, and the sweep's design. No simulator.
"""
import json
import os
import pathlib
import subprocess
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from core.geometry import clearance
from demo import exp3, expF, maze2, mazes, rollout, world

TOY = '''
from core.observation import Command
class Controller:
    def reset(self, scenario, rng):
        pass
    def step(self, obs):
        return Command(0.4, 0.0, 0.0)
'''


def env():
    e = dict(os.environ)
    e.pop("PYTHONHOME", None)
    e.pop("PYTHONPATH", None)
    return e


# ---- the maze -------------------------------------------------------------

def test_maze2_passes_every_check():
    r = maze2.verify()
    failed = [k for k, v in r.items() if k.startswith("ok_") and not v]
    assert r["all_ok"], failed


def test_maze2_passes_the_first_mazes_checks_with_the_first_mazes_thresholds():
    """'The same solvability and detour checks as the first': world.verify()'s
    ok_* keys, recomputed on maze2's worlds."""
    r = maze2.verify()
    for key in ("ok_open_solvable", "ok_blocked_solvable", "ok_door_matters",
                "ok_start_free", "ok_goal_free", "ok_bypass_navigable"):
        assert r[key], key
    assert r["detour_cost_m"] > 3.0


def test_the_door_cuts_the_short_route_and_is_the_only_difference():
    assert set(maze2.BLOCKED_WORLD.rects) - set(maze2.OPEN_WORLD.rects) == {
        tuple(map(float, maze2.DOOR))}
    through, best, on = maze2.door_on_shortest_route()
    assert on and abs(through - best) < 0.15


def test_maze2_is_a_different_layout_with_the_same_frame():
    assert maze2.BOUNDS == world.BOUNDS
    assert (maze2.HOME, maze2.GOAL) == (world.HOME, world.GOAL)
    assert set(maze2.OPEN_WORLD.rects) != set(world.OPEN_WORLD.rects)
    assert maze2.DOOR != world.DOOR


def test_maze2_is_of_similar_size_and_detour_to_the_first():
    a, b = world.verify(), maze2.verify()
    assert abs(a["open_home_to_goal"] - b["open_home_to_goal"]) < 1.0
    assert abs(a["detour_cost_m"] - b["detour_cost_m"]) < 1.0


def test_no_exp3_spawn_puts_the_go2_body_in_a_wall():
    for rep, (_, dx, dy, _) in exp3.REP_CONDITIONS.items():
        c = float(clearance((maze2.HOME[0] + dx, maze2.HOME[1] + dy),
                            maze2.OPEN_WORLD))
        assert c > maze2.GO2_CIRCUMSCRIBED + 0.05, rep


# ---- selecting a maze changes nothing about the first --------------------

def test_default_maze_is_the_original():
    m = mazes.get()
    assert m["open"] is world.OPEN_WORLD and m["blocked"] is world.BLOCKED_WORLD
    assert m["scene"] == "scenes/demo-patrol.usd"


def test_unknown_maze_is_refused():
    with pytest.raises(KeyError):
        mazes.get("maze9")


def _toy(tmp_path):
    p = tmp_path / "toy.py"
    p.write_text(TOY)
    return rollout.load_controller(p)


def test_rollout_default_is_unchanged_and_maze2_differs(tmp_path):
    cls = _toy(tmp_path)
    kw = dict(time_limit=20.0, record=True)
    a = rollout.rollout(cls(), **kw)
    b = rollout.rollout(cls(), maze="patrol", **kw)
    c = rollout.rollout(cls(), maze="maze2", **kw)
    assert a["trace"] == b["trace"]
    assert a["trace"] != c["trace"]


def test_record_isaac_dry_run_resolves_the_maze2_scene():
    out = subprocess.run(
        [sys.executable, str(REPO / "demo" / "record_isaac.py"),
         str(REPO / "demo" / "arms" / "arm0" / "k0.py"), "--maze", "maze2",
         "--seed", "1", "--dry-run"], cwd=str(REPO), env=env(),
        capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr[-500:]
    line = [l for l in out.stdout.splitlines() if "DRY RUN" in l][-1]
    js = json.loads(line.split("DRY RUN ", 1)[1])
    assert js["maze"] == "maze2" and js["scene"] == "scenes/demo-maze2.usd"


def test_record_isaac_dry_run_default_is_still_the_patrol():
    out = subprocess.run(
        [sys.executable, str(REPO / "demo" / "record_isaac.py"),
         str(REPO / "demo" / "arms" / "arm0" / "k0.py"), "--dry-run"],
        cwd=str(REPO), env=env(), capture_output=True, text=True, timeout=120)
    js = json.loads([l for l in out.stdout.splitlines()
                     if "DRY RUN" in l][-1].split("DRY RUN ", 1)[1])
    assert js["maze"] == "patrol" and js["scene"] == "scenes/demo-patrol.usd"


# ---- the sweep design -------------------------------------------------------

def test_sweep_is_the_16_controllers_times_the_exp3_reps():
    order = expF.order()
    assert len(order) == 16 * exp3.N_REPS
    assert {(a, k) for a, k, _, _ in order} == {
        (arm, f"k{i}") for arm in ("arm0", "armB") for i in range(8)}
    assert all("base" not in str(p) for _, _, p, _ in order)


def test_sweep_is_interleaved_across_arms():
    order = expF.order()
    for rep in range(exp3.N_REPS):
        block = [a for a, _, _, r in order if r == rep]
        # no arm takes more than 3 consecutive slots in a block of 16
        run = longest = 1
        for x, y in zip(block, block[1:]):
            run = run + 1 if x == y else 1
            longest = max(longest, run)
        assert longest <= 3, (rep, block)


def test_physics_command_carries_the_maze_and_the_exp3_offsets():
    for rep, (seed, dx, dy, dyaw) in exp3.REP_CONDITIONS.items():
        cmd = expF.physics_command("x.py", rep, "t.csv.gz")
        assert cmd[cmd.index("--maze") + 1] == "maze2"
        assert cmd[cmd.index("--seed") + 1] == str(seed)
        assert float(cmd[cmd.index("--start-dx") + 1]) == dx
        assert float(cmd[cmd.index("--start-dy") + 1]) == dy
        assert float(cmd[cmd.index("--start-dyaw") + 1]) == dyaw


def test_sweep_pauses_when_the_power_profile_is_wrong(tmp_path, monkeypatch, capsys):
    monkeypatch.setattr(expF, "power_profile", lambda: "balanced")
    monkeypatch.setattr(expF, "physics_episode",
                        lambda *a, **k: pytest.fail("must not boot an episode"))
    ok = expF.run_physics(list(enumerate(expF.order())), tmp_path / "p.csv", "isaac")
    assert ok is False
    assert "PAUSED before #0" in capsys.readouterr().out


def test_report_fisher_matches_the_pilot_numbers():
    """7/8 vs 2/8 must give the pilot's p = 0.041 through arm_table()."""
    rows = []
    legs = {"arm0": [4, 4, 4, 4, 4, 4, 4, 2], "armB": [2, 2, 2, 4, 4, 2, 1, 2]}
    for arm, ls in legs.items():
        for i, l in enumerate(ls):
            for rep in range(3):
                rows.append({"arm": arm, "k": f"k{i}", "rep": rep, "legs": l})
    res, p, p0 = expF.arm_table(rows)
    assert (res["arm0"]["x"], res["armB"]["x"]) == (7, 2)
    assert round(p, 3) == 0.041 and round(p0, 3) == 0.041
