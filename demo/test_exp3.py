"""demo/test_exp3.py — Exp 3's flags and design, with NO simulator.

`python -m pytest demo/test_exp3.py -q`. record_isaac.py is exercised through
its `--dry-run`, which exits before SimulationApp is constructed.
"""
import csv
import gzip
import json
import math
import os
import pathlib
import subprocess
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from core.geometry import ROBOT_RADIUS, clearance
from demo import exp3, physics_sweep, rollout
from demo.world import BLOCKED_WORLD, HOME, OPEN_WORLD

GO2_CIRCUMSCRIBED = 0.383     # 0.70 x 0.31 m body, CLAUDE.md
TOY = '''
from core.observation import Command
class Controller:
    def reset(self, scenario, rng):
        self.seed = scenario.seed
    def step(self, obs):
        return Command(0.3, 0.0, 0.1)
'''


def env():
    e = dict(os.environ)
    e.pop("PYTHONHOME", None)
    e.pop("PYTHONPATH", None)
    return e


# ---- the design ---------------------------------------------------------

def test_rep0_is_the_original_condition():
    assert exp3.rep_condition(0) == (0, 0.0, 0.0, 0.0)


def test_committed_offsets_are_the_seeded_draw():
    drawn = exp3.draw_offsets()
    for rep in range(1, exp3.N_REPS):
        assert exp3.REP_CONDITIONS[rep] == drawn[rep], rep


def test_offsets_are_within_the_declared_bounds_and_seeds_differ():
    seeds = [exp3.rep_condition(r)[0] for r in range(exp3.N_REPS)]
    assert seeds == list(range(exp3.N_REPS))
    for rep in range(1, exp3.N_REPS):
        _, dx, dy, dyaw = exp3.rep_condition(rep)
        assert abs(dx) <= exp3.OFFSET_XY and abs(dy) <= exp3.OFFSET_XY
        assert abs(dyaw) <= exp3.OFFSET_YAW
        assert (dx, dy, dyaw) != (0.0, 0.0, 0.0)


def test_every_start_pose_clears_the_real_go2_footprint():
    """Plan §3.3: offsets this small keep the robot clear of walls at HOME --
    checked against the Go2's circumscribed radius, not the 0.30 m disc."""
    for rep in range(exp3.N_REPS):
        _, dx, dy, _ = exp3.rep_condition(rep)
        p = (HOME[0] + dx, HOME[1] + dy)
        for w in (OPEN_WORLD, BLOCKED_WORLD):
            assert clearance(p, w) > GO2_CIRCUMSCRIBED + 0.2, (rep, p)


def test_unknown_rep_is_refused():
    with pytest.raises(KeyError):
        exp3.rep_condition(3)


def test_success_is_all_four_legs():
    assert exp3.LEGS_FOR_SUCCESS == len(rollout.LEG_TARGETS) == 4
    assert exp3.is_success(4) and exp3.is_success("4")
    assert not exp3.is_success(3) and not exp3.is_success(None)


# ---- the order ----------------------------------------------------------

def _ctrls():
    return ([("arm0", f"k{i}", None) for i in range(8)]
            + [("armB", f"k{i}", None) for i in range(8)]
            + [("(base)", "reference", None)])


def test_order_covers_every_episode_once_and_is_deterministic():
    a = exp3.job_order(_ctrls())
    b = exp3.job_order(_ctrls())
    assert a == b
    keys = [(arm, k, rep) for arm, k, _, rep in a]
    assert len(keys) == len(set(keys)) == 17 * 3


def test_order_is_rep_major_and_interleaves_arms():
    order = exp3.job_order(_ctrls())
    reps = [rep for *_, rep in order]
    assert reps == sorted(reps), "rep 0 must all run first"
    for r in range(3):
        block = [arm for arm, _, _, rep in order if rep == r]
        runs = max(len(list(g)) for _, g in __import__("itertools").groupby(block))
        assert runs <= 3, f"rep {r}: {runs} of one arm in a row"
        first_half = block[:len(block) // 2]
        assert {"arm0", "armB"} <= set(first_half)
    for i in range(1, len(order)):
        assert order[i][:2] != order[i - 1][:2], "a controller back to back"


# ---- the trace ----------------------------------------------------------

def test_tracking_error_is_zero_for_a_perfect_unicycle():
    dt, poses, cmds = 0.02, [(1.0, 1.0, 0.6)], []
    for _ in range(50):
        x, y, yaw = poses[-1]
        vx, wz = 0.5, 0.3
        cmds.append((vx, 0.0, wz))
        poses.append((x + vx * math.cos(yaw) * dt, y + vx * math.sin(yaw) * dt,
                      exp3.wrap(yaw + wz * dt)))
    errs = exp3.tracking_errors(poses, cmds, dt)
    assert all(abs(ev) < 1e-9 and abs(ew) < 1e-9 for ev, ew in errs)


def test_tracking_error_measures_a_stalled_robot():
    errs = exp3.tracking_errors([(0, 0, 0), (0, 0, 0)], [(0.5, 0.0, 0.2)], 0.02)
    assert errs[0][0] == pytest.approx(0.5) and errs[0][1] == pytest.approx(-0.2)
    # the last command has no next pose
    assert exp3.tracking_errors([(0, 0, 0)], [(0.5, 0, 0)], 0.02) == [(None, None)]


def test_trace_roundtrip_is_compact_and_reproducible(tmp_path):
    rows = [[i * 0.02, 0, 0, 1.1 + i * 0.01, 1.1, 0.6, 0.5, 0.0, 0.1, 0.65,
             0, 0.42, 0.01, -0.002] for i in range(9000)]
    rows[-1][12:] = [None, None]
    rows[100][10] = rows[101][10] = 1          # one contact, two steps
    p1, p2 = tmp_path / "a.csv.gz", tmp_path / "b.csv.gz"
    exp3.write_trace(p1, rows)
    exp3.write_trace(p2, rows)
    assert p1.read_bytes() == p2.read_bytes(), "gzip mtime must be pinned"
    assert p1.stat().st_size < 250_000, "a 180 s episode must stay small"
    back = exp3.read_trace(p1)
    assert list(back[0]) == list(exp3.TRACE_COLUMNS)
    assert len(back) == 9000 and back[-1]["err_v"] is None
    s = exp3.summarise(rows, 0.02)
    assert s["contacts"] == 1 and s["contact_steps"] == 2
    assert s["path_m"] == pytest.approx(89.99, abs=0.01)
    assert s["track_v_med"] == pytest.approx(0.01)


# ---- parsing and the rep 0 check ------------------------------------------

def test_result_json_is_preferred_and_result_line_is_the_fallback():
    js = {"legs": 4, "total": 4, "success": True, "leg_times": [1.0, 2.0],
          "sim_seconds": 99.0, "raised": None, "contacts": 2}
    out = ("[record] RESULT k0.py: legs 4/4  times [1.0, 2.0]  t=99.0s\n"
           + exp3.RESULT_JSON_PREFIX + json.dumps(js) + "\n")
    row = physics_sweep.row_from_output(out, "", 200.0, 1.5)
    assert row["legs"] == 4 and row["success"] and row["contacts"] == 2
    assert json.loads(row["leg_times"]) == [1.0, 2.0]
    old = physics_sweep.row_from_output(out.splitlines()[0], "", 200.0, 1.5)
    assert old["legs"] == 4 and old["success"] and "RESULT line only" in old["note"]
    two = "[record] RESULT k7.py: legs 2/4  times [43.84, 22.06]  t=180.0s"
    assert physics_sweep.row_from_output(two, "", 1, 0)["success"] is False
    dead = physics_sweep.row_from_output("boom", "", 5.0, 0.1)
    assert dead["legs"] is None and dead["note"].startswith("NO RESULT LINE")


def test_compare_rep0_against_the_pilot_csv_format():
    ref = [{"arm": "arm0", "k": "k7", "legs": "2", "leg_times": "[43.84, 22.06]",
            "sim_seconds": "180.0"},
           {"arm": "armB", "k": "k3", "legs": "4",
            "leg_times": "[20.2, 25.1, 71.4, 30.24]", "sim_seconds": "147.0"}]
    rows = [{"arm": "arm0", "k": "k7", "rep": "0", "legs": "2",
             "leg_times": "[43.84, 22.06]", "sim_seconds": "180.0"},
            {"arm": "armB", "k": "k3", "rep": "0", "legs": "4",
             "leg_times": "[20.2, 25.1, 71.4, 30.26]", "sim_seconds": "147.0"},
            {"arm": "armB", "k": "k3", "rep": "1", "legs": "1",
             "leg_times": "[1.0]", "sim_seconds": "180.0"},
            {"arm": "armX", "k": "k0", "rep": "0", "legs": "4",
             "leg_times": "[1, 2, 3, 4]", "sim_seconds": "10"}]
    got = {(a, k): v for a, k, v, _ in exp3.compare_rep0(rows, ref)}
    assert got == {("arm0", "k7"): "MATCH", ("armB", "k3"): "DIFFER",
                   ("armX", "k0"): "MISSING"}


# ---- the sweep's CLI -----------------------------------------------------

def test_sweep_refuses_to_overwrite_the_pilot_result():
    with pytest.raises(SystemExit, match="refusing"):
        physics_sweep.main(["--out", str(physics_sweep.PILOT_REFERENCE),
                            "--dry-run"])


def test_sweep_dry_run_plans_the_pilot_and_resumes(tmp_path, capsys):
    out = tmp_path / "v2" / "physics.csv"
    ks = ",".join(f"k{i}" for i in range(8))
    physics_sweep.main(["--out", str(out), "--arms", "arm0,armB", "--ks", ks,
                        "--dry-run"])
    assert "51 episodes planned, 0 already" in capsys.readouterr().out
    assert not out.exists(), "a dry run writes nothing"
    out.parent.mkdir(parents=True)
    physics_sweep.write_rows(out, [
        {"order": 0, "arm": "arm0", "k": "k0", "rep": 0, "legs": 4},
        {"order": 1, "arm": "armB", "k": "k1", "rep": 0, "legs": ""}])
    physics_sweep.main(["--out", str(out), "--arms", "arm0,armB", "--ks", ks,
                        "--dry-run"])
    assert "2 already" in capsys.readouterr().out
    physics_sweep.main(["--out", str(out), "--arms", "arm0,armB", "--ks", ks,
                        "--dry-run", "--retry-failed"])
    assert "1 already" in capsys.readouterr().out, "a sim failure is re-runnable"


def test_sweep_command_carries_the_rep_condition(tmp_path):
    cmd = physics_sweep.command(pathlib.Path("c.py"), 1, tmp_path / "t.csv.gz")
    seed, dx, dy, dyaw = exp3.rep_condition(1)
    i = cmd.index
    assert cmd[i("--seed") + 1] == "1"
    assert float(cmd[i("--start-dx") + 1]) == dx
    assert float(cmd[i("--start-dy") + 1]) == dy
    assert float(cmd[i("--start-dyaw") + 1]) == dyaw
    assert "--no-record" in cmd and cmd[i("--trace") + 1].endswith(".csv.gz")


# ---- record_isaac.py, through --dry-run (exits before SimulationApp) ------

def _record(*args):
    return subprocess.run([sys.executable, str(REPO / "demo" / "record_isaac.py"),
                           *args], cwd=str(REPO), capture_output=True, text=True,
                          env=env(), timeout=120)


def test_record_dry_run_applies_seed_and_offsets(tmp_path):
    ctrl = tmp_path / "toy.py"
    ctrl.write_text(TOY)
    p = _record(str(ctrl), "--dry-run", "--seed", "2", "--start-dx", "0.05",
                "--start-dy", "-0.05", "--start-dyaw", "0.1",
                "--trace", str(tmp_path / "t.csv.gz"))
    assert p.returncode == 0, p.stderr
    js = json.loads(p.stdout.split("DRY RUN ", 1)[1])
    assert js["seed"] == 2
    assert js["spawn"] == [round(HOME[0] + 0.05, 4), round(HOME[1] - 0.05, 4),
                           round(0.6 + 0.1, 4)]


def test_record_dry_run_defaults_are_the_original_spawn(tmp_path):
    ctrl = tmp_path / "toy.py"
    ctrl.write_text(TOY)
    p = _record(str(ctrl), "--dry-run")
    js = json.loads(p.stdout.split("DRY RUN ", 1)[1])
    assert js["seed"] == 0 and js["spawn"] == [HOME[0], HOME[1], 0.6]


@pytest.mark.parametrize("bad", [["--start-dx", "0.9"], ["--start-dyaw", "4"],
                                 ["--start-dy", "nan"], ["--trace", "x.txt"],
                                 ["--seed", "one"]])
def test_record_rejects_bad_flags_before_booting(tmp_path, bad):
    ctrl = tmp_path / "toy.py"
    ctrl.write_text(TOY)
    p = _record(str(ctrl), "--dry-run", *bad)
    assert p.returncode == 2, (bad, p.stdout, p.stderr)


def test_record_dry_run_still_applies_the_import_gate(tmp_path):
    ctrl = tmp_path / "cheat.py"
    ctrl.write_text("from core.geometry import clearance\n" + TOY)
    p = _record(str(ctrl), "--dry-run")
    assert p.returncode != 0 and "banned import" in p.stderr


# ---- the kinematic mirror -------------------------------------------------

def test_kinematic_rollout_takes_the_same_offsets():
    ns = {}
    exec(TOY, ns)
    a = rollout.rollout(ns["Controller"](), record=True, time_limit=0.1)
    b = rollout.rollout(ns["Controller"](), record=True, time_limit=0.1,
                        start_offset=(0.0, 0.0, 0.0))
    assert a["trace"] == b["trace"], "the default must be the original start"
    _, dx, dy, dyaw = exp3.rep_condition(1)
    c = rollout.rollout(ns["Controller"](), record=True, time_limit=0.1,
                        start_offset=(dx, dy, dyaw))
    t0 = c["trace"][0]
    assert t0[1:4] == (HOME[0] + dx, HOME[1] + dy, 0.6 + dyaw)


def test_sweep_pauses_when_the_power_profile_is_wrong(tmp_path, capsys, monkeypatch):
    """The 2026-11-02 reboot left the laptop in low-power and halved the speed
    of every episode. --require-profile stops at an episode boundary instead."""
    monkeypatch.setattr(physics_sweep, "power_profile", lambda: "low-power")
    monkeypatch.setattr(physics_sweep, "run_one",
                        lambda *a: pytest.fail("must not boot an episode"))
    physics_sweep.main(["--out", str(tmp_path / "v2" / "physics.csv"),
                        "--arms", "arm0", "--ks", "k0", "--no-base",
                        "--only-rep", "0", "--require-profile", "performance"])
    assert "PAUSED before #0" in capsys.readouterr().out
