"""demo/test_tracking.py — the tracking-error measure, the noise model and the
rollout's noise hook (Exp 3 analysis, Exp E). No simulator."""
import math
import pathlib
import sys

import numpy as np
import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from demo import exp3, exp_e, rollout, tracking

TOY = '''
from core.observation import Command
class Controller:
    def reset(self, scenario, rng):
        pass
    def step(self, obs):
        return Command(0.4, 0.0, 0.2)
'''


def _toy():
    ns = {}
    exec(TOY, ns)
    return ns["Controller"]()


def _trace(poses, cmds, dt=0.02):
    rows = []
    errs = exp3.tracking_errors(poses, cmds, dt)
    for i, ((x, y, yaw), (vx, vy, wz), (ev, ew)) in enumerate(
            zip(poses, cmds, errs)):
        rows.append({"t": i * dt, "x": x, "y": y, "yaw": yaw, "vx": vx,
                     "vy": vy, "wz": wz, "err_v": ev, "err_wz": ew})
    return rows


def test_a_slow_gait_shows_up_as_a_negative_forward_residual():
    dt, poses, cmds = 0.02, [(0.0, 0.0, 0.7)], []
    for _ in range(100):
        x, y, yaw = poses[-1]
        cmds.append((0.5, 0.0, 0.0))
        poses.append((x + 0.4 * math.cos(yaw) * dt,
                      y + 0.4 * math.sin(yaw) * dt, yaw))   # 80 % of commanded
    t = _trace(poses, cmds)
    res = tracking.body_residuals(t)
    assert all(r[0] == pytest.approx(-0.1) and abs(r[1]) < 1e-9 for r in res)
    e = tracking.episode_error(t[:-1] + [dict(t[-1], err_v=None, err_wz=None)])
    assert e["disp_err_mm_med"] == pytest.approx(0.1 * dt * 1000)   # 2 mm/step


def test_ar1_fit_recovers_known_parameters():
    rng = np.random.default_rng(0)
    series = []
    for _ in range(20):
        v, s = 0.0, []
        for _ in range(2000):
            v = 0.8 * v + 0.05 * rng.standard_normal()
            s.append(-0.1 + v)
        series.append(s)
    m = tracking.fit_ar1(series)
    assert m["mu"] == pytest.approx(-0.1, abs=0.01)
    assert m["phi"] == pytest.approx(0.8, abs=0.02)
    assert m["sigma"] == pytest.approx(0.05, abs=0.003)


def test_noise_sampler_is_seeded_and_paired():
    model = {c: {"mu": 0.0, "phi": 0.5, "sigma": 0.1} for c in tracking.COMPONENTS}
    a = tracking.AR1Noise(model, np.random.default_rng(5000))
    b = tracking.AR1Noise(model, np.random.default_rng(5000))
    assert [a.sample() for _ in range(10)] == [b.sample() for _ in range(10)]


def test_rollout_without_noise_is_unchanged_and_noise_moves_it():
    plain = rollout.rollout(_toy(), record=True, time_limit=2.0)
    none = rollout.rollout(_toy(), record=True, time_limit=2.0, noise=None)
    assert plain["trace"] == none["trace"]
    model = {c: {"mu": -0.05, "phi": 0.9, "sigma": 0.02}
             for c in tracking.COMPONENTS}
    noisy = rollout.rollout(_toy(), record=True, time_limit=2.0,
                            noise=tracking.AR1Noise(model,
                                                    np.random.default_rng(1)))
    assert noisy["trace"][-1][1:4] != plain["trace"][-1][1:4]
    # the controller is still told what IT commanded, never the noisy Twist
    assert {row[5] for row in noisy["trace"]} == {0.4}


def test_exp_e_design_is_the_pilot_and_the_exp3_reps():
    ctrls = exp_e.controllers(base=False)
    assert len(ctrls) == 16
    assert {a for a, _, _ in ctrls} == {"arm0", "armB"}
    assert all(p.is_file() for _, _, p in ctrls)
    assert exp3.N_REPS == 3 and exp_e.NOISE_SEED == 5000
