"""
demo/tracking.py

THE GAIT'S TRACKING ERROR, READ OFF EXP 3's PER-STEP TRACES -- and the noise
model Exp E injects into the kinematic rollout with it (prereg/expE.md).

**Tracking error, per control step** (50 Hz): the distance between where the
held Twist would have put the robot (the kinematic unicycle, integrated from
the pose at the start of the step) and where the frozen locomotion policy
actually put it,

    e_k = | p_{k+1} - (p_k + R(yaw_k) [vx, vy] dt) |        metres

which is `err_v * dt` in the trace (`demo/exp3.py`). Per episode we report the
median of e_k (mm per step) and its mean. The yaw-rate counterpart is
|err_wz| (rad/s).

**The noise model** (plan E7a). The residual between realised and commanded
velocity, in the BODY frame at the start of each step,

    r_k = R(-yaw_k) (p_{k+1} - p_k) / dt - [vx, vy]   and   wz_real - wz,

is fitted, per component, as an AR(1) process with a mean:

    n_k = mu + phi (n_{k-1} - mu) + sigma * N(0, 1)

pooled over every recorded episode of every arm (arm-neutral by
construction). The mean carries the systematic part (the gait under-shoots
commanded speed), phi the persistence (it lags, it does not jitter), sigma the
innovation. ⚠️ It is a stationary model: it does not know that the residual
depends on the command (acceleration lag) or on contact, so it is a coarse
stand-in for the physics, declared as such in prereg/expE.md.

SIM-FREE.
"""
import math
import pathlib
import statistics

from demo import exp3

COMPONENTS = ("vx", "vy", "wz")


def body_residuals(trace):
    """trace: exp3.read_trace() rows. -> list of (rvx, rvy, rwz) per step that
    has a successor, in the body frame of the start of the step."""
    out = []
    for a, b in zip(trace, trace[1:]):
        dt = b["t"] - a["t"]
        if dt <= 0:
            continue
        wx, wy = (b["x"] - a["x"]) / dt, (b["y"] - a["y"]) / dt
        c, s = math.cos(a["yaw"]), math.sin(a["yaw"])
        bvx, bvy = c * wx + s * wy, -s * wx + c * wy
        bwz = exp3.wrap(b["yaw"] - a["yaw"]) / dt
        out.append((bvx - a["vx"], bvy - a["vy"], bwz - a["wz"]))
    return out


def episode_error(trace, dt=0.02):
    """Per-episode tracking error from one trace."""
    ev = [r["err_v"] for r in trace if r["err_v"] is not None]
    ew = [abs(r["err_wz"]) for r in trace if r["err_wz"] is not None]
    return {
        "steps": len(ev),
        "disp_err_mm_med": statistics.median(ev) * dt * 1000.0,
        "disp_err_mm_mean": statistics.fmean(ev) * dt * 1000.0,
        "err_v_med": statistics.median(ev),
        "err_wz_med": statistics.median(ew),
    }


def quartiles(v):
    q = statistics.quantiles(sorted(v), n=4, method="inclusive")
    return q[0], statistics.median(v), q[2]


def fit_ar1(series_list):
    """series_list: lists of one component's residuals, one per episode.
    -> dict(mu, phi, sigma), lag-1 pairs taken WITHIN episodes only."""
    allv = [v for s in series_list for v in s]
    mu = statistics.fmean(allv)
    num = den = 0.0
    for s in series_list:
        for a, b in zip(s, s[1:]):
            num += (a - mu) * (b - mu)
            den += (a - mu) ** 2
    phi = num / den if den else 0.0
    phi = max(-0.999, min(0.999, phi))
    innov = [(b - mu) - phi * (a - mu)
             for s in series_list for a, b in zip(s, s[1:])]
    sigma = statistics.pstdev(innov) if innov else 0.0
    return {"mu": mu, "phi": phi, "sigma": sigma, "n": len(allv)}


def fit_noise_model(trace_paths):
    """-> {component: {mu, phi, sigma, n}} pooled over the given traces."""
    per = {c: [] for c in COMPONENTS}
    for p in trace_paths:
        res = body_residuals(exp3.read_trace(p))
        for i, c in enumerate(COMPONENTS):
            per[c].append([r[i] for r in res])
    return {c: fit_ar1(per[c]) for c in COMPONENTS}


class AR1Noise:
    """`rollout(noise=...)`'s sampler. Starts each component at its stationary
    distribution, so the first step is not special."""

    def __init__(self, model, rng):
        self.model, self.rng = model, rng
        self.state = []
        for c in COMPONENTS:
            m = model[c]
            sd = m["sigma"] / math.sqrt(max(1e-9, 1 - m["phi"] ** 2))
            self.state.append(m["mu"] + sd * rng.standard_normal())

    def sample(self):
        out = []
        for i, c in enumerate(COMPONENTS):
            m = self.model[c]
            self.state[i] = (m["mu"] + m["phi"] * (self.state[i] - m["mu"])
                             + m["sigma"] * self.rng.standard_normal())
            out.append(self.state[i])
        return tuple(out)
