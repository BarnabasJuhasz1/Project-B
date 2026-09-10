"""Dynamic-window local planner leaf (Fox, Burgard & Thrun 1997)."""

import math

import numpy as np

from core.observation import Command
from harness.micro import MicroScenario, room
from nodes.contract import Action, Direction, Port, PortType, Status

# ---------------------------------------------------------------------------
# Tuning constants. NONE of these are recalled from the paper; all are my own
# engineering judgement for a 0.3 m-radius quadruped driven at ~20 Hz.
# ---------------------------------------------------------------------------
_V_MAX = 0.8            # runner saturates here anyway
_W_MAX = 1.0
_A_V = 1.5              # m/s^2, assumed translational accel of the frozen policy
_A_W = 4.0              # rad/s^2, assumed rotational accel
_BRAKE_V = 1.5          # decel used in the admissibility test; == _A_V, symmetric
_BRAKE_W = 4.0

_ROBOT_RADIUS = 0.3
_SAFETY_MARGIN = 0.12   # inflation on top of the body radius
_RELAXED_MARGIN = 0.02  # used only by the second pass, see _select()

_HORIZON = 2.0          # s of forward prediction per candidate arc
_SIM_DT = 0.25          # s per prediction step -> 8 steps
_N_V = 7                # samples across the translational window
_N_W = 15               # samples across the rotational window

# Clearance beyond this is "as good as free", so all unobstructed arcs tie on
# the dist term and the heading term breaks the tie. Without a cap the planner
# drifts to the middle of the room instead of going to the goal.
_DIST_CAP = 2.0

_W_HEADING = 0.35
_W_DIST = 0.45
_W_VEL = 0.20

_GOAL_TOL = 0.30
_BLOCKED_TIMEOUT = 3.0  # s with an empty admissible set before giving up
_DT_DEFAULT = 0.05
_DT_MIN, _DT_MAX = 0.02, 0.20


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _rollout(v, w, steps):
    """Positions and headings along the circular arc of each (v, w) candidate.

    v, w: (C,) candidate velocities. steps: (T,) prediction times.
    Returns x, y, theta each (C, T) in the current robot frame.
    """
    th = w[:, None] * steps[None, :]
    straight = np.abs(w) < 1e-4
    w_safe = np.where(straight, 1e-4, w)[:, None]
    radius = v[:, None] / w_safe
    x = np.where(straight[:, None], v[:, None] * steps[None, :], radius * np.sin(th))
    y = np.where(straight[:, None], 0.0, radius * (1.0 - np.cos(th)))
    return x, y, th


def _smooth3(grid):
    """3x3 box filter with edge replication.

    This is the paper's sigma: the objective is low-pass filtered over the
    velocity grid so that a command whose neighbours are unsafe is itself
    discounted, which is what buys tolerance to error in the world model.
    """
    p = np.pad(grid, 1, mode="edge")
    acc = np.zeros_like(grid)
    for i in range(3):
        for j in range(3):
            acc += p[i:i + grid.shape[0], j:j + grid.shape[1]]
    return acc / 9.0


class DynamicWindowDrive(Action):
    """Drive toward a disclosed goal by dynamic-window search in velocity space.

    Implements Fox/Burgard/Thrun (1997). Each tick the node samples the pairs
    (v, w) reachable from the current command within one control interval given
    the acceleration limits (the dynamic window), predicts the circular arc each
    one produces, and discards any from which the robot could not brake to a
    stop before the nearest obstacle on that arc (admissibility). Survivors are
    scored by a smoothed weighted sum of goal alignment, clearance and speed,
    and the maximum is emitted as the Twist.

    Why it works on this task family: the obstruction is usually not visible
    until the robot is close to it, so a planner that commits to a route is
    wrong; a planner that re-searches the whole reachable velocity set every
    tick from the live scan is not. Admissibility is what keeps the closed-loop
    behaviour safe rather than merely reactive -- the robot never accepts a
    speed it cannot undo before contact.
    """

    PORTS = (
        Port("dwa_confidence", PortType.CONFIDENCE, Direction.OUT),
        Port("dwa_blocked", PortType.FLAG, Direction.OUT),
    )
    PRECONDITION = "The goal position has been disclosed to the robot."
    EFFECT = "Advances the robot toward the goal along the best admissible arc in the current dynamic window."
    TAGS = ("dwa", "dynamic-window", "local-planner", "collision-avoidance",
            "velocity-space", "goal-seeking")

    # Defaults so a tick before reset() cannot raise.
    _v = 0.0
    _w = 0.0
    _t_prev = None
    _block_t0 = None

    def reset(self, scenario, rng):
        self._v = 0.0
        self._w = 0.0
        self._t_prev = None
        self._block_t0 = None

    def precondition(self, obs, ports):
        return obs.goal is not None

    def tick(self, obs, ports):
        if obs.goal is None:
            # The goal latches once disclosed, so this only happens before
            # disclosure; nothing to steer at, hand control back to the parent.
            ports["dwa_confidence"] = 0.0
            ports["dwa_blocked"] = False
            return Status.FAILURE

        dt = _DT_DEFAULT
        if self._t_prev is not None:
            dt = float(np.clip(obs.t - self._t_prev, _DT_MIN, _DT_MAX))
        self._t_prev = obs.t

        gx_r, gy_r = self._goal_in_robot_frame(obs)
        d_goal = math.hypot(gx_r, gy_r)
        if d_goal <= _GOAL_TOL:
            self._v = 0.0
            self._w = 0.0
            self._block_t0 = None
            ports["dwa_confidence"] = 1.0
            ports["dwa_blocked"] = False
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        points = self._scan_points(obs)

        # Cap the top of the window so the robot can still brake to rest at the
        # goal rather than overshooting it. This shapes the search space, not
        # the objective, so the paper's three terms stay untouched.
        v_stop = math.sqrt(2.0 * _BRAKE_V * max(0.0, d_goal - _GOAL_TOL))
        v_hi = min(_V_MAX, v_stop, self._v + _A_V * dt)
        v_lo = max(0.0, min(self._v - _A_V * dt, v_hi))
        w_hi = min(_W_MAX, self._w + _A_W * dt)
        w_lo = max(-_W_MAX, self._w - _A_W * dt)

        vs = np.linspace(v_lo, v_hi, _N_V)
        ws = np.linspace(w_lo, w_hi, _N_W)

        chosen = self._select(vs, ws, points, gx_r, gy_r,
                             _ROBOT_RADIUS + _SAFETY_MARGIN)
        if chosen is None:
            # My addition, not the paper's: before declaring the window empty,
            # retry with the margin stripped back to the body radius. Otherwise
            # a single beam landing inside the inflated radius (which is not yet
            # a collision) freezes the node permanently.
            chosen = self._select(vs, ws, points, gx_r, gy_r,
                                  _ROBOT_RADIUS + _RELAXED_MARGIN)

        if chosen is None:
            return self._blocked(obs, ports, dt, v_lo, w_lo, w_hi)

        v_cmd, w_cmd, score = chosen
        self._v, self._w = v_cmd, w_cmd
        self._block_t0 = None
        ports["dwa_confidence"] = float(np.clip(score, 0.0, 1.0))
        ports["dwa_blocked"] = False
        ports.drive(Command(float(v_cmd), 0.0, float(w_cmd)))
        return Status.RUNNING

    # -- internals ----------------------------------------------------------

    def _goal_in_robot_frame(self, obs):
        """Goal relative to the pose *belief*, which may have drifted."""
        px, py, yaw = obs.pose_estimate
        dx = obs.goal[0] - px
        dy = obs.goal[1] - py
        c, s = math.cos(yaw), math.sin(yaw)
        return c * dx + s * dy, -s * dx + c * dy

    def _scan_points(self, obs):
        """Finite returns close enough to matter, as (M, 2) robot-frame points."""
        r = np.asarray(obs.ranges, dtype=float)
        a = np.asarray(obs.range_angles, dtype=float)
        # Nothing further than (max arc length + collision radius) can be hit
        # inside the prediction horizon, so drop it and keep the O(C*T*M) work
        # small.
        reach = _V_MAX * _HORIZON + _ROBOT_RADIUS + _SAFETY_MARGIN + 0.2
        keep = np.isfinite(r) & (r <= min(obs.range_max, reach))
        if not np.any(keep):
            return np.zeros((0, 2))
        rk, ak = r[keep], a[keep]
        return np.stack((rk * np.cos(ak), rk * np.sin(ak)), axis=1)

    def _select(self, vs, ws, points, gx_r, gy_r, collide_radius):
        """Score the window and return the best admissible (v, w, score)."""
        V, W = np.meshgrid(vs, ws, indexing="ij")
        v = V.ravel()
        w = W.ravel()

        n_steps = int(round(_HORIZON / _SIM_DT))
        steps = np.arange(1, n_steps + 1) * _SIM_DT
        x, y, th = _rollout(v, w, steps)

        if points.shape[0] == 0:
            dist = v * _HORIZON
        else:
            d = np.hypot(x[:, :, None] - points[None, None, :, 0],
                         y[:, :, None] - points[None, None, :, 1])
            hit = np.any(d < collide_radius, axis=2)            # (C, T)
            any_hit = np.any(hit, axis=1)
            first = np.argmax(hit, axis=1)                      # step index of hit
            # Arc length safely covered is that of the last clean step.
            dist = np.where(any_hit, v * first * _SIM_DT, v * _HORIZON)

        # Admissibility: from this command, on this arc, the robot must still be
        # able to come to rest before the obstacle. This is the paper's safety
        # condition and the reason the search space is a *window* at all.
        admissible = (v <= np.sqrt(2.0 * dist * _BRAKE_V) + 1e-9) & \
                     (np.abs(w) <= np.sqrt(2.0 * dist * _BRAKE_W) + 1e-9)
        if not np.any(admissible):
            return None

        # heading: how well the arc's endpoint faces the goal.
        end_x, end_y, end_th = x[:, -1], y[:, -1], th[:, -1]
        bearing = np.arctan2(gy_r - end_y, gx_r - end_x)
        err = np.abs(_wrap(bearing - end_th))
        heading = 1.0 - err / math.pi

        clearance = np.minimum(dist, _DIST_CAP) / _DIST_CAP
        speed = v / _V_MAX if _V_MAX > 0 else np.zeros_like(v)

        score = _W_HEADING * heading + _W_DIST * clearance + _W_VEL * speed
        # Zero out the inadmissible cells *before* smoothing so that they drag
        # their neighbours down -- commands adjacent to unsafe ones are
        # themselves suspect under an uncertain world model.
        score = np.where(admissible, score, 0.0)
        score = _smooth3(score.reshape(len(vs), len(ws))).ravel()

        score = np.where(admissible, score, -np.inf)
        best = int(np.argmax(score))
        return float(v[best]), float(w[best]), float(score[best])

    def _blocked(self, obs, ports, dt, v_lo, w_lo, w_hi):
        """Empty admissible set: brake and turn toward open space.

        My addition. The paper guarantees only that a stop is always available;
        it does not say what to do next. Braking alone would deadlock facing a
        wall, so while decelerating we also rotate toward the longest beam,
        which re-populates the admissible set within a few ticks.
        """
        r = np.asarray(obs.ranges, dtype=float)
        a = np.asarray(obs.range_angles, dtype=float)
        r = np.where(np.isfinite(r), r, obs.range_max)
        forward = np.abs(a) < 2.0
        target = float(a[forward][int(np.argmax(r[forward]))]) if np.any(forward) else math.pi

        v_cmd = max(0.0, v_lo)
        w_cmd = float(np.clip(np.clip(1.5 * target, -_W_MAX, _W_MAX), w_lo, w_hi))
        self._v, self._w = v_cmd, w_cmd

        if self._block_t0 is None:
            self._block_t0 = obs.t

        ports["dwa_confidence"] = 0.0
        ports["dwa_blocked"] = True
        ports.drive(Command(v_cmd, 0.0, w_cmd))

        if obs.t - self._block_t0 > _BLOCKED_TIMEOUT:
            return Status.FAILURE
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    # The detour around the slab plus the speed reduction the admissibility
    # test forces near it cost time against a straight run.
    "cost_metric": "time_to_goal",
}


def micro(seed=0):
    """Slab across the straight line, wide passage above it.

    goal_seeker has no sensing and drives into the slab face. The mechanism
    under test is the thing that makes the difference: straight-ahead arcs go
    inadmissible as the slab enters the scan, arcs curving upward keep full
    clearance and tie on heading, and the robot rounds the top.
    """
    world = room(6.0, 4.5, boxes=[(2.6, 0.0, 3.4, 2.6)])
    return MicroScenario(
        id="dwa_blocking_slab",
        world=world,
        start=(1.0, 1.3, 0.0),
        goal=(5.0, 1.3),
        time_limit=45.0,
        seed=seed,
        goal_known=True,
    )
