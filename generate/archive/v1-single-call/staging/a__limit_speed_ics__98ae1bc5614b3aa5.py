"""ICS-filtered goal drive for an unmapped indoor space.

Implements the Inevitable Collision State criterion of Fraichard & Asama (2004),
"Inevitable Collision States: a step towards safer robots".  The paper's point is
that being collision-free right now says nothing about safety: a state is an ICS
if *every* future control trajectory ends in collision.  Deciding that exactly is
intractable, so the paper's usable form is a conservative approximation -- test a
finite set of evasive (braking) manoeuvres over a finite horizon.  Because the ICS
set computed with a subset of controls contains the true ICS set, a command that
passes this test is genuinely safe; the test can only be too strict, never too
permissive.

So: every tick, propose goal-directed commands, propagate each one over the
interval we are actually committed to it (control period + actuation latency),
and keep it only if the state it lands in still admits at least one collision-free
braking manoeuvre.  Drive the surviving command with the most goal progress.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

# --- Robot / dynamics model -------------------------------------------------
# None of these come from the paper; they are engineering judgement standing in
# for the Go2's real limits.  The harness rolls the robot out kinematically, so
# it *could* stop dead -- assuming it cannot is the whole point of an ICS test.
_ROBOT_RADIUS = 0.30
_SAFETY_MARGIN = 0.15
_R_SAFE = _ROBOT_RADIUS + _SAFETY_MARGIN

_A_BRAKE = 1.2          # m/s^2 assumed linear deceleration limit
_TAU_LATENCY = 0.25     # s between deciding and the policy actually responding
_DT_DEFAULT = 0.1       # s, used until two ticks have been seen

_V_MAX = 0.8
_VY_MAX = 0.4           # lateral authority is worth less than forward on a quadruped
_WZ_MAX = 1.0
_YAW_GAIN = 1.6         # turn the body toward the chosen motion direction

# Evasive-manoeuvre set: brake straight, and brake through a turn either way.
# A small set is deliberate -- the approximation is conservative by construction.
_BRAKE_YAW_RATES = (0.0, 0.6, -0.6, 1.0, -1.0)

_HEADINGS = tuple(float(a) for a in np.linspace(-2.6, 2.6, 21))
_SPEEDS = (0.8, 0.55, 0.35, 0.18)

_TRAJ_SAMPLES = 8
_GOAL_TOL = 0.35
_HYST = 0.05            # mild penalty on swinging the motion direction around
_CLOUD_REACH = 2.0      # obstacles further than any reachable point cannot matter


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _sweep(state, vx, vy, wz, duration, n, decel=0.0):
    """Roll the body-frame twist forward; return sampled centres and end state.

    Velocity direction is held fixed in the body frame and its magnitude decays
    at `decel`; that is what a braking manoeuvre is here.
    """
    x, y, th = state
    if duration <= 1e-6 or n <= 0:
        return np.zeros((0, 2)), (x, y, th)
    dt = duration / n
    speed = math.hypot(vx, vy)
    if speed > 1e-9:
        ux, uy = vx / speed, vy / speed
    else:
        ux, uy = 0.0, 0.0
    pts = np.empty((n, 2))
    v = speed
    for i in range(n):
        if decel > 0.0:
            v = max(0.0, v - decel * dt)
        bx, by = ux * v, uy * v
        c, s = math.cos(th), math.sin(th)
        x += (bx * c - by * s) * dt
        y += (bx * s + by * c) * dt
        th += wz * dt
        pts[i, 0] = x
        pts[i, 1] = y
    return pts, (x, y, th)


def _clear(pts, cloud, r_safe):
    """True if no sampled body centre comes within r_safe of an obstacle return."""
    if pts.shape[0] == 0 or cloud.shape[0] == 0:
        return True
    d2 = ((pts[:, None, :] - cloud[None, :, :]) ** 2).sum(axis=2)
    return bool(d2.min() >= r_safe * r_safe)


def _admits_evasive_manoeuvre(state, vx, vy, cloud):
    """The ICS test, in its approximate form.

    `state` is where the candidate command leaves us, still carrying body-frame
    velocity (vx, vy).  If any braking manoeuvre from there stays clear all the
    way to rest, the state is not ICS w.r.t. this manoeuvre set.  Rest is safe
    forever in a static workspace, so the finite horizon loses nothing.
    """
    v0 = math.hypot(vx, vy)
    if v0 < 1e-6:
        return True  # already stopped and clear: trivially not an ICS
    duration = v0 / _A_BRAKE
    for wz_brake in _BRAKE_YAW_RATES:
        pts, _ = _sweep(state, vx, vy, wz_brake, duration, _TRAJ_SAMPLES,
                        decel=_A_BRAKE)
        if _clear(pts, cloud, _R_SAFE):
            return True
    return False


class ICSFilteredGoalDrive(Action):
    """Drive at a known goal, admitting only commands that are not ICS.

    Mechanism: Fraichard & Asama's conservative ICS approximation.  A command is
    issued only if, after the interval we are committed to it, some braking
    manoeuvre from the resulting state is still collision-free.  It should work
    because the failure mode of reactive goal-seeking indoors is not "did not see
    the wall" but "saw it too late to stop" -- the robot commits to a state from
    which no control avoids contact.  Checking one step further, against braking
    rather than against the whole control space, is cheap and one-sided: it can
    make the robot too timid, never too bold.

    The filter reads only the egocentric scan, so pose belief drift degrades the
    goal heading but cannot degrade the safety test.
    """

    PORTS = (
        Port("ics_safe_speed", PortType.SPEED, Direction.OUT),
        Port("ics_braking", PortType.FLAG, Direction.OUT),
    )
    PRECONDITION = "The goal position has been disclosed and the robot is free to move."
    EFFECT = ("Moves the robot toward the goal using only commands whose resulting "
              "state still admits a collision-free braking manoeuvre.")
    TAGS = ("ics", "safety", "collision-avoidance", "goal-directed", "braking",
            "reactive", "local-planner")

    def reset(self, scenario, rng):
        self._last_t = None
        self._dt = _DT_DEFAULT
        self._prev_chi = 0.0

    def precondition(self, obs, ports):
        return obs.goal is not None

    def _control_period(self, obs):
        """Measure the tick interval instead of assuming it."""
        if self._last_t is not None and obs.t > self._last_t:
            self._dt = min(0.25, max(0.02, obs.t - self._last_t))
        self._last_t = obs.t
        return self._dt

    def _obstacle_cloud(self, obs):
        """Frozen egocentric obstacle set for this tick.

        Beams with no return are pinned at range_max rather than dropped: unknown
        space is treated as occupied, which keeps the test conservative.  At 6 m
        those virtual points are far outside any reachable braking envelope, so
        in practice they cost nothing.
        """
        r = np.asarray(obs.ranges, dtype=float)
        a = np.asarray(obs.range_angles, dtype=float)
        r = np.where(np.isfinite(r), r, obs.range_max)
        r = np.clip(r, 0.0, obs.range_max)
        keep = r <= _CLOUD_REACH
        if not np.any(keep):
            return np.zeros((0, 2))
        r, a = r[keep], a[keep]
        return np.stack((r * np.cos(a), r * np.sin(a)), axis=1)

    def tick(self, obs, ports):
        if obs.goal is None:
            ports["ics_safe_speed"] = 0.0
            ports["ics_braking"] = True
            return Status.FAILURE

        dt = self._control_period(obs)
        px, py, pyaw = obs.pose_estimate
        gx, gy = obs.goal
        dx, dy = gx - px, gy - py
        dist = math.hypot(dx, dy)

        if dist <= _GOAL_TOL:
            ports.drive(Command(0.0, 0.0, 0.0))
            ports["ics_safe_speed"] = 0.0
            ports["ics_braking"] = False
            return Status.SUCCESS

        bearing = _wrap(math.atan2(dy, dx) - pyaw)
        ugx, ugy = math.cos(bearing), math.sin(bearing)

        cloud = self._obstacle_cloud(obs)
        # The stretch of trajectory we are committed to once this command is sent.
        horizon = dt + _TAU_LATENCY
        v_cap = min(_V_MAX, max(_SPEEDS[-1], dist))

        candidates = []
        for chi in _HEADINGS:
            for v in _SPEEDS:
                if v > v_cap:
                    continue
                vx = v * math.cos(chi)
                vy = float(np.clip(v * math.sin(chi), -_VY_MAX, _VY_MAX))
                wz = float(np.clip(_YAW_GAIN * chi, -_WZ_MAX, _WZ_MAX))
                pts, end = _sweep((0.0, 0.0, 0.0), vx, vy, wz, horizon,
                                  _TRAJ_SAMPLES)
                progress = min(end[0] * ugx + end[1] * ugy, dist)
                score = progress - _HYST * abs(_wrap(chi - self._prev_chi))
                candidates.append((score, chi, vx, vy, wz, pts, end))

        candidates.sort(key=lambda c: -c[0])

        # Best-first: the ICS test is the expensive part, so stop at the first
        # command that survives it rather than filtering the whole grid.
        for score, chi, vx, vy, wz, pts, end in candidates:
            if score <= 0.0:
                break
            if not _clear(pts, cloud, _R_SAFE):
                continue
            if not _admits_evasive_manoeuvre(end, vx, vy, cloud):
                continue
            self._prev_chi = chi
            ports.drive(Command(vx, vy, wz))
            ports["ics_safe_speed"] = float(math.hypot(vx, vy))
            ports["ics_braking"] = False
            return Status.RUNNING

        # Nothing that makes progress is safe.  Rest is never an ICS, and a disc
        # turning on the spot does not sweep new area, so this is the one command
        # always available: hold position and rotate to face the goal.
        ports.drive(Command(0.0, 0.0, float(np.clip(_YAW_GAIN * bearing,
                                                    -_WZ_MAX, _WZ_MAX))))
        ports["ics_safe_speed"] = 0.0
        ports["ics_braking"] = True
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}


def micro(seed=0):
    """A box straddling the start-goal line, open only on the goal-favouring side.

    goal_seeker drives the straight line and buries itself in the box face.  The
    detour is up and over the box top, which is also the direction that reduces
    goal distance, so no separate escape mechanism is being tested -- what decides
    the episode is whether the robot refuses to commit to states in front of the
    face from which it can no longer stop.
    """
    world = room(9.0, 6.0, boxes=[(3.0, 0.0, 3.6, 4.0)])
    return MicroScenario(
        id="ics_blocked_straight_line",
        world=world,
        start=(1.0, 1.0, 0.0),
        goal=(7.5, 5.0),
        time_limit=60.0,
        seed=seed,
        goal_known=True,
    )
