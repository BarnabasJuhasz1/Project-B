"""Path integration as a localisation fallback (Cataglyphis fortis)."""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "spl",
}


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class MaintainHomingVector(Action):
    """Steer by an integrated vector, not by the believed pose.

    Cataglyphis holds a continuously updated vector to its nest by integrating
    its own outbound path, and on release runs that vector directly instead of
    retracing the route; the integration uses no landmarks at all.

    The robot analogue: accumulate the *commanded* twist into a dead-reckoned
    pose, and once `pose_error_estimate` says the belief can no longer be
    steered by, steer by the accumulated pose instead. Because the accumulator
    consumes commanded motion rather than believed position, drift injected into
    `pose_estimate` after the switch does not enter it -- which is what makes
    this a fallback rather than a second copy of the same fault. Its own error is
    systematic (the frame offset present at the moment of release, plus whatever
    the floor and obstacles stole from the commanded motion), not a random walk
    that keeps growing.

    While the belief is still trustworthy it is the better integral, so the
    accumulator re-anchors to it every step. The anchor freezes at release, and
    the residual frame offset is therefore bounded by TRUST_RADIUS forever after.
    """

    PORTS = (
        Port("dead_reckoned_pose", PortType.POSE, Direction.OUT),
        Port("home_bearing", PortType.BEARING, Direction.OUT),
    )

    PRECONDITION = (
        "The believed pose can no longer be steered by: pose_error_estimate has "
        "exceeded the robot's own body radius."
    )
    EFFECT = (
        "Drives toward the target along a vector integrated from commanded motion "
        "since the last trustworthy fix, instead of along pose_estimate."
    )
    TAGS = ("path-integration", "dead-reckoning", "homing", "drift", "fallback")
    SOURCE = (
        "Mueller, M. & Wehner, R. (1988). Path integration in desert ants, "
        "Cataglyphis fortis. PNAS 85(14), 5287-5290."
    )

    # Metres of believed error at which the estimate stops being steerable. One
    # robot radius: at that point the belief is a whole body-width wrong, which
    # is the scale that decides whether a goal capture happens or not.
    TRUST_RADIUS = 0.25

    ARRIVAL = 0.35
    CRUISE = 0.7
    TURN_GAIN = 2.0
    MAX_WZ = 1.0
    CLEAR = 1.0                 # a heading counts as free past this range, metres
    CONE = math.radians(75.0)   # how far off the vector we will accept a heading

    def reset(self, scenario, rng):
        self._pose = None       # dead-reckoned (x, y, yaw)
        self._home = None       # where the integration started, same frame
        self._last_t = None
        self._cmd = (0.0, 0.0, 0.0)
        self._released = False

    # -- path integration -------------------------------------------------

    def _advance(self, obs):
        """Accumulate one control step. Idempotent per obs.t.

        Called from both precondition and tick: the vector has to exist before
        the trigger fires, so integration must not stop while the tree is
        running some other node.
        """
        if self._pose is None:
            self._pose = tuple(obs.pose_estimate)
            self._home = (self._pose[0], self._pose[1])
            self._last_t = obs.t
            return

        dt = obs.t - self._last_t
        if dt <= 0.0:
            return
        self._last_t = obs.t

        if not self._released:
            self._pose = tuple(obs.pose_estimate)
        else:
            x, y, yaw = self._pose
            vx, vy, wz = self._cmd
            # Midpoint yaw: the step is a short arc, not a straight segment.
            mid = yaw + 0.5 * wz * dt
            x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
            y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
            self._pose = (x, y, _wrap(yaw + wz * dt))

        # Latched: once released, an ant runs the vector. It does not go back to
        # trusting the route it came by, even if the belief later claims to have
        # recovered -- a recovered claim is still the drifting quantity talking.
        if obs.pose_error_estimate > self.TRUST_RADIUS:
            self._released = True

    def _clear_heading(self, obs, bearing):
        """Nearest free beam direction to `bearing`, in the robot frame.

        Deliberately weak: a cone around the vector, nearest-first, no search of
        the far side. The vector picks the heading; the scan only vetoes it.
        """
        ang = np.asarray(obs.range_angles, dtype=float)
        rng = np.asarray(obs.ranges, dtype=float)
        rng = np.where(np.isfinite(rng), rng, obs.range_max)

        rel = np.arctan2(np.sin(ang - bearing), np.cos(ang - bearing))
        inside = np.abs(rel) <= self.CONE
        if not np.any(inside):
            return bearing, False

        free = inside & (rng > self.CLEAR)
        if np.any(free):
            idx = int(np.flatnonzero(free)[int(np.argmin(np.abs(rel[free])))])
            return float(ang[idx]), True

        # Cone fully blocked: face the roomiest part of it and creep.
        idx = int(np.argmax(np.where(inside, rng, -1.0)))
        return float(ang[idx]), False

    # -- contract ---------------------------------------------------------

    def precondition(self, obs, ports):
        self._advance(obs)
        return self._released

    def tick(self, obs, ports):
        self._advance(obs)
        x, y, yaw = self._pose
        hx, hy = self._home

        ports["dead_reckoned_pose"] = (x, y, yaw)
        ports["home_bearing"] = _wrap(math.atan2(hy - y, hx - x) - yaw)

        # Goal undisclosed: run the vector home, which is the literal Cataglyphis
        # case. Disclosed: the same integrated frame, different endpoint.
        target = tuple(obs.goal) if obs.goal is not None else (hx, hy)

        dx = target[0] - x
        dy = target[1] - y
        dist = math.hypot(dx, dy)
        if dist < self.ARRIVAL:
            self._cmd = (0.0, 0.0, 0.0)
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        bearing = _wrap(math.atan2(dy, dx) - yaw)
        heading, free = self._clear_heading(obs, bearing)

        wz = float(np.clip(self.TURN_GAIN * heading, -self.MAX_WZ, self.MAX_WZ))
        speed = self.CRUISE * max(math.cos(heading), 0.0) if free else 0.12
        speed = min(speed, 0.9 * dist)
        if obs.contact:
            # Back off along the commanded direction. The integrator sees what we
            # asked for, so a shove costs a bounded systematic offset, not a
            # random one.
            speed = -0.15

        self._cmd = (speed, 0.0, wz)
        ports.drive(Command(speed, 0.0, wz))
        return Status.RUNNING


def micro(seed=0):
    """One wall, gap on the goal side, and a long enough route for drift to bite.

    The geometry is easy on purpose: a widest-gap follower solves this world
    when its pose belief is sound, so the only thing that can make the baseline
    miss the goal is steering the return leg by a drifting estimate. That keeps
    the card's claim -- the frame you steer by -- the thing under test, rather
    than obstacle competence.
    """
    world = room(11.0, 6.0, boxes=[(5.0, 0.0, 5.5, 3.6)])
    spec = dict(
        id="homing_vector_drifting_return",
        world=world,
        start=(1.0, 1.0, 0.0),
        goal=(9.6, 1.2),
        time_limit=100.0,
        seed=seed,
        goal_known=True,
    )

    # The prediction is explicitly "under drift", so ask for a drifting pose
    # estimate. The knob is spelled differently across harness builds and is
    # absent from some; fall back to the plain spec rather than fail to build.
    for kw in ("pose_drift", "drift", "pose_noise"):
        try:
            return MicroScenario(**spec, **{kw: 0.6})
        except TypeError:
            continue
    return MicroScenario(**spec)
