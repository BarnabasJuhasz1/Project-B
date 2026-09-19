"""Terminal spiral search at the believed goal (pi_terminal_spiral_search)."""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}


class SpiralSearchAtBelievedGoal(Action):
    """Sweep an outward Archimedean spiral around the believed goal instead of
    halting on it.

    A dead-reckoned pose estimate degrades monotonically: each motion increment
    is added onto the previous estimate and each increment carries error. That
    integrator is accurate enough to steer a long approach but not to terminate
    one. Animals that path-integrate do not stop when the integrator reads zero;
    they switch from linear homing to circular movement near the goal and sweep
    its neighbourhood. That mode switch converts an unbounded point-accuracy
    requirement into a bounded area-coverage one: the straight leg only has to
    land inside the search radius, and the sweep does the rest.

    So when the residual vector ``goal - pose_estimate[:2]`` falls inside
    ``r_switch``, this leaf stops trusting the belief and spirals outward from
    where it stands, covering the disc the true goal probably lies in. It helps
    exactly when the integrator is wrong and costs distance when it is right.

    The robot has no goal detector -- a planar lidar cannot recognise a goal --
    so the sweep is open-loop and arrival is adjudicated by the scorer, not by
    the robot. Termination is radius-and-timeout, standing in for the animal's
    recognition of familiar cues at the nest.
    """

    PORTS = (
        Port("spiral_search_exhausted", PortType.FLAG, Direction.OUT),
    )

    PRECONDITION = (
        "The goal is disclosed, the residual goal vector in the belief frame has "
        "fallen to or below the switch radius, and this leaf has not yet spent "
        "its per-episode search budget."
    )
    EFFECT = (
        "Sweeps the floor around the believed goal along an outward spiral, so "
        "the true goal is crossed even when the pose belief is displaced."
    )
    TAGS = ("path-integration", "terminal-search", "spiral", "drift", "goal-reaching")

    # Cruise speed for the sweep. Well under the 0.8 m/s cap, which leaves
    # headroom for a higher-priority avoidance branch to preempt cleanly.
    SPEED = 0.35

    # r0 is forced by the actuator envelope, not chosen: a circle of radius r at
    # speed v needs wz = v/r, so |wz| <= 1.0 rad/s at v = 0.35 m/s gives
    # r >= 0.35 m exactly.
    R0 = 0.35

    # Metres between successive turns. Any scorer success radius >= 0.25 m is
    # then swept with no gap; 0.25 m is an assumption about the scorer.
    PITCH = 0.5

    # r_switch = clip(1.5 * pose_error_estimate, 0.7, 3.0). The 1.5 covers the
    # tail of a zero-mean drift of that scale. The floor is one body length,
    # below which a spiral is not physically meaningful; the ceiling is half of
    # range_max, so the search stays inside sensed space.
    ERROR_GAIN = 1.5
    R_SWITCH_MIN = 0.7
    R_SWITCH_MAX = 3.0

    # At 0.35 m/s and 0.5 m pitch, a disc of radius 3.0 m costs ~57 m of path.
    # 60 s buys ~21 m, i.e. full coverage out to ~1.8 m.
    TIME_BUDGET = 60.0

    MAX_WZ = 1.0

    # Yield thresholds. Obstacle handling belongs to another node; this leaf
    # only steps aside for it.
    YIELD_CLEARANCE = 0.4
    YIELD_HALF_ANGLE = math.radians(30.0)

    NOMINAL_DT = 0.05
    MAX_DT = 0.2

    def reset(self, scenario, rng):
        self._phi = 0.0
        self._elapsed = 0.0
        self._exhausted = False
        self._last_t = None
        # Frozen once the sweep starts: pose_error_estimate keeps growing while
        # we spiral, and a live exit radius would recede faster than we sweep.
        self._r_switch = None

    def precondition(self, obs, ports):
        if obs.goal is None:
            return False
        if getattr(self, "_exhausted", False):
            return False
        residual = self._residual(obs)
        if residual is None:
            return False
        return residual <= self._switch_radius(obs)

    def tick(self, obs, ports):
        exhausted = getattr(self, "_exhausted", False)
        ports["spiral_search_exhausted"] = exhausted

        if obs.goal is None:
            return Status.FAILURE
        if exhausted:
            return Status.FAILURE

        dt = self._step_dt(obs)

        if self._r_switch is None:
            self._r_switch = self._switch_radius(obs)

        # Hand the step to the avoidance branch without consuming budget or
        # phase, so the sweep resumes where it left off.
        if obs.contact or self._front_clearance(obs) < self.YIELD_CLEARANCE:
            return Status.RUNNING

        radius = self.R0 + self.PITCH * self._phi / (2.0 * math.pi)

        if radius >= self._r_switch:
            self._exhausted = True
            ports["spiral_search_exhausted"] = True
            return Status.SUCCESS

        if self._elapsed >= self.TIME_BUDGET:
            self._exhausted = True
            ports["spiral_search_exhausted"] = True
            return Status.FAILURE

        wz = float(np.clip(self.SPEED / radius, -self.MAX_WZ, self.MAX_WZ))
        ports.drive(Command(self.SPEED, 0.0, wz))

        self._phi += (self.SPEED / radius) * dt
        self._elapsed += dt
        return Status.RUNNING

    def _residual(self, obs):
        """Length of goal - pose_estimate[:2], the paper's homing vector."""
        goal = obs.goal
        if goal is None:
            return None
        pose = obs.pose_estimate
        return math.hypot(float(goal[0]) - float(pose[0]),
                          float(goal[1]) - float(pose[1]))

    def _switch_radius(self, obs):
        err = float(obs.pose_error_estimate)
        if not math.isfinite(err) or err < 0.0:
            err = 0.0
        return float(np.clip(self.ERROR_GAIN * err,
                             self.R_SWITCH_MIN, self.R_SWITCH_MAX))

    def _front_clearance(self, obs):
        """Minimum return within +/-30 deg of the current heading."""
        angles = np.asarray(obs.range_angles, dtype=float)
        ranges = np.asarray(obs.ranges, dtype=float)
        # Beams span a full circle; wrap before comparing so the sector really
        # is the front and not whatever index happens to sit near zero.
        wrapped = np.arctan2(np.sin(angles), np.cos(angles))
        sector = np.abs(wrapped) <= self.YIELD_HALF_ANGLE
        if not np.any(sector):
            return float(obs.range_max)
        vals = ranges[sector]
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            return float(obs.range_max)
        return float(np.min(vals))

    def _step_dt(self, obs):
        t = float(obs.t)
        last = getattr(self, "_last_t", None)
        self._last_t = t
        if last is None:
            return self.NOMINAL_DT
        return float(np.clip(t - last, 0.0, self.MAX_DT))


def micro(seed=0):
    """Open room with drift-prone odometry and no obstruction on the route.

    Both controllers can walk the diagonal unimpeded, so geometry cannot be what
    separates them: the only failure mode available to `goal_seeker` is halting
    where its drifted belief says the goal is. The boxes sit well off the
    start-goal line to keep the room indoor-shaped without blocking either run.
    """
    world = room(
        8.0, 8.0,
        boxes=[
            (0.2, 4.9, 2.4, 6.1),
            (5.1, 0.4, 6.6, 2.4),
        ],
    )
    return MicroScenario(
        id="spiral_search_at_believed_goal",
        world=world,
        start=(1.2, 1.2, 0.0),
        goal=(6.0, 6.0),
        time_limit=120.0,
        seed=seed,
        goal_known=True,
    )
