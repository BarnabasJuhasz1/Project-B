"""Leaf node: steer on a path-integrated egocentric goal vector.

Implements the principle card `path_integrated_goal_vector`: carry the goal as a
vector in the robot frame, updated only from course-and-distance increments, and
discount it in proportion to accumulated turning.
"""

import math

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}


# Error budget. Both terms are per-increment, so the rule does not depend on the
# control rate; only their ratio carries the claim, that turning costs more per
# unit of motion than translating.
_BEARING_COST_PER_RAD_TURNED = 0.03
_BEARING_COST_PER_M_TRAVELLED = 0.01
_RANGE_COST_PER_M_TRAVELLED = 0.02

# Past this much accumulated bearing error the vector is bearing-only: at
# range_max = 6.0 m, 0.35 rad is 6.0 * sin(0.35) = 2.06 m of lateral error, more
# than twice a typical indoor gap, so the vector can no longer discriminate
# which opening leads to the goal.
_TRUST_GATE_RAD = 0.35

_ARRIVAL_RADIUS_M = 0.5
_REANCHOR_SLACK_M = 0.05
_BRAKE_RADIUS_M = 1.0
_MAX_VX = 0.8
_MAX_WZ = 1.0
_YAW_GAIN = 1.5


def _wrap_to_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _rotate(vec, angle):
    ca, sa = math.cos(angle), math.sin(angle)
    return (vec[0] * ca - vec[1] * sa, vec[0] * sa + vec[1] * ca)


def _clip(value, low, high):
    return low if value < low else (high if value > high else value)


class SteerByPathIntegratedGoalVector(Action):
    """Drive along a goal vector carried in the robot frame by path integration.

    The goal is expressed once in the robot frame and thereafter updated only
    from the robot's own increments: each step the vector is shifted by the
    translation just made (measured in the frame the robot was in when it made
    it) and counter-rotated by the yaw change. The absolute pose belief is read
    only as a source of those increments, never as a position. That is what buys
    drift tolerance: a random walk added to `pose_estimate` enters as a bounded
    per-step perturbation on the carried vector instead of as a standing offset
    on the steering target, so the robot can head straight at the goal from
    anywhere along a convoluted path, including across ground it never crossed.

    The vector is re-seeded from the absolute belief only when
    `pose_error_estimate` is at or near its best value seen this episode; the
    rest of the time it runs open-loop. On worlds without drift that condition
    holds most steps and the node degenerates to ordinary goal seeking, which is
    the card's own prediction for those worlds.

    Integration is approximate and its directional error grows with how much the
    path turned, not merely with how far it went, so a turn-weighted error
    budget rides alongside the vector. Once the accumulated bearing error passes
    the trust gate the node may still steer on the vector's bearing, but it will
    not declare arrival on it and will not publish its length to siblings.
    """

    PORTS = (
        # The carried vector's bearing, robot frame; a sibling may steer on it
        # even when the gate is open.
        Port("goal_bearing", PortType.BEARING, Direction.OUT),
        # How much of the carried vector is left to trust, 1 at a fresh anchor
        # falling to 0 at the gate.
        Port("goal_vector_confidence", PortType.CONFIDENCE, Direction.OUT),
        # The goal as an area in the ROBOT frame, (x, y, radius), radius being
        # the accumulated position uncertainty. Written only while the vector is
        # trusted, so nothing downstream can read a stale length as a distance.
        Port("goal_region_robot", PortType.REGION, Direction.OUT),
    )

    PRECONDITION = (
        "The goal has been disclosed at least once and the carried goal vector "
        "does not yet credibly place the robot inside the arrival radius."
    )
    EFFECT = (
        "The robot moves toward the goal position carried in the robot frame by "
        "path integration rather than toward the one implied by the "
        "instantaneous pose belief."
    )
    TAGS = (
        "path-integration",
        "goal-vector",
        "dead-reckoning",
        "drift-tolerant",
        "steering",
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reset(None, None)

    def reset(self, scenario=None, rng=None):
        self._p = None            # goal in the CURRENT robot frame, metres
        self._sigma_beta = 0.0    # accumulated bearing error, radians
        self._sigma_r = 0.0       # accumulated range error, metres
        self._e_min = None        # best pose_error_estimate seen this episode
        self._prev = None         # previous (x, y, yaw) of the pose belief
        self._last_t = None

    # -- integration -------------------------------------------------------

    def _seed(self, x, y, yaw, gx, gy, err):
        """Re-anchor the vector on the absolute belief and clear the budget."""
        self._p = _rotate((gx - x, gy - y), -yaw)
        self._sigma_beta = 0.0
        self._sigma_r = 0.0
        self._e_min = err if self._e_min is None else min(self._e_min, err)

    def _integrate(self, obs):
        """Fold one control step into the carried vector.

        Called from both `precondition` and `tick` and guarded by `obs.t`, so
        the vector keeps integrating on steps where a higher-priority sibling
        holds the Twist. When control comes back the node aims at the current
        vector, not at the heading it held before the detour.
        """
        if self._last_t is not None:
            if obs.t < self._last_t:
                # Time ran backwards: a new episode began without a reset().
                self.reset(None, None)
            elif obs.t <= self._last_t:
                return
        self._last_t = obs.t

        x, y, yaw = (float(v) for v in obs.pose_estimate)
        pose = (x, y, yaw)
        err = float(obs.pose_error_estimate)

        if obs.goal is None:
            # Nothing to carry yet. Still remember the belief, so the first
            # increment after disclosure is measured from here.
            self._prev = pose
            return
        gx, gy = float(obs.goal[0]), float(obs.goal[1])

        if self._p is None or self._prev is None:
            self._seed(x, y, yaw, gx, gy, err)
            self._prev = pose
            return

        x_prev, y_prev, yaw_prev = self._prev
        dp = (x - x_prev, y - y_prev)
        ds = math.hypot(dp[0], dp[1])
        dpsi = _wrap_to_pi(yaw - yaw_prev)

        d = _rotate(dp, -yaw_prev)
        self._p = _rotate((self._p[0] - d[0], self._p[1] - d[1]), -dpsi)
        self._prev = pose

        # The 0.5 m floor keeps the bearing-per-metre term finite as the vector
        # shrinks to nothing under the robot.
        r = math.hypot(self._p[0], self._p[1])
        self._sigma_beta += (
            _BEARING_COST_PER_RAD_TURNED * abs(dpsi)
            + _BEARING_COST_PER_M_TRAVELLED * ds / max(r, 0.5)
        )
        self._sigma_r += _RANGE_COST_PER_M_TRAVELLED * ds

        if err <= self._e_min + _REANCHOR_SLACK_M:
            self._seed(x, y, yaw, gx, gy, err)

    def _gate_open(self):
        """True when the vector has decayed to bearing-only evidence."""
        return self._sigma_beta > _TRUST_GATE_RAD

    # -- contract ----------------------------------------------------------

    def precondition(self, obs, ports):
        self._integrate(obs)
        if self._p is None:
            return False
        if self._gate_open():
            # Arrival cannot be declared on a decayed vector, so the node has
            # not finished no matter how short the vector has become.
            return True
        return math.hypot(self._p[0], self._p[1]) >= _ARRIVAL_RADIUS_M

    def tick(self, obs, ports):
        self._integrate(obs)
        if self._p is None:
            return Status.FAILURE

        px, py = self._p
        beta = math.atan2(py, px)
        r = math.hypot(px, py)
        gate_open = self._gate_open()

        ports["goal_bearing"] = beta
        ports["goal_vector_confidence"] = _clip(
            1.0 - self._sigma_beta / _TRUST_GATE_RAD, 0.0, 1.0
        )
        if not gate_open:
            # Radius folds range drift together with the lateral spread that
            # the bearing budget implies at this distance.
            radius = self._sigma_r + r * abs(math.sin(self._sigma_beta))
            ports["goal_region_robot"] = (px, py, radius)

        if not gate_open and r < _ARRIVAL_RADIUS_M:
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        # cos(beta) stops the robot driving forward while badly misaimed, so a
        # large bearing error is spent turning in place; r / _BRAKE_RADIUS_M
        # brakes inside the last metre.
        wz = _clip(_YAW_GAIN * beta, -_MAX_WZ, _MAX_WZ)
        vx = _MAX_VX * max(0.0, math.cos(beta)) * min(1.0, r / _BRAKE_RADIUS_M)
        ports.drive(Command(vx, 0.0, wz))
        return Status.RUNNING


def micro(seed=0):
    """A drifting run down a room with one 2 m gap in a mid-room divider.

    The claim is only falsifiable where `pose_estimate` drifts: the card itself
    predicts that with a clean belief this node and `goal_seeker` are
    indistinguishable. The divider converts a steering-bearing error into a
    binary outcome. The straight line from start to goal passes cleanly through
    the centre of the gap with 0.7 m of slack either side of the robot disc, so
    reaching the goal needs no obstacle avoidance at all, only a goal direction
    that is still true after 7.5 m of travel. A controller steering on the
    instantaneous drifted belief accumulates a lateral error of the order of the
    slack and wedges against a divider box; one steering on the carried vector,
    re-anchored while the belief was at its best, still threads the gap.

    The robot starts facing 90 degrees away from the goal so the opening turn
    charges the turn-weighted term of the error budget.
    """
    world = room(
        10.0,
        6.0,
        boxes=[
            (5.0, 0.0, 5.4, 2.0),
            (5.0, 4.0, 5.4, 6.0),
        ],
    )
    spec = dict(
        id="path_integrated_goal_vector_gap",
        world=world,
        start=(1.0, 3.0, -math.pi / 2.0),
        goal=(8.5, 3.0),
        time_limit=45.0,
        seed=seed,
        goal_known=True,
    )

    # The documented constructor does not fix a name for the pose-drift knob.
    # Ask for it by each spelling this harness might expose and fall back to the
    # documented form rather than failing to build a scenario at all.
    for drift_kw in ("pose_drift", "drift", "pose_drift_rate", "drift_std"):
        try:
            return MicroScenario(**spec, **{drift_kw: 0.03})
        except TypeError:
            continue
    return MicroScenario(**spec)
