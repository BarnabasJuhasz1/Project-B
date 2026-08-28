"""Braking-admissible forward speed for a Go2 under a frozen locomotion policy.

Implements the Dynamic Window Approach admissibility bound (Fox, Burgard &
Thrun 1997): a forward velocity is admissible only if the robot can still brake
to a stop inside the free distance ahead, i.e. v <= sqrt(2 * d * a_brake).
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


PREDICTION = {
    "statement": (
        "A controller bounding forward speed by sqrt(2 * d * a_brake), with d the "
        "free distance along the intended heading, produces fewer collisions on "
        "doorway_traverse than reactive_avoid, which brakes on the cone minimum."
    ),
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}


def _wrap(angle):
    """Fold an angle (scalar or array) into [-pi, pi)."""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class LimitSpeedToVisibleRange(Action):
    """Pursue the goal bearing at a speed the robot could still brake out of.

    Mechanism (DWA admissibility): with a braking deceleration a_brake and a
    free distance d ahead, the largest speed from which the robot can still
    stop short of the obstruction is sqrt(2 * d * a_brake). Forward speed is
    clamped to that bound every tick, so the commanded velocity is admissible
    by construction rather than corrected after a near-miss.

    d is the free distance along the *intended heading* only: beams are
    projected into the heading frame and a return counts as an obstruction just
    when it falls inside the corridor the robot body would sweep. A wall
    passing to one side is not something the robot is about to hit, so it does
    not enter d -- which is where this differs from braking on the minimum of
    the whole forward cone, and why the robot can keep useful speed while
    threading a doorway whose jambs are close on both sides.

    a_brake is not a published figure for this platform. The locomotion policy
    is inherited and frozen, so it is treated as a measured scenario constant.

    The card's trigger (free distance below the stopping distance for the
    currently commanded speed) is the condition under which the bound *binds*,
    and is evaluated inside tick() on every control step. It is deliberately
    not the gate in precondition(): a gate that goes false in open space would
    switch the node off exactly when it is the thing driving.
    """

    PORTS = (
        Port("speed_limit", PortType.SPEED, Direction.OUT),
    )

    PRECONDITION = (
        "A goal position is disclosed and not yet reached, so there is an "
        "intended heading along which free distance can bound forward speed."
    )
    EFFECT = (
        "Commanded forward speed never exceeds the speed from which the robot "
        "could still brake to a stop within the free distance measured along "
        "the intended heading."
    )
    TAGS = (
        "dynamic-window",
        "admissible-velocity",
        "braking-distance",
        "speed-limit",
        "collision-avoidance",
        "doorway",
    )
    SOURCE = (
        "Fox, D., Burgard, W. & Thrun, S. (1997). The Dynamic Window Approach "
        "to Collision Avoidance. IEEE Robotics & Automation Magazine 4(1), 23-33."
    )

    # Measured on the Go2 under the frozen locomotion policy, then held as a
    # scenario constant. Conservative: under-estimating a_brake only lowers the
    # admissible speed.
    A_BRAKE = 1.1              # m/s^2
    V_CRUISE = 0.8             # m/s, the runner's forward saturation
    W_MAX = 1.0                # rad/s, the runner's yaw saturation

    ROBOT_RADIUS = 0.3
    CLEARANCE = 0.08
    # Half-width of the corridor the body sweeps along the intended heading.
    CORRIDOR_HALF_WIDTH = ROBOT_RADIUS + CLEARANCE
    # A beam return at range r is r metres from the body centre; the hull is
    # already ROBOT_RADIUS of the way there, so that much is not free distance.
    STOP_MARGIN = ROBOT_RADIUS + 0.05

    K_YAW = 2.0
    # Beyond this heading error the body direction and the measured direction
    # are different enough that d would describe a ray we are not travelling
    # down, so turn in place instead of translating.
    ALIGN_LIMIT = 1.0          # rad
    GOAL_TOL = 0.35            # m
    MIN_ADMISSIBLE = 0.05      # m/s, below this the trajectory is inadmissible
    DEFAULT_DT = 0.05          # s, used before two ticks have been seen

    def reset(self, scenario, rng):
        self._last_v = 0.0
        self._last_t = None

    def precondition(self, obs, ports):
        if obs.goal is None:
            return False
        px, py, _ = obs.pose_estimate
        return math.hypot(obs.goal[0] - px, obs.goal[1] - py) > self.GOAL_TOL

    def _free_distance(self, obs, heading):
        """Free travel along `heading` (robot frame) before the swept corridor
        is obstructed, in metres."""
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)

        finite = np.isfinite(ranges)
        if not np.any(finite):
            return float(obs.range_max)
        r = ranges[finite]
        rel = _wrap(angles[finite] - heading)

        along = r * np.cos(rel)
        lateral = r * np.sin(rel)
        blocking = (along > 0.0) & (np.abs(lateral) <= self.CORRIDOR_HALF_WIDTH)
        if not np.any(blocking):
            return float(obs.range_max)
        return float(max(0.0, np.min(along[blocking]) - self.STOP_MARGIN))

    def tick(self, obs, ports):
        if obs.goal is None:
            # Nothing to head along; do not drive, let a search node have it.
            ports["speed_limit"] = 0.0
            return Status.FAILURE

        dt = self.DEFAULT_DT if self._last_t is None else max(1e-3, obs.t - self._last_t)
        self._last_t = obs.t

        px, py, yaw = obs.pose_estimate
        dx = obs.goal[0] - px
        dy = obs.goal[1] - py
        goal_range = math.hypot(dx, dy)

        if goal_range <= self.GOAL_TOL:
            ports["speed_limit"] = 0.0
            self._last_v = 0.0
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        heading_error = _wrap(math.atan2(dy, px * 0.0 + dx) - yaw)
        d_free = self._free_distance(obs, heading_error)

        # The bound itself. Published for siblings whether or not it binds.
        v_admissible = min(self.V_CRUISE, math.sqrt(2.0 * d_free * self.A_BRAKE))
        ports["speed_limit"] = float(max(0.0, v_admissible))

        wz = float(np.clip(self.K_YAW * heading_error, -self.W_MAX, self.W_MAX))

        if v_admissible < self.MIN_ADMISSIBLE and abs(heading_error) <= self.ALIGN_LIMIT:
            # No forward velocity along this heading is admissible: stop rather
            # than creep into the obstruction, and hand over.
            self._last_v = 0.0
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.FAILURE

        if abs(heading_error) > self.ALIGN_LIMIT:
            v = 0.0
        else:
            v = v_admissible * math.cos(heading_error)

        # Speed may fall to the bound instantly, but may only rise at a_brake:
        # commanding a step the frozen policy cannot track would invalidate the
        # deceleration the bound is computed from.
        v = min(v, self._last_v + self.A_BRAKE * dt)
        v = float(max(0.0, v))

        self._last_v = v
        ports.drive(Command(v, 0.0, wz))
        return Status.RUNNING


def micro(seed=0):
    """Doorway traverse with a wider decoy gap.

    The start->goal line runs through the 1.1 m door at y in [2.4, 3.5]. The
    2.0 m gap at the top of the divider subtends a wider bearing from the start
    and also reduces goal distance, so a widest-gap follower takes it and is
    funnelled into the 0.8 m slot between the divider and the north-east block,
    where a full-speed 90 degree turn puts it into a wall. A robot bounding
    forward speed by the free distance along its intended heading ignores the
    jambs sitting outside its swept corridor, keeps the door on its heading and
    passes through it.
    """
    world = room(
        9.0, 7.0,
        boxes=[
            (4.4, 0.0, 4.6, 2.4),   # divider, south of the door
            (4.4, 3.5, 4.6, 5.2),   # divider, between the door and the decoy gap
            (5.4, 4.3, 9.0, 7.0),   # block behind the decoy gap: leaves a 0.8 m slot
        ],
    )
    return MicroScenario(
        id="doorway_traverse_decoy_gap",
        world=world,
        start=(1.2, 4.3, 0.0),
        goal=(7.8, 1.6),
        time_limit=45.0,
        seed=seed,
        goal_known=True,
    )
