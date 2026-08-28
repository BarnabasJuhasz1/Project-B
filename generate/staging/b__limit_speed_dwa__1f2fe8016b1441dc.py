"""Speed admissibility from the Dynamic Window Approach (Fox, Burgard & Thrun 1997).

One leaf: bound the commanded forward speed by the speed from which the robot can
still brake to a stop inside the free distance measured *along the intended
heading*.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PREDICTION = {
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}


def _wrap(a):
    """Wrap angle(s) to [-pi, pi). Works on scalars and arrays."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class LimitSpeedToVisibleRange(Action):
    """Drive at a forward speed the robot can brake out of before what it can see.

    Mechanism (DWA admissibility): a forward velocity v is admissible only if
    v <= sqrt(2 * d * a_brake), where d is the distance still free along the
    trajectory the robot is about to follow and a_brake is the deceleration the
    frozen locomotion policy can actually deliver.

    d is measured as the free distance along the *intended heading*, not as the
    minimum return anywhere in the forward cone. A beam counts only if its return
    point lies inside the rectangle of half-width (robot radius + clearance) swept
    along that heading, and it then contributes its projection onto the heading.
    A wall passing to one side is not something the robot is about to hit, so it
    never enters the corridor and never slows the robot down; a door frame the
    robot is aimed between does enter it, and does.

    Why it should work: the failure mode on a tight doorway is arriving with more
    speed than the remaining free distance can absorb, so residual heading error
    cannot be corrected before contact. Bounding v by the braking reachability of
    the visible free distance makes the speed collapse exactly as the frames close
    in, buying the heading loop the time it needs. The cost is honest and paid in
    time_to_goal: the robot creeps through anything tight.
    """

    PORTS = (
        # Published so a sibling can respect the same ceiling; this node is the
        # only writer.
        Port("admissible_speed", PortType.SPEED, Direction.OUT),
    )
    PRECONDITION = (
        "The intended heading has a return inside sensor range, so the free "
        "distance along it -- not the cruise cap -- is what the forward speed "
        "must be able to brake within."
    )
    EFFECT = (
        "Commands a forward speed no greater than sqrt(2 * d * a_brake), the "
        "speed the robot can still stop from inside the free distance d measured "
        "along the intended heading."
    )
    TAGS = ("speed-limit", "braking", "admissible-velocity", "dwa", "doorway")

    # Measured on the Go2 under the inherited frozen locomotion policy, treated as
    # a scenario constant: the policy is not ours to change, so its achievable
    # deceleration is an observation about the platform, not a tuning knob.
    A_BRAKE = 1.2

    V_CRUISE = 0.8
    ROBOT_RADIUS = 0.3

    # Half-width of the swept corridor. Slightly wider than the disc so a return
    # that would graze the shoulder still counts as something to stop before.
    CORRIDOR_HALF_WIDTH = ROBOT_RADIUS + 0.05

    # Subtracted from the along-heading return so d is free *travel*, measured
    # from the front of the disc rather than from its centre.
    STANDOFF = ROBOT_RADIUS + 0.10

    YAW_GAIN = 1.8
    WZ_MAX = 1.0

    # Beyond this heading error, forward motion is not what the robot needs; it
    # turns in place. This is a carrier for the Action, not a second mechanism --
    # it never steers away from an obstacle, only towards the goal.
    ALIGN_LIMIT = 1.0

    GOAL_TOL = 0.25
    BLOCKED_PATIENCE = 2.0

    def reset(self, scenario, rng):
        self._blocked_since = None

    def precondition(self, obs, ports) -> bool:
        heading = self._intended_heading(obs)
        _, saw_return = self._corridor_range(obs, heading)
        return saw_return

    def tick(self, obs, ports) -> Status:
        goal = obs.goal
        heading = self._intended_heading(obs)

        if goal is not None:
            # Bearing comes off pose_estimate, which is a belief and may have
            # drifted. Nothing here can detect that; the speed bound is computed
            # from the scan, which is in the robot frame and unaffected by drift.
            px, py, _ = obs.pose_estimate
            if math.hypot(goal[0] - px, goal[1] - py) <= self.GOAL_TOL:
                ports["admissible_speed"] = 0.0
                ports.drive(Command(0.0, 0.0, 0.0))
                return Status.SUCCESS

        along, _ = self._corridor_range(obs, heading)
        d_free = max(0.0, along - self.STANDOFF)

        v_admissible = min(self.V_CRUISE, math.sqrt(2.0 * d_free * self.A_BRAKE))
        ports["admissible_speed"] = v_admissible

        if abs(heading) > self.ALIGN_LIMIT:
            vx = 0.0
        else:
            vx = v_admissible

        wz = max(-self.WZ_MAX, min(self.WZ_MAX, self.YAW_GAIN * heading))
        ports.drive(Command(vx, 0.0, wz))

        return self._progress_status(obs, v_admissible, heading)

    def _progress_status(self, obs, v_admissible, heading):
        """RUNNING while making way; FAILURE once braked to a standstill aligned.

        A momentary zero (sweeping past a wall mid-turn) must not abort the node,
        so the standstill has to persist before it is reported as a failure a
        sibling should take over from.
        """
        stalled = v_admissible <= 1e-3 and abs(heading) <= self.ALIGN_LIMIT
        if not stalled:
            self._blocked_since = None
            return Status.RUNNING

        if self._blocked_since is None:
            self._blocked_since = obs.t
        if obs.t - self._blocked_since >= self.BLOCKED_PATIENCE:
            return Status.FAILURE
        return Status.RUNNING

    def _intended_heading(self, obs):
        """Bearing, robot frame, of the trajectory whose free distance we measure.

        With the goal undisclosed there is no goal-directed trajectory to bound,
        so the intended heading is straight ahead: the node still refuses to
        outrun what it can see, it just has nothing to aim at.
        """
        goal = obs.goal
        if goal is None:
            return 0.0
        px, py, yaw = obs.pose_estimate
        return _wrap(math.atan2(goal[1] - py, goal[0] - px) - yaw)

    def _corridor_range(self, obs, heading):
        """Free distance along `heading`, and whether any return bounded it.

        Returns (along_distance_from_robot_centre, saw_return). Only beams whose
        return point falls inside the swept corridor are considered, which is what
        separates this from braking on the forward-cone minimum.
        """
        ranges = np.asarray(obs.ranges, dtype=float)
        finite = np.isfinite(ranges)
        # inf beams would poison the trig; clamp them to range_max and drop them
        # from the mask instead.
        r = np.where(finite, ranges, obs.range_max)

        rel = _wrap(np.asarray(obs.range_angles, dtype=float) - heading)
        along = r * np.cos(rel)
        lateral = np.abs(r * np.sin(rel))

        in_corridor = finite & (along > 0.0) & (lateral <= self.CORRIDOR_HALF_WIDTH)
        if not np.any(in_corridor):
            return float(obs.range_max), False
        return float(np.min(along[in_corridor])), True


def micro(seed=0):
    """Doorway traverse: the goal ray runs clean through a tight door.

    The straight line from start to goal passes through the doorway centre, so
    what is being tested is speed, not route choice. The doorway is 0.85 m for a
    0.6 m robot -- 0.125 m per side -- and a pillar hides it until roughly two
    metres out, so the approach is committed before the frames are visible.
    A controller that arrives at cruise speed cannot shed the residual heading
    error inside the remaining free distance and clips the frame; one that bounds
    speed by sqrt(2 * d * a_brake) with d taken along its own heading has already
    slowed to a crawl by the time the frames enter its corridor.
    """
    world = room(
        9.0,
        6.0,
        boxes=[
            (4.35, 0.00, 4.65, 3.40),   # divider, lower leaf
            (4.35, 4.25, 4.65, 6.00),   # divider, upper leaf -> 0.85 m doorway
            (2.80, 0.60, 3.40, 2.90),   # pillar occluding the doorway on approach
            (5.20, 4.70, 6.20, 5.60),   # obstacle just beyond the far side
        ],
    )
    return MicroScenario(
        id="doorway_traverse",
        world=world,
        # Start yaw is off the goal bearing, so heading error is still being
        # worked off while the doorway comes into view.
        start=(1.2, 3.0, -0.6),
        goal=(7.8, 4.65),
        time_limit=60.0,
        seed=seed,
        goal_known=True,
    )
