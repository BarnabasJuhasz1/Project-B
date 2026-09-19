"""Speed cap derived from inevitable-collision-state (ICS) reasoning.

Prediction statement, verbatim from the principle card:

    Wrapping the linear velocity command with this cap reduces the mean number
    of collisions per episode by at least 30% relative to the unwrapped
    baseline, over the same set of seeds and scenarios, because late-detected
    obstructions are met at a speed from which a stop still fits inside the
    seen-free corridor.
"""

import math

import numpy as np

from nodes.contract import Decorator, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


PREDICTION = {
    "metric": "collisions",
    "baseline": "goal_seeker",
    "min_effect": 0.3,
    "cost_metric": "time_to_goal",
}


class LimitSpeedToStoppableSeenDistance(Decorator):
    """Hold the child's linear speed at or below the speed from which the robot
    can still brake to a stop inside the space this scan has shown to be free.

    A collision-free state is not the same as a safe state: if stopping needs
    distance d, every state within d of an obstacle is one from which collision
    is already unavoidable. Safety is therefore a comparison of braking distance
    against free distance, and with no map the free distance must be measured
    from the scan alone.

    The scan's unseen boundary is treated as if it were solid, in two parts:
    beams returning inf, whose unseen space starts at range_max, and the
    occlusion shadows behind depth discontinuities, whose unseen space starts at
    the near side of the jump. An obstruction that has not been seen yet can
    only first appear on one of those two boundaries, so a robot that can stop
    short of them can stop short of anything.

    The test is one-input: "can I still stop in a straight line". That is
    sufficient rather than merely necessary here because the Go2 can hold a
    stand, so a stopped, non-touching state is never itself doomed.

    Enforcement, given that a Decorator may not emit a Twist of its own:
      * the cap is published on the OUT SPEED port every step, for a child that
        is willing to scale itself to it;
      * for a child that is not, the child's tick is released on a duty cycle of
        v_cap / V_MAX. Gating time-scales a kinematic rollout, so the child's
        path geometry is untouched and only its mean speed falls to v_cap.
    """

    PORTS = (Port("stoppable_speed", PortType.SPEED, Direction.OUT),)

    PRECONDITION = (
        "A lidar scan exists for this control step, so the distance that has "
        "been seen to be free in the direction of travel can be measured."
    )
    EFFECT = (
        "The linear speed the child is allowed to command is held at or below "
        "the speed from which the robot can brake to a stop inside the "
        "seen-free corridor ahead of it."
    )
    TAGS = (
        "safety",
        "inevitable-collision-state",
        "speed-cap",
        "braking-distance",
        "occlusion",
        "lidar",
        "mapless",
    )

    # Corridor half-width: Go2 body half-width ~0.16 m, plus 0.09 m for yaw
    # wobble and the frozen policy's lateral slop.
    W = 0.25
    # A range step larger than this between adjacent beams is read as an
    # occlusion edge rather than a surface, and its shadow mouth is treated as
    # the start of unseen space.
    D_JUMP = 0.5
    # Base origin to the front of the body, plus a 0.05 m skin.
    L_FRONT = 0.35
    # Command-to-deceleration lag through the frozen policy: two control steps.
    TAU = 0.20
    # Deliberately pessimistic effective deceleration for a trotting quadruped
    # commanded to zero. If the true figure is higher the cap is merely
    # conservative, which is the safe direction to be wrong in.
    A_BRAKE = 1.0
    # The runner's own linear saturation, i.e. the fastest the child could
    # possibly be asking for.
    V_MAX = 0.8
    # A Decorator cannot see the child's Twist, so the direction of travel is
    # taken as body-forward, where a goal-seeking linear command overwhelmingly
    # sits. The corridor projection below is written for a general bearing.
    TRAVEL_BEARING = 0.0

    DT_DEFAULT = 0.1

    _credit = 0.0
    _last_t = None

    def reset(self, scenario, rng):
        self._credit = 0.0
        self._last_t = None

    def precondition(self, obs, ports):
        ranges = obs.ranges
        return ranges is not None and len(ranges) > 1

    def tick(self, obs, ports):
        dt = self._control_period(obs)
        d_free = self._seen_free_distance(obs, self.TRAVEL_BEARING)
        v_cap = self._stoppable_speed(d_free)
        ports["stoppable_speed"] = float(v_cap)

        # A released step is a whole control period at whatever the child asks
        # for, up to the runner's saturation. Refuse to release one that could
        # end inside the front margin: the duty cycle governs mean speed, not
        # the size of a single step, and without this floor the accumulator
        # eventually spends its credit on one full-speed lunge at close range.
        if d_free < self.L_FRONT + self.V_MAX * dt:
            self._credit = 0.0
            return Status.RUNNING

        self._credit += v_cap / self.V_MAX
        if self._credit < 1.0:
            return Status.RUNNING

        self._credit -= 1.0
        return self.child.tick(obs, ports)

    def _control_period(self, obs):
        """Measure the control period from the clock instead of assuming it."""
        dt = self.DT_DEFAULT
        if self._last_t is not None:
            measured = obs.t - self._last_t
            if 0.0 < measured < 0.5:
                dt = measured
        self._last_t = obs.t
        return dt

    def _stoppable_speed(self, d_free):
        """Invert d_stop(v) = L_FRONT + v*TAU + v^2/(2*A_BRAKE) for v."""
        room_to_stop = max(0.0, d_free - self.L_FRONT)
        at = self.A_BRAKE * self.TAU
        v = -at + math.sqrt(at * at + 2.0 * self.A_BRAKE * room_to_stop)
        return float(np.clip(v, 0.0, self.V_MAX))

    def _seen_free_distance(self, obs, theta_c):
        """Distance to the nearest seen-or-unseen boundary in the travel corridor.

        Both kinds of boundary are put into one point set: real returns, inf
        beams pulled in to range_max, and one virtual point per depth
        discontinuity placed at the shadow mouth (bearing of the far beam,
        range of the near one).
        """
        r_max = float(obs.range_max)
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        r = np.where(np.isfinite(ranges), ranges, r_max)
        r = np.clip(r, 0.0, r_max)

        # The scan closes a full circle, so the last/first pair is adjacent too.
        r_next = np.roll(r, -1)
        a_next = np.roll(angles, -1)
        jump = np.abs(r_next - r) > self.D_JUMP
        far_is_next = r_next > r
        shadow_a = np.where(far_is_next, a_next, angles)[jump]
        shadow_r = np.where(far_is_next, r, r_next)[jump]

        pt_a = np.concatenate((angles, shadow_a))
        pt_r = np.concatenate((r, shadow_r))

        rel = pt_a - theta_c
        s = pt_r * np.cos(rel)
        lateral = pt_r * np.sin(rel)
        in_corridor = (s > 0.0) & (np.abs(lateral) <= self.W)
        if not np.any(in_corridor):
            return r_max
        return float(min(np.min(s[in_corridor]), r_max))


def micro(seed=0):
    """A slab lying square across the straight line from start to goal.

    goal_seeker has no sensing and drives into it at full speed, which is the
    baseline failure the card is about: the obstruction is met at a speed from
    which no stop fits. The route around it is open at both ends, so the goal is
    reachable and the world is not a dead end.
    """
    world = room(8.0, 6.0, boxes=[(3.6, 1.3, 4.4, 4.7)])
    return MicroScenario(
        id="slab_across_the_straight_line",
        world=world,
        start=(1.0, 3.0, 0.0),
        goal=(7.0, 3.0),
        time_limit=40.0,
        seed=seed,
        goal_known=True,
    )
