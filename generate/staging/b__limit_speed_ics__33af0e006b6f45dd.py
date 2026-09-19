import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status


class LimitSpeedToStoppableSeenDistance(Action):
    """Caps linear speed at what can still be braked to a stop inside seen-free space.

    Implements the inevitable-collision-state (ICS) argument: a state can be
    collision-free and still doomed, because stopping needs room. Safety is
    therefore decided by comparing braking distance against free distance. With no
    map, the honest boundary of "seen to be free" is not just the finite lidar
    returns but also the field-of-view boundary -- beams that read `inf` (unseen
    space starts at `range_max`) and occlusion shadows behind depth
    discontinuities (unseen space starts at the near side of the jump). Both are
    treated as obstacles, so the robot may only travel as fast as it can stop
    inside the region it has actually observed to be empty.

    Revises `vx` and `vy` only, and only downward: both linear components are
    scaled by one factor so the base's chosen direction of travel is preserved
    exactly. `wz` is never touched -- this node is a speed cap, not a detour, so
    the base keeps full authority over steering and can continue whatever avoidance
    or goal-seeking turn it was making, just slower.

    Declines when the base commanded no linear motion, or when no usable scan
    exists this step. Passes the command through byte-identical whenever the
    stoppable speed is at or above what the base asked for, which is the normal
    case in open space -- the cap only binds when the seen-free corridor ahead is
    short.

    Reads no pose and no goal, so it is unaffected by `pose_estimate` drift and
    remains valid on scenarios that withhold the goal.
    """

    WRITES = ("vx", "vy")

    PORTS = (
        # The stoppable speed is a limit other nodes may legitimately want to
        # respect (e.g. before committing to a manoeuvre), so it is published.
        Port("stoppable_speed", PortType.SPEED, Direction.OUT),
    )

    PRECONDITION = (
        "The base proposed a non-zero linear velocity and a lidar scan exists "
        "for this control step."
    )
    EFFECT = (
        "Scales the commanded linear velocity down, direction preserved, to the "
        "fastest speed from which the robot can still brake to a stop inside the "
        "corridor it has seen to be free."
    )
    TAGS = ("safety", "speed-limit", "ics", "braking", "occlusion", "lidar")

    # Geometry and dynamics constants (see derivation).
    L_FRONT = 0.35   # m, base origin to front of body plus a 0.05 m skin
    TAU = 0.20       # s, command-to-deceleration lag through the frozen policy
    A_BRAKE = 1.0    # m/s^2, pessimistic effective deceleration when commanded to zero
    W = 0.25         # m, corridor half-width: body half-width plus yaw wobble
    D_JUMP = 0.5     # m, depth discontinuity that opens an occlusion shadow
    V_MAX = 0.8      # m/s, runner saturation on linear speed
    V_EPS = 1e-3     # m/s, below this the base is effectively not translating

    def precondition(self, obs, ports) -> bool:
        pending = ports.command
        if pending is None:
            return False
        if math.hypot(pending.vx, pending.vy) <= self.V_EPS:
            return False
        ranges = getattr(obs, "ranges", None)
        angles = getattr(obs, "range_angles", None)
        if ranges is None or angles is None:
            return False
        return len(ranges) > 0 and len(ranges) == len(angles)

    def tick(self, obs, ports) -> Status:
        pending = ports.command
        vx_c, vy_c = float(pending.vx), float(pending.vy)
        v_c = math.hypot(vx_c, vy_c)
        theta_c = math.atan2(vy_c, vx_c)

        r_max = float(getattr(obs, "range_max", 6.0))
        d_free = self._seen_free_distance(obs, theta_c, r_max)
        v_cap = self._stoppable_speed(d_free)

        ports["stoppable_speed"] = v_cap

        # Open space: the base is already within what it can brake from, so leave
        # its command untouched rather than re-writing it with the same numbers.
        if v_cap >= v_c:
            return Status.SUCCESS

        scale = v_cap / v_c
        # cap() keeps the sign of the pending field, so capping each component at
        # its own scaled magnitude is exactly a uniform scaling of the vector.
        ports.cap(vx=abs(vx_c) * scale, vy=abs(vy_c) * scale)
        return Status.SUCCESS

    def _seen_free_distance(self, obs, theta_c: float, r_max: float) -> float:
        """Distance along the direction of travel to the near edge of unseen space."""
        angles = np.asarray(obs.range_angles, dtype=float)
        raw = np.asarray(obs.ranges, dtype=float)

        # A beam with no return has not seen anything to be free beyond the sensing
        # horizon, so the horizon itself is the obstacle (the paper's FoV_B arc).
        r = np.where(np.isfinite(raw), raw, r_max)
        r = np.clip(r, 0.0, r_max)

        # Occlusion shadows: at a depth jump, the space behind the near edge is
        # unobserved. Place a virtual point at the far beam's bearing and the near
        # beam's range -- the mouth of the shadow.
        dr = r[1:] - r[:-1]
        jump = np.abs(dr) > self.D_JUMP
        if np.any(jump):
            shadow_a = np.where(dr > 0.0, angles[1:], angles[:-1])[jump]
            shadow_r = np.minimum(r[:-1], r[1:])[jump]
            all_a = np.concatenate((angles, shadow_a))
            all_r = np.concatenate((r, shadow_r))
        else:
            all_a, all_r = angles, r

        # Project into the travel frame: s ahead, l lateral.
        delta = all_a - theta_c
        s = all_r * np.cos(delta)
        lat = all_r * np.sin(delta)

        in_corridor = (s > 0.0) & (np.abs(lat) <= self.W)
        if not np.any(in_corridor):
            return r_max
        return float(min(np.min(s[in_corridor]), r_max))

    def _stoppable_speed(self, d_free: float) -> float:
        """Invert d_stop(v) = L_FRONT + v*TAU + v^2/(2*A_BRAKE) for v."""
        room = max(0.0, d_free - self.L_FRONT)
        at = self.A_BRAKE * self.TAU
        v = -at + math.sqrt(at * at + 2.0 * self.A_BRAKE * room)
        return float(min(max(v, 0.0), self.V_MAX))


PREDICTION = {
    "statement": (
        "Wrapping the linear velocity command with this cap reduces the mean "
        "number of collisions per episode by at least 30% relative to the "
        "unwrapped baseline, over the same set of seeds and scenarios, because "
        "late-detected obstructions are met at a speed from which a stop still "
        "fits inside the seen-free corridor."
    ),
    "metric": "collisions",
    "baseline": "goal_seeker",
    "min_effect": 0.3,
    "cost_metric": "time_to_goal",
}
