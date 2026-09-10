"""Path-integration homing for a mapless indoor goal-reaching task.

Mechanism after Mueller & Wehner (1988), path integration in Cataglyphis fortis.
"""

import math

import numpy as np

from core.observation import Command
from harness.micro import MicroScenario, room
from nodes.contract import Action, Direction, Port, PortType, Status

# Candidate steering offsets from the goal vector, ordered by magnitude so that
# "smallest deflection that is free" is just a first-match scan.
_OFFSETS = tuple(
    [0.0] + [s * math.radians(d) for d in range(10, 180, 10) for s in (1.0, -1.0)]
)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class PathIntegrationHoming(Action):
    """Hold an egocentric goal vector by dead reckoning and run it off.

    The goal is converted to the robot frame exactly once, when it is first
    disclosed. From then on the vector is updated only from the robot's own
    commanded self-motion -- translate the vector back by the step taken, rotate
    it back by the yaw turned -- and the absolute pose belief is never read
    again. This is the desert-ant arrangement: a single accumulator fed by
    compass and odometer, carrying no map and taking no positional fix.

    Two consequences, and they are the reason to prefer this over reading
    obs.goal against obs.pose_estimate every tick:

    * Drift immunity. The runner adds a seeded random walk to pose_estimate on
      some scenarios. A controller that recomputes goal - pose each tick inherits
      that walk in full; an accumulator seeded once inherits only the error
      present at seeding time plus its own integration error.
    * Detour tolerance. When the direct line is blocked the node deflects to the
      smallest heading the scan says is free, and the vector keeps turning under
      it correctly, so the robot re-converges on the goal the moment the way
      opens. Turning the vector by the same increment as the body is the exact
      version of the update whose approximation Mueller & Wehner inferred from
      the ants' systematic homing errors; the 1/length sensitivity to late turns
      falls out of the geometry rather than being imposed.

    The failure mode is slip: integration assumes commanded twist equals executed
    motion, which holds for the kinematic rollout right up until the robot is in
    contact and is being pushed rather than walking. Hence the clearance margins
    below -- they are there to keep contact rare, not just to avoid collisions.
    """

    PORTS = (Port("goal_bearing", PortType.BEARING, Direction.OUT),)
    PRECONDITION = (
        "The goal position has been disclosed at least once, so an egocentric "
        "goal vector exists to accumulate."
    )
    EFFECT = (
        "Reduces the dead-reckoned distance from the robot to the goal until the "
        "goal is reached."
    )
    TAGS = ("path-integration", "dead-reckoning", "homing", "goal-vector", "drift-tolerant")

    # All constants below are engineering judgement, not values from the paper.
    V_MAX = 0.55            # m/s, below the 0.8 saturation to leave slip headroom
    W_MAX = 1.0             # rad/s, the runner's own cap
    K_YAW = 1.6             # heading P gain
    TURN_ONLY = 1.0         # rad of heading error above which we stop translating
    ARRIVE = 0.20           # m of dead-reckoned range at which we call it done
    LOOKAHEAD = 1.1         # m of clear corridor a candidate heading must offer
    HALF_WIDTH = 0.42       # m, robot radius 0.3 plus margin
    STOP_MARGIN = 0.50      # m, free distance held in reserve ahead
    HYST = 0.25             # rad bonus for keeping the side we already committed to
    MAX_DT = 0.25           # s, clamp on an integration step

    def _clear(self):
        self._vec = None          # goal vector in the CURRENT robot frame, metres
        self._t_prev = None
        self._last_cmd = (0.0, 0.0, 0.0)
        self._side = 0.0          # sign of the deflection we are committed to

    def reset(self, scenario, rng):
        self._clear()

    def precondition(self, obs, ports):
        return self._vec is not None or obs.goal is not None

    def tick(self, obs, ports):
        # Tolerate a tick that arrives before any reset.
        if not hasattr(self, "_vec"):
            self._clear()

        dt = 0.0 if self._t_prev is None else obs.t - self._t_prev
        self._t_prev = obs.t
        self._integrate(min(max(dt, 0.0), self.MAX_DT))

        if self._vec is None:
            if obs.goal is None:
                # Goal not disclosed yet: nothing to seed the accumulator with.
                # Emit nothing, so whatever runs next owns the motion.
                self._last_cmd = (0.0, 0.0, 0.0)
                return Status.FAILURE
            self._vec = self._latch(obs)

        gx, gy = self._vec
        dist = math.hypot(gx, gy)
        bearing = math.atan2(gy, gx)
        ports["goal_bearing"] = bearing

        if dist <= self.ARRIVE:
            self._last_cmd = (0.0, 0.0, 0.0)
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        heading = self._choose_heading(obs, bearing, dist)
        err = _wrap(heading)

        if obs.contact:
            # Pushed, not walking: back off before the odometer accumulates a
            # step the body never took.
            vx = -0.30
            wz = self.K_YAW * err if abs(err) > 0.1 else 0.8 * (self._side or 1.0)
        else:
            vx = 0.0 if abs(err) > self.TURN_ONLY else self.V_MAX * max(0.0, math.cos(err))
            vx = min(vx, 0.9 * dist)  # ease off as the vector runs out
            free = self._free_distance(obs, heading, self.HALF_WIDTH)
            vx = min(vx, max(0.0, free - self.STOP_MARGIN))
            wz = self.K_YAW * err

        vx = float(np.clip(vx, -self.V_MAX, self.V_MAX))
        wz = float(np.clip(wz, -self.W_MAX, self.W_MAX))

        # Store exactly what we emit: the integrator is only as good as its
        # agreement with the command the runner actually executes.
        self._last_cmd = (vx, 0.0, wz)
        ports.drive(Command(vx, 0.0, wz))
        return Status.RUNNING

    def _latch(self, obs):
        """Seed the accumulator: world goal -> robot frame, once and only once."""
        gx, gy = obs.goal
        x, y, yaw = obs.pose_estimate
        dx, dy = gx - x, gy - y
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy)

    def _integrate(self, dt):
        """Carry the goal vector through the motion just commanded.

        First-order Euler over one control step; at these dt the difference from
        the exact SE(2) increment is well under the slip we are already exposed
        to at contact.
        """
        if self._vec is None or dt <= 0.0:
            return
        vx, vy, wz = self._last_cmd
        dx = self._vec[0] - vx * dt
        dy = self._vec[1] - vy * dt
        a = -wz * dt
        ca, sa = math.cos(a), math.sin(a)
        self._vec = (dx * ca - dy * sa, dx * sa + dy * ca)

    def _free_distance(self, obs, theta, half_width):
        """Clear distance down a corridor of the robot's width about heading theta."""
        r = np.asarray(obs.ranges, dtype=float)
        r = np.where(np.isfinite(r), r, obs.range_max)
        d = np.asarray(obs.range_angles, dtype=float) - theta
        rel = np.arctan2(np.sin(d), np.cos(d))
        lon = r * np.cos(rel)
        lat = r * np.sin(rel)
        hit = (lon > 0.0) & (np.abs(lat) < half_width)
        if not np.any(hit):
            return float(obs.range_max)
        return float(np.min(lon[hit]))

    def _choose_heading(self, obs, bearing, dist):
        """Smallest deflection off the goal vector that the scan says is free."""
        # Do not demand a metre of clearance beyond a goal that is half a metre
        # away, or the node would detour around its own destination's far wall.
        need = min(self.LOOKAHEAD, max(dist - 0.15, 0.30))

        best_off, best_score = None, None
        fallback_off, fallback_free = 0.0, -1.0
        for off in _OFFSETS:
            cand = _wrap(bearing + off)
            free = self._free_distance(obs, cand, self.HALF_WIDTH)
            if free > fallback_free:
                fallback_off, fallback_free = off, free
            if free < need:
                continue
            score = abs(off)
            if self._side != 0.0 and off * self._side > 0.0:
                score -= self.HYST  # keep rounding the obstacle the way we started
            if best_score is None or score < best_score:
                best_off, best_score = off, score

        if best_off is None:
            # Boxed in: face the most open direction available and re-evaluate.
            best_off = fallback_off

        if abs(best_off) > 0.2:
            self._side = math.copysign(1.0, best_off)
        else:
            self._side = 0.0  # vector is clear again; drop the commitment
        return _wrap(bearing + best_off)


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    # Running the vector off around a barrier is strictly longer than the
    # straight line the baseline attempts and fails to complete.
    "cost_metric": "time_to_goal",
}


def micro(seed=0):
    """A barrier that makes the straight line a dead end.

    The start-to-goal chord crosses the barrier at y = 3.0, well inside it, so
    goal_seeker walks into it and stays there: no sensing, no detour, failure.
    The only way through is the 1.7 m gap at the top, which is off the goal
    bearing -- so a robot must leave the goal direction, travel roughly 3 m with
    the goal behind an obstruction, and then recover the direction. That
    recovery is what the accumulated vector supplies.
    """
    world = room(8.0, 6.0, boxes=[(3.8, 0.0, 4.2, 4.3)])
    return MicroScenario(
        id="pi_homing_barrier_detour",
        world=world,
        start=(1.0, 1.0, 0.0),
        goal=(7.0, 5.0),
        time_limit=60.0,
        seed=seed,
        goal_known=True,
    )
