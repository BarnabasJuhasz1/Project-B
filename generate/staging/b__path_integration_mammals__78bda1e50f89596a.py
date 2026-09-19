import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status


class SpiralSearchAtBelievedGoal(Action):
    """Terminal spiral search around the believed goal (pi_terminal_spiral_search).

    A dead-reckoned pose belief is good enough to steer a long approach but not
    to terminate one: its error grows without bound, so the point at which the
    residual goal vector reads zero is not the goal. Path-integrating animals
    handle this by switching mode near the believed nest -- from a linear
    homeward run to circular movements that sweep the neighbourhood -- turning
    an unbounded point-accuracy requirement into a bounded area-coverage one.

    This node implements that switch. Once ||goal - pose_estimate[:2]|| falls to
    the switch radius, it revises `vx` and `wz` to drive an Archimedean spiral
    outward from the believed goal: constant 0.35 m/s forward with
    wz = v / r(phi), r growing 0.5 m per turn. Both fields are needed because a
    spiral is a speed and a turn rate; neither can be expressed as a bound on
    what the base asked for, so both are written with `adjust`.

    It declines (writes nothing, base passes through unchanged) whenever the
    goal is undisclosed, the residual is still large, or the per-episode search
    budget is spent. It also yields mid-search -- returning RUNNING without
    writing, phase retained -- on contact or when something is inside 0.4 m
    ahead, because obstacle handling belongs to another node.
    """

    WRITES = ("vx", "wz")

    PORTS = (
        Port("search_region", PortType.REGION, Direction.OUT),
    )

    PRECONDITION = (
        "The goal has been disclosed and the residual goal vector in the belief "
        "frame has shrunk to within the drift-sized switch radius, with this "
        "episode's search budget not yet spent."
    )
    EFFECT = (
        "Replaces the commanded forward speed and turn rate with an outward "
        "Archimedean spiral about the believed goal, sweeping its neighbourhood "
        "instead of halting where the integrator reads zero."
    )
    TAGS = ("path-integration", "terminal-search", "spiral", "localisation-drift")

    # --- numbers: all from the card's derivation, none from the scenario ---
    SEARCH_SPEED = 0.35        # m/s; with |wz| <= 1.0 this forces r0 >= 0.35 m
    R0 = 0.35                  # m, tightest circle the actuator envelope allows
    PITCH_M = 0.5              # m of radius gained per full turn
    SWITCH_GAIN = 1.5          # covers the tail of a drift of scale pose_error
    SWITCH_MIN_M = 0.7         # one body length; below this a spiral is meaningless
    SWITCH_MAX_M = 3.0         # half range_max, so the sweep stays in sensed space
    BUDGET_S = 60.0            # ~21 m of path at 0.35 m/s
    YIELD_CLEARANCE_M = 0.4    # hand over to avoidance inside this
    YIELD_HALF_ANGLE = math.radians(30.0)
    MAX_DT = 0.2               # guard against a stalled or jumped clock

    # Per-episode state. Declared at class scope so a tick before the first
    # reset() still sees sane values rather than an AttributeError.
    _active = False
    _done = False
    _phi = 0.0
    _spent = 0.0
    _last_t = None
    _center = (0.0, 0.0)
    _r_switch = SWITCH_MIN_M

    def reset(self, scenario, rng):
        self._active = False
        self._done = False
        self._phi = 0.0
        self._spent = 0.0
        self._last_t = None
        self._center = (0.0, 0.0)
        self._r_switch = self.SWITCH_MIN_M

    # ------------------------------------------------------------------ utils

    def _switch_radius(self, obs):
        """Size the search from how wrong we think the belief is."""
        err = obs.pose_error_estimate
        try:
            err = float(err)
        except (TypeError, ValueError):
            err = 0.0
        if not math.isfinite(err) or err < 0.0:
            err = 0.0
        return float(np.clip(self.SWITCH_GAIN * err, self.SWITCH_MIN_M, self.SWITCH_MAX_M))

    def _residual(self, obs):
        goal = obs.goal
        pose = obs.pose_estimate
        return math.hypot(float(goal[0]) - float(pose[0]), float(goal[1]) - float(pose[1]))

    def _spiral_radius(self):
        return self.R0 + self.PITCH_M * self._phi / (2.0 * math.pi)

    def _front_clearance(self, obs):
        """Closest return within +/-30 deg of the current heading, inf if clear."""
        angles = np.asarray(obs.range_angles, dtype=float)
        ranges = np.asarray(obs.ranges, dtype=float)
        # Beam bearings are robot-frame but not necessarily wrapped to [-pi, pi).
        wrapped = np.arctan2(np.sin(angles), np.cos(angles))
        sector = np.abs(wrapped) <= self.YIELD_HALF_ANGLE
        valid = sector & np.isfinite(ranges)
        if not np.any(valid):
            return float("inf")
        return float(np.min(ranges[valid]))

    def _tick_clock(self, obs):
        t = float(obs.t)
        prev = self._last_t
        self._last_t = t
        if prev is None:
            return 0.0
        return float(np.clip(t - prev, 0.0, self.MAX_DT))

    # -------------------------------------------------------------- contract

    def precondition(self, obs, ports):
        if self._done or obs.goal is None:
            return False
        if self._active:
            return True
        return self._residual(obs) <= self._switch_radius(obs)

    def tick(self, obs, ports):
        if self._done or obs.goal is None:
            return Status.FAILURE

        dt = self._tick_clock(obs)

        if not self._active:
            # Freeze the centre and the search extent at the moment of the mode
            # switch: the belief keeps drifting, but re-centring mid-search would
            # smear the swept disc and leave gaps.
            self._active = True
            self._phi = 0.0
            self._spent = 0.0
            self._r_switch = self._switch_radius(obs)
            pose = obs.pose_estimate
            self._center = (float(pose[0]), float(pose[1]))

        if obs.contact or self._front_clearance(obs) < self.YIELD_CLEARANCE_M:
            # Write nothing: the base's command passes through untouched and the
            # avoidance branch owns this tick. Phase is kept, and the budget is
            # not charged for arc we did not sweep.
            return Status.RUNNING

        self._spent += dt
        r = self._spiral_radius()

        if r >= self._r_switch or self._spent >= self.BUDGET_S:
            # Budget is per-episode: hand back for good rather than re-triggering
            # each time the drifting belief wanders back inside the switch radius.
            self._active = False
            self._done = True
            return Status.SUCCESS

        ports.adjust(
            vx=self.SEARCH_SPEED,
            wz=float(np.clip(self.SEARCH_SPEED / r, -1.0, 1.0)),
        )
        ports["search_region"] = (self._center[0], self._center[1], r)

        self._phi += (self.SEARCH_SPEED / r) * dt
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}
