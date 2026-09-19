import math

import numpy as np

from nodes.contract import Action, Status


class SpiralSearchGoalUncertaintyDisc(Action):
    """Terminal uncertainty-scaled spiral search over the disc around the believed goal.

    Mechanism (pi_terminal_uncertainty_search): a dead-reckoned goal vector carries a
    positional error that grows monotonically along the path. Its usefulness is relative,
    not absolute: the bearing error it induces is ~atan(e/d), so far from the goal a large
    positional error still yields a good heading, while at d ~ e the heading carries under
    one bit of information about where the goal actually is. At that crossover this node
    abandons vector servoing and converts the unobservable directional error into swept
    area: an Archimedean spiral outward from the believed goal, with loop spacing equal to
    twice the arrival tolerance so the swept band tiles the disc without gaps.

    Revises `vx` and `wz`. Both are needed because the spiral law ties curvature to forward
    speed (wz = vx / r); capping vx alone would leave the commanded curvature inconsistent
    with the speed actually driven and the loops would no longer tile. `vy` is left to the
    base.

    Declines (writes nothing, base command passes through unchanged) when: the goal has not
    been disclosed; the pose belief is trustworthy at the scale of the arrival tolerance
    (pose_error_estimate <= R_TOL) -- so on non-drifting scenarios it costs nothing; the
    believed remaining distance is still large compared with the believed error; the search
    budget (radius or time) is spent; the two permitted entries are used up; or the front
    sector is obstructed, in which case it defers to whatever the base commanded for that
    tick and freezes the spiral phase.
    """

    WRITES = ("vx", "wz")
    PORTS = ()
    PRECONDITION = (
        "The goal is disclosed, the pose belief is untrustworthy at the scale of the "
        "arrival tolerance, and the believed remaining distance has fallen inside the "
        "believed positional error."
    )
    EFFECT = (
        "Replaces terminal vector-servoing with a dead-reckoned Archimedean spiral "
        "outward from the believed goal, sweeping the disc the accumulated error makes "
        "plausible."
    )
    TAGS = ("path-integration", "search", "spiral", "pose-drift", "terminal", "modifier")

    # Assumed scorer arrival radius; also floors the trigger so the leaf still fires when
    # the drift estimate is small but nonzero.
    R_TOL = 0.5
    # Crossover ratio: trigger at d_belief <= K * e, i.e. ~45 deg of bearing uncertainty.
    K = 1.0

    R0 = 0.5                  # starting spiral radius; any tighter saturates wz at vx=0.5
    PITCH = 2.0 * R_TOL       # radial gain per turn, so the r_tol-wide band tiles the disc
    VX = 0.5                  # below the 0.8 cap, leaving margin for the base's avoidance
    WZ_MAX = 1.0

    R_MAX_GAIN = 2.5          # ~95% of a 2-D random walk with 1-sigma radius e0
    R_MAX_CAP = 4.0           # keeps the swept disc inside range_max = 6.0 and bounds cost
    T_BUDGET = 60.0           # second exit, in sim seconds since entry
    MAX_ENTRIES = 2           # one re-entry after a failed sweep, then suppressed

    FRONT_HALF_ANGLE = math.radians(35.0)
    FRONT_CLEARANCE = 0.6     # m; below this we hand the tick back to the base

    def reset(self, scenario, rng):
        self._active = False
        self._entries = 0
        self._phi = 0.0
        self._centre = None
        self._r_max = 0.0
        self._t_entry = 0.0
        self._t_last = None

    def _state(self):
        # Tolerate being ticked without a prior reset().
        if not hasattr(self, "_active"):
            self.reset(None, None)

    @staticmethod
    def _belief_distance_to_goal(obs):
        gx, gy = float(obs.goal[0]), float(obs.goal[1])
        px, py = float(obs.pose_estimate[0]), float(obs.pose_estimate[1])
        return math.hypot(gx - px, gy - py)

    def precondition(self, obs, ports) -> bool:
        self._state()

        if obs.goal is None:
            return False

        e = float(obs.pose_error_estimate)
        if not math.isfinite(e) or e <= self.R_TOL:
            # Belief is trustworthy at the scale that matters: vector guidance is valid,
            # and this also terminates a sweep in progress if the estimate recovers.
            self._active = False
            return False

        if self._active:
            return True

        if self._entries >= self.MAX_ENTRIES:
            return False

        d = self._belief_distance_to_goal(obs)
        if not math.isfinite(d):
            return False
        return d <= max(self.R_TOL, self.K * e)

    def _front_blocked(self, obs) -> bool:
        if bool(obs.contact):
            return True
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        n = min(ranges.size, angles.size)
        if n == 0:
            return False
        ranges = ranges[:n]
        angles = angles[:n]
        # Wrap to (-pi, pi] before selecting the forward sector: beam bearings may be
        # published over [0, 2pi).
        wrapped = np.arctan2(np.sin(angles), np.cos(angles))
        front = ranges[np.abs(wrapped) <= self.FRONT_HALF_ANGLE]
        front = front[np.isfinite(front)]
        if front.size == 0:
            return False
        return float(front.min()) < self.FRONT_CLEARANCE

    def tick(self, obs, ports) -> Status:
        self._state()

        t = float(obs.t)
        dt = 0.0 if self._t_last is None else t - self._t_last
        # Guard against episode restarts and long stalls; dt only ever advances phase.
        if not math.isfinite(dt) or dt < 0.0:
            dt = 0.0
        dt = min(dt, 0.2)
        self._t_last = t

        if not self._active:
            # Entry: latch the spiral centre as the goal in the belief frame (goal and
            # pose_estimate share that frame) and latch the error we are searching for.
            e0 = float(obs.pose_error_estimate)
            self._centre = (float(obs.goal[0]), float(obs.goal[1]))
            self._r_max = min(self.R_MAX_GAIN * e0, self.R_MAX_CAP)
            self._phi = 0.0
            self._t_entry = t
            self._entries += 1
            self._active = True
            dt = 0.0

        if t - self._t_entry > self.T_BUDGET:
            self._active = False
            return Status.FAILURE

        r = self.R0 + (self.PITCH / (2.0 * math.pi)) * self._phi
        if r > self._r_max:
            # Radius budget spent. Deactivating here lets the base drive again; if the
            # trigger still holds later, the node re-enters once with a freshly latched
            # centre, and is suppressed after that.
            self._active = False
            return Status.FAILURE

        if self._front_blocked(obs):
            # Defer to the base's obstacle handling for this tick and freeze the phase, so
            # the detour consumes time but does not corrupt the swept geometry.
            return Status.RUNNING

        wz = self.VX / max(r, 1e-3)
        wz = min(wz, self.WZ_MAX)

        # Phase is integrated from the COMMANDED twist, not from pose_estimate deltas: the
        # search is itself dead-reckoned and so is immune to further drift in the belief.
        # Using the clamped wz keeps the integration honest if it ever saturates.
        self._phi += wz * dt

        ports.adjust(vx=self.VX, wz=wz)
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.15,
    "cost_metric": "time_to_goal",
}
