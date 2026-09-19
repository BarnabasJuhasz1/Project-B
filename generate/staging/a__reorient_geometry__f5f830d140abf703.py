import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status


class ReorientToBoundaryGeometry(Action):
    """Re-anchors heading to the metric shape of the surrounding surface layout.

    Implements geometric boundary reorientation: when the robot's own pose-error
    estimate says the belief has gone bad, the principal axis of the visible
    range cloud is compared against the axis recorded while the belief was still
    trusted, and the difference -- reduced modulo the 90-degree symmetry of a
    rectilinear indoor layout -- is the accumulated heading error. Walls subtend
    many beams and furniture subtends few, so the second-moment axis is a
    wall-dominated, non-drifting heading reference.

    The card's effect is a write to the pose belief, which this pipeline cannot
    express. Its exact image on the Twist is used instead: a belief edit of +d
    shifts the base's robot-frame goal bearing by -d on every subsequent tick, so
    the correction is applied as a sustained bias on ``wz`` rather than a
    one-shot turn (turning the body moves belief and truth together and corrects
    nothing). ``vx`` is capped to zero for the 0.5 s measurement window because
    the derivation asks for a translation freeze while the axis is averaged --
    standing still is cheaper than modelling gait-induced body sway.

    Declines, leaving the base's command untouched, when: the belief is trusted,
    no reference axis has been laid down yet, the view is too open (<40 finite
    returns) or too rotationally symmetric (eigenvalue ratio >= 0.6) to define an
    axis, fewer than 5 s have passed since the last attempt, the residual lands
    within 5 degrees of the +/-45-degree symmetry boundary where the assignment
    between adjacent symmetry copies is a coin flip, or an obstacle is close
    enough ahead that the base is plainly doing local avoidance.
    """

    WRITES = ("vx", "wz")
    PORTS = (Port("boundary_yaw_correction", PortType.BEARING, Direction.OUT),)
    PRECONDITION = (
        "The pose-error estimate exceeds 0.5 m while a boundary axis is usable, "
        "or a boundary-referenced heading correction is currently being applied, "
        "or the belief is trusted enough to refresh the reference axis."
    )
    EFFECT = (
        "Re-anchors heading to the remembered wall-axis frame by freezing "
        "translation for 0.5 s to measure the axis and then biasing the base's "
        "yaw rate by the accepted correction."
    )
    TAGS = ("reorientation", "boundary", "geometry", "drift", "heading", "yaw")

    # Trigger / trust thresholds (metres of believed pose error).
    _ERR_TRIGGER = 0.5
    _ERR_TRUST = 0.25

    # Axis admissibility.
    _MIN_FINITE = 40
    _MAX_EIG_RATIO = 0.6

    # Timing (seconds).
    _REF_TAU = 2.0
    _FREEZE = 0.5
    _COOLDOWN = 5.0
    _BIAS_TAU = 6.0

    # Symmetry refusal: |d| is capped at 45 deg by construction; within 5 deg of
    # that boundary the symmetry-copy assignment is a coin flip.
    _QUARTER = math.pi / 2.0
    _REFUSE_MARGIN = math.radians(5.0)

    # Correction bias. The base's steering gain is unknown, so the gain is fixed
    # and hard-clipped; the base's own feedback bounds the resulting offset.
    _BIAS_GAIN = 1.0
    _BIAS_MAX = 0.35

    # Do-no-harm guard on the bias.
    _GUARD_RANGE = 1.0
    _GUARD_HALF_ANGLE = math.radians(60.0)

    def reset(self, scenario, rng):
        self._t_prev = None
        self._ref_vec = None
        self._meas_vec = None
        self._meas_n = 0
        self._meas_t0 = None
        self._last_attempt_t = None
        self._bias_d = 0.0
        self._bias_t = None
        self._published = 0.0

    def _state(self):
        if not hasattr(self, "_t_prev"):
            self.reset(None, None)

    def precondition(self, obs, ports):
        self._state()
        err = float(obs.pose_error_estimate)
        if self._meas_t0 is not None:
            return True
        if err > self._ERR_TRIGGER:
            return True
        if self._bias_t is not None and abs(self._bias_d) > 1e-3:
            return True
        # Ticking on a trusted belief does not write anything; it is how the
        # reference axis gets laid down in the first place.
        return err <= self._ERR_TRUST

    def tick(self, obs, ports):
        self._state()
        t = float(obs.t)
        dt = 0.0 if self._t_prev is None else max(0.0, min(1.0, t - self._t_prev))
        self._t_prev = t

        axis = self._axis_angle(obs)
        err = float(obs.pose_error_estimate)
        yaw = float(obs.pose_estimate[2])
        theta = None if axis is None else (yaw + axis) % self._QUARTER

        # (3) Reference frame: recorded only while the belief is clearly trusted.
        if theta is not None and err <= self._ERR_TRUST and self._meas_t0 is None:
            self._update_reference(theta, dt)

        if self._meas_t0 is not None:
            return self._measure(obs, ports, t, theta)

        if (err > self._ERR_TRIGGER
                and self._ref_vec is not None
                and theta is not None
                and (self._last_attempt_t is None
                     or t - self._last_attempt_t >= self._COOLDOWN)):
            self._meas_t0 = t
            self._meas_vec = np.zeros(2)
            self._meas_n = 0
            return self._measure(obs, ports, t, theta)

        return self._apply_bias(obs, ports, t)

    # -- axis extraction ---------------------------------------------------

    def _axis_angle(self, obs):
        """Principal axis of the visible surface layout, robot frame, mod pi."""
        r = np.asarray(obs.ranges, dtype=float)
        a = np.asarray(obs.range_angles, dtype=float)
        if r.size == 0 or a.size != r.size:
            return None
        keep = np.isfinite(r) & (r > 0.0) & (r < float(obs.range_max))
        n = int(np.count_nonzero(keep))
        if n < self._MIN_FINITE:
            return None

        rr = r[keep]
        aa = a[keep]
        pts = np.stack((rr * np.cos(aa), rr * np.sin(aa)), axis=1)
        pts = pts - pts.mean(axis=0)
        cov = (pts.T @ pts) / float(n)
        if not np.all(np.isfinite(cov)):
            return None
        try:
            evals, evecs = np.linalg.eigh(cov)
        except np.linalg.LinAlgError:
            return None

        lam_min = float(evals[0])
        lam_max = float(evals[1])
        if lam_max <= 1e-9:
            return None
        # Ratio near 1.0 is an open floor or a room centre: nothing to lock onto.
        if lam_min / lam_max >= self._MAX_EIG_RATIO:
            return None

        v = evecs[:, 1]
        return math.atan2(float(v[1]), float(v[0])) % math.pi

    def _update_reference(self, theta, dt):
        v = self._unit4(theta)
        if self._ref_vec is None:
            self._ref_vec = v
            return
        alpha = 1.0 - math.exp(-dt / self._REF_TAU) if dt > 0.0 else 0.0
        self._ref_vec = (1.0 - alpha) * self._ref_vec + alpha * v

    @staticmethod
    def _unit4(theta):
        # Angles live modulo 90 deg, so quadruple before averaging on the circle.
        return np.array([math.cos(4.0 * theta), math.sin(4.0 * theta)])

    @staticmethod
    def _from4(vec):
        return (math.atan2(float(vec[1]), float(vec[0])) / 4.0) % (math.pi / 2.0)

    def _wrap_pm45(self, angle):
        return (angle + math.pi / 4.0) % self._QUARTER - math.pi / 4.0

    # -- states ------------------------------------------------------------

    def _measure(self, obs, ports, t, theta):
        """(4) Freeze, averaging the axis over the scans in the window."""
        ports.cap(vx=0.0)
        ports.cap(wz=0.0)
        ports["boundary_yaw_correction"] = 0.0
        self._published = 0.0

        if theta is not None:
            self._meas_vec = self._meas_vec + self._unit4(theta)
            self._meas_n += 1

        if t - self._meas_t0 < self._FREEZE:
            return Status.RUNNING

        self._meas_t0 = None
        self._last_attempt_t = t
        n, vec = self._meas_n, self._meas_vec
        self._meas_n, self._meas_vec = 0, None

        if n < 3 or float(np.hypot(vec[0], vec[1])) < 1e-6:
            return Status.FAILURE

        d = self._wrap_pm45(self._from4(self._ref_vec) - self._from4(vec))

        # (5) The refusal. Committing a symmetry-induced misassignment would put
        # a 90-degree error into a belief that was merely noisy.
        if abs(d) > math.pi / 4.0 - self._REFUSE_MARGIN:
            return Status.FAILURE

        self._bias_d = d
        self._bias_t = t
        return Status.SUCCESS

    def _apply_bias(self, obs, ports, t):
        """Twist-space image of ``yaw_belief += d``: a sustained yaw-rate bias."""
        if self._bias_t is None or abs(self._bias_d) <= 1e-3:
            ports["boundary_yaw_correction"] = 0.0
            self._published = 0.0
            return Status.FAILURE

        age = t - self._bias_t
        if age > 3.0 * self._BIAS_TAU:
            self._bias_d = 0.0
            self._bias_t = None
            ports["boundary_yaw_correction"] = 0.0
            self._published = 0.0
            return Status.FAILURE

        decay = math.exp(-age / self._BIAS_TAU)
        correction = self._bias_d * decay
        ports["boundary_yaw_correction"] = float(correction)
        self._published = float(correction)

        if self._blocked_ahead(obs):
            return Status.FAILURE

        bias = -self._BIAS_GAIN * correction
        bias = max(-self._BIAS_MAX, min(self._BIAS_MAX, bias))
        ports.adjust(wz=float(ports.command.wz + bias))
        return Status.SUCCESS

    def _blocked_ahead(self, obs):
        if bool(obs.contact):
            return True
        r = np.asarray(obs.ranges, dtype=float)
        a = np.asarray(obs.range_angles, dtype=float)
        if r.size == 0 or a.size != r.size:
            return False
        signed = np.arctan2(np.sin(a), np.cos(a))
        front = np.isfinite(r) & (np.abs(signed) <= self._GUARD_HALF_ANGLE)
        if not np.any(front):
            return False
        return float(np.min(r[front])) < self._GUARD_RANGE


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.15,
    "cost_metric": "time_to_goal",
}
