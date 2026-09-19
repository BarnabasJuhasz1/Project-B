import math

from nodes.contract import Action, Direction, Port, PortType, Status

_K = 4.009e-5        # deg^-1, the paper's normaliser: numerator peaks at ~90 deg
_UNIT = 0.15         # m, one "stride" for a Go2 trot
_DPHI_CAP = 90.0     # deg, the paper's ceiling on a single increment (scaled by 1/l)
_TURN_GAIN = 1.5     # s^-1
_CRUISE = 0.8        # m/s
_ARRIVE = 0.5        # m
_ARM_TRAVEL = 1.0    # m of commanded translation before the 1/l damping is meaningful
_MAX_DT = 0.2        # s, guards against a stalled or restarted clock


def _wrap180(a):
    """Wrap degrees to (-180, 180] -- the half-open side matters: delta = 180 must
    survive as +180 so that Eq. 3's (180 - delta) factor zeroes the increment."""
    return 180.0 - ((180.0 - a) % 360.0)


def _wrap_pi(a):
    return math.pi - ((math.pi - a) % (2.0 * math.pi))


class SteerByPathIntegratedGoalVector(Action):
    """Cataglyphis-style path integration (Mueller & Wehner Eq. 3) as a steering source.

    Carries two scalars instead of a position: phi, the mean direction travelled so
    far, and l, distance travelled in unit strides. Each stride it rotates phi by
    k*(180+delta)*(180-delta)*delta/l, so the stored direction stiffens as 1/l and
    late perturbations move the home vector less and less. Displacement from a
    once-set anchor is reconstructed as l*u*(cos phi, sin phi) and the goal bearing
    is taken from that, NOT from pose_estimate -- the whole point is that strides
    come from the commanded Twist (proprioception) while pose_estimate translation
    carries a random walk.

    Revises wz (adjust: steer onto the reconstructed bearing) and vx (cap: no faster
    than 0.8*cos b, so the robot does not cruise while the goal is off the nose).
    It writes neither before the goal has been disclosed, nor before 1.0 m of
    commanded translation has accumulated since the anchor -- below that l is too
    small for the damping to exist and the base's own goal bearing is better. It
    reads no lidar and claims no obstacle competence.
    """

    WRITES = ("vx", "wz")
    PORTS = (
        Port("pi_goal_bearing", PortType.BEARING, Direction.OUT),
        Port("pi_goal_bearing_valid", PortType.FLAG, Direction.OUT),
    )
    PRECONDITION = (
        "The goal has been disclosed and at least 1.0 m of commanded translation "
        "has accumulated since the path integrator was anchored."
    )
    EFFECT = (
        "Drives along the goal bearing reconstructed by the 1/l-damped stride "
        "integrator rather than the bearing implied by the drifting pose estimate."
    )
    TAGS = ("path-integration", "dead-reckoning", "goal-bearing", "drift", "ant")

    # ---------------- state ----------------

    def _clear(self):
        self._anchor = None      # (ax, ay), pose_estimate xy at disclosure time
        self._offset = None      # goal - anchor, frozen at that same instant
        self._phi = 0.0          # deg, mean direction travelled
        self._l = 1.0            # unit steps; never below 1.0, it is the divisor
        self._s = 0.0            # m, sub-stride remainder
        self._travel = 0.0       # m, commanded translation since the anchor
        self._last_t = None
        self._last_cmd = None    # (vx, vy) actually commanded on the previous step
        self._bearing = None

    def _ensure(self):
        if not hasattr(self, "_anchor"):
            self._clear()

    def reset(self, scenario, rng):
        self._clear()

    # ---------------- mechanism ----------------

    def _course_deg(self, yaw, cmd):
        """World-frame direction of travel for a commanded body-frame twist."""
        if cmd is None:
            return math.degrees(_wrap_pi(yaw))
        vx, vy = cmd
        if math.hypot(vx, vy) <= 1e-9:
            return math.degrees(_wrap_pi(yaw))
        return math.degrees(_wrap_pi(yaw + math.atan2(vy, vx)))

    def _try_anchor(self, obs):
        # Anchor at the earliest possible moment: drift is a random walk from near
        # zero, so the anchor-to-goal offset is least corrupted now and must never
        # be recomputed from a later, dirtier pose_estimate.
        if self._anchor is not None or obs.goal is None:
            return
        px, py, yaw = (float(v) for v in obs.pose_estimate)
        self._anchor = (px, py)
        self._offset = (float(obs.goal[0]) - px, float(obs.goal[1]) - py)
        self._phi = self._course_deg(yaw, self._last_cmd)
        self._l = 1.0
        self._s = 0.0
        self._travel = 0.0

    def _integrate(self, yaw, cmd, dt):
        if cmd is None or dt <= 0.0:
            return
        speed = math.hypot(cmd[0], cmd[1])
        if speed <= 0.0:
            return
        course = self._course_deg(yaw, cmd)
        self._s += speed * dt
        self._travel += speed * dt
        while self._s >= _UNIT:
            self._s -= _UNIT
            delta = _wrap180(course - self._phi)
            # Eq. 3 verbatim, in degrees, because k is quoted in deg^-1.
            dphi = _K * (180.0 + delta) * (180.0 - delta) * delta / self._l
            lim = _DPHI_CAP / self._l
            dphi = max(-lim, min(lim, dphi))
            self._phi = _wrap180(self._phi + dphi)
            self._l += 1.0

    def _p_hat(self):
        phi = math.radians(self._phi)
        r = self._l * _UNIT
        return (self._anchor[0] + r * math.cos(phi),
                self._anchor[1] + r * math.sin(phi))

    def _goal_xy(self):
        return (self._anchor[0] + self._offset[0],
                self._anchor[1] + self._offset[1])

    # ---------------- contract ----------------

    def precondition(self, obs, ports):
        # All bookkeeping happens here, on every tick including the ones we decline:
        # the stride count has to keep advancing while the base is driving, or the
        # 1.0 m arming threshold would never be crossed.
        self._ensure()

        t = float(obs.t)
        dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), _MAX_DT)
        self._last_t = t

        yaw = float(obs.pose_estimate[2])
        had_anchor = self._anchor is not None
        self._try_anchor(obs)
        if had_anchor:
            self._integrate(yaw, self._last_cmd, dt)

        # Record the base's proposal now; tick() overwrites it with what we actually
        # let through, so the integrator is always fed the executed command.
        pending = getattr(ports, "command", None)
        if pending is not None:
            self._last_cmd = (float(pending.vx), float(pending.vy))

        if self._anchor is None or obs.goal is None:
            self._bearing = None
            ports["pi_goal_bearing_valid"] = False
            return False

        gx, gy = self._goal_xy()
        px, py = self._p_hat()
        self._bearing = _wrap_pi(math.atan2(gy - py, gx - px) - yaw)
        ports["pi_goal_bearing"] = float(self._bearing)

        ready = self._travel >= _ARM_TRAVEL
        ports["pi_goal_bearing_valid"] = bool(ready)
        return ready

    def tick(self, obs, ports):
        b = self._bearing
        if b is None:
            return Status.FAILURE

        wz = max(-1.0, min(1.0, _TURN_GAIN * b))
        v_limit = _CRUISE * max(0.0, math.cos(b))

        ports.adjust(wz=wz)
        ports.cap(vx=v_limit)

        # Mirror cap semantics locally rather than re-reading the port, so the
        # integrator is fed the same number the runner will execute.
        if self._last_cmd is not None:
            base_vx, vy = self._last_cmd
            self._last_cmd = (math.copysign(min(abs(base_vx), v_limit), base_vx), vy)

        gx, gy = self._goal_xy()
        px, py = self._p_hat()
        if math.hypot(gx - px, gy - py) < _ARRIVE:
            return Status.SUCCESS
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}
