"""Path-integrated goal vector with detour resumption.

Implements the mechanism reviewed in Etienne, A. S. & Jeffery, K. J. (2004),
"Path integration in mammals," Hippocampus 14(2), 180-192: a vector to the goal
maintained by integrating self-motion (here, the commanded Twist as efference
copy), anchored once by an allothetic fix, degrading with distance travelled,
and recalibrated by external cues only when the cue conflict is small.

All numeric constants below are engineering judgement for this robot and this
sensor, not values taken from that review.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


def _wrap(a):
    """Wrap to (-pi, pi]. Works elementwise on arrays as well as on scalars."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class PathIntegrationGoalVector(Action):
    """Drive to the goal on a dead-reckoned goal vector that survives detours.

    The robot takes a single allothetic fix when the goal is first disclosed,
    then maintains the goal vector by integrating the Twists it issued. Because
    the vector is held internally rather than re-derived from the pose belief
    each tick, it stays correct while the robot is off the direct course going
    around an obstruction it could not see until it was close, and the direct
    course can be resumed the instant it opens. That resumption is the
    behavioural signature of mammalian path integration.

    The pose belief is re-admitted only under the cue-conflict rule: a small
    discrepancy recalibrates the integrator at low gain, a large one is refused.
    """

    PORTS = (
        Port("pi_pose", PortType.POSE, Direction.OUT),
        Port("pi_goal_bearing", PortType.BEARING, Direction.OUT),
        Port("pi_confidence", PortType.CONFIDENCE, Direction.OUT),
    )
    PRECONDITION = "The goal position has been disclosed at least once this episode."
    EFFECT = "Brings the robot to the goal by steering on a path-integrated goal vector that is preserved across detours."
    TAGS = ("path-integration", "dead-reckoning", "goal-vector", "detour", "navigation")

    # Runner saturation. Commands are clipped to these *before* being integrated:
    # dead reckoning on a command the runner would not execute is free error.
    MAX_V = 0.8
    MAX_W = 1.0

    V_NOM = 0.6           # cruise speed; leaves headroom for the push-off term
    V_LAT = 0.35          # lateral cap - strafing is for trimming, not travel
    K_YAW = 1.6

    CONE = math.radians(20.0)   # half-width of the corridor a heading must own
    LOOKAHEAD = 1.4             # how far ahead that corridor must be clear, m
    SAFE = 0.55                 # push-off starts inside this range, m
    ARRIVE = 0.20               # deliberately tighter than any plausible scoring
                                # radius, so we never stop just short of it
    SLOW_RADIUS = 0.8

    DEFLECT_STEP = math.radians(8.0)
    DEFLECT_MAX = math.radians(160.0)
    TIE = math.radians(15.0)    # below this the two sides are a coin flip
    FREE_RELEASE = 1.5          # hold the committed side this long after clearing
    STALL_WINDOW = 4.0          # committed detour making no headway -> flip side
    STALL_EPS = 0.15

    TRUST_POSE_ERR = 0.35       # believe the belief only when it admits to being good
    CONFLICT_MAX = 0.6          # above this the integrator keeps control outright
    RECAL_RATE = 0.3            # per second, first order
    PI_ERROR_SCALE = 10.0       # metres of travel that halve published confidence

    DT_MAX = 0.2
    DT_DEFAULT = 0.05

    # ---------------- lifecycle ----------------

    def reset(self, scenario=None, rng=None):
        self._t_prev = None
        self._cmd = (0.0, 0.0, 0.0)
        self._p = np.zeros(2, dtype=float)
        self._th = 0.0
        self._goal = None
        self._fixed = False
        self._dist_since_fix = 0.0
        self._side = 0
        self._free_since = None
        self._best_d = float("inf")
        self._best_d_t = 0.0
        self._ready = True

    def _ensure(self):
        if not getattr(self, "_ready", False):
            self.reset()

    def precondition(self, obs, ports):
        self._ensure()
        return obs.goal is not None or self._fixed

    # ---------------- tick ----------------

    def tick(self, obs, ports):
        self._ensure()

        t = float(obs.t)
        dt = self.DT_DEFAULT if self._t_prev is None else t - self._t_prev
        dt = min(max(dt, 0.0), self.DT_MAX)
        self._t_prev = t

        # Integrate the previous command first: this tick's decision is made from
        # a goal vector that already accounts for the last control interval.
        self._integrate(dt)

        if obs.goal is not None and not self._fixed:
            self._anchor(obs)

        if not self._fixed:
            ports["pi_confidence"] = 0.0
            return Status.FAILURE

        self._recalibrate(obs, dt)

        rmax = float(obs.range_max)
        rr = np.asarray(obs.ranges, dtype=float)
        rr = np.clip(np.where(np.isfinite(rr), rr, rmax), 0.0, rmax)
        aa = np.asarray(obs.range_angles, dtype=float)

        gvec = self._goal - self._p
        d = float(math.hypot(gvec[0], gvec[1]))
        bearing = _wrap(math.atan2(gvec[1], gvec[0]) - self._th)

        ports["pi_pose"] = (float(self._p[0]), float(self._p[1]), float(_wrap(self._th)))
        ports["pi_goal_bearing"] = float(bearing)
        ports["pi_confidence"] = float(1.0 / (1.0 + self._dist_since_fix / self.PI_ERROR_SCALE))

        if d <= self.ARRIVE:
            self._emit(ports, 0.0, 0.0, 0.0)
            return Status.SUCCESS

        # Never let something standing beyond the goal count as blocking it.
        need = min(self.LOOKAHEAD, max(d - 0.2, 0.4))

        if self._cone_min(bearing, rr, aa) >= need:
            heading = bearing
            if self._free_since is None:
                self._free_since = t
            elif t - self._free_since > self.FREE_RELEASE:
                self._side = 0
        else:
            self._free_since = None
            heading = self._detour_heading(bearing, need, rr, aa, t, d)

        speed = self.V_NOM
        speed *= float(np.clip(self._cone_min(0.0, rr, aa) / 1.2, 0.35, 1.0))
        speed *= float(np.clip(d / self.SLOW_RADIUS, 0.35, 1.0))

        vx = speed * math.cos(heading)
        vy = float(np.clip(speed * math.sin(heading), -self.V_LAT, self.V_LAT))
        wz = self.K_YAW * heading
        vx = max(vx, -0.25)          # reversing is sensed but still done slowly
        vx, vy = self._push_off(vx, vy, rr, aa)

        self._emit(ports, vx, vy, wz)
        return Status.RUNNING

    # ---------------- the integrator ----------------

    def _anchor(self, obs):
        """One allothetic fix, taken when the belief is freshest."""
        px, py, pyaw = obs.pose_estimate
        self._p = np.array([float(px), float(py)], dtype=float)
        self._th = float(pyaw)
        self._goal = np.array([float(obs.goal[0]), float(obs.goal[1])], dtype=float)
        self._fixed = True
        self._dist_since_fix = 0.0

    def _integrate(self, dt):
        if dt <= 0.0 or not self._fixed:
            return
        vx, vy, wz = self._cmd
        # Midpoint heading over the interval: the arc error stays second order,
        # which matters because this runs a few thousand times an episode.
        th_mid = self._th + 0.5 * wz * dt
        c, s = math.cos(th_mid), math.sin(th_mid)
        self._p += np.array([vx * c - vy * s, vx * s + vy * c], dtype=float) * dt
        self._th = _wrap(self._th + wz * dt)
        self._dist_since_fix += math.hypot(vx, vy) * dt

    def _recalibrate(self, obs, dt):
        """Cue conflict: small discrepancies reset the integrator, large ones do not."""
        if dt <= 0.0 or obs.pose_estimate is None:
            return
        if float(obs.pose_error_estimate) > self.TRUST_POSE_ERR:
            return
        ax, ay, ayaw = obs.pose_estimate
        dxy = np.array([float(ax), float(ay)], dtype=float) - self._p
        if math.hypot(dxy[0], dxy[1]) > self.CONFLICT_MAX:
            return
        g = min(1.0, self.RECAL_RATE * dt)
        self._p += g * dxy
        self._th = _wrap(self._th + g * _wrap(float(ayaw) - self._th))
        self._dist_since_fix *= (1.0 - g)

    # ---------------- steering helpers ----------------

    def _cone_min(self, ang, rr, aa):
        """Closest return inside the corridor centred on a robot-frame heading."""
        m = np.abs(_wrap(aa - ang)) <= self.CONE
        if not np.any(m):
            return 0.0
        return float(np.min(rr[m]))

    def _min_deflection(self, bearing, need, rr, aa, side):
        n = int(self.DEFLECT_MAX / self.DEFLECT_STEP)
        for i in range(1, n + 1):
            off = i * self.DEFLECT_STEP
            if self._cone_min(_wrap(bearing + side * off), rr, aa) >= need:
                return off
        return float("inf")

    def _pick_side(self, bearing, need, rr, aa):
        dl = self._min_deflection(bearing, need, rr, aa, +1)
        dr = self._min_deflection(bearing, need, rr, aa, -1)
        if abs(dl - dr) > self.TIE:
            return 1 if dl < dr else -1
        # Near-tie. A metre of wall angle should not decide which way round an
        # obstruction we go; the side with more open floor should.
        rel = _wrap(aa - bearing)
        left = rr[(rel > 0.0) & (rel < math.pi)]
        right = rr[(rel < 0.0) & (rel > -math.pi)]
        lm = float(np.mean(left)) if left.size else 0.0
        rm = float(np.mean(right)) if right.size else 0.0
        return 1 if lm >= rm else -1

    def _detour_heading(self, bearing, need, rr, aa, t, d):
        if self._side == 0:
            self._side = self._pick_side(bearing, need, rr, aa)
            self._best_d = d
            self._best_d_t = t

        # Progress is judged in the integrated frame - that is the one quantity
        # path integration gives you that reactive sensing cannot.
        if d < self._best_d - self.STALL_EPS:
            self._best_d = d
            self._best_d_t = t
        elif t - self._best_d_t > self.STALL_WINDOW:
            self._side = -self._side
            self._best_d = d
            self._best_d_t = t

        n = int(self.DEFLECT_MAX / self.DEFLECT_STEP)
        for i in range(1, n + 1):
            ang = _wrap(bearing + self._side * i * self.DEFLECT_STEP)
            if self._cone_min(ang, rr, aa) >= need:
                return ang
        return self._most_open(rr, aa)

    def _most_open(self, rr, aa):
        """Fallback when nothing on the committed side is passable."""
        best, best_a = -1.0, 0.0
        for i in range(0, aa.size, 2):
            a = float(aa[i])
            c = self._cone_min(a, rr, aa)
            if c > best:
                best, best_a = c, a
        return best_a

    def _push_off(self, vx, vy, rr, aa):
        """Safety floor, not the mechanism: bleed away from the nearest return."""
        i = int(np.argmin(rr))
        r = float(rr[i])
        if r >= self.SAFE:
            return vx, vy
        g = 0.6 * (self.SAFE - r) / self.SAFE
        return vx - g * math.cos(float(aa[i])), vy - g * math.sin(float(aa[i]))

    def _emit(self, ports, vx, vy, wz):
        vx = float(np.clip(vx, -self.MAX_V, self.MAX_V))
        vy = float(np.clip(vy, -self.MAX_V, self.MAX_V))
        wz = float(np.clip(wz, -self.MAX_W, self.MAX_W))
        self._cmd = (vx, vy, wz)
        ports.drive(Command(vx, vy, wz))


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}


def micro(seed=0):
    """Divider wall across the direct line, with its open end well off that line.

    goal_seeker drives the straight line, meets the divider at about (4.0, 2.8)
    and stays there. Reaching the goal means leaving the goal bearing by more
    than 60 degrees, travelling several metres on a heading that points away
    from the goal, and then knowing where the goal is again - which is exactly
    what the integrated vector supplies.
    """
    world = room(9.0, 6.0, boxes=[(4.0, 0.0, 4.5, 3.6)])
    return MicroScenario(
        id="pi_goal_vector_detour",
        world=world,
        start=(1.0, 1.5, 0.0),
        goal=(7.8, 4.5),
        time_limit=50.0,
        seed=seed,
        goal_known=True,
    )
