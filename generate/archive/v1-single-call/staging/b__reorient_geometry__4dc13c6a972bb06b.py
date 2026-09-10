"""Geometric reorientation from a stuck state (Cheng 1986)."""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "spl",
    "statement": (
        "Reorienting to the principal axis of the local free space when stuck "
        "produces a higher goal-reach rate than goal_seeker, which wedges "
        "against the divider for 111 s of a 120 s episode and never recovers."
    ),
}


def _wrap(a):
    """Fold a scalar angle into (-pi, pi]."""
    return float((a + math.pi) % (2.0 * math.pi) - math.pi)


def _wrap_arr(a):
    """Fold an array of angles into (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ReorientToGeometry(Action):
    """Reorient by the metric shape of the surrounding free space when wedged.

    Cheng (1986) found that disoriented rats recover their heading from the
    geometry of the enclosure -- the arrangement and relative extents of the
    surrounding surfaces -- in preference to featural cues, strongly enough to
    make systematic rotational errors that the features present would have
    resolved. Translated to a Go2 with a 181-beam planar scan: when a drive
    command stops producing displacement, stop steering by goal bearing and
    read the *shape* of the locally visible free space instead.

    The shape statistic is the second-moment matrix of the free region about
    the robot. A beam whose free length is L contributes int_0^L s^2 ds, i.e.
    L^3/3, along its own direction, so the matrix is sum_i L_i^3 u_i u_i^T and
    its dominant eigenvector is the principal axis of the clear region: the
    direction the free space is longest in. In a corridor that is the corridor;
    in a wedge against a divider it is the way out along the divider.

    The axis is a line, not a direction, so the sign is chosen by which end
    carries more free space. Only when the two ends are near-symmetric does the
    goal bearing break the tie. That ordering is the finding, not an
    implementation detail: the geometric cue is consulted first and the goal
    second, which is what separates this from a recovery that backs up and
    retries the same bearing.

    The node owns the drive command because the trigger is defined on it: "not
    moving" is only meaningful relative to a motion this node asked for. Its
    nominal behaviour is therefore deliberately sensor-blind goal seeking, so
    that the reorientation episode is the only thing distinguishing it from the
    goal_seeker baseline.
    """

    PORTS = (
        # The latched escape heading, republished each tick in the robot frame.
        Port("reorient_axis", PortType.BEARING, Direction.OUT),
        # Eigenvalue contrast of the free-space moment matrix: how elongated
        # (and so how trustworthy) the axis is. Near 0 in an isotropic space.
        Port("axis_confidence", PortType.CONFIDENCE, Direction.OUT),
        Port("reorienting", PortType.FLAG, Direction.OUT),
    )

    PRECONDITION = (
        "The robot is under a goal-directed drive command and that command has "
        "produced no displacement for longer than the stuck window."
    )
    EFFECT = (
        "Turns the robot onto the principal axis of the locally visible free "
        "space and drives along it until it is no longer wedged."
    )
    TAGS = (
        "recovery",
        "stuck",
        "reorientation",
        "free-space-geometry",
        "principal-axis",
        "unwedge",
    )

    SOURCE = (
        'Cheng, K. (1986). "A purely geometric module in the rat\'s spatial '
        'representation." Cognition 23(2), 149-178.'
    )

    # --- stuck detection ---------------------------------------------------
    STUCK_WINDOW = 1.2            # s of history the verdict is taken over
    STUCK_DISPLACEMENT = 0.12     # m of belief motion below which we are stuck
    MIN_COMMANDED_SPEED = 0.20    # m/s: below this we were not asking to move

    # --- motion ------------------------------------------------------------
    CRUISE_SPEED = 0.6
    ESCAPE_SPEED = 0.6
    TURN_GAIN = 2.5
    MAX_WZ = 1.0
    ALIGN_TOL = 0.12              # rad; "on the axis"
    MAX_TURN_TIME = 3.0
    MAX_RUN_TIME = 12.0

    # --- geometry ----------------------------------------------------------
    BODY_RADIUS = 0.3             # free space for the body centre ends here
    SIGN_SYMMETRY = 0.80          # above this ratio the two ends are a tie
    RESUME_LOOK = 2.0             # m of open goalward space that counts as free
    RESUME_SECTOR = math.radians(15.0)
    ARRIVED = 0.25

    # ----------------------------------------------------------------------
    def reset(self, scenario, rng):
        self._phase = "cruise"
        self._phase_t0 = 0.0
        self._history = []        # (t, x, y, commanded_speed)
        self._last_speed = 0.0
        self._last_t = 0.0
        self._axis_world = None   # latched escape heading, world frame
        self._confidence = 0.0

    def _ensure(self, obs):
        """Self-initialise if the runner never called reset, or on a restart."""
        if not hasattr(self, "_phase") or obs.t < self._last_t:
            self.reset(None, None)
        self._last_t = obs.t

    def _enter(self, phase, obs):
        self._phase = phase
        self._phase_t0 = obs.t
        # A fresh window must fill before the next stuck verdict, which is what
        # stops the zero-displacement samples from the wedge we just left from
        # immediately re-triggering.
        self._history = []
        self._last_speed = 0.0

    # ----------------------------------------------------------------------
    def precondition(self, obs, ports) -> bool:
        # Applicability, not the trigger. The trigger ("commanded and not
        # moving") is a statement about a command this node issues, so it can
        # only be evaluated while the node is driving -- it lives in tick as
        # the phase gate. Here we only check there is a scan to reason over.
        return obs.ranges is not None and len(obs.ranges) > 0

    def tick(self, obs, ports) -> Status:
        self._ensure(obs)

        ranges = np.asarray(obs.ranges, dtype=float)
        if ranges.size == 0:
            return Status.FAILURE

        x, y, yaw = (float(v) for v in obs.pose_estimate)
        self._history.append((float(obs.t), x, y, self._last_speed))
        cutoff = obs.t - self.STUCK_WINDOW
        while len(self._history) > 2 and self._history[0][0] < cutoff:
            self._history.pop(0)

        # --- phase transitions --------------------------------------------
        if self._phase in ("cruise", "run") and self._is_stuck(obs):
            # Wedged: re-read the geometry from here and align to it.
            self._begin_reorient(obs)
        elif self._phase == "turn":
            err = _wrap(self._axis_world - yaw)
            if abs(err) <= self.ALIGN_TOL or (
                obs.t - self._phase_t0 >= self.MAX_TURN_TIME
            ):
                self._enter("run", obs)
        elif self._phase == "run":
            if self._goal_open(obs) or obs.t - self._phase_t0 >= self.MAX_RUN_TIME:
                self._enter("cruise", obs)

        # --- command for the current phase ---------------------------------
        if self._phase == "turn":
            # Turn in place. No backing off: the escape is the reorientation.
            err = _wrap(self._axis_world - yaw)
            vx = 0.0
            wz = self._clip_wz(self.TURN_GAIN * err)
        elif self._phase == "run":
            err = _wrap(self._axis_world - yaw)
            vx = self.ESCAPE_SPEED * max(0.0, math.cos(err))
            wz = self._clip_wz(self.TURN_GAIN * err)
        else:
            vx, wz = self._cruise(obs)

        ports.drive(Command(vx, 0.0, wz))
        self._last_speed = abs(vx)

        ports["reorienting"] = self._phase in ("turn", "run")
        ports["axis_confidence"] = float(self._confidence)
        ports["reorient_axis"] = (
            _wrap(self._axis_world - yaw) if self._axis_world is not None else 0.0
        )

        if obs.goal is not None:
            gx, gy = float(obs.goal[0]), float(obs.goal[1])
            if math.hypot(gx - x, gy - y) <= self.ARRIVED:
                return Status.SUCCESS
        return Status.RUNNING

    # ----------------------------------------------------------------------
    def _clip_wz(self, wz):
        return float(max(-self.MAX_WZ, min(self.MAX_WZ, wz)))

    def _cruise(self, obs):
        """Sensor-blind goal seeking; the geometry is only consulted on stuck."""
        x, y, yaw = (float(v) for v in obs.pose_estimate)
        if obs.goal is None:
            # Goal not disclosed. Hold the current heading and keep moving --
            # reorientation is then the only thing that ever turns us, which is
            # the mechanism doing exploration rather than a second behaviour.
            return self.CRUISE_SPEED, 0.0
        gx, gy = float(obs.goal[0]), float(obs.goal[1])
        bearing = _wrap(math.atan2(gy - y, gx - x) - yaw)
        vx = self.CRUISE_SPEED * max(0.0, math.cos(bearing))
        return vx, self._clip_wz(self.TURN_GAIN * bearing)

    def _is_stuck(self, obs):
        """Commanded to translate, yet the belief has not moved."""
        if len(self._history) < 3:
            return False
        if self._history[-1][0] - self._history[0][0] < self.STUCK_WINDOW:
            return False
        mean_cmd = sum(h[3] for h in self._history) / len(self._history)
        if mean_cmd < self.MIN_COMMANDED_SPEED:
            return False
        # Spread about the oldest sample, so "moved then stopped" only reads as
        # stuck once the window has slid fully into the stationary stretch.
        # The threshold sits well above any plausible belief drift over 1.2 s
        # and well below the >=0.24 m a genuinely moving robot covers.
        x0, y0 = self._history[0][1], self._history[0][2]
        spread = max(math.hypot(h[1] - x0, h[2] - y0) for h in self._history)
        return spread < self.STUCK_DISPLACEMENT

    def _begin_reorient(self, obs):
        theta, conf = self._principal_axis(obs)
        # Latch in the world frame so the heading target is stable across the
        # manoeuvre instead of chasing a scan that changes as we turn.
        self._axis_world = _wrap(float(obs.pose_estimate[2]) + theta)
        self._confidence = conf
        self._enter("turn", obs)

    def _principal_axis(self, obs):
        """Dominant axis of the free region, and how elongated that region is.

        Returns (bearing in the robot frame, elongation in [0, 1]).
        """
        r = np.asarray(obs.ranges, dtype=float)
        ang = np.asarray(obs.range_angles, dtype=float)
        rmax = float(obs.range_max)

        # No return means free all the way out to the sensor horizon.
        free = np.where(np.isfinite(r), np.clip(r, 0.0, rmax), rmax)
        # The body centre cannot reach the surface itself.
        free = np.maximum(free - self.BODY_RADIUS, 0.0)

        # Second moment of the free wedge about the robot: int_0^L s^2 ds.
        w = free ** 3
        c, s = np.cos(ang), np.sin(ang)
        mxx = float(np.sum(w * c * c))
        myy = float(np.sum(w * s * s))
        mxy = float(np.sum(w * c * s))

        total = mxx + myy
        if total <= 1e-9:
            return 0.0, 0.0

        # Closed-form eigen-decomposition of the symmetric 2x2.
        disc = math.sqrt(max(0.25 * (mxx - myy) ** 2 + mxy * mxy, 0.0))
        l1 = 0.5 * total + disc
        l2 = 0.5 * total - disc
        theta = 0.5 * math.atan2(2.0 * mxy, mxx - myy)
        conf = float(min(1.0, max(0.0, (l1 - l2) / l1))) if l1 > 1e-9 else 0.0

        # The axis is a line. Pick the end with more free space behind it --
        # geometry first.
        ahead = np.cos(ang - theta) > 0.0
        w_ahead = float(np.sum(w[ahead]))
        w_behind = float(np.sum(w[~ahead]))
        if w_behind > w_ahead:
            theta = _wrap(theta + math.pi)
            w_ahead, w_behind = w_behind, w_ahead

        # Only if the two ends are near-symmetric does the goal get a say --
        # goal second.
        big = max(w_ahead, 1e-9)
        if w_behind / big > self.SIGN_SYMMETRY and obs.goal is not None:
            x, y, yaw = (float(v) for v in obs.pose_estimate)
            gx, gy = float(obs.goal[0]), float(obs.goal[1])
            gb = _wrap(math.atan2(gy - y, gx - x) - yaw)
            flipped = _wrap(theta + math.pi)
            if abs(_wrap(flipped - gb)) < abs(_wrap(theta - gb)):
                theta = flipped

        return _wrap(theta), conf

    def _goal_open(self, obs):
        """Has the goal direction stopped being blocked?

        A sector rather than a single ray: one ray calls a gap open while the
        body is still clipping the corner that forms it.
        """
        if obs.goal is None:
            return False
        x, y, yaw = (float(v) for v in obs.pose_estimate)
        gx, gy = float(obs.goal[0]), float(obs.goal[1])
        dist = math.hypot(gx - x, gy - y)
        bearing = _wrap(math.atan2(gy - y, gx - x) - yaw)

        ang = np.asarray(obs.range_angles, dtype=float)
        sel = np.abs(_wrap_arr(ang - bearing)) <= self.RESUME_SECTOR
        if not np.any(sel):
            return False

        r = np.asarray(obs.ranges, dtype=float)
        rmax = float(obs.range_max)
        free = np.where(np.isfinite(r), np.clip(r, 0.0, rmax), rmax)
        return float(np.min(free[sel])) >= min(dist, self.RESUME_LOOK)


def micro(seed=0):
    """A divider the straight-line route runs into, with the gap out of sight.

    The left-hand block turns the start region into a 1.5 m corridor running
    alongside the divider, so the free space there is strongly elongated along
    +y and the principal axis at the wedge point is unambiguous -- and it points
    at the gap. goal_seeker heads straight at the goal, meets the divider face
    about 0.7 m in, and pushes there for the rest of the episode.
    """
    world = room(
        8.0,
        6.0,
        boxes=[
            (0.0, 0.0, 2.4, 6.0),   # left block: makes the start region a corridor
            (3.9, 0.0, 4.2, 3.0),   # the divider; the only way through is above y=3
        ],
    )
    return MicroScenario(
        id="reorient_geometry_divider_wedge",
        world=world,
        start=(3.15, 0.8, 0.0),     # centred in the corridor, facing the divider
        goal=(6.5, 1.2),            # straight line to it is blocked low down
        time_limit=120.0,
        seed=seed,
        goal_known=True,
    )
