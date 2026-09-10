"""Uncertainty-scaled terminal coverage for a drifting pose belief."""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PRINCIPLE = {
    "mechanism": (
        "Goal capture is a disc around the TRUE goal, but a controller can only "
        "servo in belief space, so point convergence parks the robot at "
        "true = goal - localisation_error and misses whenever that error exceeds "
        "the capture radius. Physically sweeping the disc of plausible true "
        "positions puts the goal inside the covered set by construction, turning "
        "an informational failure into a bounded time cost."
    ),
    "trigger": (
        "The goal is disclosed and the belief about our own pose is uncertain by "
        "an amount comparable to or larger than the capture radius, so being "
        "'at the goal' in belief no longer implies being at the goal."
    ),
    "derivation": (
        "Track a point spiralling out of the believed goal, r(theta)=pitch*theta/2pi "
        "with pitch 0.45 m so consecutive turns overlap for a ~0.3 m capture radius, "
        "cut off at max(0.6, 2*pose_error_estimate). Advance the spiral phase only "
        "while locked onto the target, so approach and sweep are one law. Drive "
        "holonomically with wz=0: the 181-beam scan is omnidirectional, so facing "
        "the travel direction buys no sensing and costs sweep time. Yaw drift only "
        "rotates the spiral, and a rotated disc covers the same set. Veto each "
        "commanded direction on the minimum range in a +/-25 deg beam sector and "
        "deflect to the nearest clear heading, since a sweep into a wall covers nothing."
    ),
}

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}


class UncertaintyScaledGoalSweep(Action):
    """Cover the belief-uncertainty disc around the goal instead of converging to it.

    A point-convergent controller stops when ``pose_estimate == goal``, which puts
    its true position at ``goal - drift``. Once ``|drift|`` exceeds the capture
    radius that controller can never succeed, however well it tracks. This node
    servos onto a target that spirals outward from the believed goal, so the robot
    physically occupies a disc whose radius is set by ``pose_error_estimate``. The
    true goal lies inside that disc, so capture is near-certain; the price is the
    time and path length spent sweeping.

    The spiral is expressed in belief coordinates but its *shape* is not: yaw error
    rotates it and position error translates it, and neither changes the area
    covered. That is what makes the mechanism drift-tolerant rather than merely
    drift-aware.
    """

    PORTS = (
        Port("sweep_region", PortType.REGION, Direction.OUT),
        Port("sweep_active", PortType.FLAG, Direction.OUT),
    )
    PRECONDITION = "The goal position has been disclosed to the robot."
    EFFECT = (
        "The robot physically covers a disc around the believed goal whose radius "
        "is set by the reported localisation error, so the true goal falls inside "
        "the set of points actually occupied."
    )
    TAGS = ("goal-capture", "coverage", "spiral", "localisation-drift",
            "terminal-approach", "uncertainty-scaled")

    # Assumed capture radius. Deliberately conservative: under-guessing costs a
    # denser spiral, over-guessing leaves uncovered gaps between turns.
    CAPTURE_R = 0.3
    PITCH = 0.45              # <= 2 * CAPTURE_R, so consecutive turns overlap
    K_ERR = 2.0               # sweep out to twice the reported error
    R_FLOOR = 0.6             # a sweep smaller than this cannot capture anything

    APPROACH_SPEED = 0.8
    SWEEP_SPEED = 0.55
    LOCK_TOL = 0.45           # phase advances only inside this tracking error
    K_P = 2.0                 # settles at err = SWEEP_SPEED/K_P, inside LOCK_TOL
    MAX_DTHETA = 0.5          # guard against a long control step at r ~ 0
    MAX_DT = 0.2

    STOP_D = 0.55             # robot radius 0.3 plus margin
    SECTOR_HALF = math.radians(25.0)
    GOLDEN = 2.39996          # phase kick between sweeps, so retries decorrelate

    def reset(self, scenario, rng):
        self._theta = 0.0
        self._phase = 0.0
        self._t_prev = None

    def precondition(self, obs, ports):
        return obs.goal is not None

    def tick(self, obs, ports):
        if obs.goal is None:
            ports["sweep_active"] = False
            return Status.FAILURE

        self._ensure_state()
        dt = 0.0 if self._t_prev is None else min(self.MAX_DT, max(0.0, obs.t - self._t_prev))
        self._t_prev = obs.t

        gx, gy = float(obs.goal[0]), float(obs.goal[1])
        px, py, yaw = (float(v) for v in obs.pose_estimate)

        r_max = max(self.R_FLOOR, self.K_ERR * float(obs.pose_error_estimate))
        ports["sweep_region"] = (gx, gy, r_max)

        b = self.PITCH / (2.0 * math.pi)
        r = b * self._theta

        if r > r_max:
            # Disc covered without capture: re-arm with a fresh (larger) radius and
            # a rotated phase rather than retracing the same curve.
            self._theta = 0.0
            self._phase += self.GOLDEN
            ports["sweep_active"] = False
            return Status.SUCCESS

        ports["sweep_active"] = True

        ang = self._theta + self._phase
        tx = gx + r * math.cos(ang)
        ty = gy + r * math.sin(ang)

        ex, ey = tx - px, ty - py
        err = math.hypot(ex, ey)

        if err < self.LOCK_TOL and dt > 0.0:
            # Arc-length parameterisation: coverage pitch is preserved even when the
            # commanded speed is cut by the obstacle veto.
            ds = self.SWEEP_SPEED * dt
            self._theta += min(self.MAX_DTHETA, ds / math.hypot(r, b))

        cap = self.APPROACH_SPEED if err > self.LOCK_TOL else self.SWEEP_SPEED
        if err > 1e-6:
            speed = min(cap, self.K_P * err)
            wx, wy = speed * ex / err, speed * ey / err
        else:
            wx = wy = 0.0

        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * wx + s * wy
        vy = -s * wx + c * wy

        mag = math.hypot(vx, vy)
        if mag > 1e-6:
            heading, allowed = self._deflect(obs, math.atan2(vy, vx), mag)
            vx = allowed * math.cos(heading)
            vy = allowed * math.sin(heading)

        ports.drive(Command(vx, vy, 0.0))
        return Status.RUNNING

    def _ensure_state(self):
        if not hasattr(self, "_theta"):
            self.reset(None, None)

    def _deflect(self, obs, heading, mag):
        """Nearest clear heading to ``heading``, with a clearance-limited speed."""
        for offset in (0.0, 0.26, -0.26, 0.52, -0.52, 0.79, -0.79,
                       1.05, -1.05, 1.31, -1.31, 1.57, -1.57):
            cand = heading + offset
            clear = self._clearance(obs, cand)
            if clear > self.STOP_D:
                return cand, min(mag, 0.15 + 2.0 * (clear - self.STOP_D))

        # Boxed in on every candidate: creep toward the most open bearing.
        finite = np.where(np.isfinite(obs.ranges), obs.ranges, float(obs.range_max))
        return float(obs.range_angles[int(np.argmax(finite))]), 0.2

    def _clearance(self, obs, heading):
        delta = obs.range_angles - heading
        offset = np.abs(np.arctan2(np.sin(delta), np.cos(delta)))
        beams = obs.ranges[offset <= self.SECTOR_HALF]
        beams = beams[np.isfinite(beams)]
        if beams.size == 0:
            return float(obs.range_max)
        return float(np.min(beams))


def micro(seed=0):
    """Bare room, long straight run, no obstacles.

    With nothing to avoid, the only thing that can defeat ``goal_seeker`` here is
    pose drift accumulated over the 8 m approach: it converges on the believed goal
    and parks one drift-vector away from the true one. The isolated difference is
    coverage versus point convergence, which is exactly the claimed effect.
    """
    world = room(10.0, 5.0)
    return MicroScenario(
        id="uncertainty_scaled_goal_sweep",
        world=world,
        start=(0.9, 2.5, 0.0),
        goal=(8.9, 2.5),
        time_limit=50.0,
        seed=seed,
        goal_known=True,
    )
