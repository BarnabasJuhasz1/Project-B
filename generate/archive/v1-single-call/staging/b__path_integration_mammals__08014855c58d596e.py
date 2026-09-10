"""Arbitrated homing vector (Etienne & Jeffery 2004, path integration in mammals).

Mammals do not navigate on dead reckoning alone: the integrated home vector is
continuously weighed against other spatial cues, and dominates exactly when
those cues become untrustworthy. Here the two cues are

  * the pose-derived goal vector, whose error grows with TIME (the runner
    reports it directly as ``obs.pose_error_estimate``), and
  * a self-maintained dead-reckoned goal vector, anchored once and thereafter
    updated only from the twists this node commanded, whose error grows with
    PATH LENGTH.

Neither dominates everywhere, so the node computes the crossover rather than
picking a side: the emitted bearing is the inverse-variance blend of the two.
Under pose drift this keeps a usable heading long after the pose belief has
rotated the goal out from under a purely reactive controller.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "spl",
}

# Candidate headings ordered by how far they deviate from the arbitrated
# bearing, so the first admissible one is the minimal deflection.
_OFFSETS = tuple(sorted((math.radians(d) for d in range(-110, 111, 5)), key=abs))


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_arr(angles):
    return np.arctan2(np.sin(angles), np.cos(angles))


class ArbitrateHomingVector(Action):
    """Drive the inverse-variance arbitration of a pose-derived and a
    dead-reckoned goal vector.

    Why it should work: the pose belief and the integrated vector fail in
    orthogonal ways. Drift injected into ``pose_estimate`` is invisible to the
    integrator (which never reads pose after anchoring), and integration slip is
    invisible to the pose belief. Weighting each by its own declared error means
    a heading survives whichever cue is currently rotting, instead of following
    a single cue off the map.
    """

    SOURCE = ("Etienne, A. S. & Jeffery, K. J. (2004). 'Path integration in "
              "mammals.' Hippocampus 14(2), 180-192.")
    PRINCIPLE_ID = "path_integration_mammals"

    PORTS = (
        Port("arbitrated_goal_bearing", PortType.BEARING, Direction.OUT),
        Port("arbitrated_goal_confidence", PortType.CONFIDENCE, Direction.OUT),
    )

    PRECONDITION = ("The goal has been disclosed and a pose belief exists, so a "
                    "pose-derived and an integrated goal vector are both "
                    "available to arbitrate between.")
    EFFECT = ("Moves the robot along the inverse-variance arbitration of the "
              "pose-derived and dead-reckoned goal vectors.")
    TAGS = ("path-integration", "dead-reckoning", "cue-arbitration",
            "pose-drift", "goal-seeking")

    # --- error models -------------------------------------------------------
    # Integrated vector: metres of accumulated error per metre travelled.
    K_PI = 0.05
    # Neither cue is ever credited with better than this, so the weights stay
    # finite when the runner reports a zero pose error.
    SIGMA_FLOOR = 0.05
    # Blocked motion means commanded displacement overstates real displacement;
    # charge the integrator extra path length for it.
    CONTACT_SLIP_FACTOR = 3.0

    # --- steering -----------------------------------------------------------
    BODY_R = 0.3
    MARGIN = 0.25
    MIN_LOOKAHEAD = 0.6
    MAX_LOOKAHEAD = 1.8
    SPEED = 0.7
    VY_MAX = 0.35
    YAW_GAIN = 1.8
    ARRIVE = 0.35
    MAX_DT = 0.5

    def reset(self, scenario, rng):
        self._t_prev = None
        self._last_cmd = (0.0, 0.0, 0.0)
        self._pi_vec = None        # goal in the robot frame, dead reckoned
        self._path_len = 0.0

    def _ensure_state(self):
        if not hasattr(self, "_pi_vec"):
            self.reset(None, None)

    def precondition(self, obs, ports):
        return obs.goal is not None and obs.pose_estimate is not None

    # -- cue 1: dead reckoning ----------------------------------------------
    def _integrate(self, obs, dt):
        """Carry the goal vector through the motion we asked for last tick."""
        vx, vy, wz = self._last_cmd
        gx, gy = self._pi_vec

        # The robot translated (vx, vy)*dt in its own frame, so the goal moved
        # the other way; then the frame itself yawed by wz*dt.
        gx -= vx * dt
        gy -= vy * dt
        rot = -wz * dt
        c, s = math.cos(rot), math.sin(rot)
        self._pi_vec = (c * gx - s * gy, s * gx + c * gy)

        step = math.hypot(vx, vy) * dt
        self._path_len += step * (self.CONTACT_SLIP_FACTOR if obs.contact else 1.0)

    # -- cue 2: pose belief --------------------------------------------------
    @staticmethod
    def _pose_vec(obs):
        px, py, yaw = obs.pose_estimate
        gx, gy = obs.goal
        dx, dy = gx - px, gy - py
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * dx + s * dy, -s * dx + c * dy)

    # -- minimal-deflection admissibility ------------------------------------
    def _choose_heading(self, obs, bearing, dist):
        ranges = np.where(np.isfinite(obs.ranges), obs.ranges, obs.range_max)
        lookahead = float(np.clip(dist, self.MIN_LOOKAHEAD, self.MAX_LOOKAHEAD))
        # Angular half-width the body occupies at that lookahead.
        half = float(np.clip(math.atan2(self.BODY_R + self.MARGIN, lookahead),
                             0.12, 1.0))
        needed = lookahead + self.MARGIN

        best_free, best_free_theta = -1.0, bearing
        for off in _OFFSETS:
            theta = _wrap(bearing + off)
            sector = np.abs(_wrap_arr(obs.range_angles - theta)) <= half
            if not sector.any():
                continue
            clear = float(ranges[sector].min())
            if clear > best_free:
                best_free, best_free_theta = clear, theta
            if clear >= needed:
                return theta, off, clear, True
        return best_free_theta, _wrap(best_free_theta - bearing), best_free, False

    def tick(self, obs, ports):
        self._ensure_state()

        if obs.goal is None or obs.pose_estimate is None:
            # Nothing to arbitrate; a sibling node owns this situation.
            self._t_prev = obs.t
            return Status.FAILURE

        dt = 0.0 if self._t_prev is None else min(max(obs.t - self._t_prev, 0.0),
                                                  self.MAX_DT)
        self._t_prev = obs.t

        pose_vec = self._pose_vec(obs)

        if self._pi_vec is None:
            # Anchor the integrator once, from the only cue available at t0.
            self._pi_vec = pose_vec
            self._path_len = 0.0
        elif dt > 0.0:
            self._integrate(obs, dt)

        d_pose = math.hypot(*pose_vec)
        d_pi = math.hypot(*self._pi_vec)

        # Inverse-variance arbitration: time-growing error against
        # distance-growing error. This is the crossover the card asks for.
        sigma_pose = max(float(obs.pose_error_estimate), self.SIGMA_FLOOR)
        sigma_pi = max(self.K_PI * self._path_len, self.SIGMA_FLOOR)
        raw_pose = 1.0 / (sigma_pose * sigma_pose)
        raw_pi = 1.0 / (sigma_pi * sigma_pi)
        total = raw_pose + raw_pi
        w_pose, w_pi = raw_pose / total, raw_pi / total

        b_pose = math.atan2(pose_vec[1], pose_vec[0])
        b_pi = math.atan2(self._pi_vec[1], self._pi_vec[0])
        ux = w_pose * math.cos(b_pose) + w_pi * math.cos(b_pi)
        uy = w_pose * math.sin(b_pose) + w_pi * math.sin(b_pi)
        # The two cues can only cancel if they point opposite ways; trust the
        # one with the smaller declared error rather than emitting a coin flip.
        if math.hypot(ux, uy) < 1e-6:
            bearing = b_pose if w_pose >= w_pi else b_pi
        else:
            bearing = math.atan2(uy, ux)
        dist = w_pose * d_pose + w_pi * d_pi

        sigma_fused = 1.0 / math.sqrt(total)
        confidence = float(np.clip(1.0 / (1.0 + sigma_fused), 0.0, 1.0))
        ports["arbitrated_goal_bearing"] = float(bearing)
        ports["arbitrated_goal_confidence"] = confidence

        if dist <= self.ARRIVE:
            cmd = Command(0.0, 0.0, 0.0)
            self._last_cmd = (0.0, 0.0, 0.0)
            ports.drive(cmd)
            return Status.SUCCESS

        theta, offset, clear, admissible = self._choose_heading(obs, bearing, dist)

        if not admissible:
            # Boxed in: turn towards the freest direction and creep, so the
            # integrator keeps accruing a small, honest amount of path length.
            wz = float(np.clip(self.YAW_GAIN * theta, -1.0, 1.0))
            vx = 0.05 if clear > self.BODY_R + self.MARGIN else 0.0
            cmd = Command(vx, 0.0, wz)
            self._last_cmd = (vx, 0.0, wz)
            ports.drive(cmd)
            return Status.RUNNING

        speed = self.SPEED
        speed *= float(np.clip(clear / (2.0 * self.MAX_LOOKAHEAD), 0.25, 1.0))
        speed *= 1.0 - 0.5 * abs(offset) / math.pi   # slow down when deflecting
        speed = min(speed, max(0.15, dist))          # and when nearly there

        vx = speed * math.cos(theta)
        vy = float(np.clip(speed * math.sin(theta), -self.VY_MAX, self.VY_MAX))
        # Yaw follows the direction of travel, keeping the densest part of the
        # scan pointed where the body is actually going.
        wz = float(np.clip(self.YAW_GAIN * theta, -1.0, 1.0))

        cmd = Command(vx, vy, wz)
        self._last_cmd = (vx, vy, wz)
        ports.drive(cmd)
        return Status.RUNNING


def micro(seed=0):
    """One doorway roughly on the goal bearing, and a wider dead-end alcove
    beside it whose mouth also 'makes progress'.

    A gap follower prefers the wider mouth and, once its goal belief has
    drifted, has nothing left to re-acquire the doorway with. The arbitrated
    vector keeps pointing at the doorway because half of it never read pose.
    """
    world = room(
        10.0, 8.0,
        boxes=[
            (4.7, 0.0, 5.3, 4.3),    # divider, below the doorway
            (4.7, 5.7, 5.3, 8.0),    # divider, above the doorway
            (3.2, 1.2, 4.7, 1.7),    # alcove floor
            (3.2, 3.2, 4.7, 3.7),    # alcove ceiling  (mouth 1.5 m > door 1.4 m)
        ],
    )
    return MicroScenario(
        id="arbitrate_homing_vector_doorway",
        world=world,
        start=(1.3, 3.9, 0.0),
        goal=(8.5, 3.5),
        time_limit=50.0,
        seed=seed,
        goal_known=True,
    )
