"""Boundary-geometry reorientation for a Go2 with a 181-beam planar scan.

Implements `geometric_boundary_reorientation`: when the robot's own estimate of
its pose error has degraded, freeze translation, extract the principal axis of
the surrounding surface layout from the scan, and rotate the believed yaw onto
the axis frame that was recorded while the belief was still trustworthy.

Why it should work: dead reckoning drift is unbounded, but walls do not move.
The second-moment matrix of the finite returns is dominated by whatever subtends
many beams -- i.e. extended surfaces -- and is nearly insensitive to any single
small feature, which is the sensor analogue of Cheng's geometric module. That
reference is only ever determined up to the symmetry group of the layout, so the
node refuses any correction close enough to the +/-45 degree fold boundary to be
a symmetry misassignment: committing the rotational error would inject a 90
degree heading error into a belief that was merely noisy.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.15,
    "cost_metric": "time_to_goal",
}


def _wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ReorientToBoundaryGeometry(Action):
    """Re-anchor believed yaw to the principal axis of the visible boundary.

    The axis of an undirected point cloud is defined modulo 180 degrees, and an
    indoor layout is additionally near-symmetric under 90 degree rotation, so
    every axis angle is carried here as the unit vector (cos 4t, sin 4t). That
    folds the 90 degree symmetry onto the full circle: circular averaging of the
    reference and the reduction of the correction into [-45, +45) both become
    ordinary operations on that vector, with no modular branch to get wrong.

    Emits a zero Twist for the duration of the measurement freeze -- the frozen
    locomotion policy sways the body while walking, and standing still for 0.5 s
    is cheaper than modelling the sway.
    """

    PORTS = (
        Port("reanchored_pose", PortType.POSE, Direction.OUT),
        Port("pose_belief_confidence", PortType.CONFIDENCE, Direction.OUT),
    )
    PRECONDITION = (
        "The believed pose error has exceeded 0.5 m, a reference boundary axis was "
        "recorded earlier while the belief was trusted, the current scan defines an "
        "admissible axis, and at least 5 s has passed since the last completed attempt."
    )
    EFFECT = (
        "Rotates the believed yaw onto the stored boundary-axis frame, republishing the "
        "pose belief with x and y untouched and its heading re-anchored to the walls."
    )
    TAGS = ("localisation", "reorientation", "boundary", "geometry", "yaw", "drift")

    TRIGGER_ERR_M = 0.5       # belief error above which re-anchoring pays for its time
    TRUST_ERR_M = 0.25        # belief error below which the reference may be laid down
    COOLDOWN_S = 5.0
    FREEZE_S = 0.5
    REF_TAU_S = 2.0           # smooths gait-induced body sway out of the reference
    MIN_FINITE_BEAMS = 40     # ~22% of the circle; below this the cloud is a fragment
    MAX_EIG_RATIO = 0.6       # above this the view is too symmetric to define an axis
    REFUSAL_MARGIN = math.radians(5.0)
    ERR_SHRINK = 0.7          # heading is only part of belief error; position is untouched
    FOLD = 4.0                # 90 deg symmetry -> full circle

    # ---------------------------------------------------------------- state

    def reset(self, scenario, rng):
        self._ref = None              # (cos 4t, sin 4t), EMA of the world-frame axis
        self._last_seen_t = None
        self._last_fire_end_t = -1e9
        self._freeze_start_t = None
        self._freeze_acc = (0.0, 0.0)
        self._freeze_n = 0
        self._yaw_offset = 0.0        # cumulative correction applied to the belief
        self._err_scale = 1.0
        self._axis_cache = None

    def _ensure(self):
        if not hasattr(self, "_ref"):
            self.reset(None, None)

    # ------------------------------------------------------------ geometry

    def _axis_angle(self, obs):
        """Robot-frame principal axis of the visible surfaces, or None if inadmissible."""
        if self._axis_cache is not None and self._axis_cache[0] == obs.t:
            return self._axis_cache[1]

        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        usable = np.isfinite(ranges) & (ranges > 0.0) & (ranges < float(obs.range_max))

        angle = None
        if int(np.count_nonzero(usable)) >= self.MIN_FINITE_BEAMS:
            r = ranges[usable]
            a = angles[usable]
            pts = np.stack((r * np.cos(a), r * np.sin(a)), axis=1)
            pts = pts - pts.mean(axis=0)
            moment = (pts.T @ pts) / float(pts.shape[0])
            evals, evecs = np.linalg.eigh(moment)  # ascending
            lo, hi = float(evals[0]), float(evals[1])
            # A ratio near 1 is an open floor or a room centre: nothing to lock onto,
            # and any correction derived from it would be arbitrary.
            if hi > 1e-9 and (lo / hi) < self.MAX_EIG_RATIO:
                major = evecs[:, 1]
                angle = math.atan2(float(major[1]), float(major[0]))

        self._axis_cache = (obs.t, angle)
        return angle

    def _believed_yaw(self, obs):
        return _wrap_pi(float(obs.pose_estimate[2]) + self._yaw_offset)

    def _fold(self, theta):
        return (math.cos(self.FOLD * theta), math.sin(self.FOLD * theta))

    def _track_reference(self, obs):
        """Refresh the trusted-belief reference axis. Runs once per observation.

        Lives outside tick() because the reference may only be recorded while the
        belief is trusted, which is exactly when the trigger is false and tick()
        is never reached.
        """
        if self._last_seen_t is not None and obs.t <= self._last_seen_t:
            return
        dt = 0.0 if self._last_seen_t is None else float(obs.t) - self._last_seen_t
        self._last_seen_t = float(obs.t)

        if float(obs.pose_error_estimate) > self.TRUST_ERR_M:
            return
        axis = self._axis_angle(obs)
        if axis is None:
            return

        c, s = self._fold(self._believed_yaw(obs) + axis)
        if self._ref is None or dt <= 0.0:
            self._ref = (c, s)
            return
        alpha = 1.0 - math.exp(-dt / self.REF_TAU_S)
        self._ref = (
            self._ref[0] + alpha * (c - self._ref[0]),
            self._ref[1] + alpha * (s - self._ref[1]),
        )

    def _publish(self, obs, ports):
        x, y, _ = obs.pose_estimate
        ports["reanchored_pose"] = (float(x), float(y), self._believed_yaw(obs))
        err = max(0.0, float(obs.pose_error_estimate) * self._err_scale)
        ports["pose_belief_confidence"] = 1.0 / (1.0 + err / self.TRIGGER_ERR_M)

    # --------------------------------------------------------------- ticks

    def precondition(self, obs, ports):
        self._ensure()
        self._track_reference(obs)

        # An already-running freeze stays admissible so it can finish or abort
        # deliberately rather than being stranded mid-measurement.
        if (self._freeze_start_t is not None
                and float(obs.t) - self._freeze_start_t <= 2.0 * self.FREEZE_S):
            return True
        self._freeze_start_t = None

        if self._ref is None:
            return False
        if float(obs.pose_error_estimate) <= self.TRIGGER_ERR_M:
            return False
        if float(obs.t) - self._last_fire_end_t < self.COOLDOWN_S:
            return False
        return self._axis_angle(obs) is not None

    def tick(self, obs, ports):
        self._ensure()
        self._track_reference(obs)

        if self._ref is None:
            self._publish(obs, ports)
            return Status.FAILURE

        axis = self._axis_angle(obs)
        if axis is None:
            self._freeze_start_t = None
            self._last_fire_end_t = float(obs.t)
            self._publish(obs, ports)
            return Status.FAILURE

        stale = (self._freeze_start_t is not None
                 and float(obs.t) - self._freeze_start_t > 2.0 * self.FREEZE_S)
        if self._freeze_start_t is None or stale:
            self._freeze_start_t = float(obs.t)
            self._freeze_acc = (0.0, 0.0)
            self._freeze_n = 0

        c, s = self._fold(self._believed_yaw(obs) + axis)
        self._freeze_acc = (self._freeze_acc[0] + c, self._freeze_acc[1] + s)
        self._freeze_n += 1

        ports.drive(Command(0.0, 0.0, 0.0))

        if float(obs.t) - self._freeze_start_t < self.FREEZE_S:
            self._publish(obs, ports)
            return Status.RUNNING

        self._freeze_start_t = None
        self._last_fire_end_t = float(obs.t)

        acc_x, acc_y = self._freeze_acc
        if self._freeze_n == 0 or (acc_x * acc_x + acc_y * acc_y) < 1e-12:
            self._publish(obs, ports)
            return Status.FAILURE

        now = math.atan2(acc_y, acc_x)
        ref = math.atan2(self._ref[1], self._ref[0])
        # Dividing the folded difference by 4 is exactly wrap_to_pm45.
        d = _wrap_pi(ref - now) / self.FOLD

        # The refusal. Within 5 degrees of the fold boundary the assignment between
        # adjacent symmetry copies is a coin flip, and guessing wrong injects a 90
        # degree error into a belief that was only noisy. Change nothing.
        if abs(d) > (math.pi / 4.0) - self.REFUSAL_MARGIN:
            self._publish(obs, ports)
            return Status.FAILURE

        self._yaw_offset = _wrap_pi(self._yaw_offset + d)
        # Non-cumulative: heading is re-anchored, position is not, so the working
        # error estimate is shrunk once rather than compounded over firings.
        self._err_scale = self.ERR_SHRINK
        self._publish(obs, ports)
        return Status.SUCCESS


def micro(seed=0):
    """Long narrow hall, two staggered partitions, goal past both.

    Narrow on purpose: a squarer room gives a rotationally symmetric return cloud
    and the axis test correctly refuses to fire at all. The start-to-goal straight
    line clears the first partition by 0.14 m, less than the 0.3 m robot radius,
    so the sensorless baseline drives into it and stays there.
    """
    world = room(
        15.0,
        2.8,
        boxes=[
            (5.7, 0.0, 6.1, 1.1),   # opening above, y in [1.1, 2.8]
            (9.7, 1.7, 10.1, 2.8),  # opening below, y in [0.0, 1.7]
        ],
    )
    return MicroScenario(
        id="boundary_reorientation_staggered_hall",
        world=world,
        start=(1.0, 0.7, 0.0),
        goal=(14.0, 1.4),
        time_limit=120.0,
        seed=seed,
        goal_known=True,
    )
