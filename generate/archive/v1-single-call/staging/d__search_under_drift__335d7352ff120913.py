"""Coverage-as-elimination search for a Go2 with a drifting pose belief."""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PRINCIPLE = {
    "mechanism": (
        "Maintain a posterior over where an undiscovered target can still be, "
        "on the floor, in the drifting belief frame. Ground the robot has "
        "passed is eliminated, but only up to the accuracy of the frame it was "
        "recorded in, so each breadcrumb eliminates a disc dilated by the "
        "reported pose error rather than a point. Disclosure of the goal is an "
        "observation on that same posterior, collapsing it onto a blob whose "
        "width is again set by the pose error. Driving the argmax of "
        "(traversable depth x posterior mass) therefore searches before "
        "disclosure and approaches after it, with one rule and no mode switch."
    ),
    "trigger": (
        "Applies whenever the robot must choose a heading in a space it has no "
        "map of, while its own position estimate carries a non-zero, reported "
        "error -- i.e. whenever a coverage record must be written in the same "
        "frame that is decaying."
    ),
    "derivation": (
        "The 181-beam scan gives, per beam, a bearing and a free depth; the "
        "endpoint of each beam is a candidate place to go, projected through "
        "pose_estimate. Elimination radius r = R0 + K*pose_error_estimate, "
        "capped, where R0 is a conservative under-estimate of the range at "
        "which the target would have been detected in passing; novelty at an "
        "endpoint is clip(distance-to-nearest-breadcrumb / r, 0, 1), so a "
        "growing belief error monotonically shrinks the area the robot claims "
        "to have cleared. Depth is min-filtered over a +-8 deg window so a "
        "0.6 m disc actually fits through the winner, and the trail is bounded "
        "because a breadcrumb older than the drift horizon carries no usable "
        "information about the present frame."
    ),
}

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _circular_min(values, half_width):
    """Min over a +-half_width neighbourhood; the scan closes on itself."""
    out = values.copy()
    for k in range(1, half_width + 1):
        out = np.minimum(out, np.roll(values, k))
        out = np.minimum(out, np.roll(values, -k))
    return out


class TargetPosteriorSearch(Action):
    """Drive at the reachable direction holding the most target-posterior mass.

    Coverage memory is treated as a record of eliminated target hypotheses, not
    as a map. Each breadcrumb eliminates a disc whose radius grows with
    obs.pose_error_estimate, because a visit recorded in a drifting frame only
    says "the target was not within the drift of here". Disclosure of the goal
    multiplies the same posterior by a Gaussian likelihood, so the identical
    argmax that was exploring starts approaching -- and keeps sweeping the
    uncertainty blob, rather than parking on a point the drift has displaced.
    """

    PORTS = (
        Port("target_bearing", PortType.BEARING, Direction.OUT),
        Port("target_confidence", PortType.CONFIDENCE, Direction.OUT),
    )
    PRECONDITION = (
        "At least one direction in the scan offers more than a body length of "
        "traversable depth."
    )
    EFFECT = (
        "Moves the robot toward the reachable direction carrying the most "
        "posterior mass for the undiscovered target."
    )
    TAGS = ("search", "exploration", "coverage", "drift", "posterior", "twist")

    # Elimination kernel. R0 deliberately under-estimates the range at which the
    # target would be spotted in passing: under-claiming coverage costs path
    # length, over-claiming loses the target permanently.
    ELIM_R0 = 0.8
    ELIM_GAIN = 1.6
    ELIM_R_MAX = 2.5

    TRAIL_STEP = 0.3          # metres of believed travel between breadcrumbs
    TRAIL_MAX = 500           # memory horizon: older records predate the frame

    GAP_HALF_BEAMS = 4        # ~+-8 deg: the winning gap must fit a 0.6 m disc
    SAFE_MARGIN = 0.45        # robot radius plus standoff
    FRONT_HALF_ANGLE = 0.44   # rad, the wedge that must be clear to translate

    V_MAX = 0.6
    W_GAIN = 1.6
    W_MAX = 1.0
    ALIGN_MIN = 0.35          # below this cosine, turn in place
    STICKY = 1.08             # tie-break toward the previous choice
    STICKY_WINDOW = 0.26      # rad

    SATURATED = 0.05          # posterior below which the space is cleared
    BOXED_IN = 0.35           # metres of best depth below which we cannot act

    _trail = None
    _last_crumb = None
    _last_heading = 0.0

    def reset(self, scenario, rng):
        self._trail = []
        self._last_crumb = None
        self._last_heading = 0.0

    def precondition(self, obs, ports):
        ranges = self._finite_ranges(obs)
        return bool(ranges.max() > 0.6)

    def tick(self, obs, ports):
        if self._trail is None:
            self._trail = []

        px, py, yaw = obs.pose_estimate
        self._record(px, py)

        ranges = self._finite_ranges(obs)
        angles = np.asarray(obs.range_angles, dtype=float)

        # Work in angle-sorted order so the circular min-filter is meaningful.
        order = np.argsort(angles)
        angles = angles[order]
        ranges = ranges[order]

        depth = np.clip(_circular_min(ranges, self.GAP_HALF_BEAMS)
                        - self.SAFE_MARGIN, 0.0, float(obs.range_max))
        if depth.max() < self.BOXED_IN:
            # Every direction is blocked; this node has no move to offer.
            ports.drive(Command(0.0, 0.0, 0.0))
            ports["target_bearing"] = 0.0
            ports["target_confidence"] = 0.0
            return Status.FAILURE

        world_angles = angles + yaw
        endpoints = np.stack(
            (px + depth * np.cos(world_angles),
             py + depth * np.sin(world_angles)), axis=1)

        radius = min(self.ELIM_R_MAX,
                     self.ELIM_R0
                     + self.ELIM_GAIN * float(obs.pose_error_estimate))
        posterior = self._novelty(endpoints, radius)

        disclosed = obs.goal is not None
        if disclosed:
            posterior = posterior * self._likelihood(endpoints, obs)

        best_posterior = float(posterior.max())
        if not disclosed and best_posterior < self.SATURATED:
            # Nothing reachable can still hold the target at this resolution.
            ports.drive(Command(0.0, 0.0, 0.0))
            ports["target_bearing"] = 0.0
            ports["target_confidence"] = 0.0
            return Status.SUCCESS

        score = (depth / max(float(obs.range_max), 1e-6)) * posterior
        near_last = np.abs(_wrap(angles - self._last_heading)) < self.STICKY_WINDOW
        score = np.where(near_last, score * self.STICKY, score)

        best = int(np.argmax(score))
        heading = float(angles[best])
        self._last_heading = heading

        err = _wrap(heading)
        wz = float(np.clip(self.W_GAIN * err, -self.W_MAX, self.W_MAX))

        align = math.cos(err)
        front = float(ranges[np.abs(_wrap(angles)) < self.FRONT_HALF_ANGLE].min())
        if align < self.ALIGN_MIN or front < self.SAFE_MARGIN:
            vx = 0.0
        else:
            # Ease off as the wedge ahead closes, so a turn always has room.
            throttle = min(1.0, max(0.2, (front - self.SAFE_MARGIN) / 1.0))
            vx = self.V_MAX * align * throttle

        ports.drive(Command(vx, 0.0, wz))
        ports["target_bearing"] = err
        ports["target_confidence"] = float(np.clip(best_posterior, 0.0, 1.0))
        return Status.RUNNING

    @staticmethod
    def _finite_ranges(obs):
        ranges = np.asarray(obs.ranges, dtype=float)
        return np.where(np.isfinite(ranges), ranges, float(obs.range_max))

    def _record(self, px, py):
        if self._last_crumb is not None:
            dx = px - self._last_crumb[0]
            dy = py - self._last_crumb[1]
            if (dx * dx + dy * dy) < self.TRAIL_STEP ** 2:
                return
        self._trail.append((px, py))
        self._last_crumb = (px, py)
        if len(self._trail) > self.TRAIL_MAX:
            del self._trail[0]

    def _novelty(self, points, radius):
        """1 where no breadcrumb eliminates the point, falling to 0 on the trail."""
        if not self._trail:
            return np.ones(len(points))
        trail = np.asarray(self._trail, dtype=float)
        deltas = points[:, None, :] - trail[None, :, :]
        nearest = np.sqrt((deltas * deltas).sum(axis=2)).min(axis=1)
        return np.clip(nearest / radius, 0.0, 1.0)

    def _likelihood(self, points, obs):
        """Collapse the posterior onto the disclosed goal, blurred by our drift.

        The disclosed point and our own endpoints are compared in a frame we
        know to be wrong by roughly pose_error_estimate, so the blob is widened
        by that amount; the floor keeps a badly displaced disclosure from
        freezing the search entirely.
        """
        gx, gy = obs.goal
        sigma = max(0.8, 2.5 * float(obs.pose_error_estimate))
        dx = points[:, 0] - gx
        dy = points[:, 1] - gy
        return np.maximum(np.exp(-0.5 * (dx * dx + dy * dy) / (sigma * sigma)),
                          0.02)


def micro(seed=0):
    """Two lobes joined by one narrow door; the target is in the far lobe.

    A memoryless gap follower is held in the near lobe: the door is never the
    widest gap on offer, and nothing makes the lobe it already swept less
    attractive on the next tick. Elimination makes the near lobe go flat, after
    which the door is the only posterior mass left.
    """
    world = room(
        12.0, 7.0,
        boxes=[
            (7.0, 0.0, 7.35, 4.3),    # dividing wall, lower section
            (7.0, 5.5, 7.35, 7.0),    # dividing wall, upper section
            (3.0, 3.6, 4.2, 4.8),     # pillars: several wide gaps in the near lobe
            (4.6, 1.0, 5.6, 2.0),
        ],
    )
    return MicroScenario(
        id="drift_search_two_lobe",
        world=world,
        start=(1.2, 1.2, 0.0),
        goal=(10.6, 1.4),
        time_limit=150.0,
        seed=seed,
        goal_known=False,
    )
