import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status


class SelectDynamicWindowVelocity(Action):
    """Dynamic-window velocity selection (Fox, Burgard & Thrun, Eq. 13-16).

    Instead of choosing a heading and then a speed, this node searches the
    two-dimensional space of velocity pairs (vx, wz) reachable within one
    control interval, reads each pair as the circular arc the body would
    follow if it held that command, and keeps only the pairs from which
    braking at the maximum rate still stops the robot short of the nearest
    obstacle on that arc.  Over the survivors it maximises a smoothed
    weighted sum of goal heading, clearance and speed.

    Revises vx and wz because the mechanism selects the pair jointly -- the
    admissibility test in Eq. (14) couples them through dist(v, w), so
    neither field can be chosen without the other.  Caps vy to zero because
    the arc geometry (and therefore the braking guarantee) is only valid for
    a command with no lateral component; that restriction is the derivation's
    resolution of the paper's non-holonomic synchro-drive onto a holonomic
    Twist.

    Declines -- leaving the base command byte-identical -- when the goal has
    not been disclosed, when the robot is already in contact, or when no
    candidate in the sampled window is admissible.  The last case is the
    paper's hand-off to a separate recovery behaviour ("rotate away mode",
    Sec. 5.3), which is a different card and is deliberately not implemented
    here.
    """

    WRITES = ("vx", "vy", "wz")
    PORTS = (Port("safe_speed_limit", PortType.SPEED, Direction.OUT),)
    PRECONDITION = (
        "The goal is disclosed and the robot is not currently in contact."
    )
    EFFECT = (
        "Replaces the commanded velocity with the highest-scoring pair in the "
        "admissible dynamic window, so that braking to a stop before the "
        "nearest obstacle on the committed arc always remains possible."
    )
    TAGS = ("dwa", "velocity-space", "admissible", "clearance", "local-planner")

    # Search space Vs.  Reverse translation is not sampled: the frozen
    # locomotion policy is not characterised backwards.
    V_MAX = 0.8
    W_MAX = 1.0

    # Slew limits per control interval, and the braking accelerations they
    # imply at the paper's dt = 0.25 s cycle.  Obtained by holding RHINO's
    # window-to-range ratios fixed against our own limits.
    DV = 0.11
    DW = 0.175
    DT = 0.25
    ACC_V = 0.45
    ACC_W = 0.7

    N_V = 7
    N_W = 15

    # Circumscribed Go2 radius ~0.40 m plus a 0.10 m margin.
    R_INFL = 0.50

    ALPHA = 0.8
    BETA = 0.1
    GAMMA = 0.1

    W_STRAIGHT = 1e-3
    V_STOPPED = 1e-6

    def reset(self, scenario, rng):
        self._prev_v = 0.0
        self._prev_w = 0.0
        self._active = False

    def precondition(self, obs, ports) -> bool:
        if getattr(obs, "goal", None) is None:
            return False
        if obs.contact:
            return False
        return True

    def tick(self, obs, ports) -> Status:
        if not getattr(self, "_active", False):
            # First step after activation: Eq. (15) is anchored at rest,
            # since the observation carries no measured body velocity.
            self._prev_v = 0.0
            self._prev_w = 0.0

        range_max = float(obs.range_max)
        px, py = self._obstacle_points(obs, range_max)

        v_grid = np.linspace(
            max(0.0, self._prev_v - self.DV),
            min(self.V_MAX, self._prev_v + self.DV),
            self.N_V,
        )
        w_grid = np.linspace(
            max(-self.W_MAX, self._prev_w - self.DW),
            min(self.W_MAX, self._prev_w + self.DW),
            self.N_W,
        )

        scores = np.zeros((self.N_V, self.N_W), dtype=float)
        admissible = np.zeros((self.N_V, self.N_W), dtype=bool)
        dists = np.zeros((self.N_V, self.N_W), dtype=float)

        goal = obs.goal
        pose = obs.pose_estimate

        for i, v in enumerate(v_grid):
            for j, w in enumerate(w_grid):
                dist = self._arc_distance(v, w, px, py, range_max)
                dists[i, j] = dist

                # Eq. (14): keep only commands we can still brake out of.
                if v > math.sqrt(2.0 * dist * self.ACC_V):
                    continue
                if abs(w) > math.sqrt(2.0 * dist * self.ACC_W):
                    continue
                admissible[i, j] = True

                heading = self._heading_term(v, w, pose, goal)
                clearance = dist / range_max
                # A stationary candidate scores no speed reward, so pure
                # rotation only wins when nothing that translates survives.
                speed = 0.0 if v <= self.V_STOPPED else v / self.V_MAX
                scores[i, j] = (
                    self.ALPHA * heading
                    + self.BETA * clearance
                    + self.GAMMA * speed
                )

        if not admissible.any():
            # No command is recoverable: hand back to the base, and re-anchor
            # the window at rest for whenever we next activate.
            self._active = False
            return Status.FAILURE

        smoothed = self._smooth(scores, admissible)
        smoothed = np.where(admissible, smoothed, -np.inf)
        i, j = np.unravel_index(int(np.argmax(smoothed)), smoothed.shape)

        best_v = float(v_grid[i])
        best_w = float(w_grid[j])

        # Only the first command of the plan is emitted; the whole search is
        # redone next step against a fresh scan.
        ports.adjust(vx=best_v, wz=best_w)
        ports.cap(vy=0.0)

        ports["safe_speed_limit"] = float(
            np.clip(math.sqrt(2.0 * dists[i, j] * self.ACC_V), 0.0, self.V_MAX)
        )

        self._prev_v = best_v
        self._prev_w = best_w
        self._active = True
        return Status.SUCCESS

    def _obstacle_points(self, obs, range_max):
        """Scan returns as points in the robot frame, as the paper's obstacle
        line field but taken straight off the 181 beams."""
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        valid = np.isfinite(ranges) & (ranges > 0.0) & (ranges <= range_max)
        if not valid.any():
            empty = np.empty(0, dtype=float)
            return empty, empty
        r = ranges[valid]
        a = angles[valid]
        return r * np.cos(a), r * np.sin(a)

    def _arc_distance(self, v, w, px, py, range_max):
        """Distance along the arc (v, w) to the first inflated obstacle,
        saturated at range_max, which is also the paper's "large constant"
        for an arc with nothing on it."""
        if px.size == 0:
            return range_max
        if v <= self.V_STOPPED:
            return range_max

        if abs(w) < self.W_STRAIGHT:
            ahead = (np.abs(py) <= self.R_INFL) & (px > 0.0)
            if not ahead.any():
                return range_max
            return float(np.clip(px[ahead].min() - self.R_INFL, 0.0, range_max))

        radius = v / w
        cy = radius
        d = np.hypot(px, py - cy)
        hit = np.abs(d - abs(radius)) <= self.R_INFL
        if not hit.any():
            return range_max

        # Sweep angle from the robot's own bearing about the arc centre,
        # measured in the direction of travel; the paper's "gamma * r",
        # renamed phi here to keep gamma for the objective weight.
        a0 = math.atan2(-radius, 0.0)
        ap = np.arctan2(py[hit] - cy, px[hit])
        phi = np.mod(math.copysign(1.0, w) * (ap - a0), 2.0 * math.pi)
        return float(min(abs(radius) * float(phi.min()), range_max))

    def _heading_term(self, v, w, pose, goal):
        """1 - |theta|/pi, with theta the goal bearing from the pose reached
        after one interval on this arc plus the braking tail."""
        x, y, yaw = float(pose[0]), float(pose[1]), float(pose[2])

        if abs(w) < self.W_STRAIGHT:
            x += v * self.DT * math.cos(yaw)
            y += v * self.DT * math.sin(yaw)
        else:
            radius = v / w
            yaw_next = yaw + w * self.DT
            x += radius * (math.sin(yaw_next) - math.sin(yaw))
            y += radius * (math.cos(yaw) - math.cos(yaw_next))
            yaw = yaw_next

        yaw += math.copysign(w * w / (2.0 * self.ACC_W), w)
        brake_travel = v * v / (2.0 * self.ACC_V)
        x += brake_travel * math.cos(yaw)
        y += brake_travel * math.sin(yaw)

        theta = self._wrap(math.atan2(float(goal[1]) - y, float(goal[0]) - x) - yaw)
        return 1.0 - abs(theta) / math.pi

    def _smooth(self, scores, admissible):
        """sigma from Eq. (13), as a 3x3 box blur that averages only over
        admissible neighbours so inadmissible cells cannot drag a score down."""
        weights = admissible.astype(float)
        padded_s = np.pad(scores * weights, 1, mode="constant")
        padded_w = np.pad(weights, 1, mode="constant")
        rows, cols = scores.shape

        num = np.zeros_like(scores)
        den = np.zeros_like(scores)
        for dy in range(3):
            for dx in range(3):
                num += padded_s[dy:dy + rows, dx:dx + cols]
                den += padded_w[dy:dy + rows, dx:dx + cols]

        return np.where(den > 0.0, num / np.maximum(den, 1e-9), scores)

    @staticmethod
    def _wrap(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


PREDICTION = {
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.4,
    "cost_metric": "time_to_goal",
}
