import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status


class SteerThroughWidestAdmissibleGap(Action):
    """Gap-clipped goal bearing.

    When the straight line to a disclosed goal is occluded, this node replaces
    the commanded heading (``wz``) and the forward speed (``vx``) with a heading
    aimed at the admissible free-space opening angularly closest to the goal,
    held one body-clearance off either jamb.

    It revises ``wz`` because the mechanism *is* a heading substitution: neither
    goal-pursuit nor pure repulsion gets through an aperture narrower than the
    obstacle beside it. It revises ``vx`` because the failure being repaired is
    a stall against the occluder face -- a cap cannot restore forward motion,
    only a set can -- and because the speed must fall off as the aim direction
    itself closes.

    It declines (FAILURE, nothing written, base command byte-identical) when the
    goal has not been disclosed, when the goal corridor is not actually
    obstructed, or when no admissible gap exists in the forward half-plane.
    Those three cases belong to an explorer, a straight-line pursuit node and a
    recovery node respectively. It never returns SUCCESS; the tree's
    goal-reached condition terminates it.
    """

    WRITES = ("vx", "wz")
    PORTS = (Port("gap_aim_bearing", PortType.BEARING, Direction.OUT),)
    PRECONDITION = (
        "The goal position is disclosed and the corridor along the goal bearing "
        "is obstructed within the distance the robot needs to turn."
    )
    EFFECT = (
        "Steers along the free-space-clipped goal bearing -- the admissible gap "
        "closest to the goal, aimed one body-clearance inside its edges."
    )
    TAGS = ("gap", "steering", "occlusion", "aperture", "lidar", "subsumption")

    # Body scale: Go2 circumscribed radius ~0.38 m -> 0.40 m, plus 0.15 m margin.
    R_BODY = 0.40
    R_CLEAR = 0.55

    # Tightest commandable arc is v/w = 0.8/1.0 = 0.8 m; plus the body radius.
    D_BLOCK = 1.2
    # Half-width of the goal corridor: atan(R_CLEAR / D_BLOCK).
    CORRIDOR_HALF = 0.43
    # A beam is "free" only if the aperture does not close at the turning distance.
    D_FREE = 1.5

    # Beam spacing cannot resolve width beyond ~3 m, so cap the depth used for it.
    D_EVAL_MAX = 3.0
    # Body width 0.31 m plus ~0.145 m either side; below any real doorway (0.76 m).
    W_MIN = 0.60

    YAW_GAIN = 1.5
    YAW_MAX = 1.0
    YAW_DEADBAND = 0.05
    VX_MAX = 0.8
    VX_CONTACT = -0.2

    def reset(self, scenario, rng):
        self._last_aim = None

    # -- geometry helpers ------------------------------------------------

    @staticmethod
    def _wrap(a):
        """Wrap to (-pi, pi]; works on scalars and arrays."""
        return np.arctan2(np.sin(a), np.cos(a))

    def _scan(self, obs):
        """Angle-sorted beams with inf / over-range treated as free at range_max."""
        angles = self._wrap(np.asarray(obs.range_angles, dtype=float))
        raw = np.asarray(obs.ranges, dtype=float)
        r_max = float(obs.range_max)
        # An unreturned beam is free at this range, not unknown.
        ranges = np.where(np.isfinite(raw), raw, r_max)
        ranges = np.clip(ranges, 0.0, r_max)
        order = np.argsort(angles)
        return angles[order], ranges[order], r_max

    def _min_range_near(self, angles, ranges, center, half_width, r_max):
        mask = np.abs(self._wrap(angles - center)) <= half_width
        if not np.any(mask):
            return r_max
        return float(np.min(ranges[mask]))

    def _goal_bearing(self, obs):
        """Goal bearing in the robot frame, or None if the goal is undisclosed.

        Only the bearing is consumed and it is recomputed from scratch every
        step, so pose drift perturbs the *preference* and never the geometry.
        """
        goal = obs.goal
        if goal is None:
            return None
        gx, gy = float(goal[0]), float(goal[1])
        x, y, yaw = (float(v) for v in obs.pose_estimate)
        return float(self._wrap(math.atan2(gy - y, gx - x) - yaw))

    def _admissible_gaps(self, angles, ranges):
        """Maximal runs of free beams in the forward half-plane that fit the body.

        Returns (phi_lo, phi_hi, d_eval, index) per surviving run, where index is
        the lowest beam index of the run so selection can break ties on it.
        """
        forward = np.abs(angles) <= math.pi / 2.0
        if not np.any(forward):
            return []
        idx = np.flatnonzero(forward)
        free = ranges[idx] >= self.D_FREE

        gaps = []
        start = None
        for k in range(len(idx) + 1):
            is_free = bool(free[k]) if k < len(idx) else False
            if is_free and start is None:
                start = k
            elif not is_free and start is not None:
                gaps.append((start, k - 1))
                start = None

        out = []
        for lo_k, hi_k in gaps:
            lo_i, hi_i = idx[lo_k], idx[hi_k]
            phi_lo, phi_hi = float(angles[lo_i]), float(angles[hi_i])
            dphi = phi_hi - phi_lo
            if dphi <= 0.0:
                continue
            # Conservative depth: the shallower jamb, and never credited past 3 m.
            d_eval = min(float(ranges[lo_i]), float(ranges[hi_i]), self.D_EVAL_MAX)
            width = 2.0 * d_eval * math.sin(dphi / 2.0)
            if width >= self.W_MIN:
                out.append((phi_lo, phi_hi, d_eval, int(lo_i)))
        return out

    def _aim(self, gap, theta_g):
        """Bearing inside the gap closest to the goal, standing off both jambs."""
        phi_lo, phi_hi, d_eval, _ = gap
        dphi = phi_hi - phi_lo
        # Angle a body-clearance subtends at the aperture, floored so that a
        # just-admissible gap still leaves a non-empty aim interval.
        delta = min(math.atan2(self.R_CLEAR, max(d_eval, 1e-3)), dphi / 2.0 - 0.02)
        delta = max(delta, 0.0)
        low, high = phi_lo + delta, phi_hi - delta
        if low > high:
            return (phi_lo + phi_hi) / 2.0
        return float(min(max(theta_g, low), high))

    # -- contract --------------------------------------------------------

    def precondition(self, obs, ports):
        theta_g = self._goal_bearing(obs)
        if theta_g is None:
            return False
        angles, ranges, r_max = self._scan(obs)
        blocked = self._min_range_near(
            angles, ranges, theta_g, self.CORRIDOR_HALF, r_max
        )
        return blocked < self.D_BLOCK

    def tick(self, obs, ports):
        # Re-checked here so the node is correct even if ticked without its
        # precondition, and because (c) needs the gap set anyway.
        theta_g = self._goal_bearing(obs)
        if theta_g is None:
            return Status.FAILURE

        angles, ranges, r_max = self._scan(obs)
        if self._min_range_near(
            angles, ranges, theta_g, self.CORRIDOR_HALF, r_max
        ) >= self.D_BLOCK:
            return Status.FAILURE

        gaps = self._admissible_gaps(angles, ranges)
        if not gaps:
            return Status.FAILURE

        best = min(
            gaps,
            key=lambda g: (abs(float(self._wrap((g[0] + g[1]) / 2.0 - theta_g))), g[3]),
        )
        phi_aim = self._aim(best, theta_g)

        wz = 0.0 if abs(phi_aim) < self.YAW_DEADBAND else float(
            np.clip(self.YAW_GAIN * phi_aim, -self.YAW_MAX, self.YAW_MAX)
        )

        ports["gap_aim_bearing"] = phi_aim
        self._last_aim = phi_aim

        if obs.contact:
            # Back off a grazing touch rather than leaning into it, while still
            # turning toward the aperture.
            ports.adjust(vx=self.VX_CONTACT)
            ports.adjust(wz=wz)
            return Status.RUNNING

        d_ahead = self._min_range_near(
            angles, ranges, phi_aim, self.CORRIDOR_HALF, r_max
        )
        # cos() holds speed down while the heading error is still large; the
        # range term takes it to zero as the aim direction itself closes.
        vx = float(np.clip(self.VX_MAX * math.cos(phi_aim), 0.0, self.VX_MAX))
        vx *= min(1.0, d_ahead / self.D_BLOCK)

        ports.adjust(vx=vx)
        ports.adjust(wz=wz)
        return Status.RUNNING


PREDICTION = {
    "statement": (
        "On unmapped indoor layouts where the straight line from start to a "
        "disclosed goal is occluded by walls or furniture, replacing the "
        "goal-bearing heading command with the gap-clipped goal bearing raises "
        "the fraction of seeds on which the goal is reached, relative to the "
        "goal_seeker baseline, because the robot passes through apertures "
        "instead of stalling against the occluder face."
    ),
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.15,
    "cost_metric": "path_length",
}
