"""Shape-anchored goal address (Cheng): re-measure the goal against visible surfaces."""

import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status


class ReanchorGoalToVisibleShape(Action):
    """Steer the final approach by the goal's address in a locally measured shape frame.

    Mechanism (Cheng's geometric module): while the pose belief is still
    trustworthy, the goal is written down not as map coordinates but as an
    address on a metric frame built from the arrangement of visible surfaces --
    the centroid and principal axes of the finite lidar returns, with sense
    fixed by the third moment along the major axis so the frame is not blind to
    reflection.  Once the belief has drifted, that address is re-measured
    against the surfaces visible *now*, so it does not inherit the error the
    integrated-motion belief has accumulated since the latch.

    Revises `wz` (adjust: the heading to the re-anchored goal replaces the
    heading the base derived from the drifted belief -- that substitution is the
    point of the node) and bounds `vx` (cap: no faster than 0.8*cos(psi), i.e.
    crawl or stop while swinging onto the corrected bearing).  `vy` is left
    entirely to the base.

    Declines -- writing nothing, so the base's command passes through
    byte-identical -- when the goal is undisclosed, when the belief is still
    good (<= 0.30 m error; those steps are used to latch the address instead),
    when fewer than 40 beams return a surface, when the visible shape is too
    near 180-degree symmetric for its sense to be decided, when no address was
    ever latched, or when the surfaces in view no longer match the latched
    signature (we have left the room the address was written in).
    """

    WRITES = ("vx", "wz")

    PORTS = (
        Port("goal_known", PortType.FLAG, Direction.IN),
        Port("ranges", PortType.RANGES, Direction.IN),
        # The corrected goal bearing is useful to anything else that wants to
        # know where the goal is without going through the drifting belief.
        Port("shape_anchored_goal_bearing", PortType.BEARING, Direction.OUT),
    )

    PRECONDITION = (
        "The goal position has been disclosed, so a goal address can either be "
        "latched onto the visible surface arrangement or read back off it."
    )
    EFFECT = (
        "While the pose belief is untrustworthy, replaces the commanded heading "
        "with the bearing to the goal reconstructed from the latched shape-frame "
        "address, bounding forward speed by the cosine of the heading error."
    )
    TAGS = ("geometry", "shape-frame", "drift", "relocalisation", "goal-approach")

    # --- thresholds (all from the card's derivation) ----------------------
    MIN_SURFACE_BEAMS = 40      # fewer than this is a corridor mouth, not an arrangement
    TRUST_RADIUS_M = 0.30       # a belief wrong by less than half a body is not the problem
    MIN_NORMALISED_SKEW = 0.05  # below this the shape is too symmetric to give sense
    SIGNATURE_TOL = 0.25        # same surfaces seen from elsewhere, not a different room
    HEADING_GAIN = 2.0
    MAX_WZ = 1.0
    CRUISE_VX = 0.8
    MAX_HEADING_FOR_FORWARD = 1.0  # rad; beyond this, turn in place

    def reset(self, scenario, rng):
        self._address = None    # (u, v) of the goal in the latched shape frame
        self._signature = None  # (lam1, lam2, n_beams) of the latched arrangement

    def precondition(self, obs, ports):
        # Only the disclosure clause lives here: the remaining trigger clauses
        # are checked inside tick, because the *latch* has to run on exactly the
        # steps where the node does not act.
        return bool(ports["goal_known"]) and obs.goal is not None

    # ------------------------------------------------------------------ #
    # shape frame
    # ------------------------------------------------------------------ #
    def _shape_frame(self, obs):
        """Centroid and right-handed, sense-resolved axes of the visible surfaces.

        Returns (c, e1, e2, lam1, lam2, n) in the robot frame, or None if the
        scan does not define a frame whose sense is decidable.
        """
        r = np.asarray(obs.ranges, dtype=float)
        ang = np.asarray(obs.range_angles, dtype=float)
        hit = np.isfinite(r) & (r > 0.0) & (r < float(obs.range_max))
        n = int(np.count_nonzero(hit))
        if n < self.MIN_SURFACE_BEAMS:
            return None

        rr = r[hit]
        aa = ang[hit]
        pts = np.stack((rr * np.cos(aa), rr * np.sin(aa)), axis=1)

        c = pts.mean(axis=0)
        q = pts - c
        m = (q.T @ q) / n
        vals, vecs = np.linalg.eigh(m)  # ascending eigenvalues, orthonormal vectors
        lam2, lam1 = float(vals[0]), float(vals[1])
        if lam1 <= 1e-9:
            return None
        e1 = np.asarray(vecs[:, 1], dtype=float)

        # e1 is defined only up to sign, and the two signs are exactly the
        # target and its rotational equivalent. Break the tie with a statistic
        # that a 180-degree-symmetric shape cannot supply.
        proj = q @ e1
        m3 = float(np.mean(proj ** 3))
        if m3 < 0.0:
            e1 = -e1
            m3 = -m3
        if m3 / (lam1 ** 1.5) < self.MIN_NORMALISED_SKEW:
            return None  # abstain rather than commit to a possibly mirrored address

        e2 = np.array((-e1[1], e1[0]), dtype=float)  # rot90: the pair carries sense
        return c, e1, e2, lam1, lam2, n

    def _goal_offset_from_belief(self, obs):
        """Goal in the robot frame according to the (drifting) pose belief."""
        gx, gy = float(obs.goal[0]), float(obs.goal[1])
        px, py, yaw = (float(v) for v in obs.pose_estimate)
        dx, dy = gx - px, gy - py
        cs, sn = math.cos(-yaw), math.sin(-yaw)
        return np.array((cs * dx - sn * dy, sn * dx + cs * dy), dtype=float)

    def _signature_matches(self, lam1, lam2):
        lam1_l, lam2_l, _ = self._signature
        if lam1_l <= 1e-9 or lam2_l <= 1e-9:
            return False
        return (abs(lam1 / lam1_l - 1.0) <= self.SIGNATURE_TOL
                and abs(lam2 / lam2_l - 1.0) <= self.SIGNATURE_TOL)

    # ------------------------------------------------------------------ #
    def tick(self, obs, ports):
        frame = self._shape_frame(obs)
        if frame is None:
            return Status.FAILURE
        c, e1, e2, lam1, lam2, n = frame

        if float(obs.pose_error_estimate) <= self.TRUST_RADIUS_M:
            # Belief still good: write the address down (overwriting any older
            # one) and let the base drive. This branch never touches the Twist.
            rel = self._goal_offset_from_belief(obs) - c
            self._address = (float(rel @ e1), float(rel @ e2))
            self._signature = (lam1, lam2, n)
            return Status.FAILURE

        if self._address is None or not self._signature_matches(lam1, lam2):
            # Never latched, or these are not the surfaces the address was
            # written against. Fall through to steering by pose_estimate; a
            # later trustworthy step will re-latch.
            return Status.FAILURE

        u, v = self._address
        g = c + u * e1 + v * e2
        psi = math.atan2(float(g[1]), float(g[0]))

        ports["shape_anchored_goal_bearing"] = psi

        wz = max(-self.MAX_WZ, min(self.MAX_WZ, self.HEADING_GAIN * psi))
        ports.adjust(wz=wz)

        # A bound, not a set: the base may already be going slower (obstacle,
        # doorway) and this node has no business speeding it up.
        if abs(psi) <= self.MAX_HEADING_FOR_FORWARD:
            ports.cap(vx=self.CRUISE_VX * math.cos(psi))
        else:
            ports.cap(vx=0.0)

        return Status.SUCCESS


PREDICTION = {
    "statement": (
        "On runs where pose_estimate drifts, steering the final approach by the "
        "goal's address in the shape frame rather than by the pose belief raises "
        "the fraction of episodes that reach the goal, because the address is "
        "re-measured against visible surfaces every step instead of inheriting "
        "accumulated belief error."
    ),
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}
