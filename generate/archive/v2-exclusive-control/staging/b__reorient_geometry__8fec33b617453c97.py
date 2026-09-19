"""Re-anchor the goal to the arrangement of visible surfaces (Cheng's geometric module).

Implements `shape_anchored_goal_address`: the goal is stored not as a world
coordinate but as an address (u, v) on a metric frame built from the shape of
the currently visible surfaces, and that frame is re-measured from the live scan
at every control step.  Because the frame is re-measured rather than integrated,
an address expressed in it does not inherit the error that `pose_estimate`
accumulates.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import (Action, Direction, Port, PortType, Status)
from harness.micro import MicroScenario, room


class ReanchorGoalToVisibleShape(Action):
    """Steer to the goal's address in the frame of the visible surfaces.

    Mechanism (Cheng 1986, as mapped in the card):

    * Every finite lidar return is a surface point.  Their centroid and
      second-moment matrix define a metric frame: `e1` is the major
      eigenvector, `e2 = rot90(e1)`.  Taking `e2` as a fixed rotation of `e1`
      -- never as an independent eigenvector -- is what makes the frame carry
      *sense*: the frame can never be built by an operation blind to reflection.
    * `e1` is defined only up to sign, and the two signs are exactly Cheng's
      target and its rotational equivalent.  The tie is broken by the third
      moment along the axis, a statistic a 180-degree-symmetric shape cannot
      supply.  When the normalised skew is too small the shape is near
      symmetric, sense is undecidable, and this leaf abstains instead of
      committing -- the rat commits and is wrong 31 times in 100; a robot that
      abstains keeps a drifting belief, which is bad, instead of a confidently
      mirrored address, which is worse.
    * While `pose_error_estimate` is small the belief is trustworthy and the
      address is re-measured from it each step.  Once the belief goes bad the
      address is frozen and reconstructed against the live surfaces instead,
      provided the surfaces still have the shape signature that was latched.

    Why it should work: the correction is re-derived from what is in front of
    the robot at recall time, so it removes drift accumulated since the latch.
    It cannot remove drift accumulated before the latch and it does not survive
    a doorway -- a 6 m lidar sees one room's worth of surface, so this frame is
    local and temporary where Cheng's box was global and permanent.
    """

    # All five thresholds are the card's, and all five are the card author's own
    # choices rather than anything the paper states.
    MIN_SURFACE_BEAMS = 40      # fewer is a corridor mouth, not an arrangement
    MIN_NORMALISED_SKEW = 0.05  # below this the shape is ~180-deg symmetric
    POSE_TRUST_M = 0.30         # ~half a Go2 body: belief still worth latching from
    SIGNATURE_TOL = 0.25        # same surfaces from elsewhere, not a different room
    ARRIVAL_M = 0.25

    YAW_GAIN = 2.0
    MAX_WZ = 1.0
    CRUISE_VX = 0.8
    ADVANCE_BEARING = 1.0       # rad; beyond this, turn on the spot

    PORTS = (
        Port("shape_anchor_active", PortType.FLAG, Direction.OUT),
        Port("shape_goal_bearing", PortType.BEARING, Direction.OUT),
    )

    PRECONDITION = "The goal position has been disclosed."
    EFFECT = "Drives the robot toward the goal's address in the frame of the currently visible surfaces."
    TAGS = ("geometry", "shape-frame", "drift", "relocalisation", "goal-approach")

    def reset(self, scenario, rng):
        self._address = None      # (u, v): goal offset expressed in the shape frame
        self._signature = None    # (lam1, lam2): which arrangement that address belongs to

    def precondition(self, obs, ports):
        return obs.goal is not None

    def tick(self, obs, ports):
        # The goal latches once disclosed, but a tree may tick us before that.
        if obs.goal is None:
            return self._abstain(ports)

        frame = self._shape_frame(obs)
        if frame is None:
            return self._abstain(ports)
        c, e1, e2, lam1, lam2 = frame

        if float(obs.pose_error_estimate) <= self.POSE_TRUST_M:
            # Belief still trustworthy: re-measure the address against the shape
            # we can see right now, and keep it as the thing to fall back on.
            #
            # The card has the leaf return FAILURE here and lets the tree fall
            # through to steering by pose_estimate.  A leaf is also run alone in
            # its unit scenario, where there is no sibling to fall through to,
            # so it drives on this branch too -- and because the address was
            # measured from this very step's belief, the heading commanded here
            # is identical to the pose-derived heading that the fall-through
            # would have produced.  The mechanism under test is the *frozen*
            # address below; this branch only keeps it fed.
            self._address = self._measure_address(obs, c, e1, e2)
            self._signature = (lam1, lam2)
            anchored = False
        else:
            # Belief no longer trustworthy: use the latched address, but only if
            # these are still recognisably the surfaces it was latched against.
            if self._address is None or self._signature is None:
                return self._abstain(ports)
            if not self._same_arrangement(lam1, lam2):
                # Different room, or the same one seen through a doorway.  Abstain
                # and re-latch at the next step the belief is good enough.
                return self._abstain(ports)
            anchored = True

        u, v = self._address
        goal_robot = c + u * e1 + v * e2

        bearing = math.atan2(float(goal_robot[1]), float(goal_robot[0]))
        ports["shape_anchor_active"] = bool(anchored)
        ports["shape_goal_bearing"] = float(bearing)

        if float(np.hypot(goal_robot[0], goal_robot[1])) <= self.ARRIVAL_M:
            return Status.SUCCESS

        wz = float(np.clip(self.YAW_GAIN * bearing, -self.MAX_WZ, self.MAX_WZ))
        vx = self.CRUISE_VX * math.cos(bearing) if abs(bearing) <= self.ADVANCE_BEARING else 0.0
        ports.drive(Command(vx, 0.0, wz))
        return Status.RUNNING

    def _abstain(self, ports):
        ports["shape_anchor_active"] = False
        return Status.FAILURE

    def _shape_frame(self, obs):
        """Centroid, right-handed axes and eigenvalues of the visible surfaces.

        Returns None when the scan is not an arrangement of surfaces, or is one
        whose sense cannot be decided.
        """
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        surface = np.isfinite(ranges) & (ranges < float(obs.range_max))
        n = int(np.count_nonzero(surface))
        if n < self.MIN_SURFACE_BEAMS:
            return None

        r = ranges[surface]
        a = angles[surface]
        pts = np.stack((r * np.cos(a), r * np.sin(a)), axis=1)
        c = pts.mean(axis=0)
        d = pts - c

        m = (d.T @ d) / n
        vals, vecs = np.linalg.eigh(m)  # ascending
        lam2, lam1 = float(vals[0]), float(vals[1])
        if lam1 <= 1e-9:
            return None
        e1 = vecs[:, 1]

        # Sign of e1 is Cheng's rotational ambiguity: break it on a statistic
        # that a 180-degree-symmetric arrangement cannot supply.
        proj = d @ e1
        m3 = float(np.mean(proj ** 3))
        if abs(m3) < self.MIN_NORMALISED_SKEW * lam1 ** 1.5:
            return None
        if m3 < 0.0:
            e1 = -e1

        e2 = np.array([-e1[1], e1[0]])  # rot90 of e1: the pair is always right-handed
        return c, e1, e2, lam1, lam2

    def _measure_address(self, obs, c, e1, e2):
        """Goal offset from the pose belief, re-expressed in the shape frame."""
        x, y, yaw = (float(q) for q in obs.pose_estimate)
        gx, gy = (float(q) for q in obs.goal)
        dx, dy = gx - x, gy - y
        cs, sn = math.cos(yaw), math.sin(yaw)
        d = np.array([cs * dx + sn * dy, -sn * dx + cs * dy])  # R(-yaw) * (goal - pose)
        rel = d - c
        return float(rel @ e1), float(rel @ e2)

    def _same_arrangement(self, lam1, lam2):
        lam1_l, lam2_l = self._signature
        return (abs(lam1 / max(lam1_l, 1e-9) - 1.0) <= self.SIGNATURE_TOL and
                abs(lam2 / max(lam2_l, 1e-9) - 1.0) <= self.SIGNATURE_TOL)


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}


def micro(seed=0):
    """One room, one clear straight run to the goal, and a drifting belief.

    The route is deliberately free of obstructions, so the only thing that can
    make `goal_seeker` miss is the drift in `pose_estimate` -- which is exactly
    the one claimed effect this scenario has to isolate.

    The room's shape is chosen so the frame is decidable everywhere on the route:

    * 6.6 x 3.6 keeps every surface inside the 6 m lidar from anywhere on the
      route, so the visible arrangement is the whole room and the eigenvalue
      signature stays near constant as the robot moves -- Cheng's box condition.
    * The long axis (x) carries roughly twice the variance of the short one, so
      the major eigenvector cannot swap axes mid-run.
    * The two blocks chop the -x end of the room.  That truncates the left tail
      of the along-axis distribution hard enough that the third moment stays
      positive over the whole route: without it, the skew is dominated by the
      robot's own offset from the room centre and changes sign as the robot
      crosses it, which would flip the reconstructed address by 180 degrees.
    * The start yaw points away from the goal, so the robot turns on the spot
      for ~2 s first.  That buys drift-accumulation time at no path-length cost.
    """
    world = room(6.6, 3.6, boxes=[(0.0, 0.0, 1.8, 2.5),
                                  (0.0, 3.0, 1.4, 3.6)])
    return MicroScenario(
        id="reanchor_goal_to_visible_shape_drifting_belief",
        world=world,
        start=(2.6, 0.8, 3.0),
        goal=(5.3, 2.6),
        time_limit=30.0,
        seed=seed,
        goal_known=True,
    )
