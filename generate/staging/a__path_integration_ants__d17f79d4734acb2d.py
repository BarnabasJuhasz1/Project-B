"""Egocentric goal vector carried by path integration (Wehner-style ant homing).

The goal is held as a vector ``p`` expressed in the CURRENT robot frame and
updated every step from course-and-distance *increments* only.  The absolute
pose belief is read as a source of deltas, never as a position: a seeded random
walk on ``pose_estimate`` therefore enters as a bounded per-step perturbation
instead of as a standing offset on the steering target.  Because the vector is
carried rather than recomputed, it stays valid across convoluted detours and
over ground the robot has never crossed.

Integration is approximate, and the error is charged mostly to *turning*
(3% of every radian) and only lightly to translation (2% of every metre).  When
the accumulated bearing error passes the trust gate the vector is demoted to
bearing-only: it may still steer, but it may not declare arrival and its range
is not published to anyone.  Whenever ``pose_error_estimate`` returns to its
best-ever value the vector is re-seeded from absolute pose and the error budget
is zeroed, so on non-drifting worlds this degenerates to ordinary goal seeking.

Revises ``wz`` (adjust: aims the body at the carried bearing) and ``vx``
(cap: no faster than the cosine of the aiming error, braked inside the last
metre).  ``vy`` is left to whoever wrote it.

Declines entirely until the goal has been disclosed, and writes nothing once the
carried vector puts the goal inside the arrival radius with the trust gate shut.
"""

import math

from nodes.contract import Action, Direction, Port, PortType, Status


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _rotate(vx, vy, a):
    """Apply R(a) to (vx, vy)."""
    ca, sa = math.cos(a), math.sin(a)
    return ca * vx - sa * vy, sa * vx + ca * vy


class SteerByPathIntegratedGoalVector(Action):
    """Steer on a goal vector accumulated from odometric increments."""

    WRITES = ("vx", "wz")

    PORTS = (
        Port("goal_bearing", PortType.BEARING, Direction.OUT),
        Port("goal_vector_confidence", PortType.CONFIDENCE, Direction.OUT),
    )

    PRECONDITION = (
        "The goal has been disclosed at least once this episode (obs.goal latches), "
        "and the carried vector still places it beyond the 0.5 m arrival radius."
    )
    EFFECT = (
        "Points the robot along the path-integrated goal vector: yaw rate is set to "
        "its bearing and forward speed is capped by how well the body is aimed at it."
    )
    TAGS = ("path-integration", "dead-reckoning", "goal-vector", "drift", "steering")

    # Error budget, per increment, so the rule is independent of control rate.
    TURN_COST = 0.03        # rad of bearing error per rad turned
    TRANSLATE_COST = 0.01   # rad of bearing error per unit of relative displacement
    RANGE_COST = 0.02       # m of range error per m travelled

    REANCHOR_SLACK = 0.05   # m above the best-ever belief quality that still re-seeds
    TRUST_GATE = 0.35       # rad; 6.0 m * sin(0.35) = 2.06 m lateral, wider than a door
    ARRIVAL_R = 0.5         # m
    BRAKE_R = 1.0           # m; proximity braking starts here
    STEER_GAIN = 1.5
    WZ_MAX = 1.0
    VX_CRUISE = 0.8

    # Per-episode state, as class-level defaults so no __init__ is needed.
    _goal = None
    _p = None
    _prev_pose = None
    _sigma_beta = 0.0
    _sigma_r = 0.0
    _e_min = float("inf")

    def reset(self, scenario, rng):
        self._goal = None
        self._p = None
        self._prev_pose = None
        self._sigma_beta = 0.0
        self._sigma_r = 0.0
        self._e_min = float("inf")

    def precondition(self, obs, ports):
        if obs.goal is not None and self._goal is None:
            self._goal = (float(obs.goal[0]), float(obs.goal[1]))
        return self._goal is not None

    def tick(self, obs, ports):
        if self._goal is None:
            return Status.FAILURE

        x, y, yaw = (float(v) for v in obs.pose_estimate)
        err = float(obs.pose_error_estimate)

        if self._p is None:
            self._seed(x, y, yaw)
            self._e_min = err
        else:
            self._integrate(x, y, yaw)
            if err <= self._e_min + self.REANCHOR_SLACK:
                # The belief is at or near its best-ever quality: trust it once,
                # re-express the vector absolutely and clear the error budget.
                self._seed(x, y, yaw)
                self._e_min = min(self._e_min, err)

        self._prev_pose = (x, y, yaw)

        px, py = self._p
        r = math.hypot(px, py)
        beta = math.atan2(py, px)
        gate_open = self._sigma_beta > self.TRUST_GATE

        ports["goal_bearing"] = beta
        ports["goal_vector_confidence"] = max(
            0.0, min(1.0, 1.0 - self._sigma_beta / self.TRUST_GATE)
        )

        if not gate_open and r < self.ARRIVAL_R:
            # Arrived on evidence we still trust: hand the last metres back to the
            # base untouched rather than fighting it over a sub-metre target.
            return Status.SUCCESS

        wz = self.STEER_GAIN * beta
        ports.adjust(wz=max(-self.WZ_MAX, min(self.WZ_MAX, wz)))

        # cos(beta) refuses forward speed while badly misaimed; the range term
        # brakes inside the last metre, but only while the range is still usable
        # -- past the trust gate the vector is bearing-only by construction.
        limit = self.VX_CRUISE * max(0.0, math.cos(beta))
        if not gate_open:
            limit *= min(1.0, r / self.BRAKE_R)
        ports.cap(vx=limit)

        return Status.RUNNING

    def _seed(self, x, y, yaw):
        """Express the goal in the current robot frame from absolute pose."""
        gx, gy = self._goal
        self._p = _rotate(gx - x, gy - y, -yaw)
        self._sigma_beta = 0.0
        self._sigma_r = 0.0

    def _integrate(self, x, y, yaw):
        """Carry the vector forward on this step's increments alone."""
        x0, y0, yaw0 = self._prev_pose
        dx, dy = x - x0, y - y0
        ds = math.hypot(dx, dy)
        dpsi = _wrap_pi(yaw - yaw0)

        # The translation happened in the frame the robot occupied when it made it.
        dfx, dfy = _rotate(dx, dy, -yaw0)
        px, py = self._p
        self._p = _rotate(px - dfx, py - dfy, -dpsi)

        norm = max(math.hypot(*self._p), 0.5)
        self._sigma_beta += self.TURN_COST * abs(dpsi) + self.TRANSLATE_COST * ds / norm
        self._sigma_r += self.RANGE_COST * ds


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}
