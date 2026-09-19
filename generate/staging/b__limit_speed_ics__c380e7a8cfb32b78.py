"""Braking-horizon speed cap for an unmapped explorer."""

import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status


class CapSpeedToBrakingHorizon(Action):
    """Cap commanded translation at the speed the robot could still brake from
    inside the free space it has actually observed along its direction of travel.

    Mechanism (inevitable-collision-state test, scalar form): a state is safe
    only while at least one witness manoeuvre terminates collision-free. The
    witness chosen here is "hold heading, command zero translation until
    stopped", whose stopping distance is

        d_stop(v) = m + v*tau + v^2 / (2*a_brake).

    The safety criterion treats the sensing horizon itself as a candidate
    obstacle, because unexpected obstacles always appear first at the boundary
    of the field of view: every beam reading ``inf`` is therefore counted as a
    return at ``range_max``, not as free space. Inverting d_stop <= d_free gives
    a speed cap, and the commanded (vx, vy) pair is rescaled onto it.

    Revises ``vx`` and ``vy`` -- both, because the mechanism shortens the
    commanded translation vector without rotating it; capping ``vx`` alone would
    silently steer. ``wz`` is never written, so the cap can never take away the
    robot's ability to rotate in place and re-perceive, which is exactly what it
    wants the robot to do once translation is blocked.

    Declines (writes nothing, byte-identical pass-through) when the base
    commanded no translation, or when the commanded speed is already at or below
    the braking horizon -- i.e. in open space, where the horizon sits at 6 m and
    the cap exceeds the runner's own saturation limit.

    Everything is computed in the robot frame from the scan alone; no pose, no
    map, so the node is immune to pose-estimate drift.
    """

    WRITES = ("vx", "vy")

    PORTS = (
        Port("ranges", PortType.RANGES, Direction.IN),
    )

    PRECONDITION = (
        "The command pending from the layer below asks for a non-zero "
        "translational speed."
    )
    EFFECT = (
        "Rescales the commanded translation down to the largest speed from "
        "which the robot could still be braked to rest inside the free space "
        "currently observed along the direction of travel."
    )
    TAGS = ("safety", "ics", "speed-cap", "occlusion", "exploration", "reactive")

    # Numbers are the card's. The frozen policy publishes no deceleration, so
    # a_brake is the observed rate at which a Go2 tracking a zeroed velocity
    # command sheds its full 0.8 m/s -- roughly one second of gait.
    A_BRAKE = 0.8          # m/s^2, effective deceleration of the witness stop
    TAU = 0.15             # s, dead time: one control step plus policy/gait lag
    MARGIN = 0.35          # m, half the ~0.70 m body length, so the whole
                           # footprint clears the obstacle
    V_SAT = 0.8            # m/s, the runner's own translational saturation
    EPS = 1e-3             # m/s, below this the command counts as no translation

    def precondition(self, obs, ports) -> bool:
        pending = ports.command
        return math.hypot(pending.vx, pending.vy) > self.EPS

    def tick(self, obs, ports) -> Status:
        pending = ports.command
        v_cmd = math.hypot(pending.vx, pending.vy)
        if v_cmd <= self.EPS:
            return Status.FAILURE

        theta_cmd = math.atan2(pending.vy, pending.vx)

        ranges = np.asarray(ports["ranges"], dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        n = min(ranges.shape[0], angles.shape[0])
        if n == 0:
            return Status.FAILURE
        ranges = ranges[:n]
        angles = angles[:n]

        range_max = float(getattr(obs, "range_max", 6.0))

        # The whole point of the criterion: a beam with no return is not free
        # space, it is a hypothesised obstacle sitting at the sensing horizon.
        r = np.where(np.isfinite(ranges), ranges, range_max)
        r = np.clip(r, 0.0, range_max)

        d_free = self._free_distance(r, angles, theta_cmd, n, range_max)

        # With yaw rate commanded, the heading sweeps while the witness stop
        # runs, so the corridor that must be clear is the one we will be facing
        # by the time we are stopped. Take the worse of the two.
        wz = float(pending.wz)
        if abs(wz) > 0.0:
            theta_end = theta_cmd + wz * (self.TAU + v_cmd / self.A_BRAKE)
            d_free = min(
                d_free,
                self._free_distance(r, angles, theta_end, n, range_max),
            )

        v_max = self._braking_speed(d_free)
        v_out = float(np.clip(v_max, 0.0, self.V_SAT))

        if v_out >= v_cmd:
            # Already inside the horizon: stay out of the way entirely.
            return Status.FAILURE

        scale = v_out / v_cmd
        # cap() keeps the sign, so passing magnitudes shortens the translation
        # vector without rotating it.
        ports.cap(vx=abs(pending.vx) * scale, vy=abs(pending.vy) * scale)
        return Status.SUCCESS

    def _free_distance(self, r, angles, theta, n_beams, range_max):
        """Distance to the nearest return inside the swept corridor along theta."""
        delta = np.arctan2(np.sin(angles - theta), np.cos(angles - theta))

        # Beams behind the direction of travel cannot be hit by it, so backing
        # away from a wall is never capped by that wall.
        ahead = np.abs(delta) <= (math.pi / 2.0)

        # Corridor half-width is the body margin inflated by half the angular
        # gap between adjacent beams at that range, so an obstacle falling
        # between two beams is still caught.
        half_gap = 0.5 * (2.0 * math.pi / max(1, n_beams))
        lateral = r * np.abs(np.sin(delta))
        keep = ahead & (lateral <= self.MARGIN + r * half_gap)

        if not np.any(keep):
            return range_max
        return float(np.min(r[keep] * np.cos(delta[keep])))

    def _braking_speed(self, d_free):
        """Invert d_stop(v) <= d_free for v."""
        room = max(0.0, d_free - self.MARGIN)
        at = self.A_BRAKE * self.TAU
        return -at + math.sqrt(at * at + 2.0 * self.A_BRAKE * room)


PREDICTION = {
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}
