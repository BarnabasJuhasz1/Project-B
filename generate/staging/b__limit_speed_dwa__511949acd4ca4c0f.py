import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status

# Candidate velocities of the card: vx in {0.0..0.8} x wz in {-1.0..1.0}, step 0.1.
# Built once at import; the pair (0.0, 0.0) is always present, so the admissible
# set is never empty and a command always exists.
_GVX, _GWZ = (a.ravel() for a in np.meshgrid(
    np.round(np.arange(0.0, 0.801, 0.1), 3),
    np.round(np.arange(-1.0, 1.001, 0.1), 3),
    indexing="ij"))


class ClampTwistToBrakingAdmissible(Action):
    """Braking-admissible velocity filter, Eq. (14) of the dynamic-window paper.

    A commanded pair (vx, wz) is permitted only if the robot could still brake to
    a stop, under its own braking accelerations, before reaching the nearest
    obstacle lying on the circular arc that this very pair traces:

        vx <= sqrt(2 * dist(vx, wz) * VB_DOT)   and
       |wz| <= sqrt(2 * dist(vx, wz) * WB_DOT)

    where dist is measured ALONG THAT ARC (Section 5.2: gamma * r), not through a
    fixed forward stopping box. That coupling is the whole content: two commands
    with the same forward speed sweep different parts of the room and so get
    different dist, and one command is admissible slowly and inadmissible quickly.

    Revises vx and wz, because those are the two terms of the inequality, and caps
    vy to zero, because the paper's arc model carries no lateral term (Section 3.1,
    "the translational velocity always leads in the steering direction") and any
    dist computed while the body is also sliding sideways is a prediction about a
    path the robot is not on.

    Declines -- writing nothing, so the base's command passes through unchanged --
    when no beam returns inside range_max (dist saturates, nothing binds), and
    when the base's own command already satisfies both inequalities. Reverse
    commands satisfy the first inequality trivially for any dist >= 0, so a
    reversing recovery is passed through rather than projected onto the
    non-negative candidate grid.

    Everything below is computed in the ROBOT frame from ranges/range_angles.
    pose_estimate is never read, so the mechanism is immune to pose drift, and no
    FIFO obstacle memory is kept: the lidar gives a full circle every step, and
    remembered points would inherit exactly the drift this avoids.
    """

    WRITES = ("vx", "vy", "wz")
    PORTS = (Port("braking_speed_limit", PortType.SPEED, Direction.OUT),)
    PRECONDITION = (
        "A twist has been proposed upstream and the current scan holds at least "
        "one finite return inside range_max."
    )
    EFFECT = (
        "Replaces the proposed forward and yaw rates with the nearest pair that "
        "could still brake to a stop before the closest obstacle on the arc that "
        "pair traces."
    )
    TAGS = ("safety", "velocity-filter", "braking-distance", "arc-clearance",
            "collision-avoidance", "reactive", "drift-immune")

    # Go2 body is about 0.70 m by 0.31 m; circumscribed disc radius.
    RHO = 0.38
    # Braking accelerations: not RHINO's. Chosen so the full command range is
    # nulled in about a second of tracking (0.8 m/s -> 0 in 1.0 s, 1.0 rad/s -> 0
    # in 0.67 s). VB_DOT is the single safety knob.
    VB_DOT = 0.8
    WB_DOT = 1.5
    VX_MAX = 0.8
    WZ_MAX = 1.0
    # Below this yaw rate the arc radius blows up and the sweep is a straight line.
    STRAIGHT_EPS = 1e-3
    TOL = 1e-9

    def reset(self, scenario, rng):
        """No per-episode state: every tick is decided from that tick's scan alone."""

    def precondition(self, obs, ports) -> bool:
        ranges = np.asarray(obs.ranges, dtype=float)
        return bool(np.any(np.isfinite(ranges) & (ranges <= float(obs.range_max))))

    def _clearance(self, px, py, v, w, r_max):
        """Arc distance to the nearest obstacle for each candidate pair.

        px, py are obstacle points in the robot frame; v, w are equal-length
        candidate arrays. Returns dist(v, w) per Section 5.2, clamped to
        [0, r_max], already reduced by the body inflation RHO.
        """
        dist = np.full(v.shape, r_max, dtype=float)
        if px.size == 0:
            return dist

        straight = np.abs(w) < self.STRAIGHT_EPS
        if np.any(straight):
            band = (np.abs(py) <= self.RHO) & (px > 0.0)
            ahead = float(np.min(px[band])) - self.RHO if np.any(band) else r_max
            dist[straight] = min(max(ahead, 0.0), r_max)

        # vx == 0 with real yaw rate: a spin sweeps no new ground for a disc, so
        # dist saturates and only the wz inequality could bind (it cannot).
        turning = ~straight
        dist[turning & (v <= 0.0)] = r_max

        arc = turning & (v > 0.0)
        if not np.any(arc):
            return dist

        r = v[arc] / w[arc]                      # signed arc radius, centre at (0, r)
        absr = np.abs(r)
        rc = r[:, None]
        dy = py[None, :] - rc
        on_arc = np.abs(np.hypot(px[None, :], dy) - absr[:, None]) <= self.RHO

        # Angle about the centre from the robot (origin) to each obstacle point.
        # From the centre, the robot is at (0, -r) and the point at (px, py - r);
        # cross = r*px and dot = r*r - r*py give the signed swept angle.
        delta = np.arctan2(rc * px[None, :], rc * rc - rc * py[None, :])
        # Sign-correct by the direction of travel, then wrap: a point behind the
        # sweep lands near 2*pi rather than near 0.
        gamma = np.mod(delta * np.sign(w[arc])[:, None], 2.0 * math.pi)

        along = np.where(on_arc, gamma * absr[:, None], np.inf)
        dist[arc] = np.clip(np.min(along, axis=1) - self.RHO, 0.0, r_max)
        return dist

    def tick(self, obs, ports) -> Status:
        pending = ports.command

        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        r_max = float(obs.range_max)
        hit = np.isfinite(ranges) & (ranges <= r_max)
        if not np.any(hit):
            # Nothing in sight: dist saturates everywhere and no inequality binds.
            return Status.FAILURE

        rr = ranges[hit]
        aa = angles[hit]
        px = rr * np.cos(aa)
        py = rr * np.sin(aa)

        vx_d = float(np.clip(pending.vx, -self.VX_MAX, self.VX_MAX))
        wz_d = float(np.clip(pending.wz, -self.WZ_MAX, self.WZ_MAX))

        # The arc model only describes motion with no lateral term, so the sideways
        # component goes before any clearance computed from it can be trusted.
        if abs(pending.vy) > 1e-6:
            ports.cap(vy=0.0)

        # Clearance on the arc the base actually asked for. A reverse command is
        # evaluated on the vx = 0 sweep: its own inequality is satisfied by any
        # negative speed, so this only ever decides the wz term for it.
        d_des = float(self._clearance(
            px, py,
            np.array([max(vx_d, 0.0)]), np.array([wz_d]),
            r_max)[0])

        v_ceiling = math.sqrt(2.0 * d_des * self.VB_DOT)
        ports["braking_speed_limit"] = float(v_ceiling)

        if (vx_d <= v_ceiling + self.TOL
                and abs(wz_d) <= math.sqrt(2.0 * d_des * self.WB_DOT) + self.TOL):
            # Already admissible: leave the base's steering and speed untouched.
            return Status.SUCCESS

        dist = self._clearance(px, py, _GVX, _GWZ, r_max)
        admissible = ((_GVX <= np.sqrt(2.0 * dist * self.VB_DOT) + self.TOL)
                      & (np.abs(_GWZ) <= np.sqrt(2.0 * dist * self.WB_DOT) + self.TOL))

        if not np.any(admissible):
            # Unreachable -- (0, 0) always satisfies both -- but the filter must
            # degrade to a stop, never to an undefined command.
            ports.adjust(vx=0.0, wz=0.0)
            return Status.SUCCESS

        # Nearest-point projection onto the admissible set, with a rad/s of turn
        # error costing half what a m/s of speed error costs, so the leaf perturbs
        # the upstream command only as far as Eq. (14) demands.
        cost = (_GVX - vx_d) ** 2 + (0.5 * (_GWZ - wz_d)) ** 2
        best = int(np.argmin(np.where(admissible, cost, np.inf)))
        ports.adjust(vx=float(_GVX[best]), wz=float(_GWZ[best]))
        return Status.SUCCESS


PREDICTION = {
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}
