import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status


class SelectVelocityByDynamicWindow(Action):
    """Dynamic-window admissible-velocity search over (vx, wz).

    Poses collision avoidance as a constrained search in velocity space: sample
    the commands reachable from the current velocity within one acceleration-
    limited interval, discard every candidate from which the robot could not
    brake to a standstill before the first obstacle on the arc it implies, and
    pick the survivor maximising a weighted sum of goal alignment, clearance
    and speed.

    Revises `vx` and `wz` because those are exactly the two degrees of freedom
    the arc model searches over; it writes them with `adjust` rather than `cap`
    because the mechanism *selects* a velocity (which may be faster or turn the
    other way than the base asked) rather than bounding one. `vy` is left to the
    base -- zeroing it would make the arc model exact but would mean declaring
    the whole Twist.

    Declines (leaving the base's command untouched, byte-identical) when the
    surroundings are not constrained: no finite lidar return within 2.5 m at a
    bearing of |theta| <= pi/2, or a fully infinite scan. Returns FAILURE after
    emitting a stop when the admissible set is empty, so a recovery leaf can
    take over instead of this node guessing at a least-bad unsafe command.
    """

    WRITES = ("vx", "wz")
    PORTS = (Port("dwa_safe_speed", PortType.SPEED, Direction.OUT),)
    PRECONDITION = (
        "The robot is in locally constrained surroundings: at least one lidar "
        "beam with |bearing| <= pi/2 returns a finite range below 2.5 m."
    )
    EFFECT = (
        "Replaces the forward speed and turn rate with the highest-scoring "
        "velocity from which the robot can still brake before the nearest "
        "obstacle on the arc that velocity implies."
    )
    TAGS = ("dynamic_window", "velocity_space", "admissibility", "clearance",
            "local_avoidance", "lidar")

    # Actuation saturations (the runner clips to these anyway).
    V_MAX = 0.8
    W_MAX = 1.0

    # Estimated acceleration limits of the frozen locomotion policy: roughly
    # how fast a ~15 kg quadruped's tracker can follow a step in commanded body
    # velocity. These are the card's numbers, not measured ones.
    A_V = 2.0
    A_W = 4.0
    A_BRAKE = 2.0

    T_WINDOW = 0.25          # one window interval ~ policy settling time
    T_SIM = 2.0              # arc rollout horizon; 2.0 s * 0.8 m/s = 1.6 m
    R_SAFE = 0.35            # Go2 circumscribed body radius, no extra margin

    N_V = 11
    N_W = 21

    CLUTTER_RANGE = 2.5      # trigger threshold on forward beams
    CLEAR_SAT = 2.0          # clearance saturates here so room beyond 2 m
                             # cannot outbid goal progress

    ALPHA = 0.6              # heading
    BETA = 0.3               # clearance
    GAMMA = 0.1              # speed

    # Class-level defaults so the node is usable even if reset() is never called.
    _v_prev = 0.0
    _w_prev = 0.0
    _was_active = False

    def reset(self, scenario, rng):
        self._v_prev = 0.0
        self._w_prev = 0.0
        self._was_active = False

    def precondition(self, obs, ports) -> bool:
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        finite = np.isfinite(ranges)
        if not finite.any():
            self._was_active = False
            return False

        forward = np.abs(angles) <= 0.5 * math.pi
        active = bool((finite & forward & (ranges < self.CLUTTER_RANGE)).any())
        if not active:
            # Remember that the base, not us, is driving: the next active tick
            # must build its reachable window around the base's velocity.
            self._was_active = False
        return active

    def tick(self, obs, ports) -> Status:
        pending = ports.command
        px, py = self._obstacle_points(obs)
        if px.size == 0:
            return Status.FAILURE

        if self._was_active:
            v_prev, w_prev = self._v_prev, self._w_prev
        else:
            # First active tick after a gap: the body is executing the base's
            # command, so that is the velocity we are actually reachable from.
            v_prev = float(np.clip(pending.vx, 0.0, self.V_MAX))
            w_prev = float(np.clip(pending.wz, -self.W_MAX, self.W_MAX))

        dv = self.A_V * self.T_WINDOW
        dw = self.A_W * self.T_WINDOW
        vs = np.linspace(max(0.0, v_prev - dv),
                         min(self.V_MAX, v_prev + dv), self.N_V)
        ws = np.linspace(max(-self.W_MAX, w_prev - dw),
                         min(self.W_MAX, w_prev + dw), self.N_W)
        V, W = np.meshgrid(vs, ws, indexing="ij")
        V = V.ravel()
        W = W.ravel()

        dist, dist_ang = self._arc_clearance(V, W, px, py, float(obs.range_max))

        # Hard feasibility: an inadmissible command is never scored.
        keep = ((V <= np.sqrt(2.0 * self.A_BRAKE * dist) + 1e-9)
                & (np.abs(W) <= np.sqrt(2.0 * self.A_W * dist_ang) + 1e-9))

        if not keep.any():
            ports.adjust(vx=0.0, wz=0.0)
            ports["dwa_safe_speed"] = 0.0
            self._v_prev = 0.0
            self._w_prev = 0.0
            self._was_active = True
            return Status.FAILURE

        Vk = V[keep]
        Wk = W[keep]
        dk = dist[keep]

        heading = 1.0 - np.abs(self._wrap(self._target_bearing(obs)
                                          - Wk * self.T_WINDOW)) / math.pi
        clear = np.minimum(dk, self.CLEAR_SAT) / self.CLEAR_SAT
        speed = np.abs(Vk) / self.V_MAX

        score = (self.ALPHA * self._normalise(heading)
                 + self.BETA * self._normalise(clear)
                 + self.GAMMA * self._normalise(speed))
        best = int(np.argmax(score))

        v_cmd = float(Vk[best])
        w_cmd = float(Wk[best])
        ports.adjust(vx=v_cmd, wz=w_cmd)
        ports["dwa_safe_speed"] = float(
            min(self.V_MAX, math.sqrt(2.0 * self.A_BRAKE * float(dk[best]))))

        self._v_prev = v_cmd
        self._w_prev = w_cmd
        self._was_active = True
        return Status.SUCCESS

    def _obstacle_points(self, obs):
        """Finite lidar returns as Cartesian points in the robot frame."""
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        n = min(ranges.size, angles.size)
        ranges = ranges[:n]
        angles = angles[:n]
        good = np.isfinite(ranges) & (ranges > 1e-3)
        r = ranges[good]
        a = angles[good]
        return r * np.cos(a), r * np.sin(a)

    def _target_bearing(self, obs) -> float:
        """Bearing to the goal in the robot frame, or 0 if it is undisclosed.

        With no goal this degrades into "hold the committed heading on the
        fastest safe arc", which is the graceful fallback the card asks for.
        """
        goal = obs.goal
        if goal is None:
            return 0.0
        x, y, yaw = obs.pose_estimate
        return float(self._wrap(math.atan2(goal[1] - y, goal[0] - x) - yaw))

    def _arc_clearance(self, V, W, px, py, range_max):
        """Free arc length and free arc angle before the swept disc hits a point.

        Returned distances are capped at the rollout horizon and the sensor
        range, so an unobstructed arc scores the same as a very long one.
        """
        dist = np.minimum(range_max, self.T_SIM * V)
        dist_ang = np.minimum(math.pi, self.T_SIM * np.abs(W))

        straight = np.abs(W) < 1e-3
        curving = (~straight) & (V > 1e-6)

        if straight.any():
            # Swept disc travelling along +x: a point is struck when its
            # distance to the disc centre falls to R_SAFE.
            lateral = np.abs(py) <= self.R_SAFE
            if lateral.any():
                half = np.sqrt(np.maximum(self.R_SAFE ** 2 - py[lateral] ** 2, 0.0))
                s_enter = px[lateral] - half
                s_exit = px[lateral] + half
                ahead = s_exit >= 0.0
                if ahead.any():
                    d_line = float(np.maximum(s_enter[ahead], 0.0).min())
                    dist[straight] = np.minimum(dist[straight], d_line)

        if curving.any():
            Vc = V[curving]
            Wc = W[curving]
            R = Vc / Wc                      # signed turn radius
            aR = np.abs(R)
            sgn = np.sign(Wc)

            # Distance of each point from the arc centre (0, R); the swept disc
            # can only ever reach points within R_SAFE of the arc's own radius.
            rho = np.hypot(px[None, :], py[None, :] - R[:, None])
            hit = np.abs(rho - aR[:, None]) <= self.R_SAFE

            # Angle about the centre from the robot's start angle, measured in
            # the direction of travel; minus the angular slack corresponding to
            # the body radius, which makes contact register slightly early.
            beta = np.arctan2(py[None, :] - R[:, None], px[None, :])
            alpha0 = -0.5 * math.pi * sgn
            swept = np.mod((beta - alpha0[:, None]) * sgn[:, None], 2.0 * math.pi)
            slack = np.minimum(self.R_SAFE / aR, math.pi)
            swept = np.maximum(swept - slack[:, None], 0.0)

            first = np.where(hit, swept, np.inf).min(axis=1)
            dist[curving] = np.minimum(dist[curving], aR * first)
            dist_ang[curving] = np.minimum(dist_ang[curving], first)

        # Candidates with v == 0 keep dist == 0 (no arc) but an unconstrained
        # arc angle: spinning a disc body in place sweeps no new floor.
        return dist, dist_ang

    @staticmethod
    def _wrap(a):
        return np.arctan2(np.sin(a), np.cos(a))

    @staticmethod
    def _normalise(a):
        lo = float(a.min())
        hi = float(a.max())
        if hi - lo < 1e-9:
            return np.zeros_like(a)
        return (a - lo) / (hi - lo)


PREDICTION = {
    "statement": (
        "On unmapped indoor navigation with occluding walls and furniture, "
        "replacing the baseline's command selection with the dynamic-window "
        "admissible-velocity search reduces the mean number of collisions per "
        "episode by at least 50% relative to goal_seeker, over a fixed set of "
        "seeds."
    ),
    "metric": "collisions",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}
