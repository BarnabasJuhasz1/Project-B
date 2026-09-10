"""Dynamic-window admissible-velocity search for a Go2 with a 181-beam 2D scan.

Implements `dynamic_window_velocity_search`: collision avoidance posed as a
constrained search in velocity space rather than position space. Each control
step samples the (vx, wz) pairs reachable from the previous command within one
acceleration-limited interval, keeps only those from which the body could still
brake to a standstill before the nearest obstacle on the implied arc, and picks
the survivor maximising a weighted sum of goal alignment, clearance and speed.

Safety is a hard feasibility constraint derived from braking physics, so an
inadmissible command is never scored at all; progress and speed stay soft
preferences. Unlike a repulsion field it never commands a velocity it cannot
stop out of, and unlike pure pursuit it prefers the roomier of two arcs that
make comparable progress. Both constraints and two of the three objective terms
are computed in the robot frame from lidar alone, so pose drift degrades goal
alignment only -- never the safety guarantee.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

# Actuation saturations enforced by the runner.
V_MAX = 0.8
W_MAX = 1.0

# Acceleration limits. The frozen locomotion policy hides the true ones; these
# are estimates of the rate at which a ~15 kg quadruped can change commanded
# body velocity without the tracker falling behind.
A_V = 2.0
A_W = 4.0
A_BRAKE = 2.0

T_W = 0.25          # window horizon ~ settling time of the policy on a step command
T_SIM = 2.0         # arc rollout horizon; 2.0 s * 0.8 m/s = 1.6 m >> braking distance
R_SAFE = 0.35       # Go2 circumscribed body radius, no extra margin
N_V = 11
N_W = 21

ALPHA = 0.6         # heading: dominant, so the node keeps making progress
BETA = 0.3          # clearance: enough to flip a choice between similar bearings
GAMMA = 0.1         # speed: tie-break only
CLEAR_SAT = 2.0     # room beyond 2 m should not outbid goal progress

TRIGGER_RANGE = 2.5
V_EPS = 1e-6

# Below this turn rate the arc is treated as a straight line. At w = 0.05 and
# v = 0.8 the arc departs from its chord by 0.08 m over T_SIM, well inside
# R_SAFE, while R = v/w = 16 m is about where the law-of-cosines contact test
# starts losing precision to cancellation.
W_STRAIGHT = 0.05


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _minmax(x):
    """Normalise a term to [0, 1] across the surviving set."""
    lo = float(np.min(x))
    hi = float(np.max(x))
    if hi - lo < 1e-9:
        # Degenerate: the term separates nothing, so it must not bias the argmax.
        return np.ones_like(x)
    return (x - lo) / (hi - lo)


class SelectVelocityByDynamicWindow(Action):
    """Emit the best admissible velocity in the dynamic window.

    Reachability cuts the search to commands the body can actually produce next
    step; admissibility (v <= sqrt(2 * dist * a_brake), plus its rotational
    analogue) cuts it to commands it can stop out of. What is left is scored by
    0.6 * heading + 0.3 * clearance + 0.1 * speed and the argmax is driven. If
    nothing is admissible the node stops and reports FAILURE rather than
    emitting the least-bad unstoppable command, leaving recovery to another leaf.
    """

    PORTS = (
        # Mirrors the FAILURE status so a sibling recovery leaf can be gated on
        # the empty-admissible-set case without inspecting return values.
        Port("dwa_infeasible", PortType.FLAG, Direction.OUT),
    )

    PRECONDITION = (
        "The scan has at least one finite return and at least one forward beam "
        "(|bearing| <= pi/2) closer than 2.5 m, so the surroundings constrain "
        "which velocities are safe."
    )
    EFFECT = (
        "Drives the reachable velocity that maximises the weighted heading, "
        "clearance and speed score among those from which the robot could still "
        "brake to a standstill before the nearest obstacle on its arc."
    )
    TAGS = (
        "dynamic_window",
        "velocity_space",
        "collision_avoidance",
        "local_planner",
        "admissible",
        "lidar",
        "braking",
    )

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._vx_prev = 0.0
        self._wz_prev = 0.0

    def reset(self, scenario, rng):
        self._vx_prev = 0.0
        self._wz_prev = 0.0

    def precondition(self, obs, ports):
        ranges = np.asarray(obs.ranges, dtype=float)
        finite = np.isfinite(ranges)
        if not np.any(finite):
            return False
        forward = np.abs(np.asarray(obs.range_angles, dtype=float)) <= 0.5 * math.pi
        # An unconstrained window needs no search; yield to straight-line pursuit.
        return bool(np.any(finite & forward & (ranges < TRIGGER_RANGE)))

    def tick(self, obs, ports):
        px, py, blocked_now = self._obstacles(obs)
        v, w = self._window()
        dist, dist_ang = self._clearance(
            v, w, px, py, float(obs.range_max), blocked_now
        )

        admissible = (v <= np.sqrt(2.0 * A_BRAKE * dist) + 1e-9) & (
            np.abs(w) <= np.sqrt(2.0 * A_W * dist_ang) + 1e-9
        )
        if not np.any(admissible):
            self._vx_prev = 0.0
            self._wz_prev = 0.0
            ports["dwa_infeasible"] = True
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.FAILURE

        v = v[admissible]
        w = w[admissible]
        dist = dist[admissible]

        yaw = float(obs.pose_estimate[2])
        target = self._target_bearing(obs)
        # Alignment of the heading the candidate would have reached by T_W.
        heading = 1.0 - np.abs(_wrap(target - (yaw + w * T_W))) / math.pi
        clear = np.minimum(dist, CLEAR_SAT) / CLEAR_SAT
        speed = v / V_MAX

        score = ALPHA * _minmax(heading) + BETA * _minmax(clear) + GAMMA * _minmax(speed)
        best = int(np.argmax(score))

        self._vx_prev = float(v[best])
        self._wz_prev = float(w[best])
        ports["dwa_infeasible"] = False
        # vy is held at zero: the arc model behind every dist() above assumes
        # motion along a circle of radius vx/wz, and lateral drift invalidates it.
        ports.drive(Command(self._vx_prev, 0.0, self._wz_prev))
        return Status.RUNNING

    def _target_bearing(self, obs):
        """World-frame bearing to aim at, or the current heading if undisclosed."""
        x, y, yaw = obs.pose_estimate
        if obs.goal is None:
            # Degrades into "hold the committed direction on the fastest safe arc".
            return float(yaw)
        gx, gy = obs.goal
        return math.atan2(float(gy) - float(y), float(gx) - float(x))

    def _window(self):
        v_lo = max(0.0, self._vx_prev - A_V * T_W)
        v_hi = min(V_MAX, self._vx_prev + A_V * T_W)
        w_lo = max(-W_MAX, self._wz_prev - A_W * T_W)
        w_hi = min(W_MAX, self._wz_prev + A_W * T_W)
        vs = np.linspace(v_lo, v_hi, N_V)
        ws = np.linspace(w_lo, w_hi, N_W)
        grid_v, grid_w = np.meshgrid(vs, ws, indexing="ij")
        return grid_v.ravel(), grid_w.ravel()

    def _obstacles(self, obs):
        """Finite returns as robot-frame points, minus any already inside R_SAFE.

        A point nearer than R_SAFE is inside the swept disc at t=0, so the arc
        contact time is meaningless for it. Split by side: one ahead of the
        lateral axis blocks all forward motion, one at or behind it is dropped,
        because a forward arc opens the gap to it immediately. That keeps the
        node from driving into something 0.34 m in front while still letting it
        slide past a wall it is nearly brushing.
        """
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        finite = np.isfinite(ranges)
        r = ranges[finite]
        a = angles[finite]
        px = r * np.cos(a)
        py = r * np.sin(a)

        overlap = r <= R_SAFE
        blocked_now = bool(np.any(overlap & (px > 0.0)))
        keep = ~overlap
        return px[keep], py[keep], blocked_now

    def _clearance(self, v, w, px, py, range_max, blocked_now):
        """Free arc length and free arc angle for every candidate.

        Closed form rather than a stepped rollout: the robot centre rides a
        circle of radius |R| = |v/w| about C = (0, R), its angle on that circle
        is beta(t) = a0 + w*t with a0 = -sign(w)*pi/2, and an obstacle at polar
        (d, phi) about C is first swept when beta = phi - sign(w)*delta, where
        cos(delta) = (d^2 + R^2 - r_safe^2) / (2*d*|R|). Same answer as
        simulating, with no blind spot between steps.
        """
        dist = np.minimum(range_max, T_SIM * v)
        dist_ang = T_SIM * np.abs(w)

        moving = v > V_EPS
        if px.size:
            curved = moving & (np.abs(w) >= W_STRAIGHT)
            straight = moving & ~curved

            if np.any(straight):
                on_path = (np.abs(py) <= R_SAFE) & (px >= 0.0)
                if np.any(on_path):
                    entry_x = px[on_path] - np.sqrt(
                        np.maximum(R_SAFE ** 2 - py[on_path] ** 2, 0.0)
                    )
                    nearest = max(float(np.min(entry_x)), 0.0)
                    dist[straight] = np.minimum(dist[straight], nearest)
                # A straight arc sweeps no angle, so dist_ang stays unbounded.

            if np.any(curved):
                radius = v[curved][:, None] / w[curved][:, None]
                abs_r = np.abs(radius)
                dy = py[None, :] - radius
                dx = np.broadcast_to(px[None, :], dy.shape)
                d = np.hypot(dx, dy)

                # The path circle passes within R_SAFE of the point at all.
                hit = np.abs(d - abs_r) <= R_SAFE
                cos_delta = (d * d + radius * radius - R_SAFE ** 2) / (
                    2.0 * np.maximum(d, 1e-9) * np.maximum(abs_r, 1e-9)
                )
                delta = np.arccos(np.clip(cos_delta, -1.0, 1.0))

                turn_sign = np.sign(w[curved][:, None])
                start_angle = -0.5 * math.pi * turn_sign
                entry_angle = np.arctan2(dy, dx) - turn_sign * delta
                # Angle actually travelled before entry, measured in the
                # direction of turn, hence non-negative.
                travel = np.where(
                    hit,
                    np.mod(turn_sign * (entry_angle - start_angle), 2.0 * math.pi),
                    np.inf,
                )
                dist[curved] = np.minimum(dist[curved], np.min(abs_r * travel, axis=1))
                dist_ang[curved] = np.minimum(dist_ang[curved], np.min(travel, axis=1))

        if blocked_now:
            dist = np.where(moving, 0.0, dist)
        return dist, dist_ang


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
    # Admissibility caps speed by braking distance and the clearance term buys
    # detours, so the route is slower than walking straight at the goal.
    "cost_metric": "time_to_goal",
}


def micro(seed=0):
    """A 4.5 m square with a partition and one piece of furniture.

    The straight line from start to goal runs along y = x and enters the
    partition at (1.6, 1.6), so goal_seeker -- which does not sense -- drives
    into it. The detour is over the partition's free end (1.7 m of gap) and past
    the table (0.5 m of clearance above it), both far wider than the 0.7 m the
    admissibility test needs, so a robot that searches velocity space gets
    through without contact.

    The room is deliberately small: opposite wall distances along either axis
    sum to 4.5 m, and any 180-degree forward span contains two axis directions,
    so some forward beam always returns under 2.25 m. The trigger therefore
    holds at every step and the node is never gated off part-way through.
    """
    world = room(
        4.5,
        4.5,
        boxes=[
            (1.6, 0.0, 2.0, 2.8),   # partition rooted in the bottom wall
            (2.6, 2.6, 3.2, 3.2),   # table, occludes the far approach
        ],
    )
    return MicroScenario(
        id="dwa_partition_and_furniture",
        world=world,
        start=(0.7, 0.7, 0.785),
        goal=(3.8, 3.8),
        time_limit=40.0,
        seed=seed,
        goal_known=True,
    )
