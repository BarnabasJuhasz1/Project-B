"""Speed bound derived from occlusion boundaries (Fraichard & Asama 2004).

An inevitable collision state is one from which a collision follows for ANY
future control sequence.  Safety is therefore a property of the state being
entered, and a speed bound computed only against *visible* obstacles is unsound
wherever the world is occluded: the free distance it reports is an artefact of
what happens to be in line of sight.

This node implements the one repair the card asks for.  At every depth
discontinuity in the scan the far side is treated as potentially occupied at the
NEAR side's range, and the forward free distance is measured against those
phantom points as well as against real returns.  The cap therefore binds on
approach to a blind corner rather than on first sight of whatever is behind it.

Steering here is a carrier, not the contribution: it is a plain goal-directed
gap choice computed from real returns only, i.e. the same information
`reactive_avoid` uses.  The speed law is the only thing that differs from the
baseline, which is what keeps the prediction falsifiable about this mechanism.
"""

import math

import numpy as np

from core.observation import Command
from harness.micro import MicroScenario, room
from nodes.contract import Action, Direction, Port, PortType, Status

PREDICTION = {
    "metric": "min_clearance",
    "baseline": "reactive_avoid",
    "min_effect": 0.1,
    "cost_metric": "time_to_goal",
}

_ROBOT_RADIUS = 0.30
_CLEARANCE = 0.35        # margin we insist on holding beyond the robot's own body
_CORRIDOR = _ROBOT_RADIUS + 0.12   # half-width of the swept tube we test against
_DEPTH_JUMP = 0.35       # range step between adjacent beams that counts as an edge
_STOP_TIME = 2.0         # s the robot is assumed to need to shed its speed
_V_NOM = 0.8
_W_MAX = 1.0
_K_YAW = 2.0
_GOAL_TOL = 0.35
_TURN_MARGIN = 0.30      # precondition looks slightly wider than the tube: we may turn


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _dense_ranges(obs):
    """Beams with no return read `inf`; treat them as returns at the horizon so
    that a wall-to-nothing transition still registers as a depth step."""
    r = np.asarray(obs.ranges, dtype=float)
    return np.where(np.isfinite(r), r, float(obs.range_max))


def _occlusion_points(rng, ang):
    """Phantom obstacles, one per depth discontinuity.

    A discontinuity between adjacent beams is an occlusion boundary: the shorter
    beam ends on an edge, the longer one passes it and sees whatever lies beyond.
    The far side is unobserved, so it is assumed occupied at the near beam's
    range.  Hence range = min(r_i, r_j) placed at the bearing of the LONGER beam
    -- placing it at the near beam's own bearing would just duplicate a return we
    already have, and would not bind before the corner.
    """
    nxt = np.roll(rng, -1)
    nxt_ang = np.roll(ang, -1)
    step = nxt - rng
    edge = np.abs(step) > _DEPTH_JUMP
    near = np.minimum(rng, nxt)
    far_ang = np.where(step > 0.0, nxt_ang, ang)
    return near[edge], far_ang[edge]


def _corridor_distance(dist, rel_ang, half_width=_CORRIDOR):
    """Along-track distance to the nearest point inside the tube swept by a body
    of radius _ROBOT_RADIUS translating along its own x axis."""
    if dist.size == 0:
        return float("inf")
    lon = dist * np.cos(rel_ang)
    lat = dist * np.sin(rel_ang)
    inside = (lon > 0.0) & (np.abs(lat) <= half_width)
    if not np.any(inside):
        return float("inf")
    return float(np.min(lon[inside]))


class LimitSpeedToOcclusionBoundary(Action):
    """Bound forward speed by the distance to the nearest occlusion boundary.

    Mechanism: every depth discontinuity in the scan spawns a phantom obstacle on
    its far side at the near side's range; the effective free distance ahead is
    then measured against phantoms as well as real returns, and speed is capped
    at what can be stopped within it.  Rounding a blind corner this slows the
    robot on approach to the corner instead of on first sight of what is behind
    it, which is the whole difference from a dynamic-window bound.
    """

    PORTS = (
        Port("safe_speed", PortType.SPEED, Direction.OUT),
    )
    PRECONDITION = (
        "An occlusion boundary -- a depth discontinuity in the scan -- lies "
        "within the distance the robot would need to stop from its current speed."
    )
    EFFECT = (
        "Caps the commanded forward speed at what can be brought to rest before "
        "the nearest occlusion boundary, treating the far side of every depth "
        "discontinuity as occupied."
    )
    TAGS = ("safety", "speed-limit", "occlusion", "inevitable-collision-state",
            "blind-corner")
    SOURCE = ("Fraichard, T. & Asama, H. (2004). Inevitable Collision States: a "
              "step towards safer robots. Advanced Robotics 18(10), 1001-1024.")

    def reset(self, scenario, rng):
        self._speed = 0.0

    def precondition(self, obs, ports):
        rng = _dense_ranges(obs)
        ang = np.asarray(obs.range_angles, dtype=float)
        occ_d, occ_a = _occlusion_points(rng, ang)
        if occ_d.size == 0:
            return False
        # An ICS is a property of the state being ENTERED, so the stopping
        # distance is evaluated against the speed the robot would carry into the
        # occluded region -- not against zero just because it happens to be
        # standing still, which would make the bound impossible to ever engage.
        speed = max(getattr(self, "_speed", 0.0), _V_NOM)
        d = _corridor_distance(occ_d, occ_a, _CORRIDOR + _TURN_MARGIN)
        return d <= speed * _STOP_TIME

    def tick(self, obs, ports):
        rng = _dense_ranges(obs)
        ang = np.asarray(obs.range_angles, dtype=float)
        horizon = float(obs.range_max)

        # obs.goal is NOT DISCLOSED on search scenarios and latches once found.
        goal = obs.goal
        if goal is not None:
            px, py, yaw = obs.pose_estimate
            dx = float(goal[0]) - px
            dy = float(goal[1]) - py
            if math.hypot(dx, dy) <= _GOAL_TOL:
                self._speed = 0.0
                ports["safe_speed"] = 0.0
                ports.drive(Command(0.0, 0.0, 0.0))
                return Status.SUCCESS
            goal_bearing = _wrap(math.atan2(dy, dx) - yaw)
        else:
            goal_bearing = 0.0  # nothing to steer to yet: hold heading, still capped

        # --- steering: real returns only, deliberately the baseline's information
        cand = np.linspace(-1.75, 1.75, 37)
        rel = _wrap(ang[None, :] - cand[:, None])
        lon = rng[None, :] * np.cos(rel)
        lat = rng[None, :] * np.sin(rel)
        inside = (lon > 0.0) & (np.abs(lat) <= _CORRIDOR)
        free = np.minimum(np.where(inside, lon, np.inf).min(axis=1), horizon)
        score = np.minimum(free, 3.0) - 1.2 * np.abs(_wrap(cand - goal_bearing))
        score = np.where(free < _ROBOT_RADIUS + 0.15, score - 10.0, score)
        heading = float(cand[int(np.argmax(score))])

        # --- speed: real returns AND the phantoms behind every occlusion boundary
        occ_d, occ_a = _occlusion_points(rng, ang)
        d_visible = _corridor_distance(rng, ang)
        d_boundary = _corridor_distance(occ_d, occ_a)
        d_eff = min(d_visible, d_boundary, horizon)

        # Stopping is modelled as a fixed time constant rather than a
        # deceleration limit: the locomotion policy is frozen, so its braking
        # transient is not ours to assume, and the conservative reading is what
        # ICS asks for.
        stoppable = (d_eff - _ROBOT_RADIUS - _CLEARANCE) / _STOP_TIME
        v_cap = float(min(_V_NOM, max(0.0, stoppable)))

        vx = v_cap * max(0.0, math.cos(heading))
        wz = float(np.clip(_K_YAW * heading, -_W_MAX, _W_MAX))

        self._speed = vx
        ports["safe_speed"] = v_cap
        ports.drive(Command(vx, 0.0, wz))
        return Status.RUNNING


def micro(seed=0):
    """An L-shaped route whose corner is blind and whose shadow is occupied.

    The interior block hides the entire northern half from the corridor, so the
    obstacle at (6.2, 4.0) is not merely far away -- it is geometrically
    unobservable until the robot has committed to the turn.  The corridor
    furniture keeps an occlusion edge inside the stopping horizon along the whole
    route, so the trigger is live rather than intermittent.  A gap follower
    reaches the goal here but takes the corner at full speed and meets the hidden
    block at close range; the bound makes the approach slow instead.
    """
    world = room(
        9.0, 9.0,
        boxes=[
            (0.0, 2.4, 5.4, 9.0),   # interior block: forms the corridor and the blind corner
            (2.2, 0.0, 2.6, 0.9),   # furniture: an early occlusion edge
            (3.8, 0.0, 4.3, 0.8),   # furniture: keeps an edge inside the horizon
            (6.2, 4.0, 7.0, 4.8),   # what is waiting in the corner's shadow
            (8.0, 6.0, 8.5, 6.6),   # furniture on the final approach
        ],
    )
    return MicroScenario(
        id="blind_corner_occlusion_bound",
        world=world,
        start=(1.2, 1.4, 0.0),
        goal=(7.4, 7.6),
        time_limit=120.0,
        seed=seed,
        goal_known=True,
    )
