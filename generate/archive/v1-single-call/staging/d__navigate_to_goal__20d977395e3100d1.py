import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PRINCIPLE = {
    "mechanism": (
        "A reactive avoider re-chooses which side of an obstruction to pass on every "
        "tick. Where the obstruction is near-symmetric the two sides score within noise "
        "of each other, so the winner flips tick to tick, the commanded heading "
        "oscillates about the goal bearing, and its time-average points back into the "
        "obstruction - a limit cycle in front of it, or a stall in a concave pocket. "
        "Latching the side at first detection turns a repeated instantaneous choice into "
        "one sequential choice: the robot keeps a single rotational sense and therefore "
        "makes monotone progress along the obstruction's boundary, which being bounded is "
        "traversed in finite time, so the robot emerges on the far side."
    ),
    "trigger": (
        "The robot's swept corridor toward a disclosed goal is occluded within the "
        "lookahead, i.e. a detour is required and its side is not yet decided."
    ),
    "derivation": (
        "With 181 beams over a full circle (~2 deg spacing) the corridor test is: for "
        "beams within +-90 deg of a candidate heading, block if any return falls within "
        "0.5 m laterally (0.3 m body radius + 0.2 m margin) and nearer than the lookahead "
        "min(3 m, range-to-goal). Candidate headings are swept from the goal bearing in "
        "4 deg steps (two beams) up to pi. At first blockage both sides are swept once, "
        "the cheaper is latched, and a difference smaller than one scan step counts as a "
        "tie broken by a fixed constant. Thereafter only the latched sense is swept, so "
        "the commanded heading rides the obstruction edge. The latch is released only "
        "after the goal corridor tests clear for 0.3 s continuously, which is the "
        "hysteresis that stops the release itself from chattering."
    ),
}

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}


def _wrap(a):
    """Wrap to (-pi, pi]. Works elementwise on arrays."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _corridor_blocked(r, ang, heading, horizon, half_width):
    """True if any return lies inside the swept corridor along `heading`.

    `r`/`ang` must already be filtered to finite returns: inf * cos(pi/2) is nan
    and would silently drop a beam from every comparison below.
    """
    rel = _wrap(ang - heading)
    forward = r * np.cos(rel)
    lateral = r * np.sin(rel)
    hit = (forward > 0.0) & (forward < horizon) & (np.abs(lateral) < half_width)
    return bool(np.any(hit))


def _first_clear_offset(r, ang, base, horizon, sign, half_width, step):
    """Smallest offset in the given rotational sense whose corridor is clear.

    Sweeping only one sense is the whole mechanism: it is what makes successive
    commands consistent instead of independently optimal.
    """
    n = int(math.pi / step)
    for k in range(n + 1):
        off = sign * k * step
        if not _corridor_blocked(r, ang, base + off, horizon, half_width):
            return off
    return None


def _openness(r, ang, base, sign):
    """Mean return depth in the forward quadrant on one side of `base`."""
    rel = _wrap(ang - base)
    half = (rel > 0.0) & (rel < 0.5 * math.pi) if sign > 0 else (rel < 0.0) & (rel > -0.5 * math.pi)
    if not np.any(half):
        return 0.0
    return float(np.mean(r[half]))


class CommittedDetour(Action):
    """Drive to a disclosed goal, passing each obstruction on a latched side.

    Implements the committed-lateral-detour principle. While the corridor to the
    goal is clear the node simply steers down it. When the corridor is occluded it
    picks a rotational sense *once*, holds it, and from then on commands the
    smallest heading offset in that sense whose corridor is clear - which tracks
    the obstruction's edge. The latch is dropped only after the goal corridor has
    been clear continuously for a short dwell, so neither the commitment nor its
    release can chatter.

    It should work because the pathology it targets is temporal, not perceptual:
    each tick's greedy choice is individually sensible and the sequence of them is
    not. Holding one sense makes progress along the obstruction boundary monotone,
    and a bounded boundary is finite.
    """

    PORTS = (
        Port("detour_active", PortType.FLAG, Direction.OUT),
        Port("detour_side_left", PortType.FLAG, Direction.OUT),
        Port("detour_heading", PortType.BEARING, Direction.OUT),
    )
    PRECONDITION = "The goal position has been disclosed to the robot."
    EFFECT = (
        "Moves the robot to the goal, passing each occluding obstruction on a single "
        "side latched at the moment that obstruction was first detected."
    )
    TAGS = ("detour", "commitment", "hysteresis", "oscillation", "local-minimum", "goal-directed")

    HALF_WIDTH = 0.50          # 0.3 m body radius + 0.2 m margin
    LOOKAHEAD = 3.0            # half of range_max: far enough to react, near enough to be current
    SCAN_STEP = math.radians(4.0)   # two beams; finer than this is below the scan's resolution
    TIE_BAND = SCAN_STEP            # sides closer than one step apart count as tied
    CLEAR_DWELL = 0.3          # seconds the goal corridor must stay clear before unlatching
    GOAL_TOL = 0.35
    V_NOM = 0.6
    K_YAW = 2.5
    TURN_ONLY = 1.1            # rad of heading error above which we rotate without translating

    _side = None               # +1 = left, -1 = right, None = not committed
    _clear_since = None

    def reset(self, scenario, rng):
        self._side = None
        self._clear_since = None

    def precondition(self, obs, ports):
        return obs.goal is not None

    def tick(self, obs, ports):
        if obs.goal is None:
            # Goal withheld on this episode so far: nothing for this node to commit to.
            ports["detour_active"] = False
            ports["detour_side_left"] = False
            ports["detour_heading"] = 0.0
            return Status.FAILURE

        x, y, yaw = obs.pose_estimate
        gx, gy = obs.goal
        dx, dy = gx - x, gy - y
        dist = math.hypot(dx, dy)
        bearing = _wrap(math.atan2(dy, dx) - yaw)

        if dist <= self.GOAL_TOL:
            self._side = None
            self._clear_since = None
            ports["detour_active"] = False
            ports["detour_side_left"] = False
            ports["detour_heading"] = 0.0
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        finite = np.isfinite(obs.ranges)
        r = obs.ranges[finite]
        ang = obs.range_angles[finite]
        horizon = min(self.LOOKAHEAD, dist)

        if r.size == 0:
            blocked = False
        else:
            blocked = _corridor_blocked(r, ang, bearing, horizon, self.HALF_WIDTH)

        if blocked:
            self._clear_since = None
            if self._side is None:
                self._side = self._choose_side(r, ang, bearing, horizon)
        elif self._side is not None:
            # Release is debounced for the same reason the choice is latched.
            if self._clear_since is None:
                self._clear_since = obs.t
            elif obs.t - self._clear_since >= self.CLEAR_DWELL:
                self._side = None
                self._clear_since = None

        spin = False
        if self._side is None or r.size == 0:
            heading = bearing
        else:
            off = _first_clear_offset(r, ang, bearing, horizon, self._side,
                                      self.HALF_WIDTH, self.SCAN_STEP)
            if off is None:
                # Enclosed at this lookahead: keep the committed sense and rotate
                # in place until a clear corridor comes into view.
                heading = bearing + self._side * 0.5 * math.pi
                spin = True
            else:
                heading = bearing + off

        err = _wrap(heading)
        wz = float(np.clip(self.K_YAW * err, -1.0, 1.0))
        if spin or abs(err) > self.TURN_ONLY or obs.contact:
            vx = 0.0
        else:
            vx = self.V_NOM * math.cos(err)
            vx *= min(1.0, dist / 0.6)

        ports["detour_active"] = self._side is not None
        ports["detour_side_left"] = self._side == 1
        ports["detour_heading"] = float(heading)
        ports.drive(Command(vx, 0.0, wz))
        return Status.RUNNING

    def _choose_side(self, r, ang, bearing, horizon):
        """Latch a rotational sense. Called once per obstruction, never per tick."""
        if r.size == 0:
            return 1
        left = _first_clear_offset(r, ang, bearing, horizon, 1, self.HALF_WIDTH, self.SCAN_STEP)
        right = _first_clear_offset(r, ang, bearing, horizon, -1, self.HALF_WIDTH, self.SCAN_STEP)
        if left is not None and right is not None:
            # A tie is resolved by a constant on purpose: the value of committing does
            # not come from picking well, it comes from picking once.
            return -1 if abs(right) < abs(left) - self.TIE_BAND else 1
        if left is not None:
            return 1
        if right is not None:
            return -1
        return 1 if _openness(r, ang, bearing, 1) >= _openness(r, ang, bearing, -1) else -1


def micro(seed=0):
    """Mirror-symmetric pocket: equal-width gaps above and below, goal straight ahead.

    The symmetry makes the side choice a genuine tie, so a controller that re-decides
    each tick has nothing to break it; the baseline has no sensing at all and drives
    into the back of the pocket.
    """
    world = room(
        8.0, 6.0,
        boxes=[
            (3.8, 1.6, 4.2, 4.4),   # back of the pocket, across the line of sight
            (2.6, 1.6, 4.2, 2.0),   # lower flange
            (2.6, 4.0, 4.2, 4.4),   # upper flange
        ],
    )
    return MicroScenario(
        id="committed_detour_symmetric_pocket",
        world=world,
        start=(1.0, 3.0, 0.0),
        goal=(6.8, 3.0),
        time_limit=60.0,
        seed=seed,
        goal_known=True,
    )
