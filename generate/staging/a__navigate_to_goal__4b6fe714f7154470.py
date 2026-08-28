"""Bug2-style committed boundary following for a disclosed goal in an unmapped room."""

import math

import numpy as np

from core.observation import Command
from harness.micro import MicroScenario, room
from nodes.contract import Action, Direction, Port, PortType, Status

_TWO_PI = 2.0 * math.pi


def _wrap(a):
    return (a + math.pi) % _TWO_PI - math.pi


def _wrap_arr(a):
    return (a + np.pi) % _TWO_PI - np.pi


class FollowBoundaryToGoal(Action):
    """Bug2: go straight at the disclosed goal; when the corridor to it is blocked,
    commit to one side of the obstruction and hug its boundary until the goal line
    is clear again *and* we are closer than we were when we hit it.

    Boundary following is a tangent law on the nearest sensed obstacle point: head
    along (bearing_to_nearest - side * 90 deg), plus a small term that pulls us
    back to a standoff distance. That one law covers the three cases a phase
    machine usually needs separately -- surface dead ahead at the moment of
    commitment (the tangent is then +-90 deg, so we turn to put it on the chosen
    side), concave corners (the nearest point swings ahead, we turn away), convex
    corners (the nearest point falls behind, we turn in and wrap around).

    The commitment is the mechanism. A memoryless steer-at-the-widest-gap
    controller abandons a detour as soon as it increases range-to-goal, which is
    exactly what every detour does halfway through. The hit-distance release test
    is what stops us from sliding back down the same face we just climbed.
    """

    PORTS = (
        # True while we are hugging an obstruction rather than beelining.
        Port("boundary_following", PortType.FLAG, Direction.OUT),
        Port("goal_bearing", PortType.BEARING, Direction.OUT),
    )
    PRECONDITION = "The goal position has been disclosed to the robot."
    EFFECT = "The robot arrives at the goal by circumnavigating whatever blocks the straight line to it."
    TAGS = ("bug2", "boundary-following", "detour", "unmapped", "goal-known", "local-minimum")

    V_MAX = 0.55
    W_MAX = 1.0
    GOAL_TOL = 0.20
    CORRIDOR_HALF = 0.45     # robot radius 0.3 plus margin: half-width of the corridor we need
    LOOKAHEAD = 2.2          # how far ahead a blockage has to be before we care
    STANDOFF = 0.70          # distance we try to hold from the boundary being followed
    RELEASE_GAIN = 0.30      # metres of progress past the hit distance before we may let go
    MIN_FOLLOW_T = 0.8       # seconds of commitment before the release test is even evaluated
    STALL_WINDOW = 3.0
    STALL_DIST = 0.15

    # ---------------------------------------------------------------- state

    def _ensure(self):
        # Lazily built so the node works even if the tree never calls reset().
        if not hasattr(self, "_following"):
            self._clear_state()

    def _clear_state(self):
        self._following = False
        self._side = -1          # +1 == obstruction on our left, -1 == on our right
        self._hit_dist = math.inf
        self._follow_since = 0.0
        self._win_t = None
        self._win_p = (0.0, 0.0)

    def reset(self, scenario, rng):
        self._clear_state()

    # --------------------------------------------------------------- sensing

    def _scan(self, obs):
        r = np.asarray(obs.ranges, dtype=float)
        r = np.where(np.isfinite(r), r, obs.range_max)
        return np.clip(r, 0.0, obs.range_max), np.asarray(obs.range_angles, dtype=float)

    def _blocked(self, r, ang, bearing, reach, half_width):
        """Is the straight corridor of the given half-width, out to `reach`, obstructed?"""
        rel = _wrap_arr(ang - bearing)
        ahead = np.abs(rel) < (math.pi / 2.0)
        forward = r * np.cos(rel)
        lateral = r * np.sin(rel)
        return bool(np.any(ahead & (forward < reach) & (np.abs(lateral) < half_width)))

    def _escape_offset(self, r, ang, bearing, sign):
        """Smallest angular deviation from the goal bearing, on the given side, that
        opens a clear corridor. None if this side offers no way past."""
        for deg in range(15, 106, 8):
            off = sign * math.radians(deg)
            if not self._blocked(r, ang, _wrap(bearing + off), self.LOOKAHEAD, self.CORRIDOR_HALF):
                return math.radians(deg)
        return None

    def _choose_side(self, r, ang, bearing):
        """Pick the detour direction, then report which side the obstruction ends up on.

        Going around to the left leaves the obstruction on our right, hence the sign
        flip. Prefer the side whose escape needs the smaller deviation from the goal
        bearing; if neither side opens up within the scan, fall back to whichever
        half-plane simply has more room in it.
        """
        left = self._escape_offset(r, ang, bearing, +1.0)
        right = self._escape_offset(r, ang, bearing, -1.0)
        if left is not None and (right is None or left <= right):
            return -1
        if right is not None:
            return +1
        rel = _wrap_arr(ang - bearing)
        left_room = float(np.mean(r[(rel > 0.2) & (rel < 1.8)])) if np.any((rel > 0.2) & (rel < 1.8)) else 0.0
        right_room = float(np.mean(r[(rel < -0.2) & (rel > -1.8)])) if np.any((rel < -0.2) & (rel > -1.8)) else 0.0
        return -1 if left_room >= right_room else +1

    def _front_clearance(self, r, ang):
        cone = np.abs(_wrap_arr(ang)) < math.radians(25.0)
        return float(np.min(r[cone])) if np.any(cone) else math.inf

    def _nearest(self, r, ang, side, range_max):
        """Closest return in the hemisphere that contains the followed side, so the
        opposite wall of a corridor cannot steal the tangent."""
        seen = r < (range_max - 1e-3)
        half = (side * ang) > -math.radians(70.0)
        mask = seen & half
        if not np.any(mask):
            mask = seen
            if not np.any(mask):
                return None
        idx = int(np.argmin(np.where(mask, r, np.inf)))
        return float(r[idx]), float(ang[idx])

    # ------------------------------------------------------------ stall watch

    def _stalled(self, obs, x, y):
        if self._win_t is None or obs.t < self._win_t:
            self._win_t, self._win_p = obs.t, (x, y)
            return False
        if obs.t - self._win_t < self.STALL_WINDOW:
            return False
        moved = math.hypot(x - self._win_p[0], y - self._win_p[1])
        self._win_t, self._win_p = obs.t, (x, y)
        return moved < self.STALL_DIST

    # ------------------------------------------------------------------ tick

    def precondition(self, obs, ports):
        return obs.goal is not None

    def tick(self, obs, ports):
        self._ensure()

        goal = obs.goal
        if goal is None:
            # Nothing to circumnavigate towards; a sibling has to disclose the goal first.
            ports["boundary_following"] = False
            return Status.FAILURE

        r, ang = self._scan(obs)
        x, y, yaw = obs.pose_estimate
        dx, dy = goal[0] - x, goal[1] - y
        dist = math.hypot(dx, dy)
        bearing = _wrap(math.atan2(dy, dx) - yaw)
        ports["goal_bearing"] = bearing

        if dist <= self.GOAL_TOL:
            ports["boundary_following"] = False
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        stalled = self._stalled(obs, x, y)
        near = self._nearest(r, ang, self._side, obs.range_max)

        if obs.contact:
            # Peel off whatever we are leaning on before doing anything clever.
            away = -math.copysign(0.7, near[1]) if near is not None and abs(near[1]) > 1e-3 else -0.7 * self._side
            ports["boundary_following"] = self._following
            ports.drive(Command(-0.25, 0.0, away))
            return Status.RUNNING

        if self._following:
            if stalled:
                # Committed side is not paying off: reverse the commitment rather
                # than grind, and re-arm the release test from here.
                self._side = -self._side
                self._hit_dist = dist
                self._follow_since = obs.t
            elif (obs.t - self._follow_since > self.MIN_FOLLOW_T
                  and dist < self._hit_dist - self.RELEASE_GAIN
                  and not self._blocked(r, ang, bearing, min(dist, self.LOOKAHEAD),
                                        self.CORRIDOR_HALF + 0.10)):
                # Wider corridor than the commit test: hysteresis, so grazing the
                # obstruction we just left does not re-trigger a detour.
                self._following = False
        else:
            if stalled or self._blocked(r, ang, bearing, min(dist, self.LOOKAHEAD), self.CORRIDOR_HALF):
                self._following = True
                self._side = self._choose_side(r, ang, bearing)
                self._hit_dist = dist
                self._follow_since = obs.t
                near = self._nearest(r, ang, self._side, obs.range_max)

        ports["boundary_following"] = self._following

        if self._following and near is not None:
            d_min, beta = near
            offset = float(np.clip(d_min - self.STANDOFF, -0.6, 0.6))
            heading_err = _wrap(beta - self._side * (math.pi / 2.0) + self._side * offset)
        else:
            heading_err = bearing

        wz = float(np.clip(1.8 * heading_err, -self.W_MAX, self.W_MAX))

        if abs(heading_err) > 1.2:
            vx = 0.0                      # turn on the spot rather than carve a wide arc
        else:
            vx = self.V_MAX * math.cos(heading_err)
        if not self._following:
            vx = min(vx, max(0.12, 1.2 * dist))   # ease in so we do not overshoot the goal

        d_front = self._front_clearance(r, ang)
        vx *= float(np.clip((d_front - 0.40) / 0.60, 0.0, 1.0))
        if vx < 0.05:
            vx = 0.0

        ports.drive(Command(vx, 0.0, wz))
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}


def micro(seed=0):
    """A room split by a partition with the only gap at the far end.

    The straight line from start to goal runs into the partition 2.7 m out, and the
    partition is not visible as an obstruction until the robot is close to it, so
    the detour cannot be planned -- it has to be committed to on contact with the
    surface. `goal_seeker` pins itself against the partition and never leaves it.
    """
    world = room(8.0, 6.0, boxes=[(3.90, 0.0, 4.20, 4.20)])
    return MicroScenario(
        id="partition_detour_commit",
        world=world,
        start=(1.20, 1.30, 0.0),
        goal=(6.60, 1.30),
        time_limit=60.0,
        seed=seed,
        goal_known=True,
    )
