"""Boundary-following escape for a mapless quadruped with a drifting pose belief.

Mechanism: Bug-style boundary following. When the direct line to the believed
goal is blocked, latch a side and circulate the blocking surface at a fixed
standoff until the goal direction is clear again; otherwise walk the bearing.

Why it should work under drift: the follow loop reads nothing but the scan, and
the only pose-derived quantity in the whole node is the *bearing* to the goal.
A position error e at goal distance d perturbs a bearing by atan(e/d), which
stays small while there is still distance to cover.  The classic Bug leave
conditions -- the m-line, or "am I closer than at the hit point" -- instead
compare absolute positions taken minutes apart, which is precisely where a
random-walk belief has accumulated its error.  So the leave test here is a
scan-space one: a robot-width swath along the goal bearing must be clear out to
a range inflated by obs.pose_error_estimate, i.e. the less the belief is worth,
the more clear floor is demanded before the wall is abandoned.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


def _wrap(a):
    """Wrap angle(s) into (-pi, pi]. Scalars and arrays both."""
    return np.arctan2(np.sin(a), np.cos(a))


def _sector_min(obs, centre, half_width):
    """Closest return within +/-half_width of a robot-frame bearing."""
    sel = np.abs(_wrap(obs.range_angles - centre)) <= half_width
    if not np.any(sel):
        return float(obs.range_max)
    return float(np.min(np.clip(obs.ranges[sel], 0.0, obs.range_max)))


def _sector_mean(obs, centre, half_width):
    """Mean open depth of a sector; no-return beams count as fully open."""
    sel = np.abs(_wrap(obs.range_angles - centre)) <= half_width
    if not np.any(sel):
        return 0.0
    return float(np.mean(np.clip(obs.ranges[sel], 0.0, obs.range_max)))


def _beam(obs, angle):
    """Range on the single beam nearest a robot-frame bearing."""
    i = int(np.argmin(np.abs(_wrap(obs.range_angles - angle))))
    return float(min(obs.ranges[i], obs.range_max))


class BoundaryFollowToGoal(Action):
    """Reach the believed goal, circulating the boundary of whatever blocks it.

    Two modes over one mechanism.  SEEK walks the goal bearing.  FOLLOW engages
    when a robot-width swath along that bearing is obstructed, and rides the
    obstructing surface at a fixed standoff until the swath is clear again.
    The follow side is chosen once, on engagement, from whichever flank has more
    open floor -- re-deciding it every tick is how these controllers end up
    oscillating in a concave pocket instead of leaving it.

    Heading and standoff are separated: wz only aligns the body with the wall
    tangent (estimated from two side beams), vy only regulates the standoff.
    That keeps the robot parallel to the surface rather than weaving into it.
    """

    PORTS = (
        Port("boundary_follow_engaged", PortType.FLAG, Direction.OUT),
        Port("commanded_heading", PortType.BEARING, Direction.OUT),
    )

    PRECONDITION = "The goal position has been disclosed on this episode."
    EFFECT = (
        "Drives the robot onto the goal, circulating the boundary of whatever "
        "blocks the direct line instead of pressing into it."
    )
    TAGS = (
        "navigation",
        "bug",
        "wall-following",
        "boundary-following",
        "local-minimum",
        "concave-obstacle",
        "drift-robust",
    )

    # Free corridor half-width demanded before the goal direction counts as
    # walkable: robot radius 0.3 plus a margin for the drifting belief.
    SWATH = 0.55
    STANDOFF = 0.6
    FRONT_STOP = 0.9           # rotate away rather than creep into a corner
    LOST_D = 2.2               # side beam beyond this means the wall ended
    DELTA = math.radians(25.0)  # separation of the two wall-tangent beams
    ARRIVE_R = 0.35
    V_CRUISE = 0.45
    V_ROUND = 0.35             # forward speed while curling round a convex corner
    K_ALIGN = 1.2
    K_LAT = 0.8
    K_SEEK = 1.5
    MIN_FOLLOW_S = 1.0         # hysteresis: no leaving in the first moment
    LOST_TIMEOUT_S = 4.0       # curling with nothing to curl around -> open space

    # Defaults so a tick before any reset() is still well defined.
    _engaged = False
    _side = 1
    _t_engaged = 0.0
    _t_lost = None

    def reset(self, scenario, rng):
        self._engaged = False
        self._side = 1
        self._t_engaged = 0.0
        self._t_lost = None

    def precondition(self, obs, ports):
        return obs.goal is not None

    def tick(self, obs, ports):
        if obs.goal is None:
            return self._emit(ports, Command(0.0, 0.0, 0.0), 0.0, Status.FAILURE)

        px, py, yaw = obs.pose_estimate
        dx = float(obs.goal[0]) - px
        dy = float(obs.goal[1]) - py
        dist = math.hypot(dx, dy)
        bearing = float(_wrap(math.atan2(dy, dx) - yaw))

        if dist <= self.ARRIVE_R:
            self._engaged = False
            return self._emit(ports, Command(0.0, 0.0, 0.0), bearing, Status.SUCCESS)

        if obs.contact:
            cmd, heading = self._unstick(obs)
            return self._emit(ports, cmd, heading, Status.RUNNING)

        blocked = self._blocked(obs, bearing, dist)

        if self._engaged:
            settled = (obs.t - self._t_engaged) >= self.MIN_FOLLOW_S
            lost_long = (
                self._t_lost is not None
                and (obs.t - self._t_lost) >= self.LOST_TIMEOUT_S
            )
            if settled and not blocked:
                self._engaged = False
            elif lost_long:
                # Curling with no surface in reach: there is nothing here to
                # follow, so stop pretending there is.
                self._engaged = False
        elif blocked:
            self._engaged = True
            self._side = self._pick_side(obs, bearing)
            self._t_engaged = obs.t
            self._t_lost = None

        if self._engaged:
            cmd, heading = self._follow(obs)
        else:
            self._t_lost = None
            cmd, heading = self._seek(obs, bearing, dist)
        return self._emit(ports, cmd, heading, Status.RUNNING)

    def _emit(self, ports, cmd, heading, status):
        ports["boundary_follow_engaged"] = bool(self._engaged)
        ports["commanded_heading"] = float(heading)
        ports.drive(cmd)
        return status

    def _blocked(self, obs, bearing, dist):
        """Is the robot-width swath along the goal bearing obstructed?

        Depth demanded grows with how wrong the pose belief is thought to be:
        if the goal is believed 2 m off and the belief may be 1 m wrong, seeing
        2 m of floor is not evidence that the way is open.
        """
        err = max(0.0, float(obs.pose_error_estimate))
        # The 0.9 cap also keeps no-return beams (clipped to range_max) from
        # ever registering as hits: at range_max a beam is either beyond `need`
        # or far outside the swath.
        need = min(dist + err, 0.9 * float(obs.range_max))

        rel = _wrap(obs.range_angles - bearing)
        r = np.clip(obs.ranges, 0.0, obs.range_max)
        along = r * np.cos(rel)
        perp = r * np.sin(rel)
        hit = (along > 0.05) & (along < need) & (np.abs(perp) < self.SWATH)
        return bool(np.any(hit))

    def _pick_side(self, obs, bearing):
        """+1 keeps the obstacle on our left, -1 on our right.

        Round the obstacle on whichever flank has more open floor, which means
        keeping the obstacle on the opposite hand.  Ties resolve to +1 so the
        behaviour is deterministic in a symmetric pocket.
        """
        half = math.radians(50.0)
        left = _sector_mean(obs, bearing + math.pi / 2.0, half)
        right = _sector_mean(obs, bearing - math.pi / 2.0, half)
        return 1 if right >= left else -1

    def _follow(self, obs):
        s = self._side
        perp_ang = s * math.pi / 2.0
        b = _beam(obs, perp_ang)                    # perpendicular to the wall
        a = _beam(obs, perp_ang - s * self.DELTA)   # DELTA ahead of it
        front = _sector_min(obs, 0.0, math.radians(26.0))

        if front < self.FRONT_STOP:
            # Concave corner. Turn away from the followed side on the spot;
            # translating here is what buries a wall follower in the corner.
            return Command(0.0, 0.0, -s * 1.0), 0.0

        if b >= self.LOST_D or a >= self.LOST_D:
            # Convex corner: the surface fell away, so curl back onto it. The
            # radius (V_ROUND / wz) is deliberately near the standoff.
            if self._t_lost is None:
                self._t_lost = obs.t
            return Command(self.V_ROUND, 0.0, s * 0.65), 0.0
        self._t_lost = None

        # Wall tangent from the two side beams: alpha is zero when parallel,
        # negative when the body is angled into the surface.
        alpha = math.atan2(
            a * math.cos(self.DELTA) - b,
            max(a * math.sin(self.DELTA), 1e-3),
        )
        d_perp = b * math.cos(alpha)

        wz = float(np.clip(self.K_ALIGN * s * alpha, -1.0, 1.0))
        vy = float(np.clip(self.K_LAT * s * (d_perp - self.STANDOFF), -0.25, 0.25))
        vx = self.V_CRUISE * float(
            np.clip((front - self.FRONT_STOP) / 1.2, 0.2, 1.0)
        )
        return Command(vx, vy, wz), math.atan2(vy, vx)

    def _seek(self, obs, bearing, dist):
        front = _sector_min(obs, 0.0, math.radians(20.0))
        wz = float(np.clip(self.K_SEEK * bearing, -1.0, 1.0))
        # Do not translate while swinging through a large heading error.
        gate = max(0.0, math.cos(bearing))
        vx = self.V_CRUISE * gate * float(np.clip((front - 0.5) / 1.0, 0.0, 1.0))
        vx = min(vx, 0.6 * dist + 0.05)
        return Command(vx, 0.0, wz), bearing

    def _unstick(self, obs):
        """In contact: reverse and rotate away from the nearest return."""
        r = np.clip(obs.ranges, 0.0, obs.range_max)
        near = float(obs.range_angles[int(np.argmin(r))])
        wz = -1.0 if near >= 0.0 else 1.0
        return Command(-0.25, 0.0, wz), math.pi


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}


def micro(seed=0):
    """A C-shaped pocket whose mouth faces away from the goal.

    Every direction with a positive component along the goal direction is
    capped by the back wall or by an arm, so a gap follower that only steers at
    progress-making gaps presses into the pocket. The only way out runs
    backwards relative to the goal, which is what boundary following supplies.
    """
    world = room(
        9.0,
        7.0,
        boxes=[
            (4.0, 1.5, 4.4, 5.2),   # back wall of the pocket
            (2.4, 4.9, 4.4, 5.2),   # upper arm
            (2.4, 1.5, 4.4, 1.8),   # lower arm
        ],
    )
    return MicroScenario(
        id="concave_pocket_boundary_escape",
        world=world,
        start=(3.2, 3.35, 0.0),
        goal=(7.0, 3.5),
        time_limit=75.0,
        seed=seed,
        goal_known=True,
    )
