"""Map-free exploration leaf: follow the boundary of the space on one fixed side.

Intended for the search half of a goal-unknown navigation task, where obs.goal
is None and the only useful thing to do is cover ground that has not been
covered yet.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


class FollowWallToExplore(Action):
    """Right-hand wall following as a coverage primitive for unmapped interiors.

    Mechanism: hold the nearest continuous surface on one fixed side of the
    robot at a constant standoff and drive forward along it.

    Why it should work: in a simply-connected indoor space the boundary of the
    free space is a single closed curve, so a controller that keeps that
    boundary consistently on one side traverses all of it -- including narrow
    alcoves, side chambers and dead ends. Undirected reactive search (steer at
    the widest gap) systematically skips exactly those places, because a
    doorway is never the widest gap on offer. When the goal is not disclosed,
    the places the gap follower skips are precisely where the goal is hiding.

    Two deliberate omissions. The node never reads obs.pose_estimate: control
    is entirely range-based, so the seeded pose drift cannot corrupt it, and
    the coverage argument rests on the topology of the boundary rather than on
    dead reckoning. The node never reads obs.goal either -- exploring and
    approaching a disclosed goal are two effects, and this node claims one.
    A parent or sibling switches branches once the goal latches.

    Hence tick() returns RUNNING for as long as it is following. It returns
    FAILURE only when no surface is within range for several seconds, which is
    the honest report that there is nothing here to follow.
    """

    PORTS = (
        # Where the followed surface currently is, robot frame: lets a downstream
        # node reason about which way the boundary runs without re-scanning.
        Port("followed_wall_bearing", PortType.BEARING, Direction.OUT),
        # True only while the standoff controller is actually tracking a surface
        # (i.e. not rounding a corner, recovering, or searching for one).
        Port("wall_lock", PortType.FLAG, Direction.OUT),
    )
    PRECONDITION = "Some surface is returning within sensing range, so there is a boundary available to follow."
    EFFECT = "Drives the robot along that boundary at a fixed standoff on one fixed side, sweeping perimeter it has not swept yet."
    TAGS = ("exploration", "coverage", "wall-following", "map-free", "pose-free", "reactive", "search")

    SIDE = -1.0                          # -1 keeps the wall on the right, +1 on the left
    STANDOFF = 0.70                      # m from the followed surface
    CRUISE = 0.45                        # m/s along the wall
    LOOKAHEAD = 0.55                     # m ahead the distance error is projected
    KP = 1.8
    WZ_MAX = 0.9                         # below the runner's 1.0 clip, so commands stay honest

    BEAM_SPLIT = math.radians(40.0)      # angle between the two wall-estimator beams
    FRONT_HALF = math.radians(25.0)      # half-width of the "blocked ahead" sector

    FRONT_STOP = 0.85                    # rotate away below this
    FRONT_CLEAR = 1.05                   # ...and keep rotating until above this (hysteresis)

    WALL_LOST = 1.90                     # perpendicular beam beyond this: the wall fell away
    ACQUIRE_RANGE = 2.60                 # something this close means we are at a corner, not adrift
    CORNER_VX = 0.35
    CORNER_WZ = 0.50                     # CORNER_VX / CORNER_WZ = 0.7 m arc, i.e. the standoff
    CORNER_TIMEOUT = 4.0                 # stop arcing if the wall never comes back

    BACKOFF_TIME = 0.6
    BACKOFF_VX = -0.30
    BACKOFF_WZ = 0.60
    SEARCH_TIMEOUT = 8.0                 # no surface at all for this long: mechanism does not apply

    # Class-level defaults so a tick before any reset() is still well defined.
    _t_prev = None
    _backoff = 0.0
    _corner = 0.0
    _searching = 0.0
    _turning = False

    def reset(self, scenario, rng):
        # Nothing about the scenario is inspected: the node is world-agnostic by
        # construction and learns everything it uses from obs at tick time.
        self._t_prev = None
        self._backoff = 0.0
        self._corner = 0.0
        self._searching = 0.0
        self._turning = False

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    @classmethod
    def _sector_min(cls, r, ang, centre, half):
        mask = np.abs(cls._wrap(ang - centre)) <= half
        if not bool(np.any(mask)):
            return float("inf")
        return float(np.min(r[mask]))

    @classmethod
    def _beam(cls, r, ang, target, k=3):
        """Range at a bearing, as the median of the k nearest beams.

        The scan is 2 degrees per beam and a single beam can clip a box edge, so
        a tiny median keeps the two-beam wall estimator from seeing a phantom
        wall angle at corners.
        """
        d = np.abs(cls._wrap(ang - target))
        k = int(min(k, d.size))
        idx = np.argpartition(d, k - 1)[:k]
        return float(np.median(r[idx]))

    def _clean(self, obs):
        rmax = float(obs.range_max)
        r = np.asarray(obs.ranges, dtype=float)
        r = np.where(np.isfinite(r), r, rmax)
        return np.clip(r, 0.0, rmax), np.asarray(obs.range_angles, dtype=float), rmax

    def precondition(self, obs, ports):
        r, _, rmax = self._clean(obs)
        return bool(float(np.min(r)) < 0.9 * rmax)

    def _emit(self, ports, bearing, lock, vx, wz, status):
        ports["followed_wall_bearing"] = float(bearing)
        ports["wall_lock"] = bool(lock)
        ports.drive(Command(float(vx), 0.0, float(wz)))
        return status

    def tick(self, obs, ports):
        r, ang, _ = self._clean(obs)
        side = float(self.SIDE)

        t = float(obs.t)
        dt = 0.0 if self._t_prev is None else min(0.5, max(0.0, t - self._t_prev))
        self._t_prev = t

        side_mask = (side * ang) > 0.0
        if bool(np.any(side_mask)):
            bearing = float(ang[int(np.argmin(np.where(side_mask, r, np.inf)))])
        else:
            bearing = side * math.pi / 2.0

        front = self._sector_min(r, ang, 0.0, self.FRONT_HALF)
        perp = side * math.pi / 2.0
        b = self._beam(r, ang, perp)
        a = self._beam(r, ang, perp - side * self.BEAM_SPLIT)
        nearest = float(np.min(r))

        if bool(obs.contact):
            self._backoff = self.BACKOFF_TIME
        if self._backoff > 0.0:
            self._backoff = max(0.0, self._backoff - dt)
            return self._emit(ports, bearing, False,
                              self.BACKOFF_VX, -side * self.BACKOFF_WZ, Status.RUNNING)

        # Concave corner: something ahead, wall still on the followed side, so
        # spin away from that side until the way forward reopens.
        if self._turning:
            if front > self.FRONT_CLEAR:
                self._turning = False
            else:
                return self._emit(ports, bearing, False, 0.0, -side * self.WZ_MAX, Status.RUNNING)
        if front < self.FRONT_STOP:
            self._turning = True
            self._corner = 0.0
            return self._emit(ports, bearing, False, 0.0, -side * self.WZ_MAX, Status.RUNNING)

        if b < self.WALL_LOST:
            self._corner = 0.0
            self._searching = 0.0
            # Two-beam wall estimator: alpha is the heading of the robot relative
            # to the wall, positive when heading away from it.
            alpha = math.atan2(a * math.cos(self.BEAM_SPLIT) - b, a * math.sin(self.BEAM_SPLIT))
            projected = b * math.cos(alpha) + self.LOOKAHEAD * math.sin(alpha)
            err = self.STANDOFF - projected
            wz = float(np.clip(-side * self.KP * err, -self.WZ_MAX, self.WZ_MAX))
            # Slow down in proportion to how hard we are correcting, so tight
            # corrections do not turn into wide overshoots.
            vx = self.CRUISE * max(0.35, 1.0 - 0.6 * abs(wz) / self.WZ_MAX)
            return self._emit(ports, bearing, True, vx, wz, Status.RUNNING)

        # Perpendicular beam fell off the wall. If a surface is still nearby this
        # is a convex corner: arc toward the followed side at standoff radius
        # rather than extrapolating the estimator across the discontinuity.
        if nearest < self.ACQUIRE_RANGE and self._corner < self.CORNER_TIMEOUT:
            self._corner += dt
            self._searching = 0.0
            return self._emit(ports, bearing, False,
                              self.CORNER_VX, side * self.CORNER_WZ, Status.RUNNING)

        # Genuinely adrift in open floor: go straight until something appears.
        self._corner = 0.0
        self._searching += dt
        if self._searching > self.SEARCH_TIMEOUT:
            return self._emit(ports, bearing, False, 0.0, 0.0, Status.FAILURE)
        return self._emit(ports, bearing, False, self.CRUISE, 0.0, Status.RUNNING)


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    # Following the boundary is roughly twice the direct route in this world and
    # worse in rooms with long featureless perimeters. That is what a coverage
    # guarantee without a map costs.
    "cost_metric": "path_length",
}


def micro(seed=0):
    """A room split into an open hall and an occluded side chamber.

    The partition touches the bottom wall and the upper stub touches the top
    wall, so the free space stays simply connected and its boundary is one
    closed loop that runs through the doorway and around the chamber -- the
    condition under which fixed-side following is guaranteed to cover it.

    The goal sits deep in the chamber, occluded from the start by the partition
    (goal_known=False), so the robot has to find it. goal_seeker's straight line
    from (1.0, 1.0) crosses the partition at y ~ 0.93 and it has no sensing with
    which to get around.
    """
    world = room(6.0, 4.0, boxes=[(4.0, 0.0, 4.25, 2.2),
                                  (4.0, 3.4, 4.25, 4.0)])
    return MicroScenario(id="occluded_side_chamber_search",
                         world=world,
                         start=(1.0, 1.0, 0.0),
                         goal=(5.1, 0.9),
                         time_limit=60.0,
                         seed=seed,
                         goal_known=False)
