"""Boundary-shape guided detour: Cheng (1986) geometric module, applied to routing."""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status

from harness.micro import MicroScenario, room

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}

# --- shape percept -----------------------------------------------------------
# A return belongs to an "extended surface" only if it lies on a long, straight,
# connected run of beams. Everything shorter is a feature and is excluded from
# the shape representation entirely (Cheng's encapsulation).
_MIN_SURFACE_LEN = 1.2      # m, below this a run is furniture, not boundary
_MIN_SURFACE_BEAMS = 4
_MAX_RESIDUAL = 0.15        # m, perpendicular deviation that breaks a run at a corner
_GAP_MIN = 0.30             # m, point-to-point break distance at close range
_GAP_K = 3.0                # scales the break distance with beam footprint r*dtheta

# --- geometry of the robot ---------------------------------------------------
_ROBOT_RADIUS = 0.30
_CLEARANCE = 0.25
_CORRIDOR = _ROBOT_RADIUS + 0.15    # half-width of the swept lane to the goal

# --- openings ----------------------------------------------------------------
# A wall end is an opening only if the world gets substantially *farther* just
# past it. An occluding feature makes it nearer, which is why furniture edges
# never register as doorways.
_MIN_DEPTH_JUMP = 0.80      # m, > one robot diameter
_ALONG_OFFSET = 0.55        # m past the wall end, along the wall
_RADIAL_OFFSET = 0.40       # m past the wall plane, away from the robot

# --- control -----------------------------------------------------------------
_VMAX = 0.7
_WMAX = 1.0
_KW = 2.0
_GOAL_TOL = 0.35
_COMMIT_TIME = 1.5          # s, hold a chosen opening this long before re-scoring
_COMMIT_ARRIVED = 0.45      # m
_AMBIGUITY_BAND = 0.05      # relative cost difference below which shape cannot decide
_SAFE_RANGE = 0.55
_SAFE_STOP = 0.32


def _wrap(theta):
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


class ExtendedSurfaceDetour(Action):
    """Route around walls using only the shape of the extended surfaces.

    Implements the representational commitment of Cheng (1986): the surround is
    described by its extended boundary surfaces and their metric arrangement,
    and that description is encapsulated -- objects too small to be boundary do
    not enter it. Motion is then chosen as a metric relation to that shape: when
    an extended surface blocks the line to the goal, the node aims at the end of
    that surface (a real opening in the layout), not at whatever direction
    happens to look free. Because the choice is made on the boundary rather than
    on immediate free space, the node will commit to a waypoint that increases
    distance to the goal, which is what getting through a door in the far corner
    requires and what a progress-biased gap follower cannot do.

    Where the layout is symmetric about the goal ray, two openings score alike
    and shape alone cannot separate them -- Cheng's rotational error. The node
    picks one, commits, and publishes low `shape_confidence` so a caller can tell
    that the decision was arbitrary rather than informed.
    """

    PORTS = (
        Port("detour_bearing", PortType.BEARING, Direction.OUT),
        Port("shape_confidence", PortType.CONFIDENCE, Direction.OUT),
    )
    PRECONDITION = "The goal position has been disclosed to the robot."
    EFFECT = (
        "Drives the robot toward the goal on a route that rounds the ends of the "
        "extended boundary surfaces obstructing it."
    )
    TAGS = ("geometry", "boundary", "extended-surface", "detour", "shape",
            "cheng-1986", "local-minimum")

    # ------------------------------------------------------------------ state

    def reset(self, scenario, rng):
        self._commit_xy = None      # chosen opening, in the pose-belief frame
        self._commit_t = 0.0
        self._survey_dir = 0.0

    def _ensure_state(self):
        if not hasattr(self, "_commit_xy"):
            self.reset(None, None)

    # ------------------------------------------------------- shape extraction

    def _scan(self, obs):
        """Beams in angular order, rolled so index 0 starts at the widest break.

        Rolling at the largest discontinuity means a single physical surface is
        never split by the array seam, so the segmentation pass below needs no
        wrap-around case.
        """
        r = np.asarray(obs.ranges, dtype=float).ravel()
        a = np.asarray(obs.range_angles, dtype=float).ravel()
        n = min(r.size, a.size)
        r, a = r[:n], a[:n]
        order = np.argsort(a)
        r, a = r[order], a[order]

        good = np.isfinite(r)
        x = np.where(good, r * np.cos(a), np.nan)
        y = np.where(good, r * np.sin(a), np.nan)

        step = np.hypot(x - np.roll(x, 1), y - np.roll(y, 1))
        step = np.where(np.isnan(step), np.inf, step)
        k = int(np.argmax(step))
        return np.roll(r, -k), np.roll(a, -k), np.roll(x, -k), np.roll(y, -k)

    @staticmethod
    def _max_residual(x, y, i, j):
        ax, ay = x[i], y[i]
        ux, uy = x[j] - ax, y[j] - ay
        length = math.hypot(ux, uy)
        if length < 1e-6:
            return 0.0
        ux /= length
        uy /= length
        sx = x[i:j + 1] - ax
        sy = y[i:j + 1] - ay
        return float(np.max(np.abs(sy * ux - sx * uy)))

    def _extended_surfaces(self, r, x, y):
        """Greedy split of the scan into straight runs; keep only the long ones."""
        n = r.size
        if n < _MIN_SURFACE_BEAMS:
            return []
        dtheta = 2.0 * math.pi / n
        surfaces = []
        i = 0
        while i < n:
            if not np.isfinite(r[i]):
                i += 1
                continue
            j = i + 1
            while j < n and np.isfinite(r[j]):
                gap = math.hypot(x[j] - x[j - 1], y[j] - y[j - 1])
                if gap > max(_GAP_MIN, _GAP_K * r[j] * dtheta):
                    break
                if j - i >= 2 and self._max_residual(x, y, i, j) > _MAX_RESIDUAL:
                    break
                j += 1
            end = j - 1
            if (end - i + 1) >= _MIN_SURFACE_BEAMS:
                chord = math.hypot(x[end] - x[i], y[end] - y[i])
                if chord >= _MIN_SURFACE_LEN:
                    surfaces.append((i, end))
            i = max(j, i + 1)
        return surfaces

    # ------------------------------------------------------------ shape tests

    @staticmethod
    def _blocked_by_surface(surfaces, x, y, bearing, distance):
        """Does any extended surface intrude on the swept lane to the goal?"""
        ux, uy = math.cos(bearing), math.sin(bearing)
        for i, j in surfaces:
            sx = x[i:j + 1]
            sy = y[i:j + 1]
            along = sx * ux + sy * uy
            perp = np.abs(sy * ux - sx * uy)
            if np.any((along > 0.05) & (along < distance) & (perp < _CORRIDOR)):
                return True
        return False

    def _openings(self, r, x, y, surfaces):
        """Waypoints just around the ends of extended surfaces.

        Only a wall end with open depth beyond it counts. A wall end that abuts
        another wall (a corner) has no depth jump, and a wall end created by an
        occluding object has a *negative* jump, so neither becomes a waypoint.
        """
        n = r.size
        aims = []
        for i, j in surfaces:
            for end, inward, neighbour in ((i, j, i - 1), (j, i, j + 1)):
                if 0 <= neighbour < n:
                    beyond = r[neighbour]
                    if np.isfinite(beyond) and (beyond - r[end]) < _MIN_DEPTH_JUMP:
                        continue
                ex, ey = x[end], y[end]
                tx, ty = ex - x[inward], ey - y[inward]
                tlen = math.hypot(tx, ty)
                if tlen < 1e-6:
                    continue
                tx /= tlen
                ty /= tlen
                rlen = math.hypot(ex, ey)
                if rlen < 1e-6:
                    continue
                aims.append((ex + tx * _ALONG_OFFSET + (ex / rlen) * _RADIAL_OFFSET,
                             ey + ty * _ALONG_OFFSET + (ey / rlen) * _RADIAL_OFFSET))
        return aims

    # ------------------------------------------------------------------ drive

    def _emit(self, ports, obs, r, a, bearing, confidence):
        ports["detour_bearing"] = float(_wrap(bearing))
        ports["shape_confidence"] = float(min(1.0, max(0.0, confidence)))

        err = _wrap(bearing)
        wz = max(-_WMAX, min(_WMAX, _KW * err))
        vx = _VMAX * max(0.0, math.cos(err))
        vy = 0.0

        finite = np.isfinite(r)
        if np.any(finite):
            idx = int(np.argmin(np.where(finite, r, np.inf)))
            nearest = float(r[idx])
            if nearest < _SAFE_RANGE:
                # Features are absent from the shape percept, so they can only be
                # handled here: brake and slide clear without changing the aim.
                scale = (nearest - _SAFE_STOP) / (_SAFE_RANGE - _SAFE_STOP)
                vx *= max(0.0, min(1.0, scale))
                vy = -0.45 * math.sin(float(a[idx])) * (1.0 - max(0.0, scale))

        if obs.contact:
            vx = -0.35
            vy = 0.0

        ports.drive(Command(float(vx), float(vy), float(wz)))
        return Status.RUNNING

    # ------------------------------------------------------------------- tick

    def precondition(self, obs, ports):
        return obs.goal is not None

    def tick(self, obs, ports):
        self._ensure_state()

        if obs.goal is None:
            return Status.FAILURE

        px, py, yaw = obs.pose_estimate
        gx, gy = obs.goal
        dx, dy = gx - px, gy - py
        goal_dist = math.hypot(dx, dy)
        goal_bearing = _wrap(math.atan2(dy, dx) - yaw)

        r, a, x, y = self._scan(obs)

        if goal_dist <= _GOAL_TOL:
            self._commit_xy = None
            ports["detour_bearing"] = 0.0
            ports["shape_confidence"] = 1.0
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        surfaces = self._extended_surfaces(r, x, y)

        if not self._blocked_by_surface(surfaces, x, y, goal_bearing, goal_dist):
            self._commit_xy = None
            self._survey_dir = 0.0
            return self._emit(ports, obs, r, a, goal_bearing, 1.0)

        # Hold the current opening briefly. Under ambiguity the two best
        # candidates trade places tick to tick; committing is what turns an
        # undecidable choice into a route instead of a stall.
        if self._commit_xy is not None:
            cx, cy = self._commit_xy
            wx, wy = cx - px, cy - py
            lx = wx * math.cos(yaw) + wy * math.sin(yaw)
            ly = -wx * math.sin(yaw) + wy * math.cos(yaw)
            if math.hypot(lx, ly) < _COMMIT_ARRIVED:
                self._commit_xy = None
            elif obs.t - self._commit_t <= _COMMIT_TIME:
                return self._emit(ports, obs, r, a, math.atan2(ly, lx), 0.5)
            else:
                self._commit_xy = None

        aims = self._openings(r, x, y, surfaces)

        if not aims:
            # The shape percept is incomplete -- no wall end is in view. Turn in
            # place to finish it rather than guessing from partial geometry.
            if self._survey_dir == 0.0:
                self._survey_dir = 1.0 if goal_bearing >= 0.0 else -1.0
            ports["detour_bearing"] = float(goal_bearing)
            ports["shape_confidence"] = 0.0
            ports.drive(Command(0.0, 0.0, 0.8 * self._survey_dir))
            return Status.RUNNING

        self._survey_dir = 0.0

        # Score each opening by the length of the two-leg route through it.
        ugx = dx * math.cos(yaw) + dy * math.sin(yaw)
        ugy = -dx * math.sin(yaw) + dy * math.cos(yaw)
        costs = [math.hypot(ax, ay) + math.hypot(ugx - ax, ugy - ay) for ax, ay in aims]
        order = sorted(range(len(aims)), key=lambda k: costs[k])
        best = order[0]

        if len(order) > 1 and costs[best] > 1e-6:
            margin = (costs[order[1]] - costs[best]) / costs[best]
            confidence = min(1.0, margin / _AMBIGUITY_BAND)
        else:
            confidence = 1.0

        ax, ay = aims[best]
        self._commit_xy = (px + ax * math.cos(yaw) - ay * math.sin(yaw),
                           py + ax * math.sin(yaw) + ay * math.cos(yaw))
        self._commit_t = obs.t

        return self._emit(ports, obs, r, a, math.atan2(ay, ax), confidence)


def micro(seed=0):
    """A partitioned room whose only door is in the corner away from the goal.

    The straight line start->goal is cut by a 4.6 m partition. Reaching the door
    means moving away from the goal for roughly four metres, so the detour is
    only findable from the boundary layout, not from local free space. The two
    0.7 m boxes are deliberately below the extended-surface threshold: they are
    features, and their edges must not be read as openings.
    """
    world = room(
        8.0,
        6.0,
        boxes=[
            (3.85, 0.00, 4.15, 4.60),   # partition; door is the 1.4 m gap above it
            (1.00, 4.60, 1.70, 5.30),   # feature, off-route
            (6.80, 3.60, 7.50, 4.30),   # feature, off-route
        ],
    )
    return MicroScenario(
        id="extended_surface_detour_partition",
        world=world,
        start=(1.5, 1.2, 0.0),
        goal=(6.5, 1.2),
        time_limit=60.0,
        seed=seed,
        goal_known=True,
    )
