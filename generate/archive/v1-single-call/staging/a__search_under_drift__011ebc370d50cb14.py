"""Drift-discounted frontier exploration for an undisclosed goal."""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

_CELL = 0.5            # grid pitch, m
_WINDOW = 10           # half-width of the scored neighbourhood, in cells
_CLEAR_RADIUS = 3.0    # how far out a scan is trusted to have RECOGNISED the goal
_CONE = 0.30           # half-angle of the corridor the body needs, rad
_REPLAN_DT = 0.25      # s between map updates / heading choices
_STEP_MAX = 2.5        # how far ahead a candidate direction is evaluated, m
_V_MAX = 0.8
_W_MAX = 1.0


def _wrap(a):
    """Fold an angle (scalar or array) into (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _trust(err2_now, err2_written):
    """How much an entry stamped at `err2_written` is worth at `err2_now`.

    Drift is a random walk, so error variance is additive in time and the
    uncertainty in the entry's position RELATIVE to the present belief is
    err2_now - err2_written. Once that exceeds a cell the entry no longer
    says anything about the cell it is filed under, and reverts to unknown.
    """
    rel = err2_now - err2_written
    if rel <= 0.0:
        return 1.0
    return (_CELL * _CELL) / (_CELL * _CELL + rel)


class FrontierSeekDriftDiscounted(Action):
    """Drive at the reachable direction holding the most goal-probability mass.

    A coarse grid in the (drifting) belief frame records two things per cell:
    free-evidence, laid down by marching each beam out to _CLEAR_RADIUS, and
    hit-evidence at beam returns. Each entry is stamped with the pose error
    variance at write time and discounted on read by _trust, so the record
    decays at exactly the rate the frame it is written in goes stale.

    Goal-probability mass lives on cells that are unknown but border a cell
    known to be free -- the frontier. Restricting mass to the frontier is what
    stops the permanently-unknown space beyond the walls from acting as an
    attractor, and it is what makes a stale free cell (trust decayed, now
    unknown again) become worth revisiting.

    Every candidate direction is scored by the mass-weighted reduction in
    distance to that mass. When obs.goal is None the mass is the frontier and
    this is exploration; when the goal latches the posterior collapses to a
    delta at obs.goal and the same expression is goal-seeking. One mechanism,
    both halves of the task.
    """

    PORTS = (
        Port("goal_disclosed", PortType.FLAG, Direction.OUT),
        Port("explore_heading", PortType.BEARING, Direction.OUT),
        Port("frontier_confidence", PortType.CONFIDENCE, Direction.OUT),
    )
    PRECONDITION = "The scan shows at least one direction with enough depth for the robot body to advance into."
    EFFECT = "Reduces the robot's expected distance to the places the goal can still be, by driving at the reachable direction with the most goal-probability mass ahead of it."
    TAGS = ("explore", "frontier", "search", "undisclosed-goal", "pose-drift", "coverage")

    def reset(self, scenario, rng):
        self._free = {}          # (i, j) -> (weight, err2_at_write)
        self._hit = {}
        self._target = None      # committed heading, belief-world frame
        self._conf = 0.0
        self._next_replan = -1.0
        self._trail = []         # (t, x, y) of the belief pose
        self._escape_until = -1.0
        self._escape_sign = 1.0
        # A world can be left/right symmetric about the robot; a fixed tiny
        # asymmetry breaks the tie instead of letting it dither on the axis.
        self._jitter = float(rng.uniform(-0.04, 0.04)) if rng is not None else 0.02

    def precondition(self, obs, ports):
        r = np.asarray(obs.ranges, dtype=float)
        r = np.where(np.isfinite(r), r, float(obs.range_max))
        return bool(r.max() > 0.75)

    # ---------------- map ----------------

    def _bump(self, table, px, py, inc, err2):
        key = (int(math.floor(px / _CELL)), int(math.floor(py / _CELL)))
        w, w_err2 = table.get(key, (0.0, err2))
        # decay what was there, add the new look, restamp: an entry is only as
        # fresh as its most recent evidence.
        table[key] = (min(1.0, w * _trust(err2, w_err2) + inc), err2)

    def _eff(self, table, key, err2):
        ent = table.get(key)
        if ent is None:
            return 0.0
        return ent[0] * _trust(err2, ent[1])

    def _ingest(self, x, y, yaw, ranges, angles, err2, rmax):
        step = 0.5 * _CELL
        for k in range(0, len(ranges), 3):
            r = float(ranges[k])
            a = yaw + float(angles[k])
            ca, sa = math.cos(a), math.sin(a)
            # Only clear out to _CLEAR_RADIUS: past that the goal could have
            # been in the beam and gone unrecognised, so "seen" would be a lie.
            lim = min(r - 0.15, _CLEAR_RADIUS)
            d = 0.0
            while d <= lim:
                self._bump(self._free, x + ca * d, y + sa * d, 0.5, err2)
                d += step
            if r < rmax - 1e-3:
                self._bump(self._hit, x + ca * r, y + sa * r, 0.6, err2)

    def _borders_known_free(self, key, err2):
        i, j = key
        for nb in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if self._eff(self._free, nb, err2) > 0.45 and self._eff(self._hit, nb, err2) <= 0.4:
                return True
        return False

    def _frontier_mass(self, x, y, err2):
        i0 = int(math.floor(x / _CELL))
        j0 = int(math.floor(y / _CELL))
        xs, ys, ms = [], [], []
        for di in range(-_WINDOW, _WINDOW + 1):
            for dj in range(-_WINDOW, _WINDOW + 1):
                key = (i0 + di, j0 + dj)
                if self._eff(self._hit, key, err2) > 0.4:
                    continue
                f = self._eff(self._free, key, err2)
                if f > 0.45:
                    continue
                if not self._borders_known_free(key, err2):
                    continue
                xs.append((key[0] + 0.5) * _CELL)
                ys.append((key[1] + 0.5) * _CELL)
                ms.append(1.0 - f)
        if not xs:
            return None
        return np.asarray(xs), np.asarray(ys), np.asarray(ms)

    def _paint_seen(self, x, y, err2):
        """Write off the ground under the robot after a stuck event.

        Not a claim that the goal is absent -- a claim that whatever is pulling
        us here is unreachable, so it should stop pulling.
        """
        n = int(0.9 / _CELL) + 1
        i0 = int(math.floor(x / _CELL))
        j0 = int(math.floor(y / _CELL))
        for di in range(-n, n + 1):
            for dj in range(-n, n + 1):
                self._free[(i0 + di, j0 + dj)] = (1.0, err2)

    # ---------------- heading ----------------

    def _choose(self, x, y, yaw, ranges, angles, err2, goal):
        idx = np.arange(0, len(angles), 4)
        cand = angles[idx]
        depth = np.empty(idx.size)
        for m, k in enumerate(idx):
            sel = np.abs(_wrap(angles - angles[k])) <= _CONE
            depth[m] = ranges[sel].min()
        ok = depth > 0.85
        if not ok.any():
            return None

        reach = np.clip(depth - 0.45, 0.0, _STEP_MAX)
        wa = yaw + cand
        ex = x + reach * np.cos(wa)
        ey = y + reach * np.sin(wa)

        if goal is not None:
            gx, gy = goal
            val = math.hypot(gx - x, gy - y) - np.hypot(gx - ex, gy - ey)
            conf = 1.0
        else:
            field = self._frontier_mass(x, y, err2)
            if field is None:
                val, conf = reach.copy(), 0.0
            else:
                cx, cy, mass = field
                drob = np.hypot(cx - x, cy - y)
                # Near mass first: distance reduction alone treats a cell 8 m
                # off as worth as much as one 2 m off, which is how you get a
                # robot that commits to the far side of the building forever.
                w = mass * np.exp(-drob / 3.0)
                tot = float(w.sum())
                if tot < 1e-6:
                    val, conf = reach.copy(), 0.0
                else:
                    dend = np.hypot(cx[None, :] - ex[:, None], cy[None, :] - ey[:, None])
                    val = ((drob[None, :] - dend) * w[None, :]).sum(axis=1) / tot
                    conf = float(np.clip(val.max() / _STEP_MAX, 0.0, 1.0))

        score = val - 0.18 * np.abs(cand) + self._jitter * cand
        if self._target is not None:
            score = score + 0.30 * np.cos(_wrap(wa - self._target))
        score = np.where(ok, score, -np.inf)
        b = int(np.argmax(score))
        return float(wa[b]), conf

    # ---------------- stuck ----------------

    def _update_trail(self, t, x, y):
        if not self._trail or t - self._trail[-1][0] >= 0.2:
            self._trail.append((t, x, y))
        while len(self._trail) > 1 and t - self._trail[0][0] > 3.2:
            self._trail.pop(0)

    def _is_stuck(self, t):
        # Measured in the drifting frame, so drift shows up as phantom motion.
        # That makes this conservative -- it under-fires, never over-fires.
        if len(self._trail) < 2:
            return False
        t0, x0, y0 = self._trail[0]
        if t - t0 < 3.0:
            return False
        return math.hypot(self._trail[-1][1] - x0, self._trail[-1][2] - y0) < 0.35

    # ---------------- tick ----------------

    def tick(self, obs, ports):
        if not hasattr(self, "_free"):
            self.reset(None, None)

        x, y, yaw = (float(v) for v in obs.pose_estimate)
        err2 = float(obs.pose_error_estimate) ** 2
        rmax = float(obs.range_max)
        angles = np.asarray(obs.range_angles, dtype=float)
        ranges = np.asarray(obs.ranges, dtype=float)
        ranges = np.clip(np.where(np.isfinite(ranges), ranges, rmax), 0.0, rmax)

        goal = obs.goal
        if goal is not None:
            goal = (float(goal[0]), float(goal[1]))
        ports["goal_disclosed"] = goal is not None

        self._update_trail(obs.t, x, y)

        if obs.t >= self._next_replan:
            self._next_replan = obs.t + _REPLAN_DT
            self._ingest(x, y, yaw, ranges, angles, err2, rmax)
            choice = self._choose(x, y, yaw, ranges, angles, err2, goal)
            if choice is None:
                ports["explore_heading"] = 0.0
                ports["frontier_confidence"] = 0.0
                ports.drive(Command(0.0, 0.0, 0.0))
                return Status.FAILURE
            self._target, self._conf = choice

        if self._target is None:
            self._target = yaw

        bearing = float(_wrap(self._target - yaw))
        ports["explore_heading"] = bearing
        ports["frontier_confidence"] = float(self._conf)

        if goal is not None and math.hypot(goal[0] - x, goal[1] - y) < 0.2:
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        if obs.t >= self._escape_until and self._is_stuck(obs.t):
            self._escape_until = obs.t + 1.2
            self._escape_sign = 1.0 if bearing >= 0.0 else -1.0
            self._paint_seen(x, y, err2)
            self._next_replan = self._escape_until
            self._trail.clear()

        if obs.t < self._escape_until:
            ports.drive(Command(-0.25, 0.0, self._escape_sign * 0.9))
            return Status.RUNNING

        sel = np.abs(_wrap(angles - bearing)) <= _CONE
        front = float(ranges[sel].min()) if sel.any() else rmax

        if obs.contact or front < 0.5:
            turn = 2.0 * bearing if abs(bearing) > 0.15 else 0.8 * self._escape_sign
            ports.drive(Command(-0.25, 0.0, float(np.clip(turn, -_W_MAX, _W_MAX))))
            return Status.RUNNING

        align = max(0.0, 1.0 - abs(bearing) / 0.9)
        vx = _V_MAX * align * float(np.clip((front - 0.5) / 1.2, 0.12, 1.0))
        wz = float(np.clip(2.2 * bearing, -_W_MAX, _W_MAX))

        # Small sideways slide off the nearest surface: cheap clearance in a
        # corridor, and it costs nothing in forward progress.
        vy = 0.0
        kmin = int(np.argmin(ranges))
        cmin = float(ranges[kmin])
        if cmin < 0.7:
            vy = float(np.clip(-0.6 * math.sin(float(angles[kmin])) * (0.7 - cmin) / 0.7, -0.35, 0.35))

        ports.drive(Command(vx, vy, wz))
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}


def micro(seed=0):
    """Three dead-end bays off a corridor; the goal is in the far one.

    Finding it means emptying bays in turn and not returning to an emptied
    one -- which takes a record of where the robot has been. The bay dividers
    run to the top wall so each bay is a true pocket, and they occlude the
    goal from the start pose, which is what makes goal_known=False legal here.
    """
    world = room(9.0, 7.0, boxes=[(2.6, 1.6, 3.0, 7.0),
                                  (5.6, 1.6, 6.0, 7.0)])
    return MicroScenario(id="frontier_three_bay_search",
                         world=world,
                         start=(0.9, 0.8, 0.0),
                         goal=(7.5, 5.2),
                         time_limit=200.0,
                         seed=seed,
                         goal_known=False)
