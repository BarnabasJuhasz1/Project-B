"""
demo/baselines/dstar_lite.py

⚠️ HAND-WRITTEN REFERENCE BASELINE (Mindfire Exp B, review item E12). NOT A
GENERATED CONTROLLER, NOT IN ANY ARM, AND IN NO ARM'S COUNT.

D* Lite (Koenig & Likhachev, 2002), the optimised form, on an 8-connected
occupancy grid built from what the robot has seen. It is the incremental
replanner that Stentz (1994, D*) introduced and the armB controllers were
handed: the search runs from the GOAL to the robot, and when sensed costs
change only the affected vertices are repaired (UpdateVertex on every cell
whose cost changed and its eight neighbours, then ComputeShortestPath). The
key modifier k_m absorbs the robot's motion, so the priority queue is never
re-sorted.

⚠️ **INCREMENTAL ONLY. THERE IS NO FULL-REPLAN FALLBACK.** Each goal gets ONE
search tree, initialised the first time that goal is active and only ever
repaired after that -- including across legs: the tree to GOAL built on leg 0
is the one repaired when the door is found shut on leg 2. `inits` counts
initialisations per tree and the tests pin it at 1.

Everything that is not the planner -- the map, the cost function, the planning
schedule, the path follower, contact recovery -- is the SHARED block, byte
identical in demo/baselines/astar_replan.py. See demo/baselines/README.md.

    python demo/rollout.py demo/baselines/dstar_lite.py          # kinematic
    python demo/baselines/run_expB.py kinematic                  # Exp B, 3 reps
"""
# ==== SHARED BEGIN ===========================================================
# This block is BYTE-IDENTICAL in demo/baselines/dstar_lite.py and
# demo/baselines/astar_replan.py; demo/baselines/_shared.py is its single
# source and demo/baselines/test_baselines.py fails if any copy drifts. The two
# baselines differ ONLY in the planner below this block, so a difference between
# them is a difference between incremental repair and replanning from scratch,
# not between two maps, two followers or two recovery behaviours.
#
# Rules: the same as every generated controller (demo/task.md). Standard
# library + numpy + core.observation only; all state on the instance and
# cleared in reset(); step() never raises. NO WALL-CLOCK BUDGET anywhere: the
# behaviour is a pure function of the observations, so machine load cannot
# change a trajectory (Mindfire plan R5).
import heapq
import math
import time   # instrumentation only (stats["plan_ms"]); control never reads it

import numpy as np

from core.observation import Command

# ---- below this line is data, not code.

RES = 0.10            # m per grid cell
MARGIN = 4.0          # m of grid beyond the start/goal bounding box
L_OCC = 0.85          # log-odds added on a lidar hit
L_FREE = -0.40        # log-odds added on a cell a beam passed through
L_MIN, L_MAX = -2.0, 3.5
OCC_THRESHOLD = 0.4   # one hit on an unknown cell makes it occupied
R_HARD = 0.38         # m from an occupied cell centre: impassable
R_SOFT = 0.80         # m: the cost bump fades out here
SOFT_WEIGHT = 6.0     # cost of a cell at R_HARD is 1 + SOFT_WEIGHT
R_FORCE_FREE = 0.30   # m round the robot and the goal: never impassable
PLAN_EVERY = 10       # control steps between planning steps (5 Hz at 50 Hz)
V_MAX = 0.70          # m/s
W_MAX = 1.00          # rad/s
K_HEADING = 2.0       # wz per rad of heading error
TURN_IN_PLACE = 0.90  # rad of heading error above which vx = 0
LOOKAHEAD = 0.70      # m along the path
FRONT_CONE = 0.45     # rad either side of the heading, for the speed cap
BACKUP_SECONDS = 0.6
BACKUP_SPEED = 0.25
INF = float("inf")


class _Grid:
    """Persistent log-odds occupancy grid, built only from the lidar and
    pose_estimate. Unknown cells are treated as free (the freespace
    assumption D* and D* Lite are stated under), so the first plan drives
    straight at the goal and walls are discovered on the way."""

    def __init__(self, lo, hi, range_max):
        self.ox = lo[0] - MARGIN
        self.oy = lo[1] - MARGIN
        self.nx = int(math.ceil((hi[0] - lo[0] + 2 * MARGIN) / RES))
        self.ny = int(math.ceil((hi[1] - lo[1] + 2 * MARGIN) / RES))
        self.n = self.nx * self.ny
        self.range_max = float(range_max)
        self.logodds = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.samples = np.arange(0.0, self.range_max, RES * 0.5)
        nx, ny = self.nx, self.ny
        # Neighbour table: (cell, step length, corner a, corner b). A diagonal
        # step is refused when either orthogonal corner is impassable, so the
        # path never cuts a corner of an inflated obstacle.
        self.nbrs = []
        for iy in range(ny):
            for ix in range(nx):
                row = []
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        if dx == 0 and dy == 0:
                            continue
                        jx, jy = ix + dx, iy + dy
                        if not (0 <= jx < nx and 0 <= jy < ny):
                            continue
                        if dx and dy:
                            row.append((jy * nx + jx, RES * math.sqrt(2.0),
                                        iy * nx + jx, jy * nx + ix))
                        else:
                            row.append((jy * nx + jx, RES, -1, -1))
                self.nbrs.append(tuple(row))
        r = int(math.ceil(R_SOFT / RES))
        offs = []
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                d = math.hypot(dx, dy) * RES
                if d < R_SOFT:
                    offs.append((d, dx, dy))
        self.offsets = sorted(offs)
        self.pad = r

    def cell(self, x, y):
        ix = min(self.nx - 1, max(0, int((x - self.ox) / RES)))
        iy = min(self.ny - 1, max(0, int((y - self.oy) / RES)))
        return iy * self.nx + ix

    def centre(self, c):
        return (self.ox + (c % self.nx + 0.5) * RES,
                self.oy + (c // self.nx + 0.5) * RES)

    def update(self, x, y, yaw, ranges, angles):
        r = np.asarray(ranges, dtype=float)
        a = np.asarray(angles, dtype=float) + yaw
        hit = np.isfinite(r) & (r < self.range_max)
        carve = np.where(hit, r - RES * 0.5, self.range_max)
        c, s = np.cos(a), np.sin(a)
        m = self.samples[None, :] < carve[:, None]
        px = (x + c[:, None] * self.samples[None, :])[m]
        py = (y + s[:, None] * self.samples[None, :])[m]
        free = np.zeros((self.ny, self.nx), dtype=bool)
        ix = np.floor((px - self.ox) / RES).astype(int)
        iy = np.floor((py - self.oy) / RES).astype(int)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        free[iy[ok], ix[ok]] = True
        occ = np.zeros_like(free)
        hx = np.floor((x + c[hit] * r[hit] - self.ox) / RES).astype(int)
        hy = np.floor((y + s[hit] * r[hit] - self.oy) / RES).astype(int)
        ok = (hx >= 0) & (hx < self.nx) & (hy >= 0) & (hy < self.ny)
        occ[hy[ok], hx[ok]] = True
        free &= ~occ
        self.logodds[free] += L_FREE
        self.logodds[occ] += L_OCC
        np.clip(self.logodds, L_MIN, L_MAX, out=self.logodds)

    def cost(self, keep_free):
        """Per-cell traversal cost, flat: 1 in the open, rising to
        1 + SOFT_WEIGHT at R_HARD from an occupied cell, inf inside R_HARD.
        Cells within R_FORCE_FREE of any point in `keep_free` (the robot, the
        goal) are capped at 1 + SOFT_WEIGHT: the robot is where it is."""
        occ = self.logodds > OCC_THRESHOLD
        p = self.pad
        padded = np.zeros((self.ny + 2 * p, self.nx + 2 * p), dtype=bool)
        padded[p:p + self.ny, p:p + self.nx] = occ
        dist = np.full((self.ny, self.nx), INF)
        seen = np.zeros_like(occ)
        for d, dx, dy in self.offsets:
            sh = padded[p + dy:p + dy + self.ny, p + dx:p + dx + self.nx]
            new = sh & ~seen
            dist[new] = d
            seen |= sh
        cost = np.ones((self.ny, self.nx))
        band = (dist >= R_HARD) & (dist < R_SOFT)
        cost[band] = 1.0 + SOFT_WEIGHT * (
            (R_SOFT - dist[band]) / (R_SOFT - R_HARD)) ** 2
        cost[dist < R_HARD] = INF
        gx = self.ox + (np.arange(self.nx) + 0.5) * RES
        gy = self.oy + (np.arange(self.ny) + 0.5) * RES
        for (kx, ky) in keep_free:
            near = ((gx[None, :] - kx) ** 2 + (gy[:, None] - ky) ** 2
                    <= R_FORCE_FREE ** 2)
            cost[near] = np.minimum(cost[near], 1.0 + SOFT_WEIGHT)
        return cost.ravel()

    def octile(self, a, b):
        dx = abs(a % self.nx - b % self.nx)
        dy = abs(a // self.nx - b // self.nx)
        return RES * (max(dx, dy) + (math.sqrt(2.0) - 1.0) * min(dx, dy))


def edge_cost(cost, u, v, length, ca, cb):
    """Symmetric 8-connected edge cost on a cell-cost list."""
    cu, cv = cost[u], cost[v]
    if cu == INF or cv == INF:
        return INF
    if ca >= 0 and (cost[ca] == INF or cost[cb] == INF):
        return INF
    return length * 0.5 * (cu + cv)


def path_cost(cost, grid, path):
    """Total edge cost of a cell path (for the tests and the stats)."""
    total = 0.0
    for u, v in zip(path, path[1:]):
        for (w, length, ca, cb) in grid.nbrs[u]:
            if w == v:
                total += edge_cost(cost, u, v, length, ca, cb)
                break
        else:
            return INF
    return total


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class _Base:
    """Map, planning schedule, path follower and contact recovery. The planner
    is the subclass's `_planner_for(goal_cell)`, returning an object with
    `plan(cost_np, cost_list, start_cell) -> list of cells or None`."""

    def reset(self, scenario, rng):
        start = tuple(float(v) for v in scenario.start)[:2]
        goal = tuple(float(v) for v in scenario.goal)[:2]
        lo = (min(start[0], goal[0]), min(start[1], goal[1]))
        hi = (max(start[0], goal[0]), max(start[1], goal[1]))
        self.max_lin = float(getattr(scenario, "max_lin", 0.8))
        self.max_ang = float(getattr(scenario, "max_ang", 1.0))
        self.grid = _Grid(lo, hi, getattr(scenario, "range_max", 6.0))
        self.planners = {}
        self.goal = None
        self.tick = 0
        self.path_xy = None
        self.backup_until = -1.0
        self.stats = {"plans": 0, "no_path": 0, "errors": 0,
                      "plan_ms": [], "expanded": [], "last_error": None}
        self._reset_planner()

    def _reset_planner(self):
        pass

    def step(self, obs):
        try:
            return self._step(obs)
        except Exception as exc:          # step() must never raise
            self.stats["errors"] += 1
            self.stats["last_error"] = repr(exc)
            return Command(0.0, 0.0, 0.0)

    def _plan(self, x, y, goal):
        t0 = time.perf_counter()
        cost_np = self.grid.cost(((x, y), goal))
        cost = cost_np.tolist()
        gc = self.grid.cell(*goal)
        planner = self.planners.get(gc)
        if planner is None:
            planner = self.planners[gc] = self._planner_for(gc)
        path = planner.plan(cost_np, cost, self.grid.cell(x, y))
        self.stats["plans"] += 1
        self.stats["expanded"].append(planner.last_expanded)
        self.stats["plan_ms"].append((time.perf_counter() - t0) * 1000.0)
        if not path:
            self.stats["no_path"] += 1
            self.path_xy = None
            return
        pts = [self.grid.centre(c) for c in path]
        pts[-1] = goal
        self.path_xy = np.array(pts)

    def _step(self, obs):
        x, y, yaw = (float(v) for v in obs.pose_estimate[:3])
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        self.grid.update(x, y, yaw, ranges, angles)
        goal = (float(obs.goal[0]), float(obs.goal[1]))
        if goal != self.goal:
            self.goal = goal
            self.tick = 0
        if self.tick % PLAN_EVERY == 0:
            self._plan(x, y, goal)
        self.tick += 1

        rel = np.abs(_wrap(angles))
        if obs.contact and obs.t >= self.backup_until:
            behind = ranges[rel > math.pi - 0.5]
            if behind.size == 0 or np.min(behind) > 0.35:
                self.backup_until = obs.t + BACKUP_SECONDS
        if obs.t < self.backup_until:
            return Command(-BACKUP_SPEED, 0.0, 0.0)

        if self.path_xy is None:
            return Command(0.0, 0.0, 0.5 * W_MAX)
        tx, ty = self._lookahead(x, y)
        err = _wrap(math.atan2(ty - y, tx - x) - yaw)
        wz = max(-W_MAX, min(W_MAX, K_HEADING * err))
        if abs(err) > TURN_IN_PLACE:
            return Command(0.0, 0.0, wz)
        front = ranges[rel < FRONT_CONE]
        front = float(np.min(front)) if front.size else INF
        cap = max(0.15, min(1.0, (front - 0.35) / 0.5))
        vx = V_MAX * math.cos(err) * cap
        return Command(min(vx, self.max_lin), 0.0, wz)

    def _lookahead(self, x, y):
        p = self.path_xy
        head = p[:40]
        i = int(np.argmin((head[:, 0] - x) ** 2 + (head[:, 1] - y) ** 2))
        left = LOOKAHEAD
        px, py = x, y
        for j in range(i, len(p)):
            seg = math.hypot(p[j, 0] - px, p[j, 1] - py)
            if seg >= left:
                f = left / seg
                return px + f * (p[j, 0] - px), py + f * (p[j, 1] - py)
            left -= seg
            px, py = p[j, 0], p[j, 1]
        return float(p[-1, 0]), float(p[-1, 1])
# ==== SHARED END =============================================================


class DStarLite:
    """One goal-rooted D* Lite search tree over the shared grid.

    `plan()` is the whole public surface. On the first call it initialises
    (rhs(goal) = 0, goal on the queue); on every later call it moves s_start,
    bumps k_m by h(s_last, s_start), diffs the new cell costs against the
    ones it last planned on, updates the affected vertices and runs
    ComputeShortestPath. It never clears g/rhs or the queue after init.
    """

    def __init__(self, grid, goal):
        self.grid = grid
        self.goal = goal
        n = grid.n
        self.g = [INF] * n
        self.rhs = [INF] * n
        self.heap = []
        self.queued = {}              # cell -> its live key; lazy deletion
        self.km = 0.0
        self.start = None
        self.last = None
        self.cost = None
        self.cost_np = None
        self.inits = 0
        self.last_expanded = 0
        self.last_updated = 0

    # ---- the algorithm, in the paper's names
    def key(self, s):
        # ⚠️ ROUNDED TO 1e-9 ON PURPOSE. Unrounded, two mathematically equal
        # first components can differ by one ulp (k_m accumulates octile
        # distances with sqrt(2) in them), and the termination test
        # TopKey < CalculateKey(s_start) then stops one vertex early with
        # g(s_start) consistent but too low. Found by the randomised
        # D*-Lite-vs-Dijkstra test (3 of 200 sequences) and the in-episode
        # A* shadow check, 2026-11-02.
        m = min(self.g[s], self.rhs[s])
        if m == INF:
            return (INF, INF)
        return (round(m + self.grid.octile(self.start, s) + self.km, 9),
                round(m, 9))

    def _push(self, s):
        k = self.key(s)
        self.queued[s] = k
        heapq.heappush(self.heap, (k[0], k[1], s))

    def _top(self):
        heap, queued = self.heap, self.queued
        while heap:
            k1, k2, s = heap[0]
            if queued.get(s) == (k1, k2):
                return (k1, k2), s
            heapq.heappop(heap)
        return (INF, INF), None

    def update_vertex(self, u):
        cost, g = self.cost, self.g
        if u != self.goal:
            best = INF
            for (v, length, ca, cb) in self.grid.nbrs[u]:
                gv = g[v]
                if gv == INF:
                    continue
                c = edge_cost(cost, u, v, length, ca, cb)
                if c + gv < best:
                    best = c + gv
            self.rhs[u] = best
        self.queued.pop(u, None)
        if g[u] != self.rhs[u]:
            self._push(u)

    def compute_shortest_path(self):
        g, rhs, nbrs = self.g, self.rhs, self.grid.nbrs
        s0 = self.start
        expanded = 0
        while True:
            ktop, u = self._top()
            if u is None:
                break
            if not (ktop < self.key(s0) or rhs[s0] != g[s0]):
                break
            heapq.heappop(self.heap)
            del self.queued[u]
            expanded += 1
            knew = self.key(u)
            if ktop < knew:
                self.queued[u] = knew
                heapq.heappush(self.heap, (knew[0], knew[1], u))
            elif g[u] > rhs[u]:
                g[u] = rhs[u]
                for (v, _l, _a, _b) in nbrs[u]:
                    self.update_vertex(v)
            else:
                g[u] = INF
                self.update_vertex(u)
                for (v, _l, _a, _b) in nbrs[u]:
                    self.update_vertex(v)
        self.last_expanded = expanded

    def plan(self, cost_np, cost, start):
        if self.cost is None:                     # Initialize(), ONCE
            self.inits += 1
            self.cost, self.cost_np = cost, cost_np.copy()
            self.start = self.last = start
            self.rhs[self.goal] = 0.0
            self._push(self.goal)
            self.last_updated = 0
        else:
            self.start = start
            if start != self.last:
                self.km += self.grid.octile(self.last, start)
                self.last = start
            changed = np.flatnonzero(cost_np != self.cost_np).tolist()
            self.cost, self.cost_np = cost, cost_np.copy()
            touched = set(changed)
            nbrs = self.grid.nbrs
            for c in changed:
                for (v, _l, _a, _b) in nbrs[c]:
                    touched.add(v)
            for u in touched:
                self.update_vertex(u)
            self.last_updated = len(touched)
        self.compute_shortest_path()
        return self.extract_path()

    def extract_path(self, limit=5000):
        """Greedy descent of g + c from the robot. None if no finite route."""
        s, goal, g, cost = self.start, self.goal, self.g, self.cost
        if g[s] == INF and self.rhs[s] == INF:
            return None
        path, seen = [s], {s}
        while s != goal and len(path) < limit:
            best, nxt = INF, None
            for (v, length, ca, cb) in self.grid.nbrs[s]:
                c = edge_cost(cost, s, v, length, ca, cb)
                if c + g[v] < best:
                    best, nxt = c + g[v], v
            if nxt is None or nxt in seen:
                return path if len(path) > 1 else None
            s = nxt
            seen.add(s)
            path.append(s)
        return path


class Controller(_Base):
    """Shared map + follower, D* Lite planner (one incremental tree per goal)."""

    def _planner_for(self, goal_cell):
        return DStarLite(self.grid, goal_cell)
