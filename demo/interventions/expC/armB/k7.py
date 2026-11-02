# ===========================================================================
#  Navigation controller for a Unitree Go2 shuttling between two fixed points
#  in a building it has no map of.
#
#  WHAT IS TAKEN FROM THE PAPER
#  (A. Stentz, "Optimal and Efficient Path Planning for Partially-Known
#   Environments", ICRA 1994):
#
#   * The planner IS D*, built from the pseudocode of section 2.2:
#     PROCESS-STATE lines L1-L26 and MODIFY-COST L1-L3 are implemented
#     line-for-line, including the RAISE/LOWER classification by k() vs h()
#     (sec. 2.1), the INSERT / DELETE / GET-KMIN / MIN-STATE semantics on the
#     OPEN list, and backpointers b() as the representation of the path
#     ("D* uses backpointers to represent paths to the goal", sec. 2.1).
#   * The state space follows sec. 2.3: a grid of cells, "each ... connected to
#     its eight neighbors via bidirectional arcs", arc costs "small for the
#     EMPTY cells and prohibitively large for the OBSTACLE cells", with the
#     OBSTACLE cost chosen per footnote 1 to exceed any possible sequence of
#     EMPTY cells so that one threshold on h() decides whether a path must pass
#     through an obstacle (here LETHAL=1e6, threshold BLOCKED=2e4).
#   * Unknown space is OPTIMISTIC, which is the paper's own no-a-priori-map case
#     (sec. 4 / Figure 5: "path planning in the same terrain with an optimistic
#     map (all white)"; the sensor corrects it as the robot moves). Unseen cells
#     therefore cost ~1, and that alone is what makes the robot set off straight
#     at the goal on the very first trip with nothing in the map.
#   * Motion is coupled to the planner exactly as sec. 2.2 prescribes: follow the
#     backpointer sequence {X}; when the lidar disagrees with the map, call
#     MODIFY-COST on the affected arcs, then call PROCESS-STATE until
#     k_min >= h(robot state) - the condition under which h(robot)=o(robot)
#     (Property 2, sec. 3.0) - and continue driving.
#   * The efficiency argument (sec. 2.3: after the traverse "the backpointers are
#     only partially updated ... updates needed to guarantee an optimal path for
#     the robot are limited to the vicinity of the obstacle"; sec. 5.1: "D* is
#     most efficient when these changes are detected near the current starting
#     point") is why this controller keeps ONE PERSISTENT D* PER SHUTTLE
#     ENDPOINT and seeds every arc-cost change into BOTH of them. When the
#     harness flips obs.goal, the value function for the other endpoint is not
#     rebuilt - it is repaired locally, which is the whole reason repeated
#     one-way trips get cheap and fast after the first pass.
#   * Sec. 5.2, "Using a strategy of raising costs for previously visited states,
#     D* can be extended to support exploration tasks" (cf. Pirzadeh's
#     cost-incrementing wander, sec. 1.0), is used as the trap escape: when
#     progress stalls, arc costs on the recently visited cells are raised and
#     the increase propagates as RAISE states, sweeping the backpointers out of
#     the dead end - the "potential well" of sec. 2.3 - instead of oscillating.
#   * Sec. 5.1 lists "dead-reckoning error" as a handled case in Stentz [14];
#     here pose_error_estimate down-weights (and above 1.3 m suppresses) map
#     updates, and drives a small orbit search at the goal.
#
#  NOT CARRIED ACROSS: quad-trees (sec. 5.2) - useless at this grid size; the
#  a-priori / coarse map results (Figures 4, 6) - we are given no map at all;
#  the omniscient-optimal comparison (sec. 4) - no ground truth is observable.
# ===========================================================================

import math
import time
from heapq import heappush, heappop, heapify

import numpy as np

from core.observation import Command

INF = float("inf")
EPS = 1e-9

LETHAL = 1.0e6        # arc-cost multiplier of an OBSTACLE cell (footnote 1)
LETHAL_T = 1.0e5      # cost >= this  -> cell is impassable
BLOCKED = 2.0e4       # h() >= this   -> the sequence {X} must cross an obstacle
UNK_COST = 1.06       # optimistic map: unknown is almost as cheap as EMPTY
R_LETHAL = 0.26       # robot footprint inflation (hard)
R_INFL = 0.70         # soft inflation reach
INFL_GAIN = 5.0
HIT_W = 0.90
MISS_W = 0.40
OCC_UP = 0.65
OCC_DOWN = 0.15
VY_MAX = 0.45


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class _DStar:
    """D* (Stentz 1994, sec. 2.2) over a flat 8-connected cell grid.

    The cost list is SHARED with the controller and with the other endpoint's
    planner: there is one map and several value functions over it.
    """

    __slots__ = ("n", "h", "k", "t", "b", "open", "goal", "cost", "offs",
                 "edge", "prune_limit")

    def __init__(self, n, cost, offs, edge, goal):
        self.n = n
        self.cost = cost
        self.offs = offs
        self.edge = edge
        self.goal = goal
        self.h = [INF] * n
        self.k = [INF] * n
        self.t = bytearray(n)          # 0 NEW, 1 OPEN, 2 CLOSED
        self.b = [-1] * n
        self.open = []
        self.prune_limit = max(30000, 4 * n)
        self.insert(goal, 0.0)

    # ---- OPEN list primitives (sec. 2.1) --------------------------------
    def insert(self, y, hnew):
        t = self.t
        k = self.k
        h = self.h
        ty = t[y]
        if ty == 0:
            kv = hnew
        elif ty == 1:
            ky = k[y]
            kv = ky if ky < hnew else hnew
        else:
            hy = h[y]
            kv = hy if hy < hnew else hnew
        k[y] = kv
        h[y] = hnew
        t[y] = 1
        heappush(self.open, (kv, y))

    def kmin(self):
        op = self.open
        t = self.t
        k = self.k
        while op:
            kv, x = op[0]
            if t[x] == 1 and kv <= k[x] + EPS:
                return kv
            heappop(op)
        return -1.0

    def modify_cost(self, x):
        """MODIFY-COST L2: a changed arc puts its CLOSED endpoint back on OPEN."""
        if self.t[x] == 2:
            self.insert(x, self.h[x])

    def maybe_prune(self):
        if len(self.open) > self.prune_limit:
            t = self.t
            k = self.k
            self.open = [(k[i], i) for i in range(self.n) if t[i] == 1]
            heapify(self.open)

    # ---- PROCESS-STATE, lines L1..L25 -----------------------------------
    def process_state(self):
        op = self.open
        t = self.t
        k = self.k
        h = self.h
        b = self.b
        cost = self.cost
        x = -1
        while op:                                   # L1 MIN-STATE
            kv, xx = op[0]
            if t[xx] == 1 and kv <= k[xx] + EPS:
                x = xx
                break
            heappop(op)
        if x < 0:                                   # L2
            return False
        kold = k[x]                                 # L3
        heappop(op)
        t[x] = 2
        if self.edge[x]:
            return True
        hx = h[x]
        offs = self.offs
        cx = cost[x]
        if kold < hx - EPS:                         # L4  (RAISE: try to lower h)
            for off, d in offs:                     # L5
                y = x + off
                hy = h[y]
                if hy <= kold:                      # L6
                    c = 0.5 * (cx + cost[y]) * d
                    if hx > hy + c:
                        hx = hy + c                 # L7
                        b[x] = y
            h[x] = hx
        if hx == INF:
            return True
        if hx <= kold + EPS:                        # L8  LOWER state
            for off, d in offs:                     # L9
                y = x + off
                nh = hx + 0.5 * (cx + cost[y]) * d
                ty = t[y]
                by = b[y]
                if (ty == 0                                        # L10
                        or (by == x and abs(h[y] - nh) > EPS)      # L11
                        or (by != x and h[y] > nh + EPS)):         # L12
                    b[y] = x                                       # L13
                    self.insert(y, nh)
        else:                                       # L14  RAISE state
            for off, d in offs:                     # L15
                y = x + off
                c = 0.5 * (cx + cost[y]) * d
                nh = hx + c
                ty = t[y]
                by = b[y]
                if ty == 0 or (by == x and abs(h[y] - nh) > EPS):   # L16/L17
                    b[y] = x                                       # L18
                    self.insert(y, nh)
                elif by != x and h[y] > nh + EPS:                  # L20
                    self.insert(x, hx)                             # L21
                elif (by != x and hx > h[y] + c + EPS               # L23
                        and ty == 2 and h[y] > kold):              # L24
                    self.insert(y, h[y])                           # L25
        return True

    # ---- "call PROCESS-STATE until k_min >= h(Y)" (sec. 2.2, Property 2) --
    def repair(self, start, budget, max_exp):
        t = self.t
        h = self.h
        t0 = time.perf_counter()
        n = 0
        while True:
            km = self.kmin()
            if km < 0.0:
                break
            if t[start] != 0 and km >= h[start] - EPS:
                break
            if not self.process_state():
                break
            n += 1
            if n >= max_exp:
                break
            if (n & 127) == 0 and (time.perf_counter() - t0) > budget:
                break
        self.maybe_prune()
        return t[start] != 0 and h[start] < BLOCKED


class Controller:
    # ------------------------------------------------------------------ setup
    def reset(self, scenario, rng):
        self.rng = rng
        start = self._xy(getattr(scenario, "start", (0.0, 0.0)))
        goal = self._xy(getattr(scenario, "goal", (1.0, 0.0)))
        self.rmax = float(getattr(scenario, "range_max", 6.0) or 6.0)
        self.max_lin = float(getattr(scenario, "max_lin", 0.8) or 0.8)
        self.max_ang = float(getattr(scenario, "max_ang", 1.0) or 1.0)
        self.time_limit = float(getattr(scenario, "time_limit", 300.0) or 300.0)

        # --- grid covering both shuttle endpoints with room to detour -----
        sep = math.hypot(goal[0] - start[0], goal[1] - start[1])
        margin = max(9.0, 0.9 * sep)
        xmin = min(start[0], goal[0]) - margin
        xmax = max(start[0], goal[0]) + margin
        ymin = min(start[1], goal[1]) - margin
        ymax = max(start[1], goal[1]) + margin
        res = 0.2
        while True:
            nx = int((xmax - xmin) / res) + 5
            ny = int((ymax - ymin) / res) + 5
            if nx * ny <= 70000 or res > 0.45:
                break
            res *= 1.15
        self.res = res
        self.nx = nx
        self.ny = ny
        self.n = nx * ny
        self.x0 = xmin - 2.0 * res
        self.y0 = ymin - 2.0 * res

        n = self.n
        self.score = np.zeros(n, dtype=np.float64)
        self.occ = np.zeros(n, dtype=bool)
        self.seen = np.zeros(n, dtype=bool)
        self.extra = np.zeros(n, dtype=np.float64)

        ii = np.arange(n)
        gx_i = ii % nx
        gy_i = ii // nx
        self.edge_np = ((gx_i < 1) | (gx_i >= nx - 1) |
                        (gy_i < 1) | (gy_i >= ny - 1))
        self.edge = bytearray(self.edge_np.astype(np.uint8).tolist())
        self.interior = bytearray((~self.edge_np).astype(np.uint8).tolist())

        self.cost_np = np.full((ny, nx), UNK_COST, dtype=np.float64)
        self.cost_np.reshape(-1)[self.edge_np] = LETHAL
        self.cost = self.cost_np.reshape(-1).tolist()

        d1 = res
        d2 = res * math.sqrt(2.0)
        self.offs = ((-nx - 1, d2), (-nx, d1), (-nx + 1, d2),
                     (-1, d1), (1, d1),
                     (nx - 1, d2), (nx, d1), (nx + 1, d2))

        # inflation kernel: occupied at (y+dy, x+dx) penalises (y, x)
        self.P = int(math.ceil(R_INFL / res))
        kern = []
        P = self.P
        for dy in range(-P, P + 1):
            for dx in range(-P, P + 1):
                d = math.hypot(dx, dy) * res
                if d <= R_LETHAL:
                    kern.append((dy, dx, LETHAL))
                elif d <= R_INFL:
                    pen = 1.0 + INFL_GAIN * (R_INFL - d) / max(1e-6, R_INFL - R_LETHAL)
                    kern.append((dy, dx, pen))
        self.kernel = tuple(kern)

        # --- episode state -----------------------------------------------
        self.planners = {}
        self.cur_key = None
        self.goal_xy = None
        self.trips = 0
        self.k_step = 0
        self.last_t = 0.0
        self.trail = []
        self.trail_t = -1.0
        self.stuck_ref = None
        self.stuck_count = 0
        self.escape_until = -1.0
        self.escape_dir = 0.0
        self.pen_cells = set()
        self.arrive_t = None
        self.orbit_phase = 0.0
        self.contact_t = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)

        # sampling pattern for free-space carving
        step = 0.6 * res
        self.ray_s = (np.arange(int(self.rmax / step) + 1) + 0.5) * step

    @staticmethod
    def _xy(v):
        try:
            return (float(v[0]), float(v[1]))
        except Exception:
            return (0.0, 0.0)

    # ------------------------------------------------------------ geometry
    def _cell(self, x, y):
        nx = self.nx
        ny = self.ny
        ix = int((x - self.x0) / self.res + 0.5)
        iy = int((y - self.y0) / self.res + 0.5)
        if ix < 1:
            ix = 1
        elif ix > nx - 2:
            ix = nx - 2
        if iy < 1:
            iy = 1
        elif iy > ny - 2:
            iy = ny - 2
        return iy * nx + ix

    def _cx(self, idx):
        return self.x0 + (idx % self.nx) * self.res

    def _cy(self, idx):
        return self.y0 + (idx // self.nx) * self.res

    def _los(self, p, q):
        dx = q[0] - p[0]
        dy = q[1] - p[1]
        d = math.hypot(dx, dy)
        n = int(d / (0.5 * self.res)) + 1
        res = self.res
        x0 = self.x0
        y0 = self.y0
        nx = self.nx
        ny = self.ny
        cost = self.cost
        for i in range(1, n + 1):
            f = i / n
            ix = int((p[0] + dx * f - x0) / res + 0.5)
            iy = int((p[1] + dy * f - y0) / res + 0.5)
            if ix < 1 or ix >= nx - 1 or iy < 1 or iy >= ny - 1:
                return False
            if cost[iy * nx + ix] >= LETHAL_T:
                return False
        return True

    # ------------------------------------------------------- cost map upkeep
    def _recompute_cost(self, y0, y1, x0, x1):
        ny = self.ny
        nx = self.nx
        P = self.P
        y0 = max(0, y0)
        x0 = max(0, x0)
        y1 = min(ny, y1)
        x1 = min(nx, x1)
        if y1 <= y0 or x1 <= x0:
            return None
        H = y1 - y0
        W = x1 - x0
        occ2 = self.occ.reshape(ny, nx)
        pad = np.zeros((H + 2 * P, W + 2 * P), dtype=bool)
        sy0 = max(0, y0 - P)
        sy1 = min(ny, y1 + P)
        sx0 = max(0, x0 - P)
        sx1 = min(nx, x1 + P)
        pad[sy0 - (y0 - P):sy1 - (y0 - P),
            sx0 - (x0 - P):sx1 - (x0 - P)] = occ2[sy0:sy1, sx0:sx1]

        seen2 = self.seen.reshape(ny, nx)
        new = np.where(seen2[y0:y1, x0:x1], 1.0, UNK_COST)
        for (dy, dx, pen) in self.kernel:
            sub = pad[P + dy:P + dy + H, P + dx:P + dx + W]
            if sub.any():
                np.maximum(new, np.where(sub, pen, 0.0), out=new)
        new += self.extra.reshape(ny, nx)[y0:y1, x0:x1]
        ed = self.edge_np.reshape(ny, nx)[y0:y1, x0:x1]
        new[ed] = LETHAL

        cur = self.cost_np[y0:y1, x0:x1]
        ch = np.abs(new - cur) > 1e-9
        if not ch.any():
            return None
        cur[ch] = new[ch]
        yy, xx = np.nonzero(ch)
        flat = (yy + y0) * nx + (xx + x0)
        vals = new[ch]
        cl = self.cost
        for i, vv in zip(flat.tolist(), vals.tolist()):
            cl[i] = vv
        return flat

    def _apply_changes(self, flat):
        """MODIFY-COST for every arc whose cost moved, in every value function."""
        if flat is None or len(flat) == 0 or not self.planners:
            return
        interior = self.interior
        offs = self.offs
        aff = set()
        for i in flat.tolist():
            aff.add(i)
            if interior[i]:
                for off, _d in offs:
                    aff.add(i + off)
        for p in self.planners.values():
            t = p.t
            h = p.h
            for i in aff:
                if t[i] == 2:
                    p.insert(i, h[i])

    # -------------------------------------------------------------- sensing
    def _sense(self, obs, x, y, yaw, pe):
        try:
            r = np.asarray(obs.ranges, dtype=np.float64).ravel()
            a = np.asarray(obs.range_angles, dtype=np.float64).ravel()
        except Exception:
            return
        if r.size == 0 or r.size != a.size:
            return
        if pe > 1.3:
            w = 0.0
        elif pe <= 0.25:
            w = 1.0
        else:
            w = max(0.2, 1.0 - (pe - 0.25) / 1.05)

        res = self.res
        nx = self.nx
        ny = self.ny
        x0 = self.x0
        y0 = self.y0

        finite = np.isfinite(r)
        rc = np.where(finite, r, self.rmax)
        np.clip(rc, 0.0, self.rmax, out=rc)
        hit = finite & (r < self.rmax - 0.08) & (r > 0.05)
        th = a + yaw
        cs = np.cos(th)
        sn = np.sin(th)

        idx_list = []
        if w > 0.0:
            free_len = np.maximum(np.where(hit, rc - 0.55 * res, rc), 0.0)
            s = self.ray_s
            ix = ((x + np.outer(cs, s) - x0) / res + 0.5).astype(np.int64)
            iy = ((y + np.outer(sn, s) - y0) / res + 0.5).astype(np.int64)
            ok = (s[None, :] <= free_len[:, None]) & \
                 (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            midx = np.unique((iy * nx + ix)[ok])

            hx = x + cs[hit] * rc[hit]
            hy = y + sn[hit] * rc[hit]
            hix = ((hx - x0) / res + 0.5).astype(np.int64)
            hiy = ((hy - y0) / res + 0.5).astype(np.int64)
            hok = (hix >= 0) & (hix < nx) & (hiy >= 0) & (hiy < ny)
            hidx = np.unique((hiy * nx + hix)[hok])
        else:
            midx = np.empty(0, dtype=np.int64)
            hidx = np.empty(0, dtype=np.int64)

        # the robot is standing here, so here is traversable: never let the
        # map lock us in (also repairs phantoms created by pose drift)
        rc0 = self._cell(x, y)
        rfree = np.array([rc0, rc0 - 1, rc0 + 1, rc0 - nx, rc0 + nx],
                         dtype=np.int64)
        rfree = rfree[(rfree >= 0) & (rfree < self.n)]

        score = self.score
        if midx.size:
            score[midx] -= MISS_W * w
        if hidx.size:
            score[hidx] += HIT_W * w
        score[rfree] = np.minimum(score[rfree], -1.0)

        touched = np.unique(np.concatenate((midx, hidx, rfree)))
        np.clip(score[touched], -3.0, 3.0, out=score[touched]) \
            if False else None
        score[touched] = np.clip(score[touched], -3.0, 3.0)
        self.seen[touched] = True

        old = self.occ[touched]
        st = score[touched]
        new = np.where(old, st > OCC_DOWN, st > OCC_UP)
        ch = new != old
        if not ch.any():
            return
        self.occ[touched] = new
        cidx = touched[ch]
        cy = cidx // nx
        cx = cidx % nx
        P = self.P
        flat = self._recompute_cost(int(cy.min()) - P, int(cy.max()) + P + 1,
                                    int(cx.min()) - P, int(cx.max()) + P + 1)
        self._apply_changes(flat)

    # ------------------------------------------- raising visited-state costs
    def _penalize(self, points, radius, amount):
        res = self.res
        nx = self.nx
        ny = self.ny
        r = max(1, int(radius / res))
        cells = set()
        for (px, py) in points:
            cix = int((px - self.x0) / res + 0.5)
            ciy = int((py - self.y0) / res + 0.5)
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if dx * dx + dy * dy > r * r:
                        continue
                    jx = cix + dx
                    jy = ciy + dy
                    if 1 <= jx < nx - 1 and 1 <= jy < ny - 1:
                        cells.add(jy * nx + jx)
        if not cells:
            return
        idx = np.fromiter(cells, dtype=np.int64, count=len(cells))
        self.extra[idx] = np.minimum(self.extra[idx] + amount, 60.0)
        self.pen_cells |= cells
        iy = idx // nx
        ix = idx % nx
        flat = self._recompute_cost(int(iy.min()) - 1, int(iy.max()) + 2,
                                    int(ix.min()) - 1, int(ix.max()) + 2)
        self._apply_changes(flat)

    def _clear_penalties(self):
        if not self.pen_cells:
            return
        cells = self.pen_cells
        idx = np.fromiter(cells, dtype=np.int64, count=len(cells))
        self.extra[idx] = 0.0
        self.pen_cells = set()
        nx = self.nx
        iy = idx // nx
        ix = idx % nx
        flat = self._recompute_cost(int(iy.min()) - 1, int(iy.max()) + 2,
                                    int(ix.min()) - 1, int(ix.max()) + 2)
        self._apply_changes(flat)

    # -------------------------------------------------------------- planning
    def _free_goal_cell(self, gx, gy):
        idx = self._cell(gx, gy)
        if self.cost[idx] < LETHAL_T:
            return idx
        res = self.res
        nx = self.nx
        ny = self.ny
        cix = idx % nx
        ciy = idx // nx
        R = int(1.2 / res) + 2
        best = idx
        bestd = 1e18
        cost = self.cost
        for dy in range(-R, R + 1):
            jy = ciy + dy
            if jy < 1 or jy >= ny - 1:
                continue
            for dx in range(-R, R + 1):
                jx = cix + dx
                if jx < 1 or jx >= nx - 1:
                    continue
                j = jy * nx + jx
                if cost[j] >= LETHAL_T:
                    continue
                d = dx * dx + dy * dy
                if d < bestd:
                    bestd = d
                    best = j
        return best

    def _start_cell(self, x, y):
        idx = self._cell(x, y)
        cost = self.cost
        if cost[idx] < LETHAL_T:
            return idx
        nx = self.nx
        ny = self.ny
        cix = idx % nx
        ciy = idx // nx
        best = idx
        bestd = 1e18
        for dy in range(-6, 7):
            jy = ciy + dy
            if jy < 1 or jy >= ny - 1:
                continue
            for dx in range(-6, 7):
                jx = cix + dx
                if jx < 1 or jx >= nx - 1:
                    continue
                j = jy * nx + jx
                if cost[j] >= LETHAL_T:
                    continue
                d = dx * dx + dy * dy
                if d < bestd:
                    bestd = d
                    best = j
        return best

    def _path(self, p, start):
        """Follow the backpointer sequence {X} (sec. 2.1)."""
        pts = []
        b = p.b
        goal = p.goal
        idx = start
        seen = set()
        for _ in range(700):
            if idx < 0 or idx in seen:
                break
            seen.add(idx)
            pts.append(idx)
            if idx == goal:
                break
            idx = b[idx]
        return pts

    def _carrot(self, x, y, pts, gx, gy):
        if math.hypot(gx - x, gy - y) < 3.2 and self._los((x, y), (gx, gy)):
            return (gx, gy)
        if not pts:
            return (gx, gy)
        best = (self._cx(pts[0]), self._cy(pts[0]))
        acc = 0.0
        prev = (x, y)
        cand = []
        np_ = len(pts)
        for i, idx in enumerate(pts):
            pt = (self._cx(idx), self._cy(idx))
            acc += math.hypot(pt[0] - prev[0], pt[1] - prev[1])
            prev = pt
            if (i % 3) == 0 or i == np_ - 1:
                cand.append(pt)
            if acc > 4.0:
                break
        for pt in cand:
            if math.hypot(pt[0] - x, pt[1] - y) <= 3.0 and self._los((x, y), pt):
                best = pt
        return best

    # ------------------------------------------------------------- main step
    def step(self, obs):
        try:
            cmd = self._drive(obs)
            return cmd
        except Exception:
            try:
                return self._panic(obs)
            except Exception:
                return Command(0.0, 0.0, 0.0)

    def _panic(self, obs):
        p = obs.pose_estimate
        g = obs.goal
        ang = _wrap(math.atan2(float(g[1]) - float(p[1]),
                               float(g[0]) - float(p[0])) - float(p[2]))
        wz = max(-self.max_ang, min(self.max_ang, 1.5 * ang))
        return Command(0.15 * math.cos(ang), 0.0, wz)

    def _drive(self, obs):
        self.k_step += 1
        t = float(getattr(obs, "t", self.last_t + 0.02))
        dt = t - self.last_t
        if dt <= 0.0 or dt > 0.5:
            dt = 0.02
        self.last_t = t

        pose = obs.pose_estimate
        x = float(pose[0])
        y = float(pose[1])
        yaw = float(pose[2])
        pe = float(getattr(obs, "pose_error_estimate", 0.0) or 0.0)
        gx, gy = self._xy(obs.goal)
        contact = bool(getattr(obs, "contact", False))

        r = np.asarray(obs.ranges, dtype=np.float64).ravel()
        a = np.asarray(obs.range_angles, dtype=np.float64).ravel()
        if r.size != a.size or r.size == 0:
            r = np.array([self.rmax])
            a = np.array([0.0])
        rf = np.where(np.isfinite(r), r, self.rmax)

        # ---- goal flip: switch value function, do not rebuild it ----------
        if self.goal_xy is None or math.hypot(gx - self.goal_xy[0],
                                              gy - self.goal_xy[1]) > 0.5:
            if self.goal_xy is not None:
                self.trips += 1
            self.goal_xy = (gx, gy)
            self.cur_key = (round(gx * 2.0) / 2.0, round(gy * 2.0) / 2.0)
            self._clear_penalties()
            self.arrive_t = None
            self.stuck_ref = (t, x, y)
            self.stuck_count = 0
            self.escape_until = -1.0
            self.trail = []
            self.orbit_phase = float(self.rng.uniform(0.0, 2.0 * math.pi))

        # ---- sensing / MODIFY-COST ---------------------------------------
        if (self.k_step % 2) == 0 or self.k_step < 6:
            self._sense(obs, x, y, yaw, pe)

        # ---- the two persistent D* instances ------------------------------
        p = self.planners.get(self.cur_key)
        if p is None:
            gidx = self._free_goal_cell(gx, gy)
            p = _DStar(self.n, self.cost, self.offs, self.edge, gidx)
            self.planners[self.cur_key] = p
        elif (self.k_step % 37) == 0 and self.cost[p.goal] >= LETHAL_T:
            gidx = self._free_goal_cell(gx, gy)
            if gidx != p.goal and self.cost[gidx] < LETHAL_T:
                p = _DStar(self.n, self.cost, self.offs, self.edge, gidx)
                self.planners[self.cur_key] = p

        sidx = self._start_cell(x, y)
        have = p.t[sidx] != 0 and p.h[sidx] < BLOCKED
        budget = 0.010 if have else 0.040
        have = p.repair(sidx, budget, 90000)

        # ---- trail bookkeeping -------------------------------------------
        if t - self.trail_t > 0.4:
            self.trail_t = t
            self.trail.append((x, y))
            if len(self.trail) > 12:
                self.trail.pop(0)

        dg = math.hypot(gx - x, gy - y)

        # ---- stall detection -> raise costs of visited states (sec. 5.2) --
        if self.stuck_ref is None:
            self.stuck_ref = (t, x, y)
        st, sx, sy = self.stuck_ref
        if t - st > 3.5:
            if math.hypot(x - sx, y - sy) < 0.35 and dg > 0.7:
                self.stuck_count += 1
                self._penalize(self.trail[-6:] or [(x, y)], 0.45,
                               2.0 + 1.5 * min(self.stuck_count, 4))
                # escape heading: most open sector, mildly biased to the goal
                gdir = _wrap(math.atan2(gy - y, gx - x) - yaw)
                bestsc = -1e9
                bestang = gdir
                for i in range(16):
                    ang = -math.pi + i * (2.0 * math.pi / 16.0)
                    sel = np.abs(_wrap_arr(a - ang)) < 0.45
                    cl = float(rf[sel].min()) if bool(sel.any()) else self.rmax
                    sc = min(cl, 3.0) + 0.6 * math.cos(_wrap(ang - gdir))
                    if sc > bestsc:
                        bestsc = sc
                        bestang = ang
                self.escape_dir = yaw + bestang
                self.escape_until = t + 1.4 + 0.3 * min(self.stuck_count, 4)
            else:
                self.stuck_count = 0
            self.stuck_ref = (t, x, y)

        if contact:
            self.contact_t += dt
            if self.contact_t > 0.7 and t > self.escape_until:
                gdir = _wrap(math.atan2(gy - y, gx - x) - yaw)
                self.escape_dir = yaw + math.pi
                self.escape_until = t + 0.8
                self.contact_t = 0.0
        else:
            self.contact_t = max(0.0, self.contact_t - 0.5 * dt)

        # ---- pick the carrot ---------------------------------------------
        if t < self.escape_until:
            carrot = (x + 1.6 * math.cos(self.escape_dir),
                      y + 1.6 * math.sin(self.escape_dir))
        else:
            if have:
                pts = self._path(p, sidx)
                carrot = self._carrot(x, y, pts, gx, gy)
            else:
                carrot = (gx, gy)

            # goal reached in our own estimate but the harness has not
            # switched: pose error, so sweep a growing circle around it
            if dg < 0.45:
                if self.arrive_t is None:
                    self.arrive_t = t
            elif dg > 1.1:
                self.arrive_t = None
            if self.arrive_t is not None and t - self.arrive_t > 2.0:
                lim = max(0.45, min(1.1, 1.2 * pe + 0.35))
                rad = min(lim, 0.25 + 0.12 * (t - self.arrive_t - 2.0))
                self.orbit_phase += 1.2 * dt
                tgt = (gx + rad * math.cos(self.orbit_phase),
                       gy + rad * math.sin(self.orbit_phase))
                if self._los((x, y), tgt):
                    carrot = tgt

        # ---- velocity ----------------------------------------------------
        dx = carrot[0] - x
        dy = carrot[1] - y
        dd = math.hypot(dx, dy)
        ang = _wrap(math.atan2(dy, dx) - yaw) if dd > 1e-6 else 0.0

        v = self.max_lin
        v = min(v, 0.28 + 0.9 * dd)
        if self.arrive_t is not None:
            v = min(v, 0.32)
        elif dg < 0.9:
            v = min(v, 0.45)
        if 0.9 < abs(ang) < 2.2:
            v *= 0.70

        vx = v * math.cos(ang)
        vy = v * math.sin(ang)
        if vy > VY_MAX:
            vy = VY_MAX
        elif vy < -VY_MAX:
            vy = -VY_MAX
        wz = 2.0 * ang

        # clearance along the actual direction of travel
        if abs(vx) > 1e-6 or abs(vy) > 1e-6:
            mdir = math.atan2(vy, vx)
            sel = np.abs(_wrap_arr(a - mdir)) < 0.62
            dmin = float(rf[sel].min()) if bool(sel.any()) else self.rmax
            sc = (dmin - 0.30) / 0.55
            if sc < 1.0:
                sc = 0.0 if sc < 0.0 else sc
                vx *= sc
                vy *= sc

        # local repulsion (keeps us off walls the grid rounds away)
        cl = rf < 0.85
        if bool(cl.any()):
            wgt = (0.85 - rf[cl]) / 0.85
            wgt = wgt * wgt
            rx = -float(np.sum(wgt * np.cos(a[cl])))
            ry = -float(np.sum(wgt * np.sin(a[cl])))
            m = math.hypot(rx, ry)
            if m > 1e-6:
                mag = min(0.30, 0.55 * float(wgt.max()))
                if contact:
                    mag = 0.40
                vx += mag * rx / m
                vy += mag * ry / m

        if contact:
            vx *= 0.5
            vy *= 0.5

        ml = self.max_lin
        vx = ml if vx > ml else (-ml if vx < -ml else vx)
        vlim = min(ml, 0.55)
        vy = vlim if vy > vlim else (-vlim if vy < -vlim else vy)
        wz = self.max_ang if wz > self.max_ang else (
            -self.max_ang if wz < -self.max_ang else wz)

        self.last_cmd = (vx, vy, wz)
        return Command(vx, vy, wz)


def _wrap_arr(v):
    return (v + math.pi) % (2.0 * math.pi) - math.pi


# ==== EXP C INTERVENTION: NOT MODEL-WRITTEN ================================
# Appended by demo/expC.py to a byte-identical copy of demo/arms/armB/k7.py
# (sha256 70ca68a952daff956aab772adbdfab4bf3e49db2b7d90640d6fdc099c70c8e5a).
# Everything ABOVE this banner is the model's file, unchanged.
# Everything below is ours and identical in every patched copy except the body
# of _expc_full_replan. This file is NOT an armB sample and never enters an arm
# count. See demo/expC.py and prereg/expC.md.
import math as _expc_math
import time as _expc_time

_EXPC_PROGRESS_M = 0.5
_EXPC_WINDOW_S = 10.0
_EXPC_GOAL_SWITCH_M = 0.5


class _ExpCNoProgress:
    """True when the straight-line distance to the current target has not
    reached a new best, lower by _EXPC_PROGRESS_M, for _EXPC_WINDOW_S sim
    seconds. Restarts on a new target and after every firing."""

    def __init__(self):
        self.goal = None
        self.best = None
        self.t_best = None

    def update(self, t, x, y, gx, gy):
        d = _expc_math.hypot(gx - x, gy - y)
        if (self.goal is None or _expc_math.hypot(
                gx - self.goal[0], gy - self.goal[1]) > _EXPC_GOAL_SWITCH_M):
            self.goal, self.best, self.t_best = (gx, gy), d, t
            return False
        if d <= self.best - _EXPC_PROGRESS_M:
            self.best, self.t_best = d, t
            return False
        if t - self.t_best >= _EXPC_WINDOW_S:
            self.best, self.t_best = d, t
            return True
        return False


_ExpCModelController = Controller


class Controller(_ExpCModelController):
    """The model's controller plus the full-replan fallback. Nothing else."""

    def reset(self, scenario, rng):
        super().reset(scenario, rng)
        self._expc_monitor = _ExpCNoProgress()
        self.expc_fallback_fires = []
        self.expc_errors = []

    def step(self, obs):
        fired = False
        try:
            p, g = obs.pose_estimate, obs.goal
            fired = self._expc_monitor.update(
                float(obs.t), float(p[0]), float(p[1]), float(g[0]), float(g[1]))
        except Exception as exc:
            self.expc_errors.append(repr(exc)[:160])
        if not fired:
            return super().step(obs)
        t0 = _expc_time.perf_counter()
        try:
            self._expc_full_replan()
        except Exception as exc:
            self.expc_errors.append(repr(exc)[:160])
        cmd = super().step(obs)
        self.expc_fallback_fires.append({
            "t": round(float(obs.t), 2),
            "goal": [round(float(obs.goal[0]), 2), round(float(obs.goal[1]), 2)],
            "dist": round(self._expc_monitor.best, 3),
            "step_ms": round((_expc_time.perf_counter() - t0) * 1000.0, 2)})
        return cmd

    def _expc_full_replan(self):
        # Drop the cached route; keep the map. Per-controller (expC.HOOKS).
        self.planners = {}
