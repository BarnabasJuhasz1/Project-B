# =============================================================================
#  Navigation controller for a Unitree Go2 shuttling between two fixed points
#  inside a building for which no map exists.
#
#  WHAT WAS TAKEN FROM THE PAPER
#  (A. Stentz, "Optimal and Efficient Path Planning for Partially-Known
#   Environments", ICRA 1994):
#
#   * The D* algorithm itself.  `_DStar.process_state` is a direct
#     transcription of PROCESS-STATE (paper Sec. 2.2, lines L1-L26) including
#     the RAISE branch (L4-L7), the LOWER branch (L8-L13), the descendant /
#     postponed-update branch (L14-L25) and the INSERT semantics of Sec. 2.1
#     (k = h_new if NEW, min(k,h_new) if OPEN, min(h,h_new) if CLOSED).
#     `_DStar.cell_cost_changed` is MODIFY-COST (Sec. 2.2, L1-L3): edit the arc
#     cost, then re-INSERT the affected CLOSED states so the change propagates.
#   * The search is rooted at the goal and the robot drives by following the
#     backpointer sequence {X} (Sec. 2.1, Property 1).
#   * The stopping rule of Sec. 2.2: call PROCESS-STATE until the returned
#     k_min >= h(Y) for the robot's state Y, at which point h(Y) = o(Y).
#   * Footnote 1 on page 4: OBSTACLE arc cost is chosen larger than the longest
#     possible sequence of EMPTY cells, so a *threshold* on the path cost tells
#     us whether the only remaining route runs through an obstacle.  The planner
#     therefore never answers "no path"; it answers "the cheapest breach is
#     here", which is exactly what you want when the map is probably wrong.
#   * Sec. 4.0 / Fig. 5, the "optimistic map": unexplored space is assumed
#     EMPTY and is corrected by the sensor as it is observed.  Unknown cells
#     cost 1.0 here, the same as observed free space.
#   * Fig. 6, coarse-resolution maps: a coarse planning lattice cost only ~6%
#     in path quality, so D* runs on a deliberately coarse grid (~0.25 m) and a
#     reactive layer does the fine dodging that the lattice cannot express.
#   * Sec. 5.1: "D* is most efficient when these changes are detected near the
#     current starting point" - the map is only edited inside the lidar
#     footprint and those edits are fed straight into MODIFY-COST.
#   * Sec. 5.2: "Using a strategy of raising costs for previously visited
#     states, D* can be extended to support exploration tasks" (after Pirzadeh
#     [9]).  That is the escape behaviour: when the robot livelocks, the cost of
#     the cells it is sitting in is raised, MODIFY-COST is called, and D* routes
#     it somewhere else.  The raise decays so the shuttle can reuse the corridor.
#   * Table 1 / Sec. 4.0: D*'s whole advantage over the brute-force replanner is
#     that the goal-rooted tree survives.  The task shuttles between exactly two
#     points, so one persistent D* tree is kept *per goal*; trips 2..N reuse
#     both the learned map and an already-built backpointer field.
#   * Sec. 2.3 / Fig. 1: the robot in the paper's illustration discovers
#     obstacles with a *contact* sensor and enters them on the OPEN list.  The
#     contact flag is used the same way here.
#
#  NOT taken across: quad-trees (Sec. 5.2) - useless at this scale; and the
#  focussed heuristic of the replanner baseline (Sec. 4.0), because plain D*
#  already converges inside the per-step time budget on a coarse lattice.
# =============================================================================

import heapq
import math
import time

import numpy as np

from core.observation import Command


INF = float("inf")

# Arc-cost constants.  LETHAL is the paper's OBSTACLE cost: far larger than the
# longest sequence of EMPTY cells the lattice can hold, so BLOCKED_THRESH is a
# valid threshold for "this route must pass through an obstacle" (footnote 1).
LETHAL = 8000.0
BLOCKED_THRESH = 4000.0

LETHAL_R = 0.33          # metres: inside this the cell is untraversable
INFL_OUTER = 0.85        # metres: soft inflation reaches this far
INFL_W = 14.0            # peak soft penalty just outside the lethal radius

LO_OCC = 0.85            # log-odds increment for a lidar return
LO_FREE = 0.30           # log-odds decrement for carved free space
LO_HI = 6.0
LO_LO = -2.5
OCC_ON = 1.2             # hysteresis: log-odds above this -> OBSTACLE
OCC_OFF = 0.0            # log-odds below this -> EMPTY

FRUST_AMOUNT = 22.0      # Sec. 5.2 cost raise on visited states when stuck
FRUST_TTL = 28.0         # seconds before the raise is undone
FRUST_R = 0.75           # metres

SQ2 = math.sqrt(2.0)


def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clip(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


# =============================================================================
#  D*  (Stentz 1994, Section 2)
# =============================================================================
class _DStar(object):
    """One goal-rooted D* search tree over a flat 8-connected cell lattice.

    `cost` is a shared python list of per-cell traversal costs; the arc cost
    c(X, Y) is the mean of the two cell costs times the geometric length of the
    arc.  `blocked` marks the outer ring of the lattice so neighbour arithmetic
    on flat indices never leaves the array.
    """

    NEW = 0
    OPEN = 1
    CLOSED = 2

    def __init__(self, n, goal_idx, cost, blocked, nbr_off, nbr_dist):
        self.n = n
        self.goal = goal_idx
        self.cost = cost
        self.blocked = blocked
        self.nbr_off = nbr_off
        self.nbr_dist = nbr_dist
        self.h = None
        self.k = None
        self.b = None
        self.t = None
        self.open = None
        self.reset_tree()

    # ---- bookkeeping --------------------------------------------------
    def reset_tree(self):
        n = self.n
        self.h = [INF] * n
        self.k = [INF] * n
        self.b = [-1] * n
        self.t = bytearray(n)
        self.open = []
        g = self.goal
        self.h[g] = 0.0
        self.k[g] = 0.0
        self.t[g] = 1
        heapq.heappush(self.open, (0.0, g))

    # ---- INSERT (Sec. 2.1) -------------------------------------------
    def insert(self, x, hnew):
        tx = self.t[x]
        if tx == 0:
            kn = hnew
        elif tx == 1:
            kx = self.k[x]
            kn = kx if kx < hnew else hnew
        else:
            hx = self.h[x]
            kn = hx if hx < hnew else hnew
        self.h[x] = hnew
        self.k[x] = kn
        self.t[x] = 1
        heapq.heappush(self.open, (kn, x))

    # ---- GET-KMIN (with lazy deletion of stale heap entries) ----------
    def get_kmin(self):
        heap = self.open
        t = self.t
        k = self.k
        while heap:
            kv, x = heap[0]
            if t[x] != 1 or k[x] != kv:
                heapq.heappop(heap)
                continue
            return kv
        return -1.0

    # ---- PROCESS-STATE (Sec. 2.2, L1-L26) -----------------------------
    def process_state(self):
        heap = self.open
        h = self.h
        kk = self.k
        b = self.b
        t = self.t
        cost = self.cost
        blocked = self.blocked
        offs = self.nbr_off
        dists = self.nbr_dist

        # L1-L3: MIN-STATE, GET-KMIN, DELETE
        x = -1
        while heap:
            kv, cand = heapq.heappop(heap)
            if t[cand] != 1 or kk[cand] != kv:
                continue
            x = cand
            k_old = kv
            break
        if x < 0:
            return -1.0
        t[x] = 2
        hx = h[x]
        cx = cost[x]

        # L4-L7: RAISE state - try to lower h(X) from an optimal neighbour.
        if k_old < hx - 1e-9:
            for j in range(8):
                y = x + offs[j]
                if blocked[y]:
                    continue
                hy = h[y]
                if hy <= k_old:
                    cyx = 0.5 * (cost[y] + cx) * dists[j]
                    if hx > hy + cyx:
                        b[x] = y
                        hx = hy + cyx
            h[x] = hx

        if hx <= k_old + 1e-9:
            # L8-L13: LOWER state - push reductions to every neighbour.
            for j in range(8):
                y = x + offs[j]
                if blocked[y]:
                    continue
                newh = hx + 0.5 * (cx + cost[y]) * dists[j]
                ty = t[y]
                if (ty == 0
                        or (b[y] == x and abs(h[y] - newh) > 1e-7)
                        or (b[y] != x and h[y] > newh + 1e-7)):
                    b[y] = x
                    self.insert(y, newh)
        else:
            # L14-L25: still a RAISE state.
            reopened = False
            for j in range(8):
                y = x + offs[j]
                if blocked[y]:
                    continue
                cxy = 0.5 * (cx + cost[y]) * dists[j]
                newh = hx + cxy
                ty = t[y]
                if ty == 0 or (b[y] == x and abs(h[y] - newh) > 1e-7):
                    b[y] = x
                    self.insert(y, newh)
                else:
                    if b[y] != x and h[y] > newh + 1e-7:
                        # L20-L21: put X back to avoid a backpointer loop.
                        if not reopened:
                            reopened = True
                            self.insert(x, hx)
                    else:
                        # L23-L25: postpone until the neighbour is optimal.
                        if (b[y] != x and hx > h[y] + cxy + 1e-7
                                and ty == 2 and h[y] > k_old):
                            self.insert(y, h[y])
        return self.get_kmin()

    # ---- MODIFY-COST (Sec. 2.2, L1-L3) --------------------------------
    def cell_cost_changed(self, cells):
        """The cell costs (hence all incident arc costs) changed: re-open the
        affected CLOSED states so D* propagates the change."""
        t = self.t
        h = self.h
        for c in cells:
            if t[c] == 2:
                self.insert(c, h[c])

    # ---- follow the backpointer sequence {X} (Property 1) -------------
    def trace(self, start, max_len):
        pts = []
        seen = set()
        x = start
        t = self.t
        b = self.b
        g = self.goal
        ok = True
        for _ in range(max_len):
            if x in seen:
                ok = False
                break
            seen.add(x)
            pts.append(x)
            if x == g:
                break
            if t[x] == 0:
                ok = False
                break
            y = b[x]
            if y < 0:
                break
            x = y
        return pts, ok


# =============================================================================
#  Controller
# =============================================================================
class Controller(object):

    # ------------------------------------------------------------------
    def reset(self, scenario, rng):
        self.rng = rng

        self.max_lin = float(getattr(scenario, "max_lin", 0.8) or 0.8)
        self.max_ang = float(getattr(scenario, "max_ang", 1.0) or 1.0)
        self.range_max = float(getattr(scenario, "range_max", 6.0) or 6.0)
        self.time_limit = float(getattr(scenario, "time_limit", 1.0e9) or 1.0e9)

        start = self._pt(getattr(scenario, "start", (0.0, 0.0)))
        goal = self._pt(getattr(scenario, "goal", (0.0, 0.0)))

        # ---- lattice (deliberately coarse; paper Fig. 6) --------------
        margin = 24.0
        xmin = min(start[0], goal[0]) - margin
        xmax = max(start[0], goal[0]) + margin
        ymin = min(start[1], goal[1]) - margin
        ymax = max(start[1], goal[1]) + margin
        spanx = max(xmax - xmin, 4.0)
        spany = max(ymax - ymin, 4.0)
        nmax = 420.0
        res = max(0.25, spanx / nmax, spany / nmax)
        self.res = res
        self.inv_res = 1.0 / res
        self.nx = int(spanx / res) + 3
        self.ny = int(spany / res) + 3
        self.ox = xmin - res
        self.oy = ymin - res
        nx = self.nx
        ny = self.ny
        n = nx * ny
        self.n = n

        # ---- map layers ----------------------------------------------
        self.lo = np.zeros((ny, nx), dtype=np.float32)      # log-odds
        self.lo_flat = self.lo.reshape(-1)
        self.occ = np.zeros((ny, nx), dtype=np.float32)     # binary OBSTACLE
        self.frust = np.zeros((ny, nx), dtype=np.float32)   # Sec. 5.2 raises
        self.cost2d = np.ones((ny, nx), dtype=np.float32)   # optimistic: 1.0
        self.cost2d[0, :] = LETHAL
        self.cost2d[-1, :] = LETHAL
        self.cost2d[:, 0] = LETHAL
        self.cost2d[:, -1] = LETHAL
        self.cost_flat = self.cost2d.reshape(-1)
        self.costl = self.cost_flat.tolist()

        blocked = bytearray(n)
        for ix in range(nx):
            blocked[ix] = 1
            blocked[(ny - 1) * nx + ix] = 1
        for iy in range(ny):
            blocked[iy * nx] = 1
            blocked[iy * nx + nx - 1] = 1
        self.blocked = blocked

        self.nbr_off = [-nx - 1, -nx, -nx + 1, -1, 1, nx - 1, nx, nx + 1]
        self.nbr_dist = [SQ2 * res, res, SQ2 * res, res,
                         res, SQ2 * res, res, SQ2 * res]

        # ---- inflation kernel ----------------------------------------
        R = int(math.ceil(INFL_OUTER / res))
        if R < 1:
            R = 1
        self.R = R
        kern = []
        for dy in range(-R, R + 1):
            for dx in range(-R, R + 1):
                d = math.hypot(dx, dy) * res
                if d <= LETHAL_R:
                    p = LETHAL
                elif d >= INFL_OUTER:
                    continue
                else:
                    u = (INFL_OUTER - d) / (INFL_OUTER - LETHAL_R)
                    p = 1.0 + INFL_W * u * u
                kern.append((dy, dx, float(p)))
        self.kernel = kern

        # ---- planners: one persistent D* tree per goal ----------------
        self.planners = {}
        self.active_key = None
        self.dirty = set()
        self.last_hard_reset = -100.0

        # ---- lidar caches --------------------------------------------
        self.order = None
        self.sang = None
        self.nbeam = 0

        # ---- episode state -------------------------------------------
        self.step_i = 0
        self.t_prev = 0.0
        self.goal_xy = (goal[0], goal[1])
        self.hist = []
        self.escape_until = -1.0
        self.spin_dir = 1.0
        self.frust_events = []
        self.last_cmd = (0.0, 0.0, 0.0)
        self.path_world = []
        self.blocked_route = False
        self.have_plan = False
        self.trip_count = 0
        self.fail_count = 0

    # ------------------------------------------------------------------
    @staticmethod
    def _pt(v):
        try:
            a = np.asarray(v, dtype=float).ravel()
            return (float(a[0]), float(a[1]))
        except Exception:
            return (0.0, 0.0)

    def _cell(self, x, y):
        ix = int((x - self.ox) * self.inv_res)
        iy = int((y - self.oy) * self.inv_res)
        if ix < 1:
            ix = 1
        elif ix > self.nx - 2:
            ix = self.nx - 2
        if iy < 1:
            iy = 1
        elif iy > self.ny - 2:
            iy = self.ny - 2
        return iy * self.nx + ix

    def _center(self, idx):
        iy = idx // self.nx
        ix = idx - iy * self.nx
        return (self.ox + (ix + 0.5) * self.res,
                self.oy + (iy + 0.5) * self.res)

    # ==================================================================
    #  Map maintenance
    # ==================================================================
    def _prep_beams(self, angles):
        a = np.asarray(angles, dtype=np.float64).ravel()
        w = (a + math.pi) % (2.0 * math.pi) - math.pi
        self.order = np.argsort(w)
        self.sang = w[self.order]
        self.nbeam = self.sang.size
        step = self.res * 0.6
        ns = int(self.range_max / step) + 1
        self.samples = (np.arange(1, ns + 1, dtype=np.float64) * step)

    def _update_map(self, rng_s, x, y, yaw, pose_err):
        """Carve free space, stamp returns, then recompute the affected part of
        the cost field and hand the deltas to MODIFY-COST."""
        nx = self.nx
        ny = self.ny
        res = self.res
        rmax = self.range_max

        r = np.where(np.isfinite(rng_s), rng_s, rmax + 1.0)
        r = np.clip(r, 0.0, rmax + 1.0)
        hit = (r <= rmax - 0.03) & (r > 0.12)

        wa = self.sang + yaw
        ca = np.cos(wa)
        sa = np.sin(wa)

        # --- local decay: recent evidence wins (robustness to pose drift)
        w_cells = int(rmax * self.inv_res) + 3
        cxi = int((x - self.ox) * self.inv_res)
        cyi = int((y - self.oy) * self.inv_res)
        wx0 = max(0, cxi - w_cells)
        wx1 = min(nx, cxi + w_cells + 1)
        wy0 = max(0, cyi - w_cells)
        wy1 = min(ny, cyi + w_cells + 1)
        if wx1 - wx0 > 2 and wy1 - wy0 > 2:
            self.lo[wy0:wy1, wx0:wx1] *= 0.985

        # --- free-space carving (optimistic map corrected by the sensor)
        S = self.samples
        free_len = np.minimum(r, rmax) - 0.55 * res
        mask = S[None, :] < free_len[:, None]
        px = x + ca[:, None] * S[None, :]
        py = y + sa[:, None] * S[None, :]
        ix = ((px - self.ox) * self.inv_res).astype(np.int32)
        iy = ((py - self.oy) * self.inv_res).astype(np.int32)
        ok = mask & (ix >= 1) & (ix < nx - 1) & (iy >= 1) & (iy < ny - 1)
        if ok.any():
            fidx = np.unique((iy * nx + ix)[ok])
            gain = LO_FREE / (1.0 + 1.5 * pose_err)
            v = self.lo_flat[fidx] - gain
            self.lo_flat[fidx] = np.maximum(v, LO_LO)

        # --- returns
        if hit.any():
            hxw = x + ca[hit] * r[hit]
            hyw = y + sa[hit] * r[hit]
            hix = ((hxw - self.ox) * self.inv_res).astype(np.int32)
            hiy = ((hyw - self.oy) * self.inv_res).astype(np.int32)
            good = ((hix >= 1) & (hix < nx - 1) &
                    (hiy >= 1) & (hiy < ny - 1))
            if good.any():
                hidx, cnt = np.unique((hiy * nx + hix)[good],
                                      return_counts=True)
                gain = LO_OCC / (1.0 + 1.0 * pose_err)
                v = self.lo_flat[hidx] + gain * cnt
                self.lo_flat[hidx] = np.minimum(v, LO_HI)

        # --- the robot is standing here, so here is traversable
        self.lo_flat[self._cell(x, y)] = LO_LO

        # --- hysteresis -> binary OBSTACLE/EMPTY over the sensed window
        if wx1 - wx0 < 2 or wy1 - wy0 < 2:
            return
        sub_lo = self.lo[wy0:wy1, wx0:wx1]
        sub_occ = self.occ[wy0:wy1, wx0:wx1]
        new = np.where(sub_lo > OCC_ON, np.float32(1.0),
                       np.where(sub_lo < OCC_OFF, np.float32(0.0), sub_occ))
        diff = new != sub_occ
        if not diff.any():
            return
        sub_occ[:] = new
        ys, xs = np.nonzero(diff)
        by0 = wy0 + int(ys.min())
        by1 = wy0 + int(ys.max()) + 1
        bx0 = wx0 + int(xs.min())
        bx1 = wx0 + int(xs.max()) + 1
        self._recompute_cost(bx0, by0, bx1, by1)

    def _recompute_cost(self, bx0, by0, bx1, by1):
        """Rebuild the inflated cost field on a window and queue the changes."""
        R = self.R
        nx = self.nx
        ny = self.ny
        bx0 = max(1, bx0 - R)
        by0 = max(1, by0 - R)
        bx1 = min(nx - 1, bx1 + R)
        by1 = min(ny - 1, by1 + R)
        h = by1 - by0
        w = bx1 - bx0
        if h <= 0 or w <= 0:
            return

        pad = np.zeros((h + 2 * R, w + 2 * R), dtype=np.float32)
        sy0 = max(0, by0 - R)
        sy1 = min(ny, by1 + R)
        sx0 = max(0, bx0 - R)
        sx1 = min(nx, bx1 + R)
        pad[sy0 - (by0 - R):sy1 - (by0 - R),
            sx0 - (bx0 - R):sx1 - (bx0 - R)] = self.occ[sy0:sy1, sx0:sx1]

        out = np.ones((h, w), dtype=np.float32)
        for (dy, dx, p) in self.kernel:
            sub = pad[R + dy:R + dy + h, R + dx:R + dx + w]
            np.maximum(out, sub * np.float32(p), out=out)
        out += self.frust[by0:by1, bx0:bx1]

        view = self.cost2d[by0:by1, bx0:bx1]
        chg = np.nonzero(np.abs(out - view) > 1e-3)
        if chg[0].size == 0:
            return
        view[:] = out
        flat = ((by0 + chg[0]).astype(np.int64) * nx +
                (bx0 + chg[1]).astype(np.int64))
        fl = flat.tolist()
        vals = self.cost_flat[flat].tolist()
        cl = self.costl
        for i in range(len(fl)):
            cl[fl[i]] = vals[i]
        self.dirty.update(fl)

    def _drain_dirty(self, limit):
        """Feed queued arc-cost edits to every D* tree via MODIFY-COST."""
        if not self.dirty:
            return
        blocked = self.blocked
        offs = self.nbr_off
        take = []
        d = self.dirty
        for _ in range(min(len(d), limit)):
            take.append(d.pop())
        touched = set()
        for c in take:
            if blocked[c]:
                continue
            touched.add(c)
            for o in offs:
                y = c + o
                if not blocked[y]:
                    touched.add(y)
        if not touched:
            return
        for pl in self.planners.values():
            pl.cell_cost_changed(touched)

    # ==================================================================
    #  Exploration cost raises (paper Sec. 5.2)
    # ==================================================================
    def _raise_visited(self, t_now, x, y):
        rc = int(math.ceil(FRUST_R * self.inv_res))
        cxi = int((x - self.ox) * self.inv_res)
        cyi = int((y - self.oy) * self.inv_res)
        x0 = max(1, cxi - rc)
        x1 = min(self.nx - 1, cxi + rc + 1)
        y0 = max(1, cyi - rc)
        y1 = min(self.ny - 1, cyi + rc + 1)
        if x1 <= x0 or y1 <= y0:
            return
        self.frust[y0:y1, x0:x1] += np.float32(FRUST_AMOUNT)
        self.frust_events.append((t_now + FRUST_TTL, x0, y0, x1, y1,
                                  FRUST_AMOUNT))
        if len(self.frust_events) > 40:
            ev = self.frust_events.pop(0)
            self.frust[ev[2]:ev[4], ev[1]:ev[3]] -= np.float32(ev[5])
            self._recompute_cost(ev[1], ev[2], ev[3], ev[4])
        self._recompute_cost(x0, y0, x1, y1)

    def _expire_raises(self, t_now):
        if not self.frust_events:
            return
        keep = []
        for ev in self.frust_events:
            if ev[0] <= t_now:
                self.frust[ev[2]:ev[4], ev[1]:ev[3]] -= np.float32(ev[5])
                np.maximum(self.frust[ev[2]:ev[4], ev[1]:ev[3]], 0.0,
                           out=self.frust[ev[2]:ev[4], ev[1]:ev[3]])
                self._recompute_cost(ev[1], ev[2], ev[3], ev[4])
            else:
                keep.append(ev)
        self.frust_events = keep

    # ==================================================================
    #  Planning
    # ==================================================================
    def _get_planner(self, goal_idx):
        pl = self.planners.get(goal_idx)
        if pl is None:
            if len(self.planners) >= 3:
                for kk in list(self.planners.keys()):
                    if kk != self.active_key:
                        del self.planners[kk]
                        break
            pl = _DStar(self.n, goal_idx, self.costl, self.blocked,
                        self.nbr_off, self.nbr_dist)
            self.planners[goal_idx] = pl
        return pl

    def _run_planner(self, pl, target, budget, max_exp):
        """PROCESS-STATE until k_min >= h(target)  (paper Sec. 2.2)."""
        t0 = time.perf_counter()
        n = 0
        h = pl.h
        t = pl.t
        while n < max_exp:
            if t[target] != 0:
                kmin = pl.get_kmin()
                if kmin < 0.0 or kmin >= h[target] - 1e-9:
                    return True
            r = pl.process_state()
            n += 1
            if r < 0.0:
                break
            if (n & 127) == 0 and (time.perf_counter() - t0) > budget:
                break
        return t[target] != 0 and h[target] < INF

    # ==================================================================
    #  Geometry helpers
    # ==================================================================
    def _los(self, x0, y0, x1, y1, thresh):
        dx = x1 - x0
        dy = y1 - y0
        d = math.hypot(dx, dy)
        if d < 1e-6:
            return True
        n = int(d * self.inv_res * 2.0) + 1
        if n > 64:
            n = 64
        cf = self.cost_flat
        nx = self.nx
        ox = self.ox
        oy = self.oy
        ir = self.inv_res
        for i in range(1, n + 1):
            f = float(i) / n
            px = x0 + dx * f
            py = y0 + dy * f
            ix = int((px - ox) * ir)
            iy = int((py - oy) * ir)
            if ix < 1 or iy < 1 or ix >= nx - 1 or iy >= self.ny - 1:
                return False
            if cf[iy * nx + ix] > thresh:
                return False
        return True

    # ==================================================================
    #  step
    # ==================================================================
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            self.fail_count += 1
            try:
                gx, gy = self._pt(obs.goal)
                p = np.asarray(obs.pose_estimate, dtype=float).ravel()
                b = _wrap(math.atan2(gy - p[1], gx - p[0]) - float(p[2]))
                vx = 0.3 * max(0.0, math.cos(b))
                return Command(vx, 0.0, _clip(1.5 * b, -self.max_ang,
                                              self.max_ang))
            except Exception:
                return Command(0.0, 0.0, 0.0)

    # ------------------------------------------------------------------
    def _step(self, obs):
        t_now = float(getattr(obs, "t", 0.0))
        pose = np.asarray(obs.pose_estimate, dtype=float).ravel()
        x = float(pose[0])
        y = float(pose[1])
        yaw = float(pose[2]) if pose.size > 2 else 0.0
        pose_err = float(getattr(obs, "pose_error_estimate", 0.0) or 0.0)
        contact = bool(getattr(obs, "contact", False))
        gx, gy = self._pt(obs.goal)
        self.step_i += 1

        # ---- lidar, reordered by bearing once -------------------------
        raw = np.asarray(obs.ranges, dtype=np.float64).ravel()
        if self.order is None or self.order.size != raw.size:
            self._prep_beams(obs.range_angles)
        r = raw[self.order]
        sang = self.sang
        rmax = self.range_max
        rf = np.where(np.isfinite(r), np.minimum(r, rmax), rmax)
        rf = np.maximum(rf, 0.0)

        # angular min-filter -> clearance available in each direction
        K = 8
        cmin = rf.copy()
        for s in range(1, K + 1):
            np.minimum(cmin, np.roll(rf, s), out=cmin)
            np.minimum(cmin, np.roll(rf, -s), out=cmin)

        dgx = gx - x
        dgy = gy - y
        dist_goal = math.hypot(dgx, dgy)
        goal_bearing = _wrap(math.atan2(dgy, dgx) - yaw)

        # ---- goal switch = one leg of the shuttle finished ------------
        if (abs(gx - self.goal_xy[0]) + abs(gy - self.goal_xy[1])) > 0.4:
            self.goal_xy = (gx, gy)
            self.trip_count += 1
            self.hist = []
            self.escape_until = -1.0

        # ---- map update (Sec. 5.1: edits near the robot only) ---------
        if (self.step_i % 2) == 0:
            self._update_map(rf, x, y, yaw, pose_err)
        if (self.step_i % 25) == 0:
            self._expire_raises(t_now)

        # ---- contact: the paper's contact-sensor map edit (Sec. 2.3) --
        if contact:
            hx = x + 0.45 * math.cos(yaw)
            hy = y + 0.45 * math.sin(yaw)
            ci = self._cell(hx, hy)
            self.lo_flat[ci] = LO_HI
            iy = ci // self.nx
            ix = ci - iy * self.nx
            if self.occ[iy, ix] < 0.5:
                self.occ[iy, ix] = 1.0
                self._recompute_cost(ix, iy, ix + 1, iy + 1)
            if t_now > self.escape_until:
                self.escape_until = t_now + 1.1
                self.spin_dir = 1.0 if self.rng.random() < 0.5 else -1.0

        # ---- livelock detection --------------------------------------
        if not self.hist or (t_now - self.hist[-1][0]) > 0.2:
            self.hist.append((t_now, x, y))
            if len(self.hist) > 40:
                self.hist.pop(0)
        if (t_now > self.escape_until and dist_goal > 1.2
                and len(self.hist) >= 25
                and (t_now - self.hist[0][0]) > 4.5):
            x0, y0 = self.hist[0][1], self.hist[0][2]
            if math.hypot(x - x0, y - y0) < 0.35:
                self.escape_until = t_now + 1.5
                # Sec. 5.2: raise the cost of the states just visited.
                self._raise_visited(t_now, x, y)
                left = float(np.max(cmin[sang > 0.3])) if np.any(sang > 0.3) else 0.0
                right = float(np.max(cmin[sang < -0.3])) if np.any(sang < -0.3) else 0.0
                self.spin_dir = 1.0 if left >= right else -1.0
                self.hist = []

        # ---- MODIFY-COST flush, then D* ------------------------------
        self._drain_dirty(1500)

        robot_idx = self._cell(x, y)
        goal_idx = self._cell(gx, gy)
        pl = self._get_planner(goal_idx)
        self.active_key = goal_idx

        bootstrap = (pl.t[robot_idx] == 0) or (pl.h[robot_idx] >= INF)
        budget = 0.013 if bootstrap else 0.0055
        ok = self._run_planner(pl, robot_idx, budget, 60000)

        # ---- follow the backpointer sequence -------------------------
        th_des = goal_bearing
        self.have_plan = False
        self.blocked_route = False
        if ok and pl.h[robot_idx] < INF:
            self.blocked_route = pl.h[robot_idx] > BLOCKED_THRESH
            cells, clean = pl.trace(robot_idx, 90)
            if not clean and (t_now - self.last_hard_reset) > 3.0:
                # Backpointer sequence corrupted: fall back to the paper's
                # brute-force replanner behaviour - rebuild this tree.
                self.last_hard_reset = t_now
                pl.reset_tree()
                cells = []
            if len(cells) >= 2:
                self.have_plan = True
                best = None
                misses = 0
                for i in range(1, len(cells)):
                    px, py = self._center(cells[i])
                    dd = math.hypot(px - x, py - y)
                    if dd > 3.2:
                        break
                    if self._los(x, y, px, py, 10.0):
                        best = (px, py, dd)
                        misses = 0
                    else:
                        misses += 1
                        if misses >= 3:
                            break
                if best is None:
                    px, py = self._center(cells[1])
                    best = (px, py, math.hypot(px - x, py - y))
                if best[2] > 0.12:
                    th_des = _wrap(math.atan2(best[1] - y, best[0] - x) - yaw)

        # close in: trust the goal directly, the lattice is too coarse here
        if dist_goal < 1.6:
            th_des = goal_bearing

        # ---- pre-warm the other shuttle leg with leftover time --------
        if ok and not bootstrap and len(self.planners) > 1 and \
                (self.step_i % 3) == 0:
            for kk, other in self.planners.items():
                if kk != goal_idx:
                    self._run_planner(other, robot_idx, 0.0025, 2500)
                    break

        # ==============================================================
        #  Reactive layer: pick a heading near the plan that is actually
        #  free, then gate the speed on the clearance in that direction.
        # ==============================================================
        if t_now < self.escape_until:
            vx, vy, wz = self._escape(rf, cmin, sang, dist_goal)
        else:
            dev = np.abs((sang - th_des + math.pi) % (2.0 * math.pi) - math.pi)
            clr = np.clip((cmin - 0.40) / 1.30, 0.0, 1.0)
            score = 2.2 * np.cos(dev) + 0.85 * clr - 0.40 * dev
            veto = cmin < (0.42 + 0.25 * pose_err)
            score = np.where(veto, score - 1000.0, score)
            j = int(np.argmax(score))
            if score[j] < -500.0:
                j = int(np.argmax(cmin))
            th = float(_wrap(sang[j]))

            fwd = np.abs(sang) < 0.42
            front = float(np.min(rf[fwd])) if np.any(fwd) else rmax

            v = self.max_lin * _clip((front - 0.45) / 0.90, 0.12, 1.0)
            v *= _clip(float(cmin[j]) / 1.50, 0.25, 1.0)
            v *= _clip(1.15 - 0.30 * pose_err, 0.45, 1.0)
            if self.blocked_route:
                v *= 0.55
            if not self.have_plan and dist_goal > 2.0:
                v *= 0.85
            v = min(v, max(0.22, 1.20 * dist_goal))
            v = _clip(v, 0.0, self.max_lin)

            if dist_goal < 0.55:
                # fine positioning: use the strafe axis, ignore heading
                s = _clip(1.6 * dist_goal, 0.10, 0.40)
                vx = s * math.cos(goal_bearing)
                vy = _clip(s * math.sin(goal_bearing), -0.35, 0.35)
                wz = _clip(1.2 * goal_bearing, -self.max_ang, self.max_ang)
            else:
                c = math.cos(th)
                sn = math.sin(th)
                vx = v * (c if c > 0.0 else 0.0)
                vy = _clip(v * 0.6 * sn, -0.30, 0.30) if c > 0.20 else 0.0
                wz = _clip(2.6 * th, -self.max_ang, self.max_ang)

        # ---- final safety gate on the commanded direction -------------
        vx, vy = self._gate(vx, vy, cmin, sang)

        vx = _clip(vx, -self.max_lin, self.max_lin)
        vy = _clip(vy, -self.max_lin, self.max_lin)
        wz = _clip(wz, -self.max_ang, self.max_ang)
        self.last_cmd = (vx, vy, wz)
        self.t_prev = t_now
        return Command(float(vx), float(vy), float(wz))

    # ------------------------------------------------------------------
    def _escape(self, rf, cmin, sang, dist_goal):
        j = int(np.argmin(rf))
        an = float(sang[j])
        dn = float(rf[j])
        if dn < 0.95:
            vx = -0.38 * math.cos(an)
            vy = -0.26 * math.sin(an)
        else:
            vx = -0.22
            vy = 0.0
        wz = self.spin_dir * 0.85 * self.max_ang
        return vx, _clip(vy, -0.30, 0.30), wz

    # ------------------------------------------------------------------
    def _gate(self, vx, vy, cmin, sang):
        sp = math.hypot(vx, vy)
        if sp < 1e-4:
            return vx, vy
        phi = math.atan2(vy, vx)
        k = int(np.argmin(np.abs((sang - phi + math.pi) %
                                 (2.0 * math.pi) - math.pi)))
        c = float(cmin[k])
        if c < 0.34:
            return 0.0, 0.0
        if c < 0.62:
            f = (c - 0.34) / 0.28
            return vx * f, vy * f
        return vx, vy
