# ---------------------------------------------------------------------------
# Go2 shuttle navigator built on D*.
#
# What is taken from Stentz, "Optimal and Efficient Path Planning for
# Partially-Known Environments" (ICRA 1994), and where:
#
#  * D* itself (Sec. 2.1-2.2): an 8-connected grid of states, a path cost to
#    goal h(.), a key k(.), tags NEW/OPEN/CLOSED, an OPEN list sorted by k,
#    and PROCESS-STATE transcribed line-for-line from Sec. 2.2 (L1-L26),
#    including the RAISE (k<h) / LOWER (k=h) split of Sec. 2.3 that carries
#    cost increases outward and cost reductions back in.
#  * MODIFY-COST (Sec. 2.2): every discrepancy the lidar (or the contact
#    sensor, as in the point robot of Sec. 2.3) finds between map and world
#    changes cell costs; the affected states are re-INSERTed with their
#    current h, which is exactly what makes their descendants RAISE states.
#  * The stopping rule of Sec. 2.2 / Property 2: expand only until
#    k_min >= h(robot).  That is what keeps the repair local (Sec. 4,
#    Table 1: "D* replans locally ... the optimal replanner generates a new
#    global trajectory") and therefore affordable at 50 Hz.
#  * Continuum arc costs (Sec. 2.0; the "grey is five times harder" terrain of
#    Figs. 4-6): obstacles are prohibitively but finitely expensive
#    (footnote 1 of Sec. 2.3) and are wrapped in a soft cost halo, so
#    clearance is bought by cost rather than by a hard forbidden zone.
#  * The optimistic map of Fig. 5 ("an optimistic map (all white)"): unmapped
#    cells cost the same as known-empty ones, so the robot heads straight for
#    the goal and pays only for what it actually discovers.
#  * D*'s incremental nature (Sec. 4) reused across the shuttle: one cost
#    field per endpoint is kept alive for the whole episode, so when the goal
#    flips back the old field is repaired with the newly-learned obstacles
#    instead of being rebuilt.  Pending discrepancies are queued per field
#    while it is inactive.
#  * Sec. 5.2: "Using a strategy of raising costs for previously visited
#    states, D* can be extended to support exploration tasks" (cf. Pirzadeh,
#    ref. [9], who increments the cost of a location each time it is visited).
#    Used here as the escape from dead ends / livelocks.
#
# Not carried across: quad-trees (Sec. 5.2) - the lidar map is uniform-grid
# and small; and the exact backpointer walk - h(.) is descended greedily,
# which is equivalent where the sequence {X} is monotonic but degrades
# gracefully where it is not yet repaired.
# ---------------------------------------------------------------------------

import heapq
import math
import time
from collections import deque

import numpy as np

from core.observation import Command

# --- D* state tags -----------------------------------------------------------
NEW, OPEN, CLOSED = 0, 1, 2

# Obstacle arc cost: "prohibitively large" but finite (Sec. 2.3, footnote 1).
OBST_COST = 1000.0
LETHAL_TH = 100.0          # cost at/above this is treated as untraversable

# --- occupancy mapping -------------------------------------------------------
LOG_HIT = 0.95
LOG_MISS = -0.45
LOG_LO, LOG_HI = -3.0, 5.0
OCC_HI, OCC_LO = 1.0, 0.2  # hysteresis so drift-induced flicker is damped
MAP_MAX = 5.0              # only trust beams shorter than this for mapping

# --- inflation (the "continuum" of arc costs) --------------------------------
LETHAL_R = 0.30
INFL_R = 0.80
SOFT_MAX = 2.5
SOFT_SCALE = 0.25

EPS = 1e-9
SQ2 = math.sqrt(2.0)
_OFFSETS = ((-1, -1, SQ2), (-1, 0, 1.0), (-1, 1, SQ2),
            (0, -1, 1.0), (0, 1, 1.0),
            (1, -1, SQ2), (1, 0, 1.0), (1, 1, SQ2))


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def _as_point(v, default):
    try:
        a = np.asarray(v, dtype=float).ravel()
        if a.size >= 2 and np.all(np.isfinite(a[:2])):
            return np.array([float(a[0]), float(a[1])])
    except Exception:
        pass
    return np.array([float(default[0]), float(default[1])])


class _Field:
    """All D* bookkeeping for one goal state G: h, k, t, b and the OPEN list."""

    __slots__ = ("h", "k", "t", "b", "open", "pending", "goal_idx")

    def __init__(self, n, goal_idx):
        self.h = np.full(n, np.inf, dtype=np.float64)
        self.k = np.full(n, np.inf, dtype=np.float64)
        self.t = np.zeros(n, dtype=np.uint8)
        self.b = np.full(n, -1, dtype=np.int64)
        self.open = []
        self.pending = set()
        self.goal_idx = int(goal_idx)
        # "h(G) is set to zero, and G is placed on the OPEN list" (Sec. 2.2)
        self.h[self.goal_idx] = 0.0
        self.k[self.goal_idx] = 0.0
        self.t[self.goal_idx] = OPEN
        heapq.heappush(self.open, (0.0, self.goal_idx))


class Controller:
    # ------------------------------------------------------------------ reset
    def reset(self, scenario, rng):
        self.rng = rng

        self.max_lin = abs(float(getattr(scenario, "max_lin", 0.8) or 0.8))
        self.max_ang = abs(float(getattr(scenario, "max_ang", 1.0) or 1.0))
        if self.max_lin <= 0.0:
            self.max_lin = 0.8
        if self.max_ang <= 0.0:
            self.max_ang = 1.0
        self.range_max = abs(float(getattr(scenario, "range_max", 6.0) or 6.0))
        if self.range_max <= 0.0:
            self.range_max = 6.0
        self.time_limit = float(getattr(scenario, "time_limit", 1.0e9) or 1.0e9)

        start = _as_point(getattr(scenario, "start", None), (0.0, 0.0))
        goal = _as_point(getattr(scenario, "goal", None),
                         (start[0] + 5.0, start[1]))
        self.protected = [start.copy(), goal.copy()]

        # ---- planning grid (sized to hold both shuttle endpoints + slack) --
        margin = 20.0
        xlo = min(start[0], goal[0]) - margin
        xhi = max(start[0], goal[0]) + margin
        ylo = min(start[1], goal[1]) - margin
        yhi = max(start[1], goal[1]) + margin
        res = 0.25
        for _ in range(12):
            W = int(math.ceil((xhi - xlo) / res)) + 2
            H = int(math.ceil((yhi - ylo) / res)) + 2
            if W * H <= 260000:
                break
            res *= 1.25
        self.res = res
        self.inv_res = 1.0 / res
        self.W = W
        self.H = H
        self.n = W * H
        self.x0 = xlo
        self.y0 = ylo
        self._d1 = res
        self._d2 = res * SQ2
        self._off = tuple((di, dj, (self._d2 if d > 1.2 else self._d1))
                          for (di, dj, d) in _OFFSETS)

        # ---- maps ----------------------------------------------------------
        self.lo = np.zeros((H, W), dtype=np.float64)
        self.lo_flat = self.lo.reshape(-1)
        self.occ = np.zeros((H, W), dtype=bool)
        self.occ_flat = self.occ.reshape(-1)
        self.pen = np.zeros((H, W), dtype=np.float64)     # visited-state raise
        self.pen_flat = self.pen.reshape(-1)
        self.cost = np.ones((H, W), dtype=np.float64)     # optimistic (Fig. 5)
        self.cost_flat = self.cost.reshape(-1)

        # inflation stencil: (di, dj, soft weight, lethal?)
        R = int(math.ceil(INFL_R / res))
        infl = []
        for di in range(-R, R + 1):
            for dj in range(-R, R + 1):
                d = res * math.hypot(di, dj)
                if d > INFL_R + 1e-9:
                    continue
                w = SOFT_MAX * math.exp(-max(0.0, d - LETHAL_R) / SOFT_SCALE)
                infl.append((di, dj, w, d <= LETHAL_R + 1e-9))
        self._infl = tuple(infl)
        self.infl_R = R

        # free-space ray samples
        self._samples = np.arange(0.05, MAP_MAX, max(0.10, 0.6 * res))

        # ---- D* fields, one per shuttle endpoint ---------------------------
        self._fields = {}
        self._F = None
        self._goal_key = None
        self._goal_switch_t = -1.0e9

        # ---- controller state ---------------------------------------------
        self._pen_cells = set()
        self._hist = deque()
        self._prev = (0.0, 0.0, 0.0)
        self._escape_until = -1.0e9
        self._escape_spin = 1.0
        self._last_escape = -1.0e9
        self._contact_n = 0
        self._near_goal_since = None
        self._rev = False
        self._side = 1.0
        self._side_t = -1.0e9
        self._steps = 0
        self._last_t = 0.0

    # ------------------------------------------------------------ grid utils
    def _cells(self, xs, ys):
        jj = np.floor((np.asarray(xs, dtype=float) - self.x0) * self.inv_res)
        ii = np.floor((np.asarray(ys, dtype=float) - self.y0) * self.inv_res)
        ok = np.isfinite(jj) & np.isfinite(ii)
        jj = np.where(ok, jj, -1.0).astype(np.int64)
        ii = np.where(ok, ii, -1.0).astype(np.int64)
        m = ok & (ii >= 0) & (ii < self.H) & (jj >= 0) & (jj < self.W)
        return ii[m] * self.W + jj[m]

    def _cell_clamped(self, x, y):
        j = int(math.floor((x - self.x0) * self.inv_res))
        i = int(math.floor((y - self.y0) * self.inv_res))
        if j < 0:
            j = 0
        elif j >= self.W:
            j = self.W - 1
        if i < 0:
            i = 0
        elif i >= self.H:
            i = self.H - 1
        return i * self.W + j

    def _center(self, idx):
        i, j = divmod(int(idx), self.W)
        return (self.x0 + (j + 0.5) * self.res, self.y0 + (i + 0.5) * self.res)

    def _nbrs(self, x):
        W = self.W
        H = self.H
        i, j = divmod(int(x), W)
        out = []
        for di, dj, dm in self._off:
            ii = i + di
            jj = j + dj
            if 0 <= ii < H and 0 <= jj < W:
                out.append((ii * W + jj, dm))
        return out

    # --------------------------------------------------------- D* primitives
    def _insert(self, F, y, hnew):
        """INSERT(X, h_new) of Sec. 2.2."""
        if not (hnew < np.inf) or hnew != hnew:
            return
        y = int(y)
        ty = F.t[y]
        if ty == NEW:
            kn = hnew
        elif ty == OPEN:
            ky = F.k[y]
            kn = ky if ky < hnew else hnew
        else:
            hy = F.h[y]
            kn = hy if hy < hnew else hnew
        F.h[y] = hnew
        F.k[y] = kn
        F.t[y] = OPEN
        heapq.heappush(F.open, (float(kn), y))

    def _kmin(self, F):
        """GET-KMIN with lazy deletion of stale heap entries."""
        op = F.open
        tg = F.t
        kk = F.k
        while op:
            kv, cand = op[0]
            if tg[cand] != OPEN or kv > kk[cand] + 1e-12:
                heapq.heappop(op)
                continue
            return kv
        return -1.0

    def _process_state(self, F):
        """PROCESS-STATE, Sec. 2.2, lines L1-L26."""
        op = F.open
        h = F.h
        k = F.k
        tg = F.t
        b = F.b
        cost = self.cost_flat

        x = -1
        kold = 0.0
        while op:                                   # L1-L3 (MIN-STATE/DELETE)
            kv, cand = op[0]
            if tg[cand] != OPEN or kv > k[cand] + 1e-12:
                heapq.heappop(op)
                continue
            heapq.heappop(op)
            x = int(cand)
            kold = float(kv)
            tg[x] = CLOSED
            break
        if x < 0:
            return -1.0                              # L2

        hx = float(h[x])
        cx = float(cost[x])
        nbrs = self._nbrs(x)

        if kold < hx - EPS:                          # L4  (RAISE state)
            for y, dm in nbrs:                       # L5
                hy = float(h[y])
                if hy <= kold + EPS:                 # L6
                    c = 0.5 * (cx + float(cost[y])) * dm
                    if hx > hy + c + EPS:
                        b[x] = y                     # L7
                        hx = hy + c
                        h[x] = hx

        if abs(kold - hx) <= EPS:                    # L8  (LOWER state)
            for y, dm in nbrs:                       # L9
                c = 0.5 * (cx + float(cost[y])) * dm
                nh = hx + c
                hy = float(h[y])
                if (tg[y] == NEW                     # L10
                        or (b[y] == x and abs(hy - nh) > EPS)      # L11
                        or (b[y] != x and hy > nh + EPS)):         # L12
                    b[y] = x                         # L13
                    self._insert(F, y, nh)
        else:                                        # L14
            for y, dm in nbrs:                       # L15
                c = 0.5 * (cx + float(cost[y])) * dm
                nh = hx + c
                hy = float(h[y])
                if tg[y] == NEW or (b[y] == x and abs(hy - nh) > EPS):  # L16-17
                    b[y] = x                         # L18
                    self._insert(F, y, nh)
                else:                                # L19
                    if b[y] != x and hy > nh + EPS:  # L20
                        self._insert(F, x, hx)       # L21
                    else:                            # L22
                        if (b[y] != x and hx > hy + c + EPS
                                and tg[y] == CLOSED and hy > kold + EPS):
                            self._insert(F, y, hy)   # L23-L25
        return self._kmin(F)                         # L26

    def _seed_pending(self, F, limit):
        """MODIFY-COST: enter states whose arc costs changed on the OPEN list."""
        if not F.pending:
            return
        h = F.h
        tg = F.t
        cnt = 0
        pend = F.pending
        while pend and cnt < limit:
            x = pend.pop()
            cnt += 1
            if tg[x] == CLOSED:
                self._insert(F, x, float(h[x]))
            for y, _dm in self._nbrs(x):
                if tg[y] == CLOSED:
                    self._insert(F, y, float(h[y]))

    def _run_dstar(self, F, ridx, budget_s, max_exp):
        """Expand until k_min >= h(robot) (Sec. 2.2 / Property 2)."""
        deadline = time.perf_counter() + budget_s
        h = F.h
        n = 0
        if len(F.open) > 400000:                     # compact the lazy heap
            idx = np.flatnonzero(F.t == OPEN)
            F.open = [(float(F.k[i]), int(i)) for i in idx]
            heapq.heapify(F.open)
        while n < max_exp:
            km = self._kmin(F)
            if km < 0.0:
                break
            hr = float(h[ridx])
            if F.t[ridx] != NEW and hr < np.inf and km >= hr - 1e-9:
                break
            self._process_state(F)
            n += 1
            if (n & 31) == 0 and time.perf_counter() > deadline:
                break
        return n

    # -------------------------------------------------------- cost / mapping
    def _push_dirty(self, cells):
        if cells is None or cells.size == 0:
            return
        lst = cells.tolist()
        drop = []
        for key, F in self._fields.items():
            if F is not self._F and len(F.pending) > 120000:
                drop.append(key)
                continue
            F.pending.update(lst)
        for key in drop:
            del self._fields[key]

    def _refresh_cost(self, cells):
        cells = np.unique(np.asarray(cells, dtype=np.int64))
        if cells.size == 0:
            return
        R = self.infl_R
        W = self.W
        H = self.H
        dirty = []
        CH = 512
        for s in range(0, cells.size, CH):
            chunk = cells[s:s + CH]
            ci = chunk // W
            cj = chunk % W
            i0 = max(0, int(ci.min()) - R)
            i1 = min(H, int(ci.max()) + R + 1)
            j0 = max(0, int(cj.min()) - R)
            j1 = min(W, int(cj.max()) + R + 1)
            hh = i1 - i0
            ww = j1 - j0
            if hh <= 0 or ww <= 0:
                continue
            pi0 = i0 - R
            pj0 = j0 - R
            blk = np.zeros((hh + 2 * R, ww + 2 * R), dtype=bool)
            si0 = max(0, pi0)
            si1 = min(H, i1 + R)
            sj0 = max(0, pj0)
            sj1 = min(W, j1 + R)
            if si1 > si0 and sj1 > sj0:
                blk[si0 - pi0:si1 - pi0, sj0 - pj0:sj1 - pj0] = \
                    self.occ[si0:si1, sj0:sj1]
            P = np.zeros((hh, ww), dtype=np.float64)
            leth = np.zeros((hh, ww), dtype=bool)
            for di, dj, w, is_leth in self._infl:
                sl = blk[R + di:R + di + hh, R + dj:R + dj + ww]
                if is_leth:
                    leth |= sl
                np.maximum(P, sl * w, out=P)
            c = 1.0 + P + self.pen[i0:i1, j0:j1]
            occ_blk = self.occ[i0:i1, j0:j1]
            c[leth] = OBST_COST
            c[occ_blk] = OBST_COST
            # Never let inflation seal off a shuttle endpoint.
            rr = int(math.ceil(0.40 * self.inv_res))
            for pt in self.protected:
                pj = int(math.floor((pt[0] - self.x0) * self.inv_res))
                pi = int(math.floor((pt[1] - self.y0) * self.inv_res))
                ai0 = max(i0, pi - rr)
                ai1 = min(i1, pi + rr + 1)
                aj0 = max(j0, pj - rr)
                aj1 = min(j1, pj + rr + 1)
                if ai1 <= ai0 or aj1 <= aj0:
                    continue
                sub = c[ai0 - i0:ai1 - i0, aj0 - j0:aj1 - j0]
                free = ~self.occ[ai0:ai1, aj0:aj1]
                np.minimum(sub, 1.6, out=sub, where=free)
            old = self.cost[i0:i1, j0:j1]
            ch = np.abs(old - c) > 0.02
            if ch.any():
                ii, jj = np.nonzero(ch)
                dirty.append((ii + i0) * W + (jj + j0))
            old[...] = c
        if dirty:
            self._push_dirty(np.concatenate(dirty))

    def _apply_logodds(self, cells, delta, acc):
        if cells.size == 0:
            return
        lo = self.lo_flat
        v = np.clip(lo[cells] + delta, LOG_LO, LOG_HI)
        lo[cells] = v
        prev = self.occ_flat[cells]
        now = np.where(v > OCC_HI, True, np.where(v < OCC_LO, False, prev))
        ch = now != prev
        if ch.any():
            sel = cells[ch]
            self.occ_flat[sel] = now[ch]
            acc.append(sel)

    def _update_map(self, px, py, yaw, ranges, angles):
        changed = []
        n = min(ranges.size, angles.size)
        if n == 0:
            return
        r = ranges[:n]
        a = angles[:n] + yaw
        finite = np.isfinite(r) & (r > 0.02)
        rc = np.where(finite, r, self.range_max)
        rmap = np.minimum(rc, MAP_MAX)
        ca = np.cos(a)
        sa = np.sin(a)

        S = self._samples
        end = np.maximum(rmap - 0.6 * self.res, 0.0)
        m = S[None, :] < end[:, None]
        if m.any():
            xs = (px + ca[:, None] * S[None, :])[m]
            ys = (py + sa[:, None] * S[None, :])[m]
            free = np.unique(self._cells(xs, ys))
        else:
            free = np.zeros(0, dtype=np.int64)

        hit = finite & (r <= MAP_MAX) & (r < self.range_max - 0.05)
        if hit.any():
            hx = px + ca[hit] * r[hit]
            hy = py + sa[hit] * r[hit]
            hits = np.unique(self._cells(hx, hy))
        else:
            hits = np.zeros(0, dtype=np.int64)

        if free.size and hits.size:
            free = np.setdiff1d(free, hits, assume_unique=True)
        self._apply_logodds(free, LOG_MISS, changed)
        self._apply_logodds(hits, LOG_HIT, changed)
        if changed:
            self._refresh_cost(np.concatenate(changed))

    def _mark_contact(self, px, py, yaw, vx, vy):
        ang = yaw
        if abs(vx) + abs(vy) > 1e-3:
            ang = yaw + math.atan2(vy, vx)
        aa = ang + np.array([-0.7, -0.35, 0.0, 0.35, 0.7])
        xs = px + 0.38 * np.cos(aa)
        ys = py + 0.38 * np.sin(aa)
        cells = np.unique(self._cells(xs, ys))
        acc = []
        self._apply_logodds(cells, 1.6, acc)
        if acc:
            self._refresh_cost(np.concatenate(acc))

    # ------------------------------------------------- exploration penalties
    def _add_penalty(self, cells, amount):
        cells = np.unique(np.asarray(cells, dtype=np.int64))
        if cells.size == 0:
            return
        self.pen_flat[cells] += amount
        self._pen_cells.update(cells.tolist())
        self._refresh_cost(cells)

    def _clear_penalties(self):
        if not self._pen_cells:
            return
        arr = np.fromiter(self._pen_cells, dtype=np.int64,
                          count=len(self._pen_cells))
        self._pen_cells.clear()
        self.pen_flat[arr] = 0.0
        self._refresh_cost(arr)

    # ------------------------------------------------------- path extraction
    def _descend(self, F, start_idx, max_steps=180, max_len=7.0):
        """Follow decreasing h(.) - the sequence {X} of Sec. 2.1."""
        pts = []
        x = int(start_idx)
        seen = set()
        h = F.h
        cost = self.cost_flat
        acc = 0.0
        lx, ly = self._center(x)
        for _ in range(max_steps):
            if x == F.goal_idx:
                break
            seen.add(x)
            hx = float(h[x])
            cx = float(cost[x])
            best = -1
            bestv = np.inf
            for y, dm in self._nbrs(x):
                if y in seen:
                    continue
                hy = float(h[y])
                if not (hy < np.inf):
                    continue
                v = hy + 0.5 * (cx + float(cost[y])) * dm
                if v < bestv:
                    bestv = v
                    best = y
            if best < 0:
                break
            if bestv > hx + 8.0 and pts:
                break
            x = best
            gx, gy = self._center(x)
            acc += math.hypot(gx - lx, gy - ly)
            lx, ly = gx, gy
            pts.append((gx, gy))
            if acc > max_len:
                break
        return pts

    def _clear_line(self, ax, ay, bx, by):
        dx = bx - ax
        dy = by - ay
        L = math.hypot(dx, dy)
        k = int(L * self.inv_res * 2.0) + 2
        ts = np.linspace(0.0, 1.0, k)
        idx = self._cells(ax + dx * ts, ay + dy * ts)
        if idx.size == 0:
            return False
        return bool(self.cost_flat[idx].max() < LETHAL_TH)

    def _target_on_path(self, px, py, pts):
        if not pts:
            return None
        keep = []
        acc = 0.0
        lx, ly = px, py
        for (x, y) in pts:
            acc += math.hypot(x - lx, y - ly)
            lx, ly = x, y
            keep.append((x, y))
            if acc > 3.5:
                break
        i = len(keep) - 1
        while i >= 0:
            if self._clear_line(px, py, keep[i][0], keep[i][1]):
                return keep[i]
            i -= 2
        return keep[0]

    # ------------------------------------------------------------------ step
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            return Command(0.0, 0.0, 0.0)

    def _step(self, obs):
        self._steps += 1
        t = float(getattr(obs, "t", self._last_t + 0.02))
        if not math.isfinite(t):
            t = self._last_t + 0.02
        self._last_t = t

        pose = np.asarray(obs.pose_estimate, dtype=float).ravel()
        px = float(pose[0]) if pose.size > 0 and math.isfinite(pose[0]) else 0.0
        py = float(pose[1]) if pose.size > 1 and math.isfinite(pose[1]) else 0.0
        yaw = float(pose[2]) if pose.size > 2 and math.isfinite(pose[2]) else 0.0

        goal = _as_point(getattr(obs, "goal", None), (px, py))
        try:
            perr = float(getattr(obs, "pose_error_estimate", 0.0))
            if not math.isfinite(perr):
                perr = 0.0
        except Exception:
            perr = 0.0
        contact = bool(getattr(obs, "contact", False))

        ranges = np.asarray(obs.ranges, dtype=float).ravel()
        angles = np.asarray(obs.range_angles, dtype=float).ravel()
        nb = min(ranges.size, angles.size)
        ranges = ranges[:nb]
        angles = angles[:nb]
        d = np.where(np.isfinite(ranges) & (ranges > 0.0), ranges,
                     self.range_max)

        # ---- 1. map maintenance = MODIFY-COST source ----------------------
        self._update_map(px, py, yaw, ranges, angles)
        if contact:
            self._contact_n += 1
            self._mark_contact(px, py, yaw, self._prev[0], self._prev[1])
        else:
            self._contact_n = 0

        # ---- 2. which endpoint are we driving to? -------------------------
        gidx = self._cell_clamped(goal[0], goal[1])
        if self._goal_key is None or gidx != self._goal_key:
            self._goal_key = gidx
            self._goal_switch_t = t
            self._near_goal_since = None
            self._clear_penalties()
            known = False
            for pt in self.protected:
                if math.hypot(pt[0] - goal[0], pt[1] - goal[1]) < 0.6:
                    known = True
                    break
            if not known:
                self.protected.append(goal.copy())
                self._refresh_cost(np.array([gidx], dtype=np.int64))
        F = self._fields.get(gidx)
        if F is None:
            F = _Field(self.n, gidx)
            self._fields[gidx] = F
        self._F = F

        # ---- 3. incremental D* repair -------------------------------------
        ridx = self._cell_clamped(px, py)
        hr = float(F.h[ridx])
        need = (F.t[ridx] == NEW) or not (hr < np.inf)
        self._seed_pending(F, 6000 if need else 1500)
        if need or F.pending:
            self._run_dstar(F, ridx, 0.045, 120000)
        else:
            self._run_dstar(F, ridx, 0.006, 20000)
        hr = float(F.h[ridx])
        have_plan = (F.t[ridx] != NEW) and (hr < np.inf)

        # ---- 4. progress monitoring / exploration penalty -----------------
        gdist = math.hypot(goal[0] - px, goal[1] - py)
        self._hist.append((t, px, py))
        while self._hist and t - self._hist[0][0] > 5.0:
            self._hist.popleft()
        if (len(self._hist) > 20 and t - self._hist[0][0] >= 4.0
                and gdist > 0.9 and t - self._last_escape > 4.0):
            xs = [p[1] for p in self._hist]
            ys = [p[2] for p in self._hist]
            span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
            if span < 0.65:
                # Sec. 5.2: raise the cost of the states just visited.
                tx = np.array(xs)
                ty = np.array(ys)
                trail = self._cells(tx, ty)
                disc = []
                rr = 0.55
                for dxo in (-rr, 0.0, rr):
                    for dyo in (-rr, 0.0, rr):
                        disc.append(self._cells(tx + dxo, ty + dyo))
                disc.append(trail)
                self._add_penalty(np.concatenate(disc), 3.0)
                self._escape_until = t + 1.1
                self._last_escape = t
                self._escape_spin = 1.0 if self.rng.random() < 0.5 else -1.0
                self._hist.clear()
        if self._contact_n > 25 and t - self._last_escape > 2.0:
            self._escape_until = t + 0.9
            self._last_escape = t
            self._escape_spin = 1.0 if self.rng.random() < 0.5 else -1.0

        # ---- 5. pick a target --------------------------------------------
        target = None
        if gdist < 1.3 and self._clear_line(px, py, goal[0], goal[1]):
            target = (goal[0], goal[1])
        elif have_plan:
            target = self._target_on_path(px, py, self._descend(F, ridx))

        fallback = target is None
        if fallback:
            ux = goal[0] - px
            uy = goal[1] - py
            L = math.hypot(ux, uy)
            if L < 1e-6:
                ux, uy, L = math.cos(yaw), math.sin(yaw), 1.0
            ux /= L
            uy /= L
            fwd = self._cone_clear(d, angles, math.atan2(uy, ux) - yaw)
            if fwd < 1.2:
                if t - self._side_t > 2.5:
                    left = self._cone_clear(d, angles, 1.0, 0.6)
                    right = self._cone_clear(d, angles, -1.0, 0.6)
                    self._side = 1.0 if left >= right else -1.0
                    self._side_t = t
                ux, uy = (ux - self._side * uy * 1.3, uy + self._side * ux * 1.3)
                L = math.hypot(ux, uy)
                ux /= L
                uy /= L
            target = (px + 2.0 * ux, py + 2.0 * uy)

        # ---- 6. goal-proximity spiral (dead-reckoning error) --------------
        if gdist < 0.5:
            if self._near_goal_since is None:
                self._near_goal_since = t
        elif gdist > 0.95:
            self._near_goal_since = None
        if (self._near_goal_since is not None
                and t - self._near_goal_since > 4.0):
            age = t - self._near_goal_since - 4.0
            rad = _clamp(0.35 + 0.10 * age + 0.5 * perr, 0.35, 1.6)
            phi = 1.3 * age
            target = (goal[0] + rad * math.cos(phi),
                      goal[1] + rad * math.sin(phi))

        # ---- 7. velocity command -----------------------------------------
        if t < self._escape_until:
            vx, vy, wz = self._escape_cmd(d, angles)
        else:
            vx, vy, wz = self._track(px, py, yaw, target, d, angles,
                                     gdist, perr)

        a = 0.65
        vx = a * vx + (1.0 - a) * self._prev[0]
        vy = a * vy + (1.0 - a) * self._prev[1]
        wz = a * wz + (1.0 - a) * self._prev[2]
        vx = _clamp(vx, -self.max_lin, self.max_lin)
        vy = _clamp(vy, -0.6, 0.6)
        wz = _clamp(wz, -self.max_ang, self.max_ang)
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            vx = vy = wz = 0.0
        self._prev = (vx, vy, wz)
        return Command(float(vx), float(vy), float(wz))

    # ------------------------------------------------------- local behaviour
    def _cone_clear(self, d, angles, ang, half=0.55):
        if d.size == 0:
            return self.range_max
        m = np.abs(_wrap_arr(angles - ang)) < half
        if not m.any():
            return self.range_max
        return float(np.min(d[m]))

    def _repulse(self, d, angles):
        if d.size == 0:
            return 0.0, 0.0
        D0 = 1.15
        m = d < D0
        if not m.any():
            return 0.0, 0.0
        w = (D0 - d[m]) / D0
        w = w * w
        rx = -float(np.sum(w * np.cos(angles[m])))
        ry = -float(np.sum(w * np.sin(angles[m])))
        nrm = math.hypot(rx, ry)
        if nrm < 1e-6:
            return 0.0, 0.0
        mag = 0.38 * min(1.0, nrm / 6.0)
        return rx / nrm * mag, ry / nrm * mag

    def _track(self, px, py, yaw, target, d, angles, gdist, perr):
        dx = target[0] - px
        dy = target[1] - py
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return 0.0, 0.0, 0.0
        des = math.atan2(dy, dx)
        err = _wrap(des - yaw)

        # Full-circle lidar: driving backwards is as safe as forwards, so pick
        # whichever body axis is closer and save the 180-degree turn.
        if self._rev:
            if abs(err) < 1.25:
                self._rev = False
        else:
            if abs(err) > 1.95:
                self._rev = True
        head_err = _wrap(err - math.pi) if self._rev else err

        cl = self._cone_clear(d, angles, err)
        s = self.max_lin * _clamp((cl - 0.35) / 0.55, 0.0, 1.0)
        s *= 1.0 - 0.30 * min(1.0, abs(head_err) / 1.2)
        s = min(s, 0.9 * dist + 0.18)
        if gdist < 0.55:
            s = min(s, 0.34)
        if perr > 0.8:
            s *= 0.85

        vx = s * math.cos(err)
        vy = s * math.sin(err)
        rx, ry = self._repulse(d, angles)
        vx += rx
        vy += ry
        wz = _clamp(1.7 * head_err, -self.max_ang, self.max_ang)
        return vx, vy, wz

    def _escape_cmd(self, d, angles):
        if d.size == 0:
            return -0.3, 0.0, self._escape_spin * 0.8 * self.max_ang
        i = int(np.argmin(d))
        away = _wrap(float(angles[i]) + math.pi)
        cl = self._cone_clear(d, angles, away, 0.5)
        s = 0.45 * _clamp((cl - 0.3) / 0.5, 0.0, 1.0)
        vx = s * math.cos(away)
        vy = s * math.sin(away)
        rx, ry = self._repulse(d, angles)
        return vx + rx, vy + ry, self._escape_spin * 0.8 * self.max_ang


def _wrap_arr(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# ==== EXP C INTERVENTION: NOT MODEL-WRITTEN ================================
# Appended by demo/expC.py to a byte-identical copy of demo/arms/armB/k0.py
# (sha256 dc3622b547c3568e8a8eacc4ddd91545182cc2c50683e3aff99610a6319fe2ae).
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
        self._fields = {}
