# =============================================================================
# Navigation controller for a Unitree Go2 shuttling between two fixed points
# in an unmapped indoor space.  Planar lidar + odometry only; output is a Twist.
#
# WHAT IS TAKEN FROM THE PAPER
#   Stentz, "Optimal and Efficient Path Planning for Partially-Known
#   Environments", ICRA 1994.
#
#   * D* itself (Sec. 2.1-2.2).  PROCESS-STATE (lines L1-L26) and MODIFY-COST
#     (lines L1-L3) are implemented essentially line for line over an
#     8-connected grid of cells whose arc costs "range over a continuum"
#     (Sec. 1.0/2.0): the OPEN list sorted by k(), the tags t()=NEW/OPEN/
#     CLOSED, the backpointers b(), h(), and INSERT's three-case rule for k
#     (NEW: h_new, OPEN: min(k,h_new), CLOSED: min(h,h_new)).  RAISE states
#     (k<h) carry cost increases outward, LOWER states (k=h) carry cost
#     reductions back in (Sec. 2.3, Figs. 2-3) -- this is what lets the robot
#     escape a "potential well" that the lidar only reveals from inside.
#   * Replanning only until k_min >= h(robot) (Sec. 2.2 and Property 2)
#     instead of regenerating a global path.  This is the paper's efficiency
#     claim (Sec. 4.0, Table 1: local repair stays constant in complexity as
#     the environment grows) and it is the only reason a graph search of this
#     kind fits in a 50 Hz control step at all.
#   * The optimistic empty map of Fig. 5: unknown cells are assumed traversable
#     at nominal cost, so with no a-priori map the robot simply heads for the
#     goal and pays only for the discrepancies its sensor finds.
#   * Footnote 1, p. 4: OBSTACLE arc cost is a large *finite* number, so a
#     sequence to the goal always exists and a threshold on h() tells us
#     whether the best route is still obstructed.
#   * The sensor/contact-sensor model of Sec. 2.3: every discrepancy between
#     map and world (lidar return, or a bumper contact) is fed through
#     MODIFY-COST and seeded on the OPEN list of *every* live planner.
#   * Sec. 5.2, "using a strategy of raising costs for previously visited
#     states, D* can be extended to support exploration": costs are raised
#     around places where the robot actually got wedged, which pushes the
#     backpointers out of traps and stops it re-entering them.
#   * Sec. 5.1, the full a-priori-information spectrum.  Because the map and
#     BOTH endpoints' cost fields persist for the whole episode, trip 1 is the
#     Fig. 5 optimistic-map case and every later trip is replanned against an
#     increasingly complete map, approaching the Fig. 4 "omniscient optimal"
#     route.  One D* instance per endpoint means the return trip's h() field
#     is not thrown away when obs.goal flips -- it is only repaired.
#   * Sec. 1.0, Lumelsky [7]: until D* has propagated as far as the robot's
#     own cell (cold start), the fallback is exactly that behaviour -- drive
#     straight at the goal and skirt whatever blocks the way.
#
# Everything below the planner (carrot tracking, lidar corridor clearance,
# recovery) is the Go2-specific carry-across the paper does not cover.
# =============================================================================

import heapq
import math
import time
from collections import deque

import numpy as np

from core.observation import Command

# ---- map / cost model -------------------------------------------------------
RES = 0.20                 # grid resolution [m]
MARGIN = 12.0              # margin around the start/goal bbox [m]
MAX_CELLS = 150000
L_MIN, L_MAX = -3.0, 5.0   # log-odds clamps
OCC_INC, FREE_DEC = 0.90, 0.32
OCC_T, FREE_T = 1.10, 0.25  # occupancy hysteresis thresholds
HARD_R = 0.30              # cells this close to an occupied cell are OBSTACLE
PEN_R = 0.85               # soft (continuum) cost halo radius
PEN_W = 2.6
PEN_CAP = 8.0
BLOCK_COST = 5.0e4         # large but FINITE (paper footnote 1)
BLOCK_H = 1.0e4
EPS = 1e-9

# ---- reactive layer ---------------------------------------------------------
W_HALF = 0.27              # half corridor width the body needs [m]
MIN_CLEAR = 0.55
STOP_D = 0.38
LOOK = 2.2                 # carrot lookahead [m]
NCAND = 72

_OFFS8 = (
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, 1.4142135623730951), (1, -1, 1.4142135623730951),
    (-1, 1, 1.4142135623730951), (-1, -1, 1.4142135623730951),
)


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


# =============================================================================
# Occupancy / cost map.  Holds the arc-cost function c() of the paper, and
# reports which cells changed so MODIFY-COST can be applied.
# =============================================================================
class _GridMap:
    def __init__(self, pts, margin, res, max_cells):
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        lox, hix = min(xs) - margin, max(xs) + margin
        loy, hiy = min(ys) - margin, max(ys) + margin
        while True:
            nx = int(math.ceil((hix - lox) / res)) + 3
            ny = int(math.ceil((hiy - loy) / res)) + 3
            if nx * ny <= max_cells or res > 1.0:
                break
            res *= 1.25
        self.res = res
        self.nx = int(nx)
        self.ny = int(ny)
        self.n = self.nx * self.ny
        self.ox = lox - res
        self.oy = loy - res

        n = self.n
        self.logodds = np.zeros(n, np.float32)
        self.occ = np.zeros(n, bool)
        self.block = np.zeros(n, np.int32)
        self.pen = np.zeros(n, np.float32)
        self.extra = np.zeros(n, np.float32)
        self.cost = np.ones(n, np.float64)
        self.costl = [1.0] * n
        self.has_extra = False

        rad = int(math.ceil(PEN_R / res))
        hdx, hdy, pdx, pdy, pw = [], [], [], [], []
        for dy in range(-rad, rad + 1):
            for dx in range(-rad, rad + 1):
                d = math.hypot(dx, dy) * res
                if d <= PEN_R + 1e-9:
                    pdx.append(dx)
                    pdy.append(dy)
                    pw.append(PEN_W * (1.0 - d / PEN_R) ** 2)
                if d <= HARD_R + 1e-9:
                    hdx.append(dx)
                    hdy.append(dy)
        self.hdx = np.array(hdx, np.int64)
        self.hdy = np.array(hdy, np.int64)
        self.pdx = np.array(pdx, np.int64)
        self.pdy = np.array(pdy, np.int64)
        self.pw = np.array(pw, np.float32)
        self.d1x = np.array([0, 1, -1, 0, 0, 1, 1, -1, -1], np.int64)
        self.d1y = np.array([0, 0, 0, 1, -1, 1, -1, 1, -1], np.int64)

        self._ray_ds = np.arange(0.10, 12.0, 0.55 * res)

        # permanent wall around the arena so the search cannot leak out
        ax = np.arange(self.nx, dtype=np.int64)
        ay = np.arange(self.ny, dtype=np.int64)
        ring = np.concatenate([
            ax, ax + (self.ny - 1) * self.nx,
            ay * self.nx, ay * self.nx + (self.nx - 1),
        ])
        ring = np.unique(ring)
        self.block[ring] += 10000
        self.recompute(ring)

    # -- geometry ------------------------------------------------------------
    def index(self, x, y):
        ix = int((x - self.ox) / self.res)
        iy = int((y - self.oy) / self.res)
        if ix < 0:
            ix = 0
        elif ix >= self.nx:
            ix = self.nx - 1
        if iy < 0:
            iy = 0
        elif iy >= self.ny:
            iy = self.ny - 1
        return iy * self.nx + ix

    def center(self, idx):
        iy = idx // self.nx
        ix = idx - iy * self.nx
        return (self.ox + (ix + 0.5) * self.res,
                self.oy + (iy + 0.5) * self.res)

    def los(self, x0, y0, x1, y1):
        d = math.hypot(x1 - x0, y1 - y0)
        ns = int(d / (0.5 * self.res)) + 1
        blk = self.block
        for i in range(1, ns + 1):
            f = i / ns
            if blk[self.index(x0 + (x1 - x0) * f, y0 + (y1 - y0) * f)] > 0:
                return False
        return True

    def dilate1(self, cells):
        iy = cells // self.nx
        ix = cells - iy * self.nx
        jx = ix[:, None] + self.d1x[None, :]
        jy = iy[:, None] + self.d1y[None, :]
        ok = (jx >= 0) & (jx < self.nx) & (jy >= 0) & (jy < self.ny)
        return np.unique((jy * self.nx + jx)[ok])

    # -- cost ----------------------------------------------------------------
    def recompute(self, cells):
        if cells.size == 0:
            return cells
        blk = self.block[cells]
        pen = np.clip(self.pen[cells], 0.0, PEN_CAP)
        new = np.where(blk > 0, BLOCK_COST,
                       1.0 + pen.astype(np.float64) +
                       self.extra[cells].astype(np.float64))
        old = self.cost[cells]
        sel = np.abs(new - old) > 1e-3
        self.cost[cells] = new
        ch = cells[sel]
        cl = self.costl
        for c, v in zip(ch.tolist(), new[sel].tolist()):
            cl[c] = v
        return ch

    def _apply_kernel(self, cells, sign):
        iy = cells // self.nx
        ix = cells - iy * self.nx
        jx = ix[:, None] + self.hdx[None, :]
        jy = iy[:, None] + self.hdy[None, :]
        ok = (jx >= 0) & (jx < self.nx) & (jy >= 0) & (jy < self.ny)
        np.add.at(self.block, (jy * self.nx + jx)[ok], sign)
        jx = ix[:, None] + self.pdx[None, :]
        jy = iy[:, None] + self.pdy[None, :]
        ok = (jx >= 0) & (jx < self.nx) & (jy >= 0) & (jy < self.ny)
        tgt = (jy * self.nx + jx)[ok]
        w = np.broadcast_to(self.pw[None, :], jx.shape)[ok]
        np.add.at(self.pen, tgt, sign * w)
        return np.unique(tgt)

    def _transitions(self, touched):
        if touched.size == 0:
            return touched
        lo = self.logodds[touched]
        old = self.occ[touched]
        new = np.where(old, lo > FREE_T, lo > OCC_T)
        up = touched[new & ~old]
        dn = touched[old & ~new]
        if up.size == 0 and dn.size == 0:
            return np.empty(0, np.int64)
        self.occ[touched] = new
        aff = []
        if up.size:
            aff.append(self._apply_kernel(up, 1))
        if dn.size:
            aff.append(self._apply_kernel(dn, -1))
        return self.recompute(np.unique(np.concatenate(aff)))

    # -- sensor updates ------------------------------------------------------
    def update_scan(self, x, y, yaw, ranges, angles, rmax, trust):
        res = self.res
        r = np.asarray(ranges, dtype=np.float64)
        if r.size == 0:
            return np.empty(0, np.int64)
        bad = ~np.isfinite(r)
        rc = np.where(bad, rmax, np.clip(r, 0.0, rmax))
        a = np.asarray(angles, dtype=np.float64) + yaw
        ca = np.cos(a)
        sa = np.sin(a)

        ds = self._ray_ds[self._ray_ds < rmax]
        px = x + ds[None, :] * ca[:, None]
        py = y + ds[None, :] * sa[:, None]
        ix = np.floor((px - self.ox) / res).astype(np.int64)
        iy = np.floor((py - self.oy) / res).astype(np.int64)
        ok = ((ds[None, :] < (rc[:, None] - 0.6 * res)) &
              (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny))
        fidx = np.unique((iy * self.nx + ix)[ok])

        hit = (~bad) & (r > 0.10) & (r < rmax - 0.05)
        if np.any(hit):
            hxp = x + rc[hit] * ca[hit]
            hyp = y + rc[hit] * sa[hit]
            jx = np.floor((hxp - self.ox) / res).astype(np.int64)
            jy = np.floor((hyp - self.oy) / res).astype(np.int64)
            ok2 = (jx >= 0) & (jx < self.nx) & (jy >= 0) & (jy < self.ny)
            hidx = (jy * self.nx + jx)[ok2]
        else:
            hidx = np.empty(0, np.int64)

        lo = self.logodds
        if fidx.size:
            lo[fidx] -= np.float32(FREE_DEC * trust)
        if hidx.size:
            np.add.at(lo, hidx, np.float32(OCC_INC * trust))
        np.clip(lo, L_MIN, L_MAX, out=lo)

        if hidx.size:
            touched = np.union1d(fidx, np.unique(hidx))
        else:
            touched = fidx
        return self._transitions(touched)

    def mark_hit(self, x, y):
        idx = self.index(x, y)
        self.logodds[idx] = L_MAX
        return self._transitions(np.array([idx], np.int64))

    def add_penalty(self, x, y, radius, amount):
        idx = self.index(x, y)
        iy = idx // self.nx
        ix = idx - iy * self.nx
        rad = int(math.ceil(radius / self.res))
        xs = np.arange(-rad, rad + 1, dtype=np.int64)
        gx, gy = np.meshgrid(xs, xs)
        keep = (gx * gx + gy * gy) * (self.res ** 2) <= radius * radius
        jx = ix + gx[keep]
        jy = iy + gy[keep]
        ok = (jx >= 0) & (jx < self.nx) & (jy >= 0) & (jy < self.ny)
        cells = np.unique((jy * self.nx + jx)[ok])
        if cells.size == 0:
            return cells
        self.extra[cells] += np.float32(amount)
        self.has_extra = True
        return self.recompute(cells)

    def decay_penalty(self, factor):
        nz = np.nonzero(self.extra)[0]
        if nz.size == 0:
            self.has_extra = False
            return np.empty(0, np.int64)
        self.extra[nz] *= np.float32(factor)
        self.extra[nz[self.extra[nz] < 0.05]] = 0.0
        return self.recompute(nz)


# =============================================================================
# D* (Stentz 1994, Sec. 2.2).  One instance per shuttle endpoint; h() is the
# cost-to-that-endpoint field, b() the backpointers the robot follows.
# =============================================================================
class _DStar:
    NEW, OPEN, CLOSED = 0, 1, 2

    def __init__(self, n, nx, ny, goal_idx, res):
        self.n = n
        self.nx = nx
        self.ny = ny
        self.h = [0.0] * n
        self.k = [0.0] * n
        self.t = bytearray(n)
        self.b = [-1] * n
        self.open = []
        self.goal = int(goal_idx)
        self.offs = tuple((dx, dy, d * res) for dx, dy, d in _OFFS8)
        self.insert(self.goal, 0.0)

    # INSERT(X, h_new)
    def insert(self, x, hnew):
        tx = self.t[x]
        if tx == 0:
            kx = hnew
        elif tx == 1:
            kx = self.k[x]
            if hnew < kx:
                kx = hnew
        else:
            kx = self.h[x]
            if hnew < kx:
                kx = hnew
        self.k[x] = kx
        self.h[x] = hnew
        self.t[x] = 1
        heapq.heappush(self.open, (kx, x))

    # MIN-STATE / GET-KMIN with lazy deletion
    def min_state(self):
        op = self.open
        t = self.t
        k = self.k
        while op:
            kk, x = op[0]
            if t[x] != 1 or kk != k[x]:
                heapq.heappop(op)
                continue
            return kk, x
        return None

    # MODIFY-COST, line L2: seed cells whose arc costs changed
    def seed(self, cells):
        t = self.t
        h = self.h
        k = self.k
        op = self.open
        for c in cells:
            if t[c] == 2:
                hv = h[c]
                k[c] = hv
                t[c] = 1
                heapq.heappush(op, (hv, c))

    # PROCESS-STATE
    def process_state(self, cost):
        ms = self.min_state()
        if ms is None:
            return -1.0
        kold, x = heapq.heappop(self.open)
        h = self.h
        t = self.t
        b = self.b
        nx = self.nx
        ny = self.ny
        t[x] = 2
        iy = x // nx
        ix = x - iy * nx
        cx = cost[x]
        nb = []
        for dx, dy, d in self.offs:
            jx = ix + dx
            jy = iy + dy
            if 0 <= jx < nx and 0 <= jy < ny:
                yv = jy * nx + jx
                nb.append((yv, d * 0.5 * (cx + cost[yv])))
        hx = h[x]

        if kold < hx - EPS:                       # RAISE: try optimal nbrs (L4-L7)
            for yv, c in nb:
                if t[yv] != 0 and h[yv] <= kold + EPS and hx > h[yv] + c + EPS:
                    b[x] = yv
                    hx = h[yv] + c
            h[x] = hx

        if kold >= hx - EPS:                      # LOWER state (L8-L13)
            for yv, c in nb:
                hy = h[yv]
                nv = hx + c
                if (t[yv] == 0 or
                        (b[yv] == x and (hy > nv + EPS or hy < nv - EPS)) or
                        (b[yv] != x and hy > nv + EPS)):
                    b[yv] = x
                    self.insert(yv, nv)
        else:                                     # still RAISE (L14-L25)
            for yv, c in nb:
                hy = h[yv]
                nv = hx + c
                if t[yv] == 0 or (b[yv] == x and (hy > nv + EPS or hy < nv - EPS)):
                    b[yv] = x
                    self.insert(yv, nv)
                elif b[yv] != x and hy > nv + EPS:
                    self.insert(x, hx)
                elif (b[yv] != x and hx > hy + c + EPS and
                      t[yv] == 2 and hy > kold + EPS):
                    self.insert(yv, hy)
        ms = self.min_state()
        return -1.0 if ms is None else ms[0]

    # repair only until k_min >= h(robot)  (Sec. 2.2 / Property 2)
    def replan(self, ridx, cost, deadline):
        t = self.t
        h = self.h
        i = 0
        while True:
            ms = self.min_state()
            if ms is None:
                return
            if t[ridx] == 2 and ms[0] >= h[ridx] - EPS:
                return
            if self.process_state(cost) < 0.0:
                return
            i += 1
            if (i & 31) == 0 and time.perf_counter() > deadline:
                return

    # follow the backpointer sequence {X}
    def path(self, x, maxlen):
        out = []
        b = self.b
        seen = set()
        c = x
        for _ in range(maxlen):
            nb = b[c]
            if nb < 0 or nb in seen:
                break
            seen.add(nb)
            out.append(nb)
            if nb == self.goal:
                break
            c = nb
        return out


# =============================================================================
# helpers used by Controller.step (kept at module level so that Controller
# itself exposes exactly reset() and step())
# =============================================================================
def _ensure_map(ctl, x, y, gx, gy):
    if ctl.map is not None:
        return
    pts = list(ctl.bbox_pts) + [(x, y), (gx, gy)]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    span = max(max(xs) - min(xs), max(ys) - min(ys))
    margin = max(MARGIN, 0.55 * span)
    ctl.map = _GridMap(pts, margin, RES, MAX_CELLS)


def _build_dir_tables(ctl, angles):
    ctl.ang = np.asarray(angles, dtype=np.float64)
    ctl.cand = np.linspace(-math.pi, math.pi, NCAND + 1)[:-1]
    dth = ctl.ang[None, :] - ctl.cand[:, None]
    ctl.c_cos = np.cos(dth)
    ctl.c_sin = np.abs(np.sin(dth))
    ctl.i_fwd = int(np.argmin(np.abs(_wrap_arr(ctl.cand - 0.0))))
    ctl.i_left = int(np.argmin(np.abs(_wrap_arr(ctl.cand - 0.5 * math.pi))))
    ctl.i_right = int(np.argmin(np.abs(_wrap_arr(ctl.cand + 0.5 * math.pi))))
    ctl.i_back = int(np.argmin(np.abs(_wrap_arr(ctl.cand - math.pi))))


def _wrap_arr(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _clearances(ctl, ranges, rmax):
    r = np.asarray(ranges, dtype=np.float64)
    r = np.where(np.isfinite(r), np.clip(r, 0.0, rmax), rmax)
    along = ctl.c_cos * r[None, :]
    lat = ctl.c_sin * r[None, :]
    a = np.where((lat < W_HALF) & (along > 0.0), along, rmax + 1.0)
    return np.minimum(a.min(axis=1), rmax)


def _seed_all(ctl, changed):
    if changed is None or len(changed) == 0:
        return
    cells = ctl.map.dilate1(np.asarray(changed, dtype=np.int64)).tolist()
    for pl, _ in ctl.planners.values():
        pl.seed(cells)


def _planner_for(ctl, gx, gy, t):
    m = ctl.map
    key = (int(round(gx / m.res)), int(round(gy / m.res)))
    if key != ctl.cur_key:
        ctl.cur_key = key
        ctl.goal_switch_t = t
        ctl.hist.clear()
        ctl.stuck_count = 0
    ent = ctl.planners.get(key)
    if ent is None:
        if len(ctl.planners) >= 3:
            worst = min(ctl.planners.items(), key=lambda kv: kv[1][1])[0]
            if worst != key:
                del ctl.planners[worst]
        pl = _DStar(m.n, m.nx, m.ny, m.index(gx, gy), m.res)
        ctl.planners[key] = [pl, t]
        return pl
    ent[1] = t
    return ent[0]


def _carrot(ctl, pl, ridx, x, y, gx, gy):
    m = ctl.map
    gd = math.hypot(gx - x, gy - y)
    if gd < LOOK and m.los(x, y, gx, gy):
        return gx, gy, True
    if pl.t[ridx] == 0:
        return gx, gy, False              # cold start -> Lumelsky fallback
    path = pl.path(ridx, 120)
    if not path:
        return gx, gy, False
    best = None
    first = None
    for c in path:
        px, py = m.center(c)
        d = math.hypot(px - x, py - y)
        if first is None and d > 0.30:
            first = (px, py)
        if d > LOOK:
            break
        if m.los(x, y, px, py):
            best = (px, py)
    if best is not None:
        return best[0], best[1], True
    if first is not None:
        return first[0], first[1], True
    return gx, gy, False


def _choose_dir(ctl, cl, bearing, goal_dist):
    cand = ctl.cand
    w_align = 2.2 + 2.0 * max(0.0, 1.0 - goal_dist)
    score = (w_align * np.cos(cand - bearing) +
             1.7 * np.clip((cl - STOP_D) / 1.5, 0.0, 1.0) +
             0.55 * np.cos(cand - ctl.prev_dir_r))
    score -= np.where(cl < MIN_CLEAR, 6.0, 0.0)
    score -= 0.30 * np.clip(-np.cos(cand), 0.0, 1.0)
    i = int(np.argmax(score))
    return float(cand[i]), float(cl[i])


def _start_recovery(ctl, t, cl):
    ctl.rec_until = t + 1.4
    ctl.rec_start = t
    left = float(np.mean(cl[ctl.i_left - 9:ctl.i_left + 10]))
    right = float(np.mean(cl[ctl.i_right - 9:ctl.i_right + 10]))
    ctl.rec_turn = 1.0 if left >= right else -1.0


def _emit(ctl, vx, vy, wz):
    ml = ctl.max_lin
    ma = ctl.max_ang
    vx = max(-ml, min(ml, vx))
    vy = max(-ml, min(ml, vy))
    wz = max(-ma, min(ma, wz))
    px, py, pw = ctl.last_cmd
    dv = 0.055
    dw = 0.16
    vx = px + max(-dv, min(dv, vx - px))
    vy = py + max(-dv, min(dv, vy - py))
    wz = pw + max(-dw, min(dw, wz - pw))
    ctl.last_cmd = (vx, vy, wz)
    return Command(float(vx), float(vy), float(wz))


def _drive(ctl, obs):
    t = float(obs.t)
    pe = obs.pose_estimate
    x = float(pe[0])
    y = float(pe[1])
    yaw = float(pe[2])
    g = obs.goal
    gx = float(g[0])
    gy = float(g[1])
    try:
        perr = float(obs.pose_error_estimate)
        if not math.isfinite(perr):
            perr = 0.0
    except Exception:
        perr = 0.0
    ctl.n += 1

    _ensure_map(ctl, x, y, gx, gy)
    m = ctl.map
    if ctl.ang is None or len(ctl.ang) != len(obs.ranges):
        _build_dir_tables(ctl, obs.range_angles)
    rmax = ctl.range_max
    try:
        rm = float(obs.range_max)
        if math.isfinite(rm) and rm > 0.5:
            rmax = rm
    except Exception:
        pass

    trust = min(1.0, max(0.15, 1.0 / (1.0 + 2.0 * max(0.0, perr))))
    cl = _clearances(ctl, obs.ranges, rmax)

    # ---- sensor -> map -> MODIFY-COST ------------------------------------
    if ctl.n % ctl.scan_every == 0 or ctl.n <= 3:
        _seed_all(ctl, m.update_scan(x, y, yaw, obs.ranges, ctl.ang, rmax, trust))
    contact = False
    try:
        contact = bool(obs.contact)
    except Exception:
        contact = False
    if contact:
        _seed_all(ctl, m.mark_hit(x + 0.32 * math.cos(yaw),
                                  y + 0.32 * math.sin(yaw)))
        if t > ctl.rec_until and t - ctl.rec_start > 1.2:
            _start_recovery(ctl, t, cl)
    if ctl.n % 1000 == 0 and m.has_extra:
        _seed_all(ctl, m.decay_penalty(0.5))

    # ---- D* repair -------------------------------------------------------
    pl = _planner_for(ctl, gx, gy, t)
    ridx = m.index(x, y)
    if pl.t[ridx] == 0:
        budget = 0.30
    elif t - ctl.goal_switch_t < 1.0:
        budget = 0.030
    else:
        budget = 0.011
    pl.replan(ridx, m.costl, time.perf_counter() + budget)

    goal_dist = math.hypot(gx - x, gy - y)

    # ---- stuck detection / raised costs on traps (Sec. 5.2) --------------
    if ctl.n % 10 == 0:
        ctl.hist.append((t, x, y))
    if (t > ctl.rec_until + 0.8 and goal_dist > 0.7 and
            len(ctl.hist) == ctl.hist.maxlen):
        t0, x0, y0 = ctl.hist[0]
        if t - t0 > 3.0 and math.hypot(x - x0, y - y0) < 0.30:
            _start_recovery(ctl, t, cl)
            ctl.stuck_count += 1
            _seed_all(ctl, m.add_penalty(x, y, 0.50,
                                         2.5 + 1.5 * min(4, ctl.stuck_count)))
            ctl.hist.clear()

    # ---- recovery behaviour ---------------------------------------------
    if t < ctl.rec_until:
        rem = ctl.rec_until - t
        back = min(cl[ctl.i_back], cl[(ctl.i_back + 1) % NCAND],
                   cl[ctl.i_back - 1])
        if rem > 0.85 and back > 0.55:
            return _emit(ctl, -0.34, 0.0, 0.30 * ctl.rec_turn)
        return _emit(ctl, 0.0, 0.0, 0.95 * ctl.rec_turn)

    # ---- carrot + reactive direction -------------------------------------
    cxp, cyp, have = _carrot(ctl, pl, ridx, x, y, gx, gy)
    bearing = _wrap(math.atan2(cyp - y, cxp - x) - yaw)
    ctl.prev_dir_r = _wrap(ctl.prev_dir_w - yaw)
    # when localisation is poor, lean harder on live clearance than on the map
    if perr > 0.8:
        bearing = _wrap(0.7 * bearing + 0.3 * _wrap(
            math.atan2(gy - y, gx - x) - yaw))
    phi, clear_phi = _choose_dir(ctl, cl, bearing, goal_dist)
    ctl.prev_dir_w = _wrap(phi + yaw)

    # ---- velocity --------------------------------------------------------
    v_cap = ctl.max_lin * min(1.0, max(0.0, (clear_phi - STOP_D) / 0.90))
    gate = max(0.0, 1.0 - abs(phi) / 1.30)
    vx = v_cap * gate
    vy = 0.0
    if abs(phi) < 1.30:
        vy = max(-0.35, min(0.35, 0.55 * v_cap * math.sin(phi)))
    wz = 2.4 * phi

    fwd = min(cl[ctl.i_fwd], cl[ctl.i_fwd - 1], cl[(ctl.i_fwd + 1) % NCAND])
    if fwd < 0.33:
        vx = min(vx, -0.10)
        vy *= 0.5
    if vy > 0.0 and cl[ctl.i_left] < 0.45:
        vy = 0.0
    if vy < 0.0 and cl[ctl.i_right] < 0.45:
        vy = 0.0
    if perr > 1.5:
        vx *= 0.75
        vy *= 0.75
    return _emit(ctl, vx, vy, wz)


# =============================================================================
class Controller:
    def reset(self, scenario, rng):
        """Clear ALL episode state."""
        self.rng = rng
        gf = getattr
        self.range_max = float(gf(scenario, "range_max", 6.0) or 6.0)
        self.max_lin = float(gf(scenario, "max_lin", 0.8) or 0.8)
        self.max_ang = float(gf(scenario, "max_ang", 1.0) or 1.0)
        self.time_limit = float(gf(scenario, "time_limit", 1.0e9) or 1.0e9)
        st = gf(scenario, "start", (0.0, 0.0))
        gl = gf(scenario, "goal", (0.0, 0.0))
        try:
            self.bbox_pts = [(float(st[0]), float(st[1])),
                             (float(gl[0]), float(gl[1]))]
        except Exception:
            self.bbox_pts = [(0.0, 0.0)]

        self.map = None
        self.planners = {}
        self.cur_key = None
        self.goal_switch_t = -1.0e9

        self.ang = None
        self.cand = None
        self.c_cos = None
        self.c_sin = None
        self.i_fwd = 0
        self.i_left = 0
        self.i_right = 0
        self.i_back = 0

        self.n = 0
        self.scan_every = 2
        self.prev_dir_w = 0.0
        self.prev_dir_r = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.hist = deque(maxlen=20)
        self.rec_until = -1.0e9
        self.rec_start = -1.0e9
        self.rec_turn = 1.0
        self.stuck_count = 0

    def step(self, obs):
        """One 50 Hz control step."""
        try:
            return _drive(self, obs)
        except Exception:
            try:
                vx, vy, wz = self.last_cmd
                return Command(float(0.5 * vx), 0.0, float(0.5 * wz))
            except Exception:
                return Command(0.0, 0.0, 0.0)
