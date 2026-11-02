"""
Navigation controller for a Unitree Go2 shuttling between two fixed points
in an unmapped indoor building.

Architecture (all above the frozen locomotion policy, output is a Twist):

  1. Occupancy mapping .... a log-odds grid in the *estimated* pose frame,
     built from the 181-beam lidar with proper ray-clearing.  Because the
     robot covers the same building over and over, the map paid for on trip
     one is pure profit on every trip after it.
  2. Global planning ...... Dijkstra wavefront from the goal over a coarse
     (2x) grid derived from the fine map, producing a navigation function
     with no local minima.  Unknown space is traversable but slightly
     expensive, so the very first trip degenerates gracefully into
     "optimistically head for the goal and re-plan as walls appear".
     The search is *amortised* across control steps with a fixed node
     budget, so step() stays cheap and bounded.
  3. Local reactive layer . VFH-style candidate-direction scoring directly
     on the raw lidar, blending the planner's carrot direction with real
     clearance.  This is what actually squeezes through doorways.
  4. Recovery ............. stall / contact detection, back-off + spin, and
     a decaying penalty layer that steers later plans away from places
     that trapped us.
  5. Goal dither .......... if we are sitting on the *estimated* goal and
     the harness has not switched goals, the pose estimate is biased, so
     spiral outward until arrival registers.  This is the single biggest
     protection against losing an entire episode to drift.

Only stdlib + numpy + core.observation.Command are used.
"""

import heapq
import math

import numpy as np

from core.observation import Command

TWO_PI = 2.0 * math.pi
INF = float("inf")


def _wrap(a):
    """Wrap a scalar angle to (-pi, pi]."""
    return (a + math.pi) % TWO_PI - math.pi


def _wrap_arr(a):
    """Wrap an array of angles to (-pi, pi]."""
    return (a + math.pi) % TWO_PI - math.pi


def _dil3(a):
    """One 3x3 (Chebyshev radius 1) binary dilation, separably."""
    b = a.copy()
    b[:, :-1] |= a[:, 1:]
    b[:, 1:] |= a[:, :-1]
    c = b.copy()
    c[:-1, :] |= b[1:, :]
    c[1:, :] |= b[:-1, :]
    return c


class Controller:
    # ---------------- map ----------------
    RES = 0.15            # fine occupancy resolution [m]
    MARGIN = 14.0         # padding around the start/goal bounding box [m]
    MAX_N = 480           # cap on fine grid side length (cells)

    OCC_TH = 0.50         # log-odds above this -> occupied
    FREE_TH = -0.40       # log-odds below this -> known free
    LO_MIN = -2.5
    LO_MAX = 3.5
    L_HIT = 0.85
    L_MISS = 0.40

    # inflation radii, in fine cells (Chebyshev)
    K_LETH = 2            # ~0.30 m : untraversable for the planner
    K_NEAR = 4            # ~0.60 m : strongly discouraged
    K_MID = 7             # ~1.05 m : mildly discouraged

    # ---------------- planner ----------------
    PLAN_BUDGET = 900     # Dijkstra pops per control step
    PLAN_PERIOD = 1.0     # seconds between replans
    PEN_DECAY = 0.93
    PEN_MAX = 24.0

    # ---------------- local ----------------
    CLEAR_HALF = 0.33     # half-corridor width the body needs [m]
    BODY_MARGIN = 0.12
    LOOKAHEAD = 1.10

    W_HEAD = 2.40
    W_CLEAR = 1.70
    W_PREV = 0.55
    MIN_CLEAR = 0.32

    # =====================================================================
    # lifecycle
    # =====================================================================
    def reset(self, scenario, rng):
        self.rng = rng

        self.max_lin = self._pos(getattr(scenario, "max_lin", 0.8), 0.8)
        self.max_ang = self._pos(getattr(scenario, "max_ang", 1.0), 1.0)
        self.range_max = self._pos(getattr(scenario, "range_max", 6.0), 6.0)
        self.time_limit = self._pos(getattr(scenario, "time_limit", 1.0e6), 1.0e6)

        p0 = self._xy(getattr(scenario, "start", (0.0, 0.0)))
        p1 = self._xy(getattr(scenario, "goal", (0.0, 0.0)))

        # ---- grid geometry -------------------------------------------------
        lo_x = min(p0[0], p1[0]) - self.MARGIN
        hi_x = max(p0[0], p1[0]) + self.MARGIN
        lo_y = min(p0[1], p1[1]) - self.MARGIN
        hi_y = max(p0[1], p1[1]) + self.MARGIN
        span = max(hi_x - lo_x, hi_y - lo_y, 4.0)

        res = self.RES
        while span / res > self.MAX_N:
            res *= 1.2
        n = int(math.ceil(span / res)) + 4
        if n % 2:
            n += 1
        self.res = res
        self.N = n
        self.ox = 0.5 * (lo_x + hi_x) - 0.5 * n * res
        self.oy = 0.5 * (lo_y + hi_y) - 0.5 * n * res

        self.lo = np.zeros(n * n, dtype=np.float32)
        self.lo2 = self.lo.reshape(n, n)          # [row = y, col = x]
        self.bb = None                            # touched bbox [r0,r1,c0,c1]

        self.cn = n // 2
        self.res_c = 2.0 * res
        self.pen = np.zeros(self.cn * self.cn, dtype=np.float32)

        cn = self.cn
        self._offs = []
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                if di == 0 and dj == 0:
                    continue
                w = self.res_c * (1.4142135623730951 if (di and dj) else 1.0)
                if di and dj:
                    self._offs.append((di * cn + dj, w, di * cn, dj))
                else:
                    self._offs.append((di * cn + dj, w, 0, 0))

        # ray-cast sample distances
        self._ds = np.arange(0.5 * res, self.range_max, 0.7 * res, dtype=np.float64)
        if self._ds.size == 0:
            self._ds = np.array([0.5 * res])

        # ---- candidate directions -----------------------------------------
        self.cand = np.arange(-180.0, 180.0, 5.0) * (math.pi / 180.0)
        self._ang_ref = None
        self._cs = None
        self._sn = None
        self._ca0 = None
        self._sa0 = None

        # ---- episode state --------------------------------------------------
        self.k = 0
        self.t_prev = None
        self.last_pose = None
        self.skip_map = 0

        self.goal_xy = None
        self.trips = 0

        self.field = None                 # committed navigation function (list)
        self.pl_active = False
        self.heap = []
        self.pd = None
        self.pc = None
        self.pl_stop = None
        self.last_plan_t = -1.0e9
        self.force_replan = True

        self.prev_dir = 0.0
        self.hist = []
        self.mode = "nav"
        self.mode_until = -1.0
        self.mode_sign = 1.0
        self.contact_t = 0.0
        self.stuck_count = 0

        self.near_goal_t = None
        self.dither_phase = 0.0

    # =====================================================================
    # main entry point
    # =====================================================================
    def step(self, obs):
        try:
            vx, vy, wz = self._step(obs)
            if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
                vx, vy, wz = 0.0, 0.0, 0.5
        except Exception:
            vx, vy, wz = 0.0, 0.0, 0.5

        ml = self.max_lin if hasattr(self, "max_lin") else 0.8
        ma = self.max_ang if hasattr(self, "max_ang") else 1.0
        vx = max(-ml, min(ml, vx))
        vy = max(-ml, min(ml, vy))
        wz = max(-ma, min(ma, wz))
        return Command(float(vx), float(vy), float(wz))

    # ---------------------------------------------------------------------
    def _step(self, obs):
        t = float(obs.t)
        if self.t_prev is None:
            dt = 0.02
        else:
            dt = min(0.25, max(0.001, t - self.t_prev))
        self.t_prev = t
        self.k += 1

        px, py, yaw = self._pose(obs.pose_estimate)
        gx, gy = self._xy(obs.goal)
        perr = self._fl(getattr(obs, "pose_error_estimate", 0.0))
        contact = bool(getattr(obs, "contact", False))

        rr = np.asarray(obs.ranges, dtype=np.float64).ravel()
        ang = self._angles(obs.range_angles, rr.size)
        finite = np.isfinite(rr)
        r = np.where(finite, rr, self.range_max)
        r = np.clip(r, 0.05, self.range_max)

        self._cache_beams(ang)

        # ---- goal switch ---------------------------------------------------
        if self.goal_xy is None or math.hypot(gx - self.goal_xy[0], gy - self.goal_xy[1]) > 0.6:
            if self.goal_xy is not None:
                self.trips += 1
            self.goal_xy = (gx, gy)
            self.force_replan = True
            self.near_goal_t = None
            self.dither_phase = float(self.rng.uniform(0.0, TWO_PI))
            self.hist = []
            self.mode = "nav"

        # ---- pose jump guard -----------------------------------------------
        if self.last_pose is not None:
            if math.hypot(px - self.last_pose[0], py - self.last_pose[1]) > 0.6:
                self.skip_map = 15
                self.force_replan = True
        self.last_pose = (px, py)

        # ---- mapping -------------------------------------------------------
        if self.skip_map > 0:
            self.skip_map -= 1
        elif (self.k & 1) == 0:
            self._update_map(px, py, yaw, r, finite, perr)

        # ---- per-direction clearance from raw lidar ------------------------
        free = self._clearance(r)

        # ---- goal geometry / dither ----------------------------------------
        dgoal = math.hypot(gx - px, gy - py)
        tx, ty = gx, gy
        if dgoal < 0.55:
            if self.near_goal_t is None:
                self.near_goal_t = t
        elif dgoal > 1.3:
            self.near_goal_t = None

        dithering = False
        if self.near_goal_t is not None and (t - self.near_goal_t) > 3.0:
            dithering = True
            age = t - self.near_goal_t - 3.0
            rad = min(0.35 + 0.22 * age, max(1.0, 1.0 + 2.0 * perr), 2.2)
            self.dither_phase += 1.5 * dt
            tx = gx + rad * math.cos(self.dither_phase)
            ty = gy + rad * math.sin(self.dither_phase)

        # ---- stall / contact detection -------------------------------------
        if contact:
            self.contact_t += dt
        else:
            self.contact_t = max(0.0, self.contact_t - 2.0 * dt)

        self.hist.append((t, px, py))
        while self.hist and (t - self.hist[0][0]) > 3.0:
            self.hist.pop(0)

        stuck = False
        if (not dithering) and len(self.hist) > 40 and (t - self.hist[0][0]) > 2.6:
            if math.hypot(px - self.hist[0][1], py - self.hist[0][2]) < 0.30:
                stuck = True

        if self.mode == "nav" and (stuck or self.contact_t > 0.45):
            self.stuck_count += 1
            self._add_penalty(px, py, 4.0)
            self.mode = "back"
            self.mode_until = t + 0.55 + 0.35 * float(self.rng.random())
            self.mode_sign = 1.0 if self.rng.random() < 0.5 else -1.0
            self.force_replan = True
            self.hist = []
            self.contact_t = 0.0

        # ---- recovery behaviours -------------------------------------------
        if self.mode != "nav":
            if t >= self.mode_until:
                if self.mode == "back":
                    self.mode = "spin"
                    self.mode_until = t + 0.40 + 0.5 * float(self.rng.random())
                else:
                    self.mode = "nav"
                    self.hist = []
            if self.mode == "back":
                j = int(np.argmin(r))
                esc = _wrap(float(ang[j]) + math.pi)
                sc = free + 0.7 * np.cos(_wrap_arr(self.cand - esc))
                bi = int(np.argmax(sc))
                d = float(self.cand[bi])
                sp = 0.40 * self.max_lin
                return (sp * math.cos(d), 0.7 * sp * math.sin(d),
                        self.mode_sign * 0.5 * self.max_ang)
            if self.mode == "spin":
                return (0.04 * self.max_lin, 0.0, self.mode_sign * 0.85 * self.max_ang)

        # ---- global planning (amortised) -----------------------------------
        rc = self._cell_c(px, py)
        if (not self.pl_active) and (self.force_replan or
                                     (t - self.last_plan_t) > self.PLAN_PERIOD):
            self._plan_start(t, gx, gy)
        if self.pl_active:
            self._plan_run(rc)

        # ---- desired heading (robot frame) ---------------------------------
        theta_des = _wrap(math.atan2(ty - py, tx - px) - yaw)
        if (not dithering) and dgoal > 0.9 and self.field is not None:
            car = self._carrot(px, py, rc)
            if car is not None:
                theta_des = _wrap(math.atan2(car[1] - py, car[0] - px) - yaw)

        # ---- reactive direction selection ----------------------------------
        head = np.cos(_wrap_arr(self.cand - theta_des))
        prev = np.cos(_wrap_arr(self.cand - self.prev_dir))
        clr = np.minimum(free, 2.5) * 0.4
        score = self.W_HEAD * head + self.W_CLEAR * clr + self.W_PREV * prev
        score = np.where(free < self.MIN_CLEAR, score - 50.0, score)

        bi = int(np.argmax(score))
        c_best = float(free[bi])

        if c_best < self.MIN_CLEAR:
            # boxed in: creep toward the roomiest direction while turning
            bi = int(np.argmax(free))
            d = float(self.cand[bi])
            self.prev_dir = d
            sgn = 1.0 if d >= 0.0 else -1.0
            return (0.10 * self.max_lin * math.cos(d),
                    0.30 * self.max_lin * math.sin(d),
                    sgn * 0.85 * self.max_ang)

        d = float(self.cand[bi])
        self.prev_dir = math.atan2(
            0.6 * math.sin(self.prev_dir) + 0.4 * math.sin(d),
            0.6 * math.cos(self.prev_dir) + 0.4 * math.cos(d),
        )

        # ---- speed ----------------------------------------------------------
        v = self.max_lin * min(1.0, max(0.12, (c_best - 0.28) / 1.15))
        if perr > 0.8:
            v *= 0.85
        if (not dithering) and dgoal < 0.8:
            v = min(v, self.max_lin * max(0.20, 0.75 * dgoal))

        wz = 2.2 * d
        wz = max(-self.max_ang, min(self.max_ang, wz))
        v *= (1.0 - 0.25 * abs(wz) / self.max_ang)

        vx = v * math.cos(d)
        vy = 0.72 * v * math.sin(d)
        if vx < 0.0:
            vx = max(vx, -0.50 * self.max_lin)
        lat_cap = 0.65 * self.max_lin
        vy = max(-lat_cap, min(lat_cap, vy))

        # ---- short-range repulsion -----------------------------------------
        close = r < 0.9
        if bool(np.any(close)):
            w = (1.0 / np.maximum(r[close], 0.15)) - (1.0 / 0.9)
            rx = -float(np.sum(w * self._ca0[close]))
            ry = -float(np.sum(w * self._sa0[close]))
            nrm = math.hypot(rx, ry)
            if nrm > 1e-6:
                g = min(0.25 * self.max_lin, 0.015 * nrm)
                vx += g * rx / nrm
                vy += g * ry / nrm

        return (vx, vy, wz)

    # =====================================================================
    # mapping
    # =====================================================================
    def _cache_beams(self, ang):
        if (self._ang_ref is None or self._ang_ref.size != ang.size
                or not np.array_equal(self._ang_ref, ang)):
            self._ang_ref = ang.copy()
            self._ca0 = np.cos(ang)
            self._sa0 = np.sin(ang)
            diff = self.cand[:, None] - ang[None, :]
            self._cs = np.cos(diff)
            self._sn = np.abs(np.sin(diff))

    def _clearance(self, r):
        fwd = self._cs * r[None, :]
        lat = self._sn * r[None, :]
        blk = (fwd > 0.0) & (lat < self.CLEAR_HALF)
        fw = np.where(blk, fwd, 99.0)
        free = fw.min(axis=1) - self.BODY_MARGIN
        return np.clip(free, 0.0, self.range_max)

    def _update_map(self, px, py, yaw, r, finite, perr):
        n = self.N
        res = self.res
        w = float(np.clip(1.0 - 0.5 * perr, 0.30, 1.0))
        cy_ = math.cos(yaw)
        sy_ = math.sin(yaw)
        ca = self._ca0 * cy_ - self._sa0 * sy_
        sa = self._sa0 * cy_ + self._ca0 * sy_

        ds = self._ds
        lim = r - (0.5 * res + 0.08)
        m = ds[None, :] < lim[:, None]
        if bool(m.any()):
            X = px + np.outer(ca, ds)
            Y = py + np.outer(sa, ds)
            ix = np.floor((X - self.ox) / res).astype(np.int32)
            iy = np.floor((Y - self.oy) / res).astype(np.int32)
            ok = m & (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
            if bool(ok.any()):
                idx = (iy * n + ix)[ok]
                vals = self.lo[idx] - (self.L_MISS * w)
                self.lo[idx] = np.maximum(vals, self.LO_MIN)
                self._touch(idx)

        hit = finite & (r < self.range_max - 0.12)
        if bool(hit.any()):
            hx = px + ca[hit] * r[hit]
            hy = py + sa[hit] * r[hit]
            jx = np.floor((hx - self.ox) / res).astype(np.int32)
            jy = np.floor((hy - self.oy) / res).astype(np.int32)
            ok = (jx >= 0) & (jx < n) & (jy >= 0) & (jy < n)
            if bool(ok.any()):
                idx = (jy * n + jx)[ok]
                vals = self.lo[idx] + (self.L_HIT * w)
                self.lo[idx] = np.minimum(vals, self.LO_MAX)
                self._touch(idx)

    def _touch(self, idx):
        n = self.N
        mn = int(idx.min())
        mx = int(idx.max())
        cols = idx % n
        r0 = mn // n
        r1 = mx // n
        c0 = int(cols.min())
        c1 = int(cols.max())
        if self.bb is None:
            self.bb = [r0, r1, c0, c1]
        else:
            b = self.bb
            if r0 < b[0]:
                b[0] = r0
            if r1 > b[1]:
                b[1] = r1
            if c0 < b[2]:
                b[2] = c0
            if c1 > b[3]:
                b[3] = c1

    # =====================================================================
    # cost construction + planning
    # =====================================================================
    def _build_cost(self):
        n = self.N
        cost = np.full((n, n), 1.85, dtype=np.float32)
        blocked = np.zeros((n, n), dtype=bool)
        if self.bb is None:
            return cost, blocked

        pad = self.K_MID + 1
        i0 = max(0, self.bb[0] - pad)
        i1 = min(n, self.bb[1] + 1 + pad)
        j0 = max(0, self.bb[2] - pad)
        j1 = min(n, self.bb[3] + 1 + pad)
        if i1 - i0 < 3 or j1 - j0 < 3:
            return cost, blocked

        lo = self.lo2[i0:i1, j0:j1]
        occ = lo > self.OCC_TH
        fre = lo < self.FREE_TH
        unk = ~occ & ~fre

        d = occ
        leth = occ
        near = occ
        for k in range(1, self.K_MID + 1):
            d = _dil3(d)
            if k == self.K_LETH:
                leth = d.copy()
            if k == self.K_NEAR:
                near = d.copy()
        mid = d

        c = np.ones(lo.shape, dtype=np.float32)
        c += 0.85 * unk
        c += 0.70 * mid
        c += 1.90 * near
        cost[i0:i1, j0:j1] = c
        blocked[i0:i1, j0:j1] = leth
        return cost, blocked

    def _plan_start(self, t, gx, gy):
        self.force_replan = False
        self.last_plan_t = t
        self.pen *= self.PEN_DECAY

        cost2, blk2 = self._build_cost()
        cn = self.cn
        big = np.float32(24.0)
        cc = np.where(blk2, big, cost2)
        coarse = cc.reshape(cn, 2, cn, 2).min(axis=(1, 3))
        cblk = blk2.reshape(cn, 2, cn, 2).all(axis=(1, 3))

        coarse = coarse.astype(np.float64) + self.pen.reshape(cn, cn).astype(np.float64)
        cblk[0, :] = True
        cblk[-1, :] = True
        cblk[:, 0] = True
        cblk[:, -1] = True

        cf = coarse.ravel()
        cf[cblk.ravel()] = INF

        gi, gj = self._cell_c_ij(gx, gy)
        start = self._nearest_free_c(gi, gj, cblk)
        if start is None:
            self.pl_active = False
            return

        nn = cn * cn
        self.pc = cf.tolist()
        self.pd = [INF] * nn
        self.pd[start] = 0.0
        self.heap = [(0.0, start)]
        self.pl_stop = None
        self.pl_active = True

    def _nearest_free_c(self, gi, gj, cblk):
        cn = self.cn
        gi = max(0, min(cn - 1, gi))
        gj = max(0, min(cn - 1, gj))
        for rad in range(0, 16):
            i0 = max(1, gi - rad)
            i1 = min(cn - 1, gi + rad + 1)
            j0 = max(1, gj - rad)
            j1 = min(cn - 1, gj + rad + 1)
            if i0 >= i1 or j0 >= j1:
                continue
            sub = ~cblk[i0:i1, j0:j1]
            if bool(sub.any()):
                ii, jj = np.nonzero(sub)
                d2 = (ii + i0 - gi) ** 2 + (jj + j0 - gj) ** 2
                k = int(np.argmin(d2))
                return int((ii[k] + i0) * cn + (jj[k] + j0))
        return None

    def _plan_run(self, rc):
        h = self.heap
        d = self.pd
        c = self.pc
        offs = self._offs
        pop = heapq.heappop
        push = heapq.heappush
        budget = self.PLAN_BUDGET
        stop = self.pl_stop
        pops = 0
        done = False

        while h and pops < budget:
            du, u = pop(h)
            if du > d[u]:
                continue
            if stop is not None and du > stop:
                done = True
                break
            pops += 1
            for (dd, w, o1, o2) in offs:
                v = u + dd
                cv = c[v]
                if cv == INF:
                    continue
                if o1 and (c[u + o1] == INF or c[u + o2] == INF):
                    continue
                nd = du + cv * w
                if nd < d[v]:
                    d[v] = nd
                    push(h, (nd, v))

        if not h:
            done = True

        if 0 <= rc < len(d):
            dr = d[rc]
            if dr < INF:
                self.pl_stop = dr + 4.0
            else:
                self.pl_stop = stop

        if done:
            self.field = d
            self.pl_active = False
            self.heap = []

    # =====================================================================
    # path following
    # =====================================================================
    def _carrot(self, px, py, rc):
        d = self.field
        if d is None:
            return None
        nn = self.cn * self.cn
        if rc < 0 or rc >= nn:
            return None
        if d[rc] == INF:
            self.force_replan = True
            return None

        offs = self._offs
        u = rc
        prevx, prevy = px, py
        total = 0.0
        look = self.LOOKAHEAD
        cx, cy = self._c_center(u)
        for _ in range(220):
            bv = d[u]
            bu = -1
            for (dd, w, o1, o2) in offs:
                v = u + dd
                if v < 0 or v >= nn:
                    continue
                dv = d[v]
                if dv < bv:
                    bv = dv
                    bu = v
            if bu < 0:
                break
            u = bu
            cx, cy = self._c_center(u)
            total += math.hypot(cx - prevx, cy - prevy)
            prevx, prevy = cx, cy
            if total >= look:
                return (cx, cy)
        if total > 1e-6:
            return (prevx, prevy)
        return None

    def _add_penalty(self, px, py, amt):
        i, j = self._cell_c_ij(px, py)
        cn = self.cn
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                ii = i + di
                jj = j + dj
                if 0 <= ii < cn and 0 <= jj < cn:
                    k = ii * cn + jj
                    self.pen[k] = min(self.PEN_MAX, float(self.pen[k]) + amt)

    # =====================================================================
    # small helpers
    # =====================================================================
    def _cell_c_ij(self, x, y):
        cn = self.cn
        j = int(math.floor((x - self.ox) / self.res_c))
        i = int(math.floor((y - self.oy) / self.res_c))
        return (max(0, min(cn - 1, i)), max(0, min(cn - 1, j)))

    def _cell_c(self, x, y):
        i, j = self._cell_c_ij(x, y)
        return i * self.cn + j

    def _c_center(self, u):
        cn = self.cn
        i = u // cn
        j = u - i * cn
        return (self.ox + (j + 0.5) * self.res_c,
                self.oy + (i + 0.5) * self.res_c)

    def _angles(self, a, m):
        try:
            arr = np.asarray(a, dtype=np.float64).ravel()
        except Exception:
            arr = np.zeros(0)
        if arr.size != m or m == 0:
            arr = np.linspace(-math.pi, math.pi, max(m, 1), endpoint=False)
        return arr

    @staticmethod
    def _fl(v):
        try:
            f = float(v)
        except Exception:
            return 0.0
        return f if math.isfinite(f) else 0.0

    @staticmethod
    def _pos(v, default):
        try:
            f = float(v)
        except Exception:
            return default
        if not math.isfinite(f) or f <= 0.0:
            return default
        return f

    @staticmethod
    def _xy(p):
        try:
            a = np.asarray(p, dtype=np.float64).ravel()
            return (float(a[0]), float(a[1]))
        except Exception:
            return (0.0, 0.0)

    @staticmethod
    def _pose(p):
        try:
            a = np.asarray(p, dtype=np.float64).ravel()
            x = float(a[0])
            y = float(a[1])
            th = float(a[2]) if a.size > 2 else 0.0
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(th)):
                return (0.0, 0.0, 0.0)
            return (x, y, th)
        except Exception:
            return (0.0, 0.0, 0.0)
