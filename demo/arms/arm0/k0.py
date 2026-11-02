"""Map-building shuttle navigator for a Unitree Go2.

Architecture (all above the frozen locomotion layer, output is only a Twist):

  1. Mapping      : log-odds occupancy grid in the pose-estimate frame, built
                    from the 181-beam lidar with vectorised ray insertion.
  2. Global layer : a min-plus (Bellman-Ford) cost-to-go field grown from the
                    *current* goal over the grid, run incrementally under a
                    strict per-step wall-clock budget so step() stays fast.
                    Unknown cells are optimistically traversable, so the field
                    doubles as goal-directed exploration on the first pass and
                    as a true shortest path once the building is known.
                    Converged fields are cached per goal point, so the second
                    and later legs of the shuttle start with a ready plan.
  3. Local layer  : candidate-direction search over the full circle using the
                    live scan for admissibility, scored by the global field at
                    two lookahead points. This is what keeps the robot off
                    walls while a stale/unconverged field is being refreshed.
  4. Recovery     : progress/contact watchdog with back-off + spin, a slowly
                    decaying soft penalty map at repeatedly-stuck spots, and a
                    dither behaviour for when the goal point is reached in the
                    estimate frame but the harness has not switched yet.
"""

import math
import time

import numpy as np

from core.observation import Command

_TWO_PI = 2.0 * math.pi
_BIG = 1.0e12          # "unreached" field value (finite, float32-safe)
_WALL = 1.0e7          # traversal multiplier for inflated obstacle cells
_R2 = math.sqrt(2.0)

# (di, dj, is_diagonal) neighbour offsets for the min-plus relaxation.
_OFFS = ((1, 0, False), (0, 1, False), (-1, 0, False), (0, -1, False),
         (1, 1, True), (-1, 1, True), (1, -1, True), (-1, -1, True))
_OFFS_R = tuple(reversed(_OFFS))


def _wrap(a):
    """Wrap angle(s) to (-pi, pi]. Works for scalars and arrays."""
    return (a + math.pi) % _TWO_PI - math.pi


class Controller:
    # ---- grid / mapping ----
    RES = 0.15                 # target cell size (m), relaxed if grid too big
    MAX_CELLS = 130000
    MAP_EVERY = 3              # insert a scan every N control steps (~16 Hz)
    HIT = 0.90
    MISS = 0.34
    LO_MIN = -2.2
    LO_MAX = 4.0
    OCC_THR = 0.5

    # ---- planner ----
    PLAN_BUDGET = 0.005        # seconds of wavefront relaxation per step
    WIN_MARGIN = 8.0           # metres of slack around robot/goal bbox

    # ---- local layer ----
    NCAND = 61                 # candidate headings over the full circle
    HALF_W = 0.30              # half corridor width demanded ahead (m)
    STOP_M = 0.30              # keep-out distance in front of the body (m)
    LOOK = 2.0                 # max lookahead used for field sampling (m)

    # ==================================================================
    # setup
    # ==================================================================
    def reset(self, scenario, rng):
        """Clear ALL episode state."""
        self.rng = rng if rng is not None else np.random.default_rng(20240917)

        self.range_max = self._sf(getattr(scenario, "range_max", None), 6.0, 0.5, 200.0)
        self.max_lin = self._sf(getattr(scenario, "max_lin", None), 0.8, 0.05, 10.0)
        self.max_ang = self._sf(getattr(scenario, "max_ang", None), 1.0, 0.05, 20.0)
        self.time_limit = self._sf(getattr(scenario, "time_limit", None), 1.0e9, 1.0, 1.0e12)

        start = self._pt(getattr(scenario, "start", None), (0.0, 0.0))
        goal = self._pt(getattr(scenario, "goal", None), start)

        # ---------------- grid geometry ----------------
        sep = float(math.hypot(goal[0] - start[0], goal[1] - start[1]))
        marg = 9.0 + 0.55 * sep
        lox = min(start[0], goal[0]) - marg
        loy = min(start[1], goal[1]) - marg
        hix = max(start[0], goal[0]) + marg
        hiy = max(start[1], goal[1]) + marg
        res = self.RES
        nx = ny = 1
        for _ in range(16):
            nx = int((hix - lox) / res) + 3
            ny = int((hiy - loy) / res) + 3
            if nx * ny <= self.MAX_CELLS:
                break
            res *= 1.18
        self.res = float(res)
        self.inv = 1.0 / self.res
        self.nx = int(nx)
        self.ny = int(ny)
        self.ncells = self.nx * self.ny
        self.ox = float(lox)
        self.oy = float(loy)

        # ---------------- map arrays ----------------
        self.log = np.zeros((self.nx, self.ny), dtype=np.float32)
        self.logf = self.log.reshape(-1)                      # flat view
        self.blocked = np.zeros((self.nx, self.ny), dtype=bool)
        base = np.float32(self.res * 1.35)                    # unknown cell cost
        self.mult = np.full((self.nx, self.ny), base, dtype=np.float32)
        self.mult2 = np.full((self.nx, self.ny), np.float32(base * _R2), dtype=np.float32)
        self.pen_extra = np.zeros((self.nx, self.ny), dtype=np.float32)

        # ---------------- ray sampling template ----------------
        sstep = 0.8 * self.res
        nsamp = max(2, int(self.range_max / sstep) + 1)
        self.ds = ((np.arange(nsamp, dtype=np.float64) + 0.5) * sstep)

        # ---------------- candidate directions ----------------
        n = int(self.NCAND)
        self.phis = (np.arange(n, dtype=np.float64) - 0.5 * (n - 1)) * (_TWO_PI / n)
        self.absphi = np.abs(self.phis)
        self.i_fwd = int(np.argmin(np.abs(_wrap(self.phis))))
        self.i_back = int(np.argmax(np.abs(_wrap(self.phis))))
        self.left_m = (self.phis > 0.25) & (self.phis < 1.75)
        self.right_m = (self.phis < -0.25) & (self.phis > -1.75)
        self.beam_ang = None
        self.cos_rel = None
        self.sin_rel = None
        self.fwd_ok = None

        # ---------------- planner state ----------------
        self.goal_xy = goal.astype(float).copy()
        self.gkey = self._key(self.goal_xy)
        self.field_cache = {}
        self.Dact = None
        self.Dw = None
        self.Dv = None
        self.Mv = None
        self.M2v = None
        self.win = (0, self.nx, 0, self.ny)
        self.win_margin = self.WIN_MARGIN
        self.win_full = False
        self.wf_iters = 0
        self.wf_cap = 1000
        self.wf_metric = None
        self.wf_flip = False

        # ---------------- runtime state ----------------
        self.k = 0
        self.t_now = 0.0
        self.px = float(start[0])
        self.py = float(start[1])
        self.pyaw = 0.0
        self.pos_now = np.array([self.px, self.py])
        self.prev_phi = 0.0
        self.hist = []
        self.rec_mode = 0
        self.rec_end = 0.0
        self.rec_dir = 1
        self.rec_count = 0
        self.rec_last = -99.0
        self.contact_run = 0.0
        self.last = (0.0, 0.0, 0.0)

    # ==================================================================
    # helpers
    # ==================================================================
    @staticmethod
    def _sf(v, default, lo, hi):
        try:
            f = float(v)
            if math.isfinite(f) and lo <= f <= hi:
                return f
        except Exception:
            pass
        return float(default)

    @staticmethod
    def _pt(v, default):
        try:
            a = np.asarray(v, dtype=float).ravel()
            if a.size >= 2 and bool(np.all(np.isfinite(a[:2]))):
                return a[:2].astype(float).copy()
        except Exception:
            pass
        return np.asarray(default, dtype=float).ravel()[:2].astype(float).copy()

    @staticmethod
    def _key(p):
        return (round(float(p[0]), 1), round(float(p[1]), 1))

    def _cell(self, px, py):
        ci = int((px - self.ox) * self.inv)
        cj = int((py - self.oy) * self.inv)
        if ci < 0:
            ci = 0
        elif ci >= self.nx:
            ci = self.nx - 1
        if cj < 0:
            cj = 0
        elif cj >= self.ny:
            cj = self.ny - 1
        return ci, cj

    def _sample(self, D, px, py):
        ix = ((px - self.ox) * self.inv).astype(np.int32)
        iy = ((py - self.oy) * self.inv).astype(np.int32)
        np.clip(ix, 0, self.nx - 1, out=ix)
        np.clip(iy, 0, self.ny - 1, out=iy)
        return D[ix, iy]

    @staticmethod
    def _dil(a, r):
        """Diamond dilation of a boolean array by r cells."""
        out = a
        for _ in range(int(r)):
            n = out.copy()
            n[1:, :] |= out[:-1, :]
            n[:-1, :] |= out[1:, :]
            n[:, 1:] |= out[:, :-1]
            n[:, :-1] |= out[:, 1:]
            out = n
        return out

    def _mk(self, vx, vy, wz):
        if not (isinstance(vx, float) and math.isfinite(vx)):
            vx = float(vx) if math.isfinite(float(vx)) else 0.0
        if not (isinstance(vy, float) and math.isfinite(vy)):
            vy = float(vy) if math.isfinite(float(vy)) else 0.0
        if not (isinstance(wz, float) and math.isfinite(wz)):
            wz = float(wz) if math.isfinite(float(wz)) else 0.0
        m = self.max_lin
        n = math.hypot(vx, vy)
        if n > m and n > 1e-9:
            s = m / n
            vx *= s
            vy *= s
        vx = max(-m, min(m, vx))
        vy = max(-m, min(m, vy))
        wz = max(-self.max_ang, min(self.max_ang, wz))
        self.last = (vx, vy, wz)
        return Command(vx, vy, wz)

    # ==================================================================
    # perception
    # ==================================================================
    def _prep_beams(self, angs):
        self.beam_ang = angs.astype(np.float64).copy()
        d = _wrap(self.beam_ang[None, :] - self.phis[:, None])
        self.cos_rel = np.cos(d).astype(np.float32)
        self.sin_rel = np.sin(d).astype(np.float32)
        self.fwd_ok = self.cos_rel > 0.05

    def _update_map(self, pos, yaw, r, hitmask, wgt):
        ca = np.cos(self.beam_ang + yaw)
        sa = np.sin(self.beam_ang + yaw)
        ds = self.ds
        rr = r - 0.7 * self.res

        # ---- free space along every ray ----
        if bool(np.any(rr > ds[0])):
            px = pos[0] + ca[:, None] * ds[None, :]
            py = pos[1] + sa[:, None] * ds[None, :]
            ix = ((px - self.ox) * self.inv).astype(np.int32)
            iy = ((py - self.oy) * self.inv).astype(np.int32)
            m = ds[None, :] < rr[:, None]
            m &= (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            if bool(m.any()):
                flat = ix[m].astype(np.int64) * self.ny + iy[m].astype(np.int64)
                bc = np.bincount(flat, minlength=self.ncells)
                self.logf -= np.minimum(bc, 2).astype(np.float32) * np.float32(self.MISS * wgt)

        # ---- end-point hits ----
        if bool(hitmask.any()):
            hr = r[hitmask]
            hx = pos[0] + ca[hitmask] * hr
            hy = pos[1] + sa[hitmask] * hr
            ix = ((hx - self.ox) * self.inv).astype(np.int32)
            iy = ((hy - self.oy) * self.inv).astype(np.int32)
            ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            if bool(ok.any()):
                flat = ix[ok].astype(np.int64) * self.ny + iy[ok].astype(np.int64)
                bc = np.bincount(flat, minlength=self.ncells)
                self.logf += np.minimum(bc, 3).astype(np.float32) * np.float32(self.HIT * wgt)

        np.clip(self.log, self.LO_MIN, self.LO_MAX, out=self.log)

    def _update_cost(self, pos):
        """Refresh inflation / traversal cost in the patch the lidar just saw."""
        self.pen_extra *= np.float32(0.997)
        ci, cj = self._cell(pos[0], pos[1])
        rad = int((self.range_max + 0.8) * self.inv) + 6
        i0 = max(0, ci - rad)
        i1 = min(self.nx, ci + rad + 1)
        j0 = max(0, cj - rad)
        j1 = min(self.ny, cj + rad + 1)
        if i1 <= i0 or j1 <= j0:
            return
        lo = self.log[i0:i1, j0:j1]
        occ = lo > self.OCC_THR
        unk = np.abs(lo) < 0.22
        b1 = self._dil(occ, 1)
        b2 = self._dil(b1, 1)
        b4 = self._dil(b2, 2)
        pen = np.where(b4, 0.9, 0.0)
        np.copyto(pen, 2.2, where=b2)
        pen += np.where(unk, 0.35, 0.0)
        pen += self.pen_extra[i0:i1, j0:j1]
        m = (self.res * (1.0 + pen)).astype(np.float32)
        np.copyto(m, np.float32(_WALL), where=b1)
        self.mult[i0:i1, j0:j1] = m
        self.mult2[i0:i1, j0:j1] = m * np.float32(_R2)
        self.blocked[i0:i1, j0:j1] = b1

    # ==================================================================
    # global cost-to-go field
    # ==================================================================
    def _start_wave(self, pos):
        m = int(self.win_margin * self.inv) + 4
        gi, gj = self._cell(self.goal_xy[0], self.goal_xy[1])
        ri, rj = self._cell(pos[0], pos[1])
        i0 = max(0, min(gi, ri) - m)
        i1 = min(self.nx, max(gi, ri) + m + 1)
        j0 = max(0, min(gj, rj) - m)
        j1 = min(self.ny, max(gj, rj) + m + 1)
        self.win = (i0, i1, j0, j1)
        self.win_full = (i0 == 0 and j0 == 0 and i1 == self.nx and j1 == self.ny)
        self.Dw = np.full((self.nx, self.ny), np.float32(_BIG), dtype=np.float32)
        self.Dv = self.Dw[i0:i1, j0:j1]
        self.Mv = self.mult[i0:i1, j0:j1]
        self.M2v = self.mult2[i0:i1, j0:j1]
        self.wf_iters = 0
        self.wf_metric = None
        self.wf_cap = int(2.5 * ((i1 - i0) + (j1 - j0))) + 300

    def _seed_goal(self):
        if self.Dw is None:
            return
        gi, gj = self._cell(self.goal_xy[0], self.goal_xy[1])
        for rad in (1, 3, 6, 11, 18):
            i0 = max(0, gi - rad)
            i1 = min(self.nx, gi + rad + 1)
            j0 = max(0, gj - rad)
            j1 = min(self.ny, gj + rad + 1)
            free = ~self.blocked[i0:i1, j0:j1]
            if not bool(free.any()):
                continue
            sub = self.Dw[i0:i1, j0:j1]
            if rad <= 1:
                sub[free] = np.float32(0.0)
            else:
                ii, jj = np.nonzero(free)
                d2 = (ii + i0 - gi) ** 2 + (jj + j0 - gj) ** 2
                b = int(np.argmin(d2))
                sub[int(ii[b]), int(jj[b])] = np.float32(0.0)
            return
        self.Dw[gi, gj] = np.float32(0.0)

    def _relax_once(self, flip):
        D = self.Dv
        n0, n1 = D.shape
        for di, dj, dg in (_OFFS_R if flip else _OFFS):
            if di >= 0:
                da, db, sa, sb = 0, n0 - di, di, n0
            else:
                da, db, sa, sb = -di, n0, 0, n0 + di
            if dj >= 0:
                dc, dd, sc, sd = 0, n1 - dj, dj, n1
            else:
                dc, dd, sc, sd = -dj, n1, 0, n1 + dj
            if db <= da or dd <= dc:
                continue
            dst = D[da:db, dc:dd]
            step = (self.M2v if dg else self.Mv)[da:db, dc:dd]
            np.minimum(dst, D[sa:sb, sc:sd] + step, out=dst)

    def _promote(self):
        self.Dact = self.Dw
        self.field_cache[self.gkey] = self.Dw
        ci, cj = self._cell(self.pos_now[0], self.pos_now[1])
        if float(self.Dw[ci, cj]) > 1.0e6 and not self.win_full:
            self.win_margin = min(self.win_margin * 2.0, 1.0e4)
        self.Dw = None
        self.Dv = None
        self.wf_iters = 0
        self.wf_metric = None

    def _plan_tick(self, pos):
        t0 = time.perf_counter()
        if self.Dw is None:
            self._start_wave(pos)
        self._seed_goal()
        while True:
            self._relax_once(self.wf_flip)
            self.wf_flip = not self.wf_flip
            self.wf_iters += 1
            if self.wf_iters % 8 == 0:
                fin = self.Dv < 1.0e6
                c = int(fin.sum())
                s = float(self.Dv[fin].sum(dtype=np.float64)) if c else 0.0
                prev = self.wf_metric
                self.wf_metric = (c, s)
                if prev is not None and prev[0] == c and abs(prev[1] - s) < 1.0e-3 * max(1.0, c):
                    self._promote()
                    return
            if self.wf_iters >= self.wf_cap:
                self._promote()
                return
            if (time.perf_counter() - t0) >= self.PLAN_BUDGET:
                return

    def _on_goal_change(self, goal):
        self.goal_xy = goal.astype(float).copy()
        self.gkey = self._key(self.goal_xy)
        self.Dact = self.field_cache.get(self.gkey)   # reuse a previous leg's plan
        self.Dw = None
        self.Dv = None
        self.wf_iters = 0
        self.wf_metric = None
        self.win_margin = self.WIN_MARGIN
        self.hist = []
        self.rec_mode = 0
        self.rec_count = 0
        self.contact_run = 0.0
        self.prev_phi = 0.0

    # ==================================================================
    # recovery behaviours
    # ==================================================================
    def _mark_stuck_spot(self, pos, yaw):
        ang = yaw + self.prev_phi
        cx = pos[0] + 0.45 * math.cos(ang)
        cy = pos[1] + 0.45 * math.sin(ang)
        ci, cj = self._cell(cx, cy)
        rad = max(1, int(0.22 * self.inv))
        i0 = max(0, ci - rad)
        i1 = min(self.nx, ci + rad + 1)
        j0 = max(0, cj - rad)
        j1 = min(self.ny, cj + rad + 1)
        if i1 > i0 and j1 > j0:
            self.pen_extra[i0:i1, j0:j1] += np.float32(2.5)
        self._update_cost(pos)

    def _recover(self, t, dfree):
        if self.rec_mode == 1:                       # back away
            if t >= self.rec_end or float(dfree[self.i_back]) < 0.45:
                self.rec_mode = 2
                self.rec_end = t + 1.0 + 0.4 * min(self.rec_count, 3)
            else:
                return self._mk(-0.40, 0.0, 0.35 * self.rec_dir)
        if self.rec_mode == 2:                       # spin to find an opening
            if t >= self.rec_end or float(dfree[self.i_fwd]) > 2.0:
                self.rec_mode = 0
                self.rec_last = t
                self.hist = []
                return None
            return self._mk(0.05, 0.0, self.rec_dir * self.max_ang)
        if self.rec_mode == 3:                       # dither around the goal
            if t >= self.rec_end:
                self.rec_mode = 0
                self.rec_last = t
                self.hist = []
                return None
            if float(dfree[self.i_fwd]) < 0.65:
                return self._mk(0.0, 0.0, self.rec_dir * self.max_ang)
            return self._mk(0.35, 0.0, self.rec_dir * 0.95)
        return None

    # ==================================================================
    # control
    # ==================================================================
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            vx, vy, wz = self.last
            try:
                return self._mk(0.3 * vx, 0.3 * vy, 0.35)
            except Exception:
                return Command(0.0, 0.0, 0.35)

    def _step(self, obs):
        self.k += 1

        # ---------------- time ----------------
        t = self._sf(getattr(obs, "t", None), self.t_now + 0.02, -1.0e9, 1.0e12)
        if t <= self.t_now:
            t = self.t_now + 0.02
        dt = min(0.2, max(0.002, t - self.t_now))
        self.t_now = t

        # ---------------- pose ----------------
        try:
            pose = np.asarray(obs.pose_estimate, dtype=float).ravel()
        except Exception:
            pose = np.array([self.px, self.py, self.pyaw])
        if pose.size < 3 or not bool(np.all(np.isfinite(pose[:3]))):
            pose = np.array([self.px, self.py, self.pyaw])
        x = float(pose[0])
        y = float(pose[1])
        yaw = float(pose[2])
        self.px, self.py, self.pyaw = x, y, yaw
        pos = np.array([x, y])
        self.pos_now = pos
        perr = self._sf(getattr(obs, "pose_error_estimate", None), 0.0, 0.0, 1.0e6)
        contact = bool(getattr(obs, "contact", False))

        # ---------------- goal ----------------
        goal = self._pt(getattr(obs, "goal", None), self.goal_xy)
        if math.hypot(goal[0] - self.goal_xy[0], goal[1] - self.goal_xy[1]) > 0.25:
            self._on_goal_change(goal)
        gx = float(self.goal_xy[0])
        gy = float(self.goal_xy[1])
        dg = math.hypot(gx - x, gy - y)

        # ---------------- lidar ----------------
        try:
            ranges = np.asarray(obs.ranges, dtype=float).ravel()
        except Exception:
            ranges = np.full(self.phis.size, self.range_max)
        nb = int(ranges.size)
        if nb < 3:
            ranges = np.full(8, self.range_max)
            nb = 8
        angs = None
        try:
            a = np.asarray(obs.range_angles, dtype=float).ravel()
            if a.size == nb and bool(np.all(np.isfinite(a))):
                angs = a
        except Exception:
            angs = None
        if angs is None:
            angs = np.linspace(-math.pi, math.pi, nb, endpoint=False)
        if (self.beam_ang is None or self.beam_ang.size != nb
                or abs(float(self.beam_ang[0]) - float(angs[0])) > 1.0e-6):
            self._prep_beams(angs)

        finite = np.isfinite(ranges) & (ranges > 0.05)
        r = np.where(finite, ranges, self.range_max * 2.0)
        np.clip(r, 0.0, self.range_max, out=r)
        hitmask = finite & (ranges < self.range_max - 0.15)

        # ---------------- admissible directions from the live scan ----------
        r32 = r.astype(np.float32)
        fwd = self.cos_rel * r32[None, :]
        lat = self.sin_rel * r32[None, :]
        blk = self.fwd_ok & (np.abs(lat) < np.float32(self.HALF_W))
        dfree = np.where(blk, fwd, np.float32(1.0e3)).min(axis=1)
        np.clip(dfree, 0.0, self.range_max, out=dfree)
        allow = np.clip(dfree - np.float32(self.STOP_M), 0.0, np.float32(self.LOOK))
        adm = dfree > np.float32(self.STOP_M + 0.12)

        # ---------------- contact bookkeeping ----------------
        if contact:
            self.contact_run += dt
        else:
            self.contact_run = max(0.0, self.contact_run - 2.0 * dt)

        # ---------------- map + plan ----------------
        if self.k % self.MAP_EVERY == 0:
            wgt = max(0.35, min(1.0, 1.2 - 0.5 * perr))
            self._update_map(pos, yaw, r, hitmask, wgt)
            self._update_cost(pos)
        self._plan_tick(pos)

        # ---------------- stuck watchdog ----------------
        self.hist.append((t, x, y))
        while len(self.hist) > 2 and (t - self.hist[0][0]) > 3.3:
            self.hist.pop(0)
        if self.rec_mode == 0 and (t - self.rec_last) > 2.0:
            stuck = self.contact_run > 0.8
            if not stuck and len(self.hist) > 20 and (t - self.hist[0][0]) >= 2.9:
                h0 = self.hist[0]
                if math.hypot(x - h0[1], y - h0[2]) < 0.42:
                    stuck = True
            if stuck:
                lc = float(dfree[self.left_m].sum()) if self.left_m.any() else 0.0
                rc = float(dfree[self.right_m].sum()) if self.right_m.any() else 0.0
                if abs(lc - rc) < 0.5:
                    self.rec_dir = 1 if self.rng.random() < 0.5 else -1
                else:
                    self.rec_dir = 1 if lc > rc else -1
                self.rec_count = self.rec_count + 1 if (t - self.rec_last) < 14.0 else 1
                self.rec_last = t
                if dg < 0.65:
                    self.rec_mode = 3
                    self.rec_end = t + 2.2
                else:
                    self.rec_mode = 1
                    self.rec_end = t + 0.7
                    if self.rec_count >= 2:
                        self._mark_stuck_spot(pos, yaw)
                self.hist = []

        if self.rec_mode:
            cmd = self._recover(t, dfree)
            if cmd is not None:
                return cmd

        # ---------------- score candidate directions ----------------
        D = self.Dact
        if D is None and self.Dw is not None and self.wf_iters >= 10:
            D = self.Dw
        cost = (0.80 * (np.float32(self.LOOK) - allow).astype(np.float64)
                + 0.55 * self.absphi
                + 0.35 * np.abs(_wrap(self.phis - self.prev_phi)))
        cy = np.cos(self.phis + yaw)
        sy = np.sin(self.phis + yaw)
        la = allow.astype(np.float64)
        for w, s in ((0.35, 0.55), (0.65, 1.0)):
            px = x + cy * (la * s)
            py = y + sy * (la * s)
            e = np.hypot(px - gx, py - gy)
            if D is not None:
                v = self._sample(D, px, py).astype(np.float64)
                val = np.where(v < 1.0e6, v, 30.0 + 1.6 * e)
            else:
                val = 1.6 * e
            cost = cost + w * val
        if dg < 1.3:     # final approach: pull hard onto the actual point
            phig = _wrap(math.atan2(gy - y, gx - x) - yaw)
            cost = cost + 1.2 * np.abs(_wrap(self.phis - phig))
        cost = cost + np.where(adm, 0.0, 500.0)

        best = int(np.argmin(cost))
        if float(cost[best]) > 400.0:
            # boxed in everywhere: spin out of it
            self.rec_mode = 2
            self.rec_end = t + 0.9
            self.rec_last = t
            lc = float(dfree[self.left_m].sum()) if self.left_m.any() else 0.0
            rc = float(dfree[self.right_m].sum()) if self.right_m.any() else 0.0
            self.rec_dir = 1 if lc >= rc else -1
            return self._mk(0.0, 0.0, self.rec_dir * self.max_ang)

        phi = float(self.phis[best])
        self.prev_phi = phi

        # ---------------- direction -> twist ----------------
        d = float(dfree[best])
        v = self.max_lin * min(1.0, 0.30 + 0.45 * d)
        if self.contact_run > 0.05:
            v *= 0.6
        if dg < 0.55:
            v *= min(1.0, 0.45 + dg)
        ca = math.cos(phi)
        sa = math.sin(phi)
        g = 0.45 + 0.55 * max(0.0, ca)
        vx = v * ca * g
        vy = max(-0.45, min(0.45, v * sa * g))
        if abs(phi) > 1.2:
            vx *= 0.6
            vy *= 0.8
        wz = 2.2 * phi
        return self._mk(vx, vy, wz)
