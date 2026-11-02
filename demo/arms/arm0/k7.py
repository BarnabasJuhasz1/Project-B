"""
Navigation controller for a Unitree Go2 shuttling between two fixed points in an
unmapped indoor building.

Architecture (all above the frozen locomotion policy, output is a Twist):

  1.  Occupancy mapping.  A log-odds grid is accumulated over the WHOLE episode
      in a fixed "map frame".  Because the robot covers the same building many
      times, the map is the single biggest lever on trip count: leg 1 is
      exploratory, legs 2..N run a known route at full speed.

  2.  Odometry-drift compensation.  `pose_estimate` drifts.  A tiny discrete
      scan-matcher nudges a map<-odom correction so the map stays sharp.  The
      current goal is pushed through the *same* correction, which makes the
      drive target in estimate-space identical to `obs.goal` -- so the fix can
      only ever sharpen the map, never bias where we actually aim.

  3.  Global planning.  A cost-to-go field is relaxed over the inflated grid by
      vectorised Bellman sweeps, amortised over control steps at a fixed, small
      per-step budget (anytime planner).  Unknown cells are optimistically
      passable, which yields free-space-assumption exploration on leg 1.
      Converged fields are cached per goal, so flipping direction re-uses the
      previous leg's field immediately.

  4.  Reactive layer.  A VFH-style polar obstacle histogram built from the raw
      360 deg scan at 50 Hz picks the actual heading and speed.  It never
      depends on the map or the pose, so it stays correct under drift.

  5.  Recovery.  Progress watchdog + contact reflex, with a decaying penalty
      field stamped where the robot got stuck so the planner routes elsewhere.
"""

import math
from collections import deque

import numpy as np

from core.observation import Command

_TWO_PI = 2.0 * math.pi
_BIG = 1.0e9
_FIN = 1.0e8


def _wrap(a):
    return (a + math.pi) % _TWO_PI - math.pi


def _pt2(v, default=(0.0, 0.0)):
    try:
        a = np.asarray(v, dtype=float).ravel()
        if a.size >= 2 and np.all(np.isfinite(a[:2])):
            return np.array([float(a[0]), float(a[1])])
    except Exception:
        pass
    return np.array(default, dtype=float)


class Controller:
    # ---- map / planner ----------------------------------------------------
    RES = 0.20            # nominal grid resolution [m]
    MARGIN = 10.0         # padding around the start/goal bounding box [m]
    MIN_EXTENT = 20.0     # never build a grid smaller than this [m]
    MAX_CELLS = 34000     # grid budget (resolution is coarsened to fit)
    OCC_T = 0.55          # log-odds threshold for "occupied"
    UNK_T = 0.12          # |log-odds| below this is "unknown"
    L_FREE = 0.30
    L_OCC = 0.85
    L_MIN = -2.5
    L_MAX = 4.0
    INFLATE_M = 0.22      # hard inflation; kept small so doorways stay open
    SOFT_RINGS = 2        # extra rings that are merely expensive
    PEN_SOFT = 0.55
    PEN_UNKNOWN = 0.18
    PEN_TRAIL = 0.30
    PEN_FLOOR = -0.40     # keeps every edge cost strictly positive
    WORK_PER_STEP = 130000.0   # relaxation element-budget per control step

    # ---- reactive layer ---------------------------------------------------
    NC = 48               # candidate headings
    SAFE_R = 0.27         # VFH enlargement radius [m]
    BLOCK_RANGE = 3.0     # beams further than this do not veto a heading
    D_BLOCK = 0.36        # a heading needs at least this much clearance
    V_CLEAR_LO = 0.42
    V_CLEAR_HI = 1.30
    LOOKAHEAD = 1.0       # carrot distance along the descent path [m]
    W_ALIGN = 1.60
    W_CLEAR = 0.90
    W_HYST = 0.25

    # ---- recovery ---------------------------------------------------------
    STUCK_WIN = 3.0
    STUCK_DIST = 0.35
    RECOVER_T = 1.6
    EVADE_T = 0.45

    # =======================================================================
    # lifecycle
    # =======================================================================
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(1234)

        self.max_lin = self._num(getattr(scenario, "max_lin", None), 0.8)
        self.max_ang = self._num(getattr(scenario, "max_ang", None), 1.0)
        self.range_max = self._num(getattr(scenario, "range_max", None), 6.0)
        self.time_limit = self._num(getattr(scenario, "time_limit", None), 600.0)

        start = _pt2(getattr(scenario, "start", None))
        goal = _pt2(getattr(scenario, "goal", None))

        # ---- grid geometry -------------------------------------------------
        lo = np.minimum(start, goal) - self.MARGIN
        hi = np.maximum(start, goal) + self.MARGIN
        ctr = 0.5 * (lo + hi)
        ext = np.maximum(hi - lo, self.MIN_EXTENT)
        lo = ctr - 0.5 * ext
        hi = ctr + 0.5 * ext

        res = self.RES
        for _ in range(24):
            nx = int(math.ceil((hi[0] - lo[0]) / res)) + 2
            ny = int(math.ceil((hi[1] - lo[1]) / res)) + 2
            if nx * ny <= self.MAX_CELLS or res >= 0.55:
                break
            res *= 1.12
        nx = max(8, int(math.ceil((hi[0] - lo[0]) / res)) + 2)
        ny = max(8, int(math.ceil((hi[1] - lo[1]) / res)) + 2)

        self.res = float(res)
        self.inv_res = 1.0 / self.res
        self.ox = float(lo[0] - self.res)
        self.oy = float(lo[1] - self.res)
        self.nx = int(nx)
        self.ny = int(ny)
        self.ncells = self.nx * self.ny

        shp = (self.ny, self.nx)
        self.logodds = np.zeros(shp, dtype=np.float32)
        self.trail = np.zeros(shp, dtype=np.float32)
        self.blockpen = np.zeros(shp, dtype=np.float32)
        self.hard = np.zeros(shp, dtype=bool)
        self.cost1 = np.ones(shp, dtype=np.float32)
        self.costd = np.full(shp, np.float32(math.sqrt(2.0)), dtype=np.float32)
        self.lfield = np.zeros(shp, dtype=np.float32)
        self.map_hits = 0

        # relaxation budget: keep the per-step cost roughly constant
        self.iters = int(max(3, min(14, self.WORK_PER_STEP / max(1.0, 8.0 * self.ncells))))
        self.iters_boot = 30
        self.max_iters = int(2.5 * (self.nx + self.ny))

        # ---- planner state -------------------------------------------------
        self.D = None
        self.Dbuild = None
        self.build_gc = (0, 0)
        self.build_iters = 0
        self.build_cnt = -1
        self.build_stable = 0
        self.fields = {}
        self.goal_key = None

        # ---- beam caches (filled on the first observation) -----------------
        self.cand = np.linspace(-math.pi, math.pi, self.NC, endpoint=False)
        self.cand_cos = np.cos(self.cand)
        self.cand_sin = np.sin(self.cand)
        self.beam_ang = None
        self.beam_cos = None
        self.beam_sin = None
        self.absdt = None
        self.sample_d = None

        # ---- pose correction (map <- odom) ---------------------------------
        self.fix = np.zeros(3, dtype=float)
        vals = (0.0, -0.06, 0.06)
        angs = (0.0, -0.020, 0.020)
        offs = []
        for da in angs:
            for dy in vals:
                for dx in vals:
                    offs.append((dx, dy, da))
        self.sm_offsets = np.asarray(offs, dtype=float)   # index 0 == identity

        # ---- control / behaviour state -------------------------------------
        self.cmd = np.zeros(3, dtype=float)
        self.prev_phi = 0.0
        self.t_prev = None
        self.step_i = 0
        self.goal_prev = None
        self.leg_t0 = 0.0
        self.leg_best = _BIG
        self.legs = 0
        self.hist = deque()
        self.recover_until = -1.0
        self.recover_t0 = -1.0
        self.recover_dir = 1.0
        self.stuck_count = 0
        self.evade_until = -1.0
        self.evade_world = 0.0
        self.contact_t0 = None
        self.fail_count = 0

    # =======================================================================
    # main entry point -- must never raise
    # =======================================================================
    def step(self, obs):
        try:
            vx, vy, wz = self._step(obs)
        except Exception:
            self.fail_count += 1
            c = self.cmd
            vx, vy, wz = 0.25 * float(c[0]), 0.25 * float(c[1]), 0.40
            self.cmd = np.array([vx, vy, wz], dtype=float)
        lim = self.max_lin
        vx = max(-lim, min(lim, float(vx)))
        vy = max(-lim, min(lim, float(vy)))
        wz = max(-self.max_ang, min(self.max_ang, float(wz)))
        if not (vx == vx and vy == vy and wz == wz):   # NaN guard
            vx = vy = 0.0
            wz = 0.3
        return Command(vx, vy, wz)

    # =======================================================================
    def _step(self, obs):
        t = self._num(getattr(obs, "t", None), (self.t_prev or 0.0) + 0.02)
        if self.t_prev is None:
            self.t_prev = t
        self.t_prev = t
        self.step_i += 1

        # ---------------- pose, goal, scan --------------------------------
        pe = np.asarray(getattr(obs, "pose_estimate", (0.0, 0.0, 0.0)), dtype=float).ravel()
        ex = float(pe[0]) if pe.size > 0 else 0.0
        ey = float(pe[1]) if pe.size > 1 else 0.0
        eyaw = float(pe[2]) if pe.size > 2 else 0.0
        perr = self._num(getattr(obs, "pose_error_estimate", None), 0.0)

        ge = _pt2(getattr(obs, "goal", None))

        ranges = np.asarray(getattr(obs, "ranges", ()), dtype=float).ravel()
        angles = np.asarray(getattr(obs, "range_angles", ()), dtype=float).ravel()
        nb = int(min(ranges.size, angles.size))
        if nb < 4:
            return 0.0, 0.0, 0.5
        ranges = ranges[:nb]
        angles = angles[:nb]
        if self.beam_ang is None or self.beam_ang.size != nb:
            self._setup_beams(angles)

        finite = np.isfinite(ranges)
        rr = np.where(finite, ranges, self.range_max)
        rr = np.clip(rr, 0.02, self.range_max)
        hit = finite & (ranges < self.range_max - 0.02)

        # ---------------- drift compensation -------------------------------
        if (self.step_i % 5) == 0 and self.map_hits > 400 and t > 5.0:
            self._scan_match(ex, ey, eyaw, rr, hit)

        px, py, yaw = self._corrected(ex, ey, eyaw)
        gx, gy = self._to_map(float(ge[0]), float(ge[1]))
        dg = math.hypot(gx - px, gy - py)

        # ---------------- leg bookkeeping ----------------------------------
        key = (round(float(ge[0]), 1), round(float(ge[1]), 1))
        if key != self.goal_key:
            self.goal_key = key
            self.legs += 1
            self.leg_t0 = t
            self.leg_best = dg
            self.hist.clear()
            self.recover_until = -1.0
            self.evade_until = -1.0
            self.stuck_count = 0
            self.D = self.fields.get(key)     # warm start from the previous leg
            self.Dbuild = None
        self.goal_prev = np.array([gx, gy])

        # ---------------- mapping ------------------------------------------
        if (self.step_i % 2) == 0:
            self._integrate(px, py, yaw, rr, hit, perr)
        if (self.step_i % 250) == 0:
            self.logodds *= np.float32(0.99)       # forget smeared/stale cells
            self.trail *= np.float32(0.995)
        if (self.step_i % 50) == 0:
            self.blockpen *= np.float32(0.99)      # stuck-penalties expire
        if (self.step_i % 5) == 0:
            self._stamp_trail(px, py, dg)

        # ---------------- global plan (anytime, amortised) ------------------
        self._plan_tick(px, py, gx, gy)

        # ---------------- reactive clearance histogram ----------------------
        clear = self._clearance(rr)
        jmax = int(np.argmax(clear))
        cmax = float(clear[jmax])

        # ---------------- reflexes / watchdog -------------------------------
        contact = bool(getattr(obs, "contact", False))
        if contact:
            if self.contact_t0 is None:
                self.contact_t0 = t
            if t > self.evade_until:
                self._on_contact(t, px, py, yaw, rr)
            if t - self.contact_t0 > 0.9 and t > self.recover_until:
                self._trigger_recovery(t, px, py, clear)
        else:
            self.contact_t0 = None

        self.hist.append((t, px, py))
        while self.hist and (t - self.hist[0][0]) > self.STUCK_WIN:
            self.hist.popleft()
        if (t - self.leg_t0) > 2.5 and t > self.recover_until + 1.0 and dg > 0.6 and self.hist:
            t0, x0, y0 = self.hist[0]
            if (t - t0) >= self.STUCK_WIN - 0.1 and math.hypot(px - x0, py - y0) < self.STUCK_DIST:
                self._trigger_recovery(t, px, py, clear)

        # ---------------- pick a heading -----------------------------------
        emergency = False
        if t < self.evade_until:
            emergency = True
            phi = _wrap(self.evade_world - yaw)
            v = 0.32
            wz = 0.0
        elif t < self.recover_until:
            emergency = True
            phi = float(self.cand[jmax])
            v = min(0.42, 0.55 * max(0.0, cmax - 0.25))
            wz = 1.1 * phi + 0.55 * self.recover_dir
        else:
            if dg < 0.70:
                phi_des = _wrap(math.atan2(gy - py, gx - px) - yaw)
            else:
                phi_des = _wrap(self._plan_dir(px, py, gx, gy) - yaw)

            score = (self.W_ALIGN * np.cos(self.cand - phi_des)
                     + self.W_CLEAR * (np.minimum(clear, 2.5) / 2.5)
                     + self.W_HYST * np.cos(self.cand - self.prev_phi))
            ok = clear > self.D_BLOCK
            if np.any(ok):
                sc = np.where(ok, score, -1.0e6)
                j = int(np.argmax(sc))
                phi = self._refine(sc, ok, j)
                cl = float(clear[j])
            else:
                j = jmax
                phi = float(self.cand[j])
                cl = cmax

            span = self.V_CLEAR_HI - self.V_CLEAR_LO
            v = self.max_lin * min(1.0, max(0.0, (cl - self.V_CLEAR_LO) / span))
            v *= 0.40 + 0.60 * max(0.0, math.cos(phi))
            if dg < 0.5:
                v = min(v, max(0.18, self.max_lin * dg / 0.5))
            wz = 2.0 * phi

        self.prev_phi = phi

        vx = v * math.cos(phi)
        vy = 0.80 * v * math.sin(phi)
        wz = max(-self.max_ang, min(self.max_ang, wz))

        a = 0.70 if emergency else 0.40
        tgt = np.array([vx, vy, wz], dtype=float)
        self.cmd = self.cmd + a * (tgt - self.cmd)
        self.cmd[2] = self.cmd[2] + 0.25 * (wz - self.cmd[2])   # snappier yaw
        return float(self.cmd[0]), float(self.cmd[1]), float(self.cmd[2])

    # =======================================================================
    # beams
    # =======================================================================
    def _setup_beams(self, angles):
        self.beam_ang = np.array(angles, dtype=float)
        self.beam_cos = np.cos(self.beam_ang)
        self.beam_sin = np.sin(self.beam_ang)
        d = self.cand[:, None] - self.beam_ang[None, :]
        self.absdt = np.abs((d + math.pi) % _TWO_PI - math.pi).astype(np.float32)
        stp = 0.7 * self.res
        S = max(2, int(self.range_max / stp) + 1)
        self.sample_d = (np.arange(S, dtype=float) + 0.5) * stp

    def _clearance(self, rr):
        """Distance to the nearest obstacle that vetoes each candidate heading."""
        hw = np.arcsin(np.clip(self.SAFE_R / np.maximum(rr, self.SAFE_R), 0.0, 1.0))
        hw = np.where(rr < self.BLOCK_RANGE, hw, -1.0).astype(np.float32)
        blocked = self.absdt < hw[None, :]
        cl = np.where(blocked, rr[None, :].astype(np.float32), np.float32(_BIG)).min(axis=1)
        return np.minimum(cl, np.float32(self.range_max)).astype(np.float64)

    def _refine(self, sc, ok, j):
        """Parabolic refinement of the discrete heading choice."""
        phi = float(self.cand[j])
        jm = (j - 1) % self.NC
        jp = (j + 1) % self.NC
        if ok[jm] and ok[jp]:
            a, b, c = float(sc[jm]), float(sc[j]), float(sc[jp])
            den = a - 2.0 * b + c
            if den < -1e-9:
                d = 0.5 * (a - c) / den
                if -1.0 < d < 1.0:
                    phi = _wrap(phi + d * (_TWO_PI / self.NC))
        return phi

    # =======================================================================
    # frames
    # =======================================================================
    def _corrected(self, ex, ey, eyaw):
        c = math.cos(self.fix[2])
        s = math.sin(self.fix[2])
        return (c * ex - s * ey + self.fix[0],
                s * ex + c * ey + self.fix[1],
                _wrap(eyaw + self.fix[2]))

    def _to_map(self, x, y):
        c = math.cos(self.fix[2])
        s = math.sin(self.fix[2])
        return (c * x - s * y + self.fix[0], s * x + c * y + self.fix[1])

    def _scan_match(self, ex, ey, eyaw, rr, hit):
        """Tiny discrete scan-to-map matcher; only ever sharpens the map."""
        idx = np.nonzero(hit)[0]
        if idx.size < 24:
            return
        if idx.size > 90:
            idx = idx[:: max(1, idx.size // 90)]
        px, py, yaw = self._corrected(ex, ey, eyaw)
        d = rr[idx]
        lx = (d * self.beam_cos[idx])[None, :]
        ly = (d * self.beam_sin[idx])[None, :]
        off = self.sm_offsets
        cy = np.cos(yaw + off[:, 2])[:, None]
        sy = np.sin(yaw + off[:, 2])[:, None]
        wx = px + off[:, 0:1] + cy * lx - sy * ly
        wy = py + off[:, 1:2] + sy * lx + cy * ly
        ix = ((wx - self.ox) * self.inv_res).astype(np.int32)
        iy = ((wy - self.oy) * self.inv_res).astype(np.int32)
        np.clip(ix, 0, self.nx - 1, out=ix)
        np.clip(iy, 0, self.ny - 1, out=iy)
        sc = self.lfield[iy, ix].sum(axis=1)
        b = int(np.argmax(sc))
        if b == 0:
            return
        if float(sc[b]) < 1.02 * float(sc[0]) + 0.5:
            return
        dx, dy, da = 0.6 * off[b]
        nyaw = _wrap(self.fix[2] + da)
        cn, sn = math.cos(nyaw), math.sin(nyaw)
        tx = (px + dx) - (cn * ex - sn * ey)
        ty = (py + dy) - (sn * ex + cn * ey)
        # reject anything that would move us implausibly far from the estimate
        nx_ = cn * ex - sn * ey + tx
        ny_ = sn * ex + cn * ey + ty
        if math.hypot(nx_ - ex, ny_ - ey) > 3.0 or abs(nyaw) > 0.6:
            return
        self.fix[0] = tx
        self.fix[1] = ty
        self.fix[2] = nyaw

    # =======================================================================
    # mapping
    # =======================================================================
    def _integrate(self, px, py, yaw, rr, hit, perr):
        w = 1.0 / (1.0 + (max(0.0, perr) / 0.8) ** 2)
        w = min(1.0, max(0.20, w))
        cy, sy = math.cos(yaw), math.sin(yaw)
        ca = cy * self.beam_cos - sy * self.beam_sin
        sa = sy * self.beam_cos + cy * self.beam_sin

        d = self.sample_d
        lim = rr - 0.75 * self.res
        m = d[None, :] < lim[:, None]
        xs = px + ca[:, None] * d[None, :]
        ys = py + sa[:, None] * d[None, :]
        ix = ((xs - self.ox) * self.inv_res).astype(np.int32)
        iy = ((ys - self.oy) * self.inv_res).astype(np.int32)
        ok = m & (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        flat = (iy * self.nx + ix)[ok]
        if flat.size:
            cnt = np.bincount(flat, minlength=self.ncells)
            np.minimum(cnt, 1, out=cnt)
            self.logodds -= np.float32(self.L_FREE * w) * cnt.reshape(self.ny, self.nx).astype(np.float32)

        if np.any(hit):
            hx = px + ca[hit] * rr[hit]
            hy = py + sa[hit] * rr[hit]
            jx = ((hx - self.ox) * self.inv_res).astype(np.int32)
            jy = ((hy - self.oy) * self.inv_res).astype(np.int32)
            ok2 = (jx >= 0) & (jx < self.nx) & (jy >= 0) & (jy < self.ny)
            f2 = (jy * self.nx + jx)[ok2]
            if f2.size:
                c2 = np.bincount(f2, minlength=self.ncells)
                np.minimum(c2, 1, out=c2)
                self.logodds += np.float32(self.L_OCC * w) * c2.reshape(self.ny, self.nx).astype(np.float32)

        np.clip(self.logodds, self.L_MIN, self.L_MAX, out=self.logodds)

    def _stamp_trail(self, px, py, dg):
        d = self._disc(px, py, 0.22)
        if d is None:
            return
        y0, y1, x0, x1, m = d
        sub = self.trail[y0:y1, x0:x1]
        bonus = 0.20
        if dg < self.leg_best - 0.20:
            self.leg_best = dg
            bonus = 0.60
        sub[m] = np.minimum(sub[m] + np.float32(bonus), np.float32(1.0))

    # =======================================================================
    # planner
    # =======================================================================
    def _cell(self, wx, wy):
        ix = int((wx - self.ox) * self.inv_res)
        iy = int((wy - self.oy) * self.inv_res)
        if ix < 0:
            ix = 0
        elif ix >= self.nx:
            ix = self.nx - 1
        if iy < 0:
            iy = 0
        elif iy >= self.ny:
            iy = self.ny - 1
        return iy, ix

    def _disc(self, wx, wy, radius):
        cx = int((wx - self.ox) * self.inv_res)
        cy = int((wy - self.oy) * self.inv_res)
        k = max(1, int(round(radius * self.inv_res)))
        x0 = max(0, cx - k)
        x1 = min(self.nx, cx + k + 1)
        y0 = max(0, cy - k)
        y1 = min(self.ny, cy + k + 1)
        if x1 <= x0 or y1 <= y0:
            return None
        yy = np.arange(y0, y1)[:, None] - cy
        xx = np.arange(x0, x1)[None, :] - cx
        return y0, y1, x0, x1, (xx * xx + yy * yy) <= k * k

    @staticmethod
    def _dil(a):
        o = a.copy()
        o[1:, :] |= a[:-1, :]
        o[:-1, :] |= a[1:, :]
        o[:, 1:] |= a[:, :-1]
        o[:, :-1] |= a[:, 1:]
        return o

    def _rebuild_cost(self, px, py, gx, gy):
        occ = self.logodds > np.float32(self.OCC_T)
        unknown = np.abs(self.logodds) < np.float32(self.UNK_T)

        hard = occ.copy()
        for _ in range(max(1, int(round(self.INFLATE_M * self.inv_res)))):
            hard = self._dil(hard)
        soft = hard.copy()
        for _ in range(self.SOFT_RINGS):
            soft = self._dil(soft)

        # Strip the inflation (not the obstacles) around the robot and the goal
        # so the field can always reach both, then guarantee both are enterable.
        for (wx, wy, rad) in ((px, py, 0.85), (gx, gy, 0.55)):
            d = self._disc(wx, wy, rad)
            if d is not None:
                y0, y1, x0, x1, m = d
                sub = hard[y0:y1, x0:x1]
                sub[m] = occ[y0:y1, x0:x1][m]
        d = self._disc(px, py, 0.26)
        if d is not None:
            y0, y1, x0, x1, m = d
            sub = hard[y0:y1, x0:x1]
            sub[m] = False
        gc = self._cell(gx, gy)
        hard[gc] = False

        pen = np.zeros_like(self.logodds)
        pen += np.where(soft & ~hard, np.float32(self.PEN_SOFT), np.float32(0.0))
        pen += np.where(unknown, np.float32(self.PEN_UNKNOWN), np.float32(0.0))
        pen += self.blockpen
        pen -= np.float32(self.PEN_TRAIL) * np.minimum(self.trail, np.float32(1.0))
        np.maximum(pen, np.float32(self.PEN_FLOOR), out=pen)

        c1 = np.float32(1.0) + pen
        cd = np.float32(math.sqrt(2.0)) + pen
        c1[hard] = np.float32(_BIG)
        cd[hard] = np.float32(_BIG)
        self.cost1 = c1
        self.costd = cd
        self.hard = hard

        # likelihood field for the scan matcher
        lf = occ.astype(np.float32)
        for _ in range(3):
            n = lf.copy()
            np.maximum(n[1:, :], lf[:-1, :] * np.float32(0.65), out=n[1:, :])
            np.maximum(n[:-1, :], lf[1:, :] * np.float32(0.65), out=n[:-1, :])
            np.maximum(n[:, 1:], lf[:, :-1] * np.float32(0.65), out=n[:, 1:])
            np.maximum(n[:, :-1], lf[:, 1:] * np.float32(0.65), out=n[:, :-1])
            lf = n
        self.lfield = lf
        self.map_hits = int(np.count_nonzero(occ))

    def _relax(self, D, n):
        c1 = self.cost1
        cd = self.costd
        gy, gx = self.build_gc
        for _ in range(n):
            np.minimum(D[1:, :], D[:-1, :] + c1[1:, :], out=D[1:, :])
            np.minimum(D[:-1, :], D[1:, :] + c1[:-1, :], out=D[:-1, :])
            np.minimum(D[:, 1:], D[:, :-1] + c1[:, 1:], out=D[:, 1:])
            np.minimum(D[:, :-1], D[:, 1:] + c1[:, :-1], out=D[:, :-1])
            np.minimum(D[1:, 1:], D[:-1, :-1] + cd[1:, 1:], out=D[1:, 1:])
            np.minimum(D[:-1, :-1], D[1:, 1:] + cd[:-1, :-1], out=D[:-1, :-1])
            np.minimum(D[1:, :-1], D[:-1, 1:] + cd[1:, :-1], out=D[1:, :-1])
            np.minimum(D[:-1, 1:], D[1:, :-1] + cd[:-1, 1:], out=D[:-1, 1:])
            D[gy, gx] = np.float32(0.0)

    def _plan_tick(self, px, py, gx, gy):
        if self.Dbuild is None:
            self._rebuild_cost(px, py, gx, gy)
            self.build_gc = self._cell(gx, gy)
            Db = np.full((self.ny, self.nx), np.float32(_BIG), dtype=np.float32)
            Db[self.build_gc] = np.float32(0.0)
            self.Dbuild = Db
            self.build_iters = 0
            self.build_cnt = -1
            self.build_stable = 0

        k = self.iters if self.D is not None else self.iters_boot
        self._relax(self.Dbuild, k)
        self.build_iters += k

        done = False
        if self.build_iters >= 24:
            cnt = int(np.count_nonzero(self.Dbuild < np.float32(_FIN)))
            if cnt == self.build_cnt:
                self.build_stable += 1
            else:
                self.build_stable = 0
            self.build_cnt = cnt
            ry, rx = self._cell(px, py)
            if self.build_stable >= 3 and float(self.Dbuild[ry, rx]) < _FIN:
                done = True
        if self.build_iters >= self.max_iters:
            done = True

        if done:
            self.D = self.Dbuild
            if self.goal_key is not None:
                self.fields[self.goal_key] = self.Dbuild
            self.Dbuild = None

    def _plan_dir(self, px, py, gx, gy):
        """World-frame heading obtained by descending the cost-to-go field."""
        D = self.D
        if D is None:
            return math.atan2(gy - py, gx - px)
        ry, rx = self._cell(px, py)
        if float(D[ry, rx]) >= _FIN:
            y0 = max(0, ry - 6)
            y1 = min(self.ny, ry + 7)
            x0 = max(0, rx - 6)
            x1 = min(self.nx, rx + 7)
            sub = D[y0:y1, x0:x1]
            if not np.any(sub < np.float32(_FIN)):
                return math.atan2(gy - py, gx - px)
            j = int(np.argmin(sub))
            cy = y0 + j // sub.shape[1]
            cx = x0 + j % sub.shape[1]
            wx = self.ox + (cx + 0.5) * self.res
            wy = self.oy + (cy + 0.5) * self.res
            return math.atan2(wy - py, wx - px)

        cy, cx = ry, rx
        wx, wy = px, py
        la2 = self.LOOKAHEAD * self.LOOKAHEAD
        for _ in range(26):
            bv = float(D[cy, cx])
            by, bx = cy, cx
            for dy in (-1, 0, 1):
                yy = cy + dy
                if yy < 0 or yy >= self.ny:
                    continue
                row = D[yy]
                for dx in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    xx = cx + dx
                    if xx < 0 or xx >= self.nx:
                        continue
                    vv = float(row[xx])
                    if vv < bv - 1e-6:
                        bv = vv
                        by, bx = yy, xx
            if by == cy and bx == cx:
                break
            cy, cx = by, bx
            wx = self.ox + (cx + 0.5) * self.res
            wy = self.oy + (cy + 0.5) * self.res
            if (wx - px) ** 2 + (wy - py) ** 2 >= la2:
                break
        if (wx - px) ** 2 + (wy - py) ** 2 < 1e-6:
            return math.atan2(gy - py, gx - px)
        return math.atan2(wy - py, wx - px)

    # =======================================================================
    # reflexes
    # =======================================================================
    def _on_contact(self, t, px, py, yaw, rr):
        sp = math.hypot(self.cmd[0], self.cmd[1])
        if sp > 0.05:
            local = math.atan2(self.cmd[1], self.cmd[0])
        else:
            local = float(self.beam_ang[int(np.argmin(rr))])
        wdir = yaw + local
        # the lidar may not see whatever we touched -- assert it into the map
        d = self._disc(px + 0.34 * math.cos(wdir), py + 0.34 * math.sin(wdir), 0.18)
        if d is not None:
            y0, y1, x0, x1, m = d
            sub = self.logodds[y0:y1, x0:x1]
            sub[m] = np.minimum(sub[m] + np.float32(1.6), np.float32(self.L_MAX))
        self.evade_world = _wrap(wdir + math.pi)
        self.evade_until = t + self.EVADE_T

    def _trigger_recovery(self, t, px, py, clear):
        self.stuck_count += 1
        self.recover_t0 = t
        self.recover_until = t + self.RECOVER_T
        left = float(np.mean(clear[(self.cand > 0.5) & (self.cand < 2.6)])) if np.any(
            (self.cand > 0.5) & (self.cand < 2.6)) else 0.0
        right = float(np.mean(clear[(self.cand < -0.5) & (self.cand > -2.6)])) if np.any(
            (self.cand < -0.5) & (self.cand > -2.6)) else 0.0
        if abs(left - right) < 0.15:
            self.recover_dir = 1.0 if self.rng.random() < 0.5 else -1.0
        else:
            self.recover_dir = 1.0 if left > right else -1.0

        rad = 0.45 + 0.15 * min(4, self.stuck_count)
        val = np.float32(2.0 + 0.8 * min(4, self.stuck_count))
        d = self._disc(px, py, rad)
        if d is not None:
            y0, y1, x0, x1, m = d
            sub = self.blockpen[y0:y1, x0:x1]
            sub[m] = np.minimum(sub[m] + val, np.float32(9.0))

        self.hist.clear()
        self.leg_best = _BIG
        self.Dbuild = None
        if self.goal_key is not None:
            self.fields.pop(self.goal_key, None)

    # =======================================================================
    @staticmethod
    def _num(v, default):
        try:
            f = float(v)
            if f == f and abs(f) < 1e12:
                return f
        except (TypeError, ValueError):
            pass
        return float(default)
