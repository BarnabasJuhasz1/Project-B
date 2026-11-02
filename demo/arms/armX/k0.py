# =============================================================================
#  Go2 shuttle navigator.
#
#  What was taken from the source paper (Feng, Zhang, Lin, Gillis & Moldover,
#  "Determination of the Boltzmann Constant Using the Differential-Cylindrical
#  Procedure") and where it came from:
#
#  1. THE DELETION MECHANISM  (Sec. 3, Eqs. 6-11).  A perturbation that enters
#     linearly in cavity length cancels identically in the difference of two
#     cavities run at L1:L2 = 2:1 with l1 = 2*l2 -- "even if we do not recognise
#     some imperfections, we still know their perturbations will be deleted".
#     Transfer: the steering direction is read out of the geodesic cost field on
#     TWO lookahead rings in the ratio 2:1 (6 cells / 3 cells).  The leading
#     readout error of such a field goes like 1/lookahead (grid quantisation,
#     smeared map, pose drift), so the 2:1 pair lets that whole unmodelled term
#     be deleted by a difference:  theta = theta_far + (theta_far - theta_near).
#     The same "differences, not absolutes" argument is why every progress and
#     velocity estimate here is a finite difference of pose_estimate (slow
#     common-mode drift cancels) and why the collision layer runs purely in the
#     robot frame, where pose error cannot enter at all.
#
#  2. MODE-TO-MODE COMPARISON BEFORE AVERAGING  (Sec. 2.2, Eq. 3; the (200),
#     (300), (400) ratios are only averaged because they agree to <0.1e-6).
#     Transfer: the differential combination above is applied only when the two
#     rings agree; when they disagree the long ring is used alone, and the
#     extrapolated correction is clamped.  A deletion you cannot validate is
#     not used.
#
#  3. THE PAIRED RESONATORS  (Sec. 4: two cavities, same steel block, same
#     vessel, same manifold, one procedure; Sec. 5.1: "the differential
#     procedure relies on the data of each single cavity").  Transfer: the
#     shuttle has exactly two endpoints, so two cost fields are kept warm in
#     parallel under one identical build procedure and cached per endpoint.
#     Turn-around at either end therefore costs no planning time at all.
#
#  4. EXCESS HALF-WIDTH AS A VALIDITY TEST  (Eq. 12: the non-zero excess
#     half-width "measures the extent in which the theoretical prediction
#     differs from the measurement").  Transfer: the residual between commanded
#     displacement and realised displacement, and between intended direction and
#     measured clearance, is the trap/stuck detector.  A non-zero residual is
#     not corrected, it is treated as proof the map is wrong -- it stamps a soft
#     penalty into the cost map and fires recovery.
#
#  5. KEEPING AWAY FROM THE CAVITY'S NATURAL VIBRATIONS, AND EVEN MODES ONLY SO
#     FREE RECOIL CANNOT EXCITE THE RESONANCE  (Sec. 5.1 and Fig. 3: accurate
#     measurement "has to be kept distant away from the natural vibrations";
#     odd modes are longitudinally asymmetric and disturb).  Transfer:
#     asymmetric command rate limits (slow to accelerate, quick to brake) keep
#     the steering loop's bandwidth below the frozen gait's, the direction score
#     is smoothed with a symmetric (even) three-tap kernel so there is no
#     left/right bias, and an explicit anti-oscillation dwell locks one side
#     when the controller starts to ring.
#
#  6. g_fit = b0/sqrt(p) + b1 + b2*sqrt(p), WITH RANDOM ERROR GROWING AS THE
#     SIGNAL WEAKENS  (Eq. 13 and Figs. 4-5: the upwarping at low pressure is
#     "due to low signal-to-noise").  Transfer: lidar information thins with
#     range (fixed angular spacing), so the effective body radius is inflated
#     per beam by half the beam spacing at that range, long-range returns are
#     down-weighted in the map, and commanded speed follows a sqrt law in free
#     clearance -- thin information, slow driving.
# =============================================================================

import math
from collections import deque

import numpy as np

from core.observation import Command

_TWO_PI = 2.0 * math.pi


def _wrap(a):
    """Wrap an angle to (-pi, pi]."""
    return (a + math.pi) % _TWO_PI - math.pi


def _rate_limit(new, old, acc, dec):
    """Asymmetric slew limit: slow to build up, quick to give up (paper item 5)."""
    if abs(new) > abs(old) and new * old >= 0.0:
        lim = acc
    else:
        lim = dec
    d = new - old
    if d > lim:
        d = lim
    elif d < -lim:
        d = -lim
    return old + d


class Controller:
    """Reactive + geodesic-field navigator for the two-point shuttle task."""

    # ---- map / planner geometry -------------------------------------------
    RES_TARGET = 0.22          # desired grid resolution (m)
    MAX_CELLS = 34000          # keeps one relaxation sweep in the ~0.5 ms range
    PAD = 13.0                 # margin (m) around the start/goal bounding box
    OCC_INC = 0.70             # log-odds per occupied return (scaled by range)
    FREE_DEC = 0.17            # log-odds per free-space sample
    LO_MIN, LO_MAX = -2.0, 3.5
    OCC_T = 0.90               # occupancy threshold -> hard obstacle
    UNKNOWN_COST = 1.30        # mild preference for the route already learned
    BAND_COST = 1.40           # soft cost for hugging walls

    # ---- body / safety ----------------------------------------------------
    R_BODY = 0.27              # effective half-width used for clearance
    D_STOP = 0.42              # clearance at which commanded speed reaches 0
    D_FULL = 1.55              # extra clearance for full speed (sqrt law)
    D_REJECT = 0.40            # direction is rejected below this clearance
    FRONT_SLOW = 0.45
    FRONT_STOP = 0.33
    PANIC = 0.26

    # ---- control loop -----------------------------------------------------
    N_CAND = 64                # candidate travel directions over the circle
    VY_MAX = 0.28              # quadrupeds strafe worse than they walk
    ACC_VX, DEC_VX = 0.055, 0.17
    ACC_VY, DEC_VY = 0.050, 0.15
    ACC_WZ, DEC_WZ = 0.105, 0.26

    # ------------------------------------------------------------------ reset
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(0)

        self.max_lin = self._num(scenario, "max_lin", 0.8)
        self.max_ang = self._num(scenario, "max_ang", 1.0)
        self.range_max = self._num(scenario, "range_max", 6.0)
        self.time_limit = self._num(scenario, "time_limit", 1.0e9)
        if not (0.05 < self.max_lin <= 5.0):
            self.max_lin = 0.8
        if not (0.05 < self.max_ang <= 10.0):
            self.max_ang = 1.0
        if not (0.5 < self.range_max <= 100.0):
            self.range_max = 6.0

        sx, sy = self._xy(getattr(scenario, "start", (0.0, 0.0)))
        gx, gy = self._xy(getattr(scenario, "goal", (0.0, 0.0)))

        # ---- grid covering both shuttle endpoints with a generous margin ---
        lo_x = min(sx, gx) - self.PAD
        hi_x = max(sx, gx) + self.PAD
        lo_y = min(sy, gy) - self.PAD
        hi_y = max(sy, gy) + self.PAD
        res = self.RES_TARGET
        for _ in range(60):
            if ((hi_x - lo_x) / res) * ((hi_y - lo_y) / res) <= self.MAX_CELLS:
                break
            res *= 1.08
        self.res = float(res)
        self.inv = 1.0 / self.res
        self.ox, self.oy = float(lo_x), float(lo_y)
        self.nx = int((hi_x - lo_x) * self.inv) + 2
        self.ny = int((hi_y - lo_y) * self.inv) + 2
        self.ncell = self.nx * self.ny

        self.logodds = np.zeros((self.ny, self.nx), np.float32)
        self.seen = np.zeros((self.ny, self.nx), bool)
        self.penalty = np.zeros((self.ny, self.nx), np.float32)

        # ---- the "pair of resonators": one warm cost field per endpoint ----
        self.fields = {}
        self.active = None
        self.build = None
        self.last_build_t = -1.0e9
        self.force_rebuild = True
        self.goal_key = None
        self.trips = 0

        # ---- descent readout rings in the paper's 2:1 ratio ----------------
        self._rings = {}
        for R in (3, 6):
            offs = [(dy, dx)
                    for dy in range(-R, R + 1)
                    for dx in range(-R, R + 1)
                    if max(abs(dy), abs(dx)) == R]
            self._rings[R] = np.asarray(offs, np.int32)
        self.R_NEAR, self.R_FAR = 3, 6

        # ---- lidar caches (built on the first step, angles are fixed) ------
        self._nb = -1
        self._ang = None
        self._cos_b = None
        self._sin_b = None
        self._sinm = None
        self._cosm = None
        self._fd = None
        self.dth = _TWO_PI / 181.0
        self.cand = np.linspace(-math.pi, math.pi, self.N_CAND, endpoint=False)
        self.dca = _TWO_PI / self.N_CAND
        self.m_front = None
        self.m_left = None
        self.m_right = None
        self.m_rear = None

        # ---- control state -------------------------------------------------
        self.k = 0
        self.last_t = None
        self.pvx = 0.0
        self.pvy = 0.0
        self.pwz = 0.0
        self.prev_phi = 0.0
        self.ang_f = None

        # ---- monitors ("excess residual" machinery) ------------------------
        self.hist = deque()
        self.cmdq = deque()
        self.stuck_acc = 0.0
        self.block_acc = 0.0
        self.stuck_times = deque()
        self.best_prog = float("inf")
        self.best_t = 0.0
        self.last_penalty_t = -1.0e9

        # ---- anti-ringing --------------------------------------------------
        self.flips = deque()
        self.commit_until = -1.0e9
        self.commit_phi = 0.0

        # ---- recovery state machine ---------------------------------------
        self.mode = "run"
        self.mode_until = -1.0e9
        self.rec_sign = 1.0
        self.wall_side = 1.0
        self.wall_entry_prog = float("inf")

        # ---- contact sanity (never trust a sensor you cannot validate) -----
        self.use_contact = True
        self.contact_n = 0
        self.contact_true = 0
        self.last_bump_t = -1.0e9

    # ------------------------------------------------------------------ utils
    @staticmethod
    def _num(obj, name, default):
        try:
            v = float(getattr(obj, name))
            if math.isfinite(v):
                return v
        except Exception:
            pass
        return float(default)

    @staticmethod
    def _xy(v):
        try:
            a = np.asarray(v, dtype=float).ravel()
            if a.size >= 2 and math.isfinite(float(a[0])) and math.isfinite(float(a[1])):
                return float(a[0]), float(a[1])
        except Exception:
            pass
        return 0.0, 0.0

    def _cell(self, x, y):
        fx = (x - self.ox) * self.inv
        fy = (y - self.oy) * self.inv
        if not (math.isfinite(fx) and math.isfinite(fy)):
            return None
        ix = int(fx)
        iy = int(fy)
        if 0 <= ix < self.nx and 0 <= iy < self.ny:
            return iy, ix
        return None

    def _idx_of(self, xs, ys):
        fx = (xs - self.ox) * self.inv
        fy = (ys - self.oy) * self.inv
        ok = (np.isfinite(fx) & np.isfinite(fy) &
              (fx >= 0.0) & (fy >= 0.0) &
              (fx < self.nx) & (fy < self.ny))
        if not ok.any():
            return None
        ix = fx[ok].astype(np.int64)
        iy = fy[ok].astype(np.int64)
        return iy * self.nx + ix

    def _disk_offsets(self, rho):
        rho = max(float(rho), 0.0)
        k = int(math.floor(rho))
        r2 = rho * rho + 1e-9
        offs = []
        for dy in range(-k, k + 1):
            for dx in range(-k, k + 1):
                if dy * dy + dx * dx <= r2:
                    offs.append((dy, dx))
        if not offs:
            offs = [(0, 0)]
        return offs

    def _dilate(self, m, offs):
        out = np.zeros_like(m)
        ny, nx = self.ny, self.nx
        for dy, dx in offs:
            y0 = max(0, dy)
            y1 = ny + min(0, dy)
            x0 = max(0, dx)
            x1 = nx + min(0, dx)
            if y1 <= y0 or x1 <= x0:
                continue
            out[y0:y1, x0:x1] |= m[y0 - dy:y1 - dy, x0 - dx:x1 - dx]
        return out

    def _carve(self, blocked, cell, k):
        iy, ix = cell
        y0 = max(0, iy - k)
        y1 = min(self.ny, iy + k + 1)
        x0 = max(0, ix - k)
        x1 = min(self.nx, ix + k + 1)
        blocked[y0:y1, x0:x1] = False

    def _stamp(self, x, y, rad, amount):
        c = self._cell(x, y)
        if c is None:
            return
        iy, ix = c
        k = int(rad * self.inv) + 1
        y0 = max(0, iy - k)
        y1 = min(self.ny, iy + k + 1)
        x0 = max(0, ix - k)
        x1 = min(self.nx, ix + k + 1)
        self.penalty[y0:y1, x0:x1] += amount
        np.clip(self.penalty, 0.0, 40.0, out=self.penalty)

    # ------------------------------------------------------------ beam caches
    def _ensure_beams(self, ang):
        n = int(ang.size)
        if n == self._nb and self._ang is not None:
            if abs(float(ang[0]) - float(self._ang[0])) < 1e-9 and \
               abs(float(ang[-1]) - float(self._ang[-1])) < 1e-9:
                return
        self._nb = n
        self._ang = np.array(ang, dtype=np.float64)
        self._cos_b = np.cos(self._ang)
        self._sin_b = np.sin(self._ang)
        if n > 1:
            d = np.abs(np.diff(self._ang))
            d = d[np.isfinite(d) & (d > 1e-6)]
            self.dth = float(np.median(d)) if d.size else _TWO_PI / max(n, 1)
        else:
            self.dth = _TWO_PI / max(n, 1)
        delta = self._ang[:, None] - self.cand[None, :]
        self._sinm = np.sin(delta).astype(np.float32)
        self._cosm = np.cos(delta).astype(np.float32)
        step = 0.20
        kk = max(2, int(self.range_max / step) + 1)
        self._fd = (np.arange(1, kk + 1, dtype=np.float64) * step)
        a = self._ang
        self.m_front = np.abs(a) < 0.42
        self.m_left = (a > 0.35) & (a < 1.55)
        self.m_right = (a < -0.35) & (a > -1.55)
        self.m_rear = np.abs(a) > 2.60
        for m in (self.m_front, self.m_left, self.m_right, self.m_rear):
            if not m.any():
                m[0] = True

    # ------------------------------------------------------------- map update
    def _update_map(self, r_raw, px, py, yaw, pose_err):
        # Information thins with range (paper item 6): long returns move the
        # map less than short ones, and the free-space trace stops one cell
        # short of the return so walls are not erased by their own beam.
        w = 1.0 / (1.0 + 0.7 * max(0.0, pose_err - 0.3))
        if w < 0.30:
            w = 0.30

        ca = math.cos(yaw)
        sa = math.sin(yaw)
        cb = self._cos_b * ca - self._sin_b * sa
        sb = self._sin_b * ca + self._cos_b * sa

        finite = np.isfinite(r_raw)
        rm = self.range_max
        hit = finite & (r_raw > 0.12) & (r_raw < rm * 0.985)

        rfree = np.where(finite, np.minimum(r_raw, rm) - 0.20, rm * 0.98)
        np.clip(rfree, 0.0, rm * 0.98, out=rfree)

        lo = self.logodds.reshape(-1)
        sv = self.seen.reshape(-1)

        mask = self._fd[None, :] < rfree[:, None]
        if mask.any():
            fx = px + self._fd[None, :] * cb[:, None]
            fy = py + self._fd[None, :] * sb[:, None]
            idx = self._idx_of(fx[mask], fy[mask])
            if idx is not None and idx.size:
                cnt = np.bincount(idx, minlength=self.ncell).astype(np.float32)
                lo -= np.float32(self.FREE_DEC * w) * cnt
                sv[idx] = True

        if hit.any():
            rr = r_raw[hit]
            hx = px + rr * cb[hit]
            hy = py + rr * sb[hit]
            wo = np.clip(1.25 - 0.13 * rr, 0.45, 1.25) * (self.OCC_INC * w)
            fxv = (hx - self.ox) * self.inv
            fyv = (hy - self.oy) * self.inv
            ok = (np.isfinite(fxv) & np.isfinite(fyv) &
                  (fxv >= 0.0) & (fyv >= 0.0) &
                  (fxv < self.nx) & (fyv < self.ny))
            if ok.any():
                idx = (fyv[ok].astype(np.int64) * self.nx +
                       fxv[ok].astype(np.int64))
                cnt = np.bincount(idx, weights=wo[ok],
                                  minlength=self.ncell).astype(np.float32)
                lo += cnt
                sv[idx] = True

        np.clip(lo, self.LO_MIN, self.LO_MAX, out=lo)

    # --------------------------------------------------------------- planner
    def _start_build(self, key, pose_err, robot_cell, t):
        gc = self._cell(key[0], key[1])
        if gc is None:
            return
        occ = self.logodds > self.OCC_T
        rho = (0.20 + 0.25 * min(max(pose_err, 0.0), 1.0)) * self.inv
        blocked = self._dilate(occ, self._disk_offsets(max(rho, 0.7)))
        wide = self._dilate(blocked, self._disk_offsets(max(rho + 1.3, 1.6)))
        band = wide & ~blocked

        cost = np.ones((self.ny, self.nx), np.float32)
        cost[~self.seen] = np.float32(self.UNKNOWN_COST)
        cost += np.float32(self.BAND_COST) * band
        cost += self.penalty
        np.clip(cost, 0.5, 60.0, out=cost)

        # Both terminals of the relaxation must be reachable, whatever the map
        # happens to say about them.
        self._carve(blocked, gc, 2)
        if robot_cell is not None:
            self._carve(blocked, robot_cell, 1)

        cw = cost * np.float32(self.res)
        cd = cost * np.float32(self.res * 1.4142136)

        f = np.full((self.ny, self.nx), np.inf, np.float32)
        g = np.empty_like(f)
        f[gc] = 0.0

        self.build = {
            "key": key, "f": f, "g": g, "cw": cw, "cd": cd,
            "bl": blocked, "gc": gc, "done": 0,
            "maxs": int(self.nx + self.ny + 24), "chk": None,
        }
        self.force_rebuild = False

    def _sweep(self, n):
        b = self.build
        f = b["f"]
        g = b["g"]
        cw = b["cw"]
        cd = b["cd"]
        bl = b["bl"]
        gc = b["gc"]
        inf = np.float32(np.inf)
        for _ in range(n):
            np.copyto(g, f)
            np.minimum(g[1:, :], f[:-1, :] + cw[1:, :], out=g[1:, :])
            np.minimum(g[:-1, :], f[1:, :] + cw[:-1, :], out=g[:-1, :])
            np.minimum(g[:, 1:], f[:, :-1] + cw[:, 1:], out=g[:, 1:])
            np.minimum(g[:, :-1], f[:, 1:] + cw[:, :-1], out=g[:, :-1])
            np.minimum(g[1:, 1:], f[:-1, :-1] + cd[1:, 1:], out=g[1:, 1:])
            np.minimum(g[1:, :-1], f[:-1, 1:] + cd[1:, :-1], out=g[1:, :-1])
            np.minimum(g[:-1, 1:], f[1:, :-1] + cd[:-1, 1:], out=g[:-1, 1:])
            np.minimum(g[:-1, :-1], f[1:, 1:] + cd[:-1, :-1], out=g[:-1, :-1])
            g[bl] = inf
            g[gc] = 0.0
            f, g = g, f
        b["f"] = f
        b["g"] = g
        b["done"] += n

    def _plan_tick(self, t, pose_err, robot_cell):
        if self.build is None:
            need = (self.active is None or self.force_rebuild or
                    (t - self.last_build_t) > 2.0)
            if need and self.goal_key is not None:
                self._start_build(self.goal_key, pose_err, robot_cell, t)
        if self.build is None:
            return
        self._sweep(14 if self.active is None else 7)
        b = self.build
        f = b["f"]
        chk = float(np.minimum(f, np.float32(1.0e4)).sum(dtype=np.float64))
        settled = (b["chk"] is not None and abs(chk - b["chk"]) < 1e-6
                   and b["done"] >= 8)
        b["chk"] = chk
        if settled or b["done"] >= b["maxs"]:
            key = b["key"]
            self.fields[key] = f
            if key == self.goal_key:
                self.active = f
            if len(self.fields) > 4:
                for kk in list(self.fields.keys()):
                    if kk != self.goal_key:
                        del self.fields[kk]
                        break
            self.build = None
            self.last_build_t = t

    # --------------------------------------------- differential field readout
    def _dir_at(self, f, iy, ix, R):
        offs = self._rings[R]
        ys = iy + offs[:, 0]
        xs = ix + offs[:, 1]
        ok = ((ys >= 0) & (ys < self.ny) & (xs >= 0) & (xs < self.nx))
        if not ok.any():
            return None, None
        v = f[ys[ok], xs[ok]]
        o = offs[ok]
        fin = np.isfinite(v)
        if not fin.any():
            return None, None
        v = v[fin].astype(np.float64)
        o = o[fin]
        vmin = float(v.min())
        scale = max(0.35 * R * self.res, 1e-3)
        w = np.exp(-(v - vmin) / scale)
        sw = float(w.sum())
        if sw <= 1e-12:
            return None, vmin
        dy = float((w * o[:, 0]).sum()) / sw
        dx = float((w * o[:, 1]).sum()) / sw
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None, vmin
        return math.atan2(dy, dx), vmin

    def _field_heading(self, px, py):
        """World-frame descent direction, plus geodesic distance to goal."""
        if self.active is None:
            return None, None
        c = self._cell(px, py)
        if c is None:
            return None, None
        iy, ix = c
        f = self.active
        a_n, d_n = self._dir_at(f, iy, ix, self.R_NEAR)
        a_f, d_f = self._dir_at(f, iy, ix, self.R_FAR)

        dval = d_f if d_f is not None else d_n

        if a_f is not None and a_n is not None:
            # Mode-to-mode agreement test, then the 2:1 deletion (paper 1 & 2).
            dlt = _wrap(a_f - a_n)
            if abs(dlt) <= 0.45:
                corr = 0.70 * max(-0.30, min(0.30, dlt))
                return _wrap(a_f + corr), dval
            return a_f, dval
        if a_f is not None:
            return a_f, dval
        if a_n is not None:
            return a_n, dval

        # Boxed in as far as the field knows: aim at the nearest finite cell.
        R = 10
        y0 = max(0, iy - R)
        y1 = min(self.ny, iy + R + 1)
        x0 = max(0, ix - R)
        x1 = min(self.nx, ix + R + 1)
        blk = f[y0:y1, x0:x1]
        fin = np.isfinite(blk)
        if not fin.any():
            return None, None
        vals = np.where(fin, blk, np.float32(np.inf))
        j = int(np.argmin(vals))
        by, bx = divmod(j, vals.shape[1])
        dy = (y0 + by) - iy
        dx = (x0 + bx) - ix
        if dx == 0 and dy == 0:
            return None, None
        return math.atan2(dy, dx), float(vals.flat[j])

    # ------------------------------------------------------------------- step
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            self.pvx *= 0.5
            self.pvy *= 0.5
            self.pwz *= 0.5
            return Command(self.pvx, self.pvy, self.pwz)

    def _step(self, obs):
        # ---------------------------------------------------------- unpack --
        t = self._num(obs, "t", 0.0)
        dt = 0.02 if self.last_t is None else (t - self.last_t)
        if not (0.0005 < dt < 0.5):
            dt = 0.02
        self.last_t = t
        self.k += 1

        ang = np.asarray(getattr(obs, "range_angles"), dtype=np.float64).ravel()
        r_raw = np.asarray(getattr(obs, "ranges"), dtype=np.float64).ravel()
        if ang.size != r_raw.size or r_raw.size < 8:
            return self._emit(0.0, 0.0, 0.0)
        self._ensure_beams(ang)

        pose = np.asarray(getattr(obs, "pose_estimate"), dtype=np.float64).ravel()
        px = float(pose[0]) if pose.size > 0 and math.isfinite(pose[0]) else 0.0
        py = float(pose[1]) if pose.size > 1 and math.isfinite(pose[1]) else 0.0
        yaw = float(pose[2]) if pose.size > 2 and math.isfinite(pose[2]) else 0.0
        pose_err = self._num(obs, "pose_error_estimate", 0.0)
        if not math.isfinite(pose_err) or pose_err < 0.0:
            pose_err = 0.0
        gx, gy = self._xy(getattr(obs, "goal", (px, py)))
        contact = bool(getattr(obs, "contact", False))

        rm = self.range_max
        r = np.where(np.isfinite(r_raw), r_raw, rm)
        r = np.where(r < 0.10, rm, r)
        np.clip(r, 0.10, rm, out=r)
        r32 = r.astype(np.float32)

        # ------------------------------------------------ shuttle bookkeeping
        key = (round(gx, 2), round(gy, 2))
        if self.goal_key is None:
            self.goal_key = key
            self.force_rebuild = True
            self.best_prog = float("inf")
            self.best_t = t
        elif key != self.goal_key:
            # The harness flipped the target: one one-way trip is in the bag.
            self.trips += 1
            self.goal_key = key
            self.active = self.fields.get(key)     # warm field from last leg
            self.build = None
            self.force_rebuild = True
            self.ang_f = None
            self.best_prog = float("inf")
            self.best_t = t
            self.stuck_acc = 0.0
            self.block_acc = 0.0
            self.mode = "run"
            self.commit_until = -1.0e9

        # ------------------------------------------------------- map + plan --
        if (self.k & 1) == 0:
            self._update_map(r_raw, px, py, yaw, pose_err)
        if (self.k % 50) == 0 and self.penalty.any():
            self.penalty *= np.float32(0.97)
        rcell = self._cell(px, py)
        self._plan_tick(t, pose_err, rcell)

        # ------------------------------------------- desired world heading --
        gdx, gdy = gx - px, gy - py
        gdist = math.hypot(gdx, gdy)
        goal_world = math.atan2(gdy, gdx) if gdist > 1e-6 else yaw
        aw, dval = self._field_heading(px, py)
        if aw is None:
            aw = goal_world
            prog = gdist
        else:
            prog = dval if (dval is not None and math.isfinite(dval)) else gdist

        if self.ang_f is None or abs(_wrap(aw - self.ang_f)) > 1.0:
            self.ang_f = aw
        else:
            self.ang_f = _wrap(self.ang_f + 0.35 * _wrap(aw - self.ang_f))
        th = _wrap(self.ang_f - yaw)

        # ------------------------------------------------- clearance field --
        # Per-beam radius inflation: at long range the fixed beam spacing can
        # hide an obstacle edge, so uncertainty grows with range (paper 6).
        reff = np.float32(self.R_BODY) + np.minimum(
            np.float32(0.11), np.float32(0.5 * self.dth) * r32)
        re2 = (reff * reff)[:, None]
        lat = r32[:, None] * self._sinm
        alo = r32[:, None] * self._cosm
        lat2 = lat * lat
        blocking = (lat2 < re2) & (alo > np.float32(-0.02))
        dd = np.where(blocking,
                      alo - np.sqrt(np.maximum(re2 - lat2, np.float32(0.0))),
                      np.float32(9.9))
        clear = dd.min(axis=0).astype(np.float64)
        np.clip(clear, 0.0, 9.9, out=clear)

        fr = r[self.m_front]
        front = float(fr.min()) if fr.size else rm
        lmean = float(r[self.m_left].mean()) if self.m_left.any() else rm
        rmean = float(r[self.m_right].mean()) if self.m_right.any() else rm
        rear = float(r[self.m_rear].min()) if self.m_rear.any() else rm
        imin = int(np.argmin(r))
        rmin = float(r[imin])
        bear_min = float(self._ang[imin])

        kd = int(round((th + math.pi) / self.dca)) % self.N_CAND
        cl_des = float(clear[kd])

        # ------------------------------------- residual monitors (paper 4) --
        self.hist.append((t, px, py))
        while self.hist and (t - self.hist[0][0]) > 1.70:
            self.hist.popleft()
        self.cmdq.append((t, math.hypot(self.pvx, self.pvy) * dt))
        while self.cmdq and (t - self.cmdq[0][0]) > 1.70:
            self.cmdq.popleft()

        t0, x0, y0 = self.hist[0]
        moved = math.hypot(px - x0, py - y0)
        cmdlen = 0.0
        for _, c in self.cmdq:
            cmdlen += c
        span = t - t0
        if cmdlen > 0.40 and span > 1.40:
            deficit = 1.0 - moved / cmdlen
            if deficit > 0.62:
                self.stuck_acc += dt
            else:
                self.stuck_acc = max(0.0, self.stuck_acc - 1.5 * dt)

        if cl_des < 0.55:
            self.block_acc += dt
        else:
            self.block_acc = max(0.0, self.block_acc - 2.0 * dt)

        if prog < self.best_prog - 0.35:
            self.best_prog = prog
            self.best_t = t

        if self.use_contact:
            self.contact_n += 1
            self.contact_true += 1 if contact else 0
            if self.contact_n > 250 and \
                    self.contact_true > 0.60 * self.contact_n:
                self.use_contact = False      # sensor is not informative here

        bump = (self.use_contact and contact and front < 0.80 and
                self.pvx > 0.10 and (t - self.last_bump_t) > 2.0)
        if bump:
            self.last_bump_t = t

        # ------------------------------------------- trap / stuck reactions --
        while self.stuck_times and (t - self.stuck_times[0]) > 20.0:
            self.stuck_times.popleft()

        if self.mode == "run":
            if self.block_acc > 1.0 and (t - self.last_penalty_t) > 1.0:
                # Intent says go, measurement says wall: the map is wrong.
                ow = yaw + th
                self._stamp(px + (cl_des + 0.35) * math.cos(ow),
                            py + (cl_des + 0.35) * math.sin(ow), 0.45, 3.0)
                self.last_penalty_t = t
                self.force_rebuild = True
                self.block_acc = 0.0

            if self.stuck_acc > 0.60 or bump:
                self._stamp(px + 0.45 * math.cos(yaw),
                            py + 0.45 * math.sin(yaw), 0.50, 3.5)
                self.last_penalty_t = t
                self.force_rebuild = True
                self.stuck_acc = 0.0
                self.stuck_times.append(t)
                self.rec_sign = 1.0 if lmean > rmean else -1.0
                if len(self.stuck_times) >= 3 and rmin < 3.0:
                    self.mode = "wall"
                    self.mode_until = t + 6.0
                    self.wall_side = -1.0 if lmean > rmean else 1.0
                    self.wall_entry_prog = prog
                    self.stuck_times.clear()
                elif rear > 0.45:
                    self.mode = "back"
                    self.mode_until = t + 0.55
                else:
                    self.mode = "turn"
                    self.mode_until = t + 0.55
            elif (t - self.best_t) > 8.0 and rmin < 3.0:
                self.mode = "wall"
                self.mode_until = t + 6.0
                self.wall_side = -1.0 if lmean > rmean else 1.0
                self.wall_entry_prog = prog
                self.best_t = t

        if self.mode == "wall":
            if t > self.mode_until or rmin > 3.2 or \
                    prog < self.wall_entry_prog - 1.0:
                self.mode = "run"
                self.best_t = t
                self.stuck_acc = 0.0
                self.block_acc = 0.0
            else:
                # Bug-style wall follow; the local layer still vetoes impacts.
                s = self.wall_side
                corr = max(-0.60, min(0.60, 1.1 * (rmin - 0.85)))
                th = _wrap(bear_min - s * (math.pi * 0.5) + s * corr)
                kd = int(round((th + math.pi) / self.dca)) % self.N_CAND

        # ---------------------------------------- direct recovery overrides --
        if self.mode == "back":
            if t > self.mode_until or rear < 0.35:
                self.mode = "turn"
                self.mode_until = t + 0.50
            else:
                return self._guard(-0.32, 0.0, 0.30 * self.rec_sign,
                                   rmin, bear_min, front, lmean, rmean)
        if self.mode == "turn":
            if t > self.mode_until:
                self.mode = "run"
                self.best_t = t
                self.stuck_acc = 0.0
                self.block_acc = 0.0
            else:
                return self._guard(0.0, 0.0, self.max_ang * self.rec_sign,
                                   rmin, bear_min, front, lmean, rmean)

        # ------------------------------------------- direction selection ----
        ca = self.cand
        align = np.cos(ca - th)
        sc = np.sqrt(np.clip((clear - self.D_STOP) / self.D_FULL, 0.0, 1.0))
        score = 1.70 * align + 1.35 * sc + 0.40 * np.cos(ca - self.prev_phi)
        score -= 12.0 * (clear < self.D_REJECT)
        if t < self.commit_until:
            score += 0.45 * np.cos(ca - self.commit_phi)
        # Symmetric (even) kernel: no left/right bias of its own (paper 5).
        score = 0.25 * np.roll(score, 1) + 0.5 * score + 0.25 * np.roll(score, -1)
        j = int(np.argmax(score))
        phi = float(ca[j])
        cl = float(clear[j])

        # ---- anti-ringing dwell -------------------------------------------
        if phi * self.prev_phi < -0.09 and abs(phi) > 0.25 and \
                abs(self.prev_phi) > 0.25:
            self.flips.append(t)
        while self.flips and (t - self.flips[0]) > 1.5:
            self.flips.popleft()
        if len(self.flips) >= 3 and t >= self.commit_until:
            self.commit_until = t + 1.10
            self.commit_phi = self.prev_phi
            self.flips.clear()
        self.prev_phi = phi

        # ------------------------------------------------ speed and command --
        # sqrt law in free clearance: thin information -> slow (paper 6).
        v = self.max_lin * math.sqrt(
            max(0.0, min(1.0, (cl - self.D_STOP) / self.D_FULL)))
        ahe = abs(phi)
        if ahe > 1.00:
            v *= 0.40
        elif ahe > 0.50:
            v *= 0.75
        if cl < self.D_REJECT:
            v = 0.0

        cs = math.cos(phi)
        sn = math.sin(phi)
        vx = v * max(0.0, cs)
        vy = max(-self.VY_MAX, min(self.VY_MAX, v * sn))
        wz = max(-self.max_ang, min(self.max_ang, 1.90 * phi))

        # Re-check the clearance along the direction actually commanded.
        sp = math.hypot(vx, vy)
        if sp > 1e-3:
            pr = math.atan2(vy, vx)
            kk = int(round((pr + math.pi) / self.dca)) % self.N_CAND
            lim = self.max_lin * math.sqrt(
                max(0.0, min(1.0, (float(clear[kk]) - 0.40) / self.D_FULL)))
            if sp > lim:
                s = lim / sp
                vx *= s
                vy *= s

        return self._guard(vx, vy, wz, rmin, bear_min, front, lmean, rmean)

    # ------------------------------------------------- last-resort collision
    def _guard(self, vx, vy, wz, rmin, bear_min, front, lmean, rmean):
        if front < self.FRONT_SLOW and vx > 0.05:
            vx = 0.05
        if front < self.FRONT_STOP:
            vx = -0.28
            wz = self.max_ang * (1.0 if lmean > rmean else -1.0)
        if rmin < self.PANIC:
            vx = -0.30 * math.cos(bear_min)
            vy = max(-self.VY_MAX, min(self.VY_MAX, -0.22 * math.sin(bear_min)))
            if abs(wz) < 0.25:
                wz = self.max_ang * (1.0 if lmean > rmean else -1.0)
        return self._emit(vx, vy, wz)

    def _emit(self, vx, vy, wz):
        if not math.isfinite(vx):
            vx = 0.0
        if not math.isfinite(vy):
            vy = 0.0
        if not math.isfinite(wz):
            wz = 0.0
        vx = _rate_limit(vx, self.pvx, self.ACC_VX, self.DEC_VX)
        vy = _rate_limit(vy, self.pvy, self.ACC_VY, self.DEC_VY)
        wz = _rate_limit(wz, self.pwz, self.ACC_WZ, self.DEC_WZ)
        vx = max(-self.max_lin, min(self.max_lin, vx))
        vy = max(-self.max_lin, min(self.max_lin, vy))
        wz = max(-self.max_ang, min(self.max_ang, wz))
        self.pvx, self.pvy, self.pwz = vx, vy, wz
        return Command(vx, vy, wz)
