"""
Reactive-plus-mapping navigation controller for a Unitree Go2 shuttling between
two fixed points in an unmapped indoor space.

WHAT WAS TAKEN FROM THE PAPER
(X J Feng, J T Zhang, H Lin, K A Gillis, M R Moldover, "Determination of the
Boltzmann Constant Using the Differential-Cylindrical Procedure")

1. The *deletion mechanism* of the differential procedure (Sec. 3, Eqs. (7)-(11):
   perturbations that act identically on both members of a matched pair cancel in
   the difference, so that "even if we do not recognize some imperfections, we
   still know their perturbations will be deleted"). Carried across twice:
     (a) the map and the goal are both held in the *pose-estimate frame*, so the
         common-mode localisation drift cancels out of the difference
         (goal - pose) that actually steers the robot  (`self.bias`, `_target`);
     (b) steering uses a matched pair of carrot points at arc lengths L2 and
         L1 = 2*L2 along the planned path -- the paper's L1:L2 = 2:1 pair with
         equal admittance (Sec. 3, Sec. 4).  Their *common* bearing drives
         heading; only their *difference* (which isolates path curvature, the
         common frame error having been deleted) is allowed to set speed.
2. Linkage of a working sample to a primary standard through a bridging sample
   (Sec. 2.1: "Sample C bridged a linkage of the analysis in KLPRR CAS to the
   primary gravimetric standard of KRISS"), plus the before/after triple-point-of
   -water calibrations used for long-term stability (Sec. 4).  Here: t = 0 at the
   scenario start point, and every harness switch of obs.goal, are moments when
   the robot is physically standing on a *known* point.  Each such event is a
   primary standard for position; the measured offset pose_estimate - point is
   the frame bias, and the two most recent calibrations give a drift rate exactly
   as the initial/final TPW calibrations bound thermometer drift.
3. Excess half-width as the validity test of the model (Eq. (12): "the examination
   of excess half-width is a crucial assessment of the validity of theoretical
   admittance model").  The controller continuously compares predicted occupancy
   with measured lidar returns at hit points.  That residual is deliberately *not*
   corrected away: like the excess half-width it is used as a trust metric, and it
   inflates obstacles, lowers speed, and accelerates decay of stale map evidence.
4. Keep the operating mode away from the natural vibrations of the structure
   (Sec. 5.1: modes chosen below the 7.2 kHz criterion; even modes chosen because
   the longitudinally symmetric free recoil then "cause[s] no disturbance", while
   odd modes do).  Here: a watchdog on turn-rate sign changes damps the angular
   loop out of its own resonance, and left/right choices are committed with
   hysteresis so the robot cannot flip-flop between two symmetric gaps.
5. Repeated runs in both directions, interleaved to reduce bias and
   non-repeatability (Sec. 2.1 order "C-B-A-C-A-B"; Table 2).  The occupancy map,
   the proven-route trace and the dead-end penalties persist across trips, and
   cells already driven through get a lower traversal cost, so every shuttle run
   is cheaper than the one before.

Imperfections of the locomotion layer below (the analogue of endplate bending and
free recoil, Sec. 3) are never modelled: they are either deleted by the
differential framing or absorbed by the clearance margins.
"""

import math

import numpy as np

from core.observation import Command

_NEI8 = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1))
_TWO_PI = 2.0 * math.pi


def _wrap(a):
    """Wrap angle(s) to [-pi, pi)."""
    return (a + math.pi) % _TWO_PI - math.pi


class Controller:
    # ---- geometry / grid -------------------------------------------------
    RES = 0.35               # planning + mapping cell size [m]
    MAX_CELLS = 150          # hard cap per axis (keeps every sweep cheap)
    PAD = 15.0               # padding around start/goal bounding box [m]
    ROBOT_HALF_W = 0.31      # half body width used for corridor admissibility
    # ---- lidar candidate fan --------------------------------------------
    N_CAND = 60
    # ---- differential carrot pair (paper L1:L2 = 2:1) -------------------
    L_NEAR = 1.25
    L_FAR = 2.50
    # ---- navigation-field budget ----------------------------------------
    ITERS_COLD = 14
    ITERS_WARM = 7
    REBUILD_PERIOD = 1.2
    # ---- scoring weights -------------------------------------------------
    W_GOAL = 2.3
    W_COMFORT = 1.0
    W_FREE = 0.7
    W_HYST = 0.55

    # =====================================================================
    # episode setup
    # =====================================================================
    def reset(self, scenario, rng):
        self.rng = rng

        self.range_max = max(1.0, self._sf(scenario, "range_max", 6.0))
        self.max_lin = max(0.05, self._sf(scenario, "max_lin", 0.8))
        self.max_ang = max(0.05, self._sf(scenario, "max_ang", 1.0))
        self.time_limit = max(1.0, self._sf(scenario, "time_limit", 600.0))

        self.start = self._sp(scenario, "start")
        first_goal = self._sp(scenario, "goal")

        # ---- grid covering both shuttle endpoints with margin ------------
        lo = np.minimum(self.start, first_goal) - self.PAD
        hi = np.maximum(self.start, first_goal) + self.PAD
        span = np.maximum(hi - lo, 4.0)
        res = float(self.RES)
        n = np.ceil(span / res).astype(np.int64) + 2
        if int(n.max()) > self.MAX_CELLS:
            res = float(span.max() + 2.0 * res) / float(self.MAX_CELLS - 2)
            n = np.ceil(span / res).astype(np.int64) + 2
        self.res = float(res)
        self.origin = lo.astype(np.float64)
        self.nx = int(max(4, n[0]))
        self.ny = int(max(4, n[1]))
        ncell = self.nx * self.ny

        self.logodds = np.zeros((self.nx, self.ny), dtype=np.float32)
        self.trace = np.zeros((self.nx, self.ny), dtype=np.uint8)
        self.penalty = np.zeros((self.nx, self.ny), dtype=np.float32)
        self._ncell = ncell

        # ---- navigation field (amortised Bellman-Ford wavefront) ---------
        self.dist = None
        self._bd = None
        self._bc = None
        self._building = False
        self._biters = 0
        self._iter_limit = self.nx + self.ny + 60
        self._prev_nfin = -1
        self._stable = 0
        self._last_build_t = -1e9
        self._build_target = None
        self.force_build = True

        # ---- frame calibration (paper: linkage to a primary standard) ----
        self.bias = np.zeros(2, dtype=np.float64)
        self.bias_prev = np.zeros(2, dtype=np.float64)
        self.bias_rate = np.zeros(2, dtype=np.float64)
        self.bias_t = None
        self.prev_goal = None
        self.trips = 0

        # ---- model-validity residual (paper: excess half-width) ----------
        self.resid_ema = 0.0

        # ---- path / carrots ---------------------------------------------
        self.path = None
        self._path_age = 999
        self.desired = 0.0
        self.kappa = 0.0

        # ---- lidar fan caches -------------------------------------------
        self.cand = np.linspace(-math.pi, math.pi, self.N_CAND, endpoint=False)
        self._ang_sig = None
        self._cosd = None
        self._sind = None
        self._fwd = None
        self._narrow = None
        self._wide = None

        # ---- modes, watchdogs, bookkeeping ------------------------------
        self.k = 0
        self.last_t = None
        self.mode = "nav"
        self.recover_until = -1.0
        self.recover_sign = 1.0
        self.wall_until = -1.0
        self.wall_side = 1.0
        self.wall_entry_d = 1e9
        self.prev_dir_world = 0.0
        self.prev_wz = 0.0
        self.flip_hist = []
        self.damp = 1.0
        self.best_d = 1e9
        self.best_t = 0.0
        self.escapes = 0
        self.contact_count = 0
        self.last_cmd = (0.0, 0.0, 0.0)
        self._fail = 0

    # ---- small scenario helpers ----------------------------------------
    @staticmethod
    def _sf(scenario, name, default):
        try:
            v = float(getattr(scenario, name))
            if math.isfinite(v):
                return v
        except Exception:
            pass
        return float(default)

    @staticmethod
    def _sp(scenario, name):
        try:
            v = np.asarray(getattr(scenario, name), dtype=np.float64).ravel()
            out = np.zeros(2, dtype=np.float64)
            out[: min(2, v.size)] = v[: min(2, v.size)]
            if np.all(np.isfinite(out)):
                return out
        except Exception:
            pass
        return np.zeros(2, dtype=np.float64)

    # =====================================================================
    # grid helpers
    # =====================================================================
    def _cell(self, x, y):
        ix = int((x - self.origin[0]) / self.res)
        iy = int((y - self.origin[1]) / self.res)
        if ix < 0:
            ix = 0
        elif ix > self.nx - 1:
            ix = self.nx - 1
        if iy < 0:
            iy = 0
        elif iy > self.ny - 1:
            iy = self.ny - 1
        return ix, iy

    def _cells(self, xs, ys):
        ix = ((xs - self.origin[0]) / self.res).astype(np.int64)
        iy = ((ys - self.origin[1]) / self.res).astype(np.int64)
        np.clip(ix, 0, self.nx - 1, out=ix)
        np.clip(iy, 0, self.ny - 1, out=iy)
        return ix, iy

    def _center(self, ix, iy):
        return (self.origin[0] + (ix + 0.5) * self.res,
                self.origin[1] + (iy + 0.5) * self.res)

    @staticmethod
    def _dilate(m, k):
        out = m
        for _ in range(int(k)):
            src = out
            out = src.copy()
            out[1:, :] |= src[:-1, :]
            out[:-1, :] |= src[1:, :]
            out[:, 1:] |= src[:, :-1]
            out[:, :-1] |= src[:, 1:]
        return out

    # =====================================================================
    # mapping  (+ paper item 3: excess-residual as a validity test)
    # =====================================================================
    def _update_map(self, px, py, yaw, r, ang, hit):
        ca = np.cos(yaw + ang)
        sa = np.sin(yaw + ang)

        # measured-vs-predicted residual, read BEFORE this scan is folded in
        if np.any(hit):
            hx = px + r[hit] * ca[hit]
            hy = py + r[hit] * sa[hit]
            jx, jy = self._cells(hx, hy)
            if jx.size >= 8:
                prior = self.logodds[jx, jy]
                resid = float(np.mean(prior < -0.7))
                self.resid_ema = 0.97 * self.resid_ema + 0.03 * resid
        else:
            jx = jy = None

        # free space along every beam
        rr = np.minimum(r, self.range_max)
        flen = np.maximum(rr - 0.5 * self.res, 0.0)
        frac = np.linspace(0.08, 0.96, 16)[None, :]
        fx = px + (flen[:, None] * frac) * ca[:, None]
        fy = py + (flen[:, None] * frac) * sa[:, None]
        ix, iy = self._cells(fx.ravel(), fy.ravel())
        flat = ix * self.ny + iy
        cnt = np.bincount(flat, minlength=self._ncell)
        np.minimum(cnt, 3, out=cnt)
        self.logodds -= (0.22 * cnt).reshape(self.nx, self.ny).astype(np.float32)

        if jx is not None and jx.size:
            flath = jx * self.ny + jy
            cnth = np.bincount(flath, minlength=self._ncell)
            np.minimum(cnth, 2, out=cnth)
            self.logodds += (1.0 * cnth).reshape(self.nx, self.ny).astype(np.float32)

        np.clip(self.logodds, -3.0, 4.0, out=self.logodds)

    # =====================================================================
    # navigation field
    # =====================================================================
    def _start_build(self, target, px, py, perr):
        occ = self.logodds

        # decay of stale evidence, rate gated by the validity residual
        if self.resid_ema > 0.30:
            occ *= np.float32(0.93)
        else:
            occ *= np.float32(0.995)
        self.penalty *= np.float32(0.985)

        blocked = occ > 0.75
        rad = 0.30 + min(max(perr, 0.0), 0.6)
        if self.resid_ema > 0.30:
            rad += 0.15
        kinf = int(max(1, min(3, math.ceil(rad / self.res))))
        binf = self._dilate(blocked, kinf)
        near = self._dilate(binf, 1) & (~binf)

        cost = np.ones((self.nx, self.ny), dtype=np.float32)
        unknown = (occ > -0.30) & (~binf)
        cost[unknown] = np.float32(1.30)
        proven = (self.trace > 0) & (~binf)
        cost[proven] = np.float32(0.80)
        cost += np.float32(1.20) * near.astype(np.float32)
        cost += self.penalty

        # never let inflation trap the robot in its own cell
        rx, ry = self._cell(px, py)
        hard = binf.copy()
        hard[max(rx - 1, 0):rx + 2, max(ry - 1, 0):ry + 2] = False
        cost[hard] = np.float32(np.inf)

        d = np.full((self.nx, self.ny), np.inf, dtype=np.float32)
        tx, ty = self._cell(target[0], target[1])
        if hard[tx, ty]:
            found = None
            for rad_c in range(1, 9):
                x0, x1 = max(tx - rad_c, 0), min(tx + rad_c, self.nx - 1)
                y0, y1 = max(ty - rad_c, 0), min(ty + rad_c, self.ny - 1)
                sub = hard[x0:x1 + 1, y0:y1 + 1]
                ok = np.argwhere(~sub)
                if ok.size:
                    cx = ok[:, 0] + x0
                    cy = ok[:, 1] + y0
                    dd = (cx - tx) ** 2 + (cy - ty) ** 2
                    j = int(np.argmin(dd))
                    found = (int(cx[j]), int(cy[j]))
                    break
            if found is not None:
                tx, ty = found
        d[tx, ty] = 0.0

        self._bd = d
        self._bc = cost
        self._biters = 0
        self._prev_nfin = -1
        self._stable = 0
        self._building = True
        self._build_target = np.array(target, dtype=np.float64)

    def _relax(self, n, t):
        d = self._bd
        c = self._bc
        if d is None or not self._building:
            return
        for _ in range(int(n)):
            nb = np.empty_like(d)
            nb.fill(np.inf)
            nb[1:, :] = d[:-1, :]
            np.minimum(nb[:-1, :], d[1:, :], out=nb[:-1, :])
            np.minimum(nb[:, 1:], d[:, :-1], out=nb[:, 1:])
            np.minimum(nb[:, :-1], d[:, 1:], out=nb[:, :-1])

            dg = np.empty_like(d)
            dg.fill(np.inf)
            dg[1:, 1:] = d[:-1, :-1]
            np.minimum(dg[1:, :-1], d[:-1, 1:], out=dg[1:, :-1])
            np.minimum(dg[:-1, 1:], d[1:, :-1], out=dg[:-1, 1:])
            np.minimum(dg[:-1, :-1], d[1:, 1:], out=dg[:-1, :-1])

            cand = np.minimum(nb + c, dg + np.float32(1.41421) * c)
            np.minimum(d, cand, out=d)
            self._biters += 1

        nfin = int(np.isfinite(d).sum())
        if nfin == self._prev_nfin:
            self._stable += 1
        else:
            self._stable = 0
        self._prev_nfin = nfin

        if self._stable >= 2 or self._biters >= self._iter_limit:
            self.dist = d
            self._building = False
            self._last_build_t = t
            self._path_age = 999

    # ---- downhill trace through the field ------------------------------
    def _trace_path(self, px, py):
        d = self.dist
        if d is None:
            return None
        ix, iy = self._cell(px, py)
        dv = float(d[ix, iy])
        if not math.isfinite(dv):
            found = None
            for rad_c in range(1, 11):
                x0, x1 = max(ix - rad_c, 0), min(ix + rad_c, self.nx - 1)
                y0, y1 = max(iy - rad_c, 0), min(iy + rad_c, self.ny - 1)
                sub = d[x0:x1 + 1, y0:y1 + 1]
                ok = np.argwhere(np.isfinite(sub))
                if ok.size:
                    cx = ok[:, 0] + x0
                    cy = ok[:, 1] + y0
                    dd = (cx - ix) ** 2 + (cy - iy) ** 2
                    j = int(np.argmin(dd))
                    ix, iy = int(cx[j]), int(cy[j])
                    dv = float(d[ix, iy])
                    found = True
                    break
            if found is None:
                return None

        pts = [self._center(ix, iy)]
        cx, cy = ix, iy
        for _ in range(45):
            bv = dv - 1e-5
            bi = bj = -1
            for dx, dy in _NEI8:
                jx = cx + dx
                jy = cy + dy
                if jx < 0 or jy < 0 or jx >= self.nx or jy >= self.ny:
                    continue
                v = float(d[jx, jy])
                if v < bv:
                    bv = v
                    bi, bj = jx, jy
            if bi < 0:
                break
            cx, cy = bi, bj
            dv = bv
            pts.append(self._center(cx, cy))
            if dv <= 0.0:
                break
        return np.asarray(pts, dtype=np.float64)

    @staticmethod
    def _carrot(poly, length):
        """Point at arc `length` along the polyline (last point if shorter)."""
        seg = np.linalg.norm(np.diff(poly, axis=0), axis=1)
        if seg.size == 0:
            return poly[-1], 0.0
        cum = np.cumsum(seg)
        total = float(cum[-1])
        if length >= total:
            return poly[-1], total
        i = int(np.searchsorted(cum, length))
        i = min(i, poly.shape[0] - 2)
        prev = 0.0 if i == 0 else float(cum[i - 1])
        s = seg[i]
        f = 0.0 if s <= 1e-9 else max(0.0, min(1.0, (length - prev) / s))
        return poly[i] + f * (poly[i + 1] - poly[i]), total

    # =====================================================================
    # control step
    # =====================================================================
    def step(self, obs):
        try:
            cmd = self._step(obs)
            self.last_cmd = cmd
            return Command(cmd[0], cmd[1], cmd[2])
        except Exception:
            self._fail += 1
            vx, vy, wz = self.last_cmd
            return Command(0.0, 0.0, 0.3 if abs(wz) < 1e-3 else
                           max(-self.max_ang, min(self.max_ang, 0.7 * wz)))

    def _step(self, obs):
        self.k += 1
        t = self._sf(obs, "t", (self.k) * 0.02)
        if self.last_t is None:
            self.last_t = t
            self.best_t = t
        dt = t - self.last_t
        if not (1e-4 < dt < 1.0):
            dt = 0.02
        self.last_t = t

        pose = np.asarray(obs.pose_estimate, dtype=np.float64).ravel()
        px = float(pose[0]); py = float(pose[1])
        yaw = float(pose[2]) if pose.size > 2 else 0.0
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(yaw)):
            return (0.0, 0.0, 0.0)
        perr = self._sf(obs, "pose_error_estimate", 0.0)
        if not math.isfinite(perr):
            perr = 0.0
        contact = bool(getattr(obs, "contact", False))

        goal = np.asarray(obs.goal, dtype=np.float64).ravel()[:2]
        if goal.size < 2 or not np.all(np.isfinite(goal)):
            goal = self.start.copy()

        # ---- lidar sanitation -------------------------------------------
        r_raw = np.asarray(obs.ranges, dtype=np.float64).ravel()
        ang = np.asarray(obs.range_angles, dtype=np.float64).ravel()
        m = min(r_raw.size, ang.size)
        r_raw = r_raw[:m]
        ang = ang[:m]
        good = np.isfinite(r_raw) & (r_raw > 0.02)
        r = np.where(good, np.minimum(r_raw, self.range_max), self.range_max)
        hit = good & (r_raw < self.range_max * 0.985)
        self._ensure_geom(ang)

        # =================================================================
        # frame calibration against known points (paper item 2)
        # =================================================================
        if self.prev_goal is None:
            self.prev_goal = goal.copy()
            bm = np.array([px, py]) - self.start
            if float(np.hypot(bm[0], bm[1])) < 3.0:
                self.bias = bm
                self.bias_prev = bm.copy()
                self.bias_t = t
        elif float(np.hypot(*(goal - self.prev_goal))) > 0.75:
            # the harness only switches the goal once the robot has arrived:
            # the robot is standing on the OLD goal, a primary position standard
            self.trips += 1
            bm = np.array([px, py]) - self.prev_goal
            if float(np.hypot(bm[0], bm[1])) < 5.0:
                newb = 0.5 * self.bias + 0.5 * bm
                np.clip(newb, -2.5, 2.5, out=newb)
                if self.bias_t is not None and (t - self.bias_t) > 5.0:
                    rate = (newb - self.bias_prev) / (t - self.bias_t)
                    self.bias_rate = np.clip(rate, -0.01, 0.01)
                self.bias = newb
                self.bias_prev = newb.copy()
                self.bias_t = t
            self.prev_goal = goal.copy()
            self.force_build = True
            self.mode = "nav"
            self.wall_until = -1.0
            self.best_d = 1e9
            self.best_t = t

        ext = np.zeros(2)
        if self.bias_t is not None:
            ext = np.clip(self.bias_rate * (t - self.bias_t), -0.4, 0.4)
        target = goal + self.bias + ext
        dgoal = float(np.hypot(target[0] - px, target[1] - py))

        # =================================================================
        # map maintenance
        # =================================================================
        if self.k % 5 == 0:
            cx, cy = self._cell(px, py)
            self.trace[cx, cy] = 1
        if self.k % 3 == 0 and m > 0:
            self._update_map(px, py, yaw, r, ang, hit)
        if contact:
            self.contact_count += 1
            bx = px + 0.40 * math.cos(yaw)
            by = py + 0.40 * math.sin(yaw)
            cx, cy = self._cell(bx, by)
            self.logodds[cx, cy] = min(4.0, float(self.logodds[cx, cy]) + 2.0)

        # =================================================================
        # navigation field (amortised)
        # =================================================================
        moved = (self._build_target is None or
                 float(np.hypot(*(target - self._build_target))) > 0.9)
        if self.force_build or (not self._building and
                                (self.dist is None or moved or
                                 (t - self._last_build_t) > self.REBUILD_PERIOD)):
            self._start_build(target, px, py, perr)
            self.force_build = False
        self._relax(self.ITERS_COLD if self.dist is None else self.ITERS_WARM, t)

        # ---- differential carrot pair (paper item 1b) --------------------
        self._path_age += 1
        if self._path_age >= 4:
            self.path = self._trace_path(px, py)
            self._path_age = 0
        direct = _wrap(math.atan2(target[1] - py, target[0] - px) - yaw)
        if self.path is not None and self.path.shape[0] >= 1:
            poly = np.vstack((np.array([[px, py]]), self.path))
            cn, total = self._carrot(poly, self.L_NEAR)
            cf, _ = self._carrot(poly, self.L_FAR)
            bn = math.atan2(cn[1] - py, cn[0] - px)
            bf = math.atan2(cf[1] - py, cf[0] - px)
            nav_des = _wrap(bn - yaw)
            self.kappa = abs(_wrap(bf - bn))      # common bearing deleted
            if total < 0.8:
                nav_des = direct
                self.kappa = 0.0
        else:
            nav_des = direct
            self.kappa = 0.0

        # clearance of every candidate direction
        dfree, dcomf = self._clearances(r)
        ifwd = int(np.argmin(np.abs(self.cand)))
        d_ahead = float(dfree[ifwd])

        # final-approach shortcut: aim straight at the standard if it is clear
        if dgoal < 2.5:
            idir = int(np.argmin(np.abs(_wrap(self.cand - direct))))
            if float(dfree[idir]) > min(dgoal + 0.2, 1.3):
                nav_des = direct
                self.kappa = 0.0

        # =================================================================
        # mode arbitration
        # =================================================================
        if contact or d_ahead < 0.28:
            if t > self.recover_until:
                left = float(np.min(r[(ang > 0.25) & (ang < 1.6)])) if np.any(
                    (ang > 0.25) & (ang < 1.6)) else self.range_max
                right = float(np.min(r[(ang < -0.25) & (ang > -1.6)])) if np.any(
                    (ang < -0.25) & (ang > -1.6)) else self.range_max
                self.recover_sign = 1.0 if left >= right else -1.0
            self.recover_until = max(self.recover_until, t + 0.45)
            self.mode = "recover"

        if self.mode == "recover":
            if t < self.recover_until:
                self.prev_dir_world = yaw + math.pi
                wz = self.recover_sign * 0.85 * self.max_ang
                self._log_wz(wz)
                return (-0.35 * self.max_lin, 0.0, wz)
            self.mode = "nav"
            self.force_build = True

        # ---- stuck detection / escape escalation ------------------------
        if dgoal < self.best_d - 0.2:
            self.best_d = dgoal
            self.best_t = t
        if (t - self.best_t) > 8.0 and self.mode != "wall":
            self.escapes += 1
            cx, cy = self._cell(px, py)
            x0, x1 = max(cx - 2, 0), min(cx + 3, self.nx)
            y0, y1 = max(cy - 2, 0), min(cy + 3, self.ny)
            self.penalty[x0:x1, y0:y1] += np.float32(2.0)
            np.clip(self.penalty, 0.0, 12.0, out=self.penalty)
            self.force_build = True
            self.best_t = t
            self.best_d = dgoal
            if self.escapes % 2 == 1:
                self.mode = "wall"
                self.wall_until = t + 7.0
                self.wall_entry_d = dgoal
                self.wall_side = self._pick_wall_side(r, ang, nav_des)

        desired = nav_des
        if self.mode == "wall":
            if t > self.wall_until or dgoal < self.wall_entry_d - 1.3:
                self.mode = "nav"
                self.best_t = t
                self.best_d = dgoal
                self.force_build = True
            else:
                wd = self._wall_direction(r, ang)
                if wd is None:
                    self.mode = "nav"
                else:
                    desired = wd

        # =================================================================
        # candidate selection (hysteresis = paper item 4)
        # =================================================================
        prev_rel = _wrap(self.prev_dir_world - yaw)
        goal_term = np.cos(_wrap(self.cand - desired))
        score = (self.W_GOAL * goal_term
                 + self.W_COMFORT * np.clip(dcomf / 2.5, 0.0, 1.0)
                 + self.W_FREE * np.clip(dfree / 3.0, 0.0, 1.0)
                 - self.W_HYST * np.abs(_wrap(self.cand - prev_rel)) / math.pi
                 * (2.0 - self.damp))
        if self.mode == "wall":
            score += 0.6 * np.clip(dcomf / 2.5, 0.0, 1.0)

        admissible = dfree > 0.55
        if np.any(admissible):
            sc = np.where(admissible, score, -1e9)
            i = int(np.argmax(sc))
            rotate_only = False
        else:
            i = int(np.argmax(dfree))
            rotate_only = True
        a = float(self.cand[i])
        self.prev_dir_world = _wrap(yaw + a)

        # =================================================================
        # command synthesis
        # =================================================================
        v_cl = max(0.0, min(1.0, (float(dfree[i]) - 0.50) / 1.10))
        v_curv = 1.0 / (1.0 + 1.1 * self.kappa)          # from the pair difference
        trust = 1.0 - 0.35 * min(1.0, self.resid_ema / 0.35)
        trust *= 1.0 - 0.25 * min(1.0, perr / 1.5)
        V = self.max_lin * v_cl * v_curv * max(0.25, trust)

        ca = math.cos(a)
        sa = math.sin(a)
        vx = V * max(0.0, ca)
        vy = V * sa * 0.45
        if abs(a) > 1.15:
            vx = min(vx, 0.08 * self.max_lin)
            vy *= 0.3
        if rotate_only or float(dfree[i]) < 0.50:
            vx = 0.0
            vy *= 0.3

        # lateral repulsion from close side obstacles
        ml = (ang > 0.3) & (ang < 1.5)
        mr = (ang < -0.3) & (ang > -1.5)
        lmin = float(np.min(r[ml])) if np.any(ml) else self.range_max
        rmin = float(np.min(r[mr])) if np.any(mr) else self.range_max
        if lmin < 0.75:
            vy -= 0.30 * self.max_lin * (0.75 - lmin) / 0.75
        if rmin < 0.75:
            vy += 0.30 * self.max_lin * (0.75 - rmin) / 0.75

        wz = 2.2 * _wrap(a) * self.damp
        self._log_wz(wz)

        lim_y = min(0.35, self.max_lin)
        vx = max(-self.max_lin, min(self.max_lin, vx))
        vy = max(-lim_y, min(lim_y, vy))
        wz = max(-self.max_ang, min(self.max_ang, wz))
        if not math.isfinite(vx):
            vx = 0.0
        if not math.isfinite(vy):
            vy = 0.0
        if not math.isfinite(wz):
            wz = 0.0
        return (vx, vy, wz)

    # =====================================================================
    # helpers used by the control step
    # =====================================================================
    def _ensure_geom(self, ang):
        sig = (int(ang.size),
               round(float(ang[0]), 6) if ang.size else 0.0,
               round(float(ang[-1]), 6) if ang.size else 0.0)
        if self._ang_sig == sig and self._cosd is not None:
            return
        delta = _wrap(ang[None, :] - self.cand[:, None])
        self._cosd = np.cos(delta)
        self._sind = np.sin(delta)
        self._fwd = self._cosd > 0.15
        lat_unit = np.abs(self._sind)
        self._narrow = self._fwd
        self._wide = self._fwd
        self._lat_unit = lat_unit
        self._ang_sig = sig

    def _clearances(self, r):
        """Corridor clearance of each candidate heading (narrow and comfort)."""
        lat = np.abs(r[None, :] * self._sind)
        along = r[None, :] * self._cosd
        big = self.range_max + 10.0
        bn = self._fwd & (lat < self.ROBOT_HALF_W)
        bw = self._fwd & (lat < self.ROBOT_HALF_W + 0.28)
        dn = np.where(bn, along, big).min(axis=1)
        dw = np.where(bw, along, big).min(axis=1)
        np.minimum(dn, self.range_max, out=dn)
        np.minimum(dw, self.range_max, out=dw)
        return dn, dw

    def _pick_wall_side(self, r, ang, nav_des):
        hits = r < 3.0
        if not np.any(hits):
            return 1.0 if nav_des >= 0.0 else -1.0
        j = int(np.argmin(np.where(hits, r, self.range_max + 1.0)))
        b = float(ang[j])
        t_left = _wrap(b - 0.5 * math.pi)    # wall kept on the left
        t_right = _wrap(b + 0.5 * math.pi)
        if abs(_wrap(t_left - nav_des)) <= abs(_wrap(t_right - nav_des)):
            return 1.0
        return -1.0

    def _wall_direction(self, r, ang):
        hits = r < 3.0
        if not np.any(hits):
            return None
        masked = np.where(hits, r, self.range_max + 1.0)
        j = int(np.argmin(masked))
        b = float(ang[j])
        dmin = float(r[j])
        side = self.wall_side
        err = dmin - 0.85
        corr = max(-0.7, min(0.7, 1.0 * err))
        return _wrap(b - side * 0.5 * math.pi + side * corr)

    def _log_wz(self, wz):
        """Angular-loop resonance watchdog (paper item 4)."""
        self.flip_hist.append(1 if (wz * self.prev_wz) < -0.004 else 0)
        if len(self.flip_hist) > 25:
            del self.flip_hist[0]
        flips = sum(self.flip_hist)
        if flips >= 8:
            self.damp = max(0.45, self.damp * 0.90)
        else:
            self.damp = min(1.0, self.damp + 0.01)
        self.prev_wz = wz
