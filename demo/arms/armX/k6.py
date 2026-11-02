# ============================================================================
#  Go2 shuttle navigation controller
# ============================================================================
#
#  WHAT CAME FROM THE PAPER
#  (X J Feng, J T Zhang, H Lin, K A Gillis, M R Moldover, "Determination of the
#   Boltzmann Constant Using the Differential - Cylindrical Procedure")
#
#  1. The differential deletion mechanism (Sec. 3, Eqs. 6-11): if two matched
#     probes see a perturbation with equal admittance over equal area, that
#     perturbation cancels in their *difference* and never has to be modelled
#     ("even if we do not recognize some imperfections, we still know their
#     perturbations will be deleted"). Used twice here:
#       (a) `_diff_steer` integrates two mirrored lidar sectors and uses only
#           their difference, so clutter that is symmetric about the heading
#           (a corridor, a symmetric doorway) deletes itself and only the
#           asymmetry steers -> corridor centring with no corridor model.
#       (b) pose drift is obtained as the difference between two independent
#           displacement measurements - dead-reckoned `pose_estimate` and a
#           map scan-match - instead of modelling the drift sources.
#  2. "The differential procedure relies on the data of each single cavity"
#     (Sec. 5.1): each arm must be validated alone before differencing. Here a
#     correction is used only if its own arm passes a beam-count and
#     match-score test first.
#  3. Excess half-width as the validity test of the admittance model (Eq. 12,
#     Fig. 4): measured-minus-predicted residual is the trust metric, and the
#     "upwarping at low pressures" warns that residuals from few / low-SNR
#     samples must be discounted. Here the per-beam scan/map residual gates
#     (i) accepting a pose correction, (ii) the weight of map writes,
#     (iii) commanded speed; and residuals from <24 beams are thrown away.
#  4. Mode selection (Sec. 5.1 + Fig. 3): keep excitation away from the
#     cavity's natural-vibration band (>7.2 kHz) and keep the modes even, i.e.
#     longitudinally symmetric, so asymmetric free recoil - which is "extremely
#     hard to be quantitatively characterized" - causes no disturbance.
#     Here commands are rate-limited and dead-banded to stay below the body's
#     own weaving band, and yaw-rate *sign reversals* (the asymmetric mode) are
#     explicitly penalised in the local scoring, so the steering response stays
#     symmetric and never rings.
#  5. The acoustic virial series with the A_-1/p wall-slip term (Eq. 2) and the
#     known p^(+-1/2) basis of the half-width fit (Eq. 13): the speed schedule
#     is a short series in clearance *with an inverse-clearance boundary-layer
#     term*, a physically motivated basis rather than a free-form gain.
#  6. Mode-to-mode comparison by ratio and non-repeatability bookkeeping
#     (Eq. 3, Table 2): two redundant progress estimates (commanded path length
#     vs. realised displacement) are compared as a ratio to detect slipping /
#     being stuck, and the running non-repeatability of accepted corrections
#     sets how much of each new correction is applied.
#
#  Not carried across: anything about gases, resonator hardware, or absolute
#  metrology - the robot has no "pressure isotherm" to fit, so Eq. (2) is used
#  only for its functional shape, not its coefficients.
# ============================================================================

import math
from collections import deque
from heapq import heappush, heappop

import numpy as np

from core.observation import Command

TWO_PI = 2.0 * math.pi
INF = float("inf")


def _wrap(a):
    return (a + math.pi) % TWO_PI - math.pi


def _wrap_arr(a):
    return (np.asarray(a, dtype=float) + math.pi) % TWO_PI - math.pi


class Controller:
    # ---- map / planner constants -------------------------------------------
    RES = 0.15               # fine occupancy resolution [m]
    OCC_GAIN = 0.85
    FREE_GAIN = 0.12
    LOG_MIN = -2.5
    LOG_MAX = 4.5
    OCC_TH = 0.50
    FREE_TH = -0.25
    PLAN_DT = 0.40
    MATCH_DT = 0.30
    MAX_POPS = 9000
    HW = 0.70                # A* heuristic weight == minimum cell cost

    # ---- local planner constants -------------------------------------------
    R_HARD = 0.27            # body radius used against raw lidar [m]
    HORIZON = 1.0            # rollout horizon [s]
    NSAMP = 5
    LOOKAHEAD = 0.95

    NB = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
          (1, 1, 1.41421356), (1, -1, 1.41421356),
          (-1, 1, 1.41421356), (-1, -1, 1.41421356))

    # =====================================================================
    #  reset
    # =====================================================================
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(12345)

        self.range_max = float(getattr(scenario, "range_max", 6.0) or 6.0)
        self.max_lin = float(getattr(scenario, "max_lin", 0.8) or 0.8)
        self.max_ang = float(getattr(scenario, "max_ang", 1.0) or 1.0)
        self.time_limit = float(getattr(scenario, "time_limit", 600.0) or 600.0)

        start = self._as_xy(getattr(scenario, "start", (0.0, 0.0)))
        goal0 = self._as_xy(getattr(scenario, "goal", (0.0, 0.0)))

        # ---- grid sized around the two shuttle endpoints -------------------
        center = 0.5 * (start + goal0)
        span = float(np.linalg.norm(goal0 - start))
        half = max(16.0, 0.75 * span + 14.0)
        n = int(2.0 * half / self.RES)
        n = int(min(max(n, 120), 520))
        if n % 2:
            n += 1
        self.n = n
        self.m = n // 2
        self.cres = 2.0 * self.RES
        self._inv = 1.0 / self.RES
        self.ox = float(center[0] - 0.5 * n * self.RES)
        self.oy = float(center[1] - 0.5 * n * self.RES)

        self.log = np.zeros((n, n), dtype=np.float32)
        self.pen = np.zeros((self.m, self.m), dtype=np.float32)
        self._hard = np.zeros((self.m, self.m), dtype=bool)
        self._occ_c = np.zeros((self.m, self.m), dtype=bool)

        # ---- shuttle bookkeeping ------------------------------------------
        self.goal_pts = []
        self.cur_target = None
        self.routes = {}
        self.trips = 0
        self.trail = []
        self.last_trail_pt = None

        # ---- pose correction (differential arm b) -------------------------
        self.T = np.zeros(3, dtype=float)
        self.map_trust = 0.60
        self.corr_hist = deque(maxlen=12)

        # ---- control state ------------------------------------------------
        self.prev_cmd = np.zeros(3, dtype=float)
        self.path = None
        self.path_i = 0
        self.last_plan_t = -9.0
        self.last_match_t = -9.0
        self.plan_fail = 0
        self.goal_prev = None
        self.mode = "nav"
        self.mode_until = 0.0
        self.recover_n = 0
        self.recover_dir = 1.0
        self.hist = deque(maxlen=400)
        self.k = 0
        self.t_prev = 0.0
        self.integ_count = 0
        self.collisions = 0
        self.contact_t = -9.0
        self.explore_bearing = 0.0

        # ---- candidate velocity set ---------------------------------------
        degs = np.array([0.0, 12.0, -12.0, 26.0, -26.0, 42.0, -42.0,
                         62.0, -62.0, 86.0, -86.0])
        dirs = np.radians(degs)
        fracs = np.array([1.0, 0.62, 0.32])
        self.cand_th = np.repeat(dirs, fracs.size)
        self.cand_fr = np.tile(fracs, dirs.size)
        self.cand_extra = np.array([
            [0.00, 0.0, 1.00],
            [0.00, 0.0, -1.00],
            [0.00, 0.0, 0.50],
            [0.00, 0.0, -0.50],
            [-0.30, 0.0, 0.45],
            [-0.30, 0.0, -0.45],
            [-0.18, 0.0, 0.00],
        ], dtype=float)

    # =====================================================================
    #  step
    # =====================================================================
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            # never raise: creep-and-scan fallback
            return Command(0.0, 0.0, 0.35)

    # ------------------------------------------------------------------ core
    def _step(self, obs):
        t = float(getattr(obs, "t", self.k * 0.02))
        if self.k == 0:
            dt = 0.02
        else:
            dt = float(np.clip(t - self.t_prev, 1e-3, 0.1))
        self.t_prev = t
        self.k += 1

        ranges = np.asarray(obs.ranges, dtype=float).ravel()
        angles = np.asarray(obs.range_angles, dtype=float).ravel()
        if angles.size != ranges.size or angles.size == 0:
            angles = np.linspace(-math.pi, math.pi, max(ranges.size, 1),
                                 endpoint=False)
        est = np.asarray(obs.pose_estimate, dtype=float).ravel()
        if est.size < 3:
            est = np.array([est[0] if est.size else 0.0,
                            est[1] if est.size > 1 else 0.0, 0.0])
        goal = self._as_xy(obs.goal)
        try:
            perr = float(getattr(obs, "pose_error_estimate", 0.1))
        except Exception:
            perr = 0.1
        if not math.isfinite(perr):
            perr = 0.3
        contact = bool(getattr(obs, "contact", False))

        pose = self._apply_T(est)

        # ---- shuttle endpoint switch --------------------------------------
        if self.goal_prev is None or float(np.linalg.norm(goal - self.goal_prev)) > 0.5:
            self._on_goal_change(goal, pose, t)
        self.goal_prev = goal.copy()

        dist_goal = float(math.hypot(goal[0] - pose[0], goal[1] - pose[1]))

        # ---- map integration ----------------------------------------------
        trust = float(np.clip(self.map_trust / 0.60, 0.40, 1.0))
        if self.k % 2 == 1:
            self._integrate_scan(pose, ranges, angles, trust)

        # ---- contact -> hard evidence + immediate retreat ------------------
        if contact:
            if t - self.contact_t > 0.6:
                self.collisions += 1
                self._mark_contact(pose, ranges, angles)
                self._enter_recovery(t, pose, ranges, angles, hard=True)
            self.contact_t = t

        # ---- differential pose correction ---------------------------------
        if t - self.last_match_t > self.MATCH_DT and t > 0.8:
            self.last_match_t = t
            self._scan_match(pose, est, ranges, angles, perr)
            pose = self._apply_T(est)

        # ---- breadcrumb trail for the route prior -------------------------
        if self.last_trail_pt is None or \
                math.hypot(pose[0] - self.last_trail_pt[0],
                           pose[1] - self.last_trail_pt[1]) > 0.35:
            self.last_trail_pt = (pose[0], pose[1])
            if len(self.trail) < 600:
                self.trail.append((pose[0], pose[1]))

        # ---- progress / stuck monitor (Eq. 3 style ratio test) ------------
        cmd_step = float(math.hypot(self.prev_cmd[0], self.prev_cmd[1]) * dt)
        self.hist.append((t, pose[0], pose[1], dist_goal, cmd_step))
        while len(self.hist) > 2 and t - self.hist[0][0] > 4.2:
            self.hist.popleft()
        if self.mode == "nav" and len(self.hist) > 10 and \
                (t - self.hist[0][0]) > 3.0:
            t0, x0, y0, d0, _ = self.hist[0]
            disp = math.hypot(pose[0] - x0, pose[1] - y0)
            cmd_len = sum(h[4] for h in self.hist)
            improve = d0 - dist_goal
            slipping = (cmd_len > 1.0 and disp < 0.32)
            circling = (improve < 0.20 and disp < 0.55)
            if slipping or circling:
                self._enter_recovery(t, pose, ranges, angles, hard=False)

        # ---- sector-differential asymmetry (common mode deleted) ----------
        bal, front = self._diff_steer(ranges, angles)

        # ---- mode machine --------------------------------------------------
        cmd = None
        carrot = None

        if self.mode == "back":
            rear = self._cone_min(ranges, angles, math.pi, math.radians(55))
            if t >= self.mode_until or rear < 0.45:
                self.mode = "turn"
                self.mode_until = t + 0.75
            else:
                cmd = (-0.32, 0.0, 0.45 * self.recover_dir * self.max_ang)

        if cmd is None and self.mode == "turn":
            if t >= self.mode_until:
                if self.recover_n >= 2:
                    self.mode = "explore"
                    self.mode_until = t + 2.5
                    self.explore_bearing = self._open_bearing(
                        ranges, angles, self._bearing_to(pose, goal))
                else:
                    self.mode = "nav"
                    self.path = None
            else:
                cmd = (0.04, 0.0, self.recover_dir * self.max_ang)

        if cmd is None and self.mode == "explore":
            if t >= self.mode_until:
                self.mode = "nav"
                self.path = None
                self.hist.clear()
            else:
                self.explore_bearing = 0.75 * self.explore_bearing + 0.25 * \
                    self._open_bearing(ranges, angles,
                                       self._bearing_to(pose, goal))
                reach = float(np.clip(front - 0.5, 0.7, 3.0))
                th = pose[2] + self.explore_bearing
                carrot = np.array([pose[0] + reach * math.cos(th),
                                   pose[1] + reach * math.sin(th)])

        if cmd is None and self.mode == "nav":
            if self.path is None or self.path.shape[0] < 2 or \
                    (t - self.last_plan_t) > self.PLAN_DT:
                self._replan(pose, goal, t)
            if self.path is None and self.plan_fail >= 2:
                self.mode = "explore"
                self.mode_until = t + 2.0
                self.explore_bearing = self._open_bearing(
                    ranges, angles, self._bearing_to(pose, goal))
            look = self.LOOKAHEAD if front > 1.1 else 0.55
            carrot = self._carrot(pose, goal, look)
            if dist_goal < 1.2:
                carrot = goal

        if cmd is None:
            if carrot is None:
                carrot = goal
            # ---- speed schedule: truncated series in clearance with an
            # ---- inverse (boundary-layer / wall-slip) term  [Eq. 2 shape]
            d = float(np.clip(front, 0.30, 4.5))
            v = 0.20 + 0.42 * d - 0.045 * d * d - 0.030 / d
            v_cap = float(np.clip(v, 0.14, self.max_lin))
            dxw = carrot[0] - pose[0]
            dyw = carrot[1] - pose[1]
            th_goal = _wrap(math.atan2(dyw, dxw) - pose[2])
            v_cap *= float(np.clip(1.25 - 0.45 * abs(th_goal), 0.30, 1.0))
            if dist_goal < 0.9 and self.mode == "nav":
                v_cap *= float(np.clip(0.45 + 0.60 * dist_goal, 0.40, 1.0))
            v_cap *= float(np.clip(0.80 + 0.25 * (self.map_trust / 0.60),
                                   0.80, 1.0))
            cmd = self._local(pose, carrot, ranges, angles, v_cap, bal)

        return self._emit(cmd)

    # =====================================================================
    #  small helpers
    # =====================================================================
    @staticmethod
    def _as_xy(v):
        a = np.asarray(v, dtype=float).ravel()
        if a.size == 0:
            return np.zeros(2)
        if a.size == 1:
            return np.array([a[0], 0.0])
        return np.array([a[0], a[1]])

    def _apply_T(self, est):
        tx, ty, tth = self.T
        c = math.cos(tth)
        s = math.sin(tth)
        return (c * est[0] - s * est[1] + tx,
                s * est[0] + c * est[1] + ty,
                _wrap(est[2] + tth))

    def _set_T(self, est, target):
        tth = _wrap(target[2] - est[2])
        tth = float(np.clip(tth, -0.8, 0.8))
        c = math.cos(tth)
        s = math.sin(tth)
        self.T = np.array([target[0] - (c * est[0] - s * est[1]),
                           target[1] - (s * est[0] + c * est[1]),
                           tth])

    def _cell(self, x, y):
        return (int(math.floor((x - self.ox) * self._inv)),
                int(math.floor((y - self.oy) * self._inv)))

    def _ccell(self, x, y):
        return (int(math.floor((x - self.ox) / self.cres)),
                int(math.floor((y - self.oy) / self.cres)))

    @staticmethod
    def _bearing_to(pose, pt):
        return _wrap(math.atan2(pt[1] - pose[1], pt[0] - pose[0]) - pose[2])

    def _cone_min(self, ranges, angles, center, half):
        r = np.where(np.isfinite(ranges) & (ranges > 0.10),
                     ranges, self.range_max)
        msk = np.abs(_wrap_arr(angles - center)) < half
        if not msk.any():
            return self.range_max
        return float(np.min(np.minimum(r[msk], self.range_max)))

    # =====================================================================
    #  shuttle endpoint switching
    # =====================================================================
    def _on_goal_change(self, goal, pose, t):
        if self.cur_target is not None:
            self.trips += 1
            self._commit_route()
        idx = None
        for i, p in enumerate(self.goal_pts):
            if float(np.linalg.norm(p - goal)) < 1.0:
                idx = i
                break
        if idx is None:
            self.goal_pts.append(goal.copy())
            idx = len(self.goal_pts) - 1
        self.cur_target = idx
        self.trail = [(pose[0], pose[1])]
        self.last_trail_pt = (pose[0], pose[1])
        self.path = None
        self.path_i = 0
        self.plan_fail = 0
        self.hist.clear()
        self.mode = "nav"
        self.recover_n = 0
        self.pen *= 0.5

    def _commit_route(self):
        if self.cur_target is None or len(self.trail) < 4:
            return
        g = np.zeros((self.m, self.m), dtype=np.float32)
        for (x, y) in self.trail:
            jx, jy = self._ccell(x, y)
            if 0 <= jx < self.m and 0 <= jy < self.m:
                g[max(0, jy - 1):jy + 2, max(0, jx - 1):jx + 2] = 1.0
        self.routes[self.cur_target] = g

    # =====================================================================
    #  occupancy mapping
    # =====================================================================
    def _integrate_scan(self, pose, ranges, angles, weight):
        n = self.n
        fin = np.isfinite(ranges) & (ranges > 0.12)
        if not fin.any():
            return
        r = np.where(fin, ranges, self.range_max)
        r = np.minimum(r, self.range_max)
        hit = fin & (ranges < self.range_max * 0.97)
        th = angles + pose[2]
        ct = np.cos(th)
        st = np.sin(th)

        step = self.RES * 0.75
        K = int(self.range_max / step) + 1
        s = (np.arange(K, dtype=float) + 0.5) * step
        lim = (r - 0.55 * self.RES)[:, None]
        mask = s[None, :] < lim
        if mask.any():
            xs = pose[0] + ct[:, None] * s[None, :]
            ys = pose[1] + st[:, None] * s[None, :]
            ix = np.floor((xs[mask] - self.ox) * self._inv).astype(np.int32)
            iy = np.floor((ys[mask] - self.oy) * self._inv).astype(np.int32)
            ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
            if ok.any():
                np.add.at(self.log, (iy[ok], ix[ok]),
                          np.float32(-self.FREE_GAIN * weight))
        if hit.any():
            hx = pose[0] + ct[hit] * ranges[hit]
            hy = pose[1] + st[hit] * ranges[hit]
            ix = np.floor((hx - self.ox) * self._inv).astype(np.int32)
            iy = np.floor((hy - self.oy) * self._inv).astype(np.int32)
            ok = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
            if ok.any():
                np.add.at(self.log, (iy[ok], ix[ok]),
                          np.float32(self.OCC_GAIN * weight))

        self.integ_count += 1
        if self.integ_count % 4 == 0:
            np.clip(self.log, self.LOG_MIN, self.LOG_MAX, out=self.log)

    def _mark_contact(self, pose, ranges, angles):
        fin = np.isfinite(ranges) & (ranges > 0.05)
        if not fin.any():
            return
        idx = int(np.argmin(np.where(fin, ranges, 1e9)))
        d = max(float(ranges[idx]), 0.15)
        th = angles[idx] + pose[2]
        hx = pose[0] + d * math.cos(th)
        hy = pose[1] + d * math.sin(th)
        ix, iy = self._cell(hx, hy)
        if 0 <= ix < self.n and 0 <= iy < self.n:
            y0 = max(0, iy - 1)
            x0 = max(0, ix - 1)
            self.log[y0:iy + 2, x0:ix + 2] = self.LOG_MAX

    # =====================================================================
    #  differential scan match  (arm b of the differential procedure)
    # =====================================================================
    def _scan_match(self, pose, est, ranges, angles, perr):
        rmax = min(5.0, self.range_max * 0.95)
        fin = np.isfinite(ranges) & (ranges > 0.25) & (ranges < rmax)
        nb = int(np.count_nonzero(fin))
        if nb < 24:                      # low-SNR upwarp: discard (idea 3)
            return
        idx = np.nonzero(fin)[0]
        if idx.size > 72:
            idx = idx[np.linspace(0, idx.size - 1, 72).astype(int)]
        r = ranges[idx]
        a = angles[idx]
        nused = int(r.size)

        cx0, cy0 = self._cell(pose[0], pose[1])
        W = int(6.0 / self.RES)
        x0 = max(0, cx0 - W)
        x1 = min(self.n, cx0 + W + 1)
        y0 = max(0, cy0 - W)
        y1 = min(self.n, cy0 + W + 1)
        if x1 - x0 < 10 or y1 - y0 < 10:
            return
        occ = (self.log[y0:y1, x0:x1] > self.OCC_TH).astype(np.float32)
        if float(occ.sum()) < 30.0:
            return
        d1 = self._dilf(occ)
        d2 = self._dilf(d1)
        lf = np.maximum(occ, np.maximum(0.6 * d1, 0.3 * d2))
        H, Wd = lf.shape

        w = float(np.clip(0.6 * max(perr, 0.03) + 0.06, 0.06, 0.45))
        offs = np.array([-w, -0.5 * w, 0.0, 0.5 * w, w])
        dthw = float(np.clip(0.35 * w + 0.01, 0.01, 0.06))
        aths = np.array([-1.0, -0.5, 0.0, 0.5, 1.0]) * dthw

        th = a + pose[2]
        scores = np.zeros((5, 5, 5), dtype=float)
        for i in range(5):
            hx = pose[0] + r * np.cos(th + aths[i])
            hy = pose[1] + r * np.sin(th + aths[i])
            bx = (hx - self.ox) * self._inv - x0
            by = (hy - self.oy) * self._inv - y0
            for j in range(5):
                gx = bx + offs[j] * self._inv
                ixa = np.floor(gx).astype(np.int32)
                okx = (ixa >= 0) & (ixa < Wd)
                for k in range(5):
                    gy = by + offs[k] * self._inv
                    iya = np.floor(gy).astype(np.int32)
                    ok = okx & (iya >= 0) & (iya < H)
                    if int(np.count_nonzero(ok)) < 12:
                        continue
                    scores[i, j, k] = float(lf[iya[ok], ixa[ok]].sum())

        s0 = scores[2, 2, 2]
        fi = int(np.argmax(scores))
        i, j, k = np.unravel_index(fi, scores.shape)
        sb = float(scores[i, j, k])
        resid = sb / max(1.0, float(nused))     # "excess"-style diagnostic

        self.map_trust = 0.85 * self.map_trust + 0.15 * float(
            np.clip(resid, 0.0, 1.0))

        if resid < 0.25 or sb <= s0 * 1.12 + 0.5:
            return

        # non-repeatability of past accepted corrections sets the gain
        nonrep = float(np.std(self.corr_hist)) if len(self.corr_hist) >= 3 else 0.0
        gain = 0.5 / (1.0 + 4.0 * nonrep)
        gain = float(np.clip(gain, 0.15, 0.55))
        dx = gain * float(offs[j])
        dy = gain * float(offs[k])
        dth = gain * float(aths[i])
        self.corr_hist.append(math.hypot(dx, dy))
        self._set_T(est, (pose[0] + dx, pose[1] + dy, _wrap(pose[2] + dth)))

    @staticmethod
    def _dilf(a):
        o = a.copy()
        o[1:, :] = np.maximum(o[1:, :], a[:-1, :])
        o[:-1, :] = np.maximum(o[:-1, :], a[1:, :])
        b = o.copy()
        o[:, 1:] = np.maximum(o[:, 1:], b[:, :-1])
        o[:, :-1] = np.maximum(o[:, :-1], b[:, 1:])
        return o

    @staticmethod
    def _dilb(a):
        o = a.copy()
        o[1:, :] |= a[:-1, :]
        o[:-1, :] |= a[1:, :]
        b = o.copy()
        o[:, 1:] |= b[:, :-1]
        o[:, :-1] |= b[:, 1:]
        return o

    # =====================================================================
    #  global planner
    # =====================================================================
    def _build_costmap(self):
        m = self.m
        occ = self.log > self.OCC_TH
        free = self.log < self.FREE_TH
        occ_c = occ.reshape(m, 2, m, 2).any(axis=(1, 3))
        free_c = free.reshape(m, 2, m, 2).any(axis=(1, 3)) & ~occ_c
        unk = ~occ_c & ~free_c
        hard = self._dilb(occ_c)
        soft = self._dilb(hard) & ~hard
        cost = np.ones((m, m), dtype=np.float32)
        cost += 2.2 * soft
        cost += 0.9 * unk
        cost += self.pen
        route = self.routes.get(self.cur_target)
        if route is not None:
            cost -= 0.30 * route
        np.maximum(cost, self.HW, out=cost)
        self._occ_c = occ_c
        self._hard = hard
        return hard, cost

    def _replan(self, pose, goal, t):
        dtp = t - self.last_plan_t
        if 0.0 < dtp < 1e3:
            self.pen *= math.exp(-min(dtp, 5.0) / 25.0)
        self.last_plan_t = t
        pts = self._astar(pose, goal, False)
        if pts is None:
            pts = self._astar(pose, goal, True)
        if pts is None or len(pts) < 2:
            self.path = None
            self.plan_fail += 1
        else:
            self.path = self._resample(pts, 0.25)
            self.path_i = 0
            self.plan_fail = 0

    def _astar(self, pose, goal, lenient):
        m = self.m
        hard, cost = self._build_costmap()
        hard = (self._occ_c if lenient else hard).copy()

        sx, sy = self._ccell(pose[0], pose[1])
        gx, gy = self._ccell(goal[0], goal[1])
        if not (0 <= sx < m and 0 <= sy < m):
            return None
        gx = int(min(max(gx, 0), m - 1))
        gy = int(min(max(gy, 0), m - 1))
        hard[max(0, sy - 1):sy + 2, max(0, sx - 1):sx + 2] = False

        relocated = False
        if hard[gy, gx]:
            ok = False
            for rad in range(1, 9):
                ya = max(0, gy - rad)
                yb = min(m, gy + rad + 1)
                xa = max(0, gx - rad)
                xb = min(m, gx + rad + 1)
                sub = ~hard[ya:yb, xa:xb]
                if sub.any():
                    ys, xs = np.nonzero(sub)
                    dd = (ys + ya - gy) ** 2 + (xs + xa - gx) ** 2
                    q = int(np.argmin(dd))
                    gy = int(ys[q] + ya)
                    gx = int(xs[q] + xa)
                    ok = True
                    relocated = True
                    break
            if not ok:
                return None

        cf = np.where(hard, np.inf, cost).ravel().tolist()
        N = m * m
        sidx = sy * m + sx
        gidx = gy * m + gx
        if cf[sidx] > 1e17:
            cf[sidx] = 1.0

        g = [INF] * N
        par = [-1] * N
        closed = bytearray(N)
        g[sidx] = 0.0
        heap = [(0.0, sidx)]
        pops = 0
        found = False
        best_idx = sidx
        best_h = 1e18
        hwt = self.HW
        NBl = self.NB
        maxpops = self.MAX_POPS

        while heap:
            _f, idx = heappop(heap)
            if closed[idx]:
                continue
            closed[idx] = 1
            pops += 1
            if idx == gidx:
                found = True
                break
            if pops > maxpops:
                break
            jy, jx = divmod(idx, m)
            hh = (jx - gx) * (jx - gx) + (jy - gy) * (jy - gy)
            if hh < best_h:
                best_h = hh
                best_idx = idx
            gi = g[idx]
            ci = cf[idx]
            for dx, dy, sl in NBl:
                nx = jx + dx
                ny = jy + dy
                if nx < 0 or nx >= m or ny < 0 or ny >= m:
                    continue
                nidx = ny * m + nx
                if closed[nidx]:
                    continue
                cn = cf[nidx]
                if cn > 1e17:
                    continue
                if dx and dy:
                    if cf[jy * m + nx] > 1e17 or cf[ny * m + jx] > 1e17:
                        continue
                ng = gi + sl * 0.5 * (ci + cn)
                if ng < g[nidx]:
                    g[nidx] = ng
                    par[nidx] = idx
                    heappush(heap, (ng + hwt * math.hypot(nx - gx, ny - gy),
                                    nidx))

        end = gidx if found else best_idx
        if end == sidx:
            return None
        pts = []
        cur = end
        guard = 0
        while cur != -1 and guard < N:
            jy, jx = divmod(cur, m)
            pts.append((self.ox + (jx + 0.5) * self.cres,
                        self.oy + (jy + 0.5) * self.cres))
            cur = par[cur]
            guard += 1
        pts.reverse()
        pts[0] = (pose[0], pose[1])
        if found and not relocated:
            pts.append((float(goal[0]), float(goal[1])))
        return pts

    @staticmethod
    def _resample(pts, step):
        p = np.asarray(pts, dtype=float)
        if p.shape[0] < 2:
            return p
        seg = np.linalg.norm(np.diff(p, axis=0), axis=1)
        cum = np.concatenate([[0.0], np.cumsum(seg)])
        total = float(cum[-1])
        if total < 1e-6:
            return p[:1]
        s = np.arange(0.0, total, step)
        s = np.append(s, total)
        x = np.interp(s, cum, p[:, 0])
        y = np.interp(s, cum, p[:, 1])
        return np.stack([x, y], axis=1)

    def _carrot(self, pose, goal, look):
        P = self.path
        if P is None or P.shape[0] == 0:
            return np.asarray(goal, dtype=float)
        p = np.array([pose[0], pose[1]])
        lo = int(min(self.path_i, P.shape[0] - 1))
        hi = int(min(P.shape[0], lo + 40))
        seg = P[lo:hi]
        d = np.linalg.norm(seg - p, axis=1)
        self.path_i = lo + int(np.argmin(d))
        j = self.path_i
        last = P.shape[0] - 1
        while j < last and float(np.linalg.norm(P[j] - p)) < look:
            j += 1
        return P[j]

    # =====================================================================
    #  differential sector steering (common mode deleted)
    # =====================================================================
    def _diff_steer(self, ranges, angles):
        r = np.where(np.isfinite(ranges) & (ranges > 0.10),
                     ranges, self.range_max)
        r = np.minimum(r, self.range_max)
        aa = _wrap_arr(angles)
        msk = np.abs(aa) < math.radians(100.0)
        front = self._cone_min(ranges, angles, 0.0, math.radians(32.0))
        if not msk.any():
            return 0.0, front
        av = aa[msk]
        rv = np.clip(r[msk], 0.25, self.range_max)
        w = (np.cos(av) ** 2) / (rv * rv)
        left = float(w[av > 0.05].sum())
        right = float(w[av < -0.05].sum())
        tot = left + right + 1e-6
        # >0 : more clutter on the right -> asymmetry pushes us left
        bal = (right - left) / tot
        return float(np.clip(bal, -1.0, 1.0)), front

    def _open_bearing(self, ranges, angles, th_pref):
        r = np.where(np.isfinite(ranges) & (ranges > 0.10),
                     ranges, self.range_max)
        r = np.minimum(r, self.range_max)
        if r.size < 5:
            return float(th_pref)
        k = max(1, int(round(0.35 / (TWO_PI / r.size))))
        rs = r.copy()
        for sh in range(1, k + 1):
            rs = np.minimum(rs, np.roll(r, sh))
            rs = np.minimum(rs, np.roll(r, -sh))
        score = rs + 1.4 * np.cos(_wrap_arr(angles - th_pref))
        return float(_wrap(angles[int(np.argmax(score))]))

    # =====================================================================
    #  recovery
    # =====================================================================
    def _enter_recovery(self, t, pose, ranges, angles, hard):
        self.recover_n += 1
        self._add_penalty(pose, 1.0 + 0.8 * self.recover_n, 0.55)
        ob = self._open_bearing(ranges, angles, 0.0)
        self.recover_dir = 1.0 if ob >= 0.0 else -1.0
        if self.recover_n % 3 == 0:
            self.recover_dir *= -1.0
        if self.recover_n % 4 == 0:
            self.recover_dir = 1.0 if self.rng.random() < 0.5 else -1.0
        rear = self._cone_min(ranges, angles, math.pi, math.radians(55.0))
        if rear > 0.55:
            self.mode = "back"
            self.mode_until = t + (0.55 if hard else 0.45)
        else:
            self.mode = "turn"
            self.mode_until = t + 0.8
        self.path = None
        self.hist.clear()

    def _add_penalty(self, pose, amount, rad):
        jx, jy = self._ccell(pose[0], pose[1])
        if not (0 <= jx < self.m and 0 <= jy < self.m):
            return
        k = max(1, int(rad / self.cres))
        y0 = max(0, jy - k)
        x0 = max(0, jx - k)
        self.pen[y0:jy + k + 1, x0:jx + k + 1] += np.float32(amount)
        np.clip(self.pen, 0.0, 6.0, out=self.pen)

    # =====================================================================
    #  local velocity selection
    # =====================================================================
    def _local(self, pose, carrot, ranges, angles, v_cap, bal):
        fin = np.isfinite(ranges) & (ranges > 0.12) & (ranges < 3.2)
        if fin.any():
            rr = ranges[fin]
            aa = angles[fin]
            obx = rr * np.cos(aa)
            oby = rr * np.sin(aa)
            d0min = float(np.min(rr))
        else:
            obx = oby = None
            d0min = 9.0

        rad = self.R_HARD
        if d0min < rad + 0.05:
            rad = max(0.15, d0min - 0.03)

        dxw = carrot[0] - pose[0]
        dyw = carrot[1] - pose[1]
        cy_ = math.cos(pose[2])
        sy_ = math.sin(pose[2])
        cx = cy_ * dxw + sy_ * dyw
        cyy = -sy_ * dxw + cy_ * dyw
        th_goal = math.atan2(cyy, cx)
        d0 = math.hypot(cx, cyy)

        sp = v_cap * self.cand_fr
        th = self.cand_th
        vx = sp * np.cos(th)
        vy = np.clip(sp * np.sin(th), -0.45, 0.45)
        wt = 0.62 * th + 0.38 * th_goal
        wz = np.clip(1.7 * wt, -self.max_ang, self.max_ang)

        ex = self.cand_extra
        vx = np.concatenate([vx, ex[:, 0]])
        vy = np.concatenate([vy, ex[:, 1]])
        wz = np.concatenate([wz, ex[:, 2] * self.max_ang])
        thc = np.concatenate([th, np.zeros(ex.shape[0])])
        thpen = np.concatenate([np.abs(th), np.full(ex.shape[0], 1.4)])

        N = vx.size
        dts = self.HORIZON / self.NSAMP
        X = np.zeros(N)
        Y = np.zeros(N)
        PH = np.zeros(N)
        minclear = np.full(N, 9.0)
        bad = np.zeros(N, dtype=bool)
        tti = np.full(N, self.HORIZON)

        for i in range(self.NSAMP):
            phm = PH + 0.5 * wz * dts
            X = X + (vx * np.cos(phm) - vy * np.sin(phm)) * dts
            Y = Y + (vx * np.sin(phm) + vy * np.cos(phm)) * dts
            PH = PH + wz * dts
            if obx is not None:
                d = np.sqrt((X[:, None] - obx[None, :]) ** 2 +
                            (Y[:, None] - oby[None, :]) ** 2).min(axis=1)
                minclear = np.minimum(minclear, d)
                nh = (d < rad) & (~bad)
                if nh.any():
                    tti[nh] = (i + 1) * dts
                bad |= (d < rad)

        dend = np.hypot(cx - X, cyy - Y)
        progress = d0 - dend
        head_err = np.abs(_wrap_arr(np.arctan2(cyy - Y, cx - X) - PH))

        prev_w = float(self.prev_cmd[2])
        rev = (np.sign(wz) * (1.0 if prev_w >= 0 else -1.0) < 0) & \
              (abs(prev_w) > 0.15)

        score = (3.0 * progress
                 + 0.50 * np.minimum(minclear, 1.0)
                 - 0.45 * head_err
                 - 0.30 * np.abs(vy)
                 - 0.12 * thpen
                 - 0.20 * np.abs(wz - prev_w)
                 - 0.30 * rev * np.minimum(np.abs(wz), 1.0)
                 + 0.25 * bal * np.tanh(2.0 * thc))
        score = np.where(bad, score - 8.0 + 2.5 * tti, score)

        b = int(np.argmax(score))
        return (float(vx[b]), float(vy[b]), float(wz[b]))

    # =====================================================================
    #  command conditioning: stay out of the body's own ringing band
    # =====================================================================
    def _emit(self, cmd):
        vx, vy, wz = cmd
        if not math.isfinite(vx):
            vx = 0.0
        if not math.isfinite(vy):
            vy = 0.0
        if not math.isfinite(wz):
            wz = 0.0
        vx = float(np.clip(vx, -0.5 * self.max_lin, self.max_lin))
        vy = float(np.clip(vy, -0.5, 0.5))
        vy = float(np.clip(vy, -self.max_lin, self.max_lin))
        wz = float(np.clip(wz, -self.max_ang, self.max_ang))

        p = self.prev_cmd
        vx = p[0] + float(np.clip(vx - p[0], -0.10, 0.10))
        vy = p[1] + float(np.clip(vy - p[1], -0.08, 0.08))
        wz = p[2] + float(np.clip(wz - p[2], -0.16, 0.16))
        if abs(wz) < 0.03:
            wz = 0.0
        if abs(vy) < 0.02:
            vy = 0.0

        self.prev_cmd = np.array([vx, vy, wz], dtype=float)
        return Command(vx, vy, wz)
