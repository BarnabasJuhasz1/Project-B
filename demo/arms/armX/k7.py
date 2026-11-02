# ============================================================================
#  Go2 shuttle navigator: incremental occupancy map + two persistent value
#  fields (one per shuttle endpoint) + a drift-free reactive layer.
#
#  What was taken from the source paper (Feng, Zhang, Lin, Gillis & Moldover,
#  "Determination of the Boltzmann Constant Using the Differential-Cylindrical
#  Procedure") and where it came from:
#
#  * The DELETION MECHANISM of the differential procedure (Sec. 3, Eqs. 6-11):
#    a perturbation that acts identically on two channels cancels in their
#    difference, with no model of the perturbation required ("even if we do not
#    recognize some imperfections, we still know their perturbations will be
#    deleted").  Used three times here:
#      (a) each candidate bearing is measured at TWO look-ahead scales in the
#          paper's L1:L2 = 2:1 ratio (L and 2L, Eq. 9/10); their difference is
#          a corridor-DEPTH estimate that is immune to a common-mode range
#          bias and that separates "wall at L" from "open doorway";
#      (b) left-minus-right clearance for corridor centring - again a pure
#          difference, so a common range offset deletes;
#      (c) candidate clearances are scored through a RATIO to the scan median,
#          after the mode-to-mode comparison M1/M2 = c2^2/c1^2 (Eq. 3), which
#          is independent of any common calibration factor.
#    The same reasoning says *which* channel to trust: the live scan is a
#    differential (robot-relative) measurement in which pose drift is common
#    mode and deletes, while the world-frame map/goal carries the full drift -
#    so safety is enforced from the scan alone and never from the map.
#  * "Maintaining l1 and l2 both EVEN yields no free recoil disturbance"
#    (Sec. 5.1): even modes are longitudinally symmetric, so asymmetric
#    perturbations cannot couple.  Candidate clearances are averaged with an
#    even-symmetric angular kernel, which deletes the antisymmetric part of
#    the beam noise about each candidate bearing.
#  * "Accurate resonant measurements have to be kept distant away from the
#    natural vibrations of cavities" (Sec. 5.1, Fig. 3): commands are
#    slew-limited, yaw-rate damped and the chosen heading is hysteretic, which
#    keeps the steering loop's excitation away from the gait's resonance and
#    kills steering limit cycles.
#  * EXCESS HALF-WIDTH as the validity test of the model (Eq. 12: the measured
#    half-width minus the theoretical one "measures the extent in which the
#    theoretical prediction differs from the measurement"): the commanded twist
#    is the theory, the pose-estimate delta is the measurement, and their
#    normalised excess is monitored.  Non-zero excess means the motion model is
#    not describing reality (slip, blocked, bad localisation) -> cut speed and
#    arm recovery.
#  * Half-width fit g = b0/sqrt(p) + b1 + b2*sqrt(p) with low-pressure
#    upwarping from poor signal-to-noise (Eq. 13, Figs. 4-5): the speed cap
#    uses the same fractional-power shape in clearance, and the low-"pressure"
#    (tight / sparse-return) regime is treated as low SNR, where margins widen.
#  * Samples analysed in the order "C-B-A-C-A-B ... for the purpose minimizing
#    bias" (Sec. 2.1) and non-repeatability falling with more runs (Table 2):
#    recovery turns alternate side on that palindromic pattern so a systematic
#    turning bias cancels across trips, and the speed trust grows as trips
#    repeat over an increasingly stable map.
# ============================================================================

import math

import numpy as np

from core.observation import Command

_BIG = 1.0e6
_N8 = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))


def _wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(a, dtype=float) + math.pi) % (2.0 * math.pi) - math.pi


def _wrapf(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Controller:
    # ---- geometry / map ----
    RES = 0.25
    MAX_CELLS = 40000
    MARGIN = 13.0
    OCC_T = 0.85
    HIT = 0.55
    MISS = 0.25
    LO_MIN = -2.0
    LO_MAX = 3.0

    # ---- reactive layer ----
    NCAND = 72
    W_HALF = 0.33          # half width of the corridor the body needs
    R_MIN_VALID = 0.12     # below this a return is treated as self-hit
    D_STOP = 0.42
    D_FREE = 1.90
    D_REJECT = 0.46
    L1 = 1.20              # first look-ahead scale; second is 2*L1 (2:1)

    # ---- command shaping ----
    DV = 0.045             # per-step linear slew  (~2.2 m/s^2)
    DW = 0.130             # per-step angular slew (~6.5 rad/s^2)
    K_YAW = 2.20
    K_DAMP = 0.15

    # ---- recovery ----
    SIDE_PATTERN = (1.0, -1.0, -1.0, 1.0, -1.0, 1.0)   # C-B-A-C-A-B
    T_BACK = 0.65
    T_WALL = 3.20

    # =====================================================================
    #  reset
    # =====================================================================
    def reset(self, scenario, rng):
        self.rng = rng

        def sget(name, default):
            try:
                v = getattr(scenario, name)
            except Exception:
                return default
            return default if v is None else v

        self.range_max = max(0.5, float(sget("range_max", 6.0)))
        self.max_lin = max(0.05, float(sget("max_lin", 0.8)))
        self.max_ang = max(0.05, float(sget("max_ang", 1.0)))
        self.time_limit = float(sget("time_limit", 300.0))

        a = np.asarray(sget("start", (0.0, 0.0)), dtype=float).ravel()
        b = np.asarray(sget("goal", (0.0, 0.0)), dtype=float).ravel()
        p0 = a[:2].astype(float) if a.size >= 2 else np.zeros(2)
        p1 = b[:2].astype(float) if b.size >= 2 else np.zeros(2)

        # ---- grid covering both endpoints with a generous unknown margin ----
        lo = np.minimum(p0, p1) - self.MARGIN
        hi = np.maximum(p0, p1) + self.MARGIN
        span = np.maximum(hi - lo, 6.0)
        res = float(self.RES)
        while (span[0] / res + 4.0) * (span[1] / res + 4.0) > self.MAX_CELLS:
            res *= 1.15
        self.res = float(res)
        # hard inflation of one cell only while a cell is small enough that it
        # cannot seal an indoor doorway; otherwise rely on the scan layer
        self.inflate = 1 if self.res <= 0.30 else 0
        self.origin = np.array([lo[0] - res, lo[1] - res], dtype=float)
        self.nx = int(span[0] / res) + 3
        self.ny = int(span[1] / res) + 3

        shape = (self.ny, self.nx)
        self.logodds = np.zeros(shape, dtype=np.float32)
        self.lo_flat = self.logodds.reshape(-1)
        self.penalty = np.zeros(shape, dtype=np.float32)
        self.cost = np.ones(shape, dtype=np.float32)
        self.passable = np.ones(shape, dtype=bool)

        # one persistent value field per shuttle endpoint, reused every trip
        self.fields = {}
        self.V = None
        self.goal_cell = (0, 0)
        self.active_key = None

        # ---- candidate bearings: index adjacency == angular adjacency ----
        j = np.arange(self.NCAND, dtype=float)
        self.cand = _wrap(j * (2.0 * math.pi / self.NCAND))
        self.cand_cos = np.cos(self.cand)
        self.cand_sin = np.sin(self.cand)
        self.kern = np.array([1.0, 2.0, 3.0, 2.0, 1.0])   # even-symmetric
        self.j_left = self._window_idx(0.5 * math.pi, 0.35)
        self.j_right = self._window_idx(-0.5 * math.pi, 0.35)
        self.j_back = self._window_idx(math.pi, 0.45)
        self.j_front = self._window_idx(0.0, 0.30)

        # ---- beam caches (built when the first scan arrives) ----
        self.dcos = None
        self.dsin = None
        self.b_cos = None
        self.b_sin = None
        self.n_beams = -1

        # ---- episode state ----
        self.step_i = 0
        self.t_prev = None
        self.xy_prev = None
        self.yaw_prev = None
        self.cmd = np.zeros(3, dtype=float)
        self.theta_prev = 0.0
        self.excess = 0.0          # paper's "excess": model vs measurement
        self.obs_speed = 0.0
        self.yaw_rate = 0.0
        self.cmd_sp = 0.0
        self.trips = 0
        self.trust = 0.85
        self.mode = "nav"
        self.mode_t = 0.0
        self.side = 1.0
        self.side_i = 0
        self.goal_xy = None
        self.carrot = None
        self.carrot_t = -1.0e9
        self.carrot_xy = None
        self.trace = []
        self.vref = None
        self.vref_t = 0.0
        self.contact_n = 0
        self.cost_t = -1.0e9
        self.free = None
        self.jump = False

        try:
            if rng is not None:
                self.side_i = int(rng.integers(0, len(self.SIDE_PATTERN)))
        except Exception:
            self.side_i = 0

    # =====================================================================
    #  step
    # =====================================================================
    def step(self, obs):
        vx = vy = wz = 0.0
        try:
            vx, vy, wz = self._step(obs)
        except Exception:
            vx, vy, wz = 0.0, 0.0, 0.0
        try:
            if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
                vx = vy = wz = 0.0
            vx = float(min(max(vx, -self.max_lin), self.max_lin))
            vy = float(min(max(vy, -self.max_lin), self.max_lin))
            wz = float(min(max(wz, -self.max_ang), self.max_ang))
            self.cmd[0], self.cmd[1], self.cmd[2] = vx, vy, wz
        except Exception:
            vx = vy = wz = 0.0
        return Command(vx, vy, wz)

    # ---------------------------------------------------------------------
    def _step(self, obs):
        self.step_i += 1

        # ---------------- 1. parse ----------------
        t = float(getattr(obs, "t", 0.0))
        pose = np.asarray(getattr(obs, "pose_estimate", (0.0, 0.0, 0.0)),
                          dtype=float).ravel()
        x = float(pose[0]) if pose.size > 0 else 0.0
        y = float(pose[1]) if pose.size > 1 else 0.0
        yaw = float(pose[2]) if pose.size > 2 else 0.0
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
            return self._slew(0.0, 0.0, 0.0)

        try:
            perr = float(getattr(obs, "pose_error_estimate", 0.0) or 0.0)
        except Exception:
            perr = 0.0
        if not math.isfinite(perr):
            perr = 0.0
        perr = min(max(perr, 0.0), 5.0)

        contact = bool(getattr(obs, "contact", False))

        g = np.asarray(getattr(obs, "goal", (x, y)), dtype=float).ravel()
        gx = float(g[0]) if g.size > 0 else x
        gy = float(g[1]) if g.size > 1 else y
        if not (math.isfinite(gx) and math.isfinite(gy)):
            gx, gy = x, y

        rr = np.asarray(getattr(obs, "ranges", ()), dtype=float).ravel()
        aa = np.asarray(getattr(obs, "range_angles", ()), dtype=float).ravel()
        if rr.size == 0 or aa.size != rr.size:
            return self._slew(0.0, 0.0, 0.6 * self.side)

        r = np.where(np.isfinite(rr), rr, self.range_max)
        r = np.clip(r, 0.0, self.range_max).astype(np.float32)

        dt = 0.02 if self.t_prev is None else min(max(t - self.t_prev, 0.004), 0.2)

        # ---------------- 2. beam caches ----------------
        self._ensure_beam_cache(aa)

        # ---------------- 3. odometry / "excess" monitor ----------------
        self.jump = False
        if self.xy_prev is not None:
            dx = x - self.xy_prev[0]
            dy = y - self.xy_prev[1]
            disp = math.hypot(dx, dy)
            self.obs_speed = disp / dt
            self.yaw_rate = _wrapf(yaw - self.yaw_prev) / dt
            pred = math.hypot(self.cmd[0], self.cmd[1])
            # localisation correction jump: model comparison is meaningless
            if disp > 0.25 + 3.0 * pred * dt:
                self.jump = True
            elif pred > 0.06:
                # Eq.(12) analogue: fractional excess of model over measurement
                ratio = (pred - self.obs_speed) / pred
                ratio = min(max(ratio, -1.0), 1.0)
                self.excess = 0.96 * self.excess + 0.04 * ratio
        self.xy_prev = (x, y)
        self.yaw_prev = yaw
        self.t_prev = t

        slip = min(max((self.excess - 0.45) / 0.45, 0.0), 1.0)

        # ---------------- 4. goal bookkeeping ----------------
        if self.goal_xy is None:
            self.goal_xy = (gx, gy)
        elif math.hypot(gx - self.goal_xy[0], gy - self.goal_xy[1]) > 0.35:
            self.trips += 1
            self.goal_xy = (gx, gy)
            self.mode = "nav"
            self.mode_t = t
            self.vref = None
            self.carrot_xy = None
            self.trace = []
            # non-repeatability falls with repeated runs (Table 2) -> more speed
            self.trust = min(1.0, 0.85 + 0.05 * self.trips)

        key = (round(gx, 1), round(gy, 1))
        if key not in self.fields:
            if len(self.fields) >= 4:
                self.fields.clear()
            self.fields[key] = self._seed_field(gx, gy)
        self.active_key = key
        self.goal_cell = self._cell(gx, gy)

        # ---------------- 5. map update ----------------
        cw = math.cos(yaw)
        sw = math.sin(yaw)
        dirx = (cw * self.b_cos - sw * self.b_sin)
        diry = (sw * self.b_cos + cw * self.b_sin)

        if not self.jump:
            w = 1.0 / (1.0 + 0.8 * perr)
            hit = (r < self.range_max - 0.08) & (r > self.R_MIN_VALID)
            if np.any(hit):
                px = x + r[hit] * dirx[hit]
                py = y + r[hit] * diry[hit]
                idx = self._flat_idx(px, py)
                if idx.size:
                    np.add.at(self.lo_flat, idx, np.float32(self.HIT * w))
            if self.step_i % 3 == 0:
                nst = int(self.range_max / (0.6 * self.res)) + 2
                s = (np.arange(1, nst, dtype=np.float32) * (0.6 * self.res))[None, :]
                lim = (r - 0.5 * self.res)[:, None]
                m = s < lim
                if np.any(m):
                    fx = x + s * dirx[:, None]
                    fy = y + s * diry[:, None]
                    idx = self._flat_idx(fx[m], fy[m])
                    if idx.size:
                        np.add.at(self.lo_flat, idx, np.float32(-self.MISS * w))
            np.clip(self.logodds, self.LO_MIN, self.LO_MAX, out=self.logodds)

        # the cell we stand in is free by construction
        if self.step_i % 5 == 0:
            iy0, ix0 = self._cell_yx(x, y)
            sl = (slice(max(0, iy0 - 1), min(self.ny, iy0 + 2)),
                  slice(max(0, ix0 - 1), min(self.nx, ix0 + 2)))
            np.minimum(self.logodds[sl], np.float32(-0.6), out=self.logodds[sl])

        # ---------------- 6. cost / passability refresh ----------------
        if t - self.cost_t > 0.2:
            self._refresh_cost(x, y, gx, gy, perr)
            self.cost_t = t

        # ---------------- 7. value fields ----------------
        V = self._relax(self.fields[key], 7, self.goal_cell)
        self.fields[key] = V
        self.V = V
        for k2 in self.fields:
            if k2 is not key and k2 != key:
                self.fields[k2] = self._relax(self.fields[k2], 2,
                                              self._cell(k2[0], k2[1]))

        # ---------------- 8. clearance, two scales (L : 2L = 1 : 2) ----------
        use = r > self.R_MIN_VALID
        rv = np.where(use, r, np.float32(self.range_max))
        along = rv[None, :] * self.dcos
        lat = rv[None, :] * self.dsin
        ok = (np.abs(lat) < np.float32(self.W_HALF)) & (along > np.float32(0.05)) \
            & use[None, :]
        free = np.where(ok, along, np.float32(self.range_max)).min(axis=1)
        free = np.minimum(free, np.float32(self.range_max)).astype(float)
        self.free = free

        # even-symmetric angular average deletes the antisymmetric noise part
        c1 = self._smooth(np.minimum(free, self.L1))
        c2 = self._smooth(np.minimum(free, 2.0 * self.L1))
        depth = np.clip((c2 - c1) / self.L1, 0.0, 1.0)      # differential depth
        med = max(float(np.median(c1)), 0.25)
        rel = np.clip(c1 / med, 0.0, 2.0)                   # ratio comparison

        front = float(np.min(free[self.j_front]))
        back = float(np.min(free[self.j_back]))
        d_left = float(np.min(free[self.j_left]))
        d_right = float(np.min(free[self.j_right]))

        # ---------------- 9. desired global heading ----------------
        dist_goal = math.hypot(gx - x, gy - y)
        th_goal = _wrapf(math.atan2(gy - y, gx - x) - yaw)

        if (self.carrot_xy is None or t - self.carrot_t > 0.06
                or math.hypot(x - self.carrot_xy[2], y - self.carrot_xy[3]) > 0.35):
            cxy = self._carrot(x, y)
            self.carrot_t = t
            self.carrot_xy = None if cxy is None else (cxy[0], cxy[1], x, y)

        if self.carrot_xy is not None:
            th_des = _wrapf(math.atan2(self.carrot_xy[1] - y,
                                       self.carrot_xy[0] - x) - yaw)
        else:
            th_des = th_goal

        # final approach: if the goal itself is in sight and clear, aim at it
        if dist_goal < 2.6:
            jg = int(np.argmin(np.abs(_wrap(self.cand - th_goal))))
            if free[jg] > min(dist_goal, 2.4) - 0.25:
                th_des = th_goal

        # ---------------- 10. progress monitors -------------------------
        if self.step_i % 5 == 0:
            self.trace.append((t, x, y))
            while len(self.trace) > 2 and t - self.trace[0][0] > 4.5:
                self.trace.pop(0)

        stuck = False
        if self.mode == "nav" and len(self.trace) >= 2 and \
                (t - self.trace[0][0]) > 2.6:
            d = math.hypot(x - self.trace[0][1], y - self.trace[0][2])
            if d < 0.33 and self.cmd_sp > 0.18:
                stuck = True
        if slip > 0.65 and self.cmd_sp > 0.25:
            stuck = True

        iy, ix = self._cell_yx(x, y)
        vr = float(self.V[iy, ix])
        if vr >= 0.5 * _BIG:
            vr = self._local_min_value(iy, ix)
        trapped = False
        if math.isfinite(vr) and vr < 0.5 * _BIG:
            if self.vref is None or vr < self.vref - 0.5:
                self.vref = vr
                self.vref_t = t
            elif t - self.vref_t > 6.0:
                trapped = True
        else:
            self.vref = None
            self.vref_t = t

        if contact:
            self.contact_n += 1
        else:
            self.contact_n = 0
        hard_contact = (self.contact_n >= 15 and self.obs_speed < 0.20)
        if hard_contact:
            # teach the map what we just walked into
            hd = yaw
            if math.hypot(self.cmd[0], self.cmd[1]) > 0.05:
                hd = yaw + math.atan2(self.cmd[1], self.cmd[0])
            for dd in (0.32, 0.46):
                self._stamp(self.logodds, x + dd * math.cos(hd),
                            y + dd * math.sin(hd), 0.12, 1.1, self.LO_MAX)

        # ---------------- 11. recovery state machine --------------------
        if self.mode == "nav":
            if hard_contact or stuck or trapped:
                self.side = self.SIDE_PATTERN[self.side_i % len(self.SIDE_PATTERN)]
                self.side_i += 1
                self.mode = "back" if back > 0.55 else "wall"
                self.mode_t = t
                self._stamp(self.penalty, x, y, 0.45, 1.0, 3.0)
                self.fields[key] = self._seed_field(gx, gy)
                self.V = self.fields[key]
                self.vref = None
                self.vref_t = t
                self.trace = []
        elif self.mode == "back":
            if t - self.mode_t > self.T_BACK or back < 0.35:
                self.mode = "wall"
                self.mode_t = t
        else:  # wall
            if t - self.mode_t > self.T_WALL:
                self.mode = "nav"
                self.mode_t = t
                self.vref = None
                self.vref_t = t
                self.trace = []

        if self.mode == "back":
            self.cmd_sp = 0.97 * self.cmd_sp + 0.03 * 0.30
            self.theta_prev = 0.0
            return self._slew(-0.30 * self.trust, 0.0, 0.75 * self.side)

        # ---------------- 12. candidate arbitration ---------------------
        reject = self.D_REJECT + 0.06 * min(perr, 1.0)
        blocked = free < reject

        if self.mode == "wall":
            w_align, w_side, w_depth = 0.55, 1.15, 1.35
        else:
            w_align, w_side, w_depth = 2.40, 0.0, 1.15

        score = (w_align * np.cos(self.cand - th_des)
                 + 0.90 * rel
                 + w_depth * depth
                 + 0.40 * self.cand_cos
                 + 0.55 * np.cos(self.cand - self.theta_prev))
        if w_side != 0.0:
            score = score + w_side * np.cos(self.cand - self.side * 1.05)
        score = score - 60.0 * blocked

        if bool(np.all(blocked)):
            # nowhere to put a foot: pivot in place toward the preferred side
            self.theta_prev = self.side * 1.2
            self.cmd_sp = 0.97 * self.cmd_sp
            return self._slew(0.0, 0.0, 0.9 * self.side)

        j = int(np.argmax(score))
        theta = float(self.cand[j])
        self.theta_prev = theta

        # ---------------- 13. speed, Eq.(13)-shaped in clearance --------
        wj = 3
        widx = [(j + k) % self.NCAND for k in range(-wj, wj + 1)]
        room = float(np.min(free[widx]))
        if abs(theta) < 1.0:
            room = min(room, front)

        s = min(max((room - self.D_STOP) / (self.D_FREE - self.D_STOP), 0.0), 1.0)
        base = self.max_lin * (s ** 0.6) * self.trust
        base *= (1.0 - 0.35 * slip)
        base *= (1.0 - 0.18 * min(perr, 1.5) / 1.5)
        # low-"pressure" / low-SNR regime: widen the margin, go slower
        if room < 1.0:
            base *= 0.82
        if self.mode == "wall":
            base *= 0.70

        ca = math.cos(theta)
        if abs(theta) > 1.15:
            vx = 0.0
            vy = 0.0
        else:
            vx = base * max(0.0, ca)
            vy = base * 0.45 * math.sin(theta)

        # differential centring: (left - right) deletes a common range offset
        if d_left < 1.15 and d_right < 1.15:
            vy += min(max(0.55 * (d_left - d_right), -0.22), 0.22)
        vy = min(max(vy, -0.30), 0.30)

        wz = self.K_YAW * theta - self.K_DAMP * min(max(self.yaw_rate, -3.0), 3.0)

        self.cmd_sp = 0.97 * self.cmd_sp + 0.03 * math.hypot(vx, vy)
        return self._slew(vx, vy, wz)

    # =====================================================================
    #  helpers
    # =====================================================================
    def _slew(self, vx, vy, wz):
        """Rate-limit the command: keep the excitation away from the gait's
        own resonance (paper Sec. 5.1, Fig. 3)."""
        px, py, pw = self.cmd[0], self.cmd[1], self.cmd[2]
        vx = px + min(max(vx - px, -self.DV), self.DV)
        vy = py + min(max(vy - py, -self.DV), self.DV)
        wz = pw + min(max(wz - pw, -self.DW), self.DW)
        return (vx, vy, wz)

    def _ensure_beam_cache(self, aa):
        if (self.dcos is not None and aa.size == self.n_beams
                and abs(float(aa[0]) - self.a0) < 1e-9
                and abs(float(aa[-1]) - self.a1) < 1e-9):
            return
        ang = np.where(np.isfinite(aa), aa, 0.0).astype(float)
        d = ang[None, :] - self.cand[:, None]
        self.dcos = np.cos(d).astype(np.float32)
        self.dsin = np.sin(d).astype(np.float32)
        self.b_cos = np.cos(ang).astype(np.float32)
        self.b_sin = np.sin(ang).astype(np.float32)
        self.n_beams = int(aa.size)
        self.a0 = float(ang[0])
        self.a1 = float(ang[-1])

    def _window_idx(self, c, hw):
        d = np.abs(_wrap(self.cand - c))
        idx = np.nonzero(d <= hw)[0]
        if idx.size == 0:
            idx = np.array([int(np.argmin(d))])
        return idx

    def _smooth(self, v):
        out = np.zeros_like(v)
        for i in range(5):
            out += self.kern[i] * np.roll(v, i - 2)
        return out / 9.0

    # ---- grid indexing ----
    def _cell_yx(self, x, y):
        ix = int((x - self.origin[0]) / self.res)
        iy = int((y - self.origin[1]) / self.res)
        if ix < 0:
            ix = 0
        elif ix >= self.nx:
            ix = self.nx - 1
        if iy < 0:
            iy = 0
        elif iy >= self.ny:
            iy = self.ny - 1
        return (iy, ix)

    def _cell(self, x, y):
        return self._cell_yx(x, y)

    def _flat_idx(self, xs, ys):
        ix = ((np.asarray(xs) - self.origin[0]) / self.res).astype(np.int32)
        iy = ((np.asarray(ys) - self.origin[1]) / self.res).astype(np.int32)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        if not np.any(ok):
            return np.empty(0, dtype=np.int64)
        return (iy[ok].astype(np.int64) * self.nx + ix[ok].astype(np.int64))

    def _stamp(self, grid, x, y, radius, amount, cap):
        iy, ix = self._cell_yx(x, y)
        rad = max(0, int(round(radius / self.res)))
        y0, y1 = max(0, iy - rad), min(self.ny, iy + rad + 1)
        x0, x1 = max(0, ix - rad), min(self.nx, ix + rad + 1)
        sub = grid[y0:y1, x0:x1]
        np.minimum(sub + np.float32(amount), np.float32(cap), out=sub)

    @staticmethod
    def _dilate(m, r):
        out = m
        for _ in range(max(0, r)):
            p = np.pad(out, 1, mode="constant", constant_values=False)
            out = (p[1:-1, 1:-1] | p[:-2, 1:-1] | p[2:, 1:-1]
                   | p[1:-1, :-2] | p[1:-1, 2:])
        return out

    def _refresh_cost(self, x, y, gx, gy, perr):
        occ = self.logodds > np.float32(self.OCC_T)
        unknown = np.abs(self.logodds) < np.float32(0.25)
        blocked = self._dilate(occ, self.inflate)
        near = self._dilate(occ, 3)

        self.penalty *= np.float32(0.999)
        cost = (np.float32(1.0) + np.float32(1.35) * near
                + np.float32(0.30) * unknown
                + np.float32(3.5) * self.penalty)
        if perr > 0.6:
            cost = cost + np.float32(0.70) * self._dilate(occ, 5)
        np.clip(cost, 1.0, 9.0, out=cost)
        self.cost = cost.astype(np.float32)

        pas = ~blocked
        for (cy, cx) in (self._cell_yx(x, y), self._cell_yx(gx, gy)):
            pas[max(0, cy - 1):min(self.ny, cy + 2),
                max(0, cx - 1):min(self.nx, cx + 2)] = True
        self.passable = pas

        if perr > 0.5 and self.step_i % 250 == 0:
            self.logodds *= np.float32(0.97)

    # ---- value field ----
    def _seed_field(self, gx, gy):
        iy, ix = self._cell_yx(gx, gy)
        yy = (np.arange(self.ny, dtype=np.float32) - np.float32(iy)) * self.res
        xx = (np.arange(self.nx, dtype=np.float32) - np.float32(ix)) * self.res
        return np.sqrt(yy[:, None] ** 2 + xx[None, :] ** 2).astype(np.float32)

    def _relax(self, V, iters, goal_cell):
        """Warm-started Jacobi min-plus value iteration toward one endpoint.
        The field persists across trips, so the work done on the first pass of
        the building is reused on every later pass in both directions."""
        gy, gx = goal_cell
        c = self.cost * np.float32(self.res)
        cd = c * np.float32(1.41421356)
        pas = self.passable
        if V.shape != (self.ny, self.nx):
            V = self._seed_field(
                self.origin[0] + (gx + 0.5) * self.res,
                self.origin[1] + (gy + 0.5) * self.res)
        P = np.empty((self.ny + 2, self.nx + 2), dtype=np.float32)
        for _ in range(iters):
            P.fill(np.float32(_BIG))
            P[1:-1, 1:-1] = V
            m = np.minimum(np.minimum(P[:-2, 1:-1], P[2:, 1:-1]),
                           np.minimum(P[1:-1, :-2], P[1:-1, 2:])) + c
            m2 = np.minimum(np.minimum(P[:-2, :-2], P[:-2, 2:]),
                            np.minimum(P[2:, :-2], P[2:, 2:])) + cd
            V = np.minimum(m, m2)
            np.clip(V, 0.0, _BIG, out=V)
            V[~pas] = np.float32(_BIG)
            V[gy, gx] = np.float32(0.0)
        return V

    def _local_min_value(self, iy, ix):
        rad = max(1, int(round(1.0 / self.res)))
        y0, y1 = max(0, iy - rad), min(self.ny, iy + rad + 1)
        x0, x1 = max(0, ix - rad), min(self.nx, ix + rad + 1)
        sub = np.where(self.passable[y0:y1, x0:x1],
                       self.V[y0:y1, x0:x1], np.float32(_BIG))
        return float(sub.min())

    def _carrot(self, x, y):
        """Greedy descent of the value field to a look-ahead point ~2 m out."""
        V = self.V
        if V is None:
            return None
        iy, ix = self._cell_yx(x, y)
        rad = max(1, int(round(1.0 / self.res)))
        y0, y1 = max(0, iy - rad), min(self.ny, iy + rad + 1)
        x0, x1 = max(0, ix - rad), min(self.nx, ix + rad + 1)
        sub = np.where(self.passable[y0:y1, x0:x1],
                       V[y0:y1, x0:x1], np.float32(_BIG))
        k = int(np.argmin(sub))
        w = sub.shape[1]
        cy = y0 + k // w
        cx = x0 + k % w
        if float(V[cy, cx]) >= 0.5 * _BIG:
            return None

        cur = float(V[cy, cx])
        travelled = 0.0
        moved = False
        for _ in range(40):
            best = None
            bv = cur
            for dy, dx in _N8:
                ny_ = cy + dy
                nx_ = cx + dx
                if 0 <= ny_ < self.ny and 0 <= nx_ < self.nx:
                    vv = float(V[ny_, nx_])
                    if vv < bv - 1e-4:
                        bv = vv
                        best = (ny_, nx_, dy, dx)
            if best is None:
                break
            travelled += self.res * (1.41421356 if (best[2] and best[3]) else 1.0)
            cy, cx = best[0], best[1]
            cur = bv
            moved = True
            if travelled >= 1.9:
                break
        if not moved:
            return None
        return (self.origin[0] + (cx + 0.5) * self.res,
                self.origin[1] + (cy + 0.5) * self.res)
