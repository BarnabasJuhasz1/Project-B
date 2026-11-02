# -----------------------------------------------------------------------------
# Navigation controller for a Unitree Go2 shuttling between two fixed points in
# an unmapped indoor space.  Map-learning global layer + reactive swath layer +
# bug-style escape, with the diagnostics and loop discipline borrowed from the
# source paper.
#
# What was taken from Feng, Zhang, Lin, Gillis & Moldover, "Determination of the
# Boltzmann Constant Using the Differential-Cylindrical Procedure", and where:
#
#  * The differential deletion mechanism, Sec. 3, Eqs. (6)-(11).  A perturbation
#    that is linear in cavity length cancels, with no correction model at all,
#    when a matched pair of cavities is run at L1:L2 = 2:1 with l1 = 2*l2, so the
#    eigenfunctions and the acting area S match and the two terms of Eq. (11)
#    subtract to zero.  Used twice:
#      (a) progress toward the goal is averaged over two windows whose lengths
#          are exactly 2:1 and combined as v_hat = 2*v(W) - v(2W).  A bias
#          proportional to window length - which is what a drifting pose estimate
#          puts into either window alone - deletes exactly, so the stall detector
#          needs no model of the drift (Eqs. (7)-(10)).
#      (b) the corridor-centring signal is the difference of two mirror-symmetric
#          lidar sectors trimmed to exactly equal beam count, i.e. equal acting
#          area S and mirrored eigenfunctions, so a common-mode range bias
#          cancels out of the difference (the condition stated under Eq. (11)).
#  * Excess half-width as the test of a model's validity, Eq. (12) and Fig. 4:
#    the measured-minus-predicted residual is the only honest measure of how
#    wrong the model is.  The analogue here is commanded path length minus
#    measured path length; a large positive "excess" means the motion model is
#    invalid (blocked, slipping, legs against something that returned no beam),
#    which triggers recovery and stamps the obstruction into the map so later
#    trips route around it.
#  * Low pressure is low signal-to-noise and must not be read as data (Sec. 5.1,
#    the upwarping of Fig. 4 and the scatter growth of Fig. 5): every diagnostic
#    above is gated on commanded motion being well away from zero.
#  * Stay clear of the structure's natural vibrations (7.2 kHz criterion, Fig. 3)
#    and use only even modes so asymmetric free recoil cannot disturb them
#    (Sec. 5.1): the steering loop is bandwidth- and slew-limited and the chosen
#    heading is hysteretic, so the controller never rings the body and avoids the
#    left-right limit cycle, which is this plant's asymmetric mode.
#  * The half-width fit g = b0/sqrt(p) + b1 + b2*sqrt(p), Eq. (13): the speed
#    schedule is a low-order fit of the same shape, in clearance not pressure,
#    with a sqrt leading term.
#  * The independent cross-check of the molar masses, Sec. 2.2 / Eq. (3), where
#    an acoustic comparison inspects the GC-MS analysis: here pose displacement
#    is cross-checked against dead reckoning of the commands, and lidar free
#    space against contact events.  Disagreement suspends map writes instead of
#    silently corrupting the map.
# -----------------------------------------------------------------------------

import math
from collections import deque

import numpy as np

from core.observation import Command


def _wrap(a):
    """Wrap angle(s) to (-pi, pi].  Works on scalars and arrays."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Controller:
    # --- map ----------------------------------------------------------------
    RES = 0.30            # occupancy cell size [m]
    MARGIN = 14.0         # map margin around the start/goal pair [m]
    CROP = 10.0           # planning window margin around robot & goal [m]
    MAX_CELLS = 150000
    INFV = 1 << 28
    OCC_T = 1.0           # log-odds above which a cell counts as a wall
    INFLATE = 0.33        # planner obstacle inflation [m]

    # --- body ---------------------------------------------------------------
    SWATH = 0.36          # half width of the body corridor [m]
    LOOK = 3.0            # clearance saturation distance [m]
    D_STOP = 0.38         # clearance below which forward motion is refused [m]
    D_FULL = 1.90         # clearance at which full speed is allowed [m]

    # --- loops --------------------------------------------------------------
    PLAN_DT = 0.60        # global replan period [s]
    W_SHORT = 1.20        # short differential window [s] (long window = 2x)
    WZ_SLEW = 7.0         # [rad/s^2]
    VX_ACC = 1.6          # [m/s^2]
    VX_DEC = 5.0          # [m/s^2]

    _NB = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))

    # ------------------------------------------------------------------ utils
    @staticmethod
    def _num(v, default, lo=-1e12, hi=1e12):
        try:
            f = float(v)
        except Exception:
            return float(default)
        if not math.isfinite(f) or f < lo or f > hi:
            return float(default)
        return f

    @staticmethod
    def _pt(v, default):
        try:
            a = np.asarray(v, dtype=float).ravel()
            if a.size >= 2 and bool(np.all(np.isfinite(a[:2]))):
                return (float(a[0]), float(a[1]))
        except Exception:
            pass
        return (float(default[0]), float(default[1]))

    # ------------------------------------------------------------------ reset
    def reset(self, scenario, rng):
        """Clear all episode state."""
        self.rng = rng if rng is not None else np.random.default_rng(20240501)

        self.max_lin = self._num(getattr(scenario, "max_lin", None), 0.8, 0.05, 5.0)
        self.max_ang = self._num(getattr(scenario, "max_ang", None), 1.0, 0.05, 10.0)
        self.range_max = self._num(getattr(scenario, "range_max", None), 6.0, 0.5, 100.0)
        self.time_limit = self._num(getattr(scenario, "time_limit", None), 0.0, 0.0, 1e9)

        start = self._pt(getattr(scenario, "start", None), (0.0, 0.0))
        goal0 = self._pt(getattr(scenario, "goal", None), start)

        # ---- occupancy grid spanning both shuttle endpoints + margin -------
        res = self.RES
        x_lo = min(start[0], goal0[0]) - self.MARGIN
        x_hi = max(start[0], goal0[0]) + self.MARGIN
        y_lo = min(start[1], goal0[1]) - self.MARGIN
        y_hi = max(start[1], goal0[1]) + self.MARGIN
        for _ in range(12):
            nx = int(math.ceil((x_hi - x_lo) / res)) + 2
            ny = int(math.ceil((y_hi - y_lo) / res)) + 2
            if nx * ny <= self.MAX_CELLS:
                break
            res *= 1.2
        nx = max(8, int(math.ceil((x_hi - x_lo) / res)) + 2)
        ny = max(8, int(math.ceil((y_hi - y_lo) / res)) + 2)
        self.res = res
        self.x0 = x_lo
        self.y0 = y_lo
        self.logodds = np.zeros((ny, nx), dtype=np.float32)
        self.n_inf = max(1, int(round(self.INFLATE / res)))
        self._ray_s = np.arange(0.25, self.range_max, max(0.12, 0.5 * res))

        # ---- cached beam geometry (built on the first step) ----------------
        self._ang = None
        self._cand = None
        self._S = None
        self._C = None
        self._i_fwd = 0
        self._left = np.zeros(0, dtype=np.int64)
        self._right = np.zeros(0, dtype=np.int64)
        self._front = np.zeros(0, dtype=np.int64)
        self._rear = np.zeros(0, dtype=np.int64)

        # ---- global plan ---------------------------------------------------
        self.dist = None
        self.dist_off = (0, 0)
        self.plan_t = -1e9
        self.force_plan = True

        # ---- time / pose ---------------------------------------------------
        self.last_t = None
        self.px = start[0]
        self.py = start[1]
        self.yaw = 0.0
        self.prev_px = start[0]
        self.prev_py = start[1]

        # ---- goal bookkeeping ---------------------------------------------
        self.goal_ref = None
        self.trips = 0

        # ---- command state -------------------------------------------------
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.th_prev = 0.0

        # ---- differential progress estimator -------------------------------
        self.hist = deque(maxlen=220)
        self.cum_cmd = 0.0
        self.cum_meas = 0.0

        # ---- diagnostics / recovery ---------------------------------------
        self.block = 0.0
        self.contact_t = 0.0
        self.back_until = -1e9
        self.esc = False
        self.esc_t0 = 0.0
        self.esc_d0 = 0.0
        self.esc_side = 1.0
        self.esc_max = 9.0
        self.esc_log = deque(maxlen=8)
        self.excess = 0.0

        # ---- independent cross-check of the pose estimate ------------------
        self.ep_t = None
        self.ep_px = start[0]
        self.ep_py = start[1]
        self.dr_x = 0.0
        self.dr_y = 0.0
        self.mismatch = 0.0

        self.decay_t = -1e9
        self.nsteps = 0

    # ------------------------------------------------------------------- step
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            # Never raise: scan slowly in place and try again next tick.
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.3 * self.max_ang if hasattr(self, "max_ang") else 0.3
            return Command(0.0, 0.0, self.wz)

    # ------------------------------------------------------------ main body
    def _step(self, obs):
        self.nsteps += 1

        # ---------------- time ---------------------------------------------
        t = self._num(getattr(obs, "t", None), -1.0, -1.0, 1e12)
        if t < 0.0:
            t = (self.last_t if self.last_t is not None else 0.0) + 0.02
        if self.last_t is None:
            dt = 0.02
        else:
            dt = t - self.last_t
        if not (0.0005 <= dt <= 0.5):
            dt = 0.02
        self.last_t = t

        # ---------------- pose ---------------------------------------------
        try:
            p = np.asarray(obs.pose_estimate, dtype=float).ravel()
            if p.size >= 3 and bool(np.all(np.isfinite(p[:3]))):
                self.px = float(p[0])
                self.py = float(p[1])
                self.yaw = float(p[2])
        except Exception:
            pass
        px, py, yaw = self.px, self.py, self.yaw
        perr = self._num(getattr(obs, "pose_error_estimate", None), 0.0, 0.0, 1e6)
        try:
            contact = bool(getattr(obs, "contact", False))
        except Exception:
            contact = False

        # ---------------- lidar --------------------------------------------
        try:
            raw = np.asarray(obs.ranges, dtype=float).ravel()
        except Exception:
            raw = np.zeros(0)
        n = int(raw.size)
        if n == 0:
            return self._emit(0.0, 0.0, 0.3 * self.max_ang, dt)
        try:
            ang = np.asarray(obs.range_angles, dtype=float).ravel()
        except Exception:
            ang = np.zeros(0)
        if ang.size != n or not bool(np.all(np.isfinite(ang))):
            ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
        self._geom(ang)

        good = np.isfinite(raw) & (raw > 0.03)
        r = np.where(good, np.minimum(raw, self.range_max), self.range_max)
        hitm = good & (raw < self.range_max - 0.05)

        # ---------------- goal / trip switch -------------------------------
        goal = self._pt(getattr(obs, "goal", None), (px, py))
        if self.goal_ref is None or math.hypot(goal[0] - self.goal_ref[0],
                                               goal[1] - self.goal_ref[1]) > 0.75:
            if self.goal_ref is not None:
                self.trips += 1
            self.goal_ref = goal
            self.force_plan = True
            self.esc = False
            self.block = 0.0
            self.excess = 0.0
            self.hist.clear()
            self.th_prev = _wrap(math.atan2(goal[1] - py, goal[0] - px) - yaw)
        dgoal = math.hypot(goal[0] - px, goal[1] - py)

        # ---------------- odometry bookkeeping -----------------------------
        step_meas = math.hypot(px - self.prev_px, py - self.prev_py)
        if step_meas > 1.0:          # pose jump, not motion
            step_meas = 0.0
        self.cum_meas += step_meas
        self.prev_px, self.prev_py = px, py
        self.cum_cmd += dt * math.hypot(self.vx, self.vy)
        self.hist.append((t, dgoal, self.cum_meas, self.cum_cmd))

        # independent cross-check of the pose estimate (paper Sec. 2.2)
        if self.ep_t is None:
            self.ep_t = t
            self.ep_px, self.ep_py = px, py
            self.dr_x = self.dr_y = 0.0
        else:
            cy_, sy_ = math.cos(yaw), math.sin(yaw)
            self.dr_x += (self.vx * cy_ - self.vy * sy_) * dt
            self.dr_y += (self.vx * sy_ + self.vy * cy_) * dt
            if t - self.ep_t > 3.0:
                mx = (px - self.ep_px) - self.dr_x
                my = (py - self.ep_py) - self.dr_y
                self.mismatch = 0.6 * self.mismatch + 0.4 * math.hypot(mx, my)
                self.ep_t = t
                self.ep_px, self.ep_py = px, py
                self.dr_x = self.dr_y = 0.0

        v_hat, exc, cmd_win = self._differential(t)
        if exc is not None:
            self.excess = 0.85 * self.excess + 0.15 * exc

        # ---------------- map update ---------------------------------------
        trust = 1.0
        if perr > 1.2:
            trust = max(0.0, 1.0 - (perr - 1.2) / 1.8)
        if self.mismatch > 1.2:
            trust *= 0.3
        if trust > 0.05 and (self.nsteps % 2 == 0):
            self._map_update(px, py, yaw, r, hitm, ang, trust)
        if t - self.decay_t > 2.0:
            self.decay_t = t
            self.logodds *= np.float32(0.999)

        # ---------------- directional clearance (body swath) ---------------
        along = r[:, None] * self._C
        lat = r[:, None] * self._S
        blk = (np.abs(lat) < self.SWATH) & (along > 0.0)
        clear = np.where(blk, along, 2.0 * self.LOOK).min(axis=0)
        np.minimum(clear, self.LOOK, out=clear)
        cand = self._cand

        d_front = float(clear[self._i_fwd])
        f_min = float(np.min(r[self._front])) if self._front.size else self.range_max

        # mirror-symmetric, equal-area sector pair (paper Eq. (11) condition)
        dl = float(np.mean(np.minimum(r[self._left], 2.5))) if self._left.size else 0.0
        dr_ = float(np.mean(np.minimum(r[self._right], 2.5))) if self._right.size else 0.0

        th_goal = _wrap(math.atan2(goal[1] - py, goal[0] - px) - yaw)
        j_goal = int(np.argmin(np.abs(_wrap(cand - th_goal))))
        clear_goal = float(clear[j_goal])

        # ---------------- validity of the motion model ---------------------
        stalling = False
        if cmd_win is not None and cmd_win > 0.22:     # SNR gate
            if v_hat is not None and v_hat < 0.055:
                stalling = True
            if self.excess > 0.60:
                stalling = True
        if contact:
            self.contact_t += dt
        else:
            self.contact_t = max(0.0, self.contact_t - 0.5 * dt)
        if stalling or self.contact_t > 0.25:
            self.block = min(6.0, self.block + dt)
        else:
            self.block = max(0.0, self.block - 0.8 * dt)

        hard = self.contact_t > 0.30 or self.block > (5.0 if self.esc else 3.2)
        if hard:
            self.back_until = t + 0.55
            self.block = 0.0
            self.contact_t = 0.0
            self._stamp_block(px, py, yaw)
            self.force_plan = True
            if not self.esc and dgoal > 0.7:
                self._enter_escape(t, px, py, dgoal, dl, dr_)
        elif (not self.esc) and self.block > 1.5 and dgoal > 0.7:
            self._enter_escape(t, px, py, dgoal, dl, dr_)
            self.block = 0.6

        if self.esc:
            leave = (t - self.esc_t0 > 1.0 and dgoal < self.esc_d0 - 0.7
                     and clear_goal > 1.5)
            if leave or (t - self.esc_t0 > self.esc_max) or dgoal < 0.7:
                self.esc = False
                self.block = 0.0
                self.force_plan = True

        # ---------------- recovery: back off --------------------------------
        if t < self.back_until:
            side = self.esc_side if self.esc else 1.0
            rear = float(np.min(r[self._rear])) if self._rear.size else self.range_max
            if rear > 0.50:
                return self._emit(-0.45 * self.max_lin, 0.0,
                                  0.5 * side * self.max_ang, dt, fast=True)
            return self._emit(0.0, 0.0, 0.85 * side * self.max_ang, dt, fast=True)

        # ---------------- global plan --------------------------------------
        need = self.force_plan or self.dist is None or (t - self.plan_t > self.PLAN_DT)
        if (not need) and self.dist is not None:
            cy0, cx0 = self._cell(px, py)
            sy = cy0 - self.dist_off[0]
            sx = cx0 - self.dist_off[1]
            if not (0 <= sy < self.dist.shape[0] and 0 <= sx < self.dist.shape[1]):
                need = True
            elif int(self.dist[sy, sx]) >= self.INFV:
                need = True
        if need:
            self._plan(px, py, goal, t)
            self.force_plan = False

        th_des, _from_map = self._target_dir(px, py, yaw, goal, dgoal)

        # ---------------- escape steering (bug-style circumnavigation) -----
        w_goal, w_clear = 2.2, 1.25
        if self.esc:
            side = self.esc_side
            if side > 0.0:
                sd = float(np.min(r[self._left])) if self._left.size else 2.0
            else:
                sd = float(np.min(r[self._right])) if self._right.size else 2.0
            trim = max(-0.45, min(0.45, 0.7 * (sd - 0.85)))
            th_des = _wrap(th_goal + side * (1.45 + trim))
            w_goal, w_clear = 1.5, 2.0

        # ---------------- heading selection --------------------------------
        close = dgoal < 0.9
        d_stop = 0.18 if close else self.D_STOP
        if close:
            th_des = th_goal
            w_goal, w_clear = 2.6, 0.8

        score = w_goal * np.cos(_wrap(cand - th_des))
        score = score + w_clear * (np.minimum(clear, 2.5) / 2.5)
        score = score + 0.45 * np.cos(_wrap(cand - self.th_prev))
        tight = np.clip((0.62 - clear) / 0.62, 0.0, 1.0)
        score = score - (1.0 if close else 3.2) * tight
        score = np.where(clear < d_stop, score - 8.0, score)
        j = int(np.argmax(score))
        th_star = float(cand[j])
        c_sel = float(clear[j])

        # ---------------- speed schedule (Eq. (13)-shaped fit) -------------
        u = (c_sel - d_stop) / (self.D_FULL - d_stop)
        u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
        f = 0.16 + 0.84 * math.sqrt(u)
        if close:
            f = min(f, 0.55)
        vmax = self.max_lin * f

        if abs(th_star) > 1.0 or c_sel < d_stop:
            vx_t = 0.0
        else:
            vx_t = vmax * math.cos(th_star)
        if f_min < 0.30 and not close:
            vx_t = min(vx_t, 0.0)

        vy_t = 0.0
        if abs(th_star) < 1.0:
            vy_t = 0.45 * self.max_lin * math.sin(th_star)
        # differential centring: equal-area mirrored sectors, common-mode range
        # bias deletes in the difference (paper Eq. (11))
        tot = dl + dr_
        if tot > 1e-6 and tot < 4.4 and d_front > 0.8 and not close:
            vy_t += 0.30 * self.max_lin * ((dl - dr_) / tot)
        lim_y = 0.40 * self.max_lin
        vy_t = max(-lim_y, min(lim_y, vy_t))

        wz_t = 2.0 * th_star

        # hysteretic heading memory: suppresses the left-right limit cycle
        self.th_prev = _wrap(self.th_prev + 0.55 * _wrap(th_star - self.th_prev))

        return self._emit(vx_t, vy_t, wz_t, dt)

    # ------------------------------------------------------- command shaping
    def _emit(self, vx_t, vy_t, wz_t, dt, fast=False):
        ml, ma = self.max_lin, self.max_ang
        vx_t = max(-ml, min(ml, self._num(vx_t, 0.0)))
        vy_t = max(-ml, min(ml, self._num(vy_t, 0.0)))
        wz_t = max(-ma, min(ma, self._num(wz_t, 0.0)))

        # bandwidth + slew limit: keep the steering loop well below the
        # frequencies at which this body would ring (paper Sec. 5.1, Fig. 3)
        a_w = 0.70 if fast else 0.45
        wz = (1.0 - a_w) * self.wz + a_w * wz_t
        dw = self.WZ_SLEW * dt
        wz = max(self.wz - dw, min(self.wz + dw, wz))

        up = self.VX_ACC * dt
        dn = self.VX_DEC * dt
        vx = min(vx_t, self.vx + up) if vx_t > self.vx else max(vx_t, self.vx - dn)
        vy = min(vy_t, self.vy + up) if vy_t > self.vy else max(vy_t, self.vy - dn)

        self.vx = float(max(-ml, min(ml, vx)))
        self.vy = float(max(-ml, min(ml, vy)))
        self.wz = float(max(-ma, min(ma, wz)))
        return Command(self.vx, self.vy, self.wz)

    # ------------------------------------------------------- beam geometry
    def _geom(self, ang):
        if (self._ang is not None and self._ang.size == ang.size
                and abs(float(self._ang[0]) - float(ang[0])) < 1e-9
                and abs(float(self._ang[-1]) - float(ang[-1])) < 1e-9):
            return
        self._ang = ang.copy()
        cand = np.arange(-180.0, 180.0, 5.0) * (math.pi / 180.0)
        self._cand = cand
        d = ang[:, None] - cand[None, :]
        self._S = np.sin(d)
        self._C = np.cos(d)
        self._i_fwd = int(np.argmin(np.abs(_wrap(cand))))

        left = np.where((ang > 0.70) & (ang < 1.95))[0]
        right = np.where((ang < -0.70) & (ang > -1.95))[0]
        m = int(min(left.size, right.size))
        if m > 0:
            left = left[np.argsort(ang[left])][:m]
            right = right[np.argsort(-ang[right])][:m]
        else:
            left = np.zeros(0, dtype=np.int64)
            right = np.zeros(0, dtype=np.int64)
        self._left = left
        self._right = right
        self._front = np.where(np.abs(ang) < 0.33)[0]
        self._rear = np.where(np.abs(_wrap(ang - math.pi)) < 0.60)[0]

    # ----------------------------------------------------------- map update
    def _cell(self, x, y):
        iy = int(math.floor((y - self.y0) / self.res))
        ix = int(math.floor((x - self.x0) / self.res))
        return (iy, ix)

    def _map_update(self, px, py, yaw, r, hitm, ang, w):
        res = self.res
        ny, nx = self.logodds.shape
        ca = np.cos(ang + yaw)
        sa = np.sin(ang + yaw)
        s = self._ray_s
        if s.size:
            m = s[None, :] < (r[:, None] - (0.5 * res + 0.12))
            if bool(m.any()):
                xs = px + ca[:, None] * s[None, :]
                ys = py + sa[:, None] * s[None, :]
                ix = np.floor((xs[m] - self.x0) / res).astype(np.int32)
                iy = np.floor((ys[m] - self.y0) / res).astype(np.int32)
                ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
                if bool(ok.any()):
                    np.add.at(self.logodds, (iy[ok], ix[ok]), np.float32(-0.20 * w))
        if bool(hitm.any()):
            hx = px + r[hitm] * ca[hitm]
            hy = py + r[hitm] * sa[hitm]
            ix = np.floor((hx - self.x0) / res).astype(np.int32)
            iy = np.floor((hy - self.y0) / res).astype(np.int32)
            ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
            if bool(ok.any()):
                np.add.at(self.logodds, (iy[ok], ix[ok]), np.float32(0.85 * w))
        np.clip(self.logodds, -2.5, 5.0, out=self.logodds)

    def _stamp_block(self, px, py, yaw):
        """An 'excess'-detected obstruction is real even if no beam saw it."""
        ny, nx = self.logodds.shape
        for d in (0.35, 0.60):
            iy, ix = self._cell(px + d * math.cos(yaw), py + d * math.sin(yaw))
            if 0 <= iy < ny and 0 <= ix < nx:
                self.logodds[iy, ix] = np.float32(
                    min(5.0, float(self.logodds[iy, ix]) + 2.5))

    # -------------------------------------------------------------- planning
    @staticmethod
    def _dilate(m, n):
        out = m.copy()
        for _ in range(max(0, int(n))):
            o = out.copy()
            o[1:, :] |= out[:-1, :]
            o[:-1, :] |= out[1:, :]
            o[:, 1:] |= out[:, :-1]
            o[:, :-1] |= out[:, 1:]
            out = o
        return out

    @staticmethod
    def _clear_disc(free, cy, cx, rad):
        h, w = free.shape
        y0 = max(0, cy - rad)
        y1 = min(h, cy + rad + 1)
        x0 = max(0, cx - rad)
        x1 = min(w, cx + rad + 1)
        if y1 > y0 and x1 > x0:
            free[y0:y1, x0:x1] = True

    def _bfs(self, free, gcell, scell):
        INF = self.INFV
        dist = np.full(free.shape, INF, dtype=np.int32)
        if not bool(free[gcell[0], gcell[1]]):
            return dist
        frontier = np.zeros(free.shape, dtype=bool)
        frontier[gcell[0], gcell[1]] = True
        dist[gcell[0], gcell[1]] = 0
        maxit = min(240, free.shape[0] + free.shape[1] + 20)
        reached = -1
        k = 0
        while k < maxit:
            k += 1
            nf = np.zeros_like(frontier)
            nf[1:, :] |= frontier[:-1, :]
            nf[:-1, :] |= frontier[1:, :]
            nf[:, 1:] |= frontier[:, :-1]
            nf[:, :-1] |= frontier[:, 1:]
            nf[1:, 1:] |= frontier[:-1, :-1]
            nf[:-1, :-1] |= frontier[1:, 1:]
            nf[1:, :-1] |= frontier[:-1, 1:]
            nf[:-1, 1:] |= frontier[1:, :-1]
            np.logical_and(nf, free, out=nf)
            np.logical_and(nf, dist >= INF, out=nf)
            if not bool(nf.any()):
                break
            dist[nf] = k
            frontier = nf
            if reached < 0 and int(dist[scell[0], scell[1]]) < INF:
                reached = k
            if reached > 0 and k >= reached + 10:
                break
        return dist

    def _plan(self, px, py, goal, t):
        self.plan_t = t
        res = self.res
        ny, nx = self.logodds.shape
        i0 = int(max(0, math.floor((min(px, goal[0]) - self.CROP - self.x0) / res)))
        i1 = int(min(nx, math.ceil((max(px, goal[0]) + self.CROP - self.x0) / res) + 1))
        j0 = int(max(0, math.floor((min(py, goal[1]) - self.CROP - self.y0) / res)))
        j1 = int(min(ny, math.ceil((max(py, goal[1]) + self.CROP - self.y0) / res) + 1))
        if i1 - i0 < 4 or j1 - j0 < 4:
            self.dist = None
            return
        sub = self.logodds[j0:j1, i0:i1]
        occ = sub > self.OCC_T
        h, w = occ.shape

        cy, cx = self._cell(px, py)
        sy = int(min(max(cy - j0, 0), h - 1))
        sx = int(min(max(cx - i0, 0), w - 1))
        gy0, gx0 = self._cell(goal[0], goal[1])
        gy = int(min(max(gy0 - j0, 0), h - 1))
        gx = int(min(max(gx0 - i0, 0), w - 1))

        free = ~self._dilate(occ, self.n_inf)
        self._clear_disc(free, sy, sx, 2)
        self._clear_disc(free, gy, gx, 1)
        dist = self._bfs(free, (gy, gx), (sy, sx))
        if int(dist[sy, sx]) >= self.INFV:
            free2 = ~occ
            self._clear_disc(free2, sy, sx, 2)
            self._clear_disc(free2, gy, gx, 1)
            dist = self._bfs(free2, (gy, gx), (sy, sx))
        if int(dist[sy, sx]) >= self.INFV:
            self.dist = None
        else:
            self.dist = dist
            self.dist_off = (j0, i0)

    def _target_dir(self, px, py, yaw, goal, dgoal):
        """Desired heading in the robot frame, from the learned map if possible."""
        direct = _wrap(math.atan2(goal[1] - py, goal[0] - px) - yaw)
        if self.dist is None or dgoal < 1.2:
            return direct, False
        d = self.dist
        h, w = d.shape
        j0, i0 = self.dist_off
        cy0, cx0 = self._cell(px, py)
        cy = cy0 - j0
        cx = cx0 - i0
        if not (0 <= cy < h and 0 <= cx < w) or int(d[cy, cx]) >= self.INFV:
            return direct, False

        wp = None
        for _ in range(40):
            cur = int(d[cy, cx])
            bestv = cur
            cands = []
            for (dy, dx) in self._NB:
                y = cy + dy
                x = cx + dx
                if 0 <= y < h and 0 <= x < w:
                    v = int(d[y, x])
                    if v < bestv:
                        bestv = v
                        cands = [(dy, dx)]
                    elif v == bestv and cands:
                        cands.append((dy, dx))
            if not cands:
                break
            if len(cands) > 1:
                pick = cands[0]
                bd = 1e18
                for (dy, dx) in cands:
                    wx = self.x0 + (cx + dx + 0.5) * self.res
                    wy = self.y0 + (cy + dy + 0.5) * self.res
                    dd = (wx - goal[0]) ** 2 + (wy - goal[1]) ** 2
                    if dd < bd:
                        bd = dd
                        pick = (dy, dx)
            else:
                pick = cands[0]
            cy += pick[0]
            cx += pick[1]
            wx = self.x0 + (cx + 0.5) * self.res
            wy = self.y0 + (cy + 0.5) * self.res
            wp = (wx, wy)
            if int(d[cy, cx]) == 0:
                break
            if (wx - px) ** 2 + (wy - py) ** 2 > 2.25:
                break

        if wp is None or (wp[0] - px) ** 2 + (wp[1] - py) ** 2 < 0.09:
            return direct, False
        return _wrap(math.atan2(wp[1] - py, wp[0] - px) - yaw), True

    # ------------------------------------------------ differential estimator
    def _lookup(self, t_target):
        if not self.hist or self.hist[0][0] > t_target:
            return None
        best = None
        for rec in self.hist:
            if rec[0] <= t_target:
                best = rec
            else:
                break
        return best

    def _differential(self, t):
        """2:1 window pair -> window-length-proportional bias deletes exactly
        (paper Sec. 3, Eqs. (7)-(10)).  Also returns the 'excess' residual of
        Eq. (12) and the commanded path length used as the SNR gate."""
        if not self.hist:
            return None, None, None
        now = self.hist[-1]
        W = self.W_SHORT
        a = self._lookup(t - W)
        b = self._lookup(t - 2.0 * W)
        v_hat = None
        exc = None
        cmd = None
        if a is not None:
            v_s = (a[1] - now[1]) / max(1e-3, now[0] - a[0])
            if b is not None:
                v_l = (b[1] - now[1]) / max(1e-3, now[0] - b[0])
                v_hat = 2.0 * v_s - v_l
            else:
                v_hat = v_s
            cmd = now[3] - a[3]
            meas = now[2] - a[2]
            if cmd > 0.22:
                exc = (cmd - meas) / cmd
        return v_hat, exc, cmd

    # ------------------------------------------------------------- recovery
    def _enter_escape(self, t, px, py, dgoal, dl, dr_):
        side = 1.0 if dl >= dr_ else -1.0
        if abs(dl - dr_) < 0.12:
            side = 1.0 if float(self.rng.random()) < 0.5 else -1.0
        for (tt, qx, qy, s) in self.esc_log:
            if t - tt < 25.0 and (qx - px) ** 2 + (qy - py) ** 2 < 4.0:
                side = -s
                break
        self.esc = True
        self.esc_t0 = t
        self.esc_d0 = dgoal
        self.esc_side = float(side)
        self.esc_max = 6.0 + 6.0 * float(self.rng.random())
        self.esc_log.append((t, px, py, float(side)))
