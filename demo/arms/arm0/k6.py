"""
Navigation controller for a Unitree Go2 shuttling between two fixed points
inside an unknown indoor building.

Design
------
1. Occupancy mapping.  Every other control step the 360 deg lidar scan is
   ray-cast into a persistent log-odds grid expressed in the *estimated* pose
   frame.  The map is never cleared between one-way trips, so the second and
   every later traversal of the building is planned on a map that already
   contains the route.

2. Global planning.  A distance-to-goal field is grown from the goal over the
   grid with a Bellman-Ford / wavefront relaxation implemented with whole-array
   numpy shifts.  Unknown cells are traversable at a mild extra cost
   ("optimistic free space"), which makes the very first trip a goal-directed
   exploration instead of a blind bug walk.  The relaxation stops as soon as the
   wave has reached the robot, and is additionally wall-clock budgeted, so the
   control step always returns quickly.

3. Local control.  A dynamic-window search over (vx, vy, wz) candidates rolls
   each candidate forward analytically, scores it against the raw lidar points
   (instant reaction, independent of map/pose errors) and against the global
   distance field (no local minima), and the winner is rate limited.  The
   quadruped's lateral velocity is used, which makes doorways and tight corners
   much cheaper.

4. Robustness.  Stuck/no-progress/contact detection triggers an "escape to the
   most open direction" recovery plus a decaying cost stamp so the planner picks
   a different route.  Pose drift is compensated by remembering, for each of the
   two shuttle points, the estimated position at which the harness actually
   declared arrival, and aiming at that remembered offset; if the robot believes
   it is on the goal but no switch happens, it orbits the goal in a growing
   spiral until arrival registers.
"""

import math
import time

import numpy as np

from core.observation import Command


def _wrap(a):
    """Wrap angle(s) to [-pi, pi).  Works for scalars and arrays."""
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class Controller:
    # ---------------------------------------------------------------- tunables
    RES = 0.20            # nominal grid resolution [m]
    BIG = 1.0e6           # "unreachable" sentinel for the distance field
    MARGIN = 16.0         # map padding around the start/goal bounding box [m]
    MAX_CELLS = 170000    # cap on grid cells (resolution is coarsened if hit)

    OCC_TH = 0.40         # log-odds above which a cell counts as occupied
    UNK_TH = 0.12         # |log-odds| below which a cell counts as unknown
    W_FREE = 0.32         # log-odds decrement for a free ray sample
    W_OCC = 0.95          # log-odds increment for a beam endpoint
    LOG_LO = -2.0
    LOG_HI = 4.0

    R_COL = 0.32          # radius treated as a collision against lidar points
    R_PANIC = 0.20        # below this the robot only rotates / backs off

    ACC_LIN = 2.6         # command rate limits
    ACC_LAT = 2.6
    ACC_ANG = 9.0

    PLAN_EVERY = 20       # control steps between global replans
    PLAN_EVERY_FAIL = 8
    PLAN_BUDGET = 0.050   # seconds of wavefront relaxation per replan

    # ------------------------------------------------------------------ setup
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(20240917)

        self.range_max = self._f(getattr(scenario, "range_max", 6.0), 6.0, 0.5, 100.0)
        self.max_lin = self._f(getattr(scenario, "max_lin", 0.8), 0.8, 0.05, 5.0)
        self.max_ang = self._f(getattr(scenario, "max_ang", 1.0), 1.0, 0.05, 10.0)
        self.time_limit = self._f(getattr(scenario, "time_limit", 600.0), 600.0, 1.0, 1e6)

        start = self._xy(getattr(scenario, "start", (0.0, 0.0)))
        goal0 = self._xy(getattr(scenario, "goal", (0.0, 0.0)))

        # ---- grid geometry -------------------------------------------------
        lo = np.minimum(start, goal0) - self.MARGIN
        hi = np.maximum(start, goal0) + self.MARGIN
        res = self.RES
        nx = ny = 8
        for _ in range(14):
            nx = int(math.ceil((hi[0] - lo[0]) / res)) + 3
            ny = int(math.ceil((hi[1] - lo[1]) / res)) + 3
            nx = max(nx, 8)
            ny = max(ny, 8)
            if nx * ny <= self.MAX_CELLS:
                break
            res *= 1.15
        self.res = float(res)
        self.inv_res = 1.0 / self.res
        self.x0 = float(lo[0])
        self.y0 = float(lo[1])
        self.nx = int(nx)
        self.ny = int(ny)

        self.log = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.pen = np.zeros((self.ny, self.nx), dtype=np.float32)

        r_s, c_s = self._cell(float(start[0]), float(start[1]))
        self.bb = [r_s, r_s + 1, c_s, c_s + 1]   # touched-region bbox

        # ---- ray sampling offsets -----------------------------------------
        step = max(0.08, 0.6 * self.res)
        self.samples = np.arange(0.20, self.range_max + 1e-6, step, dtype=np.float64)
        if self.samples.size == 0:
            self.samples = np.array([0.2], dtype=np.float64)

        # ---- wavefront directions -----------------------------------------
        self._dirs = [(1, 0, False), (0, 1, False), (-1, 0, False), (0, -1, False),
                      (1, 1, True), (-1, -1, True), (1, -1, True), (-1, 1, True)]

        # ---- seed search offsets (nearest free cell to the goal) ----------
        offs = []
        rad = 16
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                offs.append((dr * dr + dc * dc, dr, dc))
        offs.sort()
        self.seed_offsets = offs

        # ---- dynamic window candidate set ---------------------------------
        vx = self.max_lin * np.array([-0.35, 0.0, 0.30, 0.55, 0.78, 1.00])
        vy = self.max_lin * np.array([-0.45, 0.0, 0.45])
        wz = self.max_ang * np.array([-1.0, -0.6, -0.3, 0.0, 0.3, 0.6, 1.0])
        VX, VY, WZ = np.meshgrid(vx, vy, wz, indexing="ij")
        self.cvx = VX.ravel().astype(np.float64)
        self.cvy = VY.ravel().astype(np.float64)
        self.cwz = WZ.ravel().astype(np.float64)

        T = np.array([0.22, 0.45, 0.70, 0.95, 1.20])
        th = self.cwz[:, None] * T[None, :]
        tiny = np.abs(self.cwz) < 1e-3
        w = np.where(tiny, 1.0, self.cwz)[:, None]
        straight_x = self.cvx[:, None] * T[None, :]
        straight_y = self.cvy[:, None] * T[None, :]
        arc_x = (self.cvx[:, None] * np.sin(th) + self.cvy[:, None] * (np.cos(th) - 1.0)) / w
        arc_y = (self.cvx[:, None] * (1.0 - np.cos(th)) + self.cvy[:, None] * np.sin(th)) / w
        self.rx = np.where(tiny[:, None], straight_x, arc_x)
        self.ry = np.where(tiny[:, None], straight_y, arc_y)
        self.rth = th

        # ---- scan angle cache ---------------------------------------------
        self.ang_cache = None
        self.cos_a = None
        self.sin_a = None

        # ---- planner state -------------------------------------------------
        self.D = None
        self.win = None
        self.path = None
        self.D_cur = self.BIG
        self.plan_ok = False
        self.plan_fail = 0
        self.plan_budget = self.PLAN_BUDGET
        self.last_plan_step = -10 ** 6
        self.force_replan = True

        # ---- episode state -------------------------------------------------
        self.t = 0.0
        self.t_prev = None
        self.step_i = 0
        self.prev_goal = None
        self.prev_pose = None
        self.anchors = {}
        self.trips = 0
        self.t_goal = 0.0

        self.mode = "nav"
        self.rec_t0 = -1e9
        self.rec_T = 1.2
        self.rec_u = np.array([1.0, 0.0])
        self.rec_count = 0

        self.gs_on = False
        self.gs_t0 = 0.0
        self.gs_phase = 0.0
        self.gs_sign = 1.0

        self.last_cmd = (0.0, 0.0, 0.0)
        self.hist = []
        self.best_d = float("inf")
        self.best_d_t = 0.0
        self.contact_run = 0
        self.stall_run = 0
        self.target = goal0.copy()

    # ------------------------------------------------------------ small utils
    @staticmethod
    def _f(v, default, lo=-1e12, hi=1e12):
        try:
            f = float(v)
            if math.isfinite(f) and lo <= f <= hi:
                return f
        except Exception:
            pass
        return float(default)

    @staticmethod
    def _xy(v, default=(0.0, 0.0)):
        try:
            a = np.asarray(v, dtype=float).ravel()
            if a.size >= 2 and bool(np.all(np.isfinite(a[:2]))):
                return a[:2].astype(float).copy()
        except Exception:
            pass
        return np.asarray(default, dtype=float)

    def _cell(self, x, y):
        c = int((x - self.x0) * self.inv_res)
        r = int((y - self.y0) * self.inv_res)
        if c < 0:
            c = 0
        elif c > self.nx - 1:
            c = self.nx - 1
        if r < 0:
            r = 0
        elif r > self.ny - 1:
            r = self.ny - 1
        return r, c

    # ------------------------------------------------------------------- step
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            # never raise: fall back to a trivial reactive behaviour
            try:
                return self._panic(obs)
            except Exception:
                return Command(0.0, 0.0, 0.4)

    def _panic(self, obs):
        r = np.asarray(obs.ranges, dtype=float).ravel()
        a = np.asarray(obs.range_angles, dtype=float).ravel()
        n = min(r.size, a.size)
        r = np.where(np.isfinite(r[:n]), r[:n], self.range_max)
        a = a[:n]
        k = int(np.argmax(r))
        ang = float(_wrap(a[k]))
        fwd = float(np.min(r[np.abs(_wrap(a)) < 0.5])) if np.any(np.abs(_wrap(a)) < 0.5) else 0.0
        vx = 0.35 if fwd > 1.0 else 0.0
        return Command(vx, 0.0, float(np.clip(1.5 * ang, -self.max_ang, self.max_ang)))

    # ------------------------------------------------------------------------
    def _step(self, obs):
        # ---------------- observation unpacking ----------------------------
        t = self._f(getattr(obs, "t", self.t + 0.02), self.t + 0.02)
        dt = 0.02 if self.t_prev is None else min(0.2, max(0.002, t - self.t_prev))
        self.t_prev = t
        self.t = t
        self.step_i += 1

        p = np.asarray(obs.pose_estimate, dtype=float).ravel()
        x = self._f(p[0] if p.size > 0 else 0.0, 0.0)
        y = self._f(p[1] if p.size > 1 else 0.0, 0.0)
        yaw = self._f(p[2] if p.size > 2 else 0.0, 0.0)
        perr = max(0.0, self._f(getattr(obs, "pose_error_estimate", 0.0), 0.0, -1e6, 1e6))
        contact = bool(getattr(obs, "contact", False))

        ranges = np.asarray(obs.ranges, dtype=float).ravel()
        angles = np.asarray(obs.range_angles, dtype=float).ravel()
        n = min(ranges.size, angles.size)
        if n == 0:
            return self._emit(0.0, 0.0, 0.3, dt)
        ranges = ranges[:n]
        angles = angles[:n]
        if (self.ang_cache is None or self.ang_cache.size != n
                or not np.array_equal(self.ang_cache, angles)):
            self.ang_cache = angles.copy()
            self.cos_a = np.cos(angles)
            self.sin_a = np.sin(angles)

        good = np.isfinite(ranges)
        rr = np.where(good, ranges, self.range_max)
        rr = np.clip(rr, 0.0, self.range_max)
        hit = good & (ranges > 0.12) & (ranges < self.range_max - 1e-3)

        # ---------------- goal / arrival bookkeeping -----------------------
        graw = self._xy(getattr(obs, "goal", (x, y)))
        if self.prev_goal is None:
            self.prev_goal = graw.copy()
            self.t_goal = t
            self.best_d = float("inf")
            self.best_d_t = t
        elif float(math.hypot(graw[0] - self.prev_goal[0], graw[1] - self.prev_goal[1])) > 0.30:
            if self.prev_pose is not None:
                self._record_arrival(self.prev_goal, self.prev_pose, perr)
            self.trips += 1
            self.prev_goal = graw.copy()
            self.t_goal = t
            self.gs_on = False
            self.mode = "nav"
            self.rec_count = 0
            self.hist = []
            self.best_d = float("inf")
            self.best_d_t = t
            self.force_replan = True
        self.prev_pose = np.array([x, y])

        target = self._target_for(graw, perr)
        self.target = target
        dist_tgt = float(math.hypot(target[0] - x, target[1] - y))

        # ---------------- mapping -----------------------------------------
        if (self.step_i & 1) == 0:
            self._map_update(x, y, yaw, rr, hit, perr)
        if (self.step_i % 25) == 0:
            self.pen *= 0.97

        # ---------------- contact handling ---------------------------------
        if contact:
            self.contact_run += 1
            if (self.contact_run % 5) == 1:
                self._stamp(x + 0.42 * math.cos(yaw), y + 0.42 * math.sin(yaw), 1, 0.45)
            if self.contact_run >= 10 and self.mode != "rec" and not self.gs_on:
                self._start_recovery(t, yaw, rr, escalate=True)
        else:
            self.contact_run = 0

        # ---------------- progress / stuck monitors ------------------------
        if not self.gs_on:
            if dist_tgt < self.best_d - 0.35:
                self.best_d = dist_tgt
                self.best_d_t = t
                self.rec_count = max(0, self.rec_count - 1) if dist_tgt < self.best_d + 1.0 else self.rec_count
        if (self.step_i % 5) == 0:
            self.hist.append((t, x, y))
            if len(self.hist) > 60:
                self.hist.pop(0)
        if self.mode == "nav" and not self.gs_on and len(self.hist) > 4:
            t0, x0h, y0h = self.hist[0]
            if (t - t0) > 3.2:
                if math.hypot(x - x0h, y - y0h) < 0.35:
                    self._start_recovery(t, yaw, rr, escalate=True)
            if (t - self.best_d_t) > 11.0:
                self._start_recovery(t, yaw, rr, escalate=True)
                self.best_d_t = t
                self.best_d = dist_tgt

        # ---------------- global replanning --------------------------------
        period = self.PLAN_EVERY if (self.D is not None and self.plan_fail == 0) else self.PLAN_EVERY_FAIL
        if self.force_replan or (self.step_i - self.last_plan_step) >= period:
            ok = self._plan(x, y, target, loose=False)
            if not ok and self.plan_fail >= 2:
                ok = self._plan(x, y, target, loose=True)
            self.last_plan_step = self.step_i
            self.force_replan = False
            if not ok and self.plan_fail >= 5:
                self.D = None
                self.path = None

        # current distance-field value at the robot
        self.plan_ok = False
        self.D_cur = self.BIG
        if self.D is not None:
            d0 = self._patch_min(x, y, 2)
            if d0 < 0.5 * self.BIG:
                self.D_cur = d0
                self.plan_ok = True

        # ---------------- goal-search (drift insurance) --------------------
        if self.mode == "nav" and not self.gs_on:
            if dist_tgt < 0.55 and (t - self.t_goal) > 1.2:
                self._start_goal_search(t)
            elif dist_tgt < 2.2 and self.rec_count >= 2 and (t - self.t_goal) > 7.0:
                self._start_goal_search(t)
        if self.gs_on and dist_tgt > 3.5:
            self.gs_on = False
            self.force_replan = True

        # ---------------- desired direction --------------------------------
        use_prog = False
        sscale = 1.0
        if self.mode == "rec":
            if (t - self.rec_t0) > self.rec_T:
                self.mode = "nav"
                self.force_replan = True
                self.hist = []
            u = self.rec_u
            sscale = 0.85
        if self.mode == "rec":
            pass
        elif self.gs_on:
            age = max(0.0, t - self.gs_t0)
            cyc = age % 6.0
            rad = 0.35 + 0.22 * cyc
            ang = self.gs_phase + self.gs_sign * 1.25 * age
            vt = target + rad * np.array([math.cos(ang), math.sin(ang)])
            d = vt - np.array([x, y])
            u = self._unit(d, target - np.array([x, y]))
            sscale = 0.62
        else:
            u = None
            if self.plan_ok and self.path is not None and len(self.path) >= 2:
                look = self._lookahead(x, y, 1.5)
                if look is not None:
                    d = look - np.array([x, y])
                    if float(d[0] * d[0] + d[1] * d[1]) > 0.02:
                        u = self._unit(d, target - np.array([x, y]))
                        use_prog = True
            if u is None:
                u = self._unit(target - np.array([x, y]), np.array([math.cos(yaw), math.sin(yaw)]))
                use_prog = self.plan_ok

        # ---------------- local obstacles for the DWA ----------------------
        m = (rr < 3.2) & (rr > 0.16)
        if bool(np.any(m)):
            ox = rr[m] * self.cos_a[m]
            oy = rr[m] * self.sin_a[m]
            if ox.size > 130:
                s = int(math.ceil(ox.size / 130.0))
                ox = ox[::s]
                oy = oy[::s]
        else:
            ox = np.zeros(0)
            oy = np.zeros(0)

        vx, vy, wz, cl = self._choose(x, y, yaw, u, use_prog, ox, oy)

        # ---------------- emergency / stall handling -----------------------
        if cl < self.R_PANIC:
            k = self._open_index(rr)
            ang_err = float(_wrap(self.ang_cache[k]))
            vx = -0.18 if float(np.min(rr[np.abs(_wrap(self.ang_cache)) < 0.6])) < 0.5 else 0.0
            vy = 0.0
            wz = float(np.clip(1.6 * ang_err, -self.max_ang, self.max_ang))
            if self.mode != "rec":
                self.stall_run += 3
        vx *= sscale
        vy *= sscale

        if abs(vx) < 0.05 and abs(vy) < 0.05:
            self.stall_run += 1
        else:
            self.stall_run = max(0, self.stall_run - 2)
        if self.stall_run > 60 and self.mode != "rec":
            self._start_recovery(t, yaw, rr, escalate=True)
            self.stall_run = 0

        return self._emit(vx, vy, wz, dt)

    # --------------------------------------------------------------- emitting
    def _emit(self, vx, vy, wz, dt):
        pvx, pvy, pwz = self.last_cmd
        dl = self.ACC_LIN * dt
        dt_lat = self.ACC_LAT * dt
        da = self.ACC_ANG * dt
        vx = float(np.clip(vx, pvx - dl, pvx + dl))
        vy = float(np.clip(vy, pvy - dt_lat, pvy + dt_lat))
        wz = float(np.clip(wz, pwz - da, pwz + da))
        vx = float(np.clip(vx, -self.max_lin, self.max_lin))
        vy = float(np.clip(vy, -self.max_lin, self.max_lin))
        wz = float(np.clip(wz, -self.max_ang, self.max_ang))
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            vx = vy = 0.0
            wz = 0.3
        self.last_cmd = (vx, vy, wz)
        return Command(vx, vy, wz)

    # ------------------------------------------------------- drift anchoring
    @staticmethod
    def _key(g):
        return (round(float(g[0]), 2), round(float(g[1]), 2))

    def _record_arrival(self, goal, pos, perr):
        off = np.asarray(pos, dtype=float)[:2] - np.asarray(goal, dtype=float)[:2]
        mag = float(math.hypot(off[0], off[1]))
        lim = 1.2 + 3.0 * perr
        if mag > lim and mag > 1e-6:
            off = off * (lim / mag)
        k = self._key(goal)
        old = self.anchors.get(k)
        self.anchors[k] = off if old is None else 0.5 * old + 0.5 * off

    def _target_for(self, graw, perr):
        tgt = graw.copy()
        off = self.anchors.get(self._key(graw))
        if off is not None:
            alpha = 0.75 * (perr / (perr + 0.25)) if perr > 0.0 else 0.0
            tgt = graw + alpha * off
        lox = self.x0 + 1.5 * self.res
        loy = self.y0 + 1.5 * self.res
        hix = self.x0 + (self.nx - 2.5) * self.res
        hiy = self.y0 + (self.ny - 2.5) * self.res
        tgt[0] = min(max(float(tgt[0]), lox), hix)
        tgt[1] = min(max(float(tgt[1]), loy), hiy)
        return tgt

    # ---------------------------------------------------------------- mapping
    def _map_update(self, x, y, yaw, rr, hit, perr):
        conf = float(np.clip(1.0 - 0.55 * perr, 0.30, 1.0))
        cy_ = math.cos(yaw)
        sy_ = math.sin(yaw)
        ca = cy_ * self.cos_a - sy_ * self.sin_a
        sa = sy_ * self.cos_a + cy_ * self.sin_a

        rfree = np.minimum(rr, self.range_max) - 0.55 * self.res
        S = self.samples
        M = S[None, :] < rfree[:, None]
        rmin = rmax = cmin = cmax = None
        if bool(np.any(M)):
            X = x + ca[:, None] * S[None, :]
            Y = y + sa[:, None] * S[None, :]
            cx = ((X[M] - self.x0) * self.inv_res).astype(np.int32)
            cyi = ((Y[M] - self.y0) * self.inv_res).astype(np.int32)
            np.clip(cx, 0, self.nx - 1, out=cx)
            np.clip(cyi, 0, self.ny - 1, out=cyi)
            self.log[cyi, cx] -= np.float32(self.W_FREE * conf)
            rmin = int(cyi.min()); rmax = int(cyi.max())
            cmin = int(cx.min()); cmax = int(cx.max())

        if bool(np.any(hit)):
            hx = x + ca[hit] * rr[hit]
            hy = y + sa[hit] * rr[hit]
            hc = ((hx - self.x0) * self.inv_res).astype(np.int32)
            hrw = ((hy - self.y0) * self.inv_res).astype(np.int32)
            np.clip(hc, 0, self.nx - 1, out=hc)
            np.clip(hrw, 0, self.ny - 1, out=hrw)
            self.log[hrw, hc] += np.float32(self.W_OCC * conf)
            a, b = int(hrw.min()), int(hrw.max())
            c, d = int(hc.min()), int(hc.max())
            rmin = a if rmin is None else min(rmin, a)
            rmax = b if rmax is None else max(rmax, b)
            cmin = c if cmin is None else min(cmin, c)
            cmax = d if cmax is None else max(cmax, d)

        np.clip(self.log, self.LOG_LO, self.LOG_HI, out=self.log)
        if rmin is not None:
            self.bb[0] = min(self.bb[0], rmin)
            self.bb[1] = max(self.bb[1], rmax + 1)
            self.bb[2] = min(self.bb[2], cmin)
            self.bb[3] = max(self.bb[3], cmax + 1)

    def _stamp(self, x, y, rad, amount):
        r, c = self._cell(x, y)
        r0 = max(0, r - rad); r1 = min(self.ny, r + rad + 1)
        c0 = max(0, c - rad); c1 = min(self.nx, c + rad + 1)
        sub = self.pen[r0:r1, c0:c1]
        sub += np.float32(amount)
        np.minimum(sub, np.float32(5.0), out=sub)

    # ----------------------------------------------------------- global plan
    @staticmethod
    def _dilate(mask, n):
        m = mask
        for _ in range(int(n)):
            out = m.copy()
            out[1:, :] |= m[:-1, :]
            out[:-1, :] |= m[1:, :]
            out[:, 1:] |= m[:, :-1]
            out[:, :-1] |= m[:, 1:]
            out[1:, 1:] |= m[:-1, :-1]
            out[1:, :-1] |= m[:-1, 1:]
            out[:-1, 1:] |= m[1:, :-1]
            out[:-1, :-1] |= m[1:, 1:]
            m = out
        return m

    def _relax(self, D, C1, C2, tmp):
        H, W = D.shape
        for dr, dc, diag in self._dirs:
            rt0 = dr if dr > 0 else 0
            rt1 = H + (dr if dr < 0 else 0)
            ct0 = dc if dc > 0 else 0
            ct1 = W + (dc if dc < 0 else 0)
            h = rt1 - rt0
            w = ct1 - ct0
            if h <= 0 or w <= 0:
                continue
            src = D[rt0 - dr:rt1 - dr, ct0 - dc:ct1 - dc]
            tgt = D[rt0:rt1, ct0:ct1]
            cc = (C2 if diag else C1)[rt0:rt1, ct0:ct1]
            out = tmp[:h, :w]
            np.add(src, cc, out=out)
            np.minimum(tgt, out, out=tgt)

    def _plan(self, x, y, target, loose=False):
        rb, cb = self._cell(x, y)
        gr, gc = self._cell(float(target[0]), float(target[1]))
        pad = 10
        r0 = max(0, min(self.bb[0], rb, gr) - pad)
        r1 = min(self.ny, max(self.bb[1], rb + 1, gr + 1) + pad)
        c0 = max(0, min(self.bb[2], cb, gc) - pad)
        c1 = min(self.nx, max(self.bb[3], cb + 1, gc + 1) + pad)
        H = r1 - r0
        W = c1 - c0
        if H < 4 or W < 4:
            self.plan_fail += 1
            return False

        lg = self.log[r0:r1, c0:c1]
        if loose:
            occ = lg > 1.2
            blocked = occ.copy()
            unk = np.abs(lg) < self.UNK_TH
            cost = np.ones((H, W), dtype=np.float32)
            cost += (0.30 * unk).astype(np.float32)
            n1 = self._dilate(blocked, 1)
            cost[n1] += np.float32(0.8)
        else:
            occ = lg > self.OCC_TH
            unk = np.abs(lg) < self.UNK_TH
            blocked = self._dilate(occ, 1)
            n1 = self._dilate(blocked, 1)
            n2 = self._dilate(n1, 1)
            cost = np.ones((H, W), dtype=np.float32)
            cost += (0.55 * unk).astype(np.float32)
            cost[n2] += np.float32(0.5)
            cost[n1] += np.float32(1.3)
            cost += np.minimum(self.pen[r0:r1, c0:c1], np.float32(4.0))

        lr = rb - r0
        lc = cb - c0
        a0 = max(0, lr - 2); a1 = min(H, lr + 3)
        b0 = max(0, lc - 2); b1 = min(W, lc + 3)
        blocked[a0:a1, b0:b1] = False          # we are demonstrably standing here
        np.clip(cost, 0.5, 8.0, out=cost)
        sub = cost[a0:a1, b0:b1]
        np.minimum(sub, np.float32(3.0), out=sub)

        C1 = (cost * np.float32(self.res))
        C2 = (cost * np.float32(1.41421356 * self.res))
        C1[blocked] = np.float32(self.BIG)
        C2[blocked] = np.float32(self.BIG)

        # ---- seed the wavefront at (or next to) the goal -------------------
        glr = gr - r0
        glc = gc - c0
        seed = None
        for _d2, dr, dc in self.seed_offsets:
            a = glr + dr
            b = glc + dc
            if 0 <= a < H and 0 <= b < W and not blocked[a, b]:
                seed = (a, b)
                break
        if seed is None:
            self.plan_fail += 1
            self.plan_budget = min(0.16, self.plan_budget * 1.6)
            return False

        D = np.full((H, W), np.float32(self.BIG), dtype=np.float32)
        D[seed[0], seed[1]] = np.float32(0.0)
        s0 = max(0, glr - 2); s1 = min(H, glr + 3)
        q0 = max(0, glc - 2); q1 = min(W, glc + 3)
        dv = D[s0:s1, q0:q1]
        dv[~blocked[s0:s1, q0:q1]] = np.float32(0.0)

        tmp = np.empty((H, W), dtype=np.float32)
        pr0 = max(0, lr - 3); pr1 = min(H, lr + 4)
        pc0 = max(0, lc - 3); pc1 = min(W, lc + 4)

        deadline = time.perf_counter() + self.plan_budget
        max_it = min(900, int(2.0 * (H + W)) + 40)
        reached = False
        extra = 0
        for it in range(max_it):
            self._relax(D, C1, C2, tmp)
            if reached:
                extra -= 1
                if extra <= 0 or time.perf_counter() > deadline + 0.05:
                    break
            elif (it & 3) == 3:
                if float(D[pr0:pr1, pc0:pc1].min()) < 0.5 * self.BIG:
                    reached = True
                    extra = max(6, it // 5)
                elif time.perf_counter() > deadline:
                    break

        if not reached:
            self.plan_fail += 1
            self.plan_budget = min(0.16, self.plan_budget * 1.6)
            return False

        path = self._extract(D, lr, lc, r0, c0)
        self.D = D
        self.win = (r0, r1, c0, c1)
        self.path = path
        self.plan_fail = 0
        self.plan_budget = self.PLAN_BUDGET
        return True

    def _extract(self, D, lr, lc, r0, c0):
        H, W = D.shape
        pr0 = max(0, lr - 3); pr1 = min(H, lr + 4)
        pc0 = max(0, lc - 3); pc1 = min(W, lc + 4)
        sub = D[pr0:pr1, pc0:pc1]
        k = int(np.argmin(sub))
        a = pr0 + k // sub.shape[1]
        b = pc0 + k % sub.shape[1]
        if float(D[a, b]) >= 0.5 * self.BIG:
            return None
        rows = [a]
        cols = [b]
        seen = set()
        seen.add((a, b))
        for _ in range(600):
            if float(D[a, b]) <= 0.0:
                break
            a2 = min(max(a, 1), H - 2)
            b2 = min(max(b, 1), W - 2)
            s = D[a2 - 1:a2 + 2, b2 - 1:b2 + 2]
            kk = int(np.argmin(s))
            na = a2 - 1 + kk // 3
            nb = b2 - 1 + kk % 3
            if (na, nb) in seen or float(D[na, nb]) >= float(D[a, b]):
                break
            seen.add((na, nb))
            a, b = na, nb
            rows.append(a)
            cols.append(b)
        if len(rows) < 2:
            return None
        xs = self.x0 + (np.asarray(cols, dtype=float) + c0 + 0.5) * self.res
        ys = self.y0 + (np.asarray(rows, dtype=float) + r0 + 0.5) * self.res
        return np.stack([xs, ys], axis=1)

    def _patch_min(self, x, y, k):
        if self.D is None or self.win is None:
            return self.BIG
        r, c = self._cell(x, y)
        r0, r1, c0, c1 = self.win
        lr = r - r0
        lc = c - c0
        H, W = self.D.shape
        a0 = max(0, lr - k); a1 = min(H, lr + k + 1)
        b0 = max(0, lc - k); b1 = min(W, lc + k + 1)
        if a0 >= a1 or b0 >= b1:
            return self.BIG
        return float(self.D[a0:a1, b0:b1].min())

    def _D_at(self, xs, ys):
        out = np.full(xs.shape, self.BIG, dtype=np.float64)
        if self.D is None or self.win is None:
            return out
        r0, r1, c0, c1 = self.win
        r = ((ys - self.y0) * self.inv_res).astype(np.int32)
        c = ((xs - self.x0) * self.inv_res).astype(np.int32)
        ok = (r >= r0) & (r < r1) & (c >= c0) & (c < c1)
        if bool(np.any(ok)):
            out[ok] = self.D[r[ok] - r0, c[ok] - c0]
        return out

    def _lookahead(self, x, y, dist):
        p = self.path
        if p is None or len(p) < 2:
            return None
        d2 = (p[:, 0] - x) ** 2 + (p[:, 1] - y) ** 2
        i = int(np.argmin(d2))
        rest = p[i:]
        if rest.shape[0] < 2:
            return p[-1]
        seg = np.sqrt(np.sum(np.diff(rest, axis=0) ** 2, axis=1))
        cum = np.cumsum(seg)
        j = int(np.searchsorted(cum, dist))
        idx = min(i + j + 1, p.shape[0] - 1)
        return p[idx]

    @staticmethod
    def _unit(v, fallback):
        v = np.asarray(v, dtype=float)
        nv = float(math.hypot(v[0], v[1]))
        if nv > 1e-6:
            return v / nv
        f = np.asarray(fallback, dtype=float)
        nf = float(math.hypot(f[0], f[1]))
        if nf > 1e-6:
            return f / nf
        return np.array([1.0, 0.0])

    # -------------------------------------------------------- local planning
    def _choose(self, x, y, yaw, u, use_prog, ox, oy):
        K = self.cvx.size
        if ox.size:
            dx = self.rx[:, :, None] - ox[None, None, :]
            dy = self.ry[:, :, None] - oy[None, None, :]
            d2 = dx * dx + dy * dy
            cl = np.sqrt(d2.reshape(K, -1).min(axis=1))
        else:
            cl = np.full(K, 9.0, dtype=np.float64)

        ct = math.cos(yaw)
        st = math.sin(yaw)
        ex = x + self.rx[:, -1] * ct - self.ry[:, -1] * st
        ey = y + self.rx[:, -1] * st + self.ry[:, -1] * ct

        adv = (ex - x) * u[0] + (ey - y) * u[1]
        psi = math.atan2(u[1], u[0])
        herr = np.abs(_wrap(yaw + self.rth[:, -1] - psi))

        sc = 2.2 * adv - 0.55 * herr
        sc += 0.30 * (self.cvx / self.max_lin)
        sc -= 0.30 * np.abs(self.cvy) / self.max_lin
        sc -= 0.12 * np.abs(self.cwz) / self.max_ang
        sc += 1.1 * np.clip(cl - 0.45, -0.25, 0.65)
        bad = cl < self.R_COL
        if bool(np.any(bad)):
            sc[bad] -= 60.0 + 150.0 * (self.R_COL - cl[bad])
        if use_prog and self.D is not None and self.D_cur < 0.5 * self.BIG:
            de = self._D_at(ex, ey)
            valid = de < 0.5 * self.BIG
            pr = np.where(valid, np.clip(self.D_cur - de, -2.0, 2.0), -1.8)
            sc += 1.7 * pr
        sc -= 0.25 * np.abs(self.cwz - self.last_cmd[2]) / self.max_ang

        k = int(np.argmax(sc))
        return float(self.cvx[k]), float(self.cvy[k]), float(self.cwz[k]), float(cl[k])

    # ------------------------------------------------------------- behaviours
    def _open_index(self, rr):
        n = rr.size
        if n < 7:
            return int(np.argmax(rr))
        ext = np.concatenate([rr[-3:], rr, rr[:3]])
        sm = np.convolve(ext, np.ones(5) / 5.0, mode="same")[3:3 + n]
        top = np.argsort(sm)[-6:]
        j = int(self.rng.integers(0, top.size))
        return int(top[j])

    def _start_recovery(self, t, yaw, rr, escalate=False):
        if self.mode == "rec" and (t - self.rec_t0) < self.rec_T:
            return
        k = self._open_index(rr)
        ang = yaw + float(self.ang_cache[k])
        self.rec_u = np.array([math.cos(ang), math.sin(ang)])
        self.mode = "rec"
        self.rec_t0 = t
        if escalate:
            self.rec_count = min(6, self.rec_count + 1)
        self.rec_T = 1.0 + 0.45 * min(4, self.rec_count)
        px, py = (self.prev_pose if self.prev_pose is not None else np.array([0.0, 0.0]))
        self._stamp(float(px), float(py), 1 + min(2, self.rec_count // 2),
                    0.7 + 0.5 * min(4, self.rec_count))
        self.hist = []
        self.force_replan = True

    def _start_goal_search(self, t):
        self.gs_on = True
        self.gs_t0 = t
        self.gs_phase = float(self.rng.uniform(0.0, 2.0 * math.pi))
        self.gs_sign = 1.0 if float(self.rng.random()) < 0.5 else -1.0
