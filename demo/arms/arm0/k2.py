"""
Navigation controller for a Unitree Go2 shuttling between two fixed points in
an unmapped indoor space.

Architecture (all of it sits above a frozen locomotion policy, output = Twist):

  1. Occupancy mapping .... a log-odds grid built from the 360 deg lidar, in the
     frame of `pose_estimate`.  The map is *persistent across trips* -- the
     robot walks the same building many times, so the second trip is planned on
     a map the first trip paid for.
  2. Pose correction ...... a small correlative scan matcher against a
     likelihood field derived from the accumulated map.  Heavily slew limited,
     so it can only ever nudge the odometry back towards the frame the map (and
     therefore the goal coordinates) live in.
  3. Global planning ...... a weighted wavefront (min-plus Bellman-Ford sweeps,
     fully vectorised) from the goal back to the robot, on an inflated grid in
     which *unknown space is passable but expensive*.  That single choice makes
     the planner do goal-directed exploration on trip 1 and exploit the known
     free corridors on every later trip.  Falls back to thinner inflation when
     the strict one leaves no corridor (doorways).
  4. Path following ....... string-pulled lookahead point + a swept-corridor
     reactive layer evaluated directly on the raw lidar, which owns the final
     velocity direction.  The planner proposes, the lidar disposes.
  5. Recovery ............. contact / no-clearance / no-progress detection,
     back-off, and a decaying penalty field that poisons places where the robot
     got stuck so the next plan routes around them.
"""

import math

import numpy as np

from core.observation import Command


def _wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Controller:

    # ------------------------------------------------------------------ map --
    RES = 0.20            # grid resolution [m]
    GROW_PAD = 5.0        # extra margin added whenever the grid has to grow [m]
    MAX_CELLS = 1_500_000
    L_FREE = 0.42         # log-odds decrement for a cell a beam passed through
    L_OCC = 1.00          # log-odds increment for a beam endpoint
    L_LO = -2.5
    L_HI = 4.5
    OCC_TH = 0.55         # log-odds above this  -> occupied
    UNK_TH = 0.22         # |log-odds| below this -> unknown

    # ---------------------------------------------------------------- robot --
    INFL_HI = 2           # preferred inflation [cells]  (0.40 m)
    INFL_LO = 1           # doorway fallback    [cells]  (0.20 m)
    CORRIDOR_HALF = 0.30  # half-width of the swept corridor used reactively [m]
    STOP_MARGIN = 0.32    # clearance at which the reactive layer commands zero

    # ------------------------------------------------------------- planning --
    PLAN_DT = 0.35        # nominal replan period [s]
    PLAN_MARGIN = 8.0     # window padding around (robot, goal) [m]
    MAX_WAVE_IT = 420
    WAVE_EXTRA = 8        # refinement sweeps after the robot is first reached
    LOOKAHEAD = 1.10      # base lookahead arc length [m]
    MAX_LOOK = 3.20       # max lookahead after line-of-sight string pulling [m]

    # ------------------------------------------------------------- recovery --
    STUCK_WINDOW = 3.5    # [s]
    STUCK_DIST = 0.40     # [m]
    BACKUP_TIME = 0.70    # [s]

    # ============================================================== lifecycle ==

    def reset(self, scenario, rng):
        """Clear every scrap of episode state."""
        self.rng = rng

        self.range_max = self._f(getattr(scenario, "range_max", 6.0), 6.0)
        self.max_lin = self._f(getattr(scenario, "max_lin", 0.8), 0.8)
        self.max_ang = self._f(getattr(scenario, "max_ang", 1.0), 1.0)
        self.time_limit = self._f(getattr(scenario, "time_limit", 1e9), 1e9)

        sx, sy = self._xy(getattr(scenario, "start", (0.0, 0.0)))
        gx, gy = self._xy(getattr(scenario, "goal", (0.0, 0.0)))

        # --- occupancy grid, sized to comfortably contain both shuttle ends ---
        pad = self.range_max + 8.0
        self.ox = math.floor((min(sx, gx) - pad) / self.RES) * self.RES
        self.oy = math.floor((min(sy, gy) - pad) / self.RES) * self.RES
        self.W = int(math.ceil((max(sx, gx) + pad - self.ox) / self.RES)) + 1
        self.H = int(math.ceil((max(sy, gy) + pad - self.oy) / self.RES)) + 1
        self.W = max(16, min(self.W, 900))
        self.H = max(16, min(self.H, 900))
        self.log = np.zeros((self.H, self.W), np.float32)
        self.pen = np.zeros((self.H, self.W), np.float32)

        # --- pose correction (map frame = odom frame + corr) ------------------
        self.corr = np.zeros(3, dtype=np.float64)
        self.lf = None
        self.occ_count = 0

        # --- plan / path ------------------------------------------------------
        self.path = None
        self.path_i = 0
        self.plan_blocked = None
        self.plan_ox = self.ox
        self.plan_oy = self.oy
        self.plan_x0 = 0
        self.plan_y0 = 0
        self.t_plan = -1e9
        self.plan_ok = False
        self.wall_sign = 1.0

        # --- goal bookkeeping --------------------------------------------------
        self.goal_ref = None
        self.trips = 0

        # --- recovery / motion state ------------------------------------------
        self.cmd = np.zeros(3, dtype=np.float64)
        self.mode = "nav"
        self.mode_end = -1.0
        self.escape_dir = 0.0
        self.hist_t = []
        self.hist_p = []
        self.stuck_n = 0
        self.t_last_stuck = -1e9
        self.spin_sign = 1.0

        self.k = 0
        self.last_t = 0.0

        # reactive candidate directions (robot frame offsets from desired)
        self.OFFS = np.deg2rad(
            np.array(
                [0, 10, -10, 20, -20, 32, -32, 45, -45, 60, -60, 75, -75,
                 90, -90, 110, -110, 135, -135, 160, -160, 180],
                dtype=np.float64,
            )
        )

        # 3x3x3 offset lattice used by the scan matcher
        g = (-1.0, 0.0, 1.0)
        self.SM_OFFS = np.array(
            [(i, j, k) for i in g for j in g for k in g], dtype=np.float64
        )

    # =================================================================== step ==

    def step(self, obs):
        try:
            vx, vy, wz = self._step(obs)
        except Exception:
            # Never raise: coast to a stop on the previous command.
            self.cmd *= 0.6
            vx, vy, wz = self.cmd[0], self.cmd[1], self.cmd[2]
        vx = float(np.clip(vx, -self.max_lin, self.max_lin))
        vy = float(np.clip(vy, -self.max_lin, self.max_lin))
        wz = float(np.clip(wz, -self.max_ang, self.max_ang))
        if not np.isfinite(vx):
            vx = 0.0
        if not np.isfinite(vy):
            vy = 0.0
        if not np.isfinite(wz):
            wz = 0.0
        return Command(vx, vy, wz)

    # ------------------------------------------------------------------------

    def _step(self, obs):
        self.k += 1
        t = self._f(getattr(obs, "t", None), self.last_t + 0.02)
        if not np.isfinite(t) or t < self.last_t:
            t = self.last_t + 0.02
        self.last_t = t

        # ---------------------------------------------------------- sensing --
        r = np.asarray(obs.ranges, dtype=np.float64).ravel()
        a = np.asarray(obs.range_angles, dtype=np.float64).ravel()
        n = int(min(r.size, a.size))
        r = r[:n].copy()
        a = a[:n]
        r[~np.isfinite(r)] = np.inf
        r[r <= 0.0] = np.inf

        pe = np.asarray(obs.pose_estimate, dtype=np.float64).ravel()
        rx = float(pe[0])
        ry = float(pe[1])
        ryaw = float(pe[2]) if pe.size > 2 else 0.0
        perr = abs(self._f(getattr(obs, "pose_error_estimate", 0.0), 0.0))
        contact = bool(getattr(obs, "contact", False))

        gx, gy = self._xy(obs.goal)

        # --------------------------------------------- pose correction (SLAM) --
        if self.k % 3 == 0 and t > 2.0 and self.lf is not None and self.occ_count > 250:
            self._scan_match(rx, ry, ryaw, r, a)

        px = rx + self.corr[0]
        py = ry + self.corr[1]
        yaw = ryaw + self.corr[2]

        # ------------------------------------------------------ grid growth --
        m = self.range_max + 1.5
        self._ensure(px - m, py - m, px + m, py + m)
        self._ensure(gx - 2.0, gy - 2.0, gx + 2.0, gy + 2.0)

        # ------------------------------------------------------ map update ---
        if self.k % 2 == 0:
            conf = float(np.clip(1.15 - 0.9 * perr, 0.30, 1.0))
            self._update_map(px, py, yaw, r, a, conf)

        # ------------------------------------------------- goal switch check --
        if self.goal_ref is None or math.hypot(gx - self.goal_ref[0],
                                               gy - self.goal_ref[1]) > 0.5:
            if self.goal_ref is not None:
                self.trips += 1
            self.goal_ref = (gx, gy)
            self.path = None
            self.path_i = 0
            self.plan_ok = False
            self.t_plan = -1e9
            self.pen *= 0.5
            self.hist_t = []
            self.hist_p = []
            self.mode = "nav"
            self.mode_end = -1.0

        # -------------------------------------------------- progress history --
        self.hist_t.append(t)
        self.hist_p.append((px, py))
        while len(self.hist_t) > 2 and t - self.hist_t[0] > self.STUCK_WINDOW + 0.5:
            self.hist_t.pop(0)
            self.hist_p.pop(0)

        dgoal = math.hypot(gx - px, gy - py)

        # ------------------------------------------------- recovery triggers --
        if self.mode == "nav":
            trig = False
            if contact:
                trig = True
                self._mark_contact(px, py, yaw, r, a)
            elif self._stuck(t):
                trig = True
                self.stuck_n = self.stuck_n + 1 if t - self.t_last_stuck < 25.0 else 1
                self.t_last_stuck = t
                amt = 1.0 + 0.9 * min(self.stuck_n, 4)
                self._bump_penalty(px, py, amt, 1 + min(self.stuck_n, 3))
                self.plan_ok = False
                self.t_plan = -1e9
            if trig and dgoal > 0.45:
                self.mode = "backup"
                self.mode_end = t + self.BACKUP_TIME
                self.escape_dir = self._escape_direction(r, a)
                self.hist_t = []
                self.hist_p = []
                self.spin_sign = 1.0 if self.rng.random() < 0.5 else -1.0

        if self.mode == "backup" and t >= self.mode_end:
            self.mode = "nav"
            self.hist_t = []
            self.hist_p = []

        # ------------------------------------------------------ backup mode ---
        if self.mode == "backup":
            phi = self.escape_dir
            cl = self._clearance(r, a, np.array([phi]))[0]
            if cl < 0.30:
                # nowhere to back into -- rotate in place instead
                return 0.0, 0.0, self.spin_sign * 0.9 * self.max_ang
            sp = min(0.34 * self.max_lin + 0.15, 0.55 * max(0.0, cl - 0.18))
            vx = sp * math.cos(phi)
            vy = sp * math.sin(phi)
            wz = self.spin_sign * 0.45 * self.max_ang
            return self._smooth(vx, vy, wz, rate=0.06)

        # ---------------------------------------------------------- planning --
        need = (
            (not self.plan_ok)
            or (self.path is None)
            or (t - self.t_plan > self.PLAN_DT)
            or self._path_invalid(px, py)
        )
        if need:
            self.t_plan = t
            self._plan(px, py, gx, gy)

        # --------------------------------------------------- desired heading --
        tgt = self._lookahead(px, py, gx, gy, dgoal)
        wdir = math.atan2(tgt[1] - py, tgt[0] - px)
        phi_des = _wrap(wdir - yaw)

        # ------------------------------------------------------- reactive -----
        phi, vfrac, cl = self._choose(r, a, phi_des)

        speed = vfrac * self.max_lin
        # walking sideways / backwards is slower and less stable than forwards
        speed *= max(0.30, 0.55 + 0.45 * math.cos(phi))
        # ease in on the final approach so we definitely settle inside the goal
        if dgoal < 0.85:
            speed = min(speed, max(0.22, 1.05 * dgoal))
        # be a little more careful when the localiser admits it is lost
        if perr > 0.45:
            speed *= 0.85

        if vfrac < 0.06 and cl < self.STOP_MARGIN + 0.10:
            # boxed in -- hand over to recovery next tick
            self.mode = "backup"
            self.mode_end = t + self.BACKUP_TIME
            self.escape_dir = self._escape_direction(r, a)
            self.spin_sign = 1.0 if self.rng.random() < 0.5 else -1.0
            return self._smooth(0.0, 0.0, 0.0, rate=0.08)

        vx = speed * math.cos(phi)
        vy = speed * math.sin(phi)

        # yaw servo: keep the body pointing where we are actually going
        err = phi
        if abs(err) > 2.85:
            err = self.spin_sign * abs(err)   # break the 180 deg tie consistently
        else:
            self.spin_sign = 1.0 if err >= 0.0 else -1.0
        wz = float(np.clip(1.9 * err, -self.max_ang, self.max_ang))

        return self._smooth(vx, vy, wz, rate=0.055)

    # ============================================================== mapping ===

    def _update_map(self, px, py, yaw, r, a, conf):
        fin = np.isfinite(r)
        if not fin.any():
            return
        rr = r[fin]
        aa = a[fin] + yaw
        rr = np.minimum(rr, self.range_max)
        ca = np.cos(aa)
        sa = np.sin(aa)
        res = self.RES

        # ---- free space along each ray -------------------------------------
        K = int(self.range_max / res) + 2
        tt = (np.arange(K, dtype=np.float64) * res)[None, :]
        keep = tt <= (rr[:, None] - 0.60 * res)
        if keep.any():
            fx = (px + tt * ca[:, None])[keep]
            fy = (py + tt * sa[:, None])[keep]
            ix = ((fx - self.ox) / res).astype(np.int32)
            iy = ((fy - self.oy) / res).astype(np.int32)
            ok = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
            if ok.any():
                self.log[iy[ok], ix[ok]] -= np.float32(self.L_FREE * conf)

        # ---- endpoints ------------------------------------------------------
        hit = rr < (self.range_max - 0.18)
        if hit.any():
            hx = px + rr[hit] * ca[hit]
            hy = py + rr[hit] * sa[hit]
            ix = ((hx - self.ox) / res).astype(np.int32)
            iy = ((hy - self.oy) / res).astype(np.int32)
            ok = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
            if ok.any():
                self.log[iy[ok], ix[ok]] += np.float32(self.L_OCC * conf)

        np.clip(self.log, self.L_LO, self.L_HI, out=self.log)

    def _mark_contact(self, px, py, yaw, r, a):
        """A bump is ground truth: stamp an obstacle where the lidar is closest."""
        fin = np.isfinite(r)
        if not fin.any():
            return
        idx = int(np.argmin(np.where(fin, r, np.inf)))
        d = min(float(r[idx]), 0.6)
        ang = yaw + float(a[idx])
        wx = px + d * math.cos(ang)
        wy = py + d * math.sin(ang)
        ix = int((wx - self.ox) / self.RES)
        iy = int((wy - self.oy) / self.RES)
        if 0 <= ix < self.W and 0 <= iy < self.H:
            x0 = max(0, ix - 1)
            x1 = min(self.W, ix + 2)
            y0 = max(0, iy - 1)
            y1 = min(self.H, iy + 2)
            self.log[y0:y1, x0:x1] = np.float32(self.L_HI)

    def _ensure(self, xmin, ymin, xmax, ymax):
        res = self.RES
        tx = self.ox + self.W * res
        ty = self.oy + self.H * res
        al = int(math.ceil((self.ox - xmin) / res)) if xmin < self.ox else 0
        ab = int(math.ceil((self.oy - ymin) / res)) if ymin < self.oy else 0
        ar = int(math.ceil((xmax - tx) / res)) if xmax > tx else 0
        at = int(math.ceil((ymax - ty) / res)) if ymax > ty else 0
        if al == 0 and ar == 0 and ab == 0 and at == 0:
            return
        pad = int(math.ceil(self.GROW_PAD / res))
        al = al + pad if al > 0 else 0
        ar = ar + pad if ar > 0 else 0
        ab = ab + pad if ab > 0 else 0
        at = at + pad if at > 0 else 0
        nW = self.W + al + ar
        nH = self.H + ab + at
        if nW * nH > self.MAX_CELLS or nW > 2000 or nH > 2000:
            return
        nlog = np.zeros((nH, nW), np.float32)
        nlog[ab:ab + self.H, al:al + self.W] = self.log
        npen = np.zeros((nH, nW), np.float32)
        npen[ab:ab + self.H, al:al + self.W] = self.pen
        self.log = nlog
        self.pen = npen
        self.ox -= al * res
        self.oy -= ab * res
        self.W = nW
        self.H = nH
        self.lf = None

    def _bump_penalty(self, x, y, amt, rad):
        ix = int((x - self.ox) / self.RES)
        iy = int((y - self.oy) / self.RES)
        x0 = max(0, ix - rad)
        x1 = min(self.W, ix + rad + 1)
        y0 = max(0, iy - rad)
        y1 = min(self.H, iy + rad + 1)
        if x1 > x0 and y1 > y0:
            self.pen[y0:y1, x0:x1] += np.float32(amt)
            np.clip(self.pen, 0.0, 6.0, out=self.pen)

    @staticmethod
    def _dil1(m):
        """One step of 8-connected boolean dilation."""
        o = m.copy()
        o[1:, :] |= m[:-1, :]
        o[:-1, :] |= m[1:, :]
        o[:, 1:] |= m[:, :-1]
        o[:, :-1] |= m[:, 1:]
        o[1:, 1:] |= m[:-1, :-1]
        o[1:, :-1] |= m[:-1, 1:]
        o[:-1, 1:] |= m[1:, :-1]
        o[:-1, :-1] |= m[1:, 1:]
        return o

    # ========================================================= scan matching ==

    def _scan_match(self, rx, ry, ryaw, r, a):
        sel = np.isfinite(r) & (r > 0.35) & (r < min(5.0, self.range_max - 0.4))
        idx = np.nonzero(sel)[0]
        if idx.size < 25:
            return
        if idx.size > 110:
            idx = idx[:: max(1, idx.size // 110)]
        rr = r[idx]
        aa = a[idx]

        cur = self.corr.copy()
        base_score = None
        best_score = None

        for scale in (1.0, 0.35):
            step = np.array([0.11 * scale, 0.11 * scale, 0.016 * scale])
            cand = cur[None, :] + self.SM_OFFS * step[None, :]
            ang = (ryaw + cand[:, 2])[:, None] + aa[None, :]
            wx = (rx + cand[:, 0])[:, None] + rr[None, :] * np.cos(ang)
            wy = (ry + cand[:, 1])[:, None] + rr[None, :] * np.sin(ang)
            ix = ((wx - self.ox) / self.RES).astype(np.int32)
            iy = ((wy - self.oy) / self.RES).astype(np.int32)
            np.clip(ix, 0, self.W - 1, out=ix)
            np.clip(iy, 0, self.H - 1, out=iy)
            sc = self.lf[iy, ix].sum(axis=1)
            zero = 13  # index of the (0,0,0) offset in the 3x3x3 lattice
            if base_score is None:
                base_score = float(sc[zero])
            k = int(np.argmax(sc))
            best_score = float(sc[k])
            cur = cand[k].copy()

        if base_score is None or best_score is None:
            return
        if best_score < base_score * 1.004 + 0.4:
            return

        d = cur - self.corr
        d[0] = float(np.clip(d[0], -0.012, 0.012))
        d[1] = float(np.clip(d[1], -0.012, 0.012))
        d[2] = float(np.clip(d[2], -0.0040, 0.0040))
        new = self.corr + d
        if abs(new[0]) < 4.0 and abs(new[1]) < 4.0 and abs(new[2]) < 0.7:
            self.corr = new

    # =============================================================== planner ==

    def _plan(self, px, py, gx, gy):
        res = self.RES
        occ = self.log > self.OCC_TH
        self.occ_count = int(occ.sum())

        # dilation ladder, reused for inflation, clearance costing and the
        # scan-matching likelihood field
        lad = [occ]
        for _ in range(self.INFL_HI + 4):
            lad.append(self._dil1(lad[-1]))

        # ---- likelihood field (cheap, shares the ladder) ---------------------
        lf = occ.astype(np.float32)
        lf += 0.50 * (lad[1] & ~lad[0])
        lf += 0.20 * (lad[2] & ~lad[1])
        self.lf = lf

        self.pen *= np.float32(0.985)

        # ---- planning window -------------------------------------------------
        six = int((px - self.ox) / res)
        siy = int((py - self.oy) / res)
        gix = int((gx - self.ox) / res)
        giy = int((gy - self.oy) / res)
        six = int(np.clip(six, 0, self.W - 1))
        siy = int(np.clip(siy, 0, self.H - 1))
        gix = int(np.clip(gix, 0, self.W - 1))
        giy = int(np.clip(giy, 0, self.H - 1))
        mg = int(self.PLAN_MARGIN / res)
        x0 = max(0, min(six, gix) - mg)
        x1 = min(self.W, max(six, gix) + mg + 1)
        y0 = max(0, min(siy, giy) - mg)
        y1 = min(self.H, max(siy, giy) + mg + 1)
        if x1 - x0 < 4 or y1 - y0 < 4:
            self.plan_ok = False
            return

        unk = np.abs(self.log[y0:y1, x0:x1]) < self.UNK_TH
        pen = self.pen[y0:y1, x0:x1]
        sx = six - x0
        sy = siy - y0
        tx = gix - x0
        ty = giy - y0

        for infl in (self.INFL_HI, self.INFL_LO, 0):
            blocked = lad[infl][y0:y1, x0:x1].copy()
            # the robot is, by definition, standing somewhere legal
            blocked[max(0, sy - 1):sy + 2, max(0, sx - 1):sx + 2] = False
            g = self._nearest_free(blocked, tx, ty)
            if g is None:
                continue
            ggx, ggy = g

            near1 = lad[infl + 2][y0:y1, x0:x1] & ~blocked
            near2 = lad[infl + 4][y0:y1, x0:x1] & ~blocked & ~near1
            cost = np.ones(blocked.shape, np.float32)
            cost += np.float32(0.55) * unk
            cost += np.float32(0.95) * near1
            cost += np.float32(0.35) * near2
            cost += pen

            dist = self._wave(cost, blocked, ggx, ggy, sx, sy)
            if dist is None or dist[sy, sx] > 4.9e8:
                continue

            cells = self._descend(dist, sx, sy, ggx, ggy)
            if len(cells) < 1:
                continue
            pts = np.empty((len(cells), 2), dtype=np.float64)
            for i, (cx, cy) in enumerate(cells):
                pts[i, 0] = self.ox + (cx + x0 + 0.5) * res
                pts[i, 1] = self.oy + (cy + y0 + 0.5) * res
            # always terminate exactly on the commanded goal
            pts = np.vstack([pts, np.array([[gx, gy]])])

            self.path = pts
            self.path_i = 0
            self.plan_blocked = blocked
            self.plan_ox = self.ox
            self.plan_oy = self.oy
            self.plan_x0 = x0
            self.plan_y0 = y0
            self.plan_ok = True
            return

        self.plan_ok = False

    def _wave(self, cost, blocked, gx, gy, sx, sy):
        """Weighted wavefront from (gx, gy); vectorised min-plus sweeps."""
        H, W = cost.shape
        INF = np.float32(1e9)
        c = np.where(blocked, INF, cost).astype(np.float32)
        s2 = np.float32(1.4142135)
        wc = [
            (0, 1, c), (0, -1, c), (1, 0, c), (-1, 0, c),
            (1, 1, c * s2), (1, -1, c * s2), (-1, 1, c * s2), (-1, -1, c * s2),
        ]
        dist = np.full((H, W), INF, np.float32)
        dist[gy, gx] = np.float32(0.0)
        P = np.full((H + 2, W + 2), INF, np.float32)

        # Chebyshev lower bound on how many sweeps are needed at minimum
        lim = min(self.MAX_WAVE_IT, 6 * (H + W))
        reached = -1
        for it in range(lim):
            P[1:-1, 1:-1] = dist
            m = dist
            for dx, dy, w in wc:
                nb = P[1 + dy:1 + dy + H, 1 + dx:1 + dx + W]
                m = np.minimum(m, nb + w)
            dist = m
            if reached < 0:
                if dist[sy, sx] < 4.9e8:
                    reached = it
            elif it - reached >= self.WAVE_EXTRA:
                break
        return dist

    @staticmethod
    def _nearest_free(blocked, gx, gy, maxr=16):
        H, W = blocked.shape
        gx = int(np.clip(gx, 0, W - 1))
        gy = int(np.clip(gy, 0, H - 1))
        if not blocked[gy, gx]:
            return gx, gy
        for rad in range(1, maxr + 1):
            y0 = max(0, gy - rad)
            y1 = min(H, gy + rad + 1)
            x0 = max(0, gx - rad)
            x1 = min(W, gx + rad + 1)
            sub = blocked[y0:y1, x0:x1]
            fy, fx = np.nonzero(~sub)
            if fy.size:
                yy = fy + y0
                xx = fx + x0
                d = (yy - gy) ** 2 + (xx - gx) ** 2
                k = int(np.argmin(d))
                return int(xx[k]), int(yy[k])
        return None

    @staticmethod
    def _descend(dist, sx, sy, gx, gy, maxsteps=1600):
        H, W = dist.shape
        INF = np.float32(1e9)
        D = np.full((H + 2, W + 2), INF, np.float32)
        D[1:-1, 1:-1] = dist
        y = sy + 1
        x = sx + 1
        out = []
        for _ in range(maxsteps):
            out.append((x - 1, y - 1))
            here = D[y, x]
            if here <= np.float32(0.0):
                break
            blk = D[y - 1:y + 2, x - 1:x + 2]
            k = int(np.argmin(blk))
            if blk.flat[k] >= here:
                break
            dy, dx = divmod(k, 3)
            y += dy - 1
            x += dx - 1
            if (x - 1) == gx and (y - 1) == gy:
                out.append((x - 1, y - 1))
                break
        return out

    # ========================================================= path following ==

    def _path_invalid(self, px, py):
        if self.path is None or len(self.path) == 0:
            return True
        i = min(self.path_i, len(self.path) - 1)
        d = math.hypot(self.path[i, 0] - px, self.path[i, 1] - py)
        return d > 2.5

    def _lookahead(self, px, py, gx, gy, dgoal):
        if not self.plan_ok or self.path is None or len(self.path) < 2:
            return (gx, gy)
        path = self.path
        nseg = len(path)

        # re-anchor onto the path (search forward only, bounded)
        lo = self.path_i
        hi = min(nseg, lo + 70)
        seg = path[lo:hi]
        d2 = (seg[:, 0] - px) ** 2 + (seg[:, 1] - py) ** 2
        self.path_i = lo + int(np.argmin(d2))

        # walk forward by arc length to the base lookahead
        i = self.path_i
        acc = 0.0
        j = i
        while j + 1 < nseg and acc < self.LOOKAHEAD:
            acc += float(np.hypot(path[j + 1, 0] - path[j, 0],
                                  path[j + 1, 1] - path[j, 1]))
            j += 1

        # string-pull: push the target as far ahead as line of sight allows
        best = j
        acc2 = acc
        step = 3
        tries = 0
        k = j
        while k + step < nseg and acc2 < self.MAX_LOOK and tries < 10:
            nxt = k + step
            acc2 += float(np.hypot(path[nxt, 0] - path[k, 0],
                                   path[nxt, 1] - path[k, 1]))
            if self._los(px, py, path[nxt, 0], path[nxt, 1]):
                best = nxt
                k = nxt
            else:
                break
            tries += 1

        # short final run: aim straight at the true goal
        if dgoal < 1.3 and self._los(px, py, gx, gy):
            return (gx, gy)
        return (float(path[best, 0]), float(path[best, 1]))

    def _los(self, x0, y0, x1, y1):
        b = self.plan_blocked
        if b is None:
            return False
        res = self.RES
        d = math.hypot(x1 - x0, y1 - y0)
        n = int(d / (0.6 * res)) + 2
        if n > 90:
            n = 90
        ts = np.linspace(0.0, 1.0, n)
        xs = x0 + ts * (x1 - x0)
        ys = y0 + ts * (y1 - y0)
        ix = ((xs - self.plan_ox) / res).astype(np.int32) - self.plan_x0
        iy = ((ys - self.plan_oy) / res).astype(np.int32) - self.plan_y0
        H, W = b.shape
        if (ix < 0).any() or (ix >= W).any() or (iy < 0).any() or (iy >= H).any():
            return False
        return not bool(b[iy, ix].any())

    # ============================================================== reactive ==

    def _clearance(self, r, a, phis):
        """Free distance along each of `phis` inside a swept corridor."""
        fin = np.isfinite(r)
        if not fin.any():
            return np.full(phis.shape, self.range_max, dtype=np.float64)
        rr = r[fin]
        aa = a[fin]
        d = _wrap(aa[None, :] - phis[:, None])
        fwd = rr[None, :] * np.cos(d)
        lat = rr[None, :] * np.sin(d)
        ok = (fwd > 0.0) & (np.abs(lat) < self.CORRIDOR_HALF)
        big = np.where(ok, fwd, self.range_max)
        cl = big.min(axis=1)
        return np.clip(cl, 0.0, self.range_max)

    def _choose(self, r, a, phi_des):
        offs = self.OFFS
        phis = _wrap(phi_des + offs)
        cl = self._clearance(r, a, phis)
        v = np.clip(0.55 * (cl - self.STOP_MARGIN), 0.0, 1.0)

        score = (
            v * np.cos(offs)
            + 0.16 * np.minimum(cl, 3.0) / 3.0
            - 0.11 * np.abs(offs) / math.pi
        )
        # when there is no global plan, commit to one wall-following handedness
        if not self.plan_ok:
            score += 0.10 * self.wall_sign * np.sign(offs)
        # hard veto on candidates with no room at all
        score = np.where(v < 0.02, score - 1.5, score)

        k = int(np.argmax(score))
        if v[k] < 0.02:
            k = int(np.argmax(cl))
            self.wall_sign = -self.wall_sign
        return float(phis[k]), float(v[k]), float(cl[k])

    def _escape_direction(self, r, a):
        """Point away from the nearest return, biased to whichever way is open."""
        fin = np.isfinite(r)
        if not fin.any():
            return math.pi
        idx = int(np.argmin(np.where(fin, r, np.inf)))
        base = _wrap(float(a[idx]) + math.pi)
        cands = _wrap(base + np.deg2rad(np.array([0.0, 25.0, -25.0, 50.0, -50.0])))
        cl = self._clearance(r, a, cands)
        return float(cands[int(np.argmax(cl))])

    # =============================================================== helpers ==

    def _stuck(self, t):
        if len(self.hist_t) < 5:
            return False
        if t - self.hist_t[0] < self.STUCK_WINDOW:
            return False
        x0, y0 = self.hist_p[0]
        xs = [p[0] for p in self.hist_p]
        ys = [p[1] for p in self.hist_p]
        span = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        net = math.hypot(self.hist_p[-1][0] - x0, self.hist_p[-1][1] - y0)
        return net < self.STUCK_DIST and span < 1.6 * self.STUCK_DIST

    def _smooth(self, vx, vy, wz, rate=0.055):
        tgt = np.array([vx, vy, wz], dtype=np.float64)
        lim = np.array([rate, rate, max(0.10, rate * 2.6)], dtype=np.float64)
        d = np.clip(tgt - self.cmd, -lim, lim)
        self.cmd = self.cmd + d
        return float(self.cmd[0]), float(self.cmd[1]), float(self.cmd[2])

    @staticmethod
    def _f(v, default):
        try:
            f = float(v)
            return f if np.isfinite(f) else default
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _xy(p):
        try:
            arr = np.asarray(p, dtype=np.float64).ravel()
            return float(arr[0]), float(arr[1])
        except (TypeError, ValueError, IndexError):
            return 0.0, 0.0
