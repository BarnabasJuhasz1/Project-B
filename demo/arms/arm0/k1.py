"""Map-building shuttle navigator for an unmapped indoor space.

Architecture (all above the frozen locomotion layer, Twist only):

  1. Occupancy mapping    - log-odds grid in the pose-estimate frame, built from
                            the 360 deg lidar.  Persists for the whole episode,
                            so the second and later one-way trips run over an
                            already-known building.
  2. Global navigation    - a Dijkstra-style value function (min-plus value
     function                iteration, vectorised with numpy slice shifts) over
                            the inflated cost map, anchored at the current goal.
                            Unknown cells are optimistically traversable, so the
                            same machinery explores on trip 1 and exploits later.
                            The solve is amortised across control steps with a
                            fixed per-step iteration budget.
  3. Local planner        - dynamic-window rollout of (vx, vy, wz) candidates
                            scored by the value function at the arc endpoint plus
                            clearance against the *raw* scan in the robot frame
                            (so it stays safe even when the map or the pose
                            estimate is wrong).
  4. Recovery             - stuck / no-progress detection, back-up-and-turn, and
                            decaying cost blobs dropped at trap locations so the
                            global planner reroutes instead of re-entering them.
"""

import math

import numpy as np

from core.observation import Command

_BIG = 1.0e9


def _as_xy(value, fallback):
    """Best-effort (x, y) extraction; never raises."""
    try:
        a = np.asarray(value, dtype=np.float64).ravel()
        if a.size >= 2:
            x = float(a[0])
            y = float(a[1])
            if math.isfinite(x) and math.isfinite(y):
                return np.array([x, y], dtype=np.float64)
    except Exception:
        pass
    return np.array([float(fallback[0]), float(fallback[1])], dtype=np.float64)


def _as_float(value, fallback):
    try:
        v = float(value)
        if math.isfinite(v):
            return v
    except Exception:
        pass
    return float(fallback)


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class Controller:
    # --- geometry / safety -------------------------------------------------
    ROBOT_R = 0.28          # effective footprint radius [m]
    HARD_CLEAR = 0.04       # extra clearance a trajectory must keep [m]
    SOFT_CLEAR = 0.55       # clearance above which no bonus is gained [m]
    HORIZON = 1.30          # rollout horizon [s]
    NT = 5                  # rollout samples

    # --- scoring weights ---------------------------------------------------
    W_PROG = 3.2
    W_CLEAR = 1.0
    W_SPEED = 0.35
    W_HEAD = 0.50
    W_SMOOTH = 0.30
    W_VY = 0.20
    W_PROX = 1.6

    # --- planner -----------------------------------------------------------
    PLAN_PERIOD = 0.7       # seconds between global replans
    ITERS_PER_STEP = 14     # value-iteration sweeps per control step
    ITERS_BOOT = 36         # ... while no plan has ever been published
    PUBLISH_EVERY = 50      # publish a partial field this often

    # ======================================================================
    #  lifecycle
    # ======================================================================
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(20240917)

        self.max_lin = max(0.05, _as_float(getattr(scenario, "max_lin", 0.8), 0.8))
        self.max_ang = max(0.05, _as_float(getattr(scenario, "max_ang", 1.0), 1.0))
        self.range_max = max(0.5, _as_float(getattr(scenario, "range_max", 6.0), 6.0))
        self.time_limit = _as_float(getattr(scenario, "time_limit", 600.0), 600.0)

        p0 = _as_xy(getattr(scenario, "start", (0.0, 0.0)), (0.0, 0.0))
        p1 = _as_xy(getattr(scenario, "goal", (0.0, 0.0)), (0.0, 0.0))

        # ---------------- occupancy grid ----------------------------------
        margin = 12.0
        lo = np.minimum(p0, p1) - margin
        hi = np.maximum(p0, p1) + margin
        span = np.maximum(hi - lo, 10.0)
        maxdim = 170
        res = float(np.max(span)) / float(maxdim)
        res = float(min(0.45, max(0.18, res)))
        self.res = res
        nx = int(min(maxdim, max(24, math.ceil(span[0] / res))))
        ny = int(min(maxdim, max(24, math.ceil(span[1] / res))))
        self.nx = nx
        self.ny = ny
        centre = 0.5 * (p0 + p1)
        self.gx0 = float(centre[0] - 0.5 * nx * res)
        self.gy0 = float(centre[1] - 0.5 * ny * res)

        self.logodds = np.zeros((ny, nx), dtype=np.float32)
        self.penalty = np.zeros((ny, nx), dtype=np.float32)

        # ---------------- lidar ray sampling ------------------------------
        sstep = 0.62 * res
        nsamp = int(max(2, math.ceil(self.range_max / sstep)))
        self._ray_s = (np.arange(1, nsamp + 1, dtype=np.float64) * sstep)

        # ---------------- dynamic-window candidate set --------------------
        self._build_candidates()

        # ---------------- planner state -----------------------------------
        self.D = None                 # published value function
        self._Dw = None               # working value function
        self._cs = None
        self._cd = None
        self._plan_active = False
        self._plan_iters = 0
        self._plan_max_iters = 2 * (nx + ny)
        self._plan_t0 = -1.0e9
        self._force_plan = True
        self._pen_t = 0.0

        # ---------------- episode / leg state -----------------------------
        self.goal = p1.copy()
        self._t = 0.0
        self._k = 0
        self._yaw = 0.0
        self._pos = p0.copy()
        self._have_pose = False
        self._trace = []
        self._leg_t0 = 0.0
        self._legs = 0
        self._best_dg = 1.0e9
        self._best_t = 0.0

        self._recover_until = -1.0
        self._recover_sign = 1.0
        self._escape_until = -1.0
        self._bump_until = -1.0
        self._bump_dir = 0.0
        self._contact_run = 0.0

        self._cmd = np.zeros(3, dtype=np.float64)
        self._fail = 0

    # ----------------------------------------------------------------------
    def _build_candidates(self):
        L = self.max_lin
        A = self.max_ang
        vxs = [0.0, 0.28 * L, 0.55 * L, 0.78 * L, L]
        vys = [-0.42 * L, 0.0, 0.42 * L]
        wzs = np.linspace(-A, A, 15)

        cand = []
        for vx in vxs:
            for vy in vys:
                for wz in wzs:
                    cand.append((vx, vy, float(wz)))
        for wz in (-0.95 * A, -0.45 * A, 0.0, 0.45 * A, 0.95 * A):
            cand.append((-0.40 * L, 0.0, float(wz)))

        V = np.asarray(cand, dtype=np.float64)
        nc = V.shape[0]
        self._nc = nc
        self._nt = self.NT
        self._vx = V[:, 0].astype(np.float64)
        self._vy = V[:, 1].astype(np.float64)
        self._wz = V[:, 2].astype(np.float64)

        ts = (np.arange(1, self.NT + 1, dtype=np.float64) / float(self.NT)) * self.HORIZON
        vx = self._vx[:, None]
        vy = self._vy[:, None]
        wz = self._wz[:, None]
        small = np.abs(wz) < 1.0e-4
        wsafe = np.where(small, 1.0, wz)
        th = wz * ts[None, :]
        sth = np.sin(th)
        cth = np.cos(th)
        px = np.where(small, vx * ts[None, :], (vx * sth + vy * (cth - 1.0)) / wsafe)
        py = np.where(small, vy * ts[None, :], (vx * (1.0 - cth) + vy * sth) / wsafe)

        self._tp = np.stack([px.ravel(), py.ravel()], axis=1).astype(np.float32)
        self._epx = px[:, -1].astype(np.float64)
        self._epy = py[:, -1].astype(np.float64)
        self._eth = (self._wz * self.HORIZON).astype(np.float64)

    # ======================================================================
    #  grid helpers
    # ======================================================================
    def _cell(self, x, y):
        ix = int(math.floor((x - self.gx0) / self.res))
        iy = int(math.floor((y - self.gy0) / self.res))
        inb = (0 <= ix < self.nx) and (0 <= iy < self.ny)
        ix = min(self.nx - 1, max(0, ix))
        iy = min(self.ny - 1, max(0, iy))
        return ix, iy, inb

    def _cell_xy(self, ix, iy):
        return (self.gx0 + (ix + 0.5) * self.res,
                self.gy0 + (iy + 0.5) * self.res)

    @staticmethod
    def _dilate(mask, k):
        out = mask.copy()
        for _ in range(int(k)):
            nxt = out.copy()
            nxt[1:, :] |= out[:-1, :]
            nxt[:-1, :] |= out[1:, :]
            nxt[:, 1:] |= out[:, :-1]
            nxt[:, :-1] |= out[:, 1:]
            out = nxt
        return out

    def _window(self, x, y, radius):
        ix, iy, _ = self._cell(x, y)
        k = int(max(0, round(radius / self.res)))
        y0 = max(0, iy - k)
        y1 = min(self.ny, iy + k + 1)
        x0 = max(0, ix - k)
        x1 = min(self.nx, ix + k + 1)
        return slice(y0, y1), slice(x0, x1)

    def _add_penalty(self, x, y, radius, amp):
        sy, sx = self._window(x, y, radius)
        if sy.stop > sy.start and sx.stop > sx.start:
            blk = self.penalty[sy, sx]
            np.minimum(blk + np.float32(amp), np.float32(8.0), out=blk)
            self.penalty[sy, sx] = blk

    # ======================================================================
    #  mapping
    # ======================================================================
    def _update_map(self, x, y, yaw, r_eff, ang, hit, perr):
        res = self.res
        wa = ang + yaw
        ca = np.cos(wa)
        sa = np.sin(wa)

        occ_gain = np.float32(0.95 / (1.0 + 1.6 * max(0.0, perr)))
        free_gain = np.float32(0.42)

        # --- free space along every beam ---------------------------------
        s = self._ray_s
        lim = r_eff - 0.75 * res
        m = s[None, :] < lim[:, None]
        if np.any(m):
            fx = (x + ca[:, None] * s[None, :])[m]
            fy = (y + sa[:, None] * s[None, :])[m]
            ix = np.floor((fx - self.gx0) / res).astype(np.int64)
            iy = np.floor((fy - self.gy0) / res).astype(np.int64)
            inb = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            if np.any(inb):
                flat = np.unique(iy[inb] * self.nx + ix[inb])
                lg = self.logodds.reshape(-1)
                lg[flat] -= free_gain

        # --- returns ------------------------------------------------------
        if np.any(hit):
            hr = r_eff[hit]
            hx = x + np.cos(wa[hit]) * hr
            hy = y + np.sin(wa[hit]) * hr
            ix = np.floor((hx - self.gx0) / res).astype(np.int64)
            iy = np.floor((hy - self.gy0) / res).astype(np.int64)
            inb = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            if np.any(inb):
                flat = np.unique(iy[inb] * self.nx + ix[inb])
                lg = self.logodds.reshape(-1)
                lg[flat] += occ_gain

        np.clip(self.logodds, -2.5, 3.5, out=self.logodds)

        # the robot is certainly standing in free space
        sy, sx = self._window(x, y, 0.22)
        if sy.stop > sy.start and sx.stop > sx.start:
            blk = self.logodds[sy, sx]
            np.minimum(blk, np.float32(-1.2), out=blk)
            self.logodds[sy, sx] = blk

    # ======================================================================
    #  global planner
    # ======================================================================
    def _start_plan(self, t, x, y, gx, gy):
        res = self.res
        lg = self.logodds

        # decay trap penalties
        dtp = max(0.0, t - self._pen_t)
        self._pen_t = t
        if dtp > 0.0:
            self.penalty *= np.float32(math.exp(-dtp / 30.0))

        occ = lg > np.float32(0.55)
        unk = np.abs(lg) < np.float32(0.12)

        lethal = self._dilate(occ, 1) if res <= 0.26 else occ.copy()
        k1 = max(1, int(round(0.32 / res)))
        k2 = max(1, int(round(0.45 / res)))
        near1 = self._dilate(lethal, k1)
        near2 = self._dilate(near1, k2)

        # guarantee a solvable problem around the robot and around the goal
        sy, sx = self._window(x, y, 0.45)
        lethal[sy, sx] = False
        sy, sx = self._window(gx, gy, 0.40)
        lethal[sy, sx] = False

        cost = np.ones((self.ny, self.nx), dtype=np.float32)
        cost += np.where(unk, np.float32(0.35), np.float32(0.0))
        cost += np.where(near1 & ~lethal, np.float32(2.4), np.float32(0.0))
        cost += np.where(near2 & ~near1, np.float32(0.80), np.float32(0.0))
        cost += np.minimum(self.penalty, np.float32(8.0))

        cs = cost * np.float32(res)
        cd = cost * np.float32(res * 1.41421356)
        big = np.float32(_BIG)
        cs[lethal] = big
        cd[lethal] = big
        self._cs = cs
        self._cd = cd

        D = np.full((self.ny, self.nx), big, dtype=np.float32)
        gi, gj, _ = self._cell(gx, gy)
        y0 = max(0, gj - 1)
        y1 = min(self.ny, gj + 2)
        x0 = max(0, gi - 1)
        x1 = min(self.nx, gi + 2)
        D[y0:y1, x0:x1] = np.float32(0.0)
        self._Dw = D

        self._plan_active = True
        self._plan_iters = 0
        self._plan_t0 = t
        self._force_plan = False

    def _relax(self, iters):
        D = self._Dw
        cs = self._cs
        cd = self._cd
        if D is None or cs is None:
            return True
        done = False
        for _ in range(int(iters)):
            N = D.copy()
            np.minimum(N[1:, :], D[:-1, :] + cs[1:, :], out=N[1:, :])
            np.minimum(N[:-1, :], D[1:, :] + cs[:-1, :], out=N[:-1, :])
            np.minimum(N[:, 1:], D[:, :-1] + cs[:, 1:], out=N[:, 1:])
            np.minimum(N[:, :-1], D[:, 1:] + cs[:, :-1], out=N[:, :-1])
            np.minimum(N[1:, 1:], D[:-1, :-1] + cd[1:, 1:], out=N[1:, 1:])
            np.minimum(N[:-1, :-1], D[1:, 1:] + cd[:-1, :-1], out=N[:-1, :-1])
            np.minimum(N[1:, :-1], D[:-1, 1:] + cd[1:, :-1], out=N[1:, :-1])
            np.minimum(N[:-1, 1:], D[1:, :-1] + cd[:-1, 1:], out=N[:-1, 1:])
            self._plan_iters += 1
            done = not bool(np.any(N < D - np.float32(1.0e-3)))
            D = N
            self._Dw = D
            if done:
                break
        return done

    def _plan_tick(self, t, x, y, gx, gy):
        if (not self._plan_active) and (
            self._force_plan
            or self.D is None
            or (t - self._plan_t0) > self.PLAN_PERIOD
        ):
            self._start_plan(t, x, y, gx, gy)

        if not self._plan_active:
            return

        budget = self.ITERS_BOOT if self.D is None else self.ITERS_PER_STEP
        done = self._relax(budget)
        if done or self._plan_iters >= self._plan_max_iters:
            self.D = self._Dw
            self._plan_active = False
        elif (self._plan_iters % self.PUBLISH_EVERY) < budget:
            # publish an intermediate (admissible, goal-outward-exact) field
            self.D = self._Dw

    # ----------------------------------------------------------------------
    def _value_at(self, x, y, search=True):
        D = self.D
        if D is None:
            return _BIG
        ix, iy, inb = self._cell(x, y)
        if not inb:
            return _BIG
        v = float(D[iy, ix])
        if v < 0.5 * _BIG or not search:
            return v
        y0 = max(0, iy - 3)
        y1 = min(self.ny, iy + 4)
        x0 = max(0, ix - 3)
        x1 = min(self.nx, ix + 4)
        sub = D[y0:y1, x0:x1]
        v = float(sub.min())
        return v

    def _descend(self, x, y, max_len=2.6):
        """Walk downhill on the value function to get a local waypoint."""
        D = self.D
        if D is None:
            return None
        ix, iy, inb = self._cell(x, y)
        if not inb:
            return None
        cur = float(D[iy, ix])
        if cur >= 0.5 * _BIG:
            y0 = max(0, iy - 3)
            y1 = min(self.ny, iy + 4)
            x0 = max(0, ix - 3)
            x1 = min(self.nx, ix + 4)
            sub = D[y0:y1, x0:x1]
            k = int(np.argmin(sub))
            if float(sub.flat[k]) >= 0.5 * _BIG:
                return None
            iy = y0 + k // sub.shape[1]
            ix = x0 + k % sub.shape[1]
            cur = float(D[iy, ix])
        steps = int(max(2, round(max_len / self.res)))
        for _ in range(steps):
            y0 = max(0, iy - 1)
            y1 = min(self.ny, iy + 2)
            x0 = max(0, ix - 1)
            x1 = min(self.nx, ix + 2)
            sub = D[y0:y1, x0:x1]
            k = int(np.argmin(sub))
            v = float(sub.flat[k])
            if v >= cur - 1.0e-4:
                break
            iy = y0 + k // sub.shape[1]
            ix = x0 + k % sub.shape[1]
            cur = v
        return self._cell_xy(ix, iy)

    # ======================================================================
    #  control
    # ======================================================================
    def step(self, obs):
        try:
            cmd = self._step(obs)
            self._fail = 0
            return cmd
        except Exception:
            self._fail += 1
            if self._fail > 20:
                return Command(0.0, 0.0, 0.35)
            v = self._cmd * 0.5
            return Command(float(v[0]), float(v[1]), float(v[2]))

    # ----------------------------------------------------------------------
    def _step(self, obs):
        # ---------------- time -------------------------------------------
        t = _as_float(getattr(obs, "t", None), self._t + 0.02)
        if t < self._t:
            t = self._t + 0.02
        dt = t - self._t if self._k > 0 else 0.02
        dt = float(min(0.25, max(1.0e-3, dt)))
        self._t = t
        self._k += 1

        # ---------------- pose --------------------------------------------
        try:
            pe = np.asarray(getattr(obs, "pose_estimate", None), dtype=np.float64).ravel()
        except Exception:
            pe = np.zeros(0)
        if pe.size >= 2 and np.all(np.isfinite(pe[:2])):
            x = float(pe[0])
            y = float(pe[1])
            if pe.size >= 3 and math.isfinite(float(pe[2])):
                self._yaw = float(pe[2])
        else:
            x = float(self._pos[0])
            y = float(self._pos[1])
        yaw = self._yaw
        self._pos = np.array([x, y])
        self._have_pose = True
        perr = max(0.0, _as_float(getattr(obs, "pose_error_estimate", 0.0), 0.0))

        # ---------------- goal / leg switch --------------------------------
        g = _as_xy(getattr(obs, "goal", None), self.goal)
        if float(np.hypot(g[0] - self.goal[0], g[1] - self.goal[1])) > 0.75:
            self._new_leg(t)
        self.goal = g
        gx = float(g[0])
        gy = float(g[1])

        # ---------------- lidar --------------------------------------------
        try:
            raw = np.asarray(getattr(obs, "ranges", None), dtype=np.float64).ravel()
        except Exception:
            raw = np.zeros(0)
        n = raw.size
        try:
            ang = np.asarray(getattr(obs, "range_angles", None), dtype=np.float64).ravel()
        except Exception:
            ang = np.zeros(0)
        if ang.size != n or n == 0 or not np.all(np.isfinite(ang)):
            ang = np.linspace(-np.pi, np.pi, max(1, n), endpoint=False)
        if n == 0:
            raw = np.array([self.range_max])
            ang = np.array([0.0])
            n = 1

        rmax = self.range_max
        valid = np.isfinite(raw) & (raw > 0.12)
        r_eff = np.where(valid, np.minimum(raw, rmax), rmax)
        r_eff = np.clip(np.nan_to_num(r_eff, nan=rmax, posinf=rmax, neginf=rmax), 0.12, rmax)
        hit = valid & (raw < rmax - 0.06)

        # ---------------- mapping -------------------------------------------
        if (self._k % 2) == 0:
            self._update_map(x, y, yaw, r_eff, ang, hit, perr)

        # ---------------- bookkeeping ----------------------------------------
        dg = float(math.hypot(gx - x, gy - y))
        if dg < self._best_dg - 0.30:
            self._best_dg = dg
            self._best_t = t
        if not self._trace or (t - self._trace[-1][0]) >= 0.20:
            self._trace.append((t, x, y))
            while len(self._trace) > 2 and (t - self._trace[0][0]) > 6.0:
                self._trace.pop(0)

        # ---------------- contact --------------------------------------------
        contact = bool(getattr(obs, "contact", False))
        near_idx = int(np.argmin(r_eff))
        near_r = float(r_eff[near_idx])
        near_a = float(ang[near_idx])
        if contact:
            self._contact_run += dt
        else:
            self._contact_run = 0.0
        if contact and near_r < 0.60 and self._contact_run > 0.12:
            self._bump_until = t + 0.35
            self._bump_dir = near_a
            ox = x + math.cos(near_a + yaw) * near_r
            oy = y + math.sin(near_a + yaw) * near_r
            ix, iy, inb = self._cell(ox, oy)
            if inb:
                self.logodds[iy, ix] = min(3.5, float(self.logodds[iy, ix]) + 1.4)

        # ---------------- global plan ----------------------------------------
        self._plan_tick(t, x, y, gx, gy)

        # ---------------- stuck detection -------------------------------------
        recovering = t < self._recover_until
        if not recovering and t < self._bump_until:
            return self._emit(self._bump_cmd(r_eff, ang), dt)

        if not recovering:
            if self._is_stuck(t):
                self._enter_recovery(t, x, y, r_eff, ang, hard=False)
                recovering = True
            elif (t - self._best_t) > 16.0 and dg > 1.0:
                self._enter_recovery(t, x, y, r_eff, ang, hard=True)
                recovering = True

        if recovering:
            return self._emit(self._recover_cmd(r_eff, ang), dt)

        # ---------------- line of sight to the goal ----------------------------
        los = False
        if dg < rmax * 0.95:
            bear = _wrap(math.atan2(gy - y, gx - x) - yaw)
            half = max(0.22, math.atan2(0.40, max(0.5, dg)))
            sector = np.abs(_wrap(ang - bear)) < half
            blocked = bool(np.any(sector & hit & (r_eff < dg - 0.30)))
            los = not blocked

        # ---------------- local planner ----------------------------------------
        cmd = self._choose(t, x, y, yaw, r_eff, ang, hit, gx, gy, dg, los)
        if cmd is None:
            self._enter_recovery(t, x, y, r_eff, ang, hard=False)
            return self._emit(self._recover_cmd(r_eff, ang), dt)
        return self._emit(cmd, dt)

    # ----------------------------------------------------------------------
    def _new_leg(self, t):
        self._legs += 1
        self._leg_t0 = t
        self._trace = []
        self._best_dg = 1.0e9
        self._best_t = t
        self._recover_until = -1.0
        self._escape_until = -1.0
        self._bump_until = -1.0
        self.D = None
        self._plan_active = False
        self._force_plan = True

    def _is_stuck(self, t):
        if (t - self._leg_t0) < 2.0 or len(self._trace) < 4:
            return False
        t0, x0, y0 = self._trace[0]
        if (t - t0) < 3.0:
            return False
        px, py = self._pos
        return float(math.hypot(px - x0, py - y0)) < 0.32

    def _enter_recovery(self, t, x, y, r_eff, ang, hard):
        left = float(np.mean(r_eff[(ang > 0.35) & (ang < 2.1)])) if np.any(
            (ang > 0.35) & (ang < 2.1)) else 1.0
        right = float(np.mean(r_eff[(ang < -0.35) & (ang > -2.1)])) if np.any(
            (ang < -0.35) & (ang > -2.1)) else 1.0
        if abs(left - right) < 0.15:
            sgn = 1.0 if self.rng.random() < 0.5 else -1.0
        else:
            sgn = 1.0 if left > right else -1.0
        self._recover_sign = sgn
        self._recover_until = t + (1.8 if hard else 1.3)
        self._escape_until = self._recover_until + 2.0
        self._add_penalty(x, y, 0.75 if hard else 0.55, 3.0 if hard else 1.8)
        self._trace = []
        self._best_t = t
        self._force_plan = True

    def _recover_cmd(self, r_eff, ang):
        fwd = np.abs(ang) < 0.65
        rear = np.abs(np.abs(ang) - np.pi) < 0.65
        fmin = float(np.min(r_eff[fwd])) if np.any(fwd) else self.range_max
        rmin = float(np.min(r_eff[rear])) if np.any(rear) else self.range_max
        vx = 0.0
        if fmin < 0.85 and rmin > 0.55:
            vx = -0.32 * self.max_lin / 0.8
        elif fmin > 1.2:
            vx = 0.12 * self.max_lin / 0.8
        vy = 0.0
        lat = self._recover_sign * 0.30 * self.max_lin / 0.8
        side = (ang * self._recover_sign > 0.9) & (np.abs(ang) < 2.2)
        if np.any(side) and float(np.min(r_eff[side])) > 0.75:
            vy = lat
        return np.array([vx, vy, self._recover_sign * 0.88 * self.max_ang])

    def _bump_cmd(self, r_eff, ang):
        a = self._bump_dir
        sp = 0.33 * self.max_lin / 0.8
        vx = -sp * math.cos(a)
        vy = -sp * math.sin(a)
        wz = 0.5 * self.max_ang * (1.0 if a < 0.0 else -1.0)
        return np.array([vx, vy, wz])

    # ----------------------------------------------------------------------
    def _choose(self, t, x, y, yaw, r_eff, ang, hit, gx, gy, dg, los):
        nc = self._nc
        nt = self._nt

        # --- clearance against the raw scan, in the robot frame ------------
        sel = hit & (r_eff < 4.2)
        if bool(np.any(sel)):
            rr = r_eff[sel]
            aa = ang[sel]
            ox = (rr * np.cos(aa)).astype(np.float32)
            oy = (rr * np.sin(aa)).astype(np.float32)
            dx = self._tp[:, 0:1] - ox[None, :]
            dy = self._tp[:, 1:2] - oy[None, :]
            d2 = dx * dx + dy * dy
            dmin = np.sqrt(d2.min(axis=1)).reshape(nc, nt).min(axis=1)
        else:
            dmin = np.full(nc, 9.0, dtype=np.float32)
        clear = dmin.astype(np.float64) - self.ROBOT_R

        # --- endpoints in the world ----------------------------------------
        c = math.cos(yaw)
        s = math.sin(yaw)
        ex = x + c * self._epx - s * self._epy
        ey = y + s * self._epx + c * self._epy

        # --- progress -------------------------------------------------------
        use_euclid = True
        prog = None
        psi = math.atan2(gy - y, gx - x)
        if not (los and dg < 3.0):
            drob = self._value_at(x, y, search=True)
            if self.D is not None and drob < 0.5 * _BIG:
                ix = np.floor((ex - self.gx0) / self.res).astype(np.int64)
                iy = np.floor((ey - self.gy0) / self.res).astype(np.int64)
                inb = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
                dend = np.full(nc, _BIG, dtype=np.float64)
                if np.any(inb):
                    dend[inb] = self.D[iy[inb], ix[inb]].astype(np.float64)
                bad = dend >= 0.5 * _BIG
                prog = np.where(bad, -4.0, drob - dend)
                use_euclid = False
                wp = self._descend(x, y)
                if wp is not None:
                    dxw = wp[0] - x
                    dyw = wp[1] - y
                    if (dxw * dxw + dyw * dyw) > 0.04:
                        psi = math.atan2(dyw, dxw)
        if use_euclid:
            prog = dg - np.hypot(ex - gx, ey - gy)

        denom = max(0.2, self.max_lin * self.HORIZON)
        prog_n = np.clip(prog / denom, -1.5, 1.5)

        # --- weights (relaxed while escaping a trap) -------------------------
        w_prog = self.W_PROG
        w_clear = self.W_CLEAR
        if t < self._escape_until:
            w_prog *= 0.5
            w_clear *= 2.2

        head = np.cos(_wrap(psi - (yaw + self._eth)))
        clear_n = np.clip(clear / self.SOFT_CLEAR, 0.0, 1.0)
        prox = np.clip((0.20 - clear) / 0.20, 0.0, 1.0)
        speed_n = self._vx / self.max_lin
        vy_n = np.abs(self._vy) / self.max_lin
        smooth = (np.abs(self._wz - self._cmd[2]) / (2.0 * self.max_ang)
                  + 0.5 * np.abs(self._vx - self._cmd[0]) / self.max_lin)

        score = (w_prog * prog_n
                 + w_clear * clear_n
                 + self.W_SPEED * speed_n
                 + self.W_HEAD * head
                 - self.W_PROX * prox
                 - self.W_VY * vy_n
                 - self.W_SMOOTH * smooth)

        feasible = clear > self.HARD_CLEAR
        score = np.where(feasible, score, score - 100.0)
        k = int(np.argmax(score))
        if score[k] < -50.0:
            return None

        vx = float(self._vx[k])
        vy = float(self._vy[k])
        wz = float(self._wz[k])

        # gentle slow-down when hugging something
        cl = float(clear[k])
        if cl < 0.30:
            f = 0.45 + 0.55 * max(0.0, cl) / 0.30
            vx *= f
            vy *= f
        return np.array([vx, vy, wz])

    # ----------------------------------------------------------------------
    def _emit(self, cmd, dt):
        cmd = np.asarray(cmd, dtype=np.float64).ravel()
        if cmd.size != 3 or not np.all(np.isfinite(cmd)):
            cmd = np.zeros(3)
        dv = 3.2 * dt
        dw = 9.0 * dt
        out = self._cmd.copy()
        out[0] += float(np.clip(cmd[0] - out[0], -dv, dv))
        out[1] += float(np.clip(cmd[1] - out[1], -dv, dv))
        out[2] += float(np.clip(cmd[2] - out[2], -dw, dw))
        out[0] = float(np.clip(out[0], -self.max_lin, self.max_lin))
        out[1] = float(np.clip(out[1], -self.max_lin, self.max_lin))
        out[2] = float(np.clip(out[2], -self.max_ang, self.max_ang))
        self._cmd = out
        return Command(float(out[0]), float(out[1]), float(out[2]))
