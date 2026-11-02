"""
Navigation controller for a Unitree Go2 shuttling between two fixed points in an
unmapped indoor space.

What is taken from Stentz, "Optimal and Efficient Path Planning for
Partially-Known Environments" (ICRA 1994), and where:

  * §1.0 / Fig. 5 "optimistic map": unknown cells are entered into the map as
    cheap-but-slightly-worse-than-known-free, so the robot drives straight at
    the goal through unexplored space and only deflects when the sensor
    contradicts the map.  This is what makes trip #1 fast with no floorplan.
  * §2.1 / §2.2: the plan is a cost-to-goal field h(.) rooted at the GOAL (not
    at the robot) over a grid of states with 8-connected arcs whose costs range
    over a continuum; the robot follows the backpointer sequence {X} by greedy
    descent of h.  Rooting at the goal is what lets the same field be reused
    step after step as the robot moves.
  * §2.3 RAISE / LOWER propagation: when the lidar raises an arc cost (a cell
    becomes an obstacle), states whose h is no longer supported by a neighbour
    (h(X) < min_Y h(Y)+c(Y,X)) are invalidated and the increase is pushed
    outward (RAISE sweep); then reductions are relaxed back in (LOWER sweep).
    Here the OPEN list is replaced by whole-grid vectorised sweeps, which is the
    same fixed point with numpy doing the bookkeeping.
  * §4 efficiency claim: repair is local and budgeted -- a handful of sweeps per
    50 Hz control step instead of a global replan, so the controller never
    stalls.
  * §5.2 "Using a strategy of raising costs for previously visited states, D*
    can be extended to support exploration" (and Pirzadeh's visit-cost
    increments cited in §1.0): places where the robot wedges itself get a
    persistent (slowly decaying) cost "scar", so replanning routes around them.
  * §1.0 [7] Lumelsky: when the map says no path exists, fall back to bug-style
    "head at the goal, slide along the blockage with a committed turn side".

Not carried across: the OPEN-list/k_min machinery itself (a planar-lidar Go2 at
50 Hz is better served by dense sweeps than by a Python priority queue), and the
quad-tree of §5.2.
"""

import math

import numpy as np

from core.observation import Command


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class Controller:
    # ---- map / planner geometry -------------------------------------------
    MAXDIM = 240          # cap on grid side length (cells)
    PAD = 12.0            # metres of map padding around the two endpoints
    RES0 = 0.20           # preferred cell size (m)

    # ---- occupancy log-odds ------------------------------------------------
    LO_HIT = 0.85
    LO_MISS = 0.40
    LO_MIN = -3.0
    LO_MAX = 5.0
    OCC_T = 1.2           # log-odds above this -> obstacle
    FREE_T = -0.6         # log-odds below this -> confirmed free

    # ---- arc costs (paper's c(.), ranging over a continuum) ----------------
    C_FREE = 1.0
    C_UNKNOWN = 1.18      # optimistic map: unknown is traversable, barely worse
    C_NEAR1 = 1.8         # soft inflation ring 1
    C_NEAR2 = 0.8         # soft inflation ring 2
    SCAR_MAX = 6.0

    # ---- body / driving ----------------------------------------------------
    HALF_W = 0.34         # half corridor width used for clearance tests (m)
    R_STOP = 0.48
    LOOKAHEAD = 1.05
    SWEEPS_ACTIVE = 3
    SWEEPS_IDLE = 1
    REBUILD_EVERY = 8

    # =======================================================================
    # episode setup
    # =======================================================================
    def reset(self, scenario, rng):
        self.rng = rng
        self.range_max = float(getattr(scenario, "range_max", 6.0) or 6.0)
        self.max_lin = float(getattr(scenario, "max_lin", 0.8) or 0.8)
        self.max_ang = float(getattr(scenario, "max_ang", 1.0) or 1.0)

        try:
            start = np.asarray(scenario.start, dtype=float).ravel()[:2]
        except Exception:
            start = np.zeros(2)
        try:
            goal = np.asarray(scenario.goal, dtype=float).ravel()[:2]
        except Exception:
            goal = start + 1.0
        if not np.all(np.isfinite(start)):
            start = np.zeros(2)
        if not np.all(np.isfinite(goal)):
            goal = start + 1.0

        # ---- grid extent -------------------------------------------------
        lo = np.minimum(start, goal) - self.PAD
        hi = np.maximum(start, goal) + self.PAD
        res = self.RES0
        for _ in range(12):
            w = int(math.ceil((hi[0] - lo[0]) / res)) + 2
            h = int(math.ceil((hi[1] - lo[1]) / res)) + 2
            if max(w, h) <= self.MAXDIM:
                break
            res *= 1.18
        self.res = float(res)
        self.inv_res = 1.0 / self.res
        self.W = int(max(8, w))
        self.H = int(max(8, h))
        self.x0 = float(lo[0] - self.res)
        self.y0 = float(lo[1] - self.res)

        shape = (self.H, self.W)
        self.logodds = np.zeros(shape, dtype=np.float32)
        self.scar = np.zeros(shape, dtype=np.float32)
        self.lethal_prev = np.zeros(shape, dtype=bool)
        self.cost = np.full(shape, self.C_UNKNOWN, dtype=np.float32)
        self.step_c = (self.cost * self.res).astype(np.float32)
        self.step_d = (self.cost * (self.res * math.sqrt(2.0))).astype(np.float32)
        self._pad = np.empty((self.H + 2, self.W + 2), dtype=np.float32)
        self._bpad = np.zeros((self.H + 2, self.W + 2), dtype=bool)
        e = 1e-4
        s2 = math.sqrt(2.0)
        self._dmat = np.array([[e * s2, e, e * s2],
                               [e, 0.0, e],
                               [e * s2, e, e * s2]], dtype=np.float32)

        # ray-cast sample offsets
        ns = max(2, int(self.range_max / self.res) + 1)
        self._ray_s = np.linspace(0.25, self.range_max, ns).astype(np.float64)

        # ---- planner state ------------------------------------------------
        self.fields = {}
        self.active_key = None
        self.burst = 0
        self.raise_debt = 0
        self.force_rebuild = True

        # ---- behaviour state ----------------------------------------------
        self.n = 0
        self.t0 = None
        self.last_t = None
        self.goal_since = 0.0
        self.prev_dir = 0.0
        self.bug_side = 1.0
        self.bug_mode = False
        self.spiral_phase = 0.0
        self.recover_until = -1.0
        self.recover_spin = 1.0
        self.recover_dir = math.pi
        self.last_des_world = 0.0
        self.hist = []
        self.contact_run = 0

        # ---- seed the two endpoint fields ----------------------------------
        # With an all-unknown (uniform cost) map the fixed point of the sweeps
        # is exactly the octile distance, so seed it analytically for free.
        for p in (goal, start):
            self._new_field(self._key(p[0], p[1]), float(p[0]), float(p[1]),
                            analytic=True)
        self.active_key = self._key(goal[0], goal[1])

    # =======================================================================
    # grid helpers
    # =======================================================================
    def _key(self, x, y):
        return (round(float(x) * 2.0) / 2.0, round(float(y) * 2.0) / 2.0)

    def _cell(self, x, y):
        ix = int((float(x) - self.x0) * self.inv_res)
        iy = int((float(y) - self.y0) * self.inv_res)
        return iy, ix

    def _inside(self, iy, ix, m=1):
        return (m <= iy < self.H - m) and (m <= ix < self.W - m)

    def _center(self, iy, ix):
        return (self.x0 + (ix + 0.5) * self.res,
                self.y0 + (iy + 0.5) * self.res)

    def _dilate(self, m):
        p = self._bpad
        p[:] = False
        p[1:-1, 1:-1] = m
        out = p[1:-1, 1:-1] | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | \
            p[1:-1, 2:] | p[:-2, :-2] | p[:-2, 2:] | p[2:, :-2] | p[2:, 2:]
        return out

    def _box(self, x, y, rad):
        iy, ix = self._cell(x, y)
        k = max(0, int(rad * self.inv_res))
        y0 = max(0, iy - k)
        y1 = min(self.H, iy + k + 1)
        x0 = max(0, ix - k)
        x1 = min(self.W, ix + k + 1)
        if y0 >= y1 or x0 >= x1:
            return None
        return y0, y1, x0, x1

    # =======================================================================
    # cost-to-goal fields (D* h(.) with vectorised RAISE / LOWER sweeps)
    # =======================================================================
    def _nmin(self, h):
        """min over the 8 neighbours Y of ( h(Y) + c(X,Y) ) -- the quantity D*
        compares h(X) against at lines L6/L12/L20."""
        p = self._pad
        p.fill(np.inf)
        p[1:-1, 1:-1] = h
        card = np.minimum(np.minimum(p[:-2, 1:-1], p[2:, 1:-1]),
                          np.minimum(p[1:-1, :-2], p[1:-1, 2:]))
        diag = np.minimum(np.minimum(p[:-2, :-2], p[:-2, 2:]),
                          np.minimum(p[2:, :-2], p[2:, 2:]))
        np.add(card, self.step_c, out=card)
        np.add(diag, self.step_d, out=diag)
        return np.minimum(card, diag, out=card)

    def _new_field(self, key, gx, gy, analytic=False):
        iy, ix = self._cell(gx, gy)
        if not self._inside(iy, ix, 1):
            return None
        h = np.full((self.H, self.W), np.inf, dtype=np.float32)
        if analytic:
            yy = np.abs(np.arange(self.H, dtype=np.float32) - iy)[:, None]
            xx = np.abs(np.arange(self.W, dtype=np.float32) - ix)[None, :]
            hi = np.maximum(yy, xx)
            lo = np.minimum(yy, xx)
            h = ((hi + (math.sqrt(2.0) - 1.0) * lo) *
                 (self.res * self.C_UNKNOWN)).astype(np.float32)
        h[iy, ix] = 0.0
        f = {"h": h, "cell": (iy, ix), "xy": (float(gx), float(gy)),
             "last": self.n, "warm": 0 if analytic else 260}
        if len(self.fields) >= 4:
            worst = min(self.fields.items(), key=lambda kv: kv[1]["last"])[0]
            self.fields.pop(worst, None)
        self.fields[key] = f
        if not analytic:
            self._sweep(f, 2, f["warm"])
            f["warm"] = 0
        return f

    def _sweep(self, f, n_raise, n_lower):
        h = f["h"]
        gy, gx = f["cell"]
        for _ in range(int(n_raise)):
            m = self._nmin(h)
            bad = h < (m - 1e-4)
            bad[gy, gx] = False
            if not bad.any():
                break
            h[bad] = np.inf
        for _ in range(int(n_lower)):
            m = self._nmin(h)
            if not (m < h - 1e-4).any():
                break
            np.minimum(h, m, out=h)
            h[gy, gx] = 0.0
        h[gy, gx] = 0.0

    # =======================================================================
    # mapping
    # =======================================================================
    def _update_map(self, px, py, yaw, r, ang, weight):
        s = self._ray_s
        ca = np.cos(ang + yaw)
        sa = np.sin(ang + yaw)
        xs = px + np.outer(ca, s)
        ys = py + np.outer(sa, s)
        ix = ((xs - self.x0) * self.inv_res).astype(np.int32)
        iy = ((ys - self.y0) * self.inv_res).astype(np.int32)
        ok = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
        free = ok & (s[None, :] < (r[:, None] - 0.6 * self.res))
        if free.any():
            flat = (iy[free].astype(np.int64) * self.W + ix[free].astype(np.int64))
            cnt = np.bincount(flat, minlength=self.H * self.W)
            fm = cnt.reshape(self.H, self.W) > 0
            self.logodds[fm] -= self.LO_MISS * weight
        hit = r < (self.range_max - 0.12)
        if hit.any():
            hx = px + np.cos(ang[hit] + yaw) * r[hit]
            hy = py + np.sin(ang[hit] + yaw) * r[hit]
            hix = ((hx - self.x0) * self.inv_res).astype(np.int32)
            hiy = ((hy - self.y0) * self.inv_res).astype(np.int32)
            good = (hix >= 0) & (hix < self.W) & (hiy >= 0) & (hiy < self.H)
            if good.any():
                flat = (hiy[good].astype(np.int64) * self.W +
                        hix[good].astype(np.int64))
                cnt = np.bincount(flat, minlength=self.H * self.W)
                hm = cnt.reshape(self.H, self.W) > 0
                self.logodds[hm] += self.LO_HIT * weight
        np.clip(self.logodds, self.LO_MIN, self.LO_MAX, out=self.logodds)

    def _rebuild_cost(self, px, py, perr):
        occ = self.logodds > self.OCC_T
        lethal = self._dilate(occ)
        near1 = self._dilate(lethal)
        near2 = self._dilate(near1)

        cost = np.where(self.logodds < self.FREE_T,
                        np.float32(self.C_FREE),
                        np.float32(self.C_UNKNOWN)).astype(np.float32)
        cost += np.where(near1 & ~lethal, np.float32(self.C_NEAR1),
                         np.float32(0.0))
        cost += np.where(near2 & ~near1, np.float32(self.C_NEAR2),
                         np.float32(0.0))
        cost += self.scar

        # never let the planner declare the robot's own cell or a goal lethal
        for (cx, cy, rad) in [(px, py, 0.30 + 0.5 * min(perr, 0.6))] + \
                [(f["xy"][0], f["xy"][1], 0.45) for f in self.fields.values()]:
            b = self._box(cx, cy, rad)
            if b is not None:
                lethal[b[0]:b[1], b[2]:b[3]] = False
        cost[lethal] = np.inf

        changed = bool((lethal ^ self.lethal_prev).any())
        self.lethal_prev = lethal
        self.cost = cost
        self.step_c = (cost * np.float32(self.res))
        self.step_d = (cost * np.float32(self.res * math.sqrt(2.0)))
        return changed

    def _add_scar(self, x, y, rad, amount):
        b = self._box(x, y, rad)
        if b is None:
            return
        sub = self.scar[b[0]:b[1], b[2]:b[3]]
        np.minimum(sub + np.float32(amount), np.float32(self.SCAR_MAX), out=sub)
        self.force_rebuild = True

    # =======================================================================
    # path extraction: follow the backpointers (greedy descent of h)
    # =======================================================================
    def _path_points(self, h, px, py):
        iy, ix = self._cell(px, py)
        iy = int(min(max(iy, 1), self.H - 2))
        ix = int(min(max(ix, 1), self.W - 2))
        if not np.isfinite(h[iy, ix]):
            y0 = max(1, iy - 7)
            y1 = min(self.H - 1, iy + 8)
            x0 = max(1, ix - 7)
            x1 = min(self.W - 1, ix + 8)
            win = h[y0:y1, x0:x1]
            if not np.isfinite(win).any():
                return []
            j = int(np.argmin(win))
            iy = y0 + j // win.shape[1]
            ix = x0 + j % win.shape[1]
        pts = []
        cy, cx = iy, ix
        for _ in range(70):
            if h[cy, cx] <= 0.0:
                break
            win = h[cy - 1:cy + 2, cx - 1:cx + 2] + self._dmat
            j = int(np.argmin(win))
            ny = cy - 1 + j // 3
            nx = cx - 1 + j % 3
            if (ny == cy and nx == cx) or not np.isfinite(h[ny, nx]):
                break
            cy, cx = ny, nx
            pts.append(self._center(cy, cx))
            if not self._inside(cy, cx, 1):
                break
        return pts

    def _lookahead(self, pts, px, py):
        if not pts:
            return None
        acc = 0.0
        lx, ly = px, py
        for (qx, qy) in pts:
            acc += math.hypot(qx - lx, qy - ly)
            lx, ly = qx, qy
            if acc >= self.LOOKAHEAD:
                return (qx, qy)
        return pts[-1]

    # =======================================================================
    # reactive direction choice (runs on raw lidar, below the planner)
    # =======================================================================
    def _choose_dir(self, des_robot, r, ang, half_w, bug):
        cand = self._cand
        near = r < 3.2
        if near.any():
            rn = r[near]
            an = ang[near]
            dth = _wrap(an[:, None] - cand[None, :])
            lon = rn[:, None] * np.cos(dth)
            lat = rn[:, None] * np.sin(dth)
            block = (lon > 0.0) & (np.abs(lat) < half_w)
            lonb = np.where(block, lon, np.inf)
            clear = lonb.min(axis=0)
        else:
            clear = np.full(cand.shape, 3.2)
        clear = np.minimum(clear, 3.2)

        head = np.abs(_wrap(cand - des_robot))
        smooth = np.abs(_wrap(cand - self.prev_dir))
        score = 1.00 * head + 0.25 * smooth
        score = score + 0.85 * np.maximum(0.0, 1.9 - clear)
        score = score + np.where(clear < self.R_STOP, 40.0, 0.0)
        if bug:
            # Lumelsky-style committed turn side while sliding along a blockage
            score = score + 0.35 * np.maximum(
                0.0, -self.bug_side * _wrap(cand - des_robot))
        j = int(np.argmin(score))
        return float(cand[j]), float(clear[j]), bool(score[j] >= 30.0)

    def _corridor_clear(self, d, r, ang, half_w):
        dth = _wrap(ang - d)
        lon = r * np.cos(dth)
        lat = r * np.sin(dth)
        sel = (lon > 0.0) & (np.abs(lat) < half_w)
        if not sel.any():
            return self.range_max
        return float(lon[sel].min())

    # =======================================================================
    # step
    # =======================================================================
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            return Command(0.0, 0.0, 0.35)

    def _step(self, obs):
        self.n += 1
        try:
            t = float(obs.t)
        except Exception:
            t = (self.last_t or 0.0) + 0.02
        if not np.isfinite(t):
            t = (self.last_t or 0.0) + 0.02
        dt = 0.02 if self.last_t is None else max(1e-3, min(0.2, t - self.last_t))
        self.last_t = t
        if self.t0 is None:
            self.t0 = t
            self.goal_since = t

        pose = np.asarray(obs.pose_estimate, dtype=float).ravel()
        px, py, yaw = float(pose[0]), float(pose[1]), float(pose[2])
        if not (np.isfinite(px) and np.isfinite(py) and np.isfinite(yaw)):
            return Command(0.0, 0.0, 0.3)

        g = np.asarray(obs.goal, dtype=float).ravel()[:2]
        gx, gy = float(g[0]), float(g[1])
        if not (np.isfinite(gx) and np.isfinite(gy)):
            gx, gy = px, py

        try:
            perr = float(obs.pose_error_estimate)
        except Exception:
            perr = 0.0
        if not np.isfinite(perr):
            perr = 0.0
        perr = min(max(perr, 0.0), 2.0)
        contact = bool(getattr(obs, "contact", False))

        rr = np.asarray(obs.ranges, dtype=float).ravel()
        aa = np.asarray(obs.range_angles, dtype=float).ravel()
        m = int(min(rr.size, aa.size))
        if m > 0:
            rr = rr[:m]
            aa = aa[:m]
            r = np.where(np.isfinite(rr), rr, self.range_max)
            r = np.clip(r, 0.02, self.range_max)
            ang = np.where(np.isfinite(aa), aa, 0.0)
        else:
            r = np.full(1, self.range_max)
            ang = np.zeros(1)
        if not hasattr(self, "_cand"):
            self._cand = np.linspace(-np.pi, np.pi, 61)[:-1].astype(np.float64)

        # ---- goal bookkeeping (the harness flips obs.goal on arrival) ------
        key = self._key(gx, gy)
        if key != self.active_key:
            self.active_key = key
            self.goal_since = t
            self.spiral_phase = 0.0
            self.burst = max(self.burst, 40)
            self.bug_mode = False
        f = self.fields.get(key)
        if f is None:
            f = self._new_field(key, gx, gy)
            self.force_rebuild = True
        if f is not None:
            f["last"] = self.n
            f["xy"] = (gx, gy)

        # ---- map update (weight down when the pose belief is shaky) --------
        w = float(min(1.0, max(0.30, 1.15 - 0.9 * perr)))
        self._update_map(px, py, yaw, r, ang, w)

        if contact:
            self.contact_run += 1
            imin = int(np.argmin(r))
            bx = px + math.cos(ang[imin] + yaw) * max(0.35, float(r[imin]))
            by = py + math.sin(ang[imin] + yaw) * max(0.35, float(r[imin]))
            b = self._box(bx, by, 0.12)
            if b is not None:
                self.logodds[b[0]:b[1], b[2]:b[3]] += 1.2
            if self.contact_run == 1:
                self._add_scar(bx, by, 0.35, 1.0)
        else:
            self.contact_run = 0

        # ---- cost map + budgeted D* repair ---------------------------------
        if self.force_rebuild or (self.n % self.REBUILD_EVERY == 0):
            self.force_rebuild = False
            if self._rebuild_cost(px, py, perr):
                self.burst = min(90, self.burst + 45)
                self.raise_debt = 10
        if self.n % 250 == 0:
            self.scar *= np.float32(0.90)
            self.force_rebuild = True

        credits = min(self.burst, 6)
        self.burst -= credits
        nr = 2 if self.raise_debt > 0 else 0
        if self.raise_debt > 0:
            self.raise_debt -= 1
        for k, fl in self.fields.items():
            if k == self.active_key:
                self._sweep(fl, nr, self.SWEEPS_ACTIVE + credits)
            else:
                self._sweep(fl, 1 if nr else 0, self.SWEEPS_IDLE + credits // 2)

        # ---- stuck bookkeeping ---------------------------------------------
        self.hist.append((t, px, py))
        while self.hist and (t - self.hist[0][0]) > 3.0:
            self.hist.pop(0)
        stuck = False
        if (self.recover_until < t and len(self.hist) > 40 and
                (t - self.hist[0][0]) > 2.6):
            disp = math.hypot(px - self.hist[0][1], py - self.hist[0][2])
            if disp < 0.28:
                stuck = True

        dgoal = math.hypot(gx - px, gy - py)
        rmin = float(r.min())

        if stuck and dgoal > 0.7:
            # paper §5.2: raise the cost of the state we keep failing in
            sx = px + math.cos(self.last_des_world) * 0.55
            sy = py + math.sin(self.last_des_world) * 0.55
            self._add_scar(sx, sy, 0.40, 2.2)
            self._add_scar(px, py, 0.22, 0.6)
            self.burst = min(90, self.burst + 60)
            self.raise_debt = 10
            left = float(r[(ang > 0.15) & (ang < 2.0)].mean()) if np.any(
                (ang > 0.15) & (ang < 2.0)) else 1.0
            right = float(r[(ang < -0.15) & (ang > -2.0)].mean()) if np.any(
                (ang < -0.15) & (ang > -2.0)) else 1.0
            self.recover_spin = 1.0 if left >= right else -1.0
            self.bug_side = self.recover_spin
            imin = int(np.argmin(r))
            self.recover_dir = _wrap(float(ang[imin]) + math.pi)
            self.recover_until = t + 1.1
            self.hist = []

        # ---- recovery ------------------------------------------------------
        if self.recover_until > t:
            d = self.recover_dir
            v = 0.32
            vx = v * math.cos(d)
            vy = max(-0.35, min(0.35, v * math.sin(d)))
            wz = 0.85 * self.recover_spin
            self.prev_dir = d
            return self._clamp(vx, vy, wz)

        # ---- emergency: something very close -------------------------------
        if rmin < 0.30:
            imin = int(np.argmin(r))
            d = _wrap(float(ang[imin]) + math.pi)
            vx = 0.30 * math.cos(d)
            vy = max(-0.30, min(0.30, 0.30 * math.sin(d)))
            self.prev_dir = d
            return self._clamp(vx, vy, 0.5 * self.bug_side)

        # ---- desired heading ------------------------------------------------
        target = None
        self.bug_mode = False
        if dgoal < 0.55 and (t - self.goal_since) > 5.0:
            # We believe we are on the goal but the harness has not switched:
            # the pose estimate is off, so sweep a widening circle around it.
            self.spiral_phase += 1.6 * dt
            rad = min(1.4, 0.28 + 0.13 * ((t - self.goal_since) - 5.0))
            target = (gx + rad * math.cos(self.spiral_phase),
                      gy + rad * math.sin(self.spiral_phase))
        elif dgoal < 1.1:
            target = (gx, gy)
        elif f is not None:
            pts = self._path_points(f["h"], px, py)
            target = self._lookahead(pts, px, py)

        if target is None:
            target = (gx, gy)
            self.bug_mode = True

        des_world = math.atan2(target[1] - py, target[0] - px)
        self.last_des_world = des_world
        des_robot = _wrap(des_world - yaw)

        half_w = self.HALF_W + 0.10 * min(perr, 0.8)
        d, clear_d, all_blocked = self._choose_dir(des_robot, r, ang, half_w,
                                                   self.bug_mode)
        if self.bug_mode:
            # keep the committed side until the straight shot opens up
            if self._corridor_clear(des_robot, r, ang, half_w) > 2.5:
                self.bug_side = 1.0 if _wrap(d - des_robot) >= 0 else -1.0

        if all_blocked:
            imin = int(np.argmin(r))
            away = _wrap(float(ang[imin]) + math.pi)
            self.prev_dir = away
            return self._clamp(0.22 * math.cos(away),
                               max(-0.25, min(0.25, 0.22 * math.sin(away))),
                               0.9 * self.bug_side)

        self.prev_dir = 0.7 * self.prev_dir + 0.3 * (
            self.prev_dir + _wrap(d - self.prev_dir))

        # ---- speed -----------------------------------------------------------
        clear = self._corridor_clear(d, r, ang, half_w)
        v = 0.60 * (clear - 0.34)
        v = max(0.12, min(self.max_lin, v))
        if abs(d) > 1.2:
            v *= 0.70
        if dgoal < 0.6:
            v = min(v, 0.45)
        if perr > 0.7:
            v *= 0.85

        vx = v * math.cos(d)
        vy = v * math.sin(d)
        if vy > 0.45:
            vy = 0.45
        elif vy < -0.45:
            vy = -0.45
        wz = 2.0 * d
        return self._clamp(vx, vy, wz)

    def _clamp(self, vx, vy, wz):
        if not np.isfinite(vx):
            vx = 0.0
        if not np.isfinite(vy):
            vy = 0.0
        if not np.isfinite(wz):
            wz = 0.0
        L = self.max_lin
        A = self.max_ang
        vx = max(-L, min(L, float(vx)))
        vy = max(-L, min(L, float(vy)))
        wz = max(-A, min(A, float(wz)))
        return Command(vx, vy, wz)


# ==== EXP C INTERVENTION: NOT MODEL-WRITTEN ================================
# Appended by demo/expC.py to a byte-identical copy of demo/arms/armB/k3.py
# (sha256 0bf3f73571fcd251f252c874525da8fb89318ac66c0c1a2938a7c71e2403f767).
# Everything ABOVE this banner is the model's file, unchanged.
# Everything below is ours and identical in every patched copy except the body
# of _expc_full_replan. This file is NOT an armB sample and never enters an arm
# count. See demo/expC.py and prereg/expC.md.
import math as _expc_math
import time as _expc_time

_EXPC_PROGRESS_M = 0.5
_EXPC_WINDOW_S = 10.0
_EXPC_GOAL_SWITCH_M = 0.5


class _ExpCNoProgress:
    """True when the straight-line distance to the current target has not
    reached a new best, lower by _EXPC_PROGRESS_M, for _EXPC_WINDOW_S sim
    seconds. Restarts on a new target and after every firing."""

    def __init__(self):
        self.goal = None
        self.best = None
        self.t_best = None

    def update(self, t, x, y, gx, gy):
        d = _expc_math.hypot(gx - x, gy - y)
        if (self.goal is None or _expc_math.hypot(
                gx - self.goal[0], gy - self.goal[1]) > _EXPC_GOAL_SWITCH_M):
            self.goal, self.best, self.t_best = (gx, gy), d, t
            return False
        if d <= self.best - _EXPC_PROGRESS_M:
            self.best, self.t_best = d, t
            return False
        if t - self.t_best >= _EXPC_WINDOW_S:
            self.best, self.t_best = d, t
            return True
        return False


_ExpCModelController = Controller


class Controller(_ExpCModelController):
    """The model's controller plus the full-replan fallback. Nothing else."""

    def reset(self, scenario, rng):
        super().reset(scenario, rng)
        self._expc_monitor = _ExpCNoProgress()
        self.expc_fallback_fires = []
        self.expc_errors = []

    def step(self, obs):
        fired = False
        try:
            p, g = obs.pose_estimate, obs.goal
            fired = self._expc_monitor.update(
                float(obs.t), float(p[0]), float(p[1]), float(g[0]), float(g[1]))
        except Exception as exc:
            self.expc_errors.append(repr(exc)[:160])
        if not fired:
            return super().step(obs)
        t0 = _expc_time.perf_counter()
        try:
            self._expc_full_replan()
        except Exception as exc:
            self.expc_errors.append(repr(exc)[:160])
        cmd = super().step(obs)
        self.expc_fallback_fires.append({
            "t": round(float(obs.t), 2),
            "goal": [round(float(obs.goal[0]), 2), round(float(obs.goal[1]), 2)],
            "dist": round(self._expc_monitor.best, 3),
            "step_ms": round((_expc_time.perf_counter() - t0) * 1000.0, 2)})
        return cmd

    def _expc_full_replan(self):
        # Drop the cached route; keep the map. Per-controller (expC.HOOKS).
        self.fields = {}
