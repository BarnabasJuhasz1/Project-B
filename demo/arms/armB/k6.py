# ---------------------------------------------------------------------------
# Go2 shuttle navigator.
#
# What is taken from Stentz, "Optimal and Efficient Path Planning for
# Partially-Known Environments" (ICRA 1994), and where:
#
#   * §2.1 / §2.2 -- plan BACKWARDS FROM THE GOAL.  The planner maintains a
#     path-cost-to-goal field h(X) over a grid of states and the robot drives
#     by following backpointers (steepest descent on h).  Because h is
#     anchored at G, the robot moving does not invalidate the plan; only
#     sensed arc-cost changes do.
#   * §2.3 / Figure 5, "Path Planning with an Optimistic Map" -- unexplored
#     cells are given a near-free traversal cost, so on the first trip the
#     robot commits to the direct route and only learns about obstructions
#     when the lidar contradicts the optimistic map.
#   * §2.2, the RAISE / LOWER duality -- when the sensor RAISES an arc cost,
#     stale-low h values downstream must be raised and then re-lowered; a
#     downward-only relaxation leaves the well-shaped local minima of
#     Figure 2.  Here that is done with a vectorised Bellman sweep
#     h <- min_nb( h(nb) + c(nb,X) ) which is simultaneously a RAISE pass and
#     a LOWER pass, run a few sweeps per control step so repair is amortised
#     and stays local to the discrepancy -- the efficiency argument of §4.
#   * Footnote 1 -- OBSTACLE arc cost is a large but FINITE number.  A path
#     therefore always exists, so the controller degrades gracefully (squeeze
#     through / prefer a huge detour) instead of reporting "no path".
#   * §5.2 Future Work, and Pirzadeh [9] -- "raising costs for previously
#     visited states" to support exploration: cells where the robot stalls
#     accumulate an extra, slowly decaying cost, so the repaired plan routes
#     the next attempt somewhere else.
#   * §4 / the "partially known" regime -- the MAP, not the plan, is the
#     durable artefact.  The occupancy grid survives every direction change of
#     the shuttle, so trip 2..N replan over an already-known building, and the
#     h field for each of the two endpoints is cached and warm-started
#     (Property 2: previously optimal costs stay optimal where nothing moved).
#
# Deliberately NOT carried across: the OPEN list / k_min priority machinery
# itself.  A 181-beam planar lidar rewrites a large fraction of the local grid
# every 20 ms, so a Python heap of individual state expansions is slower than
# a dense numpy sweep with the identical fixed point.  The paper's *semantics*
# (goal-rooted h, optimism, raise-then-lower, finite obstacle cost, visit
# penalties) are what is kept.
#
# Below the planner sits a lidar-frame reactive layer.  pose_estimate drifts,
# so the grid smears; the reactive layer works only on raw beams in the robot
# frame and is therefore immune to that drift.  The planner picks WHERE, the
# reactive layer picks WHETHER IT IS SAFE.
# ---------------------------------------------------------------------------

import math
import time

import numpy as np

from core.observation import Command


_S2 = 1.4142135623730951

# (dy, dx, arc length multiplier) -- 8-connected state graph.
_DIRS = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _S2), (-1, 1, _S2), (1, -1, _S2), (1, 1, _S2),
)

_BIG = 1.0e6


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class Controller:

    # ------------------------------------------------------------------
    # lifecycle
    # ------------------------------------------------------------------
    def reset(self, scenario, rng):
        self.rng = rng

        self.max_lin = self._num(getattr(scenario, "max_lin", None), 0.8)
        self.max_ang = self._num(getattr(scenario, "max_ang", None), 1.0)
        self.range_max = self._num(getattr(scenario, "range_max", None), 6.0)
        self.time_limit = self._num(getattr(scenario, "time_limit", None), 1e9)

        start = self._xy(getattr(scenario, "start", None), (0.0, 0.0))
        goal0 = self._xy(getattr(scenario, "goal", None), start)

        # ---- grid extent -------------------------------------------------
        # The building is unmapped, so the grid is a generous box around the
        # two shuttle endpoints.  Optimistic (unknown == nearly free) filling
        # means the extra area costs nothing until the lidar says otherwise.
        lox = min(start[0], goal0[0])
        loy = min(start[1], goal0[1])
        hix = max(start[0], goal0[0])
        hiy = max(start[1], goal0[1])
        span = max(hix - lox, hiy - loy, 1.0)
        margin = min(26.0, max(13.0, 0.75 * span))
        lox -= margin
        loy -= margin
        hix += margin
        hiy += margin

        w = hix - lox
        hgt = hiy - loy
        res = 0.25
        nx = ny = 0
        for _ in range(40):
            nx = int(w / res) + 3
            ny = int(hgt / res) + 3
            if nx * ny <= 52000 and nx <= 280 and ny <= 280:
                break
            res *= 1.08
        self.res = float(res)
        self.inv_res = 1.0 / self.res
        self.nx = int(nx)
        self.ny = int(ny)
        self.ncells = self.nx * self.ny
        self.x0 = float(lox)
        self.y0 = float(loy)

        # ---- map ---------------------------------------------------------
        self.lo = np.zeros((self.ny, self.nx), dtype=np.float32)      # log odds
        self.seen = np.zeros((self.ny, self.nx), dtype=bool)
        self.pen = np.zeros((self.ny, self.nx), dtype=np.float32)     # §5.2 visit cost
        self.occ_thr = 1.2
        self.miss = np.float32(0.34)
        self.unknown_cost = 1.22          # optimistic map, faint pull to known free
        # A 1-cell lethal skirt is ~0.25-0.3 m of body clearance.  If the grid
        # had to be coarsened, a 1-cell skirt would seal doorways, so drop it
        # to a (very expensive but finite) soft band instead.
        self.lethal_dilate = 1 if self.res <= 0.33 else 0

        self.C = np.full((self.ny, self.nx), np.float32(self.unknown_cost),
                         dtype=np.float32)
        self.arcs = []
        self._build_arcs()

        # ---- planner -------------------------------------------------------
        self.h = None
        self._nbb = np.full((self.ny, self.nx), np.float32(_BIG), dtype=np.float32)
        self.hcache = {}
        self.goal_xy = None
        self.gi = 0
        self.gj = 0
        self.ri = 0
        self.rj = 0
        self.pending_burst = False
        self.last_cost_k = -1000
        self.vi_per_step = int(np.clip(130000.0 / max(self.ncells, 1), 2, 6))

        self._Lm = np.array([[_S2, 1.0, _S2],
                             [1.0, 0.0, 1.0],
                             [_S2, 1.0, _S2]], dtype=np.float32)

        # ---- reactive layer -------------------------------------------------
        self.cand = np.linspace(-np.pi, np.pi, 49)[:-1].astype(np.float64)
        self.tiebreak = (self.rng.random(self.cand.size) - 0.5) * 0.04
        self.r_safe = 0.42
        self.w_goal = 2.4
        self.w_clear = 0.75
        self.w_prev = 0.45

        step = 0.7 * self.res
        ns = int(self.range_max / step) + 1
        self._ts = ((np.arange(ns, dtype=np.float64) + 0.5) * step)

        self.ang = None
        self._nbeams = -1

        # ---- episode bookkeeping ---------------------------------------------
        self.k = 0
        self.last_yaw = 0.0
        self.prev_phi = 0.0
        self.prev_clear = 2.0
        self.hist = []
        self.rec_until = -1.0
        self.rec_target = 0.0
        self.rec_count = 0
        self.trips = 0
        self.last_t = 0.0

    # ------------------------------------------------------------------
    # helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _num(v, default):
        try:
            f = float(v)
            if math.isfinite(f) and f != 0.0:
                return f
        except Exception:
            pass
        return float(default)

    @staticmethod
    def _xy(v, default):
        try:
            a = np.asarray(v, dtype=float).ravel()
            if a.size >= 2 and math.isfinite(float(a[0])) and math.isfinite(float(a[1])):
                return float(a[0]), float(a[1])
        except Exception:
            pass
        return float(default[0]), float(default[1])

    def _cell(self, x, y):
        i = int(math.floor((x - self.x0) * self.inv_res))
        j = int(math.floor((y - self.y0) * self.inv_res))
        if i < 0:
            i = 0
        elif i >= self.nx:
            i = self.nx - 1
        if j < 0:
            j = 0
        elif j >= self.ny:
            j = self.ny - 1
        return j, i

    @staticmethod
    def _dilate(m):
        out = m.copy()
        out[:-1, :] |= m[1:, :]
        out[1:, :] |= m[:-1, :]
        out[:, :-1] |= m[:, 1:]
        out[:, 1:] |= m[:, :-1]
        out[:-1, :-1] |= m[1:, 1:]
        out[:-1, 1:] |= m[1:, :-1]
        out[1:, :-1] |= m[:-1, 1:]
        out[1:, 1:] |= m[:-1, :-1]
        return out

    # ------------------------------------------------------------------
    # mapping
    # ------------------------------------------------------------------
    def _integrate(self, x, y, yaw, rr, hitmask, perr):
        ca = np.cos(self.ang + yaw)
        sa = np.sin(self.ang + yaw)
        lof = self.lo.ravel()
        sf = self.seen.ravel()

        # --- free-space carving (this is what erases pose-drift smears) ---
        ts = self._ts
        lim = np.maximum(rr - 0.7 * self.res, 0.0)[:, None]
        m = ts[None, :] <= lim
        if m.any():
            px = x + np.outer(ca, ts)
            py = y + np.outer(sa, ts)
            ix = np.floor((px - self.x0) * self.inv_res).astype(np.int32)
            iy = np.floor((py - self.y0) * self.inv_res).astype(np.int32)
            ok = m & (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            if ok.any():
                flat = (iy * self.nx + ix)[ok]
                touched = np.bincount(flat, minlength=self.ncells).astype(bool)
                lof[touched] -= self.miss
                sf[touched] = True

        # --- hits: MODIFY-COST, the discrepancy that seeds a replan ---
        if hitmask.any():
            hr = rr[hitmask]
            hx = x + ca[hitmask] * hr
            hy = y + sa[hitmask] * hr
            ix = np.floor((hx - self.x0) * self.inv_res).astype(np.int32)
            iy = np.floor((hy - self.y0) * self.inv_res).astype(np.int32)
            ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            if ok.any():
                flat = iy[ok] * self.nx + ix[ok]
                touched = np.bincount(flat, minlength=self.ncells).astype(bool)
                w = 0.9 / (1.0 + 0.45 * max(perr, 0.0))
                lof[touched] += np.float32(max(0.3, w))
                sf[touched] = True

        np.clip(self.lo, -3.0, 6.0, out=self.lo)

    def _mark_contact(self, x, y, yaw, rr):
        idx = np.argsort(rr)[:6]
        d = np.maximum(rr[idx], 0.25)
        hx = x + np.cos(self.ang[idx] + yaw) * d
        hy = y + np.sin(self.ang[idx] + yaw) * d
        ix = np.floor((hx - self.x0) * self.inv_res).astype(np.int32)
        iy = np.floor((hy - self.y0) * self.inv_res).astype(np.int32)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        if ok.any():
            self.lo[iy[ok], ix[ok]] += np.float32(2.0)
            self.seen[iy[ok], ix[ok]] = True
            np.clip(self.lo, -3.0, 6.0, out=self.lo)

    # ------------------------------------------------------------------
    # cost field  (the arc cost function c(.) of the paper)
    # ------------------------------------------------------------------
    def _relief(self, C, j, i, rad, occ, val):
        j0 = max(0, j - rad)
        j1 = min(self.ny, j + rad + 1)
        i0 = max(0, i - rad)
        i1 = min(self.nx, i + rad + 1)
        sub = C[j0:j1, i0:i1]
        so = occ[j0:j1, i0:i1]
        m = (~so) & (sub > val)
        sub[m] = np.float32(val)

    def _rebuild_cost(self, x, y):
        self.last_cost_k = self.k
        occ = self.lo > self.occ_thr
        d1 = self._dilate(occ)
        d2 = self._dilate(d1)
        d3 = self._dilate(d2)
        d4 = self._dilate(d3)

        C = np.where(self.seen, 1.0, self.unknown_cost).astype(np.float32)
        C[d4] += 1.5
        C[d3] += 2.5
        C[d2] += 5.0

        # §5.2 exploration term: stall sites get expensive, then forgiven.
        self.pen *= np.float32(0.996)
        C += self.pen

        if self.lethal_dilate:
            C[d1] = np.float32(500.0)
        else:
            C[d1] += np.float32(60.0)
        C[occ] = np.float32(1500.0)      # footnote 1: large but FINITE

        # A pocket around G and under the robot must never be sealed by
        # inflation, or the state we have to reach becomes unreachable.
        self._relief(C, self.gj, self.gi, 2, occ, 2.0)
        rj, ri = self._cell(x, y)
        self._relief(C, rj, ri, 1, occ, 2.0)

        self.C = C
        self._build_arcs()

    def _build_arcs(self):
        K = (self.C * np.float32(0.5 * self.res)).astype(np.float32)
        ny, nx = self.ny, self.nx
        arcs = []
        for dy, dx, L in _DIRS:
            dy0 = max(0, -dy)
            dy1 = ny - max(0, dy)
            dx0 = max(0, -dx)
            dx1 = nx - max(0, dx)
            if dy1 <= dy0 or dx1 <= dx0:
                continue
            dst = (slice(dy0, dy1), slice(dx0, dx1))
            src = (slice(dy0 + dy, dy1 + dy), slice(dx0 + dx, dx1 + dx))
            arc = (np.float32(L) * (K[dst] + K[src])).astype(np.float32)
            arcs.append((dst, src, arc))
        self.arcs = arcs

    # ------------------------------------------------------------------
    # PROCESS-STATE, vectorised:  h <- min_nb( h(nb) + c(nb,X) )
    # A full Bellman assignment (not a min against the old value) so that a
    # sweep RAISES stale-low states as well as LOWERING stale-high ones.
    # ------------------------------------------------------------------
    def _vi(self, iters):
        h = self.h
        nb = self._nbb
        gj, gi = self.gj, self.gi
        for _ in range(iters):
            nb.fill(np.float32(_BIG))
            for dst, src, arc in self.arcs:
                np.minimum(nb[dst], h[src] + arc, out=nb[dst])
            np.minimum(nb, np.float32(_BIG), out=nb)
            nb[gj, gi] = np.float32(0.0)
            h, nb = nb, h
        self.h = h
        self._nbb = nb

    def _vi_burst(self, deadline, max_iters):
        done = 0
        settled = 0
        while done < max_iters:
            self._vi(8)
            done += 8
            if self.h[self.rj, self.ri] < _BIG * 0.5:
                settled += 8
                if settled >= 48:
                    break
            if time.perf_counter() > deadline:
                break

    # ------------------------------------------------------------------
    # follow the backpointers (steepest descent on h) to a carrot point
    # ------------------------------------------------------------------
    def _carrot(self, x, y, lookahead):
        h = self.h
        C = self.C
        res = self.res
        j, i = self.rj, self.ri
        if h[j, i] >= _BIG * 0.5:
            return None
        px, py = x, y
        acc = 0.0
        cur = float(h[j, i])
        for _ in range(30):
            j0 = j - 1 if j > 0 else 0
            j1 = j + 2 if j + 2 <= self.ny else self.ny
            i0 = i - 1 if i > 0 else 0
            i1 = i + 2 if i + 2 <= self.nx else self.nx
            sh = h[j0:j1, i0:i1]
            sc = C[j0:j1, i0:i1]
            lm = self._Lm[j0 - j + 1:j1 - j + 1, i0 - i + 1:i1 - i + 1]
            tot = sh + lm * (np.float32(0.5 * res) * (sc + C[j, i]))
            tot[j - j0, i - i0] = np.float32(_BIG * 10.0)
            kk = int(np.argmin(tot))
            a, b = divmod(kk, tot.shape[1])
            nj = j0 + a
            ni = i0 + b
            nh = float(h[nj, ni])
            if nh >= cur - 1e-6 or nh >= _BIG * 0.5:
                break
            wx = self.x0 + (ni + 0.5) * res
            wy = self.y0 + (nj + 0.5) * res
            acc += math.hypot(wx - px, wy - py)
            px, py = wx, wy
            j, i, cur = nj, ni, nh
            if acc >= lookahead:
                break
        if acc < 1e-6:
            return None
        return px, py

    # ------------------------------------------------------------------
    # reactive layer -- pure robot frame, immune to pose drift
    # ------------------------------------------------------------------
    def _drive(self, rr, theta_d, dgoal, recovering):
        cand = self.cand
        gamma = np.arcsin(np.clip(self.r_safe / np.maximum(rr, self.r_safe), 0.0, 1.0))
        diff = np.abs(_wrap(self.ang[:, None] - cand[None, :]))
        blocked = diff < gamma[:, None]
        fd = np.where(blocked, rr[:, None], 99.0).min(axis=0)
        np.minimum(fd, self.range_max, out=fd)

        d_stop = 0.30 if dgoal < 1.0 else 0.44

        score = self.w_clear * np.clip(fd, 0.0, 3.0)
        score += self.w_goal * np.cos(_wrap(cand - theta_d))
        score += self.w_prev * np.cos(_wrap(cand - self.prev_phi))
        score -= 50.0 * np.maximum(0.0, d_stop - fd)
        score += self.tiebreak

        b = int(np.argmax(score))
        phi = float(cand[b])
        clear = float(fd[b])
        self.prev_phi = phi
        self.prev_clear = clear

        v = self.max_lin * float(np.clip((clear - 0.28) / 1.0, 0.0, 1.0))
        ap = abs(phi)
        if ap > 1.0:
            v *= 0.35
        elif ap > 0.5:
            v *= 0.72
        if recovering:
            v = min(v, 0.32)
            v = max(v, 0.10 if clear > 0.30 else 0.0)
        if dgoal < 1.0:
            v = min(v, 0.28 + 0.5 * dgoal)

        vx = v * math.cos(phi)
        vy = 0.65 * v * math.sin(phi)
        if vx < -0.32:
            vx = -0.32
        if vy > 0.40:
            vy = 0.40
        elif vy < -0.40:
            vy = -0.40
        wz = float(np.clip(1.9 * phi, -self.max_ang, self.max_ang))

        lim = self.max_lin
        vx = float(np.clip(vx, -lim, lim))
        vy = float(np.clip(vy, -lim, lim))
        return Command(vx, vy, wz)

    # ------------------------------------------------------------------
    # goal switching -- the shuttle turns round
    # ------------------------------------------------------------------
    def _switch_goal(self, gx, gy):
        if self.goal_xy is not None and self.h is not None:
            self.hcache[(self.gj, self.gi)] = self.h.copy()
            if len(self.hcache) > 4:
                self.hcache.pop(next(iter(self.hcache)))
        self.goal_xy = (gx, gy)
        self.gj, self.gi = self._cell(gx, gy)
        cached = self.hcache.get((self.gj, self.gi))
        if cached is not None:
            # Property 2 / §4: everything the previous traverse proved about
            # this goal is still valid wherever the map did not change; the
            # sweeps below only have to repair the discrepancies.
            self.h = cached.copy()
        else:
            self.h = np.full((self.ny, self.nx), np.float32(_BIG), dtype=np.float32)
        self.h[self.gj, self.gi] = np.float32(0.0)
        self.pending_burst = True
        self.hist = []
        self.rec_until = -1.0
        self.rec_count = 0
        self.trips += 1

    # ------------------------------------------------------------------
    # control step
    # ------------------------------------------------------------------
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            return Command(0.0, 0.0, 0.35)

    def _step(self, obs):
        self.k += 1

        try:
            t = float(obs.t)
            if not math.isfinite(t):
                t = self.last_t + 0.02
        except Exception:
            t = self.last_t + 0.02
        if t < self.last_t:
            t = self.last_t + 0.02
        self.last_t = t

        pose = np.asarray(obs.pose_estimate, dtype=float).ravel()
        x = float(pose[0]) if pose.size > 0 and math.isfinite(pose[0]) else 0.0
        y = float(pose[1]) if pose.size > 1 and math.isfinite(pose[1]) else 0.0
        yaw = float(pose[2]) if pose.size > 2 and math.isfinite(pose[2]) else self.last_yaw
        self.last_yaw = yaw

        gx, gy = self._xy(getattr(obs, "goal", None), (x, y))
        try:
            perr = float(obs.pose_error_estimate)
            if not math.isfinite(perr):
                perr = 0.0
        except Exception:
            perr = 0.0
        contact = bool(getattr(obs, "contact", False))

        # ---- lidar ------------------------------------------------------
        r = np.asarray(obs.ranges, dtype=np.float64).ravel()
        a = np.asarray(obs.range_angles, dtype=np.float64).ravel()
        n = min(r.size, a.size)
        r = r[:n]
        a = a[:n]
        if self.ang is None or self._nbeams != n:
            self.ang = a.copy()
            self._nbeams = n
        else:
            self.ang = a
        good = np.isfinite(r) & (r > 0.03)
        rr = np.where(good, np.minimum(r, self.range_max), self.range_max)
        rr = np.clip(rr, 0.05, self.range_max)
        hitmask = good & (r < self.range_max * 0.985)

        # ---- goal switch -------------------------------------------------
        if (self.goal_xy is None
                or (gx - self.goal_xy[0]) ** 2 + (gy - self.goal_xy[1]) ** 2 > 0.25):
            self._switch_goal(gx, gy)

        # ---- map update (MODIFY-COST) ------------------------------------
        self._integrate(x, y, yaw, rr, hitmask, perr)
        if contact:
            self._mark_contact(x, y, yaw, rr)
            if t > self.rec_until:
                self.rec_until = t + 0.55
                self.rec_target = float(self.cand[int(np.argmax(rr))])

        self.rj, self.ri = self._cell(x, y)
        dgoal = math.hypot(gx - x, gy - y)

        # ---- stall detection -> raise costs at visited states (§5.2) -----
        self.hist.append((t, x, y))
        while self.hist and t - self.hist[0][0] > 3.2:
            self.hist.pop(0)
        stalled = False
        if (t > self.rec_until + 0.4 and dgoal > 0.7 and len(self.hist) > 4
                and t - self.hist[0][0] >= 2.6):
            if math.hypot(x - self.hist[0][1], y - self.hist[0][2]) < 0.30:
                stalled = True
        if stalled:
            self.rec_count += 1
            rad = 2 if self.rec_count < 3 else 3
            j0 = max(0, self.rj - rad)
            j1 = min(self.ny, self.rj + rad + 1)
            i0 = max(0, self.ri - rad)
            i1 = min(self.nx, self.ri + rad + 1)
            self.pen[j0:j1, i0:i1] += np.float32(4.0)
            np.clip(self.pen, 0.0, 30.0, out=self.pen)
            self.rec_until = t + 1.1
            self.rec_target = float(self.cand[int(np.argmax(
                np.where(np.abs(_wrap(self.cand - self.prev_phi)) > 0.7,
                         1.0, 0.0) * 0.0 + 1.0) * 0 + np.argmax(
                np.array([0.0]) * 0 + 0))]) if False else float(
                self.cand[int(np.argmax(self._free_dirs(rr)))])
            self.hist = []
            self._rebuild_cost(x, y)
            if self.rec_count % 3 == 0:
                self.pending_burst = True

        # ---- planning (PROCESS-STATE), amortised -------------------------
        if self.pending_burst:
            self.pending_burst = False
            self._rebuild_cost(x, y)
            dl = 0.35 if self.k <= 2 else 0.14
            self._vi_burst(time.perf_counter() + dl, 6 * (self.nx + self.ny))
        else:
            if self.k - self.last_cost_k >= 8:
                self._rebuild_cost(x, y)
            self._vi(self.vi_per_step)

        # ---- pick a heading ------------------------------------------------
        recovering = t < self.rec_until
        if recovering:
            theta_d = self.rec_target
        elif dgoal < 1.2:
            dx = gx - x
            dy = gy - y
            theta_d = math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                                 math.cos(yaw) * dx + math.sin(yaw) * dy)
        else:
            look = float(np.clip(0.65 + 0.42 * min(self.prev_clear, 3.5), 0.8, 1.9))
            c = self._carrot(x, y, look)
            if c is None:
                dx = gx - x
                dy = gy - y
            else:
                dx = c[0] - x
                dy = c[1] - y
            theta_d = math.atan2(-math.sin(yaw) * dx + math.cos(yaw) * dy,
                                 math.cos(yaw) * dx + math.sin(yaw) * dy)

        return self._drive(rr, theta_d, dgoal, recovering)

    # ------------------------------------------------------------------
    def _free_dirs(self, rr):
        gamma = np.arcsin(np.clip(self.r_safe / np.maximum(rr, self.r_safe), 0.0, 1.0))
        diff = np.abs(_wrap(self.ang[:, None] - self.cand[None, :]))
        blocked = diff < gamma[:, None]
        fd = np.where(blocked, rr[:, None], 99.0).min(axis=0)
        return np.minimum(fd, self.range_max)
