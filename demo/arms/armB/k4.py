# =============================================================================
# Navigation controller for a Unitree Go2 shuttling between two fixed points
# in an unmapped indoor space.
#
# WHAT IS TAKEN FROM THE PAPER
#   Stentz, "Optimal and Efficient Path Planning for Partially-Known
#   Environments", ICRA 1994.
#
#   * Sec. 2.1 (state space / backpointers): the world is a grid of states
#     connected to their 8 neighbours by arcs whose cost ranges over a
#     continuum (cheap in known free space, graded up near obstacles,
#     prohibitively large inside them, exactly the EMPTY/OBSTACLE spread of
#     footnote 1). Navigation = hold a cost-to-goal field h(.) over that grid
#     and follow the backpointers (steepest descent on h) instead of carrying
#     an explicit path around.
#   * Sec. 2.2 MODIFY-COST: the lidar only ever edits arc costs *locally*
#     (log-odds occupancy near the robot); the field is then *repaired*
#     incrementally -- a few relaxation sweeps per 50 Hz control step -- never
#     recomputed from scratch. That is the whole efficiency argument of
#     Sec. 4 / Table 1, and here it is what makes global planning affordable
#     inside a 20 ms budget.
#   * Sec. 2.3 RAISE / LOWER duality: the sweep is a plain Bellman update
#     h(X) = min_Y (h(Y) + c(X,Y)), which propagates cost *increases* (new
#     wall found, dead end sealed -- the "potential well" of Figs. 2-3) and
#     cost *decreases* (an obstacle retracted by better evidence, a shortcut
#     discovered) with the same machinery.
#   * Sec. 3 Property 2 + Fig. 5 ("optimistic map"): unmapped cells are
#     entered as nearly-free and h is *seeded with the straight-line lower
#     bound* g_hat. Before convergence the field is therefore a lower bound
#     and the robot simply drives at the goal; sweeps only raise it toward the
#     fixed point, so what the robot follows is optimal with respect to
#     everything it has actually sensed.
#   * Sec. 5.2 ("using a strategy of raising costs for previously visited
#     states, D* can be extended to support exploration"), plus Pirzadeh's
#     penalise-revisits idea cited in Sec. 1: a decaying visit cost stamped
#     when progress stalls. This is what lifts the robot out of a sealed well
#     when the geometric map alone has not yet proved the well is sealed.
#   * Figs. 4/5/6 (complete vs. optimistic vs. coarse map): the task drives
#     the *same* building several times, so one h field is kept per endpoint
#     and *both* are kept repaired against the shared map. Trip 1 runs on the
#     optimistic map (Fig. 5 behaviour, possible dead ends); every later trip
#     runs on the learned map and approaches omniscient-optimal (Fig. 4).
#
#   Not carried across: D*'s OPEN list / k_old bookkeeping. On a small dense
#   grid a vectorised full-field sweep is cheaper per unit of wall clock than
#   a Python priority queue, and it inherits the same RAISE/LOWER semantics.
#   The paper's point-robot-with-contact-sensor is replaced by a lidar plus a
#   velocity-space local layer, because the frozen gait only accepts a Twist.
# =============================================================================

import math
import time
from collections import deque

import numpy as np

from core.observation import Command


class Controller:

    # ---------------------------------------------------------------- reset --
    def reset(self, scenario, rng):
        self.rng = rng

        # ---- scenario limits -------------------------------------------------
        self.range_max = float(getattr(scenario, "range_max", 6.0) or 6.0)
        self.max_lin = float(getattr(scenario, "max_lin", 0.8) or 0.8)
        self.max_ang = float(getattr(scenario, "max_ang", 1.0) or 1.0)
        self.time_limit = float(getattr(scenario, "time_limit", 1.0e9) or 1.0e9)

        sx, sy = self._xy(getattr(scenario, "start", (0.0, 0.0)), 0.0, 0.0)
        gx, gy = self._xy(getattr(scenario, "goal", (1.0, 0.0)), sx + 1.0, sy)

        # ---- grid ------------------------------------------------------------
        span = math.hypot(gx - sx, gy - sy)
        margin = min(24.0, max(9.0, 0.7 * span + 7.0))
        xmin, xmax = min(sx, gx) - margin, max(sx, gx) + margin
        ymin, ymax = min(sy, gy) - margin, max(sy, gy) + margin

        res = 0.20
        while ((xmax - xmin) / res + 4.0) * ((ymax - ymin) / res + 4.0) > 220000.0:
            res *= 1.15
        self.res = res
        self.inv_res = 1.0 / res
        self.ox, self.oy = xmin, ymin
        self.W = int((xmax - xmin) * self.inv_res) + 3
        self.H = int((ymax - ymin) * self.inv_res) + 3

        self.BIG = np.float32(1.0e6)
        self.OCC = 4000.0          # arc cost per metre inside an obstacle
        self.C_FREE = 1.0
        self.C_UNKNOWN = 1.18      # optimistic, but known-free is preferred

        n = self.H * self.W
        self.logodds = np.zeros((self.H, self.W), dtype=np.float32)
        self.seen = np.zeros((self.H, self.W), dtype=bool)
        self.visit = np.zeros((self.H, self.W), dtype=np.float32)
        self.cost = np.full((self.H, self.W), self.C_UNKNOWN, dtype=np.float32)
        self._sc = self.cost * np.float32(res)
        self._dc = self.cost * np.float32(res * math.sqrt(2.0))
        self._pad = np.empty((self.H + 2, self.W + 2), dtype=np.float32)
        self._ncells = n

        ii, jj = np.indices((self.H, self.W))
        self._I = ii.astype(np.float32)
        self._J = jj.astype(np.float32)

        # ---- cost-to-goal fields, one per endpoint --------------------------
        self.fields = {}

        # ---- lidar ray sampling --------------------------------------------
        self._sd = np.arange(0.12, self.range_max + 1e-9, max(0.10, 0.65 * res))
        self._phis = np.linspace(-math.pi, math.pi, 25, endpoint=False)

        # ---- episode state --------------------------------------------------
        self.step_i = 0
        self.last_t = None
        self.prev_cmd = (0.0, 0.0, 0.0)
        self.goal_key = None
        self.trips = 0
        self.cost_dirty = True
        self.hist = deque()
        self.speed_ema = 0.0
        self.h_best = float("inf")
        self.h_best_t = 0.0
        self.escape_until = -1.0
        self.escape_sign = 1.0
        self.escape_phase = 0.0
        self.escape_count = 0
        self.contact_run = 0.0
        self.last_dir = 0.0

        # seed both shuttle endpoints so the return trip starts warm
        self._ensure_field(gx, gy, 0.0, budget=0.45)
        self._ensure_field(sx, sy, 0.0, budget=0.20)

    # ----------------------------------------------------------------- step --
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            pvx, pvy, pwz = self.prev_cmd if hasattr(self, "prev_cmd") else (0.0, 0.0, 0.0)
            cmd = (0.6 * pvx, 0.6 * pvy, 0.6 * pwz)
            self.prev_cmd = cmd
            return Command(cmd[0], cmd[1], cmd[2])

    # =================================================================== core
    def _step(self, obs):
        t0 = time.perf_counter()
        self.step_i += 1

        t = float(getattr(obs, "t", 0.0) or 0.0)
        if self.last_t is None:
            self.last_t = t - 0.02
        dt = min(0.2, max(0.002, t - self.last_t))
        self.last_t = t

        pose = getattr(obs, "pose_estimate", (0.0, 0.0, 0.0))
        x = float(pose[0]); y = float(pose[1]); yaw = float(pose[2])
        perr = abs(float(getattr(obs, "pose_error_estimate", 0.0) or 0.0))
        contact = bool(getattr(obs, "contact", False))
        gx, gy = self._xy(getattr(obs, "goal", (x, y)), x, y)

        # --- goal switch = one more one-way trip started ----------------------
        key, field = self._ensure_field(gx, gy, t, budget=0.012)
        if key != self.goal_key:
            if self.goal_key is not None:
                self.trips += 1
            self.goal_key = key
            self.h_best = float("inf")
            self.h_best_t = t
            self.hist.clear()

        # --- MODIFY-COST: fold the lidar into the arc cost function ----------
        self._integrate_scan(obs, x, y, yaw, perr)

        # --- rebuild traversal costs (cheap, a few times per second) ---------
        if self.step_i % 3 == 0 or self.cost_dirty:
            self._build_cost()
            self.cost_dirty = False
        if self.step_i % 25 == 0 and self.visit.any():
            self.visit *= 0.97
            self.cost_dirty = True

        # --- incremental repair of the cost-to-goal fields -------------------
        self._repair(field, t0, 0.0045)
        if self.step_i % 4 == 0:
            for k, f in self.fields.items():
                if k != key:
                    self._relax(f, 2)

        # --- progress / stall bookkeeping ------------------------------------
        self.hist.append((t, x, y))
        while self.hist and t - self.hist[0][0] > 2.6:
            self.hist.popleft()

        ri, rj = self._cell(x, y)
        h_here = float(field["h"][ri, rj])
        if h_here < self.h_best - 0.25:
            self.h_best = h_here
            self.h_best_t = t

        stalled = False
        if len(self.hist) > 40 and t > self.escape_until + 1.0:
            t0h, x0h, y0h = self.hist[0]
            if (t - t0h) > 2.2 and math.hypot(x - x0h, y - y0h) < 0.24 \
                    and self.speed_ema > 0.14:
                stalled = True

        self.contact_run = self.contact_run + dt if contact else 0.0
        wedged = contact and self.contact_run > 0.35 and self.speed_ema > 0.1 \
            and len(self.hist) > 20 and \
            math.hypot(x - self.hist[0][1], y - self.hist[0][2]) < 0.18

        # Sec. 5.2: raise the cost of states we keep sitting in.
        if stalled or wedged:
            self._stamp_visit(x, y, 1.4)
            self._stamp_obstacle(x, y, self.last_dir, 0.55)
            self.cost_dirty = True
            self.escape_count += 1
            self.escape_sign = self._open_side(obs)
            self.escape_until = t + min(1.6, 0.85 + 0.15 * self.escape_count)
            self.escape_phase = t + 0.35
            self.hist.clear()

        # long-horizon well detection (Figs. 2-3): h refuses to come down
        if t - self.h_best_t > 13.0:
            self._stamp_visit(x, y, 1.0)
            self.cost_dirty = True
            self.h_best = h_here
            self.h_best_t = t

        # --- escape manoeuvre -------------------------------------------------
        if t < self.escape_until:
            if t < self.escape_phase:
                cmd = (-0.42, 0.0, 0.55 * self.escape_sign * self.max_ang)
            else:
                cmd = (0.10, 0.0, self.escape_sign * self.max_ang)
            return self._emit(cmd, 0.0)

        # --- follow the backpointers -----------------------------------------
        tx, ty = self._lookahead(field, ri, rj, x, y, gx, gy)
        theta_d = self._wrap(math.atan2(ty - y, tx - x) - yaw)

        # --- local velocity-space layer --------------------------------------
        cmd, chosen = self._local(obs, theta_d, gx - x, gy - y, perr, contact)
        return self._emit(cmd, chosen)

    # ============================================================ perception
    def _integrate_scan(self, obs, x, y, yaw, perr):
        r = np.asarray(getattr(obs, "ranges", ()), dtype=np.float64).ravel()
        a = np.asarray(getattr(obs, "range_angles", ()), dtype=np.float64).ravel()
        n = min(r.size, a.size)
        if n == 0:
            return
        r = r[:n]; a = a[:n]
        trust = 1.0 / (1.0 + 0.7 * perr)
        g_free = np.float32(0.45 * trust)
        g_hit = np.float32(1.25 * trust)

        fin = np.isfinite(r) & (r > 0.10)
        rc = np.where(fin, np.clip(r, 0.0, self.range_max), self.range_max + 1.0)
        ca = np.cos(a + yaw); sa = np.sin(a + yaw)
        H, W = self.H, self.W

        # free space along each ray, stopping short of the return
        lim = np.minimum(rc - 0.22, self.range_max)
        sd = self._sd
        valid = sd[None, :] < lim[:, None]
        if valid.any():
            X = x + sd[None, :] * ca[:, None]
            Y = y + sd[None, :] * sa[:, None]
            jj = ((X - self.ox) * self.inv_res).astype(np.int32)
            ii = ((Y - self.oy) * self.inv_res).astype(np.int32)
            m = valid & (ii >= 0) & (ii < H) & (jj >= 0) & (jj < W)
            if m.any():
                flat = ii[m].astype(np.int64) * W + jj[m]
                touched = np.bincount(flat, minlength=self._ncells).reshape(H, W) > 0
                np.subtract(self.logodds, g_free, out=self.logodds, where=touched)
                self.seen |= touched

        # returns
        hm = fin & (r < self.range_max - 0.08)
        if hm.any():
            hx = x + r[hm] * ca[hm]
            hy = y + r[hm] * sa[hm]
            jh = ((hx - self.ox) * self.inv_res).astype(np.int32)
            ih = ((hy - self.oy) * self.inv_res).astype(np.int32)
            k = (ih >= 0) & (ih < H) & (jh >= 0) & (jh < W)
            if k.any():
                flat = ih[k].astype(np.int64) * W + jh[k]
                hits = np.bincount(flat, minlength=self._ncells).reshape(H, W) > 0
                np.add(self.logodds, g_hit, out=self.logodds, where=hits)
                self.seen |= hits

        np.clip(self.logodds, -2.5, 6.0, out=self.logodds)
        self.cost_dirty = True

    def _build_cost(self):
        occ = self.logodds > 0.8
        hard = self._dilate(occ)            # robot body radius
        soft1 = self._dilate(hard)
        soft2 = self._dilate(soft1)

        c = np.where(self.seen, np.float32(self.C_FREE),
                     np.float32(self.C_UNKNOWN)).astype(np.float32)
        c += np.where(soft2, np.float32(0.55), np.float32(0.0))
        c += np.where(soft1, np.float32(1.25), np.float32(0.0))
        c += self.visit
        c[hard] = np.float32(self.OCC)

        # never let inflation seal off an endpoint
        for f in self.fields.values():
            i0 = max(0, f["gi"] - 2); i1 = min(self.H, f["gi"] + 3)
            j0 = max(0, f["gj"] - 2); j1 = min(self.W, f["gj"] + 3)
            blk = c[i0:i1, j0:j1]
            np.minimum(blk, np.float32(1.6), out=blk)

        self.cost = c
        self._sc = c * np.float32(self.res)
        self._dc = c * np.float32(self.res * math.sqrt(2.0))

    def _dilate(self, m):
        P = np.zeros((self.H + 2, self.W + 2), dtype=bool)
        P[1:-1, 1:-1] = m
        return (P[:-2, :-2] | P[:-2, 1:-1] | P[:-2, 2:] |
                P[1:-1, :-2] | P[1:-1, 1:-1] | P[1:-1, 2:] |
                P[2:, :-2] | P[2:, 1:-1] | P[2:, 2:])

    # ================================================================ planner
    def _ensure_field(self, gx, gy, t, budget=0.01):
        key = (round(gx * 4.0) / 4.0, round(gy * 4.0) / 4.0)
        f = self.fields.get(key)
        if f is None:
            gi, gj = self._cell(gx, gy)
            gi = int(min(max(gi, 1), self.H - 2))
            gj = int(min(max(gj, 1), self.W - 2))
            # Fig. 5 / Property 2: seed with the straight-line lower bound
            h = np.hypot((self._I - gi) * self.res,
                         (self._J - gj) * self.res).astype(np.float32)
            h[gi, gj] = 0.0
            f = {"h": h, "gi": gi, "gj": gj, "x": gx, "y": gy, "used": t}
            self.fields[key] = f
            self._repair(f, time.perf_counter(), budget, max_iters=900)
            if len(self.fields) > 4:
                worst = None
                for k2, f2 in self.fields.items():
                    if k2 == key:
                        continue
                    if worst is None or f2["used"] < self.fields[worst]["used"]:
                        worst = k2
                if worst is not None:
                    del self.fields[worst]
        f["used"] = t
        return key, f

    def _repair(self, f, t0, budget, max_iters=240):
        it = 0
        while it < max_iters:
            self._relax(f, 4)
            it += 4
            if time.perf_counter() - t0 > budget:
                break

    def _relax(self, f, iters):
        """Bellman sweep h(X) = min_Y (h(Y)+c(X,Y)); raises and lowers alike."""
        h = f["h"]
        gi = f["gi"]; gj = f["gj"]
        P = self._pad
        sc = self._sc; dc = self._dc
        BIG = self.BIG
        for _ in range(iters):
            P[:] = BIG
            P[1:-1, 1:-1] = h
            ms = np.minimum(np.minimum(P[:-2, 1:-1], P[2:, 1:-1]),
                            np.minimum(P[1:-1, :-2], P[1:-1, 2:]))
            md = np.minimum(np.minimum(P[:-2, :-2], P[:-2, 2:]),
                            np.minimum(P[2:, :-2], P[2:, 2:]))
            ms += sc
            md += dc
            np.minimum(ms, md, out=ms)
            np.minimum(ms, BIG, out=ms)
            ms[gi, gj] = 0.0
            h[:] = ms

    def _lookahead(self, f, ri, rj, x, y, gx, gy):
        """Trace backpointers (steepest descent on h) to a carrot point."""
        dgoal = math.hypot(gx - x, gy - y)
        if dgoal < 1.3:
            return gx, gy
        h = f["h"]
        gi = f["gi"]; gj = f["gj"]
        H, W = self.H, self.W
        i, j = ri, rj
        px, py = x, y
        acc = 0.0
        look = 1.05
        for _ in range(36):
            if i == gi and j == gj:
                return f["x"], f["y"]
            bh = float(h[i, j])
            bi = bj = -1
            for di in (-1, 0, 1):
                ii = i + di
                if ii < 0 or ii >= H:
                    continue
                row = h[ii]
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    jj = j + dj
                    if jj < 0 or jj >= W:
                        continue
                    v = float(row[jj])
                    if v < bh - 1e-6:
                        bh = v; bi = ii; bj = jj
            if bi < 0:
                break
            i, j = bi, bj
            cx = self.ox + (j + 0.5) * self.res
            cy = self.oy + (i + 0.5) * self.res
            acc += math.hypot(cx - px, cy - py)
            px, py = cx, cy
            if acc >= look:
                return cx, cy
        if acc > 0.25:
            return px, py
        return gx, gy

    # ============================================================ local layer
    def _local(self, obs, theta_d, dgx, dgy, perr, contact):
        r = np.asarray(getattr(obs, "ranges", ()), dtype=np.float64).ravel()
        a = np.asarray(getattr(obs, "range_angles", ()), dtype=np.float64).ravel()
        n = min(r.size, a.size)
        rmax = self.range_max
        if n:
            r = r[:n]; a = a[:n]
            ok = np.isfinite(r) & (r > 0.12)
            r_ok = r[ok]; a_ok = a[ok]
        else:
            r_ok = np.empty(0); a_ok = np.empty(0)

        phis = np.concatenate((self._phis, np.array([theta_d])))
        Rsafe = 0.32 + (0.05 if contact else 0.0)

        if r_ok.size:
            dif = a_ok[None, :] - phis[:, None]
            lon = r_ok[None, :] * np.cos(dif)
            lat = r_ok[None, :] * np.sin(dif)
            blk = (lon > 0.02) & (np.abs(lat) < Rsafe)
            fd = np.where(blk, lon, rmax).min(axis=1)
        else:
            fd = np.full(phis.size, rmax)

        vcap = self.max_lin
        dgoal = math.hypot(dgx, dgy)
        if dgoal < 0.45:
            vcap = min(vcap, 0.45)
        if contact:
            vcap = min(vcap, 0.5)

        vmax = np.clip((fd - 0.33) * 1.35, 0.0, vcap)
        prog = np.cos(phis - theta_d) * vmax
        score = (prog
                 + 0.18 * np.minimum(fd, 3.0)
                 - 0.30 * np.abs(np.sin(phis)) * vmax
                 - 0.45 * np.maximum(0.0, -np.cos(phis)))
        score = np.where(vmax <= 0.03, -1.0e9, score)

        k = int(np.argmax(score))
        if score[k] <= -1.0e8:
            # boxed in: rotate toward the open side, creep backwards
            s = self._open_side(obs)
            self.speed_ema = 0.9 * self.speed_ema
            return (-0.12, 0.0, s * self.max_ang), 0.0

        phi = float(phis[k])
        v = float(vmax[k])

        # body heading: blend path direction with the direction we will travel
        ax = 0.65 * math.cos(theta_d) + 0.35 * math.cos(phi)
        ay = 0.65 * math.sin(theta_d) + 0.35 * math.sin(phi)
        aim = math.atan2(ay, ax)
        wz = max(-self.max_ang, min(self.max_ang, 2.2 * aim))
        v *= 1.0 - 0.22 * min(1.0, abs(aim) / math.pi)

        self.speed_ema = 0.85 * self.speed_ema + 0.15 * v
        self.last_dir = phi
        return (v * math.cos(phi), v * math.sin(phi), wz), phi

    # ================================================================ helpers
    def _open_side(self, obs):
        try:
            r = np.asarray(getattr(obs, "ranges", ()), dtype=np.float64).ravel()
            a = np.asarray(getattr(obs, "range_angles", ()), dtype=np.float64).ravel()
            n = min(r.size, a.size)
            r = np.where(np.isfinite(r[:n]), np.clip(r[:n], 0.0, self.range_max),
                        self.range_max)
            a = a[:n]
            left = r[(a > 0.25) & (a < 2.0)]
            right = r[(a < -0.25) & (a > -2.0)]
            ls = float(left.sum()) if left.size else 0.0
            rs = float(right.sum()) if right.size else 0.0
            if abs(ls - rs) < 1e-6:
                return 1.0 if self.rng.random() < 0.5 else -1.0
            return 1.0 if ls > rs else -1.0
        except Exception:
            return 1.0

    def _stamp_visit(self, x, y, amount):
        i, j = self._cell(x, y)
        rad = max(1, int(round(0.5 * self.inv_res)))
        i0 = max(0, i - rad); i1 = min(self.H, i + rad + 1)
        j0 = max(0, j - rad); j1 = min(self.W, j + rad + 1)
        blk = self.visit[i0:i1, j0:j1]
        blk += np.float32(amount)
        np.minimum(blk, np.float32(5.0), out=blk)

    def _stamp_obstacle(self, x, y, ang_body, dist):
        # we pushed into something the map does not show: enter it as an arc
        # cost increase (MODIFY-COST with a contact sensor, Sec. 2.3)
        for d in (dist * 0.7, dist, dist * 1.3):
            px = x + d * math.cos(ang_body + 0.0)
            py = y + d * math.sin(ang_body + 0.0)
            i, j = self._cell(px, py)
            if 0 <= i < self.H and 0 <= j < self.W:
                self.logodds[i, j] = min(6.0, float(self.logodds[i, j]) + 1.6)
                self.seen[i, j] = True

    def _emit(self, cmd, chosen):
        pvx, pvy, pwz = self.prev_cmd
        dv = 0.13
        dw = 0.30
        vx = pvx + max(-dv, min(dv, cmd[0] - pvx))
        vy = pvy + max(-dv, min(dv, cmd[1] - pvy))
        wz = pwz + max(-dw, min(dw, cmd[2] - pwz))
        vx = max(-self.max_lin, min(self.max_lin, vx))
        vy = max(-self.max_lin, min(self.max_lin, vy))
        wz = max(-self.max_ang, min(self.max_ang, wz))
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            vx = vy = wz = 0.0
        self.prev_cmd = (vx, vy, wz)
        return Command(vx, vy, wz)

    def _cell(self, x, y):
        j = int((x - self.ox) * self.inv_res)
        i = int((y - self.oy) * self.inv_res)
        if i < 0:
            i = 0
        elif i >= self.H:
            i = self.H - 1
        if j < 0:
            j = 0
        elif j >= self.W:
            j = self.W - 1
        return i, j

    @staticmethod
    def _xy(v, dx, dy):
        try:
            return float(v[0]), float(v[1])
        except Exception:
            return float(dx), float(dy)

    @staticmethod
    def _wrap(t):
        return (t + math.pi) % (2.0 * math.pi) - math.pi
