# =============================================================================
# Go2 shuttle-navigation controller.
#
# What was taken from the source paper -- A. Stentz, "Optimal and Efficient
# Path Planning for Partially-Known Environments", ICRA 1994 -- and where:
#
#  * Sec. 2.1 / 2.2 (state space, h(G,X), backpointers): the robot plans on a
#    grid of states whose arcs carry continuous costs, and it keeps a
#    cost-to-goal field h(.) grown *outward from the goal*.  Steering is
#    "follow the backpointers": descend h(.) locally.  Because the field is
#    anchored at the goal it has no local minima once converged, and it is
#    valid for *every* cell, which is exactly what a shuttle task wants.
#
#  * Sec. 2.2 / 2.3, RAISE vs LOWER states: cost *reductions* (newly cleared
#    free space, decayed penalties) are propagated by continuing to relax the
#    existing field in place -- min-relaxation can only lower values, so it is
#    pure LOWER propagation.  Cost *increases* (a wall where the optimistic map
#    said "empty") cannot be handled that way, so they mark the field dirty and
#    trigger a background re-grow, while the previous converged field stays
#    drivable in the meantime.  That is D*'s "repair rather than replan, and
#    keep moving" behaviour, done with the compute budget a 50 Hz loop has.
#
#  * Sec. 2.3 / Fig. 2 and Fig. 5 (the "potential well" and the optimistic,
#    all-white map): unknown cells are given a near-free cost, so the robot
#    drives straight at the goal through unexplored space and only detours when
#    the lidar contradicts the map.  The first pass may wander into a dead end
#    (Fig. 5's path is ~2x omniscient-optimal); the RAISE repair pulls it back
#    out, and every later leg of the shuttle reuses the corrected map.
#
#  * Sec. 2.3, footnote 1: the OBSTACLE arc cost is set larger than the longest
#    possible sequence of EMPTY cells (12 * H * W here, above the total cost of
#    every free cell in the grid), so a simple threshold on the path cost tells
#    us whether the only route known has to pass through an obstacle -- and if
#    so we fall back to purely reactive driving instead of trusting the field.
#
#  * Property 2 (h(X) is optimal once k_min >= h(X)): used as the promotion
#    rule for the anytime relaxation.  When the value at the robot's own cell
#    has stopped changing, the sequence {X} from the robot is treated as good
#    and published for steering, without waiting for the whole grid to settle.
#
#  * Sec. 5.2 ("using a strategy of raising costs for previously visited
#    states, D* can be extended to support exploration"): raised costs are
#    stamped into the grid when the robot stalls, which is what levers it out
#    of wells; the inverse -- a small discount on cells already driven through
#    -- locks the proven shuttle corridor in across repeated trips.
#
# What did not transfer: the paper's exact OPEN-list PROCESS-STATE expansion
# assumes a point robot with a contact sensor and unbounded per-step compute.
# Here the per-step budget is fixed and the Go2 has a real footprint, so the
# graph search is a time-budgeted Jacobi relaxation and a reactive lidar layer
# sits underneath the field to guarantee clearance.
# =============================================================================

import math
import time

import numpy as np

from core.observation import Command


_INF = np.float32(1e9)
_ORTH = ((-1, 0), (1, 0), (0, -1), (0, 1))
_DIAG = ((-1, -1), (-1, 1), (1, -1), (1, 1))
_ALL8 = _ORTH + _DIAG
_SQ2 = 1.4142135623730951


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Controller:
    # ------------------------------------------------------------------ reset
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(0)

        self.max_lin = float(getattr(scenario, "max_lin", 0.8) or 0.8)
        self.max_ang = float(getattr(scenario, "max_ang", 1.0) or 1.0)
        self.range_max = float(getattr(scenario, "range_max", 6.0) or 6.0)
        self.time_limit = float(getattr(scenario, "time_limit", 1.0e9) or 1.0e9)

        start = np.asarray(getattr(scenario, "start", (0.0, 0.0)), dtype=float).ravel()
        goal = np.asarray(getattr(scenario, "goal", (0.0, 0.0)), dtype=float).ravel()
        sx, sy = float(start[0]), float(start[1])
        gx, gy = float(goal[0]), float(goal[1])

        # ---- planning grid ------------------------------------------------
        span = math.hypot(gx - sx, gy - sy)
        pad = max(9.0, 0.85 * span)
        self.xmin = min(sx, gx) - pad
        self.ymin = min(sy, gy) - pad
        xmax = max(sx, gx) + pad
        ymax = max(sy, gy) + pad

        res = 0.25
        wspan = max(xmax - self.xmin, 1.0)
        hspan = max(ymax - self.ymin, 1.0)
        max_side = 150
        res = max(res, wspan / max_side, hspan / max_side)
        self.res = float(res)
        self.inv_res = 1.0 / self.res
        self.W = max(8, int(math.ceil(wspan / self.res)) + 1)
        self.H = max(8, int(math.ceil(hspan / self.res)) + 1)
        H, W = self.H, self.W
        self.N = H * W

        # Footnote 1: obstacle cost above the total cost of every free cell.
        self.obst_mult = np.float32(12.0 * H * W)
        self._block_t = 0.5 * self.res * float(self.obst_mult)

        # ---- map state ----------------------------------------------------
        self.log = np.zeros((H, W), np.float32)        # occupancy log-odds
        self.visited = np.zeros((H, W), np.float32)    # proven-driveable bonus
        self.penalty = np.zeros((H, W), np.float32)    # Sec 5.2 raised costs
        self.occ = np.zeros((H, W), bool)
        self.occ_prev = np.zeros((H, W), bool)

        self.infl_n = max(1, int(round(0.50 / self.res)))
        self.infl_decay = np.float32(0.50)
        self._rpad = np.zeros((H + 2, W + 2), np.float32)

        cost0 = np.full((H, W), np.float32(1.15))
        self.c_orth = (np.float32(self.res) * cost0).astype(np.float32)
        self.c_diag = (np.float32(self.res * _SQ2) * cost0).astype(np.float32)

        # ---- relaxation scratch -------------------------------------------
        self._tmp = np.empty((H, W), np.float32)
        self._best = np.empty((H, W), np.float32)
        self.fields = {}
        self.active_key = None

        # ---- reactive layer tables ----------------------------------------
        self.n_cand = 36
        self.cand_ang = np.linspace(-math.pi, math.pi, self.n_cand, endpoint=False).astype(np.float64)
        self._beam_a = None
        self.cosD = None
        self.sinD = None
        self._along = None
        self._perp = None

        self.n_ring = 48
        ring = np.linspace(-math.pi, math.pi, self.n_ring, endpoint=False)
        self.ring_ang = ring
        self.ring_cos = np.cos(ring)
        self.ring_sin = np.sin(ring)
        self.look_r = np.array([0.55, 1.05, 1.60], dtype=np.float64)

        # ---- episode bookkeeping ------------------------------------------
        self.t_prev = 0.0
        self.step_i = 0
        self.ri = 0
        self.rj = 0
        self.last_cost_t = -1.0e9
        self.prev_theta = 0.0
        self.des_world = None
        self.goal_key_prev = None
        self.recover_until = -1.0e9
        self.escape_until = -1.0e9
        self.hist = []
        self.dwell = 0.0
        self.orbit_phase = 0.0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.budget = 0.004
        self.pose_prev = None

    # ------------------------------------------------------------- utilities
    def _cell(self, x, y):
        j = int((x - self.xmin) * self.inv_res)
        i = int((y - self.ymin) * self.inv_res)
        if i < 0:
            i = 0
        elif i >= self.H:
            i = self.H - 1
        if j < 0:
            j = 0
        elif j >= self.W:
            j = self.W - 1
        return i, j

    def _ensure_beams(self, angles):
        a = np.asarray(angles, dtype=np.float64).ravel()
        if (self._beam_a is not None and self._beam_a.shape == a.shape
                and np.allclose(self._beam_a, a, atol=1e-9)):
            return
        self._beam_a = a
        d = a[None, :] - self.cand_ang[:, None]
        self.cosD = np.cos(d).astype(np.float32)
        self.sinD = np.sin(d).astype(np.float32)
        self._along = np.empty_like(self.cosD)
        self._perp = np.empty_like(self.sinD)
        step = self.res * 0.7
        k = max(2, int(self.range_max / step) + 1)
        self.free_s = (np.arange(k, dtype=np.float32) * np.float32(step))

    # ------------------------------------------------------------ map update
    def _update_map(self, px, py, yaw, ranges, conf, contact):
        H, W = self.H, self.W
        r = np.asarray(ranges, dtype=np.float64).ravel()
        if r.shape != self._beam_a.shape:
            return
        finite = np.isfinite(r)
        rr = np.where(finite, r, self.range_max)
        np.clip(rr, 0.0, self.range_max, out=rr)

        wa = self._beam_a + yaw
        ca = np.cos(wa)
        sa = np.sin(wa)

        # ---- free-space carving (optimistic map gets corrected both ways) --
        rfree = np.maximum(rr - 0.55 * self.res, 0.0)
        S = self.free_s[:, None].astype(np.float64)
        valid = S < rfree[None, :]
        xs = px + S * ca[None, :]
        ys = py + S * sa[None, :]
        jj = ((xs - self.xmin) * self.inv_res).astype(np.int32)
        ii = ((ys - self.ymin) * self.inv_res).astype(np.int32)
        ok = valid & (ii >= 0) & (ii < H) & (jj >= 0) & (jj < W)
        if ok.any():
            freec = np.bincount((ii * W + jj)[ok], minlength=self.N)
        else:
            freec = None

        # ---- hits ---------------------------------------------------------
        hm = finite & (r > 0.22) & (r < self.range_max - 0.06)
        hitc = None
        if hm.any():
            hx = px + r[hm] * ca[hm]
            hy = py + r[hm] * sa[hm]
            hj = ((hx - self.xmin) * self.inv_res).astype(np.int32)
            hi = ((hy - self.ymin) * self.inv_res).astype(np.int32)
            okh = (hi >= 0) & (hi < H) & (hj >= 0) & (hj < W)
            if okh.any():
                hitc = np.bincount((hi * W + hj)[okh], minlength=self.N)

        hg = np.float32(1.35 * conf * (2.0 if contact else 1.0))
        fg = np.float32(0.50 * conf)
        flat = self.log.reshape(-1)
        if hitc is not None:
            flat += hg * np.minimum(hitc, 2).astype(np.float32)
        if freec is not None:
            flat -= fg * np.minimum(freec, 3).astype(np.float32)
        np.clip(self.log, -8.0, 12.0, out=self.log)

    # ----------------------------------------------------------- cost update
    def _rebuild_cost(self, px, py):
        H, W = self.H, self.W
        occ = self.log > 1.5
        self.occ = occ
        known_free = self.log < -1.0

        base = np.where(known_free, np.float32(1.0), np.float32(1.16)).astype(np.float32)
        base -= np.float32(0.12) * np.minimum(self.visited, np.float32(1.0))

        # soft inflation: raises cost near walls without sealing doorways
        risk = occ.astype(np.float32)
        p = self._rpad
        for _ in range(self.infl_n):
            p[:] = 0.0
            p[1:-1, 1:-1] = risk
            for dy, dx in _ALL8:
                np.maximum(risk, p[1 + dy:1 + dy + H, 1 + dx:1 + dx + W] * self.infl_decay,
                           out=risk)

        cost = base + np.float32(3.0) * risk + np.minimum(self.penalty, np.float32(6.0))
        np.clip(cost, 0.40, 1.0e6, out=cost)
        cost[occ] = self.obst_mult

        # never let the robot be trapped inside its own occupied footprint
        i, j = self._cell(px, py)
        sub = cost[max(0, i - 1):i + 2, max(0, j - 1):j + 2]
        np.minimum(sub, np.float32(2.0), out=sub)
        for f in self.fields.values():
            gi, gj = f["gi"], f["gj"]
            s2 = cost[max(0, gi - 1):gi + 2, max(0, gj - 1):gj + 2]
            np.minimum(s2, np.float32(2.0), out=s2)

        self.c_orth = (np.float32(self.res) * cost).astype(np.float32)
        self.c_diag = (np.float32(self.res * _SQ2) * cost).astype(np.float32)

        # RAISE events: anything that newly became an obstacle invalidates the
        # cost-to-goal fields (Sec. 2.2 -- min-relaxation cannot raise values).
        if bool(np.any(occ & ~self.occ_prev)):
            for f in self.fields.values():
                f["dirty"] = True
        self.occ_prev = occ.copy()
        self.penalty *= np.float32(0.97)

    # --------------------------------------------------------------- fields
    def _get_field(self, gx, gy):
        key = (round(gx, 1), round(gy, 1))
        f = self.fields.get(key)
        if f is None:
            if len(self.fields) >= 4:
                self.fields.clear()
            gi, gj = self._cell(gx, gy)
            PD = np.full((self.H + 2, self.W + 2), _INF, np.float32)
            PD[1 + gi, 1 + gj] = np.float32(0.0)
            f = {"gi": gi, "gj": gj, "gx": gx, "gy": gy, "PD": PD, "use": None,
                 "settled": False, "dirty": False, "sweeps": 0, "t0": 0.0,
                 "stable_v": None, "stable_n": 0}
            self.fields[key] = f
        return key, f

    def _reset_field(self, f, t):
        f["PD"][:] = _INF
        f["PD"][1 + f["gi"], 1 + f["gj"]] = np.float32(0.0)
        f["settled"] = False
        f["dirty"] = False
        f["sweeps"] = 0
        f["t0"] = t
        f["stable_v"] = None
        f["stable_n"] = 0

    def _sweep(self, f, nmax, deadline):
        """Anytime Jacobi relaxation of h(.) grown from the goal."""
        H, W = self.H, self.W
        PD = f["PD"]
        cur = PD[1:H + 1, 1:W + 1]
        c1, c2 = self.c_orth, self.c_diag
        tmp, best = self._tmp, self._best
        gi, gj = f["gi"], f["gj"]
        ri, rj = self.ri, self.rj
        n = 0
        while n < nmax:
            np.copyto(best, cur)
            for dy, dx in _ORTH:
                np.add(PD[1 + dy:1 + dy + H, 1 + dx:1 + dx + W], c1, out=tmp)
                np.minimum(best, tmp, out=best)
            for dy, dx in _DIAG:
                np.add(PD[1 + dy:1 + dy + H, 1 + dx:1 + dx + W], c2, out=tmp)
                np.minimum(best, tmp, out=best)
            best[gi, gj] = np.float32(0.0)
            np.subtract(cur, best, out=tmp)
            delta = float(tmp.max())
            np.copyto(cur, best)
            n += 1
            f["sweeps"] += 1

            if delta <= 1.0e-4:
                f["settled"] = True
                f["use"] = cur.copy()
                return
            if (f["sweeps"] & 7) == 0:
                v = float(cur[ri, rj])
                if v < self._block_t:
                    # Property 2 analogue: h at the robot has stopped moving,
                    # so publish the sequence {X} without waiting for the rest.
                    if f["stable_v"] is not None and abs(f["stable_v"] - v) < 1.0e-4:
                        f["stable_n"] += 1
                    else:
                        f["stable_n"] = 0
                    f["stable_v"] = v
                    if f["stable_n"] >= 2 or f["use"] is None:
                        f["use"] = cur.copy()
            if time.perf_counter() >= deadline:
                return

    def _work_field(self, f, t, deadline):
        if f["dirty"] and (f["settled"] or (t - f["t0"]) > 1.5):
            self._reset_field(f, t)
        if f["settled"]:
            # LOWER propagation only: newly cleared space / decayed penalties.
            before = float(f["PD"][1 + self.ri, 1 + self.rj])
            self._sweep(f, 2, deadline)
            after = float(f["PD"][1 + self.ri, 1 + self.rj])
            if f["use"] is None or after < before - 1.0e-4:
                f["use"] = f["PD"][1:self.H + 1, 1:self.W + 1].copy()
        else:
            self._sweep(f, 100000, deadline)

    # -------------------------------------------------------------- penalty
    def _add_penalty(self, px, py, radius, amount):
        i, j = self._cell(px, py)
        k = max(1, int(radius * self.inv_res))
        self.penalty[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1] += np.float32(amount)
        np.minimum(self.penalty, np.float32(6.0), out=self.penalty)
        for f in self.fields.values():
            f["dirty"] = True

    # ------------------------------------------------------------------ step
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            vx, vy, wz = self.last_cmd
            return Command(0.0, 0.0, float(np.clip(0.5, -self.max_ang, self.max_ang)))

    def _step(self, obs):
        t0 = time.perf_counter()
        deadline = t0 + self.budget

        t = float(getattr(obs, "t", self.t_prev))
        dt = t - self.t_prev
        if not (0.0 < dt < 1.0):
            dt = 0.02
        self.t_prev = t
        self.step_i += 1

        pose = np.asarray(getattr(obs, "pose_estimate", (0.0, 0.0, 0.0)), dtype=float).ravel()
        px, py = float(pose[0]), float(pose[1])
        yaw = float(pose[2]) if pose.size > 2 else 0.0
        if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(yaw)):
            return Command(0.0, 0.0, 0.0)

        gvec = np.asarray(getattr(obs, "goal", (px, py)), dtype=float).ravel()
        gx, gy = float(gvec[0]), float(gvec[1])

        perr = float(getattr(obs, "pose_error_estimate", 0.0) or 0.0)
        if not math.isfinite(perr) or perr < 0.0:
            perr = 0.0
        conf = 1.0 / (1.0 + 2.0 * perr)
        contact = bool(getattr(obs, "contact", False))

        self._ensure_beams(getattr(obs, "range_angles", np.linspace(-math.pi, math.pi, 181)))
        r_raw = np.asarray(getattr(obs, "ranges", None), dtype=np.float64).ravel()
        if r_raw.shape != self._beam_a.shape:
            r_raw = np.full(self._beam_a.shape, self.range_max)
        rr = np.where(np.isfinite(r_raw), r_raw, self.range_max)
        np.clip(rr, 0.12, self.range_max, out=rr)
        rr32 = rr.astype(np.float32)

        self.ri, self.rj = self._cell(px, py)

        # ---- goal bookkeeping --------------------------------------------
        key, field = self._get_field(gx, gy)
        if key != self.goal_key_prev:
            self.goal_key_prev = key
            self.active_key = key
            self.dwell = 0.0
            self.orbit_phase = float(self.rng.uniform(0.0, 2.0 * math.pi))
            self.hist = []
            self.penalty *= np.float32(0.4)
            for f in self.fields.values():
                f["dirty"] = True
        self.active_key = key

        # ---- perception ---------------------------------------------------
        if (self.step_i % 2) == 0 or contact:
            self._update_map(px, py, yaw, rr, conf, contact)
        if (self.step_i % 5) == 0:
            self.visited[self.ri, self.rj] = min(1.0, float(self.visited[self.ri, self.rj]) + 0.08)
        if (t - self.last_cost_t) > 0.25:
            self.last_cost_t = t
            self._rebuild_cost(px, py)

        # ---- plan (time budgeted) ----------------------------------------
        self._work_field(field, t, deadline)
        if time.perf_counter() < deadline:
            for k2, f2 in self.fields.items():
                if k2 is not key and k2 != key:
                    self._work_field(f2, t, deadline)
                    if time.perf_counter() >= deadline:
                        break

        # ---- stall / stuck detection (Sec. 5.2 raised costs) --------------
        self.hist.append((t, px, py))
        while self.hist and (t - self.hist[0][0]) > 3.5:
            self.hist.pop(0)
        dgoal = math.hypot(gx - px, gy - py)
        if (len(self.hist) > 20 and (t - self.hist[0][0]) > 3.0 and dgoal > 1.0
                and t > self.recover_until and t > self.escape_until):
            x0, y0 = self.hist[0][1], self.hist[0][2]
            if math.hypot(px - x0, py - y0) < 0.35:
                self._add_penalty(px, py, 0.7, 2.5)
                self.escape_until = t + 1.4
                self.hist = []

        if contact:
            self.recover_until = t + 0.55

        # ---- desired world heading from the cost-to-goal field ------------
        use_field = field["use"]
        theta_w = math.atan2(gy - py, gx - px)
        have_field = False
        if use_field is not None:
            R = self.look_r[:, None]
            X = px + R * self.ring_cos[None, :]
            Y = py + R * self.ring_sin[None, :]
            jj = ((X - self.xmin) * self.inv_res).astype(np.int32)
            ii = ((Y - self.ymin) * self.inv_res).astype(np.int32)
            np.clip(ii, 0, self.H - 1, out=ii)
            np.clip(jj, 0, self.W - 1, out=jj)
            vals = use_field[ii, jj].astype(np.float64) + R
            vals += np.where(self.occ[ii, jj], 1.0e7, 0.0)
            kbest = int(np.argmin(vals))
            if float(vals.flat[kbest]) < self._block_t:
                theta_w = float(self.ring_ang[kbest % self.n_ring])
                have_field = True

        # ---- close-range / arrival handling -------------------------------
        if dgoal < 0.5:
            self.dwell += dt
        else:
            self.dwell = 0.0

        near_goal = dgoal < 0.9
        if near_goal:
            theta_w = math.atan2(gy - py, gx - px)
            have_field = False

        searching = False
        if self.dwell > 3.0:
            # pose drift may mean we are not really there: sweep the area.
            searching = True
            rs = min(0.35 + 0.22 * (self.dwell - 3.0), max(1.0, 1.0 + 1.5 * perr), 2.0)
            self.orbit_phase += 1.6 * dt
            tx = gx + rs * math.cos(self.orbit_phase)
            ty = gy + rs * math.sin(self.orbit_phase)
            if math.hypot(tx - px, ty - py) > 1e-3:
                theta_w = math.atan2(ty - py, tx - px)

        # ---- reactive clearance layer -------------------------------------
        np.multiply(rr32[None, :], self.cosD, out=self._along)
        np.multiply(rr32[None, :], self.sinD, out=self._perp)
        corridor = np.float32(0.34)
        blocked = (self._along > np.float32(0.0)) & (np.abs(self._perp) < corridor)
        free_dist = np.where(blocked, self._along, np.float32(self.range_max)).min(axis=1)
        free_dist = free_dist.astype(np.float64)

        theta_des_r = _wrap(theta_w - yaw)
        if t < self.escape_until:
            theta_des_r = float(self.cand_ang[int(np.argmax(free_dist))])
            goal_w = 0.7
        else:
            goal_w = 2.4

        dth = (self.cand_ang - theta_des_r + math.pi) % (2.0 * math.pi) - math.pi
        score = (goal_w * np.cos(dth)
                 + 1.15 * np.minimum(free_dist, 2.6) / 2.6
                 + 0.35 * np.cos(self.cand_ang - self.prev_theta))
        d_min = 0.42 if not near_goal else 0.30
        adm = free_dist > d_min
        if adm.any():
            sc = np.where(adm, score, -1.0e9)
            kbest = int(np.argmax(sc))
        else:
            kbest = int(np.argmax(free_dist))
        theta_star = float(self.cand_ang[kbest])
        fd = float(free_dist[kbest])
        self.prev_theta = theta_star

        # ---- contact recovery ---------------------------------------------
        if t < self.recover_until:
            knear = int(np.argmin(rr))
            theta_esc = _wrap(float(self._beam_a[knear]) + math.pi)
            vx = 0.34 * math.cos(theta_esc)
            vy = float(np.clip(0.34 * math.sin(theta_esc), -0.32, 0.32))
            wz = float(np.clip(1.2 * theta_esc, -self.max_ang, self.max_ang))
            return self._emit(vx, vy, wz)

        # ---- velocity synthesis -------------------------------------------
        vmax = self.max_lin
        v_allow = vmax * float(np.clip((fd - 0.40) / 1.10, 0.0, 1.0))
        if not adm.any():
            v_allow = min(v_allow, 0.12)
        if searching:
            v_allow = min(v_allow, 0.42)
        if near_goal:
            v_allow = min(v_allow, 0.38 + 0.55 * dgoal)
        if not have_field and not near_goal and not searching:
            v_allow = min(v_allow, 0.65 * vmax)

        ae = _wrap(theta_star)
        align = math.cos(ae)
        if abs(ae) > 1.45:
            fwd = 0.0
        else:
            fwd = v_allow * max(0.0, align)

        lat_cap = 0.45 if (near_goal or searching) else 0.30
        vy = float(np.clip(0.60 * v_allow * math.sin(ae), -lat_cap, lat_cap))
        wz = float(np.clip(2.4 * ae, -self.max_ang, self.max_ang))
        fwd *= 1.0 / (1.0 + 0.45 * abs(wz))

        if abs(ae) > 1.45 and fd < 0.5:
            fwd = -0.12

        return self._emit(fwd, vy, wz)

    # ------------------------------------------------------------------ emit
    def _emit(self, vx, vy, wz):
        if not math.isfinite(vx):
            vx = 0.0
        if not math.isfinite(vy):
            vy = 0.0
        if not math.isfinite(wz):
            wz = 0.0
        m = self.max_lin
        vx = float(np.clip(vx, -m, m))
        vy = float(np.clip(vy, -m, m))
        sp = math.hypot(vx, vy)
        if sp > m and sp > 1e-9:
            s = m / sp
            vx *= s
            vy *= s
        wz = float(np.clip(wz, -self.max_ang, self.max_ang))
        self.last_cmd = (vx, vy, wz)
        return Command(vx, vy, wz)
