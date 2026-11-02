"""
Navigation controller for a Unitree Go2 shuttling between two fixed points
inside an unmapped indoor space.

Strategy
--------
The robot has to traverse the *same* building many times, in both directions,
so the single most valuable thing it can do is remember what it sees.  This
controller therefore keeps three pieces of persistent episode state:

  1. a log-odds occupancy grid in the pose-estimate frame, built from the
     360-degree lidar (free space along each ray, occupied at each return);
  2. a soft "penalty" layer that remembers places where the robot got stuck,
     so the planner routes around dead ends on later trips;
  3. a cost-to-go field (navigation function) obtained by relaxing an
     8-connected Dijkstra-like update on that grid with the *current* goal as
     the zero level set.  Unknown cells are traversable but slightly more
     expensive than proven-free cells, so the very first leg degenerates
     gracefully into "head for the goal and flow around what you find".

Motion is chosen by a small dynamic-window search: ~270 candidate
(vx, vy, wz) triples are rolled out for 0.8 s in the robot frame (the rollouts
are precomputed once), scored by the cost-to-go at the rollout endpoint plus a
clearance term measured directly against the raw lidar returns, and the best
one is emitted through per-axis rate limits.  The value field supplies global
intelligence; the lidar term supplies reflexes that never depend on the map
being right.

On top of that sit two safety nets: a scripted escape maneuver (back out,
re-aim at the freest goal-ward direction, drop a penalty where we failed) for
local minima and collisions, and an expanding-spiral probe around the goal for
the case where localization drift means "arrived" in the estimate frame is not
"arrived" in the world.

Only the standard library and numpy are used.  All episode state lives on the
instance and is (re)created in reset().  The public interface is reset() and
step(); everything prefixed with an underscore is private implementation
detail.
"""

import math
from collections import deque

import numpy as np

from core.observation import Command


_INF = float("inf")
_BIG = 200.0          # stand-in for "unreachable" in scoring (keeps gradients finite)


def _wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Controller:
    # ---------------- tunables ----------------
    RES = 0.25            # occupancy grid resolution [m]
    HALF = 40.0           # grid half-extent [m] (world spans [-40, 40]^2)
    LO_OCC = 0.65         # log-odds above this  -> obstacle
    LO_UNK = 0.30         # |log-odds| below this -> never really observed
    LO_MIN = -3.0
    LO_MAX = 4.5
    REPLAN_EVERY = 12     # control steps between value-field rebuilds
    MAX_CELLS = 26000     # planning grid budget (coarsened if exceeded)
    HORIZON = 0.8         # rollout horizon [s]
    SUBSTEPS = 4          # rollout samples
    LOOKAHEAD = 1.4       # path lookahead distance [m]
    ROBOT_R = 0.26        # body radius used for clearance [m]
    SAFE_R = 0.55         # desired clearance [m]

    # ============================================================= interface
    def reset(self, scenario, rng):
        """Clear ALL episode state.  Called once before the episode."""
        self.rng = rng if rng is not None else np.random.default_rng(0)

        self.res = float(self.RES)
        self.half = float(self.HALF)
        self.n = int(round(2.0 * self.half / self.res))

        # --- persistent maps (the whole point of the controller) ---
        self.lo = np.zeros((self.n, self.n), dtype=np.float64)
        self.lof = self.lo.reshape(-1)          # flat view, same memory
        self.pen = np.zeros((self.n, self.n), dtype=np.float64)
        self.bi0 = self.n
        self.bi1 = -1
        self.bj0 = self.n
        self.bj1 = -1

        # --- scenario limits ---
        self.range_max = self._f(getattr(scenario, "range_max", None), 6.0)
        self.max_lin = self._f(getattr(scenario, "max_lin", None), 0.8)
        self.max_ang = self._f(getattr(scenario, "max_ang", None), 1.0)
        self.time_limit = self._f(getattr(scenario, "time_limit", None), 1.0e9)
        if self.range_max <= 0.1:
            self.range_max = 6.0
        if self.max_lin <= 0.05:
            self.max_lin = 0.8
        if self.max_ang <= 0.05:
            self.max_ang = 1.0

        self._build_candidates()

        # --- value field / planner state ---
        self.val = None
        self.vi0 = 0
        self.vj0 = 0
        self.vf = 1
        self.vcell = self.res
        self.plan_k = -10 ** 6
        self.force_plan = True
        self.unreachable = False

        # --- episode bookkeeping ---
        self.k = 0
        self.last_t = None
        self.goal_xy = None
        self.leg_t0 = 0.0
        self.leg_id = 0
        self.trk = deque()
        self.best_v = _INF
        self.improve_t = 0.0

        # --- recovery state ---
        self.rec_t1 = -1.0          # end of back-up phase
        self.rec_t2 = -1.0          # end of whole maneuver
        self.rec_dir = 0.0          # world bearing to escape along
        self.rec_sign = 1.0
        self.rec_count = 0
        self.contact_n = 0

        # --- goal-probe state ---
        self.probe = False
        self.probe_t0 = 0.0
        self.probe_ph = 0.0
        self.near_t = None

        self.cmd = np.zeros(3, dtype=np.float64)
        self.fail = 0

    def step(self, obs):
        """One control step at 50 Hz.  Never raises."""
        try:
            return self._control(obs)
        except Exception:
            # Absolute last resort: creep forward-ish while turning, so a
            # transient bad observation cannot end the episode.
            self.fail += 1
            return Command(0.0, 0.0, 0.4 * self.max_ang)

    # ======================================================= small utilities
    @staticmethod
    def _f(v, default):
        try:
            if v is None:
                return float(default)
            x = float(v)
            if not math.isfinite(x):
                return float(default)
            return x
        except Exception:
            return float(default)

    def _cell(self, w):
        i = int(math.floor((w + self.half) / self.res))
        if i < 0:
            return 0
        if i >= self.n:
            return self.n - 1
        return i

    def _cw(self, i, j):
        """World centre of planning-grid cell (i, j)."""
        x = (self.vi0 + (i + 0.5) * self.vf) * self.res - self.half
        y = (self.vj0 + (j + 0.5) * self.vf) * self.res - self.half
        return (x, y)

    @staticmethod
    def _dil(a):
        """8-connected boolean dilation by one cell."""
        h, w = a.shape
        p = np.zeros((h + 2, w + 2), dtype=bool)
        p[1:-1, 1:-1] = a
        out = a.copy()
        out |= p[:-2, 1:-1]
        out |= p[2:, 1:-1]
        out |= p[1:-1, :-2]
        out |= p[1:-1, 2:]
        out |= p[:-2, :-2]
        out |= p[:-2, 2:]
        out |= p[2:, :-2]
        out |= p[2:, 2:]
        return out

    def _build_candidates(self):
        """Precompute the dynamic-window candidate set and its rollouts."""
        sl = self.max_lin / 0.8
        sa = self.max_ang / 1.0
        vxs = np.array([-0.30, 0.0, 0.20, 0.40, 0.60, 0.80]) * sl
        vys = np.array([-0.40, -0.20, 0.0, 0.20, 0.40]) * sl
        wzs = np.array([-1.0, -0.70, -0.45, -0.20, 0.0,
                        0.20, 0.45, 0.70, 1.0]) * sa
        VX, VY, WZ = np.meshgrid(vxs, vys, wzs, indexing="ij")
        self.cvx = np.ascontiguousarray(VX.reshape(-1))
        self.cvy = np.ascontiguousarray(VY.reshape(-1))
        self.cwz = np.ascontiguousarray(WZ.reshape(-1))

        K = self.cvx.size
        S = int(self.SUBSTEPS)
        dt = self.HORIZON / S
        tx = np.zeros((K, S))
        ty = np.zeros((K, S))
        x = np.zeros(K)
        y = np.zeros(K)
        for s in range(S):
            th = self.cwz * ((s + 0.5) * dt)
            c = np.cos(th)
            sn = np.sin(th)
            x = x + (self.cvx * c - self.cvy * sn) * dt
            y = y + (self.cvx * sn + self.cvy * c) * dt
            tx[:, s] = x
            ty[:, s] = y
        self.tx = tx
        self.ty = ty
        self.tth = self.cwz * self.HORIZON
        self.rev = (self.cvx < 0.0).astype(np.float64)
        self.absvy = np.abs(self.cvy)

    # ============================================================== mapping
    def _update_map(self, px, py, yaw, rr, aa):
        n = self.n
        rmax = self.range_max
        r = np.where(np.isfinite(rr), rr, rmax)
        r = np.clip(r, 0.0, rmax)
        ang = aa + yaw
        ca = np.cos(ang)
        sa = np.sin(ang)

        # ---- free space along each ray ----
        stepd = self.res * 0.85
        ns = max(2, int(rmax / stepd) + 1)
        d = (np.arange(ns) * stepd)[None, :]
        lim = (r - 0.30)[:, None]
        m = d <= lim
        xs = px + d * ca[:, None]
        ys = py + d * sa[:, None]
        ii = np.floor((xs + self.half) / self.res).astype(np.int64)
        jj = np.floor((ys + self.half) / self.res).astype(np.int64)
        ok = m & (ii >= 0) & (ii < n) & (jj >= 0) & (jj < n)
        if ok.any():
            fi = ii[ok]
            fj = jj[ok]
            np.add.at(self.lof, fi * n + fj, -0.55)
            self._grow_box(fi, fj)

        # ---- occupied at the returns ----
        hit = np.isfinite(rr) & (rr > 0.12) & (rr < rmax - 0.08)
        if hit.any():
            hx = px + rr[hit] * ca[hit]
            hy = py + rr[hit] * sa[hit]
            hi = np.floor((hx + self.half) / self.res).astype(np.int64)
            hj = np.floor((hy + self.half) / self.res).astype(np.int64)
            okh = (hi >= 0) & (hi < n) & (hj >= 0) & (hj < n)
            if okh.any():
                hi = hi[okh]
                hj = hj[okh]
                np.add.at(self.lof, hi * n + hj, 1.35)
                self._grow_box(hi, hj)

        # the cell we are standing in is free by construction
        ri = self._cell(px)
        rj = self._cell(py)
        self.lo[ri, rj] = min(self.lo[ri, rj], -1.5)
        np.clip(self.lo, self.LO_MIN, self.LO_MAX, out=self.lo)

    def _grow_box(self, fi, fj):
        a = int(fi.min())
        b = int(fi.max())
        c = int(fj.min())
        d = int(fj.max())
        if a < self.bi0:
            self.bi0 = a
        if b > self.bi1:
            self.bi1 = b
        if c < self.bj0:
            self.bj0 = c
        if d > self.bj1:
            self.bj1 = d

    def _add_penalty(self, x, y, rad_m, amount):
        i = self._cell(x)
        j = self._cell(y)
        r = max(1, int(round(rad_m / self.res)))
        i0 = max(0, i - r)
        i1 = min(self.n, i + r + 1)
        j0 = max(0, j - r)
        j1 = min(self.n, j + r + 1)
        sub = self.pen[i0:i1, j0:j1]
        gi = np.arange(i0, i1)[:, None] - i
        gj = np.arange(j0, j1)[None, :] - j
        disc = (gi * gi + gj * gj) <= (r * r)
        sub += amount * disc
        np.clip(self.pen, 0.0, 18.0, out=self.pen)

    # ============================================================== planning
    def _replan(self, px, py, gx, gy):
        self.pen *= 0.985
        n = self.n
        ri = self._cell(px)
        rj = self._cell(py)
        gi = self._cell(gx)
        gj = self._cell(gy)

        if self.bi1 < self.bi0:
            loi, hii = min(ri, gi), max(ri, gi)
            loj, hij = min(rj, gj), max(rj, gj)
        else:
            loi = min(self.bi0, ri, gi)
            hii = max(self.bi1, ri, gi)
            loj = min(self.bj0, rj, gj)
            hij = max(self.bj1, rj, gj)
        pad = 10
        i0 = max(0, loi - pad)
        i1 = min(n, hii + pad + 1)
        j0 = max(0, loj - pad)
        j1 = min(n, hij + pad + 1)
        H = max(3, i1 - i0)
        W = max(3, j1 - j0)

        f = 1
        while (H // f) * (W // f) > self.MAX_CELLS and f < 4:
            f += 1
        Hc = max(2, H // f)
        Wc = max(2, W // f)
        i1 = min(n, i0 + Hc * f)
        j1 = min(n, j0 + Wc * f)
        Hc = (i1 - i0) // f
        Wc = (j1 - j0) // f
        if Hc < 2 or Wc < 2:
            self.val = None
            return
        i1 = i0 + Hc * f
        j1 = j0 + Wc * f

        lo = self.lo[i0:i1, j0:j1]
        pn = self.pen[i0:i1, j0:j1]
        occ = lo > self.LO_OCC
        unk = np.abs(lo) < self.LO_UNK
        if f > 1:
            occ = occ.reshape(Hc, f, Wc, f).mean(axis=(1, 3)) > 0.40
            unk = unk.reshape(Hc, f, Wc, f).mean(axis=(1, 3)) > 0.50
            pn = pn.reshape(Hc, f, Wc, f).mean(axis=(1, 3))

        blocked = self._dil(occ)                    # body inflation
        ring = self._dil(blocked) & ~blocked        # soft stand-off ring
        mult = np.ones((Hc, Wc), dtype=np.float64)
        mult += 0.45 * unk                          # unknown: passable, pricier
        mult += 1.70 * ring
        mult += np.minimum(pn, 12.0)                # remembered dead ends

        gic = int(min(max((gi - i0) // f, 0), Hc - 1))
        gjc = int(min(max((gj - j0) // f, 0), Wc - 1))
        ric = int(min(max((ri - i0) // f, 0), Hc - 1))
        rjc = int(min(max((rj - j0) // f, 0), Wc - 1))
        # the goal and our own footprint must never be sealed off
        self._open(blocked, mult, gic, gjc, 3, 1.5)
        self._open(blocked, mult, ric, rjc, 2, 2.2)
        mult[blocked] = _INF

        cell = self.res * f
        self.val = self._solve(mult, cell, gic, gjc, ric, rjc)
        self.vi0 = i0
        self.vj0 = j0
        self.vf = f
        self.vcell = cell

    @staticmethod
    def _open(blocked, mult, i, j, rad, cost):
        h, w = blocked.shape
        i0 = max(0, i - rad)
        i1 = min(h, i + rad + 1)
        j0 = max(0, j - rad)
        j1 = min(w, j + rad + 1)
        sb = blocked[i0:i1, j0:j1]
        sm = mult[i0:i1, j0:j1]
        np.copyto(sm, np.maximum(sm, cost), where=sb)
        sb[:] = False

    def _solve(self, mult, cell, gi, gj, ri, rj):
        """Relax an 8-connected cost-to-go field with the goal pinned at 0."""
        H, W = mult.shape
        stepc = mult * cell
        dstepc = stepc * 1.41421356
        val = np.full((H, W), _INF, dtype=np.float64)
        val[gi, gj] = 0.0
        p = np.full((H + 2, W + 2), _INF, dtype=np.float64)

        maxs = int(min(700, 4 * (H + W) + 40))
        prev_nf = -1
        prev_cur = _INF
        done = 0
        with np.errstate(invalid="ignore"):
            while done < maxs:
                for _ in range(10):
                    p[1:-1, 1:-1] = val
                    card = np.minimum(
                        np.minimum(p[:-2, 1:-1], p[2:, 1:-1]),
                        np.minimum(p[1:-1, :-2], p[1:-1, 2:]))
                    diag = np.minimum(
                        np.minimum(p[:-2, :-2], p[:-2, 2:]),
                        np.minimum(p[2:, :-2], p[2:, 2:]))
                    cand = np.minimum(card + stepc, diag + dstepc)
                    np.minimum(val, cand, out=val)
                    val[gi, gj] = 0.0
                done += 10
                nf = int(np.count_nonzero(np.isfinite(val)))
                a0 = max(0, ri - 3)
                b0 = max(0, rj - 3)
                win = val[a0:ri + 4, b0:rj + 4]
                cur = float(win.min()) if win.size else _INF
                if nf == prev_nf and not (cur < prev_cur - 1.0e-3):
                    break
                prev_nf = nf
                prev_cur = cur
        return val

    def _val_at(self, xs, ys):
        """Cost-to-go at world points; _BIG where blocked/unknown-field."""
        out = np.full(np.shape(xs), _BIG, dtype=np.float64)
        val = self.val
        if val is None:
            return out
        H, W = val.shape
        ii = np.floor(((xs + self.half) / self.res - self.vi0) / self.vf)
        jj = np.floor(((ys + self.half) / self.res - self.vj0) / self.vf)
        ii = ii.astype(np.int64)
        jj = jj.astype(np.int64)
        ok = (ii >= 0) & (ii < H) & (jj >= 0) & (jj < W)
        if ok.any():
            v = val[ii[ok], jj[ok]]
            out[ok] = np.where(np.isfinite(v), np.minimum(v, _BIG), _BIG)
        return out

    def _descend(self, px, py):
        """Greedy descent of the value field -> lookahead target (world)."""
        val = self.val
        if val is None:
            return None
        H, W = val.shape
        i = int(math.floor(((px + self.half) / self.res - self.vi0) / self.vf))
        j = int(math.floor(((py + self.half) / self.res - self.vj0) / self.vf))
        i = min(max(i, 0), H - 1)
        j = min(max(j, 0), W - 1)

        if not math.isfinite(float(val[i, j])):
            # standing on an inflated/unknown cell: aim at the best cell nearby
            r = 6
            a0 = max(0, i - r)
            b0 = max(0, j - r)
            win = val[a0:i + r + 1, b0:j + r + 1]
            if win.size == 0 or not np.isfinite(win).any():
                return None
            idx = int(np.argmin(win))
            wi, wj = divmod(idx, win.shape[1])
            return self._cw(a0 + wi, b0 + wj)

        cur = float(val[i, j])
        ci, cj = i, j
        last = (px, py)
        dist = 0.0
        for _ in range(90):
            bi = -1
            bj = -1
            bv = cur
            for di in (-1, 0, 1):
                ii = ci + di
                if ii < 0 or ii >= H:
                    continue
                row = val[ii]
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    jj = cj + dj
                    if jj < 0 or jj >= W:
                        continue
                    v = row[jj]
                    if v < bv:
                        bv = float(v)
                        bi = ii
                        bj = jj
            if bi < 0:
                break
            nx, ny = self._cw(bi, bj)
            dist += math.hypot(nx - last[0], ny - last[1])
            last = (nx, ny)
            ci, cj = bi, bj
            cur = bv
            if dist >= self.LOOKAHEAD or bv <= 0.0:
                break
        if dist < 0.05:
            return None
        return last

    # ============================================================== control
    def _new_leg(self, t):
        self.leg_id += 1
        self.leg_t0 = t
        self.probe = False
        self.near_t = None
        self.rec_t1 = -1.0
        self.rec_t2 = -1.0
        self.rec_count = 0
        self.contact_n = 0
        self.best_v = _INF
        self.improve_t = t
        self.force_plan = True
        self.trk.clear()
        self.pen *= 0.6

    def _control(self, obs):
        t = self._f(getattr(obs, "t", None), 0.0)
        pe = obs.pose_estimate
        px = self._f(pe[0], 0.0)
        py = self._f(pe[1], 0.0)
        yaw = self._f(pe[2], 0.0)
        g = obs.goal
        gx = self._f(g[0], px)
        gy = self._f(g[1], py)
        perr = abs(self._f(getattr(obs, "pose_error_estimate", None), 0.0))
        contact = bool(getattr(obs, "contact", False))

        prev_t = self.last_t
        self.last_t = t
        if prev_t is None or t < prev_t:
            dtr = 0.02
        else:
            dtr = min(0.5, max(0.001, t - prev_t))
        self.k += 1

        # ---- new leg? (harness flipped the goal) ----
        if self.goal_xy is None or \
           (gx - self.goal_xy[0]) ** 2 + (gy - self.goal_xy[1]) ** 2 > 0.25:
            self._new_leg(t)
        self.goal_xy = (gx, gy)

        # ---- lidar ----
        rr = np.asarray(obs.ranges, dtype=np.float64).reshape(-1)
        aa = np.asarray(obs.range_angles, dtype=np.float64).reshape(-1)
        m = min(rr.size, aa.size)
        rr = rr[:m]
        aa = aa[:m]
        if m == 0:
            return self._emit(0.0, 0.0, 0.4 * self.max_ang)

        self._update_map(px, py, yaw, rr, aa)

        # ---- progress tracking ----
        if self.k % 5 == 0:
            self.trk.append((t, px, py))
        while self.trk and t - self.trk[0][0] > 3.5:
            self.trk.popleft()

        # ---- (re)plan ----
        if self.force_plan or (self.k - self.plan_k) >= self.REPLAN_EVERY:
            self._replan(px, py, gx, gy)
            self.force_plan = False
            self.plan_k = self.k

        vr = float(self._val_at(np.array([px]), np.array([py]))[0])
        if vr < self.best_v - 0.12:
            self.best_v = vr
            self.improve_t = t

        dg = math.hypot(gx - px, gy - py)
        gbear = math.atan2(gy - py, gx - px)

        # ---- goal probe: believed-arrived but the goal never switched ----
        if self.probe:
            if dg > 2.2:
                self.probe = False
        else:
            if dg < 0.5:
                if self.near_t is None:
                    self.near_t = t
                elif t - self.near_t > 1.5:
                    self.probe = True
                    self.probe_t0 = t
                    self.probe_ph = float(self.rng.uniform(0.0, 2.0 * math.pi))
            else:
                self.near_t = None

        # ---- stuck / contact detection ----
        if contact:
            self.contact_n += 1
        else:
            self.contact_n = max(0, self.contact_n - 1)

        stuck = False
        if t > self.rec_t2 and not self.probe and t - self.leg_t0 > 1.5:
            if len(self.trk) >= 4 and (t - self.trk[0][0]) > 3.0:
                d = 0.0
                for (_tt, ox, oy) in self.trk:
                    dd = math.hypot(px - ox, py - oy)
                    if dd > d:
                        d = dd
                if d < 0.30:
                    stuck = True
            if t - self.improve_t > 8.0:
                stuck = True
            if self.contact_n > 18:
                stuck = True

        if stuck:
            self.rec_count += 1
            self.rec_sign = 1.0 if (self.rec_count % 2) == 1 else -1.0
            self.rec_t1 = t + 0.55
            self.rec_t2 = t + 0.55 + min(2.4, 1.0 + 0.3 * self.rec_count)
            self.rec_dir = self._escape_dir(rr, aa, yaw, gbear)
            self._add_penalty(px, py, 0.45, 1.2)
            fx = px + 0.65 * math.cos(gbear)
            fy = py + 0.65 * math.sin(gbear)
            self._add_penalty(fx, fy, 0.45, 2.6)
            self.improve_t = t
            self.best_v = _INF
            self.contact_n = 0
            self.trk.clear()
            self.force_plan = True

        # ---- clearance helpers ----
        rfin = np.where(np.isfinite(rr), rr, self.range_max)
        fwd_m = np.abs(_wrap_arr(aa)) < 0.45
        fwd = float(rfin[fwd_m].min()) if fwd_m.any() else self.range_max
        rear_m = np.abs(_wrap_arr(aa + math.pi)) < 0.80
        rear = float(rfin[rear_m].min()) if rear_m.any() else self.range_max

        # ---- scripted recovery ----
        if t < self.rec_t2:
            if t < self.rec_t1 and rear > 0.60:
                return self._emit(-0.50 * self.max_lin,
                                  0.0,
                                  0.30 * self.max_ang * self.rec_sign)
            err = _wrap(self.rec_dir - yaw)
            wz = max(-self.max_ang, min(self.max_ang, 2.4 * err))
            vx = 0.0
            if abs(err) < 0.45 and fwd > 0.55:
                vx = 0.45 * self.max_lin
            vy = 0.0
            if abs(err) > 1.4 and rear > 0.7:
                vx = -0.20 * self.max_lin
            return self._emit(vx, vy, wz)

        # ---- pick a target ----
        if self.probe:
            self.probe_ph += 1.1 * dtr
            el = t - self.probe_t0
            rmaxp = min(1.7, 0.45 + 1.4 * perr)
            R = min(0.35 + 0.30 * el, max(0.35, rmaxp))
            tgx = gx + R * math.cos(self.probe_ph)
            tgy = gy + R * math.sin(self.probe_ph)
            field = False
            self.unreachable = False
        else:
            tg = self._descend(px, py)
            if tg is None:
                # goal currently unreachable in our map: head at it directly
                # and let the map heal (drift can paint phantom walls).
                field = False
                tgx, tgy = gx, gy
                self.unreachable = True
                if self.k % 25 == 0:
                    self.lo *= 0.97
            else:
                field = True
                tgx, tgy = tg
                self.unreachable = False

        # ---- dynamic window over the precomputed rollouts ----
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        exs = px + self.tx[:, -1] * cy - self.ty[:, -1] * sy
        eys = py + self.tx[:, -1] * sy + self.ty[:, -1] * cy

        sel = np.isfinite(rr) & (rr > 0.06) & (rr < 3.2)
        if sel.any():
            orr = rr[sel]
            oaa = aa[sel]
            if orr.size > 110:
                st = int(orr.size // 110) + 1
                orr = orr[::st]
                oaa = oaa[::st]
            ox = orr * np.cos(oaa)
            oy = orr * np.sin(oaa)
            dx = self.tx[:, :, None] - ox[None, None, :]
            dy = self.ty[:, :, None] - oy[None, None, :]
            dmin = np.sqrt(np.min(dx * dx + dy * dy, axis=(1, 2)))
            dmin0 = float(orr.min())
        else:
            dmin = np.full(self.cvx.size, 10.0)
            dmin0 = 10.0

        thresh = min(self.SAFE_R, dmin0 + 0.12)
        hard_t = min(self.ROBOT_R, max(0.11, dmin0 - 0.05))
        enc = np.clip(thresh - dmin, 0.0, None)
        cost = 2.6 * enc + 60.0 * (dmin < hard_t)

        vend = self._val_at(exs, eys)
        dend = np.hypot(tgx - exs, tgy - eys)
        if field:
            cost = cost + vend + 0.45 * dend
        else:
            cost = cost + 1.8 * dend + 0.15 * vend

        fh = yaw + self.tth
        des = np.arctan2(tgy - eys, tgx - exs)
        herr = np.abs(np.arctan2(np.sin(des - fh), np.cos(des - fh)))
        cost = cost + 0.22 * herr

        cost = cost + 0.10 * self.absvy + 0.12 * self.rev - 0.03 * self.cvx
        cost = cost + 0.10 * np.abs(self.cvx - self.cmd[0]) \
                    + 0.10 * np.abs(self.cvy - self.cmd[1]) \
                    + 0.05 * np.abs(self.cwz - self.cmd[2])

        b = int(np.argmin(cost))
        vx = float(self.cvx[b])
        vy = float(self.cvy[b])
        wz = float(self.cwz[b])

        # belt-and-braces: never drive into something right in front of us
        if fwd < 0.30 and vx > 0.0:
            vx = 0.0
        return self._emit(vx, vy, wz)

    def _escape_dir(self, rr, aa, yaw, gbear):
        """World bearing of the most promising escape direction."""
        r = np.where(np.isfinite(rr), rr, self.range_max)
        rs = np.minimum.reduce([np.roll(r, s) for s in (-2, -1, 0, 1, 2)])
        score = np.minimum(rs, 4.0) + 1.3 * np.cos(yaw + aa - gbear)
        try:
            score = score + 0.25 * self.rng.standard_normal(score.shape)
        except Exception:
            pass
        idx = int(np.argmax(score))
        return _wrap(yaw + float(aa[idx]))

    def _emit(self, vx, vy, wz):
        c = self.cmd
        dvx = float(np.clip(vx - c[0], -0.30, 0.16))
        dvy = float(np.clip(vy - c[1], -0.20, 0.14))
        dwz = float(np.clip(wz - c[2], -0.40, 0.40))
        c[0] = float(np.clip(c[0] + dvx, -self.max_lin, self.max_lin))
        c[1] = float(np.clip(c[1] + dvy, -self.max_lin, self.max_lin))
        c[2] = float(np.clip(c[2] + dwz, -self.max_ang, self.max_ang))
        return Command(float(c[0]), float(c[1]), float(c[2]))


def _wrap_arr(a):
    """Vectorised angle wrap to [-pi, pi)."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi
