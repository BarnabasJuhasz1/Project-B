"""
Navigation controller for a Unitree Go2 shuttling between two fixed points in an
unmapped indoor building.

Design
------
The robot repeats the same one-way trip over and over, so the single most
valuable thing the controller can do is *remember the building*.  Three layers:

1.  A persistent log-odds occupancy grid, built from the 360-degree lidar and
    ``pose_estimate``.  It is never cleared when the goal switches, so every
    trip after the first one is planned on a known map.

2.  A global geodesic distance field to the current goal, computed on that grid
    with a min-plus "fast sweeping" solver (vectorised per grid row, so it is
    cheap enough to refresh a few times per second).  Unobserved cells are
    treated as *traversable but slightly expensive*: the robot therefore drives
    optimistically straight at the goal on the first trip, discovers walls,
    reroutes, and on later trips prefers the corridor it already knows works.
    A greedy descent of the field yields a path, from which a carrot point is
    tracked.

3.  A dynamic-window local planner that scores a pruned set of reachable
    ``(vx, vy, wz)`` candidates by rolling them out against the *raw* lidar
    returns.  This is what actually keeps the robot off the walls, so safety
    never depends on the map being correctly registered.

On top of that: contact reflexes, stall/livelock detection with decaying
penalty blobs stamped into the planning cost, and a purely reactive gap-follower
used whenever the map cannot produce a route.
"""

import math
from collections import deque

import numpy as np

from core.observation import Command


_BLOCK = 400.0          # per-cell traversal cost standing in for "impassable"
_INF = 1.0e9
_NEIGH = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))


def _dilate(mask, k):
    """8-connected binary dilation, k times, without scipy."""
    out = mask
    for _ in range(int(k)):
        h, w = out.shape
        p = np.zeros((h + 2, w + 2), dtype=bool)
        p[1:-1, 1:-1] = out
        out = (p[1:-1, 1:-1] | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] |
               p[1:-1, 2:] | p[:-2, :-2] | p[:-2, 2:] | p[2:, :-2] | p[2:, 2:])
    return out


def _wrap(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


class Controller:
    # ------------------------------------------------------------------ setup
    def reset(self, scenario, rng):
        self.rng = rng

        self.range_max = float(getattr(scenario, "range_max", 6.0) or 6.0)
        self.max_lin = float(getattr(scenario, "max_lin", 0.8) or 0.8)
        self.max_ang = float(getattr(scenario, "max_ang", 1.0) or 1.0)
        self.time_limit = float(getattr(scenario, "time_limit", 1.0e9) or 1.0e9)

        start = np.asarray(getattr(scenario, "start", (0.0, 0.0)),
                           dtype=np.float64).ravel()[:2]
        goal = np.asarray(getattr(scenario, "goal", (0.0, 0.0)),
                          dtype=np.float64).ravel()[:2]
        if start.size < 2:
            start = np.zeros(2)
        if goal.size < 2:
            goal = np.zeros(2)

        # --- grid sizing: cover both endpoints plus a generous detour margin,
        # but cap the cell count so planning cost is bounded regardless.
        span = float(np.hypot(*(goal - start)))
        side = span + 2.0 * (self.range_max * 2.0 + 3.0)
        side = max(side, 24.0)
        max_cells = 192
        self.res = float(max(0.16, side / max_cells))
        n = int(min(max_cells, math.ceil(side / self.res) + 2))
        self.H = n
        self.W = n
        cx = 0.5 * (start[0] + goal[0])
        cy = 0.5 * (start[1] + goal[1])
        self.ox = cx - 0.5 * n * self.res
        self.oy = cy - 0.5 * n * self.res

        self.L = np.zeros((n, n), dtype=np.float32)      # log-odds occupancy
        self.pen = np.zeros((n, n), dtype=np.float32)    # livelock penalties

        self.k_hard = max(1, int(round(0.30 / self.res)))
        self.occ_inc = 0.85
        self.free_inc = 0.32
        self.l_max = 4.0

        # --- planning / path state
        self.Dfield = None
        self.Cplan = None
        self.path = None
        self.path_idx = 0
        self.plan_period = max(12, int(round(0.55 / 0.02)))
        self.next_plan = 0
        self.plan_fail = 0

        # --- episode bookkeeping
        self.k = 0
        self.goal_ref = None
        self.trips = 0
        self.last_cmd = (0.0, 0.0, 0.0)
        self.hist = deque(maxlen=64)      # (t, x, y) every ~0.2 s
        self.hist_next_t = -1.0
        self.recover_until = -1.0
        self.recover_sign = 1.0
        self.stuck_count = 0
        self.contact_until = -1.0
        self.best_field = _INF
        self.best_field_t = 0.0
        self.pose_err = 0.0

        self._build_candidates()

    def _build_candidates(self):
        ml = self.max_lin
        ma = self.max_ang
        vxs = np.array([-0.40, -0.18, 0.0, 0.18, 0.35, 0.50, 0.65, 0.80, 1.0]) * ml
        vys = np.array([-0.45, -0.20, 0.0, 0.20, 0.45]) * ml
        wzs = np.array([-1.0, -0.72, -0.48, -0.30, -0.16, -0.06, 0.0,
                        0.06, 0.16, 0.30, 0.48, 0.72, 1.0]) * ma
        VX, VY, WZ = np.meshgrid(vxs, vys, wzs, indexing="ij")
        VX = VX.ravel().astype(np.float64)
        VY = VY.ravel().astype(np.float64)
        WZ = WZ.ravel().astype(np.float64)

        nrm = np.hypot(VX, VY)
        sc = np.where(nrm > ml, ml / np.maximum(nrm, 1e-9), 1.0)
        VX = VX * sc
        VY = VY * sc

        self.cVX = VX
        self.cVY = VY
        self.cWZ = WZ
        self.cSPD = np.hypot(VX, VY)

        K = 5
        dt = 0.22
        ts = np.arange(1, K + 1) * dt
        th = WZ[:, None] * ts[None, :]
        wsafe = np.where(np.abs(WZ) > 1e-3, WZ, 1.0)[:, None]
        st = np.sin(th)
        ct = np.cos(th)
        x_turn = (VX[:, None] * st + VY[:, None] * (ct - 1.0)) / wsafe
        y_turn = (VX[:, None] * (1.0 - ct) + VY[:, None] * st) / wsafe
        straight = (np.abs(WZ) <= 1e-3)[:, None]
        self.cX = np.where(straight, VX[:, None] * ts[None, :], x_turn)
        self.cY = np.where(straight, VY[:, None] * ts[None, :], y_turn)
        self.cK = K
        self.cNear = min(3, K)

    # ------------------------------------------------------------ grid helpers
    def _cells(self, xs, ys):
        ix = np.floor((xs - self.ox) / self.res)
        iy = np.floor((ys - self.oy) / self.res)
        ok = (ix >= 0) & (ix < self.W) & (iy >= 0) & (iy < self.H)
        return ix[ok].astype(np.int32), iy[ok].astype(np.int32)

    def _cell_one(self, x, y):
        ix = int(math.floor((x - self.ox) / self.res))
        iy = int(math.floor((y - self.oy) / self.res))
        ix = min(max(ix, 0), self.W - 1)
        iy = min(max(iy, 0), self.H - 1)
        return ix, iy

    def _field_at(self, xs, ys):
        if self.Dfield is None:
            return np.zeros(np.shape(xs), dtype=np.float32)
        ix = np.clip(np.floor((xs - self.ox) / self.res), 0, self.W - 1).astype(np.int32)
        iy = np.clip(np.floor((ys - self.oy) / self.res), 0, self.H - 1).astype(np.int32)
        return self.Dfield[iy, ix]

    def _add_penalty(self, x, y, amount, radius):
        ix, iy = self._cell_one(x, y)
        r = max(1, int(round(radius / self.res)))
        a0 = max(0, iy - r)
        a1 = min(self.H, iy + r + 1)
        b0 = max(0, ix - r)
        b1 = min(self.W, ix + r + 1)
        blk = self.pen[a0:a1, b0:b1]
        np.minimum(blk + np.float32(amount), np.float32(14.0), out=blk)

    # --------------------------------------------------------------- mapping
    def _update_map(self, px, py, yaw, rvals, angles_r, finite):
        conf = float(np.clip(1.0 / (1.0 + 1.4 * self.pose_err), 0.15, 1.0))
        occ_inc = np.float32(self.occ_inc * conf)
        free_inc = np.float32(self.free_inc * conf)

        aw = angles_r + yaw
        ca = np.cos(aw)
        sa = np.sin(aw)
        rr = np.minimum(rvals, self.range_max)

        # free space along every beam
        step = self.res * 0.8
        nk = int(self.range_max / step) + 1
        ds = ((np.arange(nk) + 0.5) * step)[:, None]
        lim = (rr - 0.65 * self.res)[None, :]
        fm = (ds < lim)
        if fm.any():
            fx = px + ds * ca[None, :]
            fy = py + ds * sa[None, :]
            ix, iy = self._cells(fx[fm], fy[fm])
            if ix.size:
                cur = self.L[iy, ix]
                self.L[iy, ix] = np.maximum(cur - free_inc, np.float32(-self.l_max))

        hit = finite & (rvals < self.range_max - 0.12)
        if hit.any():
            hx = px + rvals[hit] * ca[hit]
            hy = py + rvals[hit] * sa[hit]
            ix, iy = self._cells(hx, hy)
            if ix.size:
                cur = self.L[iy, ix]
                self.L[iy, ix] = np.minimum(cur + occ_inc, np.float32(self.l_max))

    # --------------------------------------------------------------- planning
    def _make_cost(self, px, py, relax):
        L = self.L
        occ = L > 0.55
        kh = max(0, self.k_hard - (1 if relax else 0))
        hard = _dilate(occ, kh) if kh > 0 else occ
        C = np.full((self.H, self.W), 1.0, dtype=np.float32)
        unk = (L > -0.30) & ~occ
        C[unk] = np.float32(1.35)
        if not relax:
            soft = _dilate(hard, 2)
            C[soft] += np.float32(2.2)
        C += self.pen
        C[hard] = np.float32(_BLOCK)
        C[0, :] = _BLOCK
        C[-1, :] = _BLOCK
        C[:, 0] = _BLOCK
        C[:, -1] = _BLOCK

        # never let the robot's own footprint be walled in
        rx, ry = self._cell_one(px, py)
        a0 = max(1, ry - 2)
        a1 = min(self.H - 1, ry + 3)
        b0 = max(1, rx - 2)
        b1 = min(self.W - 1, rx + 3)
        sub = C[a0:a1, b0:b1]
        C[a0:a1, b0:b1] = np.where(occ[a0:a1, b0:b1], sub,
                                   np.minimum(sub, np.float32(1.0)))
        return C, (rx, ry)

    def _find_seed(self, C, gxi, gyi):
        if C[gyi, gxi] < 50.0:
            return (gxi, gyi)
        R = 26
        a0 = max(1, gyi - R)
        a1 = min(self.H - 1, gyi + R + 1)
        b0 = max(1, gxi - R)
        b1 = min(self.W - 1, gxi + R + 1)
        sub = C[a0:a1, b0:b1]
        yy, xx = np.nonzero(sub < 50.0)
        if yy.size == 0:
            return None
        d2 = (yy + a0 - gyi) ** 2 + (xx + b0 - gxi) ** 2
        j = int(np.argmin(d2))
        return (int(xx[j] + b0), int(yy[j] + a0))

    def _sweep(self, C, seed, rounds, D=None):
        """Min-plus fast sweeping: exact 1-D min-plus scans inside each row,
        vertical/diagonal coupling between consecutive rows."""
        H, W = C.shape
        cost = C * np.float32(self.res)
        costd = cost * np.float32(math.sqrt(2.0))
        Sf = np.cumsum(cost, axis=1)
        Sr = np.cumsum(cost[:, ::-1], axis=1)
        if D is None:
            D = np.full((H, W), np.float32(_INF), dtype=np.float32)
            D[seed[1], seed[0]] = 0.0

        for _ in range(int(rounds)):
            for dy, dx in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
                rows = range(1, H) if dy > 0 else range(H - 2, -1, -1)
                for r in rows:
                    pr = D[r - dy]
                    row = D[r]
                    c = cost[r]
                    cd = costd[r]
                    np.minimum(row, pr + c, out=row)
                    np.minimum(row[1:], pr[:-1] + cd[1:], out=row[1:])
                    np.minimum(row[:-1], pr[1:] + cd[:-1], out=row[:-1])
                    if dx > 0:
                        S = Sf[r]
                        np.minimum(row, np.minimum.accumulate(row - S) + S, out=row)
                    else:
                        S = Sr[r]
                        tmp = np.minimum.accumulate(row[::-1] - S) + S
                        np.minimum(row, tmp[::-1], out=row)
        return D

    def _descend(self, D, C, rx, ry, seed, gx, gy):
        """Greedy steepest descent of the field from the robot cell."""
        pts = []
        cx, cy = rx, ry
        seen = set()
        reached = False
        for _ in range(1400):
            if (cx, cy) == seed:
                reached = True
                break
            base = D[cy, cx]
            best = None
            bv = base
            for dy, dx in _NEIGH:
                ny = cy + dy
                nx = cx + dx
                if ny < 0 or ny >= self.H or nx < 0 or nx >= self.W:
                    continue
                if C[ny, nx] >= 50.0:
                    continue
                v = D[ny, nx]
                if v < bv - 1e-5:
                    bv = v
                    best = (nx, ny)
            if best is None or best in seen:
                break
            seen.add(best)
            cx, cy = best
            pts.append((self.ox + (cx + 0.5) * self.res,
                        self.oy + (cy + 0.5) * self.res))
        if reached:
            pts.append((float(gx), float(gy)))
        return reached, pts

    def _replan(self, px, py, gx, gy):
        self.Dfield = None
        self.Cplan = None
        newpath = None
        for relax in (False, True):
            C, (rx, ry) = self._make_cost(px, py, relax)
            gxi, gyi = self._cell_one(gx, gy)
            seed = self._find_seed(C, gxi, gyi)
            if seed is None:
                continue
            D = self._sweep(C, seed, 2)
            reached, pts = self._descend(D, C, rx, ry, seed, gx, gy)
            if not reached:
                D = self._sweep(C, seed, 3, D)
                reached, pts = self._descend(D, C, rx, ry, seed, gx, gy)
            if reached and len(pts) > 0:
                self.Dfield = D
                self.Cplan = C
                newpath = pts
                break
            if len(pts) > 3 and newpath is None:
                self.Dfield = D
                self.Cplan = C
                newpath = pts

        if newpath is None or len(newpath) < 1:
            self.path = None
            self.path_idx = 0
            self.plan_fail += 1
        else:
            self.path = np.asarray(newpath, dtype=np.float64)
            self.path_idx = 0
            self.plan_fail = 0

    # --------------------------------------------------------------- guidance
    def _carrot(self, px, py, speed):
        p = self.path
        if p is None or p.shape[0] == 0:
            return None
        n = p.shape[0]
        lo = max(0, min(self.path_idx, n - 1))
        hi = min(n, lo + 60)
        seg = p[lo:hi]
        d = np.hypot(seg[:, 0] - px, seg[:, 1] - py)
        j = int(np.argmin(d)) + lo
        if d.size and float(d.min()) > 3.5 * max(self.res, 0.2) + 1.5:
            # lost the path entirely; re-anchor globally
            dall = np.hypot(p[:, 0] - px, p[:, 1] - py)
            j = int(np.argmin(dall))
        self.path_idx = j

        look = float(np.clip(1.05 + 1.1 * speed, 1.05, 2.15))
        acc = 0.0
        i = j
        while i + 1 < n and acc < look:
            acc += float(np.hypot(p[i + 1, 0] - p[i, 0], p[i + 1, 1] - p[i, 1]))
            i += 1
        return float(p[i, 0]), float(p[i, 1])

    def _reactive_carrot(self, rvals, angles_r, goal_bearing_r, goal_dist):
        n = rvals.size
        if n < 5:
            return 1.5 * math.cos(goal_bearing_r), 1.5 * math.sin(goal_bearing_r)
        w = 5
        h = w // 2
        pad = np.concatenate([rvals[-h:], rvals, rvals[:h]])
        sm = pad[0:n].copy()
        for i in range(1, w):
            np.minimum(sm, pad[i:i + n], out=sm)
        cost = -np.minimum(sm, 4.0) + 2.2 * np.abs(_wrap(angles_r - goal_bearing_r))
        cost = np.where(sm > 0.9, cost, cost + 50.0)
        j = int(np.argmin(cost))
        ang = float(angles_r[j])
        reach = float(min(2.0, max(0.7, sm[j] - 0.35)))
        reach = min(reach, max(0.4, goal_dist))
        return reach * math.cos(ang), reach * math.sin(ang)

    # ------------------------------------------------------------------- step
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            vx, vy, wz = self.last_cmd
            return Command(0.0, 0.0, float(np.clip(0.4 if wz >= 0 else -0.4,
                                                   -self.max_ang, self.max_ang)))

    def _step(self, obs):
        self.k += 1
        t = float(getattr(obs, "t", self.k * 0.02) or 0.0)

        pose = np.asarray(obs.pose_estimate, dtype=np.float64).ravel()
        px = float(pose[0])
        py = float(pose[1])
        yaw = float(pose[2]) if pose.size > 2 else 0.0
        try:
            self.pose_err = float(max(0.0, obs.pose_error_estimate))
        except Exception:
            self.pose_err = 0.0

        goal = np.asarray(obs.goal, dtype=np.float64).ravel()
        gx = float(goal[0])
        gy = float(goal[1])

        raw = np.asarray(obs.ranges, dtype=np.float64).ravel()
        ang = np.asarray(obs.range_angles, dtype=np.float64).ravel()
        if ang.size != raw.size:
            ang = np.linspace(-np.pi, np.pi, raw.size, endpoint=False)
        finite = np.isfinite(raw) & (raw > 0.05)
        rvals = np.where(finite, raw, self.range_max)
        rvals = np.clip(rvals, 0.05, self.range_max)
        contact = bool(getattr(obs, "contact", False))

        # ---- goal switch: keep the map, drop everything episode-local
        if self.goal_ref is None or math.hypot(gx - self.goal_ref[0],
                                               gy - self.goal_ref[1]) > 0.5:
            if self.goal_ref is not None:
                self.trips += 1
            self.goal_ref = (gx, gy)
            self.path = None
            self.path_idx = 0
            self.Dfield = None
            self.next_plan = self.k
            self.pen *= np.float32(0.30)
            self.hist.clear()
            self.stuck_count = 0
            self.recover_until = -1.0
            self.best_field = _INF
            self.best_field_t = t

        # ---- map
        if self.k % 2 == 0:
            self._update_map(px, py, yaw, rvals, ang, finite)

        goal_dist = math.hypot(gx - px, gy - py)
        goal_bearing_r = _wrap(math.atan2(gy - py, gx - px) - yaw)

        # ---- stall / livelock detection
        if t >= self.hist_next_t:
            self.hist_next_t = t + 0.2
            self.hist.append((t, px, py))
        if (t > self.recover_until and self.contact_until < t
                and goal_dist > 0.55 and len(self.hist) >= 18):
            t0, x0, y0 = self.hist[-18]
            if t - t0 > 3.0 and math.hypot(px - x0, py - y0) < 0.32:
                self.stuck_count += 1
                self.recover_until = t + min(2.2, 0.9 + 0.35 * self.stuck_count)
                left = float(np.min(rvals[np.abs(_wrap(ang - 1.5)) < 0.8])) \
                    if np.any(np.abs(_wrap(ang - 1.5)) < 0.8) else self.range_max
                right = float(np.min(rvals[np.abs(_wrap(ang + 1.5)) < 0.8])) \
                    if np.any(np.abs(_wrap(ang + 1.5)) < 0.8) else self.range_max
                self.recover_sign = 1.0 if left >= right else -1.0
                self._add_penalty(px, py, 2.0 + 0.8 * self.stuck_count, 0.45)
                self.hist.clear()
                self.next_plan = self.k

        # livelock over longer timescales: stop re-walking the same loop
        if self.Dfield is not None:
            fd = float(self._field_at(np.array([px]), np.array([py]))[0])
            if fd < self.best_field - 0.8:
                self.best_field = fd
                self.best_field_t = t
            elif t - self.best_field_t > 9.0:
                for rec in list(self.hist)[-24:]:
                    self._add_penalty(rec[1], rec[2], 1.1, 0.35)
                self.best_field = fd
                self.best_field_t = t
                self.next_plan = self.k

        # ---- global replan
        if self.k >= self.next_plan:
            self._replan(px, py, gx, gy)
            per = self.plan_period if self.path is not None else max(8, self.plan_period // 2)
            self.next_plan = self.k + per
            self.pen *= np.float32(0.995)

        # ---- carrot
        vx0, vy0, wz0 = self.last_cmd
        speed = math.hypot(vx0, vy0)
        car = None
        if goal_dist < 1.15:
            car = (gx - px, gy - py)
            car = (car[0] * math.cos(-yaw) - car[1] * math.sin(-yaw),
                   car[0] * math.sin(-yaw) + car[1] * math.cos(-yaw))
        else:
            cw = self._carrot(px, py, speed)
            if cw is not None:
                dx = cw[0] - px
                dy = cw[1] - py
                car = (dx * math.cos(-yaw) - dy * math.sin(-yaw),
                       dx * math.sin(-yaw) + dy * math.cos(-yaw))
        use_field = self.path is not None and self.Dfield is not None and goal_dist >= 1.15
        if car is None:
            car = self._reactive_carrot(rvals, ang, goal_bearing_r, goal_dist)
            use_field = False

        # ---- contact reflex
        if contact:
            self.contact_until = t + 0.45
            j = int(np.argmin(rvals))
            a = float(ang[j])
            self._add_penalty(px + rvals[j] * math.cos(a + yaw),
                              py + rvals[j] * math.sin(a + yaw), 1.5, 0.3)

        # ---- obstacle point cloud (robot frame) for the local planner
        near = rvals < 2.9
        if np.count_nonzero(near) > 96:
            idx = np.nonzero(near)[0][::2]
        else:
            idx = np.nonzero(near)[0]
        if idx.size:
            orr = rvals[idx]
            oa = ang[idx]
            oxr = orr * np.cos(oa)
            oyr = orr * np.sin(oa)
            cur_min = float(orr.min())
        else:
            oxr = np.array([1.0e4])
            oyr = np.array([1.0e4])
            cur_min = self.range_max

        # ---- recovery override
        if t <= self.recover_until or t <= self.contact_until:
            rear = np.abs(_wrap(ang - np.pi)) < 0.7
            rear_clear = float(np.min(rvals[rear])) if np.any(rear) else self.range_max
            if rear_clear > 0.55:
                vx = -0.42 * self.max_lin / 0.8
                wz = 0.55 * self.recover_sign * self.max_ang
            else:
                vx = 0.0
                wz = 0.95 * self.recover_sign * self.max_ang
            vy = 0.0
            cmd = (float(np.clip(vx, -self.max_lin, self.max_lin)),
                   float(np.clip(vy, -self.max_lin, self.max_lin)),
                   float(np.clip(wz, -self.max_ang, self.max_ang)))
            self.last_cmd = cmd
            return Command(cmd[0], cmd[1], cmd[2])

        # ---- dynamic window
        ml = self.max_lin
        ma = self.max_ang
        spd_cap = ml
        if goal_dist < 1.0:
            spd_cap = float(np.clip(0.95 * goal_dist + 0.22, 0.26, ml))
        mask = ((np.abs(self.cVX - vx0) <= 0.42 * ml / 0.8 + 1e-9)
                & (np.abs(self.cVY - vy0) <= 0.38 * ml / 0.8 + 1e-9)
                & (np.abs(self.cWZ - wz0) <= 0.85 * ma + 1e-9)
                & (self.cSPD <= spd_cap + 1e-9))
        if np.count_nonzero(mask) < 10:
            mask = self.cSPD <= spd_cap + 1e-9
            if np.count_nonzero(mask) < 10:
                mask = np.ones_like(self.cSPD, dtype=bool)

        VX = self.cVX[mask]
        VY = self.cVY[mask]
        WZ = self.cWZ[mask]
        X = self.cX[mask]
        Y = self.cY[mask]

        dx = X[:, :, None] - oxr[None, None, :]
        dy = Y[:, :, None] - oyr[None, None, :]
        d2 = dx * dx + dy * dy
        dmin = np.sqrt(d2.min(axis=(1, 2)))
        dnear = np.sqrt(d2[:, :self.cNear, :].min(axis=(1, 2)))

        safe = min(0.30, max(0.15, cur_min - 0.09))
        feasible = dnear >= safe

        cxr, cyr = float(car[0]), float(car[1])
        dcar = np.hypot(X[:, -1] - cxr, Y[:, -1] - cyr)
        phi = math.atan2(cyr, cxr)

        J = dcar.copy()
        if use_field:
            ex = px + math.cos(yaw) * X[:, -1] - math.sin(yaw) * Y[:, -1]
            ey = py + math.sin(yaw) * X[:, -1] + math.cos(yaw) * Y[:, -1]
            f_end = self._field_at(ex, ey).astype(np.float64)
            f_now = float(self._field_at(np.array([px]), np.array([py]))[0])
            J += 0.45 * np.clip(f_end - f_now, -4.0, 14.0)

        clw = 6.0 + 2.5 * min(self.pose_err, 1.5)
        J += clw * np.clip(0.85 - dmin, 0.0, None) ** 2
        J += 0.20 * np.abs(WZ) / ma
        J += 0.30 * np.abs(VY) / ml
        J += 0.55 * np.clip(-VX, 0.0, None) / ml
        J += 0.35 * np.abs(_wrap(phi - WZ * 0.9))
        J -= 0.20 * VX / ml

        if np.any(feasible):
            Jf = np.where(feasible, J, np.inf)
            j = int(np.argmin(Jf))
        else:
            j = int(np.argmin(-dmin + 0.3 * np.abs(WZ) / ma + 0.4 * np.clip(VX, 0, None) / ml))
            if dmin[j] < 0.12:
                self.recover_until = max(self.recover_until, t + 0.7)

        vx = float(VX[j])
        vy = float(VY[j])
        wz = float(WZ[j])

        nrm = math.hypot(vx, vy)
        if nrm > ml:
            vx *= ml / nrm
            vy *= ml / nrm
        cmd = (float(np.clip(vx, -ml, ml)),
               float(np.clip(vy, -ml, ml)),
               float(np.clip(wz, -ma, ma)))
        self.last_cmd = cmd
        return Command(cmd[0], cmd[1], cmd[2])
