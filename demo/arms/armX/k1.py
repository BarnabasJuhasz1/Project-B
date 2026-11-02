# =============================================================================
#  Navigation controller for a Unitree Go2 shuttling between two fixed points
#  in an unmapped indoor space.  Twist-only output, 50 Hz, planar 360-deg lidar.
#
#  WHAT WAS TAKEN FROM THE PAPER
#  (X J Feng, J T Zhang, H Lin, K A Gillis, M R Moldover, "Determination of the
#   Boltzmann Constant Using the Differential-Cylindrical Procedure") AND WHERE:
#
#  1) The differential "deletion mechanism" (Sec. 3, Eqs. 7-11): when two
#     *matched* measurements carry the same perturbation, differencing them
#     deletes it -- "even if we do not recognize some imperfections, we still
#     know their perturbations will be deleted".  Used twice here:
#       (a) ARRIVAL-ANCHORED DRIFT DELETION.  Every time the harness switches
#           obs.goal, the robot is physically at the endpoint it just reached,
#           so (pose_estimate - that endpoint) is a direct measurement of the
#           otherwise unobservable pose-estimate drift.  The two shuttle
#           endpoints play the role of the paper's pair of cavities: they are a
#           matched pair, so the common-mode term cancels and the residual is
#           applied as a bias correction to the next target (self.bias).
#       (b) All motion / progress quantities are built from *differences* of
#           pose_estimate (never absolute pose), because a slowly varying
#           offset cancels in the difference (Eq. 9's ratio-of-perturbed-
#           readings structure).
#  2) EXCESS HALF-WIDTH AS A TEST OF THE MODEL, NOT OF THE DATA (Eq. 12,
#     Fig. 4): "the non-zero excess half-width actually measures the extent in
#     which the theoretical prediction differs from the measurement ... a
#     crucial assessment of the validity of theoretical admittance model".
#     Here the prediction is the displacement implied by the commands actually
#     sent, the measurement is the displacement observed; a large positive
#     excess means the free-space model is invalid (wedged, slipping, pushing a
#     wall) and triggers the recovery manoeuvre instead of more of the same.
#  3) THE THREE-TERM HALF-WIDTH FIT, Eq. 13, g = b0/sqrt(p) + b1 + b2*sqrt(p),
#     together with "increasing random errors with decreasing gas pressure"
#     (Fig. 5) and the low-pressure upwarping of Fig. 4 attributed to low
#     signal-to-noise: lidar hits are folded into the map with a weight
#     1/sigma(r), sigma(r) = c0/sqrt(r) + c1 + c2*sqrt(r), i.e. the same
#     three-term shape in range, and map updates are additionally de-weighted
#     when pose_error_estimate is large (that is the low-SNR regime).
#  4) THE NATURAL-VIBRATION CRITERION AND THE EVEN-MODE RULE (Sec. 5.1,
#     Fig. 3): measurements were kept below the 7.2 kHz criterion frequency and
#     restricted to even modes so the cavity's own vibration and free recoil
#     were never excited.  The steering loop monitors its own heading-sign-flip
#     rate; above a criterion rate it drops the yaw gain and slows its command
#     filter so it stops exciting the frozen locomotion layer, and all commands
#     are rate-limited for the same reason.
#  5) PROBING A RESONANCE "BY SCANNING NEARBY THE CERTAIN MODE" (Sec. 1): when
#     the robot believes it is at the goal but the harness has not switched
#     goals, it scans a slowly widening circle around the believed point rather
#     than sitting on a point that its own drift says is wrong.
#  6) MATCHED-CONDITION RATIO COMPARISON AND THE "C-B-A-C-A-B" SAMPLE ORDER
#     "for the purpose minimizing bias" (Eq. 3, Sec. 2.2; Table 2's repeated
#     runs and non-repeatability): candidate headings are compared only through
#     commensurate, normalised terms evaluated under identical conditions, and
#     the side chosen for a detour is alternated between recovery events rather
#     than held fixed, so a wrong turn is not made systematically.
# =============================================================================

import math
import time
from collections import deque

import numpy as np

from core.observation import Command

_TWO_PI = 2.0 * math.pi


def _wrap(a):
    """Wrap an angle (or array of angles) to [-pi, pi)."""
    return (a + math.pi) % _TWO_PI - math.pi


class Controller:
    # ------------------------------------------------------------------ setup
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(20261102)

        def _scal(name, default):
            try:
                v = float(getattr(scenario, name))
                if math.isfinite(v) and abs(v) > 1e-9:
                    return abs(v)
            except Exception:
                pass
            return default

        def _pt(name):
            try:
                v = np.asarray(getattr(scenario, name), dtype=float).ravel()
                if v.size >= 2 and bool(np.all(np.isfinite(v[:2]))):
                    return np.array([float(v[0]), float(v[1])])
            except Exception:
                pass
            return np.zeros(2)

        self.max_lin = min(_scal("max_lin", 0.8), 0.8)
        self.max_ang = min(_scal("max_ang", 1.0), 1.0)
        self.range_max = _scal("range_max", 6.0)
        self.time_limit = _scal("time_limit", 0.0)

        start = _pt("start")
        goal0 = _pt("goal")

        # ---- occupancy grid covering both endpoints with a generous margin.
        sep = float(np.hypot(*(goal0 - start)))
        margin = float(np.clip(0.9 * sep + 13.0, 15.0, 45.0))
        lo = np.minimum(start, goal0) - margin
        hi = np.maximum(start, goal0) + margin
        ext = np.maximum(hi - lo, 8.0)
        res = float(np.clip(float(np.max(ext)) / 400.0, 0.12, 0.30))
        nx = int(math.ceil(ext[0] / res))
        ny = int(math.ceil(ext[1] / res))
        nx += nx % 2
        ny += ny % 2
        nx = max(nx, 8)
        ny = max(ny, 8)

        self.res = res
        self.inv_res = 1.0 / res
        self.nx = nx
        self.ny = ny
        self.ox = float(lo[0])
        self.oy = float(lo[1])
        self.log = np.zeros((ny, nx), dtype=np.float32)

        # ---- coarse planning layer (2x2 blocks of the occupancy grid)
        self.pnx = nx // 2
        self.pny = ny // 2
        self.pres = 2.0 * res
        self.penalty = np.zeros((self.pny, self.pnx), dtype=np.float32)
        self.traversed = np.zeros((self.pny, self.pnx), dtype=bool)

        # ---- ray-casting sample distances and body footprint stencil
        self.ray_s = np.arange(0.0, self.range_max + res, 0.85 * res)
        fa = np.arange(0.0, _TWO_PI, _TWO_PI / 8.0)
        fx = [0.0]
        fy = [0.0]
        for rad in (0.13, 0.24):
            fx.extend((rad * np.cos(fa)).tolist())
            fy.extend((rad * np.sin(fa)).tolist())
        self.foot_x = np.asarray(fx)
        self.foot_y = np.asarray(fy)

        # ---- candidate motion directions in the body frame (6 deg spacing)
        self.cand = np.linspace(-math.pi, math.pi, 61)[:-1]
        self.cosd = None            # cos(cand - beam_angle), built lazily
        self.sind = None
        self.nbeam = -1

        # ---- episode state -------------------------------------------------
        self.k = 0
        self.t_prev = 0.0
        self.goal_prev = None
        self.bias = np.zeros(2)     # deletion-mechanism drift estimate
        self.n_arr = 0
        self.trips = 0

        self.path = None
        self.plan_t = -1e9
        self.plan_period = 0.5
        self.plan_ok = False
        self.force_replan = True

        self.th_f = 0.0
        self.th_prev = 0.0
        self.vx_prev = 0.0
        self.vy_prev = 0.0
        self.wz_prev = 0.0

        self.side = 1.0 if self.rng.random() < 0.5 else -1.0
        self.flip_hist = deque()
        self.last_sign = 0.0

        self.rec_phase = None
        self.rec_end = 0.0
        self.rec_away = 0.0
        self.rec_hold = 0.0

        self.contact_time = 0.0
        self.contact_frac = 0.0
        self.coll_count = 0
        self.clr = 0.31

        self.pred_acc = 0.0        # commanded displacement (prediction)
        self.meas_acc = 0.0        # observed displacement (measurement)
        self.win_t = 0.0
        self.excess_ema = 0.0
        self.pos_prev = None

        self.prog_t0 = None
        self.prog_d0 = None
        self.near_since = None
        self.orbit_phase = 0.0

        self.fail_count = 0

    # --------------------------------------------------------- grid utilities
    def _mark(self, xs, ys, delta):
        """Additive log-odds update at world points (duplicates applied once)."""
        if xs.size == 0:
            return
        ix = ((xs - self.ox) * self.inv_res).astype(np.int32)
        iy = ((ys - self.oy) * self.inv_res).astype(np.int32)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        if not bool(ok.any()):
            return
        f = (iy[ok].astype(np.int64) * self.nx + ix[ok].astype(np.int64))
        flat = self.log.reshape(-1)
        if delta < 0.0:
            flat[f] = np.maximum(flat[f] + np.float32(delta), np.float32(-3.0))
        else:
            flat[f] = np.minimum(flat[f] + np.float32(delta), np.float32(5.0))

    def _pidx(self, x, y):
        i = int((x - self.ox) / self.pres)
        j = int((y - self.oy) / self.pres)
        if i < 0:
            i = 0
        elif i > self.pnx - 1:
            i = self.pnx - 1
        if j < 0:
            j = 0
        elif j > self.pny - 1:
            j = self.pny - 1
        return i, j

    def _pen(self, x, y, rad, amt):
        i, j = self._pidx(x, y)
        kk = max(1, int(rad / self.pres))
        i0 = max(0, i - kk)
        i1 = min(self.pnx, i + kk + 1)
        j0 = max(0, j - kk)
        j1 = min(self.pny, j + kk + 1)
        self.penalty[j0:j1, i0:i1] += np.float32(amt)

    @staticmethod
    def _dilate(mask):
        out = mask.copy()
        out[:-1, :] |= mask[1:, :]
        out[1:, :] |= mask[:-1, :]
        out[:, :-1] |= mask[:, 1:]
        out[:, 1:] |= mask[:, :-1]
        return out

    @staticmethod
    def _clear_box(mask, i, j, kk):
        h, w = mask.shape
        mask[max(0, j - kk):min(h, j + kk + 1),
             max(0, i - kk):min(w, i + kk + 1)] = False

    @staticmethod
    def _bfs(free, gi, gj, ri, rj, maxit):
        """Boolean-frontier BFS (8-connected) from the goal cell."""
        dist = np.full(free.shape, -1, dtype=np.int32)
        if not free[gj, gi]:
            return dist, False
        fr = np.zeros(free.shape, dtype=bool)
        fr[gj, gi] = True
        dist[gj, gi] = 0
        if ri == gi and rj == gj:
            return dist, True
        for it in range(1, maxit + 1):
            nf = np.zeros_like(fr)
            nf[:-1, :] |= fr[1:, :]
            nf[1:, :] |= fr[:-1, :]
            nf[:, :-1] |= fr[:, 1:]
            nf[:, 1:] |= fr[:, :-1]
            nf[:-1, :-1] |= fr[1:, 1:]
            nf[1:, 1:] |= fr[:-1, :-1]
            nf[:-1, 1:] |= fr[1:, :-1]
            nf[1:, :-1] |= fr[:-1, 1:]
            nf &= free
            nf &= (dist < 0)
            if not bool(nf.any()):
                return dist, False
            dist[nf] = it
            fr = nf
            if dist[rj, ri] >= 0:
                return dist, True
        return dist, False

    def _trace(self, dist, ri, rj, maxlen=90):
        """Greedy descent of the BFS field, straightest-first tie breaking."""
        pts = []
        i, j = ri, rj
        d = int(dist[j, i])
        pdi, pdj = 0, 0
        h, w = dist.shape
        for _ in range(maxlen):
            pts.append((self.ox + (i + 0.5) * self.pres,
                        self.oy + (j + 0.5) * self.pres))
            if d <= 0:
                break
            best = None
            bk = None
            for dj in (-1, 0, 1):
                jj = j + dj
                if jj < 0 or jj >= h:
                    continue
                for di in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    ii = i + di
                    if ii < 0 or ii >= w:
                        continue
                    dv = int(dist[jj, ii])
                    if dv < 0 or dv >= d:
                        continue
                    key = (dv, -(di * pdi + dj * pdj))
                    if bk is None or key < bk:
                        bk = key
                        best = (ii, jj, dv, di, dj)
            if best is None:
                break
            i, j, d, pdi, pdj = best
        return pts

    # ---------------------------------------------------------------- planner
    def _plan(self, pos, target):
        t0 = time.perf_counter()
        occ = self.log > np.float32(0.9)
        pb = occ.reshape(self.pny, 2, self.pnx, 2).any(axis=3).any(axis=1)
        base = pb | ((self.penalty > np.float32(1.8)) & (~self.traversed))

        ri, rj = self._pidx(pos[0], pos[1])
        gi, gj = self._pidx(target[0], target[1])
        ncell = int(math.hypot(gi - ri, gj - rj))
        maxit = int(min(3 * ncell + 90, self.pnx + self.pny + 400, 1400))

        found = False
        for infl in (1, 0):
            blk = self._dilate(base) if infl else base.copy()
            self._clear_box(blk, ri, rj, 1)
            self._clear_box(blk, gi, gj, 1)
            dist, ok = self._bfs(~blk, gi, gj, ri, rj, maxit)
            if ok:
                pts = self._trace(dist, ri, rj)
                if len(pts) >= 1:
                    self.path = np.asarray(pts, dtype=float)
                    found = True
                    break
        if not found:
            self.path = None
        self.plan_ok = found
        el = time.perf_counter() - t0
        self.plan_period = float(min(max(30.0 * el, 0.30), 2.0))

    # ------------------------------------------------------------------- step
    def step(self, obs):
        try:
            return self._control(obs)
        except Exception:
            self.fail_count += 1
            s = 1.0 if (self.fail_count // 40) % 2 == 0 else -1.0
            return Command(0.15, 0.0, 0.35 * s)

    # ---------------------------------------------------------------- control
    def _control(self, obs):
        # ---- clock (differences only; see note 1b) -------------------------
        t = getattr(obs, "t", None)
        try:
            t = float(t)
        except Exception:
            t = self.t_prev + 0.02
        if not math.isfinite(t):
            t = self.t_prev + 0.02
        dt = t - self.t_prev
        if not (1e-4 < dt < 0.5):
            dt = 0.02
        self.t_prev = t
        self.k += 1

        # ---- pose / goal ---------------------------------------------------
        try:
            p = np.asarray(obs.pose_estimate, dtype=float).ravel()
            px, py, yaw = float(p[0]), float(p[1]), float(p[2])
            if not (math.isfinite(px) and math.isfinite(py) and math.isfinite(yaw)):
                raise ValueError
        except Exception:
            px, py, yaw = 0.0, 0.0, 0.0
        pos = np.array([px, py])

        try:
            perr = float(getattr(obs, "pose_error_estimate", 0.0))
            if not math.isfinite(perr):
                perr = 0.0
            perr = abs(perr)
        except Exception:
            perr = 0.0

        contact = bool(getattr(obs, "contact", False))
        self.contact_frac += 0.002 * ((1.0 if contact else 0.0) - self.contact_frac)

        try:
            g = np.asarray(obs.goal, dtype=float).ravel()
            goal = np.array([float(g[0]), float(g[1])])
            if not bool(np.all(np.isfinite(goal))):
                raise ValueError
        except Exception:
            goal = pos.copy()

        # ---- lidar ---------------------------------------------------------
        try:
            rng_raw = np.asarray(obs.ranges, dtype=float).ravel()
            bang = np.asarray(obs.range_angles, dtype=float).ravel()
            n = int(min(rng_raw.size, bang.size))
            if n < 4:
                raise ValueError
            rng_raw = rng_raw[:n]
            bang = bang[:n]
        except Exception:
            n = 8
            bang = np.linspace(-math.pi, math.pi, n, endpoint=False)
            rng_raw = np.full(n, self.range_max)
        finite = np.isfinite(rng_raw) & (rng_raw > 0.0)
        r = np.where(finite, rng_raw, self.range_max)
        r = np.nan_to_num(r, nan=self.range_max, posinf=self.range_max,
                          neginf=self.range_max)
        r = np.clip(r, 0.0, self.range_max)

        if self.cosd is None or self.nbeam != n:
            d = self.cand[:, None] - bang[None, :]
            self.cosd = np.cos(d)
            self.sind = np.sin(d)
            self.nbeam = n

        # ---- arrival-anchored drift deletion (paper note 1a) ---------------
        if self.goal_prev is None:
            self.goal_prev = goal.copy()
        elif float(np.hypot(*(goal - self.goal_prev))) > 0.75:
            b = pos - self.goal_prev
            lim = max(1.0, 2.0 * perr + 0.5)
            b = np.clip(b, -lim, lim)
            self.bias = b.copy() if self.n_arr == 0 else 0.55 * b + 0.45 * self.bias
            self.n_arr += 1
            self.trips += 1
            self.goal_prev = goal.copy()
            self.path = None
            self.force_replan = True
            self.near_since = None
            self.orbit_phase = float(self.rng.random()) * _TWO_PI
            self.prog_t0 = None
            self.rec_phase = None
            self.pred_acc = 0.0
            self.meas_acc = 0.0
            self.win_t = t

        target = goal + self.bias
        dgt = float(np.hypot(*(target - pos)))

        # ---- map update (Eq. 13 weighting, low-SNR gating) -----------------
        gate = 1.0 / (1.0 + (perr / 0.8) ** 2)
        gate = float(np.clip(gate, 0.20, 1.0))
        sel = slice(self.k % 2, None, 2)
        rs = r[sel]
        fs = finite[sel]
        aw = bang[sel] + yaw
        ca = np.cos(aw)
        sa = np.sin(aw)
        lim_f = np.maximum(rs - 0.18, 0.0)[:, None]
        m = self.ray_s[None, :] < lim_f
        if bool(m.any()):
            xs = px + ca[:, None] * self.ray_s[None, :]
            ys = py + sa[:, None] * self.ray_s[None, :]
            self._mark(xs[m], ys[m], -0.45 * gate)
        self._mark(px + self.foot_x, py + self.foot_y, -0.60)
        hit = fs & (rs < self.range_max - 0.10) & (rs > 0.16)
        if bool(hit.any()):
            rh = rs[hit]
            sig = 0.10 / np.sqrt(np.maximum(rh, 0.2)) + 0.04 + 0.05 * np.sqrt(rh)
            w = np.clip(0.16 / sig, 0.15, 1.10) * gate
            hx = px + ca[hit] * rh
            hy = py + sa[hit] * rh
            ix = ((hx - self.ox) * self.inv_res).astype(np.int32)
            iy = ((hy - self.oy) * self.inv_res).astype(np.int32)
            ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            if bool(ok.any()):
                f = (iy[ok].astype(np.int64) * self.nx + ix[ok].astype(np.int64))
                flat = self.log.reshape(-1)
                flat[f] = np.minimum(flat[f] + w[ok].astype(np.float32),
                                     np.float32(5.0))
        ti, tj = self._pidx(px, py)
        self.traversed[tj, ti] = True

        if self.k % 100 == 0:
            self.log *= np.float32(0.985)      # repeated runs beat stale data
            self.penalty *= np.float32(0.94)

        # ---- excess diagnostic (paper note 2) ------------------------------
        self.pred_acc += math.hypot(self.vx_prev, self.vy_prev) * dt
        if self.pos_prev is not None:
            self.meas_acc += math.hypot(px - self.pos_prev[0], py - self.pos_prev[1])
        self.pos_prev = (px, py)
        stuck = False
        if t - self.win_t > 1.2:
            if self.pred_acc > 0.30:
                excess = 1.0 - self.meas_acc / max(self.pred_acc, 1e-6)
                self.excess_ema = 0.6 * self.excess_ema + 0.4 * excess
                if excess > 0.55:
                    stuck = True
            self.win_t = t
            self.pred_acc = 0.0
            self.meas_acc = 0.0

        # ---- goal-arrival scan (paper note 5) ------------------------------
        if dgt < 0.60:
            if self.near_since is None:
                self.near_since = t
        else:
            self.near_since = None
        dwell = 0.0 if self.near_since is None else (t - self.near_since)
        if dwell > 6.0:
            self.bias *= (1.0 - 0.30 * dt)     # the anchor itself may be wrong

        # ---- long-horizon progress stall -----------------------------------
        if self.prog_t0 is None:
            self.prog_t0 = t
            self.prog_d0 = dgt
        elif t - self.prog_t0 > 6.0:
            if (self.prog_d0 - dgt) < 0.40 and dwell < 0.5 and self.rec_phase is None:
                self._pen(px + 0.85 * math.cos(yaw), py + 0.85 * math.sin(yaw),
                          0.55, 2.6)
                self.side = -self.side          # C-B-A-C-A-B style alternation
                self.force_replan = True
            self.prog_t0 = t
            self.prog_d0 = dgt

        # ---- contact / collision -------------------------------------------
        near = float(np.min(r)) if r.size else self.range_max
        a_near = float(bang[int(np.argmin(r))]) if r.size else 0.0
        if contact and self.contact_frac < 0.5:
            self.contact_time += dt
        else:
            self.contact_time = 0.0
        hit_now = self.contact_time > 0.08 and near < 0.80

        # ---- recovery finite-state machine ---------------------------------
        if self.rec_phase is not None and t >= self.rec_end:
            if self.rec_phase == "back":
                self.rec_phase = "turn"
                self.rec_end = t + 0.55 + 0.35 * float(self.rng.random())
            else:
                self.rec_phase = None
                self.rec_hold = t + 1.0
                self.force_replan = True

        if (hit_now or stuck) and self.rec_phase is None and t >= self.rec_hold:
            if hit_now:
                self.coll_count += 1
                if self.coll_count > 4:
                    self.clr = 0.36
            self._pen(px + 0.75 * math.cos(yaw), py + 0.75 * math.sin(yaw),
                      0.45, 2.2)
            self.side = -self.side
            self.rec_phase = "back"
            self.rec_end = t + 0.35
            self.rec_away = a_near if near < 1.2 else 0.0
            self.force_replan = True
            self.contact_time = 0.0

        if self.rec_phase is not None:
            if self.rec_phase == "back":
                vx = -0.38 * math.cos(self.rec_away)
                vy = -0.38 * math.sin(self.rec_away)
                wz = 0.45 * self.side
            else:
                vx = 0.10
                vy = 0.0
                wz = 0.95 * self.side
            return self._finalize(vx, vy, wz, dt, fast=True)

        # ---- replan --------------------------------------------------------
        need = (self.path is None or self.force_replan
                or (t - self.plan_t) > self.plan_period)
        if (not need) and self.path is not None and len(self.path) > 0:
            dd = np.hypot(self.path[:, 0] - px, self.path[:, 1] - py)
            if float(dd.min()) > 2.5:
                need = True
        if need:
            self.force_replan = False
            self.plan_t = t
            self._plan(pos, target)

        # ---- desired heading ------------------------------------------------
        aim = None
        if self.path is not None and len(self.path) > 0 and dgt > 1.2:
            dd = np.hypot(self.path[:, 0] - px, self.path[:, 1] - py)
            i0 = int(np.argmin(dd))
            look = 1.30
            j = i0
            while j + 1 < len(self.path):
                if math.hypot(self.path[j, 0] - px, self.path[j, 1] - py) >= look:
                    break
                j += 1
            aim = self.path[j]
        if aim is None:
            aim = target
        if dwell > 2.5:
            # scan a widening circle around the believed arrival point
            rad = min(2.2, 0.45 + 0.10 * (dwell - 2.5) + 0.8 * perr)
            self.orbit_phase += 1.2 * dt
            aim = target + rad * np.array([math.cos(self.orbit_phase),
                                           math.sin(self.orbit_phase)])

        dv = aim - pos
        if float(np.hypot(*dv)) < 1e-3:
            dv = target - pos
            if float(np.hypot(*dv)) < 1e-3:
                dv = np.array([math.cos(yaw), math.sin(yaw)])
        th_des = _wrap(math.atan2(dv[1], dv[0]) - yaw)

        # ---- matched-condition comparison of candidate headings ------------
        lon = self.cosd * r[None, :]
        lat = self.sind * r[None, :]
        blocking = (np.abs(lat) < self.clr) & (lon > -0.05)
        cfree = np.where(blocking, lon, self.range_max).min(axis=1)
        cfree = np.clip(cfree, 0.0, self.range_max)

        align = np.cos(self.cand - th_des)
        clear_t = np.clip((cfree - 0.45) / 2.0, 0.0, 1.0)
        smooth = np.cos(self.cand - self.th_prev)
        circ = self.side * np.sin(self.cand - th_des)
        score = (2.40 * align + 1.50 * clear_t + 0.45 * smooth + 0.20 * circ)
        score = score - 8.0 * (cfree < 0.55)
        b = int(np.argmax(score))

        if float(score[b]) < -4.0:
            # every direction is walled in: force a recovery turn
            self.rec_phase = "turn"
            self.rec_end = t + 0.7
            self.side = -self.side
            return self._finalize(-0.12 * math.cos(a_near),
                                  -0.12 * math.sin(a_near),
                                  0.95 * self.side, dt, fast=True)

        th = float(self.cand[b])
        dfree = float(cfree[b])

        # ---- oscillation (natural-vibration) guard, paper note 4 -----------
        s = 1.0 if th > 0.06 else (-1.0 if th < -0.06 else 0.0)
        if s != 0.0:
            if self.last_sign != 0.0 and s != self.last_sign:
                self.flip_hist.append(t)
            self.last_sign = s
        while self.flip_hist and (t - self.flip_hist[0]) > 1.5:
            self.flip_hist.popleft()
        osc = len(self.flip_hist)
        kw = 2.30 if osc < 4 else 1.10
        alpha = 0.45 if osc < 4 else 0.18

        self.th_f = _wrap(self.th_f + alpha * _wrap(th - self.th_f))
        self.th_prev = th

        # ---- speed ----------------------------------------------------------
        v = self.max_lin * float(np.clip((dfree - 0.35) / 1.10, 0.12, 1.0))
        v *= (0.55 + 0.45 * max(0.0, math.cos(th)))
        if osc >= 6:
            v *= 0.70
        if dgt < 0.9 and dwell < 2.5:
            v = min(v, max(0.22, 0.85 * self.max_lin))
        if contact and self.contact_frac < 0.5:
            v = min(v, 0.25)

        vx = v * math.cos(th)
        vy = float(np.clip(v * math.sin(th), -0.50, 0.50))
        wz = kw * self.th_f
        return self._finalize(vx, vy, wz, dt, fast=False)

    # ------------------------------------------------------------- output stage
    def _finalize(self, vx, vy, wz, dt, fast=False):
        for val in (vx, vy, wz):
            if not math.isfinite(val):
                vx, vy, wz = 0.0, 0.0, 0.3 * self.side
                break
        lin_lim = (8.0 if fast else 4.5) * dt
        ang_lim = (16.0 if fast else 10.0) * dt
        vx = self.vx_prev + float(np.clip(vx - self.vx_prev, -lin_lim, lin_lim))
        vy = self.vy_prev + float(np.clip(vy - self.vy_prev, -lin_lim, lin_lim))
        wz = self.wz_prev + float(np.clip(wz - self.wz_prev, -ang_lim, ang_lim))
        vx = float(np.clip(vx, -self.max_lin, self.max_lin))
        vy = float(np.clip(vy, -self.max_lin, self.max_lin))
        wz = float(np.clip(wz, -self.max_ang, self.max_ang))
        self.vx_prev = vx
        self.vy_prev = vy
        self.wz_prev = wz
        return Command(vx, vy, wz)
