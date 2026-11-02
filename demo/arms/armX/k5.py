# =============================================================================
#  Unitree Go2 shuttle-navigation controller
#
#  WHAT WAS TAKEN FROM THE PAPER
#  (Feng, Zhang, Lin, Gillis & Moldover, "Determination of the Boltzmann
#   Constant Using the Differential-Cylindrical Procedure"):
#
#  (P1) Differential deletion of common-mode perturbations (Sec. 3, Eqs. 6-11:
#       perturbations are *linearly additive* and linear in cavity length, so a
#       matched pair differenced against each other deletes them "without
#       depending on any correction calculation ... even if we do not recognise
#       some imperfections").  Transfer: every steering decision that can be
#       phrased as a difference of two *matched* measurements is taken that way
#       - left/right clearance integrals, and two look-ahead evaluations of the
#       global cost field - so that an unknown common-mode bias in the lidar or
#       in the map cancels to first order instead of being modelled.
#
#  (P2) The matched-pair condition L1:L2 = 2:1 with l1 = 2*l2 (Sec. 3, around
#       Eq. 11: the eigenfunctions are equal only when the ratio of coordinate
#       to length is equal, i.e. equal *sampling density*).  Transfer: the two
#       arms of every differential are built with length ratio 2:1 AND sample
#       count ratio 2:1 (8 beams over +-40 deg vs 16 beams over +-80 deg;
#       2 field samples over 1.1 m vs 4 samples over 2.2 m), so the arms are
#       directly differenceable.  Where the two arms disagree the differential
#       is distrusted (deletion condition not satisfied) and down-weighted.
#
#  (P3) Even modes only - free recoil is longitudinally symmetric for even
#       modes and therefore causes no disturbance, asymmetric for odd ones
#       (Sec. 5.1).  Transfer: the polar clearance profile is split into its
#       even (symmetric) and odd (antisymmetric) parts about the chosen
#       heading; the even part sets *speed* (no steering bias can leak in) and
#       the odd part is the only thing allowed to produce a centring steer.
#
#  (P4) "Accurate resonant measurements have to be kept distant away from the
#       natural vibrations of cavities" (Sec. 5.1, Fig. 3, >7.2 kHz).
#       Transfer: the command is never allowed to carry energy at the
#       frequencies the gait would couple to - hard slew limits on vx, vy, wz,
#       plus an oscillation detector that de-gains the yaw loop and adds
#       hysteresis when the steering command starts flipping sign.
#
#  (P5) Excess half-width as the validity test of the model (Eq. 12, Fig. 4:
#       the non-zero excess "measures the extent in which the theoretical
#       prediction differs from the measurement"; it up-warps where the
#       signal-to-noise is poor).  Transfer: an "excess" residual between what
#       the occupancy map predicts and what the lidar measures is tracked
#       continuously; it is the trust placed in the global map (gain of the
#       field term, inflation radius, speed cap, and when to start forgetting).
#
#  (P6) Acoustic virial expansion and extrapolation to the ideal-gas state
#       (Eq. 2, c^2 = A0 + A1 p + ...; and Eq. 13's low-pressure up-warp).
#       Transfer: the excess residual is fitted online in the commanded speed,
#       e(v) = e0 + e1 v + e2 v^2, and only the intercept e0 - the v -> 0
#       "unperturbed" value - is used as the map-trust statistic, so motion
#       artefacts are not mistaken for a broken map.
#
#  (P7) Sample C bridging the linkage of the KLPRR analysis to the KRISS
#       gravimetric primary standard (Sec. 2.1-2.2).  Transfer: the harness
#       switching obs.goal is a *primary standard* - at that instant the robot
#       truly is at the previous goal point - so each arrival re-anchors the
#       drifting pose estimate to it, bounding drift over the whole episode.
# =============================================================================

import math

import numpy as np

from core.observation import Command  # noqa: F401

_TWO_PI = 2.0 * math.pi


def _f(x, d):
    try:
        v = float(x)
        if math.isfinite(v):
            return v
    except Exception:
        pass
    return d


def _v2(x, d):
    try:
        a = np.asarray(x, dtype=float).ravel()
        if a.size >= 2 and bool(np.all(np.isfinite(a[:2]))):
            return np.array([float(a[0]), float(a[1])])
    except Exception:
        pass
    return np.asarray(d, dtype=float).ravel()[:2].copy()


def _wrap(a):
    return (a + math.pi) % _TWO_PI - math.pi


class Controller:
    """Reactive + mapping shuttle controller.  All state lives on the instance."""

    BIG = 1.0e6

    # ------------------------------------------------------------------ setup
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(20240501)

        self.max_lin = min(max(_f(getattr(scenario, "max_lin", 0.8), 0.8), 0.05), 2.0)
        self.max_ang = min(max(_f(getattr(scenario, "max_ang", 1.0), 1.0), 0.05), 4.0)
        rmax = _f(getattr(scenario, "range_max", 6.0), 6.0)
        self.range_max = rmax if 0.5 < rmax < 60.0 else 6.0
        self.time_limit = _f(getattr(scenario, "time_limit", 600.0), 600.0)

        self.p_start = _v2(getattr(scenario, "start", None), (0.0, 0.0))
        self.p_goal0 = _v2(getattr(scenario, "goal", None), self.p_start + np.array([1.0, 0.0]))

        # ---- occupancy / cost / potential grid -----------------------------
        span = float(np.hypot(*(self.p_goal0 - self.p_start)))
        half = max(14.0, 0.55 * span + 12.0)
        centre = 0.5 * (self.p_start + self.p_goal0)
        res = 0.30
        n = int(2.0 * half / res) + 2
        if n > 160:
            n = 160
            res = (2.0 * half) / float(n - 2)
        self.n = int(n)
        self.res = float(res)
        self.inv_res = 1.0 / self.res
        self.ox = float(centre[0] - 0.5 * self.n * self.res)
        self.oy = float(centre[1] - 0.5 * self.n * self.res)

        nn = self.n
        self.log = np.zeros((nn, nn), dtype=np.float32)      # occupancy log-odds
        self.visit = np.zeros((nn, nn), dtype=np.uint8)      # proven corridor
        self.bump = np.zeros((nn, nn), dtype=np.float32)     # learned blockages
        self.cost = None
        self.costd = None
        self.blocked = np.zeros((nn, nn), dtype=bool)
        self.goal_mask = np.zeros((nn, nn), dtype=bool)
        self.pot = None
        self.pot_build = None
        self._pad = np.empty((nn + 2, nn + 2), dtype=np.float32)
        self.build_done = False
        self.published = True
        self.cycle_sweeps = 0
        self.steps_since_rebuild = 10 ** 6
        self.force_rebuild = True

        # ---- candidate heading fan (robot frame) ---------------------------
        nc = 72
        self.nc = nc
        self.dcand = _TWO_PI / nc
        self.cand = (np.arange(nc, dtype=float) * self.dcand) - math.pi
        self.cand_c = np.cos(self.cand)
        self.cand_s = np.sin(self.cand)
        self._ang_key = None
        self._angd = None
        self.m8 = np.arange(1, 9)
        self.m16 = np.arange(1, 17)
        self.arm1 = np.array([0.55, 1.10])
        self.arm2 = np.array([0.55, 1.10, 1.65, 2.20])

        # ---- free-space ray sampling ---------------------------------------
        step = max(0.20, 0.72 * self.res)
        self.ds = np.arange(0.28, self.range_max + 1e-6, step)
        if self.ds.size == 0:
            self.ds = np.array([0.3])

        # ---- pose correction (map frame <- odom frame) ----------------------
        self.cor_t = np.zeros(2)
        self.cor_th = 0.0
        self.perr_max = 0.0

        # ---- episode bookkeeping -------------------------------------------
        self.k = 0
        self.last_t = None
        self.last_po = np.zeros(3)
        self.goal_pt = None
        self.trips = 0
        self.unknown_cost = 1.45

        # ---- behaviour state -----------------------------------------------
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.prev_dir_w = 0.0
        self.want_move = 0.0
        self.hist = []
        self.rec_t = 0.0
        self.rec_dir = 1.0
        self.rec_last_t = -99.0
        self.rec_count = 0
        self.contact_t = 0.0
        self.near_t = 0.0
        self.sweep_ph = 0.0
        self.sweeping = False
        self.wz_signs = []
        self.osc = False

        # ---- (P5)/(P6) excess residual + virial-style online fit ------------
        self.excess = 0.0
        self.e0 = 0.0
        self.trust = 1.0
        self.rls_A = np.eye(3) * 1e-3
        self.rls_b = np.zeros(3)
        self.last_forget = -99.0

    # -------------------------------------------------------------- utilities
    def _cell(self, x, y):
        i = int((x - self.ox) * self.inv_res)
        j = int((y - self.oy) * self.inv_res)
        if i < 0:
            i = 0
        elif i >= self.n:
            i = self.n - 1
        if j < 0:
            j = 0
        elif j >= self.n:
            j = self.n - 1
        return i, j

    def _cells(self, xs, ys):
        n = self.n
        ix = ((xs - self.ox) * self.inv_res).astype(np.int32)
        iy = ((ys - self.oy) * self.inv_res).astype(np.int32)
        np.clip(ix, 0, n - 1, out=ix)
        np.clip(iy, 0, n - 1, out=iy)
        return ix, iy

    def _pose(self, po):
        c = math.cos(self.cor_th)
        s = math.sin(self.cor_th)
        px = c * po[0] - s * po[1] + self.cor_t[0]
        py = s * po[0] + c * po[1] + self.cor_t[1]
        return px, py, _wrap(po[2] + self.cor_th)

    def _adjust(self, dx, dy, dth, pm, cap, po):
        """Compose a map-frame adjustment about pm; revert if it exceeds cap."""
        old_t = self.cor_t.copy()
        old_th = self.cor_th
        c = math.cos(dth)
        s = math.sin(dth)
        q = self.cor_t - pm
        self.cor_t = np.array([c * q[0] - s * q[1] + pm[0] + dx,
                               s * q[0] + c * q[1] + pm[1] + dy])
        self.cor_th = _wrap(self.cor_th + dth)
        px, py, _ = self._pose(po)
        if (math.hypot(px - po[0], py - po[1]) > cap) or (abs(self.cor_th) > 0.45):
            self.cor_t = old_t
            self.cor_th = old_th
            return False
        return True

    # ------------------------------------------------------------- 50 Hz tick
    def step(self, obs):
        try:
            return self._step(obs)
        except Exception:
            # never raise: creep forward slowly while turning
            return Command(0.05, 0.0, 0.25)

    def _step(self, obs):
        self.k += 1
        t = _f(getattr(obs, "t", 0.0), 0.0)
        if self.last_t is None:
            dt = 0.02
        else:
            dt = t - self.last_t
            if not (0.0005 < dt < 0.25):
                dt = 0.02
        self.last_t = t

        # --- scan ---------------------------------------------------------
        r = np.asarray(getattr(obs, "ranges", ()), dtype=float).ravel()
        a = np.asarray(getattr(obs, "range_angles", ()), dtype=float).ravel()
        if r.size < 3 or a.size != r.size:
            r = np.full(8, self.range_max)
            a = np.linspace(-math.pi, math.pi, 8, endpoint=False)
        key = (int(a.size), float(a[0]), float(a[-1]))
        if key != self._ang_key or self._angd is None:
            self._ang_key = key
            self._angd = np.abs(_wrap(a[:, None] - self.cand[None, :])).astype(np.float32)

        # --- pose ---------------------------------------------------------
        po = np.asarray(getattr(obs, "pose_estimate", None), dtype=float).ravel() \
            if getattr(obs, "pose_estimate", None) is not None else np.zeros(3)
        if po.size < 3 or not bool(np.all(np.isfinite(po[:3]))):
            po = self.last_po.copy()
        po = po[:3].astype(float)
        self.last_po = po.copy()

        perr = abs(_f(getattr(obs, "pose_error_estimate", 0.0), 0.0))
        if perr > 50.0:
            perr = 50.0
        self.perr_max = max(self.perr_max, perr)
        contact = bool(getattr(obs, "contact", False))

        # --- goal / arrival (P7: linkage to the primary standard) ---------
        g = _v2(getattr(obs, "goal", None),
                self.goal_pt if self.goal_pt is not None else self.p_goal0)
        if self.goal_pt is None:
            self.goal_pt = g.copy()
        elif float(np.hypot(g[0] - self.goal_pt[0], g[1] - self.goal_pt[1])) > 0.6:
            self.trips += 1
            self.unknown_cost = 1.80
            pm0 = np.array(self._pose(po)[:2])
            d = self.goal_pt - pm0
            if float(np.hypot(d[0], d[1])) > 1e-3:
                self._adjust(0.6 * d[0], 0.6 * d[1], 0.0, pm0,
                             5.0 + 2.0 * self.perr_max, po)
            self.goal_pt = g.copy()
            self.near_t = 0.0
            self.sweeping = False
            self.sweep_ph = 0.0
            self.force_rebuild = True
            self.hist = []

        px, py, yaw = self._pose(po)

        # --- scan geometry ------------------------------------------------
        finite = np.isfinite(r) & (r > 0.12)
        rr = np.where(finite, r, self.range_max)
        np.clip(rr, 0.05, self.range_max, out=rr)
        hit = finite & (r < self.range_max - 0.10)
        cw = np.cos(a + yaw)
        sw = np.sin(a + yaw)

        # --- (P5) excess residual: hits landing where the map says "free" --
        if np.any(hit):
            hx = px + cw[hit] * rr[hit]
            hy = py + sw[hit] * rr[hit]
            ix, iy = self._cells(hx, hy)
            lv = self.log[ix, iy]
            if lv.size >= 8 and t > 2.0:
                raw = float(np.mean(lv < -0.45))
                self.excess += 0.03 * (raw - self.excess)
                v_now = math.hypot(self.vx, self.vy)
                phi = np.array([1.0, v_now, v_now * v_now])
                self.rls_A *= 0.999
                self.rls_b *= 0.999
                self.rls_A += np.outer(phi, phi)
                self.rls_b += phi * raw
                if self.k % 25 == 0:
                    try:
                        sol = np.linalg.solve(self.rls_A + np.eye(3) * 1e-3, self.rls_b)
                        if np.all(np.isfinite(sol)):
                            # (P6) keep only the v -> 0 intercept
                            self.e0 = float(min(max(sol[0], 0.0), 1.0))
                        else:
                            self.e0 = self.excess
                    except Exception:
                        self.e0 = self.excess
        self.trust = float(min(max(1.0 - 1.35 * self.e0, 0.15), 1.0))

        # --- map update ---------------------------------------------------
        nn = self.n
        usable = rr - 0.30
        mask = self.ds[None, :] < usable[:, None]
        if np.any(mask):
            fx = (px + cw[:, None] * self.ds[None, :])[mask]
            fy = (py + sw[:, None] * self.ds[None, :])[mask]
            ix, iy = self._cells(fx, fy)
            cnt = np.bincount(ix.astype(np.int64) * nn + iy.astype(np.int64),
                              minlength=nn * nn).astype(np.float32)
            np.minimum(cnt, 2.0, out=cnt)
            self.log -= (0.13 * cnt).reshape(nn, nn)
        if np.any(hit):
            hx = px + cw[hit] * rr[hit]
            hy = py + sw[hit] * rr[hit]
            ix, iy = self._cells(hx, hy)
            cnt = np.bincount(ix.astype(np.int64) * nn + iy.astype(np.int64),
                              minlength=nn * nn).astype(np.float32)
            np.minimum(cnt, 3.0, out=cnt)
            self.log += (0.42 * cnt).reshape(nn, nn)
        np.clip(self.log, -2.5, 4.0, out=self.log)

        ci, cj = self._cell(px, py)
        if self.visit[ci, cj] < 250:
            self.visit[ci, cj] += 1

        if self.k % 150 == 0:
            self.log *= 0.99
        if self.e0 > 0.45 and (t - self.last_forget) > 6.0:
            self.log *= 0.85
            self.last_forget = t
            self.force_rebuild = True
        if self.k % 500 == 0:
            self.bump *= 0.9

        # --- (P1)/(P2) differential scan matching -------------------------
        if (self.k % 10 == 0) and t > 2.5 and np.any(hit):
            try:
                self._scan_match(px, py, yaw, a[hit], rr[hit], perr, po)
                px, py, yaw = self._pose(po)
                ci, cj = self._cell(px, py)
            except Exception:
                pass

        # --- global field --------------------------------------------------
        self._plan(px, py, g)

        # --- clearance fan -------------------------------------------------
        r_safe = 0.34 + 0.08 + 0.06 * (1.0 - self.trust)
        halfw = np.arctan2(r_safe, np.maximum(rr, 0.22)).astype(np.float32)
        blk = self._angd < halfw[:, None]
        clr = np.where(blk, rr[:, None].astype(np.float32),
                       np.float32(self.range_max)).min(axis=0)
        trav = np.maximum(clr.astype(float) - 0.26, 0.0)

        # --- recovery arbitration -----------------------------------------
        if contact:
            self.contact_t += dt
        else:
            self.contact_t = max(0.0, self.contact_t - 0.5 * dt)

        if self.k % 10 == 0:
            self.hist.append((t, px, py))
            if len(self.hist) > 12:
                self.hist.pop(0)

        stuck = False
        if self.rec_t <= 0.0 and len(self.hist) >= 9:
            t0, x0, y0 = self.hist[0]
            if (t - t0) > 1.7 and math.hypot(px - x0, py - y0) < 0.26 \
                    and self.want_move > 0.22:
                stuck = True
        if self.rec_t <= 0.0 and (stuck or self.contact_t > 0.28
                                  or float(np.max(trav)) < 0.18):
            self.rec_t = 1.10
            self.rec_count = self.rec_count + 1 if (t - self.rec_last_t) < 7.0 else 1
            self.rec_last_t = t
            left = float(np.sum(trav[(self.cand > 0.35) & (self.cand < 1.75)]))
            right = float(np.sum(trav[(self.cand < -0.35) & (self.cand > -1.75)]))
            if abs(left - right) < 0.4:
                self.rec_dir = 1.0 if self.rng.random() < 0.5 else -1.0
            else:
                self.rec_dir = 1.0 if left > right else -1.0
            if self.rec_count >= 2:
                self.rec_dir = -self.rec_dir
                self.rec_t = 1.5
            kmin = int(np.argmin(rr))
            self._mark_bump(px, py, yaw + float(a[kmin]),
                            2.0 + 1.5 * min(self.rec_count, 4))
            self.hist = []
            self.contact_t = 0.0

        # ------------------------------------------------------------------
        if self.rec_t > 0.0:
            self.rec_t -= dt
            vxd, vyd, wzd = -0.32, 0.0, self.rec_dir * 0.85
            if self.rec_t > 0.80:
                wzd *= 0.25
            self.want_move += 0.1 * (0.3 - self.want_move)
        else:
            vxd, vyd, wzd = self._navigate(px, py, yaw, g, trav, dt, t)

        # --- (P4) slew limiting: keep command energy out of the gait band --
        kv = 0.055
        self.vx += float(np.clip(vxd - self.vx, -kv, kv))
        self.vy += float(np.clip(vyd - self.vy, -0.045, 0.045))
        self.wz += float(np.clip(wzd - self.wz, -0.13, 0.13))
        self.vx = float(np.clip(self.vx, -self.max_lin, self.max_lin))
        self.vy = float(np.clip(self.vy, -self.max_lin, self.max_lin))
        self.wz = float(np.clip(self.wz, -self.max_ang, self.max_ang))

        self.wz_signs.append(1 if self.wz > 0.12 else (-1 if self.wz < -0.12 else 0))
        if len(self.wz_signs) > 30:
            self.wz_signs.pop(0)
        flips = 0
        prev = 0
        for s in self.wz_signs:
            if s != 0:
                if prev != 0 and s != prev:
                    flips += 1
                prev = s
        self.osc = flips >= 6

        return Command(self.vx, self.vy, self.wz)

    # ------------------------------------------------------------- behaviour
    def _navigate(self, px, py, yaw, g, trav, dt, t):
        dist = math.hypot(g[0] - px, g[1] - py)

        # ---- near-goal probing sweep (pose may be biased; arrival is truth)
        if dist < 0.45:
            self.near_t += dt
        else:
            self.near_t = max(0.0, self.near_t - 1.5 * dt)
        if self.near_t > 1.2:
            self.sweeping = True
        if self.sweeping and dist > 2.4:
            self.sweeping = False
            self.near_t = 0.0

        if self.sweeping:
            self.sweep_ph += 1.3 * dt
            rad = min(0.35 + 0.20 * (self.sweep_ph / _TWO_PI), 1.70)
            tgt = np.array([g[0] + rad * math.cos(self.sweep_ph),
                            g[1] + rad * math.sin(self.sweep_ph)])
            th_des = math.atan2(tgt[1] - py, tgt[0] - px)
            w_pot = 0.0
            w_goal = 2.6
            p0 = None
            G = np.zeros(self.nc)
        else:
            th_des, p0, G = self._field_terms(px, py, yaw, g)
            w_pot = 0.4 + 1.7 * self.trust
            w_goal = 1.6

        th_des_r = _wrap(th_des - yaw)
        dth = _wrap(self.cand - th_des_r)
        hyst = 1.15 if self.osc else 0.55
        prev_r = _wrap(self.prev_dir_w - yaw)

        score = (w_goal * np.cos(dth)
                 + w_pot * np.clip(-G, -3.0, 3.0)
                 + 0.85 * np.minimum(trav, 2.5) / 2.5
                 + hyst * np.cos(_wrap(self.cand - prev_r)))
        score[trav < 0.60] -= 1.6
        score[trav < 0.30] -= 9.0

        k0 = int(np.argmax(score))

        # ---- (P2)/(P3) even/odd decomposition about the chosen heading -----
        s8p = np.take(trav, k0 + self.m8, mode="wrap")
        s8m = np.take(trav, k0 - self.m8, mode="wrap")
        s16p = np.take(trav, k0 + self.m16, mode="wrap")
        s16m = np.take(trav, k0 - self.m16, mode="wrap")
        a8 = 0.5 * float(np.mean(s8p - s8m))      # odd part, short arm
        a16 = 0.5 * float(np.mean(s16p - s16m))   # odd part, 2x arm, 2x samples
        sym = float(np.mean(0.5 * (s8p + s8m)))   # even part only
        if a8 * a16 > 0.0:
            asym = 0.5 * (a8 + a16)               # deletion condition satisfied
        else:
            asym = 0.5 * a8                       # arms disagree -> distrust
        th_center = 0.0
        if (not self.sweeping) and dist > 1.2 and trav[k0] < 4.0:
            th_center = float(np.clip(0.20 * asym, -0.30, 0.30))
            if self.osc:
                th_center *= 0.4

        kc = int(round(k0 + th_center / self.dcand))
        win = np.take(trav, np.arange(kc - 3, kc + 4), mode="wrap")
        d_ahead = float(np.min(win))

        th_cmd = _wrap(float(self.cand[k0]) + th_center)

        f_ahead = float(np.clip((d_ahead - 0.22) / 0.85, 0.0, 1.0))
        f_sym = float(np.clip(sym / 2.0, 0.25, 1.0))
        v = self.max_lin * min(f_ahead, 0.50 + 0.50 * f_sym)
        v *= (0.78 + 0.22 * self.trust)
        v = min(v, 0.35 + 0.45 * dist)
        if self.sweeping:
            v = min(v, 0.42 * self.max_lin)
        if abs(th_cmd) > 1.75:
            v *= 0.16
        elif abs(th_cmd) > 0.90:
            v *= 0.55
        if d_ahead < 0.26:
            v = min(v, 0.08)

        kyaw = 1.75 * (0.6 if self.osc else 1.0)
        wzd = float(np.clip(kyaw * th_cmd, -self.max_ang, self.max_ang))
        vxd = v * math.cos(th_cmd)
        vyd = 0.60 * v * math.sin(th_cmd)

        self.prev_dir_w = _wrap(yaw + th_cmd)
        self.want_move += 0.08 * (v - self.want_move)
        return vxd, vyd, wzd

    # -------------------------------------------------- global cost field use
    def _field_terms(self, px, py, yaw, g):
        """Desired heading plus the 2:1 matched-arm field differential."""
        nc = self.nc
        G = np.zeros(nc)
        th_goal = math.atan2(g[1] - py, g[0] - px)
        if self.pot is None:
            return th_goal, None, G

        i0, j0 = self._cell(px, py)
        p0 = float(self.pot[i0, j0])
        if p0 >= 0.5 * self.BIG:
            lo_i, hi_i = max(0, i0 - 4), min(self.n, i0 + 5)
            lo_j, hi_j = max(0, j0 - 4), min(self.n, j0 + 5)
            sub = self.pot[lo_i:hi_i, lo_j:hi_j]
            k = int(np.argmin(sub))
            bi, bj = np.unravel_index(k, sub.shape)
            if float(sub[bi, bj]) < 0.5 * self.BIG:
                tx = self.ox + (lo_i + bi + 0.5) * self.res
                ty = self.oy + (lo_j + bj + 0.5) * self.res
                if math.hypot(tx - px, ty - py) > 0.2 * self.res:
                    return math.atan2(ty - py, tx - px), None, G
            return th_goal, None, G

        c, s = math.cos(yaw), math.sin(yaw)
        wdx = self.cand_c * c - self.cand_s * s
        wdy = self.cand_c * s + self.cand_s * c

        def arm(dists):
            xs = (px + wdx[:, None] * dists[None, :]).ravel()
            ys = (py + wdy[:, None] * dists[None, :]).ravel()
            ix, iy = self._cells(xs, ys)
            vals = self.pot[ix, iy].astype(float)
            vals = np.where(vals >= 0.5 * self.BIG, p0 + 6.0, vals)
            return vals.reshape(nc, dists.size).mean(axis=1)

        P1 = arm(self.arm1)
        P2 = arm(self.arm2)
        G1 = (P1 - p0) / float(np.mean(self.arm1))
        G2 = (P2 - p0) / float(np.mean(self.arm2))
        Gm = 0.5 * (G1 + G2)
        G = np.where(G1 * G2 > 0.0, Gm, 0.5 * Gm)

        kbest = int(np.argmin(G))
        th_des = _wrap(yaw + float(self.cand[kbest]))
        # blend with the straight-line bearing so the fan is not over-committed
        th_des = _wrap(th_des + 0.35 * _wrap(th_goal - th_des))
        return th_des, p0, G

    # ----------------------------------------------------------- planner core
    def _plan(self, px, py, g):
        need = (self.cost is None) or self.force_rebuild \
            or (self.published and self.steps_since_rebuild >= 40)
        if need:
            self._rebuild_cost(px, py, g)
            self.force_rebuild = False
            self.steps_since_rebuild = 0
            self.cycle_sweeps = 0
            self.build_done = False
            self.published = False
        self.steps_since_rebuild += 1

        if not self.published:
            k = 200 if self.pot is None else 7
            self._relax(k)
            self.cycle_sweeps += k
            if self.build_done or self.cycle_sweeps >= 430:
                self.pot = self.pot_build
                self.published = True

    def _rebuild_cost(self, px, py, g):
        nn = self.n
        occ = self.log > 0.9
        gi, gj = self._cell(g[0], g[1])
        ri, rj = self._cell(px, py)
        for (ii, jj, rad) in ((gi, gj, 2), (ri, rj, 2)):
            occ[max(0, ii - rad):ii + rad + 1, max(0, jj - rad):jj + rad + 1] = False

        d1 = self._dilate(occ)
        d2 = self._dilate(d1)
        base = np.full((nn, nn), self.unknown_cost, dtype=np.float32)
        base[self.log < -0.35] = 1.0
        base[self.visit > 0] = 0.82          # proven corridor from earlier trips
        ring1 = d1 & ~occ
        ring2 = d2 & ~d1
        base += 2.6 * ring1.astype(np.float32)
        base += 0.9 * ring2.astype(np.float32)
        base += self.bump
        cost = base * np.float32(self.res)
        cost[occ] = np.float32(self.BIG)
        self.cost = cost
        self.costd = (1.41421356 * cost).astype(np.float32)
        self.blocked = occ

        gm = np.zeros((nn, nn), dtype=bool)
        gm[max(0, gi - 1):gi + 2, max(0, gj - 1):gj + 2] = True
        self.goal_mask = gm

        P = np.full((nn, nn), np.float32(self.BIG))
        P[gm] = 0.0
        self.pot_build = P

    @staticmethod
    def _dilate(b):
        out = b.copy()
        out[1:, :] |= b[:-1, :]
        out[:-1, :] |= b[1:, :]
        out[:, 1:] |= b[:, :-1]
        out[:, :-1] |= b[:, 1:]
        return out

    def _relax(self, sweeps):
        P = self.pot_build
        C = self.cost
        Cd = self.costd
        pad = self._pad
        big = np.float32(self.BIG)
        for it in range(sweeps):
            pad.fill(big)
            pad[1:-1, 1:-1] = P
            m4 = np.minimum(np.minimum(pad[:-2, 1:-1], pad[2:, 1:-1]),
                            np.minimum(pad[1:-1, :-2], pad[1:-1, 2:]))
            md = np.minimum(np.minimum(pad[:-2, :-2], pad[:-2, 2:]),
                            np.minimum(pad[2:, :-2], pad[2:, 2:]))
            nxt = np.minimum(m4 + C, md + Cd)
            np.minimum(nxt, P, out=nxt)
            np.minimum(nxt, big, out=nxt)
            nxt[self.goal_mask] = 0.0
            if (it & 3) == 3 or it == sweeps - 1:
                if float(np.max(P - nxt)) < 1e-3:
                    P = nxt
                    self.build_done = True
                    break
            P = nxt
        self.pot_build = P

    # ------------------------------------------- differential scan alignment
    def _scan_match(self, px, py, yaw, ah, rh, perr, po):
        if ah.size < 12:
            return
        stride = max(1, int(ah.size // 60))
        ah = ah[::stride]
        rh = rh[::stride]
        ex = rh * np.cos(ah)
        ey = rh * np.sin(ah)
        L = self.log

        def sc(dx, dy, dth):
            c = math.cos(yaw + dth)
            s = math.sin(yaw + dth)
            xs = px + dx + c * ex - s * ey
            ys = py + dy + s * ex + c * ey
            ix, iy = self._cells(xs, ys)
            return float(np.sum(np.maximum(L[ix, iy], 0.0)))

        base = sc(0.0, 0.0, 0.0)
        if base < 2.0:
            return
        offs = (-1, 0, 1)
        best = []
        for scale in (1.0, 2.0):
            st = scale * self.res
            sa = scale * 0.030
            bs, bo = base, (0.0, 0.0, 0.0)
            for oi in offs:
                for oj in offs:
                    for ok in offs:
                        if oi == 0 and oj == 0 and ok == 0:
                            continue
                        v = sc(oi * st, oj * st, ok * sa)
                        if v > bs:
                            bs, bo = v, (oi * st, oj * st, ok * sa)
            best.append((bs, bo))

        (s1, o1), (s2, o2) = best
        if s1 <= base * 1.02 + 0.5:
            return
        # (P1) matched-pair deletion condition: the two scales must agree
        agree = True
        for u, w in zip(o1, o2):
            if u * w < 0.0:
                agree = False
        gain = 0.30 if agree else 0.12
        pm = np.array([px, py])
        cap = 1.0 + 2.5 * max(self.perr_max, perr)
        self._adjust(gain * o1[0], gain * o1[1], gain * o1[2], pm, cap, po)

    # ------------------------------------------------------------- blockages
    def _mark_bump(self, px, py, bearing_w, amount):
        bx = px + 0.45 * math.cos(bearing_w)
        by = py + 0.45 * math.sin(bearing_w)
        i, j = self._cell(bx, by)
        self.bump[max(0, i - 1):i + 2, max(0, j - 1):j + 2] += np.float32(amount)
        np.clip(self.bump, 0.0, 14.0, out=self.bump)
        self.log[i, j] = min(4.0, float(self.log[i, j]) + 1.2)
        self.force_rebuild = True
