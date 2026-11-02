# =============================================================================
# Unitree Go2 shuttle navigation controller.
#
# WHAT WAS TAKEN FROM THE PAPER
# (X J Feng, J T Zhang, H Lin, K A Gillis, M R Moldover, "Determination of the
#  Boltzmann Constant Using the Differential-Cylindrical Procedure") and where
# in it that came from:
#
# 1. The differential "deletion mechanism" (Sec. 3, Eqs. 6-11): a perturbation
#    that acts identically on two *matched* channels cancels in their
#    difference, so it never has to be modelled at all -- "even if we do not
#    recognize some imperfections, we still know their perturbations will be
#    deleted under the differential procedure". Used twice here:
#      (a) corridor centring from mirrored, matched lidar sector pairs whose
#          angular spans are in the 2:1 ratio the paper demands of its pair of
#          cavities (L1:L2 = 2:1 with l1 = 2*l2, Sec. 3 / Sec. 5.1), each pair
#          differenced in the *normalised* form (rL-rR)/(rL+rR) so that a
#          common multiplicative range bias deletes exactly, and the
#          disagreement between the two nested pairs used as a confidence
#          (Eq. 11: the residual L12/L12 vanishes only when the two channels
#          really are matched);
#      (b) yaw-drift estimation from two independent estimates of the same
#          displacement (dead-reckoned vs. the known endpoint-to-endpoint
#          vector); their angular difference deletes the common translation
#          bias, exactly as Eq. (9) deletes the common admittance.
# 2. Linkage to a primary standard through a bridging sample (Sec. 2.1:
#    "Sample C bridged a linkage of the analysis in KLPRR CAS to the primary
#    gravimetric standard of KRISS"). Every arrival -- i.e. every time the
#    harness switches obs.goal -- is a calibration against a point whose
#    coordinates are known exactly, and is used to re-anchor the drifting pose
#    estimate (translation and, differentially, rotation).
# 3. Ratio comparison at a fixed operating point (Eq. 3, M1/M2 = c2^2/c1^2,
#    "pinned at 300 kPa ... the minimum measurement inconsistency"): achieved
#    motion is compared to commanded motion as a dimensionless ratio, which
#    deletes unknown common gains in the frozen locomotion layer; the ratio,
#    not the absolute speed, drives stall detection.
# 4. Excess half-width as a check on the *model* rather than a correction
#    (Eq. 12, Sec. 5.1: the non-zero excess half-width "measures the extent in
#    which the theoretical prediction differs from the measurement"). The
#    commanded-vs-observed motion residual is kept as an "excess" statistic; it
#    corrects nothing, it grades the pose/locomotion model and gates how much
#    the map and the pose estimate are trusted.
# 5. Keep the excitation away from structural resonance, and use only the
#    symmetric (even) modes (Sec. 5.1: the 7.2 kHz criterion, and "free recoils
#    ... are longitudinally symmetric for even modes so to cause no disturbance
#    ... asymmetric for odd modes so to yield disturbances"). Commands are
#    rate-limited and low-pass filtered, and the steering cost is
#    mirror-symmetric with hysteresis, so the controller never drives the gait
#    with an asymmetric steering limit cycle.
# 6. Repeated runs reduce dispersion, and the stated uncertainty tells you how
#    wide to look (Sec. 2.2 / Table 2, averaging over >10 runs per sample;
#    Fig. 4, the low-SNR "upwarping" where the signal is weak). The shuttle
#    traverses the same building many times, so route evidence is accumulated
#    across completed trips and discounts the cost of already-proven corridors;
#    and the end-point search spiral is sized directly by the reported
#    pose_error_estimate instead of a fixed guess.
# =============================================================================

import math

import numpy as np

from core.observation import Command

_BIG = np.float32(1e9)
_TWO_PI = 2.0 * math.pi


def _wrap(a):
    """Wrap angle(s) to (-pi, pi]."""
    return (np.asarray(a, dtype=float) + math.pi) % _TWO_PI - math.pi


def _wrapf(a):
    return (float(a) + math.pi) % _TWO_PI - math.pi


def _as_xy(v, default=(0.0, 0.0)):
    try:
        arr = np.asarray(v, dtype=float).ravel()
        if arr.size >= 2 and np.all(np.isfinite(arr[:2])):
            return np.array([float(arr[0]), float(arr[1])])
    except Exception:
        pass
    return np.array([float(default[0]), float(default[1])])


def _as_float(v, default):
    try:
        f = float(v)
        if math.isfinite(f):
            return f
    except Exception:
        pass
    return float(default)


class Controller:
    """Reactive + incrementally-planned shuttle controller for a Go2."""

    # ------------------------------------------------------------------ reset
    def reset(self, scenario, rng):
        self.rng = rng if rng is not None else np.random.default_rng(0)

        self.range_max = _as_float(getattr(scenario, "range_max", 6.0), 6.0)
        if self.range_max <= 0.5:
            self.range_max = 6.0
        self.max_lin = abs(_as_float(getattr(scenario, "max_lin", 0.8), 0.8)) or 0.8
        self.max_ang = abs(_as_float(getattr(scenario, "max_ang", 1.0), 1.0)) or 1.0
        self.time_limit = _as_float(getattr(scenario, "time_limit", 600.0), 600.0)

        start = _as_xy(getattr(scenario, "start", (0.0, 0.0)))
        goal0 = _as_xy(getattr(scenario, "goal", (0.0, 0.0)), start)
        self.endpoints = [start.copy(), goal0.copy()]

        # ---- grid geometry (adaptive resolution, capped cell count) --------
        span = float(np.linalg.norm(goal0 - start))
        margin = float(np.clip(0.7 * span + 12.0, 14.0, 32.0))
        lo = np.minimum(start, goal0) - margin
        hi = np.maximum(start, goal0) + margin
        ext = hi - lo
        self.res = float(max(0.35, np.max(ext) / 140.0))
        self.origin = lo.astype(float)
        self.nx = int(np.clip(math.ceil(ext[0] / self.res) + 2, 24, 150))
        self.ny = int(np.clip(math.ceil(ext[1] / self.res) + 2, 24, 150))
        shp = (self.nx, self.ny)

        self.lo_grid = np.zeros(shp, dtype=np.float32)      # occupancy log-odds
        self.visits = np.zeros(shp, dtype=np.float32)       # proven-route memory
        self.trap = np.zeros(shp, dtype=np.float32)         # stuck penalty layer
        self.blocked = np.zeros(shp, dtype=bool)
        self.cellcost = np.full(shp, np.float32(self.res), dtype=np.float32)

        self._pad = np.full((self.nx + 2, self.ny + 2), _BIG, dtype=np.float32)
        self._padb = np.zeros((self.nx + 2, self.ny + 2), dtype=bool)

        self.infl = max(1, int(round(0.34 / self.res)))
        self.need = int(np.clip(0.85 * (self.nx + self.ny), 60, 300))

        # ---- pose anchoring (paper item 2) --------------------------------
        self.anchor_raw = start.copy()
        self.anchor_true = start.copy()
        self.yaw_bias = 0.0
        self.rot = np.array([[1.0, 0.0], [0.0, 1.0]])
        self.last_anchor_true = None
        self.last_anchor_corr = None

        # ---- planner fields, one per endpoint -----------------------------
        self.fields = {}
        for p in self.endpoints:
            self._make_field(p, warm=True)

        # ---- reactive layer tables ---------------------------------------
        n_cand = 60
        self.cand = np.linspace(-math.pi, math.pi, n_cand + 1)[:-1]
        self.cand_cos = np.cos(self.cand)
        self.cand_sin = np.sin(self.cand)
        self.half_w = 0.29
        self.d_cap = 2.6

        self.ring_dirs = np.linspace(-math.pi, math.pi, 25)[:-1]
        self.ring_rad = np.array([0.55, 1.1, 1.8, 2.7])
        self.ring_ox = (np.cos(self.ring_dirs)[:, None] * self.ring_rad[None, :]).ravel()
        self.ring_oy = (np.sin(self.ring_dirs)[:, None] * self.ring_rad[None, :]).ravel()
        self.ring_cost = np.tile(self.ring_rad, self.ring_dirs.size)
        self.n_ring = self.ring_dirs.size
        self.n_rad = self.ring_rad.size

        self.samp = np.arange(0.3, self.range_max + 1e-6, 0.32)

        # ---- episode state ------------------------------------------------
        self.k = 0
        self.t_prev = None
        self.prev_goal = None
        self.trips = 0
        self.pos = start.copy()
        self.pos_prev = start.copy()
        self.yaw = 0.0
        self.last_cmd = np.zeros(3)
        self.theta_prev = 0.0
        self.mode = 0          # 0 nav, 1 backup, 2 commit-slide, 3 probe
        self.mode_t = 0.0
        self.esc_sign = 1.0
        self.esc_count = 0
        self.stall_t = 0.0
        self.v_act = 0.0
        self.v_cmd = 0.0
        self.excess = 0.0          # paper item 4
        self.track_ratio = 1.0     # paper item 3
        self.pose_err = 0.0
        self.goal_dwell = 0.0
        self.probe_t = 0.0
        self.probe_phase = float(self.rng.uniform(-math.pi, math.pi))
        self.best_dist = None
        self.best_dist_t = 0.0
        self.trail = []
        self.active_key = None
        self.fail = 0
        self.contact_t = 0.0

    # ------------------------------------------------------- field machinery
    def _key(self, g):
        return (round(float(g[0]), 1), round(float(g[1]), 1))

    def _cell(self, p):
        i = int((float(p[0]) - self.origin[0]) / self.res)
        j = int((float(p[1]) - self.origin[1]) / self.res)
        return (int(np.clip(i, 0, self.nx - 1)), int(np.clip(j, 0, self.ny - 1)))

    def _make_field(self, g, warm=False):
        gi, gj = self._cell(g)
        work = np.full((self.nx, self.ny), _BIG, dtype=np.float32)
        work[gi, gj] = 0.0
        live = np.full((self.nx, self.ny), _BIG, dtype=np.float32)
        live[gi, gj] = 0.0
        fd = {"goal": np.asarray(g, dtype=float).copy(), "gi": gi, "gj": gj,
              "work": work, "live": live, "it": 0}
        self.fields[self._key(g)] = fd
        if warm:
            self._sweep(fd, self.need)
            fd["live"] = fd["work"].copy()
            fd["it"] = 0
        return fd

    def _find_field(self, g):
        key = self._key(g)
        fd = self.fields.get(key)
        if fd is not None:
            return fd
        best, bd = None, 1e18
        for f in self.fields.values():
            d = float(np.linalg.norm(f["goal"] - g))
            if d < bd:
                best, bd = f, d
        if best is not None and bd < 0.8:
            return best
        if len(self.fields) >= 4:
            self.fields.pop(next(iter(self.fields)))
        return self._make_field(g, warm=False)

    def _sweep(self, fd, iters):
        """Anytime value iteration on the optimistic (unknown == free) field."""
        D = fd["work"]
        P = self._pad
        cost = self.cellcost
        blk = self.blocked
        gi, gj = fd["gi"], fd["gj"]
        r2 = np.float32(math.sqrt(2.0))
        for _ in range(int(iters)):
            P[1:-1, 1:-1] = D
            a = np.minimum(np.minimum(P[0:-2, 1:-1], P[2:, 1:-1]),
                           np.minimum(P[1:-1, 0:-2], P[1:-1, 2:]))
            b = np.minimum(np.minimum(P[0:-2, 0:-2], P[0:-2, 2:]),
                           np.minimum(P[2:, 0:-2], P[2:, 2:]))
            cand = np.minimum(a + cost, b + r2 * cost)
            np.minimum(D, cand, out=D)
            D[blk] = _BIG
            D[gi, gj] = np.float32(0.0)
        fd["it"] += int(iters)
        if fd["it"] >= self.need:
            fd["live"] = D
            nw = np.full((self.nx, self.ny), _BIG, dtype=np.float32)
            nw[gi, gj] = 0.0
            fd["work"] = nw
            fd["it"] = 0

    # ------------------------------------------------------------- mapping
    def _rebuild_cost(self):
        occ = self.lo_grid > 0.55
        P = self._padb
        b = occ
        for _ in range(self.infl):
            P[1:-1, 1:-1] = b
            b = (P[0:-2, 1:-1] | P[2:, 1:-1] | P[1:-1, 0:-2] | P[1:-1, 2:] | b)
        blocked = b.copy()
        P[1:-1, 1:-1] = blocked
        near = (P[0:-2, 1:-1] | P[2:, 1:-1] | P[1:-1, 0:-2] | P[1:-1, 2:] | blocked)

        pen = np.ones((self.nx, self.ny), dtype=np.float32)
        pen += np.float32(1.9) * near.astype(np.float32)
        pen += self.trap
        pen -= np.float32(0.25) * np.minimum(self.visits, 3.0)
        np.clip(pen, 0.55, 40.0, out=pen)
        self.cellcost = (np.float32(self.res) * pen).astype(np.float32)

        # never let inflation seal the robot or an endpoint
        self.blocked = blocked
        ri, rj = self._cell(self.pos)
        self._unblock(ri, rj, 1)
        for fd in self.fields.values():
            self._unblock(fd["gi"], fd["gj"], 1)

    def _unblock(self, i, j, r):
        i0 = max(0, i - r); i1 = min(self.nx, i + r + 1)
        j0 = max(0, j - r); j1 = min(self.ny, j + r + 1)
        self.blocked[i0:i1, j0:j1] = False

    def _integrate(self, r, ang, trust):
        """Ray-cast the scan into the log-odds grid (corrected world frame)."""
        aw = ang + self.yaw
        cw = np.cos(aw); sw = np.sin(aw)
        S = self.samp[None, :]
        free = S < (r[:, None] - 0.40)
        if np.any(free):
            px = self.pos[0] + cw[:, None] * S
            py = self.pos[1] + sw[:, None] * S
            ix = ((px - self.origin[0]) / self.res).astype(np.int32)
            iy = ((py - self.origin[1]) / self.res).astype(np.int32)
            ok = free & (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            ixf = ix[ok]; iyf = iy[ok]
            if ixf.size:
                cur = self.lo_grid[ixf, iyf]
                self.lo_grid[ixf, iyf] = np.maximum(cur - np.float32(0.38 * trust),
                                                    np.float32(-3.0))
        hit = r < (self.range_max - 0.12)
        if np.any(hit):
            hx = self.pos[0] + cw[hit] * r[hit]
            hy = self.pos[1] + sw[hit] * r[hit]
            ix = ((hx - self.origin[0]) / self.res).astype(np.int32)
            iy = ((hy - self.origin[1]) / self.res).astype(np.int32)
            ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
            ixh = ix[ok]; iyh = iy[ok]
            if ixh.size:
                cur = self.lo_grid[ixh, iyh]
                self.lo_grid[ixh, iyh] = np.minimum(cur + np.float32(0.85 * trust),
                                                    np.float32(4.0))

    # ----------------------------------------------- pose correction (item 2)
    def _corrected(self, raw_xy, raw_yaw):
        d = raw_xy - self.anchor_raw
        p = self.anchor_true + self.rot @ d
        return p, _wrapf(raw_yaw + self.yaw_bias)

    def _reanchor(self, raw_xy, truth):
        """Calibration against a known fixed point, bridging-sample style."""
        corr, _ = self._corrected(raw_xy, 0.0)
        e = truth - corr
        ne = float(np.linalg.norm(e))

        # differential yaw update: two estimates of the same displacement
        if self.last_anchor_true is not None:
            d_true = truth - self.last_anchor_true
            d_est = corr - self.last_anchor_true
            if np.linalg.norm(d_true) > 2.0 and np.linalg.norm(d_est) > 2.0:
                da = _wrapf(math.atan2(d_true[1], d_true[0]) -
                            math.atan2(d_est[1], d_est[0]))
                self.yaw_bias = _wrapf(self.yaw_bias +
                                       float(np.clip(0.5 * da, -0.20, 0.20)))

        beta = 0.7
        shift = e * beta
        if ne * beta > 2.0:
            shift = shift * (2.0 / (ne * beta))
        self.anchor_true = corr + shift
        self.anchor_raw = raw_xy.copy()
        c, s = math.cos(self.yaw_bias), math.sin(self.yaw_bias)
        self.rot = np.array([[c, -s], [s, c]])

        # an inconsistent frame makes old evidence untrustworthy (item 4)
        if float(np.linalg.norm(shift)) > 0.6:
            self.lo_grid *= np.float32(0.5)

        self.last_anchor_true = truth.copy()
        self.last_anchor_corr = corr.copy()

    # --------------------------------------------------------------- step
    def step(self, obs):
        try:
            cmd = self._nav(obs)
        except Exception:
            self.fail += 1
            cmd = self.last_cmd
        vx = float(np.clip(cmd[0], -self.max_lin, self.max_lin))
        vy = float(np.clip(cmd[1], -self.max_lin, self.max_lin))
        wz = float(np.clip(cmd[2], -self.max_ang, self.max_ang))
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            vx = vy = wz = 0.0
        self.last_cmd = np.array([vx, vy, wz])
        return Command(vx, vy, wz)

    # ------------------------------------------------------------ main body
    def _nav(self, obs):
        self.k += 1

        t = _as_float(getattr(obs, "t", 0.0), 0.0)
        dt = 0.02 if self.t_prev is None else float(np.clip(t - self.t_prev, 0.004, 0.1))
        self.t_prev = t

        raw = _as_xy(getattr(obs, "pose_estimate", (0.0, 0.0)))
        try:
            raw_yaw = float(np.asarray(getattr(obs, "pose_estimate")).ravel()[2])
            if not math.isfinite(raw_yaw):
                raw_yaw = self.yaw
        except Exception:
            raw_yaw = self.yaw
        self.pose_err = abs(_as_float(getattr(obs, "pose_error_estimate", 0.0), 0.0))

        goal = _as_xy(getattr(obs, "goal", self.endpoints[1]), self.endpoints[1])

        # ---- arrival == calibration event ---------------------------------
        if self.prev_goal is None:
            self.prev_goal = goal.copy()
        elif float(np.linalg.norm(goal - self.prev_goal)) > 0.5:
            self.trips += 1
            self._reanchor(raw, self.prev_goal.copy())
            if self.trail:
                tr = np.asarray(self.trail, dtype=np.int32)
                self.visits[tr[:, 0], tr[:, 1]] = np.minimum(
                    self.visits[tr[:, 0], tr[:, 1]] + 1.0, 3.0)
                self.trail = []
            self.prev_goal = goal.copy()
            self.mode = 0
            self.mode_t = 0.0
            self.probe_t = 0.0
            self.goal_dwell = 0.0
            self.best_dist = None
            self.trap *= np.float32(0.5)

        self.pos_prev = self.pos
        self.pos, self.yaw = self._corrected(raw, raw_yaw)

        # ---- "excess" statistic: measurement minus model (item 4) ---------
        d_obs = float(np.linalg.norm(self.pos - self.pos_prev))
        v_obs = d_obs / dt
        v_req = float(math.hypot(self.last_cmd[0], self.last_cmd[1]))
        self.v_act += 0.08 * (v_obs - self.v_act)
        self.v_cmd += 0.08 * (v_req - self.v_cmd)
        self.excess += 0.05 * (abs(v_obs - v_req) - self.excess)
        if self.v_cmd > 0.12:
            self.track_ratio += 0.05 * (self.v_act / max(self.v_cmd, 1e-3) -
                                        self.track_ratio)
        self.track_ratio = float(np.clip(self.track_ratio, 0.0, 1.6))

        # ---- lidar ---------------------------------------------------------
        r = np.asarray(getattr(obs, "ranges", []), dtype=float).ravel()
        ang = np.asarray(getattr(obs, "range_angles", []), dtype=float).ravel()
        if r.size == 0 or ang.size != r.size:
            n = max(r.size, 1)
            ang = np.linspace(-math.pi, math.pi, n, endpoint=False)
            if r.size == 0:
                r = np.full(n, self.range_max)
        bad = ~np.isfinite(r)
        r = np.where(bad, self.range_max, r)
        r = np.clip(r, 0.10, self.range_max)
        contact = bool(getattr(obs, "contact", False))

        trust = float(np.clip(1.0 - 0.18 * self.pose_err - 0.3 * self.excess, 0.25, 1.0))
        if self.k % 2 == 0:
            self._integrate(r, ang, trust)
        if self.k % 12 == 0:
            self._rebuild_cost()
            ci, cj = self._cell(self.pos)
            if not self.trail or self.trail[-1] != (ci, cj):
                self.trail.append((ci, cj))
                if len(self.trail) > 5000:
                    self.trail = self.trail[-4000:]
            self.trap *= np.float32(0.999)

        # ---- planner maintenance ------------------------------------------
        fd = self._find_field(goal)
        self.active_key = self._key(fd["goal"])
        self._sweep(fd, 6)
        for key, other in self.fields.items():
            if key != self.active_key:
                self._sweep(other, 1)
                break

        to_goal = goal - self.pos
        dist_goal = float(np.linalg.norm(to_goal))
        bearing_goal = _wrapf(math.atan2(to_goal[1], to_goal[0]) - self.yaw)

        # ---- progress / stall bookkeeping ---------------------------------
        if self.best_dist is None or dist_goal < self.best_dist - 0.35:
            self.best_dist = dist_goal
            self.best_dist_t = t
        stalling = (self.v_cmd > 0.28 and self.v_act < 0.11) or \
                   (self.v_cmd > 0.3 and self.track_ratio < 0.22)
        self.stall_t = self.stall_t + dt if stalling else max(0.0, self.stall_t - 2.0 * dt)
        trapped = (t - self.best_dist_t) > 11.0 and dist_goal > 1.2
        if contact:
            self.contact_t += dt
        else:
            self.contact_t = max(0.0, self.contact_t - 0.5 * dt)

        # ---- admissible travel distance per candidate heading -------------
        dphi = ang[:, None] - self.cand[None, :]
        lon = r[:, None] * np.cos(dphi)
        lat = r[:, None] * np.sin(dphi)
        blockmat = (np.abs(lat) < self.half_w) & (lon > 0.0)
        dcand = np.where(blockmat, lon, 1e6).min(axis=0)
        np.minimum(dcand, self.range_max, out=dcand)
        near_min = float(np.min(r))

        # ---- differential corridor centring (paper item 1a) --------------
        vy_center, center_steer = self._differential_center(r, ang)

        # ---- desired heading ----------------------------------------------
        plan_w = float(np.clip(1.15 - 0.22 * self.pose_err, 0.35, 1.0))
        if dist_goal < 1.3 or self.mode == 3:
            theta_des = bearing_goal
            plan_w = 1.0
        else:
            theta_des, ok_plan = self._planner_heading(fd, bearing_goal)
            if not ok_plan:
                theta_des = bearing_goal
                plan_w *= 0.8

        # ---- near-goal dwell / probe spiral (paper item 6) ----------------
        if dist_goal < 0.55:
            self.goal_dwell += dt
        else:
            self.goal_dwell = max(0.0, self.goal_dwell - 0.5 * dt)
        if self.mode != 3 and self.goal_dwell > 2.2:
            self.mode = 3
            self.mode_t = 0.0
            self.probe_t = 0.0
        if self.mode == 3:
            self.probe_t += dt
            self.probe_phase += 1.4 * dt
            R = float(min(2.3, 0.35 + 0.28 * self.probe_t + 1.3 * self.pose_err))
            tgt = goal + R * np.array([math.cos(self.probe_phase),
                                       math.sin(self.probe_phase)])
            d = tgt - self.pos
            theta_des = _wrapf(math.atan2(d[1], d[0]) - self.yaw)
            if self.probe_t > 26.0:
                self.mode = 0
                self.mode_t = 0.0
                self.goal_dwell = 0.0

        # ---- recovery FSM --------------------------------------------------
        if self.mode in (0, 3) and (self.stall_t > 1.1 or self.contact_t > 0.35 or trapped):
            k_near = int(np.argmin(r))
            obstacle_dir = _wrapf(float(ang[k_near]))
            left = float(np.mean(r[(ang > 0.15) & (ang < 1.8)])) if np.any(
                (ang > 0.15) & (ang < 1.8)) else 1.0
            right = float(np.mean(r[(ang < -0.15) & (ang > -1.8)])) if np.any(
                (ang < -0.15) & (ang > -1.8)) else 1.0
            sgn = 1.0 if left > right else -1.0
            if self.esc_count % 2 == 1:
                sgn = -sgn
            self.esc_sign = sgn
            self.esc_count += 1
            self.mode = 1
            self.mode_t = 0.0
            self.stall_t = 0.0
            self.contact_t = 0.0
            self.best_dist_t = t
            self.best_dist = dist_goal
            ci, cj = self._cell(self.pos)
            i0 = max(0, ci - 1); i1 = min(self.nx, ci + 2)
            j0 = max(0, cj - 1); j1 = min(self.ny, cj + 2)
            self.trap[i0:i1, j0:j1] += np.float32(1.6)
            np.clip(self.trap, 0.0, 12.0, out=self.trap)
            self._obstacle_dir = obstacle_dir

        self.mode_t += dt
        if self.mode == 1:
            theta_des = _wrapf(getattr(self, "_obstacle_dir", 0.0) + math.pi)
            plan_w = 1.0
            if self.mode_t > 0.65:
                self.mode = 2
                self.mode_t = 0.0
        elif self.mode == 2:
            theta_des = _wrapf(bearing_goal + self.esc_sign * 1.45)
            plan_w = 1.0
            if self.mode_t > 2.4:
                self.mode = 0
                self.mode_t = 0.0

        # ---- mirror-symmetric candidate scoring with hysteresis (item 5) --
        clear_term = np.minimum(dcand, self.d_cap) / self.d_cap
        score = (2.05 * plan_w * np.cos(self.cand - theta_des)
                 + 1.15 * clear_term
                 + 0.45 * np.cos(self.cand - self.theta_prev)
                 + 0.12 * np.maximum(self.cand_cos, 0.0)
                 + 0.30 * center_steer * np.sin(self.cand))
        hard = 0.42 if self.mode != 1 else 0.26
        bad_c = dcand < hard
        if not np.all(bad_c):
            score = score - 12.0 * bad_c.astype(float)
        theta = float(self.cand[int(np.argmax(score))])
        if abs(_wrapf(theta - self.theta_prev)) < 0.09:
            theta = self.theta_prev
        self.theta_prev = theta

        d_go = float(dcand[int(np.argmin(np.abs(_wrap(self.cand - theta))))])

        # ---- speed ---------------------------------------------------------
        margin = 0.40 if self.trips == 0 else 0.34
        v_stop = math.sqrt(max(0.0, 2.0 * 1.25 * max(0.0, d_go - margin)))
        v = min(self.max_lin, v_stop)
        v *= 0.45 + 0.55 * max(0.0, math.cos(theta))
        if near_min < 0.45:
            v = min(v, 0.30)
        if self.mode == 1:
            v = min(0.42, max(v, 0.30))
        if self.mode == 3:
            v = min(v, 0.45)
        v = max(v, 0.0)

        ct, st = math.cos(theta), math.sin(theta)
        lim = self.max_lin
        if abs(st) > 1e-6:
            lim = min(lim, 0.50 / abs(st))
        v = min(v, lim)
        vx = v * ct
        vy = v * st + vy_center
        wz = 2.1 * theta + 0.35 * center_steer
        if self.mode == 1:
            wz += 0.5 * self.esc_sign
        if v < 0.06 and abs(theta) > 0.25:
            wz = math.copysign(self.max_ang, theta)

        vx = float(np.clip(vx, -self.max_lin, self.max_lin))
        vy = float(np.clip(vy, -0.55, 0.55))
        wz = float(np.clip(wz, -self.max_ang, self.max_ang))

        # ---- rate limit + low-pass: keep the drive away from the gait's
        #      own resonance and avoid an asymmetric limit cycle (item 5) ---
        prev = self.last_cmd
        max_dl = 3.0 * dt
        max_da = 7.0 * dt
        vx = prev[0] + float(np.clip(vx - prev[0], -max_dl, max_dl))
        vy = prev[1] + float(np.clip(vy - prev[1], -max_dl, max_dl))
        wz = prev[2] + float(np.clip(wz - prev[2], -max_da, max_da))
        a = 0.45
        vx = prev[0] + a * (vx - prev[0])
        vy = prev[1] + a * (vy - prev[1])
        wz = prev[2] + a * (wz - prev[2])
        if abs(wz) < 0.015:
            wz = 0.0
        return np.array([vx, vy, wz])

    # ------------------------------------------- differential centring (1a)
    def _differential_center(self, r, ang):
        """Mirrored, matched sector pairs with nested 2:1 spans.

        Each pair is differenced in the normalised form (rL-rR)/(rL+rR), so a
        common multiplicative range bias deletes exactly (Sec. 3, Eq. 11).
        The disagreement between the inner and the 2:1 outer pair measures how
        badly matched the two channels are, and gates the output.
        """
        try:
            a = ang
            pairs = ((0.35, 1.22), (0.17, 1.92))   # spans ~50 deg and ~100 deg
            diffs = []
            closed = True
            for a0, a1 in pairs:
                mL = (a > a0) & (a < a1)
                mR = (a < -a0) & (a > -a1)
                if not (np.any(mL) and np.any(mR)):
                    return 0.0, 0.0
                rL = float(np.mean(r[mL]))
                rR = float(np.mean(r[mR]))
                if rL > 0.72 * self.range_max and rR > 0.72 * self.range_max:
                    closed = False
                diffs.append((rL - rR) / max(rL + rR, 1e-3))
            if not closed:
                return 0.0, 0.0
            d_in, d_out = diffs[0], diffs[1]
            mism = abs(d_in - d_out)
            conf = float(np.clip(1.0 - 2.2 * mism, 0.0, 1.0))
            dm = 0.5 * (d_in + d_out)
            vy = float(np.clip(0.85 * dm * conf, -0.22, 0.22))
            steer = float(np.clip(0.9 * dm * conf, -0.35, 0.35))
            return vy, steer
        except Exception:
            return 0.0, 0.0

    # -------------------------------------------------- planner gradient
    def _planner_heading(self, fd, fallback):
        D = fd["live"]
        px = self.pos[0] + self.ring_ox
        py = self.pos[1] + self.ring_oy
        ix = ((px - self.origin[0]) / self.res).astype(np.int32)
        iy = ((py - self.origin[1]) / self.res).astype(np.int32)
        inb = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        np.clip(ix, 0, self.nx - 1, out=ix)
        np.clip(iy, 0, self.ny - 1, out=iy)
        val = D[ix, iy].astype(np.float64) + self.ring_cost
        val[~inb] = 1e18
        val[self.blocked[ix, iy]] = 1e18
        J = val.reshape(self.n_ring, self.n_rad).min(axis=1)
        jmin = float(np.min(J))
        if jmin > 0.4 * float(_BIG):
            return fallback, False
        best = int(np.argmin(J))
        da = _wrap(self.ring_dirs - self.ring_dirs[best])
        w = np.exp(-(J - jmin) / 0.55)
        w[np.abs(da) > 1.6] = 0.0
        w[J > 0.4 * float(_BIG)] = 0.0
        sw = float(np.sum(w))
        if sw < 1e-9:
            world = float(self.ring_dirs[best])
        else:
            world = math.atan2(float(np.sum(w * np.sin(self.ring_dirs))),
                               float(np.sum(w * np.cos(self.ring_dirs))))
        return _wrapf(world - self.yaw), True
