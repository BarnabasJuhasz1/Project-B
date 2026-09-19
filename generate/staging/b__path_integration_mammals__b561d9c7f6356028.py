import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status

_TWO_PI = 2.0 * math.pi
_SECTORS = 6
_SECTOR_WIDTH = _TWO_PI / _SECTORS


def _wrap(a):
    """Wrap an angle to [-pi, pi)."""
    return (a + math.pi) % _TWO_PI - math.pi


class ExploreByAnchoredExcursions(Action):
    """Explore an unmapped space as bounded out-and-back excursions from a home base.

    Mechanism (Golani et al. / path-integration home base): a rodent exploring
    unfamiliar ground does not perform one open-ended walk; it centres bounded
    excursions on a reference point and returns frequently, because the position
    estimate is an incremental recursive sum whose errors are cumulative and are
    cleared only by re-encountering a familiar place. Here the path integrator is
    ``obs.pose_estimate``, its cumulative error is ``obs.pose_error_estimate``,
    and the familiar place is a latched anchor ``A`` plus the 181-beam range
    signature ``S`` captured there. We cannot zero the integrator - the platform
    owns the belief - so instead we re-fit ``S`` to the live scan on arrival and
    move ``A`` by the best-fit offset, which keeps the home vector pointing at
    the same *physical* place while the belief frame drifts under it.

    Revises ``vx`` and ``wz``: the mechanism is a heading law (drive the
    excursion bearing, or the home vector) with a clearance-modulated speed, so
    it needs the turn rate to aim and the forward speed to bound the leg and to
    slow near obstacles. Both are set outright rather than capped, because the
    node is proposing a direction of travel, not bounding one. ``vy`` is left
    entirely to the base.

    Declines - and the base's command passes through untouched - whenever the
    goal has already been latched (permanently, it never re-arms: this is a
    search policy, not a travel policy) and on any tick where the robot is in
    contact, so the base owns collision recovery.
    """

    WRITES = ("vx", "wz")

    PORTS = (
        Port("goal_known", PortType.FLAG, Direction.IN),
        Port("home_bearing", PortType.BEARING, Direction.OUT),
        Port("home_anchor", PortType.REGION, Direction.OUT),
    )

    PRECONDITION = (
        "The goal has not yet been latched and the robot is free-running "
        "(not in contact this tick)."
    )
    EFFECT = (
        "Drives bounded out-and-back excursions around a re-anchored home base "
        "by setting forward speed and yaw rate."
    )
    TAGS = ("exploration", "home-base", "path-integration", "drift", "search")

    # --- excursion budget (all figures from the principle card) -------------
    PATH_BUDGET = 12.0          # m, 2 * range_max of outbound path length
    ERROR_BUDGET = 0.75         # m of *growth* in the belief's own error
    DEAD_END_RANGE = 1.5        # m; no beam beyond this ahead -> turn back
    HOME_RADIUS = 1.0           # m; close enough to attempt a re-anchor
    LOST_SLACK = 10.0           # m added to 1.5x outbound before declaring lost

    # --- drive law ----------------------------------------------------------
    CRUISE = 0.6                # m/s at zero heading error and full clearance
    TURN_GAIN = 1.8             # rad/s per rad of heading error
    STOP_CLEARANCE = 0.35       # m, body half-diagonal plus margin
    FULL_CLEARANCE = 0.80       # m, where cruise speed resumes
    CLEAR_HALF_ANGLE = math.radians(50.0)
    DETOUR_HALF_ANGLE = math.pi / 2.0
    DETOUR_MIN_RANGE = 1.2      # m; a re-aim target must be at least this open
    MAX_STEP = 1.0              # m; larger per-tick pose deltas are belief jumps

    # --- re-anchor search ---------------------------------------------------
    XY_SPAN = 2.0
    XY_STEP = 0.25
    YAW_SPAN = math.radians(30.0)
    YAW_STEP = math.radians(5.0)
    MATCH_ACCEPT = 0.35         # m mean absolute range residual
    MATCH_RATIO = 0.9           # must beat the zero-offset residual by this
    MIN_MATCH_BEAMS = 30

    _LUT_BINS = 2048

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.reset(None, None)

    # ------------------------------------------------------------------ state
    def reset(self, scenario, rng):
        self._anchor = None
        self._home_scan = None
        self._home_finite = None
        self._home_yaw = 0.0
        self._e0 = 0.0
        self._d = 0.0
        self._phase = "OUTBOUND"
        self._used = {}
        self._pick_counter = 0
        self._prev_xy = None
        self._target = 0.0
        self._outbound_len = 0.0
        self._return_d = 0.0
        self._yielded = False
        self._range_max = 6.0
        self._angles = None
        self._lut = None
        self._smooth_half = 3

    # ----------------------------------------------------------- precondition
    def precondition(self, obs, ports):
        if obs.goal is not None:
            self._yielded = True
        if self._yielded:
            return False
        if ports["goal_known"]:
            self._yielded = True
            return False
        if obs.contact:
            return False
        return True

    # ------------------------------------------------------------------ tick
    def tick(self, obs, ports):
        pose = obs.pose_estimate
        xy = np.array([float(pose[0]), float(pose[1])])
        yaw = float(pose[2])
        err = float(obs.pose_error_estimate)
        self._range_max = float(obs.range_max)

        angles = np.asarray(obs.range_angles, dtype=float)
        raw = np.asarray(obs.ranges, dtype=float)
        finite = np.isfinite(raw)
        scan = np.clip(np.where(finite, raw, self._range_max), 0.0, self._range_max)

        self._prepare_beams(angles)

        if self._anchor is None:
            self._latch(xy, yaw, scan, finite, err)
            self._target = self._choose_bearing(scan, yaw)
        else:
            step = float(np.hypot(*(xy - self._prev_xy)))
            # A belief jump is not distance walked; only real motion spends budget.
            if step < self.MAX_STEP:
                self._d += step
                if self._phase == "RETURN":
                    self._return_d += step
        self._prev_xy = xy

        if self._phase == "OUTBOUND" and self._leg_exhausted(scan, yaw, err):
            self._phase = "RETURN"
            self._outbound_len = self._d
            self._return_d = 0.0

        target = None
        if self._phase == "RETURN":
            h = self._anchor - xy
            dist = float(np.hypot(h[0], h[1]))
            if dist <= self.HOME_RADIUS:
                self._re_anchor(xy, yaw, scan, finite)
                # Budget resets whether or not the match was accepted: thrashing
                # on a rejected match costs more than one stale excursion.
                self._e0 = err
                self._d = 0.0
                self._phase = "OUTBOUND"
                self._target = self._choose_bearing(scan, yaw)
            elif self._return_d > 1.5 * self._outbound_len + self.LOST_SLACK:
                self._latch(xy, yaw, scan, finite, err)
                self._used.clear()
                self._target = self._choose_bearing(scan, yaw)
            else:
                target = math.atan2(h[1], h[0])
        if target is None:
            target = self._target

        self._drive(target, scan, yaw, ports)

        h = self._anchor - xy
        ports["home_bearing"] = _wrap(math.atan2(h[1], h[0]) - yaw)
        ports["home_anchor"] = (float(self._anchor[0]), float(self._anchor[1]),
                                self.HOME_RADIUS)
        return Status.RUNNING

    # -------------------------------------------------------------- anchoring
    def _latch(self, xy, yaw, scan, finite, err):
        self._anchor = xy.copy()
        self._home_scan = scan.copy()
        self._home_finite = finite.copy()
        self._home_yaw = yaw
        self._e0 = err
        self._d = 0.0
        self._outbound_len = 0.0
        self._return_d = 0.0
        self._phase = "OUTBOUND"

    def _re_anchor(self, xy, yaw, scan, finite):
        """Re-fit the home signature to the live scan; move the anchor by the fit.

        The belief frame has drifted, so the stored anchor no longer names the
        physical place it was captured at. Searching an offset that best explains
        the live scan as a view of the home panorama recovers that place.
        """
        rmax = self._range_max
        ca = np.cos(self._angles + yaw)
        sa = np.sin(self._angles + yaw)
        pts = np.stack((scan * ca + xy[0], scan * sa + xy[1]), axis=1)

        offs = np.arange(-self.XY_SPAN, self.XY_SPAN + 1e-9, self.XY_STEP)
        dyaws = np.arange(-self.YAW_SPAN, self.YAW_SPAN + 1e-9, self.YAW_STEP)
        home = self._home_scan
        home_ok = self._home_finite

        best = (float("inf"), 0.0, 0.0, 0.0)
        zero_res = float("inf")

        for dx in offs:
            for dy in offs:
                q = pts - (self._anchor + np.array([dx, dy]))
                qr = np.clip(np.hypot(q[:, 0], q[:, 1]), 0.0, rmax)
                qa = np.arctan2(q[:, 1], q[:, 0])
                # One index lookup per (beam, dyaw): the yaw sweep is just a
                # rotation of which stored beam each live point falls on.
                b = qa[None, :] - (self._home_yaw + dyaws[:, None])
                idx = self._beam_index(b)
                pred = home[idx]
                valid = home_ok[idx] & finite[None, :]
                counts = valid.sum(axis=1)
                diffs = np.where(valid, np.abs(pred - qr[None, :]), 0.0)
                sums = diffs.sum(axis=1)
                for k in range(dyaws.size):
                    if counts[k] < self.MIN_MATCH_BEAMS:
                        continue
                    res = float(sums[k] / counts[k])
                    if (abs(dx) < 1e-9 and abs(dy) < 1e-9
                            and abs(dyaws[k]) < 1e-9):
                        zero_res = res
                    if res < best[0]:
                        best = (res, float(dx), float(dy), float(dyaws[k]))

        if best[0] < self.MATCH_ACCEPT and best[0] < self.MATCH_RATIO * zero_res:
            self._anchor = self._anchor + np.array([best[1], best[2]])
            # Keep S's own frame consistent with the accepted fit, or every
            # later match starts from a yaw that is known to be wrong.
            self._home_yaw = _wrap(self._home_yaw + best[3])

    # ---------------------------------------------------------- leg selection
    def _leg_exhausted(self, scan, yaw, err):
        if self._d >= self.PATH_BUDGET:
            return True
        if err - self._e0 >= self.ERROR_BUDGET:
            return True
        ahead = np.abs(_wrap(self._angles + yaw - self._target)) <= self.DETOUR_HALF_ANGLE
        return not bool(np.any(scan[ahead] > self.DEAD_END_RANGE))

    def _choose_bearing(self, scan, yaw):
        """Most open allocentric bearing in a 60-degree sector not yet excursed."""
        smooth = self._smooth(scan)
        allo = (self._angles + yaw) % _TWO_PI
        sector = (np.floor(allo / _SECTOR_WIDTH).astype(int)) % _SECTORS
        free = np.ones(sector.shape, dtype=bool)
        for s in self._used:
            free &= sector != s
        if not free.any():
            lru = min(self._used, key=lambda s: self._used[s])
            self._used.clear()
            free = sector == lru
            if not free.any():
                free = np.ones(sector.shape, dtype=bool)
        best = int(np.argmax(np.where(free, smooth, -np.inf)))
        self._pick_counter += 1
        self._used[int(sector[best])] = self._pick_counter
        return _wrap(float(allo[best]))

    # ------------------------------------------------------------- drive law
    def _drive(self, target_allo, scan, yaw, ports):
        psi = _wrap(target_allo - yaw)
        clear = self._clearance(scan, psi)
        if clear < self.STOP_CLEARANCE:
            detour = self._detour(scan, psi)
            if detour is not None:
                psi = detour
                clear = self._clearance(scan, psi)
        scale = float(np.clip(
            (clear - self.STOP_CLEARANCE)
            / (self.FULL_CLEARANCE - self.STOP_CLEARANCE), 0.0, 1.0))
        vx = self.CRUISE * max(0.0, math.cos(psi)) * scale
        wz = float(np.clip(self.TURN_GAIN * psi, -1.0, 1.0))
        ports.adjust(vx=vx)
        ports.adjust(wz=wz)

    def _clearance(self, scan, psi):
        mask = np.abs(_wrap(self._angles - psi)) <= self.CLEAR_HALF_ANGLE
        if not mask.any():
            return self._range_max
        return float(scan[mask].min())

    def _detour(self, scan, psi):
        """Nearest bearing within +/-90 deg of intent that is genuinely open."""
        delta = _wrap(self._angles - psi)
        mask = (np.abs(delta) <= self.DETOUR_HALF_ANGLE) & (scan > self.DETOUR_MIN_RANGE)
        if not mask.any():
            return None
        cand = np.where(mask, np.abs(delta), np.inf)
        return float(self._angles[int(np.argmin(cand))])

    # ----------------------------------------------------------- beam helpers
    def _prepare_beams(self, angles):
        if (self._angles is not None and self._angles.size == angles.size
                and np.array_equal(self._angles, angles)):
            return
        self._angles = angles.copy()
        n = angles.size
        spacing = _TWO_PI / max(n, 1)
        self._smooth_half = max(1, int(round(math.radians(7.5) / spacing)))
        centres = (np.arange(self._LUT_BINS) + 0.5) * (_TWO_PI / self._LUT_BINS) - math.pi
        d = np.abs(_wrap(centres[:, None] - angles[None, :]))
        self._lut = np.argmin(d, axis=1).astype(np.intp)

    def _beam_index(self, bearing):
        b = ((bearing + math.pi) % _TWO_PI) * (self._LUT_BINS / _TWO_PI)
        return self._lut[np.clip(b.astype(np.intp), 0, self._LUT_BINS - 1)]

    def _smooth(self, scan):
        half = self._smooth_half
        n = scan.size
        idx = np.arange(n)
        acc = np.zeros(n, dtype=float)
        for o in range(-half, half + 1):
            acc += scan[(idx + o) % n]
        return acc / float(2 * half + 1)


PREDICTION = {
    "statement": (
        "On tasks where the goal is not disclosed, structuring exploration as "
        "bounded out-and-back excursions from a re-anchored home base finds and "
        "latches the goal on a larger fraction of seeds than open-ended reactive "
        "wandering, because the frame in which unexplored bearings are recorded "
        "is re-zeroed before drift makes that record meaningless."
    ),
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}
