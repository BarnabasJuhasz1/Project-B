"""Shape-keyed revisit detection and re-routing for map-free exploration."""

import math

import numpy as np

from nodes.contract import Action, Status


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_arr(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RerouteAtShapeRecognisedPlace(Action):
    """Recognise a place by the shape of the surrounding surfaces, then leave it by an exit
    it has not used before.

    Mechanism (Cheng's geometric module): a place is held as a featureless record of *where
    surfaces are* -- the 181-beam range profile with `inf` replaced by `range_max`. Matching
    the record to the world is a rigid, non-mirrored congruence, which on a robot-frame scan
    is exactly a circular shift of the profile; the mirrored comparison is never evaluated,
    because rats make metric and rotational errors but no reflection errors. A match is
    accepted only when the best shift beats every shift more than ~45 deg away by more than
    20% -- the margin at which two rigid fits were still confusable for the rats. Place
    identity therefore never consults `pose_estimate`, which is the point: it survives
    arbitrary drift in the pose belief.

    On an accepted, unambiguous match the node revises `wz` (turn onto a bearing at least
    60 deg from every exit already taken from that place, and clear for 1.2 m) and `vx`
    (hold still while turning, crawl at 0.5 m/s once aligned) until 1.0 m has been covered
    along the new exit. It writes nothing else: the base navigator's lateral component and
    its whole command outside the maneuver pass through byte-identical.

    It declines -- FAILURE, no write -- whenever the goal has already been disclosed, the
    scan has fewer than 30 finite returns, no record is old enough (4.0 m of commanded
    travel) to be a genuine revisit rather than the previous frame of the same corridor,
    nothing matches within tolerance, the match is rotationally ambiguous, or no exit
    bearing satisfies both the clearance and the separation constraint.
    """

    WRITES = ("vx", "wz")
    PORTS = ()
    PRECONDITION = (
        "The goal is still undisclosed, the current scan has at least 30 finite returns, and "
        "the shape memory holds a record captured more than 4.0 m of commanded travel ago."
    )
    EFFECT = (
        "On a decisive non-mirrored shape match, turns onto the clearest bearing at least "
        "60 degrees from every exit already taken from that remembered place and drives 1.0 m "
        "along it."
    )
    TAGS = ("exploration", "place-recognition", "shape-matching", "revisit", "reroute", "map-free")

    # --- descriptor / match -------------------------------------------------
    MIN_FINITE = 30
    BLACKOUT_M = 4.0            # a record younger than this is the same frame, not a revisit
    CAPTURE_SPACING_M = 1.0
    EXIT_WINDOW_M = 1.0         # travel over which a record's exit bearing is averaged
    TAU_FRAC = 0.125            # 15 cm tolerance in a 120 cm arena, read as scale-relative
    TAU_MIN_M = 0.20
    TAU_MAX_M = 0.75
    AMBIG_SEP_RAD = math.pi / 4.0   # "another rotational fit" starts ~45 deg away
    AMBIG_RATIO = 1.20              # 20% above chance == still confusable
    MAX_RECORDS = 150
    MAX_FULL_MATCH = 8          # full N x N correlation only for the best few prefiltered

    # --- act ----------------------------------------------------------------
    EXIT_SEP_RAD = math.radians(60.0)
    CLEAR_HALF_RAD = math.radians(20.0)
    CLEAR_MIN_M = 1.2           # Go2 width plus margin
    YAW_GAIN = 1.5
    ALIGN_TOL_RAD = 0.35
    CRAWL_MS = 0.5
    MAX_ACTIVE_S = 8.0          # give the command back rather than hold it forever

    _records = None

    # ------------------------------------------------------------------ state
    def _clear(self):
        self._records = []
        self._travel = 0.0
        self._last_t = None
        self._next_capture = 0.0
        self._rearm_travel = 0.0
        self._active = None
        self._override = None
        self._shift_cache = {}
        self._prof = None
        self._ang = None
        self._n_finite = 0
        self._median_finite = 0.0

    def _ensure(self):
        if self._records is None:
            self._clear()

    def reset(self, scenario, rng):
        self._clear()

    def _shift_matrix(self, n):
        m = self._shift_cache.get(n)
        if m is None:
            i = np.arange(n)
            m = (i[None, :] + i[:, None]) % n
            self._shift_cache = {n: m}
        return m

    # ------------------------------------------------------------ bookkeeping
    def _advance(self, obs, ports):
        """Run exactly once per control step, from whichever of precondition/tick comes first.

        Travel must keep integrating on the ticks where the trigger is false, otherwise the
        4.0 m blackout would never elapse and the node could never fire at all.
        """
        self._ensure()
        t = float(obs.t)
        if self._last_t is not None:
            if t < self._last_t:
                self._clear()
            elif abs(t - self._last_t) < 1e-9:
                return
        dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), 0.25)
        self._last_t = t

        cmd = self._override
        self._override = None
        if cmd is None:
            pending = getattr(ports, "command", None)
            if pending is None:
                cmd = (0.0, 0.0, 0.0)
            else:
                cmd = (float(getattr(pending, "vx", 0.0)),
                       float(getattr(pending, "vy", 0.0)),
                       float(getattr(pending, "wz", 0.0)))
        vx, vy, wz = cmd

        self._read_scan(obs)

        if dt > 0.0:
            step = math.hypot(vx, vy) * dt
            self._travel += step
            if step > 1e-6:
                bearing = math.atan2(vy, vx)
                s, c = math.sin(bearing) * step, math.cos(bearing) * step
                for rec in self._records:
                    if not rec["open"]:
                        continue
                    rec["acc_s"] += s
                    rec["acc_c"] += c
                    rec["acc_d"] += step
                    if rec["acc_d"] >= self.EXIT_WINDOW_M:
                        rec["exits"].append(_wrap(math.atan2(rec["acc_s"], rec["acc_c"])))
                        rec["open"] = False
            if self._active is not None:
                # Pose is never read; the target bearing is kept in the robot frame by
                # integrating the yaw rate we ourselves commanded.
                self._active["theta"] = _wrap(self._active["theta"] - wz * dt)
                self._active["driven"] += max(vx, 0.0) * dt

        if self._travel >= self._next_capture and self._n_finite >= self.MIN_FINITE:
            self._capture()
            self._next_capture = self._travel + self.CAPTURE_SPACING_M

    def _read_scan(self, obs):
        r = np.asarray(obs.ranges, dtype=float).ravel()
        ang = np.asarray(obs.range_angles, dtype=float).ravel()
        n = min(r.size, ang.size)
        r, ang = r[:n], ang[:n]
        if n > 2 and abs(_wrap(ang[-1] - ang[0])) < 1e-3:
            r, ang = r[:-1], ang[:-1]
        rmax = float(getattr(obs, "range_max", 6.0))
        if not math.isfinite(rmax) or rmax <= 0.0:
            rmax = 6.0
        finite = np.isfinite(r)
        prof = np.clip(np.where(finite, r, rmax), 0.0, rmax)
        self._prof = prof
        self._ang = ang
        self._n_finite = int(finite.sum())
        self._median_finite = float(np.median(r[finite])) if self._n_finite else 0.0

    def _capture(self):
        prof = self._prof
        if prof is None or prof.size < 8:
            return
        tau = min(max(self.TAU_FRAC * self._median_finite, self.TAU_MIN_M), self.TAU_MAX_M)
        self._records.append({
            "r": prof.copy(),
            "travel": self._travel,
            "tau": tau,
            "mean": float(prof.mean()),
            "exits": [],
            "open": True,
            "acc_s": 0.0,
            "acc_c": 0.0,
            "acc_d": 0.0,
        })
        if len(self._records) > self.MAX_RECORDS:
            del self._records[0]

    # ---------------------------------------------------------------- trigger
    def _trigger(self, obs):
        if getattr(obs, "goal", None) is not None:
            return False
        if self._n_finite < self.MIN_FINITE:
            return False
        if self._travel < self._rearm_travel:
            return False
        cutoff = self._travel - self.BLACKOUT_M
        for rec in self._records:
            if rec["travel"] <= cutoff and rec["exits"]:
                return True
        return False

    # ------------------------------------------------------------------ match
    def _best_match(self):
        prof = self._prof
        n = prof.size
        cutoff = self._travel - self.BLACKOUT_M
        cur_mean = float(prof.mean())

        candidates = []
        for rec in self._records:
            if rec["open"] or not rec["exits"] or rec["travel"] > cutoff:
                continue
            if rec["r"].size != n:
                continue
            # |mean(a) - mean(b)| lower-bounds mean|a - b| for every shift, so this
            # prefilter can only discard records that could never be accepted.
            lb = abs(cur_mean - rec["mean"])
            if lb > rec["tau"]:
                continue
            candidates.append((lb, rec))
        if not candidates:
            return None

        candidates.sort(key=lambda p: p[0])
        idx = self._shift_matrix(n)
        beams = np.arange(n)
        sep_beams = max(1, int(round(self.AMBIG_SEP_RAD / (2.0 * math.pi / n))))

        best = None
        for _, rec in candidates[:self.MAX_FULL_MATCH]:
            # d(k) = mean_i |r_cur[i] - r_rec[(i+k) % n]|. The mirrored profile is never
            # tried: sense is part of the record.
            d = np.abs(prof[None, :] - rec["r"][idx]).mean(axis=1)
            k = int(np.argmin(d))
            d_star = float(d[k])
            if d_star > rec["tau"]:
                continue
            sep = np.abs(beams - k)
            sep = np.minimum(sep, n - sep)
            far = sep >= sep_beams
            if far.any() and float(d[far].min()) <= self.AMBIG_RATIO * d_star:
                continue  # rotationally self-similar: the record cannot pick a fit
            if best is None or d_star < best[2]:
                best = (rec, k, d_star)
        return best

    # -------------------------------------------------------------------- act
    def _choose_bearing(self, rec, k):
        prof, ang = self._prof, self._ang
        n = prof.size
        d_ang = 2.0 * math.pi / n
        phis = np.array([_wrap(p - k * d_ang) for p in rec["exits"]], dtype=float)

        sep_to_exits = np.abs(_wrap_arr(ang[None, :] - phis[:, None])).min(axis=0)
        near = np.abs(_wrap_arr(ang[None, :] - ang[:, None])) <= self.CLEAR_HALF_RAD
        clearance = np.where(near, prof[None, :], np.inf).min(axis=1)

        ok = (sep_to_exits >= self.EXIT_SEP_RAD) & (clearance >= self.CLEAR_MIN_M)
        if not ok.any():
            return None
        scores = np.where(ok, sep_to_exits, -np.inf)
        return float(ang[int(np.argmax(scores))])

    def _finish(self, status):
        self._active = None
        self._rearm_travel = self._travel + self.BLACKOUT_M
        return status

    def _run_active(self, obs, ports):
        act = self._active
        if bool(getattr(obs, "contact", False)):
            return self._finish(Status.FAILURE)
        if obs.t - act["t0"] > self.MAX_ACTIVE_S:
            return self._finish(Status.FAILURE)
        if act["driven"] >= self.EXIT_WINDOW_M:
            act["rec"]["exits"].append(_wrap(act["theta_rec"]))
            return self._finish(Status.SUCCESS)

        err = _wrap(act["theta"])
        wz = max(-1.0, min(1.0, self.YAW_GAIN * err))
        ports.adjust(wz=wz)
        if abs(err) <= self.ALIGN_TOL_RAD:
            vx = self.CRAWL_MS
            ports.adjust(vx=vx)
        else:
            vx = 0.0
            ports.cap(vx=0.0)
        pending = ports.command
        self._override = (vx, float(getattr(pending, "vy", 0.0)), wz)
        return Status.RUNNING

    # ------------------------------------------------------------------ leaf
    def precondition(self, obs, ports):
        self._advance(obs, ports)
        if self._active is not None:
            return True
        return self._trigger(obs)

    def tick(self, obs, ports):
        self._advance(obs, ports)

        if self._active is not None:
            return self._run_active(obs, ports)

        if not self._trigger(obs):
            return Status.FAILURE

        match = self._best_match()
        if match is None:
            return Status.FAILURE
        rec, k, _ = match

        theta = self._choose_bearing(rec, k)
        if theta is None:
            return Status.FAILURE

        n = self._prof.size
        self._active = {
            "rec": rec,
            "theta": theta,
            "theta_rec": _wrap(theta + k * (2.0 * math.pi / n)),
            "driven": 0.0,
            "t0": float(obs.t),
        }
        return self._run_active(obs, ports)


PREDICTION = {
    "statement": (
        "On scenarios where obs.goal is None at episode start, inserting this leaf above the "
        "default explorer raises the fraction of seeds on which the goal is found, because "
        "shape-keyed revisit detection breaks the loop-and-re-tread behaviour that pose-keyed "
        "or pose-free exploration falls into once pose_estimate has drifted. It is falsified "
        "if goal_reached fails to improve by the stated margin, and the mechanism specifically "
        "(rather than the extra turning) is falsified if allowing the mirrored match or "
        "deleting the 1.20 ambiguity guard leaves goal_reached unchanged."
    ),
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}
