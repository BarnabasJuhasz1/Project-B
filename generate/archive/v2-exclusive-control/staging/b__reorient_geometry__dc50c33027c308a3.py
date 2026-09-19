"""RerouteAtShapeRecognisedPlace.

Prediction (verbatim from the principle card): On scenarios where obs.goal is None
at episode start, inserting this leaf above the default explorer raises the fraction
of seeds on which the goal is found, because shape-keyed revisit detection breaks the
loop-and-re-tread behaviour that pose-keyed or pose-free exploration falls into once
pose_estimate has drifted. It is falsified if goal_reached fails to improve by the
stated margin, and the mechanism specifically (rather than the extra turning) is
falsified if allowing the mirrored match or deleting the 1.20 ambiguity guard leaves
goal_reached unchanged.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

# --- descriptor / match ------------------------------------------------------
MIN_FINITE = 30            # a scan with fewer returns is not a "shape"
TAU_FRAC = 0.125           # rats' 15cm tolerance in a 120cm arena, read as a ratio
TAU_MIN, TAU_MAX = 0.20, 0.75
AMBIG_RATIO = 1.20         # 20% = observed level at which two rigid fits stay confusable
AMBIG_SEP_RAD = math.radians(45.0)
# --- memory ------------------------------------------------------------------
CAPTURE_SPACING_M = 0.5
BLACKOUT_M = 4.0
MAX_RECORDS = 200
EXIT_WINDOW_M = 1.0        # travel over which a record's exit bearing is averaged
# --- act ---------------------------------------------------------------------
EXIT_SEP_RAD = math.radians(60.0)
CLEAR_HALF_WIN_RAD = math.radians(20.0)
CORRIDOR_CLEAR_M = 1.2
TURN_GAIN = 1.5
CRAWL_SPEED = 0.5
ALIGN_TOL = 0.35
REROUTE_TIMEOUT_S = 12.0
# --- odometry surrogate ------------------------------------------------------
NOMINAL_SPEED = 0.5        # assumed speed of whoever drives when we do not
MAX_DYAW_PER_TICK = 0.35   # search window for the scan-to-scan yaw estimate
DYAW_TRUST_M = 1.5         # residual above which the yaw estimate is discarded


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RerouteAtShapeRecognisedPlace(Action):
    """Leave a place you recognise by its SHAPE alone, by an exit you have not used.

    Mechanism (Cheng's geometric module). A place is keyed by the arrangement of
    surrounding surfaces as surfaces: the 181-beam range profile with inf folded to
    range_max, which carries metric distance, angle and sense (beam order) and nothing
    about what the surfaces look like. Because range_angles are robot-frame, a change
    of heading is exactly a circular shift of that profile, so matching a stored record
    to the world rigidly is a search over the N circular shifts. The mirrored
    comparison is never evaluated: the rats made no reflection errors, so handedness is
    part of the record, not an optional invariance.

    Why it helps here: the key never consults the robot's belief about where it is, so
    it survives arbitrary pose drift, which is what makes a pose-keyed explorer re-tread
    its own loop. Its one failure mode is the one the paper measures - a shape that is
    self-congruent under rotation (a corridor, a square room) cannot pick between its
    two fits - so the node refuses to answer whenever the second-best rotational fit is
    within 20% of the best, and falls through to the explorer below.

    On an accepted, unambiguous match the record's remembered exit bearings are rotated
    into the current frame by the winning shift, and the node turns onto the bearing
    furthest from all of them that still has 1.2 m of corridor, drives one metre along
    it, and files that bearing as a further exit of the same place.
    """

    PORTS = (
        Port("place_revisited", PortType.FLAG, Direction.OUT),
        Port("reroute_bearing", PortType.BEARING, Direction.OUT),
    )
    PRECONDITION = (
        "The goal is not yet disclosed, the current scan carries at least 30 finite "
        "returns, and shape memory holds a record captured more than 4 m of "
        "accumulated travel ago."
    )
    EFFECT = (
        "Turns the robot onto a heading at least 60 degrees away from every exit "
        "previously taken from the shape-recognised place and drives 1 m along it."
    )
    TAGS = ("exploration", "place-recognition", "scan-shape", "revisit", "reroute")

    # ------------------------------------------------------------------ state

    def reset(self, scenario, rng):
        self._clear()

    def _clear(self):
        self._records = []
        self._travel = 0.0
        self._psi = 0.0          # heading integrator, arbitrary zero, never a pose
        self._t_seen = None
        self._last_cmd = None
        self._prev_prof = None
        self._cur = None
        self._n_finite = 0
        self._active = None
        self._N = None

    def _ensure(self):
        if not hasattr(self, "_records"):
            self._clear()

    # ------------------------------------------------------- scan geometry

    def _setup(self, obs):
        ang = np.asarray(obs.range_angles, dtype=np.float64)
        n = ang.size
        step = _wrap(ang[1] - ang[0])
        # a full-circle sweep usually repeats the first bearing as the last beam
        if n > 2 and abs(_wrap(ang[-1] - ang[0])) < 0.5 * abs(step):
            n -= 1
        self._N = n
        self._step = step
        self._ang = ang[:n]
        self._idx = np.add.outer(np.arange(n), np.arange(n)) % n   # idx[k, i] = (i+k) % n
        w = max(1, int(round(CLEAR_HALF_WIN_RAD / abs(step))))
        self._win = np.add.outer(np.arange(n), np.arange(-w, w + 1)) % n
        self._sep_min = max(1, int(round(AMBIG_SEP_RAD / abs(step))))
        kmax = max(1, int(math.ceil(MAX_DYAW_PER_TICK / abs(step))))
        self._dk = np.arange(-kmax, kmax + 1)
        self._didx = (np.arange(n)[None, :] + self._dk[:, None]) % n

    def _profile(self, obs):
        r = np.asarray(obs.ranges, dtype=np.float64)
        finite = np.isfinite(r)
        self._n_finite = int(finite[: self._N].sum())
        r = np.where(finite, r, float(obs.range_max))
        r = np.clip(r, 0.0, float(obs.range_max))[: self._N]
        return r

    # --------------------------------------------------------- bookkeeping

    def _observe(self, obs):
        """Advance memory exactly once per control step.

        Called from both precondition() and tick(): the tree may gate ticking on the
        precondition, and the precondition itself depends on memory having advanced.
        """
        self._ensure()
        if self._t_seen is not None and obs.t <= self._t_seen:
            return
        if self._N is None:
            self._setup(obs)
        dt = 0.0 if self._t_seen is None else min(max(obs.t - self._t_seen, 0.0), 0.5)
        self._t_seen = obs.t

        prof = self._profile(obs)

        if dt <= 0.0:
            dyaw, dist, motion_bearing = 0.0, 0.0, 0.0
        elif self._last_cmd is not None:
            vx, vy, wz = self._last_cmd
            dyaw = wz * dt
            dist = math.hypot(vx, vy) * dt
            motion_bearing = math.atan2(vy, vx) if dist > 1e-9 else 0.0
        else:
            # someone else is driving: recover the heading change from the scan itself
            dyaw = self._dyaw_from_scan(prof)
            dist = NOMINAL_SPEED * dt
            motion_bearing = 0.0      # the locomotion policy is driven forward

        self._psi += dyaw
        self._travel += dist
        if self._active is not None and self._last_cmd is not None:
            self._active["prog"] += dist

        self._accumulate_exits(motion_bearing, dist)
        self._maybe_capture(prof)

        self._prev_prof = prof
        self._cur = prof
        self._last_cmd = None

    def _dyaw_from_scan(self, prof):
        if self._prev_prof is None:
            return 0.0
        d = np.abs(self._prev_prof[self._didx] - prof[None, :]).mean(axis=1)
        j = int(np.argmin(d))
        if d[j] > DYAW_TRUST_M:
            return 0.0
        k = float(self._dk[j])
        # parabolic refinement: the beam pitch is ~2 deg, coarse next to one tick of yaw
        if 0 < j < d.size - 1:
            a, b, c = d[j - 1], d[j], d[j + 1]
            den = a - 2.0 * b + c
            if den > 1e-9:
                k += float(np.clip(0.5 * (a - c) / den, -0.5, 0.5))
        return k * self._step

    def _accumulate_exits(self, motion_bearing, dist):
        if dist <= 0.0:
            return
        for rec in self._records:
            p = rec["pend"]
            if p is None:
                continue
            # the exit is stored in the record's OWN frame
            phi = _wrap(motion_bearing + (self._psi - rec["psi"]))
            p[0] += math.cos(phi) * dist
            p[1] += math.sin(phi) * dist
            p[2] += dist
            if p[2] >= EXIT_WINDOW_M:
                if p[0] * p[0] + p[1] * p[1] > 1e-12:
                    rec["exits"].append(_wrap(math.atan2(p[1], p[0])))
                rec["pend"] = None

    def _maybe_capture(self, prof):
        if self._n_finite < MIN_FINITE:
            return
        if self._records and self._travel - self._records[-1]["s0"] < CAPTURE_SPACING_M:
            return
        med = float(np.median(prof[prof < prof.max() - 1e-9])) if np.any(
            prof < prof.max() - 1e-9) else float(np.median(prof))
        self._records.append({
            "prof": prof.copy(),
            "sorted": np.sort(prof),
            "med": med,
            "s0": self._travel,
            "used": -1e9,
            "psi": self._psi,
            "exits": [],
            "pend": [0.0, 0.0, 0.0],
        })
        if len(self._records) > MAX_RECORDS:
            del self._records[0]

    # -------------------------------------------------------------- matching

    @staticmethod
    def _tau(rec):
        return min(max(TAU_FRAC * rec["med"], TAU_MIN), TAU_MAX)

    def _best_match(self, prof):
        cur_sorted = np.sort(prof)
        best = None
        for rec in self._records:
            if rec["pend"] is not None or not rec["exits"]:
                continue
            # blackout is measured from capture OR from this record's last reroute,
            # so one place cannot fire twice inside four metres
            if self._travel - max(rec["s0"], rec["used"]) <= BLACKOUT_M:
                continue
            tau = self._tau(rec)
            # exact lower bound on the shifted L1 distance: sorting minimises L1 cost
            # over every permutation, hence over the N circular shifts in particular
            if float(np.abs(cur_sorted - rec["sorted"]).mean()) > tau:
                continue
            d = np.abs(rec["prof"][self._idx] - prof[None, :]).mean(axis=1)
            k = int(np.argmin(d))
            dstar = float(d[k])
            if dstar > tau:
                continue
            if best is None or dstar < best[1]:
                best = (rec, dstar, k, d)
        return best

    def _ambiguous(self, d, kstar):
        off = np.abs(np.arange(self._N) - kstar)
        sep = np.minimum(off, self._N - off)
        far = d[sep >= self._sep_min]
        if far.size == 0:
            return True
        return float(far.min()) <= AMBIG_RATIO * float(d[kstar])

    def _choose_exit(self, prof, rec, kstar):
        clear = prof[self._win].min(axis=1)
        # rigid congruence carries the record's exits into the current frame
        phis = np.array([_wrap(p - kstar * self._step) for p in rec["exits"]])
        if phis.size:
            margin = np.abs(_wrap(self._ang[:, None] - phis[None, :])).min(axis=1)
        else:
            margin = np.full(self._N, math.pi)
        ok = (clear >= CORRIDOR_CLEAR_M) & (margin >= EXIT_SEP_RAD)
        if not ok.any():
            return None
        m = np.where(ok, margin, -np.inf)
        ties = ok & (margin >= m.max() - 1e-9)
        j = int(np.argmax(np.where(ties, clear, -np.inf)))
        return float(self._ang[j])

    # ----------------------------------------------------------- node surface

    def precondition(self, obs, ports):
        self._observe(obs)
        if obs.goal is not None:
            return False
        if self._active is not None:
            return True
        if self._n_finite < MIN_FINITE:
            return False
        return any(self._travel - max(r["s0"], r["used"]) > BLACKOUT_M
                   and r["pend"] is None and r["exits"] for r in self._records)

    def tick(self, obs, ports):
        self._observe(obs)

        if obs.goal is not None:
            self._active = None
            return Status.FAILURE
        if self._active is not None:
            return self._drive(obs, ports)
        if self._n_finite < MIN_FINITE:
            return Status.FAILURE

        match = self._best_match(self._cur)
        if match is None:
            return Status.FAILURE
        rec, _dstar, kstar, d = match
        if self._ambiguous(d, kstar):
            return Status.FAILURE      # self-congruent shape: refuse, do not guess

        theta = self._choose_exit(self._cur, rec, kstar)
        if theta is None:
            return Status.FAILURE

        ports["place_revisited"] = True
        ports["reroute_bearing"] = float(theta)
        self._active = {"rec": rec, "k": kstar, "theta": theta,
                        "psi0": self._psi, "t0": obs.t, "prog": 0.0}
        return self._drive(obs, ports)

    def _drive(self, obs, ports):
        a = self._active
        if obs.t - a["t0"] > REROUTE_TIMEOUT_S:
            self._active = None
            return Status.FAILURE
        if a["prog"] >= EXIT_WINDOW_M:
            rec = a["rec"]
            rec["exits"].append(_wrap(a["theta"] + a["k"] * self._step))
            rec["used"] = self._travel
            self._active = None
            return Status.SUCCESS

        err = _wrap(a["theta"] - (self._psi - a["psi0"]))
        wz = float(np.clip(TURN_GAIN * err, -1.0, 1.0))
        vx = CRAWL_SPEED if abs(err) <= ALIGN_TOL else 0.0
        ports.drive(Command(vx, 0.0, wz))
        self._last_cmd = (vx, 0.0, wz)
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}


def micro(seed=0):
    """Ring corridor round a central island, goal behind a side doorway.

    The ring is the only thing offering long forward ranges, so a gap follower laps it
    and never turns 90 degrees into the 1.6 m doorway. The node's escape is shape-keyed
    and needs three visits to the same T-junction: visit one records the exit west,
    visit two must leave at least 60 deg from west so it reverses east, visit three has
    both lateral exits used and only north - the doorway - clears the separation and the
    1.2 m corridor test. Long corridor stretches are 180-degree self-congruent and are
    refused by the ambiguity guard, so the reroute fires at junctions and corners only.
    """
    world = room(10.0, 11.9, boxes=[
        (2.0, 2.0, 8.0, 5.5),      # central island: makes the free space a closed ring
        (0.0, 7.0, 4.0, 9.5),      # north wall, west of the doorway
        (5.6, 7.0, 10.0, 9.5),     # north wall, east of the doorway
    ])
    return MicroScenario(
        id="shape_revisit_ring_branch",
        world=world,
        start=(5.0, 1.0, 0.0),
        goal=(1.7, 10.7),
        time_limit=300.0,
        seed=seed,
        goal_known=False,
    )
