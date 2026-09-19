"""Bounded, sector-fanned excursions from a re-anchored home base.

Implements `home_base_anchored_excursions`: exploration of an unmapped indoor
space is structured as repeated out-and-back trips centred on a latched home
anchor, rather than one open-ended reactive walk.  The anchor is stored both as
a belief-frame point and as a 181-beam range signature; on return the signature
is re-fitted to the live scan so that the anchor's *expression in the drifted
belief frame* is re-zeroed, which is the only part of the paper's "reset the
path integrator" that a node above a frozen policy can actually do.  Excursion
bearings are drawn from allocentric 60 degree sectors that have not been used
yet, so successive trips fan around the anchor instead of re-walking one
corridor, and the record of which bearings are still unexplored stays meaningful
because the frame it is recorded in is re-zeroed before drift destroys it.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

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

# --- excursion budget (all of these numbers are the card's) -----------------
_D_BUDGET = 12.0          # m of outbound path length before we must return
_ERR_BUDGET = 0.75        # m of pose_error_estimate growth before we must return
_DEAD_END = 1.5           # m: no bearing in the forward half-plane beats this
_HOME_RADIUS = 1.0        # m: close enough to attempt a re-anchor
_LOST_SLACK = 10.0        # m added to 1.5x the outbound leg before giving up

# --- clearance / speed law --------------------------------------------------
_STOP_RANGE = 0.35        # m: Go2 body half-diagonal plus margin
_FULL_RANGE = 0.80        # m: full speed resumes here
_DETOUR_RANGE = 1.2       # m: a sliding detour bearing must beat this
_CONE_HALF = math.radians(50.0)   # clearance cone about the travel direction
_SLIDE_HALF = math.radians(90.0)  # how far the detour may slide
_SMOOTH_ARC = math.radians(15.0)
_VX = 0.6
_WZ_GAIN = 1.8
_WZ_MAX = 1.0

# --- scan matching ----------------------------------------------------------
_XY_SPAN, _XY_STEP = 2.0, 0.25
_YAW_SPAN, _YAW_STEP = math.radians(30.0), math.radians(5.0)
_MATCH_ACCEPT = 0.35      # m mean absolute beam residual
_MATCH_RATIO = 0.9        # and must beat the zero-offset residual by this much
_MIN_MATCH_BEAMS = 20

_N_SECTORS = 6
_SECTOR = 2.0 * math.pi / _N_SECTORS

_OUTBOUND, _RETURN = 0, 1


def _wrap(a):
    """Wrap to [-pi, pi). Works on scalars and arrays."""
    return (np.asarray(a) + math.pi) % (2.0 * math.pi) - math.pi


def _rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _sector_of(bearing):
    return int(math.floor((float(bearing) % (2.0 * math.pi)) / _SECTOR)) % _N_SECTORS


class ExploreByAnchoredExcursions(Action):
    """Explore an unmapped space as bounded excursions from a home base.

    Mechanism: latch a home anchor (position plus a lidar signature) on the first
    tick; drive outbound along a bearing chosen from a 60 degree sector that no
    previous excursion has used; terminate the leg on a path-length budget, a
    pose-error-growth budget, or a dead end; return to the anchor, re-fit the
    signature to the live scan and correct the anchor so the next home vector
    points at the same *physical* place despite belief drift; repeat.

    Why it should work: the record of "which bearings are still unexplored" is
    kept in the belief frame, and that frame drifts.  Returning to a recognisable
    place and re-zeroing the anchor keeps the record usable, so coverage fans out
    instead of degenerating into a random walk once drift exceeds the room scale.
    """

    PORTS = (
        Port("home_anchor", PortType.REGION, Direction.OUT),
        Port("anchor_match_confidence", PortType.CONFIDENCE, Direction.OUT),
    )
    PRECONDITION = (
        "The goal has not been disclosed yet (obs.goal is None) and the robot is "
        "free-running rather than in contact, so the search must generate its own "
        "targets."
    )
    EFFECT = (
        "Covers ground that previous excursions did not, by driving bounded "
        "out-and-back trips whose bearings fan around a re-anchored home base."
    )
    TAGS = ("explore", "search", "coverage", "home-base", "path-integration", "anchor")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._init_state()

    # ------------------------------------------------------------------ state
    def _init_state(self):
        self._anchor = None          # A: anchor xy in the belief frame
        self._home_scan = None       # S: clipped range signature at the anchor
        self._home_valid = None      # which beams of S actually returned
        self._home_yaw = 0.0
        self._e0 = 0.0               # pose_error_estimate at the last re-anchor
        self._d = 0.0                # path length since the last re-anchor
        self._d_out = 0.0            # path length of the outbound leg
        self._phase = _OUTBOUND
        self._target = 0.0           # allocentric excursion bearing
        self._sector = 0             # sector the current excursion belongs to
        self._used = set()
        self._lru = []               # sectors in order of use, oldest first
        self._last_xy = None
        self._yielded = False
        self._beam_order = None      # argsort of the beam bearings
        self._beam_ang = None        # those bearings, sorted, in [-pi, pi)
        self._smooth_w = 3
        self._rng = None

    def reset(self, scenario, rng):
        self._init_state()
        self._rng = rng

    # ------------------------------------------------------------- beam setup
    def _prepare_beams(self, obs):
        angles = np.asarray(obs.range_angles, dtype=float)
        if self._beam_ang is not None and self._beam_ang.size == angles.size:
            return
        wrapped = _wrap(angles)
        order = np.argsort(wrapped)
        self._beam_order = order
        self._beam_ang = wrapped[order]
        spacing = float(np.median(np.diff(self._beam_ang)))
        if not np.isfinite(spacing) or spacing <= 0.0:
            spacing = 2.0 * math.pi / max(1, angles.size)
        w = int(round(_SMOOTH_ARC / spacing))
        self._smooth_w = max(3, w | 1)

    def _scan(self, obs):
        """Return (clipped ranges, finite mask, smoothed ranges), sorted by bearing."""
        raw = np.asarray(obs.ranges, dtype=float)[self._beam_order]
        rmax = float(obs.range_max)
        finite = np.isfinite(raw)
        clipped = np.where(finite, np.clip(raw, 0.0, rmax), rmax)
        pad = self._smooth_w // 2
        ext = np.concatenate([clipped[-pad:], clipped, clipped[:pad]])
        kernel = np.ones(self._smooth_w) / float(self._smooth_w)
        return clipped, finite, np.convolve(ext, kernel, mode="valid")

    def _nearest_beam(self, ang):
        """Index of the beam whose bearing is closest to each (wrapped) angle."""
        sa = self._beam_ang
        n = sa.size
        k = np.searchsorted(sa, ang)
        k0 = (k - 1) % n
        k1 = k % n
        d0 = np.abs(_wrap(ang - sa[k0]))
        d1 = np.abs(_wrap(ang - sa[k1]))
        return np.where(d0 <= d1, k0, k1)

    # ----------------------------------------------------------- node surface
    def precondition(self, obs, ports):
        if self._yielded or obs.goal is not None:
            return False
        return not bool(obs.contact)

    def tick(self, obs, ports):
        # The excursion structure is a search policy, not a travel policy: once
        # the goal latches we hand over for good and never re-arm.
        if obs.goal is not None:
            self._yielded = True
            return Status.SUCCESS
        if obs.contact:
            return Status.FAILURE

        self._prepare_beams(obs)
        clipped, finite, smooth = self._scan(obs)
        pose = np.asarray(obs.pose_estimate, dtype=float)
        pos, yaw = pose[:2].copy(), float(pose[2])
        err = float(obs.pose_error_estimate)

        if self._anchor is None:
            self._latch_anchor(pos, yaw, clipped, finite, err)
            self._target = self._pick_bearing(smooth, yaw)
            ports["anchor_match_confidence"] = 1.0
        else:
            step = float(np.hypot(*(pos - self._last_xy)))
            # A belief teleport is drift, not travel; do not spend budget on it.
            if step < 1.0:
                self._d += step
        self._last_xy = pos

        home_vec = self._anchor - pos
        home_dist = float(np.hypot(*home_vec))

        if self._phase == _OUTBOUND:
            if self._leg_done(smooth, yaw, err):
                self._phase = _RETURN
                self._d_out = self._d
        else:
            if home_dist <= _HOME_RADIUS:
                self._reanchor(pos, yaw, clipped, finite, err, smooth, ports)
                home_vec = self._anchor - pos
            elif (self._d - self._d_out) > 1.5 * self._d_out + _LOST_SLACK:
                # Lost-anchor escape: the leaf must not be able to deadlock on a
                # home it can no longer reach.
                self._latch_anchor(pos, yaw, clipped, finite, err)
                self._used.clear()
                self._lru = []
                self._target = self._pick_bearing(smooth, yaw)
                ports["anchor_match_confidence"] = 0.0
                home_vec = self._anchor - pos

        if self._phase == _OUTBOUND:
            base = self._target
        else:
            base = math.atan2(float(home_vec[1]), float(home_vec[0]))

        self._drive(base, yaw, clipped, smooth, ports)
        ports["home_anchor"] = (
            float(self._anchor[0]),
            float(self._anchor[1]),
            _HOME_RADIUS,
        )
        return Status.RUNNING

    # ------------------------------------------------------------- excursions
    def _latch_anchor(self, pos, yaw, clipped, finite, err):
        self._anchor = pos.copy()
        self._home_scan = clipped.copy()
        self._home_valid = finite.copy()
        self._home_yaw = yaw
        self._e0 = err
        self._d = 0.0
        self._d_out = 0.0
        self._phase = _OUTBOUND

    def _pick_bearing(self, smooth, yaw):
        """Freest allocentric bearing in a sector no excursion has used yet."""
        allo = _wrap(yaw + self._beam_ang)
        sectors = np.floor((allo % (2.0 * math.pi)) / _SECTOR).astype(int) % _N_SECTORS
        mask = ~np.isin(sectors, np.array(sorted(self._used), dtype=int)) \
            if self._used else np.ones(allo.size, dtype=bool)

        if not mask.any():
            # All six sectors spent: wipe the record and re-walk the oldest one.
            lru = self._lru[0] if self._lru else 0
            self._used.clear()
            self._lru = []
            mask = sectors == lru
            if not mask.any():
                mask = np.ones(allo.size, dtype=bool)

        idx = np.flatnonzero(mask)
        best = idx[int(np.argmax(smooth[idx]))]
        self._sector = int(sectors[best])
        return float(allo[best])

    def _leg_done(self, smooth, yaw, err):
        if self._d >= _D_BUDGET:
            return True
        if err - self._e0 >= _ERR_BUDGET:
            return True
        rel = _wrap(self._target - yaw)
        ahead = np.abs(_wrap(self._beam_ang - rel)) <= _SLIDE_HALF
        return bool(ahead.any() and smooth[ahead].max() <= _DEAD_END)

    def _mark_sector_used(self, sector):
        self._used.add(sector)
        if sector in self._lru:
            self._lru.remove(sector)
        self._lru.append(sector)

    # ------------------------------------------------------------- re-anchor
    def _reanchor(self, pos, yaw, clipped, finite, err, smooth, ports):
        offset, residual, base_residual = self._match_home(pos, yaw, clipped, finite)
        accepted = (
            offset is not None
            and residual < _MATCH_ACCEPT
            and residual < _MATCH_RATIO * base_residual
        )
        if accepted:
            dx, dy, dth = offset
            # The fitted offset maps belief-frame points onto the home signature,
            # so the physical home sits at -R(-dth)*t in belief home coords.
            corr = _rot(self._home_yaw) @ (-(_rot(-dth) @ np.array([dx, dy])))
            self._anchor = self._anchor + corr
            ports["anchor_match_confidence"] = float(
                np.clip(1.0 - residual / _MATCH_ACCEPT, 0.0, 1.0)
            )
        else:
            ports["anchor_match_confidence"] = 0.0

        # The budget resets whether or not the match was accepted: thrashing on a
        # rejected match costs more than one excursion run on a stale anchor.
        self._mark_sector_used(self._sector)
        self._e0 = err
        self._d = 0.0
        self._d_out = 0.0
        self._phase = _OUTBOUND
        self._target = self._pick_bearing(smooth, yaw)

    def _match_home(self, pos, yaw, clipped, finite):
        """Grid-search the offset that best re-projects the live scan onto S.

        Returns (best offset or None, best residual, zero-offset residual).
        """
        live_valid = finite
        if int(live_valid.sum()) < _MIN_MATCH_BEAMS:
            return None, float("inf"), float("inf")

        rmax = float(np.max(self._home_scan)) if self._home_scan.size else 6.0
        world = pos[None, :] + clipped[:, None] * np.stack(
            [np.cos(yaw + self._beam_ang), np.sin(yaw + self._beam_ang)], axis=1
        )
        # Live points expressed in the home sensor frame under the current belief.
        u = (world - self._anchor[None, :]) @ _rot(self._home_yaw)

        grid = np.arange(-_XY_SPAN, _XY_SPAN + 1e-9, _XY_STEP)
        gx, gy = np.meshgrid(grid, grid, indexing="ij")
        gx, gy = gx.ravel(), gy.ravel()
        yaws = np.arange(-_YAW_SPAN, _YAW_SPAN + 1e-9, _YAW_STEP)

        best_res, best_off, zero_res = float("inf"), None, float("inf")
        for dth in yaws:
            rotated = u @ _rot(dth).T
            xs = rotated[None, :, 0] + gx[:, None]
            ys = rotated[None, :, 1] + gy[:, None]
            ang = _wrap(np.arctan2(ys, xs))
            rng = np.minimum(np.hypot(xs, ys), rmax)
            idx = self._nearest_beam(ang)
            pred = self._home_scan[idx]
            ok = self._home_valid[idx] & live_valid[None, :]
            counts = ok.sum(axis=1)
            totals = np.where(ok, np.abs(rng - pred), 0.0).sum(axis=1)
            res = np.where(counts >= _MIN_MATCH_BEAMS, totals / np.maximum(counts, 1), np.inf)

            k = int(np.argmin(res))
            if res[k] < best_res:
                best_res = float(res[k])
                best_off = (float(gx[k]), float(gy[k]), float(dth))
            if abs(dth) < 1e-9:
                z = np.flatnonzero((np.abs(gx) < 1e-9) & (np.abs(gy) < 1e-9))
                if z.size:
                    zero_res = float(res[int(z[0])])

        return best_off, best_res, zero_res

    # ----------------------------------------------------------------- motion
    def _drive(self, base_bearing, yaw, clipped, smooth, ports):
        rel = float(_wrap(base_bearing - yaw))
        d_min = self._cone_min(clipped, rel)

        if d_min < _STOP_RANGE:
            rel = self._slide(smooth, rel)
            d_min = self._cone_min(clipped, rel)

        scale = float(np.clip((d_min - _STOP_RANGE) / (_FULL_RANGE - _STOP_RANGE), 0.0, 1.0))
        wz = float(np.clip(_WZ_GAIN * rel, -_WZ_MAX, _WZ_MAX))
        vx = _VX * max(0.0, math.cos(rel)) * scale
        ports.drive(Command(vx, 0.0, wz))

    def _cone_min(self, clipped, rel):
        cone = np.abs(_wrap(self._beam_ang - rel)) <= _CONE_HALF
        if not cone.any():
            return float(np.min(clipped))
        return float(np.min(clipped[cone]))

    def _slide(self, smooth, rel):
        """Nearest bearing within +/-90 deg of `rel` with room to move."""
        off = np.abs(_wrap(self._beam_ang - rel))
        cand = np.flatnonzero((off <= _SLIDE_HALF) & (smooth > _DETOUR_RANGE))
        if cand.size:
            return float(self._beam_ang[cand[int(np.argmin(off[cand]))]])
        # Mine, not the card's: with no admissible detour, vx is zero and a held
        # heading gives wz zero too, which is a fixed point. Turn to the freest
        # bearing so the node always rotates its way out.
        return float(self._beam_ang[int(np.argmax(smooth))])


def micro(seed=0):
    """Hub-and-spoke hall: four corridors, each ending in an occluded pocket.

    A memoryless gap follower commits to whichever arm it is pointed down and
    shuttles between the widest openings it can see; nothing in it fans coverage
    across bearings.  The goal sits in the north-east pocket, 6.36 m from the
    start with the north-east block squarely on the line of sight, so it cannot
    be disclosed until something drives up the north arm.
    """
    world = room(
        14.0,
        14.0,
        boxes=[
            (0.0, 2.5, 6.0, 6.0),      # south-west block, pocket left open below
            (0.0, 0.0, 3.5, 2.5),
            (8.0, 0.0, 14.0, 3.5),     # south-east block, pocket off the east arm
            (8.0, 3.5, 11.5, 6.0),
            (2.5, 8.0, 6.0, 14.0),     # north-west block, pocket off the west arm
            (0.0, 11.5, 2.5, 14.0),
            (8.0, 8.0, 14.0, 11.5),    # north-east block, pocket holds the goal
            (10.5, 11.5, 14.0, 14.0),
        ],
    )
    return MicroScenario(
        id="anchored_excursions_four_arm_pockets",
        world=world,
        start=(7.0, 7.0, 0.0),
        goal=(9.6, 12.8),
        time_limit=240.0,
        seed=seed,
        goal_known=False,
    )
