"""Base navigator: the last-resort leaf of the behaviour tree.

One Action. It always accepts, always emits a Twist, always returns RUNNING.
Reactive corridor-clearance gap following toward the goal, with heading
commitment (anti-livelock), a Bug-style wall-following detour (anti-local-
minimum), timed escapes on contact or no-progress, and a novelty-driven
exploration mode for episodes where the goal has not been disclosed.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status

# --- kinematics / safety -------------------------------------------------
_V_MAX = 0.62          # nominal forward speed (runner clips at 0.8)
_W_MAX = 1.0           # yaw rate limit
_VY_MAX = 0.32         # we only ever use lateral motion for centring/escape
_SAFE_R = 0.28         # corridor half-width used for clearance (robot radius)
_TIGHT_R = 0.20        # squeeze-mode corridor half-width
_MIN_CLEAR = 0.50      # below this a direction is heavily penalised
_STOP_D = 0.45         # clearance at which forward speed reaches zero
_SLOW_D = 1.35         # clearance above which we run at full speed
_D_PREF = 3.0          # clearance saturation for scoring
_REP_D = 0.80          # near-obstacle repulsion radius

# --- commitment / hysteresis --------------------------------------------
_COMMIT_MIN = 0.70     # s: heading is held at least this long
_COMMIT_MAX = 4.00     # s: forced re-evaluation
_REFINE_W = 0.55       # rad: local refinement window around held heading
_MARGIN_SEEK = 0.45    # score a rival must beat the held heading by
_MARGIN_WALL = 0.15

# --- progress / recovery -------------------------------------------------
_STUCK_WIN = 3.0       # s of belief-pose history with no displacement
_STUCK_D = 0.25        # m
_TRAJ_KEEP = 4.5       # s
_NOPROG_T = 7.0        # s without closing on the goal -> detour
_WALL_MAX = 28.0       # s ceiling on one wall-following detour
_WALL_GAIN = 0.55      # m of goal progress needed to leave the detour
_ESCAPE_T = 0.90       # s
_WALL_STANDOFF = 0.70  # m

# --- exploration ---------------------------------------------------------
_CELL = 0.80           # m, visit grid resolution
_NOVELTY_T = 25.0      # s for a visited cell to look fresh again
_EXPLORE_COMMIT = 4.0  # s


def _wrap(a):
    """Wrap angle(s) to [-pi, pi). Works on scalars and ndarrays."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _nearest(cand, target):
    return int(np.argmin(np.abs(_wrap(cand - float(target)))))


def _corridor(r, ang, cand, radius, rmax):
    """Free distance along each candidate direction for a corridor of the
    given half-width: a return blocks a direction only if it falls inside
    the swept rectangle, and then it blocks it at its along-track range."""
    if r.size == 0:
        return np.full(cand.shape, rmax, dtype=float)
    d = _wrap(ang[None, :] - cand[:, None])
    along = r[None, :] * np.cos(d)
    lat = np.abs(r[None, :] * np.sin(d))
    blocking = (lat <= radius) & (along > -0.10)
    vals = np.where(blocking, np.maximum(along, 0.0), rmax)
    return np.clip(vals.min(axis=1), 0.0, rmax)


class BaseNavigator(Action):
    """Last-resort indoor navigator. Never declines, never stops driving."""

    PORTS = (
        Port("nav_heading", PortType.BEARING, Direction.OUT),
        Port("nav_stuck", PortType.FLAG, Direction.OUT),
        Port("nav_progress", PortType.CONFIDENCE, Direction.OUT),
    )
    PRECONDITION = (
        "Always applicable: this is the base of the fallback and it accepts "
        "every tick unconditionally."
    )
    EFFECT = (
        "Moves the robot one control step along a committed, collision-free "
        "heading that advances it toward the goal."
    )
    TAGS = (
        "base",
        "navigation",
        "gap-following",
        "obstacle-avoidance",
        "commitment",
        "recovery",
        "exploration",
    )

    # ------------------------------------------------------------------ #
    # lifecycle
    # ------------------------------------------------------------------ #
    def __init__(self, *args, **kwargs):
        try:
            super().__init__(*args, **kwargs)
        except TypeError:
            super().__init__()
        self._reset_state(None)

    def reset(self, scenario=None, rng=None):
        self._reset_state(rng)

    def _reset_state(self, rng):
        self._rng = rng if rng is not None else np.random.default_rng(20240913)
        self._last_t = None
        self._heading_w = None      # committed heading, world (belief) frame
        self._commit_t = 0.0
        self._mode = "seek"         # seek | wall | escape
        self._side = 1              # +1 wall on the left, -1 on the right
        self._escape_until = -1.0
        self._post_escape = "wall"
        self._wall_t0 = 0.0
        self._wall_entry_d = float("inf")
        self._best_d = float("inf")
        self._best_t = 0.0
        self._traj = []
        self._visits = {}
        self._stuck = False
        self._explore_t = -1e9

    def precondition(self, obs, ports):
        return True

    # ------------------------------------------------------------------ #
    # tick
    # ------------------------------------------------------------------ #
    def tick(self, obs, ports):
        try:
            vx, vy, wz, heading, stuck, prog = self._compute(obs)
        except Exception:
            # The base must never leave the tree with nothing driving.
            vx, vy, wz = 0.12, 0.0, 0.6
            heading, stuck, prog = 0.0, True, 0.0

        vx = float(np.clip(vx, -0.40, 0.80))
        vy = float(np.clip(vy, -0.60, 0.60))
        wz = float(np.clip(wz, -_W_MAX, _W_MAX))
        if not (math.isfinite(vx) and math.isfinite(vy) and math.isfinite(wz)):
            vx, vy, wz = 0.10, 0.0, 0.5

        ports["nav_heading"] = float(_wrap(heading))
        ports["nav_stuck"] = bool(stuck)
        ports["nav_progress"] = float(np.clip(prog, 0.0, 1.0))
        ports.drive(Command(vx, vy, wz))
        return Status.RUNNING

    # ------------------------------------------------------------------ #
    # the actual navigator
    # ------------------------------------------------------------------ #
    def _compute(self, obs):
        t = float(obs.t)
        if self._last_t is None or t < self._last_t - 1e-6:
            self._reset_state(self._rng)  # fresh episode without a reset()
        self._last_t = t

        # ---------------- scan ----------------
        rmax = 6.0
        try:
            rm = float(obs.range_max)
            if math.isfinite(rm) and rm > 0.2:
                rmax = rm
        except Exception:
            pass

        r = np.asarray(obs.ranges, dtype=float).ravel()
        ang = np.asarray(obs.range_angles, dtype=float).ravel()
        n = min(r.size, ang.size)
        r, ang = r[:n], ang[:n]
        r = np.where(np.isfinite(r), r, rmax)
        r = np.clip(r, 0.0, rmax)
        ang = _wrap(np.where(np.isfinite(ang), ang, 0.0))

        if n >= 8:
            cand = ang.copy()
        else:
            cand = _wrap(np.linspace(-math.pi, math.pi, 72, endpoint=False))

        clear = _corridor(r, ang, cand, _SAFE_R, rmax)
        squeezing = False
        if float(clear.max()) < 0.80:
            clear = _corridor(r, ang, cand, _TIGHT_R, rmax)
            squeezing = True

        i_fwd = _nearest(cand, 0.0)
        i_left = _nearest(cand, math.pi / 2.0)
        i_right = _nearest(cand, -math.pi / 2.0)
        r_min = float(r.min()) if n else rmax
        i_min = int(np.argmin(r)) if n else i_fwd

        # ---------------- belief pose ----------------
        pose_ok = True
        try:
            p = obs.pose_estimate
            x, y, yaw = float(p[0]), float(p[1]), float(p[2])
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(yaw)):
                raise ValueError
        except Exception:
            x = y = yaw = 0.0
            pose_ok = False

        try:
            perr = float(obs.pose_error_estimate)
            if not math.isfinite(perr) or perr < 0.0:
                perr = 0.0
        except Exception:
            perr = 0.0
        trust = 1.0 / (1.0 + perr / 1.5)

        # ---------------- goal ----------------
        gx = gy = None
        try:
            g = obs.goal
            if g is not None:
                gxx, gyy = float(g[0]), float(g[1])
                if math.isfinite(gxx) and math.isfinite(gyy):
                    gx, gy = gxx, gyy
        except Exception:
            gx = gy = None

        have_goal = (gx is not None) and pose_ok
        if have_goal:
            dx, dy = gx - x, gy - y
            d_goal = math.hypot(dx, dy)
            b_goal = _wrap(math.atan2(dy, dx) - yaw)
        else:
            d_goal = float("inf")
            b_goal = 0.0

        # ---------------- bookkeeping: visits, motion, progress ----------
        if pose_ok:
            self._visits[(int(math.floor(x / _CELL)), int(math.floor(y / _CELL)))] = t
            if (not self._traj) or (t - self._traj[-1][0]) >= 0.20:
                self._traj.append((t, x, y))
            while self._traj and (t - self._traj[0][0]) > _TRAJ_KEEP:
                self._traj.pop(0)

        stuck = False
        if self._traj and (t - self._traj[0][0]) >= _STUCK_WIN:
            far = max(math.hypot(x - sx, y - sy) for (_ts, sx, sy) in self._traj)
            stuck = far < _STUCK_D
        self._stuck = stuck

        if have_goal:
            if d_goal < self._best_d - 0.15:
                self._best_d = d_goal
                self._best_t = t
            elif not math.isfinite(self._best_d):
                self._best_d = d_goal
                self._best_t = t
        progress = 1.0 if not have_goal else float(
            np.clip(1.0 - (t - self._best_t) / _NOPROG_T, 0.0, 1.0)
        )

        # ---------------- contact -> escape ------------------------------
        touching = False
        try:
            touching = bool(obs.contact)
        except Exception:
            touching = False
        if (touching or r_min < 0.22) and self._mode != "escape":
            self._enter_escape(t, "wall" if have_goal else "seek")

        # ---------------- escape behaviour -------------------------------
        if self._mode == "escape":
            if t < self._escape_until:
                away = _wrap(ang[i_min] + math.pi) if n else math.pi
                back_clear = clear[_nearest(cand, away)]
                if back_clear > 0.40 and r_min < 1.2:
                    vx = 0.34 * math.cos(away)
                    vy = float(np.clip(0.34 * math.sin(away), -_VY_MAX, _VY_MAX))
                else:
                    vx, vy = -0.10, 0.0
                wz = 0.95 * self._side
                return vx, vy, wz, away, True, progress
            # escape finished: resume, flipping the detour side
            self._side = -self._side
            self._mode = self._post_escape
            self._heading_w = None
            self._traj = []
            self._wall_t0 = t
            self._wall_entry_d = d_goal
            self._best_t = t
            self._explore_t = -1e9

        # ---------------- mode transitions -------------------------------
        if self._mode == "seek":
            no_prog = have_goal and (t - self._best_t) > _NOPROG_T
            if stuck or no_prog:
                self._enter_wall(t, cand, clear, b_goal, d_goal)
        elif self._mode == "wall":
            if stuck:
                self._enter_escape(t, "wall")
            elif (t - self._wall_t0) > _WALL_MAX:
                self._enter_escape(t, "wall")
            elif have_goal:
                goal_clear = clear[_nearest(cand, b_goal)]
                closer = d_goal < (self._wall_entry_d - _WALL_GAIN)
                open_run = goal_clear >= min(d_goal, 2.0) - 0.05
                if closer and open_run:
                    self._mode = "seek"
                    self._heading_w = None
                    self._best_t = t
                    self._traj = []
            elif (t - self._wall_t0) > 12.0:
                self._mode = "seek"   # exploring: go back to open-space seeking
                self._heading_w = None
                self._traj = []

        if self._mode == "escape":   # re-entered just above
            return self._compute_escape_tick(t, cand, clear, ang, i_min, n, progress)

        # ---------------- desired heading per mode ------------------------
        clear_n = np.clip(clear / _D_PREF, 0.0, 1.0)
        fwd_pref = 0.5 * (1.0 + np.cos(cand))
        held_rf = None if self._heading_w is None else _wrap(self._heading_w - yaw)
        if held_rf is None:
            commit_term = np.zeros_like(cand)
        else:
            commit_term = 0.5 * (1.0 + np.cos(_wrap(cand - held_rf)))

        need_recommit = (
            held_rf is None
            or (t - self._commit_t) > _COMMIT_MAX
            or clear[_nearest(cand, held_rf)] < _MIN_CLEAR * 0.80
        )

        if self._mode == "wall":
            desired = self._wall_heading(cand, clear, ang, r, n)
            w_des, w_clear, w_fwd, w_com, w_nov = 1.90, 1.05, 0.20, 0.25, 0.0
            margin = _MARGIN_WALL
        elif have_goal:
            desired = b_goal
            w_des = 1.55 * (0.45 + 0.55 * trust)
            w_clear, w_fwd, w_com, w_nov = 1.15, 0.30, 0.55, 0.30
            margin = _MARGIN_SEEK
        else:
            desired = self._explore_heading(t, cand, clear, clear_n, fwd_pref,
                                            commit_term, x, y, yaw, pose_ok,
                                            need_recommit, held_rf)
            w_des, w_clear, w_fwd, w_com, w_nov = 1.10, 1.30, 0.30, 0.55, 0.60
            margin = _MARGIN_SEEK

        novelty = np.zeros_like(cand)
        if w_nov > 0.0 and pose_ok and need_recommit:
            novelty = self._novelty(t, x, y, yaw, cand, clear)

        goal_term = 1.0 - np.abs(_wrap(cand - desired)) / math.pi
        score = (
            w_clear * clear_n
            + w_des * goal_term
            + w_fwd * fwd_pref
            + w_com * commit_term
            + w_nov * novelty
        )
        tight = clear < _MIN_CLEAR
        score = score - np.where(tight, 3.0 * (1.0 - clear / _MIN_CLEAR), 0.0)

        # ---------------- commitment with hysteresis ----------------------
        i_best = int(np.argmax(score))
        if held_rf is None:
            i_tgt = i_best
            self._commit_t = t
        else:
            near = np.abs(_wrap(cand - held_rf)) <= _REFINE_W
            if near.any():
                idx = np.flatnonzero(near)
                i_loc = int(idx[int(np.argmax(score[idx]))])
            else:
                i_loc = _nearest(cand, held_rf)
            locked = (t - self._commit_t) < _COMMIT_MIN
            blocked = clear[i_loc] < _MIN_CLEAR * 0.80
            if blocked or (t - self._commit_t) > _COMMIT_MAX:
                i_tgt = i_best
                self._commit_t = t
            elif locked or score[i_best] <= score[i_loc] + margin:
                i_tgt = i_loc            # hold, refined locally
            else:
                i_tgt = i_best
                self._commit_t = t

        # clean shot at a known goal: take it
        if have_goal and self._mode == "seek":
            i_g = _nearest(cand, b_goal)
            if clear[i_g] >= min(d_goal + 0.25, rmax * 0.98):
                i_tgt = i_g

        target = float(cand[i_tgt])
        self._heading_w = _wrap(yaw + target)

        # ---------------- heading -> twist --------------------------------
        err = _wrap(target)
        gate_d = min(float(clear[i_tgt]), float(clear[i_fwd]))
        gate = float(np.clip((gate_d - _STOP_D) / (_SLOW_D - _STOP_D), 0.0, 1.0))
        align = max(0.0, (math.cos(err) - 0.20) / 0.80)
        vx = _V_MAX * gate * align
        if squeezing:
            vx *= 0.55

        wz = 2.0 * err
        if vx < 0.06 and abs(err) > 0.05:
            wz = math.copysign(max(abs(wz), 0.40), err)

        # corridor centring
        vy = 0.0
        cl, cr = float(clear[i_left]), float(clear[i_right])
        if cl < 1.20 and cr < 1.20:
            vy += 0.30 * (cl - cr)

        # near-obstacle repulsion
        if n:
            m = r < _REP_D
            if bool(m.any()):
                w = (_REP_D - r[m]) / _REP_D
                tot = float(w.sum())
                if tot > 1e-6:
                    rx = -float(np.sum(w * np.cos(ang[m]))) / tot
                    ry = -float(np.sum(w * np.sin(ang[m]))) / tot
                    vy += 0.45 * ry
                    if rx < 0.0:
                        vx += 0.30 * rx

        vy = float(np.clip(vy, -_VY_MAX, _VY_MAX))

        # ease off right on top of the goal, but never fully stop unless the
        # belief is trustworthy -- standing still on a bad belief is a loss.
        if have_goal:
            vx *= float(np.clip(d_goal / 0.80, 0.20, 1.0))
            if d_goal < 0.15 and perr < 0.50:
                return 0.0, 0.0, 0.0, target, stuck, progress

        if vx < 0.05 and abs(err) < 0.05 and gate > 0.25:
            vx = 0.10   # never idle in open space

        return vx, vy, wz, target, (stuck or self._mode != "seek"), progress

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    def _compute_escape_tick(self, t, cand, clear, ang, i_min, n, progress):
        away = _wrap(ang[i_min] + math.pi) if n else math.pi
        back_clear = float(clear[_nearest(cand, away)])
        if back_clear > 0.40:
            vx = 0.34 * math.cos(away)
            vy = float(np.clip(0.34 * math.sin(away), -_VY_MAX, _VY_MAX))
        else:
            vx, vy = -0.10, 0.0
        return vx, vy, 0.95 * self._side, away, True, progress

    def _enter_escape(self, t, resume_mode):
        self._mode = "escape"
        self._escape_until = t + _ESCAPE_T
        self._post_escape = resume_mode
        self._heading_w = None
        self._traj = []

    def _enter_wall(self, t, cand, clear, b_goal, d_goal):
        left = cand > 0.30
        right = cand < -0.30
        fwdish = np.abs(cand) < 1.80
        l_open = float(np.mean(clear[left & fwdish])) if bool((left & fwdish).any()) else 0.0
        r_open = float(np.mean(clear[right & fwdish])) if bool((right & fwdish).any()) else 0.0
        if abs(l_open - r_open) > 0.20:
            self._side = 1 if l_open > r_open else -1
        elif abs(b_goal) > 1e-3:
            self._side = 1 if b_goal > 0.0 else -1
        else:
            self._side = 1 if float(self._rng.random()) > 0.5 else -1
        self._mode = "wall"
        self._wall_t0 = t
        self._wall_entry_d = d_goal
        self._heading_w = None
        self._traj = []

    def _wall_heading(self, cand, clear, ang, r, n):
        """Tangent along the wall on the committed side, with a standoff
        correction. Fed to the scorer, so avoidance still has the last word."""
        s = float(self._side)
        if n == 0:
            return 0.60 * s
        sector = (s * ang > 0.15) & (s * ang < 2.60)
        if not bool(sector.any()):
            return 0.60 * s
        idx = np.flatnonzero(sector)
        j = int(idx[int(np.argmin(r[idx]))])
        d_wall = float(r[j])
        if not math.isfinite(d_wall) or d_wall > 3.5:
            return 0.60 * s
        phi = float(ang[j])
        corr = float(np.clip(d_wall - _WALL_STANDOFF, -0.55, 0.55))
        return float(_wrap(phi - s * (math.pi / 2.0) + s * 1.10 * corr))

    def _explore_heading(self, t, cand, clear, clear_n, fwd_pref, commit_term,
                         x, y, yaw, pose_ok, need_recommit, held_rf):
        """Pick (and hold) a direction into open, not-recently-visited space."""
        if held_rf is not None and not need_recommit and (t - self._explore_t) < _EXPLORE_COMMIT:
            return held_rf
        nov = self._novelty(t, x, y, yaw, cand, clear) if pose_ok else np.zeros_like(cand)
        s = 1.30 * clear_n + 0.85 * nov + 0.35 * fwd_pref + 0.45 * commit_term
        s = s - np.where(clear < _MIN_CLEAR, 3.0 * (1.0 - clear / _MIN_CLEAR), 0.0)
        self._explore_t = t
        return float(cand[int(np.argmax(s))])

    def _novelty(self, t, x, y, yaw, cand, clear):
        """Per-direction freshness of the belief-space cells it leads into."""
        out = np.zeros(cand.shape, dtype=float)
        for i in range(cand.size):
            reach = min(float(clear[i]), 3.5)
            if reach < 0.80:
                continue
            th = yaw + float(cand[i])
            cx, sy = math.cos(th), math.sin(th)
            acc = 0.0
            cnt = 0
            for dd in (1.0, 2.0, 3.0):
                if dd > reach - 0.30:
                    break
                key = (int(math.floor((x + dd * cx) / _CELL)),
                       int(math.floor((y + dd * sy) / _CELL)))
                seen = self._visits.get(key)
                if seen is None:
                    acc += 1.0
                else:
                    acc += min(1.0, max(0.0, (t - seen) / _NOVELTY_T))
                cnt += 1
            if cnt:
                out[i] = acc / cnt
        return out
