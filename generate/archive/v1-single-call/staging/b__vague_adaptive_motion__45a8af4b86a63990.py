"""MoveAdaptively - a leaf that adapts its motion to local conditions.

Card: vague_adaptive_motion (confidence: low).
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}


def _wrap(a):
    """Wrap angle(s) to (-pi, pi]. Works on scalars and arrays."""
    return np.arctan2(np.sin(a), np.cos(a))


class MoveAdaptively(Action):
    """Emit a Twist whose speed, heading weighting and mode adapt to conditions.

    The mechanism is "move adaptively, continuously". Made concrete for a
    181-beam 2D scan, that is two adaptations layered on one drive command:

    * Parameter adaptation. Candidate headings are scored on goal alignment,
      on the clearance of the swath the body would sweep through them, and on
      agreement with last tick's choice. The clearance weight is a function of
      measured local clutter and the forward speed is a function of forward
      clearance, so the same code cruises in open rooms and picks its way
      through furniture.

    * Mode adaptation. Local greedy choice can be locally optimal and globally
      wrong (a concave obstacle). When the distance-to-goal belief stops
      improving, that is the environment reporting the greedy mode has failed,
      so the node adapts by committing to boundary following on the more open
      side until it beats the distance it stalled at. The commitment is the
      point: it is what a memoryless gap follower cannot do.

    Both adaptations are driven only by `obs`, so nothing here names a room, a
    coordinate or a map.
    """

    PORTS = (
        Port("motion_confidence", PortType.CONFIDENCE, Direction.OUT),
    )
    PRECONDITION = "The robot is being driven this tick, i.e. motion is wanted."
    EFFECT = "Emits a Twist whose heading, speed and mode are adapted to the current scan and to recent progress."
    TAGS = ("action", "drive", "adaptive", "clearance", "local-minimum", "boundary-follow")

    SOURCE = "vague_adaptive_motion (test fixture card; span not verified)"
    CONFIDENCE = "low"

    # --- kinematic limits (the runner clips to these anyway) ---
    V_MAX = 0.8
    W_MAX = 1.0
    YAW_GAIN = 2.0

    # --- geometry of the body, in metres ---
    BODY_RADIUS = 0.3
    SWATH = 0.35          # half-angle of the corridor a candidate heading needs
    MIN_CLEAR = 0.55      # a heading closer-blocked than this is inadmissible
    SAFE_BUBBLE = 0.55    # inside this, sidestep away from the nearest return

    # --- speed adaptation ---
    STOP_D = 0.45         # forward clearance at which the speed law reaches zero
    CRUISE_D = 2.0        # forward clearance at which it reaches V_MAX
    OPEN_REF = 3.0        # clearance that counts as "fully open" when scoring
    CLUTTER_D = 1.5       # a forward return nearer than this counts as clutter

    # --- mode adaptation ---
    PROGRESS_EPS = 0.15   # distance improvement that counts as progress
    STALL_WINDOW = 4.0    # seconds without progress before the mode switches
    ESCAPE_MAX = 40.0     # seconds before a failed escape is abandoned
    ESCAPE_GAIN = 0.6     # distance the escape must win back to hand control back
    WALL_D = 0.8          # stand-off held from the followed boundary
    FRONT_BLOCK = 1.0     # forward clearance at which we turn off the boundary

    GOAL_TOL = 0.35

    # ------------------------------------------------------------------
    # per-episode state
    # ------------------------------------------------------------------
    def _init_state(self):
        self._mode = "seek"
        self._prev_bearing = 0.0
        self._best_dist = float("inf")
        self._best_t = 0.0
        self._escape_ref = float("inf")
        self._escape_t0 = 0.0
        self._side = 1.0            # +1 keeps the followed boundary on the left
        self._last_escape_expired = False

    def reset(self, scenario, rng):
        self._init_state()

    def precondition(self, obs, ports):
        # The card's trigger is "whenever the robot is moving", i.e. no gate.
        # Inventing a second condition here would be inventing a second node.
        return True

    # ------------------------------------------------------------------
    # scan helpers
    # ------------------------------------------------------------------
    def _sector_min(self, r, angles, centre, half_width):
        sel = np.abs(_wrap(angles - centre)) <= half_width
        if not np.any(sel):
            return float(np.max(r))
        return float(np.min(r[sel]))

    def _half_openness(self, r, angles):
        """Mean return depth on the left and right flanks."""
        left = (angles > 0.35) & (angles < 2.0)
        right = (angles < -0.35) & (angles > -2.0)
        lo = float(np.mean(r[left])) if np.any(left) else 0.0
        ro = float(np.mean(r[right])) if np.any(right) else 0.0
        return lo, ro

    # ------------------------------------------------------------------
    def tick(self, obs, ports):
        if not hasattr(self, "_mode"):
            self._init_state()

        angles = np.asarray(obs.range_angles, dtype=float)
        raw = np.asarray(obs.ranges, dtype=float)
        # inf means "no return inside sensor range", which is free space, not a
        # missing value - saturate it so it can be arithmetic like any other beam.
        r = np.where(np.isfinite(raw), raw, obs.range_max)
        r = np.clip(r, 0.0, obs.range_max)

        # obs.goal is None until disclosed. With no goal there is no alignment
        # term and no notion of progress, so the node degrades to pure adaptive
        # open-space motion rather than guessing at a direction.
        x, y, yaw = obs.pose_estimate
        if obs.goal is None:
            goal_bearing = None
            dist = None
        else:
            gx, gy = obs.goal
            dist = math.hypot(gx - x, gy - y)
            goal_bearing = float(_wrap(math.atan2(gy - y, gx - x) - yaw))

        if dist is not None and dist <= self.GOAL_TOL:
            ports["motion_confidence"] = 1.0
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        if obs.contact:
            # Touching something is the most emphatic condition report there is:
            # back off before doing anything cleverer.
            lo, ro = self._half_openness(r, angles)
            turn = 0.6 if lo >= ro else -0.6
            ports["motion_confidence"] = 0.0
            ports.drive(Command(-0.3, 0.0, turn))
            return Status.RUNNING

        self._update_mode(obs, r, angles, dist)

        if self._mode == "escape":
            desired, clearance = self._escape_bearing(r, angles)
            speed_scale = 0.8
        else:
            desired, clearance = self._seek_bearing(r, angles, goal_bearing)
            speed_scale = 1.0

        # Light low-pass on the heading; the smoothness term in the score does
        # most of the anti-dither work, this just takes the edge off mode swaps.
        desired = float(_wrap(self._prev_bearing + 0.6 * _wrap(desired - self._prev_bearing)))
        self._prev_bearing = desired

        front = self._sector_min(r, angles, 0.0, self.SWATH)
        span = self.CRUISE_D - self.STOP_D
        vx = self.V_MAX * float(np.clip((front - self.STOP_D) / span, 0.0, 1.0))
        # Rotate in place rather than arc when the heading error is large.
        vx *= max(0.0, math.cos(desired)) * speed_scale
        if dist is not None:
            vx = min(vx, max(0.15, 1.2 * dist))

        vy = 0.0
        nearest = int(np.argmin(r))
        if r[nearest] < self.SAFE_BUBBLE:
            # Holonomic sidestep away from whatever is inside the bubble; cheap,
            # and it keeps min_clearance up in doorways without stopping.
            vy = -0.3 * math.copysign(1.0, math.sin(angles[nearest]))

        wz = float(np.clip(self.YAW_GAIN * desired, -self.W_MAX, self.W_MAX))

        ports["motion_confidence"] = float(np.clip(clearance / self.OPEN_REF, 0.0, 1.0))
        ports.drive(Command(float(vx), float(vy), wz))
        return Status.RUNNING

    # ------------------------------------------------------------------
    def _update_mode(self, obs, r, angles, dist):
        """Switch between greedy seeking and committed boundary following."""
        if dist is None:
            return

        if dist < self._best_dist - self.PROGRESS_EPS:
            self._best_dist = dist
            self._best_t = obs.t

        if self._mode == "seek":
            if not np.isfinite(self._best_dist):
                self._best_dist = dist
                self._best_t = obs.t
                return
            if obs.t - self._best_t > self.STALL_WINDOW:
                lo, ro = self._half_openness(r, angles)
                side = 1.0 if lo >= ro else -1.0
                # If the previous escape ran out of time, the side it chose is
                # evidence against itself; take the other one.
                if self._last_escape_expired and side == self._side:
                    side = -side
                self._side = side
                self._mode = "escape"
                self._escape_ref = self._best_dist
                self._escape_t0 = obs.t
            return

        expired = obs.t - self._escape_t0 > self.ESCAPE_MAX
        won_back = dist < self._escape_ref - self.ESCAPE_GAIN
        if expired or won_back:
            self._last_escape_expired = expired and not won_back
            self._mode = "seek"
            self._best_dist = dist
            self._best_t = obs.t

    # ------------------------------------------------------------------
    def _seek_bearing(self, r, angles, goal_bearing):
        """Score forward-ish headings and return the best one plus its clearance."""
        cand = angles[np.abs(angles) <= 1.75]
        if cand.size == 0:
            return 0.0, float(np.min(r))

        # Clearance of the swath each candidate would sweep, not of a single ray:
        # a single ray slips through gaps the body does not fit through.
        diff = np.abs(_wrap(angles[None, :] - cand[:, None]))
        clear = np.where(diff <= self.SWATH, r[None, :], np.inf).min(axis=1)

        forward = np.abs(angles) < 1.0
        clutter = float(np.mean(r[forward] < self.CLUTTER_D)) if np.any(forward) else 0.0

        # The adaptation: in clutter, openness outweighs pointing at the goal.
        w_goal = 0.0 if goal_bearing is None else 1.0
        w_open = 0.6 + 1.6 * clutter
        w_smooth = 0.3

        align = np.zeros_like(cand) if goal_bearing is None else np.cos(_wrap(cand - goal_bearing))
        openness = np.clip(clear / self.OPEN_REF, 0.0, 1.0)
        smooth = np.cos(_wrap(cand - self._prev_bearing))

        score = w_goal * align + w_open * openness + w_smooth * smooth
        score = np.where(clear < self.MIN_CLEAR, score - 10.0, score)

        best = int(np.argmax(score))
        if clear[best] < self.MIN_CLEAR:
            # Nothing admissible ahead: spin toward the more open flank.
            lo, ro = self._half_openness(r, angles)
            return (1.2 if lo >= ro else -1.2), float(clear[best])
        return float(cand[best]), float(clear[best])

    def _escape_bearing(self, r, angles):
        """Hold a fixed stand-off from the boundary on the committed side."""
        side = self._side
        d_side = self._sector_min(r, angles, side * math.pi / 2.0, 0.4)
        front = self._sector_min(r, angles, 0.0, self.SWATH)

        if front < self.FRONT_BLOCK:
            # Inner corner: turn away from the boundary we are hugging.
            return -side * 1.2, front

        # Positive error means we have drifted off the boundary (or rounded an
        # outer corner and lost it), so steer back toward it; negative steers off.
        err = float(np.clip(d_side - self.WALL_D, -1.0, 1.0))
        desired = side * float(np.clip(1.2 * err, -0.9, 0.9))
        return desired, min(front, d_side)


def micro(seed=0):
    """A deep three-sided pocket sitting exactly on the goal bearing.

    From the start the pocket mouth is the widest gap in the scan and its
    interior returns read as several metres of free space, so a gap follower
    that steers at the widest gap making progress drives into it and then has
    no progress-making gap left. Corridors 2.1 m wide above and below the arms
    make the detour available to anything that notices it has stopped
    improving.
    """
    world = room(
        10.0,
        7.0,
        boxes=[
            (3.0, 4.6, 6.0, 4.9),   # top arm
            (3.0, 2.1, 6.0, 2.4),   # bottom arm
            (6.0, 2.1, 6.3, 4.9),   # back of the pocket
        ],
    )
    return MicroScenario(
        id="deep_pocket_adaptive_motion",
        world=world,
        start=(1.0, 3.5, 0.0),
        goal=(8.3, 3.5),
        time_limit=150.0,
        seed=seed,
        goal_known=True,
    )
