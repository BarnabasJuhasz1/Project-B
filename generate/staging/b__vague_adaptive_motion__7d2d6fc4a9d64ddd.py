"""Adaptive motion for a Go2 with a 181-beam planar scan.

Implements card `vague_adaptive_motion`: the robot moves in a way that adapts to
whatever situation it is in, continuously, for as long as it is moving.

The card's mechanism is stated at a level ("move intelligently") that does not by
itself pick out a controller, so it is made concrete here on the three levels at
which a Twist can actually adapt, chosen so the card's own prediction (+0.5
goal_reached over reactive_avoid) is the thing under test:

  speed    scaled by how deep the free corridor along the chosen heading is, so
           the robot slows where the world is tight and opens up where it is not;
  heading  chosen among directions whose swept corridor admits a 0.3 m disc,
           trading goal alignment against openness;
  strategy the level that matters. The controller watches whether its current
           mode is producing progress. Goal-seeking that stops reducing the
           distance to the goal is, by observation, the wrong behaviour for the
           present surroundings, so the robot adapts by committing to a
           boundary-following detour until progress is available again.

Only the third level can clear the card's effect floor: a gap follower already
does the first two, so an implementation stopping there would restate the
baseline. Committing (the handedness is chosen once and kept for the episode)
is what prevents the adaptation from oscillating between two modes that each
look locally attractive.

Heading choice is scan-relative throughout; pose_estimate enters only as the
goal bearing and the progress measure, so drift degrades the detour trigger
gracefully rather than steering the robot into things.
"""

import math

import numpy as np

from core.observation import Command
from harness.micro import MicroScenario, room
from nodes.contract import Action, Direction, Port, PortType, Status

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}

# Half-width of the corridor a candidate heading must keep clear: robot radius
# plus a margin thin enough to still admit a 1.8 m gap when off-centre.
SAFE_HALF_WIDTH = 0.42
MIN_CLEAR = 0.8          # a heading with less corridor depth than this is not offered
BRAKE_DIST = 1.6         # corridor depth at which the robot is allowed full speed
V_MAX = 0.75
K_W = 1.6
CLEAR_WEIGHT = 0.45      # openness vs goal alignment in heading choice
TURN_PENALTY = 0.12      # mild preference for not swinging the body around

STALL_WINDOW = 3.0       # s of no progress before the mode adapts
STALL_EPS = 0.15         # m of closing that counts as progress
DETOUR_MIN_TIME = 1.5    # s the detour is held before it may hand back
DETOUR_GAIN = 0.4        # m the detour must win back before goal-seeking resumes
DETOUR_MAX_TIME = 25.0   # s after which the detour is re-evaluated from scratch

STANDOFF = 0.7           # m the boundary-follower tries to hold off the surface
STANDOFF_GAIN = 1.1
NEAR_FOV = 2.4           # rad; ignore surfaces behind when picking one to follow
NEAR_MAX = 3.0           # m; beyond this there is no surface worth following
GOAL_TOL = 0.35


def _wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MoveAdaptively(Action):
    """Move toward the goal with speed, heading and strategy adapted to conditions.

    Should work because the two failure modes of a fixed policy are complementary:
    a straight-line policy dies at the first wall, and a purely reactive one dies
    in any concavity where every locally-improving direction is blocked. Watching
    progress rather than geometry detects the second case without needing a map --
    "I have not got closer in three seconds" is observable, whereas "I am in a
    concavity" is not, from a single scan.
    """

    PORTS = (
        Port("detouring", PortType.FLAG, Direction.OUT),
        Port("adapted_speed", PortType.SPEED, Direction.OUT),
    )
    PRECONDITION = "Applies whenever the robot is under way and has not arrived."
    EFFECT = "Emits a Twist whose speed, heading and strategy are adapted to the currently observed surroundings."
    TAGS = ("adaptive", "navigation", "reactive", "detour", "twist")
    SOURCE = "Not a real source. This card is a test fixture; see the comment above."
    CONFIDENCE = "LOW"

    # ---- lifecycle ------------------------------------------------------

    def reset(self, scenario, rng):
        self._init_state()

    def _init_state(self):
        self._detouring = False
        self._side = 0.0          # committed handedness, chosen once per episode
        self._best_dist = math.inf
        self._best_t = 0.0
        self._detour_t0 = 0.0
        self._detour_dist0 = math.inf

    # ---- perception helpers ---------------------------------------------

    def _corridor_depth(self, obs):
        """Depth of free corridor for every candidate heading in the scan.

        A beam obstructs a candidate direction only if it falls inside the strip
        the robot body would sweep along that direction, which is what makes this
        a passability test rather than a raw range reading.
        """
        ang = np.asarray(obs.range_angles, dtype=float)
        rng = np.asarray(obs.ranges, dtype=float)
        rng = np.where(np.isfinite(rng), rng, obs.range_max)
        rng = np.clip(rng, 0.0, obs.range_max)

        delta = _wrap(ang[None, :] - ang[:, None])
        along = np.cos(delta)
        lateral = np.abs(rng[None, :] * np.sin(delta))
        blocking = (along > 0.0) & (lateral < SAFE_HALF_WIDTH)
        depth = np.where(blocking, rng[None, :] * along, np.inf)
        return ang, np.minimum(depth.min(axis=1), obs.range_max), rng

    def _select_heading(self, ang, depth, target, range_max):
        """Best admissible heading, balancing alignment with `target` against openness."""
        admissible = depth > MIN_CLEAR
        if not np.any(admissible):
            return None, 0.0
        score = (np.cos(_wrap(ang - target))
                 + CLEAR_WEIGHT * (depth / max(range_max, 1e-6))
                 - TURN_PENALTY * np.abs(ang) / math.pi)
        score = np.where(admissible, score, -np.inf)
        i = int(np.argmax(score))
        return float(ang[i]), float(depth[i])

    def _follow_tangent(self, ang, rng, depth):
        """Heading that keeps the nearest surface at STANDOFF on the committed side."""
        in_fov = np.abs(ang) < NEAR_FOV
        candidates = np.where(in_fov, rng, np.inf)
        i = int(np.argmin(candidates))
        d = float(candidates[i])
        if not np.isfinite(d) or d >= NEAR_MAX:
            # Nothing to hug: aim at the most open direction so the detour keeps moving.
            return float(ang[int(np.argmax(depth))])
        correction = float(np.clip(STANDOFF_GAIN * (STANDOFF - d), -0.7, 0.7))
        return _wrap(float(ang[i]) + self._side * (math.pi / 2.0 + correction))

    def _choose_side(self, ang, depth):
        left = float(depth[np.sin(ang) > 0.0].sum())
        right = float(depth[np.sin(ang) < 0.0].sum())
        return 1.0 if left >= right else -1.0

    # ---- contract -------------------------------------------------------

    def precondition(self, obs, ports):
        # The card's trigger is unconditional -- adaptive motion applies for as
        # long as the robot is moving at all. The only thing that retires it is
        # having arrived, which is only knowable when the goal is disclosed.
        if obs.goal is None:
            return True
        gx, gy = obs.goal
        px, py, _ = obs.pose_estimate
        return math.hypot(gx - px, gy - py) > GOAL_TOL

    def tick(self, obs, ports):
        if not hasattr(self, "_detouring"):
            self._init_state()

        ang, depth, rng = self._corridor_depth(obs)
        if self._side == 0.0:
            self._side = self._choose_side(ang, depth)

        px, py, yaw = obs.pose_estimate
        goal = obs.goal
        if goal is None:
            # Goal withheld: there is no progress signal, so the only adaptation
            # available is to keep moving through the most open free corridor.
            dist = math.inf
            goal_bearing = float(ang[int(np.argmax(depth))])
        else:
            gx, gy = goal
            dist = math.hypot(gx - px, gy - py)
            if dist <= GOAL_TOL:
                ports["detouring"] = False
                ports["adapted_speed"] = 0.0
                ports.drive(Command(0.0, 0.0, 0.0))
                return Status.SUCCESS
            goal_bearing = _wrap(math.atan2(gy - py, gx - px) - yaw)

        self._update_mode(obs.t, dist, goal_bearing, ang, depth)

        if self._detouring:
            target = self._follow_tangent(ang, rng, depth)
        else:
            target = goal_bearing

        heading, corridor = self._select_heading(target, ang, depth, obs.range_max) \
            if False else self._select_heading(ang, depth, target, obs.range_max)

        vx, vy, wz = self._command(obs, ang, rng, heading, corridor)

        ports["detouring"] = bool(self._detouring)
        ports["adapted_speed"] = float(math.hypot(vx, vy))
        ports.drive(Command(vx, vy, wz))
        return Status.RUNNING

    # ---- the adaptation itself ------------------------------------------

    def _update_mode(self, t, dist, goal_bearing, ang, depth):
        """Switch between goal-seeking and boundary-following on measured progress."""
        if not math.isfinite(dist):
            return

        if self._detouring:
            held = t - self._detour_t0
            if held > DETOUR_MIN_TIME and dist < self._detour_dist0 - DETOUR_GAIN:
                # Only hand back once the goal direction is actually open, otherwise
                # goal-seeking would re-enter the same trap on the next tick.
                _, corridor = self._select_heading(ang, depth, goal_bearing, 1.0)
                if corridor > MIN_CLEAR:
                    self._detouring = False
                    self._best_dist = dist
                    self._best_t = t
            elif held > DETOUR_MAX_TIME:
                self._detouring = False
                self._best_dist = dist
                self._best_t = t
            return

        if dist < self._best_dist - STALL_EPS:
            self._best_dist = dist
            self._best_t = t
        elif t - self._best_t > STALL_WINDOW:
            self._detouring = True
            self._detour_t0 = t
            self._detour_dist0 = self._best_dist

    def _command(self, obs, ang, rng, heading, corridor):
        front = np.abs(ang) < 0.35
        front_range = float(np.min(rng[front])) if np.any(front) else obs.range_max

        if obs.contact or front_range < 0.45:
            # Too close to steer out of: back off and rotate the committed way.
            return -0.25, 0.0, 0.9 * self._side

        if heading is None:
            # Every corridor is blocked; rotate in place until one opens.
            return 0.0, 0.0, 0.9 * self._side

        wz = float(np.clip(K_W * heading, -1.0, 1.0))
        gate = math.cos(heading) if abs(heading) < 1.2 else 0.0
        speed_scale = float(np.clip(corridor / BRAKE_DIST, 0.25, 1.0))
        vx = V_MAX * max(0.0, gate) * speed_scale

        # Small strafe away from anything crowding one flank; the Go2 can do this
        # without giving up its heading, so clearance costs nothing here.
        near_i = int(np.argmin(np.where(np.abs(ang) < NEAR_FOV, rng, np.inf)))
        near_d = float(rng[near_i])
        vy = 0.0
        if near_d < 0.7:
            push = (0.7 - near_d) / 0.7
            vy = float(np.clip(-0.3 * push * math.copysign(1.0, math.sin(ang[near_i])), -0.3, 0.3))

        return vx, vy, wz


def micro(seed=0):
    """A pocket whose mouth faces the start and whose back wall hides the goal.

    The straight line to the goal runs into the pocket, and the widest
    progress-making gap does too, so reactive_avoid enters and then finds no
    admissible direction at all. Getting out means giving up ~3 m of goal
    distance, which only a controller that adapts its strategy will elect to do.
    """
    world = room(
        9.0, 6.0,
        boxes=[
            (5.0, 1.8, 5.4, 4.2),   # back wall, between robot and goal
            (3.0, 3.9, 5.4, 4.2),   # upper arm
            (3.0, 1.8, 5.4, 2.1),   # lower arm
        ],
    )
    return MicroScenario(
        id="pocket_needs_mode_switch",
        world=world,
        start=(1.0, 3.0, 0.0),
        goal=(7.2, 3.0),
        time_limit=90.0,
        seed=seed,
        goal_known=True,
    )
