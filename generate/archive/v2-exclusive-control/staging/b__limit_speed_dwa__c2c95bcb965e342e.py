"""Dynamic-window velocity selection for a Go2 with a 181-beam planar scan."""

import math

import numpy as np

from core.observation import Command
from harness.micro import MicroScenario, room
from nodes.contract import Action, Direction, Port, PortType, Status


class SelectDynamicWindowVelocity(Action):
    """Pick the Twist directly out of velocity space instead of picking a heading first.

    Every candidate (v, w) is read as the circular arc the body would follow if it
    held that command, so obstacle checking is a circle-intersection test against the
    raw scan rather than a plan. The candidate set is pruned twice: to the ADMISSIBLE
    set (braking at the maximum rate still stops us short of the nearest obstacle
    lying on that arc, Eq. 14) and to the DYNAMIC WINDOW (commands reachable from the
    one we issued last within a single interval, Eq. 15). Whatever survives is scored
    by G = 0.8*heading + 0.1*clearance + 0.1*vel, smoothed, and the argmax is emitted.

    Why it should work: commitment to a velocity is made conditional on being able to
    undo it, so the robot decelerates exactly as much as the *visible* free space
    requires and never enters a state from which stopping is impossible. The heading
    term alone would park us against the first blocking obstacle; the clearance and
    velocity terms are what let it flow around one.

    Only the heading term reads pose_estimate. Clearance and velocity come from the
    robot-frame scan, so under pose drift this degrades into a collision-free wander
    rather than into collisions.
    """

    PORTS = (Port("selected_speed", PortType.SPEED, Direction.OUT),)
    PRECONDITION = "The goal has been disclosed and the robot is not currently in contact with anything."
    EFFECT = (
        "Emits the highest-scoring forward/turn command that is both reachable within one "
        "control interval and stoppable by maximum braking before the nearest obstacle on its arc."
    )
    TAGS = ("dynamic_window", "velocity_space", "local_planner", "collision_avoidance", "goal_seeking")

    # Vs: the frozen policy is not characterised in reverse, so reverse v is not sampled.
    V_MAX = 0.8
    W_MAX = 1.0

    # Slew limits per control interval, and the interval itself. The accelerations are
    # the paper's window-to-range ratios rescaled onto our saturation limits; braking
    # accelerations equal driving accelerations, as in the paper's Example 1.
    DT = 0.25
    DV = 0.11
    DW = 0.175
    ACC_V = 0.45
    ACC_W = 0.7

    N_V = 7
    N_W = 15

    ALPHA = 0.8
    BETA = 0.1
    GAMMA = 0.1

    # Go2 footprint ~0.70 x 0.31 m -> circumscribed radius ~0.38, rounded to 0.40 plus
    # a 0.10 m margin.
    R_INFL = 0.50

    STRAIGHT_EPS = 1e-3
    ZERO_V_EPS = 1e-6
    GOAL_TOL = 0.30

    # A gap larger than this in obs.t means the leaf was not ticked last step, i.e. some
    # other behaviour was driving, so the window is no longer centred on our own command.
    REACTIVATION_GAP = 0.5

    # Defaults live on the class so tick() is safe even if reset() was never called.
    _prev_v = 0.0
    _prev_w = 0.0
    _last_t = None

    def reset(self, scenario, rng):
        self._prev_v = 0.0
        self._prev_w = 0.0
        self._last_t = None

    def precondition(self, obs, ports):
        return obs.goal is not None and not obs.contact

    def tick(self, obs, ports):
        if obs.goal is None or obs.contact:
            ports["selected_speed"] = 0.0
            return Status.FAILURE

        self._sync_window_centre(obs.t)

        goal_x, goal_y = obs.goal
        x, y, _ = obs.pose_estimate
        if math.hypot(goal_x - x, goal_y - y) <= self.GOAL_TOL:
            self._prev_v = 0.0
            self._prev_w = 0.0
            ports["selected_speed"] = 0.0
            ports.drive(Command(0.0, 0.0, 0.0))
            return Status.SUCCESS

        range_max = float(obs.range_max)
        obs_x, obs_y = self._obstacle_points(obs, range_max)

        # Vd intersected with Vs: sample inside the reachable window, clipped to limits.
        v_samples = np.linspace(
            max(0.0, self._prev_v - self.DV), min(self.V_MAX, self._prev_v + self.DV), self.N_V
        )
        w_samples = np.linspace(
            max(-self.W_MAX, self._prev_w - self.DW), min(self.W_MAX, self._prev_w + self.DW), self.N_W
        )

        scores = np.zeros((v_samples.size, w_samples.size), dtype=float)
        admissible = np.zeros_like(scores, dtype=bool)

        for i, v in enumerate(v_samples):
            for j, w in enumerate(w_samples):
                if v <= self.ZERO_V_EPS:
                    # Pure rotation sweeps no arc, so it is never obstructed; giving it
                    # vel = 0 is what keeps it from winning while anything else is open.
                    dist = range_max
                    vel = 0.0
                else:
                    dist = self._arc_clearance(v, w, obs_x, obs_y, range_max)
                    vel = v / self.V_MAX

                # Eq. (14): keep only commands we could brake out of in time.
                if v > math.sqrt(2.0 * dist * self.ACC_V):
                    continue
                if abs(w) > math.sqrt(2.0 * dist * self.ACC_W):
                    continue

                admissible[i, j] = True
                scores[i, j] = (
                    self.ALPHA * self._heading_term(v, w, obs.pose_estimate, goal_x, goal_y)
                    + self.BETA * (dist / range_max)
                    + self.GAMMA * vel
                )

        if not admissible.any():
            # Nothing survives: no command lets us translate and still stop. Emit nothing
            # and hand over to a recovery behaviour rather than guessing.
            self._prev_v = 0.0
            self._prev_w = 0.0
            ports["selected_speed"] = 0.0
            return Status.FAILURE

        smoothed = np.where(admissible, self._box_blur(scores, admissible), -np.inf)
        i, j = np.unravel_index(int(np.argmax(smoothed)), smoothed.shape)

        v_star = float(v_samples[i])
        w_star = float(w_samples[j])
        self._prev_v = v_star
        self._prev_w = w_star

        ports["selected_speed"] = abs(v_star)
        # Holonomic vy is deliberately unused: fixing vy = 0 is what makes the paper's
        # radius-v/w arc geometry exactly valid for this body.
        ports.drive(Command(v_star, 0.0, w_star))
        return Status.RUNNING

    def _sync_window_centre(self, t):
        """Re-centre the dynamic window on (0, 0) whenever this leaf resumes control."""
        if self._last_t is None or t < self._last_t or (t - self._last_t) > self.REACTIVATION_GAP:
            self._prev_v = 0.0
            self._prev_w = 0.0
        self._last_t = t

    @classmethod
    def _obstacle_points(cls, obs, range_max):
        """Scan returns as robot-frame points; infinite and out-of-range beams dropped."""
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        keep = np.isfinite(ranges) & (ranges > 0.0) & (ranges <= range_max + 1e-9)
        r = ranges[keep]
        a = angles[keep]
        return r * np.cos(a), r * np.sin(a)

    @classmethod
    def _arc_clearance(cls, v, w, obs_x, obs_y, range_max):
        """Distance travelled along the (v, w) arc before the inflated body touches a point.

        range_max stands in for the paper's "large constant" when the arc is clear: 6 m
        is literally as far as we can see, so claiming more would be inventing free space.
        """
        if obs_x.size == 0:
            return range_max

        if abs(w) < cls.STRAIGHT_EPS:
            ahead = (np.abs(obs_y) <= cls.R_INFL) & (obs_x > 0.0)
            if not np.any(ahead):
                return range_max
            return min(range_max, max(0.0, float(np.min(obs_x[ahead])) - cls.R_INFL))

        radius = v / w
        centre_y = radius
        d = np.hypot(obs_x, obs_y - centre_y)
        hit = np.abs(d - abs(radius)) <= cls.R_INFL
        if not np.any(hit):
            return range_max

        # Sweep angle from the robot's own bearing about the arc centre, taken in the
        # direction of travel; phi is the paper's gamma, renamed to free up the weight.
        start_angle = math.atan2(-radius, 0.0)
        point_angle = np.arctan2(obs_y[hit] - centre_y, obs_x[hit])
        phi = np.mod(math.copysign(1.0, w) * (point_angle - start_angle), 2.0 * math.pi)
        return min(range_max, float(np.min(abs(radius) * phi)))

    @classmethod
    def _heading_term(cls, v, w, pose, goal_x, goal_y):
        """1 - |bearing to goal| / pi, evaluated where the command plus full braking lands us."""
        x, y, yaw = pose

        if abs(w) < cls.STRAIGHT_EPS:
            end_yaw = yaw
            x += v * cls.DT * math.cos(yaw)
            y += v * cls.DT * math.sin(yaw)
        else:
            radius = v / w
            end_yaw = yaw + w * cls.DT
            x += radius * (math.sin(end_yaw) - math.sin(yaw))
            y -= radius * (math.cos(end_yaw) - math.cos(yaw))

        # The braking tail: the yaw and the forward travel still to come once we start
        # decelerating at the maximum rate. Without it, fast commands look identical to
        # slow ones and admissibility would be the only thing holding speed down.
        end_yaw += math.copysign(w * w / (2.0 * cls.ACC_W), w)
        tail = v * v / (2.0 * cls.ACC_V)
        x += tail * math.cos(end_yaw)
        y += tail * math.sin(end_yaw)

        theta = cls._wrap(math.atan2(goal_y - y, goal_x - x) - end_yaw)
        return 1.0 - abs(theta) / math.pi

    @staticmethod
    def _box_blur(values, mask):
        """3x3 box blur that ignores inadmissible cells entirely.

        Averaging a forbidden neighbour in as a zero would read it as merely bad rather
        than unavailable, which biases the argmax away from commands next to the window
        edge.
        """
        weights = mask.astype(float)
        rows, cols = values.shape
        padded_values = np.pad(values * weights, 1)
        padded_weights = np.pad(weights, 1)

        total = np.zeros_like(values)
        count = np.zeros_like(values)
        for di in range(3):
            for dj in range(3):
                total += padded_values[di:di + rows, dj:dj + cols]
                count += padded_weights[di:di + rows, dj:dj + cols]
        return total / np.maximum(count, 1.0)

    @staticmethod
    def _wrap(angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi


PREDICTION = {
    "statement": (
        "Substituting this velocity-space selection for a reactive avoider that first "
        "chooses a heading and then a speed reduces the mean number of collisions per "
        "episode by at least 40% across matched seeds in unmapped indoor scenes, because "
        "commands from which the robot cannot brake to a stop before the nearest obstacle "
        "on its own arc are never issued."
    ),
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.4,
    "cost_metric": "time_to_goal",
}


def micro(seed=0):
    """A doorway with an obstacle parked just past its exit.

    The straight line from start to goal lies inside the lower door post, so the goal is
    only reachable through a 1.30 m opening. A gap follower finds that opening but takes
    it at whatever speed it was already using, and 0.55 m beyond it a pillar spans the
    exit: at constant speed there is no turn radius that clears both. The pinch
    half-widths (0.65 m) sit just above the 0.50 m inflation radius, so an admissible
    command always exists for the dynamic-window node -- it simply has to be a slow one.
    """
    world = room(
        6.0,
        5.0,
        boxes=[
            (2.85, 0.00, 3.15, 1.90),   # lower door post
            (2.85, 3.20, 3.15, 5.00),   # upper door post
            (3.70, 1.30, 4.30, 2.70),   # pillar blocking the door's exit
        ],
    )
    return MicroScenario(
        id="dwa_doorway_with_blocked_exit",
        world=world,
        start=(0.70, 0.80, 0.0),
        goal=(5.20, 0.90),
        time_limit=75.0,
        seed=seed,
        goal_known=True,
    )
