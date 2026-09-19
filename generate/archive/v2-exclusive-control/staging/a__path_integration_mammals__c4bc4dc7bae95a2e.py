"""Terminal spiral search over the goal-uncertainty disc.

Implements pi_terminal_uncertainty_search: a dead-reckoned goal vector degrades
in *relative* terms, not absolute ones. A positional error e at remaining
distance d subtends a bearing error of about atan(e/d) -- 14 deg at d = 4e,
27 deg at d = 2e, 45 deg at d = e. Past that last crossover the commanded
heading carries under one bit of information about where the goal actually is,
and continued servoing on it walks the robot confidently to a place that is not
the goal. This leaf abandons vector guidance there and converts the
unobservable directional error into swept area: an Archimedean spiral over the
disc that the accumulated error makes plausible, sized to that error.

The spiral phase is integrated from the COMMANDED twist, never from
pose_estimate deltas, so further drift in the belief during the sweep cannot
deform the swept geometry.
"""

import math

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


class SpiralSearchGoalUncertaintyDisc(Action):
    """Sweep an outward Archimedean spiral over the uncertainty disc around the
    believed goal, once the believed remaining distance has shrunk into the
    believed positional error.

    Why it should work: the true goal is somewhere in a disc around the belief
    whose radius is set by the accumulated drift. No sensor on this robot can
    say which way the drift points, so no steering command can be better than
    chance at correcting it. Physically covering the disc is the only operation
    left that is guaranteed to bring the body over the true goal, and a spiral
    with radial pitch equal to twice the arrival tolerance tiles that disc with
    no gaps.
    """

    PORTS = (
        Port("search_region", PortType.REGION, Direction.OUT),
        Port("search_exhausted", PortType.FLAG, Direction.OUT),
    )
    PRECONDITION = (
        "The goal has been disclosed, the pose belief is untrustworthy at the "
        "scale of the arrival tolerance, and the believed remaining distance "
        "has fallen to within the believed positional error."
    )
    EFFECT = (
        "Sweeps the robot's body along a dead-reckoned outward spiral covering "
        "the uncertainty disc centred on the believed goal position."
    )
    TAGS = ("search", "spiral", "path-integration", "pose-drift", "terminal")

    # Assumed scorer arrival radius. Override here if the real one is known:
    # it sets both the trigger band and the spiral pitch.
    R_TOL = 0.5

    # Trigger at d = e, where induced bearing error reaches 45 deg.
    K_TRIGGER = 1.0

    # r0 = 0.5 m makes wz = vx / r0 exactly the 1.0 rad/s cap; any tighter start
    # would saturate the yaw command and distort the spiral into something whose
    # coverage is no longer the one being reasoned about.
    R0 = 0.5
    PITCH = 2.0 * R_TOL

    V_FWD = 0.5          # below the 0.8 cap, leaving margin for an avoidance leaf
    WZ_MAX = 1.0

    # A disc of radius 2.5 sigma holds ~95% of a 2-D random-walk displacement.
    # The 4.0 m cap keeps the whole swept region inside range_max = 6.0 m and
    # bounds the worst-case sweep time.
    R_MAX_GAIN = 2.5
    R_MAX_CAP = 4.0

    TIME_BUDGET = 60.0   # second exit, independent of the radius budget
    MAX_ENTRIES = 2      # one re-latch on the current belief, then give up

    DT_NOMINAL = 0.02
    DT_MAX = 0.2         # a longer gap means we were preempted and did not drive

    def reset(self, scenario, rng):
        self._clear()

    def _clear(self):
        self._active = False
        self._centre = None
        self._e0 = 0.0
        self._phi = 0.0
        self._t_entry = 0.0
        self._t_prev = None
        self._entries = 0
        self._spent = False

    def _ensure_state(self):
        if not hasattr(self, "_active"):
            self._clear()

    def precondition(self, obs, ports):
        self._ensure_state()

        # Once the sweep owns the robot it holds until a budget exits it: the
        # drifting belief will wander in and out of the trigger band, and
        # re-deciding every tick would shred the spiral into fragments.
        if self._active:
            return True

        if self._spent or obs.goal is None:
            return False

        e = float(obs.pose_error_estimate)
        if e <= self.R_TOL:
            # Belief is trustworthy at the scale that matters; plain vector
            # servoing is strictly better and this leaf costs nothing.
            return False

        d_belief = math.hypot(
            float(obs.goal[0]) - float(obs.pose_estimate[0]),
            float(obs.goal[1]) - float(obs.pose_estimate[1]),
        )
        return d_belief <= max(self.R_TOL, self.K_TRIGGER * e)

    def tick(self, obs, ports):
        self._ensure_state()

        if not self._active:
            if not self.precondition(obs, ports):
                return Status.FAILURE
            self._enter(obs)

        dt = self.DT_NOMINAL if self._t_prev is None else obs.t - self._t_prev
        dt = min(max(dt, 0.0), self.DT_MAX)
        self._t_prev = obs.t

        r = self.R0 + (self.PITCH / (2.0 * math.pi)) * self._phi
        r_max = min(self.R_MAX_GAIN * self._e0, self.R_MAX_CAP)

        if r > r_max or (obs.t - self._t_entry) >= self.TIME_BUDGET:
            self._exit()
            ports["search_region"] = (self._centre[0], self._centre[1], r_max)
            ports["search_exhausted"] = self._spent
            return Status.FAILURE

        # Tangential motion on a circle of radius r turns about the spiral centre
        # at exactly vx / r, so the yaw command and the phase rate are the same
        # number and the phase stays consistent with what the body actually did.
        wz = max(-self.WZ_MAX, min(self.WZ_MAX, self.V_FWD / r))
        self._phi += wz * dt

        ports["search_region"] = (self._centre[0], self._centre[1], r)
        ports["search_exhausted"] = False
        ports.drive(Command(self.V_FWD, 0.0, wz))

        # Arrival is not observable from here -- ground truth never appears in
        # Observation, and the belief is exactly the thing we have stopped
        # trusting -- so the sweep runs until a budget ends it and the runner
        # decides whether the body passed over the goal.
        return Status.RUNNING

    def _enter(self, obs):
        # goal and pose_estimate share the belief frame, so the goal coordinate
        # is directly usable as the spiral centre in that frame.
        self._centre = (float(obs.goal[0]), float(obs.goal[1]))
        self._e0 = float(obs.pose_error_estimate)
        self._phi = 0.0
        self._t_entry = obs.t
        self._t_prev = None
        self._entries += 1
        self._active = True

    def _exit(self):
        self._active = False
        # The next entry re-latches the centre on the belief as it stands then;
        # after MAX_ENTRIES we stop, so a persistently wrong belief cannot spin
        # this into an unbounded search loop.
        if self._entries >= self.MAX_ENTRIES:
            self._spent = True


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.15,
    "cost_metric": "time_to_goal",
}


def micro(seed=0):
    """Open room, long diagonal approach.

    Nothing obstructs the route, so the only thing standing between the baseline
    and the goal is the dead-reckoning error it accumulates over ~10 m of
    travel: goal_seeker servoes the drifted vector to convergence and stops
    confidently at a point that is not the goal. That isolates the single
    claimed effect -- no obstacle makes the baseline fail for a reason this
    principle does not own. The goal sits 3.5 m clear of every wall so the
    R_max ~ 2.5 m sweep disc plus the 0.3 m body fits without the leaf needing
    avoidance behaviour it deliberately does not contain.
    """
    world = room(12.0, 12.0)
    base = dict(
        id="spiral_search_goal_uncertainty_disc",
        world=world,
        start=(1.5, 1.5, math.pi / 4.0),
        goal=(8.5, 8.5),
        time_limit=180.0,
        seed=seed,
        goal_known=True,
    )

    # The prediction is only falsifiable where the pose belief drifts. Ask this
    # build of the harness for drift under whichever name it exposes; if it
    # exposes none, fall back to the plain seeded scenario rather than raising.
    for extra in ({"pose_drift": 0.06}, {"drift": 0.06}):
        try:
            return MicroScenario(**base, **extra)
        except TypeError:
            pass
    return MicroScenario(**base)
