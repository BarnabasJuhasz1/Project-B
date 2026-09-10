"""Persistent-bearing exploration for a Go2 with a 181-beam planar scan."""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PRINCIPLE = {
    "mechanism": (
        "Exploration pays in net displacement, not in path length. A memoryless "
        "direction chooser re-decides every control step, so its heading "
        "autocorrelation is one step long and successive choices partially "
        "cancel: displacement grows diffusively, like sqrt(path), and the robot "
        "keeps re-entering the convex cell it has just cleared, because the "
        "instantaneously widest opening flips between opposite sides of that "
        "cell. Choosing one bearing and holding it until it is consumed or "
        "invalidated makes the heading autocorrelation as long as the "
        "commitment, so displacement grows ballistically between obstacles. The "
        "same locomotion budget then buys strictly more unvisited ground."
    ),
    "trigger": (
        "Any decision point at which no bearing has been handed to the robot - "
        "the goal is undisclosed or there is no goal - and the free-space "
        "profile offers more than one committable direction."
    ),
    "derivation": (
        "The 181 beams span a full circle, so one beam is ~2 deg. A bearing is "
        "committable only if the robot fits down it, so every candidate is "
        "scored on a circular rolling minimum over +-4 beams (~+-8 deg), which "
        "subtends about the robot's 0.3 m radius plus margin at the typical 2.5 "
        "m commitment range; the roll wraps exactly because the scan is closed. "
        "Candidates are scored depth * ((1+cos(theta))/2)^2, the squared cosine "
        "being a persistence weight that biases re-selection toward the current "
        "heading; a small floor keeps reversal selectable when it is the only "
        "option, so persistence is a bias and never a trap. The commitment "
        "budget is the committed depth minus a 0.5 m keep-out, capped at 4 m "
        "(~5 s at 0.8 m/s) so a stale belief cannot steer a long run. The "
        "commitment is stored as a robot-frame bearing and counter-rotated each "
        "tick by the yaw increment of the pose belief, and its budget is "
        "decremented by commanded speed times dt: neither uses absolute "
        "position, so the seeded random walk in pose_estimate cannot bend it. "
        "It is dropped on three events only - budget spent, depth along it "
        "below 0.9 m, or contact - and re-selection happens immediately."
    ),
}

PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.5,
    "cost_metric": "path_length",
}


def _wrap(a):
    """Wrap an angle, or an array of angles, into (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class CommitToExplorationBearing(Action):
    """Hold one exploration bearing until it is consumed or blocked.

    Mechanism: directional persistence. Greedy re-selection of the widest
    opening is instantaneously optimal and cumulatively poor, because its
    heading decorrelates every control step and net displacement then grows
    only like sqrt(path). This node picks a bearing whose corridor the robot
    actually fits down, commits a travel budget equal to that corridor's usable
    depth, and holds the bearing against every subsequent tick's opinion until
    the budget is spent, the corridor closes, or something is touched. Between
    obstacles the run is ballistic, which is what turns metres walked into
    ground newly seen.

    The node never reads obs.goal - not even to check it for None. That is the
    point: it supplies a direction of travel when nothing else does, and a
    sibling seeker takes over through the tree once the goal latches. The
    committed bearing is published so that sibling or a decorator can see what
    this node has committed to.

    Returns RUNNING for as long as it can commit to anything, and FAILURE only
    when the robot is enclosed - no bearing has a corridor it fits down.
    """

    PORTS = (
        Port("committed_exploration_bearing", PortType.BEARING, Direction.OUT),
    )
    PRECONDITION = (
        "At least one bearing has a corridor deep and wide enough for the robot "
        "to commit to, i.e. the robot is not enclosed."
    )
    EFFECT = (
        "Drives the robot along a single committed bearing until that bearing "
        "is consumed or invalidated, so displacement grows ballistically "
        "instead of diffusively."
    )
    TAGS = ("explore", "persistence", "commitment", "hysteresis", "coverage",
            "no-map", "frontier")

    V_MAX = 0.8
    WZ_MAX = 1.0
    K_YAW = 1.8

    # Half-width of the "does the robot fit" window, in beams (~2 deg each).
    FIT_HALF_BEAMS = 4
    # Wider sector, on top of the fit window, used only for the speed brake.
    FRONT_HALF_BEAMS = 7

    MIN_COMMIT_DEPTH = 1.0
    KEEP_OUT = 0.5
    MAX_COMMIT = 4.0
    ABORT_DEPTH = 0.9

    HARD_STOP = 0.55
    SLOW_ZONE = 1.3
    TURN_ONLY = 0.6
    BACKUP_SPEED = 0.25

    # Keeps reversal reachable when nothing forward is committable.
    PERSISTENCE_FLOOR = 0.05
    # Breaks the exact left/right symmetry of mirror-image gaps, which would
    # otherwise let floating-point noise chatter the selection.
    TIE_BREAK = 0.02

    def reset(self, scenario, rng):
        self._bearing = None
        self._budget = 0.0
        self._last_t = None
        self._last_yaw = None
        self._last_vx = 0.0
        self._turn_bias = 1.0
        if rng is not None:
            self._turn_bias = 1.0 if float(rng.uniform(0.0, 1.0)) < 0.5 else -1.0

    def precondition(self, obs, ports):
        _, depth = self._profile(obs)
        return bool(np.any(depth >= self.MIN_COMMIT_DEPTH))

    def tick(self, obs, ports):
        if not hasattr(self, "_bearing"):
            self.reset(None, None)

        ang, depth = self._profile(obs)
        dyaw, moved = self._odometry(obs)

        if self._bearing is not None:
            # The commitment lives in the robot frame, so our own rotation moves
            # it. Only the yaw increment is used, never absolute belief.
            self._bearing = _wrap(self._bearing - dyaw)
            self._budget -= moved

        front = self._depth_at(ang, depth, 0.0, self.FRONT_HALF_BEAMS)

        if self._bearing is not None:
            along = self._depth_at(ang, depth, self._bearing, 0)
            spent = self._budget <= 0.0
            blocked = along < self.ABORT_DEPTH or front < self.HARD_STOP
            if spent or blocked or obs.contact:
                self._bearing = None

        if self._bearing is None:
            self._bearing, self._budget = self._select(ang, depth)

        if self._bearing is None:
            self._last_vx = 0.0
            return Status.FAILURE

        err = self._bearing
        wz = float(np.clip(self.K_YAW * err, -self.WZ_MAX, self.WZ_MAX))
        if obs.contact:
            # A commitment that ended in contact cannot be re-selected from
            # where we are standing: open up a robot radius of room first.
            vx = -self.BACKUP_SPEED
        elif abs(err) > self.TURN_ONLY:
            vx = 0.0
        else:
            gate = np.clip(
                (front - self.HARD_STOP) / (self.SLOW_ZONE - self.HARD_STOP),
                0.0, 1.0)
            vx = self.V_MAX * math.cos(err) * float(gate)

        ports["committed_exploration_bearing"] = float(err)
        ports.drive(Command(vx, 0.0, wz))
        self._last_vx = vx
        return Status.RUNNING

    def _profile(self, obs):
        """Bearings sorted ascending, and the depth the robot fits down each."""
        rmax = float(obs.range_max)
        r = np.asarray(obs.ranges, dtype=float)
        r = np.clip(np.where(np.isfinite(r), r, rmax), 0.0, rmax)
        ang = _wrap(np.asarray(obs.range_angles, dtype=float))

        order = np.argsort(ang)
        ang = ang[order]
        r = r[order]

        w = self.FIT_HALF_BEAMS
        # The scan closes the circle, so the wrap-around roll is exact rather
        # than an edge approximation, and a slit one beam wide cannot advertise
        # itself as a corridor.
        win = np.stack([np.roll(r, k) for k in range(-w, w + 1)])
        return ang, win.min(axis=0)

    def _odometry(self, obs):
        t = float(obs.t)
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t

        yaw = float(obs.pose_estimate[2])
        dyaw = 0.0 if self._last_yaw is None else _wrap(yaw - self._last_yaw)
        self._last_yaw = yaw

        # Budget is integrated from what we asked for, not from the pose belief:
        # a commitment only needs its own length, and the belief carries a
        # random walk that would corrupt it.
        return dyaw, abs(self._last_vx) * dt

    def _depth_at(self, ang, depth, target, half_beams):
        i = int(np.argmin(np.abs(_wrap(ang - target))))
        if half_beams <= 0:
            return float(depth[i])
        idx = np.arange(i - half_beams, i + half_beams + 1) % ang.size
        return float(np.min(depth[idx]))

    def _select(self, ang, depth):
        """Deepest committable bearing, biased toward the current heading."""
        ok = depth >= self.MIN_COMMIT_DEPTH
        if not np.any(ok):
            return None, 0.0

        persist = (0.5 * (1.0 + np.cos(ang))) ** 2 + self.PERSISTENCE_FLOOR
        tilt = 1.0 + self.TIE_BREAK * self._turn_bias * np.sign(ang)
        score = np.where(ok, depth * persist * tilt, -1.0)

        i = int(np.argmax(score))
        budget = min(float(depth[i]) - self.KEEP_OUT, self.MAX_COMMIT)
        return float(ang[i]), budget


def micro(seed=0):
    """Column joined to a corridor whose far end holds the undisclosed goal.

    goal_seeker walks straight at the goal and jams against the face of the
    block, which spans the whole line between start and goal. A persistent
    explorer commits to the deepest bearing (north, up the column), runs it out,
    re-commits east into the corridor and runs that to its end - where the goal
    is. Nothing here is visible to the node, which only ever sees ranges.
    """
    world = room(9.0, 6.0, boxes=[(2.0, 0.0, 9.0, 4.2),
                                  (2.0, 5.8, 9.0, 6.0)])
    return MicroScenario(
        id="persistent_bearing_corridor_run",
        world=world,
        start=(1.0, 1.0, math.pi / 2.0),
        goal=(8.0, 5.0),
        time_limit=90.0,
        seed=seed,
        goal_known=False,
    )
