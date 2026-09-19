"""Steer to a goal by an ant-style path-integrated goal vector (Eq. 3).

Implements the Cataglyphis fortis home-vector accumulator: a running mean
travel direction phi and a stride count l, updated per unit step by
phi <- phi + k(180+delta)(180-delta)delta/l, with the 1/l term making the
stored direction progressively stiffer the further the robot has walked.
Position is reconstructed as anchor + l*u*(cos phi, sin phi) and never read
back from pose_estimate, so the position random walk in the belief cannot
enter the loop. Heading is the trusted channel, translation the untrusted one.
"""

import math

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status

from harness.micro import MicroScenario, room

# Eq. 3's normaliser, quoted in the paper in deg^-1, so the whole update runs
# in degrees and converts to radians only at the point of use.
_K_DEG_INV = 4.009e-5
_MAX_DPHI_DEG = 90.0

# One "stride": a Go2 body is ~0.7 m, a comfortable trot stride about a fifth
# of that. At the 0.8 m/s cap this emits ~5 unit steps per second, dense
# enough for the 1/l filter to behave like the continuous limit of Eq. 2.
_UNIT_M = 0.15

# Commanded translation that must accumulate before the integrated vector is
# stiff enough to steer by (>= ~7 unit steps).
_ARM_DISTANCE_M = 1.0

_ARRIVE_M = 0.5
_TURN_GAIN = 1.5      # 40 deg of bearing error saturates wz
_MAX_WZ = 1.0
_MAX_VX = 0.8


def _wrap_180(deg):
    """Wrap degrees to (-180, 180]."""
    w = (deg + 180.0) % 360.0 - 180.0
    return 180.0 if w == -180.0 else w


def _wrap_pi(rad):
    """Wrap radians to (-pi, pi]."""
    w = (rad + math.pi) % (2.0 * math.pi) - math.pi
    return math.pi if w == -math.pi else w


class SteerByPathIntegratedGoalVector(Action):
    """Drive at a goal bearing taken from a stride-counted, 1/l-damped
    path integrator rather than from pose_estimate translation.

    The mechanism carries two scalars instead of a position: phi, the mean
    direction travelled so far, and l, distance travelled counted in unit
    steps. Each unit step the stored direction rotates by an amount that is a
    saturating odd function of the turn delta made relative to phi, divided by
    l. Because the increment shrinks as 1/l, a late perturbation -- a detour, a
    miscounted stride, a heading wobble -- moves the goal vector less and less
    as the walk gets longer: a one-pole low-pass on bearing whose time constant
    is distance travelled.

    Why it should work here: the stride count is fed by the Twist this node
    itself commands, never by the change in pose_estimate. The belief's
    position random walk therefore never enters the accumulator, while the
    anchor-to-goal offset is captured once, as early as possible, when that
    walk has barely started. The known price is Eq. 3's signed error, largest
    near reversals and identically zero at delta = 180 deg, where a perfect
    about-turn writes nothing at all into phi. That is the paper's own failure
    mode and it is left in.

    This leaf is a bearing source only. It reads no lidar and claims no
    obstacle competence, so it belongs below any avoidance leaf in the tree.
    """

    PORTS = (
        # False during warm-up, telling the tree to fall through to a bearing
        # computed directly from pose_estimate.
        Port("path_integrator_armed", PortType.FLAG, Direction.OUT),
        Port("goal_bearing", PortType.BEARING, Direction.OUT),
    )
    PRECONDITION = "The goal has been disclosed, i.e. obs.goal is not None."
    EFFECT = "Drives the robot toward the goal along a bearing reconstructed from a stride-counted, 1/l-damped path-integrated position estimate."
    TAGS = ("path-integration", "dead-reckoning", "goal-vector", "drift-tolerant", "bearing", "ant")

    def reset(self, scenario, rng):
        self._t_prev = None
        self._anchor = None          # (ax, ay), set once, never moved
        self._offset = None          # goal - anchor, carried as a constant
        self._phi_deg = 0.0
        self._l = 1.0                # divisor: never allowed below 1.0
        self._residue = 0.0          # sub-stride commanded translation
        self._travel = 0.0           # total commanded translation since anchor
        self._last_cmd = (0.0, 0.0)  # the Twist we asked for on the last tick

    def _ensure_state(self):
        if not hasattr(self, "_t_prev"):
            self.reset(None, None)

    def precondition(self, obs, ports):
        return obs.goal is not None

    def _set_anchor(self, x, y, gx, gy, yaw):
        """Anchor at the earliest instant the goal is known.

        Re-anchoring later would import all the belief drift accumulated up to
        that moment into the constant offset, which is the one quantity that
        has to stay clean.
        """
        self._anchor = (x, y)
        self._offset = (gx - x, gy - y)
        self._phi_deg = math.degrees(_wrap_pi(yaw))  # no command yet, so yaw
        self._l = 1.0
        self._residue = 0.0
        self._travel = 0.0

    def _integrate(self, yaw, dt):
        """Advance phi and l by the strides commanded since the last tick."""
        vx, vy = self._last_cmd
        speed = math.hypot(vx, vy)
        if speed <= 0.0 or dt <= 0.0:
            return

        moved = speed * dt
        self._travel += moved
        self._residue += moved

        # Direction of travel in the world, from the trusted heading channel
        # plus the commanded body-frame velocity direction.
        theta = math.degrees(_wrap_pi(yaw + math.atan2(vy, vx)))

        while self._residue >= _UNIT_M:
            self._residue -= _UNIT_M
            delta = _wrap_180(theta - self._phi_deg)
            dphi = _K_DEG_INV * (180.0 + delta) * (180.0 - delta) * delta / self._l
            cap = _MAX_DPHI_DEG / self._l
            dphi = max(-cap, min(cap, dphi))
            self._phi_deg = _wrap_180(self._phi_deg + dphi)
            self._l += 1.0

    def tick(self, obs, ports):
        self._ensure_state()

        if obs.goal is None:
            ports["path_integrator_armed"] = False
            return Status.FAILURE

        x, y, yaw = obs.pose_estimate
        gx, gy = float(obs.goal[0]), float(obs.goal[1])

        dt = 0.0 if self._t_prev is None else max(0.0, obs.t - self._t_prev)
        self._t_prev = obs.t

        if self._anchor is None:
            self._set_anchor(x, y, gx, gy, yaw)
        else:
            self._integrate(yaw, dt)

        # Position from the accumulator alone; pose_estimate translation is
        # not consulted once the anchor is down.
        phi = math.radians(self._phi_deg)
        px = self._anchor[0] + self._l * _UNIT_M * math.cos(phi)
        py = self._anchor[1] + self._l * _UNIT_M * math.sin(phi)
        tx = self._anchor[0] + self._offset[0]
        ty = self._anchor[1] + self._offset[1]
        dist = math.hypot(tx - px, ty - py)

        armed = self._travel >= _ARM_DISTANCE_M
        if armed:
            bearing = _wrap_pi(math.atan2(ty - py, tx - px) - yaw)
        else:
            # Warm-up. The card has the tree fall through to a pose_estimate
            # goal bearing here, but a leaf that refuses to drive can never
            # accumulate the strides that arm it, so the fall-through source is
            # reproduced inline for these first ~7 unit steps. The integrator
            # itself is already running.
            bearing = _wrap_pi(math.atan2(gy - y, gx - x) - yaw)

        ports["path_integrator_armed"] = bool(armed)
        ports["goal_bearing"] = float(bearing)

        wz = max(-_MAX_WZ, min(_MAX_WZ, _TURN_GAIN * bearing))
        # No forward drive while the goal is behind us: turn on the spot first,
        # which also keeps the stride direction theta meaningful.
        vx_cmd = _MAX_VX * max(0.0, math.cos(bearing))
        ports.drive(Command(vx_cmd, 0.0, wz))
        self._last_cmd = (vx_cmd, 0.0)

        return Status.SUCCESS if dist < _ARRIVE_M else Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}


def micro(seed=0):
    """A straight route through two narrow apertures.

    Straight-line reachable, because this leaf has no obstacle competence and
    must not be asked to detour. What it does demand is cross-track accuracy
    over a long walk: each 1.4 m aperture leaves the 0.3 m robot disc +/- 0.4 m
    of slack, and the two are in series so belief error compounds. A controller
    steering at the goal from a drifting pose_estimate acquires cross-track
    error as it walks, and having no sensing it jams against a partition face
    and stays there. An accumulator fed by commanded strides and anchored at
    t=0 does not.
    """
    world = room(
        14.0,
        6.0,
        boxes=[
            (5.0, 0.0, 5.4, 2.3),
            (5.0, 3.7, 5.4, 6.0),
            (9.0, 0.0, 9.4, 2.3),
            (9.0, 3.7, 9.4, 6.0),
        ],
    )
    return MicroScenario(
        id="path_integrated_goal_vector_two_apertures",
        world=world,
        start=(1.0, 3.0, 0.0),
        goal=(12.5, 3.0),
        time_limit=45.0,
        seed=seed,
        goal_known=True,
    )
