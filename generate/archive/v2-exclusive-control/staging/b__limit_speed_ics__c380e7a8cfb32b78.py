"""Braking-horizon speed cap for an unmapped indoor explorer.

Implements `brake_within_visible_free_space`: a state is safe only while some
manoeuvre still halts the robot strictly inside space perception has already
shown to be free, and the sensing horizon itself must be treated as a candidate
obstacle, because unexpected obstacles always appear first at the edge of the
field of view.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PREDICTION = {
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}

# All MINE: the paper fixes no stopping distance, and the frozen locomotion
# policy publishes no deceleration.
A_BRAKE = 0.8    # m/s^2, effective deceleration of a Go2 tracking a zeroed velocity
TAU = 0.15       # s, dead time (one control step + policy/gait lag) at current speed
MARGIN = 0.35    # m, half the Go2 body length, so the whole footprint clears
V_LIMIT = 0.8    # m/s, runner's translational saturation
WZ_LIMIT = 1.0   # rad/s, runner's angular saturation
K_YAW = 1.5      # 1/s, proportional turn toward the requested travel direction


def _wrap_to_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def _read(ports, name, default):
    """Port read that tolerates an upstream that has not written yet.

    An unwritten IN port raises by contract; this node is a filter and must stay
    inert rather than fatal when it is ticked before any producer has run.
    """
    try:
        value = ports[name]
    except Exception:
        return default
    return default if value is None else value


class CapSpeedToBrakingHorizon(Action):
    """Rescale the commanded translation to the braking horizon of observed free space.

    The witness manoeuvre of Property 1 is fixed to "hold heading, command zero
    translation until stopped": if the robot can complete that manoeuvre without
    leaving the space the scan has actually shown to be free, the current state
    is provably not an inevitable collision state. Inverting the stopping
    distance d_stop(v) = m + v*tau + v^2/(2a) against the free distance along the
    direction of travel gives a speed cap, and the outgoing (vx, vy) is scaled to
    it. Every beam reading `inf` is treated as an obstacle at range_max rather
    than as free space -- the field-of-view boundary is a candidate obstacle,
    which is the whole point of the mechanism.

    Free distance is computed entirely in the robot frame from `ranges` and
    `range_angles`, so the cap is immune to pose drift and needs no map. The
    angular command is never scaled, so the robot can always rotate in place and
    re-perceive, including from a full stop.
    """

    PORTS = (
        Port("travel_bearing", PortType.BEARING, Direction.IN),
        Port("travel_speed", PortType.SPEED, Direction.IN),
        Port("braking_speed_cap", PortType.SPEED, Direction.OUT),
    )

    PRECONDITION = (
        "The active leaf is asking for a non-zero translational speed this control step."
    )
    EFFECT = (
        "The emitted translational speed is reduced to the largest speed from which the "
        "robot could still brake to rest inside the free space currently observed along "
        "its direction of travel."
    )
    TAGS = ("safety", "speed-cap", "inevitable-collision-state", "occlusion", "reactive")

    def reset(self, scenario, rng):
        self._last_cap = V_LIMIT

    # -- command source -------------------------------------------------

    def _requested(self, obs, ports):
        """(bearing, speed) of the translation the tree is asking for, robot frame."""
        speed = float(_read(ports, "travel_speed", V_LIMIT))
        bearing = _read(ports, "travel_bearing", None)
        if bearing is None:
            bearing = self._default_bearing(obs)
        return _wrap_to_pi(float(bearing)), max(0.0, speed)

    @staticmethod
    def _default_bearing(obs):
        """Travel direction when nothing upstream has supplied one.

        Straight ahead while the goal is undisclosed; at the goal once it has
        latched. This only chooses *where* the translation points -- the cap
        itself never consults pose_estimate or the goal.
        """
        if obs.goal is None:
            return 0.0
        gx, gy = obs.goal
        x, y, yaw = obs.pose_estimate
        return _wrap_to_pi(math.atan2(gy - y, gx - x) - yaw)

    # -- the braking horizon --------------------------------------------

    @staticmethod
    def _free_distance(obs, theta):
        """Distance to the nearest return inside the corridor swept along `theta`."""
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        if ranges.size == 0:
            return 0.0

        # inf means "nothing seen out to the horizon", and the horizon is where
        # an unexpected obstacle appears first: treat it as an obstacle at range_max.
        r = np.where(np.isfinite(ranges), np.clip(ranges, 0.0, obs.range_max), obs.range_max)

        delta = np.arctan2(np.sin(angles - theta), np.cos(angles - theta))
        # Inflate the corridor by half a beam spacing at range, so obstacles that
        # fall between adjacent beams are still swept.
        half_beam = math.pi / max(1, ranges.size)
        half_width = MARGIN + r * half_beam

        in_corridor = (np.abs(delta) <= math.pi / 2) & (r * np.abs(np.sin(delta)) <= half_width)
        if not np.any(in_corridor):
            return float(obs.range_max)
        return float(np.clip(np.min(r[in_corridor] * np.cos(delta[in_corridor])),
                             0.0, obs.range_max))

    @staticmethod
    def _speed_cap(d_free):
        """Invert d_stop(v) = m + v*tau + v^2/(2a) <= d_free for v."""
        usable = max(0.0, d_free - MARGIN)
        at = A_BRAKE * TAU
        v_max = -at + math.sqrt(at * at + 2.0 * A_BRAKE * usable)
        return float(min(max(v_max, 0.0), V_LIMIT))

    # -- contract --------------------------------------------------------

    def precondition(self, obs, ports):
        _, v_cmd = self._requested(obs, ports)
        return v_cmd > 0.0

    def tick(self, obs, ports):
        theta_cmd, v_cmd = self._requested(obs, ports)

        # Angular command is derived from the requested bearing only. The cap
        # below never touches it, so rotating to re-perceive survives v_out = 0.
        wz = float(np.clip(K_YAW * theta_cmd, -WZ_LIMIT, WZ_LIMIT))

        d_free = self._free_distance(obs, theta_cmd)
        if abs(wz) > 0.0:
            # Heading rotates while the witness manoeuvre runs; check the heading
            # we would actually hold at the end of it and take the worse of the two.
            theta_end = _wrap_to_pi(theta_cmd + wz * (TAU + v_cmd / A_BRAKE))
            d_free = min(d_free, self._free_distance(obs, theta_end))

        v_out = self._speed_cap(d_free)
        self._last_cap = v_out
        ports["braking_speed_cap"] = v_out

        scale = min(1.0, v_out / v_cmd) if v_cmd > 0.0 else 0.0
        speed = v_cmd * scale
        ports.drive(Command(speed * math.cos(theta_cmd),
                            speed * math.sin(theta_cmd),
                            wz))
        return Status.RUNNING


def micro(seed=0):
    """Occluded-doorway search: the goal is behind a wall, the trap is behind the doorway.

    Nothing beyond the divider is visible from the start, so the far block is
    revealed only once the robot is in the gap -- late, at the edge of the field
    of view, which is exactly the case the cap is claimed to cover. A full-speed
    gap follower carries speed it cannot shed into geometry it has just
    uncovered; the capped robot never holds more speed than its observed free
    space can absorb.
    """
    world = room(
        9.0, 6.0,
        boxes=[
            (4.0, 0.0, 4.3, 2.2),   # divider, below the doorway
            (4.0, 3.4, 4.3, 6.0),   # divider, above the doorway
            (6.0, 2.0, 8.0, 3.2),   # block hidden behind the divider, on the exit line
        ],
    )
    return MicroScenario(
        id="braking_horizon_occluded_doorway",
        world=world,
        start=(1.0, 2.8, 0.0),
        goal=(5.0, 1.2),
        time_limit=60.0,
        seed=seed,
        goal_known=False,
    )
