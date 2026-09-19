"""Braking-admissible velocity filter for a Unitree Go2 (Eq. (14) of the
dynamic-window formulation), implemented as the last leaf between any policy
and the frozen locomotion policy.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PREDICTION = {
    "statement": (
        "With the braking-admissibility filter armed on top of an unchanged "
        "exploration driver, mean collisions per episode over the seed set "
        "drops by at least 50% relative to the reactive_avoid baseline on "
        "undisclosed-goal exploration scenarios, and this holds on the "
        "drifting-pose scenarios too, because the filter reads only "
        "robot-frame lidar. Falsified if the relative reduction is under 50%, "
        "or if collisions do not fall at all."
    ),
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "time_to_goal",
}

# Candidate velocities. vx is non-negative: the arc model of Section 3.1 assumes
# the translational velocity leads in the steering direction, so it says nothing
# about reversing.
_VX_GRID = np.round(np.arange(0.0, 0.8 + 1e-9, 0.1), 3)
_WZ_GRID = np.round(np.arange(-1.0, 1.0 + 1e-9, 0.1), 3)


def _read_or(ports, name, default):
    """Read an optional IN port, falling back if nothing has written it yet.

    Reading an unwritten port raises, and the exception type is the harness's to
    choose, hence the broad catch. The fallback is what makes the filter safe to
    leave permanently armed with no upstream driver attached: it still issues a
    command every step rather than silently starving the locomotion policy.
    """
    try:
        value = ports[name]
    except Exception:
        return default
    return default if value is None else float(value)


class ClampTwistToBrakingAdmissible(Action):
    """Permit a commanded velocity only if the robot could still stop, under its
    own braking accelerations, before the nearest obstacle on the arc that
    velocity traces.

    Mechanism (Eq. (14)): a pair (vx, wz) is admissible iff

        vx <= sqrt(2 * dist(vx, wz) * vb_dot)   and
        |wz| <= sqrt(2 * dist(vx, wz) * wb_dot)

    where dist is measured ALONG THE ARC (gamma * |r|, Section 5.2) to the
    closest obstacle lying on it. The content is that clearance is arc-specific
    and coupled to speed rather than a fixed forward stopping box: two commands
    with the same forward speed curve into different parts of the world and so
    have wildly different dist, and one command can be safe slow and
    inadmissible fast. The robot is therefore free to run flat out down an open
    arc while the same single inequality forces it to slow exactly as much as
    the geometry of its own chosen curve demands.

    The set is never empty -- (0, 0) satisfies both inequalities for any
    dist >= 0 -- so the filter degrades to a stop, never to an undefined
    command.

    Drift immunity: every quantity below is computed in the ROBOT frame from
    `ranges` / `range_angles` for the current step only. `pose_estimate` is
    never read and no obstacle memory is kept, so nothing here can inherit pose
    drift.

    Deliberately NOT included (each is a separate mechanism, and mixing them in
    would make the measured effect unattributable to Eq. (14)): the dynamic
    window of Eq. (15), the speed-dependent side clearance of Section 5.3, and
    the paper's FIFO obstacle-line memory.
    """

    PORTS = (
        # The desired twist from whatever is upstream -- a frontier/coverage
        # driver, a recovery, anything. The port vocabulary has no angular-rate
        # type, so the turn request arrives as the robot-frame bearing the
        # driver wants to head along and is converted below.
        Port("desired_speed", PortType.SPEED, Direction.IN),
        Port("desired_heading", PortType.BEARING, Direction.IN),
    )
    PRECONDITION = (
        "A scan is available for this control step, so arc clearance can be "
        "measured before any Twist is issued."
    )
    EFFECT = (
        "The Twist issued is the nearest velocity to the desired one from "
        "which the robot could still brake to a stop before the closest "
        "obstacle on the arc it traces."
    )
    TAGS = (
        "safety",
        "velocity-filter",
        "braking",
        "admissible-velocity",
        "arc-clearance",
        "lidar",
        "drift-immune",
    )

    # Body inflated to a disc: the Go2 is about 0.70 m by 0.31 m, so the
    # circumscribed radius is sqrt(0.35^2 + 0.155^2) ~= 0.38 m.
    BODY_RADIUS = 0.38

    # Braking accelerations. RHINO's 50 cm/s^2 and 60 deg/s^2 are not a Go2's,
    # and the true deceleration of the frozen locomotion policy is not
    # observable through this interface. These null the full command range in
    # about a second of tracking: 0.8 m/s -> 0 in 1.0 s, 1.0 rad/s -> 0 in
    # 0.67 s. VB_DOT is the single safety knob -- halving it multiplies every
    # permitted speed at a given distance by 1/sqrt(2).
    VB_DOT = 0.8
    WB_DOT = 1.5

    VX_LIMIT = 0.8
    WZ_LIMIT = 1.0

    # Below this the arc radius blows up and the straight-line case is used.
    STRAIGHT_WZ = 1.0e-3

    # A rad/s of turn error costs half what a m/s of speed error costs, so the
    # projection onto the admissible set prefers to give up speed over heading.
    TURN_COST_WEIGHT = 0.5

    # Bearing -> yaw rate: close the heading error in this many seconds.
    TURN_IN_TIME = 0.5

    _TOL = 1.0e-9

    def precondition(self, obs, ports) -> bool:
        # Only scan freshness is gated. The other half of the trigger -- that a
        # desired twist exists -- is always satisfiable via the port fallback,
        # and an all-inf scan makes the filter inert rather than inapplicable,
        # so neither may switch the node off: doing so would drop the twist.
        ranges = getattr(obs, "ranges", None)
        angles = getattr(obs, "range_angles", None)
        if ranges is None or angles is None:
            return False
        return len(ranges) == len(angles) and len(ranges) > 0

    def tick(self, obs, ports) -> Status:
        range_max = float(getattr(obs, "range_max", 6.0) or 6.0)
        px, py = self._obstacle_points(obs, range_max)
        vx_d, wz_d = self._desired(ports)

        # Include the exact desired yaw rate so that (0, wz_d) is literally a
        # candidate, as well as the exact 0.0 that guarantees non-emptiness.
        wz_cands = np.unique(np.round(np.append(_WZ_GRID, wz_d), 6))

        # Ordering vx by closeness to the request lets the loop stop early: cost
        # is at least the vx term, so once that term alone exceeds the best cost
        # found, no later vx can win.
        vx_order = _VX_GRID[np.argsort(np.abs(_VX_GRID - vx_d), kind="stable")]

        best_cost = math.inf
        best = (0.0, 0.0)
        for vx in vx_order:
            vx_cost = (vx - vx_d) ** 2
            if vx_cost >= best_cost:
                break
            for wz in wz_cands:
                clearance = self._arc_clearance(px, py, vx, wz, range_max)
                brake = 2.0 * clearance
                if vx > math.sqrt(brake * self.VB_DOT) + self._TOL:
                    continue
                if abs(wz) > math.sqrt(brake * self.WB_DOT) + self._TOL:
                    continue
                cost = vx_cost + (self.TURN_COST_WEIGHT * (wz - wz_d)) ** 2
                if cost < best_cost:
                    best_cost = cost
                    best = (float(vx), float(wz))

        # vy is zeroed rather than passed through: with no lateral term in the
        # arc model, a dist() computed for (vx, wz) would be a prediction about
        # a path the robot is not taking.
        ports.drive(Command(best[0], 0.0, best[1]))

        # A permanently armed filter is never "done"; it re-projects every step.
        return Status.RUNNING

    def _desired(self, ports):
        """Desired (vx, wz), saturated to the interface limits."""
        speed = _read_or(ports, "desired_speed", self.VX_LIMIT)
        heading = _read_or(ports, "desired_heading", 0.0)

        vx_d = min(max(speed, 0.0), self.VX_LIMIT)
        heading = math.atan2(math.sin(heading), math.cos(heading))
        wz_d = heading / self.TURN_IN_TIME
        wz_d = min(max(wz_d, -self.WZ_LIMIT), self.WZ_LIMIT)
        return vx_d, wz_d

    @staticmethod
    def _obstacle_points(obs, range_max):
        """Current-sweep obstacle points in the robot frame."""
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        hit = np.isfinite(ranges) & (ranges > 0.0) & (ranges <= range_max)
        r = ranges[hit]
        a = angles[hit]
        return r * np.cos(a), r * np.sin(a)

    def _arc_clearance(self, px, py, vx, wz, range_max):
        """Distance along the arc traced by (vx, wz) to the closest obstacle.

        Section 5.2 / Figure 19, with the body inflated to BODY_RADIUS and the
        result clamped to [0, range_max]; range_max when no obstacle lies on the
        arc, which is what makes an all-inf scan inert.
        """
        rho = self.BODY_RADIUS
        spinning = abs(wz) >= self.STRAIGHT_WZ

        if vx <= self._TOL and spinning:
            # A disc turning in place sweeps no new ground, so only the wz
            # inequality can bind -- and at this clearance it cannot.
            return range_max

        if not spinning:
            band = (np.abs(py) <= rho) & (px > 0.0)
            if not np.any(band):
                return range_max
            return float(np.clip(px[band].min() - rho, 0.0, range_max))

        # Arc case: centre C = (0, r) in the robot frame, r = vx / wz.
        r = vx / wz
        radius = abs(r)
        dx = px
        dy = py - r
        to_centre = np.hypot(dx, dy)
        on_arc = np.abs(to_centre - radius) <= rho
        if not np.any(on_arc):
            return range_max

        phi = np.arctan2(dy[on_arc], dx[on_arc])
        # Angle of the robot's own position about C, and the sense of travel:
        # the sweep is counter-clockwise about C exactly when wz > 0.
        phi_0 = -0.5 * math.pi if r > 0.0 else 0.5 * math.pi
        sense = 1.0 if wz > 0.0 else -1.0
        # Sign-corrected into [0, 2*pi) so an obstacle behind the sweep gives
        # gamma near 2*pi, not near 0.
        gamma = np.mod((phi - phi_0) * sense, 2.0 * math.pi)
        return float(np.clip((gamma * radius).min() - rho, 0.0, range_max))


def micro(seed=0):
    """Serpentine room, goal undisclosed and hidden behind the first barrier.

    Three barriers flush with alternating walls force the robot through gaps it
    can only find by covering ground, and present six convex corners. A gap
    follower takes them at speed on the inside and clips them; the arc-specific
    braking test slows exactly into each turn, because the arcs that cut the
    corner are the ones whose dist() collapses.
    """
    world = room(
        10.0,
        6.0,
        boxes=[
            (2.6, 0.0, 3.0, 3.6),
            (5.4, 2.4, 5.8, 6.0),
            (7.2, 0.0, 7.6, 3.0),
        ],
    )
    return MicroScenario(
        id="braking_admissible_serpentine",
        world=world,
        start=(1.0, 1.0, 0.0),
        goal=(9.0, 4.5),
        time_limit=120.0,
        seed=seed,
        goal_known=False,
    )
