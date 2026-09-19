"""ICS (Inevitable Collision State) braking-feasibility filter for a Go2.

Implements `inevitable_collision_state_filter`: a state is unsafe if NO
admissible future control trajectory from it avoids collision, so testing the
*instantaneous* occupancy of the commanded direction is not a safety test. The
node evaluates a small fixed repertoire of braking manoeuvres from the state the
proposed Twist would carry the robot into, and refuses any command from which
none of them terminates stopped and clear. Because the repertoire is a strict
subset of the real control set, the test can only ever be pessimistic - it may
call a state unsafe when an escape existed, never the reverse - which is what
makes a three-manoeuvre check a sound filter rather than a heuristic one.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room

PREDICTION = {
    "statement": (
        "Wrapping the motion command in the ICS braking-feasibility filter "
        "reduces the number of collisions per episode, averaged over seeds, by "
        "at least 50% relative to the reactive_avoid baseline, which avoids "
        "currently-occupied space but does not check that a stop remains "
        "feasible. The effect should be concentrated in exactly the cases the "
        "mechanism predicts: obstacles first sensed at less than d_stop(v) "
        "ahead while the robot is near saturation speed."
    ),
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "spl",
}


class FilterInevitableCollisionStates(Action):
    """Veto a proposed Twist unless some braking manoeuvre still stops clear.

    Mechanism. The frozen locomotion policy is the only thing that touches
    joints, so the sole evasive action available above it is to command the
    Twist toward zero and let the policy decelerate. Model that as a lag of
    TAU_LAG followed by constant deceleration A_BRAKE, giving

        d_stop(v) = v*TAU_LAG + v**2/(2*A_BRAKE) + R_BODY

    which is 0.99 m at the saturated 0.8 m/s - far inside range_max, so the
    test is never limited by the sensor horizon. For a proposed command we
    integrate three braking manoeuvres as unicycle sweeps in the ROBOT frame
    (hold the command one control step, then brake) and require that at least
    one of them keeps every swept body centre at least R_BODY from every finite
    lidar return. If all three fail the state is an ICS under our subset, and we
    substitute the manoeuvre with the largest clearance, at the largest speed
    scale that is itself ICS-free.

    Why it should work here. In an unmapped indoor space the obstruction is
    revealed late; a purely instantaneous avoider accepts commands that carry
    the robot past the last point at which stopping was still possible. This
    filter refuses exactly those commands, and refuses them one step early.

    The whole test is built from raw beams in the robot frame, so it never reads
    pose_estimate and is exactly as valid under drifting odometry as under clean
    odometry. No latch: the ICS property is a property of the current state and
    the current scan, so it is re-derived every step.
    """

    PORTS = (
        # The direction/speed some upstream motion node has already chosen.
        # This node never writes them - it only refuses them.
        Port("proposed_heading", PortType.BEARING, Direction.IN),
        Port("proposed_speed", PortType.SPEED, Direction.IN),
        # True on any step where the proposal was found to be an ICS.
        Port("ics_veto", PortType.FLAG, Direction.OUT),
    )

    PRECONDITION = (
        "A motion command has been proposed for this control step whose "
        "commanded planar speed exceeds 0.05 m/s."
    )
    EFFECT = (
        "The Twist emitted this step is one from which at least one manoeuvre "
        "in the fixed braking repertoire comes to rest clear of every obstacle "
        "in the current scan."
    )
    TAGS = ("safety", "ics", "braking", "veto", "collision-avoidance", "reactive")

    # --- braking model (derivation step 1) -------------------------------
    TAU_LAG = 0.4          # s, ~two gait cycles before a commanded change lands
    A_BRAKE = 1.0          # m/s^2
    R_BODY = 0.35          # m, circumscribed radius of the Go2 body: orientation-free

    # --- test discretisation (derivation step 2) -------------------------
    SWEEP_DT = 0.05        # 20 Hz unicycle integration of each braking sweep
    CTRL_DT_DEFAULT = 0.05  # used until two ticks have been seen
    WZ_MAX = 1.0
    V_MAX = 0.8
    SPEED_SCALES = (1.0, 0.75, 0.5, 0.25, 0.0)

    # --- trigger (card: trigger) -----------------------------------------
    MOVE_EPS = 0.05        # m/s, below this the command is not "moving"
    CLUTTER_RADIUS = 2.0   # m, nothing this close means nothing to check

    # Only used by the stand-in proposal, never by the safety test itself.
    K_YAW = 2.0
    GOAL_TOL = 0.25

    # Class-level so an instance is usable even if reset() was never called.
    _last_t = None
    _vetoes = 0

    def reset(self, scenario, rng):
        self._last_t = None
        self._vetoes = 0

    def precondition(self, obs, ports):
        vx, vy, _ = self._proposal(obs, ports)
        return math.hypot(vx, vy) > self.MOVE_EPS

    def tick(self, obs, ports):
        dt_ctrl = self._control_dt(obs)
        vx, vy, wz = self._proposal(obs, ports)
        speed = math.hypot(vx, vy)
        obst = self._obstacles(obs)

        # Pass-through: outside the trigger the proposal is not ours to touch.
        if speed <= self.MOVE_EPS or not self._in_clutter(obst):
            ports["ics_veto"] = False
            ports.drive(Command(vx, vy, wz))
            return Status.RUNNING

        # B0 brakes on the proposed yaw rate, B+/B- turn away while braking.
        brake_rates = (wz, self.WZ_MAX, -self.WZ_MAX)
        clearances = [
            self._min_clearance(self._sweep(vx, vy, wz, wzb, dt_ctrl), obst)
            for wzb in brake_rates
        ]

        if max(clearances) >= self.R_BODY:
            ports["ics_veto"] = False
            ports.drive(Command(vx, vy, wz))
            return Status.RUNNING

        # Every manoeuvre in the subset fails: the state the proposal leads to
        # is an ICS. Substitute rather than blindly zeroing, so the robot sits
        # on the boundary of the safe set instead of deep inside it.
        self._vetoes += 1
        ports["ics_veto"] = True
        best = int(np.argmax(clearances))
        wz_brake = brake_rates[best]
        for scale in self.SPEED_SCALES:
            sweep = self._sweep(scale * vx, scale * vy, wz_brake, wz_brake, dt_ctrl)
            if self._min_clearance(sweep, obst) >= self.R_BODY:
                # scale 0.0 leaves a pure rotation, which is a legitimate escape:
                # it changes no body-centre position but re-aims the proposal.
                ports.drive(Command(scale * vx, scale * vy, wz_brake))
                return Status.RUNNING

        ports.drive(Command(0.0, 0.0, 0.0))
        return Status.FAILURE

    # ------------------------------------------------------------------
    # the test
    # ------------------------------------------------------------------

    def _sweep(self, vx, vy, wz_cmd, wz_brake, dt_ctrl):
        """Swept polyline of body centres, robot frame, for one manoeuvre.

        Three phases: hold the proposed command for one control step, hold
        through the policy lag, then decelerate at A_BRAKE. Integrating the lag
        separately (rather than ramping linearly across lag+decel) is what makes
        the swept length agree with d_stop; the linear ramp would understate it.
        """
        v = math.hypot(vx, vy)
        phases = (
            (dt_ctrl, wz_cmd, False),
            (self.TAU_LAG, wz_brake, False),
            (v / self.A_BRAKE, wz_brake, True),
        )

        xs = [0.0]
        ys = [0.0]
        x = y = th = 0.0
        for duration, wz_seg, decelerating in phases:
            if duration <= 0.0:
                continue
            n = max(1, int(math.ceil(duration / self.SWEEP_DT)))
            h = duration / n
            for k in range(n):
                frac = 1.0 - (k + 0.5) * h / duration if decelerating else 1.0
                sx, sy = vx * frac, vy * frac
                c, s = math.cos(th), math.sin(th)
                x += (sx * c - sy * s) * h
                y += (sx * s + sy * c) * h
                th += wz_seg * h
                xs.append(x)
                ys.append(y)

        return np.stack((np.asarray(xs), np.asarray(ys)), axis=1)

    def _obstacles(self, obs, cutoff=3.0):
        """Finite beam returns as points in the robot frame.

        Beams at or beyond range_max carry no information about occupancy and
        are treated as free: sound here because the longest braking sweep is
        ~1.0 m, so every sweep terminates deep inside sensed space. Points
        further than `cutoff` cannot be reached by any sweep, so they are
        dropped to keep the per-step cost flat.
        """
        r = np.asarray(obs.ranges, dtype=float)
        a = np.asarray(obs.range_angles, dtype=float)
        keep = np.isfinite(r) & (r > 0.0) & (r < obs.range_max) & (r <= cutoff)
        r = r[keep]
        a = a[keep]
        return np.stack((r * np.cos(a), r * np.sin(a)), axis=1)

    def _in_clutter(self, obst):
        if obst.shape[0] == 0:
            return False
        return float(np.hypot(obst[:, 0], obst[:, 1]).min()) <= self.CLUTTER_RADIUS

    @staticmethod
    def _min_clearance(sweep, obst):
        if obst.shape[0] == 0:
            return float("inf")
        delta = sweep[:, None, :] - obst[None, :, :]
        return float(np.hypot(delta[:, :, 0], delta[:, :, 1]).min())

    # ------------------------------------------------------------------
    # the proposal we are filtering
    # ------------------------------------------------------------------

    def _proposal(self, obs, ports):
        """(vx, vy, wz) that some other node wants executed this step."""
        try:
            heading = self._wrap(float(ports["proposed_heading"]))
            speed = float(ports["proposed_speed"])
        except Exception:
            # Nothing upstream has written a proposal. Stand in for the motion
            # node so the filter is testable in isolation; this branch is not
            # part of the mechanism and does no planning.
            return self._pursuit_proposal(obs)

        speed = min(max(speed, 0.0), self.V_MAX)
        wz = self._clip(self.K_YAW * heading, self.WZ_MAX)
        return speed * math.cos(heading), speed * math.sin(heading), wz

    def _pursuit_proposal(self, obs):
        goal = obs.goal
        if goal is None:
            # Goal not disclosed: hold the current heading and let the filter
            # decide whether that heading is survivable.
            return self.V_MAX, 0.0, 0.0

        px, py, yaw = obs.pose_estimate
        dx, dy = goal[0] - px, goal[1] - py
        if math.hypot(dx, dy) <= self.GOAL_TOL:
            return 0.0, 0.0, 0.0

        bearing = self._wrap(math.atan2(dy, dx) - yaw)
        wz = self._clip(self.K_YAW * bearing, self.WZ_MAX)
        return self.V_MAX * max(0.0, math.cos(bearing)), 0.0, wz

    def _control_dt(self, obs):
        """Length of one control step, measured rather than assumed."""
        dt = self.CTRL_DT_DEFAULT
        if self._last_t is not None:
            observed = obs.t - self._last_t
            if 0.005 <= observed <= 0.5:
                dt = observed
        self._last_t = obs.t
        return dt

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _clip(value, limit):
        return max(-limit, min(limit, value))


def micro(seed=0):
    """A dogleg whose turns do not fit the turning circle of a full-speed robot.

    The route is: 1.2 m corridor east, hard right through a 1.1 m slot whose far
    side is a wall face 1.1 m beyond the corner, then hard left through a 1.3 m
    gap that the lower block hides until the robot has descended. At 0.8 m/s and
    |wz| <= 1.0 the turning radius is 0.8 m, so a controller that steers at the
    widest progressing gap without ever checking that a stop is still feasible
    can see every one of these gaps and still not fit the turn into it - the
    collision is committed before the gap it is steering at becomes unreachable.
    The ICS filter caps speed to keep d_stop inside the sensed free space, which
    shrinks the turning radius enough to thread the same geometry, at a cost in
    path efficiency.
    """
    world = room(
        10.0,
        7.0,
        boxes=[
            (0.0, 0.0, 5.0, 2.9),    # south block: its corner ends the corridor
            (0.0, 4.1, 6.1, 7.0),    # north block: forms the 1.2 m corridor
            (6.1, 2.9, 10.0, 7.0),   # wall face closing the corridor east
            (6.5, 0.0, 7.5, 1.6),    # second pinch, occluded from the corridor
        ],
    )
    return MicroScenario(
        id="ics_dogleg_late_reveal",
        world=world,
        start=(0.8, 3.5, 0.0),
        goal=(8.5, 1.4),
        time_limit=70.0,
        seed=seed,
        goal_known=True,
    )
