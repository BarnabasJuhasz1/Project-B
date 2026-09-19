import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status

# Braking model (derivation step 1). tau_lag is roughly two gait cycles of delay
# before a commanded change shows up in body velocity; a_brake is what the frozen
# policy delivers once it does. r_body is the circumscribed radius of the Go2 body,
# used so the check does not depend on heading.
_DT = 0.05
_TAU_LAG = 0.4
_A_BRAKE = 1.0
_R_BODY = 0.35

# Evasive-manoeuvre subset B (derivation step 2) and the substitution ladder (step 4).
_WZ_EVADE = 1.0
_SCALES = (1.0, 0.75, 0.5, 0.25, 0.0)

# Trigger (card: trigger).
_V_MOVING = 0.05
_TRIGGER_RADIUS = 2.0


class FilterInevitableCollisionStates(Action):
    """Inevitable-Collision-State veto on the command the navigator already chose.

    A state is an ICS if no admissible control from it avoids collision: the robot
    can be collision-free at this instant and still be committed, because it has lag
    and finite deceleration. So the safety test is not "is the robot clear now" but
    "does at least one braking manoeuvre from here terminate stopped and clear".
    Evaluating only a fixed subset of manoeuvres over-approximates the ICS set, which
    makes the test sound rather than heuristic: a subset can wrongly call a state
    unsafe, never wrongly call it safe.

    Three manoeuvres are simulated as a unicycle in the robot frame against the raw
    scan: hold the proposed command one control step, then brake to zero, with yaw
    rate held at the proposed value, at +1.0, and at -1.0 rad/s. If any of them stops
    clear, the command is passed through untouched.

    If none does, the node revises the planar speed -- `vx` and `vy` via `cap`, since
    this is purely a "no faster than" bound -- down to the largest scale factor at
    which some manoeuvre in the repertoire does stop clear, and `adjust`s `wz` to the
    turn-away rate when the surviving manoeuvre is one of the turning brakes. It never
    picks a direction of travel; the base's heading atan2(vy, vx) is preserved and only
    its magnitude is cut.

    It declines -- writing nothing, so the base's command passes through byte-identical
    -- whenever the commanded planar speed is at or below 0.05 m/s, whenever no finite
    return lies within 2.0 m, and whenever the proposed command is already ICS-free.
    There is no latch and no per-episode state: ICS is a property of the current state
    and the current scan, so a one-step veto that clears on the next step is correct.

    The whole test runs on beams in the robot frame and never reads pose_estimate, so
    it is exactly as valid under drifting odometry as under clean odometry.
    """

    WRITES = ("vx", "vy", "wz")
    PORTS = (Port("ics_safe_speed", PortType.SPEED, Direction.OUT),)
    PRECONDITION = (
        "The pending command asks for a planar speed above 0.05 m/s and at least one "
        "finite lidar return lies within 2.0 m of the robot."
    )
    EFFECT = (
        "Caps the commanded planar speed to the fastest value from which some braking "
        "manoeuvre in the fixed repertoire still stops clear of the scan, substituting "
        "that manoeuvre's turn-away yaw rate when it is a turning brake."
    )
    TAGS = ("safety", "ics", "braking", "stopping-distance", "veto", "lidar")

    def precondition(self, obs, ports):
        # `command` is documented on the view handed to tick; tolerate its absence here
        # and let tick re-check the trigger rather than risk raising.
        pending = getattr(ports, "command", None)
        if pending is None:
            return True
        return self._triggered(obs, pending)

    def tick(self, obs, ports):
        pending = ports.command
        if pending is None or not self._triggered(obs, pending):
            return Status.FAILURE

        vx = float(pending.vx)
        vy = float(pending.vy)
        v = math.hypot(vx, vy)
        heading = math.atan2(vy, vx)

        obstacles = self._obstacle_points(obs, self._horizon(v))
        if obstacles is None:
            # Nothing within braking reach: every manoeuvre is trivially ICS-free.
            ports["ics_safe_speed"] = v
            return Status.FAILURE

        d_near = float(np.min(np.hypot(obstacles[:, 0], obstacles[:, 1])))
        # Normally r_body. If a return is already inside the body radius -- brushing a
        # wall, a self-return -- no manoeuvre whatsoever can satisfy r_body and the node
        # would veto forever, so instead require only that the sweep does not reduce the
        # clearance the robot already has. Still conservative against standing still.
        clearance_required = min(_R_BODY, d_near)

        # (yaw rate, is this a turn-away substitution). Straight-ish brake first so that
        # it wins ties, which matters at scale 0 where all three sweep the same point.
        manoeuvres = ((float(pending.wz), False), (_WZ_EVADE, True), (-_WZ_EVADE, True))

        chosen = None
        for scale in _SCALES:
            speed = v * scale
            survivors = []
            for wz_cmd, is_evade in manoeuvres:
                path = self._brake_path(speed, heading, wz_cmd)
                clearance = self._min_clearance(path, obstacles)
                if clearance >= clearance_required:
                    survivors.append((clearance, wz_cmd, is_evade))
            if survivors:
                # Among the manoeuvres that stop clear at this scale, take the one with
                # the largest minimum clearance over its swept path.
                chosen = (scale, max(survivors, key=lambda s: s[0]))
                break

        if chosen is None:
            # No member of the repertoire stops clear at any scale: full stop.
            ports["ics_safe_speed"] = 0.0
            ports.cap(vx=0.0, vy=0.0)
            ports.adjust(wz=0.0)
            return Status.SUCCESS

        scale, (_, wz_sel, is_evade) = chosen
        ports["ics_safe_speed"] = v * scale

        if scale >= 1.0:
            # The command as proposed is ICS-free. Decline, byte-identical pass-through.
            return Status.FAILURE

        ports.cap(vx=abs(vx) * scale, vy=abs(vy) * scale)
        if is_evade and scale > 0.0:
            ports.adjust(wz=wz_sel)
        # At scale 0 there is no translation to evade, so the turn-away rate carries no
        # safety content; leaving wz to the base lets it pivot out of the dead end.
        return Status.SUCCESS

    @staticmethod
    def _triggered(obs, pending):
        if math.hypot(float(pending.vx), float(pending.vy)) <= _V_MOVING:
            return False
        ranges = np.asarray(obs.ranges, dtype=float)
        return bool(np.any(np.isfinite(ranges) & (ranges < _TRIGGER_RADIUS)))

    @staticmethod
    def _horizon(v):
        """Range beyond which a return cannot be touched by any braking sweep."""
        arc = _DT * v + _TAU_LAG * v + (v * v) / (2.0 * _A_BRAKE)
        return arc + _R_BODY + 0.25

    @staticmethod
    def _obstacle_points(obs, horizon):
        """Finite beams within reach, as points in the robot frame.

        Infinite beams carry no information nearer than range_max, and max stopping
        distance is ~0.99 m against a 6.0 m horizon, so treating them as free is sound.
        """
        ranges = np.asarray(obs.ranges, dtype=float)
        angles = np.asarray(obs.range_angles, dtype=float)
        keep = np.isfinite(ranges) & (ranges <= horizon)
        if not np.any(keep):
            return None
        r = ranges[keep]
        a = angles[keep]
        return np.column_stack((r * np.cos(a), r * np.sin(a)))

    @staticmethod
    def _brake_path(speed, heading, wz):
        """Body-centre polyline for: hold one control step, then lag, then brake out.

        Integrated as a unicycle in the robot frame at 20 Hz. The travel direction is
        fixed in the body frame, so it rotates with the body as yaw accumulates.
        """
        if speed <= 0.0:
            return np.zeros((1, 2))

        t_full = _DT + _TAU_LAG
        t_total = t_full + speed / _A_BRAKE
        n = int(math.ceil(t_total / _DT))

        path = np.empty((n + 1, 2))
        path[0] = (0.0, 0.0)
        x = y = yaw = 0.0
        for k in range(n):
            t = k * _DT
            s = speed if t < t_full else max(0.0, speed - _A_BRAKE * (t - t_full))
            x += s * math.cos(yaw + heading) * _DT
            y += s * math.sin(yaw + heading) * _DT
            yaw += wz * _DT
            path[k + 1] = (x, y)
        return path

    @staticmethod
    def _min_clearance(path, obstacles):
        dx = path[:, 0][:, None] - obstacles[None, :, 0]
        dy = path[:, 1][:, None] - obstacles[None, :, 1]
        return float(np.min(np.hypot(dx, dy)))


PREDICTION = {
    "metric": "collisions",
    "baseline": "reactive_avoid",
    "min_effect": 0.5,
    "cost_metric": "spl",
}
