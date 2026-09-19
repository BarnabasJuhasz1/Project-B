"""Ant-style outbound path integration as a steering modifier."""

import math

import numpy as np

from nodes.contract import Action, Direction, Port, PortType, Status

_ACC_TICK = 0.1          # accumulator period (s); the ant's "stride"
_UNIT = 0.08             # metres advanced per stride at the vx saturation limit
_K = 4.009e-5            # Eq. 3 constant, deg^-1
_MAX_PHI_STEP = 90.0     # Eq. 3 normalisation: one update reaches 90 deg at most
_L_READY = 6.0           # strides before phi is trusted (~0.5 m of gained ground)
_BEAM_CLEAR = 1.5        # a candidate bearing must read at least this far
_CORRIDOR_CLEAR = 0.6    # every beam near a candidate must read at least this
_CORRIDOR_HALF = 0.35    # rad, half width of that corridor (body half-width + margin)
_FRONT_HALF = math.radians(15.0)
_YAW_GAIN = 1.5
_VX_MAX = 0.8
_WZ_MAX = 1.0


def _wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


class SteerAlongAntOutboundVector(Action):
    """Hold heading on an ant's reckoned outbound bearing while the goal is unseen.

    Mechanism (Wehner-style outbound path integration): two running scalars are
    carried instead of a vector sum - a mean direction ``phi`` and a
    turn-discounted length ``l`` in strides. Per stride,
    ``l += s/UNIT - |d|/90`` and ``phi += k*(180+d)*(180-d)*d / l``. The division
    by ``l`` anneals the estimate: early strides swing ``phi`` freely, and once
    the robot has genuinely gained ground a single turn barely moves it, so the
    committed outbound bearing stiffens with distance and no time constant needs
    tuning. That turns coverage of an unmapped interior into long straight
    excursions away from the start rather than a diffusive walk.

    Both scalars are fed *only* by the commanded twist - never by
    ``obs.pose_estimate``, which carries a seeded random walk. ``phi`` and the
    idiothetic heading ``psi`` live in the same drifting frame, so their
    difference is a robot-frame bearing that stays correct however far that
    frame has slipped from the truth.

    Revises ``wz`` (``adjust``) to turn onto the admissible lidar bearing closest
    to that outbound direction, and bounds ``vx`` (``cap``) to what the chosen
    bearing's clearance and turn angle support - a bound, so it composes with
    other caps and can never make the robot faster than the base asked.

    Declines - leaving the base's command byte-identical - whenever the goal has
    latched (coverage is over and the base can home), while ``l < 6`` strides
    (``phi`` is still dominated by the first few ticks), when no bearing within
    +-90 deg of ``phi`` is admissible, and for as long as the front sector is
    blocked or the robot is in contact; obstacle emergencies belong to the
    avoidance leaves to its right, not here.
    """

    WRITES = ("vx", "wz")

    PORTS = (
        Port("goal_known", PortType.FLAG, Direction.IN),
        Port("ant_outbound_bearing", PortType.BEARING, Direction.OUT),
        Port("ant_outbound_confidence", PortType.CONFIDENCE, Direction.OUT),
    )

    PRECONDITION = (
        "The goal has not yet been disclosed, the turn-discounted path-integration "
        "length has reached 6 strides, and some lidar bearing within 90 degrees of "
        "the reckoned outbound direction is clear enough to drive down."
    )

    EFFECT = (
        "Steers the robot onto the path integrator's outbound bearing by replacing "
        "wz with a proportional turn toward the clear bearing nearest that "
        "direction, capping vx at what the chosen bearing's clearance supports."
    )

    TAGS = ("path-integration", "exploration", "coverage", "idiothetic", "steering")

    def reset(self, scenario, rng):
        self._phi = 0.0          # reckoned bearing from start, degrees, idiothetic frame
        self._psi = 0.0          # idiothetic heading, radians, same frame
        self._l = 1.0            # turn-discounted length, strides
        self._last_t = None
        self._acc_t = 0.0
        self._acc_s = 0.0
        self._acc_d = 0.0
        self._prev_cmd = (0.0, 0.0, 0.0)
        self._blocked = False
        self._sel_t = None
        self._sel = None

    # ---------------- accumulator (runs every step, trigger or not) -------------

    def _integrate(self, obs, ports):
        """Advance the ant integrator once per control step, from self-motion only."""
        if getattr(self, "_last_t", None) is None:
            self.reset(None, None)
            self._last_t = obs.t
            self._prev_cmd = self._pending_tuple(ports)
            return
        dt = obs.t - self._last_t
        if dt <= 0.0:
            return
        self._last_t = obs.t
        dt = min(dt, 0.5)

        vx, vy, wz = self._prev_cmd
        self._prev_cmd = self._pending_tuple(ports)

        self._acc_t += dt
        self._acc_s += math.hypot(vx, vy) * dt
        self._acc_d += math.degrees(wz * dt)
        self._psi = _wrap_pi(self._psi + wz * dt)

        if self._acc_t < _ACC_TICK:
            return

        s_n, d_n = self._acc_s, self._acc_d
        self._acc_t = self._acc_s = self._acc_d = 0.0

        # Eq. 1b with |d|: a signed discount would let a left turn lengthen the
        # home vector, which the geometry cannot mean.
        self._l = max(1.0, self._l + s_n / _UNIT - abs(d_n) / 90.0)

        d_n = max(-180.0, min(180.0, d_n))
        step = _K * (180.0 + d_n) * (180.0 - d_n) * d_n / self._l
        step = max(-_MAX_PHI_STEP, min(_MAX_PHI_STEP, step))
        self._phi = _wrap_deg(self._phi + step)

    def _pending_tuple(self, ports):
        cmd = getattr(ports, "command", None)
        if cmd is None:
            return (0.0, 0.0, 0.0)
        return (float(cmd.vx), float(cmd.vy), float(cmd.wz))

    # ---------------- perception helpers ---------------------------------------

    def _ranges(self, obs):
        r = np.asarray(obs.ranges, dtype=float)
        rmax = float(obs.range_max)
        return np.where(np.isfinite(r), np.minimum(r, rmax), rmax)

    def _front_blocked(self, obs, r):
        ang = np.asarray(obs.range_angles, dtype=float)
        front = np.abs((ang + math.pi) % (2.0 * math.pi) - math.pi) <= _FRONT_HALF
        return bool(front.any() and r[front].min() < _CORRIDOR_CLEAR)

    def _select(self, obs, r):
        """Clear bearing closest to the outbound direction, as (bearing, range)."""
        ang = np.asarray(obs.range_angles, dtype=float)
        n = ang.size
        if n == 0:
            return None

        # Corridor test: every beam within +-0.35 rad of a candidate must be clear.
        # The scan spans a full circle, so the neighbourhood minimum is a roll.
        half = max(1, int(round(_CORRIDOR_HALF * n / (2.0 * math.pi))))
        local = r.copy()
        for k in range(1, half + 1):
            local = np.minimum(local, np.minimum(np.roll(r, k), np.roll(r, -k)))

        theta_star = _wrap_pi(math.radians(self._phi) - self._psi)
        delta = (ang - theta_star + math.pi) % (2.0 * math.pi) - math.pi
        ok = (r >= _BEAM_CLEAR) & (local >= _CORRIDOR_CLEAR) & (np.abs(delta) <= math.pi / 2.0)
        if not ok.any():
            return None

        idx = int(np.flatnonzero(ok)[np.argmin(np.abs(delta[ok]))])
        return _wrap_pi(float(ang[idx])), float(r[idx])

    # ---------------- contract --------------------------------------------------

    def precondition(self, obs, ports):
        self._integrate(obs, ports)

        if obs.goal is not None or bool(ports["goal_known"]):
            return False
        if self._l < _L_READY:
            return False

        r = self._ranges(obs)
        if self._blocked:
            # Released only once the emergency that set it has actually cleared.
            if obs.contact or self._front_blocked(obs, r):
                return False
            self._blocked = False
        if obs.contact or self._front_blocked(obs, r):
            # Handled inside tick so the one "stop" cap still gets emitted.
            return True

        sel = self._select(obs, r)
        if sel is None:
            return False
        self._sel_t, self._sel = obs.t, sel
        return True

    def tick(self, obs, ports):
        self._integrate(obs, ports)

        ports["ant_outbound_bearing"] = _wrap_pi(math.radians(self._phi) - self._psi)
        ports["ant_outbound_confidence"] = min(1.0, self._l / 60.0)

        r = self._ranges(obs)
        if obs.contact or self._front_blocked(obs, r):
            self._blocked = True
            pending = ports.command
            # Cap only a forward request: capping a reverse one would strand the
            # base's own escape, which is not this node's business to override.
            if pending.vx > 0.0:
                ports.cap(vx=0.0)
                self._prev_cmd = (0.0, pending.vy, pending.wz)
            return Status.FAILURE

        sel = self._sel if self._sel_t == obs.t else self._select(obs, r)
        if sel is None:
            return Status.FAILURE
        e, d_e = sel

        wz_cmd = max(-_WZ_MAX, min(_WZ_MAX, _YAW_GAIN * e))
        speed_cap = _VX_MAX * max(0.0, math.cos(e))
        speed_cap *= max(0.0, min(1.0, (d_e - _CORRIDOR_CLEAR) / 1.4))

        pending = ports.command
        ports.adjust(wz=wz_cmd)
        ports.cap(vx=speed_cap)

        # Feed the integrator what was actually commanded, not what the base asked.
        vx_eff = math.copysign(min(abs(pending.vx), speed_cap), pending.vx)
        self._prev_cmd = (vx_eff, pending.vy, wz_cmd)
        return Status.SUCCESS


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.15,
    "cost_metric": "min_clearance",
}
