"""Ant-style path-integration outbound steering for unmapped coverage.

Implements ant_path_integration_outbound_bearing: a foraging ant carries two
running scalars rather than a vector sum - a mean outbound direction phi_n and a
turn-discounted length l_n in stride units - and updates them incrementally from
self-motion alone.  Steering the robot to hold its heading on phi_n (i.e.
directly away from the reckoned start) yields long, straight, self-lengthening
excursions instead of a diffusive walk, which is what covering unseen indoor
floor cheaply requires.

Two properties are the reason to steal it, and neither is accuracy.  The update
is purely idiothetic - it consumes one unit advance and one turn angle per tick
and never an external position fix - so it cannot inherit the error of
obs.pose_estimate, which on some scenarios is a seeded random walk away from the
truth.  And the division by l_n anneals the estimate: early ticks swing phi
hard, and once l_n is large a single turn barely moves it, so the committed
outbound bearing stiffens with distance travelled with no tuned time constant.
"""

import math

import numpy as np

from core.observation import Command
from nodes.contract import Action, Direction, Port, PortType, Status
from harness.micro import MicroScenario, room


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "reactive_avoid",
    "min_effect": 0.15,
    "cost_metric": "min_clearance",
    "statement": (
        "On unmapped indoor scenarios where obs.goal is None until seen, a leaf that holds "
        "heading on the ant integrator's outbound bearing phi_n (Eq. 3, fed only by commanded "
        "twist) reaches the goal on a larger fraction of seeds than a reactive_avoid baseline, "
        "because turn-discounted displacement from the start grows monotonically and "
        "self-stiffens with distance instead of diffusing; the advantage must persist on "
        "scenarios where pose_estimate drifts, since the integrator never reads it."
    ),
}


# --- ant accumulator (card: "Mapping" and "Numbers") -------------------------
ACC_DT = 0.1          # s, one ant stride -> one 10 Hz accumulator tick
U_STRIDE = 0.08       # m, distance covered in one tick at the 0.8 m/s saturation limit
K_ANT = 4.009e-5      # deg^-1, the paper's Eq. 3 constant
INC_CLIP_DEG = 90.0   # Eq. 3 is normalised so the increment maximally reaches 90 deg
L_MIN = 6.0           # strides (~0.5 m) before phi_n counts as defined
BOOTSTRAP_TICKS = 6   # driven accumulator ticks allowed before the l_n gate binds

# --- steering (card: "Output"; all of these numbers are the card's, sized from
#     the Go2 body width ~0.31 m and the 0.8 m/s, 1.0 rad/s saturations) -------
FREE_MIN = 1.5        # m, a bearing must see at least this far to be admissible
CLEAR_MIN = 0.6       # m, minimum return anywhere in the fan around that bearing
LOBE = 0.35           # rad, half-width of that fan
WZ_GAIN = 1.5         # rad/s per rad of bearing error
WZ_MAX = 1.0          # rad/s, runner saturation
VX_MAX = 0.8          # m/s, runner saturation
VX_RAMP = 1.4         # m, range over which forward speed ramps in above CLEAR_MIN
FWD_HALF = math.radians(15.0)   # the "forward 30 deg sector" emergency check
HALF_PI = 0.5 * math.pi


def _wrap_pi(a):
    """Wrap radians to (-pi, pi].  Works on scalars and on ndarrays."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_deg(a):
    """Wrap degrees to (-180, 180]."""
    return (a + 180.0) % 360.0 - 180.0


class SteerAlongAntOutboundVector(Action):
    """Steer the robot along an ant path-integrator's outbound bearing.

    Per accumulator tick the leaf consumes only its own last commanded twist:
    s_n = hypot(vx, vy) * dt metres of advance and d_n = degrees(wz * dt) of
    turn.  From those it maintains an idiothetic heading psi_n, the paper's
    turn-discounted odometer l_{n+1} = l_n + s_n/u - |d_n|/90 (a 90 deg turn
    cancels a whole unit of advance, so l measures ground genuinely gained away
    from the start rather than path walked), and the outbound bearing by Eq. 3,
    phi_{n+1} = phi_n + k(180 + d)(180 - d)d / l_n.

    Both phi and psi live in the same idiothetic frame, so their difference
    theta* = phi_n - psi_n is a robot-frame bearing that stays correct even as
    the pair drifts together - which is the whole point of never reading
    obs.pose_estimate.  The leaf then drives at the admissible lidar bearing
    closest to theta*, so obstacles deflect it but never redefine where "away"
    is; as l_n grows those deflections stop moving phi and the excursion
    straightens out by itself.
    """

    PORTS = (
        Port("outbound_bearing", PortType.BEARING, Direction.OUT),
        Port("outbound_vector_ready", PortType.FLAG, Direction.OUT),
    )
    PRECONDITION = (
        "The goal has not been disclosed yet, the turn-discounted ant odometer has run far "
        "enough for the outbound bearing to be defined, and at least one admissible lidar "
        "bearing lies within 90 degrees of it."
    )
    EFFECT = (
        "Drives the robot outward along the reckoned outbound bearing, increasing "
        "turn-discounted displacement from the start pose and so covering unseen floor."
    )
    TAGS = ("exploration", "coverage", "path-integration", "idiothetic", "unknown-goal", "ant")

    # Per-episode state, declared at class level so that a tick arriving before
    # reset() still reads sane defaults instead of raising.  Every write below
    # goes to the instance.
    _phi_deg = 0.0        # reckoned outbound bearing from the nest, idiothetic frame
    _psi_rad = 0.0        # idiothetic heading, psi_0 = 0 at the start pose
    _l = 1.0              # turn-discounted length, strides, clamped at >= 1
    _ready = False        # has l ever reached L_MIN (latched)
    _driven_ticks = 0     # accumulator ticks in which we actually commanded motion
    _t_prev = None
    _pend_s = 0.0         # metres of advance not yet folded into a stride
    _pend_d = 0.0         # degrees of turn not yet folded into a stride
    _pend_dt = 0.0
    _last_cmd = (0.0, 0.0, 0.0)

    def reset(self, scenario, rng):
        self._phi_deg = 0.0
        self._psi_rad = 0.0
        self._l = 1.0
        self._ready = False
        self._driven_ticks = 0
        self._t_prev = None
        self._pend_s = 0.0
        self._pend_d = 0.0
        self._pend_dt = 0.0
        self._last_cmd = (0.0, 0.0, 0.0)

    # ------------------------------------------------------------------ ant --
    def _stride(self, s_n, d_n):
        """One ant stride: Eq. 3 for phi, Eq. 1b (with |d|) for l."""
        l_n = self._l

        # Eq. 3 verbatim, on l_n as it stood before this stride.  Within the
        # Go2's |wz| <= 1.0 rad/s this is |d_n| <= 5.73 deg, so the product
        # collapses to phi += 1.299 * d_n / l_n: the ant's systematic ~30%
        # overshoot over an exact vector sum is kept deliberately, since for
        # choosing "which way is away" it is irrelevant and balanced left/right
        # turning cancels it.
        inc = K_ANT * (180.0 + d_n) * (180.0 - d_n) * d_n / l_n
        inc = max(-INC_CLIP_DEG, min(INC_CLIP_DEG, inc))
        self._phi_deg = _wrap_deg(self._phi_deg + inc)

        # |d_n|, not signed d_n: signed would make a left turn lengthen the home
        # vector, which the geometry cannot mean.
        self._l = max(1.0, l_n + s_n / U_STRIDE - abs(d_n) / 90.0)
        self._psi_rad += math.radians(d_n)

        if s_n > 1e-9:
            self._driven_ticks += 1
        if self._l >= L_MIN:
            self._ready = True

    def _ingest(self, obs):
        """Fold elapsed time at the last commanded twist into whole strides.

        Idempotent within one observation, so precondition() and tick() may both
        call it on the same obs without double-counting.
        """
        if self._t_prev is None or obs.t < self._t_prev:
            self._t_prev = obs.t
            return
        dt = obs.t - self._t_prev
        if dt <= 0.0:
            return
        self._t_prev = obs.t

        vx, vy, wz = self._last_cmd
        self._pend_dt += dt
        self._pend_s += math.hypot(vx, vy) * dt
        self._pend_d += math.degrees(wz) * dt

        while self._pend_dt >= ACC_DT:
            frac = ACC_DT / self._pend_dt
            s_n = self._pend_s * frac
            d_n = self._pend_d * frac
            self._pend_s -= s_n
            self._pend_d -= d_n
            self._pend_dt -= ACC_DT
            self._stride(s_n, d_n)

    def _vector_defined(self):
        """Gate (b) of the trigger, made deadlock-free.

        l_n >= 6 is latched: once the bearing is defined it stays defined, even
        if heavy turning later discounts l back down.  The bootstrap window
        matters because l only grows from motion this leaf itself commands - a
        leaf gated purely on l >= 6 that is the only thing able to move the
        robot would never drive, never accumulate, and never open its own gate.
        Inside the window phi is still exactly its initial value, i.e. straight
        out of the nest, so nothing ill-defined gets through.  The window counts
        driven ticks rather than wall-clock, so a leaf held up by an obstacle at
        t = 0 can still bootstrap once the way clears.
        """
        return self._ready or self._driven_ticks < BOOTSTRAP_TICKS

    # ------------------------------------------------------------- geometry --
    def _plan(self, obs):
        """Return (theta_star, chosen bearing or None, range along it, blocked)."""
        rng_max = float(obs.range_max)
        r = np.asarray(obs.ranges, dtype=float)
        r = np.where(np.isfinite(r), r, rng_max)
        ang = np.asarray(obs.range_angles, dtype=float)

        # A bearing is admissible only if it sees FREE_MIN ahead and the whole
        # +-LOBE fan around it clears CLEAR_MIN - the beams are spread over a
        # full circle, so the fan is found by angle, never by index offset.
        delta = _wrap_pi(ang[None, :] - ang[:, None])
        lobe_min = np.where(np.abs(delta) <= LOBE, r[None, :], np.inf).min(axis=1)
        admissible = (r >= FREE_MIN) & (lobe_min >= CLEAR_MIN)

        theta_star = _wrap_pi(math.radians(self._phi_deg) - self._psi_rad)
        err = np.abs(_wrap_pi(ang - theta_star))
        candidate = admissible & (err <= HALF_PI + 1e-9)

        fwd = np.abs(ang) <= FWD_HALF
        blocked = bool(obs.contact) or (bool(fwd.any()) and float(r[fwd].min()) < CLEAR_MIN)

        if not bool(candidate.any()):
            return theta_star, None, 0.0, blocked

        i = int(np.argmin(np.where(candidate, err, np.inf)))
        return theta_star, float(ang[i]), float(r[i]), blocked

    def _idle(self):
        """Nothing was commanded, so the integrator is fed nothing next tick."""
        self._last_cmd = (0.0, 0.0, 0.0)

    # ----------------------------------------------------------- behaviour --
    def precondition(self, obs, ports):
        # The accumulator is state, not an action: it runs from t = 0 on every
        # control step this leaf sees, whether or not the leaf then applies.
        self._ingest(obs)

        if obs.goal is not None:
            return False
        if not self._vector_defined():
            return False
        _, bearing, _, _ = self._plan(obs)
        return bearing is not None

    def tick(self, obs, ports):
        self._ingest(obs)
        theta_star, bearing, d_e, blocked = self._plan(obs)

        ports["outbound_bearing"] = float(theta_star)
        ports["outbound_vector_ready"] = bool(self._ready)

        # Goal disclosed and latched: coverage is over, this leaf no longer
        # applies.  FAILURE rather than SUCCESS so control falls through to the
        # sibling that walks to the now-known goal; the accumulator keeps running.
        if obs.goal is not None:
            self._idle()
            return Status.FAILURE

        if not self._vector_defined():
            self._idle()
            return Status.FAILURE

        # Emergency: stop translating and hand over.  This leaf never handles
        # obstacle emergencies itself - the recovery leaf to its right does.
        if blocked:
            ports.drive(Command(0.0, 0.0, 0.0))
            self._idle()
            return Status.FAILURE

        # No open bearing within 90 deg of "away": trigger clause (c) fails.
        if bearing is None:
            self._idle()
            return Status.FAILURE

        # e is the selected robot-frame bearing: wz turns onto it, cos e scales
        # forward speed by how well the body already points along it, and the
        # range ramp backs off as the chosen direction shortens.
        e = bearing
        wz = max(-WZ_MAX, min(WZ_MAX, WZ_GAIN * e))
        ramp = max(0.0, min(1.0, (d_e - CLEAR_MIN) / VX_RAMP))
        vx = VX_MAX * max(0.0, math.cos(e)) * ramp

        ports.drive(Command(vx, 0.0, wz))
        self._last_cmd = (vx, 0.0, wz)
        return Status.RUNNING


def micro(seed=0):
    """A corridor the mechanism follows and a chamber the widest gap swallows.

    The only route to the goal is the 2.4 m corridor along the south wall.
    North of it is one 16 x 7 m open chamber, reachable from the start strip and
    from two bay mouths cut into the corridor ceiling.  From the start pose the
    chamber subtends far more open angle than the corridor does, and at every
    bay the mouth is a wider gap than the corridor continuation, so a gap
    follower is pulled off route at t = 0 and again each time it drifts back -
    it diffuses over 100 m^2 of empty floor with no goal in it.

    The ant integrator sets phi_0 to the start heading, which points down the
    corridor; every bay leaves the eastward beam admissible at zero bearing
    error, and l_n is in the hundreds of strides by mid-corridor, so the bays
    cannot move the committed outbound bearing.  The goal is 13.3 m away, well
    beyond the 6 m sensor horizon, so it is not visible from the start.
    """
    world = room(
        16.0, 10.0,
        boxes=[
            # corridor ceiling, broken by two 2 m bay mouths onto the chamber
            (3.0, 2.4, 6.5, 3.0),
            (8.5, 2.4, 11.0, 3.0),
            (13.0, 2.4, 16.0, 3.0),
        ],
    )
    return MicroScenario(
        id="ant_outbound_corridor_vs_chamber",
        world=world,
        start=(1.2, 1.2, 0.0),
        goal=(14.5, 1.2),
        time_limit=50.0,
        seed=seed,
        goal_known=False,
    )
