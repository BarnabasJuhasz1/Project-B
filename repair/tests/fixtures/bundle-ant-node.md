# node-3cb6a0af73f4

## The principle card this node was built from

```yaml
id: ant_path_integration_goal_vector
node_name: SteerByPathIntegratedGoalVector

mechanism: A Cataglyphis fortis forager continuously maintains a single running estimate of the direction back to its start point, and it does NOT do this by true vector summation (the paper's central negative result). It carries two scalars: phi_n, the mean direction in which it has so far travelled, and l_n, the distance so far travelled counted in unit lengths (strides). Each unit step, having turned by delta relative to phi_n, it rotates its stored direction by an increment that is (a) proportional to a saturating odd function of delta and (b) inversely proportional to l_n. The paper's approximate form is Eq. 2, phi_{n+1} = phi_n + delta / l_n, and the form that fits all their data is Eq. 3:

    phi_{n+1} = phi_n + k * (180deg + delta) * (180deg - delta) * delta / l_n,
    with k = 4.009 x 10^-5 deg^-1

k is exactly the normaliser that makes the numerator peak at 90 degrees (the product is maximal at delta ~= 103.9deg, where it evaluates to 89.99deg), as the paper states. The home direction at any moment is phi_n + 180deg. Why it helps: the 1/l_n term makes the stored direction progressively stiffer the further the animal has walked, so late perturbations - a detour, a stride miscount, a momentary heading wobble - move the home vector by an amount that shrinks as 1/l. It is a one-pole low-pass on bearing whose time constant is distance travelled, and it needs no position variable at all: only turns and step counts. Its price is a known, signed, systematic error (Fig. 2B), largest for near-reversals, and zero at delta = 180deg where the increment vanishes identically. The paper argues this is affordable because left and right turns are equally frequent in nature and their opposite-signed errors cancel (Discussion, p. 5289).

trigger: Fires when the goal has been disclosed (goal is not None) AND at least 1.0 m of commanded translation has accumulated since the integrator's anchor was set, so that l_n >= ~7 unit steps and the 1/l_n damping is actually doing something. Below that threshold the port is false and the tree should fall through to whatever computes goal bearing directly from pose_estimate. The leaf is a bearing source only: it reads no lidar and claims no obstacle competence, so it must sit at lower priority than an avoidance leaf.

derivation: What maps onto what. (i) The ant's drift-free celestial compass maps onto pose_estimate yaw plus the commanded Twist direction. The paper shows the result is unchanged whether the sun or the polarization compass is used (p. 5288), so heading is treated here as the trusted channel and translation as the untrusted one - which is also the split this robot has, since a random walk added to (x, y, yaw) corrupts accumulated displacement far faster than it corrupts instantaneous heading. This split is my reading, not the paper's. (ii) The ant's stride counter (proprioception, not position) maps onto the integral of the Twist this node itself commands: d = sqrt(vx^2 + vy^2) * dt. It deliberately does NOT map onto ||delta pose_estimate||, which would reinject exactly the drift the mechanism is meant to survive. Mine. (iii) The ant's unit length (one stride) maps onto u = 0.15 m. Mine: a Go2 body is ~0.7 m and a comfortable trot stride is roughly a fifth of that, and at the 0.8 m/s speed cap this emits ~5 unit steps per second, which is dense enough for the 1/l_n filter to behave like the continuous Eq. 2 limit the paper takes ("As l_n is much larger than the unit length, Eq. 1a can be transformed (for l -> inf) to Eq. 2"). (iv) The ant integrates toward the nest, i.e. toward the origin of its own path. This robot's goal is a fixed known point that is generally not the path origin, so I extend the scheme: the integrator estimates displacement from an anchor, and the anchor-to-goal offset is carried as a constant. This extension is mine.
State and update, exactly.
  - Anchor: at episode start if goal is not None, else at the first step where
    goal latches from None to known. Store A = (ax, ay) = pose_estimate xy at
    that instant, and the constant offset G = goal - A. Never re-anchor
    afterwards; re-anchoring from a later pose_estimate would import the drift
    accumulated up to that moment. Anchor early: drift is a random walk from
    near zero, so G is least corrupted at the earliest possible anchor.
  - Initialise phi = current world travel direction = wrap(yaw + atan2(vy, vx))
    in degrees (use yaw if commanded speed is zero); l = 1.0 (never let l fall
    below 1.0 - it is the divisor).
  - Each control step accumulate s += sqrt(vx^2 + vy^2) * dt using the Twist
    this node commanded on the previous step. While s >= u = 0.15:
      s -= u
      theta = wrap(yaw + atan2(vy, vx)) in degrees   # direction of this unit step
      delta = wrap_to_(-180, 180](theta - phi)
      dphi  = 4.009e-5 * (180.0 + delta) * (180.0 - delta) * delta / l   # Eq. 3
      dphi  = clip(dphi, -90.0/l, +90.0/l)            # the paper's 90deg cap
      phi   = wrap(phi + dphi)
      l    += 1.0
    Eq. 3 is used verbatim in DEGREES, because k is quoted in deg^-1; convert
    only at the end. Note the built-in behaviour that delta = 180deg gives
    dphi = 0 - a perfect reversal writes nothing into phi. That is the paper's
    own failure mode, not a bug to patch out.
    (The paper also gives a step-length update, Eq. 1b, but that line is not
    legibly reproduced in the text I was given, so I use the unambiguous
    l_{n+1} = l_n + 1 and flag that this differs from the paper if Eq. 1b
    contains a turn-dependent term.)
  - Position estimate, replacing pose_estimate translation entirely:
      p_hat = A + l * u * (cos phi, sin phi)          # phi in radians here
    and the reciprocal home direction phi + 180deg is what the ant would use
    to return to A.
  - Goal bearing: b = wrap_to_(-pi, pi]( atan2(A.y + G.y - p_hat.y,
                                               A.x + G.x - p_hat.x) - yaw ).
  - Output Twist: wz = clip(1.5 * b, -1.0, 1.0) rad/s;
    vx = 0.8 * max(0.0, cos b) m/s; vy = 0.0.
    The gain 1.5 s^-1 is mine, chosen so a 40-degree bearing error saturates
    wz and the turn completes in under a second at the 1.0 rad/s cap.
  - Terminate (SUCCESS) when ||A + G - p_hat|| < 0.5 m; RUNNING otherwise;
    FAILURE if goal is None.
All numbers above except k = 4.009e-5 deg^-1, the 90-degree cap and the 1/l_n and (180+delta)(180-delta)delta structure are mine.

prediction:
  statement: On scenarios where pose_estimate drifts, a robot steering to a goal bearing reconstructed from this Eq.-3 integrator (translation from commanded Twist, heading from yaw, anchored once) reaches the goal on a strictly larger fraction of seeds than the same robot steering to the bearing recomputed each step from pose_estimate directly, because a 1/l_n-damped accumulator fed by proprioception does not inherit the position random walk. It will simultaneously travel further to get there, since Eq. 3 under-rotates phi on near-reversals and near-reversals are common in cluttered indoor space.
  metric: goal_reached
  baseline: goal_seeker
  min_effect: 0.1
  cost_metric: path_length
```

## The node, as generated

```python
import math

from nodes.contract import Action, Direction, Port, PortType, Status

_K = 4.009e-5        # deg^-1, the paper's normaliser: numerator peaks at ~90 deg
_UNIT = 0.15         # m, one "stride" for a Go2 trot
_DPHI_CAP = 90.0     # deg, the paper's ceiling on a single increment (scaled by 1/l)
_TURN_GAIN = 1.5     # s^-1
_CRUISE = 0.8        # m/s
_ARRIVE = 0.5        # m
_ARM_TRAVEL = 1.0    # m of commanded translation before the 1/l damping is meaningful
_MAX_DT = 0.2        # s, guards against a stalled or restarted clock


def _wrap180(a):
    """Wrap degrees to (-180, 180] -- the half-open side matters: delta = 180 must
    survive as +180 so that Eq. 3's (180 - delta) factor zeroes the increment."""
    return 180.0 - ((180.0 - a) % 360.0)


def _wrap_pi(a):
    return math.pi - ((math.pi - a) % (2.0 * math.pi))


class SteerByPathIntegratedGoalVector(Action):
    """Cataglyphis-style path integration (Mueller & Wehner Eq. 3) as a steering source.

    Carries two scalars instead of a position: phi, the mean direction travelled so
    far, and l, distance travelled in unit strides. Each stride it rotates phi by
    k*(180+delta)*(180-delta)*delta/l, so the stored direction stiffens as 1/l and
    late perturbations move the home vector less and less. Displacement from a
    once-set anchor is reconstructed as l*u*(cos phi, sin phi) and the goal bearing
    is taken from that, NOT from pose_estimate -- the whole point is that strides
    come from the commanded Twist (proprioception) while pose_estimate translation
    carries a random walk.

    Revises wz (adjust: steer onto the reconstructed bearing) and vx (cap: no faster
    than 0.8*cos b, so the robot does not cruise while the goal is off the nose).
    It writes neither before the goal has been disclosed, nor before 1.0 m of
    commanded translation has accumulated since the anchor -- below that l is too
    small for the damping to exist and the base's own goal bearing is better. It
    reads no lidar and claims no obstacle competence.
    """

    WRITES = ("vx", "wz")
    PORTS = (
        Port("pi_goal_bearing", PortType.BEARING, Direction.OUT),
        Port("pi_goal_bearing_valid", PortType.FLAG, Direction.OUT),
    )
    PRECONDITION = (
        "The goal has been disclosed and at least 1.0 m of commanded translation "
        "has accumulated since the path integrator was anchored."
    )
    EFFECT = (
        "Drives along the goal bearing reconstructed by the 1/l-damped stride "
        "integrator rather than the bearing implied by the drifting pose estimate."
    )
    TAGS = ("path-integration", "dead-reckoning", "goal-bearing", "drift", "ant")

    # ---------------- state ----------------

    def _clear(self):
        self._anchor = None      # (ax, ay), pose_estimate xy at disclosure time
        self._offset = None      # goal - anchor, frozen at that same instant
        self._phi = 0.0          # deg, mean direction travelled
        self._l = 1.0            # unit steps; never below 1.0, it is the divisor
        self._s = 0.0            # m, sub-stride remainder
        self._travel = 0.0       # m, commanded translation since the anchor
        self._last_t = None
        self._last_cmd = None    # (vx, vy) actually commanded on the previous step
        self._bearing = None

    def _ensure(self):
        if not hasattr(self, "_anchor"):
            self._clear()

    def reset(self, scenario, rng):
        self._clear()

    # ---------------- mechanism ----------------

    def _course_deg(self, yaw, cmd):
        """World-frame direction of travel for a commanded body-frame twist."""
        if cmd is None:
            return math.degrees(_wrap_pi(yaw))
        vx, vy = cmd
        if math.hypot(vx, vy) <= 1e-9:
            return math.degrees(_wrap_pi(yaw))
        return math.degrees(_wrap_pi(yaw + math.atan2(vy, vx)))

    def _try_anchor(self, obs):
        # Anchor at the earliest possible moment: drift is a random walk from near
        # zero, so the anchor-to-goal offset is least corrupted now and must never
        # be recomputed from a later, dirtier pose_estimate.
        if self._anchor is not None or obs.goal is None:
            return
        px, py, yaw = (float(v) for v in obs.pose_estimate)
        self._anchor = (px, py)
        self._offset = (float(obs.goal[0]) - px, float(obs.goal[1]) - py)
        self._phi = self._course_deg(yaw, self._last_cmd)
        self._l = 1.0
        self._s = 0.0
        self._travel = 0.0

    def _integrate(self, yaw, cmd, dt):
        if cmd is None or dt <= 0.0:
            return
        speed = math.hypot(cmd[0], cmd[1])
        if speed <= 0.0:
            return
        course = self._course_deg(yaw, cmd)
        self._s += speed * dt
        self._travel += speed * dt
        while self._s >= _UNIT:
            self._s -= _UNIT
            delta = _wrap180(course - self._phi)
            # Eq. 3 verbatim, in degrees, because k is quoted in deg^-1.
            dphi = _K * (180.0 + delta) * (180.0 - delta) * delta / self._l
            lim = _DPHI_CAP / self._l
            dphi = max(-lim, min(lim, dphi))
            self._phi = _wrap180(self._phi + dphi)
            self._l += 1.0

    def _p_hat(self):
        phi = math.radians(self._phi)
        r = self._l * _UNIT
        return (self._anchor[0] + r * math.cos(phi),
                self._anchor[1] + r * math.sin(phi))

    def _goal_xy(self):
        return (self._anchor[0] + self._offset[0],
                self._anchor[1] + self._offset[1])

    # ---------------- contract ----------------

    def precondition(self, obs, ports):
        # All bookkeeping happens here, on every tick including the ones we decline:
        # the stride count has to keep advancing while the base is driving, or the
        # 1.0 m arming threshold would never be crossed.
        self._ensure()

        t = float(obs.t)
        dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), _MAX_DT)
        self._last_t = t

        yaw = float(obs.pose_estimate[2])
        had_anchor = self._anchor is not None
        self._try_anchor(obs)
        if had_anchor:
            self._integrate(yaw, self._last_cmd, dt)

        # Record the base's proposal now; tick() overwrites it with what we actually
        # let through, so the integrator is always fed the executed command.
        pending = getattr(ports, "command", None)
        if pending is not None:
            self._last_cmd = (float(pending.vx), float(pending.vy))

        if self._anchor is None or obs.goal is None:
            self._bearing = None
            ports["pi_goal_bearing_valid"] = False
            return False

        gx, gy = self._goal_xy()
        px, py = self._p_hat()
        self._bearing = _wrap_pi(math.atan2(gy - py, gx - px) - yaw)
        ports["pi_goal_bearing"] = float(self._bearing)

        ready = self._travel >= _ARM_TRAVEL
        ports["pi_goal_bearing_valid"] = bool(ready)
        return ready

    def tick(self, obs, ports):
        b = self._bearing
        if b is None:
            return Status.FAILURE

        wz = max(-1.0, min(1.0, _TURN_GAIN * b))
        v_limit = _CRUISE * max(0.0, math.cos(b))

        ports.adjust(wz=wz)
        ports.cap(vx=v_limit)

        # Mirror cap semantics locally rather than re-reading the port, so the
        # integrator is fed the same number the runner will execute.
        if self._last_cmd is not None:
            base_vx, vy = self._last_cmd
            self._last_cmd = (math.copysign(min(abs(base_vx), v_limit), base_vx), vy)

        gx, gy = self._goal_xy()
        px, py = self._p_hat()
        if math.hypot(gx - px, gy - py) < _ARRIVE:
            return Status.SUCCESS
        return Status.RUNNING


PREDICTION = {
    "metric": "goal_reached",
    "baseline": "goal_seeker",
    "min_effect": 0.1,
    "cost_metric": "path_length",
}
```

## What it scored

verdict: DEGENERATE
reason:  lost 10 world(s) the base won [seeds 2004, 2007, 2008, 2009, 2010 (+5 more)] without colliding more, and stuck 3.86x as often and travelled 0.62x the distance
n:       20 worlds
outcomes: both_fail 10, both_ok 0, broke 10, fixed 0
its own claim: goal_reached moved not measured against a floor of 0.1

## Per-world outcomes

seed  base  node outcome      fire  changed
---------------------------------------------
  2000     0     0 both_fail   0.985    0.985
  2001     0     0 both_fail   0.953    0.953
  2002     0     0 both_fail   0.981    0.981
  2003     0     0 both_fail   0.986    0.986
  2004     1     0 broke       0.938    0.938
  2005     0     0 both_fail   0.959    0.959
  2006     0     0 both_fail   0.987    0.987
  2007     1     0 broke       0.984    0.984
  2008     1     0 broke       0.981    0.981
  2009     1     0 broke       0.986    0.986
  2010     1     0 broke       0.982    0.982
  2011     1     0 broke       0.984    0.984
  2012     1     0 broke       0.979    0.979
  2013     0     0 both_fail   0.909    0.909
  2014     1     0 broke       0.911    0.911
  2015     1     0 broke       0.982    0.982
  2016     0     0 both_fail   0.986    0.986
  2017     0     0 both_fail   0.986    0.986
  2018     0     0 both_fail   0.976    0.976
  2019     1     0 broke       0.985    0.985

## The conditions the measurement ran under

Every world below was generated by core.worldgen and scored kinematically.

  tier                : maze
  drift               : None
  localisation        : None
  goal_known          : True
  sensor_noise        : None
  time_limit          : 120.0 s

⚠️ An axis reading `None` was NOT VARIED. `drift: None` means the pose
estimate handed to the controller is exact — there is no localisation
error of any kind in these worlds. If this node's claim depends on an
axis that reads None, it has not been tested against its own claim.

## The contract: what this node could have read

A node sits in a Layer. The shared base proposes a whole Twist every
tick; later nodes revise only the fields they declared in WRITES.

Two combinators and no more:
  ports.cap(vx=...)     min() of magnitude, sign kept
  ports.adjust(vx=...)  LAST WRITER WINS — it replaces the field

A node that declines leaves the base's command byte-identical.

Port types available: BEARING, CONFIDENCE, FLAG, POSE, RANGES, REGION, SPEED

Ports the shared base publishes every tick, readable by any later node:
  nav_heading   BEARING  the direction the base has chosen to travel
  nav_stuck     FLAG     the base believes it is stuck or off its plan
  nav_progress  SCALAR   1.0 falling to 0.0 as time passes with no progress

⚠️ The base gates its own forward speed on being aligned with
nav_heading, and raises its turn rate to a floor until it is. A node
that overwrites wz without reading nav_heading removes the base's
ability to reach the alignment its own speed gate is waiting for.
