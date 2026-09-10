"""
core/scenario.py

A scenario is one task on one scene: where the robot starts, where it must get
to, how long it has, what it can sense, and which seed it runs under.

Two things live here rather than in the controller on purpose.

**The sensor configuration belongs to the scenario.** Beam count, field of view
and maximum range are part of the task, identical across conditions A, B, C and
D. If a controller could choose them, a condition could win by asking for a
better sensor, and the ablation would measure nothing.

**The seed belongs to the scenario.** One np.random.Generator is built from it
and handed to controller.reset(). Nothing in core/ or controllers/ may touch the
global np.random.

load() validates and raises. A scenario whose goal is unreachable does not fail
at run time with a confusing zero score — it fails at load, loudly, because the
SPL of an impossible task is not a number anyone should report.

    python -c "import core.scenario as s; print([x.id for x in s.all_scenarios()])"
"""

import math
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from core.geometry import (LAB42, clearance, geodesic_distance,
                           occupancy_grid, segment_clearance,
                           shortest_path_length, ROBOT_RADIUS)
from core.metrics import GOAL_RADIUS
from core.scene_spec import bounds


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

DEFAULT_SCENE = "scenes/lab42_v1.usd"

# Sensor configuration. Pre-registration material: fixed once, identical in
# every condition, and not to be tuned after the first evaluation run.
#
# 360 deg because the analytic raycast costs nothing and a forward-only scan
# would make "back out of the corner" untestable. 181 beams is 2 deg spacing,
# which resolves the 0.9 m doorway from 8 m away. 6.0 m is half the room's long
# axis: far enough to plan through the doorway, short enough that the blind
# corner is genuinely a surprise and LimitSpeedToVisibleRange has something to
# bite on.
N_BEAMS = 181
FOV = 2.0 * math.pi
RANGE_MAX = 6.0

# Twist saturation, applied by the runner to every condition alike.
# Matches the modest indoor limits drive_go2.py already uses.
MAX_LIN = 0.8      # m/s
MAX_ANG = 1.0      # rad/s

# How close the robot must be, with a clear line of sight, before a withheld
# goal is disclosed to it. PRE-REGISTRATION MATERIAL: fixed once, identical in
# every condition, and not to be tuned after the first evaluation run.
#
# Bounded on both sides, and neither bound is arbitrary. It must sit well above
# GOAL_RADIUS (0.5) or detection would coincide with arrival and a search task
# would test nothing but wandering. It must sit well below RANGE_MAX (6.0), or
# the robot is handed the goal position at a range its own sensor could not
# plausibly have resolved a small object at — which would be ground truth
# entering Observation through the back door. 2.0 m also means the divider and
# the y=5 wall genuinely hide the goal across most of the room, which is the
# property that makes lab42 usable as a search environment at all.
DETECT_RADIUS = 2.0


@dataclass(frozen=True)
class DriftSpec:
    """Localisation drift: a seeded random walk on the pose estimate.

    DECLARED HERE, IMPLEMENTED IN chunk 13. It lives in this module rather than
    with the implementation because the sealed held-out split references it and
    that module has to import before the drift model exists. A scenario that
    sets `drift` fails loudly at validate() until it does — see _drift_ready().

    Seeded from Scenario.seed, so drift is part of the episode, not noise on top
    of it: the same scenario and seed produce the same drift in every condition.
    """

    sigma_xy: float      # metres per sqrt(second) of random walk on (x, y)
    sigma_yaw: float     # radians per sqrt(second) of random walk on yaw


# The localiser kinds. A closed vocabulary, because "unknown kind" must be an
# error at validate() rather than a silent fall-through to a perfect pose.
PERFECT = "perfect"
SLAM = "slam"
KINDS = frozenset({PERFECT, SLAM})


@dataclass(frozen=True)
class LocalisationSpec:
    """What produces `pose_estimate`, when it is not `drift` and not the truth.

    DECLARED HERE, IMPLEMENTED IN chunks 19 and 23 — the same pattern
    `DriftSpec` follows one class up, and for the same reason: the field list is
    fixed now so that nothing after chunk 19 has to touch the `Scenario`
    contract. `kind="perfect"` is constructible today (chunk 19 builds
    `PerfectLocaliser`); `kind="slam"` fails loudly at `make_localiser()` until
    chunk 23 exists.

    The SLAM parameters below are declared and unused. That is deliberate: they
    are the knobs chunks 21-24 need, written down while the contract is being
    changed anyway, so that adding one later is not a second contract change.
    Every one of them is a determinism decision as much as an accuracy one —
    epic 03 spends accuracy on determinism wherever the two trade, so there is
    no convergence tolerance here and no iterate-until-small.
    """

    kind: str = PERFECT
    # Real SLAM runs at scan rate, not control rate; chunk 24 measures the
    # budget and sets this. 0.0 means "every control step".
    scan_rate: float = 0.0
    # Scans per submap (chunk 22) and the fixed Gauss-Newton iteration count
    # (chunk 23). A FIXED count, never a tolerance: it is the only formulation
    # whose cost and whose output are both identical run to run.
    submap_scans: int = 0
    solver_iterations: int = 0
    # Huber kernel width on loop-closure edges only. Odometry edges are not
    # robustified: a bad odometry edge means the front end failed, and that
    # should surface rather than be absorbed.
    huber_delta: float = 0.0

    # ⚠️ DEAD-RECKONING NOISE, added 2026-08-31 while chunk 19 was still open.
    #
    # Chunk 19 said `update()` receives the true pose because "a localiser needs
    # somewhere to derive its own odometry from -- the simulator standing in for
    # wheel encoders and an IMU, and chunk 20 is what stops it being a free
    # ride." **Chunk 20 does not.** Its four effects are all LIDAR effects
    # (range noise, dropout, spurious max-range, angular jitter) and it says
    # "four effects, and no more". Nothing degrades odometry.
    #
    # With perfect odometry the estimate is exact, there is nothing for a loop
    # closure to correct, and the epic's whole claim -- that the pose estimate
    # becomes a consequence of what the robot did -- is unmeasurable. So the
    # noise lives HERE rather than in SensorNoiseSpec: it is a property of the
    # localiser's dead reckoning, not of the lidar, and a scenario may sensibly
    # want one without the other.
    #
    # Units match DriftSpec's deliberately -- per sqrt(second), so the two are
    # directly comparable when chunk 25 asks whether SLAM beats the random walk.
    odom_sigma_lin: float = 0.0        # metres per sqrt(second)
    odom_sigma_ang: float = 0.0        # radians per sqrt(second)


@dataclass(frozen=True)
class SensorNoiseSpec:
    """Per-beam lidar noise. DECLARED HERE, IMPLEMENTED IN chunk 20.

    `core.geometry.raycast` is exact — the analytic distance to the nearest
    rectangle, with no noise, no dropouts and no mixed pixels. Scan matching
    against exact data is not SLAM; it is a lookup that reports centimetre
    accuracy and means nothing. So chunk 21 is uninterpretable without this, and
    the overstatement would be invisible.

    **Off by default, and opt-in per scenario.** `Observation.ranges` is read by
    every controller and every node that exists; turning noise on globally would
    change `reactive_avoid`'s behaviour and silently invalidate `results/` and
    the sealed split in one commit.

    ⚠️ `inf` is load-bearing. `core/geometry.py` returns `inf` rather than
    `range_max` specifically so a controller can tell "nothing out there" from
    "a wall exactly at the sensor limit", and `LimitSpeedToVisibleRange` turns
    on exactly that difference. Both `dropout` and `spurious_max_range` produce
    `inf`, so a noisy scenario WEAKENS a distinction a card depends on. That is
    realistic, and it is a real effect on that card's node — it goes in the
    write-up rather than being tuned away.
    """

    # Gaussian range noise, as a fraction of the measured range. A 2D lidar at
    # this scale is roughly +/-3 cm at 6 m, so 0.005 is the stand-in figure.
    # A plausible sensor, not the Go2's actual sensor; chunk 20 says so.
    sigma_range_frac: float = 0.0
    # Per-beam Bernoulli probability of no echo at all -> inf.
    dropout: float = 0.0
    # Per-beam Bernoulli probability that a beam which DID hit reports inf.
    spurious_max_range: float = 0.0
    # Gaussian jitter on the beam bearing, applied BEFORE casting, so the
    # matcher cannot exploit a perfectly regular beam grid. Radians.
    sigma_angle: float = 0.0


@dataclass(frozen=True)
class Scenario:
    id: str
    family: str
    split: str                       # "dev" or "held_out"
    start: Tuple[float, float, float]        # (x, y, yaw)
    goal: Tuple[float, float]
    time_limit: float                # sim seconds
    seed: int
    scene: str = DEFAULT_SCENE
    n_beams: int = N_BEAMS
    fov: float = FOV
    range_max: float = RANGE_MAX
    max_lin: float = MAX_LIN
    max_ang: float = MAX_ANG

    # Is Observation.goal populated at t=0? False makes this a search task: the
    # controller is handed None until it has found the goal itself. The goal
    # above is unchanged either way and still scores the episode — withholding
    # it is a fact about the controller's view, not about the task's solution.
    goal_known: bool = True
    detect_radius: float = DETECT_RADIUS

    # None means the pose estimate is exact, which is what every episode
    # recorded so far assumed. Declared before it is implemented; see DriftSpec.
    drift: Optional[DriftSpec] = None

    # Epic 03. ADDITIVE and MUTUALLY EXCLUSIVE with `drift` -- validate()
    # rejects a scenario setting both, because one field decides what produces
    # `pose_estimate` and two fields both claiming to is a scenario nobody can
    # interpret. `drift` is NOT renamed, NOT deprecated and NOT touched: the
    # sealed split, every committed result and core/tests/test_drift.py all
    # reference it, and renaming a field to make a new one fit tidily is how a
    # frozen contract stops being frozen.
    localisation: Optional[LocalisationSpec] = None
    sensor_noise: Optional[SensorNoiseSpec] = None

    def beam_angles(self):
        """Beam bearings in the robot frame.

        endpoint=False on a full circle, otherwise the first and last beam are
        the same ray and one bearing is sampled twice.
        """
        half = 0.5 * self.fov
        full = abs(self.fov - 2.0 * math.pi) < 1e-9
        return np.linspace(-half, half, self.n_beams, endpoint=not full)

    @property
    def world(self):
        """The geometry this scenario is measured against.

        A derived property, deliberately not a field: chunk 11a declared the
        Scenario field list closed, and nothing here is stored, sealed or
        hashed. It exists so the rollout can ask every scenario the same
        question, and so harness.micro.MicroScenario — which is NOT a benchmark
        scenario and must never be mistaken for one — can answer it differently.
        """
        return LAB42

    def rng(self):
        """The controller's randomness. Handed to controller.reset()."""
        return np.random.default_rng(self.seed)

    def drift_rng(self):
        """The drift model's randomness — a SEPARATE stream from rng().

        This separation is load-bearing, not tidiness. Drift must be identical
        across conditions on the same scenario and seed (#10), and the four
        conditions draw different amounts of randomness from their own rng. If
        the drift shared that stream, condition A and condition B would face
        different drift on the "same" episode and the ablation would be
        comparing two different worlds.

        The salt is arbitrary and fixed; what matters is that it is not the
        controller's stream and never changes.
        """
        return np.random.default_rng([self.seed, 0xD41F7])

    def sensor_rng(self):
        """The sensor noise model's randomness (chunk 20). A THIRD stream.

        Same argument as `drift_rng`, and it is the one that protects the
        ablation: noise must be identical across conditions however much
        randomness a controller draws. If noise came off the shared stream, a
        condition that happened to sample one extra number would face a
        different scan sequence, and the four conditions would be compared on
        different worlds.
        """
        return np.random.default_rng([self.seed, 0x5E7150])

    def localisation_rng(self):
        """The localiser's randomness (chunk 23). A FOURTH stream, for the same
        reason again.

        Separate from `sensor_rng` as well: a localiser that draws — for a
        sampled initial guess, say — must not shift the noise realisation the
        scan matcher is being asked to cope with, or changing the back end would
        silently change the front end's input.
        """
        return np.random.default_rng([self.seed, 0x10CA11])

    def disclosed_at(self, pose_xy):
        """Would the goal be disclosed to a controller standing here?

        The rule, in one place, because the runner and the sim-free rollout in
        core/tests/kinematic.py both have to apply it and a node that behaves
        differently under the two is a node whose unit test means nothing.

        Line of sight is exact rather than sampled: segment_clearance() returns
        0.0 precisely where the segment crosses a wall, and it is the same
        primitive the visibility graph is built from — so "can see through
        there" and optimal_distance()'s "can go through there" agree by
        construction rather than by coincidence. It is deliberately NOT the
        inflated grid: sight lines are rays, not corridors a body fits down.

        This answers a question about one pose. The LATCH lives in the caller,
        because it is episode state and this module holds no episode state.
        """
        if self.goal_known:
            return True
        gx, gy = self.goal
        x, y = float(pose_xy[0]), float(pose_xy[1])
        if math.hypot(gx - x, gy - y) > self.detect_radius:
            return False
        return float(segment_clearance(np.array([x, y]),
                                       np.array([gx, gy]))) > 0.0

    def optimal_distance(self):
        """The SPL denominator: least distance that satisfies the goal test.

        Two things it is deliberately NOT.

        Not the 8-connected grid geodesic. That can only travel at multiples of
        45 degrees and so overstates this route by 0.4769 m on
        `doorway_traverse` — 4.5%, and unchanged by grid resolution, because
        digitisation bias belongs to the connectivity.

        Not the distance to the goal POINT. The episode ends the moment the
        robot is within GOAL_RADIUS of the goal, and path_length is summed only
        to that step, so charging the denominator for a final half metre the
        robot is never asked to walk measures a different journey from the
        numerator.

        Together those two were about 1.0 m of slack in an ~10.6 m denominator,
        enough that every controller within ~9% of optimal scored exactly 1.0.
        """
        return shortest_path_length(self.start[:2], self.goal,
                                    stop_radius=GOAL_RADIUS)


# The development set: seen during generation, prompts iterated freely against
# it. The held-out set is deliberately NOT authored here — it must be sealed
# before any generation happens, and is the next task after this epic.
DEV_SCENARIOS = [
    Scenario(
        id="doorway_traverse",
        family="navigate_to_goal",
        split="dev",
        # The scene's own GO2_START and GOAL, so the default episode is the
        # scene as built. Yaw 0 = facing +x, straight at the divider.
        start=(1.5, 1.5, 0.0),
        goal=(10.5, 6.5),
        # ~11.1 m geodesic at a 0.8 m/s cap is ~14 s optimal. 120 s is an 8x
        # margin: generous enough that a slow-but-correct controller is scored
        # as a success, tight enough that a stuck one terminates.
        time_limit=120.0,
        seed=1001,
    ),
    Scenario(
        id="blind_corner",
        family="navigate_to_goal",
        split="dev",
        # South of the doorway against the divider, facing north. The route
        # rounds (6, 5) — the corner the y=5 wall exists to create — so what is
        # east of the divider is occluded until the robot is in the doorway.
        # This isolates the visibility/speed trade-off from the rest of the run.
        start=(5.0, 3.0, math.pi / 2),
        goal=(8.0, 6.0),
        # ~5.2 m geodesic, ~7 s optimal. Short limit on purpose: a controller
        # that creeps everywhere should not score the same as one that only
        # slows where it cannot see.
        time_limit=45.0,
        seed=1002,
    ),
    Scenario(
        id="return_home",
        family="navigate_to_goal",
        split="dev",
        # Same geometry reversed. Cheap asymmetry check: a controller tuned to
        # one direction of the doorway should show up here.
        start=(10.5, 6.5, math.pi),
        goal=(1.5, 1.5),
        time_limit=120.0,
        seed=1003,
    ),
]


# ---------------------------------------------------------------------------

_BY_ID = {s.id: s for s in DEV_SCENARIOS}


def _held_out():
    """The sealed split, imported LAZILY and nowhere else in this module.

    Lazy on purpose. A module-level import would mean that everything importing
    core.scenario transitively imports core.scenarios_held_out, and the AST
    guard in core/tests/test_held_out.py — which is what the seal is actually
    made of — would be checking a condition nothing could violate and nothing
    could satisfy. Keeping it inside the function keeps the guard meaningful.
    """
    from core.scenarios_held_out import HELD_OUT_SCENARIOS
    return HELD_OUT_SCENARIOS


def all_scenarios(split=None):
    """Dev scenarios, always. Held-out only when asked for BY NAME.

    `all_scenarios()` and `all_scenarios("dev")` never return a held-out
    scenario — a batch runner that iterates "everything" must not silently
    evaluate against the split it is supposed to be held out from.
    """
    if split == "held_out":
        return list(_held_out())
    return [s for s in DEV_SCENARIOS if split is None or s.split == split]


def _drift_ready():
    """Does the drift model exist yet? Feature probe, not a version flag.

    Same shape as the API ladders in drive_go2.py: ask whether the thing is
    importable rather than asserting what release it arrived in. Chunk 13
    creates core/drift.py; until then a scenario carrying a DriftSpec must fail
    loudly, because the alternative is a return_degraded episode running with
    drift silently disabled and scoring localisation_failures: 0 — which is
    exactly the structural bug epic 02 exists to close.
    """
    try:
        from core.drift import DriftModel        # noqa: F401
    except ImportError:
        return False
    return True


def _sensor_noise_ready():
    """Does the sensor noise model exist yet? Feature probe, not a version flag.

    Chunk 20 creates `core/sensor_noise.py`. Until then a scenario carrying a
    SensorNoiseSpec must fail loudly, because the alternative is chunks 21-23
    matching scans against `core.geometry.raycast`'s EXACT analytic ranges,
    reporting centimetre accuracy, and being wrong in a way nothing shows.
    """
    try:
        from core.sensor_noise import SensorNoise      # noqa: F401
    except ImportError:
        return False
    return True


def validate(scenario):
    """Raise ValueError if the scenario is not runnable. Returns it otherwise.

    Every check here is one that would otherwise surface as a confusing zero
    score hours into a batch run.
    """
    min_x, min_y, max_x, max_y = bounds()
    for label, (x, y) in (("start", scenario.start[:2]), ("goal", scenario.goal)):
        if not (min_x <= x <= max_x and min_y <= y <= max_y):
            raise ValueError(
                f"scenario {scenario.id!r}: {label} ({x}, {y}) is outside the "
                f"room bounds {bounds()}")

    grid = occupancy_grid()
    for label, (x, y) in (("start", scenario.start[:2]), ("goal", scenario.goal)):
        if not grid.is_free((x, y)):
            raise ValueError(
                f"scenario {scenario.id!r}: {label} ({x}, {y}) is inside a wall "
                f"or within the robot radius of one")

    # Reachability is decided on the grid, not on the visibility graph. The
    # grid field is exhaustive over the whole room, so "no path" is a statement
    # about the floorplan; the visibility graph answers one query and a bug in
    # it would masquerade as an unreachable goal.
    d_grid = geodesic_distance(scenario.start[:2], scenario.goal)
    if not math.isfinite(d_grid):
        raise ValueError(
            f"scenario {scenario.id!r}: goal {scenario.goal} is UNREACHABLE from "
            f"start {scenario.start[:2]} — no path exists on the inflated grid. "
            f"Note that lab42 has a sealed room at x in [6.075, 11.925], "
            f"y in [0.075, 4.925] with no door.")

    # optimal_distance() subtracts GOAL_RADIUS from a start->goal path, which is
    # only the same thing as the distance to the goal DISC when that disc is
    # entirely walkable — otherwise the run-in it shortens is not run-in-able.
    if float(clearance(np.asarray(scenario.goal, dtype=float))) < ROBOT_RADIUS + GOAL_RADIUS:
        raise ValueError(
            f"scenario {scenario.id!r}: goal {scenario.goal} is within "
            f"{ROBOT_RADIUS + GOAL_RADIUS} m of a wall, so part of the "
            f"GOAL_RADIUS disc the episode terminates in is not reachable. "
            f"optimal_distance() would then understate the denominator. Move "
            f"the goal, or change how it is measured.")

    d = scenario.optimal_distance()
    if not math.isfinite(d):
        raise ValueError(
            f"scenario {scenario.id!r}: goal {scenario.goal} is reachable on the "
            f"occupancy grid ({d_grid:.3f} m) but the visibility graph found no "
            f"taut path. That is a bug in core.geometry.shortest_path, not a "
            f"property of the scenario.")

    if scenario.time_limit <= 0:
        raise ValueError(f"scenario {scenario.id!r}: time_limit must be positive")
    if scenario.n_beams < 1:
        raise ValueError(f"scenario {scenario.id!r}: n_beams must be positive")

    # Search tasks. Both bounds on detect_radius are load-bearing, not
    # tidiness: see DETECT_RADIUS.
    if not scenario.goal_known:
        if scenario.detect_radius <= GOAL_RADIUS:
            raise ValueError(
                f"scenario {scenario.id!r}: detect_radius "
                f"{scenario.detect_radius} is not greater than GOAL_RADIUS "
                f"{GOAL_RADIUS}, so the goal would be disclosed no earlier than "
                f"the episode ends. That is not a search task.")
        if scenario.detect_radius > scenario.range_max:
            raise ValueError(
                f"scenario {scenario.id!r}: detect_radius "
                f"{scenario.detect_radius} exceeds range_max "
                f"{scenario.range_max}, so the goal would be disclosed at a "
                f"range the robot's own sensor cannot resolve. That is ground "
                f"truth entering Observation.")

    # One field decides what produces `pose_estimate`. Two fields both claiming
    # to is not a scenario anyone can interpret, and the failure would be
    # silent: whichever the runner happened to check first would win.
    if scenario.drift is not None and scenario.localisation is not None:
        raise ValueError(
            f"scenario {scenario.id!r}: sets BOTH drift and localisation, and "
            f"they are mutually exclusive — each one fully determines "
            f"pose_estimate. Use `drift` for the seeded random walk that the "
            f"sealed split and every committed result use; use `localisation` "
            f"for epic 03's localisers ({sorted(KINDS)}). For a ground-truth "
            f"control, LocalisationSpec(kind={PERFECT!r}) is the explicit "
            f"spelling of what leaving both unset already does.")

    if scenario.localisation is not None:
        kind = scenario.localisation.kind
        if kind not in KINDS:
            raise ValueError(
                f"scenario {scenario.id!r}: unknown localisation kind "
                f"{kind!r}. Known: {sorted(KINDS)}")
        # NOT probed here. `make_localiser()` owns the readiness check, because
        # `validate()` runs on every scenario at import and must not drag a
        # SLAM stack in behind it. The scenario is well-formed; whether it can
        # RUN is the runner's question, and it fails loudly there.

    if scenario.sensor_noise is not None and not _sensor_noise_ready():
        raise ValueError(
            f"scenario {scenario.id!r}: carries a SensorNoiseSpec but "
            f"core.sensor_noise does not exist yet, so the scan would be the "
            f"exact analytic raycast and every scan-matching number computed "
            f"from it would be an overstatement — invisibly. SensorNoiseSpec "
            f"is declared ahead of its implementation on purpose (chunk 19); "
            f"chunk 20 implements it.")

    # A declared-but-unimplemented field is a hard error, never a silent no-op.
    if scenario.drift is not None and not _drift_ready():
        raise ValueError(
            f"scenario {scenario.id!r}: carries a DriftSpec but core.drift does "
            f"not exist yet, so the pose estimate would be exact and "
            f"localisation_failures would be structurally 0. DriftSpec is "
            f"declared ahead of its implementation on purpose (chunk 11a) so "
            f"the sealed split can reference it; chunk 13 implements it.")

    return scenario


def load(scenario_id):
    """Fetch a scenario by id, validated. Raises KeyError or ValueError.

    Dev ids resolve without touching the sealed split. A held-out id resolves
    too — Week 5 has to be able to run these — but naming one is an explicit
    act, which is the line the seal draws. What it must never be is a side
    effect of asking for "all scenarios"; see all_scenarios().
    """
    if scenario_id in _BY_ID:
        return validate(_BY_ID[scenario_id])
    for scenario in _held_out():
        if scenario.id == scenario_id:
            return validate(scenario)
    raise KeyError(
        f"unknown scenario {scenario_id!r}. Dev: {sorted(_BY_ID)}. "
        f"Held-out ids exist but are not listed here on purpose; see "
        f"core/scenarios_held_out.py.")
