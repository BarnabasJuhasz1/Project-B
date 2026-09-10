"""
core/localisation.py

**The socket a localiser plugs into. Not a localiser.**

Chunk 19 of epic 03 — the epic's [11a](../plans/done/11a-task-contract.md): the
single change to the `Scenario` contract, made once, ahead of every
implementation that needs it. `SlamLocaliser` is chunk 23; the sensor noise that
makes it non-trivial is chunk 20. Nothing here does any SLAM.

WHY THIS EXISTS
---------------
`core/drift.py` is a seeded random walk added to the true pose, and it has one
property that is the binding constraint on the whole experiment: **no controller
can influence the recorded pose error.** Drift is injected open-loop, so
`localisation_failures` is bit-identical across controllers that behave nothing
alike, and a controller can lower it only by finishing sooner. That is why
`cards/store.py` refuses any prediction naming it and why the metric is a
manipulation check rather than a performance number.

A real localiser changes exactly one thing, and it is the right one: **the pose
estimate becomes a consequence of what the robot did.** This module generalises
the *producer* of that estimate and leaves the contract alone.

    Localiser (protocol) --+-- NoDrift          today's default: estimate = truth
                           +-- DriftModel       today's random walk -- UNCHANGED
                           +-- PerfectLocaliser ground truth, as a CONTROL
                           +-- SlamLocaliser    chunk 23
                                     |
                                     +--> pose_estimate, pose_error_estimate

THE STRUCTURAL CLAIM
--------------------
**A localiser needs no change to what a controller sees.** It writes into
`Observation.pose_estimate` and `Observation.pose_error_estimate`, two fields
that already exist, and no node, no controller, no condition and no metric is
rewritten. That is chunk 11a's decision paying off: `Observation` carries a
*belief* and never ground truth, precisely so the thing producing that belief
could be replaced later without touching anything written against it.

TWO THINGS THAT ARE EASY TO GET WRONG
-------------------------------------
**`error_estimate()` is predicted uncertainty, never realised error.** Same rule
as `drift.py`, restated here because this is the single easiest way for ground
truth to re-enter `Observation` through the back door. A localiser knows its own
noise model; it does not know its own realisation. A localiser that returned the
realised error would make `IsLocalisationDegraded` a node that reads the answer.

**`update()` receives the true pose, and that is fenced.** A localiser needs
somewhere to derive odometry from, and in simulation that is the simulator
standing in for wheel encoders and an IMU; chunk 20's noise model is what stops
it being a free ride. It is the one place in the codebase where ground truth
crosses into a component that feeds `Observation`, so the fence is explicit: the
localiser is not handed `scenario.goal`, not handed the world, not handed the
occupancy grid, and its output reaches a controller through `estimate()` and
`error_estimate()` only.

SIM-FREE. Imports no isaacsim/omni/torch/pxr/carb, enforced by
core/tests/test_import_guard.py.
"""

try:                                    # pragma: no cover - typing only
    from typing import Protocol, runtime_checkable
except ImportError:                     # pragma: no cover
    Protocol, runtime_checkable = object, (lambda cls: cls)

# `core.scenario` imports nothing from here, so this is a plain one-directional
# dependency and belongs at the top like any other. Keeping it that way is what
# lets the sealed split import core.scenario without dragging a localiser in
# behind it.
from core.scenario import KINDS, PERFECT, SLAM


@runtime_checkable
class Localiser(Protocol):
    """What produces `Observation.pose_estimate`.

    Five methods, and `diagnostics()` is one of them on purpose: a localiser
    that can only report a pose cannot be debugged, and cannot be reported on
    separately from the navigation stack — which is epic 03's named confounding
    risk. Once every condition has a localiser that can fail, a difference in
    goal-reach rate may be a difference in localisation rather than in
    navigation, and `PerfectLocaliser` plus these diagnostics are how the two
    are told apart.
    """

    def reset(self, scenario, rng):
        """Start of episode. `rng` is the localiser's own stream, never the
        controller's — see `Scenario.localisation_rng`."""

    def update(self, dt, true_pose, ranges, angles):
        """One control step. Called BEFORE the Observation is built, so the
        estimate a controller sees is the one this step produced."""

    def estimate(self):
        """The believed pose, `(x, y, yaw)`. No arguments: a localiser holds its
        own state, which is the whole difference from `drift.estimate(pose)`."""

    def error_estimate(self):
        """PREDICTED radial position uncertainty in metres. Never the realised
        error — see the module docstring."""

    def diagnostics(self):
        """A flat dict of scalars, or `{}`. Written to a `slam.csv` sidecar."""


# ---------------------------------------------------------------------------
# Below this line is code, not data.


class PerfectLocaliser:
    """Ground truth, as an experimental CONTROL.

    Behaviourally identical to `drift.NoDrift`, but reachable through the new
    `Scenario.localisation` field rather than by omission. That distinction is
    the entire point: it exists so that any SLAM result can be run against a
    ground-truth control **on the same scenario and the same seed**, and the
    difference attributed to localisation rather than to navigation.

    Epic 03 names confounding as one of its three risks and this is the
    mitigation. Chunk 25 makes reporting the delta mandatory rather than
    optional; building the control now costs nothing and means chunk 24 has it
    on day one.

    ⚠️ This is **not** a localiser and must never be reported as one. It is the
    null hypothesis: what the numbers look like when localisation is free.
    """

    def __init__(self):
        self._pose = (0.0, 0.0, 0.0)

    def reset(self, scenario, rng):
        self._pose = (0.0, 0.0, 0.0)

    def update(self, dt, true_pose, ranges, angles):
        x, y, yaw = true_pose
        self._pose = (float(x), float(y), float(yaw))

    def estimate(self):
        return self._pose

    def error_estimate(self):
        return 0.0

    def diagnostics(self):
        # Deliberately empty rather than {"error": 0.0}: an empty dict is what
        # suppresses the slam.csv sidecar, and a control run has nothing to
        # diagnose. See run_episode.py, which writes the file only when some
        # localiser actually reports something.
        return {}


class DriftLocaliser:
    """`core.drift`'s random walk, wearing the `Localiser` interface.

    ⚠️ **An adapter, and deliberately so.** Chunk 19 as written said `NoDrift`
    and `DriftModel` should "grow `diagnostics()` and be declared to satisfy
    `Localiser`", but they cannot satisfy it as written: the protocol's
    `estimate()` takes no arguments because a localiser holds its own state,
    whereas `drift.estimate(true_pose)` is handed the truth at call time, and
    `advance(dt)` is not `update(dt, true_pose, ranges, angles)`.

    Widening `drift.py`'s own methods to match would mean changing the signature
    of a component that the sealed split, every committed result in `results/`
    and `core/tests/test_drift.py` all depend on — to make a new interface fit
    tidily. That is how a frozen contract stops being frozen. So the drift
    models keep their methods **byte-for-byte** and the adaptation lives here,
    where it is one class and no risk.

    The mapping is exact, which is what makes the trace byte-identical:
    `update()` does what the runner's `drift.advance(dt)` did and remembers the
    pose the runner would have passed to `estimate()` one line later.
    """

    def __init__(self, drift):
        self._drift = drift
        self._true_pose = (0.0, 0.0, 0.0)

    def reset(self, scenario, rng):
        # `rng` is ignored: a DriftModel was already seeded from
        # `scenario.drift_rng()` when it was constructed, and re-seeding it here
        # from a different stream would change every drifted episode ever
        # committed. The parameter exists to satisfy the protocol, not to be
        # used.
        self._drift.reset()
        self._true_pose = (0.0, 0.0, 0.0)

    def update(self, dt, true_pose, ranges, angles):
        # `ranges` and `angles` are ignored. A random walk does not look at the
        # world -- which is precisely the property epic 03 exists to end, and
        # the reason `localisation_failures` is a manipulation check today.
        self._drift.advance(dt)
        x, y, yaw = true_pose
        self._true_pose = (float(x), float(y), float(yaw))

    def estimate(self):
        return self._drift.estimate(self._true_pose)

    def error_estimate(self):
        return self._drift.error_estimate()

    def diagnostics(self):
        return self._drift.diagnostics()

    @property
    def drift(self):
        """The wrapped model. For the runner's log line and for tests; nothing a
        controller can reach."""
        return self._drift


# ⚠️ `_slam_ready()` LIVED HERE AND IS RETIRED (chunk 23).
#
# It was a feature probe: until `core/slam/localiser.py` existed, a scenario
# asking for SLAM had to fail LOUDLY, because the alternative was an episode
# running with a silently perfect pose and scoring `localisation_failures: 0`,
# which looks exactly like success. The module exists now, so the probe can only
# ever return True and a probe that cannot fail is a comment pretending to be
# code. `SlamLocaliser` is imported directly below.
#
# The loud failure did not go away, it MOVED and got sharper:
# `SlamLocaliser.reset()` refuses a spec with zero odometry noise, which is the
# remaining way to configure SLAM into being a free ride.


def make_localiser(scenario):
    """The localiser for this scenario. One place, because the runner and
    `core/tests/kinematic.py` must build it identically — a node whose unit
    scenario localises differently from the benchmark is a node whose unit
    scenario proves nothing. Exactly the argument `core.drift.make_drift`
    already makes, one level up.

    The resolution order is not arbitrary. `drift` is checked first and
    `validate()` rejects a scenario setting both, so the field that twelve
    sealed scenarios and every committed manifest already use keeps its meaning
    and cannot be shadowed by a field none of them has.
    """
    from core.drift import make_drift

    spec = getattr(scenario, "localisation", None)
    if spec is None:
        # The default path, and the one every committed episode took. It is the
        # SAME object it always was, wrapped -- byte-identity here is
        # structural, not something a test has to keep checking.
        return DriftLocaliser(make_drift(scenario))

    if spec.kind == PERFECT:
        return PerfectLocaliser()
    if spec.kind == SLAM:
        # Imported lazily, and that is not a style preference: `core.slam` pulls
        # in scipy, and `core.scenario` is imported by the sealed split, by
        # every node's unit harness and by `runs/design.py`. A top-level import
        # here would make scipy a hard dependency of code that does no SLAM.
        from core.slam.localiser import SlamLocaliser
        return SlamLocaliser(spec, scenario.localisation_rng())

    raise ValueError(
        f"scenario {scenario.id!r}: unknown localisation kind {spec.kind!r}. "
        f"Known: {sorted(KINDS)}")

