# 19 — The localisation contract

**Epic:** [03](../03-epic-slam.md) · **Depends on:** — · **Needs Isaac Sim:** no

## Why

This is epic 03's [11a](../done/11a-task-contract.md): **the single change to the `Scenario`
contract, made once, ahead of every implementation that needs it.** Nothing after this chunk
touches the field list.

The pattern is copied deliberately. `DriftSpec` was declared in `core/scenario.py` by 11a and
implemented by 13, so that the sealed split could reference a field whose implementation did not
exist yet, and a scenario setting it failed loudly at `validate()` until it did. The same applies
here twice over: `LocalisationSpec` and `SensorNoiseSpec` are declared now and implemented in 23
and 20 respectively.

## Decided

**`drift` is not renamed, not deprecated, not touched.** It stays exactly as it is and keeps
meaning "seeded random walk". Three things reference it — the sealed split, every committed
result in `results/`, and `core/tests/test_drift.py` — and renaming a field to make a new one fit
tidily is how a frozen contract stops being frozen.

`localisation` is **additive and mutually exclusive** with `drift`: `validate()` rejects a
scenario setting both, with a message saying why. One field decides what produces
`pose_estimate`, and two fields both claiming to is a scenario nobody can interpret.

*Rejected:* generalising `drift` to hold any spec. Cleaner on paper, and it silently changes the
meaning of a field that twelve sealed scenarios and every committed manifest already use.

**The `Localiser` protocol has five methods, and `diagnostics()` is one of them.** A localiser
that can only report a pose cannot be debugged and cannot be reported on separately from the
navigation stack — which is the epic's named confounding risk. `diagnostics()` returns a flat
dict of scalars, written into the trace as extra columns.

⚠️ **`TRACE_COLUMNS` must stay fixed-width**, because scoring is a pure function of the trace and
`scripts/rescore.py` depends on it. So diagnostics go to a **separate sidecar file**
(`slam.csv` in the run directory), the way `nodes.csv` does for node tracing — see
[18](../done/18-node-tracing.md), which solved this exact problem and whose reasoning applies
unchanged.

**`PerfectLocaliser` is part of this chunk, not a later one.** It returns the true pose with an
error estimate of 0.0 — behaviourally identical to today's `NoDrift`, but reachable through the
new `localisation` field. It exists so that any SLAM result can be run against a ground-truth
control on the same scenario and seed, and the difference attributed. Building it now costs
nothing and means chunk 24 has its control on day one.

**`error_estimate()` is predicted uncertainty, never realised error.** Same rule as `drift.py`,
restated in the protocol docstring and asserted by a test, because it is the single easiest way
for ground truth to re-enter `Observation` through the back door. A localiser knows its own noise
model; it does not know its own realisation.

## Build

- `core/localisation.py` — the `Localiser` protocol, `PerfectLocaliser`, and `make_localiser()`
  resolving a spec to an implementation the way `core.drift.make_drift` already does.
  ```python
  class Localiser(Protocol):
      def reset(self, scenario, rng) -> None: ...
      def update(self, dt, true_pose, ranges, angles) -> None: ...
      def estimate(self) -> Tuple[float, float, float]: ...
      def error_estimate(self) -> float: ...
      def diagnostics(self) -> dict: ...
  ```
  `update()` receives the **true pose** because a localiser needs somewhere to derive its own
  odometry from — that is the simulator standing in for wheel encoders / IMU, and chunk 20 is
  what stops it being a free ride. It is the one place in the codebase where ground truth crosses
  into a component that feeds `Observation`, and it is fenced: the localiser is not handed
  `scenario.goal`, not handed the world, and its output goes through `estimate()` only.
- `core/scenario.py` — declare `LocalisationSpec` and `SensorNoiseSpec` (field lists only, no
  behaviour), add `Scenario.localisation` and `Scenario.sensor_noise`, extend `validate()`.
- `core/drift.py` — `NoDrift` and `DriftModel` grow `diagnostics()` and are declared to satisfy
  `Localiser`. No behavioural change; a test asserts the traces are byte-identical.
- `scripts/run_episode.py` — build the localiser through `make_localiser()` instead of
  `make_drift()`, and write `slam.csv` when diagnostics are non-empty.
- `core/tests/kinematic.py` — the same substitution, so a unit scenario can carry a localiser.

## Done when

- `$P -m pytest core/tests/ -q` passes, and **every existing test passes unchanged** — this
  chunk adds capability and changes no behaviour.
- A scenario with neither `drift` nor `localisation` produces a **byte-identical** trace to the
  one it produces today, checked against a committed result in `results/`.
- A scenario setting both `drift` and `localisation` is rejected by `validate()` with a message
  that says which to use.
- A scenario setting `localisation` fails loudly until chunk 23 exists — the `_drift_ready()`
  pattern from 11a, reused.
- `PerfectLocaliser` and `NoDrift` produce identical traces on the same scenario and seed.
- `Localiser.error_estimate()` is asserted **not** to equal the realised error, on a scenario
  where the two provably differ.
- `TRACE_COLUMNS` is unchanged, asserted by the existing test.
- `scripts/rescore.py` re-scores a localiser run from its trace with no simulator, unchanged.

## Not in scope

Any SLAM. This chunk builds the socket, not the thing that plugs into it. `SlamLocaliser` is
chunk 23; the noise model that makes it non-trivial is chunk 20.
