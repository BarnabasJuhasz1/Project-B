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

---

## Status 2026-08-31 — built, sim-free suite green, three deviations

`core/localisation.py`, the two specs, the four rng streams, both call sites and 19 tests.
**589 sim-free tests pass and every pre-existing one is unchanged**, which is this chunk's first
Done-when and the reason it was safe to take ahead of its epic.

### Why this chunk was taken and 20–27 were not

The epic's scheduling constraint — *"must not land between chunk 17 and the first evaluation
run"* — binds anything that changes what a controller sees or what a metric measures. Chunk 19
changes neither: `drift` is untouched, `TRACE_COLUMNS` is unchanged, no scenario in the repo sets
`localisation` or `sensor_noise`, and the default path resolves to the same `NoDrift` object it
always did. **Chunk 20 is where the constraint starts to bite**, because sensor noise changes
`Observation.ranges`, which every controller and every node reads.

### Deviation 1 — `DriftLocaliser` is an adapter, not a widened `DriftModel`

The plan said `NoDrift` and `DriftModel` should *"grow `diagnostics()` and be declared to satisfy
`Localiser`"*. **They cannot satisfy it as written.** The protocol's `estimate()` takes no
arguments — a localiser holds its own state, which is the whole difference from a random walk —
whereas `drift.estimate(true_pose)` is handed the truth at call time, and `advance(dt)` is not
`update(dt, true_pose, ranges, angles)`.

Widening `drift.py`'s own methods would mean changing signatures that the sealed split, every
committed result in `results/` and `core/tests/test_drift.py` all depend on, to make a new
interface fit tidily — which is how a frozen contract stops being frozen. So the drift models
keep their methods byte-for-byte, gain only `diagnostics()`, and `DriftLocaliser` does the
adaptation in one class. A test asserts `DriftModel.estimate`'s and `.advance`'s signatures are
still what they were.

### Deviation 2 — `validate()` does not probe SLAM readiness; `make_localiser()` does

The plan reused 11a's `_drift_ready()` pattern wholesale. For `sensor_noise` that is right and it
is implemented that way. For `localisation` it is not: `validate()` runs on every scenario at
import, and probing there would drag the SLAM stack in behind every `load()`. So `validate()`
decides whether the scenario is **well-formed** (known kind, not also carrying `drift`) and
`make_localiser()` decides whether it can **run**. Both still fail loudly; a scenario asking for
SLAM cannot silently execute with a perfect pose, which is the property the pattern exists for.

### Deviation 3 — `sensor_rng()` landed here, not in chunk 20

Chunk 20's Build assigns itself `sensor_rng()`. It is declared here instead, alongside
`localisation_rng()`, because this is the chunk that changes the contract and adding a stream
later would be a second contract change — which is the thing 19 exists to prevent. A **fourth**
stream was added that no chunk asked for: `localisation_rng()` is separate from `sensor_rng()` so
that a localiser which draws cannot shift the noise realisation its own front end is being asked
to cope with. Salts are mirrored onto `harness.micro.MicroScenario`, or a unit scenario would
localise differently from the benchmark.

### The byte-identity check is stronger than the plan asked for

The Done-when says "checked against a committed result in `results/`". Those are physics runs, so
a kinematic rollout cannot be compared to one directly. Instead `test_localisation.py` carries a
**verbatim copy of the pre-chunk-19 rollout** and asserts the new path reproduces it row for row
— on two scenarios, both reference controllers, **with drift and without**. The drifted case is
the one that matters: an ordering mistake between `advance(dt)` and `estimate(pose)` shows up
there and nowhere else. Writing that copy caught a real discrepancy immediately (the termination
radius is 0.4, not 0.5), which is what a regression test of a substitution is for.

### The physics half: byte-identical, and it caught a stale baseline

`run_episode.py --scenario doorway_traverse --controller reactive_avoid --no-record`, diffed
against the trace on disk. **774 lines, byte-identical**, so the runner change is invisible under
real physics as well as under kinematics.

Worth recording how that check nearly gave a false negative. The obvious baseline — the trace the
committed `results/doorway_traverse__reactive_avoid__seed1001.json` points at, from 2026-08-24 —
differs from today's at byte 59 of line 1, and it is **not** chunk 19's doing: that trace has 13
columns and the current `TRACE_COLUMNS` has 14, because chunk **11a** added `goal_known` after it
was recorded. Diffing against it would have reported a difference that chunk 19 did not cause.
The right baseline is the post-11a run of the same episode
(`doorway_traverse__reactive_avoid__seed1001__20260826-173511`), and against that it is exact.

⚠️ **Five committed results are scored from 13-column traces.** They re-score correctly —
`rescore.py` and `test_results_integrity.py` are both green — but any future byte-comparison must
pick a post-11a baseline or it will blame the wrong chunk. Worth a line in `results/README.md`.

`scripts/check_determinism.sh doorway_traverse reactive_avoid` → **PASS: 774 lines identical**.
The inner loop changed, so this needed re-running; it did not need changing.

### Done when — all green

- ✅ Every existing test passes unchanged. **592 sim-free tests** (570 before this chunk).
- ✅ Byte-identical trace with neither field set, under physics and under the kinematic rollout.
- ✅ Both `drift` and `localisation` → rejected by `validate()`, message names which to use.
- ✅ `localisation` fails loudly until chunk 23; `sensor_noise` until chunk 20.
- ✅ `PerfectLocaliser` and `NoDrift` agree exactly.
- ✅ `error_estimate()` asserted not to equal the realised error, where the two provably differ.
- ✅ `TRACE_COLUMNS` unchanged; diagnostics go to a `slam.csv` sidecar on nodes.csv's terms.
- ✅ `rescore.py` unchanged and green.
- ✅ `core.preregistration.verify()` passes — asserted by a test in this chunk, because "adding
  Scenario fields does not move the digest" is exactly the kind of claim that should not be
  taken on trust.

**Not moved to `done/`**: the chunk is complete, but epic 03's Outcome convention wants the
epic's own state recorded, and 20–27 are deliberately not starting. Move it when cycle 1's
evaluation is written up and the epic actually opens.

### One side effect worth knowing

The verification run wrote `results/doorway_traverse+cap60s__reactive_avoid__seed1001.json` — a
new committed-projection entry that did not exist before. It is untracked, `in_container: false`
so it can never be reported, and `test_results_integrity.py` is green with it present. The
`+cap60s` suffix is the default `--max-seconds 60` relabelling, not anything this chunk changed.
Delete it if the committed set should stay as it was.

---

## Outcome — NOT RECORDED AT THE TIME

⚠️ **This is a stub, written 2026-09-15 when the chunk was moved to `done/`. It is a
reconstruction from the artefacts on disk, not a contemporaneous record.** No Checks table
exists for this chunk and none can honestly be written: what was checked while it was built,
what was ruled out, and what was deliberately left unchecked are not recoverable. A reader who
needs that evidence should assume it does not exist rather than that it was clean.

**Moved on the following evidence, all verified 2026-09-15:**

- `core/localisation.py` exists and carries the `Localiser` Protocol, `PerfectLocaliser`, and
  `error_estimate()`.
- `core/drift.py` carries `NoDrift` and its own `error_estimate()`.
- `core/scenario.py` carries `LocalisationSpec` (line 102) and `Scenario.localisation` (line
  227), with `validate()` rejecting a scenario that sets **both** `drift` and `localisation`
  (line 540) and rejecting an unknown localisation kind (line 550).
- `core/scenario.py` carries `localisation_rng()` as a stream separate from `rng()`.
- `core/tests/test_localisation.py` and `core/tests/test_drift.py` exist and pass.
- `TRACE_COLUMNS` is unchanged and still asserted by the existing test.
- The board recorded this chunk as **complete — byte-identical under physics**, and chunks
  21, 22, 22a, 23, 24a and 24b all depend on it and are done with Outcomes of their own.
- The `_slam_ready()` guard this chunk added was retired by chunk 23, which its Outcome records.

**Not verified here:** the byte-identical-trace claim was not re-run; the
`PerfectLocaliser` / `NoDrift` trace-equality check was not re-run; the assertion that
`error_estimate()` differs from the realised error was not re-run. All three are Done-when
items and all three are believed to have passed when the chunk was built.
