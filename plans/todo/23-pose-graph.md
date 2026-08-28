# 23 — Pose graph: the back end

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 22 · **Needs Isaac Sim:** no

## Why

Chunk 22 proposes constraints. This chunk **applies** them: a 2D pose graph whose nodes are
submap origins and whose edges are odometry and loop closures, optimised so the whole trajectory
is consistent with everything observed.

It is also the chunk that finally produces `SlamLocaliser` — the thing `Scenario.localisation`
has been failing loudly for since chunk 19.

## Decided

**Sparse Gauss-Newton with a fixed iteration count.** Not Levenberg-Marquardt with a convergence
tolerance, and not "iterate until the update is small". A fixed count is the only formulation
whose cost and whose *output* are both identical run to run, and this epic spends accuracy on
determinism wherever the two trade.

`scipy.sparse.linalg.spsolve` is available (scipy 1.17 in the env, checked). Build the sparse
normal equations directly; the graph is small enough that nothing cleverer is warranted.

*Rejected:* g2o / GTSAM bindings. Faster, better tested, and they bring a compiled dependency
into a `core/` that is deliberately importable with no simulator and no build step — and their
iteration behaviour is one more thing to pin.

**A Huber robust kernel on loop-closure edges.** Chunk 22's three gates will not catch every bad
closure; the kernel bounds the damage one does rather than letting it dominate the least squares.
Odometry edges are *not* robustified — a bad odometry edge means the front end failed and that
should surface, not be absorbed.

**Optimise on closure, not every step.** The graph changes only when a submap finishes or a
closure is accepted. Re-solving at the control rate would be almost all wasted work and would put
a variable cost in the inner loop. Between solves, the current pose is the last optimised submap
origin composed with the running match — which is what chunk 22 already produces.

**`error_estimate()` comes from the graph, and is a prediction.** Same rule as everywhere else in
this project: it must never be the realised error. Two candidates, and the chunk picks one and
records why:

- the marginal covariance of the current pose (principled, needs a sparse inverse — expensive);
- a proxy: accumulated odometry uncertainty since the last accepted closure (cheap, monotone
  between closures, drops when one lands — which is exactly the qualitative shape a controller
  needs to condition on).

Start with the proxy; it is the behaviour `IsLocalisationDegraded` reads, and it is honest as long
as the docstring says it is a proxy. Promote to the marginal only if a card needs a calibrated
number.

**Angle wrapping and the yaw parameterisation are where this goes wrong.** Residuals on θ must be
wrapped to (−π, π] *inside* the residual, the information matrix must be scaled so metres and
radians are commensurable, and the anchor node must be fixed or the system is singular. All three
are one-line mistakes with three-hour debugging sessions; each gets a test.

## Build

- `core/slam/graph.py` — `PoseGraph`: nodes, edges, `residuals()`, `jacobian()`, `solve()`.
  Fixed iterations, fixed anchor, Huber on closure edges.
- `core/slam/localiser.py` — `SlamLocaliser` implementing chunk 19's protocol, wiring
  20 → 21 → 22 → 23 together, and exposing `diagnostics()`: submap count, edge count, accepted
  and rejected closures by gate, last solve residual, iterations, and solve wall-clock.
- `core/scenario.py` — `LocalisationSpec` becomes constructible; the `_drift_ready()`-style guard
  from chunk 19 is retired.

## Done when

- **`SlamLocaliser` satisfies the chunk-19 protocol** and a scenario setting `localisation` now
  loads and runs.
- **A synthetic graph with a known answer is recovered.** Build a square trajectory with exact
  odometry plus one exact closure; the optimiser must return the ground-truth poses to solver
  precision. If this fails, nothing downstream is interpretable.
- **Angle wrapping**: a loop with a net rotation of 2π optimises correctly, not to a trajectory
  spun once round.
- **A fixed anchor**: the system is non-singular and the solution does not translate freely.
- **The Huber kernel bounds a bad closure**: inject a deliberately wrong closure edge and assert
  the trajectory error stays within a stated multiple of the un-injected case.
- **Bitwise determinism**: the same episode twice gives byte-identical `slam.csv`, and identical
  optimised poses. Threading off (`OMP_NUM_THREADS=1` where BLAS is involved), and stated.
- **End to end on a benchmark scenario**: `doorway_traverse` with SLAM localisation reaches the
  goal under `reactive_avoid`, with final pose error reported against `PerfectLocaliser` on the
  same seed.
- **SLAM beats the random walk on the same scenario** — final pose error under `SlamLocaliser`
  is lower than under `DriftModel` at DRIFT_MODERATE. If it is not, the epic has not earned its
  place and that is the finding.

## Not in scope

The cost budget and the localiser rate (chunk 24), and any recalibration of thresholds or metrics
(chunk 25). Get it correct here; make it affordable and comparable next.
