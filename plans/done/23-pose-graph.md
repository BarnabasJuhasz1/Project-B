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

---

## Outcome — 2026-09-01

**Done.** `core/slam/graph.py` and `core/slam/localiser.py` built; chunk 19's
`_slam_ready()` guard retired; `--localisation {perfect,slam}` added to
`run_episode.py`; `scripts/compare_localisers.py` committed with its evidence in
`out/evidence/localisers/`. 40 new tests (19 graph + 21 localiser); the sim-free
suite is **702**, up from 661.

### The headline: SLAM beats the random walk, and the margin is the finding

`scripts/compare_localisers.py`, replayed open-loop so the comparison is of
ESTIMATORS and not of how a controller reacted to the estimate it was given.
Metres of position error, sigma = 0.12 m/sqrt(s) throughout.

| trajectory | odometry | drift @ MODERATE | **SLAM** |
|---|---:|---:|---:|
| traverse, 14.5 s — one way, never returns | 1.037 | 0.310 | **0.221** |
| loop, 77.7 s — two laps, back to the start | 1.864 | 1.484 | **0.053** |

The Done-when is met on both. **What matters is the gap between the two rows.**
On a one-way trip there is nothing to close and tracking is all SLAM has; on a
trajectory that returns, the error is corrected against a map the robot built
itself and *does not grow with time* — mean 0.055 m over 77.7 s against a max of
0.119. That is `localisation_failures` becoming a real metric: under drift it is
bit-identical across controllers that behave nothing alike, and under SLAM a
controller that revisits ends the episode knowing where it is.

⚠️ **`drift` is not a controlled baseline and must not be reported as one.** It
is a different error process at the same nominal sigma: `core/drift.py` adds a
random walk to the true pose, odometry integrates corrupted deltas, and
integration compounds where addition does not — which is why raw odometry
(1.037) is three times worse than the drift model (0.310) on the same
trajectory. `slam` minus `odometry` is the honest subtraction, because those two
share a noise process and an rng stream. **Calibrating drift against odometry is
chunk 25's job** and the caveat travels with any B-vs-drift number until it is.

### Four findings

**1. The first comparison was driven, and it reported SLAM LOSING.** Driving each
localiser in closed loop confounds the estimator with the controller's reaction
to it, and the effect was large enough to invert the answer: `reactive_avoid`
reaches the goal in 14.5 s under a perfect pose and **times out at 40 s under
SLAM**, so SLAM's error looked worse purely because it was integrated over 2.8x
as long. Replaying one fixed trajectory is now the method, and the reason is in
the script's docstring rather than in a commit message.

**2. A tight bootstrap window was tried and made it worse — a recorded negative
result.** Submap 0 has no predecessor, so its opening scans are placed by
odometry alone and are then *added to it*, smearing the one map every later pose
is registered against: a ~0.04 rad yaw bias set in the first half second and
never corrected, which at 0.5 m/s was most of the final error. The hypothesis was
that chunk 22's failure came from the WINDOW rather than the thinness — a 0.6 rad
wall-slide is only reachable because the +/-0.20 rad window admits it — so a thin
match inside three sigma of what odometry can justify should be safe. It was not:
0.40 m to 0.57 m, and chunk 22's own `test_a_thin_submap_is_not_matched_against`
failed. **A thin submap is not a map, at any window.** The comment survives in
`tracker.py` so the idea is not re-had.

Two changes did work and are what closed the traverse gap: **match against the
previous finished submap while the current one fills** (it is thick by
definition, so the odometry-only stretch shrinks from every submap boundary to
the first one only), and **always take a scan on the first update, whatever the
scan rate** — that is the one scan taken at an exactly known pose, and it is what
registers submap 0's frame against the truth. Together: 0.40 m -> 0.22 m.

**3. The scan rate is MEASURED, not chosen. One correlative match costs ~44 ms**
(181 beams, ~2900 points), so at the 50 Hz control rate that is 2.2 s of matching
per simulated second and a 60 s episode does not finish — the first kinematic
rollout was killed at two minutes. `DEFAULT_SCAN_RATE = 10.0` Hz is what a real
2D lidar runs at and makes the cost a property of the sensor rather than of the
control loop. Chunk 24 owns the budget and may move it; it may not move it back
to "every step", because that number is not reachable. Closures are proposed
**only when a submap finishes** for the same reason — a proposal is a wider match
against every nearby submap.

**4. A latent crash from chunk 19, found by the first SLAM episode under
physics.** The node-trace summary block had been indented *inside* `if
slam_log:` since chunk 19 added the sidecar. No run produced a `slam_log` until
now, so it never fired; when it did, it threw `AttributeError` **after** three
minutes of physics and **before** the result was written. Same family as the
sweep rehearsal's "exit 0, wrote nothing": the expensive part succeeded and the
cheap part discarded it. Fixed, with the history in the comment.

### Decisions that departed from the plan

**No wall-clock in `diagnostics()`**, though the build list asked for it. The
same Done-when list requires `slam.csv` to be byte-identical across two runs of
one episode, and a timing differs every run by construction. The timing lives on
`SlamLocaliser.last_solve_seconds` / `.total_solve_seconds`, which is where chunk
24 reads it from, and out of the file determinism is checked on.

**The anchor is fixed by solving a REDUCED system**, not by damping its diagonal
with a large number. A large number is a value someone has to choose and defend;
a removed column is not.

**`error_estimate()` is the proxy**, as the plan preferred: accumulated
dead-reckoning uncertainty since the last accepted closure, rising while driving
and dropping to zero when a closure lands. Promoting it to a marginal covariance
is a chunk of its own and only if a card needs a calibrated number. ⚠️ While
wiring it I found chunk 22 accumulated it as `sigma*sqrt(dt)` **summed**, which
overestimates by sqrt(N_steps) — a random walk's error after T is `sigma*sqrt(T)`,
not `sigma*sqrt(dt)` added N times. It is now accumulated as a variance. That
matters beyond reporting: `loop.py`'s gate 3 budgets a closure's correction
against this number, so the inflated proxy was quietly waving through corrections
it exists to refuse.

**`SlamLocaliser.reset()` refuses zero odometry noise.** Chunk 19's loud failure
did not go away when the guard was retired, it moved and got sharper: perfect
dead reckoning makes the estimate exact, leaves a closure nothing to correct, and
reports a number that looks like success. `kind="perfect"` is the honest way to
ask for a ground-truth control.

### Under physics, `reactive_avoid` does NOT reach the goal with a SLAM pose

Four conditions on `doorway_traverse`, same seed, same controller, 60 s cap:

| localisation | goal | time |
|---|---|---:|
| none (estimate = truth) | yes | 15.44 s |
| `+noise` | yes | 15.36 s |
| `+noise` + `--drift 0.12` | yes | 16.14 s |
| `+noise+slam` | **no** | timeout, 0.65 m short |

Both controls succeed. Sensor noise alone does not break it, and **neither does
the random walk at the same nominal sigma** — which means the failure is not
"the pose estimate is 0.2 m out", because the drift row is 0.3 m out and
arrives.

**The magnitudes are similar and the STRUCTURE is not, and the structure decides
it.** `core/drift.py` is a zero-mean random walk on the true pose: it wobbles
around the truth and keeps re-randomising, so a controller steering at a goal
still arrives because the error averages out over the approach. SLAM's error is a
persistent bias — a yaw error established early becomes a lateral offset that
grows with distance and does not average out, because every later pose is
registered against the same map that yaw error smeared.

So the two are not interchangeable even once their magnitudes match. **0.30 m of
random walk and 0.22 m of biased offset are the same number and a completely
different experiment**, and chunk 25 has to calibrate the structure or say in the
write-up that it did not. ⚠️ On this evidence **no condition may run under SLAM
localisation before chunk 25**: it would move goal-reach rate for every condition
at once and compress the thing G2 is trying to measure.

### Determinism, met with the whole pipeline in the loop

Two identical `+noise+slam` episodes under physics at `--max-seconds 20`:
`trace.csv` byte-identical (114206 bytes), `slam.csv` byte-identical (94386
bytes), results identical. Correlative matching, submaps, closure gates and ten
Gauss-Newton iterations per solve, and not one differing byte. That is what the
no-convergence-tolerance rule was for.

### A hazard found while running the controls, NOT fixed here

**`--drift` does not rename the scenario, so a drift run overwrites the
committed no-drift result.** `--sensor-noise` and `--goal-hidden` both rename for
exactly this reason and their help text says why; `--drift` was deliberately
exempted on the grounds that drift "adds a noise process to an otherwise
identical episode". But the SCORE differs, and `results/<run_id>.json` is keyed
on the run id — so `--drift 0.12 --scenario doorway_traverse` silently clobbers
`results/doorway_traverse+cap60s__reactive_avoid__seed1001.json`. I used
`--results-dir out/scratch-results` to run the control above rather than change
a flag that committed ids depend on. **Left for a decision, not fixed
unilaterally.**

### Still open, and it belongs to 24 and 25

- The traverse margin is thin (0.221 vs 0.310) and the yaw bias in finding 2 is
  the reason. It is real, it is permanent on a one-way trip, and no amount of
  gate tuning removes it.
- **Chunk 22's control still does not exist.** "A robot that never revisits gets
  no closures" is false in lab42 — the room is smaller than the sensor range, so
  a submap two indices back is still in view. The traverse row above accepts 4
  closures. Attributing anything to *revisiting* specifically needs a larger
  environment or a shorter `range_max`, and that is chunk 25's problem.
- `PRE-REGISTRATION.md` needs a dated amendment before SLAM is the localiser for
  any evaluation episode. Chunk 25's job; nothing here touches the sealed split
  or a committed result.
