# 05 — A larger, randomised environment suite

**Status: PLANNING, 2026-09-13. Nothing built.** Written because chunk 44's growth loop cannot
run on the current environment and the reason is structural, not a matter of effort.

---

## Why this epic exists

Chunk 44 decided that a node earns its place by improving a tree on a shared suite. That suite
does not exist.

- **The dev set is three scenarios**, and `reactive_avoid` solves all three. A growth loop needs
  an environment where the default behaviour *fails*, repeatedly, in different ways.
- **Success rate over three scenarios takes four values.** Most additions tie. Chunk 44 requires
  a pre-declared tie-break as a stopgap; the real fix is here.
- **One room.** lab42 is a single static floorplan: a divider, a doorway, a blind corner. Every
  node ever generated has been aimed at that one geometry.

## ⚠️ The reason this epic matters more than it looks: it fixes the power problem

This is the strongest argument for doing it, and it is not about realism.

`Scenario.seed` currently reaches only `rng()` and `drift_rng()`. On an undrifted scenario a
controller that ignores its rng produces a **bit-identical episode at every seed** — measured on
three controllers, and confirmed under physics in the sweep rehearsal, where all six
(scenario × controller) pairs were identical across all three replicate seeds. So:

- `core/stats.py` resamples **scenarios, not episodes**, because the replicate seeds are the same
  episode.
- The effective N is **12**, not 36.
- `scripts/power_analysis.py` puts the pre-registered rule at **~0.16 power** against a true
  30-point gap. Going from 3 seeds to 10 buys a few points for 3.3x the wall clock.

**A procedurally randomised environment converts the seed axis from a no-op into genuine
samples.** Same scenario family, different layout per seed, means replicate seeds stop being
copies and effective N stops being the scenario count. That is the single largest available
improvement to the study's statistical power, and no amount of extra episodes on the current
suite buys it.

⚠️ It also **invalidates the current bootstrap's justification**: `stats.py` resamples scenarios
*because* seeds are degenerate. If seeds stop being degenerate, the resampling unit has to be
re-derived, and that is `PRE-REGISTRATION.md` material.

## What the user asked for

1. **More, and more complex, environments.**
2. **Randomisation.**
3. **Moving objects** the robot must avoid while navigating.
4. **Localisation stressors** — things that make pose estimation genuinely hard.

All four are in scope for the epic. They are **not** equally cheap, and the ordering below is by
what each one breaks, not by what it adds.

## What each one actually costs

### 1. Randomised static layouts — CHEAPEST, DO FIRST

`core/scene_spec.py` is data and `core/geometry.World` is a frozen tuple of axis-aligned rects
with `bounds`. A generator that emits a `World` from a seed is a small, sim-free piece of code.
Everything downstream already takes `world=` — `occupancy_grid`, `raycast`, `clearance`,
`geodesic_distance`, `shortest_path` — because the micro-world harness forced that
parameterisation already.

Watch: `_GRID_CACHE`, `_FIELD_CACHE` and `_VISGRAPH_CACHE` are keyed on the world, which is
correct but means **one grid per generated layout**. A thousand random layouts is a thousand
cached grids. Needs a bound.

Also needs: a **solvability filter**. `core.scenario.validate()` already uses the geodesic field
to check a scenario is solvable; a generator must reject layouts that are not, and must do so
without silently biasing the distribution toward easy rooms.

### 2. Moving objects — EXPENSIVE, AND IT BREAKS THE SPL DENOMINATOR

The good news, established rather than assumed: **`Observation` needs no change.** A moving
obstacle reaches the controller through `ranges`, the 2D scan, exactly as a wall does. The field
list is frozen and 11a was the one change to it. Nothing here reopens it.

The bad news is `core/geometry.py` is static by construction:

- `raycast` is analytic against a frozen tuple of rects. A moving obstacle makes the world a
  function of **time**, so every geometry query becomes `f(world, t)`.
- The caches above are keyed on the world alone. Time-varying geometry either defeats them or
  needs a different scheme.
- ⚠️ **`shortest_path` is the SPL denominator, and with moving obstacles "the optimal path" is
  no longer well defined.** A path that is optimal at t=0 may be blocked at t=8. SPL is a named
  secondary in `PRE-REGISTRATION.md`. **This is a decision, not an implementation detail** — see
  Open decisions.
- `core/tests/kinematic.py` must move the obstacles identically to the runner, or the fast loop
  and physics diverge further than the 2026-09-12 calibration already found.

Cheapest honest version: obstacles on **deterministic, seeded, scripted trajectories** (a patrol
segment, a fixed loop), not agents with their own policies. Reproducibility is a hard
requirement and an interacting agent is a second controller in the episode.

### 3. Localisation stressors — ⚠️ ENTANGLED WITH THE PRE-REGISTRATION AND EPIC 03

Partly built already: `core/drift.py` (seeded random walk), `core/sensor_noise.py`
(`SensorNoiseSpec`), `core/localisation.py` (`LocalisationSpec`, perfect / slam).
`core.scenario.validate()` refuses a scenario that sets **both** drift and localisation.

⚠️ **`03-epic-slam.md` must not land between now and the first evaluation run.** The seed
offsets and the sealed split's drift magnitudes are calibrated against `core/drift.py`'s current
model, and replacing it moves them underneath `PRE-REGISTRATION.md`. Anything in this epic that
changes how localisation degrades inherits that constraint exactly.

Stressors that do **not** require SLAM (feature-poor corridors, symmetric rooms that induce
aliasing, long featureless runs) are really *layout* properties and belong with item 1 — they
make localisation hard by geometry rather than by changing the estimator. **Prefer those.**

## Open decisions — these are the user's, and the epic should not start without them

1. ~~Does this suite REPLACE lab42 as the benchmark?~~ **DECIDED 2026-09-13 by the user: YES,
   the new suite REPLACES lab42 as the benchmark.** Non-negotiable #10 stays satisfied — there
   is still exactly one benchmark, it is the new one. Consequences, all of which follow
   necessarily and none of which are optional:

   - **A dated append to `PRE-REGISTRATION.md` is REQUIRED**, not optional. It is the document
     every Week 5 number is scored under and it names lab42's split.
   - **The 20 committed `results/*.json` become historical**, scored on a superseded benchmark.
     Repo convention is superseded records are KEPT and never deleted — but they must be
     LABELLED, or a later reader will pool them with new numbers.
   - **lab42's sealed split is retired for benchmarking.** `core/scenarios_held_out.py`,
     `git tag split-v1` and the `00b3312b…` seal stay exactly as they are — that split is never
     re-sealed. The new suite needs a **new** split, authored and sealed separately under chunk
     11's procedural rule: **in a session that has never had the held-out file in context, and
     that has not generated.**
   - **G2's X = 0.30 and the whole power analysis were derived against lab42's 12 scenarios**
     and do not transfer. Both must be re-derived — chunk 50.
   - **`core/preregistration.py` will raise on `verify()`** once the scene constants move. That
     is the registry working as designed; the frozen-constant update is part of the amendment,
     not a workaround for it.
   - **lab42 is NOT deleted.** It remains the dev/debug scene, the fast-loop world for
     `plot_kinematic.py` and `ablate.py`, and the only way to reproduce the historical results.
     It is no longer what anything is *scored* on.
2. **What is SPL's denominator when obstacles move?** Options: the static-world taut path
   (ignores the obstacles, stays comparable to every existing result); the time-respecting
   shortest path (correct, expensive, and a new implementation); or drop SPL to a
   report-only metric on dynamic scenarios and score them on goal-reach alone.
3. **Does the growth loop run on this suite before or after the first evaluation?** The sealed
   split at `00b3312b…` is never re-sealed. A new suite needs a **new** split, sealed under
   chunk 11's procedural rule — in a session that has never had the held-out file in context.
4. **How many layouts per family, and is the layout set frozen or drawn fresh per episode?**
   Frozen is reproducible and re-usable as a regression suite; fresh-per-episode is a better
   generalisation test and cannot be a regression suite. Chunk 44's growth loop needs a
   **frozen** suite to compare incumbent against incumbent+1.

## Provisional chunk breakdown

Deliberately ordered so that each chunk is useful even if the next is never built.

| # | Chunk | Depends on | Breaks |
|---|---|---|---|
| 45 | `World` generator: seeded random static layouts + solvability filter + cache bound | — | nothing |
| 46 | Scenario families over generated layouts; the frozen suite for chunk 44's loop | 45 | nothing |
| 47 | Geometry-induced localisation difficulty (symmetry, featureless runs) | 45 | nothing |
| 48 | Time-parameterised geometry: `raycast(..., t)`, cache strategy, kinematic parity | 45 | SPL denominator, caches |
| 49 | Scripted moving obstacles in USD + the runner | 48 | sim-side work, first non-sim-free chunk |
| 50 | Re-derive the resampling unit and power under randomisation | 46 | `core/stats.py`, PRE-REGISTRATION |

45, 46 and 47 are **sim-free** and unblock chunk 44's growth loop on their own. 48 and 49 are
where the cost is. 50 is where the pre-registration has to be reopened.

## Not in scope

Agents with their own navigation policies (a second controller in the episode, and not
reproducible); multi-room or multi-building environments (out of scope project-wide);
photorealistic or textured variation; any change to `Observation`.

## Chunk 45 outcome — built 2026-09-13, and two findings that outlive it

`core/worldgen.py` + `core/tests/test_worldgen.py` (20 tests). Three tiers,
reproducible from `(tier, seed)`, ~250 ms per world, solvability 48/48.

| tier | tortuosity | free | bottleneck | obstacles |
|---|---|---|---|---|
| open | 1.11 +- 0.07 | 0.75 | 0.84 m | 3.8 |
| cluttered | 1.78 +- 0.44 | 0.63 | 0.53 m | 10.6 |
| maze | 2.10 +- 0.43 | 0.52 | 0.40 m | 19.5 |

### ⚠️ FINDING 1 — `shortest_path_length` does not scale, and SPL is a named secondary

Profiled on one 22-rect `maze` world:

| step | cost |
|---|---|
| `occupancy_grid` | 23.9 ms |
| `_sample_endpoints` | 314.2 ms |
| `geodesic_distance` | 87.5 ms |
| `_bottleneck_radius` | 105.2 ms |
| **`shortest_path_length`** | **43,536 ms** |

The visibility graph samples arcs at every obstacle corner at `ARC_STEP_DEG = 2`,
so its cost is quadratic in obstacle count. lab42 has 7 rects; a generated world
has 3-4x that and pays **forty seconds**.

**This is an epic-level problem, not a worldgen one.** `shortest_path_length` is
the SPL denominator, SPL is a named secondary in `PRE-REGISTRATION.md`, and
scoring an episode on a generated world will pay that cost once per world. Three
ways out, none free, and the choice is the user's:

1. Raise `ARC_STEP_DEG` for generated worlds — cheap, but it changes the SPL
   denominator's accuracy, and the constant's docstring justifies 2 degrees
   carefully (4.6e-5 m error per corner). Pre-registration material.
2. Optimise the visibility graph (prune arc nodes not on the convex frontier,
   or spatial-index the segment/rect tests).
3. Score SPL from the GEODESIC on generated worlds, accepting the documented
   ~4.5% overstatement, and say so.

`measure()` sidesteps it for now with `with_taut=False` by default and screens
difficulty on the geodesic. **That is a workaround inside the generator and it
does not help the scorer.**

### ⚠️ FINDING 2 — a latent bug in `core/geometry.py`, fixed

`geodesic_distance` read:

    grid = grid or occupancy_grid()
    field = geodesic_field(goal_xy, grid)

**dropping `world` on both lines**, so `geodesic_distance(a, b, world=W)` with no
explicit grid answered about **lab42** rather than `W`. Fixed 2026-09-13;
unsolvable generated worlds went 32/48 -> 0/48.

No committed result is affected: every other caller uses the lab42 default, and
`core/scenario.py:477` is correct today precisely because every current scenario
IS lab42. ⚠️ **Chunk 46 changes that.** `core.scenario.validate()` will need a
world argument the moment a scenario lives on a generated world, and until it
has one it will validate generated scenarios against lab42 and silently pass or
fail them for the wrong reason.

### What the tiers do and do not mean

Density and tightness order strictly. **Tortuosity does not** — the means differ
but the distributions overlap badly (cluttered reaches 2.90, maze descends to
1.23), and tuning the `detour` parameter moved cluttered only 1.73 -> 1.87. The
test asserts the ordering that holds and explicitly declines to assert the one
that does not. Read the tiers as "how cluttered and how tight", not "how far
around".


## ⚠️ Difficulty is REDEFINED, 2026-09-13 — the base fails, or the world is too easy

The user's design, recorded in full in `plans/done/47-strongest-base-node.md`:

> The base node is maximised; the ENVIRONMENT is the free variable. If the base succeeds
> 100% of the time, that does not mean the base is too strong — **it means the environment is
> too easy.**

**Difficulty is the base node's failure rate.** Chunk 45's geometry statistics — tortuosity,
free fraction, bottleneck radius, obstacle count — are kept as cheap sim-free diagnostics but
are now **proxies**. Where a proxy and the base's failure rate disagree, the proxy is wrong.

Consequences for this epic:

- Chunk 45's "the `open` tier may be too easy" is no longer a judgement call; it is a
  measurement waiting to be taken.
- The tiers are **calibrated against the base**, not designed against geometry. Hardening a
  tier is the expected outcome of a calibration round, not a failure of chunk 45.
- ⚠️ **Adversarial selection is now a declared threat**: the suite is tuned until the base
  fails. Absolute success rates on it may never be reported as "how well the system
  navigates". And the tuning target is the **shared base only** — never any arm's nodes.
- The provisional chunk table's 47 (geometry-induced localisation difficulty) becomes **51**;
  47 is the base node, because chunk 44's growth loop waits on it.

### Finding 1 has a cheap resolution: PRECOMPUTE THE DENOMINATOR

Confirmed 2026-09-13 that `core.scenario.validate()` calls `optimal_distance()` ->
`shortest_path_length`, so the 43 s cost lands on **every generated scenario at validation
time**, not only at scoring time. A 100-scenario suite would spend ~72 minutes validating.

But the suite is **frozen**. So the SPL denominator is a constant per scenario, and the fix is
to compute it **once, at suite-freeze time, and commit it as data** — the scenario carries its
own `optimal_distance` instead of recomputing a fixed number on every score.

That is better than the three options listed above on every axis: `ARC_STEP_DEG` stays at 2
degrees so the denominator keeps its documented accuracy, no visibility-graph optimisation is
needed, SPL stays exactly what `PRE-REGISTRATION.md` says it is, and the value becomes
auditable because it is committed rather than recomputed. The 72 minutes is paid once, offline.

⚠️ It does mean a committed suite carries a number that must be re-derived if
`core.geometry.shortest_path` ever changes — the same discipline `core/preregistration.py`
already applies to frozen constants, and it should be checked the same way.

**Calibration (chunk 47) does not need it at all**: the base's failure rate is `goal_reached`,
which needs no denominator. Chunk 47 therefore skips `validate()` and `score()`'s SPL path
deliberately, and says so.


### ⚠️ Chunk 45 shipped a reproducibility bug, found 2026-09-13

`generate(tier, seed)` seeded on `abs(hash(tier))`. Python randomises str hashing per process,
so **every process drew a different world for the same (tier, seed)** — 21, 24, 31 rects on
three consecutive runs. The module's central claim, and chunk 45's "reproducible" Checks row,
were both false.

Caught only because the same base scored 40%, 30% and 25% on three runs of the same tier.
Fixed with `zlib.crc32`. The lesson for the rest of this epic: **a reproducibility test that
runs in one process proves nothing about reproducibility.** Anything this epic freezes — the
suite, the split, a precomputed SPL denominator — has to be checked across interpreters.
