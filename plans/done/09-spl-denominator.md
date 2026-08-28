# 09 — The SPL denominator saturates

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 02, 04 · **Needs Isaac Sim:** no

Follow-up to [04](../done/04-metrics.md), found after the epic closed while reading the committed
result files. Sim-free: everything here is `core/geometry.py`, `core/metrics.py` and pytest.

## Why

`results/doorway_traverse__reactive_avoid__seed1001.json` records a path **shorter than the route
it is scored against**:

```
path_length        10.6314
optimal_distance   11.0711      = 4 + 5·√2 exactly
spl                1.0
```

The controller did not beat the optimum. The denominator carries about **1.0 m of slack** from two
independent biases, each worth roughly half a metre on this scenario.

**Bias 1 — the numerator and the denominator measure different journeys.** `score()` terminates the
episode at the first step within `GOAL_RADIUS = 0.5` of the goal and sums `path_length` only to
that index (`end = goal_i`). But `optimal_distance()` is `geodesic_distance(start, goal)` — start to
the goal *centre*. The last half-metre is in the denominator and cannot be in the numerator.
Shortest grid path from start to anywhere on the 0.5 m ring is **10.5089**, not 11.0711.

**Bias 2 — 8-connected Dijkstra cannot travel at arbitrary angles.** Movement is restricted to
multiples of 45°, so the octile path overshoots the taut-string path. The true continuous shortest
legal path here rounds the inflated corner at (6.076, 5.375) and measures **10.5611**; the grid
returns 11.0711. That is **+0.51 m, 4.8% over**, and it does *not* shrink with resolution:

```
res=0.05     11.0711
res=0.025    11.0711
res=0.0125   11.0711
```

Digitisation bias is inherent to the connectivity, not to the cell size. Refining the grid is not a
fix. `core/geometry.py:238` already says the geodesic "overestimates a true continuous shortest path
by a few percent on diagonal runs"; this chunk measures how much and decides whether to accept it.

Stacking both corrections, the distance actually required is ≈ **10.06 m** (10.5611 continuous, less
the 0.5 m never walked). The robot travelled 10.6314 — about **5.7% above optimal**, a sensible
gap-follower result. The `1.0` is an artefact.

### Why this is not cosmetic

`core/metrics.py:206` clamps the ratio:

```python
spl = (optimal / max(path_length, optimal)) if goal_reached and path_length > 0 else 0.0
```

The `max()` is why the file reports `1.0` and not `1.04`. Combined with ~9% of slack in the
denominator, **every controller within ~9% of true optimal scores exactly 1.0.** SPL is on G2's
secondary metric list. A metric that pins at its ceiling for every competent controller cannot
separate condition B from D, which is the comparison the description says matters most. The
discriminating range has been compressed away before any evaluation run.

The bias is identical across conditions, so it does not favour anyone — it is a loss of resolution,
not a confound. That is why this is worth fixing but is not urgent enough to touch mid-evaluation.

## Decide first

**This is pre-registration material and changing it moves a published number.** `optimal_distance`
appears in all six committed result files. Do not start until the user has picked an option; record
the choice in the Outcome and in the epic's metrics table, which currently defines `spl` as
`success * d_geodesic / max(d_travelled, d_geodesic)` with `d_geodesic` specified as 8-connected.

| | Option | Removes | Cost |
|---|---|---|---|
| **A** | Measure the denominator to the goal boundary: geodesic to the nearest cell within `GOAL_RADIUS` | Bias 1 | ~10 lines, one cached field query |
| **B** | A, plus replace octile Dijkstra with a visibility graph over the inflated rectangles | Both | A day; lab42 is six axis-aligned boxes, so the graph is small and exact |
| **C** | Accept both biases, document the ceiling, drop SPL from the reported secondary metrics | — | Free, but loses the path-efficiency metric G2 asks for |

Recommendation: **B**. It is the only option that makes `spl == 1.0` mean "optimal" rather than
"within 9% of optimal", and lab42's geometry is exactly the case a visibility graph handles exactly.
A alone leaves the 4.8% octile ceiling in place, which is still enough to saturate a good controller.

Do not ship a partial B. An exact denominator with a boundary-mismatched numerator is worse than
either bias alone, because the error stops being conservative in a known direction.

## Build

Under option B:

- `core/geometry.py`
  - `shortest_path_length(start_xy, goal_xy, clearance_radius=ROBOT_RADIUS)` — visibility graph over
    the corners of the `ROBOT_RADIUS`-inflated wall rectangles plus start and goal, edges where the
    segment is collision-free, Dijkstra over that. Exact for axis-aligned boxes; corners of the
    inflated region are arcs, so use the rectangle corner offset by the radius on both axes and
    accept the ≤ 0.09 m corner-cutting error, or model the arcs — say which in the Outcome.
  - Keep `geodesic_distance()` and the occupancy grid. `raycast`, `clearance` and the reachability
    validity check in `scenario.validate()` all depend on the grid, and the grid is what proves a
    scenario is solvable. Only the SPL denominator moves.
- `core/scenario.py`
  - `optimal_distance()` returns the shortest legal path from start to the **goal-reach boundary**,
    i.e. the minimum over the `GOAL_RADIUS` ring, not to the goal centre. This imports `GOAL_RADIUS`
    from `core.metrics` — if that import direction is wrong for the package, pass it in as an
    argument from `score()` and leave the scenario ignorant of the metric.
  - Keep the unreachable-goal `ValueError`. It must still fire on the sealed room.
- `core/metrics.py`
  - Leave the `max()` clamp in. With an exact denominator it should never bind, so make that
    testable: emit `spl_uncapped` alongside `spl`, or assert in tests that the clamp is unused on
    a legal path. A clamp that silently starts binding again is how this bug returns.
  - Update the pre-registration note at the top of the module with the new definition.
- Re-score the six files in `results/` from their traces if the traces still exist; otherwise
  re-run them and say in the Outcome that the numbers were regenerated, not re-scored.

## Done when

- `core/tests/test_geometry.py`
  - `shortest_path_length((1.5,1.5),(10.5,6.5))` ≈ **10.5611** (the tangent-point path via
    (6.076, 5.375)), within 1 cm.
  - It is **≤** `geodesic_distance` for the same endpoints on all three dev scenarios — the exact
    path can never be longer than the grid path.
  - Straight-line case with no obstacle between: equals the Euclidean distance exactly. The octile
    version fails this for any angle that is not a multiple of 45°, which is the regression test for
    the whole chunk.
  - Unreachable goal (a point inside the sealed room at x ∈ [6.075, 11.925], y ∈ [0.075, 4.925])
    still returns `inf`, and `scenario.validate()` still raises.
- `core/tests/test_metrics.py`
  - A synthetic trace following the exact shortest path and stopping at the goal boundary scores
    `spl == 1.0` **without the clamp binding**.
  - A trace 20% longer scores `spl` ≈ 0.83, i.e. the metric now discriminates in the band where it
    previously saturated.
  - Existing metric tests still pass unchanged where they do not assert on `optimal_distance`.
- Whole `core/tests/` suite green, still no simulator, still under ~15 s.
- `results/README.md` and the epic's metrics table state the frozen definition of `d_geodesic`.
- `results/*.json` regenerated, and the Outcome records old and new `optimal_distance` and `spl`
  side by side for all six, so the change to a published number is auditable.

## Not in scope

Re-running anything in the container, and the `FALL_HEIGHT` / `contact` items in
`results/README.md` — those are separate pre-registration decisions. Note in passing that
[04](../done/04-metrics.md)'s frozen-values block still lists `FALL_HEIGHT = 0.15`, superseded by
0.08 in [07](../done/07-runner.md); the epic's metrics table says 0.15 too. Both want fixing when
the constants are frozen for real, but not here.

## Outcome

Done 2026-08-24. **Option B**, in full: the denominator is now the taut continuous path, measured
to the goal-reach boundary. `core/geometry.py`, `core/scenario.py`, `core/metrics.py`,
`scripts/rescore.py`; **114 tests, 12 s**, still sim-free, still no GPU (was 109 after 08; 21 added
here, and none of the existing ones needed changing).

### The number in the plan body was wrong

The plan quoted the true continuous path as **10.5611 m** via a tangent point at (6.076, 5.375).
That path is not walkable: it clears the divider's inflated corner but the run-in from the start
cuts the divider's inflation zone, reaching a minimum clearance of **0.1798 m** against a required
0.30. It was a lower bound found by minimising over one circle without checking the segments, so
the plan's headline "4.8%, +0.51 m" was slightly overstated.

The real taut path is **10.5942 m**, and it wraps the divider's northwest corner arc rather than
touching it at a point — the polyline hugs the arc from (5.799, 5.298) round to (5.927, 5.366)
before the straight run to the goal. Corrected figures:

```
octile grid geodesic          11.0711
taut continuous path          10.5942     octile overstates by 0.4769 m = 4.5%
less GOAL_RADIUS              10.0942     <- optimal_distance now
robot's actual path           10.6314     (doorway_traverse / reactive_avoid)
```

So the controller was **5.3% above optimal**, and the denominator had been carrying 0.9769 m of
slack in 10.09 — 9.7%, which is the saturation band the plan predicted.

### How the answer was made trustworthy

An exact-looking number nobody can check is the failure mode this chunk was most exposed to, so
three independent confirmations, all kept as tests:

- **Convergence.** Refining the arc sampling 40× (10° → 0.25°) moves the answer monotonically down
  from 10.594598 to 10.594232 and stops. At the 2° default it is 1.4e-5 m above the limit.
- **An independent optimiser.** scipy SLSQP over an 80-waypoint polyline with 12 clearance samples
  per segment converges to 10.594350 while still violating clearance by 4.5e-4 m — two unrelated
  methods agreeing to **1.2e-4 m**, with the discrepancy in the direction the residual violation
  predicts. scipy is used only to check the answer, never at run time; `core/` gained no dependency.
- **The path is walkable.** `shortest_path()` returns the polyline, and a test walks every segment
  and asserts it clears `ROBOT_RADIUS` along its whole length. It comes out at exactly 0.300000.

### Differed from the plan

- **Arcs, not square corners.** The plan left the choice open. Square corners would have kept a
  bias of up to 0.43·R = 0.13 m per rounded corner, which is the same failure in miniature. The
  arcs are sampled at `ARC_STEP_DEG = 2.0`, with the samples placed at radius `R / cos(step/2)`
  rather than `R` — that puts the **chords** on the true inflated boundary instead of inside it,
  so every graph edge is a path the robot could actually walk and the length errs upward by
  4.6e-5 m per corner. Correct by construction rather than by tolerance.
- **`segment_clearance()` is analytic, not sampled.** A segment and an axis-aligned box are both
  convex, so when disjoint the closest pair always involves a vertex of one and a point on the
  other; checking the two endpoints against the box and the box's four corners against the segment
  is exhaustive. Sampling a segment would have missed thin obstacles, and the divider is 0.15 m
  thick. Cross-checked against 4001-point sampling on 300 random segments.
- **56 graph nodes, not thousands.** Only the outward quarter of each wall corner is sampled, and
  points buried in another inflated wall or outside the room are dropped. First call 0.03 s, cached
  after. This is why the suite did not get slower.
- **`validate()` keeps using the grid for reachability.** The plan implied `optimal_distance()`
  would carry it. It should not: the grid field is exhaustive over the room, so "unreachable" is a
  statement about the floorplan, whereas the visibility graph answers one query and a bug in it
  would masquerade as an unreachable goal. `validate()` now runs the grid check first and raises a
  *different*, explicitly "this is a bug in core.geometry" error if the two ever disagree.
- **A new guard in `validate()`.** Subtracting `GOAL_RADIUS` from a start→goal path equals the
  distance to the goal *disc* only while that disc is entirely walkable. All three dev scenarios
  satisfy it with room to spare, but a held-out scenario with a goal tucked into a corner would
  not, and the denominator would then be silently short. `validate()` now rejects a goal within
  `ROBOT_RADIUS + GOAL_RADIUS` of a wall, and a test confirms the subtraction agrees with measuring
  to 181 sampled points on the disc.
- **`scripts/rescore.py` was not in the plan.** The plan said to re-score the six committed files;
  doing that from an interactive session would have violated the working agreement that anything
  reported comes from a committed script. It re-scores from `trace.csv` with no simulator, and
  stamps `rescored_utc` / `rescored_git_commit` / `rescored_from` into the manifest, because after
  a re-score the numbers no longer come from the `git_commit` that flew the robot.
- **The clamp was kept and made visible.** `spl = min(1, spl_uncapped)`, with `spl_uncapped`
  reported. It should now never bind, so `spl_uncapped > 1` became a fault signal — the robot cut
  a corner no legal path could — rather than the routine occurrence it used to be.

### The published numbers, before and after

All six re-scored from their own traces. `optimal_distance` for `doorway_traverse` and
`return_home` is now bit-identical: the taut path is direction-free, where the grid seeded its
field at the goal cell and quantised the two directions differently.

| result | `optimal_distance` | | `spl` | |
|---|---|---|---|---|
| | old | new | old | new |
| `doorway_traverse` / `reactive_avoid` | 11.0711 | 10.0942 | **1.0** | **0.9495** |
| `return_home` / `reactive_avoid` | 11.0711 | 10.0942 | **1.0** | **0.9853** |
| `blind_corner` / `reactive_avoid` | 5.1506 | 4.2654 | **1.0** | **0.9909** |
| `doorway_traverse` / `goal_seeker` | 11.0711 | 10.0942 | 0.0 | 0.0 |
| `return_home` / `goal_seeker` | 11.0711 | 10.0942 | 0.0 | 0.0 |
| `blind_corner` / `goal_seeker` | 5.1506 | 4.2654 | 0.0 | 0.0 |

Three identical 1.0s became three distinguishable scores. The response curve, on synthetic traces
that walk the taut path with a controlled detour:

```
inefficiency   +0%     +5%     +10%    +20%    +50%
old spl        1.000   1.000   0.995   0.909   0.721
new spl        0.999   0.952   0.907   0.828   0.657
```

The old denominator was flat until about +9%, which is the saturation this chunk existed to remove.

### Found on the way, not fixed here

- **Event counts sit on a floating-point knife edge.** Re-scoring from `trace.csv`, whose
  timestamps differ from the in-memory ones by 6.7e-12 s, moves `stuck_events` from 2 to 1 and
  `time_stuck` from 78.76 s to 111.36 s on `doorway_traverse__goal_seeker__seed1001`.
  `EVENT_RELEASE = 0.5` s at a 0.02 s step is exactly 25 steps and `_events()` compares
  `>= release` with no tolerance, so accumulated float error decides whether a span closes. This is
  pre-existing and nothing to do with the denominator; 09 only supplied the perturbation that
  exposed it. The stuck figures in the committed files are the post-re-score side of that coin.
  Written up as [10](10-event-debounce-tolerance.md), and it must be settled before any
  event count is reported.
- **The optimal path grazes.** The taut path runs at exactly `ROBOT_RADIUS`, so the trace
  `clearance` column along it would be 0.0, which is `<= COLLISION_CLEARANCE = 0.05` — a perfectly
  optimal robot scores a collision. That is arguably correct for a circumscribing-circle proxy
  around a 0.65 × 0.31 m body, but it is another reason the `contact` question in
  `results/README.md` needs settling before collisions are reported.
- `plans/done/04-metrics.md` and the epic's metrics table both still list `FALL_HEIGHT = 0.15`,
  superseded by 0.08 in [07](07-runner.md). The epic's SPL row is updated here; the `FALL_HEIGHT`
  rows are left for whoever freezes the constants.

### Still open

The frozen definition now reads: **`d_optimal` = taut continuous shortest path from start to the
`GOAL_RADIUS` disc, over obstacles inflated by `ROBOT_RADIUS`.** That must be written down with
G2's threshold X, the seed set, the episode count and the secondary metric list before the first
evaluation run. None of those are fixed yet.
