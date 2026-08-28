# 04 — Metrics

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 02, 03 · **Needs Isaac Sim:** no

## Why

Scoring has to be a pure function of a recorded trace, separate from the run loop. That way the
metrics can be unit-tested against synthetic traces in milliseconds, re-scored after the fact when
a threshold changes, and reused unchanged by the Week 3 unit harness.

## Build

`core/metrics.py`:

- `EpisodeTrace` — per **control** step: `t`, true pose `(x, y, z, yaw)`, issued `Command`,
  `clearance`, `contact`, `pose_estimate`.
- Thresholds as named constants above a data divider. Values are proposed in the epic's metrics
  table; `FALL_HEIGHT = 0.15` is reused from `walk_forward.py:137`.
- `score(trace, scenario)` → dict with `goal_reached`, `time_to_goal`, `spl`, `collisions`,
  `near_misses`, `stuck_events`, `recovery_time`, `localisation_failures`, `path_length`,
  `termination` (`"goal"` / `"timeout"` / `"fell"`).

**These constants are pre-registration material.** They may be tuned this week; once evaluation
starts they must not move. Record the frozen values alongside G2's decision threshold.

## Done when

`core/tests/test_metrics.py` passes on synthetic traces:

- A clean run to the goal → `goal_reached`, `spl` close to 1, zero collisions.
- **One long scrape along a wall scores `collisions == 1`, not one per step.** This is the
  debouncing check and the easiest thing to get wrong.
- A trace that never reaches the goal → `goal_reached` false, `spl == 0`, `time_to_goal` null,
  `termination == "timeout"`.
- Commanded forward but stationary for longer than `STUCK_WINDOW`, then moving again → exactly
  one stuck event with a plausible `recovery_time`.
- `pose_estimate` drifting past `LOC_FAIL_THRESHOLD` and back → one localisation failure.
- Base below `FALL_HEIGHT` → `termination == "fell"`, goal not credited.

## Outcome

Done 2026-08-22. `core/metrics.py`, `core/tests/test_metrics.py`. **19 tests, 0.37 s**;
whole `core/tests/` suite now 58 passed in ~1 s.

### Frozen values — pre-registration material

```
GOAL_RADIUS          0.5    m     COLLISION_CLEARANCE  0.05  m
FALL_HEIGHT          0.15   m     NEAR_MISS_CLEARANCE  0.25  m
STUCK_WINDOW         2.0    s     EVENT_RELEASE        0.5   s
STUCK_DISPLACEMENT   0.1    m     LOC_FAIL_THRESHOLD   1.0   m
STUCK_COMMAND_EPS    0.05   m/s
```

These must be written down with G2's decision threshold X before the first evaluation run and must
not move afterwards. Tuning a threshold after seeing results turns an ablation into a search over
thresholds. The module says so at the top.

### Differed from the plan

- **`clearance` in a trace is a *gap*, not a centre distance.** `core.geometry.clearance()` returns
  distance from a point to the nearest wall; the trace stores that minus `ROBOT_RADIUS`, so it goes
  negative when the body overlaps geometry. Without this, `COLLISION_CLEARANCE = 0.05` would have
  meant the robot's *centre* 5 cm from a wall — i.e. 25 cm inside it — and the collision metric
  would essentially never have fired.
- **`EVENT_RELEASE = 0.5 s` hysteresis added.** Plain rising-edge counting handles a continuous
  scrape but not a controller oscillating across the threshold, which would score ~100 collisions
  for one wall-follow. `test_chatter_across_the_threshold_is_still_one_collision` pins it.
- **Near misses exclude collisions.** Counting the near band raw would report every collision as a
  near miss too; the two numbers would move together and carry no independent information. A near
  miss is now an approach whose closest point stayed above `COLLISION_CLEARANCE`.
- **Collision is `contact OR clearance ≤ threshold`.** The analytic clearance is a circumscribing
  circle proxy and is conservative in one direction; a physics contact report beats it.
- **Stuck requires commanded linear speed above `STUCK_COMMAND_EPS`.** Standing still on purpose,
  and turning in place, are not failures. Both have tests.
- **`recovery_time` is measured from detection, not from onset**, and lags by `STUCK_WINDOW` by
  construction — you cannot know the robot is stuck until it has failed to move for a while. Also
  reports `time_stuck` (total) alongside `recovery_time` (mean per event).
- Path length accumulates only up to termination, so wandering after arrival cannot depress SPL,
  and SPL is capped at 1 because the grid-quantised geodesic slightly overestimates a true
  continuous shortest path on diagonal runs.
- Extra fields in the result: `min_clearance`, `duration`, `control_steps`, `optimal_distance`,
  `path_length`, `time_stuck`. All cheap, all worth having when a run looks wrong.
