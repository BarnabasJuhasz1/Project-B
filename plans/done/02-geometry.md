# 02 — Geometry: grid, raycast, clearance, geodesic

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 01 · **Needs Isaac Sim:** no

## Why

Three consumers need to reason about the walls without the simulator: the range sensor that
feeds `Observation.ranges`, the clearance signal behind the collision and near-miss metrics, and
the geodesic shortest path that is the denominator of SPL. All three are exact functions of
`core.scene_spec.WALLS`, so none of them needs a physics query or a GPU sensor.

## Build

`core/geometry.py`, constants above a data divider:

```
GRID_RESOLUTION = 0.05     # m per cell → 240 x 160 over the lab42 bounds
ROBOT_RADIUS    = 0.30     # m, inflation radius for the occupancy grid
```

- `raycast(origin_xy, yaw, angles, max_range)` → ranges array. Analytic ray/rectangle
  intersection against `wall_rects()`; `inf` where nothing is hit inside `max_range`.
- `clearance(xy)` → distance to the nearest wall surface, `0.0` inside a wall.
- `occupancy_grid(resolution=GRID_RESOLUTION, inflate=ROBOT_RADIUS)` → bool array plus origin
  and resolution.
- `geodesic_field(goal_xy, grid)` → float array, Dijkstra from the goal, 8-connected with
  correct diagonal cost, `inf` where unreachable.
- `geodesic_distance(start_xy, goal_xy)` → float, cached per goal.

## Done when

`core/tests/test_geometry.py` passes, covering at least:

- Ray from `(6.0, 4.0)` facing `-y` hits the south perimeter at **3.925 m**
  (wall centre y=0, half-thickness 0.075).
- Ray from `(3.0, 1.5)` facing `+x` hits the divider at **2.925 m** (divider face at x=5.925).
- Ray facing `+y` from mid-room with `max_range` short of the wall returns `inf`.
- `geodesic_distance((1.5, 1.5), (10.5, 6.5))` exceeds the Euclidean **10.30 m** — the divider
  blocks the straight line — and lands under 15 m. It should route via the doorway near
  `(6, 6.5)`, roughly 11–12 m.
- A point inside a wall has `clearance == 0.0`.

## Outcome

Done 2026-08-22. `core/geometry.py` + `core/tests/test_geometry.py`. Suite: **20 passed in 0.74 s**,
no simulator.

Measured values:

| | |
|---|---|
| grid | 160 × 240 cells, 29 145 free of 38 400, built in 9 ms |
| `geodesic_distance((1.5,1.5), (10.5,6.5))` | **11.071 m** vs Euclidean 10.296 m, computed in 89 ms |
| sealed room `(9.0, 2.5)` → goal | `inf` |
| goal inside a wall | field is all-`inf` |

### Differed from the plan

- **The chunk's first test point was wrong.** It said *"ray from `(6.0, 4.0)` facing −y hits the
  south perimeter at 3.925 m"*. The arithmetic is right but `(6.0, 4.0)` is **inside the divider** —
  the divider spans y ∈ [0, 5] at x = 6, so that origin is buried in geometry and correctly returns
  `0.0`. Moved the origin to `(3.0, 4.0)` in the west room, which gives the intended 3.925 m, and
  kept `(6.0, 4.0)` as a test of the buried-sensor case instead. Same family of mistake as the
  sealed room: reading `WALLS` as centrelines and forgetting the divider occupies the doorway
  column all the way up to y = 5.
- **`raycast` returns `inf`, never `max_range`, past the sensor limit.** So a controller can
  distinguish "nothing out there" from "a wall exactly at the limit". `LimitSpeedToVisibleRange`
  turns on precisely that difference, so collapsing them would have quietly removed the signal the
  blind corner exists to test.
- **Diagonal moves may not cut between two occupied cells.** Without that check the Dijkstra field
  leaks diagonally through the 0.15 m walls. `test_sealed_room_is_unreachable` is the guard.
- **Grid and field are both cached**, keyed on `(resolution, inflate)` and on the goal. Every
  scenario on this floorplan wants the same grid, and the field is the SPL denominator for every
  episode of a scenario.
- `Grid` is a small `__slots__` class rather than a bare tuple, so `world_to_cell` / `cell_to_world`
  / `is_free` live next to the array they index.

### Discovered

- **`ROBOT_RADIUS = 0.30` leaves the doorway comfortably passable.** Inflated, the divider occupies
  x ∈ [5.625, 6.375] up to y = 5.3 and the blind-corner wall occupies y ∈ [4.625, 5.375]; the gap to
  the north perimeter is still ~2.3 m wide. No risk of the geodesic declaring the only route closed.
- The geodesic is **not** bitwise symmetric — the goal cell seeds the field, so there-and-back
  quantise differently. They agree to within ~2 cells, which is what the test asserts. Worth knowing
  before anyone compares `doorway_traverse` against `return_home` SPL numbers directly.
