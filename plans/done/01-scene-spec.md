# 01 — Extract the scene spec

**Epic:** [00 — reproducible scored episode](../00-epic-reproducible-episode.md)
**Depends on:** nothing · **Needs Isaac Sim:** no

## Why

`WALLS`, `GO2_START` and `GOAL` live in `scripts/build_scene.py:52-85`, which constructs
`SimulationApp` and calls `parse_args()` at module import (`build_scene.py:28,32`). Importing it
to read the floorplan boots a simulator and eats `argv`. Every later chunk needs that geometry,
so it has to come out first.

## Build

- `core/__init__.py`, `core/tests/__init__.py`
- `core/scene_spec.py` — move the constants verbatim from `build_scene.py:52-85`, keeping the
  `# ... is data, not code` divider comment. Add two derived helpers:
  - `wall_rects()` → each `(cx, cy, length, yaw)` expanded to a 2D rectangle using
    `WALL_THICKNESS`, in a form the geometry chunk can consume.
  - `bounds()` → `(min_x, min_y, max_x, max_y)` computed from `WALLS`, not hardcoded.
    `recorder.py:82` currently hardcodes `FALLBACK_SCENE_BOUNDS = (0.0, 0.0, 12.0, 8.0)`;
    leave `recorder.py` alone but note the duplication in a comment.
- `scripts/build_scene.py` — import from `core.scene_spec` instead of defining the constants.
  `scripts/` is one level under the root, so derive the repo root the way `recorder.py:54` does
  and put it on `sys.path`. The import is sim-free, so it may sit above the `SimulationApp` line.
- `core/tests/test_import_guard.py` — import every `core` module in a subprocess and assert
  `isaacsim`, `omni` and `torch` are absent from `sys.modules` afterwards. This is the check that
  keeps the sim-free half sim-free as the package grows.

## Done when

- `~/miniforge3/envs/env_isaacsim/bin/python -m pytest core/tests/ -q` is green.
- `python scripts/build_scene.py --out /tmp/check.usd` still runs and logs the same six walls.
- `python -c "from core.scene_spec import bounds; print(bounds())"` prints `(0.0, 0.0, 12.0, 8.0)`
  in well under a second, with no simulator banner.

## Outcome

Done 2026-08-22. `core/scene_spec.py` holds the floorplan; `scripts/build_scene.py` imports it.

Checks:

- `pytest core/tests/ -q` → 2 passed in 0.13 s.
- `python -c "from core.scene_spec import bounds; print(bounds())"` → `(0.0, 0.0, 12.0, 8.0)` in
  **26 ms**, no simulator banner.
- Rebuilt to a scratch USD and diffed the six `/World/Walls/wall_NN` prims against the committed
  `scenes/lab42_v1.usd`: translate, orient and scale identical on all six. The committed artefact
  is not stale with respect to the spec, at least for the walls.

### Differed from the plan

- **`wall_segments()` was added** alongside `wall_rects()` and `bounds()`. `bounds()` needs the
  wall *centrelines*, not the rects — the perimeter walls straddle the boundary, so rects give
  `x ∈ [-0.075, 12.075]` and the occupancy grid would carry a ring of cells outside the building.
  Centrelines give exactly `(0, 0, 12, 8)`, matching `recorder.py:82`'s `FALLBACK_SCENE_BOUNDS`.
- **Axis-alignment is now an enforced precondition.** `_along_x()` raises `ValueError` naming the
  offending wall if a yaw is not a multiple of 90°. Every lab42 wall is axis-aligned, and that is
  what lets chunk 02's raycast and grid be analytic and exact. Computing the rects with `cos`/`sin`
  also leaked float error into `bounds()` (`-2.4e-16` instead of `0.0`); the quarter-turn branch is
  exact. Angled walls need oriented boxes throughout `core/geometry.py` — the error message says so.
- **`FORBIDDEN` in the import guard is wider than the chunk specified**: `pxr` and `carb` joined
  `isaacsim`, `omni`, `torch`. Both are simulator-side and both are importable without
  `SimulationApp`, so they are exactly the imports that would slip in unnoticed.
- The guard has a second test, `test_probe_actually_imported_something`. An empty module list would
  make the first test pass vacuously if `walk_packages` ever stopped finding anything.

### Discovered

**`pytest` was not installed in `env_isaacsim`.** It was resolving to a broken copy in
`~/.local/lib/python3.12/site-packages` (missing `pygments`), which shadows the conda env because
user site-packages precedes it on `sys.path`. Fixed with
`~/miniforge3/envs/env_isaacsim/bin/python -m pip install --ignore-installed pytest`
(pytest 9.1.1, pluggy, iniconfig, pygments; also pinned `packaging` back to 26.0, which is what
`isaacsim-core 6.0.1.0` requires — it had been at 26.3).

That user-site leak is worth remembering: **anything in `~/.local` shadows the conda env**, so a
local run can differ from the container for reasons that have nothing to do with the code. One more
argument for the containerised runs being the ones that produce reported numbers.
