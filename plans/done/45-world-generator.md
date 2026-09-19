# 45 — The seeded world generator

**Epic:** [05](../05-epic-environment-suite.md) · **Depends on:** — · **Needs Isaac Sim:** no
**Status: DONE 2026-09-13.** SIM-FREE.

---

## What it is

`core/worldgen.py`. `generate(tier, seed)` -> a `GeneratedWorld`: a
`core.geometry.World`, a start, a goal, and a measured `Difficulty`. Three tiers,
reproducible from `(tier, seed)` alone, ~250 ms each.

`scripts/plot_worlds.py` draws a contact sheet, because a difficulty number
cannot tell you a "maze" is really one room with furniture.

## Design decisions worth keeping

- **Recursive partition, not random walls.** Splitting the largest cell and
  leaving one doorway reproduces the motif that makes buildings hard: rooms
  joined by single apertures, so a greedy controller meets a wall and must
  commit to a doorway leading away from the goal. Random walls make clutter,
  which is an easier and different problem.
- **Difficulty is measured by `core.geometry`**, the same code that scores the
  benchmark, so a world cannot be hard by assertion.
- **Symmetry is the localisation stressor** and it is geometric. Two halves that
  scan alike create genuine pose ambiguity without touching `core/drift.py`, so
  it inherits none of epic 03's ordering constraint.
- **`WorldgenError` rather than a fallback.** Substituting an easier room when a
  tier fails would bias the distribution toward exactly the rooms the suite
  exists to avoid.

## Outcome

Built as planned. Two findings escaped the chunk and are recorded in the epic:
`shortest_path_length` is quadratic and costs 43 s on a generated world (SPL is
a named secondary), and `core/geometry.geodesic_distance` was silently dropping
its `world` argument.

### Checks

| check | outcome | verdict |
|---|---|---|
| Reproducibility: same `(tier, seed)` -> same rects/start/goal | identical, all 3 tiers | ⚠️ **WRONG — same-process only** |
| Reproducibility ACROSS PROCESSES | 21/24/31 rects on three runs of the same call | ⚠️ **FAILED, fixed 2026-09-13** (`abs(hash(tier))` -> `zlib.crc32`) |
| Distinct seeds give distinct worlds | 12/12 distinct per tier | confirmed |
| Solvable by `geodesic_distance`, the function `scenario.validate` uses | 48/48 after the geometry fix | confirmed |
| Endpoints clear of walls and >= 6 m apart | all, 3 tiers x 12 seeds | confirmed |
| Bottleneck >= ROBOT_RADIUS (robot physically fits) | all | confirmed |
| Geometry inside ROOM bounds, non-degenerate rects | all | confirmed |
| Tier ordering on free / bottleneck / obstacles | strict, small variance, n=25 | confirmed |
| Tier ordering on **tortuosity** | **NOT reliable** — see below | **ruled out** |
| `min_corridor` as min clearance over free cells | always ~0.30 m regardless of layout | ruled out — measured the definition of "free" |
| `min_corridor` as min clearance along the taut path | always ~0.30 m | ruled out — `shortest_path` hugs the inflated boundary by construction |
| 8-connected labelling as the cause of unsolvable worlds | wrong cause; it was the `world` bug | ruled out |
| Full sim-free suite after the `core/geometry.py` fix | 868 passed, 1 pre-existing failure (`region` leak) | confirmed |
| `preregistration.verify()` | OK, digest intact | confirmed |

⚠️ **Tortuosity does not separate `cluttered` from `maze`.** The means differ
(1.78+-0.44 vs 2.10+-0.43, n=25) but the distributions overlap badly — cluttered
reaches 2.90, maze descends to 1.23 — and tuning `detour` over 2/3/4 moved
cluttered only 1.73 -> 1.87. The test asserts the ordering that holds and
declines to assert this one. **Read the tiers as "how cluttered and how tight",
not "how far around".**

⚠️ **The min_corridor story is worth reading before adding any geometry metric.**
Two natural definitions both returned ~0.30 m for every tier, because both were
measuring `ROBOT_RADIUS` — the first is how `occupancy_grid` defines a free cell,
the second is where `shortest_path` puts a taut path. The bottleneck (largest
inflation radius at which start and goal stay connected) is the version that
carries information.

### Not checked

Nothing was run under physics; no generated world has been through Isaac Sim or
`build_scene.py`, and no USD exists for one. No scenario has been built on a
generated world — that is chunk 46, which also has to give
`core.scenario.validate()` a world argument.
