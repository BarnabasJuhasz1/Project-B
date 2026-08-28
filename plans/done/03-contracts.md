# 03 — Contracts: Observation, Command, Controller, Scenario

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 02 · **Needs Isaac Sim:** no

## Why

This is the interface every benchmark condition gets written against, so it is worth fixing
before anything is written against it. See the epic's *controller contract* section for the two
properties that matter: ground truth never enters `Observation`, and `reset()` exists so episode
state lives on the instance rather than in a mutable default (`drive_go2.py:135`).

## Build

- `core/observation.py` — `Observation`, `Command`, `Controller` protocol, exactly as specified
  in the epic. Frozen dataclasses.
- `core/scenario.py` — `Scenario` dataclass:

  | field | note |
  |---|---|
  | `id`, `family`, `split` | `split` is `"dev"` or `"held_out"` |
  | `scene` | USD path relative to the repo root |
  | `start` | `(x, y, yaw)` · `goal` | `(x, y)` |
  | `time_limit`, `seed` | |
  | `n_beams`, `fov`, `range_max` | sensor config — **identical across all four conditions**, so it belongs to the scenario, not the controller |

  Plus `DEV_SCENARIOS` holding the three from the epic (`doorway_traverse`, `blind_corner`,
  `return_home`) and `load(scenario_id)`, which validates and raises rather than returning a
  silently broken scenario:
  - start and goal inside `bounds()`
  - both in free space on the inflated grid
  - `geodesic_distance(start, goal)` finite

## Heads-up: lab42 has a sealed room

Worth confirming before authoring more scenarios. From `WALLS`, the divider spans y ∈ [0, 5] at
x = 6 and the blind-corner wall spans x ∈ [6, 12] at y = 5. Together with the south and east
perimeters that fully encloses **x ∈ [6.075, 11.925], y ∈ [0.075, 4.925]** — roughly 29 m² of the
96 m² floorplan, with no door. The comment on that wall says it *"creates a blind corner at
(6, 5)"*, which it does, but it also seals the room behind it.

That may well be intentional dead space. Either way it is the natural unreachable-goal test case,
so use a point inside it for the validation test.

## Done when

- All three dev scenarios `load()` without raising.
- A scenario with its goal inside a wall raises.
- A scenario with its goal at `(9.0, 2.5)` — inside the sealed room — raises with a message that
  says unreachable, not just invalid.

## Outcome

Done 2026-08-22. `core/observation.py`, `core/scenario.py`, `core/tests/test_contracts.py`.
Suite: **39 passed in 0.93 s** (whole `core/tests/`).

Dev set, validated:

| id | start → goal | geodesic | limit | optimal at 0.8 m/s |
|---|---|---|---|---|
| `doorway_traverse` | (1.5, 1.5, 0°) → (10.5, 6.5) | 11.071 m | 120 s | ~13.8 s |
| `blind_corner` | (5.0, 3.0, 90°) → (8.0, 6.0) | 5.151 m | 45 s | ~6.4 s |
| `return_home` | (10.5, 6.5, 180°) → (1.5, 1.5) | 11.071 m | 120 s | ~13.8 s |

### The sealed room is confirmed

`geodesic_distance((9.0, 2.5), (10.5, 6.5))` is `inf`, and `validate()` raises with **UNREACHABLE**
in the message plus the exact extent of the sealed region. A bare "invalid" would send someone
hunting for a typo in coordinates that are fine.

Still worth a decision: **is that 29 m² of dead space intentional?** Nothing here depends on the
answer — the doorway route works either way — but if a door was meant to be there, `WALLS` needs
one wall shortened and the scene rebuilt, and every geodesic above changes.

### Differed from the plan

- **`blind_corner` was re-sited.** The chunk said "just west of (6, 5) → east of it". Placed
  literally that would put the start inside the divider, which occupies x ∈ [5.625, 6.375] inflated,
  all the way to y = 5. It is now (5.0, 3.0) facing north → (8.0, 6.0): the route runs up the west
  face of the divider and rounds the corner, so what lies east is occluded until the robot is in the
  doorway. That is the trade-off the wall exists to create.
- **Sensor and Twist limits moved onto `Scenario`** (`n_beams=181`, `fov=2π`, `range_max=6.0`,
  `max_lin=0.8`, `max_ang=1.0`) with a test asserting they are identical across all scenarios.
  A condition must not be able to win by asking for a better sensor. These are pre-registration
  material and are commented as such.
- **`beam_angles()` drops the duplicate endpoint on a full circle.** `linspace(-π, π, 181)` samples
  the same bearing twice, which would double-weight straight-behind in any beam average.
- **`clamp()` is applied by the runner, not the controller.** A generated node returning 40 m/s
  should be scored on what the robot actually did, and every condition must saturate identically.
- Added `test_dev_scenarios_are_solvable_within_the_time_limit` — asserts optimal traverse time is
  under a third of the limit, so a scenario cannot silently degrade into a measurement of the speed
  cap.

### Discovered

`typing.Protocol` exposes `__protocol_attrs__`, so the requirement that a controller has `reset()`
is itself testable rather than merely documented. That is the structural fix for the
`drive_go2.py:135` mutable-default bug; chunk 05 adds the behavioural half.
