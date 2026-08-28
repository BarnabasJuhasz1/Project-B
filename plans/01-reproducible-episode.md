# 01 — The reproducible scored episode

**Status:** approved 2026-08-22, not yet implemented.
**Week:** 1. **Unblocks:** the Week 2 B-vs-D pilot.

## Context

Week 1 of the execution plan asks for four things. Three are done: the pinned Isaac Sim
container (6.0.1, pulled locally), the Go2 under its frozen locomotion policy
(`scripts/drive_go2.py`, `scripts/testing/walk_forward.py`), and a Lab 42-like floorplan
(`scripts/build_scene.py` → `scenes/lab42_v1.usd`). The fourth — **"one scripted episode
running headless and reproducibly"** — is not.

`drive_go2.py --demo waypoints --headless` is scripted and headless, but it is not an
*episode*: there is no seed, no termination condition, no metrics, no result file, and no
provenance record. Nothing downstream can be built on it. Week 2 needs "both conditions on
two or three scenarios at a fixed episode count"; Week 3's unit harness needs to construct
micro-worlds and score them; Week 5's four-condition ablation needs identical episode counts
and a pre-registered primary metric. All three reuse the same machinery, and none of it exists.

Four concrete obstacles in the current code:

1. **Nothing is importable.** All three scripts construct `SimulationApp` at module import
   (`build_scene.py:32`, `drive_go2.py:84`, `walk_forward.py:98`), so `WALLS`, `GOAL` and
   `GO2_START` are trapped inside a file that boots a simulator and parses `argv` on import.
   Metrics, shortest-path computation and scenario specs cannot depend on them.
2. **Episode state is module-level.** `demo_waypoints` keeps its route index in a mutable
   default argument (`drive_go2.py:135`). A second episode in the same process starts
   mid-route — fatal for a batch runner.
3. **Ground truth is the only pose source.** `base_pose()` reads the true articulation pose and
   hands it straight to the velocity source. If controllers are written against that, the
   localisation-failure metric and `IsLocalisationDegraded` can never be measured.
4. **No tests, no scoring, no provenance.** And `scenes/lab42_v1.usd` is an 18 MB binary crate
   tracked in git, already rewritten once by a rebuild, with no way to check it against the
   spec that produced it.

Outcome: `run_episode.py` runs one scenario with one controller headless, deterministically,
and writes a scored `result.json` plus a manifest naming everything needed to reproduce it.

## Design: sim-free core, thin sim entry points

The central move is splitting code that needs Isaac Sim from code that does not. The sim-free
half is importable in ~50 ms, unit-testable, and reusable by the Week 3 unit harness.

```
Repo/
  core/                       # sim-free. NO isaacsim/omni/torch imports, enforced by a test.
    scene_spec.py             # WALLS, GO2_START, GOAL, lighting — moved out of build_scene.py
    geometry.py               # wall boxes → occupancy grid, 2D raycast, clearance, geodesic
    observation.py            # Observation + Command + Controller protocol
    scenario.py               # Scenario dataclass + dev-set registry
    metrics.py                # EpisodeTrace → scored result. Thresholds as named constants.
    manifest.py               # provenance capture
    tests/                    # first tests in the repo; pytest, no simulator
  controllers/
    goal_seeker.py            # generic, no sensing. Expected to collide.
    reactive_avoid.py         # generic, uses ranges. Expected to reach the goal.
  scripts/
    go2_policy.py             # sim-side helpers, imported AFTER SimulationApp
    run_episode.py            # the new entry point
    build_scene.py            # modified: imports core.scene_spec
  results/                    # committed. Scored result.json per run, small text.
  out/runs/<run_id>/          # gitignored. trace.csv, manifest.json, optional mp4.
```

`core/` is a naming call — it holds the shared sim-free library. `harness/` is deliberately
left free for Week 3's unit-scenario harness, which will be built on top of `core/`.

### Reuse, do not re-derive

`drive_go2.py` and `walk_forward.py` already contain three fiddly things that work and are
currently duplicated between them. Lift them into `scripts/go2_policy.py` rather than writing
a third copy:

- the policy-class probe ladder (`drive_go2.py:89-105`) — in 6.0.1 it resolves to
  `isaacsim.robot.policy.examples.robots.go2.Go2FlatTerrainPolicy`
- `base_pose()` (`drive_go2.py:180-193`) — handles batched warp arrays from 6.0 articulations
- `make_command()` (`drive_go2.py:168-177`) — the preallocated torch tensor on `robot._device`

Plus the two things `walk_forward.py` gets right and `drive_go2.py` does not:
`World.clear_instance()` before construction, and deriving the control rate from `policy._dt` /
`policy._decimation` instead of hardcoding it.

Leave `drive_go2.py` and `walk_forward.py` themselves alone for now — they work, and CLAUDE.md
records them as smoke tests. Refactoring them onto `go2_policy.py` is a follow-up.

`--record` wires to the existing `recorder.Recorder` exactly as `drive_go2.py:264-273` does.
Off by default: benchmark runs must not pay for rendering.

## The controller contract

This is the interface every benchmark condition will be written against, so it is fixed now.

```python
@dataclass(frozen=True)
class Observation:
    t: float                    # sim seconds since episode start
    ranges: np.ndarray          # N-beam 2D scan, metres, inf where no return
    range_angles: np.ndarray    # beam bearings in the robot frame, radians
    range_max: float            # what "as far as I can see" means
    pose_estimate: tuple        # (x, y, yaw) — the robot's BELIEF
    pose_error_estimate: float  # scalar localisation-confidence proxy
    goal: tuple                 # (x, y), same frame as pose_estimate
    contact: bool

@dataclass(frozen=True)
class Command:
    vx: float; vy: float; wz: float     # the frozen Twist, unchanged

class Controller(Protocol):
    def reset(self, scenario: Scenario, rng: np.random.Generator) -> None: ...
    def step(self, obs: Observation) -> Command: ...
```

Two properties matter more than the field list:

- **Ground truth never enters `Observation`.** The runner holds the true pose separately and
  uses it only for scoring. `pose_estimate` is exactly the true pose today, so nothing behaves
  differently — but drift can be injected into that one field later and every controller,
  including `IsLocalisationDegraded`, becomes measurable with no rewrites.
- **`reset()` exists so episode state lives on the instance.** This is the fix for the
  `demo_waypoints` mutable-default bug, made structural rather than remembered.

`ranges` comes from an analytic 2D raycast against the wall boxes in `core/geometry.py` — the
walls are the only obstacles and their geometry is known exactly, so this is deterministic,
fast, needs no GPU sensor pipeline, and is swappable for an RTX lidar behind the same two
fields.

The controller is called once per **control** step (every `decimation` physics steps), not
every physics step, matching how the policy actually consumes commands.

## Metrics

`core/metrics.py`, all thresholds as named constants above a data divider, following the
convention in `build_scene.py`. **These are pre-registration material — proposed here, to be
frozen and written down before the first evaluation run.**

| Metric | Definition | Constant |
|---|---|---|
| `goal_reached` | true pose within radius of goal, base upright at that moment | `GOAL_RADIUS = 0.5` m, `FALL_HEIGHT = 0.15` m (reused from `walk_forward.py:137`) |
| `time_to_goal` | first t satisfying the above, else null | |
| `spl` | `success * d_geodesic / max(d_travelled, d_geodesic)` | |
| `collisions` | debounced *entries* into contact or minimum clearance | `COLLISION_CLEARANCE = 0.05` m |
| `near_misses` | entries into the near band | `NEAR_MISS_CLEARANCE = 0.25` m |
| `stuck_events`, `recovery_time` | commanded speed above eps but displacement below threshold over a window; recovery is time to exit | `STUCK_WINDOW = 2.0` s, `STUCK_DISPLACEMENT = 0.1` m |
| `localisation_failures` | entries into pose-estimate error above threshold | `LOC_FAIL_THRESHOLD = 1.0` m |

`d_geodesic` is computed offline from `WALLS`: rasterise to a 0.05 m occupancy grid over
x ∈ [0, 12], y ∈ [0, 8] (240×160 cells), inflate by `ROBOT_RADIUS = 0.30` m, then Dijkstra
from the goal with 8-connectivity. Cached per scenario. This gives a free validity check —
**if the goal is unreachable from the start, the scenario fails to load** rather than
producing a silently meaningless SPL.

Clearance is analytic against the same wall boxes, so collisions need no physics contact
sensors and stay deterministic. Debouncing matters: one long scrape must score 1, not 400.

## Determinism

- Control rate derived from `policy._dt` / `policy._decimation`; `World.clear_instance()` first
  so the dt arguments actually apply.
- One `np.random.Generator` seeded from `scenario.seed`, passed into `controller.reset()`. No
  global `np.random` or `random` anywhere in `core/` or `controllers/` — enforced by a test.
- Fixed step count from `time_limit`; no wall-clock dependence.
- Verification: same scenario and seed twice → identical `trace.csv`.

**Honest limitation, to be recorded in the results doc:** GPU physics is not guaranteed bitwise
deterministic across drivers or hardware. The check above is same-machine repeatability. The
manifest records GPU name, driver, Isaac version, container digest and the sha256 of the USD
actually loaded, so cross-machine runs are distinguishable; across machines we claim
statistical equivalence, not bitwise.

The USD hash also closes the gap left by an 18 MB binary artefact in git: the manifest names
the bytes that were loaded, whether or not they still match `build_scene.py`.

## Scenarios

Three dev-set scenarios on the existing geometry, in `core/scenario.py` as data. The held-out
set is declared but deliberately not authored here — it must be sealed before generation, and
is the immediate next task.

| id | start → goal | tests |
|---|---|---|
| `doorway_traverse` | (1.5, 1.5) → (10.5, 6.5) | the default route: doorway at y ∈ [5, 8], then the blind corner at (6, 5) |
| `blind_corner` | just west of (6, 5) → east of it, short limit | the visibility/speed trade-off in isolation |
| `return_home` | (10.5, 6.5) → (1.5, 1.5) | same geometry reversed, cheap asymmetry check |

`time_limit = 120` s for `doorway_traverse` (~12 m route, 0.8 m/s cap ≈ 20 s optimal, 6× margin).

## Reference controllers

Both generic, both scenario-agnostic, both labelled in their docstrings as harness-validation
references that **must not appear in a benchmark condition** — the convention `drive_go2.py`
and `walk_forward.py` already follow.

- `goal_seeker.py` — turn to face the goal, walk at it, no sensing. Expected to walk into the
  divider on `doorway_traverse`. Proves the collision, stuck and timeout metrics fire.
- `reactive_avoid.py` — goal attraction plus repulsion from `ranges`. Expected to reach the
  goal. Proves the success and SPL path fires, and sanity-checks that the scenario is solvable.

A harness where the reference controller always succeeds proves nothing; one where it always
fails proves less. Two controllers exercise both paths.

## Verification

1. `pytest core/tests/` — no simulator, runs in seconds. Covers: raycast against known wall
   boxes, geodesic distance to a hand-computed value, one-long-scrape-scores-one, stuck
   detection, unreachable-goal rejection, and an import guard asserting `core/` pulls in no
   `isaacsim`/`omni`/`torch`.
2. `run_episode.py --scenario doorway_traverse --controller goal_seeker --headless`
   → `goal_reached: false`, `collisions > 0`.
3. `run_episode.py --scenario doorway_traverse --controller reactive_avoid --headless`
   → `goal_reached: true`, SPL in a plausible band, `result.json` written to `results/`.
4. Run (3) twice, `diff` the two `trace.csv` files → identical.
5. Run (3) once with `--record` and eyeball the mp4 against the trace.

Runs against `~/miniforge3/envs/env_isaacsim/bin/python`. Containerising against
`nvcr.io/nvidia/isaac-sim:6.0.1` is the last Week 1 task, before any number is reported.

## Out of scope for this step

The full scenario suite and the sealed held-out split (next plan); card ingestion; the node
generator and unit harness; the composition step; the batch runner over multiple episodes;
any of the four conditions; refactoring `drive_go2.py`/`walk_forward.py` onto `go2_policy.py`.
