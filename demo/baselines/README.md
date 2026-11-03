# `demo/baselines/` — hand-written reference baselines (Mindfire Exp B)

⚠️ **NOT GENERATED, NOT AN ARM, IN NO ARM'S COUNT.** These two controllers were
written by hand to answer one question (review item E12): **is incremental
replanning itself fragile on the Go2 gait, or only the models' implementations
of it?** Nothing in `render.ARMS`, `physics_sweep.discover()`, `analyse.py` or
`code_features.py` reaches this directory, and a test pins that.
Pre-registration: `prereg/expB.md`.

| file | what |
|---|---|
| `dstar_lite.py` | D\* Lite, incremental repair only, one tree per goal, initialised once |
| `astar_replan.py` | A\* from scratch every planning step |
| `_shared.py` | the map / cost / schedule / follower / recovery block, pasted byte-identical into both |
| `assemble.py` | the only way that block is pasted; `--check` verifies |
| `test_baselines.py` | import gate, no wall clock, shared block identical, A\* and D\* Lite vs Dijkstra, blocked-route repair, kinematic patrol, kept out of arms |
| `run_expB.py` | `kinematic`, `physics` (needs the committed prereg; checks `powerprofilesctl` per episode), `report` |

Both are single files under the generated controllers' rules (`demo/task.md`):
stdlib + numpy + `core.observation`, state cleared in `reset()`, `step()` never
raises, 180 s budget, same 181-beam lidar and pose. **No wall-clock budget**,
so machine load cannot change a trajectory.

## Design

**Shared (identical in both):** log-odds occupancy grid, 0.10 m cells, built
only from the lidar (free along each beam, occupied at the hit; unknown = free).
Cost per cell: impassable within 0.38 m of an occupied cell, a quadratic soft
cost to 0.80 m, 1 elsewhere; cells within 0.30 m of the robot and the goal are
never impassable. 8-connected, no corner cutting, edge cost = length × mean of
the two cell costs, octile heuristic. Plan every 10 control steps (5 Hz) and on
each goal switch. Pure-pursuit follower, 0.7 m lookahead, 0.7 m/s, turn in
place above 0.9 rad heading error, speed capped by front clearance; 0.6 s
back-up on contact.

**D\* Lite** (Koenig & Likhachev 2002): goal-rooted search, `k_m` for robot
motion, the changed cells and their neighbours re-evaluated each planning
step, then ComputeShortestPath. A tree per goal (home, goal), so the tree to
the goal built on leg 0 is the one repaired when the door is found shut on
leg 2. `inits == 1` per tree for the whole episode (tested). There is no
full-replan code path.

⚠️ **A floating-point defect found while writing it.** Keys mix `k_m` and
octile distances with √2, so two mathematically equal keys can differ by one
ulp; the termination test `TopKey < CalculateKey(s_start)` then stopped one
vertex early with g(s_start) consistent but too low. The randomised
Dijkstra test hit it in 3 of 200 sequences, and in the patrol the robot **lost
leg 2 kinematically**. Keys are rounded to 1e-9; 0 mismatches since over 15,000
random checks and 693 in-episode A\* shadow comparisons.

**A\***: fresh open list, g and parents every call; nothing but the map
carries over.

## Results

`demo/out/expB/RESULTS.md`. Physics, 2026-11-03: **D\* Lite 0/3, A\* 0/3**, every episode lost to the same in-place-spin stall of the gait, so by the pre-registered both-fail row Exp B answers nothing about incremental replanning.

    python -m pytest demo/baselines/test_baselines.py -q
    python demo/baselines/run_expB.py kinematic
    flock /tmp/mindfire-isaac.lock python demo/baselines/run_expB.py physics
    python demo/baselines/run_expB.py report --v2 demo/out/v2/physics.csv

## Exp B2 (`b2/`, `prereg/expB2.md`)

The same two planners, byte for byte, with only the shared follower changed:
hard turns on an arc at 0.4 m/s and at most 0.8 rad/s, and a watchdog that
drives straight briefly when a commanded turn stalls. `b2/build.py` builds the
B2 files from the ones above, and `b2/test_b2.py` pins that only the follower
differs. Physics: **D\* Lite 3/3, A\* 3/3**, so a correct, purely incremental
D\* Lite is not in itself fragile on this gait (`demo/out/expB2/RESULTS.md`).

    python demo/baselines/run_expB.py kinematic --variant b2
    flock /tmp/mindfire-isaac.lock python demo/baselines/run_expB.py physics --variant b2
