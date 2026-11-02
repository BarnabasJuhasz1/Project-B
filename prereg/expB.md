---
created: 2026-11-02
experiment: Exp B (Mindfire review item E12) — hand-written reference baselines
status: pre-registered, committed before the first physics episode of demo/baselines/run_expB.py
---

# Exp B — is incremental replanning itself fragile on this gait?

## Question

Under Isaac Sim physics, the armB controllers (written with Stentz 1994, D\*, in
the prompt) complete all four trips far less often than arm0's (pilot 2/8 vs
7/8; Exp 3, majority of 3 reps, 2/8 vs 4/8). The armB controllers that repair
their plan purely incrementally, with no full-replan path, failed 3 of 3 in the
pilot. **Is incremental replanning itself fragile on this gait, or only the
model's implementations of it?**

## The two baselines (frozen at this commit)

Both written by hand, `demo/baselines/`, never in `demo/arms/`, and in **no
arm's count, test or denominator**.

| | planner |
|---|---|
| `dstar_lite.py` | D\* Lite (Koenig & Likhachev 2002), goal-rooted, **incremental repair only**: one search tree per goal, initialised once, then only UpdateVertex + ComputeShortestPath on the cells whose cost changed. No full-replan fallback anywhere. |
| `astar_replan.py` | A\* **from scratch at every planning step**, same graph, same edge costs, same heuristic. |

Everything else is one shared block, byte-identical in both files
(`demo/baselines/_shared.py`, checked by test): a persistent log-odds
occupancy grid built only from the lidar and `pose_estimate` (unknown = free),
0.10 m cells, inflation (impassable within 0.38 m of an occupied cell, soft cost
to 0.80 m), planning every 10 control steps (5 Hz) and on each goal switch, a
pure-pursuit follower (0.7 m lookahead, 0.7 m/s cap, turn in place above
0.9 rad of heading error, speed capped by front clearance), and a 0.6 s back-up
on contact. No wall-clock budget: the trajectory is a pure function of the
observations. Same interface, sensing, import rules and 180 s budget as the
generated controllers (`demo/task.md`).

**Correctness evidence before this commit:** D\* Lite matches a from-scratch
Dijkstra after every repair over 2,400 random cost-change/motion steps; an
in-episode shadow check found 0 of 693 planning steps where D\* Lite's path
cost differed from A\*'s; the blocked-route test repairs without
re-initialising. While writing it, a one-ulp floating-point tie in the key
comparison made the first version stop its search early with a consistent but
too-low g(start), and that version **failed leg 2 kinematically**. Keys are now
rounded to 1e-9. Recorded because it is exactly the kind of defect a correct-looking
D\* implementation can carry.

## Kinematic gate (passed before this commit)

Each baseline must complete all four trips in all 3 kinematic reps
(`demo/rollout.py`, Exp 3's rep conditions). Result
(`demo/out/expB/kinematic.csv`): **D\* Lite 3/3, A\* 3/3**, 0 collisions.

## Physics design

- Runner: `demo/record_isaac.py` via `physics_sweep.run_one`, unchanged: frozen
  Go2 policy, analytic lidar, door shut from leg 2, 180 s budget.
- Reps: 3 per baseline, Exp 3's `REP_CONDITIONS` (rep 0 = original start;
  reps 1–2 = seeds 1–2, start offsets (−0.088, 0.077, 0.023) and
  (0.038, −0.069, 0.066)), order from `exp3.job_order` (rep-major, alternating).
- One Isaac Sim process at a time (`flock /tmp/mindfire-isaac.lock`).
  `powerprofilesctl get` must say `performance` before each episode, otherwise
  the run pauses.
- Runner refuses to start unless this file is committed and the two baseline
  files are unchanged since this commit; the commit hash is written in every row.

## Primary metric

- **Episode success** = all four trips completed within 180 s.
- **Baseline succeeds** = success in at least 2 of its 3 reps (majority of 3,
  as Exp 3).

## Pass criteria and reading (fixed in advance)

| D\* Lite | A\* | reading |
|---|---|---|
| succeeds | succeeds | **Incremental replanning is not in itself fragile on this gait.** The armB failures are attributed to the models' implementations (or to other features of the generated code), not to the mechanism. |
| fails | succeeds | **Incremental repair is fragile on this gait even when correct**, given this map and follower. Supports the mechanism-level reading. |
| succeeds | fails | Incremental repair is not the fragile part; reported as is. |
| fails | fails | **The shared harness (map, follower, recovery) does not work under physics.** No claim about incremental replanning either way. |

## Recorded per episode, descriptive

Trips completed; time per trip; collisions (contact onsets, `gap ≤ 0`);
tracking error (`track_v_med`, `track_v_p95`, `track_wz_med` from the trace);
controller step time (median, p95, max) and **planning time per planning step**
(the trace's `step_ms` on planning steps, by the schedule above), plus
non-planning step time. Kinematic only: search expansions per plan.

## Comparison with the arms

Descriptive only. Each baseline is one controller (n = 1, 3 reps), so **no test
against the arms** is made. Reported beside Exp 3's arm0 and armB rows
(`demo/out/v2/physics.csv`: episode success, majority-of-3 success, mean legs,
contacts, tracking error).

## Declared limitations

- n = 1 controller per planner type. A pass or a fail is about these two
  implementations under one map/follower design, on one hand-authored world.
- The follower is pure pursuit on the planned path, not the rollout-scored
  velocity selection most arm0 controllers use. If both fail, that may be the
  follower, which is why the both-fail row makes no claim.
- Local machine, `in_container: false`, as Exp 3.
- Written after the pilot and Exp 3 results were known; the design choices
  (cell size, inflation, speed) were made to pass the kinematic gate only and
  were not tuned on physics.
