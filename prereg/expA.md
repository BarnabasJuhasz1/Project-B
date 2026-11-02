# Pre-registration: Exp A, the open-maze control

Written and committed on 2026-11-02, **before any Exp A episode was run**. The commit hash of this file is recorded in every result row (`prereg_commit`) and in the sweep manifest. Changes after the first episode are dated appends at the end of this file, never edits.

Status of the evidence: the 16 controllers are the **pilot** (8 arm0 written without the paper, 8 armB written with Stentz 1994, D\*), already evaluated under physics in the pilot (`demo/out/physics.csv`) and in Exp 3 (`demo/out/v2/physics.csv`, 3 reps). This experiment is exploratory with respect to the pilot hypothesis, but its analysis is fixed here in advance.

## 1. Question

In the blocked maze, the armB controllers fail mostly at **leg 2**, the first trip after the door shuts (pilot legs 2 2 2 4 4 2 1 2). Do the armB controllers fail **only when they have to replan** around the blocked route, or **do they fail anyway**, even when the short route stays open?

## 2. Hypothesis

H-A (replanning-specific failure): with the door never shutting, armB controllers complete all four trips about as often as arm0 controllers do. If instead armB fail in the open maze too, their failure is not specific to replanning.

## 3. Design

- **Controllers:** `demo/arms/{arm0,armB}/k0..k7.py`, unchanged (16).
- **Apparatus:** Exp 3's, unchanged: `demo/record_isaac.py` (Isaac Sim 6.0.1, frozen Go2 flat-terrain policy, analytic lidar, 50 Hz control), `PatrolScenario`, 180 s budget for the whole four-trip episode, arrival radius as in `demo/rollout.py`.
- **The one change:** `record_isaac.py --door-never-shuts`. The door prim stays 50 m below the floor on all four legs and the lidar is cast against `OPEN_WORLD` on all four legs. Default behaviour without the flag is unchanged.
- **Reps:** 3 per controller, the Exp 3 conditions exactly (`demo/exp3.py` `REP_CONDITIONS`): rep 0 seed 0, offset (0, 0, 0); rep 1 seed 1, (−0.088 m, 0.077 m, 0.023 rad); rep 2 seed 2, (0.038 m, −0.069 m, 0.066 rad). 48 episodes.
- **Order:** `exp3.job_order` with seed 41: rep-major blocks, arms interleaved by a jittered stratified shuffle. Runner: `demo/exp_ad.py A`.
- **Machine discipline:** one Isaac Sim process at a time (`flock /tmp/mindfire-isaac.lock`); the runner checks before every episode that the power profile is `performance` (ACPI `platform_profile` and `powerprofilesctl get`) and pauses otherwise; an episode during which it changed is quarantined and rerun. It refuses to run under any cgroup CPU limit. Nothing heavy runs alongside.
- **Recorded per episode:** trips completed, time per trip, collisions (contact events), tracking error (median and p95 of |realised − commanded| planar velocity, median and p95 of |yaw-rate error|), planning time per step (median, p95, max of the controller's `step()` wall time; every step in the gzip trace), load average, power profile, cpu.max.

## 4. Primary metric and test

- **Unit:** the controller. **Success** for an episode = all four trips within 180 s (`legs == 4`). A controller **succeeds** if it succeeds in **at least 2 of its 3 reps**.
- **Primary test:** Fisher exact, two-sided, arm0 vs armB on controller successes in the open maze (8 vs 8).

## 5. Secondary (descriptive, labelled exploratory)

- Per-episode success per arm (episodes are not independent; no inference rests on it).
- Paired per controller, open (Exp A) vs blocked (Exp 3) majority success; exact McNemar within each arm.
- Which leg each failing episode stops on; trip times; collisions; tracking error; step time.

## 6. Pass criteria and reading, fixed in advance

| result in the open maze | reading |
|---|---|
| armB ≥ 6/8 and Fisher p ≥ 0.05 | **H-A supported**: armB's blocked-maze failures are tied to the route change (replanning) |
| armB ≤ 4/8 | **H-A rejected**: armB fail even with the route open; the failure is not specific to replanning |
| anything else (armB 5/8, or ≥ 6/8 with p < 0.05) | mixed; report the counts, no directional claim |

A non-significant Fisher test is not evidence of equality; with 8 per arm it has little power, and the counts are reported beside it.

## 7. Analysis script

`demo/exp_ad_report.py`, committed with this file. It reads `demo/out/expA/physics.csv` and Exp 3's `physics.csv` (read only) and writes `demo/out/expA/SUMMARY.md`. Anything not in it is labelled exploratory.

## 8. Known limitations

- One hand-authored world; the pilot controllers only; local Isaac Sim, `in_container: false`.
- Isaac Sim is not bit-reproducible here for wall-clock-budgeted controllers (Exp 3 rep 0 matched the pilot in 11 of 17 controllers).
- The open maze removes the replanning demand but also shortens legs 2 and 3, so it also gives more time slack.
