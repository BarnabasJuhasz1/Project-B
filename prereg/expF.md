# Pre-registration: Mindfire Exp F, a second maze

Written and committed 2026-11-02, **before any of the 16 controllers has been run
on this maze in either simulator.** The commit that adds this file is the
pre-registration; every result file records its hash
(`demo/out/expF/manifest-*.json`, field `prereg_commit`). Anything not listed
here is exploratory and is labelled that way.

## 1. Question

The pilot result (Mindfire preprint, Table I): on the first maze
(`demo/world.py`, "patrol"), under Isaac Sim physics, **arm0 (no paper) 7/8
against armB (Stentz 1994, D\*) 2/8** controllers completed all four trips,
Fisher two-sided p = 0.041. Review item E2: **does this belong to one
geometry?** Exp F runs the same 16 controllers, unchanged, on a second
hand-authored maze with a different topology.

## 2. Hypothesis

- **H_F:** on maze2 under Isaac Sim physics, the controller-level success rate
  differs between arm0 and armB, in the pilot's direction (arm0 > armB).
- The test is **two-sided**. The direction is stated as the expectation and is
  not used to make the test one-sided (as `demo/PREREG-v2.md` §3).

## 3. Units, sample, and what is held fixed

- **Controllers:** the 16 pilot controllers `demo/arms/{arm0,armB}/k0..k7.py`,
  **unchanged**. sha256 (first 16 hex) at registration, also logged per episode:

  | | k0 | k1 | k2 | k3 | k4 | k5 | k6 | k7 |
  |---|---|---|---|---|---|---|---|---|
  | arm0 | 1611cb168ed46b19 | be0105d6d9478909 | 994b2bd45d098912 | cfd732383d7223e8 | ada97fa0953eeca4 | d55a6bf04aa8fd84 | 422a564474a0c246 | 4a656df88b4a6896 |
  | armB | dc3622b547c3568e | 51b58b5eb74b4dd5 | d8f855696060c384 | 0bf3f73571fcd251 | c0422969be1bba1f | 54a3a4f116d013d7 | 69411dd5e493f373 | 70ca68a952daff95 |

- **Hardcoding check, done before registration:** neither `demo/task.md` nor
  `demo/prompt.md` describes the maze. All 16 controllers size their map from
  `scenario.start` / `scenario.goal` plus a margin (arm0 k4: a fixed ±40 m
  grid); none carries the first maze's walls, bounds or door. So nothing had to
  be worked around.
- **Unit of analysis:** the controller (n = 8 per arm). Reps of one controller
  are not independent samples.
- **Held fixed, identical to Exp 3:** task, 180 s whole-episode budget, four
  legs, door shut from leg 2 (switched while the robot is at home), 50 Hz
  control, analytic lidar against the same `World` the kinematic loop uses,
  `pose_error_estimate = 0`, the frozen Go2 locomotion policy, the runner
  `demo/record_isaac.py`, HOME (1.10, 1.10), GOAL (12.90, 8.90), spawn yaw 0.6.
- **Reps: 3 per controller**, `demo/exp3.py` `REP_CONDITIONS`, unchanged:
  rep 0 = seed 0, start offset (0, 0, 0); rep 1 = seed 1,
  (−0.088 m, +0.077 m, +0.023 rad); rep 2 = seed 2, (+0.038 m, −0.069 m,
  +0.066 rad). The same offsets for every controller (paired design).
- **Order:** interleaved across arms, `exp3.job_order` with `ORDER_SEED` 3 over
  the 16 controllers (rep-major blocks, jittered stratified shuffle). The base
  controller is not in the sweep.

## 4. The maze (`demo/maze2.py`)

14 × 10 m, the first maze's bounds, endpoints and spawn. Two full-width
dividing walls make three bands.

| feature | geometry (m) |
|---|---|
| lower divider | y 3.40–3.55, **one** opening x 4.00–5.50 (shared by both routes) |
| upper divider | y 6.60–6.75, WEST opening x 1.70–3.10 (the detour) and the **DOOR** x 8.70–10.30 (on the home→goal line) |
| partial walls | x 6.40–6.55 hanging from the upper divider down to y 5.20; x 5.20–5.35 hanging from the north wall down to y 8.60 |
| clutter (8 blocks) | listed in `demo/maze2.py`; none carries topology |

**Topology vs the first maze.** First maze: the routes split at home, and the
shut door turns a central room into a cul-de-sac the robot must back out of.
Maze2: the routes share a prefix (the single lower opening), split in the
middle band, and with the door shut the robot must turn its back on the goal,
run ~6 m west along the middle band to the west opening and return east along
the whole top band. Same premise: the shut door sits on the straight line
home→goal.

**Checks (`python demo/expF.py check`), all measured by `core.geometry`:**

| check | maze2 | first maze |
|---|---|---|
| short route, door open (geodesic) | 15.03 m | 15.03 m |
| route with the door shut | 19.39 m | 19.36 m |
| detour cost (must be > 3 m) | 4.36 m | 4.33 m |
| a shortest open route passes through the door | yes | yes |
| centre free width at the tightest point of the shut-door route (must be ≥ 0.60 m) | **0.74 m**, whole route | 0.77 m in its bypass window; 0.35 m over the whole route |
| start / goal clearance beyond ROBOT_RADIUS | 0.65 / 0.65 m | 0.65 / 0.65 m |
| Go2 circumscribed radius 0.383 m clear of walls at every rep's spawn | yes (min clearance 0.86 m) | yes |

⚠️ The navigability check here is **stricter** than the first maze's: it
bisects an inflated grid over the whole route, where `world.bypass_bottleneck()`
scans one window. Under the stricter check the first maze's shut-door route has
a 0.35 m centre slot somewhere outside its window. Disclosed, not fixed: the
first maze is untouched.

**Calibration, on reference controllers only.** One draft had a clutter block on
the home→goal line in the bottom band; the frozen base
(`generate/base/base_controller.py`) was deflected along the lower divider and
failed leg 0 with the door open. The block was moved off the line. On the
registered maze, kinematic: base 2/4 (legs 0–1 in 26.5 s and 57.7 s, then fails
at the shut door; first maze 27.0 s, 28.7 s, 2/4); `controllers/reactive_avoid`
2/4 (first maze 1/4). An earlier draft also moved two clutter blocks that left
sub-0.60 m slots. **No arm controller was run on any draft.**

**Sanity episodes, base controller, rep 0** (`demo/out/expF/sanity.csv`, traces
in `demo/out/expF/traces-sanity/`):

| simulator | trips | trip times | contacts |
|---|---|---|---|
| kinematic | 2/4 | 26.5 s, 57.7 s | 0 |
| Isaac Sim | 1/4 | 29.3 s | 0 |

Under physics the Go2 settled at (1.037, 1.071), 0.59 m clear of every wall,
and completed leg 0. On leg 1 (door open) the base commanded a pure turn
(`wz = -1`, `vx = 0`) at the goal for ~20 s while the realised yaw did not move
(0.02 rad throughout, tracking error 1.0 rad/s, no contact, 0.66 m clearance),
then took the long way home and ran out of budget. That is the gait not turning
in place, not the maze: removing the clutter block nearest the goal changed
nothing kinematically (leg 1 57.6 s vs 57.7 s), so the maze was **not** changed
for it. Leg 1 is slow kinematically because the base follows the upper divider
past the open door to the west opening. The door's geometry is checked by
`scene-check`; that it shuts on leg 2 under physics is read from the `door`
column of every sweep trace. (First maze, physics, base: 3/4.)

Scene: `scenes/demo-maze2.usd`, built by
`python demo/export_demo_world.py --maze maze2` with the first maze's exporter
(lab42 lighting, wall height and colour; the door is the prim `/World/Door`).
`python demo/expF.py scene-check` checks every wall block and the door against
`demo/maze2.py`.

## 5. Outcomes

- **Episode success:** all **4** trips completed within the 180 s budget.
- **Controller success (PRIMARY):** episode success in **at least 2 of the 3**
  reps (as `demo/PREREG-v2.md` §4).
- **Recorded for every episode** (`demo/out/expF/{physics,kinematic}.csv`, plus
  a per-step gzip trace): trips completed, time per trip, collisions (contact
  onsets), path length, tracking error (|realised − commanded| planar velocity,
  and yaw-rate error, median and p95 per episode, per step in the trace), the
  controller's planning time per step (`step()` wall ms: median, p95, max per
  episode; per step in the trace), load average, power profile.

## 6. Analysis (`python demo/expF.py report`)

- **Primary:** arm0 vs armB controller success under **Isaac Sim physics**,
  Fisher exact, **two-sided, α = 0.05**, with Wilson 95 % intervals per arm.
- **Secondary, labelled as such:**
  - S1: the same test on **rep 0 alone** (the pilot's one-episode format).
  - S2: the same test in the **kinematic** simulator (`demo/rollout.py`, same
    reps and offsets).
  - S3: descriptive per arm: mean trips, leg-2 time, collisions per metre,
    tracking error, planning time per step, and how often all 3 reps agree.
- **Pass criteria, fixed now:**

  | outcome | reading |
  |---|---|
  | p < 0.05 and arm0 > armB | **the pilot effect replicates on a second maze** |
  | p ≥ 0.05 and arm0 > armB | same direction, not significant: **not a replication**; report the intervals |
  | arm0 ≤ armB | **does not replicate**; the pilot effect is not general across these two mazes |

- ⚠️ With n = 8 per arm only a large difference can reach p < 0.05 (the pilot's
  7/8 vs 2/8 gives 0.041; 6/8 vs 2/8 gives 0.13). A non-significant result is
  **not** evidence of equality.
- The two mazes are **not pooled** into one test. A side-by-side table with
  Exp 3 is exploratory.

## 7. Procedure and exclusions

- One Isaac Sim process at a time on the laptop (`flock /tmp/mindfire-isaac.lock`).
  No other heavy job runs during a physics episode; the kinematic sweep runs
  under the same lock.
- Before every physics episode `powerprofilesctl get` must report
  `performance`; otherwise the sweep pauses at the episode boundary and resumes
  later. The profile is logged per episode.
- A controller that raises scores the legs it completed (it is a result). An
  episode where the **simulator** produced no result line is re-run once; if it
  fails again it is reported as missing and the controller is analysed on its
  remaining reps (majority of the reps that exist; ties count as failure).
- Local runs, `in_container: false`, disclosed as for Exp 3.
- Order of work after this commit: kinematic sweep, then physics sweep, then
  `demo/expF.py report` exactly as committed.
