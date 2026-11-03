# Exp B results — hand-written D* Lite and A* baselines (2026-11-03)

Pre-registration: `prereg/expB.md`, **commit `cec7f345e1dfeaf180f01418a67191a290d9e4bf`**
(written in every row of `physics.csv`). Baselines unchanged since that commit,
checked by the runner. Physics run 2026-11-03 01:30–01:47 CET, local machine,
`in_container: false`, AC online, `powerprofilesctl get` = `performance` before
and after every episode. Reproduce the tables: `python demo/baselines/run_expB.py report --v2 demo/out/v2/physics.csv`.

⚠️ The baselines are in no arm and in no arm's count.

## Kinematic gate (before the prereg commit): passed

| baseline | rep 0 | rep 1 | rep 2 | collisions |
|---|---|---|---|---|
| D\* Lite | 4/4 | 4/4 | 4/4 | 0 |
| A\* replan | 4/4 | 4/4 | 4/4 | 0 |

Leg times about 23 / 25 / 53–60 / 34 s. Planning time per planning step: D\*
Lite median 5.5 ms, max about 220 ms (the repair after the door is found shut);
A\* median 6.7 ms, max 58 ms.

## Physics: primary result

| baseline | rep | trips | trip times s | collisions | track_v med m/s | track_wz med rad/s | plan ms med / p95 / max |
|---|---|---|---|---|---|---|---|
| A\* replan | 0 | 2 | 23.96, 26.40 | 0 | 0.0001 | 1.00 | 44.6 / 49.6 / 59.3 |
| A\* replan | 1 | 2 | 24.04, 26.68 | 0 | 0.0000 | 1.00 | 13.4 / 22.8 / 32.6 |
| A\* replan | 2 | 2 | 24.04, 29.20 | 0 | 0.0000 | 1.00 | 11.7 / 16.9 / 27.1 |
| D\* Lite | 0 | 2 | 24.18, 28.88 | 0 | 0.0000 | 1.00 | 6.8 / 18.1 / 804.7 |
| D\* Lite | 1 | 2 | 24.24, 26.92 | 0 | 0.0000 | 1.00 | 5.7 / 16.1 / 286.3 |
| D\* Lite | 2 | 1 | 24.10 | 0 | 0.0001 | 1.00 | 5.2 / 10.3 / 136.4 |

**D\* Lite 0/3, A\* 0/3. Both fail by majority of 3.** The pre-registered
reading is the both-fail row: **the shared map/follower does not work under
physics, and Exp B makes no claim about incremental replanning either way.**
The question is not answered by this run.

Notes on the columns. Collisions = contact onsets (`gap ≤ 0`). Tracking-error
medians are dominated by the stalled stretch below (a commanded 1 rad/s turn
executed as 0), so they are not comparable with the arms' values. Planning time
is the trace's `step_ms` on planning steps. It is wall time, inflated in rep 0
by other agents' CPU load (load average 5.5 and 22 against about 9–14 later). It cannot move a
trajectory: neither baseline reads the clock.

## Why every episode failed (descriptive, read off the traces)

All six fail in the **same way, before the planners differ in anything**. 5 of 6
stall at HOME right after leg 1, and the sixth (D\* Lite rep 2) at the GOAL after leg
0. In each case the goal has just flipped and the follower turns in place
(`vx = 0`, `wz = ±1.0`, the saturated rate). Over about 1 s the frozen gait's yaw rate
decays to zero, and the robot then stands still for the rest of the episode
(109–155 s) while the same spin command is repeated. A\* rep 0 got further: it
froze on leg 2 inside the central room at (5.64, 5.83), also mid-spin. In 5 of 6
episodes the robot never reached the door, so the door-repair step that
separates D\* Lite from A\* was never exercised.

The kinematic simulator executes a spin exactly, which is why the gate passed.

## Exploratory, NOT pre-registered: the same stall in Exp 3's arm controllers

A post-hoc scan of the 51 Exp 3 traces for stretches where the controller
commands motion (|wz| > 0.5 or vx > 0.2) and the pose does not change
(< 1 cm, < 0.02 rad over 1 s). Longest such stretch, in seconds:

| controller | reps (legs) | longest stall s | pure-spin fraction of steps |
|---|---|---|---|
| armB k1 | 2 / 2 / 2 | 42 / 85 / 83 | 0.57–0.67 |
| armB k5 | 2 / 1 / 1 | 69 / 5 / 24 | 0.77–0.88 |
| armB k6 | 1 / 1 / 1 | 154 / 153 / 150 (forward command, not a spin) | 0.00 |
| arm0 k4 | 4 / 2 / 2 | 1 / 108 / 63 | 0.01 / 0.60 / 0.38 |
| base | 3 / 1 / 1 | 18 / 8 / 19 | 0.05–0.24 |
| all other arm0/armB controllers | | ≤ 17, mostly ≤ 4 | ≤ 0.54 |

Every Exp 3 episode with a stall longer than 20 s failed. armB k2 spins a
lot (pure-spin fraction 0.33–0.54) but its stalls are short, and armB k0 and
k7 fail without long stalls, so this is **one failure mode, not the
explanation**. It is a candidate mechanism for the physics-vs-kinematic gap
that has nothing to do with D\*: a controller that turns in place at the
saturated rate can stall the frozen gait. It is a hypothesis from data already
seen and must be tested on fresh episodes.

## What would answer the original question

The follower is the confound. A dated amendment (Exp B2) that changes **only
the shared follower**, identically in both baselines, would answer it: for
example, never command a pure saturated spin (cap |wz| below 1.0, or keep a
small forward speed while turning). That means re-passing the kinematic gate,
committing an amendment before any episode, and 6 more physics episodes
(about 20 min). Not run: it changes the frozen baselines, so it needs approval.
