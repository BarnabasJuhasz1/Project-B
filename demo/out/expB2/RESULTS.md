# Exp B2 results — the baselines with the repaired follower (2026-11-03)

Pre-registration: `prereg/expB2.md`, a dated amendment to `prereg/expB.md`.
**Commit `162c65b51a7bbca262df2a15106510c3303d8c75`** on origin/main, pushed
before the first B2 episode and written in every row of `physics.csv`. It was
first committed locally as `c049be6` (identical content). The B2 baselines are
unchanged since that commit, which the runner checks. Run 2026-11-03, about 05:45–06:11 CET, local machine,
`in_container: false`. A mains adapter was online and `powerprofilesctl get` said
`performance` before every episode (and after it, per the `power_profile` column). Exp B's
own results (`demo/out/expB/`) are untouched.

    python demo/baselines/run_expB.py report --variant b2 --v2 demo/out/v2/physics.csv

## Kinematic gate (before the prereg commit)

D\* Lite 3/3, A\* 3/3, 0 collisions, 0 watchdog firings
(`demo/out/expB2/kinematic.csv`).

## Physics: primary result

| baseline | rep | trips | trip times s | total s | collisions | track_v med / p95 m/s | track_wz med / p95 rad/s | plan ms med / p95 / max | watchdog firings |
|---|---|---|---|---|---|---|---|---|---|
| A\* replan | 0 | **4** | 23.94, 31.42, 81.76, 40.38 | 177.5 | 0 | 0.057 / 0.135 | 0.095 / 0.49 | 20.2 / 109.6 / 154.9 | 18 |
| A\* replan | 1 | **4** | 24.06, 31.12, 79.00, 41.68 | 175.9 | 0 | 0.056 / 0.138 | 0.095 / 0.47 | 23.1 / 202.3 / 350.2 | 19 |
| A\* replan | 2 | **4** | 24.16, 32.34, 81.04, 40.32 | 177.9 | 0 | 0.056 / 0.144 | 0.095 / 0.50 | 20.1 / 105.2 / 161.9 | 21 |
| D\* Lite | 0 | **4** | 24.20, 31.28, 78.62, 41.90 | 176.0 | 0 | 0.054 / 0.129 | 0.093 / 0.49 | 14.8 / 93.0 / 1365.9 | 19 |
| D\* Lite | 1 | **4** | 24.24, 32.38, 79.00, 41.92 | 177.6 | 0 | 0.055 / 0.134 | 0.093 / 0.51 | 13.7 / 103.0 / 1409.9 | 20 |
| D\* Lite | 2 | **4** | 24.10, 27.42, 77.02, 41.78 | 170.3 | 0 | 0.054 / 0.127 | 0.091 / 0.39 | 15.0 / 101.5 / 1358.9 | 14 |

**D\* Lite 3/3, A\* 3/3. Both succeed by majority of 3.** The pre-registered
reading is the succeeds/succeeds row: **a correct, purely incremental D\* Lite
is not in itself fragile on this gait.** It completes the blocked-route patrol
under physics exactly as often as a planner that replans from scratch. So the
armB failures under physics belong to the models' implementations, or to other
features of the generated code, and not to incremental replanning as a mechanism.

Columns: collisions = contact onsets (`gap ≤ 0`; minimum gap 0.06–0.10 m). The
tracking error is now comparable with the arms' (Exp 3 median `track_v_med`
0.075 arm0, 0.077 armB). Planning ms is the trace's `step_ms` on planning
steps. Non-planning steps take a median of 0.7 ms. D\* Lite's median planning step is
cheaper than A\*'s (14 vs 21 ms), but its worst step is far costlier (about 1.4 s, the
repair after the door is found shut). Wall time cannot move these trajectories,
because neither baseline reads the clock. Watchdog firings are reconstructed from the
trace (onsets of the unstick command: `wz = 0`, `|vx| = 0.30`).

## Caveats that go with the result

- **Thin time margin.** Every episode used 170–178 s of the 180 s budget. Leg 2
  (the detour) took 77–82 s against about 60 s kinematically. Both planners are equally
  close to the limit, so this does not separate them, but a slightly slower
  follower would have turned 3/3 into 0/3 for both.
- **The watchdog does real work.** It fired 14–21 times per episode, so the
  gait still stops turning often under B2's capped arc turns. Success depends on
  recovering from those stalls, which few generated controllers do. Exp B, without the
  watchdog and with full-rate turns in place, went 0/6.
- n = 1 controller per planner, one hand-authored world, local machine. B2 was
  designed after Exp B failed and after Exp 3's traces were seen (declared in
  the amendment).

## Against arm0 and armB (descriptive, no test)

| | episodes, all four trips | controllers, majority of 3 | mean trips | median contacts |
|---|---|---|---|---|
| D\* Lite (B2) | 3/3 | 1/1 | 4.00 | 0 |
| A\* replan (B2) | 3/3 | 1/1 | 4.00 | 0 |
| arm0 (Exp 3) | 14/24 | 4/8 | 3.12 | 3 |
| armB (Exp 3) | 6/24 | 2/8 | 2.29 | 0 |
| base (Exp 3) | 0/3 | 0/1 | 1.67 | 0 |
| D\* Lite / A\* (Exp B, before the follower fix) | 0/3 each | 0/1 each | 1.67 / 2.00 | 0 |

Read together: what decided the hand-written baselines under physics was the
**follower** (turning style plus stall recovery), not the planner. Exp B and B2
share the identical planners and go from 0/6 to 6/6 with only the follower
changed. That fits the exploratory finding in `demo/out/expB/RESULTS.md`:
long commanded-turn stalls occur in several failing Exp 3 controllers of both arms
(armB k1, k5, k6; arm0 k4 reps 1–2). That link stays exploratory and is not
a tested claim about the arms.
