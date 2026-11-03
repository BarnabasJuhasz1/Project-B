---
created: 2026-11-03
experiment: Exp B2 (Mindfire review item E12, follow-up to Exp B) — dated amendment to prereg/expB.md
status: pre-registered, committed before the first physics episode of `run_expB.py physics --variant b2`
approved: Barna, 2026-11-03, "Yes, sure. Let's approve a follow-up also."
---

# Exp B2 — the same question, with the shared follower repaired

This is a **dated amendment (2026-11-03)** to `prereg/expB.md`. It does not
edit that file, and Exp B's baselines and results stay as they are
(`demo/baselines/{dstar_lite,astar_replan}.py`, `demo/out/expB/`).

## Why

Exp B ended in its pre-registered both-fail row: D\* Lite 0/3, A\* 0/3. All six
episodes failed the same way, before the planners differed: after a goal flip the
shared follower turned in place at the saturated 1.0 rad/s, the frozen gait's
yaw rate decayed to zero within about 1 s, and the robot stood still for the rest of
the episode (`demo/out/expB/RESULTS.md`). The question, **is incremental
replanning itself fragile on this gait, or only the model's implementations?**,
is unanswered.

## What changes: the shared follower only

New files `demo/baselines/b2/{dstar_lite,astar_replan}.py`, built by
`demo/baselines/b2/build.py` from the Exp B files. They are identical except for the shared
block, whose single source is `demo/baselines/b2/_shared.py`. Tests
(`demo/baselines/b2/test_b2.py`) pin that the planner code is byte-identical to
Exp B's and that the map, cost function, planning schedule and path extraction
(`_Grid`, `edge_cost`, `path_cost`, `_plan`, `_lookahead`) are unchanged.

| | Exp B | Exp B2 |
|---|---|---|
| hard turn (heading error > 0.9 rad) | in place, vx 0, \|wz\| 1.0 | **on an arc**: vx 0.40 m/s, \|wz\| 0.80. In place at \|wz\| 0.80 only when front clearance < 0.80 m |
| \|wz\| cap otherwise | 1.0 | **0.80** |
| stall watchdog | none | **if \|wz\| ≥ 0.5 was commanded for the last 0.5 s and the pose turned < 25 % of it, drive straight for 0.6 s** (forward 0.3 m/s, or back 0.3 m/s if the front is < 0.6 m), then resume |

The watchdog reads only `pose_estimate`, which every controller gets. No
wall-clock budget is added.

**Where the numbers come from, declared post hoc.** They were chosen from the
Exp 3 and Exp B physics traces, not by running these baselines under physics.
For 0.5 s windows of a constant command, the share where the robot turned less
than 20 % of the commanded rate was 0.96 for vx 0 / \|wz\| 1.0 (n = 19,854),
0.93 for vx 0.15 / \|wz\| 1.0, and **0 of 337** for vx ≥ 0.4 at any \|wz\|.
(Constant-command windows over-represent stalls, because a stalled controller
repeats itself, so these are not stall rates.) The values are a single choice
made once, with no physics trial of B2 before this commit.

## Kinematic gate (passed before this commit)

`run_expB.py kinematic --variant b2` → `demo/out/expB2/kinematic.csv`:
**D\* Lite 3/3, A\* 3/3**, 0 collisions, 0 watchdog firings, max \|wz\|
commanded 0.80 (tested). All Exp B tests and 11 new B2 tests pass.

## Unchanged from prereg/expB.md

Runner (`record_isaac.py` via `physics_sweep.run_one`), world, door schedule,
180 s budget, Exp 3's three rep conditions and order, majority-of-3 rule,
everything recorded per episode, the comparison with the arms (descriptive,
no test), and every declared limitation. Primary metric: **episode success =
all four trips within 180 s; a baseline succeeds with ≥ 2 of 3.**

The pre-registered reading table is the same:

| D\* Lite | A\* | reading |
|---|---|---|
| succeeds | succeeds | incremental replanning is not in itself fragile on this gait; the armB failures belong to the models' implementations (or other features of the generated code) |
| fails | succeeds | incremental repair is fragile on this gait even when correct, given this map and follower |
| succeeds | fails | incremental repair is not the fragile part |
| fails | fails | the shared harness still does not work under physics; no claim |

## Added checks

- Before **each** episode: a mains adapter must report online
  (`/sys/class/power_supply/*`, type `Mains`) **and** `powerprofilesctl get` must
  say `performance`, otherwise the run pauses (exit 3) and is reported.
- The runner refuses to start unless this file is committed and clean and the
  two B2 baseline files are unchanged since this commit. The hash goes in every
  row of `demo/out/expB2/physics.csv`.

## Additional declared limitations

- B2 is a second attempt, designed after Exp B failed and after Exp 3's traces
  were seen. A B2 success shows that **a** follower exists under which the
  question can be answered; it does not license reporting Exp B's follower as
  adequate.
- The watchdog is a recovery behaviour that most generated controllers do not
  have. It is identical in both baselines, so it cannot favour either planner,
  but it widens the gap between the baselines and the arms. Any comparison with
  arm0/armB is context, not a like-for-like test.
- The number of watchdog firings per physics episode is not observable from the
  trace directly. It is reconstructed from the trace by the same rule and
  reported descriptively.
