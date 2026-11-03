---
created: 2026-11-03
status: done
---

# Exp C results — full-replan fallback ablation (E7b)

> ⚠️ **INTERVENTION ON MODEL-WRITTEN CODE.** Every patched controller is a
> model's file with a block **we** appended (`demo/interventions/expC/`). These
> are not armB samples and enter no arm count, no Fisher test and no Exp 1-4
> number.

Pre-registration: `prereg/expC.md`, commit **`db8c9120127b`** (pushed to
origin/main before the first physics episode). That hash is recorded in every row
of `demo/out/expC/physics.csv` (`prereg_commit`). Analysis:
`python demo/expC.py report`, exactly as committed.

## Verdict: DOES NOT SUPPORT H-C

Under physics, 3 reps per controller with Exp 3's seeds and start offsets, each
paired with its own unpatched Exp 3 episode (`demo/out/v2/physics.csv`):

| k | Exp 4 label | unpatched legs r0/r1/r2 | patched legs | fallback firings | controller |
|---|---|---|---|---|---|
| k0 | incremental_only | 2 2 2 | 2 2 2 | 11 11 11 | fail → fail |
| k1 | incremental_only | 2 2 2 | 2 2 2 | 11 12 11 | fail → fail |
| k2 | incremental_with_fallback | 2 2 2 | 2 2 2 | 13 11 11 | fail → fail |
| k3 | incremental_only | 4 4 4 | 4 4 4 | 2 2 2 | success → success |
| k4 | incremental_only | 4 4 4 | 4 4 4 | 1 1 2 | success → success |
| k6 | incremental_only | 1 1 1 | 1 1 1 | 15 15 15 | fail → fail |
| k7 | incremental_with_fallback | 2 2 2 | **3** 2 2 | 5 8 8 | fail → fail |

- **Primary (all 7): rescued 0 of 5, broken 0 of 2 → DOES NOT SUPPORT.** Sign
  test p = 1.000. The episode level is the same: 0 episodes gained success and 0 lost it, out of 21.
- Secondary, as pre-declared: the 5 `incremental_only` controllers rescue 0 of 3, and the
  leg-2 failures (k0, k1, k2, k7) rescue 0 of 4. No rescued episode exists, so
  the mechanism check has nothing to check.
- The one movement is **k7 rep 0, 2 → 3 trips** (leg 2 completed in 101.8 s
  after 5 firings), which is still a failure. Kinematically, the same patch cost k7
  trips (4 → 2 and 4 → 3), so the direction on k7 is not stable.

## What it says about Hypothesis 2

The fallback **fired 11-15 times in every failing episode and changed nothing**.
In 11 of 21 pairs the patched leg times match the unpatched ones to within
0.1 s (k1 and k6 match in all three reps, k0 in two, and k0 r1 is off by 0.3 s). So rebuilding the route from scratch on the
controller's own map **returned the route the controller already had**. For
these controllers, the plan at the point of failure was **not stale** in
the sense Hypothesis 2 means. A fresh plan does the same thing, so the failure
lies outside the planner's search state: in the map the controller has built,
its path following or local control, or (for k6) somewhere before the door event
at all.

⚠️ **Read the null as stated in the prereg:** *this* fallback, with *this*
trigger, rescued none of these seven controllers. It does not show that no
fallback could help. It is strong evidence against the specific story "a
D\*-repaired search tree goes stale and a full replan would fix it", because the
full replan was run, repeatedly, and produced the same behaviour.

## Secondary metrics (per episode, paired; descriptive)

- **Collisions:** unchanged in 15 of 21 pairs. k4 rose in all three reps (0→2,
  0→1, 1→6) while still completing 4/4, and k7 fell in reps 0 and 2 (7→4, 7→4).
- **Tracking error** (`track_v_med`) moved by less than 0.01 m/s in every pair
  except k7 r0 (0.108 → 0.085) and k7 r2 (0.096 → 0.106).
- **Planning time per step:** the median step_ms was barely affected. A firing step costs a median of
  6-45 ms for most controllers, and **157-305 ms for k1**, whose cold start has a
  0.30 s D\* budget. The rep-2 p95 values rose for every controller, which
  matches the load below, not the patch.
- **Fallback errors:** 0 in all 21 episodes. Power profile: `performance`
  before and after every episode.

## Disclosures

- **Load.** Reps 0-1 ran at load1 ≈ 6.6-8.9, close to the unpatched Exp 3 rows.
  During rep 2 (episodes #14-#20), Jarvis's voice prototype (llama-server, STT,
  TTS) ran beside the sim, with load1 mean 12-26 and wall time per episode
  ~300-380 s against ~140-200 s. Not stopped (not this project's process). No trip
  count differs between rep 2 and reps 0-1 for any controller, so it does not
  bear on the verdict. It does inflate the rep-2 step_ms p95.
- The trigger thresholds were chosen after reading unpatched traces (prereg §3).
- Local Isaac Sim 6.0.1, `in_container: false`, one hand-authored world.
- Per-step traces are in `demo/out/expC/traces/` and are not committed, to keep
  the repo lean (~0.2 MB each); they are rebuildable by re-running.

## Reproduce

```bash
P="env -u PYTHONHOME -u PYTHONPATH ~/miniforge3/envs/env_isaacsim/bin/python"
$P demo/expC.py check
$P demo/expC.py kinematic                      # sim-free gate
flock /tmp/mindfire-isaac.lock $P demo/expC.py physics
$P demo/expC.py report
```
