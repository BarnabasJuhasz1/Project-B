---
created: 2026-11-02
status: FROZEN AT THE COMMIT THAT ADDS THIS FILE. Changes are dated appends to §9.
---

# Exp C — full-replan fallback ablation (review item E7b): pre-registration

> ⚠️ **AN INTERVENTION ON MODEL-WRITTEN CODE.** Every controller this experiment
> runs is a model's file **with a block we appended**. The patched copies are
> **not** samples of any arm. They never enter an arm count, a Fisher test, a
> pass rate, `demo/arms/` or any Exp 1-4 number. The originals are untouched and
> their sha256 are checked before every run (`demo/expC.py:check_originals`).

Approved by Barna on 2026-11-02. Committed **before the first physics episode**;
`demo/expC.py physics` refuses to start until this file is in git history, and
writes the hash of the commit that added it into every result row
(`prereg_commit`).

## 1. Hypothesis

Report Hypothesis 2: *a plan that is only ever repaired incrementally goes stale
when the gait does not follow commands exactly, and that, not the paper as such,
is why most with-paper (armB) controllers fail under physics while succeeding
kinematically.* Exp 4 showed it **correlationally** (`demo/EXP4_PILOT.md`).
This is the **causal** test: add the missing piece and see whether the failures
go away.

**H-C (directional, stated before the data):** giving each incremental-repair
armB controller one uniform full-replan fallback, triggered when the robot stops
making progress toward its current target, **raises its physics success**, and
does not break a controller that already succeeds.

## 2. Controllers

The seven armB controllers that Exp 4 codes as having incremental repair
(consensus labels, both raters agreed on planner type for all 16 files):

| k | Exp 4 planner label | where the route lives | unpatched Exp 3 legs, reps 0/1/2 |
|---|---|---|---|
| k0 | `incremental_only` | `self._fields` (D\* `_Field` per goal) | 2 / 2 / 2 |
| k1 | `incremental_only` | `self.planners` (`_DStar` per goal) | 2 / 2 / 2 |
| k2 | `incremental_with_fallback` | `self.planners` (`_DStar` per goal) | 2 / 2 / 2 |
| k3 | `incremental_only` | `self.fields` (swept h per goal) | 4 / 4 / 4 |
| k4 | `incremental_only` | `self.fields` (relaxed h per goal) | 4 / 4 / 4 |
| k6 | `incremental_only` | `self.h` + `self.hcache` | 1 / 1 / 1 |
| k7 | `incremental_with_fallback` | `self.planners` (`_DStar` per goal) | 2 / 2 / 2 |

⚠️ **Only five of the seven are `incremental_only`.** k2 already rebuilds a
tree from scratch when its backpointer trace is corrupted (rate-limited to once
per 3 s), and k7 when its goal cell turns lethal (checked every 37 steps).
Neither is a no-progress trigger, so both get the same patch, but they are a
**separate stratum** in §5. armB k5 is coded `full_periodic` and is not patched.
All seven were confirmed in the code (`demo/expC.py:HOOKS` names the attribute).

So, unpatched, by the majority-of-3 rule: **5 controllers fail (k0, k1, k2, k6,
k7) and 2 succeed (k3, k4)**, and every rep agrees with its controller's
majority. The rows are `demo/out/v2/physics.csv` (Exp 3, complete, 51 rows,
`arm == armB`), already observed when this file was written; only the patched
outcome is unknown. A rescue is therefore possible for at most 5 controllers and
a break for at most 2.

## 3. The intervention — one rule, the same everywhere

Each original is copied **byte for byte** to `demo/interventions/expC/armB/kN.py`
and a block is **appended** (`demo/interventions/expC/diffs/armB_kN.diff`, one
readable diff per controller; every diff is additions only). The block:

- **Trigger** (`_ExpCNoProgress`, identical text in all seven):
  `d` = straight-line distance from `obs.pose_estimate` to `obs.goal`, in metres,
  on **sim time** `obs.t`. Progress = `d` reaching a new best that is at least
  **`PROGRESS_M = 0.5` m** below the previous best. The trigger fires when no
  progress has been made for **`WINDOW_S = 10.0` s**. It restarts (best := current
  `d`, clock := now) after every firing and whenever `obs.goal` moves by more
  than 0.5 m (a new leg).
- **Action:** discard the controller's cached **route** state — its D\* trees or
  cost-to-goal fields, for every goal — and let the model's own code rebuild it
  **from scratch on its current map, in the same step**, exactly as it does for a
  goal it has never seen. The map, the cost grid, penalties, recovery state and
  every other attribute are kept. The only per-controller text is the 1-5 line
  `_expc_full_replan` body naming where that controller keeps its route
  (`HOOKS`; k6's re-runs the no-cache branch of its own `_switch_goal` and sets
  its own `pending_burst`).
- **Nothing else.** When the trigger does not fire, the patched `step()` is
  `super().step(obs)`: no rng draw, no attribute the model's code reads, no
  change to the command.

**How the two numbers were chosen, disclosed:** after reading the unpatched
Exp 3 traces (reps 0-1). Successful leg-2 detours go **20-50 s** without 0.5 m of
straight-line progress, because backing out of the cul-de-sac moves the robot
*away* from the goal. Failing armB episodes stall for **86-160 s**. 10 s is short
enough to fire many times inside a failing stall and leave time to recover in the
180 s budget. The trigger was **not** tuned to stay quiet on legitimate detours:
it will fire ~2-5 times on a successful detour, and a full replan on a correct map
returns the same route, so a firing that hurts is itself evidence about the
controller.

## 4. Runs

- **Physics:** each patched copy, Isaac Sim 6.0.1 with the frozen Go2 policy,
  `demo/record_isaac.py`, **3 reps with Exp 3's conditions** (`demo/exp3.py
  REP_CONDITIONS`: rep 0 = seed 0, offset 0; rep 1 = seed 1, (−0.088, 0.077,
  0.023); rep 2 = seed 2, (0.038, −0.069, 0.066)). 21 episodes, rep-major order
  (`exp3.job_order`, seed 3). One Isaac process at a time
  (`flock /tmp/mindfire-isaac.lock`), and the run pauses unless
  `powerprofilesctl get` says `performance` before each episode.
- **Comparison:** each patched episode is **paired with its own unpatched Exp 3
  episode** — same controller, same rep, same seed and start offset —
  `demo/out/v2/physics.csv`, `arm == armB`.
- **Kinematic gate (run before this commit, §8):** every patched copy, 3 reps,
  `demo/rollout.py`. Pass = no crash, no exception inside the block, and every
  patched episode with zero firings reproduces the original trajectory exactly.

**Recorded per episode** (`demo/out/expC/physics.csv` + a per-step gzip trace):
trips completed (`legs`), time per trip (`leg_times`), collisions (`contacts`),
tracking error (`track_v_med`, `track_v_p95`, `track_wz_med`), planning time per
step (`step_ms_med/p95/max`: the controller's own `step()` wall time, the
fallback included), fallback firings (`fires`, `fire_times`, `fire_step_ms`,
`expc_errors`), load average, power profile, controller sha, `prereg_commit`.

## 5. Primary metric, analysis and pass criteria

- **Episode success:** all four trips within 180 s (`legs == 4`).
- **Controller success:** success in the **majority of its 3 reps** (Exp 3's rule).
- **Primary analysis:** a paired per-controller comparison over **all seven**:
  *rescued* = unpatched failure → patched success, *broken* = unpatched success →
  patched failure. Code: `demo/expC.py:analyse`, `verdict`, run by
  `python demo/expC.py report`.

| outcome | verdict |
|---|---|
| broken ≥ 1 | **HARMS** — the fallback turns a working controller into a failing one; H-C is not supported whatever the rescues |
| broken = 0, rescued ≥ 3 | **SUPPORTS** H-C |
| broken = 0, rescued = 2 | **INCONCLUSIVE** |
| broken = 0, rescued ≤ 1 | **DOES NOT SUPPORT** H-C, for this trigger |

- **Reported beside it, never instead of it:** the exact two-sided sign test
  (McNemar exact) on the discordant controllers. ⚠️ **With 7 controllers it
  cannot reach p < 0.05 unless at least 6 are discordant in one direction**
  (5 of 5 gives p = 0.0625), so the verdict table above is the decision and the
  p-value is descriptive.
- **Pre-declared secondary readings (labelled secondary):**
  1. the same analysis on the **5 `incremental_only`** controllers;
  2. the same analysis on the **leg-2 failures** (k0, k1, k2, k7: unpatched they
     fail *after* the door shuts, which is where a stale plan would bite). H-C's
     mechanism predicts the rescues land here. ⚠️ **k6 fails on leg 1 with the
     door still open**, so Hypothesis 2 does not predict a k6 rescue, and a k6
     rescue would point to a stuck-recovery effect rather than staleness;
  3. episode level, 21 pairs: rescued/broken episodes and the exact sign test.
     ⚠️ Reps of one controller are **not independent**; this is not a test of H-C;
  4. per episode, paired: legs completed, leg-2 time, contacts, tracking error,
     step_ms median/p95/max, firings per episode, and the firing count in
     successful vs failed episodes. Descriptive.
  5. **mechanism check** for every rescued episode: did leg 2 complete within
     30 s after a firing? If rescues happen with no firing near leg 2, the
     rescue is not the fallback.

## 6. What this cannot show

- It is **our** code inside the model's. A rescue shows the model's planner
  *could* have worked with a fallback; it says nothing about the paper's effect
  on generation, and it is never an armB number.
- **One trigger.** A null is "this fallback did not help", not "no fallback
  would". The thresholds were chosen after seeing unpatched traces (§3).
- One hand-authored world; 7 controllers; local Isaac Sim, `in_container: false`.
- **Load and wall-clock budgets.** Most of these controllers bound planning with
  `time.perf_counter()` (plan R5). The unpatched rows were run during the Exp 3
  sweep, part of which ran beside another agent's jobs (load1 up to ~12 on 22
  cores, noted in `demo/out/v2/sweep.log`). Load is logged per episode on both
  sides; a rescue or a break that coincides with a large load difference is
  flagged in the results.
- A from-scratch rebuild under a 5-45 ms per-step budget can leave a controller
  **without a route for a few steps**; its own cold-start behaviour then drives.
  That is part of the intervention, not a confound, and step_ms shows its cost.

## 7. Files

| file | what |
|---|---|
| `demo/expC.py` | the design as data (`EXP4_LABEL`, `ORIGINAL_SHA256`, `PROGRESS_M`, `WINDOW_S`, `HOOKS`, the block), `make`, `check`, `kinematic`, `physics`, `report` |
| `demo/interventions/expC/armB/k*.py` | the seven patched copies — **intervention, not samples** |
| `demo/interventions/expC/diffs/armB_k*.diff` | one readable diff per controller |
| `demo/test_expC.py` | invariants: hashes, copies == build, the block identical across copies but for the hook, trigger behaviour, copies unreachable from any arm, the analysis |
| `demo/record_isaac.py` | +5 lines: logs `controller.expc_fallback_fires` into `RESULT_JSON` after the episode; absent for every other controller |
| `demo/out/expC/kinematic.csv`, `demo/out/expC/physics.csv` | results |

## 8. Kinematic gate result (before this commit)

`python demo/expC.py kinematic` → `demo/out/expC/kinematic.csv`, 2026-11-02,
7 controllers × 3 reps × (original, patched). ⚠️ Run with `nice -n 19`,
`CPUQuota=100%`, one BLAS thread, **while other agents' Isaac runs held the
lock**; the wall-clock-budgeted controllers are sensitive to that, so these
numbers are a gate, not a result.

| k | original legs r0/r1/r2 | patched legs | firings | patched trajectory identical to original | errors in block |
|---|---|---|---|---|---|
| k0 | 2 2 2 | 2 2 2 | 12 12 12 | 0/3 | 0 |
| k1 | 4 1 2 | 4 1 2 | 2 14 12 | 2/3 | 0 |
| k2 | 4 4 4 | 4 4 4 | 5 4 3 | 0/3 | 0 |
| k3 | 4 4 4 | 4 4 4 | 3 2 2 | 3/3 | 0 |
| k4 | 4 4 4 | 4 4 4 | 1 1 1 | 0/3 | 0 |
| k6 | 4 4 4 | 4 4 4 | 5 4 3 | 0/3 | 0 |
| k7 | 4 4 4 | **4 2 3** | 4 8 6 | 0/3 | 0 |

**Gate: PASS.** No crash, no exception inside the block, every copy loads
through the import gate.

- ⚠️ **The zero-firing identity check was never exercised**: every patched
  episode fired at least once (as §3 predicts for the leg-2 detour). Identity is
  shown the other way round instead: k3 (3/3) and k1 (2/3) **fired and still
  reproduced the original trajectory exactly**, i.e. the from-scratch rebuild
  returned the same route. So the block changes nothing but the route state.
- ⚠️ **Kinematically, the fallback HARMS k7** (4 → 2 and 4 → 3 legs in reps 1-2),
  firing from ~68 s, during the leg-2 detour. No exception: it is k7's own
  from-scratch path (a new `_DStar`, 40 ms per step until it has a route) that
  runs after each firing. k7 fails under physics unpatched (2/2/2), so in the
  primary analysis it can only be rescued, not broken; a k7 non-rescue is
  expected on this evidence, not a surprise.
- ⚠️ **Kinematically, the fallback rescues nothing**: k0 fails before and after
  (12 firings per episode), k1's rep 1-2 failures are unchanged. The kinematic
  simulator cannot test Hypothesis 2 (its premise is the physics/kinematics gap),
  so this neither supports nor refutes H-C; it is recorded so nobody reads a
  physics rescue as something the kinematic loop already showed.

## 9. Amendments

None.
