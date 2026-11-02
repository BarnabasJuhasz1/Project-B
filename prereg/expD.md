# Pre-registration: Exp D, the machine-speed test

Written and committed on 2026-11-02, **before any Exp D episode was run**. The commit hash of this file is recorded in every result row (`prereg_commit`) and in the sweep manifest. Changes after the first episode are dated appends at the end of this file, never edits.

## 1. Question

Does slowing the machine hurt the controllers that limit their planning by the wall clock more than the others?

## 2. Hypothesis

H-D: controllers that bound their per-step planning with the wall clock lose trips on a slowed machine more often than controllers that do not. Because most armB controllers are clock-limited, a slow machine would bias any arm comparison against armB.

**Clock-limited, defined mechanically:** the controller source calls `time.perf_counter(`, `time.time(` or `time.monotonic(` (the same regex as `demo/exp3_report.py`). On the 16 pilot controllers this gives **arm0: 2 of 8** (k0, k6) and **armB: 7 of 8** (all but k3). ⚠️ The brief and `demo/repeatability.py` say "6 of 8 armB"; that count dates from when armB had 7 controllers ("6 of 7"). The code says 7 of 8 and `demo/out/v2/code_features.csv` (`time_budget_calls > 0`) agrees. The mechanical count is used.

## 3. Design

- **Controllers:** `demo/arms/{arm0,armB}/k0..k7.py`, unchanged (16), once each.
- **Condition:** Exp 3 rep 0 exactly (seed 0, offset (0, 0, 0)), original door schedule (the door shuts before leg 2), same apparatus as Exp 3 (`demo/record_isaac.py`, frozen Go2 policy, 180 s budget).
- **The slow machine:** the whole episode process tree runs inside a systemd user scope with **`CPUQuota=200%`** (cgroup v2 `cpu.max = 200000 100000`, i.e. at most 2 CPUs of the laptop's 22 logical CPUs, Intel Core Ultra 9 185H). A normal-speed episode uses about 3 to 4.5 CPUs, so the limit slows it to roughly half speed, about what the 2026-11-02 `low-power` incident did. The command:

  `flock /tmp/mindfire-isaac.lock systemd-run --user --scope -p CPUQuota=200% env -u PYTHONHOME -u PYTHONPATH python demo/exp_ad.py D`

  The runner checks its own `cpu.max` before every episode and refuses to start one unless it is `200000 100000`; each episode process reports the value it saw, and a mismatch stops the sweep and quarantines the row. **The power profile is never changed**: it must read `performance` (ACPI and `powerprofilesctl get`) before every episode, otherwise the sweep pauses.
- **Order:** `exp3.job_order` with seed 42, arms interleaved. 16 episodes. Runner: `demo/exp_ad.py D`.
- **Reference (normal speed):** Exp 3 rep 0, `demo/out/v2/physics.csv` and its traces, read only. The pilot (`demo/out/physics.csv`) is a second normal-speed run used as a noise floor.
- **Recorded per episode:** as Exp A: trips, time per trip, collisions, tracking error, planning time per step (median, p95, max; every step in the trace), load average, cpu.max, power profile.

## 4. Primary metric, tests

- **Per-controller change:** Δtrips = trips(D) − trips(Exp 3 rep 0). A controller is **degraded** if Δtrips < 0.
- **Test 1 (H-D):** Fisher exact, two-sided, degraded vs not, clock-limited (9) vs not clock-limited (7).
- **Test 2 (per arm):** Fisher exact, two-sided, arm0 vs armB on all-four-trip success under D; and arm0 vs armB on degraded counts.

## 5. Secondary (descriptive)

- Planning time per step: per controller, D vs Exp 3 rep 0 (median, p95, mean from the trace); median ratio per group (clock-limited / not, arm0 / armB). Manipulation check: the non-clock-limited controllers' step time should rise under the limit (their work per step is fixed). Clock-limited controllers' step time is capped by their budget, so they show the slowdown as less planning per step rather than longer steps.
- Noise floor: how many controllers' trips already differ between the pilot and Exp 3 rep 0 at normal speed; degraded beyond both (trips(D) < min(pilot, Exp 3)).
- Trip times, collisions, tracking error, D vs Exp 3.

## 6. Pass criteria and reading, fixed in advance

| result | reading |
|---|---|
| degraded share higher in clock-limited and Test 1 p < 0.05 | **H-D supported**: slowing hurts the clock-limited controllers more |
| higher in clock-limited, p ≥ 0.05 | direction as predicted, not significant; counts only |
| not higher | **H-D not supported** |

## 7. Analysis script and power

`demo/exp_ad_report.py`, committed with this file; it writes `demo/out/expD/SUMMARY.md`. With 9 vs 7 controllers, Fisher can only reach p < 0.05 for near-complete separation (e.g. 6/9 vs 0/7 degraded gives p ≈ 0.011; 5/9 vs 0/7 gives p ≈ 0.034; 4/9 vs 0/7 gives p ≈ 0.088). A null is not evidence that speed does not matter.

## 8. Known limitations

- One run per controller, one world, local Isaac Sim, `in_container: false`.
- The CPU limit slows the simulator and the controller together (they share the process); sim time is unaffected, wall time per step is not.
- Arm and clock-limitation are almost confounded (7/8 vs 2/8), so Test 1 and Test 2 cannot fully separate "armB" from "clock-limited".
- Exp 3 rep 0 differed from the pilot in 6 of 17 controllers at normal speed, mostly clock-limited ones; the noise-floor comparison is reported for that reason.

## Amendment 2026-11-02 (during the sweep, before any result was analysed)

Episode #9 (armB k6) deadlocked at simulator start-up: 0.48 s of CPU in 9 minutes, 64 MB resident, blocked on a futex before Isaac Sim booted. That is the same start-up hang seen 3 times in Exp A. The hung process was killed by hand. The runner recorded no result, and the CPU check then stopped the sweep because a dead process reports no `cpu.max`. That bug is fixed (`ea8d12b`); the row is kept in `demo/out/expD/physics_quarantine.csv`. The episode is **rerun once** on resume, under the same CPU limit, and the runner timeout drops to 1200 s. A rerun that also fails is reported as missing, not as 0 trips, because a hang is a simulator failure and not a controller outcome. Nothing else changes.
