# Exp D: slow machine (cgroup CPU limit)

Pre-registration: `prereg/expD.md` @ `4401a1a539ece1f1d2f1453c886f3eb920a9abe3`. CPU limit cpu.max = `200000 100000` (seen by every episode process). Rep 0 condition (seed 0, no offset), door schedule as in Exp 3.

| arm | k | clock | legs pilot | legs Exp 3 r0 | legs D | Δ (D − Exp3) | degraded | trip times D (s) | trip times Exp 3 (s) | contacts D/E3 | track_v med D/E3 | step_ms med D/E3 | step_ms p95 D/E3 | step_ms mean D/E3 | wall s D/E3 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| arm0 | k0 | 1 | 4 | 4 | 4 | 0 | 0 | [18.12, 36.22, 56.24, 29.1] | [18.16, 36.14, 55.58, 32.44] | 19/23 | 0.0713/0.0721 | 5.5/5.41 | 8.28/6.21 | 6.87/5.23 | 279.5/141.0 |
| arm0 | k1 | 0 | 4 | 4 | 4 | 0 | 0 | [20.58, 26.96, 71.44, 43.02] | [20.58, 26.96, 71.44, 43.02] | 0/0 | 0.0672/0.0672 | 1.66/1.18 | 6.01/3.66 | 3.02/1.81 | 323.4/138.3 |
| arm0 | k2 | 0 | 4 | 4 | 4 | 0 | 0 | [20.98, 25.0, 53.7, 36.96] | [20.98, 25.0, 53.7, 36.96] | 7/7 | 0.075/0.075 | 0.6/0.45 | 2.54/1.54 | 1.5/0.89 | 250.0/84.9 |
| arm0 | k3 | 0 | 4 | 4 | 4 | 0 | 0 | [20.56, 19.88, 60.2, 27.88] | [20.56, 19.88, 60.2, 27.88] | 0/0 | 0.0649/0.0649 | 0.75/0.67 | 1.54/1.31 | 1.58/0.99 | 237.3/99.1 |
| arm0 | k4 | 0 | 4 | 4 | 4 | 0 | 0 | [19.82, 20.28, 54.12, 26.76] | [19.82, 20.28, 54.12, 26.76] | 3/3 | 0.0909/0.0909 | 1.03/0.88 | 4.97/4.39 | 1.91/1.22 | 226.4/95.2 |
| arm0 | k5 | 0 | 4 | 4 | 4 | 0 | 0 | [22.46, 23.04, 103.24, 31.06] | [22.46, 23.04, 103.24, 31.06] | 0/0 | 0.073/0.073 | 0.47/0.4 | 2.25/1.77 | 0.93/0.6 | 326.6/128.8 |
| arm0 | k6 | 1 | 4 | 4 | 4 | 0 | 0 | [18.54, 19.2, 45.96, 24.66] | [18.38, 18.88, 40.48, 24.96] | 0/0 | 0.0768/0.0762 | 0.8/0.73 | 1.74/1.5 | 1.24/0.82 | 205.5/82.8 |
| arm0 | k7 | 0 | 2 | 2 | 2 | 0 | 0 | [43.84, 22.06] | [43.84, 22.06] | 35/35 | 0.1033/0.1033 | 0.8/0.74 | 1.36/1.14 | 1.07/0.76 | 321.3/128.9 |
| armB | k0 | 1 | 2 | 2 | 2 | 0 | 0 | [23.92, 21.08] | [23.96, 21.22] | 0/0 | 0.0788/0.077 | 0.84/0.76 | 6.87/3.19 | 1.78/1.14 | 332.7/133.5 |
| armB | k1 | 1 | 2 | 2 | 2 | 0 | 0 | [20.38, 50.1] | [20.38, 50.1] | 1/1 | 0.0266/0.0288 | 0.59/0.51 | 11.78/11.68 | 2.24/1.85 | 339.2/139.8 |
| armB | k2 | 1 | 2 | 2 | 2 | 0 | 0 | [20.74, 72.32] | [20.74, 72.32] | 0/0 | 0.0467/0.0467 | 0.65/0.49 | 6.35/5.98 | 1.71/0.99 | 324.4/108.7 |
| armB | k3 | 0 | 4 | 4 | 4 | 0 | 0 | [20.2, 25.1, 71.4, 30.24] | [20.2, 25.1, 71.4, 30.24] | 0/0 | 0.08/0.08 | 1.66/1.53 | 2.8/2.51 | 2.36/1.57 | 274.0/114.5 |
| armB | k4 | 1 | 4 | 4 | 4 | 0 | 0 | [19.96, 20.04, 50.46, 25.7] | [20.0, 20.12, 40.72, 25.7] | 2/0 | 0.07/0.0686 | 5.24/5.21 | 7.99/5.81 | 5.84/5.25 | 212.7/137.5 |
| armB | k5 | 1 | 2 | 2 | 2 | 0 | 0 | [20.84, 42.26] | [20.84, 48.12] | 0/0 | 0.1002/0.0922 | 0.84/0.72 | 4.3/4.25 | 1.99/1.23 | 324.3/133.8 |
| armB | k6 | 1 | 1 | 1 | 1 | 0 | 0 | [21.34] | [21.34] | 0/0 | 0.2745/0.2745 | 1.32/1.24 | 2.3/2.04 | 1.91/1.35 | 319.2/135.4 |
| armB | k7 | 1 | 2 | 2 | 2 | 0 | 0 | [27.54, 26.6] | [27.54, 26.58] | 7/7 | 0.105/0.1082 | 0.52/0.47 | 2.79/2.12 | 1.39/0.89 | 328.9/131.5 |

## Primary (prereg §4)

| | arm0 | armB | Fisher p (two-sided) |
|---|---|---|---|
| slow machine, all four trips | 7/8 | 2/8 | 0.041 |
| normal speed, Exp 3 rep 0 (reference) | 7/8 | 2/8 | 0.041 |
| degraded vs Exp 3 rep 0 (fewer trips) | 0/8 | 0/8 | 1.000 |

| | clock-limited | not clock-limited | Fisher p (two-sided) |
|---|---|---|---|
| degraded vs Exp 3 rep 0 | 0/9 | 0/7 | 1.000 |
| mean Δ trips | 0.0 | 0.0 | — |

Noise floor at normal speed: Exp 3 rep 0 trips differ from the pilot's in 0/16 controllers. Degraded beyond BOTH normal-speed runs (D < min(pilot, Exp 3)): clock-limited 0/9, not 0/7.

## Planning time per step (secondary, prereg §5)

| group | median of per-controller step_ms med, D | same, Exp 3 r0 | median ratio D/E3 | median ratio p95 |
|---|---|---|---|---|
| clock-limited | 0.84 | 0.73 | 1.11 | 1.16 |
| not clock-limited | 0.8 | 0.74 | 1.17 | 1.19 |
| arm0 | 0.8 | 0.73 | 1.14 | 1.23 |
| armB | 0.84 | 0.74 | 1.11 | 1.12 |

**Pre-registered reading (prereg §6):** not supported: clock-limited controllers were not degraded more often.

Reference: Exp 3 = `demo/out/v2/physics.csv` (51 rows), pilot = `demo/out/physics.csv`. UNDER PHYSICS, one hand-authored world, local Isaac Sim 6.0.1 (in_container false).

## Commits

The `prereg_commit` column holds the hashes as run, on branch `mindfire-expAD` before it was rebased onto main. Those commits stay reachable through the tag **`mindfire-expAD-as-run`** (`c62d17e`). On main the same content is:

| as run | on main | what |
|---|---|---|
| `4401a1a` | `c4237d7` | prereg/expA.md + prereg/expD.md, runner, analysis (before any episode) |
| `42489f2` | `72ae396` | Exp A amendment: rerun 4 simulator failures once, timeout 1200 s |
| `ea8d12b` | `8a3b590` | runner fix: a dead episode has no cpu_max |
| `80d5b0c` | `90d51d2` | Exp D amendment: start-up hang of armB k6, rerun once |
