# Exp 3 summary — 51 episodes

Success = all four legs. UNDER PHYSICS, local machine, in_container false. One hand-authored world. Descriptive only.

## 1. Rep 0 vs demo/out/physics.csv: 11/17 exact

| arm | k | wall-clock budget | verdict | legs v2 / pilot | 4-leg success same? | detail |
|---|---|---|---|---|---|---|
| (base) | reference | no | MATCH | 3 / 3 | yes |  |
| arm0 | k0 | yes | DIFFER | 4 / 4 | yes | legs 4 vs 4, times [18.16, 36.14, 55.58, 32.44] vs [18.14, 42.22, 57.44, 33.28], t 142.3 vs 151.1 |
| arm0 | k1 | no | MATCH | 4 / 4 | yes |  |
| arm0 | k2 | no | MATCH | 4 / 4 | yes |  |
| arm0 | k3 | no | MATCH | 4 / 4 | yes |  |
| arm0 | k4 | no | MATCH | 4 / 4 | yes |  |
| arm0 | k5 | no | MATCH | 4 / 4 | yes |  |
| arm0 | k6 | yes | MATCH | 4 / 4 | yes |  |
| arm0 | k7 | no | MATCH | 2 / 2 | yes |  |
| armB | k0 | yes | DIFFER | 2 / 2 | yes | legs 2 vs 2, times [23.96, 21.22] vs [24.26, 21.12], t 180.0 vs 180.0 |
| armB | k1 | yes | MATCH | 2 / 2 | yes |  |
| armB | k2 | yes | DIFFER | 2 / 2 | yes | legs 2 vs 2, times [20.74, 72.32] vs [20.86, 124.58], t 180.0 vs 180.0 |
| armB | k3 | no | MATCH | 4 / 4 | yes |  |
| armB | k4 | yes | DIFFER | 4 / 4 | yes | legs 4 vs 4, times [20.0, 20.12, 40.72, 25.7] vs [20.0, 20.12, 41.98, 25.84], t 106.6 vs 108.0 |
| armB | k5 | yes | DIFFER | 2 / 2 | yes | legs 2 vs 2, times [20.84, 48.12] vs [21.08, 23.26], t 180.0 vs 180.0 |
| armB | k6 | yes | MATCH | 1 / 1 | yes |  |
| armB | k7 | yes | DIFFER | 2 / 2 | yes | legs 2 vs 2, times [27.54, 26.58] vs [35.2, 26.6], t 180.0 vs 180.0 |

## 2. Per arm

| arm | controllers | episodes success | maj-of-3 success | all 3 reps agree | rep 0 | rep 1 | rep 2 | mean legs |
|---|---|---|---|---|---|---|---|---|
| armB | 8 | 6/24 | 2/8 | 8/8 | 2/8 | 2/8 | 2/8 | 2.29 |
| arm0 | 8 | 14/24 | 4/8 | 4/8 | 7/8 | 3/8 | 4/8 | 3.12 |
| (base) | 1 | 0/3 | 0/1 | 1/1 | 0/1 | 0/1 | 0/1 | 1.67 |

Per controller (legs in rep 0 / 1 / 2):

- **armB**: k0 2/2/2, k1 2/2/2, k2 2/2/2, k3 4/4/4, k4 4/4/4, k5 2/1/1, k6 1/1/1, k7 2/2/2
- **arm0**: k0 4/4/4, k1 4/2/4, k2 4/3/3, k3 4/4/4, k4 4/2/2, k5 4/2/2, k6 4/4/4, k7 2/0/1
- **(base)**: reference 3/1/1

## 3. Tracking error: commanded vs realised displacement per 50 Hz step

e_k = |p_{k+1} - (p_k + R(yaw_k)[vx, vy] dt)|, i.e. how far the frozen gait lands from where the held Twist would have put a unicycle. Per episode: the median over its steps.

| arm | k | rep | legs | median e (mm/step) | mean e (mm/step) | median err_v (m/s) | median abs err_wz (rad/s) |
|---|---|---|---|---|---|---|---|
| armB | k0 | 0 | 2 | 1.54 | 2.27 | 0.077 | 0.099 |
| armB | k0 | 1 | 2 | 1.54 | 2.31 | 0.077 | 0.102 |
| armB | k0 | 2 | 2 | 1.56 | 2.33 | 0.078 | 0.099 |
| armB | k1 | 0 | 2 | 0.58 | 0.89 | 0.029 | 0.854 |
| armB | k1 | 1 | 2 | 0.05 | 0.61 | 0.002 | 0.950 |
| armB | k1 | 2 | 2 | 0.15 | 0.74 | 0.007 | 0.950 |
| armB | k2 | 0 | 2 | 0.94 | 1.39 | 0.047 | 0.929 |
| armB | k2 | 1 | 2 | 1.22 | 1.54 | 0.061 | 0.246 |
| armB | k2 | 2 | 2 | 1.08 | 1.46 | 0.054 | 0.506 |
| armB | k3 | 0 | 4 | 1.60 | 2.68 | 0.080 | 0.106 |
| armB | k3 | 1 | 4 | 1.55 | 2.37 | 0.077 | 0.104 |
| armB | k3 | 2 | 4 | 1.54 | 2.47 | 0.077 | 0.103 |
| armB | k4 | 0 | 4 | 1.37 | 1.64 | 0.069 | 0.096 |
| armB | k4 | 1 | 4 | 1.37 | 1.68 | 0.069 | 0.098 |
| armB | k4 | 2 | 4 | 1.46 | 1.83 | 0.073 | 0.098 |
| armB | k5 | 0 | 2 | 1.84 | 2.26 | 0.092 | 0.991 |
| armB | k5 | 1 | 1 | 3.56 | 3.62 | 0.178 | 0.998 |
| armB | k5 | 2 | 1 | 3.77 | 3.82 | 0.189 | 0.997 |
| armB | k6 | 0 | 1 | 5.49 | 5.01 | 0.275 | 1.000 |
| armB | k6 | 1 | 1 | 5.36 | 4.87 | 0.268 | 1.000 |
| armB | k6 | 2 | 1 | 5.49 | 5.01 | 0.275 | 1.000 |
| armB | k7 | 0 | 2 | 2.16 | 3.33 | 0.108 | 0.093 |
| armB | k7 | 1 | 2 | 2.22 | 3.27 | 0.111 | 0.099 |
| armB | k7 | 2 | 2 | 1.93 | 2.87 | 0.096 | 0.094 |
| arm0 | k0 | 0 | 4 | 1.44 | 2.12 | 0.072 | 0.139 |
| arm0 | k0 | 1 | 4 | 1.50 | 2.28 | 0.075 | 0.146 |
| arm0 | k0 | 2 | 4 | 1.58 | 2.97 | 0.079 | 0.154 |
| arm0 | k1 | 0 | 4 | 1.34 | 1.69 | 0.067 | 0.077 |
| arm0 | k1 | 1 | 2 | 1.60 | 2.06 | 0.080 | 0.085 |
| arm0 | k1 | 2 | 4 | 1.35 | 1.78 | 0.068 | 0.080 |
| arm0 | k2 | 0 | 4 | 1.50 | 2.05 | 0.075 | 0.115 |
| arm0 | k2 | 1 | 3 | 1.60 | 2.15 | 0.080 | 0.112 |
| arm0 | k2 | 2 | 3 | 1.59 | 2.05 | 0.079 | 0.108 |
| arm0 | k3 | 0 | 4 | 1.30 | 1.59 | 0.065 | 0.092 |
| arm0 | k3 | 1 | 4 | 1.30 | 1.69 | 0.065 | 0.092 |
| arm0 | k3 | 2 | 4 | 1.32 | 1.73 | 0.066 | 0.090 |
| arm0 | k4 | 0 | 4 | 1.82 | 2.43 | 0.091 | 0.094 |
| arm0 | k4 | 1 | 2 | 0.14 | 1.08 | 0.007 | 0.994 |
| arm0 | k4 | 2 | 2 | 0.76 | 1.58 | 0.038 | 0.173 |
| arm0 | k5 | 0 | 4 | 1.46 | 2.30 | 0.073 | 0.107 |
| arm0 | k5 | 1 | 2 | 2.38 | 4.58 | 0.119 | 0.125 |
| arm0 | k5 | 2 | 2 | 2.01 | 3.97 | 0.101 | 0.121 |
| arm0 | k6 | 0 | 4 | 1.52 | 1.91 | 0.076 | 0.084 |
| arm0 | k6 | 1 | 4 | 1.44 | 2.04 | 0.072 | 0.086 |
| arm0 | k6 | 2 | 4 | 1.55 | 2.12 | 0.078 | 0.088 |
| arm0 | k7 | 0 | 2 | 2.07 | 3.06 | 0.103 | 0.109 |
| arm0 | k7 | 1 | 0 | 1.36 | 2.39 | 0.068 | 0.035 |
| arm0 | k7 | 2 | 1 | 2.52 | 3.44 | 0.126 | 0.127 |
| (base) | reference | 0 | 3 | 1.42 | 1.82 | 0.071 | 0.136 |
| (base) | reference | 1 | 1 | 1.21 | 1.78 | 0.061 | 0.100 |
| (base) | reference | 2 | 1 | 1.12 | 1.72 | 0.056 | 0.115 |

Per arm (episode medians):

| arm | episodes | median e mm/step | IQR | median err_v m/s | IQR |
|---|---|---|---|---|---|
| armB | 24 | 1.55 | [1.33, 2.18] | 0.077 | [0.067, 0.109] |
| arm0 | 24 | 1.50 | [1.35, 1.60] | 0.075 | [0.067, 0.080] |
| (base) | 3 | 1.21 | [1.17, 1.32] | 0.061 | [0.058, 0.066] |

arm0 vs armB, Mann-Whitney U two-sided: **controller level** (mean of each controller's episode medians, n = 8 vs 8) U = 26.0, p = 0.574; episode level (n = 24 vs 24, NOT independent -- reps of one controller are clustered) U = 249.0, p = 0.427.

## 4. Machine

- wall time: 1.84 h over 51 episodes (median 134 s, range 83-175 s)
- load1_before: median 7.19, range 1.88-11.80 (22 cores)
- load1_mean: median 7.47, range 3.66-11.85 (22 cores)
- load1_max: median 9.33, range 5.86-14.12 (22 cores)
- power profile per episode: {'(not recorded)': 22, 'performance': 29}

## 5. Quarantined low-power rows vs their reruns

| arm | k | rep | low-power legs (wall s) | rerun legs (wall s) | leg times low-power | leg times rerun |
|---|---|---|---|---|---|---|
| armB | k5 | 1 | 2 (341.6) | 1 (156.8) | [21.16, 23.72] | [21.22] |
| armB | k3 | 1 | 4 (241.6) | 4 (117.6) | [20.22, 25.08, 54.72, 31.76] | [20.22, 25.08, 54.72, 31.76] |
| arm0 | k4 | 1 | 2 (310.9) | 2 (149.4) | [20.36, 20.26] | [20.36, 20.26] |
| arm0 | k7 | 1 | 0 (292.4) | 0 (148.4) | [] | [] |
| armB | k2 | 2 | 2 (234.8) | 2 (156.0) | [20.76, 89.42] | [20.76, 89.42] |

