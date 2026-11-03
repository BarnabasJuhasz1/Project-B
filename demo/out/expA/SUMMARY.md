# Exp A: open maze (door never shuts)

Pre-registration: `prereg/expA.md` @ `4401a1a539ece1f1d2f1453c886f3eb920a9abe3`. 48 episodes. Success = all four trips; a controller succeeds if it does so in at least 2 of its 3 reps.

| arm | k | clock | open legs r0/r1/r2 | open maj | blocked (Exp 3) legs r0/r1/r2 | blocked maj | trip times, first rep (s) | contacts | track_v med | step_ms med |
|---|---|---|---|---|---|---|---|---|---|---|
| arm0 | k0 | 1 | 4/4/4 | 1 | 4/4/4 | 1 | [18.14, 36.06, 24.14, 20.8] | 22 | 0.07 | 5.38 |
| arm0 | k1 | 0 | 4/4/4 | 1 | 4/2/4 | 1 | [20.58, 26.96, 23.2, 22.12] | 0 | 0.06 | 1.09 |
| arm0 | k2 | 0 | 4/4/4 | 1 | 4/3/3 | 0 | [20.98, 25.0, 29.98, 25.04] | 5 | 0.08 | 0.56 |
| arm0 | k3 | 0 | 4/4/4 | 1 | 4/4/4 | 1 | [20.56, 19.88, 31.72, 21.22] | 1 | 0.06 | 0.71 |
| arm0 | k4 | 0 | 4/4/4 | 1 | 4/2/2 | 0 | [19.82, 20.28, 20.1, 20.28] | 0 | 0.09 | 0.96 |
| arm0 | k5 | 0 | 4/4/4 | 1 | 4/2/2 | 0 | [22.46, 23.04, 23.38, 22.84] | 0 | 0.07 | 0.4 |
| arm0 | k6 | 1 | 4/4/4 | 1 | 4/4/4 | 1 | [18.38, 18.88, 19.0, 19.1] | 0 | 0.07 | 0.77 |
| arm0 | k7 | 0 | 3/0/1 | 0 | 2/0/1 | 0 | [43.84, 22.06, 34.88] | 84 | 0.08 | 0.78 |
| armB | k0 | 1 | 4/4/4 | 1 | 2/2/2 | 0 | [23.96, 21.22, 23.42, 21.12] | 0 | 0.05 | 0.77 |
| armB | k1 | 1 | 4/4/4 | 1 | 2/2/2 | 0 | [20.38, 50.1, 25.98, 46.12] | 0 | 0.05 | 0.54 |
| armB | k2 | 1 | 2/3/3 | 0 | 2/2/2 | 0 | [20.74, 72.32] | 0 | 0.05 | 0.61 |
| armB | k3 | 0 | 4/4/4 | 1 | 4/4/4 | 1 | [20.2, 25.1, 21.88, 24.58] | 0 | 0.07 | 1.45 |
| armB | k4 | 1 | 4/4/4 | 1 | 4/4/4 | 1 | [20.0, 20.14, 19.7, 20.02] | 0 | 0.07 | 5.05 |
| armB | k5 | 1 | 4/4/4 | 1 | 2/1/1 | 0 | [20.84, 26.96, 23.14, 23.42] | 0 | 0.08 | 0.77 |
| armB | k6 | 1 | 1/1/1 | 0 | 1/1/1 | 0 | [21.34] | 0 | 0.27 | 1.28 |
| armB | k7 | 1 | 4/4/4 | 1 | 2/2/2 | 0 | [27.54, 26.58, 35.14, 26.78] | 21 | 0.08 | 0.47 |

## Primary (prereg §4)

| | arm0 | armB | Fisher p (two-sided) |
|---|---|---|---|
| open maze, controllers (majority of 3) | 7/8 | 6/8 | 1.000 |
| open maze, episodes (descriptive) | 21/24 | 18/24 | 0.461 (episodes not independent) |
| blocked maze, Exp 3 (reference) | 4/8 | 2/8 | 0.608 |

## Open vs blocked, paired per controller (secondary)

- arm0: succeeds open only 3, blocked only 0, both 4, neither 1 (exact McNemar p = 0.250, n = 8)
- armB: succeeds open only 4, blocked only 0, both 2, neither 2 (exact McNemar p = 0.125, n = 8)

**Pre-registered reading (prereg §6):** armB succeed in the open maze: their failures are tied to the blocked route (replanning).

Reference: Exp 3 = `demo/out/v2/physics.csv` (51 rows), pilot = `demo/out/physics.csv`. UNDER PHYSICS, one hand-authored world, local Isaac Sim 6.0.1 (in_container false).

## Commits

The `prereg_commit` column holds the hashes as run, on branch `mindfire-expAD` before it was rebased onto main. Those commits stay reachable through the tag **`mindfire-expAD-as-run`** (`c62d17e`). On main the same content is:

| as run | on main | what |
|---|---|---|
| `4401a1a` | `c4237d7` | prereg/expA.md + prereg/expD.md, runner, analysis (before any episode) |
| `42489f2` | `72ae396` | Exp A amendment: rerun 4 simulator failures once, timeout 1200 s |
| `ea8d12b` | `8a3b590` | runner fix: a dead episode has no cpu_max |
| `80d5b0c` | `90d51d2` | Exp D amendment: start-up hang of armB k6, rerun once |
