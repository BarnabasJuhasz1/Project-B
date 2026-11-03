# Exp F results: the 16 pilot controllers on a second maze (maze2)

Pre-registration: `prereg/expF.md`, **commit `d95d401f5c352475e8db6236b1276cbad6ed33a6`** (`d95d401` on the Exp F branch, the hash every manifest records; the rebase onto origin/main made it **`57df798`** there, with `prereg/expF.md` byte-identical)
(also in `manifest-kinematic.json` and `manifest-physics.json`, field
`prereg_commit`). Analysis: `python demo/expF.py report`, as committed.
Local runs, `in_container: false`. Every physics episode started with
`powerprofilesctl get` = `performance` and the AC adapter online; all 48
rows log `performance` before and after. The test suite ran after the sweep, under the lock: 1,016 passed and 1 failed (`compose/tests/test_no_papers.py::test_no_card_vocabulary_reaches_the_rendered_node_block`, which fails the same way on origin/main).

## The maze

`demo/maze2.py`, picture `demo/out/expF/maze2.png`. Scene
`scenes/demo-maze2.usd` (18.8 MB, sha256 `a2790d74d0a5009c…`). It is not
committed because it is rebuilt by `python demo/export_demo_world.py --maze maze2`.
`scene-check-maze2.json`: 19/19 wall blocks and the door match `maze2.py`
(the same check passes on `demo-patrol.usd`, `scene-check-patrol.json`). In all
48 physics traces the door is open on legs 0–1 and shut on legs 2–3.

## Primary: Isaac Sim physics, controller success (all 4 trips in ≥ 2 of 3 reps)

| arm | k0 | k1 | k2 | k3 | k4 | k5 | k6 | k7 | success | Wilson 95 % |
|---|---|---|---|---|---|---|---|---|---|---|
| arm0 | 4 2 2 | 4 4 4 | 4 4 4 | 4 4 4 | 4 4 4 | 4 4 4 | 4 4 4 | 2 4 1 | **6/8** | 0.41–0.93 |
| armB | 4 4 4 | 2 2 2 | 4 4 4 | 4 4 4 | 4 4 4 | 2 2 2 | 1 1 1 | 4 4 4 | **5/8** | 0.31–0.86 |

(trips per rep 0 1 2)

**Fisher exact, two-sided: p = 1.000.** Pre-registered reading: arm0 > armB
by one controller, p ≥ 0.05, so **not a replication**. The pilot effect
(7/8 vs 2/8) does not carry over to maze2. With n = 8 a non-significant
result is not evidence that the arms are equal.

arm0 k6 rep 2 died during simulator start-up (no result line, 27 s in). It got
the one pre-registered retry (`note` = `RETRY…` in `physics.csv`), which
scored 4/4. k6 had already succeeded in reps 0 and 1, so the retry could not
change any verdict.

## Secondary

| | arm0 | armB | Fisher p |
|---|---|---|---|
| S1 physics, rep 0 only (the pilot's format) | 7/8 | 5/8 | 0.569 |
| S2 kinematic, ≥ 2 of 3 reps | 8/8 | 7/8 | 1.000 |
| S2 kinematic, rep 0 only | 8/8 | 7/8 | 1.000 |

S3, descriptive, per episode (physics / kinematic):

| | arm0 | armB |
|---|---|---|
| mean trips | 3.62 / 3.96 | 3.12 / 3.71 |
| leg 2 (the shut-door trip) completed | 20/24 / 24/24 | 15/24 / 21/24 |
| leg 2 time, median | 29.2 s / 26.2 s | 30.4 s / 28.0 s |
| collisions per metre | 0.042 / 0.045 | 0.009 / 0.016 |
| tracking error, median of episode medians: speed, yaw rate | 0.070 m/s, 0.091 rad/s | 0.069 m/s, 0.111 rad/s |
| planning time per step, median of episode medians (p95), worst step | 1.19 ms (5.59), 115 ms | 1.31 ms (6.07), 321 ms |
| controllers whose 3 reps agree, physics | 6/8 | 8/8 |

Mean load average during physics episodes: 7.7, on a machine also running other agents' jobs.

## Exploratory: next to maze 1 (Exp 3, `demo/out/v2/physics.csv`)

| | maze 1 (Exp 3) | maze2 (Exp F) |
|---|---|---|
| ≥ 2 of 3 reps | arm0 4/8, armB 2/8, p = 0.608 | arm0 6/8, armB 5/8, p = 1.000 |
| rep 0 only | arm0 7/8, armB 2/8, p = 0.041 | arm0 7/8, armB 5/8, p = 0.569 |

armB k0, k2 and k7 fail every rep on maze 1 (2 trips) and succeed in every
rep on maze2. armB k1, k5 and k6 fail on both mazes. So most of armB's maze-1
failures were tied to that geometry. The mazes are not pooled.

## Sanity episodes (base controller, not one of the 16)

Kinematic 2/4 (26.5 s, 57.7 s, then fails at the shut door). Isaac 1/4: on
leg 1 the gait did not turn in place for ~20 s under `wz = -1`. See
`prereg/expF.md` §4.

## Files

`physics.csv`, `kinematic.csv`, `sanity.csv`: one row per episode
(trips, trip times, contacts, path, tracking error, step time, load, power
profile, controller sha). `traces/`, `traces-kinematic/`, `traces-sanity/`:
per-step gzip CSVs (`demo/exp3.py` `TRACE_COLUMNS`). `manifest-*.json`: git
head, prereg commit, order, controller hashes.
