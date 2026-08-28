# 08 — Determinism and end-to-end verification

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 07 · **Needs Isaac Sim:** yes

## Why

"Reproducibly" is the word in the Week 1 deliverable that the previous chunks make plausible but
do not demonstrate. This chunk demonstrates it, and writes down honestly how far the claim goes.

## Build and check

1. **Success path.**
   `run_episode.py --scenario doorway_traverse --controller reactive_avoid --headless`
   → `goal_reached: true`, `spl` in a plausible band. Confirms the scenario is solvable and that
   the success half of the scoring code fires. If `reactive_avoid` cannot do it, that is a finding
   about the controller or the scenario — record which, do not quietly retune the metric.
2. **Determinism.** Run the same command twice, `diff` the two `trace.csv` files. Identical.
   Wire this as `scripts/check_determinism.sh` so it stays runnable.
3. **All three scenarios** run to completion under both controllers — six episodes, no crashes.
4. **Recording.** One run with `--record`; check the mp4 against the trace.
5. **Commit the first results** under `results/`.

## Record the limitation

GPU physics is not guaranteed bitwise deterministic across drivers or hardware, so check 2 is
**same-machine repeatability**, not a universal claim. The manifest already records GPU, driver,
Isaac version, container digest and USD hash, so cross-machine runs are distinguishable. Across
machines the claim is statistical equivalence, not bitwise. Put this in the results doc now,
while it is cheap, rather than discovering it during Week 5.

## Done when

All five checks pass, the limitation is written down, and CLAUDE.md's status section is updated
to record Week 1's fourth item as complete.

## Then

Two Week 1 items remain, both unblocked by this epic:
- the full scenario suite and the **sealed** held-out split for G3
- 5–10 curated principle cards
- containerising against `nvcr.io/nvidia/isaac-sim:6.0.1`, before any number is reported

## Outcome

Done 2026-08-22. `scripts/check_determinism.sh`, `results/README.md`, first six results committed
to `results/`. Four of the five checks pass; the fifth (committing) is left to the user per the
working agreement.

### 1. Success path — PASS

`reactive_avoid` solves all three dev scenarios. `goal_seeker` solves none.

| scenario | controller | term | goal | spl | col | stuck | dur | path | opt |
|---|---|---|---|---|---|---|---|---|---|
| doorway_traverse | goal_seeker | timeout | ✗ | 0.00 | 1 | 2 | 120.0 | 9.51 | 11.07 |
| doorway_traverse | reactive_avoid | goal | ✓ | 1.00 | 1 | 0 | 15.4 | 10.63 | 11.07 |
| blind_corner | goal_seeker | timeout | ✗ | 0.00 | 1 | 4 | 45.0 | 3.18 | 5.15 |
| blind_corner | reactive_avoid | goal | ✓ | 1.00 | 1 | 0 | 6.8 | 4.30 | 5.15 |
| return_home | goal_seeker | timeout | ✗ | 0.00 | 1 | 2 | 120.0 | 3.58 | 11.07 |
| return_home | reactive_avoid | goal | ✓ | 1.00 | 1 | 0 | 15.5 | 10.24 | 11.07 |

### 2. Determinism — PASS

`scripts/check_determinism.sh doorway_traverse reactive_avoid` → 774 lines byte-identical.
Stronger than required: the sweep's `goal_seeker doorway_traverse` reproduced an earlier standalone
invocation to the last digit (`time_stuck 78.75999999998858`, `path_length 9.511971890516447`,
`min_clearance 0.00963687896728499`), so repeatability holds across separate processes, not only
within one script.

### 3. All six episodes — PASS

Six runs, exit code 0, no crashes. `goal_seeker` is ~5× slower in wall time than `reactive_avoid`
at equal sim time — it spends 80+ s pressed against the divider and contact solving dominates.
Worth budgeting for: a failing condition costs more wall clock than a succeeding one, and the
four-condition ablation runs many of both.

### 4. Recording — PASS, after fixing a bug it exposed

458 frames, 30 fps, 2562×720 (two 1280 panes plus the divider), 15.27 s against a 15.44 s episode.
Frames checked against the trace: at t=8 the robot is beside the divider at (5.47, 4.11) and at
t=15 it is on the green goal marker at (9.69, 6.30). Both match. The top-down pane also shows the
sealed room plainly.

### 5. Commit — not done

Left to the user. Nothing in this repo has been committed; the working agreement says not to
without being asked. **Every result in `results/` therefore carries `git_dirty: true` and is not
reportable until that changes.**

---

## Two findings

### `--record` was changing the run it recorded

The first recorded run reached the goal in **11.0 s** where the same scenario, controller and seed
unrecorded took **15.44 s**, with a visibly different trajectory. `Episode.drive()` was passing the
render flag through to `world.step(render=...)`, and `world.step(render=True)` does not advance the
same amount of simulated time as `world.step(render=False)`.

Fixed: physics now always steps with `render=False`, and a frame is drawn afterwards by
`world.render()`. Re-recorded, the run is now identical to the headless one —
`duration 15.43999999999976`, `path_length 10.631375495703065`,
`min_clearance -0.06887885173534616`, matching digit for digit.

This mattered more than a recording bug normally would: **a video that does not depict the run that
was scored is worse than no video**, because it will be used to explain results.

### ⚠️ SPL is saturated at 1.00 and currently carries no information

All three successful runs score exactly 1.00, because the actual path is *shorter* than the
geodesic denominator in every case:

| scenario | path | geodesic | denominator overestimates by |
|---|---|---|---|
| doorway_traverse | 10.63 | 11.07 | 4.0% |
| return_home | 10.24 | 11.07 | 7.5% |
| blind_corner | 4.30 | 5.15 | **16.5%** |

Two causes, both in `core/geometry.py`:

1. **The grid is inflated by `ROBOT_RADIUS = 0.30`, a circumscribing radius.** The Go2 is
   0.65 × 0.31 m and is not a disc, so it legitimately cuts corners closer than an inflated-grid
   path can — the runs reach `min_clearance` of −0.02 to −0.07, i.e. inside the inflated boundary.
2. **8-connectivity quantises headings to 45°**, which overestimates any path at an intermediate
   angle by up to ~8%.

Per this chunk's own instruction, recording rather than quietly retuning. But **SPL cannot be
reported as a secondary metric while it is clamped at 1.00 for every success** — it would show no
difference between conditions no matter how they differed. Options, to decide before
pre-registration:

- inflate the geodesic grid by the **inscribed** radius (~0.155 m, half the body width) instead
- use an any-angle search (Theta\*) or a visibility graph for a true continuous shortest path
- keep the conservative denominator and drop SPL from the reported metrics

The first is the smallest change and the most defensible; it needs a decision, not a guess.

## Limitation recorded

`results/README.md` states the reproducibility claim and its limits: same machine is bitwise,
across machines is statistical equivalence only, because GPU physics is not bitwise deterministic
across drivers or hardware. It also lists what may not be reported (`in_container: false`,
`git_dirty: true`, reference-controller runs, `library_snapshot: null`) and the two open metric
questions carried forward from chunks 04 and 07.

CLAUDE.md's status, repo layout, commands and Isaac gotchas are updated.
