# 24a — The cost model and the scan rate, measured without a GPU

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 23 · **Needs Isaac Sim:** **no**

## Why

Chunk [24](../todo/24-slam-determinism.md) is a gate that needs the container, and it bundles two
questions that do not actually have to travel together:

1. **Is the SLAM pipeline bitwise reproducible under physics?** Genuinely needs Isaac Sim. There
   is no CPU-side substitute and this chunk does not pretend otherwise.
2. **What does SLAM cost, and what scan rate should it run at?** Does **not** need Isaac Sim, and
   this is the chunk that says so and proves it.

`SlamLocaliser.update()` takes a dt, a pose and an array of ranges. It cannot tell a physics step
from a kinematic one, so its cost is a pure function of what it is handed. Isaac Sim adds a
constant that is already measured, is the same for every condition, and is not a SLAM cost:
~0.9 s per rendered frame, 21 s of wall clock for a 15 s episode headless.

So the rate is **decided here and confirmed there**. Renting a GPU to discover a number a laptop
can measure is the expensive way round, and the rate is pre-registration material for cycle 2 —
chunk 24's own `Decided` says picking it after seeing the results would be choosing the answer.
Picking it *before* the GPU is rented is the strongest version of that.

## Decided

**The scan rate is 5 Hz. `DEFAULT_SCAN_RATE` MOVES from 10.**

⚠️ **This reverses a decision taken earlier in this same chunk**, and the reversal is the more
useful half of the finding. The first table measured one 14.5 s trajectory, where 5 Hz finishes too
few submaps to propose a single closure and lands at 0.364 m against 10 Hz's 0.221 — so 5 Hz was
recorded as a false economy. Measuring the *same* trajectory at three lengths shows that conclusion
was an artefact of the episode, not a property of the rate:

| sim s | 5 Hz cost | 5 Hz final | 10 Hz cost | 10 Hz final |
|---:|---:|---:|---:|---:|
| 37.9 | **0.281** | **0.030** | 0.931 | 0.068 |
| 77.7 | **0.445** | 0.055 | 1.669 | 0.053 |
| 117.6 | **0.590** | 0.068 | 2.421 | 0.061 |

**3.3-4.1x cheaper at indistinguishable accuracy**, and at the shortest length 5 Hz is outright
better. Every held-out scenario carries a `time_limit` of 120-240 s, which is 8-16x the traverse —
so the length at which 5 Hz fails is one no evaluation episode has.

It is also the difference between an affordable sweep and an unaffordable one. At 117.6 s, 10 Hz
costs 2.4x real time and 5 Hz costs 0.59x; over the pre-registered 26 460 sim-seconds that is
~35 GPU-hours against ~8.7, both including chunk 25's mandatory `PerfectLocaliser` control.

**Why the extra scans buy so little is the interesting part.** At three laps 10 Hz accepts **811**
closure edges against 5 Hz's **148**, and the extra 663 move the final error by 0.007 m. Closure
search is quadratic in the submap count and the scan rate sets that count, so **the rate is a
quadratic lever on the dominant cost and a linear one on matching** — which is why halving it
saves 4x and costs nothing measurable.

*Rejected:* 10 Hz. What chunk 23 assumed and what the first traverse table appeared to confirm.
2.4x real time at held-out episode lengths, for 0.007 m.

*Rejected:* 20 Hz. 4.3x real time on the loop, and *worse* than 10 Hz on final error (0.057 vs
0.053). Past the point of diminishing returns in both directions at once.

*Rejected:* 2 Hz. 0.123x real time and genuinely cheapest, but final error degrades to 0.089 m and
it is slower than any real 2D lidar — which would make the cost a property of a made-up sensor.

⚠️ **The residual risk, recorded rather than argued away.** On a short one-way episode 5 Hz
localises worse: 0.364 m against 0.221 on the 14.5 s traverse, because the closure machinery never
starts. A controller that reaches the goal quickly produces exactly such an episode. The
counter-argument is that on those episodes the goal was reached anyway, so localisation did not
bind — but the causality could run the other way, and this is the clause to re-read if a SLAM
condition underperforms on fast successes.

⚠️ **This moved a committed number.** `out/evidence/localisers/` was produced at 10 Hz. Its claim
(SLAM beats the random walk) is unaffected and in fact strengthened — 5 Hz at 77.7 s gives 0.055 m
against drift's 1.484 — but its *conditions* are now non-default and the README says so. It also
surfaced a test that had been **silently skipping itself**: `test_a_closure_edge_joins_two_submap_
nodes_not_the_robot` drives 800 steps, which is 8 submaps at 10 Hz and 4 at 5, and its
`pytest.skip` fired. The rate is now pinned in that test and the assertion is unconditional.

**⚠️ The chunk-24 runtime assertion on thread counts is REPLACED by a measurement, and this is a
departure from the plan.** Chunk 24's `Decided` says to assert `OMP_NUM_THREADS == 1` at runtime
because "a parallel sparse solve is a non-deterministic sparse solve". Measured: 1 thread and 22
produce **byte-identical** output, and the reason is structural rather than lucky — `spsolve` is
SuperLU and single-threaded, and the only dense products in `core/slam` are the 3x3 blocks in
`graph.py` and an `(N,2) @ (2,2)` in `scan.py`. Nothing reaches a BLAS threading threshold, so
there is no thread-dependent reduction to be non-deterministic.

An assertion that raises on unset env vars would therefore have been **false security that also
breaks every existing run** — nothing in this repo sets those vars today. What replaces it:

- `test_the_pipeline_is_byte_identical_across_blas_thread_counts`, which contrasts 1 thread
  against the machine's maximum through `threadpoolctl` and is a **regression guard on the
  reason**. Add a dense solve or a threaded factorisation and it fails, which is exactly when the
  assertion question should reopen.
- The env vars still go in the container entrypoint. They cost nothing and they make the claim
  hold across machines rather than only on this one — which is the part chunk 24 was right about.
- `check_determinism.sh` deliberately does **not** pin threads, so the check cannot pass for a
  reason it has not established.

*Rejected:* asserting anyway, on the grounds that it is cheap insurance. An assertion whose
premise has been measured false teaches the next reader something untrue about the pipeline.

**Timings never enter `slam.csv`.** Chunk 23 set this rule and 24a keeps it: the sidecar must be
byte-identical across two runs of one episode, and a timing differs every run by construction.
Cost lands on `SlamLocaliser.cost()`; determinism is checked on the file. Two channels, because
that is the only way to have both — and a test now enforces the fence.

**⚠️ The requested scan rate is not the delivered scan rate, and this chunk REPORTS that rather
than fixing it.** Two effects, both deterministic, neither a rounding nicety:

| nominal | delivered | why |
|---:|---:|---|
| 50 Hz | 50.0 Hz | the control rate itself |
| 20 Hz | 16.7 Hz | **quantisation** — 0.05 s is 2.5 control steps, and a scan lands on a step |
| 10 Hz | 9.98 Hz | — |
| 5 Hz | 4.53 Hz | **float accumulation** — ten additions of 0.02 give 0.19999999999999998 |
| 2 Hz | 1.98 Hz | — |

Nothing about reproducibility is at stake. What is at stake is that the rate is pre-registration
material, and a document saying "5 Hz" would describe an apparatus that ran at 4.53 Hz.

**Not fixed here, deliberately.** Carrying the remainder instead of resetting `_since_scan` to
zero would fix the second effect and move every committed SLAM number in
`out/evidence/localisers/`. That is a `Decided` belonging to chunk 25, which is where cycle 2's
apparatus is settled, and the repo's convention is that a change moving committed numbers is
proposed rather than made in passing — chunk 23 left the `--drift` renaming hazard the same way.

⚠️ **The chosen 5 Hz is the rate this bites HARDEST on**, delivering 4.53 Hz — 91% of nominal.
When the answer here was 10 Hz (99.8%) the two questions were independent and this section said so;
after the rate moved they are not. Two consequences, both binding:

- Every cost and accuracy number in this chunk is a measurement of **4.53 Hz wearing a 5 Hz
  label**. The comparison against 10 Hz is unaffected — both rows carry their own delivered rate,
  and a 9% shift does not close a 3.3-4.1x cost gap — so the *decision* stands.
- **Cycle 2's pre-registration must say 4.53 Hz, or fix the accumulator first.** It may not say
  "5 Hz" and mean this. That is now the highest-priority item chunk 25 inherits from here.

## Build

- `core/slam/tracker.py` — per-phase wall-clock accumulators (`match_seconds`, `closure_seconds`,
  and the counts). `perf_counter()` is ~50 ns against a ~44 ms match, four parts per million, so
  the instrumentation is permanently on rather than flag-guarded.
- `core/slam/localiser.py` — `cost()`, with `other` derived by subtraction so the parts and the
  total cannot disagree. Plus a stale comment fixed: it claimed the default rate was "every step"
  when `DEFAULT_SCAN_RATE` is 10 Hz.
- `scripts/benchmark_slam.py` — the cost table, cost and accuracy in the same row, on the two
  trajectories `compare_localisers.py` already defends. Plus the sweep budget, derived from
  `runs.design` rather than typed.
- `scripts/check_determinism.sh` — pass-through args so a localiser scenario can be checked, and
  `slam.csv` diffed alongside `trace.csv`. Absence on both sides is fine; presence on one is a
  failure.
- `core/tests/test_slam_localiser.py` — four tests: the phase sum, the wall-clock fence, the rate
  lever, and the thread-count guard.

## Done when

- A cost table is committed covering `PerfectLocaliser`, `DriftModel` and `SlamLocaliser` at every
  candidate rate, on the same trajectories, with **accuracy in the same row as cost**.
- The chosen rate is recorded with the table that justified it, and marked pre-registration
  material for chunk 25.
- The sweep budget arithmetic is written out and derived from the frozen constants.
- The thread-count question is settled by measurement, and whatever is concluded is enforced by a
  test rather than by a comment.
- `check_determinism.sh` diffs `slam.csv` and accepts localiser flags — **written and syntax-
  checked here, RUN in chunk 24.**
- The full sim-free suite passes.

## Not in scope

**Determinism under physics, and the containerised cost table.** Both are chunk 24 and both need
the GPU. This chunk exists to make that day short: everything that can be decided first, is.

Changing any algorithm to hit a budget — chunk 24's rule, inherited. The rate was the lever and it
did not need to move.

## Outcome — 2026-09-04

**Done, and it found something the plan did not anticipate.** The rate is set, the threading
question is settled, and a third finding came out of doubting the budget's own arithmetic.

Built: per-phase timers in `core/slam/tracker.py`, `SlamLocaliser.cost()`,
`scripts/benchmark_slam.py` (cost table, `--scaling`, `--budget-only`), the
`check_determinism.sh` extension, and four tests. **709 sim-free tests pass**, up from 702.

### The rate: 5 Hz, arrived at by first getting it wrong

The traverse alone (14.5 s, one way) says 10 Hz:

| localiser | x real | match | closure | solve | final err |
|---|---:|---:|---:|---:|---:|
| slam @ 50 Hz | **7.318** | 34.46 | **69.58** | 2.23 | 0.147 |
| slam @ 20 Hz | 0.918 | 8.92 | 4.29 | 0.11 | 0.229 |
| slam @ 10 Hz | 0.350 | 4.16 | 0.88 | 0.03 | 0.221 |
| slam @ 5 Hz | 0.222 | 3.21 | **0.00** | 0.00 | 0.364 |

50 Hz is unaffordable, confirming chunk 23 by measurement — and **understating** it: at the
control rate **closure search costs twice what matching does**, a term nobody had costed.

**And the 5 Hz row is a trap.** Its closure column is `0.00`, which reads as "too slow for the
mechanism to work" and was recorded that way. It is not: 14.5 s is too short to finish the submaps
a closure needs *at any useful rate*, and the same trajectory family at three lengths says the
opposite — see the corrected `Decided`. **5 Hz is 3.3-4.1x cheaper than 10 at indistinguishable
accuracy, and better at the shortest length measured.** `DEFAULT_SCAN_RATE` moved to 5.0.

The general lesson, which is worth more than the number: **a rate cannot be chosen from one
trajectory**, because the rate interacts with episode length through the submap count, and the
submap count is what the closure machinery needs to start. The first table was not wrong about what
it measured; it was wrong about what it generalised to.

### ⚠️ THE FINDING: cost is SUPERLINEAR in episode length, and the budget was wrong

`budget()` multiplies one ratio by the sweep's total sim-seconds. That is only valid if cost per
simulated second is constant. It is not. `--scaling 1 2 3 4` on the loop, at 10 Hz:

| laps | sim s | submaps | match | closure | solve | total | x real |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 37.9 | 19 | 17.11 | 18.09 | 0.41 | 35.70 | 0.942 |
| 2 | 77.7 | 39 | 32.87 | 83.09 | 2.12 | 118.20 | 1.521 |
| 3 | 117.6 | 59 | 33.64 | 150.50 | 5.56 | 189.82 | 1.615 |
| 4 | 157.4 | 79 | 70.95 | **402.38** | 17.98 | 491.58 | **3.123** |

**A 4.2x longer episode costs 3.31x more per simulated second.** The phases say exactly which
term: **matching is linear** (17.11 -> 70.95, 4.1x against 4.2x of episode) and **closure search
is quadratic** (18.09 -> 402.38, 22x ≈ 4.2²). The solve is worse still at 44x, being called more
often on a bigger graph.

The mechanism is not subtle once measured. `loop.candidates()` returns *every* finished submap
within `SEARCH_RADIUS`, and lab42 is smaller than the sensor range — chunk 23's Outcome already
noted that "a submap two indices back is still in view". So the candidate set grows with the
episode, each proposal is a wide-window match, and the total is O(submaps²).

**Three consequences, in order of how much they matter.**

**1. The held-out episodes are 120–240 s, and both measured trajectories are shorter.** The
traverse is 14.5 s. Extrapolating from it under-reads a 240 s episode, in the unsafe direction.
`budget()` now prints this caveat and `--scaling` is the test of it.

**2. The wall-clock budget is now a function of what the controller DOES**, which is the thing the
experiment measures. The same rate costs 0.35x real time on the one-way traverse and 3.12x on a
trajectory that revisits, because revisiting is what generates closure candidates. A sweep's
runtime can no longer be predicted from the episode count alone, so `budget()` reports a **range**
rather than a number. This is the same shape as the epic's central claim — SLAM cost, like
`localisation_failures`, has become controller-dependent — and it is a scheduling problem rather
than a scientific one.

**3. The budget Done-when cannot be met by the current algorithm on the long held-out
scenarios.** At 26 460 sim-seconds, a loop-like 3.1x is ~23 hours of SLAM alone, doubled to ~46 by
chunk 25's mandatory `PerfectLocaliser` control. Traverse-like behaviour is ~2.6 hours. The true
figure is between and depends on the conditions.

⚠️ **This is a finding for the epic, not a licence to retune.** Chunk 24's own `Not in scope`
says so: *"If the budget cannot be met at any reasonable rate, that is a finding for the epic, not
a licence to retune chunks 21–23."* The fix is a **bound on the candidate set per proposal
round** — best-N by distance, or a decay in revisit frequency — which is a design change with its
own accuracy cost, and belongs to a chunk with a `Decided` section. It is not a knob to turn while
reading a budget table.

**The rate is the lever, and the scaling finding is what makes it a strong one.** The rate is a
*linear* lever on matching but a *quadratic* one on closure search, because halving the rate halves
the submap count too. That is why moving 10 -> 5 Hz saved 3.3-4.1x rather than 2x, and it is why
the rate was able to absorb this finding on its own without touching an algorithm. It will not
absorb it twice: 2 Hz is already below any real 2D lidar.

### Threading: the assertion is not warranted, and the reason is structural

1 thread against 22: **byte-identical**. Nothing in `core/slam` reaches a BLAS threading
threshold — `spsolve` is SuperLU, and the only dense products are the 3x3 blocks in `graph.py`
and an `(N,2) @ (2,2)` in `scan.py`. An assertion raising on unset env vars would have been false
security that also broke every existing run, since nothing in this repo sets those vars.
`test_the_pipeline_is_byte_identical_across_blas_thread_counts` replaces it as a guard on the
reason. Chunk 24's plan is annotated.

### The rate you ask for is not the rate you get

20 Hz delivers 16.7 (quantisation: 0.05 s is 2.5 control steps); 5 Hz delivers 4.53 (float
accumulation: ten additions of 0.02 reach 0.19999999999999998). Deterministic, so reproducibility
is untouched — but the rate is pre-registration material, and a document saying "5 Hz" would name
an apparatus that never ran. **Reported, not fixed**: the fix moves every committed SLAM number
and is chunk 25's. The chosen 10 Hz is the least affected rate on the table, at 99.8% of nominal.

### Still open, and it belongs to 24 and 25

- **The candidate-set bound.** Named above. The rate change bought enough headroom that the sweep
  is affordable (~0.59x real time at 117.6 s), so this is no longer blocking — but the cost is
  still quadratic and still unpredictable across conditions, and 2 Hz is not available as a second
  application of the same lever. It remains the largest open design item in epic 03.
- **The 5 Hz residual risk**: short one-way episodes localise worse (0.364 m vs 0.221 on the 14.5 s
  traverse) because the closure machinery never starts. Re-read that clause in `Decided` if a SLAM
  condition underperforms specifically on fast successes.
- **The delivered rate is 4.53 Hz, not 5**, and the chosen rate is the one the accumulator defect
  bites hardest on. Chunk 25 either states 4.53 in the pre-registration or fixes the accumulator
  and re-runs this chunk's tables. It may not write "5 Hz".
- `check_determinism.sh`'s SLAM path is **written and syntax-checked, never run** — it needs the
  simulator, and that is chunk 24 by construction.
- The cost table is a laptop table. Chunk 24 re-runs it in the container, and the *shape* is what
  should transfer, not the absolute seconds.
- The loop table omits the 50 Hz row: ~40 minutes of wall clock to restate what the traverse
  already showed. Reachable with `--rates 50`.
