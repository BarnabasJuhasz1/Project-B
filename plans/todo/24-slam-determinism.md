# 24 — Determinism and cost gate

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 23, 24a · **Needs Isaac Sim:** **yes**

> ⚠️ **This chunk is now the PHYSICS half only.** Chunk
> [24a](../doing/24a-slam-cost-model.md) took everything here that did not need a GPU: the cost
> model, the phase breakdown, the scan-rate decision (**5 Hz**, measured), the sweep budget, the
> threading question, and the `check_determinism.sh` extension. What remains is the part with no
> CPU-side substitute — **bitwise determinism under physics, in the container** — plus re-running
> `scripts/benchmark_slam.py` there so the committed cost table is a containerised number like
> every other reported one.
>
> Two of this chunk's `Decided` clauses were **overtaken by 24a's measurements** and are marked
> below. Read 24a's Decided section before executing this one.

## Why

**This is a gate.** Two properties this project has committed to are exactly the two a SLAM
pipeline threatens, and neither can be established after the fact:

- **Same-machine bitwise determinism**, checked by `scripts/check_determinism.sh` since chunk 08.
  A localiser that is "almost always identical" cannot produce a reported number here.
- **A wall-clock budget that permits seed sweeps.** Chunk 17 fixes an episode count; if SLAM
  makes an episode ten times more expensive, that count becomes unaffordable and the
  pre-registration for cycle 2 has to know before it is written.

Chunks 21–23 each assert determinism on their own component. This chunk asserts it **end to end,
under physics, in the container** — which is the only place the claim actually has to hold.

## Decided

**The localiser runs at scan rate, not control rate.** Real 2D lidar runs at 5–20 Hz; the control
loop is 50 Hz. Matching every control step is three to ten times the work for a pose the sensor
could not have supported anyway. So `LocalisationSpec` carries a rate, the localiser integrates
odometry between updates, and this is a **realism improvement as well as a cost one**.

The rate is **pre-registration material**: it changes how good localisation is, and picking it
after seeing the results would be choosing the answer.

✅ **SETTLED BY 24a: 5 Hz**, measured before the GPU was rented, which is the strongest available
version of "not picked after seeing the results". `DEFAULT_SCAN_RATE` moved from 10: at held-out
episode lengths 5 Hz is 3.3-4.1x cheaper than 10 at indistinguishable accuracy, and it is the
difference between ~4.3 and ~17.8 SLAM hours over the pre-registered episode set. This chunk
confirms the number in the container; it does not re-open it.

⚠️ **It delivers 4.53 Hz, not 5** — see 24a's Decided on the accumulator. Anything this chunk
writes down must say 4.53.

**Determinism is claimed same-machine only, and the claim is written down.** Floating-point
reductions differ across BLAS builds, CPU generations and thread counts. Chunk 08 already scopes
the existing claim this way; SLAM does not widen it. The container is what makes it reproducible
for anyone else.

**Threading off, explicitly.** `OMP_NUM_THREADS=1`, `MKL_NUM_THREADS=1`, set in the container
entrypoint and asserted at runtime by the localiser rather than assumed. A parallel sparse solve
is a non-deterministic sparse solve.

⚠️ **OVERTAKEN BY 24a, in its second half.** The env vars in the entrypoint stand — they cost
nothing and they widen the claim across machines. **The runtime assertion does not**: measured,
1 thread and 22 produce byte-identical output, because nothing in `core/slam` reaches a BLAS
threading threshold (`spsolve` is SuperLU; the only dense products are 3x3 blocks and an
`(N,2) @ (2,2)`). An assertion whose premise is measured false is not cheap insurance. What
replaces it is `test_the_pipeline_is_byte_identical_across_blas_thread_counts`, a regression guard
on the *reason*. Do not add the assertion back without re-running that test first.

**If the budget is exceeded, the lever is the rate — not the algorithm.** Stated in advance so
that a slow result does not become an argument for quietly loosening a convergence criterion or
shrinking a search window until the numbers fit. Search-window and submap sizes are fixed by
chunks 21 and 22 and are not cost levers here.

## Build

- ~~Extend `scripts/check_determinism.sh`~~ — **done in 24a.** It takes pass-through
  `run_episode.py` args and diffs `slam.csv` alongside `trace.csv`. Written and syntax-checked
  there; **running it is this chunk's job.**
- ~~A runtime assertion in `SlamLocaliser.reset()` that thread counts are 1~~ — **dropped, see
  the Decided note above.** A test replaces it.
- ~~`scripts/benchmark_slam.py`~~ — **built in 24a**, with cost and accuracy in the same row and a
  `--scaling` mode. **Re-run it in the container**, because the reported number has to be a
  containerised one.
- Container entrypoint documentation for the thread env vars. **Still to do**, and still worth
  doing: it is what makes the determinism claim hold off this machine.

## Done when

- **`scripts/check_determinism.sh` passes on a SLAM scenario**, twice, with `trace.csv` and
  `slam.csv` both byte-identical.
- **Determinism holds across a fresh container**, not only a fresh process — the run that matters
  is the containerised one.
- **Determinism holds with the recorder on and off**, since rendering never advances physics and
  must not perturb the localiser either. This is the existing invariant, extended.
- **A cost table is committed** *from the container*: per-episode wall clock for
  `PerfectLocaliser`, `DriftModel` and `SlamLocaliser` at each candidate rate, on the same
  scenario, headless. 24a's table is the CPU-side version and the shape to expect; this one is the
  reportable one.
- ~~The chosen localiser rate is recorded~~ — **done in 24a: 5 Hz** (delivering 4.53), with its
  table. Confirm, do not re-derive.
- ~~The thread-count assertion fires when the env vars are unset~~ — **dropped**; the replacement
  test passes.
- ~~A seed sweep of the pre-registered episode count is shown to be affordable~~ — **arithmetic
  done in 24a** and derived from `runs.design`. ⚠️ Re-check it against the container's own
  wall clock, and against 24a's scaling finding.

## Not in scope

Changing any algorithm to hit a budget — see Decided. If the budget cannot be met at any
reasonable rate, that is a finding for the epic, not a licence to retune chunks 21–23.
