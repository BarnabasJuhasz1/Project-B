# 24 — Determinism and cost gate

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 23 · **Needs Isaac Sim:** **yes**

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

**Determinism is claimed same-machine only, and the claim is written down.** Floating-point
reductions differ across BLAS builds, CPU generations and thread counts. Chunk 08 already scopes
the existing claim this way; SLAM does not widen it. The container is what makes it reproducible
for anyone else.

**Threading off, explicitly.** `OMP_NUM_THREADS=1`, `MKL_NUM_THREADS=1`, set in the container
entrypoint and asserted at runtime by the localiser rather than assumed. A parallel sparse solve
is a non-deterministic sparse solve.

**If the budget is exceeded, the lever is the rate — not the algorithm.** Stated in advance so
that a slow result does not become an argument for quietly loosening a convergence criterion or
shrinking a search window until the numbers fit. Search-window and submap sizes are fixed by
chunks 21 and 22 and are not cost levers here.

## Build

- Extend `scripts/check_determinism.sh` to diff `slam.csv` as well as `trace.csv`, and to run the
  localiser-carrying scenario.
- A runtime assertion in `SlamLocaliser.reset()` that thread counts are 1, raising with the env
  vars to set. A silent parallel solve is the failure this catches.
- `scripts/benchmark_slam.py` — wall clock per episode, broken down by matching, closure search
  and solving, on a fixed scenario in the container. Committed, because the number is reported.
- Container entrypoint documentation for the thread env vars.

## Done when

- **`scripts/check_determinism.sh` passes on a SLAM scenario**, twice, with `trace.csv` and
  `slam.csv` both byte-identical.
- **Determinism holds across a fresh container**, not only a fresh process — the run that matters
  is the containerised one.
- **Determinism holds with the recorder on and off**, since rendering never advances physics and
  must not perturb the localiser either. This is the existing invariant, extended.
- **A cost table is committed**: per-episode wall clock for `PerfectLocaliser`, `DriftModel` and
  `SlamLocaliser` at each candidate rate, on the same scenario, headless, in the container.
- **The chosen localiser rate is recorded** with the table that justified it, and marked
  pre-registration material for chunk 25.
- **The thread-count assertion fires** when the env vars are unset — tested, not assumed.
- A seed sweep of the pre-registered episode count is shown to be affordable, with the arithmetic
  written out.

## Not in scope

Changing any algorithm to hit a budget — see Decided. If the budget cannot be met at any
reasonable rate, that is a finding for the epic, not a licence to retune chunks 21–23.
