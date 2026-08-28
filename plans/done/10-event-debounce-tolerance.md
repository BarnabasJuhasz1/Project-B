# 10 — Event counts sit on a floating-point knife edge

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 04 · **Needs Isaac Sim:** no

Found while executing [09](../done/09-spl-denominator.md), which re-scored the committed results
from their traces and made this visible. Not caused by 09; 09 only supplied the perturbation that
exposed it. Sim-free.

## Why

`core/metrics.py:_events()` ends a span when the signal has been clear for `EVENT_RELEASE`:

```python
elif t[i] - clear_since >= release:
```

`EVENT_RELEASE = 0.5` s at a 0.02 s control step is **exactly 25 steps**. So the comparison is
`0.5 >= 0.5` on floats that were accumulated by summing `dt` 25 times, and the last bits decide it.
There is no tolerance. `_stuck_spans()` right above it already guards the same hazard with
`t[i] - t[j] < STUCK_WINDOW - 1e-9`; `_events()` never got the same treatment.

Measured, not hypothesised. `out/runs/doorway_traverse__goal_seeker__seed1001/trace.csv` stores
timestamps rounded to 6 dp, which differs from the in-memory trace by at most **6.7e-12 s**:

```
timestamps as accumulated in memory   stuck_events 2   time_stuck  78.76 s
the same trace, written and read back stuck_events 1   time_stuck 111.36 s
```

Two spans merged into one because a clear gap of exactly 25 steps fell the other side of the
comparison, and `time_stuck` then swallowed the 32.6 s gap between them. **A 41% swing in a
reported secondary metric from a perturbation that changed nothing the robot did.**

`collisions` and `near_misses` and `localisation_failures` all run through the same `_events()`
and are exposed identically — this episode simply happened to trip it on the stuck signal.

### Why it matters beyond one file

The reproducibility claim in `results/README.md` is "same machine, bitwise". That still holds:
identical traces score identically. But it is a much weaker claim than it looks, because the
metrics turn out to be discontinuous in the trace at a scale far below anything physical. The
cross-machine claim is "statistical equivalence" — and comparing distributions is exactly the
setting where a metric that flips on the last bit of a float adds variance that no amount of
seeding controls.

It also means **re-scoring is not currently a safe operation**, which undercuts the design
property `core/metrics.py` opens by claiming: that scoring is a pure function of a trace and can
be redone when a threshold moves.

## Decided — option A

Pre-registration material, same as 09 — it moves `collisions`, `near_misses`, `stuck_events`,
`time_stuck` and `recovery_time` on committed files.

| | Option | Effect |
|---|---|---|
| **A** | Tolerance: `t[i] - clear_since >= release - 1e-9`, matching `_stuck_spans` | Smallest change. A gap of exactly `EVENT_RELEASE` now reliably **closes** a span |
| **B** | Count in control steps rather than seconds — `release_steps = round(EVENT_RELEASE / dt)` | Removes float comparison from the debouncer entirely; needs `dt` threaded into `score()` |
| **C** | Choose an `EVENT_RELEASE` that is not an integer multiple of the control step, e.g. 0.51 s | Dodges this instance without fixing the mechanism; a different control rate re-creates it |

**Decided: A.** `t[i] - clear_since >= release - 1e-9`, matching the guard `_stuck_spans` already
carries one function above. B is cleaner in principle but threads `dt` through a function that
today needs only the trace, and the trace already carries `t`. C is not a fix — a different
control rate re-creates the same knife edge somewhere else.

**Decided, and this is the part that was actually undefined: a clear gap of exactly
`EVENT_RELEASE` CLOSES the span.** The tolerance points that way deliberately. `EVENT_RELEASE` is
the duration after which the signal is considered released, so a gap that reaches it has released;
requiring one step *more* than the stated constant would make the constant a lie by one step.
Write that sentence next to the constant, not just in this plan.

Also assert in a test that `EVENT_RELEASE / dt` landing on an integer is handled deterministically
— that is the whole failure mode, and it disappears from view the moment the constants change.

This decision is collected by chunk [17](../done/17-pre-registration.md). Do not re-open it there.

## Build

- `core/metrics.py`: the chosen tolerance, and a comment saying which way the boundary falls and
  why — matching the density of the `EVENT_RELEASE` comment already there.
- Audit the other threshold comparisons in `score()` for the same hazard. `>= GOAL_RADIUS`,
  `<= COLLISION_CLEARANCE` and `> LOC_FAIL_THRESHOLD` compare against measured floats rather than
  against accumulated sums, so they are not knife edges in the same way — confirm that rather
  than assuming it, and record the finding.
- Re-score `results/` with `scripts/rescore.py` once it is settled.

## Done when

- `core/tests/test_metrics.py` has a test that builds a trace whose clear gap is **exactly**
  `EVENT_RELEASE / dt` steps and asserts the event count, both from `t = i * dt` and from `t`
  accumulated by repeated addition. It must give the same answer for both. This test fails on
  today's code — check that it does before fixing, or it is not testing the bug.
- The same test at a different control rate (0.01 s, 0.05 s) where the gap is again an exact
  multiple.
- `scripts/rescore.py --dry-run` over `results/` reports **no change** to `collisions`,
  `near_misses`, `stuck_events`, `time_stuck` or `recovery_time` — a trace round-trip through
  6 dp must be a no-op for every event count. That is the real acceptance test.
- Whole `core/tests/` suite green, no simulator.

## Not in scope

`FALL_HEIGHT` as a height rather than a tilt, and `contact` being a geometric proxy — both still
open in `results/README.md`, both separate pre-registration decisions.

---

## Outcome

**Done 2026-08-25.** Option A as decided. `core/tests/` 132 → **150 passed**, no simulator.

### What the plan got right

The failing test came first and it did fail: a clear gap of exactly `EVENT_RELEASE/dt + 1`
samples closed the span with `t = i*dt` and did not with `t` accumulated. Worth noting **it only
failed at dt = 0.05** — at 0.02 and 0.01 the accumulated sum happened to land on the favourable
side. That is the whole character of the bug: whether it bites is luck of the bits, so a test at
one control rate would have called the code correct.

The boundary direction is now stated next to the constant, which was the genuinely undefined part:
**a clear gap of exactly `EVENT_RELEASE` closes the span.** `clear_since` is the *first* clear
sample, so the shortest closing gap is `release/dt + 1` samples — worth writing down, because the
off-by-one is not obvious from the constant.

### What the plan got wrong

**The audit found a second tie, and the plan predicted it would find none.** It named
`>= GOAL_RADIUS`, `<= COLLISION_CLEARANCE` and `> LOC_FAIL_THRESHOLD` and reasoned they compare
measured floats rather than accumulated sums. That reasoning is correct, and those three are
fine — but it scoped the audit to comparisons against *metric constants* and missed the one
against an *index*:

```python
j = np.searchsorted(t, t[i] - STUCK_WINDOW, side="left")
```

`STUCK_WINDOW = 2.0` s is an integer multiple of the control step exactly as `EVENT_RELEASE` is —
100 steps at 50 Hz — so `t[i] - STUCK_WINDOW` lands on `t[i-100]` by construction, and lands the
wrong side of it in float. Measured, over accumulated timestamps:

```
dt=0.02  accumulated  window=100  off-by-one at 489/1900 samples   (26%)
dt=0.01  accumulated  window=200  off-by-one at 381/1800 samples
dt=0.05  accumulated  window= 40  off-by-one at 482/1960 samples
```

`searchsorted` returned `i-99`, the window came up one step short, and the guard below it —
the `- 1e-9` guard that was cited in this plan as the *model* for the fix — then hit `continue`
and dropped the sample from the stuck signal entirely. The existing guard was not protecting the
computation; it was discarding it. Both now use `EVENT_RELEASE_EPS`, and the probe reports
0 off-by-one at all three rates.

**This is why the acceptance test could not have passed on the `_events` fix alone.** After
fixing only `_events`, `doorway_traverse__goal_seeker__seed1001` still moved `stuck_events` 2 → 1
on a re-score. The two ties feed each other: the window error perturbs the `active` signal that
`_events` then debounces.

### What moved in `results/`

`collisions` and `near_misses` unchanged on all six runs. Every stuck figure moved:

| run | `stuck_events` | `time_stuck` | `recovery_time` |
|---|---|---|---|
| `doorway_traverse__goal_seeker` | 2 → **1** | 78.76 → **111.36** | 39.38 → **111.36** |
| `blind_corner__goal_seeker` | 4 → **4** | 3.50 → **16.20** | 0.87 → **4.05** |
| `return_home__goal_seeker` | 1 → 1 | 110.66 | 110.66 |

The direction is a correction, not a churn. `goal_seeker` is documented as wedged for ~110 s of a
120 s episode; **111.36 s of a 119.98 s run describes that and 78.76 s did not.** Dropping a
quarter of the stuck samples had been fragmenting one long span into two and understating
`time_stuck` by a third.

### The acceptance test

`scripts/rescore.py --dry-run` after re-scoring: **0 of 6 result files changed.** Re-scoring is
idempotent, which is the property `core/metrics.py` opens by claiming and did not have.

The synthetic round-trip test is the one with teeth, and it took a second pass to get there: as
first written it *passed on the broken code*, because at dt = 0.02 the 6 dp round trip happened to
be benign. Parametrised over dt it fails at 0.05 pre-fix, flipping `collisions` 1 → 2 across a
write and a read. A round-trip test that passes before the fix is not testing the round trip.

### Also changed

`scripts/rescore.py:read_trace` claimed 6 dp is "well below every threshold in core/metrics.py".
True in magnitude, false in consequence — that sentence was the reason the hazard went unnoticed,
so it now says what was actually true and why.

### Left open, deliberately

`FALL_HEIGHT` as a height rather than a tilt, and `contact` as a geometric proxy. Both still in
`results/README.md`, both still pre-registration material for chunk 17.

### For chunk 17

Two constants to collect, not one: `EVENT_RELEASE = 0.5` with its boundary direction, and
`EVENT_RELEASE_EPS = 1e-9` shared by `_events()` and `_stuck_spans()`. And a rule worth stating
generally: **any threshold that is an integer multiple of the control step is an exact tie and
needs the tolerance.** `EVENT_RELEASE` and `STUCK_WINDOW` both were. A future constant that is
will be too, and nothing in the code makes that visible at the point of definition.
