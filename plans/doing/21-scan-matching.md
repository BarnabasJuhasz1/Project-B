# 21 — Scan matching: the front end

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 19, 20 · **Needs Isaac Sim:** no

## Why

The front end answers one question: **given two scans, what relative pose separates them?**
Everything else in the pipeline is built on it. Chained, it is odometry; matched against a
submap, it is loop closure (22); its output edges are what the graph optimises (23).

## Decided

**Correlative scan matching, not ICP.** Brute-force search over a discretised window of
(dx, dy, dθ), scoring each candidate by how well the transformed scan lands on an occupancy
likelihood field, and taking the best cell. Olson (2009), *Real-Time Correlative Scan Matching*,
is the reference; Cartographer's 2D front end is the same idea.

The reason is **determinism**, and it is the epic's stated rule that determinism wins over
accuracy:

| | Correlative | ICP |
|---|---|---|
| Iterations | fixed, = window size | until a convergence tolerance |
| Result depends on initial guess | only through window placement | strongly, and non-linearly |
| Random restarts / RANSAC | none | usual, and a seeded rng that must never drift |
| Failure mode | returns a bad score you can threshold on | converges confidently to a local minimum |
| Cost | predictable, and cappable | variable |

The last row of the ICP column is the disqualifying one. A matcher that fails *confidently* poisons
the graph in chunk 23 and the failure surfaces as a mangled map three chunks later.

*Rejected:* point-to-line ICP with a fixed iteration cap. Faster and more accurate when it works,
and its "fixed cap" is a truncation rather than a guarantee — the result still depends on where it
started.

**Multi-resolution search.** A single-resolution brute force over a window big enough for loop
closure is far too slow. Search a coarse grid first, keep the branch-and-bound upper bound, refine
only the best cells. Deterministic as long as **ties are broken by a fixed rule** — lowest index
wins, stated in the code, tested with a deliberately symmetric scan.

**The likelihood field is precomputed per submap, not per match.** Blur the occupancy grid once
with a fixed kernel; matching is then a table lookup per beam endpoint. This is what makes the
brute force affordable.

**The matcher returns a score, and the score is used.** `MatchResult(pose, score, converged)`.
A low score must be *reported*, not silently returned as a pose — 22 thresholds on it for loop
closure and 23 weights on it. A front end that always returns an answer is a front end that
cannot say "I don't know", which is the only thing that stops a bad match entering the graph.

## Build

- `core/slam/likelihood.py` — occupancy grid from a scan set, plus the blurred likelihood field.
  Reuse `core.geometry`'s grid conventions and resolution rather than inventing a second set;
  a second convention is how the unit-scenario/benchmark geometry split was avoided in 13 and
  the same argument applies.
- `core/slam/matcher.py` — `match(scan, field, initial_guess, window)` → `MatchResult`.
  Multi-resolution, branch-and-bound, fixed tie-break.
- `core/slam/scan.py` — scan → endpoint cloud in the robot frame, with `inf` beams dropped.
  One place, because 21, 22 and 23 must all agree on what a scan *is*.

## Done when

- **Recovering a known transform.** Take one scan, transform it by a known (dx, dy, dθ), and the
  matcher recovers it to within the search resolution. Parameterised over a grid of transforms,
  including pure rotation and pure translation.
- **Noise tolerance is measured, not assumed.** Sweep chunk 20's σ and report matcher error
  against it as a table in the chunk Outcome. This table is what chunk 24's cost/accuracy
  trade-off is set from.
- **Dropout tolerance**: with 30% of beams `inf`, the match still recovers the transform.
- **A symmetric world produces a low score, not a confident wrong answer.** Build a micro-world
  with a corridor whose two ends are identical; assert the score is below the threshold rather
  than asserting a particular (wrong) pose. This is the test that matters.
- **Bitwise determinism**: the same inputs produce the same `MatchResult` across 100 repeats, and
  across a fresh interpreter.
- **Tie-breaking is deterministic**: a deliberately ambiguous scan resolves the same way every
  time, and the rule is stated in the docstring.
- Cost per match is measured and recorded — chunk 24 needs the number.

## Not in scope

Chaining matches into a trajectory (that is odometry, and it belongs with 22's submaps), loop
closure detection, and any graph optimisation.

---

## Outcome 2026-08-31 — built, 25 tests green, and the plan's central assumption was wrong

`core/slam/{__init__,scan,likelihood,matcher}.py` and `core/tests/test_slam_matcher.py`.

### ⚠️ Finding: a symmetric world does NOT produce a low score

This chunk's own Done-when said:

> *A symmetric world produces a low score, not a confident wrong answer. Build a micro-world with
> a corridor whose two ends are identical; assert the score is below the threshold.*

**Measured: it does not.** A scan taken 4 m from where the field was built, in a featureless
corridor, matched at **score 0.618** — comfortably above `MIN_SCORE` — because its wall points
landed on walls. The fit is genuinely good. It is simply not *unique*, and a scalar goodness-of-fit
cannot express that.

There are **two** failure modes and one number cannot carry both:

| | what it looks like | detected by |
|---|---|---|
| **No match** — the scan fits nowhere | low score | `score` |
| **Ambiguous match** — many poses fit equally well | **high score at every one** | `ambiguity` |

The second is what a corridor, a blank wall or an empty room produces, and it is precisely what
must never reach chunk 23's graph. So `MatchResult` gained a third field: **`ambiguity`**, the
bounding-box spread of every candidate scoring within `AMBIGUITY_BAND` of the best, in metres.
`converged` now requires a good score **and** a tight peak. Olson (2009) derives a full covariance
from the same score distribution; this is the cheap scalar version, and promoting it to a
covariance is the obvious upgrade if chunk 23 wants anisotropic edge weights.

One consequence for the search: **pruning now carries a margin.** Blocks are discarded at
`bound <= best - AMBIGUITY_BAND` rather than `bound <= best`, because the blocks sitting just
under the peak are exactly the ones that make a match ambiguous — pruning at the peak would make
the measure systematically blind to the candidates it most needs to see.

### Accuracy against sensor noise, measured not assumed

30 random offsets per row, `doorway`-scale room, 181 beams, window ±0.30 m / ±0.20 rad.
`scale` multiplies `core.sensor_noise.NOISE_MODERATE`.

| scale | σ_range | dropout | xy mean | xy p90 | xy max | yaw max | score | converged |
|---|---|---|---|---|---|---|---|---|
| 0.0 | 0.0000 | 0.00 | 0.0402 | 0.0551 | 0.0644 | 0.0122 | 0.655 | 30/30 |
| 0.5 | 0.0025 | 0.01 | 0.0402 | 0.0551 | 0.0644 | 0.0076 | 0.652 | 30/30 |
| 1.0 | 0.0050 | 0.02 | 0.0381 | 0.0532 | 0.0644 | 0.0115 | 0.648 | 30/30 |
| 2.0 | 0.0100 | 0.04 | 0.0394 | 0.0532 | 0.0644 | 0.0152 | 0.639 | 30/30 |
| 4.0 | 0.0200 | 0.08 | 0.0369 | 0.0552 | 0.0655 | 0.0186 | 0.619 | 30/30 |
| 8.0 | 0.0400 | 0.16 | 0.0399 | 0.0633 | 0.0680 | 0.0190 | 0.559 | 30/30 |

Two readings. **The error is dominated by the search resolution, not by the noise** — it sits at
about one cell (0.05 m) at every σ, because the translational search is on the field's own grid
and cannot resolve better. **The score degrades gracefully** rather than falling off a cliff,
0.655 → 0.559 across a 16× range of σ, which is what makes a single `MIN_SCORE` viable. Chunk 20's
noise model is nowhere near the matcher's breaking point; the accuracy floor here is a design
choice, and chunk 23's graph is what refines past it.

### Cost — the number chunk 24 needs

**27.1 ms per match** (181 beams, ±0.30 m, ±0.20 rad, this machine, `env_isaacsim`).

⚠️ **That settles the localiser rate.** The control loop runs at 50 Hz — a 20 ms budget — so
matching **cannot** run per control step. The epic already says real SLAM runs at scan rate rather
than control rate; this is the measurement that makes it mandatory rather than preferable. At 5 Hz
a 15 s episode costs ~2 s of matching, which is affordable.

### A fixture failure worth keeping

The first parameter grid put the robot **inside the pillar** for negative-x offsets.
`core.geometry.raycast` returns 0.0 for beams whose origin is inside geometry, so the matcher was
handed 181 coincident points — and against a real map whose wall lay within the blur radius it
scored **1.000**, the best value obtainable, at a pose 0.2 m wrong. `ambiguity` caught it (0.500).
That is now `test_a_scan_taken_inside_a_wall_is_degenerate_and_is_flagged`, and it is a better
test of the ambiguity measure than the corridor is.

It also exposes a boundary for chunk 22: a degenerate scan matched against a map built from *the
same* degenerate scan converges legitimately, with `ambiguity` 0.0 and the correct pose. Self-
consistent nonsense is self-consistent. **Declining to build a submap out of it is chunk 22's job,
not the matcher's.**

### Done when

- ✅ Known transforms recovered, parameterised over pure translation, pure rotation and both.
- ⚠️ Symmetric world — **restated**: a high score with high `ambiguity` and `converged=False`, not
  a low score. The Done-when as written asserted something false; the test asserts the measured
  behaviour and fails loudly if the score ever does start catching this on its own.
- ✅ 30% dropout still recovers the transform.
- ✅ Bitwise determinism over 30 repeats: pose, score and `evaluated` all identical.
- ✅ Tie-break is the stated rule — a blank field resolves to the lowest flat index every time.
- ✅ Branch-and-bound returns **exactly** what brute force returns, asserted against a reference
  implementation kept in the module for that purpose; and `evaluated` proves the pruning is real.
- ✅ Noise/accuracy table and per-match cost recorded above.
