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
