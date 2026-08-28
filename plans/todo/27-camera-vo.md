# 27 — Camera visual odometry

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 24 · **Needs Isaac Sim:** **yes**
**Status: BONUS. Deferred, deliberately, and it may never be done.**

## Why it is written down anyway

Because "we did not do camera VO" should be a decision with reasons attached, not a gap someone
finds later and assumes was an oversight. And because there is exactly one thing it buys that
chunks 19–26 cannot buy at any price.

## The one thing it buys

**A second sensing modality**, and therefore `RequireSensorAgreement` — the last of the
description's example leaves that is *structurally* untestable rather than merely deferred. With
one modality there is nothing to cross-check against, and no amount of work on the lidar pipeline
changes that.

It would also make `DeviateToDisambiguate` stronger: visual features are far more sensitive to
viewpoint than a 2D scan is, so "move to disambiguate" has more to bite on.

## Why it is deferred

**Cost, and it is decisive.** Rendering is ~0.9 s per frame and that is *fixed overhead rather
than pixels* — measured at 0.87 s/frame at 1280×720 against 0.97 s/frame at 960×540, so lowering
the resolution does not help. A 15 s episode already takes 240 s at 15 fps. Camera VO needs frames
every step of every episode of every seed of every condition, and a seed sweep stops being
affordable by orders of magnitude. Nothing about the algorithm choice changes this; it is the
renderer.

**It is not sim-free.** Everything in 19–26 runs under `pytest` against the analytic raycast, with
no simulator, in seconds per iteration. That is the only reason a pose-graph back end is a
reasonable thing to build inside this project. A camera pipeline needs `isaacsim` in the loop for
every test, and `core/tests/test_import_guard.py` exists precisely to keep that out of `core/`.

**Determinism gets harder.** RTX rendering, feature detection thresholds, and RANSAC-based
essential-matrix estimation are three new sources of run-to-run variation, against a project whose
determinism claim is bitwise.

**And the classical/learned line gets awkward.** A hand-tuned ORB pipeline is defensible under
non-negotiable #2. Anything modern enough to work well is learned, and "no model is trained in
this project" would need a footnote — which is a real cost to the argument, not a technicality.

## If it is ever done

Sketch only, deliberately — this is not a plan to execute, it is enough to start one.

- **Stereo or RGB-D, not monocular.** Monocular VO has no scale, and recovering it from a lidar
  it is supposed to be independent of defeats the purpose.
- **Run it offline, from recorded frames.** The recorder already writes an mp4 per run and
  rendering never advances physics, so frames can be captured once and VO run afterwards, out of
  the episode loop. This does not make it cheap, but it decouples the cost from the control rate
  and keeps determinism testable without a simulator in the assertion.
- **It joins the existing pose graph as another edge type**, not as a parallel pipeline. Chunk 23
  is built for that; a second, separate estimator would need arbitration logic nobody has
  pre-registered.
- **`RequireSensorAgreement` becomes a Decorator**, and Decorators need chunk 16's composition
  work for the same reason Conditions do — see [26](26-slam-cards.md).

## Done when

Not applicable. This chunk is a recorded decision, not scheduled work. If it is picked up, it
needs its own plan with real "Done when" checks, and its own cost table before a line is written.
