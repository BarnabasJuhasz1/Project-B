# 22 — Submaps and loop closure

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 21 · **Needs Isaac Sim:** no

## Why

Chained scan matches drift. Every match carries a small error, the errors compound, and after a
lap of lab42 the estimate is metres out — which is a more expensive way of reproducing
`core/drift.py`.

**Loop closure is the entire point of building SLAM here.** It is the mechanism by which *what
the robot chose to do* changes how well it knows where it is: a robot that returns to somewhere
it has been can correct its accumulated error, and one that never revisits cannot. Every card
this epic unlocks depends on that being true and measurable.

## Decided

**Submaps, not scan-to-scan.** Accumulate N scans into a submap, match new scans against the
current submap, and close loops against *finished* submaps. Consecutive-scan matching gives a
noisier trajectory and gives loop closure nothing stable to match against.

**Loop closure is proposed by geometry, not by appearance.** Candidates are finished submaps
whose origin lies within a search radius of the current pose estimate, inflated by the current
uncertainty. There is no place-recognition descriptor and there does not need to be one at this
scale.

*Rejected:* a scan-descriptor index (Scan Context and similar). Necessary at building scale,
overkill in a 12 × 8 m room, and another component to make deterministic.

**Three gates before a closure is accepted, and all three are required.** This is the part to get
right, because a false loop closure does not degrade the map, it **destroys** it — the graph
folds two different places onto each other and every pose after it is wrong.

1. **Score threshold** — the chunk-21 match score must clear a fixed bar, set from 21's noise
   sweep and *fixed in advance*, not tuned until the demo works.
2. **Search-window containment** — a match landing on the edge of the search window is a match
   that wanted to go further; reject it rather than clamp it.
3. **Geometric consistency** — the proposed correction must be within the current pose
   uncertainty by a fixed multiple. A closure claiming a 4 m correction when the estimate says
   0.3 m of drift is proposing that the estimate was wrong by an order of magnitude, and it is
   far more likely the match is wrong.

**A rejected closure is logged, with which gate rejected it.** Same reasoning as
`generate/admit.py`'s recorded rejections: "the loop was there and we did not see it" and "we saw
it and rejected it" are different failures with different fixes, and nothing else can tell them
apart. These counters go into `diagnostics()` and therefore into `slam.csv`.

**lab42 is a loop-closure trap, and that is useful.** The divider at x = 6 and the wall along
y = 5 make a room with two structurally similar corridors. That is a realistic false-closure
hazard sitting in the benchmark scene already, and it should be tested against deliberately
rather than discovered during evaluation.

## Build

- `core/slam/submap.py` — `Submap`: accumulated scans, its likelihood field (21), its origin
  pose, and whether it is finished. Fixed scan count per submap, so submap boundaries do not
  depend on trajectory.
- `core/slam/loop.py` — candidate proposal by radius, the three gates, and the rejection counters.
- Odometry chaining: current pose = last submap origin ∘ match against the current submap.

## Done when

- **A closed loop reduces error.** Drive a square in a micro-world, return to the start, and
  assert the corrected estimate is closer to truth than the uncorrected chain. This is the
  chunk's headline claim and it should be a single, readable test.
- **A false closure is refused.** Build the symmetric-corridor micro-world from chunk 21 and
  assert no closure is accepted between the two ends, and that the rejecting gate is recorded.
- **The lab42 divider does not produce a false closure** on a full traverse of `doorway_traverse`.
- **A robot that never revisits gets no closures** and its error grows monotonically — the
  control case, and the thing that makes the mechanism attributable.
- Every gate has a test that exercises it in isolation, with the other two satisfied.
- Rejection counters appear in `diagnostics()` and in `slam.csv`.
- Submap boundaries are a pure function of scan count, asserted, so two runs cut them identically.

## Not in scope

Correcting the trajectory. This chunk *detects* and *proposes* closures; applying them is the
graph optimisation in chunk 23. Keeping the two apart is what lets 22's gates be tested without
a solver in the loop.
