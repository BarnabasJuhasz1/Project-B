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

---

## Outcome 2026-08-31 — built, 24 tests, and two of the plan's premises were wrong

`core/slam/{submap,loop,tracker}.py` and `core/tests/test_slam_loop.py`. Full sim-free suite
**661 tests**.

### ⚠️ Finding 1 — chunk 21's ambiguity measure was only half built, and rotation is the worse half

Chunk 21 added `ambiguity` after discovering that a symmetric world scores *well*. It measured
spread in **(x, y) only**. Driving a square in a rectangular room exposed the other half:

```
k=1  true=(2.15,2.00,+0.00)  est=(2.00,2.03,-0.18)  score=1.00  ambiguity=0.00  converged=True
k=3  true=(2.46,2.00,+0.00)  est=(2.05,2.03,-0.58)  score=1.00  ambiguity=0.00  converged=True
```

Perfect score, zero translational ambiguity, and the **heading is 0.58 rad wrong after three
steps**. Same cause as the corridor, one axis over: a room made of long straight walls has the
property that rotating a scan about the sensor SLIDES its points along those walls, and a point
sliding along a ridge stays on the ridge. Rotation is genuinely weakly constrained by that
geometry and a translational spread cannot see it.

`MatchResult` now carries **`ambiguity_theta`** as well, gated separately at `MAX_SPREAD_THETA`
(0.05 rad). Separately rather than folded into one scalar: different units, different failure
geometry, and chunk 23 may want anisotropic edge weights. Headings are compared as *wrapped
differences from the best*, or a tie spanning ±π would report a spread of 2π and every match near
the wrap boundary would be called ambiguous.

It immediately caught a case chunk 21 had recorded as acceptable: a degenerate scan matched
against a map built from itself has a unique *translational* peak (`ambiguity` 0.0) and was
converging. Its `ambiguity_theta` is 0.400 — a single cluster of coincident points carries no
rotational information at all. Chunk 21's test was updated to record that both halves are needed.

### ⚠️ Finding 2 — a thin submap is worse than no submap

Matching against a submap holding 1–3 scans returned **score 1.00 with the heading 0.6 rad
wrong**. A one-scan field is exactly the outline the next scan makes, so the match is a self-fit
that any wall-sliding rotation satisfies. Over one lap the estimate diverged to **1.43 m, against
dead reckoning's 0.11 m** — scan matching was an order of magnitude worse than doing nothing.

`Tracker` now places the first `MIN_SCANS_TO_MATCH = 5` scans of each submap by odometry alone.

### ⚠️ Finding 3 — in a room smaller than the sensor range, loop closure is not distinguishable from tracking

The Done-when asked for a control: *"a robot that never revisits gets no closures."* Measured: a
robot driving a straight line **accepted 40**. That is not a bug. lab42 is 12 × 8 m and the sensor
reaches 10 m, so a submap two indices back is still in view, overlaps the current scan, and
matching against it is a genuine constraint — correct, useful, and not a loop.

The premise was dropped rather than the gates tuned until it held. The test now asserts the
property that actually matters for safety: **every accepted closure is geometrically correct**
(the implied pose within 0.75 m of truth), on a trajectory that never revisits.

⚠️ **This bears on the epic's headline claim.** Epic 03 says *"a robot that returns to somewhere
it has been can correct its accumulated error, and one that never revisits cannot."* In lab42 that
distinction is weak, because everything is always in view. The mechanism is real, but attributing
a result to *revisiting* specifically will need either a larger environment or a shorter
`range_max` — and that is a chunk 25 recalibration question, not something to paper over here.

### The headline claim, scoped to what this chunk owns

The Done-when says "a closed loop reduces error", but this chunk explicitly does not correct the
trajectory — that is chunk 23. Measured over one lap: mean error **0.105 m with scan matching
against 0.107 m without**. Tracking alone lands where dead reckoning lands, which is the correct
outcome for this chunk and precisely why chunk 23 exists.

What is asserted instead is the property chunk 23 depends on: **the pose implied by an accepted
closure is closer to truth than the chained estimate it would replace**, on a majority of accepted
closures over a second lap. If that were false, applying closures in chunk 23 would make the map
worse and the epic would have no mechanism.

### A contract gap fixed in chunk 19 rather than worked around here

Chunk 19 said `update()` receives the true pose because *"chunk 20 is what stops it being a free
ride"*. **Chunk 20 does not** — its four effects are all lidar effects and it says "four effects,
and no more". Nothing degraded odometry, so the estimate would have been exact and a loop closure
would have had nothing to correct.

`LocalisationSpec` gained `odom_sigma_lin` / `odom_sigma_ang`, in `core/scenario.py`, while chunk
19 was still open — it is the localiser's dead reckoning, not the lidar, and a scenario may
sensibly want one without the other. Units match `DriftSpec`'s (per √second) so chunk 25 can
compare SLAM against the random walk directly.

### Done when

- ✅ Submap boundaries are a pure function of scan count, asserted.
- ✅ Points stored in the submap frame, so `relocate()` is free and keeps the cached field.
- ✅ All three gates tested in isolation with the other two satisfied, plus a positive control.
- ✅ The symmetric corridor is refused, by the ambiguity gate chunk 21's finding added.
- ✅ The lab42 divider produces no false closure.
- ⚠️ "A robot that never revisits gets no closures" — **replaced**, see finding 3.
- ⚠️ "A closed loop reduces error" — **scoped**, see above; the reduction is chunk 23's.
- ✅ Rejection counters by gate in `diagnostics()`, with a fixed key set from the first step so
  `slam.csv` columns cannot misalign.
- ✅ The tracker is deterministic: identical trail and identical diagnostics across runs.
