# 24b — Seeing the SLAM run

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 23, 24a · **Needs Isaac Sim:** **no**

## Why

Chunks 21–23 built a scan matcher, submaps, four closure gates and a pose-graph back end, and
every one of them was verified the same way: a number in a test, or a number in `slam.csv`. That
is the right way to *check* a SLAM pipeline and it is a poor way to *understand* one. The
questions a reviewer actually asks — does the map look like lab42? does the estimate track the
truth or wander off down a corridor? what does the trajectory do at the instant a closure lands?
— are all geometric, and none of them is a scalar.

It is also the question the project will be asked in the write-up. Epic 03's whole claim is that
**the pose estimate becomes a consequence of what the robot did.** A table of `closures_accepted`
does not show that to anyone. A map with the truth drawn through it does, in one image.

And there is a narrower, harder-nosed reason. `core/slam/localiser.py` carries a docstring
warning that `error_estimate()` is **a proxy, not a marginal covariance** — accumulated
dead-reckoning uncertainty that rises while driving and drops on a closure. Whether that proxy is
anywhere near the realised error is a question chunk 26's cards will lean on
(`IsLocalisationDegraded` reads it) and **nothing in the repo currently plots the two against each
other.** This chunk makes that plot a one-liner.

## Decided

**The map is recorded, not re-derived.** The alternative — reconstruct the map afterwards from
`trace.csv` by re-running the localiser — was rejected outright: it would make the picture a
*second* run of the pipeline rather than a view of the one that produced the trace, and the two
could disagree without anything failing. The recorder captures what the live localiser held.

**Recording is OPT-IN and inert, on `nodes/trace.py`'s precedent.** `Tree.enable_trace()` is the
model: a traced run produces a byte-identical `trace.csv`, and the same rule applies here.
`SlamLocaliser.enable_recording()` adds no arithmetic to the estimate, draws nothing from the
rng, and is off unless asked for. A test pins it, because "the visualiser changed the run" is
exactly the failure that would poison the evidence it exists to produce.

**Points are stored in the SUBMAP frame, and the graph is stored per change.** This is not a
storage optimisation, it is the same structural fact `submap.py` is built on: a submap's points
never move relative to its origin, so a closure that relocates the origin carries the whole
submap. Recording local points plus a sequence of graph poses therefore reproduces the map at
*every* instant of the episode exactly, for the price of one copy of the points — and it means
**the animation shows the map snap when a closure lands**, which is the single most informative
frame in any SLAM video and the one thing a final-state picture cannot show.

**The renderer is sim-free and reads a run directory.** Not a fourth pane in
`scripts/recorder.py`. Two reasons and the second is decisive:

1. `recorder.py` costs ~0.9 s per frame because it renders RTX cameras. The SLAM view needs no
   renderer, and coupling it to one would put a 40-minute price on a picture that takes seconds.
2. **A container run and a local run then get the same treatment.** The reported episodes will be
   produced on rented hardware with no display; `visualise_slam.py <run_dir>` runs afterwards, on
   a laptop, against the committed artefacts. A pane inside the video would be reachable only by
   re-simulating.

**matplotlib, not `nodes/diagram.py`'s hand-rolled PIL.** `diagram.py` is PIL because it
composites into a video frame at an exact pixel size. Nothing here does; these are plots with
axes, legends and shared time bases, and matplotlib is already an env dependency (`diagram.py`
borrows its font). The mp4 goes to ffmpeg through the same rawvideo pipe `recorder.py` uses,
rather than through a matplotlib animation writer, so there is one way of making a video in this
repo.

**⚠️ The visualiser draws ground truth, and that is allowed here precisely because it is not in
the loop.** Every other consumer of `Observation` is forbidden ground truth by contract. This one
reads `trace.csv` after the episode is over and scores nothing — `core/metrics.py` never imports
it. Comparing the estimate against the truth is the entire point, and the reason it is safe is
that the comparison happens strictly downstream of the run.

## Build

- `core/slam/record.py` — `MapRecorder`. Submap local points with the scan index each point was
  added at; graph poses snapshotted on change (node added, or solve) with the reason; edges with
  `kind` and the scan they appeared at. `save_npz()` / `load_npz()`, and `poses_at(scan)` /
  `points_at(scan)` as the forward-fill the renderer reads through.
- `core/slam/localiser.py` — `enable_recording()`, and the three feed points (`_sync_nodes`,
  `_add_closures`, `_optimise`) plus per-scan point capture in `_update`.
- `scripts/run_episode.py` — `--slam-map`, writing `slam_map.npz` on the same terms `nodes.csv`
  and `slam.csv` are written on: a run with nothing to report writes no file.
- `scripts/visualise_slam.py` — sim-free. `map.png`, `error.png`, `closures.png`, `slam.mp4`.
- Tests in `core/tests/test_slam_record.py`.

## Done when

- A recorded run and an unrecorded run of the same episode produce **byte-identical** `slam.csv`
  and an identical pose stream.
- `MapRecorder.points_at(k)` reproduces `Submap.global_points()` at the end of the episode, to
  the last bit, for every submap.
- One real physics episode exists in `out/evidence/slam-visualisation/` with its four artefacts
  and a README saying what each answers.
- The uncertainty proxy is plotted against the realised error and **whatever that plot says is
  written down**, including if it says the proxy is badly scaled.

## Not in scope

**A live viewer.** No GUI, no interactive scrubbing. The mp4 is the animation.

**An occupancy grid.** The map drawn is the accumulated scan points, which is what the pipeline
actually holds — `core/slam/likelihood.py` builds a blurred field per submap and there is no
global grid anywhere in the localiser. Rasterising one for the picture would be drawing a data
structure the SLAM does not have.

**Fixing anything the pictures reveal.** If the proxy is badly calibrated, or the map shears, this
chunk reports it. Acting on it is 25 or 26.

---

## Outcome — 2026-09-04

**Done, and the pictures did what pictures are for: they found five things that the eighteen
scalars in `slam.csv` were not reporting, and one of them is a hole in `slam.csv` itself.**

The build went as planned and is not the interesting part. `core/slam/record.py` keeps submap-frame
points plus a graph snapshot per change; `SlamLocaliser.enable_recording()` is opt-in and inert
(`core/tests/test_slam_record.py`, 15 tests, 35 s — the load-bearing one runs the episode twice and
diffs the full-precision pose stream and the diagnostics); `scripts/visualise_slam.py` draws four
artefacts from a run directory with no simulator. Two run-directory sources: `run_episode.py
--slam-map` under physics, and `compare_localisers.py --save-run`, which is what makes the pictures
reproducible on a laptop.

Full findings and numbers: [`out/evidence/slam-visualisation/README.md`](../../out/evidence/slam-visualisation/README.md).
The five in order of how much they bind:

### 1. ⚠️ `error_estimate()` reports LOW uncertainty during the episode where localisation fails

Not a scale error — it disagrees in **opposite directions** on the two runs. On the clean two-lap
replay it is **3.6x too high**. On the physics episode that failed, it is **0.4x too low**: the
realised error is above `LOC_FAIL_THRESHOLD` on 24.5% of steps and averages 0.969 m after t=20 s,
while the proxy peaks at 0.557 m and reaches 1.0 m on **zero** steps.

The mechanism is finding 2: the proxy resets on every *accepted* closure, and this episode accepted
37 of 37. **The more closures a run accepts, the more confident the proxy becomes, independently of
whether they helped.**

**This binds chunk 26.** `IsLocalisationDegraded` reads this field, and on the one episode in this
repo where localisation actually degraded it would have said it had not. Either a card conditions
on the *shape* (rise, reset, relative change) or the proxy is calibrated first — and calibrating it
moves every committed SLAM number, so it is 25's decision, not 24b's.

### 2. ⚠️ A stationary robot manufactures loop closures, and nothing stops it

The robot turned on the spot for 40 s and finished **nine submaps inside a 0.13 x 0.19 m box**,
then closed essentially all of them against each other: 37 proposals, **37 accepted, 0 rejected**,
median endpoint separation **0.105 m**, and exactly **one** edge in the whole episode spanning more
than a metre.

Every one of those closures is valid and none is useful. `loop.py`'s four gates all judge the
*quality* of a proposed match, and all four are correctly satisfied; **nothing gates on the two
submaps being anywhere apart**, which is a statement about a proposal's value rather than its
validity.

> **Fixed as chunk [22a](22a-closure-travel-gate.md), 2026-09-04.** ⚠️ And this paragraph's own
> first instinct — *"a separation gate"* — **was the wrong fix, and measuring it is what caught
> that.** Gating the two submaps' Euclidean *separation* is the natural reading of "the closures
> were 10 cm apart" and would have broken loop closure outright: on the genuine set the median
> separation is 2.50 m and the **minimum is 0.02 m**, because a revisit *is* being in the same
> place. What separates the populations is **travel along the trajectory**, so that is what
> `MIN_SUBMAP_TRAVEL` measures. The episode drops 37 accepted closures to 1; the 5 Hz loop is
> byte-identical and suppresses nothing.

⚠️ It compounds 24a's superlinear-cost finding in the worst way: a stationary robot generates
submaps at the maximum rate and each proposal round is quadratic in the submap count, so **the
cheapest possible behaviour produces the most expensive possible search.**

### 3. `slam.csv` could not explain its own verdict, and now can

80% of the physics episode read "match not converged" beside a healthy score (0.5–0.83 against
`MIN_SCORE` 0.35) and a small translational ambiguity (0.05–0.08 m against `MAX_SPREAD` 0.25).
`converged` is the AND of three conditions and the file carried two, so the cause was reachable
only by elimination. **`match_ambiguity_theta` is now in `Tracker.diagnostics()`** and in the match
panel, plotted against its own limit alongside the other two, with a regression test asserting the
verdict stays reconstructible from the columns beside it.

⚠️ **`MatchResult`'s docstring was also wrong** — it said `converged` "means `score >= MIN_SCORE`",
which is the seductive half, and it is what made the first reading of the plot wrong. Fixed.

The new column shows two things on the *clean* loop, where 98.3% of matches converge: the heading
gate is by far the tightest of the three (0.0265 rad mean against a 0.05 limit — 53% of its limit
with no failures at all, versus a score at twice its limit throughout), and it is a **six-state**
gate, since `THETA_STEP` is 0.01 and the spread is always a multiple of it.

### 4. The heading error during a turn is scan staleness, not a bad match

9.3° peaks at every on-the-spot turn, and the natural reading — that matching degrades under
rotation — is wrong: while turning the match score is *higher* (0.718 vs 0.686) and the position
error is *lower* (0.067 vs 0.076 m). It is the held estimate going stale, exactly as
`SlamLocaliser._update` says it will: at the delivered 4.55 Hz one interval is 0.220 s, so a
1.0 rad/s turn carries up to 12.6° of staleness.

**A cost of the 5 Hz decision that 24a's tables could not show** — 24a measured the rate against
wall clock and final error, not against what a held estimate costs a rotating robot. It also
**independently confirms 24a's delivered-rate finding** from the trace rather than from the code:
0.2200 s per scan, 4.55 Hz for a requested 5, against the 4.53 Hz 24a derived from the accumulator.

### 5. The first episode in this repo where a localiser changed a primary metric

A `PerfectLocaliser` control was run — 30 s, and it turns the episode from an anecdote into a
finding. Same scenario, same controller, same seed, **same sensor noise**; only the localiser
differs:

| | goal | time | SPL | localisation failures | collisions |
|---|---|---:|---:|---:|---:|
| perfect pose | **reached** | 15.36 s | 0.950 | 0 | 1 |
| SLAM | **timeout** | — | 0.000 | **10** | 1 |

So the confound is ruled out rather than argued away: with the noise held on and the estimator
swapped, the episode solves at near-optimal efficiency. The identical collision count says the
same from the other side — that collision is a noise-or-controller artefact and is unmoved by the
localiser.

**The mechanism is one glance at `map.png`: the robot thinks it has arrived.** It believed itself
**0.017 m** from the goal at t = 18.5 s while actually **1.015 m** away, and its belief put it
inside `GOAL_RADIUS` on 65.5% of steps while it was truly inside on none. Everything after that —
the 40 s spin, the nine submaps in a 13 cm box, the 37 degenerate closures — follows from that one
belief.

⚠️ **A design decision is validated in passing:** the runner terminates on the TRUE distance to the
goal, not the believed one. Scoring on `pose_estimate` would have recorded this episode as a
success at t = 18.5 s.

⚠️ One episode, one seed, one scenario, and a debugging run. It shows a localiser *can* decide the
outcome, not how often it does; that is the sweep's job.

### A correction to chunk 24a's budget, found on the way

24a's `+control` column **doubles the SLAM hours for the mandatory `PerfectLocaliser` control, and
should not.** The control's localiser costs 0.015x real time against SLAM's 0.444x at 5 Hz — about
**3%** of the SLAM compute, not 100%. What doubles is the episode COUNT, and so the physics wall
clock, which that table has always said is on top and the same for every condition.

The error is conservative and flipped no verdict, so the 5 Hz decision stands untouched. It is
corrected in `out/evidence/slam-cost/README.md` as a dated append because the number will be used
to size a GPU rental, and over-provisioning the SLAM half by 2x is a real cost.

### What was NOT done in this chunk

**Nothing the pictures revealed was fixed here.** The chunk's own Not-in-scope says so: a closure
gate, a calibrated proxy and whatever should be done about a controller that spins for 40 s are
each design changes that move committed numbers.

**Finding 2 was then fixed on request, as chunk [22a](22a-closure-travel-gate.md)** — see the
inset above. ⚠️ Doing so did **not** fix finding 1: the proxy nearly doubled and still never
reaches `LOC_FAIL_THRESHOLD`, because its ceiling is `σ_odom · √(time since the last closure)` and
at σ = 0.12 that needs 69 closure-free seconds. Findings 3 and 4 stand as reported. Finding 1's
calibration and the σ question remain 25's.

### Cheap consequence worth keeping

The two physics episodes run before and after `match_ambiguity_theta` was added produced
**byte-identical traces**, which is a free confirmation that a diagnostics column is a diagnostics
column.

### Checks run, and what they said

Recorded because the eliminations are the expensive half of the work and are
invisible otherwise. ✅ = held, ⚪ = came back clean (nothing found), ❌ = failed and was fixed.

| # | Check | Outcome | Verdict |
|---|---|---|---|
| 1 | Recording is inert — full-precision pose stream, `error_estimate()` and `diagnostics()` across a recorded and an unrecorded run of one episode | identical, every step | ✅ |
| 2 | Same, under physics: two Isaac Sim episodes before and after adding `match_ambiguity_theta` | **byte-identical `trace.csv`** | ✅ |
| 3 | `points_at()` at the end of an episode vs `Submap.global_points()` | `np.array_equal`, not `approx` | ✅ |
| 4 | `.npz` round trip with `allow_pickle=False` | exact, including closures and solve scans | ✅ |
| 5 | Recorder scan index vs `slam.csv`'s `scans` column, asserted every step | equal — the join cannot drift | ✅ |
| 6 | `RecordedMap.points_at` vs `MapRecorder.points_at` | differed in the last bit at first (BLAS matmul vs open-coded rotation); made to go through `scan.transform` in both | ❌ fixed |
| 7 | Two graph snapshots at one scan index (submap finished *and* solved) | `poses_at` must return the later; first version of the test asserted otherwise | ❌ fixed |
| 8 | Full sim-free suite after the `slam.csv` column change | **727 passed, 0 failed, 0 skipped** | ✅ |
| 9 | `results/` ↔ `out/runs/` integrity after three local Isaac Sim runs | 39 passed | ✅ |
| 10 | `tidy_out.py --dry-run` — does it threaten the evidence folder? | keeps 31 pinned dirs, never enters `evidence/` | ⚪ |
| 11 | Did anything overwrite a **committed** result? | all five touched `results/*.json` are untracked; nothing committed was replaced | ⚪ |
| 12 | Chunk 11's procedural seal | `core/scenarios_held_out.py` never opened this session; `visualise_slam.goal_for()` searches `DEV_SCENARIOS` only and deliberately does **not** call `scenario.load()`, which would resolve a held-out id | ⚪ |
| 13 | Does `--slam-map` on a non-SLAM localiser break? | duck-typed, no-ops; `visualise_slam` on the control directory exits with a clear message rather than a traceback | ⚪ |

**Not checked:** determinism *across machines* (out of scope, and the project only claims
same-machine); the visualiser under a held-out scenario (none has ever been run); whether the mp4
renders correctly on any player but ffmpeg's own decoder.
