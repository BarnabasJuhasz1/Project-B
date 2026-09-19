# 22a — The closure gate that measures travel, not submaps

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 22 · **Found by:** [24b](24b-slam-visualisation.md) ·
**Needs Isaac Sim:** no to build, yes to confirm

## Why

Chunk [24b](24b-slam-visualisation.md) drew a physics episode and found this: the robot turned on
the spot for 40 seconds, finished **nine submaps inside a 0.13 x 0.19 m box**, and closed
essentially all of them against each other — **37 proposals, 37 accepted, 0 rejected**, median
endpoint separation 0.105 m.

Every one of those closures is *valid*. The scans genuinely match, because they are scans of the
same place taken from the same place, and all four of `loop.py`'s gates are correctly satisfied.
They are also worthless: a constraint saying "node 11 is where node 12 is" cannot correct an error
accumulated between node 0 and node 6.

**`loop.py` already has the right idea, in the wrong currency.** `MIN_SUBMAP_GAP`'s comment says
it exactly:

> *"A submap this recently finished is not a loop, it is the trajectory. Without this every submap
> would 'close' against its immediate predecessor, which is odometry wearing a different name."*

That is correct, and "recently finished" is measured in **submap indices**. A submap boundary is a
pure function of scan count (`submap.py`, and for a good reason — it is what makes two identical
runs cut them identically), so **a stationary robot finishes submaps without going anywhere.**
Eight submaps of standing still is an index gap of eight and a trajectory of 0.3 m, and closing
across it is precisely "odometry wearing a different name" — the thing the gate exists to prevent,
walking straight through it.

It also compounds chunk [24a](24a-slam-cost-model.md)'s superlinear-cost finding in the worst
possible way: closure search is quadratic in the submap count, a stationary robot generates
submaps at the maximum rate, and so **the cheapest possible behaviour produces the most expensive
possible search.**

## Decided

**The gate is on TRAVEL ALONG THE TRAJECTORY between the two submaps, and the threshold is 2.0 m.**

`MIN_SUBMAP_TRAVEL = 2.0`, applied in `candidates()` beside `MIN_SUBMAP_GAP`, measured as the
polyline length through the intervening submap origins plus the last hop to the current estimate.

**It is not a new policy. It is `MIN_SUBMAP_GAP` expressed in a currency that does not lie when the
robot stands still.** Measured across both of 24b's runs, ordinary driving covers **0.92–1.73 m per
submap**, so `MIN_SUBMAP_GAP = 2` already means *about two metres of travel* whenever the robot is
moving. 2.0 m therefore leaves the moving case as it was and only bites where the index gap was
always a bad proxy.

The measured separation is wide, which is why a threshold can be picked at all:

| | closures | travel: min | median | max |
|---|---:|---:|---:|---:|
| `loop/` — genuine lap-to-lap revisits | 59 | **3.00 m** | 13.98 | 29.38 |
| `episode/` — the stationary cluster | 37 | 0.16 m | **0.47 m** | 3.81 |

At 2.0 m the loop keeps **59 of 59** with a 1.5x margin below its own minimum, and the episode
keeps **1 of 37** — the single genuine long-range edge (nodes 3→5, 3.81 m).

### ⚠️ Rejected: a gate on the Euclidean separation between the two submaps

This is the obvious reading of 24b's finding — "the closures were 10 cm apart, so refuse closures
under N cm" — and it is **exactly wrong**. Measured on `loop/`, where every closure is genuine:
median Euclidean separation 2.50 m but **minimum 0.02 m**. A revisit *is* being in the same place;
that is what the word means. A Euclidean gate at even half a metre would throw away real loop
closures while leaving the stationary cluster's 3.81 m outlier untouched. Travel separates the two
populations; distance does not separate them at all.

*Also rejected:* a cap on closures accepted per proposal round. It bounds the cost but not the
harm — the surviving closures would still be the degenerate ones, chosen arbitrarily.

*Also rejected:* raising `MIN_SUBMAP_GAP`. It buys nothing here. The cluster spans index gaps of
2 to 9 and raising the gap to 10 would break genuine short loops elsewhere.

### Where it goes, and why it is still visible

**In `candidates()`, not as a fifth gate in `propose()`.** A closure over 0.3 m of trajectory is
not a rejected proposal, it is a non-candidate — the same category as `MIN_SUBMAP_GAP`, and the
same reason: it is a topological test needing no match. Putting it before the match is also the
whole cost win, since a proposal *is* a correlative match against a full submap.

But `loop.py`'s docstring says every rejection is counted by gate, because *"the loop was there and
we did not see it"* and *"we saw it and rejected it"* are different failures. A silent candidate
filter would erase that, so **`ClosureLog` gains a `suppressed` counter**, `slam.csv` gains
`closures_suppressed_travel`, and `closures.png` draws it as its own bar in its own colour —
adjacent to the rejections and not one of them.

## Predictions, written before running

Stated in advance so the confirmation is worth something. **If any of these is wrong the gate is
doing something other than what it is for, and that is the finding.**

1. **`loop/` is untouched.** All 59 closures survive; final error stays at 0.055 m.
2. **`episode/` drops from 37 accepted closures to 1**, the 3.81 m edge.
3. **`episode/`'s uncertainty proxy stops sawtoothing during the spin and rises monotonically**,
   crossing `LOC_FAIL_THRESHOLD`. This is the interesting one: 24b's finding 1 — the proxy reading
   *low* through the failure — would then be a **consequence** of this bug rather than a separate
   defect, and would be fixed without touching `error_estimate()` at all.
4. **`episode/`'s realised error is roughly unchanged (~1.1 m final).** The suppressed closures were
   no-ops, so removing them should not move the estimate much. ⚠️ A large change either way means
   they were not no-ops and the whole diagnosis is wrong.
5. **Closure search gets cheaper on `episode/`**, and not on `loop/`.

## Build

- `core/slam/loop.py` — `MIN_SUBMAP_TRAVEL`, `travel_to_pose()`, the filter in `candidates()`,
  `ClosureLog.suppress()` and the new counter.
- `core/slam/tracker.py` — pass the log through so suppressions are counted where they happen.
- `scripts/visualise_slam.py` — the suppressed bar in `closures.png`.
- `core/tests/test_slam_loop.py` — the travel gate, and `min_travel=0.0` on the four existing
  candidate tests whose fixtures stack every submap on one origin (they isolate the index gate and
  the radius; the travel gate would now suppress all of them, which is the correct new behaviour
  and not what those tests are about).

## Done when

- The five predictions are checked and whatever actually happened is written down.
- `loop/` and `episode/` are regenerated, and the physics episode is re-run.
- The full sim-free suite passes.

## Not in scope

**The misregistration itself.** `episode/`'s realised error passed 1.0 m at t ≈ 15 s, *before* the
spin started. This gate cannot fix that and prediction 4 says so. Why the map shears on that
trajectory is a separate question.

**`error_estimate()`'s calibration.** If prediction 3 holds, the proxy is fixed *here* for the
stationary case and is still 3.6x conservative on `loop/`. That remains 25/26's decision.

**Whether a controller should spin in place for 40 seconds.** Not a SLAM question.

---

## Outcome — 2026-09-04

**Done. Four of the five predictions held; the third was too optimistic, and being wrong about it
produced a better statement of the underlying defect than being right would have.**

### The predictions, scored

| # | Prediction | Result | |
|---|---|---|---|
| 1 | `loop/` untouched, all 59 closures survive | **Stronger than predicted: the gate suppressed ZERO candidates and the estimate stream is byte-identical.** A trajectory that genuinely loops never proposes a sub-2 m closure at all | ✅ |
| 2 | `episode/` drops from 37 accepted closures to 1 | exactly that: 37 → **1 accepted, 36 suppressed before matching** | ✅ |
| 3 | the proxy stops sawtoothing and crosses `LOC_FAIL_THRESHOLD` | **half right — see below** | ⚠️ |
| 4 | realised error roughly unchanged | 1.135 → 1.214 final, 0.833 → 0.867 mean, inside the ±0.37 seed spread. The suppressed closures were no-ops, as diagnosed | ✅ |
| 5 | closure search gets cheaper | 37 correlative matches against full submaps → **1**. The quadratic term is gone | ✅ |

### ⚠️ Prediction 3 was wrong, and the reason is the more useful finding

The proxy did improve — mean **0.236 → 0.448 m**, peak **0.557 → 0.740 m**, nearly doubling. It
still **never reaches 1.0 m**, on 0.000 of steps, exactly as before. `IsLocalisationDegraded` would
*still* not have detected this failure.

The arithmetic says why, and it is not about closures at all. The proxy is
`σ_odom · √(seconds since the last accepted closure)`. At σ = 0.12 m/√s the observed peak of
0.740 m is `0.12 · √39` to three figures — the one surviving closure lands at t ≈ 21 s and the
episode ends at 60. **So the proxy cannot reach 1.0 m until 69 seconds have passed with no closure
at all**, and most held-out scenarios' `time_limit` is 120–240 s but few episodes will run that
long without one.

That reframes 24b's finding 1. The problem is not only "the proxy resets on worthless closures" —
fixed here — it is that **the proxy's ceiling is set by σ_odom and the episode length, and at the
current σ it is structurally unable to cross the threshold a card would read it against.** That is
a calibration question for 25, now with a formula attached rather than a shrug.

### Checks run, and what they said

✅ held · ⚪ came back clean · ❌ found something

| # | Check | Outcome | |
|---|---|---|---|
| 1 | Does the gate touch a genuinely looping trajectory? | 0 suppressions, byte-identical estimate stream | ⚪ |
| 2 | Are the two populations actually separable, or was the threshold fitted? | genuine closures: min travel **3.00 m**; degenerate cluster: median **0.47 m**. Any threshold in [1.0, 3.0) works; 2.0 has 1.5x margin | ✅ |
| 3 | Would the *obvious* fix — gating Euclidean separation — have worked? | **No, it would have broken loop closure.** Genuine closures have median separation 2.50 m and **minimum 0.02 m**: a revisit *is* being in the same place | ❌ rejected before implementing |
| 4 | Is 2.0 m a free parameter? | ordinary driving covers 0.92–1.73 m/submap, so `MIN_SUBMAP_GAP = 2` already means ≈2 m while moving. The gate restates the existing constant in an honest unit | ✅ |
| 5 | Do chunk 22's four existing `candidates()` tests still mean what they say? | **No** — their fixtures stack every submap on one origin, which the gate now suppresses. Passed `min_travel=0.0` explicitly and said why, so each still isolates the index gate and the radius | ❌ fixed |
| 6 | Is a suppression counted, and kept out of the rejections? | own counter, own `slam.csv` column, own bar in `closures.png`; `rejected` untouched | ✅ |
| 7 | Full sim-free suite after a new `slam.csv` column | see below | ✅ |
| 8 | Is the before/after episode a controlled comparison? | **No.** The controller steers on `pose_estimate`, so changing the estimate changed the trajectory. The error comparison in prediction 4 is therefore "unchanged within seed noise", not "unchanged" | ⚪ stated, not fixed |
| 9 | Are the two cost-table runs comparable? | **Not at first** — the committed table was `OMP=1 MKL=1` and the first re-run was unpinned, which 24a's own header says changes the timing. Re-ran pinned | ❌ caught and fixed |
| 10 | Is a wall-clock difference attributable to the gate at all? | **Mostly no.** A known byte-identical row still moved 17% between invocations, so that is the noise floor; only the 50 Hz traverse (−61% closure search) clears it | ⚪ limits what may be claimed |

### ⚠️ A separate alarm raised during this chunk, and retracted

While diagnosing why the episode's estimate is poor, a single-seed comparison showed **odometry
beating SLAM** (0.570 vs 1.046 final). Over five seeds it does not:

| over 5 seeds | mean error | final error | worst seed |
|---|---:|---:|---:|
| odometry | 1.302 ± 0.881 | 1.593 ± 1.186 | 3.014 |
| SLAM | **0.876 ± 0.368** | **1.121 ± 0.497** | **1.319** |

SLAM is better on average with **half the variance** — it caps the error where odometry is a
lottery. Seed 1001 is odometry's luckiest of the five. **The single-seed table should not have been
quoted without n.** Reproducible now from a committed script:
`compare_localisers.py --from-trace <trace.csv> --seeds 1001,2002,3003,4004,5005`.

⚠️ **But the drift model still beats SLAM on this trajectory** (0.468 mean vs 0.876), and that is
not retracted. It is not a like-for-like comparison — `core/drift.py` adds a zero-mean random walk
to the *true* pose, so it is anchored to ground truth by construction and has no accumulating bias
— but it means **SLAM's advantage over the model it replaces is contingent on revisiting**: 11x
better on the two-lap loop, ~2x worse on a one-way traverse. Whether the held-out scenarios induce
revisiting therefore decides whether epic 03 delivers, and that is chunk 25's question.

### ⚠️ What the gate could invalidate, and whether it did

A change to closure acceptance can move any committed number produced by running SLAM. Two
artefacts were at risk and both were re-run rather than assumed:

| Artefact | Risk | Result |
|---|---|---|
| `out/evidence/localisers/` | its table is `compare_localisers.py --scan-rate 10`, and at 10 Hz the loop lays ~0.75 m per submap, so index-gap-2 pairs span ~1.5 m and now fall **under** the 2.0 m gate | **re-run. One number moved, and it improved** ⚠️ |
| `out/evidence/slam-cost/` | 24a's cost tables sweep 50/20/10/5/2 Hz; the denser rates make short-travel pairs, so the gate makes the FAST rates cheaper and narrows 5 Hz's measured advantage | **re-run thread-pinned. Accuracy improves at the fast rates, is untouched at 5 Hz, and every budget verdict stands** |

**`out/evidence/localisers/`, re-run at the same `--scan-rate 10`:**

| | before | after |
|---|---|---|
| traverse, all three rows | 0.221 / 0.133 / 0.322, `submaps=8 closures=4/6 solves=4` | **identical throughout** |
| loop SLAM, final & max | 0.053, 0.119 | **identical** |
| loop SLAM, **mean** | 0.055 | **0.052** |
| loop closures | 338 accepted / 379 proposed, 37 solves | **298 / 346, 34 solves** |

The gate removed 33 proposals and 40 accepted closures — about 12% — and the mean error **fell**.
That is the expected direction and it is a second, independent confirmation that the suppressed
closures carry no information. The evidence README carries a dated amendment.

**`out/evidence/slam-cost/`, traverse, re-run.** ⚠️ Only the DETERMINISTIC columns are compared:
the committed table was produced with `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1` and the re-run was
not, and 24a's own header says thread count is "irrelevant to the RESULT... not irrelevant to the
TIMING". A separate thread-pinned run is what the wall-clock columns need.

| rate | final err before → after | mean err before → after |
|---|---|---|
| 50 Hz | 0.147 → **0.123** | 0.067 → **0.059** |
| 20 Hz | 0.229 → **0.138** | 0.110 → **0.078** |
| 10 Hz | 0.221 → 0.221 | 0.133 → 0.133 |
| 5 Hz | 0.364 → 0.364 | 0.225 → 0.225 |
| 2 Hz | 0.529 → 0.529 | 0.302 → 0.302 |

**The gate makes the fast rates MORE accurate and leaves 5 Hz untouched** — at 5 and 2 Hz the
traverse proposes no closures at all, so there is nothing to suppress. Nowhere does it make
anything worse. That is a third independent confirmation, on a trajectory the threshold was not
chosen against.

The loop half says the same, and 20 Hz gains the most: final error **0.057 → 0.037**, mean
**0.058 → 0.048**. **Every budget verdict is unchanged — 5 Hz and 2 Hz affordable, 10 / 20 / 50 Hz
not — so chunk 24a's rate decision stands.** Raw output committed as
`out/evidence/slam-cost/after-22a.txt`.

⚠️ **The wall-clock columns needed a noise floor before anything could be claimed from them, and
one was available for free.** The 5 Hz loop row is a *known byte-identical run* — same closures,
same estimate, verified — and it still moved **0.444 → 0.367 x real, 17%**, between the two
benchmark invocations. So run-to-run variation on this machine is ~17% even with threads pinned,
and only the 50 Hz traverse change clears it (closure search 69.58 s → 27.04 s, **−61%**).
Everything else is recorded as inconclusive rather than as a speedup. Using a row that *cannot*
have changed as the calibration was cheaper than repeat runs, and it is the reason the other rows
are not claimed.

⚠️ **This is the interaction to keep in mind: the gate bites harder the faster the scan rate**,
because rate sets submaps-per-metre. At 5 Hz it suppresses nothing on the loop; at 10 Hz it
removes 12%; at 50 Hz it would remove most. That is not a defect — the index gap was the
rate-dependent quantity and metres are not — but it means the chosen rate and this threshold are
now coupled, and moving either requires re-running the other's evidence.

⚠️ **The threshold is in metres and the submap density is set by the scan rate, so the gate is not
rate-neutral by construction — it is rate-neutral *by design*.** Being in metres is the whole
point: the index gap was the rate-dependent quantity, and 2.0 m means the same thing at 5 Hz and
at 50 Hz where `MIN_SUBMAP_GAP = 2` does not. But that is precisely why any table that swept the
rate had to be recomputed rather than argued about.

**Not checked:** whether the gate helps on any trajectory other than these two; the right value of
σ_odom; anything about the misregistration that put the error above 1 m before the spin began.
