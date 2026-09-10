# Epic 03 — localisation as a real estimate

**Weeks:** a second cycle, after the first four-condition evaluation is run and written up.
**Chunks:** 19–27 · **Needs Isaac Sim:** no, for 19–26. See "Why this is sim-free".

## Why

`core/drift.py` is a seeded random walk added to the true pose. It was the right thing to
build — it unblocked a third of the card families for the cost of 148 lines — and it has one
property that is now the binding constraint on the whole experiment:

> **No controller can influence the recorded pose error.** Drift is injected open-loop, so
> `localisation_failures` is bit-identical across controllers that behave nothing alike, and a
> controller can lower it only by finishing sooner.

That was chunk 14's finding and it is why `cards/store.py` refuses any prediction naming
`localisation_failures`, and why the metric is a **manipulation check** rather than a
performance number. It is also why three of the description's own example leaves are either
untestable or testable only in a degraded form.

A real SLAM pipeline changes exactly one thing, and it is the right one: **the pose estimate
becomes a consequence of what the robot did.** A robot that keeps geometry in view, that revisits
a known place to close a loop, that slows where the scan is ambiguous, localises better than one
that does not — and for the first time the benchmark can see the difference.

### What it unlocks, concretely

| Leaf | Today | With SLAM |
|---|---|---|
| `IsLocalisationDegraded` | reads a number no action can change | reads a number its own tree's actions move |
| `MaintainHomingVector` | replaces the pose estimate; arbitration is untestable | `path_integration_mammals` predicts *arbitration against other cues* — that is what SLAM supplies |
| `DeviateToDisambiguate` | out of scope: legibility has no metric | a deviation that closes a loop has a **measurable** payoff in pose error. Different dependent variable, same mechanism |
| `RequireSensorAgreement` | structurally untestable, one modality | still blocked — needs chunk 27 |

### And what it costs the argument

Be honest about this in the write-up. Today the sim-to-real story is: *the locomotion policy is
inherited and frozen, so the gap is small.* That is true of the legs and says nothing about
perception, where a random walk is not a model of anything. SLAM closes the perception half of
the gap and, in doing so, **adds a second system that can fail**. A navigation difference and a
localisation difference become hard to separate. The mitigation is built in — see `PerfectLocaliser`
below — but it has to be used.

---

## The structural claim this epic rests on

**SLAM needs no change to what a controller sees.** It writes into
`Observation.pose_estimate` and `Observation.pose_error_estimate`, the two fields that already
exist, and **no node, no controller, no condition and no metric is rewritten.**

That is not luck. It is chunk 11a's decision paying off: `Observation` carries a *belief* and
never ground truth, precisely so the thing producing that belief could be replaced later without
touching anything written against it. `core/drift.py`'s docstring says as much — "drift can later
be injected into that one field and every controller becomes measurable with no rewrites
anywhere else". A localiser is the same substitution, one step further.

So the epic's shape is: **generalise the producer, leave the contract alone.**

```
                          ┌─ NoDrift            (today's default: estimate = truth)
Localiser (protocol) ─────┼─ DriftModel         (today's random walk — UNCHANGED)
                          ├─ PerfectLocaliser   (ground truth, as an experimental CONTROL)
                          └─ SlamLocaliser      (this epic)
                                   │
                                   └─► pose_estimate, pose_error_estimate
```

---

## Four decisions, and the alternatives rejected

### 1. Classical, not learned

Non-negotiable #2 forbids training any locomotion or navigation policy. Scan matching and pose
graph optimisation train nothing — they are geometry and least squares. **A learned VO front end
would void the non-negotiable**, and it would also make bitwise determinism far harder.

*Rejected:* a learned feature matcher, or an off-the-shelf deep VO. Better accuracy, and it
would turn "no model is trained in this project" into a sentence with a footnote.

### 2. 2D lidar SLAM, not camera VO

Three reasons, in order of weight:

- **It is sim-free.** A scan matcher over `core.geometry.raycast` runs under plain `pytest` with
  no simulator and no GPU. That is what makes it developable at all; the rest of `core/` was
  built this way for the same reason.
- **The sensor already exists** and is already the one every node reads. Nothing new enters
  `Observation`.
- **Rendering costs ~0.9 s per frame** and that is fixed overhead, not pixels. A camera pipeline
  multiplies every episode's wall clock by its frame count, and a seed sweep becomes impossible.

*Rejected:* camera VO first. It is chunk 27, it is bonus, and its real value is being a *second
modality* rather than being better.

### 3. The map stays internal to the localiser

SLAM builds a map. Exposing it would be a new `Observation` field and would change **what every
condition is able to do** — a robot with a map is a different experiment, not a better-localised
one. Phase 1 exposes pose and uncertainty only.

*Rejected:* exposing an occupancy grid or a `REGION` port of explored space. It is a defensible
future cycle and it needs its own pre-registration, because it moves the primary metric for
reasons unrelated to localisation.

### 4. The sealed split keeps its drift model and its meaning

G3 measures a frozen library against a frozen split. **Retrofitting SLAM onto the existing
held-out scenarios would void it retroactively** — the split was sealed at `00b3312b…` with a
2 × 2 of goal-disclosure × drift, and those scenarios mean what they meant.

SLAM scenarios are a **new split, sealed separately, in a later chunk**, and the cycle-1 numbers
stay comparable. Anyone tempted to "upgrade" the existing split should read chunk 11's Outcome
first.

---

## Why this is sim-free (19–26)

Everything in this epic except chunk 27 runs against `core.geometry`'s analytic raycast and the
kinematic rollout. No `isaacsim`, no `omni`, no GPU, no scene load, tested under `pytest` in
seconds per iteration. `core/tests/test_import_guard.py` already enforces that `core/` imports
none of those, and the SLAM modules live in `core/` and inherit the rule.

The eventual **numbers** still come from the pinned container, as everything reported does. But
the algorithm is developed and verified without it, which is the only reason a pose-graph back
end is a reasonable thing to attempt inside this project at all.

---

## Three risks, named before they are hit

**Determinism is the big one.** Same-machine bitwise determinism is a committed property, checked
by `scripts/check_determinism.sh`, and SLAM is exactly where it goes wrong: convergence
tolerances, variable iteration counts, RANSAC, dict iteration order, threading, and BLAS
parallelism. Every algorithmic choice in chunks 21–23 is made **in favour of determinism over
accuracy**, and chunk 24 is a gate that will not pass on a "usually identical" result.

**Cost.** Scan matching at the 50 Hz control rate over 181 beams, plus a graph optimisation, is
not free. Real SLAM runs at scan rate rather than control rate, and so should this: chunk 24a
measures the budget and sets the localiser rate, which is a realism improvement as well as a
cost one. **Settled at 5 Hz, on a laptop** (moved from 10 — see 24a's Decided for why the first
answer was wrong) — the localiser's cost is a pure function of the scans and poses it is handed, so
this was never a GPU question. The measurement also moved the dominant
cost: at the control rate **closure search costs twice what matching does**, because a proposal
round is a wider match against every nearby submap and submaps finish five times as often.

⚠️ **And it found a fourth risk the epic had not named: cost is SUPERLINEAR in episode length.**
A 4.2x longer episode costs 3.31x more *per simulated second*. Matching is linear; **closure search
is quadratic** (22x over 4.2x of episode), because `loop.candidates()` returns every finished
submap within `SEARCH_RADIUS` and lab42 is smaller than the sensor range — so the candidate set
grows with the episode. Two things follow. The held-out scenarios run 120–240 s and both measured
trajectories are shorter, so a budget extrapolated from them **under-reads in the unsafe
direction**. And the same rate costs 0.35x real time on a one-way traverse against 3.12x on a
trajectory that revisits, which means **the sweep's wall clock is a function of what the controller
does** — the same shape as the epic's central claim, and a scheduling problem rather than a
scientific one.

**The rate absorbed it and the sweep is affordable** — 10 -> 5 Hz, which is a *quadratic* lever on
closure search and saved 3.3-4.1x. ⚠️ **There is no second application of it**: 2 Hz is already
below any real 2D lidar. **The real fix is a bound on the candidate set per proposal round** —
best-N by distance, or a decay in revisit frequency. It is a design change with its own accuracy
cost, it needs a `Decided` section, and it is **the largest open design item in this epic**. It is explicitly *not* a knob to turn while
reading a budget table: chunk 24's own `Not in scope` says a budget that cannot be met is a finding
for the epic, not a licence to retune 21–23.

**Confounding.** Once every condition has a localiser that can fail, a difference in goal-reach
rate may be a difference in localisation rather than in navigation. `PerfectLocaliser` exists to
separate them: run the same condition twice, once with ground-truth pose, and the delta is the
localisation contribution. Chunk 25 makes reporting it mandatory rather than optional.

---

## The chunks

| # | Chunk | Depends on | Needs Isaac Sim |
|---|---|---|---|
| 19 | [The localisation contract](doing/19-localisation-contract.md) | — | no |
| 20 | [The sensor noise model](doing/20-sensor-noise.md) | 19 | no |
| 21 | [Scan matching: the front end](doing/21-scan-matching.md) | 19, 20 | no |
| 22 | [Submaps and loop closure](doing/22-loop-closure.md) | 21 | no |
| 23 | [Pose graph: the back end](doing/23-pose-graph.md) | 22 | no |
| 22a | [The closure travel gate](doing/22a-closure-travel-gate.md) | 22 | no |
| 24a | [The cost model and the scan rate](doing/24a-slam-cost-model.md) | 23 | no |
| 24b | [Seeing the SLAM run](doing/24b-slam-visualisation.md) | 23, 24a | no |
| 24 | [Determinism and cost gate](todo/24-slam-determinism.md) | 23, 24a | **yes** |
| 25 | [Recalibration: what moves](todo/25-slam-recalibration.md) | 24 | no |
| 26 | [The cards SLAM unlocks](todo/26-slam-cards.md) | 25 | no |
| 27 | [Camera VO](todo/27-camera-vo.md) — bonus, deferred | 24 | **yes** |

**19 is a gate, exactly as 11a was.** It is the single change to the `Scenario` contract, it
declares fields that later chunks implement without touching the field list, and nothing else in
this epic starts until it lands.

**24 is a gate.** A SLAM pipeline that is not bitwise reproducible cannot produce a reported
number in this project, and finding that out in Week 5 would be too late.

---

## The scheduling constraint

**This epic must not land between chunk 17 and the first evaluation run.**

A pre-registration is written against a specific measurement apparatus. Chunk 17 fixes G2's
threshold, seed set and episode count against the *drift* model; introducing SLAM changes what
`localisation_failures` measures, changes the achievable goal-reach rate on every drifted
scenario, and would make the pre-registered rule a rule about a different experiment.

Land it **after** cycle 1's evaluation is run and written up. Then it gets its own
pre-registration — chunk 25 collects the material — and cycle 2 is a clean comparison against a
cycle 1 whose numbers still mean what they said.
