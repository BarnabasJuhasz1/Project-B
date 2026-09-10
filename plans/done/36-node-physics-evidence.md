# 36 — A single node under physics, and the first A/B evidence pair

**Status:** todo. Sim-REQUIRED (the only chunk in this epic that is).
**Blocks:** nothing. **Blocked by:** nothing — the machinery exists.

## Why

`harness/micro.py` says it in its own docstring: a micro-world is rolled out
kinematically, with the frozen locomotion policy absent entirely, so **a node
that passes its unit scenario may still fail under physics**. Admission is a
NECESSARY condition, never a sufficient one.

Seven generated nodes are admitted. **None has ever emitted a Twist through
`policy.forward`.** `out/evidence/nodes/` is empty and has been since it was
created. Every claim about the library is currently a claim about an idealised
unicycle.

This chunk closes that for the smallest interesting case: **one card, both
conditions, one environment, two videos.**

## What is NOT in scope

- **This is not G2 and produces no G2 number.** G2 is four conditions over the
  sealed split, 36 episodes each, scored by `core/stats.py`. This is dev-set
  evidence for `out/evidence/nodes/`, and every artefact it produces says so.
- No new generation calls. This runs nodes already admitted.
- No change to `Observation`, `Scenario`, the harness, or the import allowlist.
- Not the fixed-consumer harness that would make a Condition admissible — that
  is its own chunk and this one does not need it.

## The one change to the code

`controllers.make()` accepts a registry name or `composed:<id>`. It needs a
third form:

```python
NODE_PREFIX = "node:"        # node:<node_id> -> the admission tree, as a Controller
```

resolving to exactly the tree `generate/admit.py:build_tree()` already builds —
`Tree(Sequence(scaffold, [ContractPorts(...), node_cls(metadata)]))` — against
the library materialised from `generate/staging/`.

**Import `build_tree`, never re-declare the tree.** The whole value of the
result is that the thing under physics is the *same* tree the admission decision
was taken on. Two constructions would let them drift, and only a wiring failure
would say so.

`run_episode.py` passes `--controller` straight through, so **the runner does
not change**. Nor does `runs/spec.py`: a `node:` controller is not bound to any
condition, and `runs/spec.py` already refuses scaffolding — it should refuse a
bare `node:` binding too, for the same reason, and a test should pin that.

## The comparison, and the trap in it

⚠️ **A bare A-vs-B node comparison does not measure the conditions.** Measured
kinematically on 2026-09-07, on `doorway_traverse` and `blind_corner`:

| card | arm | node | reached | SPL | collisions | path |
|---|---|---|---|---|---|---|
| `reorient_geometry` | A | `ExtendedSurfaceDetour` | yes | 0.980 | 0 | 10.30 m |
| `reorient_geometry` | B | `ReorientToGeometry` | **no** | 0.000 | 8 | 36.03 m |
| `limit_speed_ics` | A | `ICSFilteredGoalDrive` | yes | 0.950 | 0 | 10.62 m |
| `limit_speed_ics` | B | `LimitSpeedToOcclusionBoundary` | **no** | 0.000 | 0 | **0.00 m** |

A wins both, and **the reason is not mechanism quality**. Every A node is a
self-contained navigator; every B node is a narrow single mechanism. B's
`LimitSpeedToOcclusionBoundary` travels zero metres because it is a speed
*limiter* with nothing to limit.

That asymmetry is itself the finding, and it points the opposite way to the
naive reading: **the card's Derivation pins one mechanism and B implemented
exactly that; a citation alone left A free to build a whole controller wearing
the paper's name.** A scores better partly by doing *more than its source
licenses* — `generate/FINDINGS.md` §1 (the harness cannot detect card
infidelity) showing up in behaviour instead of in a transcript.

**A shared carrier does not rescue it, and that was checked.** Wrapping each
node as `Sequence[ContractPorts, Fallback[NODE, toy_avoid_obstacles]]` changed
almost nothing: both nodes are Actions whose preconditions hold, so they return
RUNNING and the Fallback short-circuits before the carrier ticks — finding 1's
starvation, again. Only `LimitSpeedToOcclusionBoundary` moved at all (0.00 m →
5.24 m) and it still timed out.

**The root cause, stated once: the node contract cannot express a modifier.** A
speed cap is not a Condition (it must affect the Twist), not an Action (it does
not choose where to go), and not a Decorator as defined — a Decorator "governs
whether the child's [Twist] happens", never what it is. B's mechanism had
nowhere legal to live, so it was written as an Action with a built-in carrier,
which is the two-effect violation and the orphan port and the starvation, all
one cause. **This chunk reports that; it does not fix it.**

## Build

1. `controllers/__init__.py`: the `node:` form, via `admit.build_tree`. Refuse
   an id that is not an admitted, non-superseded, non-retired library entry.
2. A test: `node:<id>` resolves to a `Tree`, and to the same node ids as
   `build_tree`; `runs/spec.py` refuses a `node:` binding as a condition.
3. Four physics episodes on `doorway_traverse`, `--no-record` first, to confirm
   the nodes survive contact with the locomotion policy at all:
   `node:gen_a_69724a1f2c31010e`, `node:gen_b_4dc13c6a972bb06b`,
   `node:gen_a_98ae1bc5614b3aa5`, `node:gen_b_f199f233fd9cd65e`.
4. Re-run the chosen pair **with recording** at the default 15 fps. Budget: a
   15 s episode is ~240 s at 15 fps; a 120 s timeout episode is closer to half
   an hour, and **the B arm will time out**, so cap it — `--max-seconds 30`,
   which renames the scenario `doorway_traverse+cap30s` and cannot overwrite the
   committed full-length result.
5. `out/evidence/nodes/<card_id>-a-vs-b/` with both mp4s, both `result.json`s,
   a README saying what question it answers, what the answer was, the exact
   commands, and **the trap above in full** so the videos cannot be quoted as a
   condition effect.

## Done when

- `node:<id>` resolves, is tested, and is refused as a condition binding.
- Four scored physics episodes exist, each with `in_container:false` and
  therefore explicitly not reportable.
- One A/B video pair is in `out/evidence/nodes/` with a README carrying the
  caveat.
- The plan's Outcome records a **Checks** table, including whether each node's
  kinematic result survived physics — and `n`, which is 1 per cell.

## Open question for the reader

Every number here is local, so `in_container:false`. Re-running the four
episodes inside `nvcr.io/nvidia/isaac-sim:6.0.1` costs one image pull and makes
the evidence reproducible in the sense §6 requires. Worth doing in this chunk,
or deferred with the rest of the container work?

---

## ⚠️ Updated 2026-09-07 — five pairs are now available, not two

This plan was written when only `reorient_geometry` and `limit_speed_ics` had
both an A and a B node. Chunk 37's validity gate changed that: **all five pilot
cards are now paired**, ten nodes in `lib-019-ed0c5cc44084`, nothing to generate.

| card | A | B |
|---|---|---|
| `limit_speed_dwa` | `DynamicWindowDrive` | `LimitSpeedToVisibleRange` |
| `limit_speed_ics` | `ICSFilteredGoalDrive` | `LimitSpeedToOcclusionBoundary` |
| `path_integration_ants` | `PathIntegrationHoming` | `MaintainHomingVector` |
| `path_integration_mammals` | `PathIntegrationGoalVector` | `ArbitrateHomingVector` |
| `reorient_geometry` | `ExtendedSurfaceDetour` | `ReorientToGeometry` |

**The recommendation is unchanged: record the video pair on `reorient_geometry`**,
because it is the pair where both arms visibly move — A rounds the divider and
arrives, B wanders 36 m with 8 collisions. `limit_speed_ics` remains the cleaner
*diagnostic* (B travels 0.00 m) and the worse *video*.

**Run chunk 39 first.** It produces the kinematic table this chunk confirms under
physics, costs minutes rather than half an hour of rendering, and tells you which
pair is actually worth the render budget.

---

## Outcome — 2026-09-07

**Done. Seven physics episodes, three videos, and the first evidence in this
project that any generated node works under the frozen locomotion policy.**

Answer to the open question at the top of the plan: **deferred.** Every episode
is `in_container: false` and the README says so; re-running inside
`nvcr.io/nvidia/isaac-sim:6.0.1` is chunk 31's work and doing it here would have
made this chunk depend on the container pull.

### The one code change, as specified

`controllers.NODE_PREFIX = "node:"` plus `controllers.library_nodes()`.
`generate.admit.build_tree` is **imported, never re-declared**, and
`core/tests/test_controllers.py` pins that `node:<id>` produces the same leaves
in the same order as `build_tree` — the whole value of the result is that the
thing under physics is the thing the admission decision was taken on.

`load_library(only=[node_id])` is what refuses an id: it reads the same
committed records `pass_rate` and `compose.library` read, so "in the library" has
one definition rather than two. Superseded (`gen_b_1f2fe8016b1441dc`), retired
(`gen_a_4b6fe714f7154470`) and unknown ids all fail; the composition-oriented
`LibraryReconstructionError` is re-raised as `KeyError` so the message is not a
non-sequitur. `runs/spec.py` refuses a `node:` binding as a condition, with a
test. The runner did not change.

### The numbers

`doorway_traverse`, seed 1001, headless, 60 s cap (`doorway_traverse+cap60s`).

| card | arm | node | reached | SPL | coll | stuck | path |
|---|---|---|---|---|---|---|---|
| `reorient_geometry` | A | `ExtendedSurfaceDetour` | **yes** | 0.975 | 0 | 0 | 10.35 m |
| `reorient_geometry` | B | `ReorientToGeometry` | no | 0.000 | 1 | 1 (49.6 s) | 6.83 m |
| `limit_speed_ics` | A | `ICSFilteredGoalDrive` | **yes** | 0.931 | 0 | 0 | 10.85 m |
| `limit_speed_ics` | B | `LimitSpeedToOcclusionBoundary` | no | 0.000 | 0 | 0 | 0.04 m |

Kinematic → physics: A/`ExtendedSurfaceDetour` 0.980 → 0.975,
A/`ICSFilteredGoalDrive` 0.950 → 0.931, B/`LimitSpeedToOcclusionBoundary`
0.00 m → 0.04 m with the precondition firing **0 of 3000** steps. **The plan's
prediction held: nothing that passed kinematically failed under physics.**

### ⚠️ The finding the plan did not anticipate

**A verdict surviving physics is not the same as a failure mechanism surviving
it.** `ReorientToGeometry` fails both ways, but the diagnosis is different:

| | kinematic | physics |
|---|---|---|
| path | 36.03 m | 6.83 m |
| collisions | 8 | 1 |
| stuck events | 0 | **1, for 49.6 of 60 s** |
| min clearance | — | **0.005 m** |

The unicycle slides along walls and accumulates useless path; the Go2 **pins
itself against the divider and stops**. Two of `PRE-REGISTRATION.md`'s four named
secondaries — `stuck_events` and `recovery_time` — are **zero in every kinematic
row and non-zero here**, so a failure characterised on the unicycle is not the
failure the robot has. This matters beyond one node: `harness/ablate.py`,
`scripts/ab_compare.py` and every admission decision run on that unicycle.

### Checks

| check | outcome | verdict | n |
|---|---|---|---|
| Does `node:<id>` resolve to the same leaves as `build_tree`? | yes, leaf-for-leaf in tick order | **confirmed** | 1 node, asserted |
| Does every live library node resolve and expose reset/step? | yes | **confirmed** | 14 |
| Are superseded / retired / unknown ids refused? | all three raise `KeyError` | **confirmed** | 3 |
| Is `node:observation_ports` refused? | yes — the port publisher drives nothing | **confirmed** | 1 |
| Does `runs/spec.py` refuse a `node:` condition binding? | yes | **confirmed** | 1 |
| Do `node:` names leak into `available()`? | **no** | **ruled out** | 14 |
| Did each kinematic verdict survive physics? | **4 of 4** | **confirmed** | 4 episodes, n=1 per cell |
| Did each kinematic failure *mechanism* survive physics? | **3 of 4.** `ReorientToGeometry`'s did not | **confirmed (negative)** | 4 |
| Does recording perturb physics? | **no** — recorded A `trace.csv` byte-identical to the `--no-record` run; recorded 30 s B identical to the 60 s B run's first 30 s | **ruled out** | 2 pairs |
| Did the 60 s cap bite on either A run? | no — both reached the goal at 16.5 s / 14.4 s | **ruled out** | 2 |
| Full sim-free suite | **796 passed, 1 failed** — the pre-existing `test_no_papers.py` port-name gap, untouched | **confirmed** | 797 |

**Not checked:** any scenario but `doorway_traverse`; any seed but 1001 (the
scenario is undrifted and chunk 39 measured 60/60 groups seed-invariant, so this
is a cheap omission rather than a gap); the other three paired cards under
physics; anything in a container; whether either A node's *paper mechanism* does
the work — chunk 39 already showed `ExtendedSurfaceDetour` reaching the goal with
Cheng's mechanism ablated, and nothing here re-opens that.

### One gap found, deliberately not fixed

`manifest["library_snapshot"]` is **null** in all seven runs. `run_episode.py`
takes it from `--tag`, and a `node:` (or `composed:`) controller could derive it
— it materialises the library to resolve the id at all. Left alone because this
chunk's scope says the runner does not change, and because these runs are
`in_container: false` and therefore already non-reportable. **It would matter to
a run that is reportable (#8), so it belongs to chunk 31.**

### What this hands forward

Nothing here weakens chunk 40's case; it strengthens it. The physics run shows
the same two failure shapes the contract forces on B — `ReorientToGeometry`
drives and starves everything else, `LimitSpeedToOcclusionBoundary` publishes
into the void and never fires — now with a video of each. **40 is still next.**

---

## Addendum — 2026-09-07: *why* B fails, diagnosed rather than described

The Outcome above reports that both B nodes fail. Asked why, the answer is
**three different causes, none of them "the mechanism is wrong"**, and one of
them is a repo-wide defect.

### `ReorientToGeometry` (B, Cheng 1986) — its mechanism never ran

The node is deliberately `goal_seeker` plus one recovery: its own docstring says
the cruise behaviour is *"deliberately sensor-blind goal seeking, so that the
reorientation episode is the only thing distinguishing it from the goal_seeker
baseline."* That is **card fidelity, not incompetence** — the card's mechanism is
geometric reorientation *from a stuck state*, and the node isolates exactly that.

Under physics the recovery **fired zero times in 3000 control steps.** The robot
wedged against the divider at t≈10 s and pushed at 0.5-0.6 m/s for the remaining
50 s. Cause: the stuck window can only pass on exact float equality, and the
runner's accumulated `t` never ties where the harness's `step * dt` does — see
**`plans/todo/41-timebase-mismatch.md`**, which this diagnosis opened.

Measured, same node, three timebases:

| where | stuck verdicts | outcome |
|---|---|---|
| its own unit micro-world (`step * dt`) | 1 | **reaches the goal** |
| `doorway_traverse` kinematic (`step * dt`) | 8, four phase transitions | fails, 36 m |
| `doorway_traverse` physics (`t += dt`) | **0** | fails, pinned 49.6 s |

So the 36 m of wall-sliding in the chunk-39 table and the 49.6 s of pinning here
are **not the same failure at all**: kinematically the mechanism runs and does
not save it; under physics the mechanism is never reached.

### `LimitSpeedToOcclusionBoundary` (B, Fraichard & Asama 2004) — two causes

**(a) The trigger is mis-tuned for lab42, and it is a near miss.** Probing the
precondition directly at four poses — including the robot's nose 0.23 m from the
divider and standing at the blind corner — it is **False everywhere**. It is not
that no occlusion boundary exists: at the divider four are found, but at ±79-87°,
*beside* the robot rather than in the 0.42 m-half-width forward corridor the
distance is measured in, so the corridor distance is `inf`. At the blind corner
eight are found and the nearest in-corridor one is **1.994 m against a 1.600 m
threshold** — 0.39 m short.

**(b) The self-lock, which is the structural half.** The node put the
*mechanism's* trigger in `precondition` and its *carrier* (gap-choice steering) in
`tick`. So it can only steer while the cap is needed — and the cap is needed only
near a corner, which it can only reach by steering. Precondition False → the
`Sequence` fails → **nothing drives**, 0.04 m, which is the robot settling.

That is the modifier problem stated as a mechanism rather than as an argument: a
speed cap's natural form is *always run, modify what someone else produced*. Forced
to be an Action with one precondition, its author had exactly two places to put
things and both are wrong. **This is chunk 40's case, made concrete.**

### What this does and does not say about condition B

It does **not** say the corpus does not help. Both B nodes implement their card
faithfully; one was disabled by a float comparison in the repo's own timebase, the
other by a threshold 0.39 m off and by a contract with nowhere to put a modifier.
**None of the three causes is "the paper's mechanism does not work"** — and note
that no measurement in this project has yet tested that claim for either card.

### Checks (addendum)

| check | outcome | verdict | n |
|---|---|---|---|
| Does `ReorientToGeometry`'s stuck detector fire under physics? | **0 of 3000 steps** | **confirmed (negative)** | 3000 |
| Would it fire on the CSV's 6-dp timestamps? | yes, 804 times, first at 33.2 s | **confirmed** | 3000 |
| Does it fire under the kinematic rollout? | yes, 8 verdicts, 4 phase transitions | **confirmed** | 1 episode |
| Does it fire in its own unit micro-world? | yes, once — and it reaches the goal | **confirmed** | 1 |
| Size of the timebase difference | **3.78e-12 s** over 60 s | **confirmed** | 3000 |
| Is `LimitSpeedToOcclusionBoundary`'s precondition ever True in lab42? | **no**, at 5 probed poses incl. the blind corner | **confirmed (negative)** | 5 poses |
| Are there occlusion boundaries at all? | yes — 4 at the divider, 8 at the corner; they fall outside the forward corridor or 0.39 m beyond threshold | **ruled out** ("no boundaries" was the wrong hypothesis) | 2 poses |
| Does the node drive when its precondition is False? | **no** — the Sequence fails and nothing drives | **confirmed** | 3000 |

**Not checked:** whether fixing the timebase makes `ReorientToGeometry` succeed
(chunk 41 step 3 — firing is not working); whether the ICS thresholds are wrong
or lab42 is simply outside the geometry the paper addresses; the other three B
nodes, whose failures in the chunk-39 table have not been diagnosed at all and
may have entirely different causes.
