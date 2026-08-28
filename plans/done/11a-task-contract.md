# 11a — The task contract: changed once, here

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** — · **Needs Isaac Sim:** no
**Blocks:** 11, and therefore 14–16.

## Why

Chunks 11 and 12 each defer this to the other and neither owns it:

> 11: *"`goal` is a required field of the frozen `Observation`… **Coordinate with chunk 12.**"*
> 12: *"If chunk 11 chooses task families where the goal is not known, `Observation.goal` must
> change shape. **Settle it here.**"*

It cannot be deferred further. Chunk 11 seals a split containing four task families, three of
which reference contract fields that do not exist yet, and a sealed split that will not load is
not sealed — it is broken. The epic is also explicit that this is a **one-time** change: *"must
be made once, now, not per-condition."*

So this chunk owns **every** frozen-contract change the held-out families need, made in one
sitting, before the split is authored. Nothing after this chunk changes `Observation` or the
`Scenario` field list. If something turns out to be missing, that is a finding worth recording,
not a licence to amend the contract mid-experiment.

## Decided

Four decisions, all pre-registration material, all feeding chunk [17](../done/17-pre-registration.md).

### 1. `Observation.goal` becomes `Optional[Tuple[float, float]]`

`None` means **the goal has not been disclosed to the controller**. It does not mean there is no
goal.

`Scenario.goal` stays required and unchanged. `core/metrics.py:187` and `run_episode.py:272`
score against it, so the runner always knows where the goal is and the episode still terminates
on true proximity to it. The change is *only* to what the controller is handed — which is the
same discipline `pose_estimate` already follows, and for the same reason: ground truth never
enters `Observation`.

Two call sites guard: `controllers/goal_seeker.py:44` and `controllers/reactive_avoid.py:66`.
Both are one line. `goal_seeker` is *expected to fail* and may simply hold `STOP` when the goal
is undisclosed; `reactive_avoid` is a harness-validation baseline and may do the same. Neither is
a benchmark condition, so neither needs a search behaviour — say so in their docstrings rather
than quietly giving a reference controller a capability the conditions have to invent.

### 2. Detection: line of sight, latched

`obs.goal` is `None` until the goal is **within `DETECT_RADIUS` and in line of sight**, then it
populates and **stays populated for the rest of the episode**.

```python
if not disclosed and dist(pose, scenario.goal) <= DETECT_RADIUS \
        and segment_clearance(pose_xy, goal_xy) > 0.0:
    disclosed = True        # latches; never falls back to None
```

`segment_clearance(a, b)` returns `0.0` exactly where the segment crosses a wall, so this is an
exact analytic occlusion test, not a sampled one — and it is the **same primitive the visibility
graph uses**, so detection and `optimal_distance()`'s notion of "can see through there" agree by
construction rather than by coincidence. Sim-free, so a unit scenario can exercise it.

Latching matters. Without it, `obs.goal` flickers to `None` every time the robot's own turn
occludes the goal, and every generated node would have to carry its own memory of a fact the
contract already established. The precondition a node conditions on is *"has the goal been found
yet"*, and that is monotonic.

`DETECT_RADIUS = 2.0` m, proposed. Rationale, to be frozen in chunk 17: it must sit well above
`GOAL_RADIUS = 0.5` so that detection is a distinct phase rather than a synonym for arrival, and
well below `RANGE_MAX = 6.0` so the divider and the y=5 wall genuinely hide the goal across most
of the room. A goal marker is a small object and is reasonably detected at shorter range than a
wall. **Do not tune this after the first evaluation run.**

### 3. `Scenario` gains the fields the four families need — declaration only

| Field | Default | Meaning | Implemented by |
|---|---|---|---|
| `goal_known: bool` | `True` | is `obs.goal` populated at t = 0 | this chunk |
| `detect_radius: float` | `DETECT_RADIUS` | detection range when `goal_known` is False | this chunk |
| `drift: Optional[DriftSpec]` | `None` | localisation drift | chunk [13](13-unit-harness.md) |

`DriftSpec` is declared here as a frozen dataclass with its fields and their units, because chunk
[11](11-held-out-split.md) seals scenarios that reference it and the sealed module must import.
Its *behaviour* is built in chunk 13.

**No `DisturbanceSpec`.** An earlier draft of the split carried a `resume_after_bump` family and
this chunk would have had to declare a scheduled-impulse type for it. Chunk 11 dropped that family
in favour of `search_degraded`, so the type is not declared — a contract field that nothing uses
is a contract field someone will use inconsistently later. If a bump family returns, it is a
contract change and it gets its own chunk, which is exactly the discipline this chunk exists to
impose.

### 4. A declared-but-unimplemented field is a hard error, never a silent no-op

`validate()` raises if a scenario sets `drift` or `disturbance` while the implementation is
absent. This is the whole reason declaration-only is safe: the failure mode it guards against is
a `return_degraded` scenario running happily with drift silently disabled and scoring
`localisation_failures: 0`, which is exactly the bug that is *already* in every committed result
and which epic 02 opens by naming.

Detection by feature probe, not by a version flag — the same shape as the `try/except` ladders in
`drive_go2.py`.

## Build

- `core/observation.py` — `goal: Optional[Tuple[float, float]]`, with the docstring saying what
  `None` means and that it latches. Extend the module docstring's *"ground truth never enters
  Observation"* paragraph to cover goal disclosure, since it is now the second instance of the
  same principle.
- `core/scenario.py` — the three fields, `DriftSpec`, `DETECT_RADIUS` under the
  data divider with the other pre-registration constants, and `validate()` raising per (4).
  `goal_known=False` with `detect_radius <= GOAL_RADIUS` is also a validation error: detection
  would coincide with arrival and the family would test nothing.
- `scripts/run_episode.py` — disclosure state in the episode loop, latched, written into the
  trace as a `goal_known` column so a re-score can tell when the robot found it. **Adding a trace
  column changes `TRACE_COLUMNS`** — check what that does to `scripts/rescore.py` reading older
  traces, and make old traces still re-scorable or say plainly that they are not.
- `controllers/goal_seeker.py`, `controllers/reactive_avoid.py` — the `None` guard and the
  docstring note.

## Done when

- `core/tests/` covers: `obs.goal is None` on a `goal_known=False` scenario at t = 0; it
  populates on the first step where both conditions hold; it never returns to `None` afterwards,
  including on a step where the goal is occluded again.
- A test places the goal behind the divider at a distance under `DETECT_RADIUS` and asserts it is
  **not** detected — occlusion, not just range. This is the test that fails if `segment_clearance`
  is used with the wrong sign.
- `validate()` rejects `goal_known=False` with `detect_radius <= GOAL_RADIUS`, and rejects a
  scenario setting `drift` before the drift model exists.
- **Every number in `results/` is unchanged.** All three dev scenarios default to
  `goal_known=True`, so the disclosure path must be provably inert for them — assert it with
  `scripts/rescore.py --dry-run`, do not assume it.
- `core/tests/test_import_guard.py` still green; nothing here needs a simulator.

## Not in scope

The drift model (13), authoring any held-out scenario (11), and any search *behaviour* — this chunk makes search expressible, it does not make anything search.

## Known consequence, recorded deliberately

With no cue field, a search scenario is a **coverage** problem: the robot explores until the goal
enters line of sight. Frontier coverage, path integration used to avoid re-searching, and
geometric reorientation are all testable against that. **Area-restricted search is not.** ARS
concentrates search around an encounter *rate*, and a single goal is a single encounter. Chunk
[14](14-first-cards.md)'s tier 3 must be read with that in mind: `RestrictSearchArea` becomes
testable only if a later cycle adds a cue field or multiple targets, and the honest write-up says
so rather than reporting it as a failed node.

`spl` is also near-meaningless for the search family — its denominator is the omniscient taut
path, which no honest search can approach. Goal-reach rate is the comparator there. Chunk 17 must
record that secondary metrics are reported **per family**, never pooled across all four.

---

## Outcome

**Done 2026-08-25.** All four decisions built as written. `core/tests/` 150 → **165 passed**,
no simulator. `results/` numerically untouched, and proven so twice over.

### The contract now

`Observation.goal` is `Optional[Tuple[float, float]]`; `Scenario` gained `goal_known`,
`detect_radius` and `drift`; `DETECT_RADIUS = 2.0` and `DriftSpec` are declared in
`core/scenario.py` under the data divider. `Scenario.goal` is unchanged and still scores every
episode. **This is the last change to the field list** — anything later that wants a fact the
contract does not carry is a finding to record, not a field to add.

### One thing the plan did not specify, and it mattered

The plan said the runner holds the latch. It did not say **where the rule itself lives**, and the
rule has two callers: `scripts/run_episode.py` and `core/tests/kinematic.py`. Two copies of a
detection rule is two disclosure schedules, and a node whose unit scenario discloses the goal on a
different step from the benchmark is a node whose unit scenario proves nothing about the
benchmark. So the rule is one method — `Scenario.disclosed_at(pose_xy)` — and both callers apply
it identically. The latch (one `or`) is duplicated; the rule is not.

`disclosed_at` is deliberately stateless. It answers "would the goal be disclosed to a controller
standing here", and the caller owns the memory, because `core/scenario.py` holds no episode state
and should not start.

### Line of sight

`segment_clearance(a, b) > 0.0`. It returns exactly 0.0 where the segment crosses a wall, so the
test is analytic rather than sampled, and it is **the same primitive the visibility graph is built
from** — so "can see through there" and `optimal_distance()`'s "can go through there" agree by
construction rather than by coincidence. It is deliberately not the inflated occupancy grid: a
sight line is a ray, not a corridor a body fits down.

Both bounds on `detect_radius` are enforced by `validate()` and both are load-bearing. Below
`GOAL_RADIUS`, detection coincides with arrival and the family tests nothing. Above `range_max`,
the goal arrives at a range the robot's own sensor could not have resolved it at — **that is
ground truth entering `Observation` through the back door**, and it is the failure this contract
exists to prevent, so it is a hard error rather than a convention.

### The trace column was the risky part, and the plan was right to flag it

Adding `goal_known` to `TRACE_COLUMNS` would have orphaned **every committed trace**:
`scripts/rescore.py:read_trace` compared the header against `TRACE_COLUMNS` with `!=` and raised.
Silently losing re-scorability was not acceptable — it is the property `core/metrics.py` opens by
claiming and the reason chunks 09 and 10 were executable at all.

Fixed by moving the parsing onto `EpisodeTrace.read_csv()`, beside `write_csv`, so the reader and
writer cannot drift apart, and by matching columns **by name** with a `TRACE_BACKFILL` for columns
added later. `goal_known` backfills to `True`, which is not a guess: every episode recorded before
the goal could be withheld ran on a scenario that disclosed it from t=0. Unknown columns and
un-backfillable missing ones are still hard errors.

A side benefit worth noting: chunk 10's round-trip test had to re-implement CSV parsing inline
because the reader lived in `scripts/`. It no longer does.

### Proof that nothing moved

Two independent checks, because "the change is inert" is exactly the claim that gets asserted and
not tested:

1. `scripts/rescore.py --dry-run` — **0 of 6 result files changed**, with 13-column traces read
   under a 14-column `TRACE_COLUMNS`.
2. Re-ran `blind_corner__reactive_avoid__seed1002` headless on the changed code. The new trace is
   **byte-identical in all 343 rows** to the committed one once `goal_known` is stripped, and
   `goal_known` is 1 throughout. That is stronger than (1): it shows the *simulation* is
   unperturbed, not merely that re-scoring is stable.

Local run, not the container — debugging evidence, not a reported number, and the runner said so
itself.

### Reference controllers

Both `goal_seeker` and `reactive_avoid` hold still when `obs.goal is None`, and both docstrings
say why at length. `reactive_avoid` has the sensing to explore and deliberately does not: giving
the success-side reference controller a search behaviour would quietly install a fourth,
unregistered baseline, and inventing exploration is what the generated conditions are *for*.

### Not covered by any test, and recorded rather than papered over

~~**The runner's own latch line is not executed by any test**~~ — **CLOSED 2026-08-25**, after
chunk 11 authored the search scenarios and `controllers/scaffold_sweep.py` gave the robot a reason
to move. Ran `held_search_past_corner` under the simulator: the latch flipped at step 284
(t = 5.68 s, pose (5.04, 6.37)), **0 of 284 rows disclosed before it and 169 of 169 after**, and
`disclosed_at()` agreed at the flip step while disagreeing one step earlier. So the runner fires on
the first permitted step, exactly as `core/tests/kinematic.py` does — the off-by-one this section
was written to worry about does not exist. The original text follows.

**The runner's own latch line was not executed by any test** when this chunk closed, because there
was no `goal_known=False` scenario until chunk 11 authored one. What is tested is `disclosed_at` (thoroughly, including
occlusion, range, and the inclusive boundary at exactly `DETECT_RADIUS`) and the identical latch
in `core/tests/kinematic.py` (fires on the first permitted step, never reverts, survives the goal
leaving detection range). The residual risk is one `or` in `run_episode.py`. Chunk 11 closes it by
existing; until then it is worth knowing rather than assuming.

`test_drift_is_rejected_until_the_drift_model_exists` **inverts when chunk 13 lands** — it skips
itself once `core.drift` imports, and the note in it says to retire it and assert the opposite.

### For chunk 11

`Scenario.disclosed_at` is the function to author search scenarios against. A start pose from
which the goal is already disclosed at t = 0 is not a search task, and `validate()` cannot catch
it because it is a property of the start/goal *pair* — chunk 11's Done-when already requires that
assertion, and `disclosed_at(s.start[:2])` is the one-liner that makes it.

### For chunk 17

`DETECT_RADIUS = 2.0` joins the frozen constants, with both bounds and their reasons.
