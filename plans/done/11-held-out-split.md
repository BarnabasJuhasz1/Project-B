# 11 — Seal the held-out split

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** [11a](../done/11a-task-contract.md) · **Needs Isaac Sim:** no

**This is a gate.** G3 measures "the fraction of held-out scenarios solved by recombination
alone". A split sealed after generation has begun measures nothing, and cannot be repaired
afterwards by any amount of care. Nothing in chunks 14–16 may run until this is done.

## Why

`core/scenario.py` carries `DEV_SCENARIOS` and a comment that has been waiting since chunk 03:

> The held-out set is deliberately NOT authored here — it must be sealed before any generation
> happens, and is the next task after this epic.

`Scenario.split` already exists and is `"dev"` on all three. Nothing is held out yet.

## Decided — option B, twelve scenarios, four families

**What varies: routes *and* task families.** Not a different building — non-negotiable #10 is
"one unmodified benchmark, one environment", and multi-environment generalisation is out of
scope. Option A (new start/goal pairs only) was rejected because the doorway and the blind corner
are lab42's only two features, so a library that solves `doorway_traverse` solves any other route
through the same doorway and G3 would measure close to nothing. Option C (furniture) was
considered and **declined** for this cycle: it is the strongest test, but it needs a second USD
variant, a second `usd_sha256`, and a careful restatement of what "unmodified" means, and none of
that buys anything B does not already buy.

**Twelve scenarios, three per family**, against the three dev ones. Three per family is the
smallest count at which a per-family breakdown is reportable rather than anecdotal; the pooled
G3 fraction then has thirteen possible values. This number is an input to chunk
[17](../done/17-pre-registration.md)'s episode-count calculation — 12 scenarios × the seed set is N per
condition, and N sets the smallest detectable X. Do not change it after that table is read.

### The four families are a 2 × 2

|  | `goal_known = True` | `goal_known = False` |
|---|---|---|
| **no drift** | `navigate_to_goal` (3) | `search_unmapped` (3) |
| **drift on** | `return_degraded` (3) | `search_degraded` (3) |

This is deliberate and it is the most useful property of the split. Disclosure and drift are the
two axes 11a's contract exposes, they are crossed rather than confounded, and so the contribution
of each mechanism family is separable from the results without a separate ablation.

- **`navigate_to_goal`** — three routes that are not the dev routes. Tests that primitives
  transfer across geometry. The weakest family on its own, and it is here as the control.
- **`search_unmapped`** — `goal_known=False`. The description's own running example. The
  controller explores until the goal enters line of sight within `DETECT_RADIUS`; `obs.goal`
  then latches. Tests frontier coverage and geometric reorientation.
- **`return_degraded`** — goal disclosed, pose estimate drifting. Tests `IsLocalisationDegraded`
  and `MaintainHomingVector`. Depends on chunk [13](13-unit-harness.md)'s drift model.
- **`search_degraded`** — both. **This is the family that carries G3's actual claim**, because it
  is the only one that cannot be solved by any single primitive: the robot must explore for an
  undisclosed goal *while* its pose estimate drifts, so an exploration node and a path-integration
  node have to be recombined. A library that solves the other three and fails this one has
  demonstrated transfer but not composition, and those are different results.

**`resume_after_bump` was considered and dropped.** A bump is an external impulse on the
articulation: it would be the only Isaac-Sim-dependent work in an otherwise sim-free epic, it
needs the impulse tuned so it disturbs without toppling — and a topple terminates the episode as
`fell`, so the tuning loop interacts with the scored metrics. `search_degraded` tests
recombination more directly at none of that cost. `DisturbanceSpec` is therefore **not** declared
in 11a; if a later cycle wants the family, that is a contract change and it gets its own chunk.

## What "sealed" means operationally

The honest version is narrower than it sounds:

- The file is a separate module that the generation and composition paths **never import**, with
  a test that greps for the import the way `test_import_guard.py` already greps for `isaacsim`.
- Its SHA-256 goes in the manifest of every held-out run, so a result can prove which split it
  scored against.
- **It does not protect against the LLM having read the file in an earlier session.** Generation
  is LLM-driven and its context is not a sandbox. The mitigation is procedural: hold generation
  sessions that have never had the held-out module in context, and never paste it into a prompt.
  Write that limitation into the results, because a reviewer will ask.

One consequence of the procedural half: **the session that authors this file is contaminated for
generation.** Chunks 15 and 16 must run in a session that has never had it in context. That is a
scheduling constraint on the humans, not something a test can enforce, and it is the reason this
chunk sits early — sealing late means the same session did both.

## Build

- `core/scenarios_held_out.py` — `HELD_OUT_SCENARIOS`, same `Scenario` dataclass,
  `split="held_out"`. Separate module, not a list in `scenario.py`, so "did anything import it"
  is a one-line check. Uses 11a's `goal_known`, `detect_radius` and `drift` fields.
- `core/scenario.py` — `all_scenarios(split=...)` already filters; extend `_BY_ID` and `load()`
  to reach held-out scenarios only when explicitly asked, never from `all_scenarios(None)`.
- `core/manifest.py` — `held_out_sha256`, alongside `usd_sha256`.
- `scripts/seal_split.py` — print the hash to be committed.
- Seed assignment: held-out seeds must not collide with the dev seeds (1001–1003) and must be
  derivable from a stated base, so "was a seed added after seeing results" is answerable by
  inspection. Chunk 17 owns the base; this chunk owns not colliding with it.

## Done when

- `core/tests/test_held_out.py`:
  - twelve scenarios, three per family, asserted by count — an off-by-one here silently changes
    the G3 denominator;
  - every held-out scenario passes the same `validate()` as the dev ones — in bounds, start and
    goal free, goal reachable, positive time limit, and the goal-disc clearance guard from
    [09](../done/09-spl-denominator.md);
  - every `goal_known=False` scenario has a start position from which the goal is **not** already
    detectable at t = 0. A search task the robot solves on its first observation is not a search
    task, and `validate()` cannot catch it because it is a property of the pair;
  - `all_scenarios()` and `all_scenarios("dev")` return **no** held-out scenario;
  - no module under `controllers/`, `nodes/`, `generate/` or `compose/` imports
    `core.scenarios_held_out`, checked by AST like `test_import_guard.py` does;
  - the recorded SHA-256 matches the file, so a changed split is a failing test and not a
    silent re-definition.
- Each `drift`-carrying scenario fails loudly on load until chunk 13 lands, per 11a's decision 4.
  Assert that; a family that silently runs without drift is the exact bug epic 02 opens by naming.
- The chosen option, the twelve scenarios, the 2 × 2 rationale, the dropped bump family and the
  sealing limitation are written into `plans/done/11-*.md`'s Outcome **and** `results/README.md`.
- `git tag split-v1` on the sealing commit, named in the Outcome.

## Not in scope

Authoring the generation prompts, and running anything against the held-out set. The split is
sealed and then not touched until Week 5. Note that three of the four families **cannot be run at
all** until chunk 13 exists — that is expected, and it is not a reason to weaken the seal.

---

## Outcome

**Done 2026-08-25.** Option B, twelve scenarios, four families, as decided. `core/tests/`
165 → **200 passed**, no simulator. `results/` numerically untouched.

Sealed at **`00b3312b0755cc94d252ccb2eb6a096cf0867aae19ef61c6c71cb31572870a63`**, recorded in
`core/tests/test_held_out.py` and stamped into every manifest as `held_out_sha256` — on dev runs
too, deliberately, because a dev result carrying the hash is the evidence that the split already
existed with this content while development was happening.

### Two bugs the plan's own checks caught, both of which would have shipped silently

**The 2 x 2 was a 2 x 1.** All six search scenarios were authored without `goal_known=False`, and
the field defaults to `True` — so `search_unmapped` and `search_degraded` disclosed their goals
from t=0 and were, functionally, six more `navigate_to_goal` runs. Every count-based check passed:
twelve scenarios, three per family, four family labels, all validating. The family label is a
string somebody typed, and it agreed with nothing.

`test_the_two_axes_are_actually_crossed` now asserts the `(family, goal_known, drift)` triple
rather than the count, and it is the single most valuable test in this chunk. **A sealed split with
this bug in it would have made G3 measure route transfer while reporting it as compositional
generalisation**, and the seal would have prevented anyone from fixing it.

**Sealing the split made it unrunnable.** `scripts/run_episode.py` built `--scenario` from
`choices=[s.id for s in all_scenarios()]`. The moment `all_scenarios()` correctly stopped returning
held-out scenarios — which is exactly what this plan asked for — Week 5 could no longer run any of
them. The plan asked for `all_scenarios()` to exclude them and did not notice that the only entry
point consumed that list.

Fixed by dropping `choices=` for a `type=` validator delegating to `load()`, which matches the
repo's own convention that argparse validates before the simulator boots. It also refuses a
drift-carrying scenario at parse time, with `load()`'s explanation, rather than 20 s into a boot.

### The `--help` leak, which is subtler than it looks

The obvious fix — adding held-out ids to `choices=` — would have printed them on `--help`. An id
like `held_search_deep_right` describes its task, and the failure mode sealing cannot survive is
the split reaching a model's context. So `--help` lists dev ids and says held-out ids resolve but
are not listed; `load()`'s KeyError does the same; and `scripts/seal_split.py` prints the split's
*shape* — counts, the 2 x 2, seed range, total time limit — but never an id, a start or a goal, so
its output is safe to paste into a commit message. There is a test asserting `--help` leaks nothing,
because a future `choices=` would silently undo all of it.

### Lazy import, and why the guard would otherwise be theatre

`core/scenario.py` imports the split from inside `_held_out()`, never at module level. A
module-level import would mean everything importing `core.scenario` transitively imports the split,
and the AST guard over `controllers/`, `nodes/`, `generate/`, `compose/` would be checking a
condition nothing could violate and nothing could satisfy. The test asserts both halves: the
import exists (or `load()` can no longer reach held-out ids) and is not at module level.

### What could not be done

**`git tag split-v1` was not applied.** `plans/`, `core/` and `results/` are all still untracked,
so there is no sealing commit to tag — and the working agreement is not to commit unless asked.
Recorded in `results/README.md` as an open action: tag the commit that first tracks
`core/scenarios_held_out.py`, and re-check the hash then.

**This session is contaminated for generation.** Authoring the split put it in context, which is
the procedural half of the seal that no test can enforce. Chunks 15 and 16 must run in a session
that has never had `core/scenarios_held_out.py` in it. Raised before starting and confirmed.

### Cost, for chunk 17

Time limits total **1920 s per condition per seed** if every episode runs to timeout — roughly
45 min wall clock at the measured 1.4x headless ratio, before multiplying by four conditions and
the seed set. Limits are deliberately generous relative to the taut paths (4.60-11.51 m) because
exploring is not walking, and a limit tuned to the optimal route fails a competent searcher for
lack of time, biasing against the mechanisms the corpus is meant to supply. If chunk 17's
arithmetic says this is unaffordable, the seed set is the lever — **not** these limits.

### For chunk 13

Six scenarios refuse to load until `core/drift.py` exists. The drift magnitudes are sealed:
`DRIFT_MODERATE` (sigma_xy 0.12 m/root-s) crosses `LOC_FAIL_THRESHOLD` at ~69 s, `DRIFT_SLOW`
(0.09) at ~123 s, each sized to land near the middle of the episodes it is used in. The units make
that arithmetic implementation-independent. If the threshold turns out not to be crossed, **that is
a finding about the drift model and it is fixed there** — a split that gets tuned after sealing is
not a split.

`test_drift_scenarios_refuse_to_load_until_the_model_exists` skips itself once `core.drift`
imports, and should then be replaced by an assertion that the drift is non-zero.
