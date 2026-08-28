# Epic 02 — the traceable node library

**Covers:** execution-plan Weeks 2–4. **Chunks:** 11a, 11–17.
**Depends on:** epic [00](00-epic-reproducible-episode.md), complete.

## Why this epic exists

Everything built so far sits *below* the experiment. `Observation → Command` is the contract,
`run_episode.py` scores against it, and two hand-written controllers prove the scoring fires in
both directions. None of that is the thing under test.

The thing under test is §4 of the description: **the generator emits individual behaviour-tree
leaf nodes, not a monolithic stack.** `controllers/reactive_avoid.py` is exactly the monolith §4
forbids — one class, one `step()`, gap-following welded to speed control welded to goal seeking.
It is the shape condition A produces, which is why it is a useful reference and why it can never
be a condition.

This epic builds: the node contract, the library that holds nodes, the unit harness that decides
admissibility, the first hand-curated cards, the generator, and the composition step.

## What is already load-bearing

- `core/observation.py` — `Controller` is deliberately two methods so "a generator emitting a
  behaviour tree, a hand written baseline and a Nav2 bridge all fit". A tree is a `Controller`.
- `clamp()` already anticipates generated code: "a generated node that returns 40 m/s should be
  scored on what the robot did, not rejected".
- `core/metrics.py` — a unit scenario needs a scored micro-world, and scoring is already a pure
  function of a trace.
- `core/geometry.py` — micro-worlds can be built and scored with no simulator.
- `manifest["library_snapshot"]` — the slot for non-negotiable #8, `None` since the first run.

## Three gaps that block whole families of nodes — all three now resolved

Found by reading the scored results against §4's own list of example leaves. Each one makes
certain cards **untestable**, so each was a scheduling constraint, not a detail. All three have
now been decided; the decisions are recorded here and collected by chunk
[17](done/17-pre-registration.md).

**1. Localisation is not simulated.** `run_episode.observe()` sets `pose_estimate` to the true
pose — its own docstring says so. `localisation_failures` is therefore structurally 0 in every
result committed so far. `IsLocalisationDegraded` and `MaintainHomingVector` cannot pass or fail a
unit scenario until a drift model exists.
→ **Resolved by chunk [13](done/13-unit-harness.md): a seeded random walk with a magnitude
parameter.** `DriftSpec` is declared early by [11a](done/11a-task-contract.md) so the sealed split
can reference it, and a scenario carrying `drift` fails loudly on load until 13 implements it.
`RequireSensorAgreement` remains **structurally untestable** — one sensing modality, nothing to
cross-check — and that is a property of the sensor suite, not a gap to close this cycle.

**2. There is no search task.** All three dev scenarios hand the controller the goal in
`Observation.goal`, which is a required field of the frozen contract.
→ **Resolved by chunk [11a](done/11a-task-contract.md): `Observation.goal` becomes
`Optional[Tuple[float, float]]`, `None` means not disclosed, and it latches on line-of-sight
detection within `DETECT_RADIUS = 2.0` m.** `Scenario.goal` stays required — the runner still
scores against it. No `cue` field was added, deliberately, and the price is recorded below.

**3. Legibility has no metric.**
→ **Decided: out of scope this cycle.** `DeviateToDisambiguate` measures an observer's inference
about intent; none of `score()`'s 18 keys is a property of anyone's belief, and a legible path is
by construction longer than the optimal one, so the node would score *negatively* on SPL and
chunk 13's gate would reject it for the wrong reason. It is marked for a later cycle rather than
abandoned — on chunk 14's "would the model invent it anyway?" screen it is one of the strongest
cards available, so it is out on cost, not on merit. Reviving it needs a candidate-goal set per
scenario, an observer model over `core.geometry`'s geodesic costs, a scalar metric and a new
frozen constant, and it must be pre-registered before it is reported.

## The price of resolving gap 2 without a cue field

**Area-restricted search — the description's own running example — is not testable this cycle.**
ARS concentrates search around an encounter *rate*; 11a's contract yields a single goal detected
once, so there is no rate and no gradient. This is a real loss and it goes in the write-up rather
than being quietly dropped: *the running example turned out not to be measurable under our own
contract* is a finding about the pipeline.

What the search family does still test is coverage, path integration and geometric reorientation
— three separable mechanisms, which is what the coverage-count argument in G1 actually needs.

## Chunks

| # | Chunk | Depends on | Needs Isaac Sim |
|---|---|---|---|
| 11a | [The task contract](done/11a-task-contract.md) | — | no |
| 11 | [Seal the held-out split](done/11-held-out-split.md) | 11a | no |
| 12 | [The node contract and the library](done/12-node-contract.md) | 11a | no |
| 13 | [Unit-scenario harness and promotion gate](done/13-unit-harness.md) | 11a, 12 | no |
| 14 | [The first hand-curated cards](done/14-first-cards.md) | 12 | no |
| 15 | [The generator](done/15-generator.md) | 12, 13, 14 | no |
| 16 | [Composition](done/16-composition.md) | 12, 15 | no |
| 17 | [Pre-registration](done/17-pre-registration.md) | 10, 11a, 11 | no |
| 18 | [Node tracing](done/18-node-tracing.md) | 12 | no |

**11a comes first**, ahead of the gate. It is the one-time change to the frozen `Observation` and
`Scenario` contracts, and chunk 11 seals a split that references fields 11a declares — a sealed
split that will not import is not sealed, it is broken. Chunks 11 and 12 each deferred this
question to the other; 11a exists so that neither has to.

**11 is a gate**, not a preference: G3 measures held-out scenarios "never seen", and a split
sealed after generation has begun measures nothing. Nothing in 14–16 runs until it is done.

**17 is done.** `PRE-REGISTRATION.md` is written and its enforcement test is green; the one thing
left is `git tag prereg-v1`, which needs the files tracked and therefore needs the user to ask for
a commit. **Week 5 does not start until that tag exists** — a pre-registration written, or tagged,
after the first run is not one.

Every chunk in this epic is sim-free. 15 and 16 call an LLM but still produce and check code
without a simulator; only the eventual scored runs need one, and those belong to Week 5. Note that
half the sealed split — `return_degraded` and `search_degraded` — cannot be *run* until 13 lands.

## A scheduling constraint no test can enforce

Chunk 11's seal is only half technical. The other half is procedural: **chunks 15 and 16 must run
in a session that has never had `core/scenarios_held_out.py` in context.** Sealing and generating
in one session voids G3, and nothing in the repo can detect that it happened. It is written into
11, 15 and 16, and it is the reason 11 sits early rather than late.
