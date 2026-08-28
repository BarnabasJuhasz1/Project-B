# 14 — The first hand-curated cards

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** 12 · **Needs Isaac Sim:** no
**Gated by:** [11](../done/11-held-out-split.md). No card is written until the split is sealed.

## Why

The execution plan's Week 1 asks for "5–10 paper cards, ready to be ingested". §3 is emphatic
about the order: **"Pilot first. Before any pipeline is built, one day with a handful of
hand-extracted papers, comparing B against D."** No ingestion daemon until the pilot says the
direction is worth pursuing.

Hand-extracted means hand-extracted. The point of the pilot is to find out whether cards carry
information the model does not already have, and a card written by the model from memory answers
that question with itself.

## Decided

**Which failure the first cards target.** Cards are cheap to write and expensive to evaluate, so
the first handful should aim at *measured* failures of the current baselines rather than at
whatever is most interesting. What the committed results actually show:

| Measured failure | Where | Metric it moves |
|---|---|---|
| Clips the wall rounding the blind corner: `min_clearance` −0.069, 1 collision | `doorway_traverse` / `reactive_avoid` | collisions, near-misses |
| Wedged 110 s of 120 s, never recovers | `*_goal_seeker` | goal-reach, stuck events, recovery time |
| Potential-field predecessor parked permanently at (5.36, 3.47) | noted in `reactive_avoid`'s docstring | goal-reach |
| SPL 0.949 — greedy alignment overshoots the corner | `doorway_traverse` / `reactive_avoid` | SPL |

**What is not yet measurable.** Do not spend the first cards on mechanisms the harness cannot
score. G1's unit pass rate would count them as failures for the wrong reason. As of the decisions
in [11](../done/11-held-out-split.md), [11a](../done/11a-task-contract.md) and [13](../done/13-unit-harness.md), the
status of each family is now settled rather than pending:

| Family | Status | Why |
|---|---|---|
| Speed/visibility, geometric reorientation | **testable today** | tier 1 below, unchanged |
| Localisation, path integration | **testable after chunk 13** | drift model decided: seeded random walk |
| Search / coverage | **testable after 11a** | `goal_known=False`, detection latches on line of sight |
| Area-restricted search | **NOT testable this cycle** | see below |
| Legibility | **out of scope this cycle** | see below |

**Area-restricted search is out, and this is a substantive loss.** It is the description's own
running example. ARS concentrates search around an encounter *rate*, and 11a's contract gives a
single goal detected once — one encounter, so there is no rate to concentrate around and no
gradient to climb. Making it testable needs a cue field or multiple targets, and 11a declined
both to keep the frozen contract to a single change. **Write this in the results.** "The running
example turned out not to be measurable under our own contract" is a real finding about the
pipeline, and burying it would be the dishonest option. It also sharpens what the search family
*does* test: coverage, path integration and geometric reorientation, which are three separable
mechanisms and still make the coverage-count argument.

**Legibility is out of scope this cycle, decided.** `DeviateToDisambiguate` (Dragan, Lee &
Srinivasa 2013) measures an *observer's* inference about intent, and nothing in `score()`'s 18
keys is a property of anyone's belief. Worse, a legible path is by construction longer than the
optimal one, so under today's metric set the node scores **negatively** on SPL and chunk 13's
gate would reject it — not because the mechanism is wrong but because nothing measures what it
does. Do not write the card. **Mark it for a later cycle**: it is one of the strongest cards on
the whole shortlist against chunk 14's third screen (*would the model invent it anyway?* — almost
certainly not), so the reason it is out is cost, not merit. Reviving it needs a candidate-goal
set per scenario, an observer model over `core.geometry`'s geodesic costs, a scalar metric, and a
new frozen constant for observer rationality — roughly one sim-free chunk, and it must be
pre-registered before any run that reports it.

**Extraction confidence has to mean something. Decided: a 3-point scale with written criteria.**
The field exists so that G1 can report "the fraction carrying a source span with extraction
confidence"; a 0–1 float nobody calibrates makes that fraction meaningless. Write the three
criteria into `cards/schema.py` next to the field, not into a plan file.

## Build

- `cards/schema.py` — the six fields as a dataclass: Source, Mechanism, Trigger, Derivation,
  Prediction, Confidence. Validation: a Source must carry a resolvable identifier **and a page or
  section span**, and a Prediction must be falsifiable against something `core/metrics.py`
  already computes, or the card is rejected at load.
- `cards/store.py` — load, list, retrieve. Retrieval keeps the working set small (#7).
- `cards/*.yaml` — 5–10 hand-written cards. One file each, diffable, versioned.
- `cards/tests/` — every card in the store validates; every Prediction names a metric that exists.

## Done when

- 5–10 cards validate, each with a real source span checked by a human against the actual paper.
- At least three target the measured failures above and are testable **today**, without chunk 13's
  drift model and without a search scenario.
- Each Prediction maps to a metric key in `core.metrics.score()`'s output, checked by a test.
- The Outcome records, per card, which of the three claimed advantages it is meant to supply —
  novelty, coverage, or provenance — because those are what G1 reports and they are not the same
  claim.

## Not in scope

The ingestion daemon, corpus polling, deduplication and indexing. §7 has them in scope for the
project; §3 has them explicitly after the pilot.

## Candidate shortlist

Screened on three questions, in this order. **Does it target a failure this repo has actually
measured? Can the harness score it today? Would the model invent it anyway?** The third is the
one that decides whether a card earns its place, because B−D is the comparison that matters most
and a card restating the model's own default answer moves that difference by zero.

⚠️ **These serve the coverage and provenance axes only, never novelty.** The description's first
claimed advantage is "findings published after the training cutoff". Every reference below is by
construction inside the training distribution of the model suggesting it. The novelty axis cannot
be sourced this way at all — it requires polling recent venues, which is what §5's ingestion
daemon is for. Say so in the write-up rather than letting the pilot imply otherwise.

⚠️ **Citations are starting points and must be verified against the actual paper before a card is
written.** The card schema requires a source span precisely because a stated threshold is worth
nothing without one, and that applies to this table more than anywhere.

### Tier 1 — testable today, targets a measured failure

**Speed bounded by stopping distance within the *visible* range → `LimitSpeedToVisibleRange`.**
Targets the blind-corner collision (`min_clearance` −0.069, 1 collision). The scene was built for
this: the y=5 wall exists to create the corner at (6, 5). The baseline's own comment admits it is
the naive version — "it uses the nearest return in the cone, not the free distance along the
intended path, and it has no notion of an occlusion boundary at all". The model's default is
*brake when a sensor sees something*; the literature's rule is *never exceed the speed at which
you can stop within the distance you can currently see*, which binds **before** anything is
visible. Different rule, quantitative, falsifiable.
- Fox, Burgard & Thrun (1997), "The Dynamic Window Approach to Collision Avoidance", IEEE
  Robotics & Automation Magazine 4(1) — admissible velocity `v ≤ sqrt(2·d·a_brake)`.
- Fraichard & Asama (2004), "Inevitable Collision States: a step towards safer robots",
  Advanced Robotics 18(10) — a state from which collision is unavoidable under *any* future
  control. Sharper and more falsifiable than a speed cap.
- Gibson & Crooks (1938), "A theoretical field-analysis of automobile-driving", American Journal
  of Psychology 51 — the "field of safe travel" and minimum stopping zone. The human-factors
  origin of the rule; worth a card of its own for the occlusion framing.
- The occlusion-aware planning literature is the obvious fourth source and I could not name a
  canonical citation with confidence. Search it properly before writing the card.

**Reorientation from enclosure geometry → `ReorientToGeometry`.** Targets the worst measured
failure in the repo: `goal_seeker` wedged for 110 s of a 120 s episode, and the potential-field
predecessor parked permanently at (5.36, 3.47). Scores against `stuck_events`, `recovery_time`,
`time_stuck` and goal-reach, all already computed. Strong on the B−D screen: asked to invent a
recovery principle a model produces back-up-and-retry or wall-following; using the *metric shape*
of the surrounding free space as the primary reorientation cue, ahead of any feature, is a
specific empirical finding rather than an obvious engineering move.
- Cheng (1986), "A purely geometric module in the rat's spatial representation", Cognition 23 —
  the founding result.
- Gallistel (1990), *The Organization of Learning* — the geometric module treatment.
- Cheng & Newcombe (2005), "Is there a geometric module for spatial orientation? Squaring theory
  and evidence", Psychonomic Bulletin & Review 12 — a review carrying the counter-evidence, which
  is what a calibrated Confidence field needs.

**Deliberately excluded from tier 1: local-minimum escape.** It is a real measured failure, but
asked cold the model produces Bug/wall-following (Lumelsky & Stepanov 1987) and is *right*. A
card there buys nothing over condition D. Worth stating in the write-up as a negative result
about where corpora help, rather than quietly omitting it.

### Tier 2 — blocked on chunk 13's drift model

**Path integration → `MaintainHomingVector`.** Now more central than when this was written:
chunk 11's `search_degraded` family cannot be solved without it, so this is no longer a
nice-to-have card but the one the sealed split most directly demands. `return_home` already
exists and is currently trivial, because `pose_estimate` is the true pose. Under drift it becomes the actual test, and
the mechanism is a continuously-updated home vector that needs no landmarks. Scores against
`localisation_failures` and goal-reach.
- Müller & Wehner (1988), "Path integration in desert ants, *Cataglyphis fortis*", PNAS 85 —
  the classic, and it carries a systematic-error model, which is the falsifiable part.
- Etienne & Jeffery (2004), "Path integration in mammals", Hippocampus 14.
- Collett & Collett (2000), "Path integration in insects", Current Opinion in Neurobiology 10.

**`RequireSensorAgreement` is structurally untestable**, not merely blocked. There is one sensing
modality — a 2D range scan — plus a pose estimate. Nothing to cross-check against. It needs a
second modality before it means anything; note that rather than writing a card that cannot fail.

### Tier 3 — search family now exists; ARS specifically does not fit it

Chunk 11 sealed `search_unmapped` and `search_degraded`, so search cards are writable. But see
the decision above: **`RestrictSearchArea` itself is not testable**, because 11a's single latched
detection gives one encounter and ARS needs an encounter rate. The citations below stay on the
record for the cycle that adds a cue field or multiple targets; do not write the card now and do
not count it as a failed node.

What *is* writable against the search family: frontier/coverage mechanisms, and path integration
used to avoid re-searching ground already covered — which is also what makes `search_degraded`
solvable at all.

**Area-restricted search → `RestrictSearchArea`.** Deferred, not rejected.
- Kareiva & Odell (1987), "Swarms of predators exhibit 'preytaxis' if individual predators use
  area-restricted search", The American Naturalist 130.
- Benhamou (1992), "Efficiency of area-concentrated searching behaviour in a continuous patchy
  environment", Journal of Theoretical Biology 159.
- Bénichou, Loverdo, Moreau & Voituriez (2011), "Intermittent search strategies", Reviews of
  Modern Physics 83 — two-phase search with optimal duration ratios. Quantitative, and a good
  provenance test: the model will assert an exponent with equal confidence either way.
- Viswanathan et al. (1999), "Optimizing the success of random searches", Nature 401 — the Lévy
  exponent result, and contested enough since that a Confidence field has to earn its value.

### Tier 4 — out of scope this cycle, revisit later

**Legible motion → `DeviateToDisambiguate`.** Dragan, Lee & Srinivasa (2013), "Legibility and
predictability of robot motion", HRI 2013 — precisely the "deviate early rather than evenly"
result the description cites. **Decided out of scope**, for the reasons in *Decided* above. Kept
here rather than deleted because it is the highest-scoring card on the B−D screen and the first
thing to revisit if a later cycle has room for a legibility metric.

### Proposed pilot set

Five cards, unchanged in shape by the decisions above: three tier 1
(`LimitSpeedToVisibleRange` from the DWA and the ICS framings — deliberately two cards on one
mechanism to test whether the *derivation* matters, plus `ReorientToGeometry`), and two tier 2
written now but tested after chunk 13. That gives the B-vs-D pilot three cards it can actually
score this week, and it deliberately includes one mechanism the model would invent unaided
(`LimitSpeedToVisibleRange` in its weak form) as an internal control.

The two tier-2 cards should both be path-integration framings rather than one path integration
and one degradation detector, because `search_degraded` needs the homing vector specifically.

---

## Outcome

**Done 2026-08-25, with one item deliberately left open — see *What I could not do*.**
`core/tests/` + `nodes/tests/` + `harness/tests/` + `cards/tests/` = **285 passed**, no simulator.
`results/` untouched.

Built `cards/schema.py`, `cards/store.py`, five cards, `cards/tests/test_cards.py` and
`cards/VERIFICATION.md`.

### What I could not do, and why it is not a shortcut

**No source span is filled in. All five cards are UNVERIFIED and G1's provenance fraction is
0.0.** There is no corpus on this machine — only `Project B.pdf` — so every span would have been
written from memory.

That is exactly the failure the provenance axis exists to detect. The description's own framing is
that a model "states a threshold or an exponent with equal confidence whether or not it is
confabulating", and a generated span would have made G1's provenance number a measure of fluency
rather than traceability, invisibly. The plan says as much itself: *"Citations are starting points
and must be verified against the actual paper before a card is written."*

So the schema was changed rather than the practice. `Verification` is a first-class state,
`is_traceable` is False without it, and `Source.__post_init__` refuses to accept VERIFIED without
both a span and a `verified_by` — a verification that does not say where, and who looked, is not
one. `cards/VERIFICATION.md` lists what to check per card, including which claims are the
load-bearing ones. **The mechanisms are unaffected**: Mechanism, Trigger, Derivation and Prediction
are falsifiable by the harness whatever the citation says, so the pilot runs on unverified cards.
Only provenance is blocked.

`test_no_pilot_card_is_traceable_yet` pins the current honest state and **will fail on the first
verification**. That is intended; retiring it is part of finishing the job.

### The finding this chunk produced

**`localisation_failures` is not a performance metric, and no card may predict it.**

Writing the two path-integration cards forced the question of what they could honestly claim.
Drift is injected open-loop and `run_episode.py` records `drift.estimate()` straight into the
trace, so **no controller can influence the recorded pose error**. Measured: on a drifted
`doorway_traverse` the localisation error at step 500 is *bit-identical* across `goal_seeker` and
`scaffold_sweep`, and both score exactly 2 failures because both run 6000 steps. `reactive_avoid`
scores 0 only because it finished in 798.

A controller can lower it **only by finishing sooner**. `cards/store.py` now rejects a prediction
naming it — alongside `optimal_distance`, `scenario` and `seed` — as *not falsifiable about the
node*, and `test_localisation_failures_really_is_controller_independent` asserts the property so a
change to the drift model cannot quietly invalidate the rule. Recorded in `results/README.md`
because it is a G2 secondary-metric decision: it is a **manipulation check** (did drift bite?), not
a performance number, and reporting it as one would let "condition B localises better" be read out
of a figure that cannot show it.

Both path-integration cards therefore predict `goal_reached` under drift instead.

### Design decisions the plan did not specify

**Prediction is structured, not prose.** §4 says the Prediction becomes the unit test, which is
only mechanical if it names something `score()` computes. A `Prediction` carries `metric`,
`baseline` and `min_effect`, and `Card.unit_scenario(micro)` returns a chunk-13 `UnitScenario`
directly. `test_the_card_to_admission_path_runs_end_to_end` drives card → unit scenario → admission
decision with no human in the loop.

**`cost_metric` is required.** A card that cannot say what its mechanism makes *worse* is a card
that will be reported selectively. `LimitSpeedToVisibleRange` buys clearance with time, and both
speed cards say so.

**A card may not claim novelty**, enforced in `Card.__post_init__`. Every source a model can
suggest is inside its own training distribution, so a hand-curated pilot cannot supply that axis at
all — it needs §5's ingestion daemon. Saying so is a finding; claiming it would be a lie.

### The pilot set, and what each is for

Five cards, three mechanisms, deliberately with two mechanisms carried by **two derivations each**:

| Card | Node | Predicts | Confidence | Role |
|---|---|---|---|---|
| `limit_speed_dwa` | LimitSpeedToVisibleRange | `collisions` ↓ vs reactive_avoid | MEDIUM | **the internal control** |
| `limit_speed_ics` | LimitSpeedToVisibleRange | `min_clearance` ↑ vs reactive_avoid | LOW | same mechanism, occlusion derivation |
| `reorient_geometry` | ReorientToGeometry | `goal_reached` ↑ vs goal_seeker | LOW | strongest on the B−D screen |
| `path_integration_ants` | MaintainHomingVector | `goal_reached` ↑ under drift | MEDIUM | replace the pose estimate |
| `path_integration_mammals` | MaintainHomingVector | `goal_reached` ↑ under drift | LOW | arbitrate, don't replace |

`limit_speed_dwa` is the yardstick: asked cold, a model produces roughly "brake in proportion to how
close the nearest thing is", so this card should move B against D by very little **and that is
what it is for**. The pairing is a claim about the schema itself — if two cards with the same
`node_name` and different `Derivation` produce nodes that score differently, the Derivation field
carries information rather than decorating the citation. Testable inside the pilot.

No card is HIGH confidence. That is the honest reading of the written criteria: HIGH requires the
span to point at the equation or threshold itself, and no span has been checked.

### Tier 2 arrived early

The plan expected the two path-integration cards to be "written now but tested after chunk 13".
Chunk 13 landed first, so all five are testable today.

### For chunk 15

`CardStore.by_metric()` is the failure-driven retrieval axis from §5: a failure signature names a
metric, and that turns it into a query. `Card.unit_scenario()` is the capability-driven path.
A rejected card is rejected **at load**, so the generator never sees one that cannot become a test.

### For chunk 17

`localisation_failures` must be declared a manipulation check or dropped — not left in the
secondary list unqualified. And the provenance fraction reported for G1 must come from
`CardStore.provenance()`, which will read 0.0 until `cards/VERIFICATION.md` is worked through.
