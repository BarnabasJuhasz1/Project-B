# 15 — The generator

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** 12, 13, 14 · **Needs Isaac Sim:** no

## Why

Week 3: "LLM gets one card + the node contract spec, candidate nodes come out of the generator,
and then the automated harness decides which are allowed into the library."

This is where conditions A, B and D differ, and **only** here. Same model, same task, same
contract, same harness, same locomotion policy — the only thing that varies is what the
generation prompt contains. If anything else differs between conditions, the ablation measures
that instead.

## Decided

**One card per call, or many.** §4 says "one card + the node contract spec". One card per call
makes the source span attributable to exactly one node, which is what G1's provenance fraction
needs. It also makes coverage a clean count. **Decided: one.**

**What condition D is told.** "Same model, told to devise its own principles, no corpus." The
prompt must ask for the same *artefact* — a principle with a mechanism, a trigger, a derivation
and a falsifiable prediction — so that D produces cards-without-sources rather than
nodes-without-cards. Otherwise B vs D compares two different pipelines and the reading is lost.
This is the single most important prompt decision in the project; write both prompts side by side
and diff them.

**What condition A is told. Decided: A gets the node contract.** A is therefore "structured
generation without literature", the decomposition into typed leaves is held constant across A, B
and D, and **B−A isolates the corpus and nothing else**. The alternative — A as a
`reactive_avoid`-shaped monolith — is closer to the description's literal "writes the stack cold"
and closer to what asking an LLM to program a Go2 actually produces, but it changes two things at
once and B−A stops being attributable to either.

Two things follow and both must be written into the results rather than left implicit:

- The description's "writes the stack cold" is **ambiguous**, and this is an interpretation of
  it. Say so, and say which reading was taken and why.
- The project consequently **cannot report what the decomposition alone buys**. That would need a
  fifth condition (a monolithic A1 against the structured A2), and #10 requires identical episode
  counts across conditions, so it is not free. Record it as a known limitation, not as an
  oversight.

**Determinism of generation.** Temperature, model id and prompt hash must be recorded per node,
or a library cannot be regenerated. Record them in node metadata, not only in a log.

## Build

- `generate/prompts/` — one file per condition (A, B, D), plain text, versioned. Diffable, since
  the diff between them *is* the independent variable.
- `generate/generator.py` — card (+ contract spec) → candidate node source + its unit scenario.
  Writes to a staging area, never straight into the library.
- `generate/admit.py` — run chunk 13's harness on the candidate; on pass, append to the library
  with metadata: card id, source span, extraction confidence, model id, prompt hash, temperature.
- Rejected candidates are **kept**, with the reason. G1 reports a unit pass rate, which needs a
  denominator; and a rejected node is the most informative artefact the pipeline produces.

## Done when

- One hand-curated card from chunk 14 produces a candidate node that compiles, type-checks
  against the port vocabulary, and is judged by the harness with no human in the loop.
- A deliberately bad card — vague Mechanism, unfalsifiable Prediction — produces a candidate that
  is **rejected**, and the rejection reason is recorded.
- Every admitted node carries a source span traceable to a real card, checked by a test.
- The same card, same prompt, same model id and temperature 0 produces a byte-identical candidate,
  or the metadata explains why not.
- `generate/` never imports `core.scenarios_held_out`, checked by the guard from chunk 11.
- The three prompt files differ **only** in their literature block — asserted by a test that
  diffs them, not by reading. If A, B and D drift apart anywhere else, the ablation measures that
  difference instead of the corpus, and a diff test is the only thing that catches it early.
- Every admitted node's metadata carries model id, temperature and prompt hash, and a test fails
  if any is absent. A library that cannot say which model produced it cannot be regenerated.

## Before running

**This chunk must run in a session that has never had `core/scenarios_held_out.py` in context.**
Chunk 11 records why: the seal is procedural on this side and no test can enforce it. Sealing and
generating in one session voids G3.

## Not in scope

Failure-driven generation (that needs evaluation runs to produce failure signatures), corpus
retrieval, and composition.

## Outcome

**Done 2026-08-26.** `core/` + `nodes/` + `harness/` + `cards/` + `generate/` =
**356 passed**, no simulator, ~93 s. `results/` untouched. 64 of those tests are new and
none of them calls a model.

Built `generate/prompts/condition_{a,b,d}.md`, `generate/backends.py`, `generate/generator.py`,
`generate/admit.py`, `generate/tests/` (4 files), `generate/FINDINGS.md`,
`scripts/generate_node.py`, and one deliberately-bad card fixture. Four real `claude -p`
calls against `claude-opus-5` were made; their transcripts are committed.

**Read `generate/FINDINGS.md` before trusting any of this.** The four findings there are the
chunk's substantive output, and two of them bind chunk 17.

### The Done-when list, item by item

| Check | State |
|---|---|
| A hand-curated card produces a candidate judged with no human in the loop | ✅ `limit_speed_dwa` → ADMITTED, `collisions` 0.0 vs 2.0 |
| A deliberately bad card produces a **rejection** with a recorded reason | ⚠️ **partially — see below** |
| Every admitted node carries a source span traceable to a real card | ✅ but the span is EMPTY and the node is therefore untraceable, by design |
| Same card, same prompt, temperature 0 → byte-identical candidate | ❌ **impossible; the metadata explains why** |
| `generate/` never imports `core.scenarios_held_out` | ✅ chunk 11's guard, plus a second copy inside `generate/tests/` |
| The three prompts differ only in their literature block | ✅ asserted by stripping the delimited block and diffing |
| Every admitted node carries model id, temperature and prompt hash | ✅ refused at admission if not |

### The bad card was ADMITTED, and that is the chunk's main finding

The plan expected a vague card to be rejected. It was not, and the reason matters:
**the harness tests whether the NODE works, not whether the node implements the CARD.** Handed a
contentless mechanism, the model invented a real one — clearance-weighted heading selection plus a
commitment-to-boundary-following mode switch, none of it on the card — and the invented mechanism
passed on its own merits.

Run **once** the model refused outright, and said in advance exactly what would go wrong:

> If I write a node body, every line of it is my invention wearing the card's name. […] The
> scenario would be well-formed and the number would clear the floor, and the number would mean
> nothing.

Run **twice**, with byte-identical inputs, it did the thing it had warned about. Both transcripts
are committed. This is a validity threat to condition B — B's advantage could be manufactured by
attribution rather than by the corpus — and **chunk 17 must declare card fidelity**, either as a
stated limitation absorbed by B−D or as a pre-registered check. `FINDINGS.md` §1.

The rejection path is nonetheless demonstrated and tested: `NO_CANDIDATE_EMITTED` (the refusal),
`IMPORT_TIME_ERROR` (the first real run, before the prompt named where `Command` lives), and
eleven more reasons under `generate/tests/test_admit.py`, each on a handwritten candidate
constructed to fail in exactly one way.

### Temperature-0 determinism is not available, on any current model

`temperature` was **removed** from the Messages API on Opus 5, Fable 5, Sonnet 5, Opus 4.8 and
4.7 — passing it is a 400, not a no-op. Pinning generation to an older model to buy determinism
would trade the generator's capability for it.

Replaced by the **transcript**: every call writes `generate/transcripts/<prompt_hash>.json`, and
`ReplayBackend` rebuilds a candidate byte-for-byte from committed files with no model, no key and
no network. Generation is therefore **reproducible but not repeatable**, the two runs of the bad
card are the proof, and `test_the_generation_model_does_not_accept_a_temperature` fails loudly if
this ever changes back. Chunk 17 must say which of the two the reported numbers rest on.

### Decisions the plan did not specify

**The model call is `claude -p`, not the API.** There is no API key and no `anthropic` SDK on this
machine. The CLI is a real call to a real model and it starts a **fresh session per call**, so A,
B and D cannot leak into each other through a shared context — the same class of contamination as
chunk 11's seal, and not otherwise structurally prevented. Run with `--setting-sources ''` from a
scratch directory so this repo's own CLAUDE.md is not in the model's context; verified by probe
("NO PROJECT CONTEXT"). It still carries Claude Code's tool definitions, which is **identical
across all three conditions** and so cannot differentiate them. `AnthropicBackend` is written for
the container run and is **NOT EXERCISED**.

**The import allowlist is the load-bearing check.** A candidate may import `math`, `numpy`,
`core.observation`, `nodes.contract`, `harness.micro` — nothing else. That makes "no coupling to a
specific scenario" mechanical (no `core.scenario`), keeps ground truth out (no `core.geometry`),
and extends chunk 11's seal to code that does not exist when the AST guard runs. ⚠️ `admit.py`
**executes model-written Python with no sandbox**; the allowlist is a static barrier against
carelessness, not against intent, and says so.

**The card's Prediction is authoritative for condition B**, so B cannot pass by predicting
something easier than the card did. Whether the model *restated* it faithfully is recorded as
`prediction_restated_faithfully` rather than being fatal: rejecting on it would conflate "the
mechanism does not work" with "the model copied a number wrong".

**Only Actions can be admitted today**, and a Condition or Decorator is rejected with
`KIND_NOT_TESTABLE_ALONE`. A unit scenario measures a navigation metric over a rollout and only an
Action emits a Twist. Testing a Condition means pairing it with a fixed consumer and comparing
against the same tree with a stand-in — which is a composition, and composition is chunk 16. All
five pilot cards produce Actions, so nothing is blocked; the reason hands 16 the problem stated.

**Fixture cards are excluded from G1's pass rate**, mechanically: a `card_id` that does not
resolve in the store was never part of the corpus. Excluded ids are returned, not dropped.

### A contract collision, found and fixed

Chunks 12 and 14 disagreed. `nodes/contract.py` required a CARD-origin node to carry a source
span; chunk 14 deliberately left every span empty because there is no corpus on this machine. A
condition-B node was therefore **unconstructible**, and both ways out were lies — substitute the
citation for the span, or record a corpus node as INVENTED.

Fixed by separating **origin** (where the node came from) from **traceability** (whether anyone
opened the paper). A CARD node must name its card and confidence; the span stays optional and
`is_traceable` still demands it. An unverified card now yields a node that is CARD and
untraceable — counted in G1's denominator, not its numerator. This is the only edit to a
previously-closed chunk's file, and `contract.py` records the reason inline.

### What I did not do

**Only one node exists in any library.** G1 wants ≥20 admissible nodes and the coverage count is
per condition; four calls is a pilot of the pipeline, not the pipeline's output. Generating the
library is the obvious next run and needs no new code — `scripts/generate_node.py` in a loop over
`cards/` — but it costs real model calls and the vague-card finding should be settled first, since
it changes what an admitted B node is evidence of.

**No source span was verified.** Unchanged from chunk 14; `cards/VERIFICATION.md` still reads
0 of 5, and G1's provenance fraction is still 0.0.

### For chunk 17

1. **Card fidelity must be declared.** §1. Either a stated limitation with B−D as the control, or
   a pre-registered check. It cannot be left implicit.
2. **State whether reported numbers come from a live call or a replayed transcript.** They are
   different claims and only the second is reproducible.
3. **B−A does not isolate the corpus.** A is not asked to articulate a principle and D is, so
   B−A confounds the corpus with the act of principle articulation while B−D isolates the corpus.
   CLAUDE.md currently asserts the stronger claim; it should be softened to match the prompts, or
   the prompts changed. Recorded here because the prompts are now written and the asymmetry is
   deliberate — it is what makes D a different condition from A rather than a rewording of it.
4. **A cleared effect floor is not evidence of generalisation.** The generator designs its own
   micro-world, so it can build one adversarial to the baseline. `FINDINGS.md` §2.

### For chunk 16

`FINDINGS.md` §2 suggests a cheap partial check that belongs with composition: run each admitted
node on the *other* admitted nodes' micro-worlds. A node implementing a separable mechanism should
degrade gracefully; one tuned to a self-designed trap should collapse. That is also the natural
place to make Conditions and Decorators testable.
