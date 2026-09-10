# 34 — Condition C: invented principles, informed by failure context

**Epic:** [04](../04-epic-evaluation.md) · **Depends on:** 29, 31 · **Needs Isaac Sim:** no
**Status: PLANNED ONLY.** Nothing here is built. Written 2026-09-06 alongside the condition
restructure so that C's definition is on record before any C call is made.

## What C is

C is the **old condition D, given context about how the robot actually fails.** It receives:

- the task family (the same `TASKS` entry every condition gets), and
- **failure context**: what went wrong when this robot ran this class of task — episodes that
  timed out, collisions, stuck events, how long a run lasted before it ended, and similar
  observable outcomes.

It does **not** receive a corpus, a citation, or a card. It devises the principle itself, exactly
as the old D did, and states it before implementing — the `PRINCIPLE` block in
`generate/prompts/condition_d.md` carries over unchanged.

So the three-arm structure is:

| | receives |
|---|---|
| **A** | the **citation** of a paper, and none of its content |
| **B** | the citation **and** the extracted card — mechanism, trigger, derivation, prediction |
| **C** | no corpus; the task **and failure context from real episodes** |

**A and B differ by the paper's content. B−A is what extraction buys.** C is the floor: what the
model produces with no literature at all, but with evidence about this robot in this environment.

## Why this is the right shape

This is the project's own **second generation trigger**, which the description has specified from
the start and which has never been built: *"failure-driven (during evaluation): failure signatures
become retrieval queries for targeted additions."* Chunk 29 costed it as lever (iii) and noted the
one property that matters here:

> A signature is produced by a scored episode, so it is **neutral by construction**: the same
> signature is handed to A, B and D.

That neutrality is why C can receive failure context without becoming a different experiment —
provided the same context is available to every condition that wants it. **Decide that explicitly
before building**: either failure context is C's treatment alone (and C−A then confounds "no
corpus" with "has failure data"), or it is offered to all three (and C is then genuinely the
no-corpus arm). ⚠️ **This is the central open question of this chunk and it is not settled here.**

## What has to exist first

C cannot precede the sweep. A failure signature is a property of a scored episode, so:

1. **31 must have run at least once.** Until then there are no failures to summarise, and a
   hand-written "the robot tends to get stuck" is the author's prior, not evidence.
2. **The signature format has to be decided and frozen** — which metrics, over which episodes,
   summarised how. `core/metrics.py:score()`'s eighteen keys are the vocabulary; the question is
   which subset, and at what aggregation, is handed to a generator without handing it the
   scenario. ⚠️ **A signature that names a scenario is leakage** — the same rule that governs
   `TASKS`. It must be expressed in the contract's terms (timeouts, collisions, stuck events,
   episode duration), never as "it fails in the room with the divider".
3. **The prompt has to be written**, as `generate/prompts/condition_c.md`, differing from the
   others **only inside the literature block** — `generate/tests/test_prompts.py` enforces that
   by stripping the block and diffing the rest, and C is not exempt.

## Build sketch

- `generate/prompts/condition_c.md` — the old D block plus a `{{FAILURE_CONTEXT}}` slot.
- `generate/generator.py` — a third rendering path beside `{{CARD}}` and `{{CITATION}}`;
  `CONDITIONS` becomes `("a", "b", "c")` and D is retired the way the naive arm was.
- A failure-context builder, sim-free, reading committed results rather than re-simulating —
  the same discipline `scripts/rescore.py` follows.
- `generate/tests/test_ceiling.py` — C's ceiling is `|TASKS|` unless the failure context varies
  per call, in which case **the ceiling question reopens** and must be costed before spending.
- `runs/spec.py` — the condition→controller binding for C.

## Not in scope

Reviving the Nav2 human baseline (the old C). **That is retired**: it is not being built this
cycle, and the letter has been reused. Say so in the write-up rather than leaving a reader to
wonder what happened to it.

## Open questions

1. Is failure context C's treatment alone, or available to every condition? (See above — this
   decides what C−A means.)
2. Does the failure context vary per call? If it does, the prompt-identity ceiling stops binding
   C, and C could out-enumerate A and B. That would be a new uncontrolled difference.
3. How many episodes of failure does a signature summarise, and from which split? ⚠️ It may
   **never** be the held-out split — that is the seal.
