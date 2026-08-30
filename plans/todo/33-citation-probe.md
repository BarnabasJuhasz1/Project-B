# 33 — The citations probe: does extraction earn its place?

**Epic:** [04](../04-epic-evaluation.md) · **Depends on:** 14, 15 · **Needs Isaac Sim:** **no**
**Blocks:** nothing. This chunk is a *probe*, not a gate — the evaluation runs without it.

⚠️ **This is not a fifth G2 condition, and it must not become one.** `runs/spec.py` refuses a
fifth condition by design, and the reason is in the source: #10's identical-episode-count
argument is stated over exactly four. Adding one costs an amendment to a tagged
pre-registration plus a full extra sweep. This chunk buys the most interesting part of that
question for **zero episodes**.

## The question

`PRE-REGISTRATION.md` §1 records that only **B−D** isolates the corpus. True, but B−D is a
*bundle*. The corpus does two separable things:

1. it **points** the model at a specific finding, and
2. it **supplies that finding's content**, extracted and verified by a human.

Call the pointing-only treatment **E**: the model is given a card's `citation` — author, title,
year, venue — and the node contract, and nothing else. No mechanism, no trigger, no derivation,
no span. Then:

```
B − D  =  everything the corpus buys          (already pre-registered)
E − D  =  what POINTING buys                  (direction alone; the model recalls the paper)
B − E  =  what EXTRACTION buys                (same paper, same target — the only difference
                                               is whether a human actually read it)
```

**B − E is the sharpest question in the design.** If it is near zero, the extraction step is
doing nothing the model could not recall unaided, and that is a finding worth more than a
null headline. If it is large, the novelty claim in CLAUDE.md is demonstrated rather than
asserted.

## Why E is not A, and does not replace it

A and E differ on **two axes at once**, so A−E is not a clean contrast — the same defect chunk
17 diagnosed in B−A:

| | Articulates a principle? | Directed at a specific mechanism? | Given the mechanism's content? |
|---|---|---|---|
| **A** | no | no | no |
| **D** | yes | no | no |
| **E** | yes | **yes** (by citation) | no |
| **B** | — (handed one) | yes | **yes** |

**A stays.** It is the literal operationalisation of the claim under test — *"an LLM asked
plainly to program a Go2 to do X"* — and it is the **only** condition that measures the model's
*unprompted* mechanism distribution. That distribution is precisely what G1's coverage argument
is about (*"a prompted model returns the handful of mechanisms most represented in its training
distribution"*), and it cannot be read off E, because in E the mechanism was named for it. A is
the floor; E is a decomposition of the B−D gap. Different questions, neither substitutes.

E's comparators are D and B. A's are B and C. Do not build a comparison on A−E.

## The cheap form: a card diff, no episodes at all

E's most valuable output does not need the simulator, the container, Nav2 or the sweep. It is a
**text-level comparison of what the model reconstructs against what the paper actually says**,
and `cards/` already holds the ground truth for five citations with four verified spans.

For each card in `cards/`:

- render a **citation-only** prompt asking the model to produce a card in the frozen
  six-field schema for that reference,
- parse it with `cards/schema.py` so it is a real card and not prose,
- diff it field-by-field against the human-extracted card, and
- score the **Prediction** field hardest: it carries a structured `(metric, baseline,
  min_effect, cost_metric)`, so a reconstructed threshold or exponent is either right or it is a
  confabulation, and `cards/VERIFICATION.md` says which of the five spans is verified.

That last point is the whole reason this is worth doing. CLAUDE.md's third corpus claim is
provenance: *"a model states a threshold or an exponent with equal confidence whether or not it
is confabulating."* This chunk **measures** that, on real citations, with a verified span as the
answer key, for the price of five model calls.

## Build

- `cards/probe/` — a directory for E-cards, committed. Never loaded by `cards/store.py`, so a
  reconstructed card can never reach condition B's retrieval. ⚠️ A test asserts this: an E-card
  in the B store would silently corrupt the headline comparison.
- `generate/prompts/condition_e.md` — the citation-only prompt. Built from `condition_d.md`
  (E articulates a principle, as D does) with the citation substituted into the literature
  block, so the same strip-and-diff test that governs A/B/D governs E. **This adds a fourth
  prompt file; it does not modify the three existing ones**, so no committed transcript is
  orphaned.
- `generate/generator.py` — accept condition `e`, which takes a card but renders only its
  `citation`. The existing guard ("a card reaching A or D would void the ablation silently")
  must be extended, not bypassed: E gets the citation and provably nothing else, asserted by a
  test that greps the rendered prompt for the mechanism and derivation text and fails if found.
- `scripts/probe_citations.py` — runs the diff over `cards/`, writes
  `cards/probe/FINDINGS.md`: per card, which fields were reconstructed, which were wrong, and
  whether the Prediction's numbers match the verified span.

## Done when

- `cards/probe/FINDINGS.md` reports, per citation, a field-level reconstruction result and an
  explicit verdict on the Prediction's numbers against the verified span.
- A test proves `cards/store.py` cannot load an E-card.
- A test proves `condition_e.md` renders no mechanism, trigger or derivation text.
- The strip-the-literature-block diff test covers four prompts, not three.
- The sim-free suite stays green, and `PRE-REGISTRATION.md` is **unchanged** — this chunk
  reports outside G2 and pre-registers nothing.

## Not in scope

**Running E as a G2 condition.** No episodes, no sweep binding, no entry in
`runs/spec.py:CONDITION_LABELS`, no amendment to the tagged pre-registration. If the card diff
shows E is worth 45 episodes, that is a cycle-2 decision made *with* this chunk's evidence in
hand rather than without it.

Generating E *nodes* and admitting them into a library. The probe stops at the card. Node
generation from an E-card would need a fifth library, a fifth snapshot tag and #8 compliance,
and none of that is bought by the question this chunk asks.

## Open questions

1. Which model produces the E-cards? It must be the same model as B's generation, or B−E
   confounds extraction with model identity.
2. Is a field-level diff scoreable without a human judge? The Prediction field is mechanical;
   Mechanism and Derivation are prose and probably need a stated rubric, applied once, recorded.
3. `limit_speed_ics`'s span is unverified on purpose (wrong edition of the paper). It has no
   answer key, so it is a **fifth call with no scoreable result** — include it for completeness
   or exclude it and say so?
