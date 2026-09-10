# 35 — The unseen corpus: testing the novelty claim

**Epic:** [04](../04-epic-evaluation.md) · **Depends on:** 29, 34 · **Needs Isaac Sim:** no
**Status: PLANNED ONLY.** Written 2026-09-06 at the user's direction, so that the two-stage
corpus design is on record before the first stage is reported.

## Why this exists

CLAUDE.md names three things a corpus can supply that a model cannot supply from itself:
**novelty**, **coverage**, **provenance**. This cycle's pilot corpus tests coverage and
provenance and — through the citation arm — extraction. **It cannot test novelty**, and that is
deliberate rather than an oversight.

The five pilot cards are drawn from work the model knows. That was confirmed, not assumed:
condition A was handed each reference alone on 2026-09-06 and recognised all four it was asked
about, correctly and unprompted. Holding recall high and constant is what makes **B−A a clean
measure of extraction** — the derivation step, given a model that already knows the source. The
committed `limit_speed_dwa` pair is that result: A reconstructed the Dynamic Window Approach
faithfully and was rejected; B, given the card's derivation onto a 181-beam scan, was admitted.

Novelty is the opposite experiment and needs the opposite corpus.

| corpus | holds constant | measures |
|---|---|---|
| pilot, in-distribution | the model knows the paper | **extraction** (B−A) |
| **this chunk**, post-cutoff | the model cannot know the paper | **novelty** |

## What has to be true of the corpus

1. **Published after the model's training cutoff**, and demonstrably so — the publication date is
   part of the card, and it is checkable. A preprint that circulated earlier does not qualify.
2. **Same schema, same pipeline.** Six fields, `cards/*.yaml`, `CardStore` loads it, spans
   verified against the PDF on disk per `cards/VERIFICATION.md`. Nothing in `generate/` or
   `compose/` changes — that is the point of the source-agnostic card interface.
3. **Same admission gate.** A node from an unseen paper earns its place the same way, or the
   comparison is between a corpus and a lowered bar.
4. ⚠️ **Cards must be extracted by a human who read the paper**, exactly as the pilot cards were.
   A card written from a model's summary of an unseen paper measures nothing.

## The instrument already exists

The citation arm reads this corpus for free. Because A is handed the reference alone, **its
transcripts record whether the model claims to recognise each source** — so "unseen" becomes an
observation with a per-card answer rather than an assumption. Three outcomes, all informative:

- **A says it does not recognise the paper** → the corpus is genuinely out of distribution, and
  B−A on it is the novelty measure this chunk exists for.
- **A claims recognition of a post-cutoff paper** → ⚠️ **a confabulation, caught**. This is
  precisely what the provenance axis was invented to detect, and it is a reportable finding on
  its own, independent of any episode.
- **A recognises it because the finding predates the paper** → the citation is new, the mechanism
  is not. The card needs a different source, and the screen caught it before any calls were spent.

`generate/probe.py` (chunk 33) answers the same question at the card level and is the cheap
pre-screen: run it on a candidate citation **before** committing the card, and a reconstruction
that comes back accurate means the paper is not unseen.

## Open questions

1. **How many cards, and does the pilot corpus stay in?** Mixing seen and unseen cards in one
   library confounds the two measurements; running them as separate libraries doubles the sweep.
   ⚠️ #10 requires identical episode counts across conditions, and this is where that bites.
2. **Which model cutoff, and what happens when it moves?** A corpus that is unseen today is
   in-distribution after the next model release. The date is a property of the (model, corpus)
   pair and must be recorded in the manifest, not assumed.
3. **Is a null result interpretable?** If B−A does not fire on unseen work, that is either "the
   corpus adds nothing" or "these particular papers add nothing", and the pre-registration's
   existing rule — a rule that does not fire may not be reported as "the corpus does not help" —
   applies with more force here, not less.

## Not in scope

Changing the card schema, the node contract, the admission gate, or any condition's prompt. If
this corpus needs any of those, the comparison to the pilot corpus is gone.
