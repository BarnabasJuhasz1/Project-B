# `generate/cards/` — model-written cards, never principle cards

⚠️ **Nothing in this directory was extracted from anything.** These are the cards
each condition wrote *for itself*, one per grid row, in chunk 42's card call.
Nobody has opened a paper to write any of them and every `span` in here is a
**claim**.

`cards/*.yaml` is the corpus: five principle cards, extracted by a human, four
of five spans verified against the PDF (`cards/VERIFICATION.md`). That store is
what G1's *corpus* provenance number is computed over, and it is now an input to
the **card call** only — the citation A is pointed at, the key naming the PDF B
reads, and the answer key both are checked against. **It no longer reaches a
node prompt at all.**

## What is in here

```
generate/cards/<condition>/<condition>__<ref-or-task>__<row key>.yaml
generate/cards/<condition>/<...>.refusal.md     the call produced no card
```

The stem is the same one `generate/staging/` uses for the candidate that card
produced, which is what lets `generate.admit.load_generated_card` find the card
a node was judged against.

| arm | what its card call was given |
|---|---|
| `a/` | the reference alone — authors, title, venue, year |
| `b/` | the source document, in full |
| `d/` | nothing |

## Guards

- **`cards.store` cannot read this directory**, either way in: the corpus glob
  is non-recursive, and pointing `CardStore.load()` at `generate/cards/` or at
  one of its arms raises. `generate/tests/test_cardgen.py` proves both.
- **No card in here is ever counted as traceable.** `verified` is written as
  `unverified` unconditionally and never carried over from the model's own
  claim; `NodeMetadata.span_verified` is what `is_traceable` demands, and
  nothing in the generation path sets it.
- **A refusal is kept.** G1 reports a rate and a rate needs a denominator, so a
  call that produced no card is staged as `.refusal.md` rather than retried.

## Committed

Yes — for the same reason `generate/transcripts/` and `generate/staging/` are.
The card is the artefact that makes the node call reproducible **with no corpus
at all**: replay renders the node prompt from the committed card, so a container
run needs no PDF. What needs the PDF is exactly one step, re-running B's card
call from scratch. See `generate/corpus.py`.
