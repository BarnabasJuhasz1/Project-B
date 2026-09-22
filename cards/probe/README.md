# `cards/probe/` — reconstructed cards, never real ones

> ⚠️ **PROMOTED, 2026-09-21: this directory feeds chunk 59.** `plans/todo/33-citation-probe.md`
> came back out of `hold/` because it is **card-level and architecture-independent**, and its
> three reconstructions plus their `.diff.json` are folded into the card-level report
> (`plans/todo/59-card-level-report.md`) as the recall-against-a-human-extraction half of the
> provenance result.
>
> ⚠️ **Nothing below changes.** These remain model reconstructions, never principle cards;
> `cards.store` still refuses to load this directory two ways over; they are counted in no G1
> number and no episode is ever run from one. The report must say so where the numbers appear.

⚠️ **Nothing in this directory is a principle card.** These are what a model
recalls when it is handed **a citation and nothing else**. They are not extracted
from any document, nobody has opened the papers to write them, and a span in
here is a guess until someone checks it.

They exist to answer one question, which is chunk 33's:

> Same paper, same target mechanism. The only difference is whether a human
> actually read it. **Does the extraction step buy anything?**

`B − D` is what the corpus buys in total, and it is pre-registered. This
decomposes it: `E − D` is what *pointing* at a finding buys, `B − E` is what
*extracting* it buys over the model's own recall. `B − E` is the sharp one — if
it is near zero, the extraction step is doing nothing the model could not recall,
and that is a finding worth more than a null headline.

## What is in here

| File | What it is |
|---|---|
| `<card_id>.yaml` | one reconstruction, keyed to the real card it is a reconstruction *of* |
| `FINDINGS.md` | the written result: what was reconstructed, what was wrong, and what the numbers did against the verified span |

## Guards

- **`cards.store` cannot read this directory.** Its glob is non-recursive, so
  these are invisible to a normal load; and pointing `CardStore.load()` straight
  at `cards/probe/` raises. `cards/tests/` proves both.
- **These are not counted in any G1 number.** Card count, provenance fraction and
  coverage are computed over `cards/*.yaml` only.
- **This is not a fifth condition.** `runs/spec.py` refuses one, and no episode
  is ever run from anything in here. See `plans/todo/33-citation-probe.md`.

## Reproducing

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P scripts/probe_citations.py --dry-run        # the plan; calls nothing
$P scripts/probe_citations.py --card limit_speed_dwa
```
