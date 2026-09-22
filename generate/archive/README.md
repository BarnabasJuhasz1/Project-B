# `generate/archive/` — superseded generation records, kept intact

> ⚠️ **A FOURTH PIPELINE EXISTS FROM 2026-09-21: `v4-edited-base`.** Epic 07 drops the behaviour
> tree; the generated artefact is a whole `Controller` produced by editing the frozen base.
> It stages into its own directory with its own `pipeline_version`, and ⚠️ **`admit.pass_rate`
> must refuse to pool it**, exactly as it refuses `v1-single-call`.
>
> ⚠️ **v3 (the modifiers now in `generate/staging/`) is NOT archived by this pivot.** It is the
> comparison epic 07 is measured against, and moving it would break the growth tables. The
> mechanical reason v2 had to move — `plan_hash` is a pure function of `(condition, task, card)`
> and the cards are not regenerated, so a new candidate lands on an identical stem — is handled
> differently this time: **`plan_hash` takes a sample index** (k = 3 samples per row), so v4
> stems cannot collide with v3's.

⚠️ **Nothing here is deleted and nothing here is live.** These are complete
records of generation pipelines that no longer exist. They are real evidence
about real questions and the denominators they belong to may not be edited after
the fact — but they are not part of the current experiment, and mixing them into
it would compute one number over two treatments.

## `v1-single-call/`

The **one-call** pipeline, retired by chunk 42 on 2026-09-08. A condition's
prompt carried a human's extracted card (B), a citation (A) or nothing (D), and
the model wrote a leaf node straight from it. There was no card call and no
shared node prompt.

```
v1-single-call/staging/       21 candidates + their verdicts
v1-single-call/transcripts/   36 model calls, keyed by prompt hash
```

The prompts that produced them are still in `generate/prompts/` as
`condition_{a,b,d}.md` — **kept and never edited**, because the transcript hash
is over the *rendered* prompt. `generate.generator.render_v1()` is the frozen
renderer that reproduces them; a test checks it still yields `d6ae93648c7b4f25`
and `5771c092006dccbf`.

### What still depends on these, and how it reaches them

Address them with `generator.V1_STAGING` / `generator.V1_TRANSCRIPTS`, never by
copying them back:

| consumer | why it needs v1 |
|---|---|
| `scripts/ab_compare.py` | chunk 39's paired A/B table is *about* these records — `DEFAULT_PIPELINE = "v1-single-call"` |
| `compose/compositions/*.json` (8) | every one names a v1 node and a `lib-006-…` snapshot; they reconstruct through `load_library(staging=..., only=...)` |
| `results/…node-gen_*.json` (7) | committed episode results for v1 nodes |
| `out/evidence/nodes/ab-comparison/`, `…/reorient-geometry-a-vs-b/` | the written findings |
| `generate/tests/test_replay.py`, `test_provenance.py`, `test_ab_compare.py` | they replay the committed v1 transcripts |

### Reading the numbers

```python
from generate.admit import pass_rate
from generate.generator import V1_STAGING
pass_rate(V1_STAGING, pipeline_version="v1-single-call")   # a 5/5, b 5/5, d 4/4
```

`pass_rate` still refuses to pool pipelines even when pointed here, which is
belt and braces: the directory separation and the version stamp say the same
thing twice.

## `v2-exclusive-control/`

The **card-first, exclusive-control** pipeline, retired by chunk 52 on
2026-09-14. Two calls — the arm writes a card, then one shared node prompt
(`generate/prompts/node.md`) implements it — but the node it asked for was a
leaf that **drives**: it won the tick outright and everything below it in the
`Fallback` went silent.

```
v2-exclusive-control/staging/       15 candidates + their verdicts
v2-exclusive-control/transcripts/   15 NODE calls, keyed by prompt hash
```

⚠️ **The CARD calls are NOT here.** They are still live, in
`generate/transcripts/`, and so are the cards themselves in `generate/cards/`.
The card call did not change: v3 reuses every committed card byte for byte and
regenerates only the node. What was retired is the second call, and only the
second call's records moved.

### Why it was retired — the measurement, not a preference

Growth round 1, 2026-09-14: each admitted node placed in
`Fallback[specialist, BASE]` and scored against the base alone on 20 identical
`maze` worlds.

- Not one of eleven nodes improved the tree. **`fixed` was 0 in every row.**
- Fire rate inside the tree was bimodal with nothing between: five nodes never
  fired, six fired on 87.6-100% of ticks. At 100% the base is never ticked at
  all, so the tree scores exactly what the specialist scores alone.
- **B−A was exactly zero on every column.**

The prompt `generate/prompts/node.md` that produced them is **kept and never
edited**, for the same reason `condition_{a,b,d}.md` are: the transcript hash is
over the rendered prompt. `generator.V2_NODE_PROMPT` is how to reach it.

### What still depends on these

Address them with `generator.V2_STAGING` / `generator.V2_TRANSCRIPTS`:

| consumer | why it needs v2 |
|---|---|
| `compose/compose.py` | `RECONSTRUCTION_STAGING` walks live → v2 → v1 |
| `generate/FINDINGS.md`, growth round 1 | the findings are *about* these records |
| `out/growth/*.csv` | the round-1 table names these node ids |

```python
from generate.admit import pass_rate
from generate.generator import V2_STAGING
pass_rate(V2_STAGING, pipeline_version="v2-card-first")   # a 4/5, b 7/10
```
