# `generate/archive/` — superseded generation records, kept intact

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
