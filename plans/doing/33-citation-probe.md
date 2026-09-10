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

⚠️ **Deviation from this plan as first written, 2026-08-30.** The draft said "accept condition
`e`" and "add `generate/prompts/condition_e.md`" while also saying generating E *nodes* is not
in scope. Those conflict: adding `e` to `CONDITIONS` makes `scripts/generate_batch.py` enumerate
it and generate nodes, makes `test_prompts.py`'s three-way strip-and-diff apply to a prompt whose
deliverable is a card rather than a node, and makes E look like a condition — which is precisely
what this chunk says it must not become. **`CONDITIONS` stays `("a", "b", "d")`.** E is a
card-reconstruction probe with its own prompt, and a test asserts `condition_e.md` does not
exist. The B−E / E−D decomposition remains the motivation, to be answered in a later cycle if
the card diff justifies the episodes.

- ✅ `cards/probe/` — reconstructions and `FINDINGS.md`, committed, with a README saying in the
  first line that nothing in it is a principle card. `cards.store` cannot read it two ways over:
  its glob is non-recursive, and `CardStore.load()` pointed straight at the directory **raises**.
  Both asserted.
- ✅ `generate/prompts/probe_card.md` — the citation-only prompt. Carries the citation, the
  system description and the six-field schema, and explicitly invites *"I do not know this
  document"* as a correct answer, because an honest refusal is a result and a plausible
  reconstruction is the failure being measured.
- ✅ `generate/probe.py` — render, leak-check, parse, diff. `leaked()` runs on **every call**
  and refuses before the model is reached: if the card's mechanism, trigger, derivation, span or
  prediction statement got into the prompt, E measures transcription rather than recall and the
  output would look identical. A reconstruction is parsed by `cards.store.card_from_dict` — the
  real loader, now public — so a reconstruction that would not load as a card is a failed
  reconstruction rather than something the probe softens.
- ✅ `scripts/probe_citations.py` — `--dry-run`, `--card`, `--verified-only`, `--report`.
  Writes `cards/probe/FINDINGS.md` **generated from the diffs**, never hand-maintained.

The span result is deliberately left **UNRESOLVED** in the generated tables rather than scored.
Whether a reconstructed span is real is a question about a PDF, and the generator has no PDF —
writing a verdict nobody checked would be the same confabulation the probe exists to detect, one
level up. `FINDINGS.md` has a *Written verdicts* section a human owns.

## Done when

- ⬜ `cards/probe/FINDINGS.md` reports, per citation, a field-level reconstruction result and an
  explicit verdict on the Prediction's numbers against the verified span. **Needs the five model
  calls and then a human with the papers open.**
- ✅ A test proves `cards/store.py` cannot load a probe card — both by glob and by direct load.
- ✅ A test proves the probe prompt renders no mechanism, trigger, derivation, span or
  prediction statement, on every card in the store, and that the guard fires when poisoned.
- ✅ A test proves no fifth condition was smuggled in: `CONDITIONS == ("a", "b", "d")` and
  `condition_e.md` does not exist. (The strip-the-literature-block test still covers three
  prompts, not four — see the deviation above.)
- ✅ The sim-free suite stays green (565 tests), and `PRE-REGISTRATION.md` is **unchanged** —
  this chunk reports outside G2 and pre-registers nothing.

## Not in scope

**Running E as a G2 condition.** No episodes, no sweep binding, no entry in
`runs/spec.py:CONDITION_LABELS`, no amendment to the tagged pre-registration. If the card diff
shows E is worth 45 episodes, that is a cycle-2 decision made *with* this chunk's evidence in
hand rather than without it.

Generating E *nodes* and admitting them into a library. The probe stops at the card. Node
generation from an E-card would need a fifth library, a fifth snapshot tag and #8 compliance,
and none of that is bought by the question this chunk asks.

**Status 2026-08-30: machinery built and tested; no model call made.**
`scripts/probe_citations.py --dry-run` lists all five citations and which have an answer key
(four do; `limit_speed_ics` deliberately does not). Running it for real is five calls.

## Open questions

1. Which model produces the E-cards? It must be the same model as B's generation, or B−E
   confounds extraction with model identity.
2. Is a field-level diff scoreable without a human judge? The Prediction field is mechanical;
   Mechanism and Derivation are prose and probably need a stated rubric, applied once, recorded.
3. `limit_speed_ics`'s span is unverified on purpose (wrong edition of the paper). It has no
   answer key, so it is a **fifth call with no scoreable result** — include it for completeness
   or exclude it and say so?

---

## Status 2026-08-31 — the five calls are made. Three findings.

All five citations probed with `claude-cli` / `claude-opus-5` — **the same model and backend
every condition-B generation used**, checked against the committed transcripts, so open question
1 is answered: B−E is not confounded with model identity. Five transcripts committed. Results:

| Citation | Outcome | Exact fields |
|---|---|---|
| `limit_speed_dwa` | loadable card | 4/5 |
| `path_integration_ants` | loadable card | 3/5 |
| `limit_speed_ics` | loadable card | 2/5 |
| `path_integration_mammals` | **card returned, loader refused it** | — |
| `reorient_geometry` | **card returned, loader refused it** | — |

Open question 3 resolved in favour of **including** `limit_speed_ics`. It has no answer key and
its span row reads "no answer key" rather than a score, so it costs one call and contaminates no
number; when the *Advanced Robotics* edition is obtained the reconstruction is already on disk
and becomes scoreable without re-running.

### Finding 1 — the probe crashed on the one answer its own prompt calls correct

Two of the five calls died with `AttributeError: 'NoneType' object has no attribute 'strip'`.
Cause: `cards/store.py:card_from_dict` read every optional field with `dict.get(key, default)`,
and **that default fires only when the key is MISSING**. A YAML `span:` with nothing after it
parses to `None`, not to `""`, so the None sailed past the loader and detonated at a distant
`.strip()` in `probe.diff()`.

`generate/prompts/probe_card.md` explicitly invites *"I do not know this document"*, and an
empty span is how a model writes it — so the failure landed precisely on the honest-refusal path
the probe exists to reward. Fixed at the loader with `_opt()`, which treats a null and an
omission as the same thing, since they mean the same thing. This was **never probe-specific**: a
hand-written card with a bare `span:` would have crashed the same way.

**No model call was lost.** `probe()` writes the transcript before it diffs, so both crashed
calls were recovered with `--backend replay` at zero cost — which is the committed-transcript
design from chunk 15 paying for itself in the way it was meant to.

### Finding 2 — the two non-producers had not refused, and the report said they had

`path_integration_mammals` and `reorient_geometry` each returned a well-formed card in the
frozen six-field schema. Both were rejected by `card_from_dict`, both for **the same reason**:
each named `localisation_failures` as its `prediction.metric`, and `CONTROLLER_INDEPENDENT`
refuses it because no controller can move it — it is identical across controllers that behave
nothing alike and varies only with episode length.

`diff()` filed both under the fixed note *"the model returned no card"*, which is false, and
`write_findings()` printed that note **in preference to the recorded error**, so the reason
never reached the report. Two very different outcomes — the prompt working, and a fluent
reconstruction failing a falsifiability rule — were collapsed into one heading that named the
less interesting of them.

Fixed: `diff()` no longer asserts a cause it cannot know (it is handed `None` and cannot tell
the two apart); `probe()`, which saw the exception, sets the specific note; and `FINDINGS.md`
now carries **Returned a card the loader refused** and **Declined to reconstruct** as separate
sections with the headline naming all three outcomes. The split is derived from `error` rather
than a new key, so diffs already on disk classify correctly.

⚠️ **This is a result about the probe, not yet about the model.** Two of five reconstructions
proposing an unfalsifiable metric is the sharpest thing the run produced, and it is worth
stating carefully in the write-up: the human cards for these two citations name metrics a
controller *can* move, and the model — pointed at the same papers by citation alone — did not.
That is a claim about **one sample per citation**, on a pipeline whose non-repeatability chunk
15 already established. It is a lead, not a measured rate.

### Finding 3 — the verdicts still need a human, and that is by design

`cards/probe/FINDINGS.md` reports every mechanical field and marks `limit_speed_dwa`'s
reconstructed span **UNRESOLVED**. The generator has no PDF, and writing a verdict nobody
checked would be the same confabulation the probe exists to detect, one level up.

**The Done-when is therefore still open**, and what remains is not machinery: someone opens
`out/corpus/limit_speed_dwa.pdf` and decides whether the model's claimed span is real, then
writes the *Written verdicts* section. The reconstructions, diffs and transcripts are all on
disk and committed; nothing needs re-running.

Suite: **570 sim-free tests** (565 + 5 regression tests for the two defects above).
