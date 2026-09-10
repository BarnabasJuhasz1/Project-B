# 42 — Card-first generation: every arm writes its own card

**Status:** doing. SIM-FREE. Model calls required to populate; the machinery is
testable with `--backend replay` and `--dry-run` alone.
**Blocked by:** nothing.

---

## What changes, in one sentence

Generation becomes **two calls instead of one**: each condition's model first
writes a **principle card**, and then an **identical** node prompt turns that
card into a leaf.

## Why — this is the diagrammed design, and it was never built

`ProjectB_pipeline.drawio` (2026-09-02) has three card-writing boxes:

| box | subtitle |
|---|---|
| `A - LLM Writing Card` | *(from training memory of that citation)* |
| `B - LLM Writing Card` | *(with literature context)* |
| `C - LLM Writing Card` | *(by inventing a solution)* — today's **D** |

→ `Cards` → `Node` → `Validity Check` → `Node Library`.

The code has never done this. Until the 2026-09-06 restructure A was the naive
arm (`condition_a.md`: *"Nothing is supplied"*); after it, A renders the human
card's citation and B renders its content, and **both go straight to a node**.
One human card, two nodes. This chunk builds what the diagram says.

## The point of it: the independent variable moves to the card step

Today the arms differ inside a delimited literature block in three otherwise
identical **node** prompts, and `test_prompts.py` diffs the rest to prove it.
That works, but the arms then differ at the step that also writes the code.

After this chunk:

- **the card prompts differ** — and only inside a delimited `SOURCE BLOCK`;
- **the node prompt is one file**, shared by every condition, fed a card and a
  task family and **nothing identifying** — no condition letter, no paper, no
  human-written text.

So "the arms differ by exactly one thing" stops being a diff test over three
files and becomes structural: after the card call, the pipeline cannot tell the
arms apart. This is the same move `compose/` already made — a composition sees
port contracts and a task spec, never the papers — applied one step earlier.

## What each arm's card call receives

| arm | source block | origin |
|---|---|---|
| **A** | `{{CITATION}}` — authors, title, venue, year, and nothing else | `CITATION` |
| **B** | `{{PAPER}}` — the extracted full text of the PDF | `CARD` |
| **D** | nothing at all | `INVENTED` |

**B is given the paper, not the human card.** That is the user's decision of
2026-09-08 and it is a real change of treatment: B was reading a human's
extraction and is now reading the document. The human cards stop being an input
to generation and become what they always claimed to be — the corpus record, the
verified-span answer key, and the source of G1's *corpus-level* provenance
number.

### The card step is task-conditioned, for all three

All three card prompts carry `{{TASK}}` in the **shared** body. D needs a
problem to invent against, and a prompt that carried `{{TASK}}` for one arm and
not another would differ outside the source block, which is the one property
this design is built to keep.

Consequence: A and B write one card per (paper × task family), not one per
paper. The reachable ceiling is therefore **unchanged** — `|cards| × |TASKS|` =
20 for A and B, `|TASKS|` = 4 for D — and `test_ceiling.py`'s numbers hold.

## Hashing, staging and resume — what has to move and why

`plan()` today computes the node prompt hash up front, because `render()` is a
pure function of the frozen inputs. **It no longer can**: the node prompt
depends on a card that does not exist until the model writes it.

So the **card prompt hash becomes the row key**. It is still a pure function of
`(condition, task, paper)`, so `--dry-run` stays the real code path minus the
calls, `staging/` stays the only state, and a stem collision is still refused
before the first call.

- stem: `<condition>__<ref-or-task>__<card_prompt_hash>`
- `node_id`: `gen_<condition>_<card_prompt_hash>`
- the record carries **both** hashes; `node_prompt_hash` is written after the
  second call.

Replay still chains: replaying the card call reproduces the card, which
reproduces the node prompt, which replays the node call.

## The corpus is an input, and B's transcript may not carry the paper

`out/corpus/` is gitignored and must never be committed — the PNAS copy carries
the downloader's IP address. But `write_transcript` stores the prompt **in
full**, and B's card prompt is a whole paper.

So transcripts gain **redaction**: B's card transcript stores the prompt with
the paper text replaced by a marker carrying the PDF's `sha256`, the extractor
and the character count. The hash recorded is the hash of the **real** prompt,
so replay still finds it.

⚠️ **Stated plainly rather than papered over:** B's card call is reproducible
only on a machine that has the PDF. That is the same status the scene assets
have — an input, named and hashed, not an output. Everything downstream of the
card *is* reproducible from committed files: the generated cards are committed,
and the node call replays from them with no corpus at all.

## Provenance — the number this changes, and the honest answer

Today a B node inherits the **human** card's verified span and confidence, so 4
of 5 are traceable. After this chunk nothing a human wrote reaches the node, so
carrying the human span onto a node built from a **model-written** card would be
handing B provenance it did not earn — the exact failure `Origin.CITATION` was
invented to prevent.

**Decision: the node carries its own generated card's claimed span and
confidence, and traceability still requires a human.** `NodeMetadata` gains
`span_verified`, `is_traceable` requires it, and no generated card has it.

Consequences, all of which go in the write-up:

1. **G1's node-level provenance fraction goes to 0.0** until someone verifies a
   generated span. The **corpus-level** fraction (`CardStore.provenance()`,
   0.80) is untouched — it was always a fact about the corpus.
2. **A's claimed spans become a measurement rather than a leak.** A is pointed
   at a paper it cannot read; a span it invents is precisely the confabulation
   the provenance axis exists to detect. `NodeMetadata`'s CITATION guard is
   relaxed to *record* a claimed span and tightened to forbid `span_verified`.
3. **B's spans are checkable against `cards/VERIFICATION.md`** — B was shown the
   document, so its span has an answer key wherever the human card is verified.
   Scoring that agreement is a follow-up, not this chunk.

## Old records are not deleted and not pooled

Every staged candidate and transcript on disk was produced by prompts that no
longer exist. The 2026-09-06 precedent is `retired_reason`, applied by hand.
This chunk does it **mechanically instead**: candidate metadata gains
`pipeline_version`, and `pass_rate` buckets by it and **refuses to pool**.

Nothing is deleted, nothing is rewritten, the existing library and the committed
compositions keep working, and no number can silently average two treatments.

## Build

1. `generate/corpus.py` — PDF → text, sha256, cache, frozen char budget.
2. `generate/prompts/card_{a,b,d}.md` — identical but for the source block.
3. `generate/prompts/node.md` — ONE node prompt, fed a card.
4. `generate/cardgen.py` — the card call: render, extract via the real loader,
   write to `generate/cards/<condition>/`.
5. `generate/generator.py` — two-phase `generate()`.
6. `generate/batch.py` — grid keyed on the card prompt hash.
7. `generate/admit.py` — metadata from the generated card; `pass_rate` buckets.
8. `nodes/contract.py` — `generated_card_id`, `span_verified`, relaxed guard.
9. `cards/store.py` — refuse `generate/cards/` the way it refuses `cards/probe/`.
10. Tests, and the docs: CLAUDE.md, a dated append to `PRE-REGISTRATION.md`,
    `generate/FINDINGS.md`.

## Done when

- `$P scripts/generate_batch.py --dry-run` prints the grid with card hashes.
- One real A, B and D row generated end to end and replayable.
- The node prompt is proven identical for every condition by a test.
- A test proves B's transcript carries no paper text.
- The sim-free suite is green.

## Not in scope

- Verifying any generated span against a PDF.
- Scoring B's span agreement against `cards/VERIFICATION.md`.
- Retiring or regenerating the existing library.
- Any change to G2's rule, the seed set, the episode count or the sealed split.

---

## Outcome — 2026-09-08

**Built and green.** Sim-free suite: **836 passed, 1 failed** — the failure is the
pre-existing `compose/tests/test_no_papers.py` port-name gap that chunk 39 recorded
as known and explicitly told later chunks not to fix (`home` comes from the port
`home_bearing`). It fails identically before and after this chunk.

**No model was called.** Everything below was verified with `--dry-run`, direct
rendering, and `ReplayBackend` over a hand-built fixture pair.

### What was built

| | |
|---|---|
| `generate/corpus.py` | PDF -> text, sha256, cache, frozen 200k-char guard. pdftotext, pdfplumber as fallback |
| `generate/cardgen.py` | the card call; `CITATION_CONDITIONS` / `PAPER_CONDITIONS` / `CORPUS_CONDITIONS` live here now |
| `generate/prompts/card_{a,b,d}.md` | spliced from one shared body, so they are identical outside the source block **by construction**, not by correction |
| `generate/prompts/node.md` | one file, every condition |
| `generate/cards/{a,b,d}/` | committed generated cards + a README |
| `generator.render_v1()` | FROZEN. Reproduces `d6ae93648c7b4f25` exactly, so no v1 transcript is orphaned |
| `NodeMetadata.span_verified`, `.generated_card_id` | the provenance change |
| `pass_rate(pipeline_version=...)` | refuses to pool v1 and v2 |
| `write_transcript(prompt_text=...)` | B's paper redacted from the record; hash still over the real prompt |

Tests: `test_cardgen.py` (20, new), `test_prompts.py` (18, rewritten),
`test_replay.py` (+6 for the two-call chain and the redaction), `test_contract.py`
(+2 for the provenance rule).

### Checks

| check | outcome | verdict | n |
|---|---|---|---|
| Are the three card prompts identical outside the source block? | yes, byte-identical | **confirmed** | 3 |
| Are the three source blocks all different? | yes | **confirmed** | 3 |
| Can `render_node` tell the arms apart? | **no** — signature is `(card, task)`, pinned by a test | **confirmed (negative)** | 1 |
| Does the ceiling move? | **no** — still (a 20, b 20, d 4); the card call is task-conditioned so the grid keeps its shape | **ruled out** | 44 rows |
| Does `plan_hash` stay computable with no corpus? | yes — the document is stood in for by its id | **confirmed** | 44 rows |
| Does the card file share a stem with its candidate? | yes, after keying `GeneratedCard.stem` on the row key rather than its own prompt hash | **confirmed** | fixture chain |
| Does the two-call chain replay byte-identically? | yes, twice, including the written card | **confirmed** | 2 runs |
| Does the replayed chain admit? | yes, `origin=invented`, `is_traceable=False` | **confirmed** | 1 |
| Does the paper reach a committed transcript? | **no** — redacted, `prompt_redacted: true`, sha256 present, hash still resolves | **confirmed (negative)** | 1 |
| Does `render_v1` still reproduce the committed v1 hash? | yes, `d6ae93648c7b4f25` | **confirmed** | 1 |
| Do the v1 records keep the provenance they had? | yes — `_node_metadata` falls back to the human card when there is no generated one, so the v1 library loads unchanged | **confirmed** | 14 nodes |
| Does `pass_rate` pool the two pipelines? | **no** — v1 skipped by default and named under `_superseded_pipeline`; readable with `pipeline_version="v1-single-call"` (a 5/5, b 5/5, d 4/4) | **confirmed (negative)** | 21 records |
| Can `cards.store` load a generated card? | **no**, both at `generate/cards/` and at `generate/cards/<arm>/` | **confirmed (negative)** | 2 paths |
| Does the real corpus still load? | yes, 5 cards | **confirmed** | 5 |
| Does B's prompt carry the document and not the human extraction? | yes / no respectively, checked field by field | **confirmed** | 1 card |
| Do all five PDFs extract cleanly? | yes, 17k-87k chars, no glyph-splitting | **confirmed** | 5 |

**Not checked:** anything under physics; anything on the sealed split (**never**);
whether a real model actually produces a loadable card for each arm — that needs
model calls and is the immediate next step; whether B's generated spans agree with
`cards/VERIFICATION.md`, which is the follow-up this chunk deliberately does not take.

### The finding, and it cost nothing

The no-source arm was instructed to write
`(no source: the principle was devised, not extracted)` into a YAML field.
**That is a scanner error** — `mapping values are not allowed here` — so every
card D was told to write would have been refused by the loader, and D would have
reported a 0% card rate for a punctuation mistake the prompt asked for. Found
while building the replay fixture. The sentinel now uses a dash, and
`test_the_no_source_sentinel_survives_a_yaml_round_trip` pins it.

### What this hands to the next chunk

- **The v2 library is empty.** No row has been generated. 44 rows x 2 calls is the
  full grid; `--limit` exists.
- **B's rows need the corpus.** All five PDFs are on this machine and
  `--dry-run` now marks any row whose PDF is missing, so a machine without them
  finds out before the first call rather than one call in.
- **The probe is now redundant with A's card call.** `generate/probe.py` turns a
  citation into a reconstructed card; A's card call does exactly that, on the live
  pipeline, and commits the result. Folding one into the other is a real
  simplification and is deliberately **not** taken here — it would change what
  chunk 33's committed diffs are diffs *of*.

---

## Outcome, part 2 — the first two v2 rows, 2026-09-08

**Generated for real.** One A row and one B row, both `navigate_to_goal`, both on
`limit_speed_dwa` — the same paper, which is what makes the pair a comparison.
Four model calls (card + node, twice), `claude-cli`, `claude-opus-5`.

| arm | card written | node | verdict |
|---|---|---|---|
| A (citation) | `dynamic_window_velocity_search` → `SelectVelocityByDynamicWindow` | `gen_a_fac8c7b94fe6b1c6` | ADMITTED on validity |
| B (paper) | `dynamic_window_velocity_selection` → `SelectDynamicWindowVelocity` | `gen_b_c2c95bcb965e342e` | ADMITTED on validity |

Both fired 600/600 ticks and drove. **Neither contribution was measured**: both
arms wrote a micro-world whose goal sits too close to a wall
(`optimal_distance()` would understate the denominator). Same failure mode in
both arms, independently — that is a generator-level problem, not an arm effect,
and it is the third and fourth instance of it.

### ⚠️ The provenance result, and it is the sharpest evidence the project has

Same paper. The only difference is whether the model could read it.

| | A — citation only | B — handed the PDF |
|---|---|---|
| span | *"Section 'The Dynamic Window Approach', subsections on admissible velocities … **Section/equation NUMBERS not asserted - not recalled with certainty.**"* | *"Sec. 4.1 Eq. (14) admissible velocities, Eq. (15) dynamic window, Eq. (16) Vr = Vs ∩ Va ∩ Vd; Sec. 4.2 Eq. (13) objective function and the target-heading definition '180 − θ'; Sec. 5.2 with Fig. 19 …; Sec. 5.4 'Parameter Settings' for α = 0.8, β = 0.1, γ = 0.1; Sec. 4 for the 0.25 s cycle time."* |
| baseline | `goal_seeker` | `reactive_avoid` |
| min_effect | 0.5 | 0.4 |

**A declined to invent section numbers, exactly as the prompt asked.** That is
the honest-refusal behaviour the provenance axis was built to reward, observed
rather than hoped for.

**Every one of B's six claimed spans checks out against the PDF**, verified
against `out/corpus/limit_speed_dwa.pdf`:

| claim | in the document? |
|---|---|
| α = 0.8, β = 0.1, γ = 0.1 | ✅ *"we found values of 0.8, 0.1, and 0.1 for … to give good results"* |
| 0.25 s cycle time | ✅ *"such a cycle is performed every 0.25 seconds"* |
| Eq. (14) = admissible velocities | ✅ *"Va = … (14) … the set of velocities which allow the robot to stop without colliding"* |
| Eq. (16) = Vr = Vs ∩ Va ∩ Vd | ✅ verbatim |
| Sec. 5.4 "Parameter Settings" | ✅ heading present |
| target heading "180 − θ" | ✅ *"It is given by 180 , , where … is the angle of the target point relative to the robot's heading"* (pdftotext mangles "− θ") |

n = 1 card, so this is a demonstration and not a rate. It is what the follow-up
chunk should measure across all five papers.

### Three couplings the run exposed, all fixed

Adding the first v2 records to the live `generate/staging/` broke three
assertions that had silently assumed one pipeline. None was a number bump.

1. `scripts/ab_compare.py` enumerated staging **unfiltered**, so it would have
   paired a v1 A node against a v2 B node and called the difference an arm
   effect. It now has `DEFAULT_PIPELINE = "v1-single-call"` — the pipeline its
   committed evidence is about — matching `pass_rate`'s rule exactly.
2. `test_every_generated_node_both_drives_and_publishes_a_port` counted 16 where
   chunk 39 recorded 14. Fixed by the same filter; the *property* still holds for
   all 16.
3. `compose/tests/test_library.py` asserted `not md.source_span` on a citation
   node — the v1 rule. Updated to the chunk-42 rule: a citation node may carry a
   **claimed** span, never `span_verified`, and `is_traceable` stays false.

Also fixed: `scripts/generate_node.py` crashed with `AttributeError` on a
successful admission whenever `result.unit` was None, which chunk 37 made the
normal case for an unmeasured contribution. It printed a traceback over a
generation that had worked. It now reports the validity verdict and the firing
rate.

Suite after all of it: **836 passed, 1 failed** — the same pre-existing
`test_no_papers.py` port-name gap.

### ⚠️ Open, and it is the user's call: the live library now MIXES pipelines

`compose.library.load_library()` is unfiltered, so the two v2 nodes joined the
v1 ones: **`lib-017-39f8b89528dc`, 17 nodes — card 6, citation 6, invented 4,
handwritten 1 — traceable 4 (0.24).**

That is correct for an append-only store (#7: nothing is compressed away) and
wrong for a *reported* number, because half the library would be one experiment
and half the other. #8 says every reported result names a library snapshot tag,
and this tag names a mixture.

**Not decided here.** The options are (a) filter the library by pipeline the way
`pass_rate` and `ab_compare` now do, (b) keep it mixed and require a reportable
sweep to name a filtered sub-snapshot, or (c) regenerate the whole grid so the
question stops existing. Committed compositions name `lib-006`/`lib-019` and
reconstruct through `keep=`, so none of the three breaks history.

---

## Outcome, part 3 — the v1 records archived, 2026-09-08

**User's decision: archive, do not delete; keep the human corpus.** The one-call
records moved to `generate/archive/v1-single-call/` — 63 staging files and 36
transcripts, **nothing removed**. 75 of them were untracked and would have been
unrecoverable.

| | before | after |
|---|---|---|
| `generate/staging/` | 69 files | **6** — the two v2 rows |
| `generate/transcripts/` | 40 | **4** — two card calls, two node calls |
| live library | `lib-017-39f8b89528dc`, 17 nodes, mixed pipelines | **`lib-003-02717879ef21`, 3 nodes** — 1 card, 1 citation, 1 handwritten |

`cards/*.yaml` untouched: the five human cards are the **input** to the card
call — A's citation and B's PDF key — so removing them would stop generation
working at all.

### The open question from part 2 is closed by this

The library no longer mixes pipelines. `load_library()` reads the live directory
alone, so the tag a reportable result names describes one experiment.

### What still reaches the archive, and why each one must

Widening resolution is not reopening the slate: `load_library()` with no
argument is still live-only. These are read paths for records that already
exist.

| consumer | why |
|---|---|
| `compose.compose.RECONSTRUCTION_STAGING` (live + archive) | all 8 committed compositions name v1 nodes; without it every one becomes `UNRECONSTRUCTABLE_LIBRARY` |
| `controllers.library_nodes()` / `make("node:…")` | `results/` commits **7 episodes** whose controller is a v1 `node:` id. A committed result that cannot name its own controller is not reproducible |
| `scripts/ab_compare.py` | chunk 39's paired table is *about* the v1 records — `STAGING` and `DEFAULT_PIPELINE` now agree |
| the `replay` fixtures in both test packages | `GOOD_PROMPT_HASH` / `VAGUE_PROMPT_HASH` are v1 transcripts |

`compose/library.py` gained `staging_dirs()`: `staging` may now name several
directories, earlier ones winning. A latent bug fell out of it —
`load_library` did `Path(staging or STAGING)`, which raised a `TypeError` from
inside pathlib that mentioned neither staging nor the tuple.

### ⚠️ The one failure that vanished is DORMANT, not fixed

The suite is now **837 passed, 0 failed** — including
`compose/tests/test_no_papers.py::test_no_card_vocabulary_reaches_the_rendered_node_block`,
which had failed since chunk 39 and which chunk 39 told later chunks not to fix.

**It did not get fixed.** It failed on the word `home`, which reaches the
rendered node block through the port name `home_bearing`. The only node
declaring that port is `b__path_integration_ants__1dd85c92eac01838`, and it is
now in the archive. The test gap — `contracts` is built from EFFECT +
PRECONDITION + TAGS + node_id and omits **port names**, while `render_node`
renders them — is exactly as it was, and it will fail again the moment a v2 node
publishes a port whose name shares a word with a card. **A green suite here is
not evidence the leak assertion is sound.**

### Not touched, and worth a decision separately

`cards/probe/` — chunk 33's five model reconstructions and their diffs. They are
model-written cards, so they arguably fall under "prune", but they are a
different experiment's committed evidence, are counted in no G1 number, and no
node was ever built from them. Left alone.
