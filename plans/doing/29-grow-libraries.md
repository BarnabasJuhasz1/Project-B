# 29 — Grow the libraries to a reportable size

**Epic:** [04](../04-epic-evaluation.md) · **Depends on:** 15, 28 · **Needs Isaac Sim:** no
**Blocks:** 31, 32 — and, through them, every G1, G2 and G3 number this cycle reports.

**Status 2026-08-30: the machinery is built; the decision and the corpus run are not.**
The ceiling is now a test rather than a paragraph, `mechanism_family` reaches the library, and
`scripts/generate_batch.py --dry-run` prints the 21-combination grid. What remains is the corpus
run (external input) and the scale decision below (the user's).

⚠️ **This chunk contains a decision the chunk cannot make.** Read "The ceiling" and
"Decide before executing" first. Executing the Build section before that decision is taken
will spend model calls against a target that is unreachable by construction.

## Why

G1 asks for **≥20 admissible nodes per condition**. The libraries today:

| Condition | Admitted | Rejected | Input |
|---|---|---|---|
| **A** | **0** | 1 (`UNIT_SCENARIO_FAILED`) | no literature |
| **B** | **1** | 0 | 5 cards |
| **D** | **0** | 1 (`UNIT_SCENARIO_FAILED`) | invented principle |

Two of the three conditions have an **empty** library. A condition with an empty library
retrieves nothing, composes nothing and scores nothing, so G2's B−D — the comparison the
project says matters most — currently has one side of it missing entirely. G3 measures
recombination over a frozen library and cannot measure recombination of one node.

This is the epic's own flag — *"the one that decides whether this cycle has a result at all"* —
turned into work.

## The ceiling, which is structural and was not known when the epic was written

`generate/generator.py:render()` is a **pure function of `(condition, task, card)`**. There is
no nonce, no sample index and no seed. `Candidate.node_id` is `gen_<condition>_<prompt_hash>`
and `Candidate.stem` is `<condition>__<card_or_task>__<prompt_hash>`, so **two calls with the
same inputs produce the same node id and the same staging stem, and the second overwrites the
first.** `backends.write_transcript()` keys on the same hash and overwrites the same way.

`TASKS` holds three families (`navigate_to_goal`, `search_for_goal`, `navigate_under_drift`).
`cards/` holds five cards. The number of *distinct* nodes reachable without changing anything:

| Condition | Distinct prompts | Ceiling (if every one is admitted) | G1 target |
|---|---|---|---|
| **A** | 3 tasks | **3** | 20 |
| **B** | 5 cards × 3 tasks | **15** | 20 |
| **D** | 3 tasks | **3** | 20 |

Observed pass rate is 1 of 2 non-fixture calls, so the *expected* yield is materially below
even that. **A and D cannot reach 20 by spending more calls.** More budget buys nothing at all
for two of the three conditions. That is the fact the scale decision has to be taken against,
and it is why this plan exists before any generation runs.

Note also that the two conditions capped at 3 are exactly the two that have admitted **zero**
nodes so far. The binding constraint is not the corpus. It is the prompt-identity design.

⚠️ **B's 15 is a count of prompts, not of mechanisms — and B's mechanism count is a labelling
decision, not a measurement.** `node_name` is a hand-typed field (`cards/schema.py:146`,
*"the leaf this card is meant to produce"*); nothing derives it from the paper. The five cards
carry three distinct values, **by design**: chunk 14's Outcome records *"five cards, three
mechanisms, deliberately with two mechanisms carried by two derivations each"*, because the
pairing tests whether the `Derivation` field carries information or merely decorates the
citation. So two of the five cards are spent on a schema experiment rather than on coverage.

Unlike A's and D's 3, **B's 3 is not structural** — it is a property of this pilot set, and any
new card is free to carry a new `node_name`. But two consequences survive:

- G1's coverage measure is **"distinct admissible primitives"**, so the node count and the
  mechanism count are different numbers and both must be reported. `CardStore.provenance()`
  now returns both. ⚠️ **`nodes/library.py` still does not** — `add()` de-duplicates on
  `node_id` and never looks at mechanism, so `len(library)` is a *node* count that silently
  overstates coverage. Carrying the family from the card through to the library entry, so
  `library.provenance()` can report mechanisms too, is work in this chunk's Build section.
- **The coverage headline is set by hand-typed names.** ✅ **Addressed 2026-08-30, ahead of
  this chunk** — see chunk 14's later amendment. `node_name` is now **1-1 with the card and
  enforced by `CardStore`**, and a separate `mechanism_family` records which cards derive one
  principle. `provenance()` reports **both** numbers: `nodes` (= sources) and `mechanisms`
  (= families, G1's coverage measure). `cards/CORPUS-BRIEF.md` §5b carries the rule that a new
  family needs a one-line justification of what it does that no existing family does, so the
  count cannot be inflated for free by splitting one principle across two names.
  Today: **5 cards, 5 nodes, 3 mechanisms.**

## The rule any fix has to obey

The three conditions differ **only** in their literature block — `generate/tests/` asserts it by
stripping the block and diffing the rest, and that assertion is what makes the independent
variable an independent variable. It follows that:

> **Any axis used to enumerate more nodes within a condition must be condition-neutral** —
> available in identical form to A, B and D.

This is what rules out the obvious fix. Cards cannot be the enumeration axis, because *having
cards is B's treatment*: enumerating over them lifts B alone and turns library size into a
second, uncontrolled difference between conditions. The same test applies to every lever below.

## The levers, and what each costs

**(i) More cards.** Lifts B only. Each verified card adds up to 3 B-nodes — but, per the
mechanism note above, adds a distinct *mechanism* only if its `node_name` is new.
`cards/CORPUS-BRIEF.md` already specifies this work — it asks a literature-search agent for
20–30 open-access PDFs against a written filter — and it has never been run. This is Week 2's
batch card extraction, unbuilt, inherited here. **Not condition-neutral**, so it cannot be the
answer to the ceiling on its own; it is still worth running, because G1's provenance fraction
and every later cycle depend on it.

**(ii) More task cells.** Condition-neutral, and the only lever that moves A and D at parity.
`TASKS` is derived from **chunk 11a's frozen contract axes** — `goal_known` × `pose_drifts` —
not from any scenario; that is stated in the source and it is why generation is not leakage.
The 2×2 has **four cells and `TASKS` names three**: the hidden-goal-under-drift combination is
unused, so one additional family is available *from the existing contract* with no new
justification needed. Beyond that fourth cell, a new family must be argued from the contract in
writing. ⚠️ **A family invented by reading the held-out split is leakage and voids G3.** The
chunk 11 procedural rule applies: authored in a session that has never had
`core/scenarios_held_out.py` in context.

**(iii) Failure signatures as queries.** Condition-neutral, and the project's own designed
second generation trigger — *"failure signatures become retrieval queries for targeted
additions"* — referenced across `compose/`, `PRE-REGISTRATION.md` and the description, and
**never built**. A signature is produced by a scored episode, so it is neutral by construction:
the same signature is handed to A, B and D. Cost: it needs the sweep to have run at least once,
so it cannot precede 31, and it is a real build rather than a parameter change.

**(iv-a) Silent resampling.** Call the same prompt *n* times and key the staging record and the
transcript by `(prompt_hash, attempt)` instead of `prompt_hash` alone. **The prompt text does not
change**, so `generate/tests/`'s block-diff assertion holds, no committed transcript is orphaned
and the independent variable is untouched. Nearly free to build. ⚠️ **But it buys identity, not
diversity**: at the same prompt the model returns the same handful of mechanisms most
represented in its training distribution, which is the very phenomenon CLAUDE.md says the corpus
is supposed to beat. Expect near-duplicate mechanisms under distinct node ids. Useful for
measuring *within-prompt variance* — a real and currently unmeasured quantity — and not useful
for coverage.

**(iv-b) A declared sample index.** Tell the model "this is attempt *k* of *n*, produce a
mechanism distinct from the others". This does buy diversity, and it **changes the prompts** —
orphaning every committed transcript, which is the exact ground on which chunk 15 rejected a
prompt change. It also changes the elicitation: asking one model for *n* distinct mechanisms is
a different measurement from asking it once, so the coverage count is no longer the quantity the
pilot measured. Recorded so the difference from (iv-a) is not lost — these are not one lever.

## Decide before executing

The options are not "how many calls", they are "what does this cycle report against G1".

**Option 1 — hit ≥20 per condition.** Requires (ii) well beyond the free fourth cell, or
(iv-b), plus (i) for B. Largest scope: a card-extraction run, several new task families or a
prompt change, and roughly 60+ generation calls at the observed pass rate. Highest risk of
landing in Week 6 with nothing run.

**Option 2 — a common ceiling, honestly reported.** Take the largest *n* every condition can
reach on equal terms — 3 today, **4 with the free fourth contract cell from (ii)** — and report
a coverage count of *n* against a target of 20, with the ceiling arithmetic above as the reason.
Preserves the ablation exactly: identical inputs per condition, #10 intact. Cheapest, and it is
a real result about the *pipeline* rather than a missing one.

**Option 3 — grow B with cards, cap A and D, and report the asymmetry.** Runs (i) only.
Produces the strongest B library and a written statement that B−A and B−D are then confounded
with library size. ⚠️ This is a **worse** confound than the one chunk 17 already accepted: a
larger library composes better for reasons that have nothing to do with the corpus, and it
would look identical in the numbers. It is exactly the condition-neutrality rule above being
broken. Recorded here to be rejected explicitly, not silently.

**Recommended: Option 2, taking the free fourth cell, with (i) run anyway** — the corpus work in
`CORPUS-BRIEF.md` is independently valuable (G1's provenance fraction, and every later cycle),
it is sim-free, and it can proceed while the coverage decision is settled. Report the count
every condition can reach on equal terms, per node **and** per distinct mechanism.

Whichever is chosen becomes a **dated append to `PRE-REGISTRATION.md`** — G1's reported target
is pre-registration material and `prereg-v1` is now tagged (`a84cd46`), so it is an append and
never an edit.

## Build

Ordered so that everything before the decision point is useful under all three options.

- ✅ **Re-measure the ceiling as a test, not as prose.** `generate/tests/test_ceiling.py`
  asserts the distinct-prompt count per condition (a=3, b=15, d=3) against a stated G1 target
  of 20, that no two inputs collide on one prompt, and that a condition added to `CONDITIONS`
  without being costed fails. `cards/tests/` pins the card side: 5 cards / 5 nodes /
  3 mechanisms, and a shared `node_name` is rejected at load.
- ✅ **Carry `mechanism_family` from the card into the library entry.**
  `NodeMetadata.mechanism_family` (defaulting to the node id via `.family`), populated at
  admission from `Card.family`, and `Library.provenance()` now reports `mechanism_count` and
  `by_mechanism` alongside `node_count`. Deliberately **not** in `LibraryEntry.identity()`, so
  the snapshot tag is unchanged — committed results name `lib-006-23c57a77269e` and it still
  reproduces.
- ✅ **A batch driver** — `generate/batch.py` + `scripts/generate_batch.py`, split the way
  `runs/sweep.py` and `scripts/run_sweep.py` are, so the grid is testable without a CLI.
  `plan()` is a pure function of the frozen inputs; `--dry-run` is the real path minus the
  model call and prints all 21 combinations with what is already staged. Resumes from
  `generate/staging/` and **never from a progress file** (chunk 28's defect), **never retries a
  rejection** (a pass rate needs a denominator), backend defaults to `replay`, `--limit` is the
  budget lever, and a stem collision is refused before the first call. A row that raises is
  recorded and the rest of the grid still runs.
- ⬜ **NEXT, and the first item that needs a decision or an external input.**
  **The corpus run** — execute `cards/CORPUS-BRIEF.md` as written, verify every span against
  the PDF actually on disk with `scripts/testing/fetch_and_verify.py`, and update
  `cards/VERIFICATION.md`. An unverified span is not counted in G1's numerator. Prefer cards
  whose `node_name` is **new**, since a duplicate mechanism adds nothing to the coverage count.
- **Then, and only then, the option chosen above.**
- **`nodes/library.py:provenance()` reports the coverage count per condition**, as both a node
  count and a **distinct-mechanism count**, so G1's number comes from the store rather than
  from a paragraph.

## Done when

- `python scripts/generate_node.py --pass-rate` reports a non-zero admitted count for **every**
  condition. A and D at zero is the state this chunk exists to end.
- ✅ The distinct-prompt ceiling and the distinct-mechanism count are asserted by tests, and
  the tests name the current numbers.
- ✅ `scripts/generate_batch.py --dry-run` prints the full combination set and what it would
  skip, making no model call.
- Every card in `cards/` is either verified in `cards/VERIFICATION.md` or explicitly listed
  there as outstanding with a reason.
- The chosen coverage target is a dated append in `PRE-REGISTRATION.md`.
- `snapshot()` gives a library tag that a reportable sweep spec can name (#8), and the full
  sim-free suite stays green.

## Not in scope

Running the evaluation (31). Condition C (30). G3 (32). The citations probe (33) — it shares
this chunk's machinery but answers a different question and must not be allowed to grow into a
fifth G2 condition here. Anything needing the container or a GPU. Verifying that a node
**implements its card** — chunk 15 finding 1 established that the harness cannot, and
`PRE-REGISTRATION.md` §8 already absorbs it as a stated limitation; this chunk does not reopen
it.

## Open questions for the decision

1. Is a coverage count below 20 acceptable for the write-up, given the ceiling is structural
   and demonstrable rather than a budget shortfall?
2. Take the free fourth contract cell (hidden goal under drift) as a task family? It is the one
   condition-neutral gain available with no new justification and no prompt change.
3. Should within-prompt variance be measured at all via (iv-a), given it is nearly free and
   currently unknown — or does an unused measurement not earn its model calls?
