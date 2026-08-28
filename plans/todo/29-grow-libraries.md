# 29 — Grow the libraries to a reportable size

**Epic:** [04](../04-epic-evaluation.md) · **Depends on:** 15, 28 · **Needs Isaac Sim:** no
**Blocks:** 31, 32 — and, through them, every G1, G2 and G3 number this cycle reports.

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
first.**

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

## The three ways the ceiling lifts, and what each costs

**(i) More cards.** Lifts B only, linearly: each verified card adds up to 3 B-nodes. Twenty
B-nodes needs ~7 cards; a comfortable margin at the observed pass rate needs ~12–15.
`cards/CORPUS-BRIEF.md` already specifies this work — it asks a literature-search agent for
20–30 open-access PDFs against a written filter — and it has never been run. This is Week 2's
batch card extraction, unbuilt, inherited here. **It does nothing for A or D**, so on its own
it widens the gap between conditions rather than closing it.

**(ii) More task families.** Lifts all three conditions equally, and is the only lever that
moves A and D. `TASKS` is derived from **chunk 11a's frozen contract axes**, not from any
scenario — that is stated in the source and it is why generation is not leakage. Adding a
family means deriving a new one from the same axes. ⚠️ **A family invented by reading the
held-out split is leakage and voids G3.** Any family added here must be justified from the
contract in writing, and the chunk 11 procedural rule applies: authored in a session that has
never had `core/scenarios_held_out.py` in context.

**(iii) A declared sample index in the prompt.** Add an explicit "this is attempt *k* of *n*,
produce a mechanism distinct from the others" to the prompt, making *k* part of the hash and
therefore part of the identity. This is the only lever that scales A and D **without** claiming
new task families. Two costs, and they are not small: it **changes the prompts**, which
`generate/tests/` asserts differ only inside the literature block and which would orphan every
committed transcript (chunk 15 rejected a prompt change on exactly this ground); and asking one
model for *n* distinct mechanisms is a different elicitation from asking it once, so the
coverage count it produces is not the same quantity the pilot measured.

## Decide before executing

The options are not "how many calls", they are "what does this cycle report against G1".

**Option 1 — hit ≥20 per condition.** Requires (ii) or (iii), plus (i) for B. Largest scope:
a card-extraction run, new task families or a prompt change, and roughly 60+ generation calls
at the observed pass rate. Highest risk of landing in Week 6 with nothing run.

**Option 2 — a common ceiling, honestly reported.** Take the largest *n* every condition can
reach on equal terms — today 3, or higher with (ii) restricted to what A and D can match —
and report a coverage count of *n* against a target of 20, with the ceiling arithmetic above
as the reason. Preserves the ablation exactly: identical inputs per condition, #10 intact.
Cheapest, and it is a real result about the *pipeline* rather than a missing one.

**Option 3 — grow B with cards, cap A and D, and report the asymmetry.** Runs (i) only.
Produces the strongest B library and a written statement that B−A and B−D are then confounded
with library size. ⚠️ This is a **worse** confound than the one chunk 17 already accepted: a
larger library composes better for reasons that have nothing to do with the corpus, and it
would look identical in the numbers. Recorded here to be rejected explicitly, not silently.

**Recommended: Option 2, with (i) run anyway** — the corpus work in `CORPUS-BRIEF.md` is
independently valuable (G1's provenance fraction, and every later cycle), it is sim-free, and
it can proceed while the coverage decision is settled. Report the count every condition can
reach on equal terms.

Whichever is chosen becomes a **dated append to `PRE-REGISTRATION.md`** — G1's reported target
is pre-registration material and `prereg-v1` is now tagged (`a84cd46`), so it is an append and
never an edit.

## Build

Ordered so that everything before the decision point is useful under all three options.

- **Re-measure the ceiling as a test, not as prose.** `generate/tests/` asserts the number of
  distinct prompts per condition, so the arithmetic above cannot silently rot when `TASKS` or
  `cards/` changes. This is the cheapest item and it belongs first.
- **A batch driver** — `scripts/generate_batch.py`. Enumerates (condition, card, task),
  skips what is already staged, calls the generator once per combination, records every
  outcome, and **never retries a rejection** (chunk 28's argument: retrying until it works has
  no denominator). Resumable from `generate/staging/`, the same way the sweep resumes from
  `results/`. Refuses to run a combination whose stem already exists unless `--force`, so the
  overwrite hazard above becomes an error rather than a silent loss.
- **The corpus run** — execute `cards/CORPUS-BRIEF.md` as written, verify every span against
  the PDF actually on disk with `scripts/testing/fetch_and_verify.py`, and update
  `cards/VERIFICATION.md`. An unverified span is not counted in G1's numerator.
- **Then, and only then, the option chosen above.**
- **`nodes/library.py:provenance()` reports the coverage count per condition**, so G1's number
  comes from the store rather than from a paragraph.

## Done when

- `python scripts/generate_node.py --pass-rate` reports a non-zero admitted count for **every**
  condition. A and D at zero is the state this chunk exists to end.
- The distinct-prompt ceiling is asserted by a test, and the test names the current numbers.
- `scripts/generate_batch.py --dry-run` prints the full combination set and what it would skip,
  making no model call.
- Every card in `cards/` is either verified in `cards/VERIFICATION.md` or explicitly listed
  there as outstanding with a reason.
- The chosen coverage target is a dated append in `PRE-REGISTRATION.md`.
- `snapshot()` gives a library tag that a reportable sweep spec can name (#8), and the full
  sim-free suite stays green.

## Not in scope

Running the evaluation (31). Condition C (30). G3 (32). Anything needing the container or a
GPU. Verifying that a node **implements its card** — chunk 15 finding 1 established that the
harness cannot, and `PRE-REGISTRATION.md` §8 already absorbs it as a stated limitation; this
chunk does not reopen it.

## Open questions for the decision

1. Is a coverage count below 20 acceptable for the write-up, given the ceiling is structural
   and demonstrable rather than a budget shortfall?
2. If task families are added (ii), who authors them, and in a session that has never seen the
   sealed split?
3. Is a prompt change (iii) worth orphaning every committed transcript, so soon after chunk 15
   rejected a prompt change for that reason?
