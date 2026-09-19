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

---

## Decided 2026-08-31 — Option 2, and the fourth cell is taken

**Open question 1 — answered: yes.** A coverage count below 20 is what this cycle reports,
because the ceiling is structural (`render()` is a pure function of `(condition, task, card)` and
the node id IS the prompt hash) rather than a budget shortfall. Recorded as a dated append to
`PRE-REGISTRATION.md`, which is where G1's reported target now lives. Options 1 and 3 were
rejected there with reasons.

**Open question 2 — answered: yes, take the fourth cell.** `goal_hidden × drift` (search under
drift) is the unused cell of 11a's frozen 2×2. It is condition-neutral, needs no prompt change and
orphans no transcript, and it moves the common ceiling every condition reaches on equal terms:

| | today | with the fourth cell |
|---|---|---|
| A | 3 | **4** |
| D | 3 | **4** |
| B | 15 | **20** |

⚠️ **NOT AUTHORED YET, AND DELIBERATELY NOT IN THIS SESSION.** Chunk 11's procedural rule
applies: a task family invented by reading the held-out split is leakage and voids G3, so the
family must be written in a session that has **never had `core/scenarios_held_out.py` in
context**. The session that took this decision hashed that file and grepped for its hash but never
opened it — probably clean, and "probably" is exactly what the rule exists to rule out. **Author
the fourth family in a fresh session**, then update `generate/tests/test_ceiling.py`'s numbers
from (3, 15, 3) to (4, 20, 4) in the same change.

**Open question 3 — still open.** Within-prompt variance via silent resampling (lever iv-a) is
nearly free and currently unmeasured, but it buys identity rather than diversity. Not blocking.

**Status: this chunk is PAUSED at the user's direction, 2026-08-31**, along with the corpus run
and everything else in the literature-ingestion track, in favour of finishing epic 03's SLAM
pipeline. The corpus run remains the last blocker on the library, and therefore on 31 and 32.

---

## Executed 2026-09-05 — open question 2 is done, in a clean session

Open question 2 was **executed**, not re-decided. The fourth cell of chunk 11a's frozen 2x2 —
hidden goal **and** drifting pose — is now `generate/generator.py:TASKS["search_under_drift"]`,
and `generate/tests/test_ceiling.py`'s data block moved from (3, 15, 3) to **(4, 20, 4)** in the
same change, as the 2026-08-31 decision required.

**The procedural rule was honoured.** The family was authored in a session that had never had
`core/scenarios_held_out.py` in context: the file was never opened, no unscoped `grep -r` was run
across the repo, and the only inputs to the wording were `core/observation.py`'s field semantics
and `core/scenario.py`'s `goal_known` / `drift` / `disclosed_at()` contract. No scenario list was
consulted for wording or naming; the name comes from the two axes and the existing convention.

**What the description says, and why it is not the two paragraphs concatenated.** The entry states
what the controller sees — `obs.goal` is None until it latches on line-of-sight detection,
`obs.pose_estimate` takes a seeded random walk, `obs.pose_error_estimate` reports the believed
error, the scan is unaffected — and then names the property that belongs to neither cell alone:
*covering ground that has not been covered requires a record of where the robot has already been,
and that record can only be kept in the drifting frame, so it decays while it is being written.*
Coverage under drift is not search plus drift; it is search whose own memory is the thing
degrading.

**The ceiling moved exactly as costed**, and `--dry-run` confirms it against the real grid:

| | before | after |
|---|---|---|
| A | 3 | **4** |
| D | 3 | **4** |
| B | 15 | **20** |
| grid rows | 21 | **28** |

**No transcript was orphaned.** Every prompt hash present in the 21-row grid is byte-identical in
the 28-row grid — the new family only adds rows. That was the condition under which the fourth
cell was preferred over lever (iv-b), and it held.

---

## Outcome — 2026-09-05/06

Two things were done: the fourth task family was authored (open question 2, above), and the
generation batch was run for the two conditions that had **zero** admitted nodes. **Condition A
is no longer empty.** Condition D still is.

### What differed from the plan

- **The plan costed the ceiling but not the blast radius of hitting it.** Admitting one node
  changed the library snapshot tag from `lib-006-23c57a77269e` to `lib-007-b3669417fd86`, and
  that **broke all five committed compositions** — `compose/tests/test_compose.py` went from
  green to 5 failures, four `SNAPSHOT_MISMATCH` and one "no transcript for prompt
  35dffdc96c85a735". The guard is behaving exactly as designed (#7 append-only, #8 every result
  names its snapshot); what was not anticipated is that **growing the library invalidates every
  composition recorded against the smaller one**, including the committed works/does-not-work
  pair that chunk 16's finding rests on. Left failing and reported rather than patched: the fix
  is either ~5 re-composition calls against `lib-007`, or teaching the tests to reconstruct the
  historical library, and that is a scope decision. ⚠️ **This is a standing cost of every future
  admission, not a one-off**, and it lands again on each node the corpus run adds.
- **`UNIT_SCENARIO_FAILED` conflates two unrelated failures.** Two of D's three new rejections
  never tested a node at all — the harness refused the generator's own micro-world
  (`goal (8.9, 2.5) is unreachable from (0.9, 2.5)`; `goal (8.0, 5.0) is too close to a wall for
  the whole GOAL_RADIUS disc to be reachable`). They are recorded under the same reason code as a
  node that ran and did not work. **G1's pass rate therefore currently mixes "the node failed"
  with "the generator wrote an invalid world"**, and the second is a statement about the
  generator, not about the condition. Distinguishable today only in the `detail` string. Not
  changed here — altering a reason code rewrites recorded outcomes — but it should be a
  separate code before G1 is reported.
- **The fourth cell paid off in the way the plan hoped, and not in the way it predicted.** It
  did not yield an admitted node in either condition. It did produce the only condition-A
  candidate that was not wall-following (see below), which is the coverage argument rather than
  the count argument.
- The admitted A node `BoundaryFollowToGoal` **declares two OUT ports while being an Action that
  drives** — the same violation `compose/FINDINGS.md` records for the one existing admitted node.
  So it is a recurring generator behaviour, not a one-off, and the follow-up chunk that enforces
  "an Action that drives may not also publish a port" will invalidate two nodes, not one.

### What was discovered

**Condition A returns one mechanism almost regardless of the task cell.** Across A's four
candidates: `FollowBoundaryToGoal`, `BoundaryFollowToGoal`, `FollowWallToExplore` — three names
for boundary/wall-following — and one exception, `FrontierSeekDriftDiscounted`, from the new
`search_under_drift` cell. **Two distinct mechanisms proposed across four cells.** This is
CLAUDE.md's coverage claim visible inside condition A's own output: a prompted model returns the
handful of mechanisms most represented in its training distribution. It is evidence for the
coverage argument that does not depend on B beating anything.

**Condition D behaves oppositely and still admits nothing.** D's four candidates are four
distinct mechanisms — `CommittedDetour`, `UncertaintyScaledGoalSweep`, `CommitToExplorationBearing`,
`TargetPosteriorSearch`. Told to invent a principle, the model diversifies; told nothing, it
converges on wall-following. **D proposes more distinct mechanisms than A and admits fewer
(0 vs 1)**, which is worth stating plainly in the write-up: novelty of proposal and admissibility
are not the same axis, and G1 counts the second.

### The counts, reported separately

**Nodes admitted, per condition** (the library is materialised from `generate/staging/`):

| Condition | Admitted | Judged | Pass rate | Reachable ceiling |
|---|---|---|---|---|
| **A** | **1** | 4 | 0.25 | 4 |
| **B** | 1 | 1 | 1.00 | 20 (19 cells unrun) |
| **D** | **0** | 4 | 0.00 | 4 |

**Distinct mechanisms, which is G1's actual coverage measure — a different number:**

| Condition | Mechanisms admitted | Mechanisms proposed |
|---|---|---|
| **A** | **1** (boundary-following) | 2 across 4 cells |
| **B** | **1** (`LimitSpeedToVisibleRange`) | 1 of 5 card families run |
| **D** | **0** | 4 across 4 cells |

⚠️ `Library.provenance()` reports `mechanism_count: 3` for the materialised library, and that
number is **not** a coverage count. A node with no card gets its `mechanism_family` defaulted to
its own node id, so every card-less node is its own family by construction and A's and D's
mechanism counts can only be read off the node bodies by hand. This is the same defect the plan
already flagged for `len(library)`, surviving one level up. **Do not report
`provenance()["mechanism_count"]` as G1's coverage number for A or D.**

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| Full sim-free suite before the batch | 736 passed, 252 s | confirmed green |
| Full sim-free suite after the batch | 731 passed, **5 failed**, 241 s | regression confirmed, cause isolated below |
| Is the regression caused by the fourth task family or by the admission? | Rebuilt the library from a staging tree with only the new A node excluded: tag returns to `lib-006-23c57a77269e` exactly | **the admission, not the task family**; the fourth cell is exonerated. n=1, but it is a deterministic hash |
| `test_batch.py` needs no count edit | Lines 41-43 derive from `len(TASKS)`; read, not assumed | confirmed |
| Does the fourth cell orphan any transcript? | All 21 pre-existing prompt hashes are byte-identical in the 28-row grid | ruled out — this was the condition for preferring cell 4 over lever (iv-b) |
| Grid size after the change | `--dry-run`: 28 rows, **3 skipped** — not the 6 predicted | the other three staged records are two `vague_adaptive_motion` fixture runs (not in the card store, so not grid rows) and one superseded `limit_speed_dwa` hash; `--pass-rate` lists both exclusions explicitly |
| Leakage: was the held-out split ever in context? | `core/scenarios_held_out.py`, `core/tests/test_held_out.py`, `plans/done/11-held-out-split.md` and PRE-REGISTRATION §4 never opened; no unscoped `grep -r` run; wording sourced only from `core/observation.py` and `core/scenario.py` | chunk 11's procedural rule honoured |
| Is the pass rate stable? | **Not checked, and not checkable here.** n=1 per cell, 7 judged calls total | inconclusive **by construction** — FINDINGS #3 says the same prompt gave a refusal on one call and an admission on the next. A's 0.25 and D's 0.00 are not estimates of anything |
| Card fidelity of any admitted node | **Not checked.** FINDINGS #1 establishes the harness cannot; PRE-REGISTRATION §8 absorbs it | out of scope, unchanged |
| Behaviour under physics | **Not checked.** Every result here is a kinematic micro-world; an admitted node may still fail under physics | outstanding |
| Import allowlist | Not weakened, not extended; no candidate was made to pass | confirmed |

### Still open after this chunk

- **D remains at zero.** The chunk's own "Done when" — a non-zero admitted count for *every*
  condition — is **not met**. A and B are non-zero; D is not, after 4 judged calls.
- B's 19 unrun cells (~19 calls) were deliberately not spent: the user asked to see A and D's
  pass rate first.
- The corpus run (`cards/CORPUS-BRIEF.md`) is still the last blocker on B's library.
- The five broken compositions need a decision (re-compose vs. reconstruct).

---

## Superseded in part, 2026-09-06 — the condition restructure moves A's ceiling

This chunk's central arithmetic was **"A and D cannot reach 20 by spending more calls"**. Half of
it no longer holds. Condition A was redefined on 2026-09-06: it is handed each card's **citation**
and none of its content, so it enumerates over the store exactly as B does and its ceiling is
**20**, not 4. See `PRE-REGISTRATION.md`, dated append 2026-09-06.

| | this chunk (2026-09-05) | after the restructure |
|---|---|---|
| A | 4 | **20** |
| B | 20 | 20 |
| D | 4 | 4 |
| grid rows | 28 | **44** |

**What survives.** The ceiling finding itself is intact and is now sharper: it was never about
budget, it was about `render()` being a pure function of `(condition, task, card)`. A escaped it
by acquiring a corpus axis, which is exactly what the arithmetic predicted would be needed.
**D is the one arm still capped**, and it is the arm this chunk's "Done when" is still blocked on.

**What is withdrawn.** The 2026-09-05 Outcome above reports A at 1 admitted / 4 judged, pass rate
0.25, one mechanism. **Those four records are retired**: they were produced by prompts that no
longer exist, they carry a `retired_reason`, and `pass_rate` reports them under `_retired` rather
than counting them. A's admitted count under the current definition is **0 of 0** — no call has
been made. The observation they support — that the naive arm returned wall-following in three of
four task families — is still a real finding **about the retired naive arm**, and should be
reported as that, never as a property of condition A.

**One thing the retirement fixed by itself.** The Outcome above records that admitting a node
broke all five committed compositions by moving the library tag to `lib-007-b3669417fd86`.
Retiring that node returns the library to **`lib-006-23c57a77269e`** and the five tests pass
again. The underlying cost is unchanged and still stands: **the next admission will break them
again**, and that decision is still open.

---

## The citation arm's first run — 2026-09-06

Five real `claude-cli` calls, condition A under its new definition, one task family
(`navigate_to_goal`) against all five citations. **3 admitted, 1 rejected, 1 backend timeout.**

| citation | node | outcome |
|---|---|---|
| `limit_speed_dwa` | `DynamicWindowDrive` | rejected, `UNIT_SCENARIO_FAILED`, effect 0.000 |
| `limit_speed_ics` | — | **failed**: `BackendError: generation timed out after 600s` |
| `path_integration_ants` | `PathIntegrationHoming` | **ADMITTED**, effect 1.000 |
| `path_integration_mammals` | `PathIntegrationGoalVector` | **ADMITTED**, effect 1.000 |
| `reorient_geometry` | `ExtendedSurfaceDetour` | **ADMITTED**, effect 1.000 |

A's pass rate is **3/4 = 0.75** on judged rows. The timeout is not a rejection — it staged
nothing, so the row is correctly PENDING and may be re-run without violating "never retry a
rejection". ⚠️ n = 1 per cell; FINDINGS #3 still applies.

**The model recognised every reference it was given, and said so unprompted.** All four
recognitions were checked by hand against the real papers and are accurate: Müller & Wehner
(1988) as the *Cataglyphis fortis* two-leg channel study; Etienne & Jeffery (2004) as the
mammalian dead-reckoning review; Cheng (1986) as the rectangular-arena geometric module; Fox,
Burgard & Thrun (1997) as velocity-space search with the dynamic window and the admissibility
bound.

**This is by design, and it is what makes the pilot corpus a control rather than a weakness.**
Confirmed by the user 2026-09-06. The five pilot cards are deliberately drawn from work well
inside the training distribution, which holds recall constant and isolates one variable:

> On a corpus the model already knows, **B−A is what EXTRACTION buys** — the derivation step,
> not the knowledge.

The `limit_speed_dwa` pair is exactly that result in miniature: A reconstructed the paper's real
algorithm faithfully and was rejected; B, given the card's Derivation turning the same finding
into a rule for a Go2 with a 181-beam scan, was admitted. A had the knowledge and still failed.

**A second corpus of unseen, post-cutoff papers is planned** and is what tests the *novelty*
claim — CLAUDE.md's first claim, the one this cycle cannot address. See
`plans/todo/35-unseen-corpus.md`. The two stages measure different things and the write-up must
not let one stand in for the other:

| corpus | holds constant | measures |
|---|---|---|
| pilot (in-distribution) | the model knows the paper | **extraction** — what the derivation step buys |
| unseen (post-cutoff) | the model cannot know the paper | **novelty** — what a corpus supplies that the model cannot |

⚠️ The citation arm is what makes this measurable at all: A's transcripts record whether the
model claims recognition, so "in-distribution" stops being an assumption about the corpus and
becomes an observation with a per-card answer.

**The prompt's anti-confabulation clause worked.** The DWA response separated recall from
invention without being asked twice: *"What I do not recall from the paper are its parameter
values, so every constant below is my own engineering judgement"*, then named its two additions
and one approximation. That is the provenance discipline the project's third claim is about, and
it is now visible in a transcript rather than assumed.

### Two defects found and fixed by this run

1. **The staging stem was computed twice and the copies disagreed.** `Candidate.write()` built
   `condition__{card_id or task}__hash` while `Candidate.stem` built
   `condition__{card_id or citation_id or task}__hash`, so A's module landed under the TASK and
   its verdict under the CITATION. A split pair reads as PENDING to `batch.plan()` (which tests
   for the `.json`) while already carrying a verdict — **so the row would be generated again**,
   which is exactly the retry that leaves G1's pass rate without a denominator. `write()` now
   uses `self.stem`; a test pins all three shapes. The eight files this run mis-named were
   renamed, and no content was affected — the sidecars were correct throughout.
2. **`compose/library.py` rebuilt every citation node as `Origin.INVENTED`.** It read only
   `card_id` when re-deriving metadata, so `by_origin` reported `invented: 3` for nodes that were
   pointed at papers. Since `by_origin` is where G1's provenance numbers are read off, the loss
   would have been silent and reportable. Now `card: 1, citation: 3, handwritten: 1`, pinned by a
   test. **A third, related gap was closed at the same time**: `pass_rate`'s fixture-exclusion
   rule also keyed on `card_id` alone, so an A candidate generated from a test-fixture card would
   have entered G1's numbers invisibly.

### Still open

**The five committed compositions are broken again**, and this time it is not undoable by a
retirement: the library legitimately grew from 6 to 9 entries, `lib-006-23c57a77269e` →
`lib-009-afacf77bb805`, and `compose/tests/test_compose.py` fails 5 tests with
`SNAPSHOT_MISMATCH`. This is the standing cost recorded in the 2026-09-05 Outcome, now paid.

**A cheap fix exists and needs no model calls.** Every composition record already carries
`retrieval.considered` — the full list of node ids in the library at composition time — and
`retrieval.library_size`. So the historical library is exactly reconstructible from the committed
record, and a composition could stay reproducible for the life of the project rather than until
the next admission. The alternative is re-composing all five against `lib-009` (~5 calls, and it
discards the committed works/does-not-work pair chunk 16's finding rests on). **Recommended:
reconstruct. Not implemented — it changes what "reproducing a composition" means and that is the
user's call.**

---

## The malformed-candidate leak, and the `navigate_to_goal` family completed — 2026-09-06

Two things, one session: a defect that was quietly shrinking G1's denominator was fixed, and
the last unpaired cell in the `navigate_to_goal` family was generated in both arms.

### `CANDIDATE_RAISED` — a candidate's own fault is now always counted

`b/navigate_to_goal/path_integration_ants` (prompt hash `1dd85c92eac01838`) died with
`AttributeError: 'float' object has no attribute 'sigma_xy'`. The cause is the candidate's:
its `micro()` probes for a drift keyword and hands `MicroScenario` a **bare float**,
`drift=0.6`, where a `DriftSpec` belongs.

```python
for kw in ("pose_drift", "drift", "pose_noise"):
    try:
        return MicroScenario(**spec, **{kw: 0.6})
    except TypeError:
        continue
```

Nothing in `admit()` could see it. `MicroScenario.drift` is typed `object`, so construction
succeeds; `validate()` does not type it, so validation passes; the world dies five frames down
in `core.drift.DriftModel.__init__`, inside `run_unit`'s **baseline** aggregation.

**The cost was not a crash.** `admit()` wrapped its body in `try/except Rejected`, so anything
else propagated to `batch.run()`'s generic handler, which counts the row `failed` and writes no
`.result.json`. The candidate was generated, never judged, and **absent from the pass rate
entirely** — while `batch.plan()` reported the row `staged, unjudged`, which `pending()` skips.
So the row could neither be counted nor re-run.

Fixed with a catch-all in `admit()` under the reason `CANDIDATE_RAISED`, carrying the **full
traceback** in `detail`. The docstring already promised "never raises for a candidate's own
faults"; the code now honours it. Pinned by four tests in `generate/tests/test_admit.py`, one
of which reproduces the exact `drift=0.6` shape on a handwritten fixture.

**What was deliberately NOT done**, and why:

- **The harness was not taught to tolerate a float.** The candidate is wrong. Counting it is
  the fix; coercing it would delete the finding.
- **`MicroScenario.validate()` did not grow a `drift` type check.** It would have caught this
  one fault — and routed it into `run_unit`'s "malformed micro-world" path, which surfaces as
  `UNIT_SCENARIO_FAILED`. That reason code is *already* over-loaded (this chunk's 2026-09-05
  Outcome records it conflating "the node failed" with "the generator wrote an invalid world"),
  so the check would have made attribution worse in exchange for covering one case of a general
  defect. `CANDIDATE_RAISED` is general and is its own code.
- **The import allowlist was not widened**, and no candidate was made to pass.

⚠️ **The honest caveat, recorded in the code.** This clause cannot distinguish a candidate's
fault from a **harness bug raised while judging one**, and it will record the second as the
first. That is why the traceback is kept rather than the message alone: a run of
`CANDIDATE_RAISED` rejections all pointing at the same harness frame is a bug report, not a
pass rate, and only the traceback makes that visible.

**The row was judged, not regenerated.** The `.py` and `.json` were already on disk — only the
verdict was missing — so `admit_staged()` was run against the existing artefact. Zero model
calls, and it preserves the candidate that demonstrates the bug. Regenerating was rejected on
two grounds: `generate_batch.py` **skips** the row as already staged (verified by `--dry-run`),
so it would have needed `--force`, and `--force` overwrites the staged `.py` at the same prompt
hash — destroying the evidence and re-rolling a judgement, which is the retry the denominator
rule forbids.

**It cost B a point, which is the point.** B's pass rate went from 2/3 = 0.67 to 2/4 = 0.50 the
moment the candidate re-entered the denominator it had been silently leaving.

### The citation arm's second run: `limit_speed_ics`, both arms, both admitted

The cell A's 600 s backend timeout left empty on 2026-09-06. The timeout staged nothing, so both
rows were genuinely PENDING and neither call is a retried rejection.

| condition | node | outcome | scored against |
|---|---|---|---|
| **A** | `ICSFilteredGoalDrive` | **ADMITTED**, `goal_reached` 1.0 vs 0.0, effect 1.000 | its own PREDICTION |
| **B** | `LimitSpeedToOcclusionBoundary` | **ADMITTED**, `min_clearance` 0.289 vs −0.118, effect 3.462 | the CARD's prediction |

⚠️ **This pair is not an effect-size comparison and must not be read as one.** A states its own
claim by design — handing it the card's metric, baseline and minimum effect would give it the
extracted content through the back door — so the two nodes cleared *different bars on different
metrics*. What the pair supports is the coverage and provenance axes, not B−A on effect.

**Both are untraceable, for different reasons.** A is `Origin.CITATION`, which can never be
traceable. B is `Origin.CARD` and untraceable because `limit_speed_ics` is the card
`cards/VERIFICATION.md` leaves **deliberately unverified** — the PDF on disk is the IROS 2003
conference paper and the card cites the *Advanced Robotics* 18(10) 2004 journal version. The
honest state, and it is enforced rather than conventional.

**A recognised the reference — five for five — and separated recall from invention unprompted.**

> "I recognise this reference. Fraichard & Asama's ICS paper defines a state as an *inevitable
> collision state* when **no** future control trajectory, over an infinite horizon, avoids
> collision — collision-free is not the same as safe."

The recall was checked by hand and is accurate, including the conservative approximation (ICS
computed w.r.t. a subset of controls is a superset of the true ICS). It then named its own
additions — "Numbers that are **my engineering judgement, not from the paper**" — and, more
sharply than the DWA transcript did, attributed one conservatism to a *different* literature:
"One extra conservatism, which belongs to the later limited-FOV line of work rather than to the
2004 paper". That is the provenance discipline visible in a transcript rather than assumed, and
it is a stronger instance than any recorded so far.

### The counts, reported separately

**Nodes admitted per condition** (`generate/staging/`, retired and fixture records excluded):

| Condition | Admitted | Judged | Pass rate | was |
|---|---|---|---|---|
| **A** | 4 | 5 | **0.80** | 3/4 = 0.75 |
| **B** | 3 | 5 | **0.60** | 2/3 = 0.67 |
| **D** | 0 | 4 | 0.00 | unchanged |

**Distinct mechanisms — G1's actual coverage measure, counted BY HAND.** ⚠️
`provenance()["mechanism_count"]` reports **11** for the materialised library and that is not a
coverage number: a card-less node defaults its `mechanism_family` to its own node id, so every
A node is its own family by construction.

| Condition | Nodes | Distinct mechanisms | read off as |
|---|---|---|---|
| **A** | 4 | **3** | ICS/speed-limiting, path integration (×2 nodes), geometric reorientation |
| **B** | 3 | **2** | `LimitSpeedToVisibleRange` (×2 cards), `ReorientToGeometry` |

B's collapse from 3 nodes to 2 families is `mechanism_family` **working as designed**: two cards
deriving one principle (`limit_speed_dwa`, `limit_speed_ics`) share a family, and G1 counts
families. It is the first time that rule has actually bound on real data.

A's 4 nodes give 3 mechanisms — and none of them is wall-following. **The retired naive arm
returned boundary-following in three of four task families; the citation arm has not returned it
once in five.** Pinning the model to a reference appears to be what breaks the monoculture. n=1
per cell and FINDINGS #3 still applies, so this is an observation, not an estimate.

**The `navigate_to_goal` family is now complete in both arms — 5 of 5 cells paired**, 5 of 20
overall:

| card | A | B |
|---|---|---|
| `limit_speed_dwa` | rejected / `UNIT_SCENARIO_FAILED` | **ADMITTED** |
| `limit_speed_ics` | **ADMITTED** | **ADMITTED** |
| `path_integration_ants` | **ADMITTED** | rejected / `CANDIDATE_RAISED` |
| `path_integration_mammals` | **ADMITTED** | rejected / `UNIT_SCENARIO_FAILED` |
| `reorient_geometry` | **ADMITTED** | **ADMITTED** |

Library `lib-010-4d2ec71816f3` → **`lib-012-9a766931282c`** (12 entries with toys; by origin:
card 3, citation 4, handwritten 5).

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| Reproduce the `sigma_xy` failure | `admit_staged` on the staged candidate raised `AttributeError` at `core/drift.py:55`, via `run_unit` → `_aggregate` → `rollout` → `make_localiser` | confirmed, exact frame identified |
| Is the fault the candidate's or the harness's? | `micro()` passes `drift=0.6`, a float, to a field typed `object` | **the candidate's**; harness left unchanged |
| Does the fix record it? | `CANDIDATE_RAISED`, detail carries message + full traceback | confirmed |
| Does the row now reach G1's denominator? | B 2/3 = 0.67 → 2/4 = 0.50 on the same staging tree | confirmed — the number moved because the fix works |
| Would the re-run command have run anything? | `--dry-run` on that row: `staged, unjudged`, **skipped**; 1 skipped, 3 to run | **no** — regenerating needed `--force`, which overwrites the evidence |
| `generate/tests/test_admit.py` | 41 passed | confirmed |
| Full sim-free suite, before the fix | 741 passed, 5 failed (as briefed) | baseline |
| Full sim-free suite, after the fix | 744 passed, **6 failed** — a 6th failure not in the brief | see below; **not caused by this work** |
| Is the 6th failure mine? | `compose/tests/test_no_papers.py` fails **identically** with the new `.result.json` moved aside | **ruled out** — pre-existing |
| What causes it? | `gen_b_4dc13c6a972bb06b` declares ports `reorient_axis` / `reorienting`; `render_node` renders the ports line, but the test builds its `contracts` set from EFFECT + PRECONDITION + TAGS + node_id and **omits port names**. Adding port names to `contracts` clears every card to zero leak | **a test gap, not a leak** — port names *are* the port contract, which is exactly what #6 says composition may see. Left unfixed: widening a leak assertion is a decision, not a repair |
| Do the two new nodes add any leak? | `no_papers` still flags only `reorient_geometry` | ruled out |
| Full sim-free suite, after the two admissions | 742 passed, **8 failed** | 7 compose + 1 no_papers |
| Cost of growing the library | the 5 known `SNAPSHOT_MISMATCH` failures became **7**: `test_composite_ids_are_deterministic` and `test_a_composite_carries_no_provenance` now fail with `UNRETRIEVED_NODE` | ⚠️ **the standing cost is compounding, not constant** — two admissions cost two more tests |
| Are the composition records reconstructible? | 6 of 8 carry `retrieval.considered` with the full id list; the 2 repair records carry `considered: 0` but name their parent in `repair_of`, and the parent has it | **feasible for all 8** — `repair()` reusing the previous retrieval is what makes the chain sound rather than a guess |
| Is `6f9686b0` reconstructible? | **Not checked.** It starves on `gen_b_1f2fe8016b1441dc`, the *superseded* generation; reconstruction would have to materialise a superseded node, which `load_composition` refuses by design (#8) | **inconclusive** — the one record where reconstruct may not be sufficient |
| Was the pass rate's stability checked? | **No, and it is not checkable here.** n=1 per cell, 14 judged calls total | inconclusive **by construction**, FINDINGS #3 |
| Card fidelity of either new node | **Not checked.** FINDINGS #1: the harness cannot | out of scope, unchanged |
| Behaviour under physics | **Not checked.** Both results are kinematic micro-worlds | outstanding |
| Import allowlist | Not weakened, not extended; no candidate was made to pass | confirmed |
| Leakage: was the held-out split ever in context? | `core/scenarios_held_out.py`, `core/tests/test_held_out.py`, `plans/done/11-held-out-split.md` and PRE-REGISTRATION §4 never opened; every `grep` named its directories explicitly, and the one `core/` search passed `--exclude=scenarios_held_out.py --exclude=test_held_out.py` | chunk 11's procedural rule honoured |

### Still open

- **The composition decision is now more urgent, not less**: 7 failing tests, and each admission
  adds more. Reconstruct vs. re-compose is still the user's call.
- `test_no_papers.py`'s `contracts` set omits port names. One line, but it is a leak assertion.
- **D remains at 0 of 4.** This chunk's "Done when" is still unmet.
- 15 of 20 A/B cells unpaired; the other three task families are untouched in both arms.

---

## The composition decision, taken and implemented — 2026-09-06

**User's decision: reconstruct.** The exemption variant — allowing a
reconstruction to materialise a superseded node — was offered and **declined**,
so the one record that needs it stays a refusal by choice.

Reproducing a composition used to mean "run the recorded plan against the
library as it stands today". It now means **"run the recorded plan against the
library the record names"**, rebuilt from that record's own
`retrieval.considered` with **no model call**.

### What was built

- `compose/library.py`: `load_library(..., only=<node ids>)` restricts the build
  to a set of ids, filtering **before** each candidate's source is read or
  executed — a historical reconstruction must not run, or be broken by, a
  candidate admitted after the composition it is rebuilding. New
  `LibraryReconstructionError` when a requested id is not an admitted,
  non-superseded, non-retired entry today.
- `compose/compose.py`: `_library_as_recorded()` resolves the historical id set
  (own `considered`, else the parent's via `repair_of`) and **verifies the
  rebuilt library's own `snapshot()` against the tag in the record**. New public
  `library_as_recorded(composition_id)`.
- `compose/tests/conftest.py`: `recorded_library` / `recorded_found` fixtures.
  Any test replaying a committed record now uses them.

**#8 is enforced harder, not weakened.** The tag hashes the assembled contents,
so matching it *proves* the set was reassembled exactly — where before the tag
was asserted about a library nobody rebuilt. What is given up is only the
accidental property that a composition stopped loading whenever something *else*
was admitted, which was never a check on anything.

`only=` is a **filter over committed records, never a source of them**. An id
that was never admitted cannot be conjured by writing it into a JSON file;
`test_reconstruction_cannot_conjure_a_node_that_was_never_admitted` pins it,
because without it "rebuild the library the record names" would be a route to a
primitive G3 never admitted.

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| Do the live records reconstruct? | All three rebuild to `lib-006-23c57a77269e` **bit for bit**, n=6/6 entries each | confirmed against the record's own tag |
| Does retrieval over the reconstruction reproduce the record? | `retrieved` list identical for `NAVIGATE_ID` | confirmed — retrieval is a pure function of (library, spec) |
| Do the repairs chain correctly? | `REPAIR_ID` has `considered: []`, resolves through `repair_of`, reconstructs to the recorded tag | confirmed |
| Which records do NOT reconstruct? | The five naming `gen_b_1f2fe8016b1441dc` — the four archived `lib-006-a05b0b267dd4` records and `6f9686b0` | **expected**: the superseded generation is dropped by `admitted_stems` so one mechanism is not in the library twice. Refused loudly as `UNRECONSTRUCTABLE_LIBRARY`, naming the node |
| Is `6f9686b0`'s finding lost? | No — read off the plan structurally by `test_a_composition_can_succeed_and_still_not_work`, unchanged; the half of the pair that runs still runs | ruled out |
| Does the runner path still work? | `controllers.make("composed:<id>")` returns a `Tree` for all three live compositions | confirmed — `run_episode.py --controller composed:<id>` needs no change |
| Is the reconstruction test vacuous? | No: it asserts the LIVE library's snapshot **differs** from the recorded one, so it fails if the two ever converge and the other assertions go quiet | confirmed, and pinned |
| `compose/tests/` | 57 passed, 1 failed | the 1 is the pre-existing `no_papers` port-name gap |
| Full sim-free suite | **754 passed, 1 failed**, 212 s | from 741/5 at session start, via 742/8 after the two admissions |
| Model calls spent on this | **zero** | confirmed |
| Behaviour under physics | **Not checked.** No composed tree has run in the simulator — `compose/FINDINGS.md` §5 is unchanged | outstanding |

### Still open

- `compose/tests/test_no_papers.py`'s `contracts` set omits **port names** while
  `render_node` renders them, so `gen_b_4dc13c6a972bb06b`'s `reorient_axis` /
  `reorienting` ports read as a card-vocabulary leak. Diagnosed, **not fixed**:
  port names *are* the port contract, which is what #6 says composition may see,
  but widening a leak assertion is a decision rather than a repair.
- **D remains at 0 of 4**, and 15 of 20 A/B cells are unpaired.

---

## Closing note — 2026-09-15, moved to `done/`

⚠️ **Moved because its Done-when checks now pass, NOT because this chunk made them pass.**
Chunks 42 (card-first generation) and 52 (the layered pipeline regeneration) closed the
outstanding items. Recorded here so the board is accurate and the credit is not.

| Done-when | State 2026-09-15 | Closed by |
|---|---|---|
| `--pass-rate` non-zero for **every** condition | ✅ **A 5, B 10, D 1** — 16 admitted, all `pipeline_version: v3-layered` | 42, then 52's regeneration |
| The ceiling and mechanism counts asserted by tests | ✅ `generate/tests/test_ceiling.py` | this chunk |
| `generate_batch.py --dry-run` prints the grid, no model call | ✅ now a 44-row grid | this chunk |
| Every `cards/` card verified or listed outstanding with a reason | ✅ `cards/VERIFICATION.md`: 4 of 5 verified, provenance 0.80, `limit_speed_ics` open on purpose (wrong edition of the paper) | 14 / this chunk |
| Coverage target is a dated append in `PRE-REGISTRATION.md` | ✅ 2026-08-31, *"a common ceiling, not ≥20"* | this chunk |
| `snapshot()` gives a tag a reportable spec can name, suite green | ✅ suite 887 passed, 1 failed (the pre-existing `no_papers` gap) | this chunk |

### ⚠️ The decision this chunk existed to force was never taken, and the shortfall is worse than the table looks

**"D remains at 0 of 4"** above is closed only in the weakest sense: D has **one** admitted
node. The 2026-08-31 amendment says G1 is reported against *"the largest count every condition
reaches on equal terms"*. After the 2026-09-06 restructure the reachable ceilings are
**A 20 / B 20 / D 4**, so the common ceiling is **4** — and D stands at **1 of 4**.

So the number G1 would report today is **1**, against a stated target of 20. The arms are also
at incomparable counts (A 5, B 10, D 1), which `plans/done/42`'s Outcome flags and which means
nothing in the library may currently be read as an arm comparison.

**Growing the library is therefore still open work** — it is simply no longer blocked on
machinery, which is what this chunk built. It needs model calls and a scale decision, and it
belongs to whichever chunk picks up the growth loop.

### Also still open, carried forward unchanged

- `compose/tests/test_no_papers.py`'s `contracts` set still omits port names while
  `render_node` renders them. Diagnosed here, **still not fixed**, and chunk 42's Outcome
  confirmed the prediction that it would resurface: it now fails on **`region`** (card
  `reorient_geometry`) via the ports `search_region` / `goal_region_robot`, rather than on
  `reorient_axis`. Chunk 39 told later chunks to leave it alone; they have.
- **Behaviour under physics** remains unchecked for any composed tree, as this chunk's own
  Checks table recorded. Still true: no composition exists over the v2 or v3 library at all.
