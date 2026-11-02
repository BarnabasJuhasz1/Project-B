# 59 — The card-level report: coverage, provenance, and the confabulation check

**Epic:** [07](../07-epic-whole-controller.md) · **Depends on:** nothing · **Needs Isaac Sim:** no
**Status: PLANNED, NOT BUILT.** Written 2026-09-21.

⚠️ **THIS CHUNK GOES FIRST AND DOES NOT DEPEND ON 60.** It needs no robot, no controller, no
simulator and no model call. It is the result most likely to survive the remaining week, and it
cannot be invalidated by whatever the execution architecture does next. That is why it is first
— not because it is the most interesting.

---

## What it reports

CLAUDE.md names three things a corpus can supply that a model cannot supply from itself:
**novelty**, **coverage**, **provenance**. Novelty is out of scope this cycle by design
(`todo/35`, the unseen corpus). **This chunk reports the other two, and they were always
properties of the CARDS, never of the nodes.** Nothing about them depends on behaviour trees,
on the `Layer`, or on any controller ever running.

Deliverable: `scripts/report_cards.py`, and `out/evidence/cards/README.md` carrying the numbers,
the question each answers, and the command that reproduces it.

---

## Part 1 — Coverage, and the trap in counting it

⚠️ **`CardStore.family()` returns `mechanism_family or node_name`, and NO GENERATED CARD SETS
`mechanism_family`.** `node_name` is enforced 1-1 with the card at load. So counting families
over `generate/cards/` returns **exactly the card count** — A = 5, B = 10 — and would report
that B has twice A's coverage when the only thing that differs is that B ran two task families.
**That number would be an artefact of the generation grid, not a measure of anything.**

The human corpus does not have this problem: `cards/*.yaml` sets `mechanism_family` and its
5 cards collapse to **3 families** (`LimitSpeedToVisibleRange` ×2, `MaintainHomingVector` ×2,
`ReorientToGeometry` ×1).

**So generated cards need a family assignment, and it must be blind.**

- **Procedure: blind manual labelling.** The 15 A/B cards are emitted stripped of arm, id, stem
  and filename — mechanism / trigger / derivation only — in a shuffled order under opaque keys.
  The labeller assigns each to a mechanism family, creating families as needed. The key is
  rejoined afterwards. n = 15 is small enough to do by hand in an hour and a human is the
  correct instrument for "are these the same mechanism".
- ⚠️ **The arm must be hidden, and the labelling must be recorded before the join.** If the
  labeller can see which arm wrote a card, coverage stops being a measurement. Same discipline as
  chunk 11's sealed split, for the same reason, and no test can detect the violation.
- The mapping is committed as data (`cards/families.yaml` or similar), with the date, who
  labelled, and the stripped text they saw.

**Then report:** distinct families per arm, per source reference, and for the human corpus; the
count of families A and B share; and the count each has that the other does not. ⚠️ **The
per-arm counts are comparable only because the arms were given the same five references** — say
so beside the number.

## Part 2 — Provenance, reported as three separate numbers that must never be pooled

| number | what it is | current value |
|---|---|---|
| **corpus-level** | `CardStore.provenance()` over `cards/*.yaml` — human-extracted, spans verified against the document | **0.80** (4 of 5; `limit_speed_ics` open on purpose, wrong edition of the paper — see `cards/VERIFICATION.md`) |
| **node-level** | `NodeMetadata.span_verified`, which `is_traceable` demands | **0.0**, and that is the honest state: nothing in the generation path sets it |
| **generated-card-level** | spans in `generate/cards/{a,b}/` | **claims, never traceable.** `cards.store` refuses to load the directory |

⚠️ **Report all three, labelled, and never report one as another.** This is already stated in
CLAUDE.md and it is the single easiest number in this project to misreport.

## Part 3 — The confabulation check, and it is the sharpest thing here

CLAUDE.md's provenance claim is: *"A model states a threshold or an exponent with equal
confidence whether or not it is confabulating."* **The A cards are a direct test of that**, and
the answer is already partly visible on disk.

Arm A wrote its card from the **citation alone** — authors, title, venue, year, no content. Any
span, threshold, coefficient or equation number in an A card is a claim the model could not have
read. The B cards, and the five PDFs in `out/corpus/`, are the ground truth to check against.

⚠️ **The preliminary read contradicts the premise, and that is a result.** Four of A's five cards
left `span` **empty**. The fifth filled it and hedged explicitly in the text:

> *"Section/equation NUMBERS not asserted - not recalled with certainty."*

That is calibrated declining, not equal-confidence confabulation. **If it holds across the
quantitative content it is a finding that partially refutes this project's own stated
motivation**, and those are worth more than confirmations.

**So the check is not about spans, which A mostly declined to give. It is about numbers.**
Extract every quantitative claim from each A card — thresholds, exponents, coefficients, stated
constants, named equations — and classify each against the source document:

| verdict | meaning |
|---|---|
| `correct` | the value appears in the document and matches |
| `wrong` | the value appears in the document and does not match ← **confabulation** |
| `absent` | the value is not in the document at all ← **confabulation** |
| `declined` | the card states it does not recall, or omits the field ← **calibration** |
| `unverifiable` | the claim is not the kind of thing the document settles |

Report `wrong + absent` over all quantitative claims as the **confabulation rate**, and
`declined` as the **calibration rate**, for A and for B. ⚠️ **B is not assumed correct** — B read
the document, but `generate/FINDINGS.md` already records that the harness cannot detect card
infidelity, so B's numbers are checked by the same procedure against the same PDFs.

⚠️ **`out/corpus/` is an input and is NEVER committed** — the PNAS copy carries the downloader's
IP address. The extracted claims and their verdicts are committed; the documents are not.

## Part 4 — Fold in the citation probe

`todo/33` (moved back from `hold/` on 2026-09-21) already reconstructs principle cards from
citations alone and diffs them field-by-field against the human card. Three reconstructions plus
their `.diff.json` are on disk in `cards/probe/`. **That is the same question as Part 3 asked a
different way** — recall against a human extraction, rather than numeric claims against the
document — and the two belong in one report.

⚠️ `cards/probe/` is **model reconstructions, never principle cards**. `cards.store` refuses to
load it two ways over, it is counted in no G1 number, and no episode is ever run from it. The
report must say so where the numbers appear.

---

## Checks this chunk must run and record

Per the working agreements, the Outcome carries check → outcome → verdict, with n where the
result is stochastic, and an explicit note of what was **not** checked.

- **Does `family()` collapse generated cards to node_name?** Confirmed already by inspection
  (no `mechanism_family` in any of the 15). Record it; it is the reason Part 1 exists.
- **Does the blind labelling reproduce?** Label a held-back subset twice, or by two people, and
  report agreement. n = 15 is too small for a kappa to mean much — **say so** rather than
  reporting one.
- **Do the three provenance numbers come from three different code paths?** They must, and a test
  should fail if any two are computed from the same store.
- **Is every A quantitative claim classified?** Count claims, not cards. Report the denominator.
- **Not checked, and say so:** whether a card's mechanism is *implementable*, and whether the
  model would have produced the same card from a different citation format. Both are out of scope.

## What would make this chunk wrong

Reporting a coverage count computed from `node_name`. Pooling any two of the three provenance
fractions. Labelling families with the arm visible. Treating B's numbers as ground truth instead
of checking them.

---

# Outcome (2026-09-23) — **PARTIAL. Parts 2, 3 (arm A) and 4 are done; Part 1 and Part 3 (arm B) are not.**

## What was built

| file | what it is |
|---|---|
| `scripts/extract_claims.py` | the DENOMINATOR, mechanically. Claims, attribution, scope. Decides no verdict |
| `generate/cards/CLAIMS.yaml` | the verdicts, by hand, each with corpus file + line + quotation |
| `scripts/report_cards.py` | the report. Four parts, `--part N` to run one |
| `out/evidence/cards/README.md` | the numbers, the question each answers, the commands |
| `scripts/make_labelling_sheet.py` | (cut 2026-09-22) the blind coverage sheet, still awaiting a labeller |

## What differed from the plan

**1. The chunk assumed the confabulation check would be about SPANS, then said it
is about NUMBERS instead. Both turned out to be too coarse.** Counting numbers is
not counting claims about the document. A card's derivation is mostly *engineering*
— `r_body = 0.35 m is the Go2's circumscribed body radius`, `wz in [-1.0, 1.0]` —
and no paper about desert ants can settle those. Scoring them `absent` would have
manufactured a confabulation rate out of the card's own design work.

So `extract_claims.py` carries **two mechanical axes** the plan did not name:

- **attribution** — `document` / `unattributed` / `self`, decided by the *sentence*,
  not the card. ⚠️ `self` wins when both are present: *"the paper's condition, with
  the Go2 number substituted"* cites the paper **and** declares the substitution,
  and scoring it `document` would mark a declared substitution as a confabulated
  recall.
- **scope** — `robot` / `source` / `unscoped`. ⚠️ `unscoped` is a real answer and is
  **included** in the denominator: the honest direction to fail in is toward a
  reader, not toward silence on the friendly side.

**2. A third axis appeared that the plan did not anticipate and that is worth
keeping: `arithmetic`.** Several of the model's own numbers are checkable *without
any paper* — they are stated derivations from its own premises. All 5 recompute
correctly. **Reported on its own line and never pooled with the document verdicts**:
"its reasoning is sound" and "its recall is sound" are different claims.

**3. Part 1 is blocked on a human, not on machinery.** The sheet was cut on
2026-09-22 and nothing on disk holds a label. `report_cards.py` **refuses to print
a coverage number** and prints the reason instead.

## Checks

| check | outcome | verdict |
|---|---|---|
| Does `family()` collapse generated cards to `node_name`? | 0 of 15 generated cards set `mechanism_family`; the count returns the card count exactly (A 5, B 10) | **confirmed** — and it is why Part 1 exists |
| Do the three provenance numbers come from three code paths? | corpus = `CardStore.provenance()`; node = `compose.library.load_library()` + `NodeMetadata.is_traceable`; generated-card = refused at load, no fraction exists | **confirmed** — 0.80 / 0.00 (0 of 17) / n/a |
| Is every A quantitative claim classified? | **n = 71 claims** over 5 cards. 30 machine-checkable; **4 settleable by a document**, 26 `unverifiable`, 0 `UNCLASSIFIED` | **confirmed**, denominator reported |
| Does A confabulate its quantitative claims? | **0 of 4 wrong-or-absent.** All four verify against the corpus text, with line numbers | **confirmed for n = 4** — ⚠️ a denominator of 4 cannot carry a general conclusion |
| Does A assert spans it could not have read? | **4 of 5 spans empty, 1 hedged in the card's own text, 0 asserted.** B: 10 of 10 asserted | **confirmed** — ⚠️ this *partially refutes* CLAUDE.md's "equal confidence whether or not it is confabulating" |
| Is the model's own arithmetic sound? | **5 of 5** recompute correctly (atan crossovers, the Rayleigh 95% disc, both spiral sweep times) | **confirmed**, and kept separate |
| Does the citation probe agree? | 3 of 5 produced a diff (4/5, 2/5, 3/5 exact fields); `node_name` disagrees on **all three** | **confirmed**, denominator is 3 |

**Two bugs found and fixed in this chunk's own instruments — both would have moved
a reported number:**

- ⚠️ **The extractor's first regex required whitespace before a unit, so it missed
  `a 180-degree rotation`** — arm A's single sharpest document claim — while
  catching `180 degrees` two sentences later. A denominator that drops the claim
  under test is worse than no denominator. Fixed (`[-\s]*`), recorded in the
  script, A's claim count went 69 → 71.
- ⚠️ **`report_cards.py` reported the node-level fraction from an exception
  fallback** rather than from the library, which would have printed the right
  number (0.00) for the wrong reason and made the "three code paths" check
  vacuous. Now loads `compose.library.load_library()` for real: 0 of 17.

**Found, not a bug, and worth carrying:** ⚠️ **2 of 5 citation-probe runs produced
no card at all, and both failed identically** — the reconstruction chose
`prediction.metric = localisation_failures`, which `cards.store` refuses because no
controller can move it. Both were the drift-facing cards. The probe's denominator
is **3, not 5**, and the failure says something about what the model reaches for
when asked to predict under drift.

## What was NOT checked, and says so

- **Arm B's claims are extracted (188 claims, 81 checkable) but NOT classified.**
  B is *not* assumed correct — `generate/FINDINGS.md` records that the harness
  cannot detect card infidelity at all. **No B number is reportable until the same
  procedure runs against the same PDFs.**
- **Coverage.** Blocked on the blind labelling, which is a human's job by design.
- **Labelling reproducibility.** Cannot be checked before there is one labelling.
  ⚠️ When it exists, n = 15 is too small for a kappa to mean much — report agreement
  on the 5 embedded human controls and say the n out loud.
- **Qualitative recall.** This check covers *quantitative* claims only. A's correct
  recall of Fraichard & Asama's Property 4 (a subset of evasive manoeuvres gives a
  conservative over-approximation) carries no number and is outside the denominator
  by construction. Stated here rather than discovered by a reader later.
- **Whether a card's mechanism is implementable**, and **whether the model would
  produce the same card from a different citation format.** Both out of scope, as
  the chunk said.

## What this means for the epic

The chunk's own prediction held: **this is the result most likely to survive the
week, and it is the one that cuts against the project's stated motivation rather
than for it.** Arm A, given only a citation, *declined* to assert spans and got
every document-settleable number right. The provenance axis as CLAUDE.md states it
— "equal confidence whether or not it is confabulating" — **is not what this model
did**, at n = 4 claims and 5 spans.

⚠️ That does **not** dissolve the provenance argument; it relocates it. The model
declined at the *card* level, where declining is cheap and visible. Whether it
declines when asked for a *controller* — chunk 60's edit call, where a number must
be written into code for the thing to run at all — is a different question and is
not answered here.

## Test suite, run twice, and one PRE-EXISTING failure this chunk did not cause

`core nodes harness cards generate compose runs`: **908 passed, 1 failed, ~8m45s**,
identical across two runs. ⚠️ **CLAUDE.md says "565 tests, ~116 s" and that is
stale** — it is 909 tests and closer to nine minutes.

**Not caused by this chunk.** The only tracked change in the tree is the `git mv`
of this plan file; the four new files are additions and no test reads one.

```
FAILED compose/tests/test_no_papers.py::
       test_no_card_vocabulary_reaches_the_rendered_node_block
AssertionError: reorient_geometry: ['region'] appear in the rendered node block
and in the card, and in no node's public contract
```

**Diagnosed, not fixed** — `compose/` is off the live path after the pivot and
this is outside the chunk. What it is:

- The test builds its `contracts` set from `EFFECT`, `PRECONDITION`, `TAGS` and
  `node_id`, and **omits the port vocabulary**, which `render_node` legitimately
  emits because non-negotiable #6 is precisely *"composition sees only port
  contracts"*.
- ⚠️ **`REGION` is one of the seven CLOSED port types** in `nodes/contract.py`.
  `content_words("(Region, out)")` is `{'region'}` on its own, so the rendered
  block leaks the word **whatever the node is called**. Two B nodes carry it
  (`gen_b_78bda1e50f89596a`'s `search_region`, `gen_b_b561d9c7f6356028`'s
  `home_anchor`), and the human card `reorient_geometry` happens to use the word
  too. **A false positive on closed contract vocabulary.**
- ⚠️ **But a real channel is hiding underneath it and must not be closed by
  accident.** A port *name* is free text the generator chooses, so a port name
  CAN carry card vocabulary into the composer prompt. Here the two are
  indistinguishable because `search_region` and the type `Region` both tokenise
  to `region`. **Any fix must add the closed PortType names to `contracts` and
  leave port NAMES in the check**, or it silently deletes a guard against the
  leak the test exists to catch.
