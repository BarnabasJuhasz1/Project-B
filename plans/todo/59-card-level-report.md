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
