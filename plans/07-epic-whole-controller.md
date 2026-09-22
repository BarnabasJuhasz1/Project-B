# Epic 07 — the whole-controller arm

**Opened 2026-09-21, on the user's explicit decision.** Epic 06 is complete; the repair loop
works and found real defects. This epic exists because fixing nodes is no longer the bottleneck.

⚠️ **Read this before any chunk in it.** The chunks reference these definitions rather than
repeating them, the same way epic 06's chunks reference `REPAIR-LOOP.md`.

⚠️ **REVISED 2026-09-21, same day, before any chunk was written.** The first draft of this epic
generated a complete controller **from scratch** per card. That is superseded by the design
below — the model **edits the existing base navigator** instead. The reason is in
[Why editing and not writing](#why-editing-and-not-writing). No draft chunk existed, so nothing
is orphaned.

---

## Why, in one paragraph

**The problem is not that the results are negative. It is that not one of them is
interpretable.** Every number this project has produced against the corpus is confounded by a
defect in the apparatus that has nothing to do with the corpus: every growth world ran at
`drift: None` while 6 of 16 cards predicate on drift; `modifier.md` never advertised
`nav_heading` / `nav_stuck` / `nav_progress`, so every node in every arm was written blind to
what the base intended; round 1 scored `goal_reached` only while 6 of 16 cards predict
`collisions`; count metrics rewarded a robot that stopped, and did. Each was found *after* a
number had been reported. A clean negative would be publishable. **"We could not tell" is not.**

## Why the behaviour tree is the part to drop

The claim under test — CLAUDE.md's first line — is about **navigation code**. Behaviour trees
appear nowhere in it. The leaf decomposition was chosen to make G3 possible and to make "no
scenario coupling" mechanical; it bought seven independent failure surfaces (arbitration,
`WRITES` narrowing, base blindness, selectivity, layer order, retrieval, composition), **none of
which is the independent variable**, and that is where every confound above lives.

The evidence that it is the apparatus and not the cards:

- **B−A came out exactly zero, three times now**, under three execution models: exclusive
  control, layered modifiers, and layered modifiers **with the drift hazard switched on**
  (`out/growth/maze+drift0.10.csv`, 2026-09-18). Epic 06 removed the best available excuse and
  the answer did not move. ⚠️ **That elimination is the strongest single argument for this epic**
  and must be stated in the write-up: the confound was repaired and the null survived it.
- The library splits **bimodally** — 4 nodes never fire, 6 fire on ~100% of ticks and overwrite
  the base entirely, all 6 regress — with nothing in between. Sixteen papers do not produce a
  bimodal distribution. An execution contract does.
- **Zero of sixteen nodes improved their own claimed metric while still doing the task**, and two
  of the three apparent wins were a robot that had stopped walking.
- The one node that helps is **D's**, the arm handed no corpus at all, at n = 1 against 15.

## The gap that threatens the premise independently of trees

⚠️ **There is no "model alone" condition, and the hypothesis names one.** The naive arm was
retired (see CLAUDE.md, *The conditions*). D is not it: D still articulates a principle before
implementing. So the project currently cannot say that the corpus beats *a frontier model asked
plainly*, which is the comparison the claim is about. **This epic restores that control**, and it
is the cheapest thing in it.

---

## Why editing and not writing

Handing a model one card and asking for a whole navigator makes it invent goal seeking, gap
selection, stuck recovery and escape from nothing. **That uncontrolled remainder is most of the
emitted code, it is not from the card, and it is where the variance would live.** It is the old
confound wearing a new hat.

`generate/base/base_navigator.py` already is a monolithic navigator: 538 lines, an
explore / wall-follow / escape state machine with named seams (`_compute`, `_explore_heading`,
`_wall_heading`, `_novelty`, `_enter_escape`, `_enter_wall`). It was written by a **neutral**
call — node contract and task family, no card, no citation, no corpus — which is as close to
*"an LLM asked plainly to program a Go2 to do X"* as this project has ever had. It is frozen and
it is already scored on the 20 maze worlds: **0.50 undrifted, 0.40 at drift 0.10.**

So the model is handed that source and asked to **fold the principle into it**. What this buys:

| | |
|---|---|
| **Variance** | The navigation scaffolding is held constant and identical across arms. This is the property that made nodes low-variance — the base absorbing the noise — **without** the `Layer` / `WRITES` / arbitration contract that killed them. |
| **Interpretability** | The deliverable is a **diff**. For the first time the project can show, line by line, what the card bought. |
| **Pairing** | Every arm modifies the same artefact, already scored on the same worlds. The comparison is paired by construction. |
| **Size of change** | Nothing new has to be validated for the comparison to mean something. |

### ⚠️ The objection, stated here so it is not discovered later

**`Layer[ports, BASE, node]` was already "base + card-derived delta".** Does editing reproduce
the failure? The argument that it does not is that editing removes the three mechanisms that
produced every documented failure — and it must be read as a hypothesis, not a settled fact:

| Failure under `Layer` | Under source editing |
|---|---|
| Nodes written **blind** — `modifier.md` never advertised `nav_heading` / `nav_stuck` / `nav_progress` | The model reads the base's actual source. Blindness is impossible. |
| **Last-writer-wins steering** — the ant node destroyed the base's turn-in-place gate while capping a speed that never bit once (`REPAIR-LOOP.md`) | No arbitration layer exists. An edit is coherent with the surrounding control flow or it does not parse. |
| **Bimodal firing** — never fires, or fires ~100% and overwrites everything | An edit has no fire rate. Conditionality is written into control flow the model can see. |

⚠️ **This hypothesis is testable for three model calls, and chunk 60 tests it before spending
anything else** — see [the p gate](#the-p-gate).

### The risk this design carries, and its direction

**Anchoring.** Handed working code, a model makes conservative edits. If a card implies a
structurally different navigator, "expand the base" will not produce it. ⚠️ **This biases the
result toward null.** A null from this arm therefore means *"cards did not improve this
navigator"*, never *"literature does not help"*, and the write-up must carry that distinction
from the start rather than discover it afterwards. It is the safer direction to be wrong in, and
it is not a free pass.

---

## The arms

⚠️ **The control arm is `p`, for *plain*** — from the claim's own wording, "an LLM asked
**plainly**". It is **not** called `n`, because `n` means sample size everywhere else in this
project and the collision is exactly what the "say `n` out loud" rule exists to prevent.

⚠️ **Asking a model to improve code is itself a treatment.** So `p` is not the frozen base: it is
**the base edited with no new information given**. That separates the act of editing from the
card, the same way the 2026-09-06 restructure separated principle articulation from the corpus.

```
BASE   frozen, k = 1, already scored (0.50 undrifted / 0.40 at drift 0.10)
  │
  ├─ p   "improve this navigator"                       k samples   P − BASE = does asking again help?
  ├─ a   "...incorporating this principle" + A's card   k samples   A − P    = does a citation-derived card help?
  └─ b   "...incorporating this principle" + B's card   k samples   B − P    = does a document-derived card help?
                                                                    B − A    = what EXTRACTION buys
```

⚠️ **Every arm that gets a card gets a *card*.** Not a raw citation. A's card was written from the
citation alone and B's from the full document; that difference is the independent variable and it
is already committed to disk. Handing `a` a bare reference string and `b` a six-field YAML would
make the arms differ in **format** as well as in source, and format would dominate. It would also
orphan all 15 committed cards.

⚠️ **The cards do not move.** Every card under `generate/cards/{a,b}/` is reused byte for byte,
the way chunk 52 reused them when the execution model changed. The independent variable and the
execution model must never move in the same step — that rule is why the v2 records are readable
at all. **This epic makes zero card calls.**

⚠️ **`generate.cardgen.CONDITIONS` stays `("a","b","d")` and no existing code changes.** Arm `p`
exists only inside this epic's own registry. It is a control for the *edit* call, not a fourth
card condition; nothing generates a card for it, because having no card is what it is.

⚠️ **D is not in this epic.** Frozen 2026-09-15, standing instruction. Its record stays and
still counts.

## The dossier: what the card block contains

**All five of the arm's cards, every time. No retrieval.**

A and B are built on the **same five references** — `limit_speed_dwa`, `limit_speed_ics`,
`path_integration_ants`, `path_integration_mammals`, `reorient_geometry`. A has one card each
(5), B has two each (10) only because B ran two task families. **At the source level the arms are
matched**, and feeding the whole set makes them matched at the prompt level too.

This fixes, for free, the warning in CLAUDE.md that the arms are not at comparable row counts and
that nothing may yet be read as an arm comparison: **cards become inputs and the task becomes a
parameter**, so both arms run all four families off five matched cards.

⚠️ **No retrieval step.** Retrieval is another apparatus surface, and this cycle is what apparatus
surfaces cost. Non-negotiable #6 ("composition sees only port contracts, never the papers") does
not apply, because there is no composition.

⚠️ **A card's `Prediction` may name a task.** A card written under `navigate_to_goal` fed into a
`search_for_goal` edit is a mild mismatch. It is accepted deliberately — the mechanism is not
task-bound, and filtering would reintroduce retrieval — and it is recorded as a limitation.

## k, and the arithmetic

**k = 3.** One generation is one draw from a stochastic process; `generate/FINDINGS.md` already
records that temperature-0 determinism does not exist on any current model, so k identical calls
give k genuinely different edits for free. Editing suppresses most of the variance that would
force k = 5–10 from scratch, but it does not remove it: what still varies is **where** the model
cuts in, **how much** it rewrites, and **whether it no-ops**.

| arms | tasks | k | **controllers** | generation calls | card calls |
|---|---|---|---|---|---|
| 3 (`p`, `a`, `b`) | 4 | 3 | **36** | 36 | **0** |

All four task families are kept. Drift is the axis 6 of 16 cards explicitly predicate on and it
is finally wired (`out/growth/maze+drift0.10.csv`); dropping to two families would discard the
axis epic 06 just repaired.

Scoring is kinematic and paired on seeds, as `measure_growth` already is: 36 × 20 = 720 rollouts,
tens of minutes, with the ~22 s SPL denominator precomputed once per world and cached. **Physics
is a confirmation set at the end, never the scoring loop** — see `todo/53`.

## The chunks

| # | Chunk | Depends on | Model calls | Status |
|---|---|---|---|---|
| 59 | [The card-level report: coverage and provenance](todo/59-card-level-report.md) | — | no | todo |
| 60 | [The edit prompt, the `p` gate, and the generation path](todo/60-edit-generation.md) | — | yes | todo |
| 61 | [The paired measurement, and the diff readout](todo/61-paired-measurement.md) | 60 | no | todo |

⚠️ **59 does not depend on 60 and must not be scheduled behind it.** It is the result most likely
to survive, it needs no robot, no controller and no simulator, and most of it is already on disk.
**It goes first for that reason, not because it is the most interesting.** The project is in week
5 of 6 (started 2026-08-21).

⚠️ **60 contains a gate that can stop this epic for three model calls.** Do not generate `a` or
`b` before it reports.

---

## What is deliberately given up, and must be stated in the write-up

Not omitted quietly. A reader of §4 will look for each of these.

1. ⚠️ **Non-negotiable #4 is SUSPENDED FOR THIS ARM, on the user's explicit instruction
   (2026-09-21).** "The generator emits leaf nodes, never a monolithic stack" is exactly what this
   arm stops doing. It is suspended, not repealed: the node library, its records and its
   pass-rate denominators are untouched and still mean what they meant.
2. **G3 is not measurable from this arm.** Compositional generalisation needs primitives to
   recombine. An edited controller has none. G3 stands on the node library or it does not stand.
   ⚠️ What this arm *can* measure is ordinary generalisation — a controller generated against one
   world set, scored on another — and that must never be reported as G3.
3. **G1's library framing narrows to the card level.** "≥20 admissible leaves" is a statement
   about nodes. The coverage count (distinct mechanism *families*) and the provenance fraction
   were always properties of the **cards**, and that is where chunk 59 reports them.
4. **Mechanism-level attribution is gone.** A dossier of five cards buys an arm effect, not a
   per-mechanism effect. The diff readout in chunk 61 partially recovers it, qualitatively.
5. **The fidelity problem is only partly addressed.** A controller can be handed a card and
   ignore it, exactly as `generate/FINDINGS.md` records for nodes (the deliberately bad card was
   admitted). ⚠️ The diff makes this **visible for the first time** — you can read whether the
   mechanism appears — but reading a diff is not an automatic check, and the `p` control absorbs
   what the reading misses.

## What must not move

- The frozen locomotion policy, and the Twist as the only interface. Untouched.
- **Arm-blindness of the edit prompt.** `render_edit(cards, task)` takes no condition parameter,
  for the reason `render_node` does not: the moment it can tell the arms apart, the ablation has
  a second place to differ. Pinned by a test, and by the strip-the-source-block-and-diff test
  that `card_{a,b,d}.md` already uses.
- **The base.** `generate/base/base_navigator.py` is the input to every edit and is never itself
  regenerated. Its committed score is the reference every number in this epic is paired against.
- The sealed split. `core/scenarios_held_out.py` never enters this epic, by the same AST guard.
  Generated worlds only.
- `results/`, the committed compositions, and every v1/v2/v3 record. This arm stages into its own
  directory with its own `pipeline_version`; **no pass rate and no growth table pools across it.**

## The two risks that decide whether this is worth running

**1. The ground-truth leak.**
`Controller.reset(scenario, rng)` is handed the whole `Scenario` — `goal`, `start`, `seed`,
`scene`, the lot. `controllers/reactive_avoid.py:66` stores it wholesale, legitimately, because a
human wrote it. A **generated** controller that reads `scenario.goal` has perfect goal knowledge
on a goal-hidden scenario, and the search family stops being a task. A controller that reads
`scenario.seed` or `scenario.id` is coupled to the scenario, which is precisely what #4's "no
coupling" clause made mechanical for nodes.

⚠️ **This is the `PortView` problem in a new place, and it gets the `PortView` answer: a
restricted view, enforced, not a rule anyone has to remember.** Chunk 60 wraps the edited class so
`reset` receives only the actuation and sensing limits it legitimately needs. The runner and the
reference controllers are unchanged. **The base itself must pass this guard unmodified** — if it
does not, the guard is wrong, and that is a cheap and worthwhile first test of it.

**2. The sample unit changed, and the row key must change with it.**
⚠️ **`plan_hash` is a pure function of `(condition, task, card)`**, so all k samples of a row land
on one stem. That is exactly the collision that forced the v2 archive move in chunk 52 — two
experiments cannot share a row key, and neither can two samples. **`plan_hash` takes a sample
index in this epic, and that lands before the first call, not after.**

⚠️ **Say `n` out loud**, everywhere, as always.

## Pre-registration

⚠️ **This arm is not in `PRE-REGISTRATION.md` and cannot be reported under it without a dated
append.** The append must declare: that a whole-controller arm exists; that non-negotiable #4 is
suspended for it; the `p` control and that it is an *edited* base rather than the frozen one; k;
the dossier rule (all cards, no retrieval); **the analysis unit and the bootstrap cluster**; that
best-of-k is a secondary declared in advance; and that no number from this arm pools with any
node-library number.

⚠️ **The bootstrap unit is a pre-registration decision and is not obvious.** `core/stats.py`
clusters on scenarios because replicate seeds were the same episode. There are now two nested
factors, controller sample and world. The default this epic proposes: **an arm's score on a world
is the mean over its k controllers; the bootstrap resamples worlds.** That is n = 20, not n = 60,
and only one of those is honest. Chunk 61 settles it and writes the append.

It is an append, never an edit. `prereg-v1` is tagged at `a84cd46`.

**This is not a blocker on building the arm.** It is a blocker on reporting from it, and it is
written here so the first scoring run does not discover it.

## What success looks like for the epic

⚠️ **Not "the corpus wins".** Success is **an interpretable number** — one where a null result
means the corpus did not help, rather than meaning the apparatus was broken again.

Concretely: card-fed edits and the no-card control, scored on the same generated worlds, against
the same frozen base, on the full metric set, with each card's own claimed metric marked, count
metrics reported per metre, the drift axis on where a card names it, k stated, the spread across
k reported, and the diffs readable. If B beats P, that is the project's result. If it does not,
**that is also the project's result**, and for the first time it is one that can be written down.
