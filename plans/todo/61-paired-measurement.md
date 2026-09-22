# 61 — The paired measurement, and the diff readout

**Epic:** [07](../07-epic-whole-controller.md) · **Depends on:** 60 · **Needs Isaac Sim:** no
(confirmation only, via [53](53-physics-on-a-generated-world.md))
**Status: PLANNED, NOT BUILT.** Written 2026-09-21. **Makes no model calls.**

---

## What it measures

36 edited controllers — 3 arms × 4 task families × k = 3 — against the **frozen base**, paired on
the same 20 maze worlds `measure_growth` already uses, kinematically. 720 rollouts, tens of
minutes, with the ~22 s SPL denominator precomputed once per world and then cached by
`core.geometry`.

Deliverable: `scripts/measure_controller.py` and `out/evidence/controllers/README.md`.

⚠️ **It reuses `measure_growth`'s pairing rather than re-declaring it**, the way
`debug_growth_case.py` imports `measure_growth._build` — so it cannot report a comparison the
pairing never made. What changes is only the thing being paired: a whole controller, not
`Layer[ports, BASE, node]`.

## The four scoring rules inherited from epic 06, none of them optional

1. **The full metric set**, not `goal_reached` alone. Round 1 scored goal-reach only while 6 of
   16 cards predicted `collisions`, and judged them on a claim they never made.
2. **Each card's own claimed metric is marked.** A dossier means an arm carries several claimed
   metrics; mark all of them and report each.
3. ⚠️ **Every COUNT metric is reported per metre travelled, beside the raw count, with a flag when
   the two disagree.** A robot that stops scores perfectly on collisions, and that already
   produced an apparent win whose sign reversed under this rule.
4. ⚠️ **The drift axis is switched ON where a card names it.** `core.worldgen.to_scenario()`
   never set `drift`; epic 06 fixed that and `out/growth/maze+drift0.10.csv` is the result. Every
   task family in the `*_under_drift` pair runs drifted. A mechanism that exists to survive a
   hazard is not measured with the hazard switched off.

## The analysis, and what must be declared before the first scoring run

⚠️ **Score every one of the k samples. Never pick.**

| readout | what it answers | status |
|---|---|---|
| **Mean over k**, paired on worlds | *If you feed the generator literature, is the code you get back better on average?* — the claim | **PRIMARY** |
| **sd across the k samples** | how high-variance "LLM writes navigation code" actually is | **report always** |
| **Fraction of samples beating the base** | robust, assumption-free, immediately legible | **report always** |
| **Best-of-k** | *does the corpus expand what is reachable?* | **SECONDARY, declared in advance** |

**The comparisons, in the order they should be read:**

```
P − BASE   does asking a model to improve its own navigator help, with no literature at all?
A − P      does a citation-derived card help?
B − P      does a document-derived card help?          ← the corpus effect
B − A      what EXTRACTION buys — the same paper, read or not     ← the headline
```

⚠️ **B − P is the corpus effect, not B − BASE.** If `p` beats the base, B − BASE contains the
free improvement from a second pass and is not a corpus number.

⚠️ **Best-of-k rises mechanically with k.** It is comparable only because k is identical across
arms, and it must be declared before scoring or it is a cherry-pick. **The one thing that would
sink this epic: scoring k controllers, picking the best, and reporting it as the arm's score.**

### The bootstrap unit is a pre-registration decision and is not obvious

`core/stats.py` clusters on **scenarios**, because on the undrifted held-out scenarios the
replicate seeds are the same episode and episode-level resampling would shrink every interval by
sqrt(3) for free. There are now two nested factors: **controller sample** and **world**.

**The proposal: an arm's score on a world is the mean over its k controllers; the bootstrap
resamples worlds.** That is **n = 20**, not n = 60. Sixty is the number that would flatter the
result and it is not honest — the k samples of one row are not independent observations of a
world, they are repeated measures of one generation process.

⚠️ **This chunk writes the dated append to `PRE-REGISTRATION.md`**, and the append lands
**before** the first scoring run, not after it. It must declare: the whole-controller arm; that
#4 is suspended for it; that `p` is an *edited* base and not the frozen one; k; the dossier rule
(all cards, no retrieval); the analysis unit and bootstrap cluster above; best-of-k as a declared
secondary; and that **no number from this arm pools with any node-library number**. It is an
append, never an edit; `prereg-v1` is tagged at `a84cd46`.

## The diff readout — the part that is new

Chunk 60 commits a unified diff per sample. **This is the first time the project can show what a
card actually bought**, and it gives three things no rate does:

1. **Fidelity.** Does the card's mechanism appear in the code? ⚠️ `generate/FINDINGS.md` records
   that the harness cannot detect card infidelity — the deliberately-bad card was admitted. A
   diff can be *read* against the card. **That is a human reading, not an automatic check**, and
   it must be reported as one, with the reader and the date.
2. **Coverage on code, not on cards.** How many distinct mechanisms appear across an arm's diffs.
   Chunk 59 counts families in the *inputs*; this counts them in the *output*, and the two can
   disagree — a card that never reaches the code is coverage the robot never had.
3. **Selectivity.** ⚠️ The project named selectivity as the prime suspect for the node library's
   bimodality and never measured it. In a diff you can read directly whether the edit is gated on
   a condition or applied unconditionally. Report it per sample: **gated / unconditional / no-op**.

⚠️ **The diff reading must be blind to the arm**, by the same argument as chunk 59's family
labelling. Strip the arm before reading; rejoin afterwards.

## Physics

⚠️ **Kinematic is the scoring loop; physics is a confirmation set, and no number from this chunk
is a benchmark number.** `plot_kinematic.py`'s warning applies unchanged: the frozen locomotion
policy is absent, so nothing here is reportable as a G2 number.

The confirmation runs through `todo/53` — a handful of worlds, in the pinned container, at the
end. ⚠️ **`todo/41` (the harness/runner timebase mismatch) bites exactly here**: it is the defect
that makes a kinematic verdict and a physics verdict disagree for a reason that is neither the
card nor the controller. Fix 41 before the confirmation set, or the confirmation cannot be read.

⚠️ And the standing correction from `REPAIR-LOOP.md`: **no prose may say the base *solves* a world
without saying kinematically.** Under physics the base also times out on maze 2004.

---

## Checks this chunk must run and record

- **Does `measure_controller` import the pairing from `measure_growth` rather than re-declaring
  it?** Must.
- **Is the set balanced?** Every arm, every task, every k, every world — and it must **raise** on
  an incomplete set the way `runs/aggregate.py` does, never silently report a partial one.
- **Does the drifted half actually drift?** Assert non-zero pose error on a sample episode.
  ⚠️ This is the exact defect epic 06 found; it must be asserted, never assumed.
- **Do the raw count and the per-metre count ever disagree in sign?** Report every case. This
  already happened once.
- **Is `n` stated on every number?** n = 20 worlds, k = 3 samples, 36 controllers. Say it out loud.
- **Not checked, and say so:** novelty (out of scope, `todo/35`); G3 (no primitives to recombine —
  ⚠️ generalisation across world sets may be reported, but **never as G3**); and whether the
  anchoring bias suppressed a real effect, which this design cannot detect.

## What would make this chunk wrong

Reporting best-of-k as the primary. Bootstrapping at n = 60. Reading diffs with the arm visible.
Reporting B − BASE as the corpus effect when `p` moved. Pooling any of it with a node-library
number. Calling a kinematic result a benchmark number.
