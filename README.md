# Project B — literature-informed navigation code generation

**The claim under test:** injecting principles extracted from research literature into the
code-generation step produces materially better navigation code than a frontier model produces
alone. **No model is trained.** Improvement, if any, comes from program synthesis under selection
over a frozen locomotion policy — which is what keeps the sim-to-real gap small.

---

## ⚠️ Read these, in this order

| | |
|---|---|
| 1 | **[`../CLAUDE.md`](../CLAUDE.md)** — the operating manual and the source of truth for every decision. Start with its **Status** box. |
| 2 | **[`plans/07-epic-whole-controller.md`](plans/07-epic-whole-controller.md)** — the current epic. |
| 3 | **[`PIPELINE.md`](PIPELINE.md)** — the vocabulary, if *card*, *node*, *arm* and *admission* are not yet one picture. |
| 4 | **[`plans/README.md`](plans/README.md)** — the board. The folder a plan sits in **is** its status. |

## ⚠️ The project pivoted on 2026-09-21

**The behaviour-tree execution model is dropped.** A generated artefact is no longer a leaf node
in a tree. A model is handed `generate/base/base_navigator.py` — a 538-line monolithic navigator
written by a neutral call, frozen, already scored — and asked to **fold a principle into it**.
The deliverable is a whole `core.observation.Controller` **and a readable diff**.

No `Layer`, no `WRITES`, no arbitration, no layer order, no retrieval, no composition.

**Why.** Not because the results were negative — because **not one of them was interpretable**.
B−A came out exactly zero three times, under three execution models, the last of them with the
drift hazard switched on. The library split bimodally: 4 nodes never fired, 6 fired on ~100% of
ticks and overwrote the base entirely, and all 6 regressed. Zero of sixteen nodes improved their
own claimed metric while still doing the task.

**The control the project never had.** The hypothesis names *"an LLM asked plainly"* and no such
condition existed — the naive arm was retired and D is not it. Arm **`p`** (*plain*) is the base
edited with no card and no new information. ⚠️ **B − P is the corpus effect, not B − BASE**, and
the letter is `p` and never `n`, because `n` means sample size everywhere here.

```
BASE   frozen, k=1, already scored (0.50 undrifted / 0.40 at drift 0.10)
  |- p   "improve this navigator"                       P - BASE
  |- a   "...incorporating this principle" + A's card   A - P
  \- b   "...incorporating this principle" + B's card   B - P  = the corpus effect
                                                        B - A  = what EXTRACTION buys
```

**3 arms × 4 task families × k = 3 = 36 controllers, 36 generation calls, ZERO card calls.**
Every committed card is reused byte for byte: the independent variable and the execution model
must never move in the same step.

⚠️ **Nothing is deleted.** The node library, `compose/`, the v1/v2/v3 records and every
pass-rate denominator still load and still mean what they meant. Four superseded *plans* moved to
[`plans/trash/`](plans/trash/README.md), with a per-file reason. `plans/hold/` is empty.

## Required reading before using a subsystem

- [`generate/FINDINGS.md`](generate/FINDINGS.md) — before any generation. The harness **cannot
  detect card infidelity**, and temperature-0 determinism does not exist on any current model.
- [`compose/FINDINGS.md`](compose/FINDINGS.md) — a composition can succeed and produce a tree
  that does not work. ⚠️ Historical: composition is not on the live path.
- [`REPAIR-LOOP.md`](REPAIR-LOOP.md) — the verdict taxonomy, and **why a worse number is not a
  broken node**. Epic 07's case is assembled out of what this found.
- [`METRICS.md`](METRICS.md) — what the numbers mean, and the count-per-metre rule.
- [`PRE-REGISTRATION.md`](PRE-REGISTRATION.md) — ⚠️ **the document every Week 5 number is scored
  under. Amendments are DATED APPENDS, never edits.**
- [`cards/VERIFICATION.md`](cards/VERIFICATION.md) — before reporting any provenance number.

## Running things

Everything goes through the conda interpreter — never the system `python3`, which has no
`isaacsim`. ⚠️ **Every number that gets reported must come from a headless run inside the pinned
container** `nvcr.io/nvidia/isaac-sim:6.0.1`; a local run is stamped `in_container:false`.

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P -m pytest core/tests/ nodes/tests/ harness/tests/ cards/tests/ \
             generate/tests/ compose/tests/ runs/tests/ -q      # 565 tests, ~116 s
$P scripts/run_episode.py --scenario doorway_traverse --controller reactive_avoid
$P scripts/measure_growth.py --tier maze --n 20 --metrics       # does adding this help?
$P scripts/plot_kinematic.py --controller reactive_avoid        # SIM-FREE triage, ~1.5 s
```

⚠️ `~/.local/lib/python3.12/site-packages` precedes the conda env on `sys.path`, so anything
installed there shadows it. One more reason reported numbers come from the container.

## Layout

`core/` is the sim-free library everything is built on — scene spec, geometry, contracts,
metrics, stats, world generation. `controllers/` holds the reference baselines, `nodes/` and
`harness/` the (historical) leaf library and its gate, `cards/` the human principle corpus,
`generate/` the generation path, `compose/` the (historical) composer, `runs/` the evaluation
sweep, `scripts/` every entry point. `out/` is gitignored scratch; `out/evidence/` is not — see
[`out/README.md`](out/README.md).

The full annotated tree is in [`../CLAUDE.md`](../CLAUDE.md) under **Repo layout**.
