# 17 — Pre-registration

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** 10, 11a, 11 · **Needs Isaac Sim:** no
**Blocks:** the first evaluation run. Nothing in Week 5 starts until this is committed and tagged.

## Why

`CLAUDE.md` marks four things ⚠️ **not yet fixed**: G2's decision threshold X, the seed set, the
episode count, and the secondary metric list. `results/README.md` adds that the metric constants
in `core/metrics.py` are pre-registration material and are not frozen either. Nothing owns any of
them.

Meanwhile the decisions are being made anyway, one per chunk, and landing in six different
`done/` Outcome sections: chunk [10](../done/10-event-debounce-tolerance.md) fixes which way an event
boundary falls, [11a](../done/11a-task-contract.md) fixes `DETECT_RADIUS` and the disclosure semantics,
[11](../done/11-held-out-split.md) fixes the split contents and count, [13](../done/13-unit-harness.md) fixes the
effect-size threshold for admission, [14](../done/14-first-cards.md) fixes the confidence scale.
Collected as they are made, this document is a byproduct. Reconstructed in Week 5 from six
Outcome sections, it is archaeology — and the entire evidential value of a pre-registration is
that it **demonstrably predates the first run**. A document written afterwards, however honest,
is not one.

This is also the chunk that makes G2's decision rule a rule rather than an aspiration. "B must
exceed D by at least X points with non-overlapping bootstrap confidence intervals" is not a
falsifiable claim until X is a number.

## Decide first

### X and the episode count are the same decision

They are usually treated as independent and they are not. Non-overlapping 95% CIs on two
proportions requires roughly

```
|p_B - p_D|  >  1.96 * (SE_B + SE_D)      SE = sqrt(p(1-p)/N)
```

which at the worst case p ≈ 0.5 is `1.96 / sqrt(N)`. So the episode count **per condition**
dictates the smallest X the rule can ever detect:

| N per condition | smallest detectable X | e.g. |
|---|---|---|
| 36 | **33 points** | 12 held-out × 3 seeds |
| 60 | 25 points | 12 × 5 |
| 120 | 18 points | 12 × 10, or 15 × 8 |
| 240 | 13 points | 12 × 20 |
| 480 | 9 points | 12 × 40 |

Choosing X = 10 with N = 36 pre-registers a rule that **cannot fire even if the effect is real**,
and that is a worse outcome than a null result because it is indistinguishable from one. Pick the
episode count first, read X off the table, and only then ask whether an effect of that size is
one worth claiming.

The counterweight is wall clock. Headless with `--no-record`, a run costs about 1.4× sim time —
`CLAUDE.md` measures a 15 s episode at 21 s — so a 120 s timeout episode is roughly 170 s and a
successful traverse rather less. At N = 120 across 4 conditions that is 480 episodes, plausibly
**8–12 hours** of container time per full evaluation sweep, and there will be more than one
sweep. Measure the real per-episode cost on the dev set before committing, rather than trusting
that ratio.

### Which scenarios does G2 score over?

G2 is the four-condition ablation; G3 is compositional generalisation on the held-out set. It is
not stated whether G2's primary metric is computed over the dev set, the held-out set, or both.
They give different readings: dev-only measures the pipeline where the prompts were iterated;
held-out-only conflates G2 with G3. **Recommend both, reported separately, with the held-out
figure as the headline** — and write that down, because "we report the one that worked" is the
exact failure a pre-registration exists to prevent.

### The secondary metric list

`score()` returns 18 keys today. Reporting all of them and highlighting the significant ones is
multiple comparisons by another name. Recommend naming **four or five** secondaries in advance —
SPL, collisions, stuck events, time to goal — and reporting the rest as descriptive only, marked
as such.

Two constraints from earlier chunks that must be carried in:

- **Per-family, never pooled.** [11a](../done/11a-task-contract.md) records that `spl`'s denominator is
  the omniscient taut path, so it is near-meaningless for `search_unmapped`. Pooling SPL across
  four families produces a number that means nothing in particular.
- **Three metrics are still on unsettled foundations.** `FALL_HEIGHT` is a height where a fall is
  a tilt; `contact` is a geometric proxy with no physics contact sensor wired; event counts sat
  on a float knife edge until chunk 10. Any of the three may be reported, but the caveat travels
  with the number.

### The seed set

Fixed integers, written down, identical across all four conditions (#10). Recommend deriving them
from a single stated base rather than listing arbitrary values, so that "did anyone add a seed
after seeing the results" is answerable by inspection.

## Already decided — collect these, do not re-open

| Decision | Where it was made |
|---|---|
| Event span closes on a gap of exactly `EVENT_RELEASE`; tolerance `- 1e-9` | [10](../done/10-event-debounce-tolerance.md) |
| Held-out split varies routes **and task families** (option B); 12 scenarios, 3 per family | [11](../done/11-held-out-split.md) |
| `Observation.goal` is `Optional`; `None` = not disclosed; detection latches on line of sight | [11a](../done/11a-task-contract.md) |
| `DETECT_RADIUS = 2.0` m | [11a](../done/11a-task-contract.md) |
| Condition A **is** given the node contract; B−A therefore isolates the corpus alone | [15](15-generator.md) |
| Legibility is **out of scope this cycle**; no legibility metric is defined or reported | epic [02](../02-epic-node-library.md) |
| Area-restricted search is **not testable** without a cue field or multiple targets | [11a](../done/11a-task-contract.md) |

## Build

- `PRE-REGISTRATION.md` at the repo root. Sections: primary metric and decision rule (X, stated
  as a number); the seed set; the episode count and which scenarios it covers; the named
  secondaries; every frozen constant in `core/metrics.py`, `core/scenario.py` and `core/geometry.py`
  with its value and one line of rationale; what is explicitly *not* reported and why.
- The bootstrap procedure spelled out concretely enough to be reimplemented: resampling unit
  (episode? scenario? scenario-then-seed?), number of resamples, interval type, RNG seed.
  Resampling episodes when episodes are clustered within scenarios understates the interval;
  say which you do.
- `core/tests/test_preregistration.py` — hashes the frozen constants and compares against a value
  recorded in the document. **Changing a constant without editing the document is a failing
  test.** That is the enforcement mechanism; without it the document is a comment.
- `git tag prereg-v1` on the commit, before any evaluation run. Named in `results/README.md`.
- `core/manifest.py` — record the tag alongside `library_snapshot` and `held_out_sha256`, so a
  result proves which pre-registration it ran under.

## Done when

- X is a number, and the table above shows it is detectable at the chosen episode count.
- Every ⚠️ in `CLAUDE.md` and every item under *Open before evaluation* in `results/README.md`
  is either resolved in the document or explicitly deferred **with a reason**.
- `test_preregistration.py` fails when any frozen constant is edited, demonstrated by editing one.
- The tag exists and precedes every commit under `results/` produced from Week 5 onward.

## Not in scope

Running anything. Amending the document after the first evaluation run — an amendment is allowed,
but it is a **dated append**, never an edit, and the write-up reports both versions. Say that in
the document itself.

---

## Outcome

**Done 2026-08-26.** `PRE-REGISTRATION.md` is at the repo root, `core/tests/` +
`nodes/tests/` + `harness/tests/` + `cards/tests/` + `generate/tests/` + `compose/tests/` =
414 → **437 passed**, no simulator, ~112 s. `results/` untouched — `rescore.py --dry-run`
reports 0 of 6 changed.

Built: `PRE-REGISTRATION.md`, `core/stats.py` (the decision rule as code),
`core/preregistration.py` (the frozen-constant registry and its digest),
`core/tests/test_preregistration.py` (10 tests), `core/tests/test_stats.py` (12 tests),
`scripts/power_analysis.py`, `scripts/check_seed_offsets.py`, and two new manifest fields.

**`git tag prereg-v1` is NOT applied**, for the same reason `split-v1` is not: `core/`,
`plans/` and `results/` are still untracked and the working agreement is not to commit
unless asked. This is the one Done-when clause left open, and it is the one that blocks
Week 5. Tag the commit that first tracks `PRE-REGISTRATION.md`, then re-run
`pytest core/tests/test_preregistration.py` and confirm `prereg_tag` stops being `null`.

### The plan's central arithmetic was wrong, and in the unsafe direction

The plan's *Decide first* section is built on `X > 1.96/sqrt(N)` for two independent
proportions at the episode level, and concludes that N = 36 detects a 33-point gap. **Neither
the estimator nor the sample size in that formula matches this experiment.** Two facts, the
second of which nobody had noticed:

1. Episodes are **clustered** within scenarios, so the effective sample size is nearer the
   scenario count than the episode count.
2. **On the six undrifted held-out scenarios the replicate seeds are the same episode.**
   `Scenario.seed` reaches exactly two places — `rng()`, handed to the controller, and
   `drift_rng()`. A controller that does not draw from its rng, on a scenario with no drift,
   produces a **bit-identical** rollout at every seed. Measured on `reactive_avoid`,
   `goal_seeker` and `toy_search_tree`: one trace hash across seeds 1002 / 7 / 999 with drift
   off, three with drift on.

So the bootstrap resamples **scenarios**, the effective N is **12**, and `scripts/power_analysis.py`
— which simulates the real estimator rather than substituting a closed form — puts the rule at:

| true B − D gap | disjoint intervals (the rule) | paired difference CI excludes 0 |
|---|---|---|
| 0.20 | 0.04 | 0.23 |
| 0.30 | 0.16 | 0.51 |
| 0.40 | 0.40 | 0.74 |
| 0.50 | 0.75 | 0.93 |

**The episode count is nearly irrelevant and the scenario count is everything.** 3 seeds → 10
seeds moves power a few points and costs 3.3× the wall clock. The one lever that would help is
more scenarios, and the split is sealed.

That inverts the plan's own advice. It says "pick the episode count first, read X off the
table". The right order here is: recognise the scenario count is fixed at 12, accept that the
interval criterion — not X — is what binds, and pick X as the largest floor that does not bind
*before* the intervals do. **X = 0.30.**

### The consequence is pre-registered rather than left to be discovered

The rule is expected **not** to fire on a genuine 20-point corpus advantage. That could have
been discovered in Week 5 and written up as a null. Instead the document states, in advance,
that a non-firing rule **may not be reported as "the corpus does not help"**, pre-registers the
paired difference CI as a reported secondary, and fixes the reading of all four
fires/doesn't-fire combinations before any of them exist. That is the honest form of an
underpowered design: say so first.

### The seed rule turned into a manipulation check

`SEED_OFFSETS = (0, 10000, 30000)`, applied to each scenario's own sealed seed so that
**replicate 0 is exactly the episode chunk 11 sealed and chunk 13 verified**. 20000 is skipped:
at that offset one of the six drifted held-out scenarios never crosses `LOC_FAIL_THRESHOLD`
inside its time limit, so drift would be nominally on and functionally absent — the treatment
would not have been applied, and the episode would silently become a fourth `navigate_to_goal`.

The offsets are defined by a **stated rule** — the first three multiples of 10000 at which every
drifted held-out scenario crosses — rather than picked, and the rule cannot be seed-shopping
because drift is open-loop and bit-identical across conditions (chunk 14's finding is what makes
that argument available). `scripts/check_seed_offsets.py --scan 5` is the rule, run, and it
prints shape only so its output is safe to paste into a commit message.

### Enforcement has two halves, and only the second has teeth

`core/preregistration.py` checks (1) that the digest recorded in the document matches the digest
of the live values, and (2) that **every recorded value matches the live value, name by name**.
(1) alone is satisfiable by pasting in a new hash. (2) forces whoever moves a constant to write
the new number into the document, next to the rationale, where a reader will see it.

Demonstrated by editing `GOAL_RADIUS` 0.5 → 0.55 in the real file, not only under monkeypatch:
two tests fail, one naming the constant and both values, one on the digest. Reverted.

### Four questions the plan asked other chunks to hand over, answered

- **B−A is confounded** (`CLAUDE.md`'s open ⚠️). Resolved by **softening the claim, not changing
  the prompts** — changing them would orphan every committed transcript and staged candidate,
  which is a larger loss than the confound, and B−D was already the plan's own top-ranked
  contrast. Only B−D isolates the corpus.
- **Card fidelity** (`generate/FINDINGS.md` §1). Declared a **stated limitation absorbed by
  B−D**, not a pre-registered fidelity check: a check needs a human rater study, and the
  threat is conservative with respect to B−D — if B's nodes are model-invented mechanisms
  wearing card names, B and D draw on the same source and the gap shrinks.
- **May a driving Action also publish a port?** (`compose/FINDINGS.md` §2). **No** — that is a
  second effect. Enforcement re-opens admission for every existing node, so it is a named
  follow-up chunk; the existing violation is **reported as a known defect, not hidden**.
- **`localisation_failures`** (chunk 14). A **manipulation check**, never a performance
  secondary, stated in §5.

### What the document deliberately does not resolve

Eight items are listed as deferred **with reasons** rather than silently omitted: a tilt-based
fall criterion, a physics contact sensor, SLAM, the effect-multiplicity check, a card-fidelity
rater study, verifying the five card spans, a fifth condition, and `split-v1`. Each names why.
The two metric caveats that cannot be fixed this cycle — `FALL_HEIGHT` is a height where a fall
is a tilt, and `contact` is a geometric proxy — are pre-registered to **travel with every figure**
rather than the metrics being withheld.

### Costs, measured rather than assumed

From the committed manifests, headless with `--no-record` on the local RTX 4060:
`return_home__goal_seeker` 160.8 s wall for 120.0 s sim (**1.34×**) and
`blind_corner__reactive_avoid` 9.0 s wall for 6.82 s sim (**1.32×**). Worst case for a full
four-condition sweep at 3 seeds: ≈ 7.3 h of sim, ≈ **9.8 h of wall clock**, and real sweeps will
be well under that. The document says to **re-measure on the benchmark machine** before the
first sweep, because a laptop GPU is explicitly not the target.

### For whoever runs Week 5

`core.stats.decide()` is the rule. Call it; do not reimplement it. It raises rather than
computing when the two conditions do not cover the same scenarios, which is #10 enforced
instead of remembered. `core.stats.seeds_for(scenario.seed)` is the seed set.
