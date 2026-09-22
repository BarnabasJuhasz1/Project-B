# 60 — The edit prompt, the `p` gate, and the generation path

**Epic:** [07](../07-epic-whole-controller.md) · **Depends on:** nothing · **Needs Isaac Sim:** no
**Status: PLANNED, NOT BUILT.** Written 2026-09-21. **Makes model calls.**

⚠️ **This chunk contains a gate that can stop epic 07 for three model calls.** Build the path,
run `p`, read the gate, and only then generate `a` and `b`. Do not batch all 36.

---

## What it builds

`generate/prompts/edit.md` — **one prompt for every arm**, fed the base navigator's source, a
task family, and a delimited card block that is **absent** for `p`. It asks for a complete
revised `core.observation.Controller`, not a patch file: a model that emits a unified diff will
get the hunks wrong, and the artefact that gets scored must be the artefact that was generated.
The diff is computed afterwards, by us, from the two sources.

`generate/edit.py` — `render_edit(cards, task)`. ⚠️ **No condition parameter**, for the reason
`render_node` has none: the moment it can tell the arms apart, the ablation has a second place to
differ. Pinned by two tests — one that the signature has no arm, and the strip-the-source-block
-and-diff test that `card_{a,b,d}.md` already uses.

`scripts/generate_edits.py` — walks (arm × task × sample), one call each, stages the result.
Resumes from staging and **never retries a rejection**: a pass rate needs a denominator. Backend
defaults to replay; `ClaudeCliBackend` runs a fresh session per call with `--setting-sources ''`
so this CLAUDE.md never reaches the model's context. Every transcript is committed — that is what
replaces temperature-0 determinism.

## ⚠️ `plan_hash` must take a sample index, and this lands first

`plan_hash` is a pure function of `(condition, task, card)`. With k = 3 all three samples of a row
collide on one stem. **That is exactly the collision that forced the v2 archive move in chunk 52**
— two experiments cannot share a row key, and neither can two samples of one row. Add the sample
index to the hash input before the first call, not after.

Staging goes to its own directory with its own `pipeline_version` (`v4-edited-base`). ⚠️ **No
pass rate and no growth table pools across it**, and `admit.pass_rate` must refuse to, the way it
already refuses to pool `v1-single-call`.

## The `p` gate

**Run arm `p` first: 4 tasks × k = 3 = 12 calls.** ⚠️ Reduce to one task family × k = 3 = **3
calls** if the budget is tight; the gate only needs to know whether the framing works at all.

`p` is *"here is a navigator, improve it"* with **no card and no new information**. It is the
control for the act of editing, which is itself a treatment — the same reasoning that made
principle articulation a confound in the old B−A.

**The gate:**

| what `p` does against the frozen base | verdict |
|---|---|
| Roughly level, or better | **Proceed.** The edit framing works. |
| Uniformly and substantially worse on every sample | ⚠️ **Stop and report.** The framing is broken — a model handed working code makes it worse — and epic 07's central hypothesis is refuted for 3 calls instead of 36. |
| Wildly variable — some samples far better, some far worse | **Proceed, and raise k.** The variance argument was right and k = 3 is too small. Say the new k out loud and declare it before scoring `a` or `b`. |

⚠️ **A `p` that beats the base is not a problem, it is a result.** It would mean a second pass
over its own output improves a model's navigation code without any literature at all — which is a
genuine finding about the generator and a genuine threat to reading B−BASE as a corpus effect.
It is precisely what `p` exists to detect, and it is why B−P, not B−BASE, is the headline.

## The ground-truth leak guard

⚠️ **This is the one new load-bearing guard, and it is the `PortView` problem in a new place.**

`Controller.reset(scenario, rng)` hands over the whole `Scenario` — `goal`, `start`, `seed`,
`scene`. `controllers/reactive_avoid.py:66` stores it wholesale, legitimately, because a human
wrote it. An **edited** controller that reads `scenario.goal` has perfect goal knowledge on a
goal-hidden scenario and the search family stops being a task; one that reads `scenario.seed` or
`scenario.id` is coupled to the scenario, which is what #4's "no coupling" clause made mechanical
for nodes.

The answer is the `PortView` answer: **a restricted view, enforced at `reset`, not a rule anyone
has to remember.** The generated class is wrapped so `reset` receives only the actuation and
sensing limits it legitimately needs. The runner and the reference controllers are unchanged.

⚠️ **The frozen base must pass this guard unmodified.** If it does not, the guard is wrong, not
the base. That is the first test to write and it costs nothing.

## Validity, and it is validity only

Same posture as chunk 37: **this gate decides whether the artefact is a controller, never whether
it is a good one.** Contribution is measured in 61, under pairing, on a shared suite.

1. **AST import allowlist** — static, and the load-bearing one. ⚠️ **This executes model-written
   Python with no sandbox**, and it is now a whole navigator rather than a leaf. The allowlist
   keeps `core.geometry`'s ground truth out and extends the seal to code the AST guard cannot see.
2. **`core.scenarios_held_out` is unreachable**, by the same AST guard that is what the seal is
   made of.
3. **It imports, constructs, and `reset`s** under the restricted view.
4. **It steps.** A short kinematic rollout completes without raising. A controller that raises on
   tick 3 is not a controller.
5. **It does not read ground truth**, per the guard above.

A rejection is **recorded with its reason and never retried**.

## The diff is a first-class artefact

For every accepted sample, commit the unified diff against `generate/base/base_navigator.py`
alongside the source and the transcript. ⚠️ **This is the readout the project has never had** —
chunk 61 reads it for fidelity, coverage-on-code and selectivity — and it is free here and
expensive to reconstruct later.

Record per sample: lines added, lines removed, which named methods were touched
(`_compute`, `_explore_heading`, `_wall_heading`, `_novelty`, `_enter_escape`, `_enter_wall`),
and whether the edit is a **no-op** in behaviour terms. A no-op is data, not a failure.

---

## Checks this chunk must run and record

- **Does `render_edit` have an arm parameter?** Must not. Test.
- **Does stripping the card block make the three arms' prompts byte-identical?** Test, as
  `card_{a,b,d}.md` already is.
- **Does the frozen base pass the ground-truth guard unmodified?** Must. Cheapest possible test of
  the guard.
- **Do k samples of one row get distinct stems?** Must, after the `plan_hash` change.
- **Does `admit.pass_rate` refuse to pool `v4-edited-base` with v1/v2/v3?** Must.
- **The `p` gate itself** — report all k samples' rates against the base, the spread, and the
  verdict. ⚠️ **Say `n` out loud.** k = 3 is three observations and a one-sample impression is
  not a verdict.
- **Not checked, and say so:** whether the edit is *faithful* to the card. That is read from the
  diff in 61 and is a human reading, not an automatic check. `generate/FINDINGS.md` already
  records that the harness cannot detect card infidelity, and nothing here changes it.

## What would make this chunk wrong

Generating `a` and `b` before the gate reports. Letting the prompt see the arm. Pooling a v4 pass
rate with any earlier pipeline. Shipping the guard without testing it against the base first.
