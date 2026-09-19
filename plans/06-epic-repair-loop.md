# Epic 06 — the autonomous diagnose-and-repair loop

**Opened 2026-09-16, on the user's instruction. Every other epic is on hold — `plans/hold/`.**

⚠️ **Read [`../REPAIR-LOOP.md`](../REPAIR-LOOP.md) first.** It holds the verdict taxonomy, the
repair action space, the arm-blindness decision and the traps. The chunks below reference those
definitions and do not repeat them.

---

## The goal of the core version, stated so it can be checked

> **One command takes one node id, and runs the loop end to end on one arm and one task until
> the node reaches a terminal verdict — with every round, every model call and every measurement
> recorded.**

Concretely, after chunk 58:

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P scripts/repair_node.py --node gen_b_44d12df4d25b437f --rounds 3
```

That is the whole deliverable. Not a library-wide sweep, not four conditions, not physics
confirmation as a gate. **One node, one arm, one task, autonomously, with an audit trail.**

## Why this is achievable quickly

Most of the substrate exists. The loop is an orchestration and judgement layer, not new
machinery:

| Need | Already built |
|---|---|
| Per-(node × world) outcomes | `scripts/measure_growth.py` → `<tier>-worlds.csv`, with `fixed`/`broke`/`both_ok`/`both_fail`, per-world `fire`/`changed` and measured difficulty |
| The full metric set, per node's own claim | `measure_growth.py --metrics`, which already marks each node's predicted metric and reports counts per metre |
| Evidence for one case | `scripts/debug_growth_case.py` → trajectories, timeline, mp4, README |
| Re-making the node call alone | `scripts/regenerate_modifiers.py` |
| Worlds as a pure function of (tier, seed) | `core/worldgen.py` |
| Arm-blind card projection | `generator.card_yaml` — mechanism/trigger/derivation/prediction/node_name only |
| Model calls, fresh session, committed transcripts | `generate/backends.py` |
| Physics on a generated world | `scripts/export_world.py` + `run_episode.py --controller layer:<id>` (chunk 53) |

What does not exist is the judgement: a verdict rule, a blind evidence bundle, a probe for
discriminating worlds, a diagnosis call, and a budgeted loop over the four.

## The chunks

| # | Chunk | Depends on | Model calls | Needs Isaac Sim | Status |
|---|---|---|---|---|---|
| 54 | [The verdict taxonomy as code](done/54-verdict-taxonomy.md) | — | **no** | no | **done** |
| 55 | [The arm-blind evidence bundle](done/55-evidence-bundle.md) | 54 | **no** | no | **done** |
| 56 | [The discriminating-world probe](done/56-environment-probe.md) | 54 | **no** | no | **done** |
| 57 | [The diagnosis call](done/57-diagnosis-call.md) | 55 | yes | no | **done** |
| 58 | [The loop](done/58-repair-loop.md) | 54–57 | yes | no | **done** |

**Three of the five need no model call at all**, which is deliberate: 54, 55 and 56 are pure
functions over data that already exists, they test under plain `pytest` in milliseconds, and
they are where the actual design risk lives. The same ordering argument as epic 00, where six of
eight chunks were sim-free.

⚠️ **54 comes first and nothing else is meaningful without it.** Every other chunk either feeds
the verdict rule or acts on its output. A loop built before the taxonomy would encode the
taxonomy implicitly in control flow, where it cannot be tested and cannot be reported.

## Deliberately NOT in the core

| Deferred | Why, and where it goes |
|---|---|
| `REGENERATE_CARD` | **On the user's instruction, 2026-09-16.** Documented as an extension in `REPAIR-LOOP.md`; needs its own chunk because a changed card is a new `plan_hash` and therefore a new row, never an edit |
| `REVISE_SHARED_PROMPT` | Structurally the fairest fix and the largest blast radius — every committed candidate becomes a prior-pipeline record |
| Physics as a loop gate | The loop runs kinematic (~35× faster). Physics confirms a terminal verdict, through commands that already exist |
| Library-wide sweep | One node first. The sweep is trivial once one node works and misleading before |
| Four conditions | Arm B only. The loop is arm-**blind**, which is what makes extending to A a configuration change rather than a redesign |
| The repair-budget constant | The mechanism ships in the core; the number is written down before it is used for any cross-arm comparison |

## The pre-registration position

⚠️ **Nothing in this epic may be reported under `PRE-REGISTRATION.md` as it stands.** The repair
loop is a second generation stage and it is not described there. Before any repaired node appears
in a cross-arm number, a **dated append** to the Amendments section must declare: that the loop
exists, that it is blind to condition, the per-node round budget, the repair/confirmation seed
split, and the consequence that the node prompt is identical in procedure rather than byte for
byte.

It is an append, never an edit. `prereg-v1` is tagged at `a84cd46`.

**This is not a blocker on building the loop.** It is a blocker on reporting from it, and it is
written here so that the first cross-arm run does not discover it.

## What success looks like for the epic

The loop, run on `gen_b_44d12df4d25b437f`, independently identifies at least one of the three
defects found by hand on 2026-09-16 — the monotone trigger latch, the `adjust`/gate arbitration
conflict, or the unchecked turn-symmetry condition — and takes a repair action that follows from
it.

⚠️ **A loop that terminates cleanly without finding any of them has failed**, however tidy its
output. That is the check this first target was chosen to make possible.

---

## ✅ The core is built — 2026-09-17

All five chunks are done. `scripts/repair_node.py --node <id> --rounds N` runs end to end to a
terminal verdict with every round, model call and measurement on disk.

**The criterion above is met, on round 1.** The loop independently identified the `adjust`/gate
arbitration conflict in full, the monotone trigger latch in substance (*"a self-locking loop"*),
and the `drift: None` environment mismatch — plus one defect the hand diagnosis missed: `_p_hat`
uses arc length where it needs net displacement, so the reconstructed position diverges at the
world's tortuosity ratio. The repair it wrote took round 1's `DEGENERATE` to round 2's `ACCEPT`.

⚠️ **Three findings that bear on what comes next, all in chunk 58's Outcome:**

1. **A whole run can be green and meaningless.** `entry_for` returned the always-present
   `ContractPorts` shim for every repaired node, so the loop measured the shim; the resulting
   `INERT`/`INERT`/oscillation/`UNREPAIRED` run looked entirely plausible. Fixed, pinned by a
   test, and the polluted round archived. **Before the library-wide sweep, assume any
   loop-produced number is an artefact until something outside the loop confirms it.**
2. **The lineage carries the latest node, never the best one.** Round 2 reached `ACCEPT` and the
   budget was then spent moving away from it. A cross-arm comparison built on "the node the loop
   ended with" is not the same experiment as one built on "the best node the loop found", and the
   two must not be mixed.
3. **Three of the five exits have never been produced by a real run** — `HELPED`, `UNTESTABLE`
   and `NOT_TRANSFERABLE`. `CONFIRM_SEEDS` has not been read once. The budget of 3 was never
   enough to reach a confirmation set.

---

## ⚠️ CORRECTION — 2026-09-18, after confirming round 1 from outside the loop

**A dated append, never an edit.** Two claims in the section above are retracted. The full
account, with every check and its sample size, is the **2026-09-18 CORRECTION** section of
[chunk 58's Outcome](done/58-repair-loop.md); the evidence and the reproducing commands are
`out/evidence/nodes/gen_b_44d12df4d25b437f__r01/README.md`.

- ⚠️ **"The repair it wrote took round 1's `DEGENERATE` to round 2's `ACCEPT`" is wrong.** Under
  the fixed classifier `__r01` is **`REGRESSED`** — it loses four worlds the base wins (seeds
  2007, 2009, 2011, 2014) and fixes none. The recorded `ACCEPT` was produced while
  `measure_live` built its `Measurement` with `metrics={}`, which made `DEGENERATE`, `REGRESSED`
  and `HELPED` unreachable for every repaired node. That bug is fixed; the run was never
  re-derived against the fix. **The loop reduced the regression (10 broke → 4), it did not
  remove it.**
- ⚠️ **Finding 2 above is worse than stated.** There was no `ACCEPT` to walk away from: **every
  round of the only valid run regressed or was inert.** "Keep the best" would have had nothing to
  keep. The problem is real and the illustration was not.
- ✅ **The epic's success criterion is UNAFFECTED** and the paragraph asserting it stands. It
  rests on the diagnosis transcripts, not on a verdict, and the loop did find all three
  hand-diagnosed defects plus one nobody had.
- ⚠️ **Finding 1 applied to this epic's own headline.** *"Assume any loop-produced number is an
  artefact until something outside the loop confirms it"* was written here on 2026-09-17, and the
  number that needed it most was in the table two paragraphs above it.

**Also found, and recorded in no document before now:** under physics the base does **not** solve
`gen-maze-02004`, the world this epic's motivating narrative calls *"a world the base alone
solves"*. It solves it kinematically. The growth pairing is internally consistent and the verdicts
stand for the suite they were measured on, but the prose claim is false under the frozen
locomotion policy — see `REPAIR-LOOP.md:15` and `plans/done/55-evidence-bundle.md:128`.
