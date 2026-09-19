> ⚠️ **ON HOLD since 2026-09-16, on the user's instruction.** All work is paused while
> the autonomous diagnose-and-repair loop is built — see [`../../REPAIR-LOOP.md`](../../REPAIR-LOOP.md)
> and [epic 06](../06-epic-repair-loop.md). Nothing here is cancelled or superseded; this file is
> intact and resumes where it stopped. Move it back to `plans/doing/` to restart.

# 52 — Nodes that support the base instead of replacing it: a layered command pipeline

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** 47 · **Needs Isaac Sim:** later
**Status: DOING — built 2026-09-14, user's design.** SIM-FREE.

⚠️ **ABSORBS AND SUPERSEDES [40 — The contract cannot express a modifier](40-modifier-kind.md).**
40's measurement stands and is carried forward below. Its *recommendation* is refuted by
evidence 40 did not have. 40 is not deleted; it is marked superseded and kept, because it is
where the diagnosis was first made.

---

## What chunk 40 established, and it was right

> **No generated node in the library declares a single IN port.** Fourteen generated nodes, and
> the only IN port anywhere is `goal_known`, read by three toys. Every published output —
> `speed_limit`, `safe_speed`, `goal_bearing`, `pi_pose`, `reorient_axis`, `ics_safe_speed`,
> `home_bearing` — is **orphaned**: computed, written to the `PortStore`, cleared unread.

The cause it identified:

> **The node contract has no way to express a modifier.**

A speed cap is not a `Condition` (it must affect the Twist), not an `Action` (it does not choose
where to go), and not a `Decorator` — that "wraps one child and modifies its status or its rate.
**Never emits a Twist of its own**". Governing *whether* is not rewriting *what*.

Every corpus mechanism was therefore forced into an `Action` with a carrier welded on, which
produced the two-effect violation, the orphan ports, and the starvation.

## What is new since 40, and why its recommendation no longer holds

**Growth round 1, 2026-09-14** (`generate/FINDINGS.md`): each admitted node placed in
`Fallback[specialist, BASE]` against the base alone, 20 `maze` worlds, paired.

- **Not one of eleven nodes improved the tree. `fixed` = 0 in every row, both arms.**
- Precondition fire rate inside the tree is **bimodal with nothing between**: five nodes at
  0.0%, six at 87.6-100%.
- At 100% the specialist takes first refusal every tick and **the base is never ticked at all** —
  its precondition is not even evaluated. The tree scores what the specialist scores alone.
- **B−A is exactly zero on every column.**

So the problem is not only that ports are orphaned. It is that **exclusive control is the wrong
execution model for what the corpus supplies.** These mechanisms are adjustments to an ongoing
behaviour; a behaviour tree hands one node the whole Twist and silences the rest.

**This refutes 40's recommended option 2 ("a consuming Action" — the driving Action reads
`speed_limit` and applies it itself).** Under option 2 the consumer must know about the modifier
*in advance*. The base is a single model-written navigator and would have to be regenerated to
consume every port any future node might publish — which is precisely the open-ended growth loop
chunk 44 needs, made impossible. Option 2 works for one hand-picked pair and does not scale to
"add another node next round".

## ⚠️ And one structural fact that rules out the obvious cheap fix

There is **no composite that ticks everybody**:

    Sequence: if status is not Status.SUCCESS: return status   # stops on RUNNING
    Fallback: if status is not Status.FAILURE: return status   # stops on SUCCESS/RUNNING

A base returning `RUNNING` stops a `Sequence` dead. "Put the base first and let later nodes
override" is therefore **not** a reinterpretation of the existing tree — it needs a new
composite whatever else is decided.

`drive()` is also a one-shot write, and `contract.py` says **"The Twist is NOT a port: only an
Action may drive()."** A later node cannot read what an earlier one proposed, so "cap the speed,
leave the heading alone" is currently inexpressible even if everyone ticked.

## The decision

**A layered command pipeline with field-scoped modifiers.**

1. **A new composite that ticks every child in order**, regardless of status. Working name
   `Layer`. Earlier children are the base behaviour; later children are higher priority.
2. **The pending Twist becomes readable state** on the `PortStore`, not a one-shot write. A node
   may read what has been proposed so far.
3. **A modifier declares which FIELDS of the Twist it may write** — `vx`, `vy`, `wz` — and the
   `PortView` refuses the others. A speed limiter that declares `vx` *structurally cannot*
   rewrite `wz`.

Point 3 is the load-bearing one. It makes "supports the base rather than replacing it"
**enforced rather than hoped for**, which is the same move as the `PortView` restriction that
makes "no coupling to a scenario" mechanical rather than conventional (#4). It also keeps nodes
small by construction: a block that may only touch `vx` cannot grow into a navigator.

This is subsumption architecture, which is worth saying in the write-up because it is itself
literature rather than an invention of ours.

### Rejected, with reasons

- **40's option 1, a fourth `Modifier` KIND wrapping a child.** Still one-child, so it inherits
  the tree's exclusivity; and adding a KIND is a heavier contract change than adding a composite.
- **40's option 2, a consuming Action.** Refuted above: it does not scale to open-ended growth.
- **40's option 3, let a `Decorator` rewrite the Twist.** Contradicts a load-bearing docstring.
- **Decomposing the base into small base nodes** (considered 2026-09-14). Attractive for the
  science, but the base is 538 lines of interlocking state — commitment timers, wall-following
  mode, a visit grid — and splitting it while *also* changing the execution model would leave no
  way to attribute a result to either change. **Revisit after the pipeline works, not before.**

## ⚠️ The threat this introduces, declared before building

**The arbitration rule becomes part of the experiment.** Whoever decides how conflicting
proposals combine is injecting design that came from neither arm's corpus, and B−A would then
partly measure *our* arbiter.

Mitigations, all three required:

- The combination rule is **minimal and fixed**: per-field last-writer-wins, and `min()` for
  anything declared a cap. No scoring, no weights, no tuning.
- It is **identical across arms** and stated in the write-up.
- It is **declared in `PRE-REGISTRATION.md` before the first measurement**, not after.

⚠️ **Layer ORDER is also a researcher degree of freedom.** With priority by position, the
composition's ordering decision is load-bearing. It must be produced the same way for every arm
— by the LLM composer under an identical prompt — and never hand-tuned per arm.

## Grandfathering — the decision 40 asked for and did not take

**The existing 11 nodes are NOT grandfathered.** They are Actions that drive; under this
contract they must be regenerated as modifiers. Cost: a new prompt plus ~22 model calls.

That is cheap relative to what it buys, and today's evidence says almost nothing is lost:
**0 of 11 could be composed with a base at all.** They remain in `generate/staging/` as v2
records with their pass-rate denominators intact — this changes what can be *composed*, not what
was *admitted*, exactly as chunk 37's validity gate intends.

## Build

1. The decision written into `nodes/contract.py` as a docstring section, with the four rejected
   options and why — 40's three plus base decomposition.
2. `Layer` composite + a readable pending command + field-scoped write permission, with tests.
   Tests must include: a modifier cannot write a field it did not declare; a modifier that
   declines leaves the base's command byte-identical.
3. A modifier prompt, and the node contract section rewritten for it.
4. **One worked pair**: an A node and a B node from the same card, in the same layer slot, above
   the same base, differing by nothing else. This is the artefact 40 asked for and 39 could not
   produce.
5. Re-run `scripts/measure_growth.py`. The `fixed` column is the number that has to move.

## Done when

- At least one generated OUT port is read by another node in a composed tree.
- A modifier measurably changes the base's Twist **without replacing it** — base drives, modifier
  adjusts, both tick.
- `measure_growth.py` shows a non-zero `fixed` for at least one node.
- The grandfathering decision and the arbitration rule are recorded in `nodes/contract.py`,
  `compose/FINDINGS.md` and `PRE-REGISTRATION.md`.

## Not in scope

The sealed split. Physics (chunk 36). Decomposing the base. Moving obstacles (48/49).


---

# Outcome (2026-09-14)

## What was built

| | where |
|---|---|
| `Layer` — ticks every child, position is priority | `nodes/tree.py` |
| `WRITES`, `is_modifier`, validated at construction | `nodes/contract.py` |
| readable pending command, `cap()` / `adjust()`, field scope | `nodes/contract.py:PortView` |
| the decision + **four** rejected options | `nodes/contract.py` module docstring |
| 14 tests | `nodes/tests/test_layer.py` |
| the modifier prompt | `generate/prompts/modifier.md` |
| re-make the node call only | `scripts/regenerate_modifiers.py` |
| `Layer[ports, BASE, node]` in the growth loop, + `fire` / `changed` columns | `scripts/measure_growth.py` |
| the arbitration rule, declared before the first measurement | `PRE-REGISTRATION.md`, 2026-09-14 |
| what the change obsoleted, and a gate on deleting it | `DELETION-CANDIDATES.md`, `scripts/prune_obsolete.py` |

## What differed from the plan

**1. The v2 records had to be ARCHIVED, not left in `generate/staging/`.** The plan said they
would stay. They could not: `plan_hash` is a pure function of (condition, task, card), the cards
are deliberately *not* regenerated, so every v3 candidate lands on the identical stem. Two
experiments cannot share a row key, and `batch.plan()` would have read a v2 rejection as a
verdict on a v3 candidate — "a rejection is never retried" is the rule G1's denominator depends
on. They are in `generate/archive/v2-exclusive-control/`, intact, denominators unchanged, under
the same rule v1 lives by.

**2. No fourth `Kind`, and the plan left that open.** A modifier is an `Action` with a narrowed
`WRITES`. Recorded in `contract.py` as a decision rather than left implicit.

**3. `micro()` is no longer requested at all.** The plan did not say this. It follows from what
the chunk already decided: a modifier cannot drive alone, so a micro-world it wrote around a
base it was never shown is evidence about nothing. `load_candidate` still accepts one and still
measures a candidate that supplies it; nothing was thrown away.

**4. The base was NOT regenerated.** It is a full driver, valid under the new contract
unchanged, and frozen. One fewer call, and every growth number stays relative to the same node.

## Checks

| check | outcome | verdict |
|---|---|---|
| `Sequence`/`Fallback` cannot tick everybody | both stop at the first child, asserted in `test_layer.py` | confirmed — the new composite was necessary, not a preference |
| a modifier cannot write an undeclared field | `CommandFieldError`, n=1 fixture + 2 real generated nodes | confirmed |
| a declining modifier leaves the command byte-identical | object **identity**, not float equality, n=2 tests | confirmed |
| a cap that does not bite leaves it byte-identical too | identity, n=1 | confirmed |
| two caps commute | n=1, both orders | confirmed |
| every pre-chunk-52 `Action` stays valid | `Action.WRITES` defaults to all three; 883-test suite passes | confirmed |
| ⚠️ **gating the tree shape on `is_modifier` was WRONG** | 2 generated nodes declared all three fields and then *revised* the pending command with `cap`/`adjust`; they got the baseless tree and were rejected for the harness's shape | **ruled out** — every candidate now gets the base. Whether a node drives or revises is a fact about its `tick`, not its declaration. |
| ⚠️ **the row key for the paper arm is `key`, not `prompt_hash`** | six B candidates staged under stems their own cards did not share; `admit_staged` failed on a missing card file | fixed in `node_call`; the six were re-staged from their transcripts by **replay**, at no cost |
| 5 of 16 calls returned `terminal_reason: api_error` | 0 tokens, $0, at 5-way concurrency; all succeeded on retry at 2-way | transient, not a prompt problem. Recorded as `.error.json` and retried behind an explicit `--retry-failed`. |
| growth harness end to end under `Layer` | smoke run, 2 nodes x 5 maze seeds, base 1/5 | works; `fire=95% chg=0%` on one node is the diagnostic round 1 could not produce |

**Not checked:** anything under physics. Everything here is kinematic and sim-free, and
`measure_growth.py` says so first.

## ⚠️ Round 2 was scored on the wrong metric for 6 of 16 nodes (2026-09-15)

The table below reports `goal_reached`. **Six of the sixteen nodes predict `collisions`** — the
whole speed-limiter family — so "+0%, does nothing" was a verdict on a claim they never made.
`measure_growth.py --metrics` now scores the full `core.metrics.score()` set and marks each
node's own claimed metric.

Re-scored on the same 20 worlds: **3 nodes improved their own claimed metric, 5 made it worse,
8 did not move it.** But two of the three wins are a robot that stopped —
`gen_b_c2c95bcb965e342e` cut collisions 0.40 → 0.30 while walking 6.2 m instead of 30.0 and
reaching the goal in **0 of 20** worlds; **per metre it collides 3.6x more than the base**, and
the win reverses. So **zero of sixteen improved their claim while still doing the task**.

⚠️ **Methodological finding worth more than the round:** a lower-is-better COUNT metric rewards
a robot that stops, so `collisions`, `near_misses` and `stuck_events` are now reported **per
metre travelled** beside the raw count, with a flag when the two disagree. This bears on
`PRE-REGISTRATION.md` §5's named secondaries and is raised there, not settled.

## Against the chunk's own "Done when"

| criterion | status |
|---|---|
| a generated OUT port is read by another node in a composed tree | ⚠️ **NOT MET** — see below |
| a modifier changes the base's Twist **without replacing it** | **met** — `test_layer.py` proves the mechanism; the growth run's `changed` column measures it per node |
| `measure_growth.py` shows a non-zero `fixed` | see below |
| grandfathering + arbitration recorded in `contract.py`, `compose/FINDINGS.md`, `PRE-REGISTRATION.md` | **met** |

⚠️ **The port criterion is not met, and the reason is structural rather than a
shortfall.** Chunk 40 measured that *no* generated node declared a single IN port. Four of
sixteen now do — but all four read the harness's own restatements of `Observation`
(`goal_known`, `ranges`), **not another generated node's output**. Every node is written from
one card, in isolation, having never seen the library; it has nothing to read. Port-to-port
composition is a **composition** problem — it needs the composer, or a retrieval-aware node
prompt — and this chunk's contract change only makes it *possible*, not likely. Carry it
forward rather than claiming it.

⚠️ **And "keeps nodes small by construction" is only half-delivered.** 13 of 16 nodes narrowed
`WRITES`: 11 to `("vx", "wz")`, 2 to `("vx", "vy")`. Three declared all three fields.
**None declared a single field.** A node that may only touch `vx` is the shape the design
argument rests on, and the library has not produced one.

## The number that decides whether this was worth it — and it moved

`fixed` in `scripts/measure_growth.py`. **0 in every row of round 1**, eleven nodes, both arms.

Round 2, `Layer[ports, BASE, node]`, 16 nodes, 20 maze worlds, base 10/20:

| | nodes | improved | regressed | net-zero | `fixed` |
|---|---|---|---|---|---|
| A | 5 | 0 | 3 | 2 | **0** |
| B | 10 | 0 | 4 | 6 | **0** |
| D | 1 | **1** | 0 | 0 | **2** |

`gen_d_88c49cee31e01477` (`SteerThroughWidestAdmissibleGap`) scored 12/20 against the base's
10/20 — **2 fixed, 0 broken** — and **replicated on twenty fresh worlds** (seeds 3000-3019:
base 8/20, pair 12/20, 4 fixed, 0 broken). Pooled: **18/40 → 24/40, six fixed, none broken.**

**So the layered pipeline is no longer unfalsified**, the gate in `scripts/prune_obsolete.py`
passes, and chunk 52's third "Done when" is met.

⚠️ **And almost everything else in the chunk's thesis is not yet earned.** Fourteen of sixteen
nodes contribute nothing or actively harm; `fixed` is **0 in both corpus arms**; **B−A is still
exactly zero**; the single hit is in the arm handed no corpus at all, at n = 1 node against 15.

The `fire`/`changed` columns say why, and this is the finding to carry forward: the library
splits into *never fires* (4), *fires and changes nothing* (3), *fires ~100% and changes ~100%*
(6, **all of which regress**), and exactly one node that fires 62% and changes 57%. **Selectivity
is the variable**, the contract now makes it visible and safe, and nothing in the generator
selects for it. That is the next chunk's question — not "does the corpus help".
