# 40 — The contract cannot express a modifier, and that is why B is inert

**Status:** todo. SIM-FREE. **This is the chunk that makes a FAIR A/B comparison
possible.** 39 gets numbers; 40 gets numbers that mean what a reader will assume.
**Blocked by:** nothing.

⚠️ **SUPERSEDED 2026-09-14 by [52 — a layered command pipeline](52-layered-command-pipeline.md).** The measurement below stands and is carried
forward into 52. The RECOMMENDATION below (option 2, a consuming Action) is REFUTED:
growth round 1 showed the problem is not only orphaned ports but that exclusive
control is the wrong execution model, and option 2 requires the consumer to know
about each modifier in advance — which cannot support an open-ended growth loop.
**Kept, not deleted: this is where the diagnosis was first made.**

---

## Read this first if you are a fresh session

`P=~/miniforge3/envs/env_isaacsim/bin/python`, run from `Repo/`. Read `CLAUDE.md`,
`compose/FINDINGS.md` (findings 1 and 2 especially), and the Outcomes of
`plans/doing/37` and `plans/doing/38`.

## The problem, measured

**No generated node in the library declares a single IN port.** Fourteen
generated nodes, and the only IN port anywhere is `goal_known`, read by three
toys. Every published output — `speed_limit`, `safe_speed`, `goal_bearing`,
`pi_pose`, `reorient_axis`, `ics_safe_speed`, `home_bearing` — is **orphaned**:
computed, written to the `PortStore`, cleared unread.

They are dead rather than merely unwired, and the reason is structural:

> **The node contract has no way to express a modifier.**

A speed cap is not a `Condition` (it must affect the Twist), not an `Action` (it
does not choose where to go), and not a `Decorator` as defined — `nodes/contract.py`
says a Decorator "wraps one child and modifies its status or its rate. **Never
emits a Twist of its own** — it governs whether the child's happens." Governing
*whether* is not rewriting *what*.

So every B mechanism was forced into an `Action` with a carrier welded on. That
single cause produces all three recorded symptoms:

- the **two-effect violation** (`compose/FINDINGS.md` §2) — now **7 of 7**
  generated nodes, not the two originally recorded;
- the **orphan port** — the node consumed its own computation, so nobody needs it;
- the **starvation** (`compose/FINDINGS.md` §1) — an Action that drives cannot
  sit in a `Sequence` with another driving Action.

And it is why, in every composition taken since 2026-08-28, the model **omitted
the generated node outright**, each time reasoning correctly that sequencing it
ahead of a driving Action *"would not shape that action's motion, it would just
consume ticks and starve it."* The composed condition B is toys only.

## The decision this chunk exists to take

**Can an Action's Twist be modified by another node, and if so how?** Three
options; the chunk must pick one and record why.

1. **A fourth leaf kind — `Modifier`.** Wraps a child like a Decorator but may
   rewrite the Twist the child emitted. Most expressive; changes the closed leaf
   vocabulary, which is a contract change of the same weight as chunk 11a's, and
   `nodes/contract.py` says adding a port type is a chunk — adding a KIND is more.
2. **A consuming Action.** Keep three kinds; require that the driving Action
   *reads* `speed_limit` as an IN port and applies it itself. No contract change,
   and it makes the port live. But it pushes the mechanism into the consumer,
   so the literature-derived node becomes advisory and the carrier decides
   whether to obey — which is a fidelity problem of its own and must be said.
3. **Let a Decorator rewrite the Twist.** Smallest edit, but it contradicts the
   documented meaning of Decorator, and that docstring is load-bearing elsewhere.

**Recommendation: 2, then 1 if 2 proves too weak.** Option 2 needs no contract
change, can be tested immediately with existing nodes, and would let chunk 39's
"carried" framing become a genuinely fair slot: the same consuming Action in both
arms, the A node and the B node swapped in as the only difference.

⚠️ Whichever is picked, **it invalidates nodes.** Enforcing "an Action that
drives may not also publish a port" would invalidate **all seven** generated
nodes measured on 2026-09-07 — not the two `compose/FINDINGS.md` records. Under
chunk 37's validity gate they stay *admitted* (they still run); what changes is
whether they can be composed. Decide explicitly whether existing nodes are
grandfathered, and record it — do not let it be discovered.

## Build

1. The decision, written into `nodes/contract.py` as a docstring section, with
   the two rejected options and why.
2. The mechanism, whichever it is, plus tests.
3. **One worked pair**: an A node and a B node from the same card, in the same
   tree slot, with the same consumer, differing by nothing else. That is the
   artefact chunk 39 cannot produce and the one a reader will ask for.
4. Re-run `scripts/node_contribution.py` on the resulting tree — currently it is
   only meaningful on a three-toy composition, because the generated nodes reach
   no tree.

## Done when

- At least one generated OUT port is read by another node in a composed tree.
- A fair A/B pair exists: same slot, same consumer, one node different.
- The grandfathering decision is recorded in `nodes/contract.py` and in
  `compose/FINDINGS.md`.

## Not in scope

- Generation calls, prompt changes, new cards.
- The sealed split.
- Physics — chunk 36.
