# 38 — Node contribution: leave-one-out ablation in a tree, over real scenarios

**Status:** todo. SIM-FREE for the sweep; chunk 36 confirms under physics.
**Blocked by:** 37 (which stops rejecting the nodes this measures).

## Why

Chunk 37 stops asking "does this node work alone in a world it invented?".
This chunk asks the question that replaces it:

> **Does a tree containing this node do better than the same tree without it,
> across scenarios the generator did not design?**

That is the question the user posed directly, and it is the only form that
handles the real case: **a node may never trigger in one scene and be decisive
in another.** The old gate killed such a node for showing nothing in the world it
happened to be tested in. An average over a scenario set does not.

It is also the honest replacement for a micro-world we have now measured to be
non-representative: `ExtendedSurfaceDetour` passes its own unit scenario, fails
it when ablated — and *still solves both dev scenarios* with the mechanism gone.
The world was honest and unrepresentative at the same time.

## The design

### Replace, never delete — the load-bearing detail

Removing a node from a `Sequence` changes the tree's **control flow**, not just
its content: the remaining children's short-circuit behaviour shifts, so
"tree without X" is not a clean control. The node is **swapped for a stand-in**
that returns what X returns when its precondition fails (`Status.FAILURE`),
preserving structure exactly.

`generate/admit.py`'s own docstring already anticipates this as the way a
Condition would have to be tested — *"pairing it with a fixed consumer and
comparing against the same tree with the candidate replaced by a stand-in"* — so
**this chunk builds one mechanism and gets two things**: node contribution, and
the Condition/Decorator testability that was previously a rejection reason.

### What is measured

For a tree T with nodes n1..nk, over scenario set S and seeds K:

- `full` = T scored over S x K
- `ablated(i)` = T with ni replaced by the stand-in, scored over S x K
- `contribution(i)` = full - ablated(i), per metric, with the per-scenario
  breakdown kept, not just the mean

Report **the distribution, not only the mean**: a node that changes nothing on
eleven scenarios and rescues the twelfth is exactly the case this exists for, and
a mean hides it. Also report `precondition_fired` per scenario, so "did not help"
and "never triggered" are distinguishable — they are different findings and the
old gate conflated them.

### Where it runs

`core/tests/kinematic.py` over the **dev scenarios**. Real lab42 geometry, real
sensing, real drift and localisation, real `core.metrics.score()` — and an
idealised unicycle with the frozen locomotion policy absent. Representative
world, unrepresentative physics: **one axis fixed, not two.** The file's own
docstring says no number produced there may be reported, and that stands.

⚠️ **The sealed split is NOT used here.** This is development measurement, it
feeds back into which nodes get built, and running it against the held-out
scenarios would be the leak G3 exists to prevent.

Cost: seconds. A 5-node tree is 6 variants x 12 scenarios x 3 seeds ~ 216
rollouts. The same sweep under physics is 4-13 hours headless, worse with
timeouts, which is why physics is confirmation (chunk 36) and not the sweep.

## Build

1. `harness/ablate.py`: `StandIn` (a leaf returning FAILURE, HANDWRITTEN
   metadata, declaring the ports of the node it replaces so wiring still
   type-checks), `ablate(tree, node_id)` -> a new tree, `contribution(tree,
   scenarios, seeds)` -> per-node, per-scenario table.
2. `scripts/node_contribution.py`: run it over a composed tree or over the
   admission tree of every library node. `--json` for the record, a table by
   default. SIM-FREE.
3. Tests: the stand-in preserves tree structure (same node count, same composite
   ids); ablating a node the tree never ticks changes nothing; ablating a
   load-bearing node changes the score; a node that fires in one scenario of
   several shows a non-zero contribution there and zero elsewhere.
4. Re-run the Cheng ablation through this harness and confirm it reproduces the
   hand-rolled 2026-09-07 numbers (SPL 0.980 -> 0.934 on `doorway_traverse`).

## Done when

- Contribution is measurable for every library node without editing its source.
- The Cheng result is reproduced by committed code rather than a scratch script.
- A per-node contribution table exists for the dev set, distribution not mean.
- `out/evidence/nodes/contribution/` holds it with a README.

## Not in scope

- Physics confirmation — chunk 36.
- Any use of the sealed split.
- Changing what a condition is, or any generation call.

---

## Outcome — 2026-09-07

Built and working. `harness/ablate.py`, `harness/tests/test_ablate.py` (7 tests),
`scripts/node_contribution.py`.

### It does the thing it was built for

Measured on the committed composition over all three dev scenarios:

| node | mean delta | helped in | never fired in |
|---|---|---|---|
| `toy_is_goal_known` | **+1.000** | 3/3 | 0/3 |
| `toy_avoid_obstacles` | **+1.000** | 3/3 | 0/3 |
| `toy_sweep_for_goal` | +0.000 | 0/3 | **3/3** |

`toy_sweep_for_goal` is in the tree and **never ticks** on any dev scenario — all
three disclose the goal, so the Fallback never reaches the sweep branch. It is
reported as `never_fired_in 3/3`, not as a useless node. That is exactly the case
the old gate could not express and the reason this chunk exists: a node may be
dead in one scene and decisive in another, and the two are different findings.

### ⚠️ A claim in the plan body is WRONG, and it matters

The plan said this chunk would "re-run the Cheng ablation through this harness
and confirm it reproduces the hand-rolled numbers". **It cannot, and the reason
is a real limitation rather than a bug.**

`ablate()` removes a **whole node** from a tree. The Cheng result removed a
**mechanism from inside a node** — `_extended_surfaces` stubbed while the other
six layers of `ExtendedSurfaceDetour` kept running. Those are different
operations and only the first is mechanical: doing the second generically would
need the model to mark which lines are the paper's, which is precisely what
`generate/FINDINGS.md` §1 says cannot be checked.

**So node contribution does not detect mechanism dilution.** A monolith whose
paper contributes nothing still scores a large positive contribution as a node,
because deleting the whole node deletes the six support layers with it. The
Cheng finding — 2026-09-07, mechanism ablated, node still solves both dev
scenarios — remains reachable only by hand.

This is the same shape as card infidelity in a new place, and it should be
declared as a threat rather than left to be rediscovered.

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| Does the stand-in preserve control flow? | Same node count, same composite ids, pinned by a test | confirmed — deleting would have promoted every later child in a Sequence |
| Is a never-fired node distinguishable from a useless one? | `toy_sweep_for_goal`: ticks 0, `never_fired_in` 3/3, delta 0.000 | confirmed, pinned |
| Is a load-bearing node separable from it? | `toy_avoid_obstacles`: +1.000, helped 3/3, fired every tick | confirmed, pinned |
| Does ablating an absent node fail loudly? | `KeyError` | confirmed — silence would report a real zero for a node that was never there |
| Is the per-scenario distribution kept? | Yes, with ticks / fired / drove per cell | confirmed |
| Does it reproduce the Cheng ablation? | **No, and it cannot** — see above | **the plan's claim is withdrawn** |
| Sealed split touched? | No. `DEV_SCENARIOS` only, stated in the module docstring, the script docstring and the test docstring | confirmed |
| Physics | **Not checked.** Kinematic throughout | outstanding — chunk 36 |
| Full sim-free suite | **764 passed, 1 failed**, 256 s | the 1 is the pre-existing `no_papers` port-name gap |

### Still open

- `out/evidence/nodes/contribution/` not yet written — the sweep is currently
  only meaningful on one composed tree of three toys, because the 14 generated
  nodes reach no tree (every published port is still orphaned).
- **The contribution sweep is not yet useful on the real library** for that
  reason. It becomes useful when nodes read each other's ports, which needs the
  modifier-kind question the contract cannot currently express.
