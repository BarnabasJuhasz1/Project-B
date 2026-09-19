# 39 — The first comparable A vs B numbers

**Status:** todo. SIM-FREE, **zero model calls**, runs in minutes.
**This is the fastest path to a comparable A/B number and it should be done first.**
**Blocked by:** nothing. 37 and 38 are done and are what made it possible.

---

## Read this first if you are a fresh session

Interpreter: `P=~/miniforge3/envs/env_isaacsim/bin/python`, run from `Repo/`.
Full sim-free suite (~4 min, currently **764 passed, 1 failed**):

```bash
$P -m pytest core/tests/ nodes/tests/ harness/tests/ cards/tests/ \
    generate/tests/ compose/tests/ runs/tests/ -q
```

The one failure is pre-registered as known: `compose/tests/test_no_papers.py`
builds its `contracts` set from EFFECT + PRECONDITION + TAGS + node_id but omits
**port names**, while `render_node` renders them. Counting port names as contract
clears every card to zero. It is a test gap, not a leak — port names *are* the
port contract, which is what #6 says composition may see — and it is left unfixed
because widening a leak assertion is the user's call. **Do not "fix" it as a
side effect of this chunk.**

Background you need, in order: `CLAUDE.md`, `generate/FINDINGS.md`,
`compose/FINDINGS.md`, `plans/done/37-admission-becomes-validity.md` (Outcome),
`plans/done/38-node-contribution-ablation.md` (Outcome).

### The state as of 2026-09-07

Chunk 37 redefined admission: a node is admitted if it **runs**, not if it moves
a metric in a world it designed for itself. Consequently **all five pilot cards
now have both an A node and a B node**, where two days ago only two did:

| card | A (citation only) | B (given the card) |
|---|---|---|
| `limit_speed_dwa` | `DynamicWindowDrive` | `LimitSpeedToVisibleRange` |
| `limit_speed_ics` | `ICSFilteredGoalDrive` | `LimitSpeedToOcclusionBoundary` |
| `path_integration_ants` | `PathIntegrationHoming` | `MaintainHomingVector` |
| `path_integration_mammals` | `PathIntegrationGoalVector` | `ArbitrateHomingVector` |
| `reorient_geometry` | `ExtendedSurfaceDetour` | `ReorientToGeometry` |

All ten are in the library (`lib-019-ed0c5cc44084`), all on the
`navigate_to_goal` task family, all derived from the **same five papers** — the
arms differ by exactly one thing, whether the extracted card was supplied.
**That is a clean paired design and nothing has to be generated to use it.**

---

## Why

The project has never reported a number comparing A with B. It now can, for the
price of a few minutes of CPU.

## ⚠️ The trap, and it must be in the write-up, not discovered by a reader

**A produces whole navigators; B produces single mechanisms.** Measured
2026-09-07 with each node alone as the controller on `doorway_traverse`:

| arm | node | reached | SPL | collisions | path |
|---|---|---|---|---|---|
| A | `ExtendedSurfaceDetour` | yes | 0.980 | 0 | 10.30 m |
| B | `ReorientToGeometry` | no | 0.000 | 8 | 36.03 m |
| A | `ICSFilteredGoalDrive` | yes | 0.950 | 0 | 10.62 m |
| B | `LimitSpeedToOcclusionBoundary` | no | 0.000 | 0 | **0.00 m** |

B loses because it is **not trying to navigate**. `LimitSpeedToOcclusionBoundary`
travels zero metres: it is a speed limiter with nothing to limit, and its trigger
(an occlusion boundary within stopping distance) never occurs because nothing
carries it to a corner.

This is *not* evidence that the corpus does not help. The card's Derivation pins
B to one mechanism and B implements exactly that — fidelity. A citation alone
leaves the model free to build a complete controller wearing the paper's name.
**A's advantage is partly A doing more than its source licenses**, and it cannot
be checked, because a citation node can be diffed against nothing
(`generate/FINDINGS.md` §1).

The sharpest evidence: `ExtendedSurfaceDetour` with **Cheng's mechanism ablated**
(`_extended_surfaces` stubbed to return `[]`) **still reaches the goal** on both
dev scenarios (SPL 0.934 / 0.951, +1 collision). Six support layers do the work;
the paper is one layer of seven. The same ablated node **fails its own unit
scenario**, effect 0.000 — so the micro-world was honest and unrepresentative at
the same time.

## What to measure

Two framings, **both reported, neither presented alone**. Same ten nodes, same
dev scenarios (`doorway_traverse`, `blind_corner`, `return_home`), same seeds,
the full metric vector — `goal_reached`, `spl`, `collisions`, `near_misses`,
`stuck_events`, `time_to_goal`, `path_length` — plus the precondition firing rate
from `harness/ablate.py`, which is what separates "did not help" from "never
fired".

1. **Solo** — `Sequence[ContractPorts, NODE]`, the admission tree
   (`generate.admit.build_tree`). Answers: *is this node a complete controller?*
   A wins by construction. Report it as a measure of **breadth, not merit.**
2. **Carried** — `Sequence[ContractPorts, Fallback[NODE, toy_avoid_obstacles]]`,
   an identical fixed carrier in both arms so it cancels, with the node's
   contribution measured by `harness.ablate.contribution`. Answers: *what does
   this node add to a fixed baseline controller?*

⚠️ **Carried is fairer but still not fair**, and this was checked on 2026-09-07:
both A and B nodes are Actions whose preconditions usually hold, so they return
RUNNING and the `Fallback` short-circuits before the carrier ticks — A *replaces*
the carrier rather than augmenting it. Only `LimitSpeedToOcclusionBoundary`
changed at all (0.00 m -> 5.24 m). **Report the firing rate beside every number
so a reader can see when the carrier never ran.**

**The genuinely fair comparison does not exist yet** and needs chunk 40: no
generated node declares a single IN port, so B's mechanisms cannot be wired to
anything and cannot act as modifiers. Say that in the write-up rather than
implying the comparison is clean.

## The paired statistic

Report **per-card differences**, A minus B, five of them — the paired design is
the whole point of generating both arms from the same paper. **Do not bootstrap
a CI over n=5 cards and present it as a result**; report the five differences and
their signs. `core/stats.py` is G2's rule and this is not G2 — **do not call
`decide()` here**, and do not let this number be mistaken for the pre-registered
one.

## Build

1. `scripts/ab_compare.py` — SIM-FREE. Enumerates the paired cells from
   `generate/staging/` (the same exclusion rules `pass_rate` uses: skip
   `retired_reason`, `superseded_by`, and refs not in the card store), builds
   both framings, scores over `DEV_SCENARIOS` x seeds, prints a per-card paired
   table and `--json` for the record. Reuses `harness/ablate.py` and
   `generate.admit.build_tree`; **writes no new tree-building code.**
2. Tests: the pairing enumerates exactly the five cards above; a retired or
   fixture record is excluded; both framings build for all ten nodes.
3. `out/evidence/nodes/ab-comparison/` — the table, the JSON, and a README
   carrying **the whole trap section above**, the command, and the date.

## Done when

- A per-card A-vs-B table exists for all five cards under both framings.
- Every row carries its firing rate.
- The README states plainly that this measures breadth as much as merit, that
  the fair comparison needs chunk 40, and that no number here is a G2 number.
- `plans/` Outcome records a Checks table with `n` stated.

## Not in scope

- Physics — chunk 36.
- The sealed split. **Never.** This is development measurement and running it on
  the held-out scenarios is the leak G3 exists to prevent.
- Any generation call, any prompt change, any new card.

---

## Outcome — 2026-09-07

**Done, and it answered a different question than it asked.** Built
`scripts/ab_compare.py` and `generate/tests/test_ab_compare.py` (12 tests);
wrote `out/evidence/nodes/ab-comparison/` — `README.md`, `table.txt`,
`comparison.json`. Zero model calls, ~10 minutes of laptop CPU, no GPU.

### The number the chunk asked for

Five paired cards, library `lib-019-ed0c5cc44084`, dev scenarios only.

| framing | `goal_reached` A−B | `spl` A−B | A > B |
|---|---|---|---|
| solo | [+1.000, +1.000, +1.000, +1.000, +0.667] | [+0.935, +0.956, +0.959, +0.989, +0.648] | **5/5** |
| carried | [+0.333, +1.000, +0.000, +1.000, +0.667] | [+0.276, +0.956, −0.023, +0.989, +0.648] | **4/5** |

Reported as five signed differences. Not bootstrapped, `decide()` not called.

### ⚠️ The finding the plan did not anticipate, and it is the important one

The plan framed the trap as "solo measures breadth, so use the carrier". Adding
the **carrier-only reference row** — which the plan did not ask for and which was
added after the first run made the ablation deltas unreadable — changes the
headline:

**`Sequence[ContractPorts, AvoidObstacles]` alone reaches the goal on all three
dev scenarios at SPL 0.982.** `AvoidObstacles` is a handwritten fixture, is
scaffolding under #3, and neither prompt has ever seen it.

Against that yardstick, leave-one-out inside the carried tree:

- **All five A nodes are inert.** Δreach `+0.000` across the board, ΔSPL
  *negative* in four of five. `PathIntegrationGoalVector` at **+0.007 SPL is the
  only one of ten generated nodes that beats the fixture at all.**
- **Four of five B nodes actively destroy it**, to −1.000 reach — by returning
  RUNNING, short-circuiting the `Fallback` and starving the carrier. This is
  `compose/FINDINGS.md` finding 1 reproduced as a number rather than an anecdote.
- **The fifth B node never fires** (9/9), so its zero is not a result. Its
  carried row is *numerically identical* to the carrier-only row — the single
  cell where B ties A on reach and beats it on SPL is entirely the carrier.

So the honest one-line reading is **"A beats B on every card, and neither arm
beats a handwritten toy"** — not "the corpus does not help", which the evidence
does not support and which `PRE-REGISTRATION.md`'s not-fired rule forbids
asserting from a rule that did not fire.

### Checks

| check | outcome | verdict | n |
|---|---|---|---|
| Does the pairing enumerate exactly the five pilot cards? | yes, and each has exactly one A and one B | **confirmed** | 5 cards / 10 nodes |
| Are the retired naive-arm records excluded? | 4 excluded, none reached the pairing | **confirmed** | 4 |
| Is the `vague_adaptive_motion` fixture excluded? | yes, both generations | **confirmed** | 2 |
| Is the superseded `gen_b_1f2fe8016b1441dc` excluded while its replacement is kept? | yes | **confirmed** | 1 |
| Does the pairing agree with `pass_rate`'s denominator? | admitted counts match per arm | **confirmed** | 2 arms |
| Do both framings build for all ten nodes? | yes, all 20 trees construct and wire | **confirmed** | 20 |
| Do the three replicate seeds add information? | **no — 60 of 60 (cell × scenario) groups bit-identical across seeds 0,1,2** | **ruled out** | 60 groups / 540 episodes |
| Does any generated node declare an IN port? | **none** | **confirmed (negative)** | 10 paired |
| How many generated nodes both drive and publish a port? | **14 of 14** in the live library | **confirmed** | 14 |
| Does the carrier framing actually let the carrier run? | **no — 8 of 10 cells have `carrier_drove` = 0.00** | **confirmed (negative)** | 10 cells |
| Does any generated node beat the handwritten carrier? | one, `PathIntegrationGoalVector`, by +0.007 SPL | **confirmed** | 10 |
| Full sim-free suite still green? | 775 passed, 1 failed — the pre-existing `test_no_papers.py` port-name gap, untouched | **confirmed** | 776 |

**Not checked:** anything under physics (chunk 36); anything on the sealed split
(never — that is the leak G3 exists to prevent); whether the effects survive a
carrier other than `AvoidObstacles`, which is the single arbitrary choice in the
design and the obvious next sensitivity test.

### What this hands to chunk 40

The blocker is now measured rather than argued. **No generated node reads a
port** and **14 of 14 both drive and publish one**, so a B mechanism has exactly
two ways to affect the robot: drive itself (and starve whatever else would have),
or publish into the void. Both are visible in the table above. A fair A/B
comparison does not exist until the contract can express a modifier, and both
facts are now pinned by tests so 40 landing shows up as a failing assertion
rather than as a stale paragraph.

`8 of 10 carrier_drove = 0.00` is also a concrete acceptance target for 40: the
number should move.
