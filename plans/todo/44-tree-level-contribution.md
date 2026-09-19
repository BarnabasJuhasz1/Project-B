# 44 — Contribution moves from the micro-world to the tree

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** 42, 43 · **Needs Isaac Sim:** yes
**Status: DECIDED 2026-09-12, NOT BUILT.** Recorded so the decision is on file before any
code moves.

---

## The decision, in one sentence

**A node stops earning its place by clearing an effect floor in a micro-world it wrote for
itself, and starts earning it by improving an incumbent behaviour tree on a shared scenario
suite, under physics.**

## Why — the micro-world cannot measure contribution

Measured across all 15 v2 nodes (see `generate/FINDINGS.md`, 2026-09-12):

- Seven nodes have had contribution measured. **None cleared the floor**; `contribution_rate`
  is 0.0 in **both** arms.
- Three were flagged `MALFORMED: the fixed baseline already succeeds on this micro-world`.
- One node was ADMITTED having fired 0/600 ticks and driven 0 times.
- `precondition_fired` is **not comparable across nodes**: every fraction was measured in a
  world written by the same model that wrote the trigger it tests.

The generator authors the mechanism, the trigger, *and* the world that judges them. That loop
cannot be closed from inside. It is broken by measuring in a world neither arm's model wrote.

## What is kept and what is scrapped

| | Verdict |
|---|---|
| **Validity** — does it build, wire and tick? (`UNSATISFIED_PORT`, `INVALID_NODE`) | **KEPT.** Sim-free, milliseconds, model-independent. Caught 4 real defects on 2026-09-12. |
| **Contribution** — does it help? (effect floor in a generated micro-world) | **SCRAPPED as an admission criterion.** Moves to the tree. |

⚠️ This weakens **non-negotiable #5** ("a node is admissible only if it passes its unit
scenario in isolation"), which comes from the project description. **Weakened on the user's
explicit instruction, 2026-09-12**, with the evidence above as the reason. It is not
abandoned — the unit scenario remains as a validity gate — but it is no longer sufficient
evidence that a node does anything.

It promotes **non-negotiable #9** (the promotion gate: beat the incumbent on a frozen
regression suite) from a secondary check to *the* admission criterion. The mechanism already
exists in `harness/gate.py`.

## The shared base node — DECIDED

**One base node, identical across every arm and every tree.** Each task family gets one tree
per arm; every one of those trees is grown on the *same* base.

The alternative — an arm-generated base — was rejected: the two trees would differ from their
first node, so B−A would confound the base with everything built on it, and no individual
node's effect would be attributable.

⚠️ **This is an exemption from non-negotiable #3** ("scaffolding must never enter a benchmark
condition"). Taken deliberately, on the user's instruction, 2026-09-12. The base is declared a
**substrate**: part of the harness, not part of any condition, byte-identical in both arms, and
**stated in the write-up** rather than glossed. The consequence — a tree is not purely "arm A's
nodes" — is accepted, because measuring what an arm's nodes *add* requires the thing they are
added to to be constant.

**Open:** what the base actually is. `reactive_avoid` is a `Controller`, not a `Node`, and is
marked "NEVER a condition"; wrapping it is the obvious candidate but the wrapper is not written.

## ⚠️ Correction 2026-09-13: `harness/ablate.py` ALREADY EXISTS

This chunk was written as though tree-level contribution needed building. It does not.
Chunk 37 built `harness/ablate.py`, which asks "does this node help?" **of a tree, over a set
of scenarios the generator did not design** — the same argument this chunk makes. It provides
`ablate()`, `ablatable()`, `score_tree()` and `contribution()`.

What is actually missing is narrower: **the shared base node**, and **the physics path**.
`ablate.py`'s own sweep runs on `core/tests/kinematic.py` over the dev scenarios and its
docstring says no number from it may be reported; physics confirmation is chunk 36.

`ablate.py` measures contribution by **removal**, and swaps the node for a same-kind stand-in
returning FAILURE rather than deleting it — because deleting a child from a `Sequence` or
`Fallback` silently promotes every child after it, so "tree without X" would differ in two ways
at once.

**DECIDED 2026-09-13: the growth loop measures by ADDITION, not removal.** Incumbent tree vs
incumbent + the new node, same suite, keep it if the tree improved. Removal's promotion problem
does not arise. `ablate.py` is retained for **attribution after the fact** — which node in a
finished tree is carrying the result, and which are dead weight that never fires — not for the
keep/drop decision.

## ⚠️ The insertion rule is load-bearing, not a detail

Adding is only clean if the addition cannot disturb what is already there. The tree is:

    Fallback[ specialist_1, specialist_2, ..., specialist_k, base ]

and a new node is appended **immediately before the base**, at the end of the specialist list.

- Each specialist is tried in order; a failed precondition returns FAILURE and the next is tried.
- The **base is last** and drives whenever no specialist claimed control.
- `specialist_1..k` keep their exact positions, so their control flow is unchanged bit for bit.
- `specialist_{k+1}` can only act where every existing node already declined.

⚠️ **`Fallback[base, new_node]` is the trap.** The base drives, returns RUNNING or SUCCESS,
`Fallback` short-circuits, and the new node is NEVER TICKED — every addition would measure "no
change" and nothing would look broken. Observed for real on 2026-09-12: a bare one-leaf tree
froze for 98.2% of ticks because a selective precondition had nothing to fall through to.

Inserting at the front or the middle lets the new node pre-empt existing ones, so the
comparison confounds the new mechanism with a reordered priority. **Append-before-base, or the
measurement is not additive.**

## ⚠️ The tie-break must be fixed BEFORE the first round

Success rate over three dev scenarios takes four values. Most additions will tie, and a
keep/drop rule decided after seeing a tie is a story about whoever ran the loop.

Fix in advance, and identically for every arm: **if goal-reach rate is equal, keep on SPL, then
on collisions, then reject.** The real fix is a larger scenario suite — see the environment
suite epic — but the tie-break is needed regardless.

**The comparison itself is deterministic**, which is a genuine advantage: same-machine runs are
bitwise reproducible and undrifted scenarios give identical episodes across replicate seeds, so
"did the tree improve?" needs no statistical test. That is also the sharpest form of the
overfitting risk below — a deterministic keep/drop rule applied repeatedly to one suite IS
gradient descent on that suite.

## How a tree grows

1. Run the incumbent tree on the shared suite, **under physics**, and score it.
2. Take a failure signature from the result.
3. Generate a node against it (failure-driven generation — already in CLAUDE.md as the second
   generation trigger).
4. Admit on **validity** only.
5. Add it to the tree; re-run the whole suite; keep it **only if the tree improved**
   (`harness/gate.py`).
6. Repeat.

## ⚠️ Two methodological traps this design must not fall into

**Overfitting.** Growth is "see a failure, fix that failure". Growing and scoring on the same
scenarios fits the test set and makes the success rate meaningless. **Grow on dev, freeze,
measure once on held-out.** The growth loop must never see `core/scenarios_held_out.py`.

**Unequal growth.** "Add nodes until it works" can be run longer for one arm. The stopping rule
must be **fixed in advance and identical across arms** — same number of rounds, same model-call
budget, same procedure for choosing which failure to attack. Without it, B−A measures how long
we kept going. Cheap to fix now, impossible to fix afterwards.

## The environment

The dev set is three scenarios and `reactive_avoid` solves all three, so it cannot drive this
loop. A harder set is needed, with **graded** difficulty — a base that always fails gives no
signal to improve against. `core/scene_spec.py` is data, so new geometry is cheap. The sealed
split is **not** touched.

## Cost, measured 2026-09-12

| | 3 dev scenarios | one 15 s episode |
|---|---|---|
| Kinematic rollout | 1.5 s | 0.27-0.60 s |
| Physics, `--no-record` | ~60 s | ~21 s |
| Physics, mp4 @ 15 fps | ~12 min | ~240 s |

Kinematic is ~35x faster than headless physics and ~400x faster than a recorded run — but it
is **not a benchmark path** and omits the frozen locomotion policy entirely. Use it to triage
(does the node fire at all, is the tree obviously broken); take every reported number from
physics.

## Not in scope

Changing `PRE-REGISTRATION.md`. ⚠️ **G1 reports "unit pass rate", and this changes what that
number means** — that requires a DATED APPEND to the Amendments section, never an edit, and it
has NOT been written. Do that before any G1 number is reported.
