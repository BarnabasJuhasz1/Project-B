# 16 — Composition

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** 12, 15 · **Needs Isaac Sim:** no

## Why

Week 4, and non-negotiable #6: **composition sees only port contracts and a task specification.
Never the papers.** That separation is what makes G3 meaningful — if composition could read the
mechanism descriptions it would be doing generation again, and "solved by recombination alone"
would not be a distinct claim.

## Decided

**What the task specification contains.** It is the retrieval query and the composition input,
and it must be expressible for a held-out scenario without leaking the held-out module into the
prompt. Propose: start region, goal condition, sensor configuration, time limit, and the failure
signature if this is a repair — all of which are already `Scenario` fields.

Two of those fields now carry real information rather than being constant across the benchmark,
because of [11a](../done/11a-task-contract.md) and [11](../done/11-held-out-split.md):

- **`goal_known`** — whether the goal is disclosed at t = 0. This is the single most
  consequential bit in the spec: it decides whether the composed tree needs an exploration
  branch at all. It is a task property, not a mechanism, so it is legitimate for composition to
  see it.
- **`drift is not None`** — whether the pose estimate degrades. Same argument.

Those two bits are exactly the axes of chunk 11's 2 × 2, so composition sees the *shape* of each
held-out family without ever seeing a held-out scenario. That is the intended separation, and it
is worth a test: a TaskSpec rendered from a held-out scenario must contain no id, no coordinates
and no seed — only the family's axes and the sensor configuration.

**How retrieval keeps the working set small.** #7 makes the library append-only and monotonic, so
by Week 5 it may hold well over 20 nodes and cannot all go in one prompt. Retrieval by port type
and tag is the mechanism the epic's contract was designed for; the ranking is the open question.
Recommend: retrieve by required output port type, then by tag overlap with the task spec, and
**log what was retrieved per composition**, because "the library had the right node but retrieval
never surfaced it" is a completely different failure from "the library lacked the node" and G3
cannot tell them apart otherwise.

**Repair loop or single shot.** A composition that fails could be re-prompted with the failure.
That is the failure-driven trigger from §5, and it blurs into generation if it is allowed to
create new primitives. G3 requires "no new primitives generated" on held-out. Recommend: repair
is allowed to **rewire**, never to create, and the two must be separately logged.

## Build

- `compose/spec.py` — `TaskSpec`, derived from a `Scenario` with everything the papers touched
  stripped out. A test asserts no mechanism text reaches it.
- `compose/retrieve.py` — port-type and tag retrieval over the library snapshot.
- `compose/compose.py` — TaskSpec + retrieved port contracts → a `nodes.tree.Tree`, which is a
  `core.observation.Controller` and therefore runs under `run_episode.py` unchanged.
- Every composition records the library snapshot tag (#8), the retrieved set, and the prompt hash.

## Done when

- A tree composed from two toy leaves solves `doorway_traverse` through `run_episode.py` with no
  runner changes and no controller registered by hand.
- A TaskSpec built from a held-out scenario leaks nothing identifying: no id, no start or goal
  coordinates, no seed. Tested against the sealed module directly, in a test that is allowed to
  import it precisely because a test is not the generation path.
- The composition prompt provably contains no Mechanism or Derivation text — asserted by a test
  that greps the rendered prompt against the card store, not by inspection.
- A composition that fails is re-composable from the same library without generating anything new,
  and the log distinguishes "rewired" from "new primitive".
- Retrieval is logged per composition, so a G3 miss can be attributed to library or to retrieval.

## Before running

Same constraint as chunk [15](15-generator.md): a session that has never had
`core/scenarios_held_out.py` in context.

## Not in scope

The four-condition evaluation, the batch runner over seeds, and bootstrap confidence intervals —
Week 5, and they need G2's threshold X fixed first, which is chunk
[17](17-pre-registration.md)'s job and must be done before the first evaluation run rather than
before this chunk.

---

## Outcome

**Done.** `compose/` is built: `spec.py`, `retrieve.py`, `library.py`,
`compose.py`, `prompts/compose.md`, plus `scripts/compose_tree.py` and 50 tests.
Five real `claude -p` calls against `claude-opus-5`; five committed compositions,
all replayable with `--backend replay`. 414 tests pass across the repo.

⚠️ **Read `compose/FINDINGS.md` before using any of it.**

### The decision the chunk actually turned on

**Composition returns a JSON tree over node ids. It never returns Python.**

The plan did not specify the output format, and it is the whole difference
between G3 being a claim and being a measurement. If composition emitted code,
"solved by recombination alone, **no new primitives**" would be a rule the model
was asked to respect, checkable only by reading what it wrote and arguing about
whether a helper function counts. A plan whose entire vocabulary is a list of
retrieved node ids makes it structural: there is no channel through which a new
primitive can arrive, and `build()` rejects an id that was not retrieved.

It also settles the repair loop for free. The plan required repair to "rewire,
never create", and separately logged. Since a repair is handed the **previous
composition's retrieval**, its vocabulary is identical — so it *cannot* create,
and the log's only job is to say which kind it was (`kind`, `repair_of`).
`created_primitives` is in every record and is structurally always empty; it is
recorded anyway so a reader can check the claim rather than be told it.

### What differed from the plan

**`time_limit` is not a TaskSpec field.** The Decided section listed it. Within
three of the four held-out families it is the *only* field that differs between
the three scenarios, so a spec carrying it can tell an episode from its family —
exactly what G3 must not be able to do. Nothing is lost: no node reads a clock,
the runner enforces the limit, and the manifest records it. See FINDINGS §4,
including the point that a leak test run only against the dev set would have
missed this.

**The start region is not a TaskSpec field either.** Also proposed, also out.
Any bucketing coarse enough not to identify a scenario is too coarse to compose
differently against, and a tree conditioning on where it started would be
coupled to a scenario, which #4 forbids leaves from being. Dropped rather than
blurred.

**`describe()` is generated from the axes, not looked up by family.** A table
keyed by family could hold a sentence saying more than the axes do — including,
eventually, something a paper suggested — and nothing would catch it. Generating
the prose means it cannot carry information the fields do not.

**The leak test lives in `core/tests/test_compose_leak.py`, not in
`compose/tests/`.** Chunk 11's AST guard forbids anything under `compose/` from
importing the sealed split and makes no exception for tests. Weakening the guard
to make room for a test would be weakening the seal in order to test the seal.
That file also asserts the guard is no longer **vacuous** for `compose/` — it
skips a package directory that does not exist, so it had been checking nothing
there, and a guard that silently checks nothing looks identical to one that
passes.

### Two additions outside `compose/`

**`ContractPorts` is in the composition library, and it had to be.**
`generate/admit.py` publishes `goal_known`, `pose` and `ranges` from
`Observation` so a candidate reading one can be tested alone. Composition has to
offer the same three, or a node admitted while reading `ranges` is
UNSATISFIABLE at composition time and its unit scenario proved nothing about the
run. The class is imported, not re-declared: one definition means the two steps
cannot drift apart.

**`nodes/toys.py` gained `AvoidObstacles`**, wrapping `controllers.reactive_avoid`
exactly as `SweepForGoal` wraps `scaffold_sweep`. NOT navigation and never a
benchmark condition. Without it there is no composable Action that reaches the
goal, and the chunk's headline check would have been a tree that constructs
rather than a tree that works.

**`controllers.make()` resolves a `composed:<id>` namespace.** That is how a
composed tree reaches `run_episode.py --controller composed:<id>` with **no
runner change and nothing registered by hand** — `run_episode.py` already calls
`controllers.make()`, and a `Tree` is already a `Controller`. Deliberately a
namespace rather than registry entries: compositions are produced by the
pipeline, and a registry someone must remember to update goes stale.
`available()` still lists only the static names, so nothing here can be swept
into an evaluation by a batch runner iterating the safe list.

### Done-when, and the one item that is not literally met

- ✅ A tree composed from toy leaves **reaches the goal** on `doorway_traverse`
  — goal_reached, SPL 0.956 — resolving through `controllers.make()` with no
  runner change and nothing registered by hand.
  ⚠️ **Measured on `core/tests/kinematic.py`, not through `run_episode.py`.**
  This chunk is sim-free; the rollout is an idealised unicycle with the frozen
  locomotion policy absent entirely and no number from it may be reported. The
  containerised run is where that becomes a number, like every other number in
  this project.
- ✅ A TaskSpec from a held-out scenario leaks nothing: no id, no seed, and no
  number without a disclosed field behind it — and no scenario in a family is
  distinguishable from its siblings.
- ✅ The composition prompt provably contains no Mechanism or Derivation text,
  by four tests: sentence-level across the whole prompt, word-level across the
  rendered node block, structural against `LibraryEntry.identity()`, and a check
  that the prompt names only retrieved nodes.
- ✅ A failed composition is re-composable from the same library with nothing new
  generated, and the log distinguishes rewired from created.
- ✅ Retrieval is logged per composition — considered, scored, retained with a
  reason, dropped, unsatisfiable ports, orphan ports — so a G3 miss is
  attributable to library or to retrieval.

### What this hands chunk 17

- **Whether an Action that drives may also publish a port.** The admitted
  condition-B node does both; the contract says one effect; the harness cannot
  tell. FINDINGS §2.
- **`MAX_SEED_ACTIONS = 8`** is what composition was offered, so it belongs in
  the pre-registration alongside the seed set and the episode count.
- **A composition that succeeds is not a composition that works.** Any G3 count
  has to be over episodes run, never over compositions that built.
