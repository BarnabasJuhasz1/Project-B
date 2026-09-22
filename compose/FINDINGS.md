# compose/ — findings

> ⚠️ **HISTORICAL FROM 2026-09-21. COMPOSITION IS NOT ON THE LIVE PATH.** The behaviour-tree
> execution model is dropped; there is no retrieval step and no composer. See
> `plans/07-epic-whole-controller.md`.
>
> **Nothing here is retracted.** The eight committed compositions still resolve and still mean
> what they meant, and the findings below are still the reason several design decisions were
> made. Two of them outlive the architecture and should be carried into the write-up:
> - **A composition can SUCCEED and produce a tree that does not work** (committed pair). The
>   general form — *an artefact that passes its own generation check is not thereby a working
>   controller* — is exactly why epic 07's admission is **validity only** and contribution is
>   settled by scoring against the base.
> - **The admitted node has TWO effects**, which `nodes/contract.py` forbids and neither the
>   harness nor composition could detect. Reported, not hidden.
>
> ⚠️ **G3 (compositional generalisation) stands on the node library or it does not stand.** A
> whole controller has no primitives to recombine, and generalisation across world sets must
> never be reported as G3.

⚠️ **Read this before using anything in `compose/` and before writing chunk 17.**
Two of these change what a reported number means.

Eight real `claude -p` calls against `claude-opus-5` produced the records in
`compose/compositions/`. Every one is replayable from `generate/transcripts/`
with `--backend replay`, no key and no network.

The library moved on 2026-08-28. Verifying `limit_speed_dwa`'s citation meant
dropping `span` and `confidence` from the generation prompt (provenance work must
not change the code the model writes), which re-generated the one admitted node
as `gen_b_d6ae93648c7b4f25` and moved the snapshot from `lib-006-a05b0b267dd4` to
`lib-006-23c57a77269e`. The three live compositions were **re-composed** against
it. The lib-006-a05b0b267dd4 records stay: G3 needs a denominator and a finding
needs its evidence.

| composition | snapshot | outcome | kept because |
|---|---|---|---|
| `navigate_to_goal__…__56b34662` | 23c57a77269e | composed, **reaches the goal** (SPL 0.96) | the headline demonstration |
| `search_for_goal__…__2b74bf7b` | 23c57a77269e | composed | the other axis, built from a family rather than a scenario |
| `r_search_for_goal__…__6004b2de` | 23c57a77269e | composed | the repair path, rewiring only |
| `navigate_to_goal__…__9c21b87c` | a05b0b267dd4 | composed, reached the goal | archived — the predecessor of `56b34662` |
| `search_for_goal__…__11022fcc` | a05b0b267dd4 | composed | archived — the predecessor of `2b74bf7b` |
| `r_search_for_goal__…__9532339b` | a05b0b267dd4 | composed | archived — the predecessor of `6004b2de` |
| `navigate_to_goal__…__6f9686b0` | a05b0b267dd4 | composed, **tree does not reach the goal** | finding 1 |
| `navigate_to_goal__…__e516fff9` | a05b0b267dd4 | `MULTIPLE_PLANS` | finding 3 |

⚠️ **`6f9686b0` is no longer executable.** The Action it starves on *is*
`gen_b_1f2fe8016b1441dc`, the superseded generation, so rebuilding the tree would
need a library that no longer loads and `load_composition` refuses that by design
(#8). Finding 1 is now read off the plan structurally —
`test_a_composition_can_succeed_and_still_not_work` walks the archived plan for an
Action sequenced ahead of another Action — and the half of the pair that still
runs is still run.

---

## 1. A composition can succeed and the tree still not work — and no port contract could have said so

The strongest finding in this chunk, and it is a committed pair rather than an
anecdote: two compositions, same library, same retrieval, same snapshot. One
reaches the goal on `doorway_traverse` at SPL 0.96. The other puts
`gen_b_1f2fe8016b1441dc` ahead of `toy_avoid_obstacles` inside a `Sequence`,
where RUNNING short-circuits, and starves it. The robot drives at the divider
with a speed cap and times out.

The model's reasoning was explicit and wrong in an interesting way: *"in open
corridors it fails harmlessly and control drops through"*. That is true of the
node's precondition and false of the environment — indoors the precondition
holds almost always. **Nothing in a port contract says how often a precondition
fires**, and how often it fires is exactly what decides whether that Sequence is
a speed limiter or a starvation bug.

Two consequences:

- **The demonstration that composition works is not evidence on its own.** Both
  records are kept so the pair can be read together.
- **The prompt was tightened** to spell out the short-circuit consequence
  ("two actions in one sequence almost never both run"), which is a fact about
  the tick semantics the model was demonstrably mis-modelling, and applies
  identically to every condition, so it cannot confound. It helped — the two
  compositions taken after it both reason correctly about the trap and both
  decline to fall into it — but it does not fix the underlying gap.

**The 2026-08-28 re-composition is a third data point on the same line, and a
stronger one**, because it is against a library the model had not seen before.
All three new compositions **omit the generated node outright**, and each says
why in the same terms: nothing in the retrieved set reads `speed_limit`, so
sequencing the node ahead of a driving Action *"would not shape that action's
motion, it would just consume ticks and starve it."* That is the trap named
correctly and avoided, three times out of three. Note what it costs: the only
literature-derived node in the library now reaches no tree at all — finding 2
(the orphan port) and finding 1 (starvation) compound, and the composed
condition B is at present toys only.

## 2. `gen_b`'s node has two effects, the harness could not tell, and composition is where it bit

`nodes/contract.py` requires **one precondition and one effect** — "a node that
needs two of either is two nodes; that is not a style rule, it is what makes a
unit scenario able to isolate one claimed effect."

The admitted condition-B node writes `admissible_speed` **and** drives towards
the goal. Its own comment calls the goal-seeking *"a carrier for the Action, not
a second mechanism"*, which is a reasonable thing to say and is not what the
contract asks. Its unit scenario tested the speed effect, passed, and the node
was admitted.

This is the sibling of `generate/FINDINGS.md` §1. There the harness could not
detect **card infidelity**; here it cannot detect **effect multiplicity**. Both
have the same shape: the harness tests that the node moves a metric, and a node
can move that metric while doing more than it claims.

It also produced the visible symptom in finding 1, and a second one:
`admissible_speed` is written by that node and read by nothing in the library,
so `retrieve()` now reports **orphan ports** — an OUT port no retrieved node
reads. A node whose only output is orphaned is inert in any tree built from that
set, however fluently a rationale explains its contribution. That line went into
the retrieval log because the first run produced exactly such a rationale.

**Chunk 17 should decide whether an Action that drives may also publish a port**,
and if the answer is no, admission needs a check the current harness does not
have.

## 3. Reproducible, not repeatable — again, and it cost a record

Chunk 15 established that temperature-0 determinism does not exist on current
models. The same prompt here produced a valid plan split across two fenced
blocks on one call (`MULTIPLE_PLANS`) and a single well-formed block on the
next.

The refusal is **kept, not retried away** — G3 needs a denominator, and a
pipeline that quietly re-rolls until it likes the answer has no denominator at
all. The response was a complete, valid plan whose two halves would have merged
unambiguously; merging them was refused for the same reason
`generator.extract_code` refuses two python blocks, which is that the harness
must not be the thing that assembles the composition.

Because a composition record is keyed by prompt hash, **tightening the prompt
orphans every record taken against the old one**. The orphaned record stays
auditable: the transcript holds the full prompt text, so nothing is lost but the
ability to re-render it from the current template.

## 4. `time_limit` cannot be in a TaskSpec, and only the leak test found it

Chunk 16's Decided section listed the time limit among the task-specification
fields. It is out. Within three of the four held-out families, **`time_limit` is
the only field that differs between the three scenarios**, so a spec carrying it
can tell an episode from its family — which is precisely what G3 must not be
able to do.

Nothing is lost: no node reads a clock, the runner enforces the limit, and the
episode's time limit is recorded in the run manifest, where a per-episode fact
belongs. The field is not merely unrendered but absent, because a spec holding
something it never shows the model is a leak waiting for someone to render it.

Worth noting how it was caught: not by reading, but by
`core/tests/test_compose_leak.py` asserting that every scenario in a family
renders identically. The equivalent check on the dev set passes either way —
the dev scenarios' time limits happen not to be a problem. **A leak test that
runs only against the dev set would have missed this.**

## 5. What is NOT established

- **No composed tree has run in the simulator.** Every number here is from
  `core/tests/kinematic.py`, which is an idealised unicycle with the frozen
  locomotion policy absent entirely, and says so in its own docstring. The
  reported numbers come from `run_episode.py` inside the pinned container.
- **The library is six nodes, four of them fixtures.** Retrieval's ranking has
  not been exercised at a size where ranking matters, and #7 guarantees it will
  be. The one thing already visible: node `TAGS` are model-chosen at generation
  time, so tag overlap is weak, and the answer to that is the retrieval log
  rather than a cleverer score.
- **No held-out scenario has been composed for.** The leak tests prove a
  held-out TaskSpec discloses nothing; they do not prove the library can solve
  one.

---

## ⚠️ 2026-09-06 — reproducing a composition now means the library the record names

**Decided by the user.** The table above, and finding 1's note that `6f9686b0`
"is no longer executable", were both written under a definition of reproduction
that made a composition perishable: `load_composition` reloaded the library **as
it stands today** and refused on `SNAPSHOT_MISMATCH` if it had grown.

#7 (append-only) and #8 (every result names its snapshot) together made that
self-defeating. **Every admission invalidated every composition recorded against
the smaller library**, and the cost compounded rather than being paid once — on
2026-09-06 two admissions took `compose/tests/` from five failures to seven, the
two new ones surfacing as `UNRETRIEVED_NODE` because today's retrieval crowds
out a node a recorded plan names.

It now means: **run the recorded plan against the library the record names**,
rebuilt from that record's own `retrieval.considered` — the full list of ids the
library held at composition time — with **no model call**. A repair records
`considered: []`, because `repair()` reuses the previous retrieval and creates
nothing, so its library is its parent's, reached through `repair_of`. That chain
is a design guarantee, not an inference.

**#8 is not weakened; it is enforced harder.** The rebuilt library's own
`snapshot()` must equal the tag in the record. The tag hashes the assembled
contents, so matching it *proves* the set was reassembled exactly, where before
the tag was only asserted about a library nobody rebuilt. Verified on the three
live records: all three reconstruct to `lib-006-23c57a77269e` bit for bit, and
retrieval over the reconstruction reproduces each record's `retrieved` list
exactly.

What is given up is only the accidental property that a composition stopped
loading whenever something *else* was admitted — which was never a check on
anything. `only=` is a **filter over the committed admission records, never a
source of them**: an id that was never admitted cannot be conjured by writing it
into a JSON file, and a test pins that.

**`6f9686b0` is still not executable, and the reason is unchanged** — it starves
on `gen_b_1f2fe8016b1441dc`, the superseded generation, which `admitted_stems`
drops so one mechanism is not in the library twice. That is right for the live
library and is exactly what makes this one record's library unrecoverable.
Reconstruction refuses it explicitly, with `UNRECONSTRUCTABLE_LIBRARY` naming
the missing node, rather than returning a short library that would hash to a tag
naming a set it is not. Finding 1 is still read off the plan structurally and
the half of the pair that runs still runs. **The user was offered an exemption
allowing a reconstruction to materialise a superseded node and declined it**, so
this stays a refusal by choice, not by oversight.

New public entry point: `compose.compose.library_as_recorded(composition_id)`.

---

## ⚠️ 2026-09-07 — finding 1 is not an anecdote, it is four nodes out of five, and finding 2 is 14 of 14

Chunk 39 measured both of the findings above rather than illustrating them, over
ten generated nodes on the three dev scenarios. Evidence and full tables:
`out/evidence/nodes/ab-comparison/README.md`.

**Finding 1 (starvation) generalises.** Put each generated node in front of a
fixed carrier — `Sequence[ContractPorts, Fallback[NODE, AvoidObstacles]]` — and
ablate it back to the carrier alone:

| arm | Δ goal_reached when the node is removed |
|---|---|
| A ×5 | `+0.000` on every one |
| B ×4 | `−0.333`, `−1.000`, `−1.000`, `−0.667` |
| B ×1 | `+0.000`, and it **never fires** (9/9) |

A negative delta means **the tree does better without the node**. Four of the
five condition-B nodes take a carrier that solves all three dev scenarios and
break it, by returning RUNNING and short-circuiting the `Fallback` — exactly the
mechanism `6f9686b0` demonstrated once. The composer that declined to sequence
the node ahead of a driving Action, three times out of three, was **right**, and
now there is a number saying so.

**Finding 2 (two effects) is universal, not a single node.** Every generated node
in the live library — **14 of 14** — both calls `ports.drive()` and publishes an
OUT port. Not one of them declares a single IN port. The §2 note asking chunk 17
to "decide whether an Action that drives may also publish a port" was written
about one violation; there is no generated node that does not violate it.

**The two findings are one cause.** `nodes/contract.py` has no *modifier* kind: a
speed cap is not a Condition, not an Action and not a Decorator as defined, so
every mechanism was forced into an Action with a carrier welded on. That gives a
B node exactly two ways to affect the robot — drive itself and starve whatever
else would have, or publish into the void — and the table above is both of them.
`plans/trash/40-modifier-kind.md` is the decision (moved 2026-09-21).

**⚠️ The reference row that reframes all of it.** The carrier alone —
`AvoidObstacles`, a handwritten fixture that is scaffolding under #3 and that
neither prompt has ever seen — reaches the goal on all three dev scenarios at
SPL 0.982. **Nine of the ten generated nodes fail to beat it**; the tenth,
`PathIntegrationGoalVector`, beats it by +0.007 SPL. Composition rate was already
not a proxy for goal-reach rate; this adds that **admission is not a proxy for
usefulness either**, and that the composed condition B being "toys only" was
costing less than it appeared to.

Kinematic, dev scenarios only, not reportable, not a G2 number — the same
standing caveats as everything else in this file.

---

## ⚠️ 2026-09-14 — a tree is now a LAYER, and the ordering decision is load-bearing

Chunk 52 replaced exclusive control with a layered command pipeline, and two things follow for
composition specifically.

**1. `Layer` joins `Sequence` and `Fallback` in the plan vocabulary.** It ticks every child, in
order, and position is priority: the base proposes a whole command and later children revise
the Twist fields they declared. Neither existing composite could do this — `Sequence` stops on
anything that is not SUCCESS and `Fallback` on anything that is not FAILURE, and the base
returns RUNNING every tick by its prompt's one hard requirement.

**2. ⚠️ LAYER ORDER IS A RESEARCHER DEGREE OF FREEDOM AND IS TREATED AS ONE.** With priority by
position, the composer's ordering decision changes what the tree does. It is therefore produced
**by the model, under a prompt identical across arms** — `prompts/compose.md`, which sees port
contracts and a task spec and never the papers (#6) — and is **never hand-tuned per arm**. A
tree whose node order a human adjusted is not reportable. The combination rule itself (per-field
last-writer-wins, `min()` for caps) is fixed, minimal and declared in `PRE-REGISTRATION.md` in
the 2026-09-14 amendment, for the same reason: it came from neither corpus, so B−A would
otherwise partly measure our arbiter.

**Finding 2 above is resolved structurally rather than by enforcement.** "An Action that drives
may not also publish a port" was decided and left unenforced. A modifier does not drive: it
revises declared fields and may publish an OUT port beside that, which is the shape the corpus
actually wanted and the reason fourteen nodes had a navigator welded on. The old violation
stands in the archived v2 records, reported and not hidden.

**Reconstruction reaches three pipelines now.** `RECONSTRUCTION_STAGING` walks live → v2 archive
→ v1 archive. `load_library()` with no argument is still live-only, so the current library stays
one experiment.

**⚠️ And `compose.md` is now SUPERSEDED, KEPT and NEVER EDITED**, joining
`generate/prompts/node.md` and `condition_{a,b,d}.md` under the same rule. The live prompt is
`compose_layered.md`; `compose.COMPOSE_PROMPT` points at it and `V1_COMPOSE_PROMPT` at the old
one. Editing `compose.md` in place is what was tried first, and it orphaned two committed
compositions within the minute — the transcript hash is over the RENDERED prompt, which is the
whole point of it. `render()` and `compose()` take an optional `prompt_file` so a committed
record can still be replayed against the prompt it was actually taken against, the same role
`generator.render_v1` plays for v1 nodes. It is not a knob: a live composition renders
`COMPOSE_PROMPT` or the comparison is between two different prompts.
