# compose/ — findings

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
