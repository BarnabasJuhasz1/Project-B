# 12 — The node contract and the library

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** [11a](../done/11a-task-contract.md) · **Needs Isaac Sim:** no

## Why

§4 is specific and the repo has none of it:

> individual behaviour tree leaf nodes, each with a typed port interface, one precondition, one
> effect, and no coupling to a specific scenario

`controllers/reactive_avoid.py` is the monolith this forbids — goal seeking, gap selection and
speed control in one `step()`, none separable, none testable alone. It is condition A's shape.

The contract is the interface three later chunks are written against, so it is worth getting
right before any of them exist. It is also what composition sees **instead of** the papers.

## Decided

**How much behaviour-tree machinery.** A full BT library (sequence, fallback, parallel,
decorators, tick semantics, blackboard) is a lot of surface for a 6-week project. The minimum
that satisfies §4 is: leaves with typed ports, the three leaf kinds, and enough composition to
express a tree. **Decided: write the minimum ourselves** rather than take a dependency —
`py_trees` brings ROS-flavoured assumptions and its own blackboard model, and `core/` is
deliberately dependency-light and sim-free.

**What a port is typed over.** Ports carry values between nodes, and the type system is what
composition reasons about without seeing papers. Propose a small closed vocabulary — `Pose`,
`Bearing`, `Speed`, `Ranges`, `Confidence`, `Flag`, `Region` — and make adding one a deliberate
act, because an open vocabulary makes retrieval-by-port meaningless. **Decided: that closed
seven-type vocabulary**, and adding an eighth is a chunk, not an edit.

**Blackboard or explicit wiring.** A blackboard is easier to generate against and much harder to
check: any node can read anything, and "no coupling to a specific scenario" stops being
enforceable. **Decided: explicit typed ports**, wired by the composition step.

**The search question is settled elsewhere and is no longer open here.** It was the one thing
this chunk and chunk 11 each deferred to the other. It now belongs to
[11a](../done/11a-task-contract.md), which lands before both: `Observation.goal` becomes
`Optional[Tuple[float, float]]`, `None` means the goal has not been disclosed, and detection
latches on line of sight within `DETECT_RADIUS`. **Nothing in this chunk may change the
`Observation` field list.** A node that wants a fact the contract does not carry is a finding to
record, not a licence to add a field mid-experiment.

The consequence for the port vocabulary is direct: a precondition port over "has the goal been
found" is a `Flag`, sourced from `obs.goal is not None`, and it is monotonic because 11a latches
it. Nodes must not each re-derive that.

## Build

- `nodes/contract.py`
  - `PortType` enum; `Port(name, type, direction)`.
  - `Node` base: `precondition(obs, ports) -> bool`, `effect` declared not executed,
    `tick(obs, ports) -> Status`, `metadata` carrying source span, extraction confidence, card id.
  - `Status` — `SUCCESS`, `FAILURE`, `RUNNING`. Three values, not more.
  - The three leaf kinds as base classes: `Condition`, `Action`, `Decorator`.
- `nodes/tree.py` — `Sequence`, `Fallback`, and a `Tree` that is a `core.observation.Controller`,
  so a composed tree drops into `run_episode.py` with no runner changes. That compatibility is
  the point of the whole contract; test it.
- `nodes/library.py` — append-only store. Add, list, retrieve by port type and by tag. Snapshot
  to a tag that goes in `manifest["library_snapshot"]`. **Nothing is ever removed** (#7).
- `nodes/__init__.py` — registry, mirroring `controllers/`.

## Done when

- `nodes/tests/test_contract.py`: a hand-written `Sequence` of two toy leaves drives
  `doorway_traverse` through `run_episode.py`'s own loop under `core/tests/kinematic.py`, with no
  change to `run_episode.py`.
- A node whose ports do not type-check is rejected at composition time, not at tick time.
- Every node carries non-empty metadata, or is rejected. G1 reports "the fraction carrying a
  source span with extraction confidence", which is unanswerable if the field is optional.
- `library.snapshot()` is stable: same nodes in, same tag out.
- Adding a node never mutates or removes an existing one; a test asserts append-only across two
  snapshots.
- `core/tests/test_import_guard.py` extended: `nodes/` is sim-free too, and `nodes/` does not
  import `core.scenarios_held_out` (chunk [11](../done/11-held-out-split.md)'s guard).
- A leaf ticking on a scenario with `obs.goal is None` neither raises nor silently reads a stale
  goal. The two toy leaves are enough to test this; every real search node depends on it.

## Not in scope

Generation, and any real node. Two toy leaves are enough to prove the contract holds.

---

## Outcome

**Done 2026-08-25.** `core/tests/` + `nodes/tests/` = **226 passed** (201 + 25), no simulator.
`results/` untouched.

Built `nodes/contract.py`, `nodes/tree.py`, `nodes/library.py`, `nodes/toys.py`,
`nodes/__init__.py`, `nodes/tests/test_contract.py`. The plan's decisions were taken as written:
minimal BT machinery hand-rolled, the closed seven-type port vocabulary, explicit typed ports
rather than a blackboard.

### Two decisions the plan left open that turned out to matter

**The Twist is not a port.** The port vocabulary has `Speed` and `Bearing` but the frozen
interface is `(vx, vy, wz)`, and nothing in the seven types is a Twist. Rather than add an eighth
type — which the plan explicitly makes a chunk-level decision, not an edit — the command is
deliberately outside the port system: **ports describe information flowing between nodes; the
Twist is the actuation channel, and it is the one thing that crosses out of the experiment.** An
Action emits one with `ports.drive(command)`.

The payoff is unplanned and larger than the problem: `drive()` raises for a Condition or a
Decorator, so **the three leaf kinds become a real distinction instead of a label**. §4 names three
kinds and gives no test that tells them apart; now there is one.

**"Explicit ports, not a blackboard" needed an enforcement mechanism.** The plan gave the reason —
"any node can read anything, and 'no coupling to a specific scenario' stops being enforceable" —
but not the mechanism. Each node is handed a `PortView` built fresh per tick, restricted to the
ports it declared; reading or writing anything else raises `PortAccessError`. Non-negotiable #4
is now checkable by a test rather than trusted to a generator's good manners, which matters
because chunk 15's whole point is that the generator's output is not trusted.

### Provenance: "no source" had to be a value, not an omission

The plan said "every node carries non-empty metadata, or is rejected", reasoning that G1's
fraction is unanswerable if the field is optional. But conditions A and D **legitimately** produce
nodes with no corpus source, so a missing span cannot simply be an error.

Resolved with a required `Origin` — `CARD`, `INVENTED`, `HANDWRITTEN` — and a `__post_init__` that
rejects a CARD node lacking any of card id, source span or confidence. So "invented" and "nobody
recorded it" are distinguishable, and `NodeMetadata.is_traceable` is exactly what G1 reports the
fraction of. `Library.provenance()` computes G1's numbers rather than anyone typing them into a
report.

`Confidence` is defined here as a three-point enum because node metadata carries the type; chunk
14 still owns writing the criteria into `cards/schema.py`.

### Wiring is static where it can be and loud where it cannot

`check_wiring()` runs at `Tree(...)` construction: every IN port must be produced, with a matching
type, by a node earlier in flattened tick order. The test that demonstrates this has **no
Observation anywhere in it** — the tree cannot be built, so it can never reach an episode. That is
the whole point: a wiring fault surfacing mid-episode would score a timeout and read as a bad idea
rather than a bad connection, and G3 counts solved scenarios.

**Known blind spot, recorded rather than discovered later:** a producer inside a `Fallback` branch
appears earlier in the flattening but may not actually run. The static check cannot see that.
`PortView` catches it at read time with a message naming exactly this cause. Worth revisiting if
generated trees turn out to use Fallback heavily.

### The snapshot tag is order-independent

`Library.snapshot()` hashes contents sorted by node id, so the same nodes give the same tag
regardless of admission order — a stronger property than the plan's "same nodes in, same tag out",
and the useful one, since two runs that built the library in different orders should not appear to
have used different libraries. Identity includes ports, effect and precondition, not just the id:
a node whose ports changed is a different node, and the tag must say so. `manifest["library_snapshot"]`
— None since the first run — now has something to hold, and a test asserts the connection.

### The import guard was verified by breaking it

Extending `core/tests/test_import_guard.py` to walk `nodes/` is one line, and a one-line change
that silently covers nothing is exactly the failure this guard exists to prevent. Checked by
planting `import torch` in `nodes/library.py`: the guard failed, naming torch. Restored, it passes.
`nodes/` was also already inside chunk 11's `SEALED_FROM`, so the held-out AST guard began covering
it the moment the directory existed.

### For chunk 13

`nodes.tree.tick_node` applies the precondition before the tick and returns FAILURE without
ticking when it is false. That is what makes a unit scenario able to isolate one claimed effect:
control the precondition, observe the effect or its absence. The harness should drive nodes
through `tick_node`, not by calling `tick()` directly, or it will be testing something the
benchmark never does.

### For chunk 15

`Library.add()` instantiates the candidate and throws the instance away, so `Node.__init__`'s
checks — real Kind, non-empty EFFECT, valid metadata, no duplicate port names — run at admission
rather than at first tick. A generated node that cannot be constructed is rejected with the reason,
which is the denominator G1's unit pass rate needs.

### Not in scope, as planned

No real nodes. The registry holds only the two toy leaves, which carry `Origin.HANDWRITTEN` and
say in their docstring that they are not navigation and must never appear in a benchmark
condition — the same warning `controllers/goal_seeker.py` carries, for the same reason.


### Verified under physics, after the fact (2026-08-25)

The headline claim — a composed tree drops into `run_episode.py` with no runner changes — was
demonstrated against `core/tests/kinematic.py` only when this chunk closed. It has since been run
for real: `nodes.toys.toy_tree` registered in `controllers/` and driven through
`scripts/run_episode.py --scenario blind_corner --controller toy_tree`, scored normally (timeout,
1 collision, `min_clearance` -0.031 — the expected "no sensing, walks into the divider" shape).
**A behaviour tree runs the benchmark path unmodified.**

Registering it exposed a hole that was not this chunk's, but was found by it: `controllers/`
had no separation between "resolvable" and "is a benchmark condition", so a Week 5 batch runner
doing `for c in controllers.available()` would have swept scaffolding into the evaluation. That is
the same failure as `all_scenarios()` returning the sealed split, and it now has the same fix —
`available()` is the safe list, `make()` resolves anything by name, and `run_episode.py` validates
`--controller` through `make()` rather than `choices=`.
