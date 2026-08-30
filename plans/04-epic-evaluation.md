# Epic 04 — the evaluation sweep

**Covers:** execution-plan Weeks 5–6. **Chunks:** 28–32.
**Depends on:** epic [00](00-epic-reproducible-episode.md) and epic
[02](02-epic-node-library.md), both complete, and
[`PRE-REGISTRATION.md`](../PRE-REGISTRATION.md), tagged `prereg-v1`.

## Why this epic exists

Everything built so far produces **one** episode. `run_episode.py` runs a scenario with a
controller, scores it and writes a manifest; `core/stats.py` says what to do with a *set* of
scored episodes. Nothing in the repo turns the first into the second.

That gap is not a loop. Four conditions × 15 scenarios × 3 seeds is 180 episodes and
roughly ten hours of container time, run against model-generated controllers that may crash,
on a machine that may not survive the night, producing files that must satisfy non-negotiable
#10 exactly or the comparison is void. The properties that matter are **resumability**,
**refusing to produce an unreportable number**, and **refusing to compare two conditions that
did not run the same episodes** — none of which a shell loop has.

This epic builds the sweep, the aggregation, the missing fourth condition, and the container
run that makes any of it reportable.

## What is already load-bearing

- `scripts/run_episode.py` — one episode, scored, with provenance. The sweep drives it and
  does not reimplement it.
- `core/stats.py` — `decide()` is G2's rule. The aggregation calls it; nothing recomputes it.
- `core/preregistration.py` — the digest every result carries, so a sweep can prove what it
  ran under.
- `results/<run_id>.json` — the committed projection. **Resumability comes from this
  directory existing**, not from a state file the sweep would have to keep honest.
- `controllers.available()` / `.scaffolding()` / `.compositions()` — the safe listing that
  keeps non-negotiable #3 mechanical.

## The honest dependency picture

Chunk 28 is the only one of the five that is unblocked today. The rest are blocked on things
this epic cannot decide by itself, and saying so here is cheaper than discovering it in
Week 5.

| Blocker | Blocks | Status |
|---|---|---|
| `git tag prereg-v1` | every reported number | needs a commit; the tree is untracked |
| **The library is one node.** G1 asks for ≥20, and conditions A and D have **zero** admitted nodes each | 29, 31, 32 | a condition with an empty library composes nothing. This is the largest real gap in the project |
| No card corpus on any machine | growing the library, G1's provenance fraction | Week 2's batch card extraction was never built |
| Condition C (Nav2) does not exist | B−C only | chunk 30 |
| The container run has never been exercised | every reported number | chunk 31 |

**Chunk 29 is the one that decides whether this cycle has a result at all.** Four generation
calls produced one admitted node. G1's target is ≥20 *per condition*. Either generation runs
at a scale nobody has attempted yet, or the write-up reports a coverage count of one against
a target of twenty and says why. That is a project-level decision, not a chunk-level one, and
it is flagged here rather than buried.

## Chunks

| # | Chunk | Depends on | Needs Isaac Sim |
|---|---|---|---|
| 28 | [The batch runner and aggregation](done/28-batch-runner.md) | 17 | no (exercised on the dev set) |
| 29 | [Grow the libraries to a reportable size](doing/29-grow-libraries.md) | 15, 28 | no |
| 30 | Condition C — the Nav2 baseline | 28 | **yes** |
| 31 | The container run | 28, 29 | **yes** |
| 32 | G3 — composition over the held-out split | 16, 29, 31 | **yes** |
| 33 | [The citations probe](todo/33-citation-probe.md) — does extraction earn its place? | 14, 15 | no |

**28 is done.** The rest are named so the shape is visible; each gets written when its blockers
clear, because a plan written against a blocked dependency is a guess.

## Design decisions that belong to the epic, not to one chunk

### One subprocess per episode

Not one process running many episodes. Three reasons, in order of weight:

1. **A model-generated controller may crash the process.** `generate/` and `compose/` both
   execute model-written Python with no sandbox, and a segfault inside the simulator must
   cost one episode, not the sweep.
2. **`World` is a singleton** with known re-entrancy gotchas already recorded in `CLAUDE.md`.
   Reloading a scene in-process is a source of state nobody has characterised.
3. **The determinism guarantee was measured through subprocesses.**
   `scripts/check_determinism.sh` runs `run_episode.py` twice and diffs the traces, so
   subprocess-per-episode is the configuration the bitwise claim was made about. Running
   them in-process would be a different claim needing a new measurement.

The price is a simulator boot per episode — about 20 s, so roughly an hour across 180
episodes against a sweep already near ten. Accepted, and measured rather than assumed.

### Resumability comes from `results/`, never from a state file

An episode is done if its committed result exists and is reportable. A state file is a second
source of truth that can disagree with the first, and the disagreement always surfaces at the
worst moment. The consequence is that "which episodes are still missing" is answerable by
`ls`, and re-running a partial sweep is safe by construction rather than by care.

### A sweep refuses to produce a number it may not report

`results/README.md` lists what may not be reported — `in_container: false`, `git_dirty: true`,
scaffolding controllers, `library_snapshot: null`, `prereg_tag: null`. Those are currently
prose. The sweep checks them **before booting the simulator** and refuses, with an explicit
opt-out for dev-set rehearsals whose output goes somewhere that is not `results/`.

### Aggregation refuses an unbalanced comparison

Non-negotiable #10 is identical episode counts across conditions. `core.stats.decide()`
already raises when two conditions cover different scenarios; the aggregation must not paper
over that by scoring what it has. A condition that crashed on two scenarios is not a weaker
result, it is a different benchmark.

### Scenario-major, condition-minor

The sweep iterates scenarios on the outside and conditions on the inside. Determinism means
the order cannot change any result, so the ordering is chosen for what a *partial* sweep
tells you: interrupted after two hours, scenario-major leaves a balanced set across all four
conditions, and condition-major leaves one condition finished and three untouched.
