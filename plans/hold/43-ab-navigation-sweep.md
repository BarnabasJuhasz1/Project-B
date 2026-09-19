> ⚠️ **ON HOLD since 2026-09-16, on the user's instruction.** All work is paused while
> the autonomous diagnose-and-repair loop is built — see [`../../REPAIR-LOOP.md`](../../REPAIR-LOOP.md)
> and [epic 06](../06-epic-repair-loop.md). Nothing here is cancelled or superseded; this file is
> intact and resumes where it stopped. Move it back to `plans/doing/` to restart.

# 43 — The first A vs B numbers under physics, on one task family

**Status:** doing. **Needs Isaac Sim** for the last step only. **Blocked by:** nothing.
**Supersedes nothing.** This is chunk 42's successor: 42 built the two-call pipeline and
left the v2 library empty.

---

## Read this first if you are a fresh session

`P=~/miniforge3/envs/env_isaacsim/bin/python`, run from `Repo/`. Read `CLAUDE.md`,
`generate/FINDINGS.md`, `compose/FINDINGS.md`, and the three-part Outcome of
`plans/done/42-card-first-generation.md`.

⚠️ **Do not open `core/scenarios_held_out.py`.** This chunk generates nodes, and chunk 11's
procedural gate says generation must happen in a session that has never had that file in
context. Nothing here needs it: **this chunk runs on the dev set only.**

## What this chunk is for

To put a number next to A and B on one task family, under physics, end to end — generation
through composition through a scored sweep. Not to report one. The scope is deliberately
one family (`navigate_to_goal`) and one split (`dev`), because the question being answered
is *"does the whole pipeline produce a navigator at all?"*, and that question does not need
the sealed split and must not spend it.

**D is paused, on the user's instruction.** Its one attempted row cost $4.69 and 157,894
thinking tokens before exiting 1; the record is
`generate/staging/d__navigate_to_goal__88c49cee31e01477.error.json`. Resuming D is its own
chunk.

## What was missing, measured before starting

| # | Gap | Size |
|---|---|---|
| 1 | The library held **one node per arm** — `limit_speed_dwa` for A and for B, nothing else | 8 rows, 16 model calls |
| 2 | **Composition could not be restricted to one arm's nodes** | small code change |
| 3 | No composition existed over the v2 library — all 8 committed ones name v1 nodes | 2 model calls |
| 4 | No sweep spec bound A and B to composed trees | a file |

Gap 2 was the only real code gap, and it is the one that decides whether the comparison
means anything. `compose.compose()` already took a `library` argument, but
`load_library()` returned **everything** and `scripts/compose_tree.py` had no way to ask for
less. A and B share one task family and one card set, so an unpartitioned library lets
condition A retrieve a node condition B wrote — and B−A stops measuring extraction.

## Decided: the partition is by CONDITION, not by ORIGIN

`load_library(conditions=("a",))` filters on the `condition` recorded in the admission
record — the same field the row key, the stem and the node id are built from. Rejected:
filtering on `NodeMetadata.origin`. Origin is *derived* at load time from which of
`card_id` / `citation_id` is set, so it is a second definition of "whose node is this"
that can disagree with the first. One definition, and it is the one the record was written
under.

Three consequences, all wanted:

- **`ContractPorts` is exempt and always present.** It is the harness's own port publisher
  and belongs to no arm; a partition that dropped it could not satisfy a single input port,
  and the resulting library gap would look exactly like a retrieval failure.
- **An arm with nothing admitted is empty, not an error** — unlike `only`, which raises.
  "This condition has generated nothing yet" is a real state of the experiment.
- **A partition hashes to a different snapshot tag than the union**, which is the point:
  #8 says a result names the library it came from, and what A was composed over is A's
  partition.

`Composition.conditions` records it. The key is **omitted when None** rather than written
as null, so the eight compositions committed before the partition existed are not rewritten
to say something they already said.

## Build

1. `load_library(conditions=...)` in `compose/library.py`, filtered at the record before the
   model-written Python is read or executed.
2. `Composition.conditions`, threaded through `compose()` and `repair()`, recorded in
   `as_dict()`, honoured by the no-`considered` reconstruction fallback.
3. `--condition` on `scripts/compose_tree.py`, repeatable, with a guard that refuses to
   compose over a partition holding nothing but the port publisher.
4. Four tests in `compose/tests/test_library.py`: disjointness and completeness of the
   partition, the port publisher's exemption, the empty arm, and tag divergence.
5. Generate the 8 missing `navigate_to_goal` rows for A and B.
6. Compose one tree per arm, over that arm's partition.
7. A sweep spec binding A and B, `reportable: false`.
8. Run it on the dev set.

## Done when

- [x] `load_library(conditions=("a",))` and `("b",)` return disjoint sets, both containing
      `ContractPorts`, hashing to different tags from each other and from the union.
- [x] The full sim-free suite passes with no committed record rewritten.
- [ ] Each arm has ≥2 admitted `navigate_to_goal` nodes.
- [ ] One composition per arm, `ok: true`, over that arm's partition.
- [ ] `run_sweep.py --dry-run` accepts the spec.
- [ ] 18 dev episodes scored, aggregated, and read.

## Not in scope

- **The held-out split.** Dev only. The seal is not spent on a pipeline shakedown.
- **A reportable number.** This runs locally, so every result carries `in_container:false`.
  §6 of the description says a reported number comes from `nvcr.io/nvidia/isaac-sim:6.0.1`,
  and this chunk does not pretend otherwise.
- **Condition D**, paused above.
- **Chunk 40's modifier kind.** 39 showed every generated node drives *and* publishes and
  none reads a port. That is still true and this chunk does not fix it — it measures what
  the pipeline produces as it stands.

## Risks known before the run

1. **A's `limit_speed_dwa` node declines to act in open space.** Its precondition demands a
   forward return inside 2.5 m, and its docstring says so outright: *"An unconstrained
   window needs no search; yield to straight-line pursuit."* B's node has no such
   precondition — `goal is not None and not obs.contact` — and drives throughout. If none of
   A's other four nodes drives toward the goal in the open, **A's tree cannot navigate, for a
   reason that has nothing to do with the citation treatment.** This is chunk 40's finding
   arriving in a new place and it is the single most likely way this chunk produces a
   misleading table.
2. **Contribution is going unmeasured.** Both v2 rows admitted so far carry *"the candidate's
   own micro-world is malformed: goal is too close to a wall for the whole GOAL_RADIUS disc
   to be reachable"*. Two for two is a pattern. Admission still happens; the contribution
   number chunk 37 added does not.
3. **`pass_rate` on n=1 per arm is not a rate.** It reads 1.0 / 1.0 today and will stay
   uninformative at n=5.

## Found while preparing, not caused by this chunk

⚠️ **Four of the seven tree-producing committed compositions no longer resolve.**
`controllers.make()` raises `UNRECONSTRUCTABLE_LIBRARY` on them, all naming the same node:

```
gen_b_1f2fe8016b1441dc were in the library when this composition was recorded
and are not admitted entries today. The usual cause is a SUPERSEDED generation.
```

| composition | resolves |
|---|---|
| `navigate_to_goal__…__56b3466284b7d9e8` | ✅ |
| `navigate_to_goal__…__6f9686b026c3f472` | ❌ |
| `navigate_to_goal__…__9c21b87c9c158f0b` | ❌ |
| `r_search_for_goal__…__6004b2deaf87a659` | ✅ |
| `r_search_for_goal__…__9532339b5987cf40` | ❌ |
| `search_for_goal__…__11022fcc9c42c02a` | ❌ |
| `search_for_goal__…__2b74bf7b40d6ed9a` | ✅ |

**Verified pre-existing**: measured 3 ok / 4 fail with this chunk's changes stashed, so the
arm partition did not cause it. The cause is a supersession, not the archive move —
`admitted_stems` drops a superseded stem, which is right for the live library and is exactly
what makes the historical one unrecoverable.

This **contradicts `CLAUDE.md`**, which says the eight committed compositions "all still
resolve, through `compose.compose.RECONSTRUCTION_STAGING`", and chunk 42's part-3 Outcome,
which claims the same. Both were true of the archive move and neither accounted for the
supersession. **Not fixed here** — it is evidence about `RECONSTRUCTION_STAGING`'s limits and
deserves its own chunk, and nothing in this chunk depends on those four records.
