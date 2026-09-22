# Marked for deletion — obsolete under the layered command pipeline

> ⚠️ **STILL NOTHING HAS BEEN DELETED, AND THE GATE NOW MATTERS MORE, NOT LESS (2026-09-21).**
> The layered command pipeline this list was written for is **itself superseded** by
> `plans/07-epic-whole-controller.md`. That does **not** promote anything here to deletable —
> it does the opposite. `scripts/prune_obsolete.py --gate` refuses to delete until
> `measure_growth.py` shows a non-zero `fixed`, on the principle that **"we replaced it" and
> "the replacement is better" are different claims and only the second justifies throwing
> anything away**. The architecture has now been replaced twice without that claim ever being
> established, which is precisely the argument for keeping everything.
>
> ⚠️ **The node library, `compose/`, `nodes/tree.py:Layer` and every v1/v2/v3 record stay.**
> They still load, still carry their snapshot tags, and still back every committed composition
> and every pass-rate denominator. Epic 07's case is assembled out of them.
>
> **Superseded PLANS are a different thing and were moved, not deleted**, to
> `plans/trash/` on 2026-09-21, each with a per-file reason in `plans/trash/README.md`.

**Created 2026-09-14, chunk 52.** Nothing in this list has been deleted. It is
the list of things that become dead weight *once the layered pipeline is
confirmed to work*, written down now while the reasons are fresh, so that the
pruning is one reviewed commit rather than an archaeology exercise in three
weeks.

`scripts/prune_obsolete.py` is this file as code. It is **dry-run by default**
and it **refuses to delete anything until the confirmation gate below passes**.

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P scripts/prune_obsolete.py             # what would go, and why. Deletes nothing.
$P scripts/prune_obsolete.py --apply     # only if the gate passes
```

## The confirmation gate

The pipeline counts as confirmed when **both** of these are true, and the script
checks them rather than trusting a memory of them:

1. `generate/staging/` holds at least one **admitted** candidate stamped
   `pipeline_version: v3-layered`.
2. A v3 growth run (`out/growth/<tier>.csv`) shows **`fixed` > 0** for at least
   one node. That column was 0 in every row of round 1 and it is the number the
   whole chunk exists to move; until it moves, the new structure has not been
   shown to be better than the one it replaced and nothing should be thrown away
   on the strength of it.

⚠️ Until both hold, `--apply` refuses. `--force` overrides and says so loudly.

## Tier 1 — delete once the gate passes

These are superseded by a decision that is already recorded, and nothing reads
them.

| path | why it is dead |
|---|---|
| `harness/ablate.py` | **Removal-based ablation.** The user's decision, 2026-09-13: *"I will not measure a node's contribution by removing it from the tree. I will measure a node's contribution by adding it."* `scripts/measure_growth.py` is the addition-based measurement and it is what non-negotiable #9 now means. |
| `scripts/node_contribution.py` | The CLI over `harness/ablate.py`. Same decision, same reason. Also leave-one-out, also over lab42's dev scenarios, which stopped being the benchmark on 2026-09-13. |
| `harness/tests/test_ablate.py` | Tests for the above. Goes with them or it fails. |
| `scripts/rejudge.py` | A one-shot migration: re-judge v2 staging under chunk 37's redefined rule. It has been run, its output is committed, and the staging it reads is now in `generate/archive/v2-exclusive-control/`. |

## Tier 2 — delete only with the v2 archive, and probably never

These are **kept deliberately** and are listed so that nobody prunes them by
accident while tidying. They are coupled: the transcript hash is over the
*rendered* prompt, so deleting a prompt orphans every transcript taken against
it.

| path | what it backs |
|---|---|
| `generate/prompts/node.md` | every v2 node transcript, and `generate/base.py`'s spliced contract section — the base is **frozen** and was written against this text |
| `compose/prompts/compose.md` | every committed composition transcript. Superseded by `compose_layered.md`; reach it through `compose.V1_COMPOSE_PROMPT` |
| `generate/prompts/condition_{a,b,d}.md` | every v1 transcript |
| `generate/archive/v1-single-call/` | 8 committed compositions, 7 committed episode results, chunk 39's A/B table |
| `generate/archive/v2-exclusive-control/` | growth round 1, and every finding in `generate/FINDINGS.md` drawn from it |
| `results/*.json` (20) | historical lab42 results. Superseded as a benchmark, never as a record. |

**The rule these live under:** a record of a real model call, or of a real
scored episode, is evidence. Evidence is not deleted because it is no longer
convenient. If the user decides a finding needs no backing artefact, that is a
decision to record, not a cleanup.

## Tier 3 — narrow once the micro-world path has no live users

`harness/unit.py`'s `run_unit` / `UnitScenario` / `required_effect`, and
`cards/schema.py`'s `unit_scenario()`, implement the **micro-world contribution
measurement**. Chunk 37 stopped gating on it, chunk 44 moved contribution to the
tree, and chunk 52's modifier prompt does not ask for a `micro()` at all.

They are still live: `harness/gate.py` (#9's regression suite) calls `run_unit`,
and a candidate that supplies a `micro()` anyway is still measured on it.

**Do not delete. Revisit when** `harness/gate.py` has been rewritten against the
growth suite — that is the chunk that finishes non-negotiable #9 — and no staged
v3 candidate defines a `micro()`. `scripts/prune_obsolete.py` reports both
conditions so the answer is a command rather than a guess.

## Not on this list, and worth saying why

- **`controllers/reactive_avoid.py` and `goal_seeker.py`.** Reference
  controllers and the monolith the node contract exists to forbid. They are
  never a condition and they are never deleted.
- **lab42** (`scripts/build_scene.py`, `scenes/lab42_v1.usd`,
  `core/scene_spec.py`, the dev scenarios). No longer the benchmark; still the
  dev/debug and fast-loop scene, per non-negotiable #10.
- **The 11 v2 node modules.** Archived, not deleted — see Tier 2.
