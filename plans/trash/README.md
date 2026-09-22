# plans/trash/ — superseded by the whole-controller architecture (2026-09-21)

⚠️ **NOTHING IN HERE IS DELETED, AND NOTHING IN HERE IS CANCELLED WORK THAT MIGHT COME BACK.**
These are chunks whose *deliverable no longer exists*. They plan against the behaviour-tree
execution model that [epic 07](../07-epic-whole-controller.md) drops.

This folder follows the same rule as `generate/archive/` and `scripts/prune_obsolete.py`: a
superseded record is kept, because the record is how a reader learns why the architecture
changed. "We replaced it" and "the replacement is better" are different claims, and only the
second would justify throwing anything away — see `DELETION-CANDIDATES.md`.

**A file lands here only if both are true:** it is not in `done/`, and its deliverable is a
behaviour-tree artefact (a node, a tree, a `Layer`, a composition, an arbitration rule).

| file | what it planned | why it is here |
|---|---|---|
| `40-modifier-kind.md` | A fourth `Kind` in `nodes/contract.py` so the contract could express a modifier | Self-marked superseded by 52 on 2026-09-14. There is no node contract on the live path. **Its diagnosis is where the inertness of B was first identified** and is carried into epic 07's "why the tree is the part to drop". |
| `44-tree-level-contribution.md` | Admission moves from the micro-world to improving an incumbent **tree** | There are no trees. The idea survives in a changed form — a generated controller earns its place by beating the base — and epic 07 inherits it. |
| `43-ab-navigation-sweep.md` | A/B sweep over the **node library**, arm-partitioned retrieval | Partially landed (`dfe022a`, library partitioned by arm). The retrieval step it protects does not exist without composition. |
| `52-layered-command-pipeline.md` | `Layer`, `WRITES`, `ports.cap` / `ports.adjust`, the arbitration rule | **This is the architecture being dropped.** It was largely built; `nodes/tree.py:Layer` and the v3 regeneration are its output, and the v2 archive exists because of it. Kept as the full statement of the design whose failure motivated epic 07. |

## What did NOT come here, and why

- **Everything in `done/`.** Those carry Outcome sections and are the project's evidence base.
  Epic 07's argument is built out of them.
- **The epics `00`–`06`.** They are the record of how the architecture arrived where it did.
  `02-epic-node-library.md` and `06-epic-repair-loop.md` carry dated superseded banners instead.
- **`33-citation-probe.md` and `53-physics-on-a-generated-world.md`**, which were in `hold/`.
  Both are architecture-independent — one is card-level, one is a physics confirmation — and
  both went back to `todo/` rather than here.
