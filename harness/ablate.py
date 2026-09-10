"""
harness/ablate.py

**Does this node help?** — asked of a TREE, over scenarios the generator did not
design.

WHY THIS EXISTS
----------------
Chunk 37 stopped gating admission on "does this node work alone in a world it
invented for itself". This is the question that replaces it, and it is a
different question in three ways that matter:

- **It is asked of a tree, not a leaf.** A speed limiter, a stuck-escape or a
  Condition cannot move a navigation metric alone; that is a fact about the leaf
  kinds, not about the mechanisms. In a tree they can.
- **It is asked over a SET of scenarios.** A node may never trigger in one scene
  and be decisive in another. The old gate killed such a node for showing
  nothing in the single world it happened to be tested in; a distribution over
  scenarios does not.
- **The scenarios are not the generator's.** `ExtendedSurfaceDetour` passes its
  own unit scenario, FAILS it when Cheng's mechanism is ablated, and still
  solves both dev scenarios with that mechanism gone. The micro-world was honest
  and unrepresentative at the same time, and no improvement to micro-worlds
  fixes that, because the generator picks them.

REPLACE, NEVER DELETE — THE LOAD-BEARING DETAIL
------------------------------------------------
Removing a node from a `Sequence` changes the tree's **control flow**, not just
its content: `Sequence` short-circuits on anything that is not SUCCESS and
`Fallback` on anything that is not FAILURE, so deleting a child silently
promotes every child after it. "Tree without X" would then differ from "tree
with X" in two ways and the measurement would be uninterpretable.

So the node is **swapped for a stand-in of the same kind** that returns what the
node returns when its precondition fails — `Status.FAILURE` — leaving the shape,
the child count and the composite ids exactly as they were.

`generate/admit.py` already named this as the way a Condition would have to be
tested: *"pairing it with a fixed consumer and comparing against the same tree
with the candidate replaced by a stand-in"*. One mechanism, two uses.

⚠️ **NOT A BENCHMARK PATH.** The sweep runs on `core/tests/kinematic.py` over the
DEV scenarios: real lab42 geometry, real sensing, real drift, real scoring — and
an idealised unicycle with the frozen locomotion policy absent. Representative
world, unrepresentative physics: one axis fixed, not two. No number produced here
may be reported; physics confirmation is chunk 36.

⚠️ **THE SEALED SPLIT IS NEVER USED HERE.** This measurement feeds back into
which nodes get built, so running it against the held-out scenarios is exactly
the leak G3 exists to prevent.

SIM-FREE.
"""

from nodes.contract import (Action, Condition, Decorator, Kind, Status)
from nodes.toys import toy_metadata
from nodes.tree import Composite, Tree

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# What a stand-in returns. FAILURE is not an arbitrary choice: it is what
# `nodes.tree.tick_node` returns for a node whose precondition is false, so an
# ablated node behaves exactly like the real one on a tick where it declines to
# act. The ablation therefore measures "this node never fires" and nothing else.
STANDIN_STATUS = Status.FAILURE


def _standin_class(kind, ports):
    """A leaf of `kind` declaring `ports`, that does nothing and fails.

    The ports are copied from the node being replaced so that `Tree`'s
    construction-time wiring check still passes — a tree whose producer vanished
    would not build, and "it did not build" is not a measurement.
    """
    base = {Kind.CONDITION: Condition, Kind.ACTION: Action,
            Kind.DECORATOR: Decorator}[kind]

    class StandIn(base):
        PORTS = tuple(ports)
        PRECONDITION = "never — this is an ablation stand-in"
        EFFECT = "nothing; it holds the tree's shape while a node is removed"
        TAGS = ("harness", "ablation", "scaffold")

        def precondition(self, obs, ports):
            return False

        def tick(self, obs, ports):              # pragma: no cover — gated out
            return STANDIN_STATUS

    return StandIn


def _rebuild(node, target_id, replaced):
    """Copy the tree, swapping `target_id` for a stand-in of the same kind."""
    if isinstance(node, Composite):
        return type(node)(node.metadata,
                          [_rebuild(c, target_id, replaced)
                           for c in node.children])
    if isinstance(node, Decorator):
        child = _rebuild(node.child, target_id, replaced)
        if node.metadata.node_id == target_id:
            replaced.append(target_id)
            return _standin_class(Kind.DECORATOR, node.PORTS)(
                toy_metadata(f"standin_{target_id}"), child)
        return type(node)(node.metadata, child)
    if node.metadata.node_id == target_id:
        replaced.append(target_id)
        return _standin_class(node.KIND, node.PORTS)(
            toy_metadata(f"standin_{target_id}"))
    # Reconstructed rather than reused: a leaf carries per-episode state, and
    # sharing an instance between the intact and ablated trees would leak it.
    return type(node)(node.metadata)


def ablate(tree, node_id):
    """`tree` with `node_id` replaced by a stand-in. Raises if it is not there."""
    replaced = []
    root = _rebuild(tree.root, node_id, replaced)
    if not replaced:
        raise KeyError(f"{node_id!r} is not in this tree; nothing was ablated")
    return Tree(root, snapshot=getattr(tree, "snapshot", None))


def ablatable(tree):
    """Node ids worth ablating: the leaves, minus the harness's own scaffold."""
    out = []
    for node in tree.nodes():
        if isinstance(node, Composite):
            continue
        node_id = node.metadata.node_id
        if node_id.startswith(("standin_", "unit_", "composed_")):
            continue
        out.append(node_id)
    return out


def score_tree(tree_factory, scenarios, seeds=(0,)):
    """Mean of each metric over `scenarios` x `seeds`, plus the per-scenario rows."""
    import dataclasses

    from core.metrics import score
    from core.tests.kinematic import rollout

    rows = []
    for scenario in scenarios:
        for seed in seeds:
            sc = dataclasses.replace(scenario, seed=int(seed))
            tree = tree_factory()
            log = tree.enable_trace()
            result = score(rollout(sc, tree), sc)
            rows.append({"scenario": sc.id, "seed": int(seed),
                         "score": result, "fired": log.summary()})
    return rows


def contribution(tree_factory, scenarios, seeds=(0,), metric="goal_reached",
                 node_ids=None):
    """Leave-one-out: what does each node add, per scenario and on average?

    Returns one entry per node with `mean` AND `per_scenario`. **Read the
    distribution, not only the mean** — a node that changes nothing on eleven
    scenarios and rescues the twelfth is exactly the case this exists for, and a
    mean hides it.

    `fired` is reported beside the effect so that "did not help" and "never
    triggered" stay distinguishable. The old gate reported them as one number.
    """
    intact = score_tree(tree_factory, scenarios, seeds)
    baseline = {(r["scenario"], r["seed"]): r for r in intact}
    ids = node_ids if node_ids is not None else ablatable(tree_factory())

    out = {}
    for node_id in ids:
        ablated = score_tree(lambda nid=node_id: ablate(tree_factory(), nid),
                             scenarios, seeds)
        per_scenario, deltas = {}, []
        for row in ablated:
            key = (row["scenario"], row["seed"])
            full = baseline[key]["score"].get(metric)
            gone = row["score"].get(metric)
            full = 0.0 if full is None else float(full)
            gone = 0.0 if gone is None else float(gone)
            delta = full - gone
            deltas.append(delta)
            seen = baseline[key]["fired"].get(node_id, {})
            per_scenario[f"{row['scenario']}@{row['seed']}"] = {
                "intact": full, "ablated": gone, "delta": delta,
                "ticks": seen.get("ticks", 0),
                "fired": seen.get("precondition_passed", 0),
                "drove": seen.get("drove", 0),
            }
        out[node_id] = {
            "metric": metric,
            "mean_delta": sum(deltas) / len(deltas) if deltas else 0.0,
            "n": len(deltas),
            "helped_in": sum(1 for d in deltas if d > 0),
            "hurt_in": sum(1 for d in deltas if d < 0),
            "never_fired_in": sum(1 for v in per_scenario.values()
                                  if v["fired"] == 0),
            "per_scenario": per_scenario,
        }
    return out
