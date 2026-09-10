"""
controllers/

Reference controllers for validating the harness. NOT benchmark conditions —
see each module's docstring.

The registry maps a name to a zero-argument factory so `run_episode.py
--controller <name>` resolves without an import chain per controller, and so
--help can list what exists without importing any of them. Generated conditions
will register here the same way.

Sim-free, like core/: nothing here may import isaacsim, omni or torch. The
import guard in core/tests/test_import_guard.py covers this package too.
"""

import importlib

# name -> "module:attribute"
_REGISTRY = {
    "goal_seeker": "controllers.goal_seeker:GoalSeeker",
    "reactive_avoid": "controllers.reactive_avoid:ReactiveAvoid",
    "scaffold_sweep": "controllers.scaffold_sweep:ScaffoldSweep",
    "toy_tree": "nodes.toys:toy_tree",
    "toy_search_tree": "nodes.toys:search_tree",
}

# Registered so `run_episode.py --controller <name>` can reach them, and kept
# OUT of available() so nothing can sweep them into an evaluation.
#
# This is the same shape as core.scenario.all_scenarios() excluding the sealed
# split, and for the same reason: a batch runner that iterates "everything"
# must not silently include things that are not conditions. Non-negotiable #3
# says scaffolding must never enter a benchmark condition; making available()
# the safe list is what turns that from a rule people remember into a rule the
# code enforces. Naming one is an explicit act; getting one by accident is not
# possible.
_SCAFFOLD = ("scaffold_sweep", "toy_tree", "toy_search_tree")


def available():
    """Benchmark-eligible controllers. Never includes scaffolding."""
    return sorted(n for n in _REGISTRY if n not in _SCAFFOLD)


def scaffolding():
    """Registered but never a condition. Each says so in its own docstring."""
    return sorted(_SCAFFOLD)


# Names starting with this resolve to a recorded composition rather than to an
# entry in _REGISTRY. That is how a composed behaviour tree reaches
# `run_episode.py --controller composed:<id>` with NO change to the runner and
# nothing registered by hand: a Tree is already a core.observation.Controller,
# so all that was ever missing was a way to name one.
#
# Deliberately a namespace rather than new registry entries. Compositions are
# produced by the pipeline, not written by a person, and a registry someone has
# to remember to update is a registry that goes stale. `available()` still lists
# only the static names, so nothing here can be swept into an evaluation by a
# batch runner that iterates the safe list.
COMPOSED_PREFIX = "composed:"

# Names starting with this resolve to ONE ADMITTED LIBRARY NODE, wrapped in the
# smallest tree that is a Controller — exactly the tree the admission decision
# was taken on. That is the whole point: chunk 36 puts a node under physics for
# the first time, and the thing under physics has to be the same thing the
# harness admitted, or the result compares two constructions rather than two
# worlds.
#
# `generate.admit.build_tree` is IMPORTED, never re-declared. Two constructions
# of "the admission tree" would drift, and only a wiring failure would say so.
#
# A `node:` controller is NOT a benchmark condition. It is one leaf plus the
# `ContractPorts` scaffold that feeds it, which is a probe, not a navigator, and
# `runs/spec.py` refuses it as a condition binding for the same reason it
# refuses `scaffold_sweep`. It stays out of available() accordingly.
NODE_PREFIX = "node:"


def library_nodes():
    """The ids of every admitted, non-superseded, non-retired library entry.

    Resolvable through make() as `node:<id>`, listed separately from
    available() because a single leaf is evidence about that leaf and never a
    condition. `ContractPorts` is excluded: it is the harness's own port
    publisher, so `node:observation_ports` would be a tree that only ticks
    scaffolding.
    """
    from compose.compose import RECONSTRUCTION_STAGING
    from compose.library import load_library, CONTRACT_PORTS_ID
    # LIVE **AND** ARCHIVED, deliberately, and it is the one place that wants
    # both. `results/<run_id>.json` commits seven episodes whose controller is
    # `node:gen_a_69724a1f2c31010e` and friends — v1 nodes, archived on
    # 2026-09-08 — and a committed result that can no longer name its own
    # controller is not reproducible, which is the working agreement this repo
    # runs on. Re-scoring one must keep working.
    #
    # This does NOT reopen the clean slate. `load_library()` with no argument
    # still reads the live directory alone, so the library G1 counts, G3
    # measures over and a composition retrieves from is the current pipeline's;
    # this widens only the RESOLUTION of a node id someone already recorded.
    return [NODE_PREFIX + e.node_id
            for e in load_library(staging=RECONSTRUCTION_STAGING).all()
            if e.node_id != CONTRACT_PORTS_ID]


def compositions():
    """The ids of every recorded composition that produced a tree.

    Resolvable through make(), listed separately from available() because a
    composed tree is a benchmark condition only once someone has said which
    composition and named the library snapshot it came from.
    """
    from compose.compose import available as composed_available
    return [COMPOSED_PREFIX + node_id for node_id in composed_available()]


def make(name):
    """Instantiate a controller by name, scaffolding included.

    Resolving by name is the explicit act; `available()` is the safe listing.
    """
    if name.startswith(NODE_PREFIX):
        # Imported here for the same cycle reason as the composed branch below.
        from compose.library import load_library, CONTRACT_PORTS_ID
        from generate.admit import build_tree
        node_id = name[len(NODE_PREFIX):]
        if node_id == CONTRACT_PORTS_ID:
            raise KeyError(
                f"{name!r} is the harness's own port publisher, not a "
                f"generated node. A tree containing only it drives nothing.")
        # `only=` is what refuses an id that is not admitted: load_library
        # rebuilds from the SAME committed records `pass_rate` reads, drops
        # superseded and retired ones, and raises when a requested id does not
        # materialise. Re-implementing that rule here would be a second
        # definition of "in the library" that could disagree with the first.
        from compose.compose import RECONSTRUCTION_STAGING
        from compose.library import LibraryReconstructionError
        try:
            # The same widened path `library_nodes()` lists over, for the same
            # reason: a committed result names its controller by id and must
            # stay re-scorable after the 2026-09-08 archive move.
            entry = load_library(staging=RECONSTRUCTION_STAGING,
                                 only=[node_id]).get(node_id)
        except LibraryReconstructionError as exc:
            # Re-raised as KeyError so an unknown `node:` id fails the same way
            # an unknown registry name does. The underlying message is written
            # for composition's historical rebuild and reads as a non-sequitur
            # here; the cause is kept attached.
            raise KeyError(
                f"{name!r} is not an admitted library node today. It is "
                f"either unknown, superseded by a re-record, or retired with "
                f"its condition. See controllers.library_nodes().") from exc
        return build_tree(entry.node_cls, entry.metadata)
    if name.startswith(COMPOSED_PREFIX):
        # Imported here rather than at module level: compose/ reaches back into
        # nodes/, which reaches back into controllers/ for the fixture leaves.
        # A module-level import would be a cycle.
        from compose.compose import load_composition
        return load_composition(name[len(COMPOSED_PREFIX):])
    if name not in _REGISTRY:
        raise KeyError(
            f"unknown controller {name!r}. Known: {available()}. "
            f"Scaffolding, resolvable but never a condition: {scaffolding()}. "
            f"A recorded composition is named {COMPOSED_PREFIX}<id>; see "
            f"controllers.compositions(). A single admitted library node is "
            f"named {NODE_PREFIX}<node_id>; see controllers.library_nodes().")
    module_name, _, attr = _REGISTRY[name].partition(":")
    return getattr(importlib.import_module(module_name), attr)()
