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

import os
import importlib
import importlib.util

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

# Names starting with this resolve to `Layer[ContractPorts, BASE, node]` — the
# GROWTH PAIRING, which is the tree `scripts/measure_growth.py` actually
# measures and the one every `fixed` / `broke` / `fire` / `changed` number is
# about. `node:` is the node ALONE and answers a different question; under the
# layered execution model a modifier alone drives nothing, so `node:<id>` on a
# v3 node is a tree that cannot navigate by construction.
#
# `measure_growth._build` is IMPORTED, never re-declared, for exactly the reason
# `node:` imports `build_tree`: two constructions of "base + node" would drift
# and only a wiring failure would say so.
#
# ⚠️ NOT A BENCHMARK CONDITION, for the same reason `node:` is not. It contains
# the shared base — a declared SUBSTRATE under non-negotiable #3's single
# exemption — plus one leaf, which is a probe, not an arm. `runs/spec.py`
# refuses it as a condition binding.
LAYER_PREFIX = "layer:"


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


BASE_NAME = "base"


def _library_entry(node_id, name):
    """node id -> the admitted library entry, or KeyError. ONE definition.

    Shared by the `node:` and `layer:` branches of make() so that "in the
    library" cannot mean two different things depending on which prefix was
    typed.
    """
    from compose.compose import RECONSTRUCTION_STAGING
    from compose.library import (CONTRACT_PORTS_ID, LibraryReconstructionError,
                                 load_library)
    # ⚠️ A REPAIR ROUND IS NOT IN THE LIBRARY AND MUST NOT BE SEARCHED FOR
    # THERE. `repair/loop.py` stages every round under
    # out/repair/<lineage>/round-NN/ and each round reuses the PARENT'S STEM —
    # `plan_hash` is a pure function of (condition, task, card) and a repair
    # re-makes neither — so three rounds and the original all carry the same
    # filename. Adding the round directories to RECONSTRUCTION_STAGING would
    # therefore be a four-way stem collision resolved by directory order, which
    # is the "resolver that guesses" that already cost one whole loop run
    # (see repair/evidence.py:entry_for). The id's own `__rNN` suffix names the
    # round, so delegate to the resolver that reads it and RAISES on ambiguity.
    #
    # This makes a repaired node runnable under physics — `layer:<id>__r01` —
    # which is the only way to confirm a loop verdict from outside the loop's
    # own kinematic machinery. It is not a widening of the benchmark surface:
    # `layer:` and `node:` are already declared not to be conditions,
    # `runs/spec.py` refuses both as a binding, and `library_nodes()` still
    # lists neither, so a repair round is reachable only by being named.
    if "__r" in node_id:
        from repair.evidence import entry_for
        try:
            return entry_for(node_id)
        except (KeyError, ValueError) as exc:
            raise KeyError(f"{name!r}: {exc}") from exc
    if node_id == CONTRACT_PORTS_ID:
        raise KeyError(
            f"{name!r} is the harness's own port publisher, not a "
            f"generated node. A tree containing only it drives nothing.")
    # `only=` is what refuses an id that is not admitted: load_library
    # rebuilds from the SAME committed records `pass_rate` reads, drops
    # superseded and retired ones, and raises when a requested id does not
    # materialise. Re-implementing that rule here would be a second
    # definition of "in the library" that could disagree with the first.
    try:
        # The same widened path `library_nodes()` lists over, for the same
        # reason: a committed result names its controller by id and must
        # stay re-scorable after the 2026-09-08 archive move.
        return load_library(staging=RECONSTRUCTION_STAGING,
                            only=[node_id]).get(node_id)
    except LibraryReconstructionError as exc:
        # Re-raised as KeyError so an unknown id fails the same way an unknown
        # registry name does. The underlying message is written for
        # composition's historical rebuild and reads as a non-sequitur here;
        # the cause is kept attached.
        raise KeyError(
            f"{name!r} is not an admitted library node today. It is "
            f"either unknown, superseded by a re-record, or retired with "
            f"its condition. See controllers.library_nodes().") from exc


def _growth_build():
    """`scripts/measure_growth.py:_build` — the growth pairing, by path.

    measure_growth is a script, not a package module, so this is the only way
    to import it. Worth the ugliness: the alternative is a second copy of the
    layering rule, and a `layer:` tree that differed from the measured one
    would make every physics run a comparison of two constructions.
    """
    spec = importlib.util.spec_from_file_location(
        "_measure_growth",
        os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "scripts", "measure_growth.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._build


def make(name):
    """Instantiate a controller by name, scaffolding included.

    Resolving by name is the explicit act; `available()` is the safe listing.
    """
    if name == BASE_NAME:
        # THE SHARED BASE NODE (chunk 47), as a one-leaf tree.
        #
        # ⚠️ Deliberately NOT in `_REGISTRY` and therefore not in `available()`.
        # It is a SUBSTRATE, not a condition: the same node in every arm, so it
        # cancels from every between-arm difference. Listing it beside
        # goal_seeker and reactive_avoid would invite it into a sweep spec as a
        # fifth condition, which #10 forbids and which would double-count it.
        # Resolvable by name because calibration must be able to run it —
        # `scripts/measure_base.py` is the whole reason it exists.
        from generate.admit import build_tree
        from generate.base import load
        candidate = load()
        return build_tree(candidate.node_cls, candidate.metadata)
    if name.startswith(NODE_PREFIX):
        # Imported here for the same cycle reason as the composed branch below.
        from generate.admit import build_tree
        entry = _library_entry(name[len(NODE_PREFIX):], name)
        return build_tree(entry.node_cls, entry.metadata)
    if name.startswith(LAYER_PREFIX):
        # The growth pairing, resolvable under physics. Which ids are
        # admissible is decided by the same helper the `node:` branch uses.
        from generate.base import load as load_base
        entry = _library_entry(name[len(LAYER_PREFIX):], name)
        return _growth_build()(entry.node_cls, entry.metadata, load_base())
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
            f"named {NODE_PREFIX}<node_id>; see controllers.library_nodes(). "
            f"That node layered on the shared base — the growth pairing — is "
            f"named {LAYER_PREFIX}<node_id>.")
    module_name, _, attr = _REGISTRY[name].partition(":")
    return getattr(importlib.import_module(module_name), attr)()
