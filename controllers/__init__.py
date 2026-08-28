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
            f"controllers.compositions().")
    module_name, _, attr = _REGISTRY[name].partition(":")
    return getattr(importlib.import_module(module_name), attr)()
