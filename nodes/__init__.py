"""
nodes/

The traceable node library: leaf nodes with typed ports, the composites that
combine them, and the append-only store that holds them.

The registry mirrors `controllers/` — name to "module:attribute", so a node
resolves without an import chain per node and so a listing does not import any
of them. Generated nodes register here the same way the reference controllers
do, and for the same reason: `nodes/toys.py` is scaffolding and says so in its
own docstring, exactly as `controllers/goal_seeker.py` does.

Sim-free, like `core/` and `controllers/`: nothing here may import isaacsim,
omni, torch, pxr or carb, and nothing here may import
`core.scenarios_held_out` — composition sees port contracts and a task
specification, never the sealed split. Both are enforced by tests.
"""

import importlib

# name -> "module:attribute". Empty of real nodes on purpose: chunk 12 builds
# the contract, chunk 15 generates the nodes that fill this in.
_REGISTRY = {
    "toy_is_goal_known": "nodes.toys:IsGoalKnown",
    "toy_drive_to_goal": "nodes.toys:DriveToGoal",
}


def available():
    return sorted(_REGISTRY)


def node_class(name):
    """Resolve a node class by name. Raises KeyError listing the known ones."""
    if name not in _REGISTRY:
        raise KeyError(f"unknown node {name!r}. Known: {available()}")
    module_name, _, attr = _REGISTRY[name].partition(":")
    return getattr(importlib.import_module(module_name), attr)
