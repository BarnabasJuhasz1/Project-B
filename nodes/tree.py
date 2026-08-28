"""
nodes/tree.py

Composites, and the Tree that is a `core.observation.Controller`.

That compatibility is the point of the whole contract. `Controller` was made two
methods so "a generator emitting a behaviour tree, a hand written baseline and a
Nav2 bridge all fit"; this is the chunk that cashes that in. A composed tree
drops into `scripts/run_episode.py` with **no change to the runner**, and
`nodes/tests/test_contract.py` asserts exactly that.

WIRING IS CHECKED WHEN THE TREE IS BUILT, NOT WHEN IT TICKS
-----------------------------------------------------------
A node whose ports do not type-check is rejected at composition time. Catching
it at tick time would mean a generated tree fails in the middle of an episode,
scores a timeout, and looks like a bad idea rather than a bad wiring — and G3
counts solved scenarios, so those are not the same result at all.

The static check is deliberately the simple one: every IN port must be produced,
with the same type, by a node appearing EARLIER in flattened tick order. It has
a known blind spot, stated here rather than discovered later — a producer inside
a `Fallback` branch appears earlier in the flattening but may not actually run.
`PortView` catches that at read time with a message that says so. Static where
it can be, loud where it cannot.
"""

from core.observation import STOP, Command

from nodes.contract import (Direction, Kind, Node, PortStore, PortView, Status)


class WiringError(ValueError):
    """A tree whose ports do not connect. Raised at construction."""


class Composite(Node):
    """Internal node. Not a leaf, so it declares no ports and no effect of its
    own — a composite's effect is whatever its children do, and pretending
    otherwise would put text in front of composition that describes nothing."""

    KIND = Kind.DECORATOR      # governs children without emitting a Twist
    EFFECT = "control flow only"

    def __init__(self, metadata, children):
        super().__init__(metadata)
        children = tuple(children)
        if not children:
            raise WiringError(
                f"{metadata.node_id}: a composite with no children is a node "
                f"that cannot do anything; leave it out instead.")
        for child in children:
            if not isinstance(child, Node):
                raise WiringError(f"{metadata.node_id}: {child!r} is not a Node")
        self.children = children

    def reset(self, scenario, rng):
        for child in self.children:
            child.reset(scenario, rng)


class Sequence(Composite):
    """Tick children in order until one does not succeed.

    FAILURE or RUNNING from any child short-circuits. All SUCCESS is SUCCESS.
    This is the "and then" of a behaviour tree.
    """

    def tick(self, obs, ports):
        for child in self.children:
            status = tick_node(child, obs, ports.store)
            if status is not Status.SUCCESS:
                return status
        return Status.SUCCESS


class Fallback(Composite):
    """Tick children in order until one does not fail.

    SUCCESS or RUNNING from any child short-circuits. All FAILURE is FAILURE.
    This is the "or else" — the recovery idiom, and the reason a stuck-escape
    node can sit behind a normal-driving node without either knowing about the
    other.
    """

    def tick(self, obs, ports):
        for child in self.children:
            status = tick_node(child, obs, ports.store)
            if status is not Status.FAILURE:
                return status
        return Status.FAILURE


def tick_node(node, obs, store):
    """Tick one node, with its precondition and its restricted port view.

    The precondition gates the node: false means FAILURE without ticking. That
    is what "one precondition" means operationally — it is not advisory text, it
    decides whether the effect happens, and a unit scenario can therefore
    isolate the effect by controlling the precondition.

    Records to `store.log` when tracing is on. The recording is the ONLY thing
    tracing changes: the status returned and the Twist emitted are identical
    either way, which is what lets a traced run be compared with an untraced one.
    """
    view = PortView(store, node)
    log = store.log
    passed = node.precondition(obs, view)
    if not passed:
        if log is not None:
            log.record(node, False, Status.FAILURE, False)
        return Status.FAILURE

    if log is None:
        return node.tick(obs, view)

    before = store.command
    log.enter()
    status = node.tick(obs, view)
    log.leave()
    log.record(node, True, status, store.command is not before)
    return status


def flatten(node):
    """Every node in tick order, composites included. What the wiring check and
    the library's port index both walk."""
    yield node
    for child in getattr(node, "children", ()):
        yield from flatten(child)
    child = getattr(node, "child", None)
    if child is not None:
        yield from flatten(child)


def check_wiring(root):
    """Raise WiringError unless every IN port is produced, with a matching type,
    by a node earlier in tick order. Returns the produced port map."""
    produced = {}
    for node in flatten(root):
        for port in node.ports:
            if port.direction is Direction.IN:
                if port.name not in produced:
                    raise WiringError(
                        f"{node.metadata.node_id} reads port {port.name!r} "
                        f"({port.type.value}) but nothing earlier in the tree "
                        f"writes it. Produced so far: {sorted(produced)}.")
                have = produced[port.name]
                if have is not port.type:
                    raise WiringError(
                        f"{node.metadata.node_id} reads port {port.name!r} as "
                        f"{port.type.value}, but it is produced as "
                        f"{have.value}. Port types are the only thing "
                        f"composition can reason about without seeing the "
                        f"papers; a mismatch here is not recoverable at tick "
                        f"time.")
        for port in node.ports:
            if port.direction is Direction.OUT:
                clash = produced.get(port.name)
                if clash is not None and clash is not port.type:
                    raise WiringError(
                        f"{node.metadata.node_id} writes port {port.name!r} as "
                        f"{port.type.value}, already produced as {clash.value}.")
                produced[port.name] = port.type
    return produced


class Tree:
    """A behaviour tree, and a `core.observation.Controller`.

    Nothing about the runner knows this is a tree. It has reset() and step(),
    which is the entire contract, and `run_episode.py` is unchanged.
    """

    def __init__(self, root, snapshot=None):
        check_wiring(root)
        self.root = root
        self.snapshot = snapshot        # the library tag, for non-negotiable #8
        self._store = PortStore()
        self.tick_log = None

    def enable_trace(self):
        """Record which node ticked, when, and which one drove.

        Opt-in, and off by default: a benchmark run must not pay for a
        diagnostic, and must behave identically with it on. `run_episode.py`
        turns it on for any controller that offers it, so a generated tree
        becomes explicable without the runner knowing what a tree is.
        """
        from nodes.trace import TickLog
        self.tick_log = TickLog()
        self._store.log = self.tick_log
        return self.tick_log

    def reset(self, scenario, rng):
        self._store.clear()
        self._store.log = self.tick_log     # clear() must not drop the log
        self.root.reset(scenario, rng)

    def step(self, obs):
        """One control step. Ports do NOT persist across steps.

        Clearing the store every step is a decision. A port that survived would
        be episode memory smuggled through the wiring, invisible to the port
        types composition reasons about; a node that needs memory keeps it on
        itself, where reset() clears it and a reader can see it.
        """
        self._store.clear()
        self._store.log = self.tick_log
        if self.tick_log is not None:
            self.tick_log.begin_step(obs.t)
        tick_node(self.root, obs, self._store)
        command = self._store.command
        return STOP if command is None else command

    def nodes(self):
        return list(flatten(self.root))

    def __repr__(self):
        leaves = [n for n in self.nodes() if not isinstance(n, Composite)]
        return f"<Tree {len(leaves)} leaves snapshot={self.snapshot}>"
