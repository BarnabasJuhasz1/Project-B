"""
nodes/contract.py

The leaf-node contract from §4: individual behaviour-tree leaves, each with a
typed port interface, one precondition, one effect, and no coupling to a
specific scenario.

`controllers/reactive_avoid.py` is the monolith this exists to forbid — goal
seeking, gap selection and speed control welded into one `step()`, none
separable, none testable alone. That is condition A's natural shape, which is
why it is a useful reference and why it can never be a condition.

FOUR THINGS THIS FILE DECIDES
-----------------------------
**A closed port vocabulary.** Seven types, listed below. Composition reasons
about ports without ever seeing a paper, so retrieval-by-port is only meaningful
while the vocabulary is small and shared. Adding an eighth is a chunk, not an
edit — an open vocabulary makes `retrieve(PortType.X)` return one node and no
longer be retrieval at all.

**Explicit ports, not a blackboard.** A blackboard is easier to generate against
and much harder to check: any node can read anything, and "no coupling to a
specific scenario" stops being enforceable. Each node is handed a PortView
restricted to the ports it declared, and touching anything else raises. The
restriction is the enforcement.

**The Twist is not a port.** It is the frozen interface, the one thing that
crosses out of the experiment, and it is deliberately outside the port system:
ports describe information flowing between nodes, not the actuation channel. An
Action emits one with `ports.drive(command)`. Conditions and Decorators cannot,
which is what makes the three leaf kinds a real distinction rather than a label.

**Provenance is required, and "no source" is a value rather than an omission.**
G1 reports "the fraction carrying a source span with extraction confidence".
Conditions A and D legitimately produce nodes with no corpus source, so a
missing span cannot be an error — but if it were simply absent you could not
tell "invented" from "forgot to record it", and the fraction would be
unanswerable. So `origin` is required and explicit, and a CARD node must name
its card and its extraction confidence or it is rejected. It need not carry a
span: an unverified card yields a node that is CARD and *untraceable*, which is
a state chunk 14 made first-class and `is_traceable` still refuses to count.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple


class Status(Enum):
    """Three values, not more. A richer status set is the first step towards a
    tick semantics nobody can hold in their head, and composition has to reason
    about these without running them."""

    SUCCESS = "success"
    FAILURE = "failure"
    RUNNING = "running"


class PortType(Enum):
    """The closed vocabulary. See the module docstring before adding to it."""

    POSE = "Pose"              # (x, y, yaw) — a belief, never ground truth
    BEARING = "Bearing"        # radians, robot frame unless a port says otherwise
    SPEED = "Speed"            # m/s, non-negative
    RANGES = "Ranges"          # an array of beam returns, metres
    CONFIDENCE = "Confidence"  # [0, 1], how much to trust something
    FLAG = "Flag"              # a boolean fact about the episode
    REGION = "Region"          # an area of the floor: (x, y, radius)


class Direction(Enum):
    IN = "in"
    OUT = "out"


class Kind(Enum):
    CONDITION = "condition"
    ACTION = "action"
    DECORATOR = "decorator"


class Origin(Enum):
    """Where a node came from. Required on every node, because it is the
    denominator and the numerator of G1's provenance fraction."""

    CARD = "card"                  # extracted from a principle card (condition B)
    INVENTED = "invented"          # the model devised the principle (condition D)
    HANDWRITTEN = "handwritten"    # a human wrote it: toys, harness fixtures


class Confidence(Enum):
    """Extraction confidence, three points rather than an uncalibrated float.

    The written criteria for each level belong to `cards/schema.py` and are
    chunk 14's job; the TYPE belongs here, because node metadata carries it and
    this module decides what metadata must contain.
    """

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


@dataclass(frozen=True)
class Port:
    name: str
    type: PortType
    direction: Direction

    def __post_init__(self):
        if not self.name:
            raise ValueError("a port must have a name")


@dataclass(frozen=True)
class NodeMetadata:
    """Provenance. Required, and validated on construction rather than trusted.

    `model_id`, `prompt_hash` and `temperature` are None for handwritten nodes
    and required in practice for generated ones — chunk 15 enforces that at
    admission, because it is the chunk that knows what generated the node. This
    module only enforces what is true of every node everywhere.
    """

    node_id: str
    origin: Origin
    card_id: Optional[str] = None
    source_span: Optional[str] = None
    confidence: Optional[Confidence] = None
    model_id: Optional[str] = None
    prompt_hash: Optional[str] = None
    temperature: Optional[float] = None
    # Which mechanism this node is a derivation OF, carried through from the
    # card (`Card.family`). G1's coverage measure is DISTINCT ADMISSIBLE
    # PRIMITIVES, and node_id is 1-1 with the source, so len(library) counts
    # SOURCES. Two papers arguing one principle produce two nodes and one
    # mechanism; without this field the library reports the corpus's advantage
    # as twice what it is. None means "its own mechanism", which is right for
    # an INVENTED node: nothing pairs with it.
    mechanism_family: Optional[str] = None

    def __post_init__(self):
        if not self.node_id:
            raise ValueError("node metadata must carry a node_id")
        if not isinstance(self.origin, Origin):
            raise ValueError(
                f"{self.node_id}: origin must be an Origin, not {self.origin!r}. "
                f"It is required because G1's provenance fraction cannot "
                f"distinguish 'invented' from 'nobody recorded it' otherwise.")
        if self.origin is Origin.CARD:
            # `source_span` is NOT in this list, and chunk 15 is why. Chunk 14
            # left every pilot card's span empty on purpose — there is no corpus
            # on this machine, and a span written from memory is the exact
            # confabulation the provenance axis exists to detect — so requiring
            # one here would have made a condition-B node unconstructible, and
            # the only ways out were both lies: substitute the citation for the
            # span, or record a corpus node as INVENTED.
            #
            # ORIGIN AND TRACEABILITY ARE DIFFERENT FACTS. Origin is where the
            # node came from; traceability is whether anyone has opened the
            # paper. `is_traceable` below still demands the span, so an
            # unverified card produces a node that is CARD and untraceable, and
            # G1's fraction counts it in the denominator and not the numerator.
            # That is the honest state, and it is now representable.
            missing = [n for n, v in (("card_id", self.card_id),
                                      ("confidence", self.confidence))
                       if not v]
            if missing:
                raise ValueError(
                    f"{self.node_id}: origin is CARD but {missing} are empty. A "
                    f"node claiming a corpus source must name the card and the "
                    f"extraction confidence, or the provenance fraction G1 "
                    f"reports is a guess.")
        if self.confidence is not None and not isinstance(self.confidence, Confidence):
            raise ValueError(
                f"{self.node_id}: confidence must be a Confidence, not "
                f"{self.confidence!r} — a free float is a number nobody calibrated.")

    @property
    def family(self):
        """The mechanism this node derives. Defaults to `node_id`, so a node
        that pairs with nothing is its own mechanism and the count is right
        without anyone filling the field in."""
        return self.mechanism_family or self.node_id

    @property
    def is_traceable(self):
        """Exactly what G1 reports the fraction of."""
        return (self.origin is Origin.CARD
                and bool(self.source_span) and self.confidence is not None)


class PortAccessError(KeyError):
    """A node touched a port it never declared. Raised rather than returning
    None, because silently reading nothing is how a blackboard turns into
    coupling nobody can see."""


class PortView:
    """A node's restricted view of the port store, built fresh for each tick.

    This class IS the "explicit ports, not a blackboard" decision. A node can
    read the IN ports it declared and write the OUT ports it declared, and
    anything else raises — so "no coupling to a specific scenario" is a property
    a test can check rather than a convention a generator might respect.
    """

    def __init__(self, store, node):
        self._store = store
        self._node = node
        self._readable = {p.name for p in node.ports if p.direction is Direction.IN}
        self._writable = {p.name for p in node.ports if p.direction is Direction.OUT}

    def __getitem__(self, name):
        if name not in self._readable:
            raise PortAccessError(
                f"{self._node.metadata.node_id} read port {name!r}, which it "
                f"does not declare as an input. Declared inputs: "
                f"{sorted(self._readable)}")
        if name not in self._store.values:
            raise PortAccessError(
                f"{self._node.metadata.node_id} read port {name!r} before "
                f"anything wrote it. Wiring is checked at composition time; a "
                f"read this early means the producer ticks after the consumer.")
        return self._store.values[name]

    def __setitem__(self, name, value):
        if name not in self._writable:
            raise PortAccessError(
                f"{self._node.metadata.node_id} wrote port {name!r}, which it "
                f"does not declare as an output. Declared outputs: "
                f"{sorted(self._writable)}")
        self._store.values[name] = value

    def __contains__(self, name):
        return name in self._readable and name in self._store.values

    @property
    def store(self):
        """The underlying store. Composites need it to build their children's
        views; leaves have no business touching it, and the restricted __getitem__
        above is the reason they never need to."""
        return self._store

    def drive(self, command):
        """Emit a Twist. Only an Action may: the Twist is the frozen interface
        and deciding what crosses it is what an Action IS."""
        if self._node.KIND is not Kind.ACTION:
            raise PortAccessError(
                f"{self._node.metadata.node_id} is a {self._node.KIND.value} and "
                f"called drive(). Only an Action emits a Twist — that is the "
                f"distinction between the three leaf kinds, not the label.")
        self._store.command = command


class PortStore:
    """The values behind every PortView in one tick, plus the Twist slot."""

    def __init__(self):
        self.values = {}
        self.command = None
        # None unless Tree.enable_trace() was called. Opt-in so a benchmark run
        # allocates nothing and behaves identically whether or not anyone is
        # watching; see nodes/trace.py.
        self.log = None

    def clear(self):
        self.values.clear()
        self.command = None


class Node:
    """A behaviour-tree leaf.

    Subclasses declare, as class attributes:
      KIND        one of Kind
      PORTS       tuple of Port — the typed interface composition reasons about
      EFFECT      one sentence: what this node changes. DECLARED, NOT EXECUTED.
      PRECONDITION  one sentence: when it applies. The code is precondition().
      TAGS        tuple of str, for retrieval

    and pass NodeMetadata to __init__.

    One precondition and one effect, per §4. A node that needs two of either is
    two nodes; that is not a style rule, it is what makes a unit scenario able to
    isolate one claimed effect.
    """

    KIND = None
    PORTS: Tuple[Port, ...] = ()
    EFFECT = ""
    PRECONDITION = ""
    TAGS: Tuple[str, ...] = ()

    def __init__(self, metadata):
        if not isinstance(metadata, NodeMetadata):
            raise ValueError(
                f"{type(self).__name__}: metadata must be a NodeMetadata. Every "
                f"node carries provenance or is rejected — G1 counts it.")
        if not isinstance(self.KIND, Kind):
            raise ValueError(
                f"{type(self).__name__}: KIND must be one of {list(Kind)}. "
                f"Subclass Condition, Action or Decorator rather than Node.")
        if not self.EFFECT:
            raise ValueError(
                f"{type(self).__name__}: EFFECT is empty. The effect is what "
                f"composition selects on and what the unit scenario tests; a "
                f"node that cannot say what it changes cannot be admitted.")
        names = [p.name for p in self.PORTS]
        if len(set(names)) != len(names):
            raise ValueError(f"{type(self).__name__}: duplicate port names {names}")
        self.metadata = metadata

    @property
    def ports(self):
        return self.PORTS

    def reset(self, scenario, rng):
        """Clear per-episode state. Called once per episode.

        Mirrors `core.observation.Controller.reset` deliberately, including the
        rng: a node that wants randomness draws it from here and never from the
        global np.random, so an episode is reproducible from its seed.
        """

    def precondition(self, obs, ports):
        """Does this node apply right now? Default: always."""
        return True

    def tick(self, obs, ports):
        raise NotImplementedError

    def __repr__(self):
        return f"<{type(self).__name__} {self.metadata.node_id}>"


class Condition(Node):
    """Answers a question about the world. Never emits a Twist."""

    KIND = Kind.CONDITION


class Action(Node):
    """Does something. The only kind that may emit a Twist."""

    KIND = Kind.ACTION


class Decorator(Node):
    """Wraps one child and modifies its status or its rate. Never emits a Twist
    of its own — it governs whether the child's happens."""

    KIND = Kind.DECORATOR

    def __init__(self, metadata, child):
        super().__init__(metadata)
        if not isinstance(child, Node):
            raise ValueError(
                f"{type(self).__name__}: a decorator wraps exactly one Node")
        self.child = child
