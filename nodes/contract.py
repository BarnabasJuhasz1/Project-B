"""
nodes/contract.py

The leaf-node contract from §4: individual behaviour-tree leaves, each with a
typed port interface, one precondition, one effect, and no coupling to a
specific scenario.

`controllers/reactive_avoid.py` is the monolith this exists to forbid — goal
seeking, gap selection and speed control welded into one `step()`, none
separable, none testable alone. That is condition A's natural shape, which is
why it is a useful reference and why it can never be a condition.

FIVE THINGS THIS FILE DECIDES
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

**A command is built in LAYERS, not seized by one node.** Chunk 52, and the
evidence for it is chunk 40's measurement plus growth round 1. See the section
below; it is the newest of the five and the only one that was added after the
library had been measured.

WHY THE TWIST IS ASSEMBLED RATHER THAN CLAIMED (CHUNK 52)
----------------------------------------------------------
`drive()` used to be the whole story: one Action won the tick and wrote the
Twist, everyone else was silent. Two measurements killed that.

- **Chunk 40.** Of fourteen generated nodes, not one declared a single IN port.
  Every published output — `speed_limit`, `safe_speed`, `goal_bearing`,
  `pi_pose`, `reorient_axis`, `ics_safe_speed`, `home_bearing` — was computed,
  written, and cleared unread. The contract had no way to express *adjust the
  command someone else is producing*, so every corpus mechanism was forced into
  an Action with a navigator welded on beside it.
- **Growth round 1, 2026-09-14.** Each admitted node placed in
  `Fallback[specialist, BASE]` against the base alone, 20 maze worlds, paired.
  **Not one of eleven improved the tree; `fixed` was 0 in every row, both arms.**
  Fire rates were bimodal with nothing in between: five nodes never fired, six
  fired on 87.6-100% of ticks — and at 100% the base is never ticked at all, so
  the tree scores exactly what the specialist scores alone.

The mechanisms this project's corpus supplies are **adjustments to an ongoing
behaviour**: cap the speed to what is visible, bias the heading towards a
remembered home vector, re-align to the geometry of the room. A behaviour tree
hands one node the entire Twist and silences the rest, which is the wrong
execution model for all of them.

So, three changes, and they are the smallest set that makes a modifier
expressible:

1. **`Layer`** (`nodes/tree.py`) ticks *every* child in order. Neither existing
   composite can: `Sequence` stops on anything that is not SUCCESS, `Fallback`
   on anything that is not FAILURE, and a base returning RUNNING stops both
   dead. Earlier children are the base behaviour; later children are higher
   priority and may revise what the earlier ones proposed.
2. **The pending command is readable state.** `PortView.command` returns what
   has been proposed so far this tick, or None. Without it "cap the speed and
   leave the heading alone" is inexpressible even if everybody ticks.
3. **A node declares which FIELDS of the Twist it may write** — `WRITES`, a
   subset of `TWIST_FIELDS`. A limiter that declares `("vx",)` *structurally
   cannot* rewrite `wz`; `PortView` raises.

Point 3 is the load-bearing one and it is the same move as `PortView` itself:
it makes "this node supports the base rather than replacing it" **enforced**
rather than hoped for, exactly as the restricted view makes "no coupling to a
scenario" mechanical rather than conventional. It also keeps nodes small by
construction — a block that may only touch `vx` cannot grow into a navigator.

This is subsumption architecture. Worth saying in the write-up, because it is
literature rather than an invention of ours.

⚠️ **THE ARBITRATION RULE IS PART OF THE EXPERIMENT, AND IS THEREFORE FIXED
AND MINIMAL.** Whoever decides how conflicting proposals combine is injecting
design that came from neither arm's corpus, and B-A would then partly measure
*our* arbiter. So there are exactly two combinators, both trivial, both
identical across arms, and both declared in `PRE-REGISTRATION.md` before the
first measurement:

    adjust(field=value)   per-field LAST WRITER WINS
    cap(field=value)      per-field min(|pending|, |value|), sign preserved

No scoring, no weights, no priorities beyond position in the Layer, nothing
tunable. ⚠️ Layer ORDER is the same hazard and gets the same treatment: it is
produced by the LLM composer under a prompt identical across arms, and is never
hand-tuned per arm.

REJECTED, WITH REASONS (so they are not re-proposed)
-----------------------------------------------------
- **A fourth `Modifier` KIND that wraps a child** (chunk 40's option 1). Still
  one child, so it inherits the tree's exclusivity — the thing being fixed. And
  a KIND is a heavier contract change than a composite: `Kind` is what
  composition, retrieval and the diagram all key on.
- **A consuming Action** (40's option 2, and 40's own recommendation): the
  driving Action reads `speed_limit` and applies it itself. Refuted by growth
  round 1 — the consumer must know about each modifier *in advance*, so the base
  would have to be regenerated to consume every port any future node might
  publish. That works for one hand-picked pair and makes chunk 44's open-ended
  growth loop impossible.
- **Letting a `Decorator` rewrite the Twist** (40's option 3). Contradicts the
  load-bearing docstring three lines down: a decorator governs *whether* the
  child's command happens and never emits one of its own. Governing whether is
  not rewriting what.
- **Decomposing the base into small base nodes** (considered 2026-09-14).
  Attractive for the science, and the right thing eventually. Not now: the base
  is 538 lines of interlocking state — commitment timers, a wall-following mode,
  a decaying visit grid — and splitting it while *also* changing the execution
  model would leave no way to attribute a result to either change. Revisit after
  the pipeline works.

⚠️ **NO NEW `Kind`, AND A MODIFIER IS AN `Action`.** The three leaf kinds are
unchanged. What makes a node a modifier is a narrowed `WRITES`, which is a
property `Node.is_modifier` reports and a composer can read — not a fourth
label. This is the decision the chunk left open and it is recorded here because
it is the kind of thing that gets quietly reversed.
"""

import math
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


# The three fields of the frozen interface, in the order `Command` declares
# them. A node's WRITES is a subset of this tuple and nothing else: the Twist is
# `(vx, vy, wz)` and it is never going to be anything else, so an open set here
# would only be a place for a typo to hide.
TWIST_FIELDS = ("vx", "vy", "wz")


class Origin(Enum):
    """Where a node came from. Required on every node, because it is the
    denominator and the numerator of G1's provenance fraction."""

    CARD = "card"                  # extracted from a principle card (condition B)
    CITATION = "citation"          # POINTED at a source, never given its content
                                   # (condition A). The model was handed a
                                   # reference and had to recall the mechanism
                                   # itself, so whatever it implemented is its
                                   # own recollection wearing the paper's name.
                                   # Never traceable: nobody opened the paper,
                                   # so no span can honestly be recorded.
    INVENTED = "invented"          # the model devised the principle (condition D)
    SUBSTRATE = "substrate"        # THE SHARED BASE NODE (chunk 47). Model-written
                                   # from no source at all, and NOT a condition: it is
                                   # the same node in every arm, so it cancels from
                                   # every between-arm difference. Distinct from
                                   # INVENTED on purpose — D INVENTS A PRINCIPLE and is
                                   # scored as an arm; this invents nothing and is
                                   # scored as nothing. Distinct from HANDWRITTEN
                                   # because a model wrote it, and recording it as
                                   # handwritten would be false in the one field that
                                   # exists to say where code came from.
                                   # Never traceable; carries no span.
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
    # WHICH MODEL-WRITTEN CARD THIS NODE WAS BUILT FROM — chunk 42.
    # Every arm writes its own card now, so `card_id` (which CORPUS entry the
    # arm was pointed at) and this (which card the node was actually implemented
    # from) are different facts. D has the second and not the first; a
    # handwritten node has neither.
    generated_card_id: Optional[str] = None
    # Has a HUMAN opened the document and checked `source_span`?
    #
    # Before chunk 42 this did not need to exist: a CARD node inherited a human
    # card's span, so a non-empty span WAS a verified one. Now the span on a
    # generated node is the model's own claim about a paper — checkable for B,
    # which read the document, and pure confabulation risk for A, which did not.
    # Recording the claim is the point; counting it as provenance would make
    # G1's fraction a measure of fluency. So the claim is stored and
    # `is_traceable` demands this flag, which nothing in the generation path
    # sets.
    span_verified: bool = False

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
        if self.origin is Origin.CITATION:
            # A citation node must say WHICH reference it was pointed at — that
            # is the whole of its treatment, and a pointed node that cannot name
            # its pointer is indistinguishable from an invented one.
            if not self.card_id:
                raise ValueError(
                    f"{self.node_id}: origin is CITATION but card_id is empty. "
                    f"The reference it was pointed at IS condition A's "
                    f"independent variable; a node that cannot name it is an "
                    f"INVENTED node with a misleading label.")
            # RELAXED BY CHUNK 42, and the relaxation is the measurement.
            #
            # This used to forbid a span outright: before 42 the only span a
            # citation node could carry was the HUMAN card's, and inheriting it
            # would have handed A provenance B paid for. Now A writes its own
            # card from the reference alone, and a span in it is A's own claim
            # about a document A never saw — which is precisely the
            # confabulation the provenance axis exists to detect. Refusing to
            # store it would throw away the finding.
            #
            # What stays forbidden is the part that was ever the risk:
            # `span_verified`. Nobody has opened a paper on a citation node's
            # behalf and nobody can, so a verified span here would be a claim
            # about work that did not happen, and `is_traceable` is what B-A
            # would have been confounded by.
            if self.span_verified:
                raise ValueError(
                    f"{self.node_id}: origin is CITATION but span_verified is "
                    f"set. Verification means a human opened the document, and "
                    f"a citation node was handed the reference only — marking "
                    f"one verified would make A look traceable and confound "
                    f"B-A.")
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
        """Exactly what G1 reports the fraction of.

        ⚠️ **CHUNK 42 CHANGED WHAT THIS COUNTS, AND THE NUMBER MOVED.** It used
        to be satisfied by a CARD node with a span, because that span came from
        a human-extracted card and 4 of the 5 pilot cards are verified. The node
        is now built from a card the MODEL wrote, so the span is a claim until
        somebody checks it, and `span_verified` is that check.

        Nothing in the generation path sets it, so the node-level fraction is
        0.0 today. That is the honest state and it is not a regression in the
        corpus: `CardStore.provenance()` still reports 0.80 over `cards/*.yaml`,
        which is a fact about the corpus and was never a fact about the nodes.
        The two numbers are reported separately and must not be swapped.
        """
        return (self.origin is Origin.CARD
                and bool(self.source_span) and self.confidence is not None
                and self.span_verified)


class PortAccessError(KeyError):
    """A node touched a port it never declared. Raised rather than returning
    None, because silently reading nothing is how a blackboard turns into
    coupling nobody can see."""


class CommandFieldError(PortAccessError):
    """A node wrote a Twist field it never declared, or revised a command that
    nobody had proposed yet.

    A subclass of PortAccessError on purpose: it is the same failure — touching
    something outside the declared interface — and everything that already
    catches an undeclared access catches this too. The separate class exists so
    a rejection can say WHICH kind of overreach it was, since "reads a port it
    did not declare" and "rewrites a field it promised to leave alone" are very
    different findings about a generator.
    """


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

    # --- the command, which is assembled rather than claimed (chunk 52) ----

    @property
    def command(self):
        """What has been proposed so far this tick, or None if nothing has.

        Readable by any node, and deliberately not a port: a port is information
        one node computes for another, and this is the state of the actuation
        channel itself. A modifier that cannot see the pending command cannot
        adjust it, and adjusting it is the entire point of the layered pipeline.
        """
        return self._store.command

    def _require_action(self, what):
        if self._node.KIND is not Kind.ACTION:
            raise PortAccessError(
                f"{self._node.metadata.node_id} is a {self._node.KIND.value} and "
                f"called {what}(). Only an Action emits a Twist — that is the "
                f"distinction between the three leaf kinds, not the label.")

    def _require_fields(self, what, fields):
        """THE FIELD SCOPE. A node may only touch what it declared in WRITES.

        This is the check that makes "supports the base rather than replacing
        it" a property rather than a promise, and it is the reason a modifier
        cannot quietly grow into a navigator.
        """
        allowed = set(self._node.WRITES)
        denied = sorted(f for f in fields if f not in allowed)
        if denied:
            raise CommandFieldError(
                f"{self._node.metadata.node_id} called {what}() on {denied}, "
                f"which it does not declare in WRITES. Declared: "
                f"{sorted(allowed)}. A node's WRITES is the promise that it "
                f"adjusts one aspect of the command and leaves the rest to "
                f"whoever proposed it; widening it in code rather than in the "
                f"declaration would make the promise unreadable to composition.")

    def _pending(self, what):
        command = self._store.command
        if command is None:
            raise CommandFieldError(
                f"{self._node.metadata.node_id} called {what}() before anything "
                f"proposed a command. A modifier revises a command someone else "
                f"produced, so it must sit AFTER a driver in the Layer — put the "
                f"base earlier, or make this node drive() a whole command "
                f"instead.")
        return command

    def _write(self, fields):
        import dataclasses
        self._store.command = dataclasses.replace(self._store.command, **fields)
        for name in fields:
            self._store.command_writers[name] = self._node.metadata.node_id

    def drive(self, command):
        """Propose a WHOLE Twist. Only a full driver may.

        Full means `WRITES` covers all three fields, which is the default for an
        `Action` and is what the shared base is. A node that narrowed its WRITES
        has declared itself a modifier and must use `adjust`/`cap`: letting it
        drive() would let it replace the command it promised only to revise,
        which is the failure mode chunk 52 exists to prevent.
        """
        self._require_action("drive")
        if set(self._node.WRITES) != set(TWIST_FIELDS):
            raise CommandFieldError(
                f"{self._node.metadata.node_id} declares WRITES="
                f"{tuple(self._node.WRITES)} and called drive(), which writes "
                f"all of {TWIST_FIELDS}. A node that writes every field is a "
                f"driver and should say so by leaving WRITES alone; a node that "
                f"writes some of them is a modifier and adjusts or caps.")
        self._store.command = command
        for name in TWIST_FIELDS:
            self._store.command_writers[name] = self._node.metadata.node_id

    def adjust(self, **fields):
        """Revise named fields of the pending command. LAST WRITER WINS.

        One of the two combinators, and it is deliberately the dumbest one
        available: a later node in the Layer simply overwrites the field. No
        weights, no blending, nothing to tune — see the arbitration note in the
        module docstring for why the rule must stay this boring.
        """
        self._require_action("adjust")
        self._require_fields("adjust", fields)
        self._pending("adjust")
        self._write({k: float(v) for k, v in fields.items()})

    def cap(self, **fields):
        """Bound the MAGNITUDE of named fields. `min(|pending|, |value|)`, sign
        kept.

        The second combinator, and the one most of the corpus actually wants:
        "limit speed to the visible range" is a cap, not a replacement. It is
        order-independent between caps, which is the property that stops two
        limiters in one Layer from depending on which was composed first.
        """
        self._require_action("cap")
        self._require_fields("cap", fields)
        command = self._pending("cap")
        out = {}
        for name, value in fields.items():
            bound = abs(float(value))
            current = float(getattr(command, name))
            if abs(current) > bound:
                out[name] = math.copysign(bound, current)
        if out:
            self._write(out)


class PortStore:
    """The values behind every PortView in one tick, plus the Twist slot.

    `command` is the PENDING command, not a final answer. Under chunk 52 it is
    built up across a `Layer`: an earlier child proposes one and a later child
    may revise individual fields of it. `Tree.step` takes whatever is here when
    the root has finished ticking.
    """

    def __init__(self):
        self.values = {}
        self.command = None
        # Which node last wrote each Twist field this tick — `{"vx": node_id}`.
        # Three keys at most, so it costs nothing and is always maintained
        # rather than being a tracing option: `PortView.adjust` needs it to say
        # WHO is being overridden when a write is refused, and a refusal with no
        # name in it is a message nobody can act on.
        self.command_writers = {}
        # None unless Tree.enable_trace() was called. Opt-in so a benchmark run
        # allocates nothing and behaves identically whether or not anyone is
        # watching; see nodes/trace.py.
        self.log = None

    def clear(self):
        self.values.clear()
        self.command = None
        self.command_writers.clear()


class Node:
    """A behaviour-tree leaf.

    Subclasses declare, as class attributes:
      KIND        one of Kind
      PORTS       tuple of Port — the typed interface composition reasons about
      EFFECT      one sentence: what this node changes. DECLARED, NOT EXECUTED.
      PRECONDITION  one sentence: when it applies. The code is precondition().
      TAGS        tuple of str, for retrieval
      WRITES      which Twist fields it may write. Empty for a Condition or a
                  Decorator; all three for an Action by default; a NARROWED
                  subset is what makes an Action a modifier (chunk 52).

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
    # Empty here because the default leaf writes nothing. `Action` overrides it
    # with all three fields, so every Action written before chunk 52 keeps its
    # meaning exactly and nothing had to be edited to stay valid.
    WRITES: Tuple[str, ...] = ()

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
        writes = tuple(self.WRITES)
        unknown = [f for f in writes if f not in TWIST_FIELDS]
        if unknown:
            raise ValueError(
                f"{type(self).__name__}: WRITES names {unknown}, and the Twist "
                f"has exactly {list(TWIST_FIELDS)}. The frozen interface is "
                f"three floats; a fourth field is a typo, not an extension.")
        if len(set(writes)) != len(writes):
            raise ValueError(f"{type(self).__name__}: duplicate WRITES {writes}")
        if writes and self.KIND is not Kind.ACTION:
            raise ValueError(
                f"{type(self).__name__}: a {self.KIND.value} declares "
                f"WRITES={writes}. Only an Action may touch the Twist, whether "
                f"it writes all of it or one field of it — that is what the "
                f"three kinds distinguish.")
        if not writes and self.KIND is Kind.ACTION:
            raise ValueError(
                f"{type(self).__name__}: an Action with WRITES=() can never "
                f"emit anything, which makes it a Condition wearing the wrong "
                f"base class. Leave WRITES alone to drive the whole command, or "
                f"name the fields this node adjusts.")
        self.metadata = metadata

    @property
    def ports(self):
        return self.PORTS

    @property
    def is_modifier(self):
        """Does this node REVISE a command rather than produce one?

        A property rather than a fourth `Kind`, which is the decision recorded
        in the module docstring: composition can read it, retrieval can filter
        on it, and the leaf vocabulary stays at three.
        """
        return (self.KIND is Kind.ACTION
                and set(self.WRITES) != set(TWIST_FIELDS))

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
    """Does something. The only kind that may touch the Twist.

    Two shapes, distinguished by `WRITES` and nothing else (chunk 52):

    - **a driver** — `WRITES` left alone, so it covers all three fields. It
      calls `ports.drive(Command(...))` and proposes a whole command. The shared
      base is the only one of these that matters.
    - **a modifier** — `WRITES` narrowed to the fields it adjusts, e.g.
      `("vx",)`. It reads `ports.command`, calls `ports.cap(vx=...)` or
      `ports.adjust(vx=...)`, and *cannot* write anything else. It must sit
      after a driver in a `Layer`.

    `is_modifier` reports which. There is no fourth `Kind`; see the module
    docstring for why not.
    """

    KIND = Kind.ACTION
    # All three, so every Action written before chunk 52 is a driver and keeps
    # its exact previous behaviour. Narrowing this is what declares a modifier.
    WRITES = TWIST_FIELDS


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
