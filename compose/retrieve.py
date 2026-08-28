"""
compose/retrieve.py

Which nodes go in the composition prompt, and why each one.

Non-negotiable #7 makes the library append-only and monotonic, so by Week 5 it
may hold well over twenty nodes and cannot all go in one prompt. Retrieval is
what keeps the working set small — deletion is not available and never will be.

TWO MECHANISMS, AND ONLY ONE OF THEM IS RANKING
------------------------------------------------
**Port closure is the hard mechanism.** An Action that reads `goal_known: Flag`
needs a producer of `goal_known: Flag` in the retrieved set, or the tree cannot
be wired at all — `nodes.tree.check_wiring` rejects it at construction. So after
ranking picks a working set of Actions, retrieval walks their unsatisfied IN
ports and pulls in whatever produces them. This is what "retrieve by port type"
actually buys, and it is not a preference: without it, ranking would routinely
hand the model a set from which no legal tree exists.

**Tag and text overlap is the soft mechanism**, and it is weak. Node TAGS and
EFFECT are written by the model at generation time, so the vocabulary is not
controlled; a node tagged `("dwa", "braking")` overlaps nothing a task spec says.
That is a finding rather than a bug — chunk 16 predicted retrieval would be the
open question — and the answer to it is the log, not a cleverer score.

WHY THE LOG IS LOAD-BEARING
---------------------------
"The library had the right node but retrieval never surfaced it" and "the library
lacked the node" are completely different failures, and G3 — the fraction of
held-out scenarios solved by recombination alone — cannot tell them apart from
the outcome. Every retrieval therefore records what it considered, what each
candidate scored, why each retained one was retained, and what was dropped. A
G3 miss is attributable afterwards or it is not attributable at all.
"""

import re
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

from nodes.contract import Direction, Kind

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# How many Actions the ranking may seed the working set with, before port
# closure adds whatever they need. Not a tuning knob to be swept: it is the
# "keep the working set small" of #7 made concrete, and moving it changes what
# composition was offered, so it belongs in the composition record and — once
# G3 is being reported — in the pre-registration.
MAX_SEED_ACTIONS = 8

# Tag overlap counts double. A tag is a deliberate label; a word in an EFFECT
# sentence is prose that happened to match. Both are weak, and the weighting
# says which is less weak rather than pretending either is strong.
TAG_WEIGHT = 2.0
TEXT_WEIGHT = 1.0

_WORD = re.compile(r"[a-z]+")


@dataclass(frozen=True)
class Scored:
    """One library entry, scored against the query. Kept whether or not it was
    retrieved — the dropped ones are half of what the log is for."""

    node_id: str
    kind: str
    score: float
    matched_tags: Tuple[str, ...]
    matched_words: Tuple[str, ...]

    def as_dict(self):
        return {"node_id": self.node_id, "kind": self.kind,
                "score": round(self.score, 3),
                "matched_tags": list(self.matched_tags),
                "matched_words": list(self.matched_words)}


@dataclass
class Retrieval:
    """What composition was offered, and the reasoning that got it there."""

    entries: List[object] = field(default_factory=list)
    scored: List[Scored] = field(default_factory=list)
    reasons: Dict[str, str] = field(default_factory=dict)
    query_tags: Tuple[str, ...] = ()
    library_size: int = 0
    unsatisfiable: List[str] = field(default_factory=list)

    @property
    def node_ids(self):
        return tuple(e.node_id for e in self.entries)

    def as_dict(self):
        return {
            "query_tags": list(self.query_tags),
            "library_size": self.library_size,
            "retrieved": list(self.node_ids),
            "reasons": dict(self.reasons),
            "considered": [s.as_dict() for s in self.scored],
            "dropped": [s.node_id for s in self.scored
                        if s.node_id not in self.node_ids],
            # An IN port nothing in the library produces. NOT an error here:
            # retrieval reports it and composition decides. It is the single
            # most useful line in a G3 post-mortem, because it says the library
            # is incomplete rather than that retrieval missed something.
            "unsatisfiable_ports": list(self.unsatisfiable),
            # An OUT port no retrieved node reads. The node writing it is inert
            # in any tree built from this set — it still ticks, it still passes
            # its own unit scenario, and it changes nothing.
            #
            # Recorded because the first real composition run showed a model
            # placing such a node and then explaining, fluently and at length,
            # what it was contributing. The rationale is not evidence and this
            # line is: a tree whose only use of a node is an orphan port has not
            # actually recombined it.
            "orphan_ports": self.orphan_ports(),
        }

    def orphan_ports(self):
        written, read = set(), set()
        for entry in self.entries:
            written |= {f"{p.name}:{p.type.value}" for p in entry.node_cls.PORTS
                        if p.direction is Direction.OUT}
            read |= {f"{p.name}:{p.type.value}" for p in entry.node_cls.PORTS
                     if p.direction is Direction.IN}
        return sorted(written - read)


def _words(entry):
    """The node's own declared text, lowercased into a bag of words.

    `EFFECT`, `PRECONDITION` and `TAGS` only. These are exactly the fields
    `LibraryEntry.identity()` calls "what composition selects on"; mechanism,
    derivation and source are not reachable from a LibraryEntry at all, which is
    the structural half of non-negotiable #6.
    """
    cls = entry.node_cls
    text = " ".join([cls.EFFECT, cls.PRECONDITION, " ".join(cls.TAGS)]).lower()
    return set(_WORD.findall(text))


def score_entry(entry, query_tags):
    """How well one entry answers the query. Deterministic, and explainable."""
    tags = {t.lower() for t in entry.node_cls.TAGS}
    query = {t.lower() for t in query_tags}
    matched_tags = tuple(sorted(tags & query))
    matched_words = tuple(sorted(_words(entry) & query))
    score = TAG_WEIGHT * len(matched_tags) + TEXT_WEIGHT * len(matched_words)
    return Scored(entry.node_id, entry.node_cls.KIND.value, score,
                  matched_tags, matched_words)


def _provides(entry):
    return {(p.name, p.type) for p in entry.node_cls.PORTS
            if p.direction is Direction.OUT}


def _requires(entry):
    return {(p.name, p.type) for p in entry.node_cls.PORTS
            if p.direction is Direction.IN}


def retrieve(library, spec, max_actions=MAX_SEED_ACTIONS):
    """TaskSpec -> the working set, with the reasoning recorded.

    The result is ordered: producers first, then the Actions that read them.
    That is not cosmetic — `check_wiring` requires every IN port to be produced
    by a node EARLIER in tick order, so handing the model a list in dependency
    order means the obvious tree is also a legal one.
    """
    query = spec.tags()
    scored = sorted((score_entry(e, query) for e in library.all()),
                    # Descending score, then node_id, so a tie is broken the
                    # same way on every machine and in every run. An unstable
                    # order here would make the composition prompt — and so the
                    # prompt hash, and so the whole record — irreproducible.
                    key=lambda s: (-s.score, s.node_id))
    by_id = {e.node_id: e for e in library.all()}

    result = Retrieval(scored=scored, query_tags=query,
                       library_size=len(library))

    # 1. Seed with Actions. Only an Action emits a Twist, so a working set with
    #    no Action composes to a tree that cannot move the robot.
    seeds = [s for s in scored if s.kind == Kind.ACTION.value][:max_actions]
    chosen = []
    for s in seeds:
        chosen.append(by_id[s.node_id])
        result.reasons[s.node_id] = (
            f"action, rank by tag/text overlap (score {s.score:g}"
            + (f", tags {list(s.matched_tags)}" if s.matched_tags else "")
            + ")")

    # 2. Close over ports. Iterate to a fixed point: a producer pulled in may
    #    itself read a port, and stopping after one pass would hand the model a
    #    set that still cannot be wired.
    satisfied = set()
    for entry in chosen:
        satisfied |= _provides(entry)
    while True:
        needed = set()
        for entry in chosen:
            needed |= _requires(entry) - satisfied
        if not needed:
            break
        added = False
        for name, port_type in sorted(needed, key=lambda p: (p[0], p[1].value)):
            producer = next(
                (by_id[s.node_id] for s in scored
                 if by_id[s.node_id] not in chosen
                 and (name, port_type) in _provides(by_id[s.node_id])),
                None)
            if producer is None:
                signature = f"{name}:{port_type.value}"
                if signature not in result.unsatisfiable:
                    result.unsatisfiable.append(signature)
                continue
            chosen.append(producer)
            satisfied |= _provides(producer)
            result.reasons[producer.node_id] = (
                f"port closure: produces {name!r} ({port_type.value}), which a "
                f"retrieved node reads")
            added = True
        if not added:
            break

    # 3. Dependency order. Producers of a port before its consumers, and the
    #    seed ranking preserved within each layer.
    result.entries = _topological(chosen)
    return result


def _topological(entries):
    """Producers before consumers, stable, and never infinite.

    A cycle is possible in principle — two nodes each reading a port the other
    writes — and is left at the end rather than raised on. `check_wiring` is the
    place that decides a tree is illegal; retrieval's job is to offer a set, and
    an offer that quietly dropped a node would be invisible in the log.
    """
    remaining = list(entries)
    ordered, satisfied = [], set()
    while remaining:
        ready = [e for e in remaining if not (_requires(e) - satisfied)]
        if not ready:
            ordered.extend(remaining)
            break
        for entry in ready:
            ordered.append(entry)
            satisfied |= _provides(entry)
            remaining.remove(entry)
    return ordered
