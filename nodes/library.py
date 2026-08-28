"""
nodes/library.py

The node library. **Append-only** (non-negotiable #7) and monotonic: nothing is
ever removed, nothing is ever edited, and nothing is compressed away. Retrieval
is what keeps the working set small at composition time, not deletion.

That is not tidiness. G1 counts distinct admissible primitives per condition,
and a library you can delete from is a library whose count depends on when you
looked. G3 freezes the library at a tagged snapshot after development, and a tag
that can silently come to mean different contents is not a freeze.

THE SNAPSHOT TAG
----------------
`snapshot()` hashes the library's *contents*, sorted by node id, so the same
nodes give the same tag regardless of the order they were added in. It goes into
`manifest["library_snapshot"]` — the slot that has been None since the first run
— so every reported result names the frozen library it came from, which is
non-negotiable #8.
"""

import hashlib
import json

from nodes.contract import Direction, Node, NodeMetadata


class LibraryError(ValueError):
    pass


class LibraryEntry:
    """One admitted node: the class, and the provenance it was admitted with."""

    def __init__(self, node_cls, metadata):
        self.node_cls = node_cls
        self.metadata = metadata

    @property
    def node_id(self):
        return self.metadata.node_id

    def build(self, *args, **kwargs):
        """Instantiate. Composition calls this; the library never holds live
        nodes, because a node with episode state on it is not a library entry."""
        return self.node_cls(self.metadata, *args, **kwargs)

    def identity(self):
        """Everything that makes this entry what it is, as JSON-ready data.

        Ports, effect and precondition are included because they are what
        composition selects on: a node whose ports changed is a different node
        even under the same id, and the snapshot tag must say so.
        """
        m = self.metadata
        return {
            "node_id": m.node_id,
            "kind": self.node_cls.KIND.value,
            "effect": self.node_cls.EFFECT,
            "precondition": self.node_cls.PRECONDITION,
            "tags": sorted(self.node_cls.TAGS),
            "ports": sorted(
                [p.name, p.type.value, p.direction.value]
                for p in self.node_cls.PORTS),
            "origin": m.origin.value,
            "card_id": m.card_id,
            "source_span": m.source_span,
            "confidence": m.confidence.value if m.confidence else None,
            "model_id": m.model_id,
            "prompt_hash": m.prompt_hash,
            "temperature": m.temperature,
        }

    def __repr__(self):
        return f"<LibraryEntry {self.node_id}>"


class Library:
    """Append-only store of admitted nodes."""

    def __init__(self):
        self._entries = []
        self._by_id = {}

    # -- adding -------------------------------------------------------------

    def add(self, node_cls, metadata):
        """Append one node. Raises rather than replacing an existing id.

        The node is INSTANTIATED here and thrown away, so that Node.__init__'s
        checks — a real Kind, a non-empty EFFECT, valid metadata, no duplicate
        port names — run at admission rather than the first time something ticks
        it. A library that admits a node it cannot build is worse than one that
        refuses it.
        """
        if not isinstance(metadata, NodeMetadata):
            raise LibraryError(f"{node_cls!r}: metadata must be a NodeMetadata")
        if not (isinstance(node_cls, type) and issubclass(node_cls, Node)):
            raise LibraryError(f"{node_cls!r} is not a Node subclass")
        if metadata.node_id in self._by_id:
            raise LibraryError(
                f"{metadata.node_id!r} is already in the library. The library is "
                f"append-only (#7): a node is never replaced or edited. If this "
                f"is a revision, give it a new id — the old one stays, and the "
                f"snapshot tags before and after both remain meaningful.")
        try:
            node_cls(metadata)
        except Exception as exc:
            raise LibraryError(
                f"{metadata.node_id}: rejected, it cannot be constructed: "
                f"{exc}") from exc

        entry = LibraryEntry(node_cls, metadata)
        self._entries.append(entry)
        self._by_id[entry.node_id] = entry
        return entry

    # -- reading ------------------------------------------------------------

    def __len__(self):
        return len(self._entries)

    def __contains__(self, node_id):
        return node_id in self._by_id

    def all(self):
        """Insertion order, which is the order they were admitted in."""
        return list(self._entries)

    def get(self, node_id):
        if node_id not in self._by_id:
            raise KeyError(f"no node {node_id!r} in the library")
        return self._by_id[node_id]

    def by_port(self, port_type, direction=None):
        """Entries declaring a port of this type. The primary retrieval axis:
        composition asks "what produces a Bearing" without seeing a paper."""
        out = []
        for entry in self._entries:
            for port in entry.node_cls.PORTS:
                if port.type is port_type and (direction is None
                                               or port.direction is direction):
                    out.append(entry)
                    break
        return out

    def by_tag(self, tag):
        return [e for e in self._entries if tag in e.node_cls.TAGS]

    def tags(self):
        return sorted({t for e in self._entries for t in e.node_cls.TAGS})

    # -- provenance ---------------------------------------------------------

    def provenance(self):
        """G1's numbers, computed rather than asserted: how many nodes, and what
        fraction carry a source span with extraction confidence."""
        total = len(self._entries)
        traceable = sum(e.metadata.is_traceable for e in self._entries)
        return {
            "node_count": total,
            "traceable_count": traceable,
            "traceable_fraction": (traceable / total) if total else 0.0,
            "by_origin": {
                origin: sum(e.metadata.origin.value == origin
                            for e in self._entries)
                for origin in sorted({e.metadata.origin.value
                                      for e in self._entries})
            },
        }

    # -- freezing -----------------------------------------------------------

    def contents(self):
        """Sorted by node id, so the tag does not depend on admission order."""
        return sorted((e.identity() for e in self._entries),
                      key=lambda d: d["node_id"])

    def snapshot(self):
        """A stable tag for this library's contents. Goes in the manifest."""
        blob = json.dumps(self.contents(), sort_keys=True,
                          separators=(",", ":")).encode("utf-8")
        digest = hashlib.sha256(blob).hexdigest()[:12]
        return f"lib-{len(self._entries):03d}-{digest}"
