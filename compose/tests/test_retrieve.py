"""
compose/tests/test_retrieve.py

Retrieval keeps the working set small (#7) and records why, so that a G3 miss is
attributable to the library or to retrieval rather than to neither.
"""

import dataclasses

from compose.retrieve import MAX_SEED_ACTIONS, retrieve, score_entry
from compose.spec import TaskSpec
from nodes.contract import Direction, Kind


def _requires(entry):
    return {(p.name, p.type) for p in entry.node_cls.PORTS
            if p.direction is Direction.IN}


def _provides(entry):
    return {(p.name, p.type) for p in entry.node_cls.PORTS
            if p.direction is Direction.OUT}


def test_every_in_port_of_a_retrieved_node_has_a_retrieved_producer(found):
    """The hard mechanism. Without port closure, ranking would routinely hand
    the model a set from which no legal tree exists — and the model would be
    blamed for a wiring failure retrieval caused."""
    provided = set()
    for entry in found.entries:
        provided |= _provides(entry)
    for entry in found.entries:
        assert _requires(entry) <= provided


def test_producers_come_before_consumers(found):
    """check_wiring requires every IN port to be produced EARLIER in tick order,
    so a retrieval listed in dependency order means the obvious tree is legal."""
    satisfied = set()
    for entry in found.entries:
        assert _requires(entry) <= satisfied
        satisfied |= _provides(entry)


def test_a_condition_is_retrieved_only_because_something_reads_it(library, spec):
    """`toy_is_goal_known` scores on tags too, but the reason recorded for it
    must be the port closure — that is what actually pulled it in, and a log
    that recorded the flattering reason would mislead a post-mortem."""
    found = retrieve(library, spec)
    assert found.reasons["toy_is_goal_known"].startswith("port closure")


def test_retrieval_is_deterministic(library, spec):
    """The prompt is built from this, the prompt hash is built from the prompt,
    and the whole record hangs off the hash. An unstable order here would make
    every composition irreproducible."""
    runs = [retrieve(library, spec).node_ids for _ in range(5)]
    assert len(set(runs)) == 1


def test_ranking_responds_to_the_axes(library, spec, search_spec):
    """Hiding the goal must move the search action up the ranking. If it does
    not, the query is not doing anything and retrieval is just the library."""
    def rank(found, node_id):
        return [s.node_id for s in found.scored].index(node_id)

    seen = retrieve(library, spec)
    hidden = retrieve(library, search_spec)
    assert rank(hidden, "toy_sweep_for_goal") < rank(seen, "toy_sweep_for_goal")


def test_the_working_set_is_capped(library, spec):
    """#7 makes the library monotonic, so the cap is the only thing keeping the
    prompt finite. Seeded Actions are capped; port closure is not, because a
    node pulled in to satisfy a port is not optional."""
    found = retrieve(library, spec, max_actions=1)
    actions = [e for e in found.entries if e.node_cls.KIND is Kind.ACTION]
    assert len(actions) == 1


def test_a_port_nothing_produces_is_reported_not_hidden(library, spec):
    """An IN port with no producer anywhere in the library is a LIBRARY gap. It
    is reported rather than raised: retrieval offers a set, composition decides,
    and this line is what tells a G3 post-mortem which of the two failed."""
    found = retrieve(library, spec, max_actions=99)
    assert found.unsatisfiable == []          # the library is complete today

    # A node reading a port nothing writes, injected without touching the
    # library — the append-only store (#7) has no remove().
    from nodes.contract import Action, Port, PortType

    class ReadsNothingKnown(Action):
        PORTS = (Port("nobody_writes_this", PortType.CONFIDENCE, Direction.IN),)
        PRECONDITION = "never"
        EFFECT = "nothing; a fixture for the unsatisfiable-port path"

    from nodes.library import Library
    from nodes.toys import toy_metadata
    probe = Library()
    probe.add(ReadsNothingKnown, toy_metadata("probe_reads_unknown"))
    found = retrieve(probe, spec)
    assert found.unsatisfiable == ["nobody_writes_this:Confidence"]


def test_an_orphan_out_port_is_reported(library, spec):
    """A node whose only output nothing reads is INERT in any tree built from
    this set. The first real composition run put such a node in a tree and
    explained at length what it was contributing; the rationale is not evidence
    and this line is."""
    found = retrieve(library, spec)
    # Derived from the generated node rather than written down. The port used to
    # be called `admissible_speed`; the node was re-generated when its card was
    # verified and the model named the same output `speed_limit`. The NAME was
    # never the finding — that nothing in the retrieved set reads the port is.
    generated = [e for e in found.entries if e.metadata.card_id]
    assert generated, "expected the generated node in the retrieved set"
    orphans = found.orphan_ports()
    for entry in generated:
        for port in entry.node_cls.PORTS:
            if port.direction is Direction.OUT:
                assert f"{port.name}:{port.type.value}" in orphans, (
                    f"{port.name} is read by something now. If a node in the "
                    f"library consumes it, this finding has been fixed and "
                    f"compose/FINDINGS.md should say so.")


def test_the_log_names_everything_considered(library, spec):
    """Half of what the log is for is what was NOT retrieved."""
    found = retrieve(library, spec, max_actions=1)
    record = found.as_dict()
    assert len(record["considered"]) == len(library)
    assert record["dropped"]
    assert set(record["retrieved"]) & set(record["dropped"]) == set()


def test_scoring_reads_only_the_public_contract(library):
    """Effect, precondition and tags — the fields LibraryEntry.identity() calls
    'what composition selects on'. A card is not reachable from here, and this
    asserts the fields the score is actually built from."""
    entry = library.get("toy_avoid_obstacles")
    scored = score_entry(entry, ("obstacle",))
    assert scored.matched_tags == ("obstacle",)
    assert scored.matched_words == ("obstacle",)
