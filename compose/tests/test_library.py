

def test_a_citation_node_rebuilds_as_citation_not_invented(tmp_path):
    """The library is rebuilt from staging, so it re-derives every node's
    origin. Reading only `card_id` there would rebuild every condition-A node
    as INVENTED — losing the fact that it was pointed at a paper, and with it
    the distinction B-A is measured over. `by_origin` is where G1's provenance
    numbers are read off, so the loss would be silent and reportable.
    """
    from compose.library import load_library
    lib = load_library()
    origins = lib.provenance()["by_origin"]
    if "citation" not in origins:
        import pytest
        pytest.skip("no condition-A node admitted yet")
    # A citation node is pointed, never read: never traceable.
    #
    # ⚠️ WHAT "never traceable" IS MADE OF CHANGED IN CHUNK 42, and the old
    # assertions (`not source_span`, `not confidence`) were the v1 rule. Under
    # v1 the only span a citation node could carry was the HUMAN card's, so
    # forbidding it outright was right. Under v2 condition A writes its OWN card
    # from the reference alone, and a span in that card is A's claim about a
    # document A never saw — which is exactly the confabulation the provenance
    # axis exists to detect, so it is RECORDED rather than refused.
    #
    # `span_verified` is what carries the old guarantee now: nobody has opened a
    # paper on a citation node's behalf and nobody can, so it is never set and
    # `is_traceable` is never true. That last assertion is the one that matters
    # and it is unchanged.
    for entry in lib.all():
        md = entry.metadata
        if md.origin.value == "citation":
            assert md.card_id, "a citation node must name its reference"
            assert not md.span_verified, (
                "a citation node cannot have a verified span: verification "
                "means a human opened the document, and this arm was handed "
                "the reference only")
            assert not md.is_traceable
            if md.generated_card_id is None:
                # A v1 record: it never had a card of its own, so it must carry
                # nothing from anybody else's.
                assert not md.source_span
                assert not md.confidence


# --- the arm partition ------------------------------------------------------
#
# G2 compares arms that share a task family and a card set. Without a partition
# at load time, condition A's composition retrieves condition B's nodes and the
# comparison measures nothing — so these are tests about whether the ablation is
# structurally possible, not about a convenience flag.

def test_each_arm_gets_only_its_own_nodes():
    from compose.library import load_library, CONTRACT_PORTS_ID
    full = load_library()
    generated = [e for e in full.all() if e.node_id != CONTRACT_PORTS_ID]
    if not generated:
        import pytest
        pytest.skip("no generated node admitted yet")

    seen = {}
    for arm in ("a", "b", "d"):
        lib = load_library(conditions=(arm,))
        ids = {e.node_id for e in lib.all()} - {CONTRACT_PORTS_ID}
        # A generated node id is built from its row key, which carries the arm.
        assert all(i.startswith(f"gen_{arm}_") for i in ids), (
            f"condition {arm}'s partition holds {sorted(ids)}, and at least "
            f"one of those was written by another arm")
        seen[arm] = ids

    # Disjoint, and together they account for every generated node. A node that
    # fell out of every partition would be invisible to every condition while
    # still counting toward the library's size.
    assert set.intersection(*(v for v in seen.values() if v)) == set()
    assert set.union(*seen.values()) == {e.node_id for e in generated}


def test_the_port_publisher_belongs_to_no_arm_and_is_always_present():
    """Retrieval's port closure needs a producer to find. `ContractPorts` is
    handwritten and belongs to no condition, so an arm filter that dropped it
    would make every partition unable to satisfy a single input port — a
    library gap that looks exactly like a retrieval failure."""
    from compose.library import load_library, CONTRACT_PORTS_ID
    for arm in ("a", "b", "d"):
        lib = load_library(conditions=(arm,))
        assert CONTRACT_PORTS_ID in {e.node_id for e in lib.all()}


def test_an_arm_with_nothing_admitted_is_empty_not_an_error():
    """`only` raises when a requested id does not materialise, because a
    historical rebuild that silently shrinks would hash to a tag naming a set it
    is not. An ARM is different: "this condition has generated nothing yet" is a
    real state of the experiment and the empty library is the honest answer."""
    from compose.library import load_library, CONTRACT_PORTS_ID
    lib = load_library(conditions=("nonexistent-arm",))
    assert {e.node_id for e in lib.all()} == {CONTRACT_PORTS_ID}


def test_a_partition_hashes_differently_from_the_whole_library():
    """#8: a result names the snapshot it came from. Condition A was composed
    over A's partition, so A's record must not be able to name the union's tag —
    if the two hashed alike, every arm would report the same library and the
    tag would stop identifying anything."""
    from compose.library import load_library, CONTRACT_PORTS_ID
    full = load_library()
    if len([e for e in full.all() if e.node_id != CONTRACT_PORTS_ID]) < 2:
        import pytest
        pytest.skip("needs two arms with an admitted node to be meaningful")
    tags = {arm: load_library(conditions=(arm,)).snapshot()
            for arm in ("a", "b")}
    assert tags["a"] != tags["b"]
    assert full.snapshot() not in tags.values()
