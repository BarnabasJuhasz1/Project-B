

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
