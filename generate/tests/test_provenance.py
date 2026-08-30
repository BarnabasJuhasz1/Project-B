"""
generate/tests/test_provenance.py

What G1 reports, and the seal, checked on the generation path.

G1 is "≥20 admissible behaviour-tree nodes, each passing an isolated unit
scenario. Report: node count, unit pass rate, fraction carrying a source span
with extraction confidence." Two of those three are produced here — the count
and the fraction — and the fraction is the one that can be inflated by accident.

The rule these tests exist to hold: **origin and traceability are different
facts.** A node knows which card it came from; whether anyone has opened that
paper is a separate question with its own answer. Four of the five pilot cards
now answer yes (cards/VERIFICATION.md); `limit_speed_ics` still answers no. A
test that let the two blur would let G1's provenance number climb without anyone
reading anything, which is why every assertion below reads traceability off the
CARD and never off the node's own claim about itself.
"""

import ast
import os

import pytest

from cards.schema import Verification
from generate.admit import admit
from generate.generator import generate
from nodes.contract import Origin
from nodes.library import Library

from generate.tests.conftest import GOOD_PROMPT_HASH


@pytest.fixture
def admitted(pilot_card, replay, staging):
    """The committed condition-B generation, admitted into a fresh library."""
    library = Library()
    candidate = generate("b", "navigate_to_goal", replay, card=pilot_card,
                         staging=staging, record=False)
    result = admit(candidate.metadata(), candidate.source, card=pilot_card,
                   library=library)
    assert result.admitted, f"{result.reason}: {result.detail}"
    return library, result


def test_an_admitted_card_node_names_the_card_it_came_from(admitted, cards):
    """"Every admitted node carries a source span traceable to a real card."" —
    the traceability is to a card that EXISTS IN THE STORE, checked by
    resolving it rather than by pattern-matching the string."""
    library, _ = admitted
    entry, = library.all()
    assert entry.metadata.origin is Origin.CARD
    assert entry.metadata.card_id == "limit_speed_dwa"
    assert cards.get(entry.metadata.card_id).node_name == "LimitSpeedToVisibleRange"  # limit_speed_dwa; ics is now LimitSpeedToOcclusionBoundary


def test_the_confidence_travels_from_the_card_to_the_node(admitted, cards):
    library, _ = admitted
    entry, = library.all()
    assert entry.metadata.confidence.value == cards.get("limit_speed_dwa").confidence.value


def test_a_cards_verification_state_reaches_the_node_either_way(admitted, cards):
    """THE test on this axis, and it now runs in the direction that matters.

    It used to assert that every node was untraceable, because every pilot card
    was UNVERIFIED. limit_speed_dwa was verified on 2026-08-27 against the real
    PDF, so the assertion is inverted — which its own docstring called the good
    outcome. What is being pinned is unchanged: a node's traceability is the
    CARD's verification state and never the model's say-so.
    """
    library, _ = admitted
    entry, = library.all()
    card = cards.get(entry.metadata.card_id)
    assert entry.metadata.is_traceable is card.is_traceable, (
        "a node's traceability diverged from its card's. G1's provenance "
        "fraction would then measure fluency rather than anyone having read "
        "the paper — see cards/VERIFICATION.md.")
    expected = 1.0 if card.is_traceable else 0.0
    assert library.provenance()["traceable_fraction"] == expected


def test_a_node_may_not_substitute_the_citation_for_the_span(admitted, cards):
    """The specific lie the schema exists to prevent: a plausible-looking
    provenance string that nobody checked."""
    library, _ = admitted
    entry, = library.all()
    card_span = cards.get(entry.metadata.card_id).source.span
    # The span is the card's, verbatim - not the model's paraphrase of it, and
    # not the citation wearing a span's clothes.
    assert entry.metadata.source_span == card_span
    assert "Fox" not in (entry.metadata.source_span or ""), (
        "the citation was substituted for the span: a plausible-looking "
        "provenance string that points at no particular part of the paper.")
    assert "1997" not in (entry.metadata.source_span or "")


def test_the_library_reports_what_g1_asks_for(admitted, pilot_card):
    library, _ = admitted
    prov = library.provenance()
    assert prov["node_count"] == 1
    assert prov["by_origin"] == {"card": 1}
    # traceable_count was hardcoded 0 while every pilot card was unverified.
    # limit_speed_dwa's span was checked against the PDF on 2026-08-27
    # (cards/VERIFICATION.md), so it is now 1. Read off the card rather than
    # written down, because the remaining cards get verified one at a time and a
    # constant here would fail once per verification for no reason.
    assert prov["traceable_count"] == (1 if pilot_card.is_traceable else 0)


def test_an_admitted_node_carries_model_id_and_prompt_hash(admitted):
    """"A library that cannot say which model produced it cannot be
    regenerated."""
    library, _ = admitted
    entry, = library.all()
    assert entry.metadata.model_id == "claude-opus-5"
    assert entry.metadata.prompt_hash == GOOD_PROMPT_HASH
    identity = entry.identity()
    for field in ("model_id", "prompt_hash", "temperature", "origin",
                  "card_id", "confidence"):
        assert field in identity, field


def test_the_snapshot_tag_covers_the_generation_metadata(pilot_card, replay,
                                                         staging):
    """#8: every reported result names the frozen library snapshot it came
    from. Two libraries holding the same node from different models must not
    share a tag, or the snapshot cannot mean what it claims."""
    library_a, library_b = Library(), Library()
    candidate = generate("b", "navigate_to_goal", replay, card=pilot_card,
                         staging=staging, record=False)

    admit(candidate.metadata(), candidate.source, card=pilot_card,
          library=library_a)
    admit({**candidate.metadata(), "model_id": "some-other-model"},
          candidate.source, card=pilot_card, library=library_b)

    assert library_a.snapshot() != library_b.snapshot()


# --- the seal --------------------------------------------------------------

def test_nothing_in_generate_imports_the_sealed_split():
    """Chunk 11's AST guard, restated where the generation path lives.

    `core/tests/test_held_out.py` already walks `generate/` — this is the same
    check run from inside the package it protects, so that a reader of
    `generate/` finds it without having to know the guard exists elsewhere.
    Deliberately does not import the split to check for it.
    """
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    offenders = []
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            if not name.endswith(".py"):
                continue
            path = os.path.join(dirpath, name)
            tree = ast.parse(open(path, encoding="utf-8").read(), filename=path)
            for node in ast.walk(tree):
                if isinstance(node, ast.Import):
                    hit = any(a.name.startswith("core.scenarios_held_out")
                              for a in node.names)
                elif isinstance(node, ast.ImportFrom):
                    hit = (node.module or "").startswith("core.scenarios_held_out")
                else:
                    continue
                if hit:
                    offenders.append(os.path.relpath(path, root))
    assert not offenders, (
        f"{offenders} import the sealed held-out split. G3 measures scenarios "
        f"never seen during generation; a generation path that can read them "
        f"measures nothing.")


def test_no_prompt_mentions_the_held_out_split_by_name():
    """The prompts are the model's whole input. A prompt naming the split file
    would put it one tool call away even though nothing imports it."""
    from generate.generator import CONDITIONS, prompt_path
    for condition in CONDITIONS:
        text = prompt_path(condition).read_text(encoding="utf-8")
        assert "scenarios_held_out" not in text
        assert "held_out" not in text
        assert "held-out" not in text.lower()
