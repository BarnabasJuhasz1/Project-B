"""
generate/tests/test_prompts.py

The prompts ARE the independent variable, so these tests are not style checks.

**CHUNK 42 SPLIT THE PROMPT IN TWO AND MOVED THE VARIABLE.** Generation used to
be one call whose prompt differed per condition, and chunk 15's rule was "the
three files differ only in their literature block". It is now:

    the CARD call   card_{a,b,d}.md — three files, differing ONLY inside a
                    delimited SOURCE BLOCK. Chunk 15's rule, moved.
    the NODE call   node.md — ONE file, for every condition, fed a card and
                    nothing that identifies which arm wrote it.

So there are two invariants to hold, and the second is the stronger one: after
the card exists the pipeline cannot tell the arms apart, which makes "the
conditions differ by exactly one thing" structural rather than a diff over three
files. These tests pin both.

The three card files are real files rather than a template rendered three ways,
and that is deliberate: a reviewer has to be able to read `card_b.md` whole and
see exactly what the model was told. The cost of that redundancy is drift, and
the diff test is what buys it back.
"""

import pytest

from generate import cardgen
from generate.cardgen import (CITATION_CONDITIONS, PAPER_CONDITIONS,
                              SOURCE_BLOCK, CardGenerationError, render,
                              shared_body, source_block)
from generate.generator import (CONDITIONS, CORPUS_CONDITIONS, NODE_PROMPT,
                                TASKS, card_yaml, render_node, task_text)


# --- the card call: chunk 15's rule, moved ---------------------------------

def test_every_condition_has_a_card_prompt():
    for condition in CONDITIONS:
        assert cardgen.prompt_path(condition).exists(), condition


def test_condition_c_has_no_card_prompt():
    """C is an off-the-shelf Nav2 stack. If it ever acquires a card prompt,
    someone has quietly turned the human baseline into a fourth generated
    condition."""
    with pytest.raises(CardGenerationError):
        cardgen.prompt_path("c")


def test_the_card_prompts_differ_only_in_the_source_block():
    """THE test. Strip the delimited block from each file; what is left must be
    byte-identical across A, B and D."""
    bodies = {c: shared_body(c) for c in CONDITIONS}
    reference = bodies["b"]
    for condition, body in bodies.items():
        assert body == reference, (
            f"card_{condition}.md differs from card_b.md OUTSIDE the source "
            f"block. Everything except that block is held constant across the "
            f"ablation; a difference here means B−A and B−D measure this drift "
            f"as well as the corpus.")


def test_the_source_blocks_are_all_different():
    """The mirror image, and it is not a formality: three identical blocks would
    pass the test above perfectly and make the ablation vacuous."""
    blocks = {c: source_block(c) for c in CONDITIONS}
    assert len(set(blocks.values())) == len(CONDITIONS), (
        "two conditions have the same source block, so they are the same "
        "condition")


def test_the_shared_card_body_is_not_trivially_small():
    """A body that shrank to nothing would satisfy the diff test and would mean
    the card schema had migrated into the per-condition blocks, where it could
    differ between conditions unnoticed."""
    body = shared_body("b")
    assert len(body) > 4000, len(body)
    for required in ("mechanism", "trigger", "derivation", "prediction",
                     "confidence", "goal_seeker", "reactive_avoid",
                     "localisation_failures", "{{TASK}}"):
        assert required in body, (
            f"{required!r} is not in the SHARED card body. The schema, the "
            f"metric list and the task family must be identical across "
            f"conditions; anything that drifted into a source block is a "
            f"difference between conditions nobody declared.")


def test_each_arm_owns_exactly_one_source_slot():
    """A is pointed at a reference, B is handed a document, D gets neither.
    A prompt with two slots, or with the wrong one, is a different experiment."""
    slots = {c: cardgen.prompt_path(c).read_text(encoding="utf-8")
             for c in CONDITIONS}
    assert [c for c in CONDITIONS if "{{CITATION}}" in slots[c]] == \
        list(CITATION_CONDITIONS)
    assert [c for c in CONDITIONS if "{{PAPER}}" in slots[c]] == \
        list(PAPER_CONDITIONS)
    for condition in CONDITIONS:
        assert not ("{{CITATION}}" in slots[condition]
                    and "{{PAPER}}" in slots[condition]), condition


def test_a_source_cannot_reach_the_arm_that_has_none(pilot_card):
    """Passing a source to a condition with nowhere to put it must fail loudly.
    Silently ignoring it would run D with a corpus and call it D."""
    with pytest.raises(CardGenerationError):
        render("d", task_text("navigate_to_goal"), card=pilot_card)


def test_condition_a_receives_the_citation_and_none_of_the_card(pilot_card):
    """**The invariant the whole restructure rests on.**

    A and B are pointed at the same paper. B is handed the document; A is given
    the reference and must recall the rest. If any extracted field leaked into
    A's prompt, B−A would measure transcription rather than extraction, and it
    would look exactly the same in the numbers.
    """
    text = render("a", task_text("navigate_to_goal"), card=pilot_card)
    assert pilot_card.source.citation.strip() in text

    flat = " ".join(text.split())
    for name, field in (("mechanism", pilot_card.mechanism),
                        ("trigger", pilot_card.trigger),
                        ("derivation", pilot_card.derivation),
                        ("prediction", pilot_card.prediction.statement)):
        assert " ".join(field.split()) not in flat, f"{name} leaked into A"
    # The id and node_name read like a summary of the finding, so they are
    # withheld too: `limit_speed_dwa` would hand A the mechanism in the pointer.
    assert pilot_card.id not in text
    assert pilot_card.node_name not in text
    if pilot_card.source.span:
        assert pilot_card.source.span not in text


def test_condition_b_receives_the_document_and_not_the_human_extraction(
        pilot_card):
    """**What chunk 42 changed about B.**

    B used to be handed a human's card. It is now handed the paper, and the
    human extraction must not travel with it — otherwise the arm is reading
    somebody's answer next to the question and B−A measures nothing about
    extraction.
    """
    paper = pytest.importorskip("generate.corpus")
    if not paper.available(pilot_card.id):
        pytest.skip("out/corpus/ is not populated on this machine")
    text = render("b", task_text("navigate_to_goal"), card=pilot_card,
                  paper=paper.load(pilot_card.id))

    flat = " ".join(text.split())
    for name, field in (("mechanism", pilot_card.mechanism),
                        ("derivation", pilot_card.derivation),
                        ("prediction", pilot_card.prediction.statement)):
        assert " ".join(field.split()) not in flat, f"{name} leaked into B"
    assert pilot_card.node_name not in text


def test_a_missing_source_is_refused_rather_than_rendered_empty(pilot_card):
    for condition in CORPUS_CONDITIONS:
        with pytest.raises(CardGenerationError):
            render(condition, task_text("navigate_to_goal"), card=None)


def test_every_card_placeholder_is_substituted(pilot_card):
    """A `{{TASK}}` that survived into the prompt is a template bug the model
    would have to interpret, differently each time."""
    class _StandIn:
        text = "(document)"

    for condition in CONDITIONS:
        card = pilot_card if condition in CORPUS_CONDITIONS else None
        paper = _StandIn() if condition in PAPER_CONDITIONS else None
        for task in TASKS:
            text = render(condition, task_text(task), card=card, paper=paper)
            assert "{{" not in text, (condition, task,
                                      text[text.find("{{"):][:60])


def test_the_task_specification_is_identical_across_conditions(pilot_card):
    """The retrieval query is held constant. If A and B were asked about
    different tasks, B−A would be a difference of task."""
    class _StandIn:
        text = "(document)"

    for task, spec in TASKS.items():
        for condition in CONDITIONS:
            card = pilot_card if condition in CORPUS_CONDITIONS else None
            paper = _StandIn() if condition in PAPER_CONDITIONS else None
            assert spec in render(condition, task_text(task), card=card,
                                  paper=paper), (condition, task)


def test_the_card_block_markers_are_well_formed():
    """One begin, one end, in that order, in every file."""
    for condition in CONDITIONS:
        text = cardgen.prompt_path(condition).read_text(encoding="utf-8")
        assert text.count("<!-- ---- SOURCE BLOCK BEGIN ---- -->") == 1
        assert text.count("<!-- ---- SOURCE BLOCK END ---- -->") == 1
        assert len(SOURCE_BLOCK.findall(text)) == 1


# --- the node call: one prompt, blind to the arm ---------------------------

def test_there_is_exactly_one_node_prompt():
    """The strongest property chunk 42 buys. A second node prompt — or a
    `condition` parameter on `render_node` — would give the ablation a second
    place to differ, and the diff test above would not see it."""
    assert NODE_PROMPT.exists()
    import inspect
    params = list(inspect.signature(render_node).parameters)
    assert params == ["card", "task"], (
        f"render_node takes {params}. It must not be able to tell the arms "
        f"apart: the moment it can, the node call becomes a second independent "
        f"variable.")


def test_the_node_prompt_renders_the_card_and_withholds_its_provenance(
        pilot_card):
    """Mechanism, trigger, derivation and prediction are the principle and go in.
    Citation, span and confidence are provenance, belong in node metadata, and
    are what would tell the node call which arm it was serving — the arm with no
    source announces itself in a citation line."""
    rendered = card_yaml(pilot_card)
    for field in (pilot_card.mechanism, pilot_card.trigger,
                  pilot_card.derivation, pilot_card.prediction.statement):
        assert " ".join(field.split()) in " ".join(rendered.split())
    assert pilot_card.prediction.metric in rendered
    assert pilot_card.prediction.baseline in rendered
    assert pilot_card.node_name in rendered

    assert "citation" not in rendered.lower()
    assert pilot_card.source.citation.strip() not in rendered
    assert cardgen.NO_SOURCE not in rendered
    if pilot_card.source.span:
        assert pilot_card.source.span not in rendered
    assert pilot_card.confidence.value not in rendered.split()


def test_two_arms_cards_render_through_the_same_template(pilot_card, vague_card):
    """The rendered node prompts must differ by the card and by nothing else."""
    a = render_node(pilot_card, "navigate_to_goal")
    b = render_node(vague_card, "navigate_to_goal")
    for text, card in ((a, pilot_card), (b, vague_card)):
        stripped = text.replace(card_yaml(card), "{{CARD}}")
        assert "{{CARD}}" in stripped
    assert a.replace(card_yaml(pilot_card), "") == \
        b.replace(card_yaml(vague_card), "")


def test_every_node_placeholder_is_substituted(pilot_card):
    for task in TASKS:
        text = render_node(pilot_card, task)
        assert "{{" not in text, text[text.find("{{"):][:60]


def test_no_prompt_names_a_scenario():
    """Nothing in the prompts may name a benchmark scenario.

    The dev set is legitimately "seen during generation" and a card's own
    Prediction statement may cite one — `limit_speed_dwa` names
    doorway_traverse — but the PROMPT TEMPLATE must not, or every generated node
    inherits a scenario the templates chose. The held-out split is a harder rule
    and is covered by the seal guard in core/tests/test_held_out.py.
    """
    from core.scenario import DEV_SCENARIOS
    names = {s.id for s in DEV_SCENARIOS} | {"lab42"}
    paths = [cardgen.prompt_path(c) for c in CONDITIONS] + [NODE_PROMPT]
    for path in paths:
        text = path.read_text(encoding="utf-8")
        found = sorted(n for n in names if n in text)
        assert not found, (
            f"{path.name} names {found}. A prompt that names a scenario couples "
            f"every node it generates to that scenario, which is what §4 "
            f"forbids.")
