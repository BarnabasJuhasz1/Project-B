"""
generate/tests/test_prompts.py

The prompts ARE the independent variable, so these tests are not style checks.

Chunk 15: "The three prompt files differ **only** in their literature block —
asserted by a test that diffs them, not by reading. If A, B and D drift apart
anywhere else, the ablation measures that difference instead of the corpus, and
a diff test is the only thing that catches it early."

The three files are real files rather than a template rendered three ways, and
that is deliberate: a reviewer has to be able to read `condition_b.md` whole and
see exactly what the model was told. The cost of that redundancy is drift, and
these tests are what buys it back.
"""

import re

import pytest

from generate.generator import (CONDITIONS, LITERATURE_BLOCK, TASKS,
                                literature_block, prompt_path, render,
                                shared_body)


def test_every_condition_has_a_prompt():
    for condition in CONDITIONS:
        assert prompt_path(condition).exists(), condition


def test_condition_c_has_no_prompt():
    """C is an off-the-shelf Nav2 stack. If it ever acquires a generation
    prompt, someone has quietly turned the human baseline into a fourth
    generated condition."""
    with pytest.raises(KeyError):
        prompt_path("c")


def test_the_prompts_differ_only_in_the_literature_block():
    """THE test. Strip the delimited block from each file; what is left must be
    byte-identical across A, B and D."""
    bodies = {c: shared_body(c) for c in CONDITIONS}
    reference = bodies["b"]
    for condition, body in bodies.items():
        assert body == reference, (
            f"condition_{condition}.md differs from condition_b.md OUTSIDE the "
            f"literature block. Everything except that block is held constant "
            f"across the ablation; a difference here means B−A and B−D measure "
            f"this drift as well as the corpus.")


def test_the_literature_blocks_are_all_different():
    """The mirror image, and it is not a formality: three identical blocks
    would pass the test above perfectly and make the ablation vacuous."""
    blocks = {c: literature_block(c) for c in CONDITIONS}
    assert len(set(blocks.values())) == len(CONDITIONS), (
        "two conditions have the same literature block, so they are the same "
        "condition")


def test_the_shared_body_is_not_trivially_small():
    """A body that shrank to nothing would satisfy the diff test and would mean
    the contract spec had migrated into the per-condition blocks, where it could
    differ between conditions unnoticed."""
    body = shared_body("b")
    assert len(body) > 6000, len(body)
    for required in ("PortType", "Observation", "PREDICTION", "MicroScenario",
                     "goal_seeker", "reactive_avoid"):
        assert required in body, (
            f"{required!r} is not in the SHARED body. The node contract, the "
            f"output format and the harness rules must be identical across "
            f"conditions; anything that drifted into a literature block is a "
            f"difference between conditions nobody declared.")


def test_only_condition_b_has_a_card_slot():
    """The corpus is B's independent variable and only B's."""
    with_slot = [c for c in CONDITIONS
                 if "{{CARD}}" in prompt_path(c).read_text(encoding="utf-8")]
    assert with_slot == ["b"], with_slot


def test_a_card_cannot_reach_condition_a_or_d(pilot_card):
    """Passing a card to a condition with nowhere to put it must fail loudly.
    Silently ignoring it would run condition A with B's inputs and call it A."""
    for condition in ("a", "d"):
        with pytest.raises(Exception):
            render(condition, "navigate_to_goal", pilot_card)


def test_condition_b_without_a_card_fails():
    from generate.generator import GenerationError
    with pytest.raises(GenerationError):
        render("b", "navigate_to_goal", card=None)


def test_every_placeholder_is_substituted(pilot_card):
    """A `{{TASK}}` that survived into the prompt is a template bug the model
    would have to interpret, differently each time."""
    for condition in CONDITIONS:
        card = pilot_card if condition == "b" else None
        for task in TASKS:
            text = render(condition, task, card)
            assert "{{" not in text, (condition, task,
                                      text[text.find("{{"):][:60])


def test_the_task_specification_is_identical_across_conditions(pilot_card):
    """The retrieval query is held constant. If A and B were asked about
    different tasks, B−A would be a difference of task."""
    for task, spec in TASKS.items():
        for condition in CONDITIONS:
            card = pilot_card if condition == "b" else None
            assert spec in render(condition, task, card), (condition, task)


def test_the_card_reaches_the_model_intact(pilot_card):
    """Condition B's whole treatment is the card's content. A field dropped in
    rendering is a treatment nobody administered."""
    text = render("b", "navigate_to_goal", pilot_card)
    for field in (pilot_card.mechanism, pilot_card.trigger,
                  pilot_card.derivation, pilot_card.prediction.statement):
        assert " ".join(field.split()) in " ".join(text.split()), field[:60]
    assert pilot_card.prediction.metric in text
    assert pilot_card.prediction.baseline in text


def test_no_prompt_names_a_scenario():
    """Nothing in the prompts may name a benchmark scenario.

    The dev set is legitimately "seen during generation" and a card's own
    Prediction statement may cite one — `limit_speed_dwa` names
    doorway_traverse — but the PROMPT TEMPLATE must not, or every generated
    node inherits a scenario the templates chose. The held-out split is a
    harder rule and is covered by the seal guard in core/tests/test_held_out.py.
    """
    from core.scenario import DEV_SCENARIOS
    names = {s.id for s in DEV_SCENARIOS} | {"lab42"}
    for condition in CONDITIONS:
        text = prompt_path(condition).read_text(encoding="utf-8")
        found = sorted(n for n in names if n in text)
        assert not found, (
            f"condition_{condition}.md names {found}. A prompt that names a "
            f"scenario couples every node it generates to that scenario, which "
            f"is the thing §4 forbids.")


def test_the_block_markers_are_well_formed():
    """One begin, one end, in that order, in every file."""
    for condition in CONDITIONS:
        text = prompt_path(condition).read_text(encoding="utf-8")
        assert text.count("<!-- ---- LITERATURE BLOCK BEGIN ---- -->") == 1
        assert text.count("<!-- ---- LITERATURE BLOCK END ---- -->") == 1
        assert len(LITERATURE_BLOCK.findall(text)) == 1
