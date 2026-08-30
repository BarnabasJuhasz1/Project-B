"""
generate/tests/test_ceiling.py

**How many distinct nodes can each condition reach at all?**

`generator.render()` is a pure function of `(condition, task, card)`. There is no
nonce, no sample index and no seed, and `Candidate.node_id` IS the prompt hash —
so two calls with the same inputs produce the same node id, the same staging stem
and the same transcript key, and the second silently overwrites the first. The
reachable library is therefore not a budget question. It is arithmetic:

    A:  |TASKS|                 D:  |TASKS|                 B:  |cards| x |TASKS|

G1 asks for >=20 admissible nodes per condition, so A and D **cannot reach it by
spending more calls**. That fact drives the whole of plans/todo/29, and it was
prose in a plan file until this module made it a test. The numbers below are
asserted explicitly rather than derived, so that adding a task family or a card
fails here and the plan gets re-read, rather than the ceiling silently moving
under a decision that was taken against it.

No test in this module calls a model: rendering a prompt is free.
"""

import pytest

from cards.store import CardStore
from generate import backends
from generate.generator import CONDITIONS, TASKS, render

# ---- the ceiling, stated. Below this line is data, not code. ----------------
# Update these WITH plans/todo/29-grow-libraries.md, never instead of it.
EXPECTED_TASKS = 3
EXPECTED_CARDS = 5
EXPECTED_PROMPTS = {"a": 3, "b": 15, "d": 3}
G1_TARGET = 20
# ---- above this line is data ------------------------------------------------


@pytest.fixture(scope="module")
def store():
    return CardStore.load()


def _prompts(condition, store):
    """Every distinct prompt this condition can produce, as hashes."""
    cards = list(store) if condition == "b" else [None]
    return {backends.prompt_hash(render(condition, task, card))
            for task in TASKS for card in cards}


def test_the_inputs_are_the_size_the_plan_costed(store):
    assert len(TASKS) == EXPECTED_TASKS, sorted(TASKS)
    assert len(store) == EXPECTED_CARDS, [c.id for c in store]


@pytest.mark.parametrize("condition", sorted(EXPECTED_PROMPTS))
def test_the_reachable_node_count_is_fixed_by_the_prompt_inputs(condition, store):
    """Distinct prompts == distinct node ids, because the id IS the hash."""
    assert len(_prompts(condition, store)) == EXPECTED_PROMPTS[condition]


def test_a_and_d_cannot_reach_g1s_target_by_spending_more_calls():
    """The finding that plans/todo/29 exists to force a decision about. This is
    an upper bound assuming EVERY candidate is admitted; the observed pass rate
    is well below 1, so the expected yield is lower still."""
    for condition in ("a", "d"):
        assert EXPECTED_PROMPTS[condition] < G1_TARGET
    assert EXPECTED_PROMPTS["a"] == EXPECTED_PROMPTS["d"] == len(TASKS)


def test_condition_b_scales_with_the_corpus_and_a_and_d_do_not():
    """Why 'more cards' is not the answer to the ceiling: it is not
    condition-neutral. Cards are B's treatment, so enumerating over them lifts
    B alone and makes library size a second, uncontrolled difference between
    conditions — which is chunk 29's Option 3, recorded to be rejected."""
    assert EXPECTED_PROMPTS["b"] == EXPECTED_CARDS * EXPECTED_TASKS
    assert EXPECTED_PROMPTS["a"] == EXPECTED_PROMPTS["d"]  # no card term at all


def test_no_two_inputs_collide_on_one_prompt(store):
    """A collision would mean two (card, task) pairs sharing a node id, so one
    would overwrite the other in staging and nobody would see it happen."""
    seen = {}
    for condition in sorted(EXPECTED_PROMPTS):
        cards = list(store) if condition == "b" else [None]
        for task in TASKS:
            for card in cards:
                h = backends.prompt_hash(render(condition, task, card))
                key = (condition, task, card.id if card else None)
                assert h not in seen, f"{key} collides with {seen[h]}"
                seen[h] = key


def test_every_condition_with_a_prompt_is_costed():
    """A fourth condition added to CONDITIONS without a line in EXPECTED_PROMPTS
    would be generated against but never costed, so chunk 29's arithmetic would
    quietly stop describing the pipeline."""
    assert set(EXPECTED_PROMPTS) == set(CONDITIONS)
