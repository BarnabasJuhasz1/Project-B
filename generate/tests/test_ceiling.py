"""
generate/tests/test_ceiling.py

**How many distinct nodes can each condition reach at all?**

`generator.render()` is a pure function of `(condition, task, card)`. There is no
nonce, no sample index and no seed, and `Candidate.node_id` IS the prompt hash —
so two calls with the same inputs produce the same node id, the same staging stem
and the same transcript key, and the second silently overwrites the first. The
reachable library is therefore not a budget question. It is arithmetic:

    A:  |cards| x |TASKS|       D:  |TASKS|                 B:  |cards| x |TASKS|

G1 asks for >=20 admissible nodes per condition, so D **cannot reach it by
spending more calls**. A could once not either: until 2026-09-06 it was handed
no literature at all and its ceiling was |TASKS|. It is now handed the CITATION
of a card — the reference and none of its content — so it enumerates over the
store exactly as B does and its ceiling is B's. **Only D is still capped.**
That fact drives the whole of plans/done/29, and it was
prose in a plan file until this module made it a test. The numbers below are
asserted explicitly rather than derived, so that adding a task family or a card
fails here and the plan gets re-read, rather than the ceiling silently moving
under a decision that was taken against it.

No test in this module calls a model: rendering a prompt is free.
"""

import pytest

from cards.store import CardStore
from generate.generator import (CONDITIONS, CORPUS_CONDITIONS, TASKS,
                                plan_hash)

# ---- the ceiling, stated. Below this line is data, not code. ----------------
# Update these WITH plans/done/29-grow-libraries.md, never instead of it.
#
# Moved 2026-09-05 from (3, 15, 3) to (4, 20, 4) by chunk 29's open question 2:
# the fourth cell of chunk 11a's frozen 2x2 (hidden goal AND drifting pose) was
# taken as a task family. It is the only condition-neutral growth lever — it
# multiplies A, B and D alike — so the ablation is untouched and no committed
# transcript is orphaned.
#
# Moved again 2026-09-06 from (4, 20, 4) to (20, 20, 4) by the condition
# restructure: A is now handed each card's CITATION, so it enumerates over the
# store and reaches B's ceiling. **A reaches G1_TARGET; D does not.** The
# ceiling finding survives, narrowed to the one condition that still has no
# corpus axis at all.
#
# UNCHANGED by chunk 42, and that is worth stating rather than assuming. Every
# arm now writes its own card before its node, and all three card prompts are
# task-conditioned, so the grid keeps the shape it had: A and B enumerate
# |cards| x |TASKS|, D enumerates |TASKS|. What moved is which prompt the hash
# is over — the CARD prompt, since the node prompt depends on a card that does
# not exist until the model writes one. See generator.plan_hash.
EXPECTED_TASKS = 4
EXPECTED_CARDS = 5
EXPECTED_PROMPTS = {"a": 20, "b": 20, "d": 4}
G1_TARGET = 20
# ---- above this line is data ------------------------------------------------


@pytest.fixture(scope="module")
def store():
    return CardStore.load()


def _prompts(condition, store):
    """Every distinct prompt this condition can produce, as hashes."""
    cards = list(store) if condition in CORPUS_CONDITIONS else [None]
    return {plan_hash(condition, task, card)
            for task in TASKS for card in cards}


def test_the_inputs_are_the_size_the_plan_costed(store):
    assert len(TASKS) == EXPECTED_TASKS, sorted(TASKS)
    assert len(store) == EXPECTED_CARDS, [c.id for c in store]


@pytest.mark.parametrize("condition", sorted(EXPECTED_PROMPTS))
def test_the_reachable_node_count_is_fixed_by_the_prompt_inputs(condition, store):
    """Distinct prompts == distinct node ids, because the id is derived from
    the row key, which is the card prompt's hash."""
    assert len(_prompts(condition, store)) == EXPECTED_PROMPTS[condition]


def test_d_cannot_reach_g1s_target_by_spending_more_calls():
    """The finding that plans/done/29 exists to force a decision about, now
    narrowed to D. This is an upper bound assuming EVERY candidate is admitted;
    the observed pass rate is well below 1, so the expected yield is lower still.

    A is deliberately no longer in this test. It used to be capped at |TASKS|
    because it received no literature; since 2026-09-06 it receives a citation
    per card, so its ceiling is the corpus's and the cap is D's alone."""
    assert EXPECTED_PROMPTS["d"] < G1_TARGET
    assert EXPECTED_PROMPTS["d"] == len(TASKS)


def test_the_conditions_that_see_the_corpus_scale_with_it():
    """A and B both enumerate over the store — A over citations, B over the
    extracted content — so they reach the same ceiling from the same inputs.
    That equality is the point of the restructure: A and B differ by the card's
    CONTENT and by nothing else, including how many calls each can make."""
    assert EXPECTED_PROMPTS["a"] == EXPECTED_PROMPTS["b"]
    assert EXPECTED_PROMPTS["a"] == EXPECTED_CARDS * EXPECTED_TASKS
    assert set(CORPUS_CONDITIONS) == {"a", "b"}


def test_more_cards_still_does_not_lift_d():
    """Chunk 29's condition-neutrality rule, restated for the new structure.

    A new card lifts A and B together and leaves D exactly where it was, so the
    corpus is no longer a lever that moves one arm alone — which is what made
    chunk 29's Option 3 a confound. It is still not neutral with respect to D,
    and D's ceiling is now the binding one for G1."""
    assert EXPECTED_PROMPTS["b"] == EXPECTED_CARDS * EXPECTED_TASKS
    assert EXPECTED_PROMPTS["d"] == EXPECTED_TASKS      # no card term at all
    assert "d" not in CORPUS_CONDITIONS


def test_no_two_inputs_collide_on_one_prompt(store):
    """A collision would mean two (card, task) pairs sharing a node id, so one
    would overwrite the other in staging and nobody would see it happen."""
    seen = {}
    for condition in sorted(EXPECTED_PROMPTS):
        cards = list(store) if condition in CORPUS_CONDITIONS else [None]
        for task in TASKS:
            for card in cards:
                h = plan_hash(condition, task, card)
                key = (condition, task, card.id if card else None)
                assert h not in seen, f"{key} collides with {seen[h]}"
                seen[h] = key


def test_every_condition_with_a_prompt_is_costed():
    """A fourth condition added to CONDITIONS without a line in EXPECTED_PROMPTS
    would be generated against but never costed, so chunk 29's arithmetic would
    quietly stop describing the pipeline."""
    assert set(EXPECTED_PROMPTS) == set(CONDITIONS)
