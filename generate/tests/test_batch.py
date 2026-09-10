"""
generate/tests/test_batch.py

The grid, and the three refusals it is built around.

**No test here calls a model.** `plan()` renders prompts and hashes them, which
is free; `run()` is exercised against a stub backend and a temporary staging
directory, never against `generate/staging/`, which is committed and is G1's
denominator.
"""

import json

import pytest

from generate import batch
from generate.cardgen import CITATION_CONDITIONS, PAPER_CONDITIONS
from generate.generator import CONDITIONS, CORPUS_CONDITIONS, TASKS


@pytest.fixture(scope="module")
def cards():
    from cards.store import CardStore
    return CardStore.load()


@pytest.fixture
def empty(tmp_path):
    """A staging directory with nothing in it, so 'already staged' is a
    property of the test rather than of whatever is committed."""
    return tmp_path


# --- the grid --------------------------------------------------------------

def test_the_grid_is_the_ceiling(cards, empty):
    """Same arithmetic as test_ceiling.py, reached the other way: through the
    thing that will actually spend the calls."""
    rows = batch.plan(cards=cards, staging=empty)
    per_condition = {c: sum(r["condition"] == c for r in rows)
                     for c in CONDITIONS}
    corpus, alone = len(cards) * len(TASKS), len(TASKS)
    assert per_condition == {"a": corpus, "b": corpus, "d": alone}
    n_corpus = len(CORPUS_CONDITIONS)
    assert len(rows) == len(TASKS) * (
        (len(CONDITIONS) - n_corpus) + n_corpus * len(cards))


def test_the_reference_lands_in_the_right_column(cards, empty):
    """One store entry, two treatments, and they must never be confused.

    `card_id` set means the source's CONTENT reached the model — since chunk 42
    that is the document itself (B). `citation_id` set means only the reference
    did (A). D gets neither, and no row may carry both — that would be a node
    claiming a provenance it did not receive."""
    for row in batch.plan(cards=cards, staging=empty):
        assert not (row["card_id"] and row.get("citation_id")), row["stem"]
        assert (row["card_id"] is not None) == (
            row["condition"] in PAPER_CONDITIONS)
        assert (row.get("citation_id") is not None) == (
            row["condition"] in CITATION_CONDITIONS)


def test_the_node_id_is_the_row_key(cards, empty):
    """Which is why the grid IS the ceiling: no nonce, no sample index.

    The key is the CARD prompt's hash since chunk 42 — the node prompt depends
    on a card that does not exist until the model writes one, so it cannot key a
    grid that must be computable before any call. See generator.plan_hash."""
    for row in batch.plan(cards=cards, staging=empty):
        assert row["node_id"] == f"gen_{row['condition']}_{row['prompt_hash']}"
        assert row["stem"].endswith(row["prompt_hash"])


def test_the_plan_is_deterministic_and_diffable(cards, empty):
    assert batch.plan(cards=cards, staging=empty) == \
        batch.plan(cards=cards, staging=empty)


def test_the_grid_can_be_filtered_without_changing_its_shape(cards, empty):
    rows = batch.plan(conditions=["b"], tasks=["search_for_goal"],
                      card_ids=["limit_speed_dwa"], cards=cards, staging=empty)
    assert len(rows) == 1
    assert rows[0]["condition"] == "b"
    assert rows[0]["card_id"] == "limit_speed_dwa"
    assert rows[0]["task"] == "search_for_goal"


def test_no_two_combinations_share_a_stem(cards, empty):
    assert batch.collisions(batch.plan(cards=cards, staging=empty)) == []


def test_a_stem_collision_is_detected_rather_than_silently_overwritten():
    """Constructed, because it cannot happen with today's inputs — and the
    failure mode is invisible if it ever does: the second write just wins."""
    row = {"stem": "b__x__deadbeef", "condition": "b", "task": "t",
           "card_id": "x"}
    assert batch.collisions([dict(row), dict(row)])


# --- resume, and the refusal to retry --------------------------------------

def _stage(directory, row, admitted, reason=None):
    (directory / f"{row['stem']}.json").write_text("{}", encoding="utf-8")
    (directory / f"{row['stem']}.result.json").write_text(
        json.dumps({"admitted": admitted, "reason": reason}), encoding="utf-8")


def test_state_comes_from_staging_not_from_a_progress_file(cards, empty):
    """Chunk 28's lesson: a progress file can disagree with the artefacts it
    describes. The staged record IS the state."""
    rows = batch.plan(cards=cards, staging=empty)
    assert all(not r["staged"] for r in rows)
    _stage(empty, rows[0], admitted=True)
    after = batch.plan(cards=cards, staging=empty)
    assert after[0]["staged"] and after[0]["admitted"] is True
    assert sum(r["staged"] for r in after) == 1


def test_a_rejection_is_skipped_and_never_retried(cards, empty):
    """Re-running a combination until it is admitted produces a pass rate with
    no denominator — and the pass rate is the one number G1 asks for."""
    rows = batch.plan(cards=cards, staging=empty)
    _stage(empty, rows[0], admitted=False, reason="UNIT_SCENARIO_FAILED")
    rows = batch.plan(cards=cards, staging=empty)
    todo = batch.pending(rows)
    assert rows[0]["stem"] not in {r["stem"] for r in todo}
    assert len(todo) == len(rows) - 1


def test_force_is_the_only_way_past_and_it_is_not_the_default(cards, empty):
    rows = batch.plan(cards=cards, staging=empty)
    _stage(empty, rows[0], admitted=False, reason="UNIT_SCENARIO_FAILED")
    rows = batch.plan(cards=cards, staging=empty)
    assert len(batch.pending(rows, force=True)) == len(rows)


def test_limit_caps_the_spend(cards, empty):
    rows = batch.plan(cards=cards, staging=empty)
    assert len(batch.pending(rows, limit=4)) == 4
    assert len(batch.pending(rows, limit=0)) == 0


def test_an_unreadable_record_is_still_a_record(cards, empty):
    """It counts as staged. Overwriting it would destroy whatever G1's
    denominator was supposed to contain, and that is worse than a gap."""
    rows = batch.plan(cards=cards, staging=empty)
    (empty / f"{rows[0]['stem']}.json").write_text("{}", encoding="utf-8")
    (empty / f"{rows[0]['stem']}.result.json").write_text(
        "not json", encoding="utf-8")
    rows = batch.plan(cards=cards, staging=empty)
    assert rows[0]["staged"] and rows[0]["reason"] == "UNREADABLE_RECORD"
    assert rows[0]["stem"] not in {r["stem"] for r in batch.pending(rows)}


# --- running ---------------------------------------------------------------

class _Boom:
    """A backend that always raises, to prove one bad row does not abandon the
    rest of the grid."""

    def complete(self, prompt, **kwargs):
        raise RuntimeError("no model here")

    def __getattr__(self, name):
        raise RuntimeError("no model here")


def test_a_failed_row_is_recorded_and_the_rest_still_run(cards, empty):
    rows = batch.plan(conditions=["a"], cards=cards, staging=empty)
    summary = batch.run(rows, _Boom(), cards=cards, record=False,
                        staging=empty)
    assert summary["failed"] == len(rows)
    assert summary["admitted"] == summary["rejected"] == 0
    assert len(summary["results"]) == len(rows)
    assert all(r["outcome"] == "failed" for r in summary["results"])
    # Nothing was written into staging, so a later run still sees them pending.
    assert all(not r["staged"] for r in batch.plan(conditions=["a"],
                                                   cards=cards, staging=empty))


def test_describe_says_what_will_happen(cards, empty):
    rows = batch.plan(conditions=["a"], cards=cards, staging=empty)
    assert "PENDING" in batch.describe(rows[0])
    _stage(empty, rows[0], admitted=True)
    rows = batch.plan(conditions=["a"], cards=cards, staging=empty)
    assert "ADMITTED" in batch.describe(rows[0])
