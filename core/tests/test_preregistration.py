"""
core/tests/test_preregistration.py

The enforcement half of chunk 17. `PRE-REGISTRATION.md` is a claim about what
was fixed before the first evaluation run; these tests are what makes the claim
checkable rather than a comment.

The one with teeth is `test_editing_a_constant_fails_this_suite`, which edits a
constant and asserts the check notices. A guard that has never been seen to fail
is not a guard.

    pytest core/tests/test_preregistration.py -q
"""

import re

import pytest

import core.metrics
import core.stats
from core import preregistration as prereg


def test_the_document_exists_and_parses():
    recorded, digest = prereg.parse_document()
    assert len(digest) == 64
    assert recorded, "the prereg-constants block is empty"


def test_every_frozen_constant_matches_the_document():
    prereg.verify()


def test_the_recorded_digest_matches_the_live_values():
    _, recorded_digest = prereg.parse_document()
    assert prereg.digest() == recorded_digest


def test_the_document_records_exactly_what_the_code_freezes():
    recorded, _ = prereg.parse_document()
    assert set(recorded) == set(prereg.values()), (
        "the document and core.preregistration.FROZEN list different things")


def test_editing_a_constant_fails_this_suite(monkeypatch):
    """The demonstration chunk 17's Done-when clause asks for.

    Move a threshold, and the check must fail with a message naming which one.
    Reverted by monkeypatch, so the suite leaves nothing behind.
    """
    monkeypatch.setattr(core.metrics, "GOAL_RADIUS", 0.75)
    with pytest.raises(AssertionError) as excinfo:
        prereg.verify()
    message = str(excinfo.value)
    assert "core.metrics.GOAL_RADIUS" in message
    assert "0.75" in message
    # And the digest moves too, so neither check is load-bearing alone.
    _, recorded_digest = prereg.parse_document()
    assert prereg.digest() != recorded_digest


def test_editing_only_the_document_also_fails():
    """The other direction: a value written in the document that the code does
    not hold. Without this, `verify()` could be satisfied by editing prose."""
    with open(prereg.document_path()) as fh:
        text = fh.read()
    tampered = text.replace("core.stats.X = 0.3", "core.stats.X = 0.05")
    assert tampered != text, "the X line moved; update this test"
    recorded, _ = prereg.parse_document(tampered)
    assert recorded["core.stats.X"] != prereg.values()["core.stats.X"]


def test_a_frozen_file_is_hashed_by_content():
    vals = prereg.values()
    for path in prereg.FROZEN_FILES:
        assert vals[path].startswith("sha256:")
        assert len(vals[path]) == len("sha256:") + 64


def test_the_held_out_split_hash_agrees_with_chunk_11():
    """Two independent records of the same seal. If they ever disagree, one of
    them was updated without the other and the split was re-sealed."""
    sealed = ("00b3312b0755cc94d252ccb2eb6a096cf0867aae19ef61c6c71cb31572870a63")
    assert prereg.values()["core/scenarios_held_out.py"] == "sha256:" + sealed


def test_the_document_names_the_decision_threshold_in_prose_too():
    """X has to be findable by a reader who does not open the code block --
    it is the number the whole document exists to fix."""
    with open(prereg.document_path()) as fh:
        text = fh.read()
    assert re.search(r"X = 0\.30", text), "X is not stated in prose"


def test_the_amendment_section_is_append_only_in_form():
    with open(prereg.document_path()) as fh:
        text = fh.read()
    assert "## Amendments" in text
    assert "never edit above this line" in text
