"""
generate/tests/test_replay.py

The reproducibility claim, exercised on the two committed transcripts.

Chunk 15 asks that "the same card, same prompt, same model id and temperature 0
produces a byte-identical candidate, **or the metadata explains why not**". It is
the second branch, and these tests are what make the explanation checkable
rather than a paragraph in a docstring:

- temperature-0 determinism is unavailable — the parameter was removed from the
  Messages API on this model family, so the pipeline never had it to lose;
- what replaces it is the transcript, and replaying one reproduces the candidate
  byte-for-byte with no model, no key and no network;
- the metadata says which of the two it is, on every node, and a node whose
  metadata does not say so is refused at admission.

The transcripts under `generate/transcripts/` are the record of real
`claude -p` calls against `claude-opus-5`. They are committed for the same
reason `scenes/lab42_v1.usd` is: the pipeline must be re-runnable from files.
"""

import json

import pytest

from generate import backends
from generate.admit import admit
from generate.backends import (TEMPERATURE_MODELS, BackendError, ReplayBackend,
                               prompt_hash)
from generate.generator import extract_code, generate, render
from nodes.contract import Direction

from generate.tests.conftest import GOOD_PROMPT_HASH, VAGUE_PROMPT_HASH


def test_the_committed_transcript_matches_the_prompt_it_was_taken_against(
        pilot_card, replay):
    """The hash is over the RENDERED prompt, so this fails the moment a prompt
    file is edited. That is the intended behaviour and it is worth stating:
    editing a prompt orphans every transcript taken against it, because the
    prompt is the independent variable and a changed one is a changed
    experiment."""
    prompt = render("b", "navigate_to_goal", pilot_card)
    assert prompt_hash(prompt) == GOOD_PROMPT_HASH, (
        "generate/prompts/ has changed since the committed transcript was "
        "recorded. Re-record it with scripts/generate_node.py, and note in the "
        "chunk Outcome that the recorded generation is a different one.")
    assert replay.path_for(prompt).exists()


def test_replaying_is_byte_identical(pilot_card, replay, staging, tmp_path):
    """Twice through the replay backend, same bytes both times. This is the
    reproducibility property the project actually has."""
    first = generate("b", "navigate_to_goal", replay, card=pilot_card,
                     staging=staging, record=False)
    second = generate("b", "navigate_to_goal", replay, card=pilot_card,
                      staging=staging, record=False)
    assert first.source == second.source
    assert first.prompt_hash == second.prompt_hash == GOOD_PROMPT_HASH
    assert first.response.model_id == "claude-opus-5"


def test_the_replayed_candidate_is_the_one_that_was_admitted(pilot_card, replay,
                                                             staging):
    """End to end with no model: transcript -> candidate -> harness ->
    ADMITTED, reproducing the committed result in generate/staging/."""
    candidate = generate("b", "navigate_to_goal", replay, card=pilot_card,
                         staging=staging, record=False)
    result = admit(candidate.metadata(), candidate.source, card=pilot_card)
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert result.unit.metric == pilot_card.prediction.metric
    assert result.unit.effect >= result.unit.required

    committed = json.loads(
        (backends.REPO_ROOT / "generate" / "staging" /
         f"b__limit_speed_dwa__{GOOD_PROMPT_HASH}.result.json")
        .read_text(encoding="utf-8"))
    assert committed["admitted"]
    assert committed["unit"]["effect"] == result.unit.effect, (
        "the committed record and a fresh replay disagree. Scoring is a pure "
        "function of the rollout, so this means the harness or the metrics "
        "moved under a result that is already committed.")


def test_replay_refuses_to_invent_a_generation(replay):
    """A missing transcript is an error, not an empty candidate. Replay
    reproduces a recorded call; it cannot manufacture one."""
    with pytest.raises(BackendError) as exc:
        replay.complete("a prompt that was never sent to anything")
    assert "no transcript" in str(exc.value)


def test_a_transcript_records_what_it_has_to(pilot_card, replay):
    """Everything needed to attribute a node and to regenerate it."""
    prompt = render("b", "navigate_to_goal", pilot_card)
    data = json.loads(replay.path_for(prompt).read_text(encoding="utf-8"))

    assert data["prompt"] == prompt, (
        "the transcript stores the prompt in full, not just its hash — the "
        "difference between a transcript you can audit and a hash you have to "
        "trust")
    assert data["prompt_hash"] == prompt_hash(prompt)
    assert data["meta"]["card_id"] == "limit_speed_dwa"
    assert data["meta"]["condition"] == "b"
    for field in ("text", "model_id", "backend", "temperature",
                  "sampling_note"):
        assert field in data["response"], field


def test_the_generation_model_does_not_accept_a_temperature():
    """The fact the whole determinism story turns on, pinned so that a future
    model change is a test failure rather than a silent 400 in Week 5."""
    assert backends.DEFAULT_MODEL not in TEMPERATURE_MODELS, (
        f"{backends.DEFAULT_MODEL} now accepts a sampling temperature. If that "
        f"is right, temperature-0 determinism is available again and "
        f"generate/backends.py's docstring, generate/FINDINGS.md and chunk 17 "
        f"all need revisiting.")


def test_a_null_temperature_always_carries_its_reason(pilot_card, replay):
    """`temperature: None` on a node is only honest if something says why."""
    prompt = render("b", "navigate_to_goal", pilot_card)
    response = replay.complete(prompt)
    assert response.temperature is None
    assert "does not accept a temperature parameter" in response.sampling_note


def test_the_bad_card_transcript_is_the_one_that_was_admitted(vague_card,
                                                              replay, staging):
    """generate/FINDINGS.md §1, pinned as a test.

    The deliberately-bad card produced an ADMITTED node. That is not a bug to
    fix here and the test does not assert it is fine — it asserts the finding is
    still true, so that a change which quietly starts rejecting vague cards is
    noticed and written up rather than absorbed.
    """
    candidate = generate("b", "navigate_to_goal", replay, card=vague_card,
                         staging=staging, record=False)
    assert candidate.prompt_hash == VAGUE_PROMPT_HASH
    result = admit(candidate.metadata(), candidate.source, card=vague_card)
    assert result.admitted, (
        "the vague card is now REJECTED. That is a real improvement, and "
        "generate/FINDINGS.md §1 plus chunk 17's card-fidelity decision must be "
        "rewritten rather than left saying the opposite.")


def test_the_vague_card_node_does_not_implement_the_vague_card(vague_card,
                                                               replay, staging):
    """The mechanism of §1: the node's declared EFFECT describes machinery the
    card never mentions. This is what "the harness tests the node, not the
    card" means, made concrete."""
    candidate = generate("b", "navigate_to_goal", replay, card=vague_card,
                         staging=staging, record=False)
    node_cls, _, _, _ = __import__(
        "generate.admit", fromlist=["load_candidate"]).load_candidate(
            candidate.source, "vague_check")

    # Read off the node's OUT ports rather than off a list of words the last
    # generation happened to use. The ports ARE the composition-visible surface
    # (#6: composition sees port contracts and a task spec, never the papers),
    # so an invented port is the strongest form of the finding: the machinery
    # the card did not supply is the machinery the composer gets to wire up.
    # A word list needs retuning every time the node is re-recorded; this does
    # not, and it says something sharper.
    card_text = " ".join((vague_card.mechanism, vague_card.trigger,
                          vague_card.derivation,
                          vague_card.prediction.statement)).lower()
    out_ports = [p.name for p in node_cls.PORTS if p.direction is Direction.OUT]
    assert out_ports, "expected the node to publish something"
    invented = [name for name in out_ports
                if not any(tok in card_text for tok in name.split("_"))]
    assert invented, (
        f"every published port {out_ports} is named on the card. The node is "
        f"faithful to it — which is a real improvement, and generate/FINDINGS.md "
        f"§1 plus chunk 17's card-fidelity decision must be rewritten rather "
        f"than left saying the opposite.")


def test_a_card_free_condition_records_no_card(replay, staging):
    """A and D produce nodes with `card_id` None by construction. G1 needs
    'invented' and 'nobody recorded it' to be different values, and this is the
    half of that which lives in the generator."""
    from generate.generator import Candidate
    candidate = Candidate("a", "navigate_to_goal", "x = 1", "prompt",
                          backends.Response("t", "m", "replay",
                                            sampling_note="n"))
    assert candidate.metadata()["card_id"] is None
    assert candidate.condition == "a"
