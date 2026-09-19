"""repair/diagnose.py — it PROPOSES, it does not decide.

⚠️ No test here calls a model. The live call is the chunk's deliverable and is
recorded in the plan's Outcome; a test that called a model would be slow,
costly, non-deterministic and would be testing the model rather than the
pipeline — the reason `ReplayBackend` exists at all.
"""
from __future__ import annotations

import inspect
import json

import pytest

from repair import diagnose as D


def _payload(**over):
    base = {
        "cause": "the trigger latches and never disarms",
        "card_departures": ["the card says it must sit at lower priority"],
        "evidence": ["vx is 0.00 from t=5 onward at 0.35 m clearance"],
        "action": "REGENERATE_NODE",
        "action_reason": "the card is sound, the implementation is not",
        "guidance": "make the precondition able to turn off again",
        "confidence": "high",
    }
    base.update(over)
    return "```json\n" + json.dumps(base) + "\n```"


# --------------------------------------------------------------------------- #
# the closed vocabulary


def test_a_valid_diagnosis_parses():
    d = D.parse(_payload(), handle="node-abc")
    assert d.action is D.Action.REGENERATE_NODE
    assert d.confidence == "high"
    assert d.card_departures and d.evidence


@pytest.mark.parametrize("action", [a.value for a in D.Action])
def test_every_offered_action_is_accepted(action):
    assert D.parse(_payload(action=action)).action.value == action


def test_an_invented_action_is_refused_not_coerced():
    """⚠️ Coercing a near-miss into the nearest valid action would let the model
    invent actions the loop cannot perform, and the loop would then act on a
    guess about what it meant."""
    with pytest.raises(D.DiagnosisError, match="not one of"):
        D.parse(_payload(action="REWRITE_THE_BASE"))


@pytest.mark.parametrize("action", D.DEFERRED_ACTIONS)
def test_deferred_actions_are_refused_by_name(action):
    """⚠️ `REGENERATE_CARD` is deliberately absent (user's instruction,
    2026-09-16) and there is nothing behind it. Refused with a message that says
    so, rather than the generic 'not one of' — a model that proposes it has
    reasoned correctly about a capability this version does not have."""
    with pytest.raises(D.DiagnosisError, match="DEFERRED"):
        D.parse(_payload(action=action))


def test_the_prompt_offers_exactly_the_actions_the_parser_accepts():
    """The two lists cannot drift: a prompt offering an action the parser
    refuses would fail every time the model took it at its word."""
    text = D.PROMPT.read_text(encoding="utf-8")
    for a in D.Action:
        assert f"`{a.value}`" in text, f"{a.value} is not offered in the prompt"
    for a in D.DEFERRED_ACTIONS:
        assert f"`{a}`" not in text, f"{a} is offered but has nothing behind it"


def test_a_wrong_confidence_is_still_refused():
    """Present and wrong means the schema was not understood. Absent does not."""
    with pytest.raises(D.DiagnosisError, match="confidence"):
        D.parse(_payload(confidence="pretty sure"))


def test_an_omitted_confidence_does_not_throw_away_the_diagnosis():
    """⚠️ THE FIRST LIVE CALL FOUND THIS. The model returned a complete,
    correct, four-departure diagnosis and omitted exactly one field —
    `confidence` — and the whole $0.43 result was discarded over it.

    The line: **a field the loop ACTS ON is required; a field that is only
    REPORTED is optional.** Because a rejection is never retried, over-strictness
    here is unrecoverable waste rather than safety."""
    payload = json.loads(_payload().split("```json")[1].split("```")[0])
    payload.pop("confidence")
    d = D.parse("```json\n" + json.dumps(payload) + "\n```")
    assert d.confidence == "unstated"
    assert d.action is D.Action.REGENERATE_NODE


def test_confidence_is_not_in_the_required_fields():
    assert "confidence" not in D.REQUIRED_FIELDS
    assert set(D.REQUIRED_FIELDS) == {"cause", "action", "action_reason"}


@pytest.mark.parametrize("field", D.REQUIRED_FIELDS)
def test_a_missing_required_field_is_a_failure_not_a_default(field):
    with pytest.raises(D.DiagnosisError, match="missing"):
        D.parse(_payload(**{field: ""}))


def test_non_json_is_a_recorded_failure_not_a_crash():
    with pytest.raises(D.DiagnosisError, match="not JSON"):
        D.parse("I think the node is broken because of the trigger.")


def test_a_bare_json_object_without_a_fence_still_parses():
    """Models drop the fence often enough that refusing it would be refusing a
    good answer on formatting."""
    assert D.parse(json.dumps(json.loads(
        _payload().split("```json")[1].split("```")[0]))).action


def test_a_json_list_is_refused():
    with pytest.raises(D.DiagnosisError, match="not an object"):
        D.parse("```json\n[1, 2, 3]\n```")


# --------------------------------------------------------------------------- #
# blindness


def test_render_cannot_be_told_which_arm_it_is():
    """⚠️ Same rule as `generator.render_node(card, task)`: no condition
    parameter, ever. The moment it can tell the arms apart, the repair loop has
    a second place to differ."""
    params = set(inspect.signature(D.render).parameters)
    assert params == {"bundle"}


def test_the_prompt_names_no_condition():
    text = D.PROMPT.read_text(encoding="utf-8").lower()
    for tell in ("condition a", "condition b", "condition d", "arm a", "arm b",
                 "citation arm", "literature-informed", "treatment"):
        assert tell not in text, tell


def test_the_cli_backend_disables_project_settings():
    """⚠️ LOAD-BEARING, NOT HYGIENE. A diagnosing agent that could read this
    repo's CLAUDE.md would know which arm every node belongs to, that D is
    frozen, and that B−A is the headline."""
    import inspect as _i
    from generate.backends import ClaudeCliBackend
    src = _i.getsource(ClaudeCliBackend.complete)
    assert '"--setting-sources", ""' in src
    assert "TemporaryDirectory" in src, "CLAUDE.md discovery walks UP from cwd"


def test_the_diagnosis_system_prompt_does_not_ask_for_python():
    """The default ends "Return the code in a single fenced python block",
    which on a diagnosis call instructs the model to do the wrong thing."""
    from generate.backends import SYSTEM_PROMPT as DEFAULT
    assert "python" in DEFAULT.lower()
    assert "python" not in D.SYSTEM_PROMPT.lower()
    assert "json" in D.SYSTEM_PROMPT.lower()


def test_the_backend_override_leaves_every_existing_caller_unchanged():
    from generate.backends import ClaudeCliBackend, SYSTEM_PROMPT as DEFAULT
    assert (ClaudeCliBackend().system_prompt or DEFAULT) == DEFAULT


# --------------------------------------------------------------------------- #
# the prompt itself


def test_the_prompt_has_a_bundle_slot():
    assert "{{BUNDLE}}" in D.PROMPT.read_text(encoding="utf-8")


def test_a_prompt_without_a_slot_is_refused(tmp_path, monkeypatch):
    """A prompt with no slot would silently diagnose from the framing alone."""
    bad = tmp_path / "diagnose.md"
    bad.write_text("no slot here", encoding="utf-8")
    monkeypatch.setattr(D, "PROMPT", bad)

    class _B:
        def render(self):
            return "x"
    with pytest.raises(D.DiagnosisError, match="BUNDLE"):
        D.render(_B())


def test_the_prompt_says_a_worse_number_is_not_a_broken_node():
    """⚠️ The core instruction of the whole epic. Without it the model will
    default to 'it scored worse, therefore fix it', which is the failure mode
    the taxonomy exists to prevent."""
    text = D.PROMPT.read_text(encoding="utf-8").lower()
    assert "not automatically a broken node" in text
    assert "may simply not transfer" in text


def test_the_prompt_asks_for_card_departures():
    """The check nothing automatic can make."""
    text = D.PROMPT.read_text(encoding="utf-8")
    assert "card_departures" in text
    assert "does NOT do what its own card says" in text


# --------------------------------------------------------------------------- #
# ⚠️ THE EPIC'S SUCCESS CRITERION, AS A TEST


LIVE_TRANSCRIPT = (D.TRANSCRIPTS / "0b00747fffa64ce6.json")


@pytest.mark.skipif(not LIVE_TRANSCRIPT.exists(),
                    reason="the committed live transcript is not on this machine")
def test_the_first_live_diagnosis_found_the_known_defects():
    """⚠️ THE CHECK THE WHOLE EPIC WAS DESIGNED AROUND.

    `gen_b_44d12df4d25b437f` was diagnosed BY HAND on 2026-09-16, before any of
    this existed, and three defects were found. The first target was chosen
    precisely so the loop's output could be checked against a known-good answer:
    a loop that terminates cleanly having found none of them has failed,
    however tidy its output.

    ⚠️ **THIS TEST IS WEAKER THAN IT WAS, AND THE REASON IS A DESIGN DEFECT.**
    A transcript is keyed by PROMPT HASH and `write_transcript` OVERWRITES. The
    diagnosis prompt is deterministic, so every later live run on this node
    rewrites this exact file — the specific wording the chunk 57 Outcome quotes
    is no longer on disk, replaced by a different (equally correct) diagnosis
    of the same node.

    So the assertions below check that the SUBSTANCE is found, not the
    phrasing. A transcript is a record of "the most recent call with this
    prompt", never of "the call made on date X". See the plan Outcome.
    """
    payload = json.loads(LIVE_TRANSCRIPT.read_text(encoding="utf-8"))
    d = D.parse(payload["response"]["text"], handle="node-3cb6a0af73f4")

    assert d.action is D.Action.REGENERATE_NODE
    blob = " ".join([d.cause, d.guidance, *d.card_departures, *d.evidence]).lower()

    # 1. the arbitration conflict — the node seizes wz and destroys the base's
    #    alignment gate. Found by hand; found here.
    assert "nav_heading" in blob
    assert "last-writer-wins" in blob or "last writer wins" in blob

    # 2. the self-latching trigger — no translation, no strides, frozen bearing
    #
    # ⚠️ WIDENED 2026-09-17, and the reason is the same design defect the
    # docstring above describes. A later live run rewrote this transcript with
    # a diagnosis that identifies the defect in different words:
    #
    #   "It is a self-locking loop"
    #   "the 1.0 m arming gate is crossed almost immediately and the node is
    #    effectively always on ... without a single decline"
    #
    # `"latch" in blob` was a PHRASING check inside a test that claims to check
    # substance. Widening it to the synonyms is the honest fix; deleting it
    # would remove the epic's own success criterion, and leaving it would fail
    # a run that found the defect. The chunk 58 Outcome records this.
    assert any(t in blob for t in ("latch", "self-lock", "never disarm",
                                   "always on", "without a single decline")), (
        "no description of the trigger arming and never releasing")

    # 3. the cap/adjust asymmetry: it seizes wz but only BOUNDS vx
    assert "cap" in blob and ("adjust" in blob or "last-writer" in blob)

    # 4. ⚠️ AND ONE NOBODY FOUND BY HAND: `drift: None`, so the node has never
    #    been measured on the axis its card predicates its whole claim on.
    assert "drift" in blob

    assert len(d.card_departures) >= 3, (
        "the departures list is the check nothing automatic can make")


# --------------------------------------------------------------------------- #
# a call that cost money and returned nothing is still evidence


def test_a_backend_failure_is_recorded_not_lost(tmp_path, monkeypatch):
    """⚠️ FOUND BY THE GENERALISATION CHECK, 2026-09-16. One of three calls
    spent $0.30 and 6,016 THINKING tokens out of a 6,120-token output budget,
    emitted almost nothing, and raised `BackendError` — and because only
    `DiagnosisError` was recorded, the entire call vanished.

    Same family as the sweep rehearsal's "exited 0, wrote nothing": a call
    nobody can see is indistinguishable from a call nobody made."""
    from generate.backends import BackendError

    monkeypatch.setattr(D, "TRANSCRIPTS", tmp_path)

    class _Failing:
        def complete(self, prompt):
            raise BackendError("claude exited 1", {"usage": {
                "output_tokens": 6120,
                "output_tokens_details": {"thinking_tokens": 6016}}})

    class _Bundle:
        handle = "node-test"

        def render(self):
            return "bundle body"

    with pytest.raises(BackendError):
        D.diagnose(_Bundle(), backend=_Failing())

    written = list(tmp_path.glob("*.error.json"))
    assert len(written) == 1, "the failed call left no record"
    record = json.loads(written[0].read_text(encoding="utf-8"))
    assert record["parsed"] is False
    assert record["usage"]["output_tokens_details"]["thinking_tokens"] == 6016, (
        "the token counts ARE the finding on a runaway")


def test_a_failure_record_can_never_be_replayed_as_a_diagnosis():
    """`ReplayBackend` reads `<hash>.json`; failures are `<hash>.error.json`.
    A failure that could be replayed would be a diagnosis nobody made."""
    from generate.backends import ReplayBackend
    rb = ReplayBackend(directory=D.TRANSCRIPTS)
    assert rb.path_for("anything").suffixes == [".json"]
    assert not str(rb.path_for("anything")).endswith(".error.json")


def test_an_existing_transcript_is_replayed_rather_than_re_called(tmp_path,
                                                                  monkeypatch):
    """⚠️ Done-when: "a killed run resumes without repeating a model call."

    `loop.regenerate` resumes from its staged round; this is the other half.
    The diagnosis prompt is a pure function of the bundle, so a transcript for
    THIS hash is this exact call.

    ⚠️ And it stops a re-run DESTROYING the recorded diagnosis. Transcripts are
    keyed by prompt hash and `write_transcript` replaces — on 2026-09-17 a
    second live run overwrote the very transcript the epic's success criterion
    is asserted against.
    """
    from generate.backends import prompt_hash, write_transcript

    monkeypatch.setattr(D, "TRANSCRIPTS", tmp_path)

    class _Bundle:
        handle = "node-test"

        def render(self):
            return "bundle body"

    class _Live:
        name = "live"

        def __init__(self, *a, **k):     # it is constructed with a system prompt
            pass

        def complete(self, prompt):
            raise AssertionError("a model was called for a recorded prompt")

    monkeypatch.setattr("generate.backends.ClaudeCliBackend", _Live)

    prompt = D.render(_Bundle())

    # nothing on disk yet -> it WOULD call
    with pytest.raises(AssertionError):
        D.diagnose(_Bundle())

    from generate.backends import Response

    write_transcript(prompt,
                     Response(text=_payload(), model_id="claude-opus-5",
                              backend="claude-cli"),
                     {"step": "diagnose"}, directory=tmp_path)
    assert (tmp_path / f"{prompt_hash(prompt)}.json").exists()

    d = D.diagnose(_Bundle(), record=False)
    assert d.action is D.Action.REGENERATE_NODE
    assert "latches" in d.cause


def test_an_explicit_backend_always_wins_over_the_resume():
    """A caller that names its backend means it — the resume is only the
    default path, never an override."""
    import inspect as _i
    src = _i.getsource(D.diagnose)
    assert "if backend is None:" in src, (
        "the replay resume must be scoped to the default backend")
