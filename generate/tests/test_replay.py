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
from pathlib import Path

import pytest

from generate import backends
from generate.admit import admit
from generate.backends import (TEMPERATURE_MODELS, BackendError, ReplayBackend,
                               prompt_hash)
from generate.generator import (Candidate, extract_code, generate, render_v1,
                                task_text)
from nodes.contract import Direction

from generate.tests.conftest import GOOD_PROMPT_HASH, VAGUE_PROMPT_HASH

FIXTURE_TRANSCRIPTS = (Path(__file__).resolve().parent / "fixtures"
                       / "transcripts")


def _v1_candidate(condition, task, card, replay):
    """One candidate the way the SUPERSEDED one-call pipeline built it.

    Chunk 42 made `generate()` a two-call function, so it can no longer replay a
    v1 transcript — but the v1 transcripts are real records of real model calls
    and the tests below are about them. `render_v1` is frozen for exactly this,
    and this helper is the rest of what v1's `generate()` did: render, replay,
    take the fenced block, stage nothing.
    """
    prompt = render_v1(condition, task, card)
    response = replay.complete(prompt)
    return Candidate(condition, task, extract_code(response.text), prompt,
                     response,
                     card_id=card.id if condition == "b" else None,
                     citation_id=card.id if condition == "a" else None,
                     key=prompt_hash(prompt))


def test_the_committed_transcript_matches_the_prompt_it_was_taken_against(
        pilot_card, replay):
    """The hash is over the RENDERED prompt, so this fails the moment a prompt
    file is edited. That is the intended behaviour and it is worth stating:
    editing a prompt orphans every transcript taken against it, because the
    prompt is the independent variable and a changed one is a changed
    experiment."""
    prompt = render_v1("b", "navigate_to_goal", pilot_card)
    assert prompt_hash(prompt) == GOOD_PROMPT_HASH, (
        "generate/prompts/ has changed since the committed transcript was "
        "recorded. Re-record it with scripts/generate_node.py, and note in the "
        "chunk Outcome that the recorded generation is a different one.")
    assert replay.path_for(prompt).exists()


def test_replaying_is_byte_identical(pilot_card, replay, staging, tmp_path):
    """Twice through the replay backend, same bytes both times. This is the
    reproducibility property the project actually has."""
    first = _v1_candidate("b", "navigate_to_goal", pilot_card, replay)
    second = _v1_candidate("b", "navigate_to_goal", pilot_card, replay)
    assert first.source == second.source
    assert first.prompt_hash == second.prompt_hash == GOOD_PROMPT_HASH
    assert first.response.model_id == "claude-opus-5"


def test_the_replayed_candidate_is_the_one_that_was_admitted(pilot_card, replay,
                                                             staging):
    """End to end with no model: transcript -> candidate -> harness ->
    ADMITTED, reproducing the committed result in generate/staging/."""
    candidate = _v1_candidate("b", "navigate_to_goal", pilot_card, replay)
    result = admit(candidate.metadata(), candidate.source, card=pilot_card)
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert result.unit.metric == pilot_card.prediction.metric
    assert result.unit.effect >= result.unit.required

    from generate.generator import V1_STAGING
    committed = json.loads(
        (V1_STAGING / f"b__limit_speed_dwa__{GOOD_PROMPT_HASH}.result.json")
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
    prompt = render_v1("b", "navigate_to_goal", pilot_card)
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
    prompt = render_v1("b", "navigate_to_goal", pilot_card)
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
    candidate = _v1_candidate("b", "navigate_to_goal", vague_card, replay)
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
    candidate = _v1_candidate("b", "navigate_to_goal", vague_card, replay)
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


def test_the_staged_files_and_the_verdict_share_one_stem(tmp_path, pilot_card):
    """`Candidate.write()` and `Candidate.stem` must not compute the stem twice.

    They used to, and the copies disagreed as soon as a condition carried a
    `citation_id` instead of a `card_id`: the module was staged under the TASK
    and the verdict under the CARD. `batch.plan()` tests for the `.json` to
    decide whether a row is already staged, so a split pair reads as PENDING
    while a verdict for it already exists — and the row gets generated again,
    which is precisely the retry that leaves G1's pass rate without a
    denominator.
    """
    from generate.generator import Candidate

    class _R:
        text, model_id, backend = "x", "m", "replay"
        temperature, sampling_note = None, "n/a"

    for kwargs in ({"card_id": pilot_card.id},
                   {"citation_id": pilot_card.id},
                   {}):
        cand = Candidate("a", "navigate_to_goal", "# code", "prompt", _R(),
                         **kwargs)
        module = cand.write(tmp_path)
        assert module.stem == cand.stem, (kwargs, module.stem, cand.stem)
        assert (tmp_path / f"{cand.stem}.json").exists(), kwargs


# --- chunk 42: the two-call chain ------------------------------------------

@pytest.fixture
def chain():
    """`ReplayBackend` over the hand-built card+node fixture pair. See
    generate/tests/fixtures/transcripts/README.md — no model was called."""
    return ReplayBackend(FIXTURE_TRANSCRIPTS)


def test_the_card_and_node_calls_replay_as_one_chain(chain, tmp_path):
    """**The property chunk 42 has to keep.** Replaying the card call
    reproduces the card, which renders the node prompt, which replays the node
    call — so a two-call pipeline is still regenerable from committed files with
    no model, no key and no network."""
    candidate = generate("d", "navigate_to_goal", chain,
                         staging=tmp_path / "staging",
                         cards_dir=tmp_path / "cards",
                         transcripts=FIXTURE_TRANSCRIPTS, record=False)
    assert candidate.generated is not None
    assert candidate.generated.card.node_name == "LimitSpeedToVisibleRange"
    assert candidate.source and "class LimitSpeedToVisibleRange" in candidate.source
    # The stem is the CARD prompt's row key, and the node id derives from it.
    assert candidate.node_id == f"gen_d_{candidate.key}"
    assert candidate.stem.endswith(candidate.key)


def test_the_chain_is_byte_identical_twice(chain, tmp_path):
    first = generate("d", "navigate_to_goal", chain,
                     staging=tmp_path / "s1", cards_dir=tmp_path / "c1",
                     transcripts=FIXTURE_TRANSCRIPTS, record=False)
    second = generate("d", "navigate_to_goal", chain,
                      staging=tmp_path / "s2", cards_dir=tmp_path / "c2",
                      transcripts=FIXTURE_TRANSCRIPTS, record=False)
    assert first.source == second.source
    assert first.key == second.key
    assert first.prompt_hash == second.prompt_hash
    assert (tmp_path / "c1" / "d" / f"{first.stem}.yaml").read_bytes() == \
        (tmp_path / "c2" / "d" / f"{second.stem}.yaml").read_bytes()


def test_the_generated_card_is_committed_beside_its_candidate(chain, tmp_path):
    """The card file and the staged candidate share a stem, which is what lets
    `admit_staged` find the card a node was judged against."""
    candidate = generate("d", "navigate_to_goal", chain,
                         staging=tmp_path / "staging",
                         cards_dir=tmp_path / "cards",
                         transcripts=FIXTURE_TRANSCRIPTS, record=False)
    path = tmp_path / "cards" / "d" / f"{candidate.stem}.yaml"
    assert path.exists()
    assert (tmp_path / "staging" / f"{candidate.stem}.py").exists()
    assert (tmp_path / "staging" / f"{candidate.stem}.json").exists()

    text = path.read_text(encoding="utf-8")
    assert "MODEL-WRITTEN CARD — NOT A PRINCIPLE CARD" in text, (
        "the header is what tells anyone opening this file that nobody "
        "extracted it from anything")

    from generate.admit import load_generated_card
    meta = json.loads(
        (tmp_path / "staging" / f"{candidate.stem}.json").read_text("utf-8"))
    reloaded = load_generated_card(candidate.stem, meta,
                                   directory=tmp_path / "cards")
    assert reloaded.id == candidate.generated.card.id
    assert reloaded.prediction.metric == candidate.generated.card.prediction.metric


def test_the_replayed_chain_admits(chain, tmp_path):
    """End to end with no model: two transcripts -> card -> node -> harness ->
    ADMITTED, with the node scored against the card the model wrote for it."""
    candidate = generate("d", "navigate_to_goal", chain,
                         staging=tmp_path / "staging",
                         cards_dir=tmp_path / "cards",
                         transcripts=FIXTURE_TRANSCRIPTS, record=False)
    result = admit(candidate.metadata(), candidate.source,
                   generated_card=candidate.generated.card)
    assert result.admitted, f"{result.reason}: {result.detail}"
    assert result.metadata["origin"] == "invented"
    assert result.metadata["generated_card_id"] == "fixture_speed_limit"
    # The card is the model's own, so its span is a claim and never provenance.
    assert result.metadata["span_verified"] is False
    assert result.metadata["is_traceable"] is False


def test_a_record_carries_the_pipeline_that_made_it(chain, tmp_path):
    """`pass_rate` refuses to pool v1 and v2 records, and this is the field it
    reads. A record with no version is v1 by construction — every one of them
    predates the field."""
    from generate.generator import PIPELINE_VERSION
    candidate = generate("d", "navigate_to_goal", chain,
                         staging=tmp_path / "staging",
                         cards_dir=tmp_path / "cards",
                         transcripts=FIXTURE_TRANSCRIPTS, record=False)
    assert candidate.metadata()["pipeline_version"] == PIPELINE_VERSION

    from generate.generator import V1_STAGING
    committed = json.loads(
        (V1_STAGING / f"b__limit_speed_dwa__{GOOD_PROMPT_HASH}.json")
        .read_text("utf-8"))
    assert "pipeline_version" not in committed


def test_the_paper_never_reaches_a_committed_transcript(tmp_path, pilot_card):
    """⚠️ **out/corpus/ is gitignored and must stay that way** — the PNAS copy
    carries the downloader's IP address. B's card prompt is a whole paper, and
    `write_transcript` stores prompts in full, so without redaction committing
    transcripts would commit the corpus through the back door.

    What must survive redaction is the HASH: it is over the real prompt, so a
    machine holding the PDF re-renders it byte-identically and replay still
    finds the file.
    """
    from generate import cardgen, corpus

    marker = "UNMISTAKABLE-PAPER-BODY-TEXT-0451"
    paper = corpus.Paper(card_id=pilot_card.id, text=marker * 40,
                         sha256="0" * 64, extractor="stub",
                         char_count=len(marker) * 40, truncated=False,
                         path=Path("/nowhere.pdf"))

    class _Backend:
        def complete(self, prompt):
            return backends.Response("no card here", "claude-opus-5", "stub",
                                     sampling_note="stub")

        def describe(self):
            return {"backend": "stub"}

    real_load, corpus_module = corpus.load, cardgen.corpus
    corpus_module.load = lambda card_id, **kw: paper
    try:
        cardgen.write_card("b", "navigate_to_goal", task_text("navigate_to_goal"),
                           _Backend(), card=pilot_card, key="k",
                           directory=tmp_path / "cards",
                           transcripts=tmp_path / "transcripts", record=True)
    finally:
        corpus_module.load = real_load

    written = list((tmp_path / "transcripts").glob("*.json"))
    assert len(written) == 1
    data = json.loads(written[0].read_text(encoding="utf-8"))
    assert marker not in data["prompt"], (
        "the paper text is in the transcript. Committing that commits the "
        "corpus, which out/corpus/ is gitignored to prevent.")
    assert data["prompt_redacted"] is True
    assert "PAPER TEXT REDACTED" in data["prompt"]
    assert paper.sha256[:16] in data["prompt"]

    # The hash is of the REAL prompt, so replay still resolves it.
    real_prompt = cardgen.render("b", task_text("navigate_to_goal"),
                                 card=pilot_card, paper=paper)
    assert data["prompt_hash"] == prompt_hash(real_prompt)
    assert written[0].stem == prompt_hash(real_prompt)


# ---------------------------------------------------------------------------
# ResumeBackend — finishing a row whose first call already succeeded.
# ---------------------------------------------------------------------------

class _RefusingBackend(backends.Backend):
    """A live backend that must never be reached. Stands in for the model."""

    name = "refusing"

    def __init__(self):
        self.calls = []

    def complete(self, prompt):
        self.calls.append(prompt)
        raise AssertionError(
            "the live backend was called for a prompt that already has a "
            "transcript — resuming would have re-made a recorded call")

    def describe(self):
        return {"backend": "claude-cli", "model_id": backends.DEFAULT_MODEL}


class _OneShotBackend(_RefusingBackend):
    """A live backend that answers once, from a canned response."""

    def __init__(self, response):
        super().__init__()
        self.response = response

    def complete(self, prompt):
        self.calls.append(prompt)
        return self.response


def test_resume_replays_a_call_that_already_has_a_transcript(chain, tmp_path):
    """The reason the backend exists: condition D's card call succeeded and its
    node call timed out (2026-09-09). Finishing the row must not re-make the
    card call, because the second card would be a DIFFERENT card written over
    the first under the same deterministic stem."""
    live = _RefusingBackend()
    resume = backends.ResumeBackend(live=live, directory=FIXTURE_TRANSCRIPTS)

    candidate = generate("d", "navigate_to_goal", resume,
                         staging=tmp_path / "staging",
                         cards_dir=tmp_path / "cards",
                         transcripts=FIXTURE_TRANSCRIPTS, record=False)

    assert live.calls == [], "both calls were on record; neither should be live"
    assert [what for what, _ in resume.log] == ["replayed", "replayed"]
    # And the result is the replayed chain's result, not a degraded one.
    assert candidate.generated.card.node_name == "LimitSpeedToVisibleRange"
    assert "class LimitSpeedToVisibleRange" in candidate.source


def test_resume_makes_only_the_call_that_is_missing(chain, tmp_path):
    """The half-finished row: the card transcript is present, the node
    transcript is not. Exactly one live call, and it is the node call."""
    # A transcript directory holding the CARD call only.
    partial = tmp_path / "transcripts"
    partial.mkdir()
    card_hash = None
    for path in FIXTURE_TRANSCRIPTS.glob("*.json"):
        meta = json.loads(path.read_text(encoding="utf-8")).get("meta", {})
        if meta.get("kind") == "card":
            (partial / path.name).write_bytes(path.read_bytes())
            card_hash = path.stem
    assert card_hash, "the fixture pair should carry a card call"

    # The node response the live backend will stand in for.
    node_text = None
    for path in FIXTURE_TRANSCRIPTS.glob("*.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        if data.get("meta", {}).get("kind") != "card":
            node_text = data["response"]["text"]
    assert node_text

    live = _OneShotBackend(backends.Response(
        text=node_text, model_id=backends.DEFAULT_MODEL, backend="claude-cli",
        temperature=None, sampling_note="fixture"))
    resume = backends.ResumeBackend(live=live, directory=partial)

    candidate = generate("d", "navigate_to_goal", resume,
                         staging=tmp_path / "staging",
                         cards_dir=tmp_path / "cards",
                         transcripts=partial, record=False)

    assert len(live.calls) == 1, "only the missing call should reach the model"
    assert [what for what, _ in resume.log] == ["replayed", "called"]
    assert "class LimitSpeedToVisibleRange" in candidate.source


def test_resume_describes_itself_as_the_live_backend(chain):
    """`generate()` re-records every call it makes, so a replayed call rewrites
    its own transcript. If `describe()` said "resume", the meta of a committed
    transcript would change as a side effect of resuming, and would then
    describe the session that re-read the call rather than the one that made
    it. Where a response came from is in `response.backend`, which replay
    round-trips untouched."""
    resume = backends.ResumeBackend(live=_RefusingBackend(),
                                    directory=FIXTURE_TRANSCRIPTS)
    assert resume.describe() == {"backend": "claude-cli",
                                 "model_id": backends.DEFAULT_MODEL}


def test_resume_is_reachable_by_name():
    made = backends.make("resume", directory=FIXTURE_TRANSCRIPTS)
    assert isinstance(made, backends.ResumeBackend)
    assert isinstance(made.live, backends.ClaudeCliBackend)


# ---------------------------------------------------------------------------
# A node call that produces nothing usable still has to leave a record.
# ---------------------------------------------------------------------------

class _FailingNodeBackend(backends.Backend):
    """Replays the card call, then fails the node call the way the CLI does."""

    name = "failing"

    def __init__(self, directory, payload):
        self.replay = ReplayBackend(directory)
        self.payload = payload
        self.calls = 0

    def complete(self, prompt):
        self.calls += 1
        if self.calls == 1:
            return self.replay.complete(prompt)
        raise BackendError("claude exited 1", payload=self.payload)

    def describe(self):
        return {"backend": "claude-cli", "model_id": backends.DEFAULT_MODEL}


PAYLOAD = {
    "stop_reason": "stop_sequence",
    "duration_api_ms": 1781693,
    "total_cost_usd": 3.7068440000000002,
    "usage": {"output_tokens": 136423,
              "output_tokens_details": {"thinking_tokens": 134242}},
}


def test_a_failed_node_call_is_recorded_against_its_row(tmp_path):
    """2026-09-09: D's node call burned 136k output tokens, returned no code,
    and left NOTHING behind — indistinguishable from a row nobody ran."""
    from generate.generator import generate as gen
    backend = _FailingNodeBackend(FIXTURE_TRANSCRIPTS, PAYLOAD)
    staging = tmp_path / "staging"

    with pytest.raises(BackendError):
        gen("d", "navigate_to_goal", backend, staging=staging,
            cards_dir=tmp_path / "cards", transcripts=FIXTURE_TRANSCRIPTS,
            record=True)

    errors = list(staging.glob("*.error.json"))
    assert len(errors) == 1, "the failed call should leave exactly one record"
    rec = json.loads(errors[0].read_text(encoding="utf-8"))
    assert rec["condition"] == "d"
    assert rec["kind"] == "node"
    # The token counts ARE the finding; losing them loses the reason.
    assert rec["usage"]["output_tokens"] == 136423
    assert rec["usage"]["output_tokens_details"]["thinking_tokens"] == 134242
    assert rec["total_cost_usd"] == PAYLOAD["total_cost_usd"]
    assert rec["pipeline_version"] == "v2-card-first"


def test_an_error_record_is_not_a_staged_candidate(tmp_path):
    """The row must stay PENDING. An error record is evidence, not a verdict:
    whether a runaway call belongs in G1's denominator is a pre-registration
    question, and writing it as a refusal would answer it silently."""
    from generate.generator import generate as gen
    from generate.batch import plan
    backend = _FailingNodeBackend(FIXTURE_TRANSCRIPTS, PAYLOAD)
    staging = tmp_path / "staging"

    with pytest.raises(BackendError):
        gen("d", "navigate_to_goal", backend, staging=staging,
            cards_dir=tmp_path / "cards", transcripts=FIXTURE_TRANSCRIPTS,
            record=True)

    stem = json.loads(next(staging.glob("*.error.json"))
                      .read_text(encoding="utf-8"))["stem"]
    assert not (staging / f"{stem}.py").exists()
    assert not (staging / f"{stem}.json").exists()
    assert not (staging / f"{stem}.refusal.md").exists()

    rows = plan(staging=staging)
    row = [r for r in rows if r["stem"] == stem]
    assert row and row[0]["staged"] is False, (
        "an error record must not make the row look already generated")
