"""
generate/tests/test_cardgen.py

The card call — chunk 42's new step, and the only place the conditions differ.

`test_prompts.py` covers the prompts themselves. This covers the machinery
around them: what each arm is handed, what happens to a response that is not a
card, where a written card lands, and the two guards that stop a model-written
card being mistaken for an extracted one.

**No test here calls a model.** Every response is a string a test wrote.
"""

import json
from pathlib import Path

import pytest

from cards.store import CardError, CardStore
from generate import backends, cardgen, corpus
from generate.cardgen import (CardGenerationError, GeneratedCard, extract_card,
                              treatment, write_card)
from generate.generator import PIPELINE_VERSION, task_text

CARD_YAML = """```yaml
id: fixture_card
node_name: SlowNearOcclusions
source:
  citation: "%s"
  span: ''
mechanism: Slow down where the scan is short.
trigger: When the forward free distance is below the stopping distance.
derivation: v <= sqrt(2 * d * a_brake), with a_brake ours rather than anyone's.
prediction:
  statement: Fewer collisions than reactive_avoid.
  metric: collisions
  baseline: reactive_avoid
  min_effect: 0.5
  cost_metric: time_to_goal
confidence: low
```"""


class _Backend:
    """Returns whatever the test told it to. Records the prompt it saw."""

    def __init__(self, text):
        self.text, self.prompts = text, []

    def complete(self, prompt):
        self.prompts.append(prompt)
        return backends.Response(self.text, "claude-opus-5", "stub",
                                 sampling_note="stub backend")

    def describe(self):
        return {"backend": "stub"}


def _yaml(citation=None):
    # json.dumps, not an f-string: the pilot citations contain quotes and
    # commas, and a naively quoted YAML scalar is a parser error. A
    # double-quoted JSON string is a valid YAML double-quoted scalar.
    return CARD_YAML % json.dumps(citation or cardgen.NO_SOURCE)[1:-1]


# --- who is handed what ----------------------------------------------------

def test_each_arm_has_exactly_one_treatment():
    assert treatment("a") == "citation"
    assert treatment("b") == "paper"
    assert treatment("d") == "none"


def test_the_treatments_partition_the_conditions():
    """A condition in two treatment lists would be handed two things, and the
    arm it was supposed to be would not exist."""
    from generate.generator import CONDITIONS
    assert not set(cardgen.CITATION_CONDITIONS) & set(cardgen.PAPER_CONDITIONS)
    assert set(cardgen.CORPUS_CONDITIONS) == (
        set(cardgen.CITATION_CONDITIONS) | set(cardgen.PAPER_CONDITIONS))
    assert set(cardgen.CORPUS_CONDITIONS) <= set(CONDITIONS)


# --- parsing the response --------------------------------------------------

def test_a_card_is_parsed_by_the_real_loader(pilot_card):
    card, substituted = extract_card(_yaml(), "d")
    assert card.node_name == "SlowNearOcclusions"
    assert card.prediction.metric == "collisions"
    assert not substituted


def test_a_response_with_no_yaml_block_is_a_result_not_an_error():
    """The card prompts invite an honest "I do not know this paper" outside the
    fence. That answer belongs in the denominator, not in the errors."""
    assert extract_card("I do not know this document.", "a") is None


def test_two_yaml_blocks_are_refused_rather_than_picked_between():
    with pytest.raises(CardGenerationError, match="2 fenced yaml blocks"):
        extract_card(_yaml() + "\n" + _yaml(), "d")


def test_a_card_naming_an_unmovable_metric_is_refused_by_the_loader():
    """The real loader, not a laxer one. Both cards the first citations probe
    rejected named `localisation_failures`, which no controller can move, and
    the refusal WAS the result."""
    bad = _yaml().replace("metric: collisions", "metric: localisation_failures")
    with pytest.raises(CardError, match="cannot be moved by a controller"):
        extract_card(bad, "d")


def test_the_no_source_arm_gets_its_citation_filled_in_rather_than_refused():
    """The schema requires a citation — an extraction from nowhere is a
    contradiction — but the arm with no source has none by construction. The
    substitution is recorded rather than silent."""
    blank = _yaml().replace(f'citation: "{cardgen.NO_SOURCE}"', "citation: ''")
    card, substituted = extract_card(blank, "d")
    assert substituted
    assert card.source.citation == cardgen.NO_SOURCE


def test_the_no_source_sentinel_survives_a_yaml_round_trip():
    """It used to contain a colon, so `citation: (no source: ...)` was a scanner
    error and every card the arm was TOLD to write would have been refused for
    a punctuation mistake the prompt asked for."""
    import yaml
    assert ":" not in cardgen.NO_SOURCE
    parsed = yaml.safe_load(f"citation: {cardgen.NO_SOURCE}\n")
    assert parsed["citation"] == cardgen.NO_SOURCE


# --- what gets written -----------------------------------------------------

def test_a_written_card_round_trips(tmp_path, pilot_card):
    backend = _Backend(_yaml(pilot_card.source.citation))
    generated, prompt_hash, _ = write_card(
        "a", "navigate_to_goal", task_text("navigate_to_goal"), backend,
        card=pilot_card, key="rowkey", directory=tmp_path,
        transcripts=tmp_path / "t", record=False)

    path = tmp_path / "a" / f"{generated.stem}.yaml"
    assert path.exists()
    assert generated.stem == f"a__{pilot_card.id}__rowkey"

    back = cardgen.load_generated(path)
    assert back.card.id == generated.card.id
    assert back.condition == "a" and back.treatment == "citation"
    assert back.ref_id == pilot_card.id
    assert back.key == "rowkey"
    assert back.prompt_hash == prompt_hash


def test_a_written_card_is_never_marked_verified(tmp_path, pilot_card):
    """Verification means a human opened the paper. Nobody has, for any file in
    this directory, and the model does not get to say otherwise."""
    claims = _yaml(pilot_card.source.citation).replace(
        "span: ''", "span: 'Section 3.2'").replace(
        "confidence: low", "confidence: high")
    backend = _Backend(claims)
    generated, _, _ = write_card(
        "a", "navigate_to_goal", task_text("navigate_to_goal"), backend,
        card=pilot_card, key="k", directory=tmp_path, record=False)

    data = (tmp_path / "a" / f"{generated.stem}.yaml").read_text("utf-8")
    assert "verified: unverified" in data
    # The CLAIM is kept — it is the confabulation the provenance axis exists to
    # detect, and throwing it away would throw away the finding.
    assert "Section 3.2" in data


def test_a_refusal_is_staged_under_its_own_extension(tmp_path, pilot_card):
    backend = _Backend("I do not know this document.")
    generated, _, _ = write_card(
        "a", "navigate_to_goal", task_text("navigate_to_goal"), backend,
        card=pilot_card, key="k", directory=tmp_path, record=False)
    assert generated is None
    refusals = list((tmp_path / "a").glob("*.refusal.md"))
    assert len(refusals) == 1
    assert "I do not know" in refusals[0].read_text("utf-8")
    assert not list((tmp_path / "a").glob("*.yaml"))


# --- the guards ------------------------------------------------------------

def test_the_card_store_refuses_the_generated_directory():
    """Same refusal `cards/probe/` gets, for the same reason: a model's own card
    entering the corpus would put its claims into the number G1 reports ABOUT
    the corpus."""
    for path in (cardgen.CARDS_DIR, cardgen.CARDS_DIR / "b"):
        with pytest.raises(CardError, match="GENERATED"):
            CardStore.load(path)


def test_the_real_corpus_still_loads():
    """The mirror image: the guard must not have swallowed the real store."""
    assert len(CardStore.load()) == 5


def test_the_paper_arm_refuses_to_run_without_a_corpus_entry(tmp_path):
    with pytest.raises(CardGenerationError, match="no corpus entry"):
        write_card("b", "navigate_to_goal", task_text("navigate_to_goal"),
                   _Backend(_yaml()), card=None, directory=tmp_path,
                   record=False)


def test_a_missing_pdf_says_so_and_names_the_fetcher(tmp_path):
    with pytest.raises(corpus.CorpusError, match="fetch_and_verify"):
        corpus.load("not_a_real_card", directory=tmp_path)


# --- the corpus reader -----------------------------------------------------

@pytest.mark.skipif(not corpus.available("limit_speed_dwa"),
                    reason="out/corpus/ is not populated on this machine")
def test_a_paper_extracts_to_text_that_is_not_glyph_soup():
    """pdftotext rather than pdfplumber, and the difference is not cosmetic: on
    this exact PDF pdfplumber returns "t h e c o n s t r u c t i o n". A card
    written from that is a card about nothing."""
    paper = corpus.load("limit_speed_dwa")
    assert paper.char_count > 10_000
    assert not paper.truncated
    assert len(paper.sha256) == 64
    assert "dynamic window" in paper.text.lower()
    assert " t h e " not in paper.text


def test_truncation_is_visible_to_the_model_and_to_the_record(tmp_path):
    """A silent truncation is a card written from half a paper by a model that
    believed it had the whole one."""
    pdf = tmp_path / "tiny.pdf"
    src = corpus.paper_path("limit_speed_dwa")
    if not src.is_file():
        pytest.skip("out/corpus/ is not populated on this machine")
    pdf.write_bytes(src.read_bytes())
    paper = corpus.load("tiny", directory=tmp_path, budget=500)
    assert paper.truncated
    assert "TRUNCATED" in paper.text
    assert paper.char_count > 500        # the marker is counted, not hidden


def test_the_redaction_names_the_document_without_reproducing_it():
    paper = corpus.Paper(card_id="x", text="secret body", sha256="a" * 64,
                         extractor="pdftotext", char_count=11, truncated=False,
                         path=Path("/nowhere.pdf"))
    redaction = paper.redaction()
    assert "secret body" not in redaction
    assert "a" * 16 in redaction
    assert "pdftotext" in redaction


# --- the pipeline version --------------------------------------------------

def test_pass_rate_refuses_to_pool_two_pipelines(tmp_path):
    """A v1 record and a v2 record with the same letter came from different
    treatments. Averaging them would be one number over two experiments."""
    from generate.admit import pass_rate

    for name, version, admitted in (("old", "v1-single-call", True),
                                    ("new", PIPELINE_VERSION, False)):
        (tmp_path / f"{name}.result.json").write_text(json.dumps({
            "node_id": name, "admitted": admitted, "reason": "r", "detail": "",
            "metadata": {"condition": "b", "pipeline_version": version}}))

    current = pass_rate(tmp_path)
    assert current["b"]["admitted"] == 0 and current["b"]["rejected"] == 1
    assert current["_pipeline_version"] == PIPELINE_VERSION
    assert any("v1-single-call" in row
               for row in current["_superseded_pipeline"])

    historical = pass_rate(tmp_path, pipeline_version="v1-single-call")
    assert historical["b"]["admitted"] == 1 and historical["b"]["rejected"] == 0


def test_a_record_with_no_version_field_is_v1(tmp_path):
    """Every record staged before chunk 42 predates the field, so its absence
    is not missing data — it is the answer."""
    from generate.admit import pass_rate
    (tmp_path / "old.result.json").write_text(json.dumps({
        "node_id": "old", "admitted": True, "reason": "r", "detail": "",
        "metadata": {"condition": "a"}}))
    assert "a" not in pass_rate(tmp_path)
    assert "a" in pass_rate(tmp_path, pipeline_version="v1-single-call")
