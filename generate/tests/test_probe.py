"""
generate/tests/test_probe.py

Chunk 33's guards, and the diff.

**No test here calls a model.** The probe's model call is one line; everything
that decides anything is the prompt (rendered, free), the leak check (string
comparison) and the diff (arithmetic over two cards).
"""

import json

import pytest

from cards.store import CardStore, CardError, card_from_dict
from generate import probe


@pytest.fixture(scope="module")
def cards():
    return CardStore.load()


@pytest.fixture(scope="module")
def dwa(cards):
    return cards.get("limit_speed_dwa")


# --- the prompt carries the citation and NOTHING else -----------------------

def test_the_prompt_contains_the_citation(dwa):
    assert "Fox" in probe.render(dwa.source.citation)


def test_the_prompt_leaks_no_mechanism_trigger_derivation_or_span(cards):
    """The probe's entire claim. If the card's content reached the prompt, E
    would be measuring transcription rather than recall — and the result would
    look exactly the same."""
    for card in cards:
        prompt = probe.render(card.source.citation)
        assert probe.leaked(prompt, card) == [], card.id


def test_a_leak_is_detected_rather_than_assumed(dwa):
    """Constructed, because the real prompt has no way to leak — and a guard
    that has never fired is a guard nobody has tested."""
    poisoned = probe.render(dwa.source.citation) + "\n" + dwa.mechanism
    assert "mechanism" in probe.leaked(poisoned, dwa)


def test_probe_refuses_to_call_a_model_on_a_leaking_prompt(dwa, monkeypatch):
    monkeypatch.setattr(probe, "render",
                        lambda citation: "prelude " + dwa.derivation)

    class _Never:
        def complete(self, prompt):
            raise AssertionError("a model was called with a leaking prompt")

    with pytest.raises(probe.ProbeError, match="reached the probe prompt"):
        probe.probe(dwa, _Never(), record=False)


def test_a_citation_is_required():
    with pytest.raises(probe.ProbeError, match="needs a citation"):
        probe.render("   ")


# --- parsing ----------------------------------------------------------------

def _yaml_of(card, **overrides):
    data = {
        "id": overrides.get("id", card.id),
        "node_name": overrides.get("node_name", card.node_name),
        "source": {"citation": card.source.citation,
                   "span": overrides.get("span", "")},
        "mechanism": overrides.get("mechanism", card.mechanism),
        "trigger": card.trigger,
        "derivation": card.derivation,
        "prediction": {
            "statement": card.prediction.statement,
            "metric": overrides.get("metric", card.prediction.metric),
            "baseline": card.prediction.baseline,
            "min_effect": overrides.get("min_effect",
                                        card.prediction.min_effect),
            "cost_metric": card.prediction.cost_metric,
        },
        "confidence": overrides.get("confidence", card.confidence.value),
    }
    import yaml
    return "prose\n\n```yaml\n" + yaml.safe_dump(data) + "```\n"


def test_a_reconstruction_is_parsed_by_the_real_loader(dwa):
    """A reconstruction that would not load as a card is a failed
    reconstruction, not something the probe papers over with a laxer parser."""
    card = probe.extract_card(_yaml_of(dwa))
    assert card.node_name == dwa.node_name


def test_an_unparseable_reconstruction_is_rejected_by_the_card_rules(dwa):
    """core.metrics.score() decides what a metric is, here as everywhere."""
    with pytest.raises(CardError, match="not produced by"):
        probe.extract_card(_yaml_of(dwa, metric="vibes"))


def test_no_yaml_block_is_a_result_not_an_error():
    """The prompt explicitly invites 'I do not know this document', and that
    answer belongs in the denominator — the same way generator.extract_code
    treats a refusal."""
    assert probe.extract_card("I do not know this paper.") is None


def test_two_yaml_blocks_are_refused(dwa):
    with pytest.raises(probe.ProbeError, match="fenced yaml blocks"):
        probe.extract_card(_yaml_of(dwa) + _yaml_of(dwa))


# --- the diff ---------------------------------------------------------------

def test_a_perfect_reconstruction_scores_every_exact_field(dwa):
    result = probe.diff(dwa, probe.extract_card(_yaml_of(dwa)))
    assert result["reconstructed"]
    assert result["exact_match_count"] == result["exact_field_count"]
    assert result["prose"]["mechanism"]["similarity"] == 1.0


def test_a_wrong_number_is_caught(dwa):
    model = probe.extract_card(_yaml_of(dwa, min_effect=dwa.prediction.min_effect + 1))
    result = probe.diff(dwa, model)
    assert result["numeric"]["prediction.min_effect"]["match"] is False
    assert result["exact_match_count"] < result["exact_field_count"]


def test_a_wrong_confidence_is_caught(dwa):
    other = "low" if dwa.confidence.value != "low" else "high"
    result = probe.diff(dwa, probe.extract_card(_yaml_of(dwa, confidence=other)))
    assert result["exact"]["confidence"]["match"] is False


def test_a_refusal_diffs_to_a_recorded_non_result(dwa):
    result = probe.diff(dwa, None)
    assert result["reconstructed"] is False
    assert result["card_id"] == dwa.id


def test_an_invented_span_is_only_checkable_against_a_verified_one(cards):
    """The provenance question, and the reason limit_speed_ics is in the probe
    at all. A span the model invented where the human has NOT verified one is
    unfalsifiable, and must not be counted either way."""
    verified = next(c for c in cards if c.is_traceable)
    unverified = next(c for c in cards if not c.is_traceable)

    with_span = probe.diff(verified, probe.extract_card(
        _yaml_of(verified, span="Section 4, eq. (14)")))
    assert with_span["span"]["has_answer_key"] is True
    assert with_span["span"]["checkable"] is True

    no_key = probe.diff(unverified, probe.extract_card(
        _yaml_of(unverified, span="Section 4, eq. (14)")))
    assert no_key["span"]["has_answer_key"] is False
    assert no_key["span"]["checkable"] is False, \
        "an unverifiable claim was counted as checked"

    empty = probe.diff(verified, probe.extract_card(_yaml_of(verified)))
    assert empty["span"]["model_offered_one"] is False
    assert empty["span"]["checkable"] is False


# --- the store guard --------------------------------------------------------

def test_a_reconstruction_can_never_be_loaded_as_a_real_card(dwa, tmp_path):
    """A probe card reaching condition B's retrieval would corrupt the headline
    comparison silently — same shape, confabulated content."""
    fake = tmp_path / "cards" / "probe"
    fake.mkdir(parents=True)
    with pytest.raises(CardError, match="citations-probe store"):
        CardStore.load(fake)


def test_the_real_store_does_not_see_the_probe_directory(cards):
    """The glob is non-recursive, so cards/probe/*.yaml is already invisible.
    Asserted rather than relied on."""
    from generate.probe import PROBE_DIR
    assert PROBE_DIR.name == "probe"
    assert all("probe" not in c.id for c in cards)
    assert len(cards) == len(list(PROBE_DIR.parent.glob("*.yaml")))


def test_a_written_reconstruction_says_what_it_is(dwa, tmp_path):
    card = probe.extract_card(_yaml_of(dwa, span="Section 4"))
    result = probe.diff(dwa, card)
    probe.write_result(result, card, tmp_path)
    text = (tmp_path / f"{dwa.id}.yaml").read_text(encoding="utf-8")
    assert "MODEL RECONSTRUCTION" in text
    assert "NOT A PRINCIPLE CARD" in text
    # `verified` is never carried over: nobody opened the paper.
    assert "verified: unverified" in text


def test_findings_are_generated_from_the_diffs(dwa, tmp_path):
    card = probe.extract_card(_yaml_of(dwa, span="Section 4"))
    probe.write_result(probe.diff(dwa, card), card, tmp_path)
    path = probe.write_findings(probe.load_results(tmp_path), tmp_path)
    text = path.read_text(encoding="utf-8")
    assert "UNRESOLVED" in text, "an unchecked span was reported as settled"
    assert "not a fifth condition" in text.lower()
    assert dwa.id in text


def test_a_probe_condition_was_not_added_to_the_ablation():
    """`runs/spec.py` refuses a fifth condition and #10 is why. The probe must
    not smuggle one in through the generator."""
    from generate.generator import CONDITIONS
    assert CONDITIONS == ("a", "b", "d")
    assert not (probe.PROMPT.parent / "condition_e.md").exists()


# --- refusal and rejection are different results ----------------------------

def test_a_rejected_reconstruction_is_not_reported_as_a_refusal(dwa, tmp_path):
    """Both non-producers on the first real run had returned a card.

    `path_integration_mammals` and `reorient_geometry` each produced a card in
    the frozen schema naming `localisation_failures` as the prediction metric,
    and `card_from_dict` refused it — a metric no controller can move is not
    falsifiable about a node. FINDINGS.md filed both under "No card returned",
    which read as though the model had declined. It had not, and the reason it
    failed is the more interesting half of the result.
    """
    refusal = probe.diff(dwa, None)
    refusal["note"] = "the model declined to reconstruct — no yaml block"

    rejection = probe.diff(dwa, None)
    rejection["card_id"] = "rejected_one"
    rejection["error"] = ("CardError: prediction.metric "
                          "'localisation_failures' cannot be moved")

    assert not probe._rejected(refusal)
    assert probe._rejected(rejection)

    probe.write_result(refusal, None, tmp_path)
    probe.write_result(rejection, None, tmp_path)
    text = probe.write_findings(probe.load_results(tmp_path),
                                tmp_path).read_text(encoding="utf-8")

    assert "Returned a card the loader refused" in text
    assert "Declined to reconstruct" in text
    assert "localisation_failures" in text, "the reason was dropped"
    # The headline names every outcome, not just the producers.
    assert "returned one the loader refused" in text
    assert "declined to reconstruct" in text


def test_the_diff_alone_does_not_guess_why_there_is_no_card(dwa):
    """`diff()` is handed None and cannot tell the two apart, so it must not
    claim to. `probe()` sets the specific note, because only it saw the
    exception."""
    note = probe.diff(dwa, None)["note"]
    assert "returned no card" not in note
    assert "no card was parsed" in note


def test_rejection_is_derived_from_the_error_not_a_stored_key(dwa):
    """Diffs written before `outcome` existed are on disk and must still
    classify correctly, so the split reads `error` — the signal that was always
    there."""
    old = probe.diff(dwa, None)
    old["error"] = "CardError: something"
    assert "outcome" not in old
    assert probe._rejected(old)
