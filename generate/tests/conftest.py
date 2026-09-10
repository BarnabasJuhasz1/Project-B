"""
generate/tests/conftest.py

Shared fixtures. **No test in this package calls a model.**

Every model-dependent test replays a committed transcript from
`generate/transcripts/`, which is exactly the property `ReplayBackend` exists to
provide: a test that called a live model would be slow, would cost money, would
need a network, and would be testing the model rather than the pipeline. It
would also be non-deterministic, which chunk 15 established the hard way — the
same prompt to the same model produced a refusal on one call and an admitted
node on the next.
"""

import pathlib

import pytest

FIXTURES = pathlib.Path(__file__).resolve().parent / "fixtures"

# The two committed transcripts, and what each is for. Both were produced by a
# real `claude -p` call against claude-opus-5; see generate/FINDINGS.md.
GOOD_PROMPT_HASH = "d6ae93648c7b4f25"    # limit_speed_dwa -> admitted
# Re-recorded 2026-08-27. The previous generation, 1f2fe8016b1441dc, was
# taken against a prompt that rendered the card's span and confidence;
# both were dropped from generator._render_card when the card was
# verified. That record is kept in staging/, marked superseded_by, and
# excluded from the pass rate — see generate/admit.py:pass_rate.
VAGUE_PROMPT_HASH = "7d2d6fc4a9d64ddd"   # the bad card    -> admitted anyway
# Re-recorded in the same pass, for the same reason. The fixture card's span is
# empty and its confidence is low, so dropping the two fields changed its
# rendered prompt by two blank lines and nothing else - but the hash is over the
# rendered prompt, so the transcript was orphaned exactly as the pilot card's
# was. 45a8af4b86a63990 is likewise kept and likewise superseded.


@pytest.fixture(scope="session")
def cards():
    from cards.store import CardStore
    return CardStore.load()


@pytest.fixture(scope="session")
def pilot_card(cards):
    """`limit_speed_dwa` — the pilot set's internal control, and the card the
    committed transcript was taken against."""
    return cards.get("limit_speed_dwa")


@pytest.fixture(scope="session")
def vague_card():
    """The deliberately bad card. Loaded BY PATH, never from the store: `cards/`
    is what G1's provenance fraction is computed over, and a fixture sitting in
    it would be counted."""
    from cards.store import load_card
    return load_card(FIXTURES / "vague_adaptive_motion.yaml")


@pytest.fixture
def replay():
    """Replay over the ARCHIVED v1 transcripts.

    Both committed transcripts this package replays — `GOOD_PROMPT_HASH` and
    `VAGUE_PROMPT_HASH` — were recorded by the one-call pipeline and moved to
    `generate/archive/v1-single-call/` on 2026-09-08 so that
    `generate/transcripts/` holds the current experiment and nothing else. The
    records are unchanged; only their address is.

    Chunk 42's two-call chain has its own fixture pair and its own backend —
    see `test_replay.py:chain`. Do not point that at this directory: a v1
    transcript cannot answer a v2 prompt, and the failure would look like a
    pipeline bug rather than a fixture mistake.
    """
    from generate.backends import ReplayBackend
    from generate.generator import V1_TRANSCRIPTS
    return ReplayBackend(V1_TRANSCRIPTS)


@pytest.fixture
def staging(tmp_path):
    """A throwaway staging directory. Tests must never write into
    `generate/staging/`, which is the committed record of what the real runs
    produced."""
    out = tmp_path / "staging"
    out.mkdir()
    return out
