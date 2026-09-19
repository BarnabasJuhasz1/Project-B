"""repair/evidence.py — the bundle is BLIND, and stays blind.

⚠️ The committed fixture is the point of this file. A future change that
reopens a leak should fail a diff rather than a reviewer's attention.
"""
from __future__ import annotations

import inspect
from pathlib import Path

import pytest

from repair import evidence as E
from repair.verdict import classify_node

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
FIXTURE = Path(__file__).parent / "fixtures" / "bundle-ant-node.md"
NODE = "gen_b_44d12df4d25b437f"

pytestmark = pytest.mark.skipif(
    not (REPO_ROOT / "out" / "growth" / "maze-worlds.csv").exists(),
    reason="the committed growth table is not on this machine")


@pytest.fixture(scope="module")
def b():
    return E.bundle(NODE)


# --------------------------------------------------------------------------- #
# blindness


def test_no_arm_tell_survives(b):
    assert b.leaks() == (), b.leaks()


def test_the_bundle_cannot_be_told_which_arm_it_is(b):
    """⚠️ STRUCTURAL, NOT CONVENTIONAL. `bundle()` has no condition parameter,
    the same way `generator.render_node(card, task)` has none — the moment it
    can tell the arms apart, the loop has a second place to differ."""
    params = set(inspect.signature(E.bundle).parameters)
    for banned in ("condition", "arm", "treatment", "origin"):
        assert banned not in params


def test_the_card_block_carries_no_citation(b):
    """The card's `source` names the paper, and a card written from no source
    at all says so in that line. Neither may reach the agent.

    ⚠️ Scoped to the CARD BLOCK on purpose. The node SOURCE names its own paper
    in a docstring, and that is not an A-vs-B tell — both arms are pointed at
    the same paper. Scrubbing a model's own code comments would corrupt the
    artefact being diagnosed."""
    card = b.card.lower()
    for field in ("citation", "span:", "verified", "confidence", "generated:",
                  "paper_sha256", "ref_id", "condition"):
        assert field not in card, field


def test_the_card_block_renders_only_the_allowed_fields(b):
    """An allowlist, not a denylist: a denylist silently passes a field added
    later. `generator.card_yaml` IS the allowlist and is reused, not copied.

    ⚠️ Checked field by field rather than by parsing, because `card_yaml`
    emits multi-line values UNINDENTED — a prose line containing a colon looks
    exactly like a top-level key, and a naive scan reads the mechanism text as
    schema. (That the rendered block is not valid YAML is pre-existing and
    fine: it is a prompt, not a document anything loads.)
    """
    import re
    for field in E.CARD_FIELDS_RENDERED:
        assert re.search(rf"^{field}:", b.card, re.M), f"{field} is missing"
    for field in ("source", "span", "verified", "confidence",
                  "mechanism_family", "generated"):
        assert not re.search(rf"^{field}:", b.card, re.M), f"{field} LEAKED"


def test_author_surnames_do_not_reach_the_card_block(b):
    """No human card's authors may appear in what the agent reads as the card."""
    import re
    surnames = set()
    for path in (REPO_ROOT / "cards").glob("*.yaml"):
        text = path.read_text(encoding="utf-8")
        # ⚠️ `citation:` is a YAML FOLDED BLOCK (`>-`), so the text is on the
        # following indented lines and `citation:\s*(.+)` captures nothing.
        # The first version of this test did exactly that and passed
        # vacuously — the assert below is what caught it.
        found = re.search(r"^  citation:.*?\n((?:    .*\n)+)", text, re.M)
        if found:
            # ⚠️ SURNAMES, not every capitalised word. A citation's TITLE words
            # are not arm tells — "Cataglyphis" is the animal the mechanism is
            # about and the card cannot describe itself without it. A surname
            # is the thing that identifies the source, and it appears as
            # `Surname, X.` before an initial.
            surnames.update(re.findall(r"([A-ZÀ-Þ][a-zà-ÿ]{2,}),\s+[A-Z]\.",
                                       found.group(1)))
    assert surnames, "no citations found; this test would pass vacuously"
    leaked = {s for s in surnames if s.lower() in b.card.lower()}
    assert not leaked, leaked


def test_the_handle_is_opaque_and_not_the_node_id(b):
    assert NODE not in b.render()
    assert b.handle.startswith("node-")
    assert "_a_" not in b.handle and "_b_" not in b.handle


def test_the_handle_is_stable_and_salted():
    """⚠️ A handle derived from the id ALONE is brute-forceable: three arms, one
    known plan hash, three guesses. The salt is what stops that."""
    import hashlib
    assert E.handle_for(NODE) == E.handle_for(NODE)
    assert E.handle_for(NODE) != E.handle_for("gen_a_44d12df4d25b437f")
    unsalted = "node-" + hashlib.sha256(NODE.encode()).hexdigest()[:E.HANDLE_LEN]
    assert E.handle_for(NODE) != unsalted


def test_two_arms_with_the_same_hash_get_different_handles():
    assert E.handle_for("gen_a_deadbeef") != E.handle_for("gen_b_deadbeef")


# --------------------------------------------------------------------------- #
# content


def test_the_bundle_names_the_axes_that_were_not_varied(b):
    """⚠️ THE MOTIVATING CASE. `to_scenario()` never sets `drift`, so every
    growth world hands the controller a PERFECT POSE ESTIMATE — while this
    node's card predicts an effect "on scenarios where pose_estimate drifts".
    An agent shown only the geometry cannot reach that."""
    assert "drift               : None" in b.conditions
    assert "was NOT VARIED" in b.conditions
    assert "pose" in b.conditions.lower()


def test_the_card_still_names_drift_so_the_mismatch_is_visible(b):
    """The finding is only reachable if BOTH halves are in the bundle."""
    assert "drift" in b.card.lower()


def test_the_bundle_carries_per_world_rows_not_just_the_aggregate(b):
    """`fixed = 2` never says WHICH two, and a diagnosis needs the geometry
    that broke."""
    assert b.worlds.count("\n") >= 20
    assert "2004" in b.worlds and "broke" in b.worlds


def test_the_contract_block_says_what_was_available_and_unused(b):
    """"It never read the base's chosen heading" is only a finding if the
    heading was there to read."""
    for token in ("nav_heading", "nav_stuck", "nav_progress",
                  "LAST WRITER WINS", "cap(", "adjust("):
        assert token in b.contract, token


def test_the_verdict_reaches_the_agent(b):
    assert b.verdict.label is classify_node(NODE).label
    rendered = b.render()
    assert "DEGENERATE" in rendered and "n:       20 worlds" in rendered


def test_the_node_source_is_included_whole(b):
    assert "class SteerByPathIntegratedGoalVector" in b.node_source
    assert "WRITES" in b.node_source


# --------------------------------------------------------------------------- #
# the fixture, and the budget


def test_the_rendered_bundle_matches_the_committed_fixture(b):
    """⚠️ THE REGRESSION GUARD. A change that reopens a leak, or drops a
    section, fails here as a diff."""
    assert FIXTURE.exists(), f"regenerate with: python -m repair.evidence --node {NODE}"
    assert b.render() == FIXTURE.read_text(encoding="utf-8")


def test_the_fixture_itself_is_clean():
    """Belt and braces: the committed artefact is scanned directly, so a
    fixture regenerated from a leaky build fails even if `leaks()` changed."""
    text = FIXTURE.read_text(encoding="utf-8").lower()
    for tell in E.ARM_TELLS:
        assert tell.lower() not in text, tell
    assert NODE not in text


def test_the_bundle_fits_one_model_call(b):
    """⚠️ Measured, not assumed. A bundle that does not fit is not evidence."""
    chars = len(b.render())
    assert chars < 60_000, f"{chars} chars (~{chars // 4} tokens) is too large"


def test_a_case_can_be_attached_and_stays_clean():
    """The expensive path: two kinematic rollouts. Checked for leaks too,
    because a trajectory table is generated text like any other."""
    with_case = E.bundle(NODE, case_seed=2004)
    assert with_case.leaks() == ()
    assert "gen-maze-02004" in with_case.case
    assert "(ended)" in with_case.case, "the base finishes first; say so"
    assert "KINEMATIC" in with_case.case, "never let a rollout read as reportable"


def test_worst_case_seed_prefers_a_world_the_node_lost():
    from repair.verdict import load_measurement
    m = load_measurement(NODE)
    seed = E.worst_case_seed(m)
    row = next(w for w in m.worlds if w.seed == seed)
    assert row.outcome == "broke"


def test_the_node_source_may_name_its_own_paper_and_that_leak_is_bounded():
    """⚠️ A KNOWN, DELIBERATE, BOUNDED LEAK — recorded so it is not mistaken
    for an oversight.

    The ant node's own docstring says "Mueller & Wehner Eq. 3". That is the
    MODEL'S OWN CODE COMMENT, and three things make it acceptable:

      1. **A and B are pointed at the SAME paper.** Naming it cannot separate
         them, and A-vs-B is the headline comparison.
      2. It separates {A, B} from D — a weaker leak, and one the card's content
         carries anyway, since a card written from no source reads differently
         whatever is stripped from it. D is frozen regardless.
      3. Scrubbing it would corrupt the artefact under diagnosis. The node's
         comments are evidence about what the model believed it was
         implementing, which is exactly what a diagnosis needs.

    If this ever has to change, the fix is a scrubbed COPY of the source for
    the prompt plus the original on disk — not an edit to the staged file.
    """
    src = E.node_source(NODE)
    assert "Wehner" in src, (
        "if this fails the leak closed by accident; decide deliberately "
        "whether that is wanted before deleting this test")
    assert "Wehner" not in E.card_block(NODE), "the CARD must stay clean"
    # ⚠️ Uses E.ARM_TELLS, not a second loose list. A bare "condition" substring
    # matches `PRECONDITION`, which is a contract keyword — the first version of
    # this test failed on exactly that. The tells are punctuated for a reason.
    for tell in E.ARM_TELLS:
        assert tell.lower() not in src.lower(), (
            f"{tell} is an explicit LABEL, not a code comment")


def test_a_drifted_run_is_not_described_as_undrifted():
    """⚠️ THE THIRD DIAGNOSIS WRITTEN AGAINST AN EXPERIMENT THAT DID NOT HAPPEN.

    `conditions_block` built its scenario from the DEFAULTS whatever axis the
    loop was running on. So the first `--drift 0.10` lineage — 20 worlds all
    named `+drift0.10` — told the diagnosing agent `drift: None`, and four
    lines later told it that an axis reading None has not been tested against
    the card's claim. The agent asked for the measurement to be re-made with
    drift enabled. It already had been, and the round was spent on it."""
    from repair.evidence import conditions_block

    drifted = conditions_block("maze", seed=2000, drift=0.10)
    assert "drift               : None" not in drifted
    assert "sigma_xy=0.1" in drifted
    assert "Asking for" in drifted, "and it says so in as many words"


def test_the_undrifted_conditions_text_is_byte_identical():
    """⚠️ Every committed diagnosis transcript is keyed by a hash over the
    RENDERED prompt. A cosmetic change to the clean text orphans all of them —
    the same rule that keeps `prompts/node.md` and `compose.md` frozen."""
    from repair.evidence import conditions_block

    a = conditions_block("maze", seed=2000)
    b = conditions_block("maze", seed=2000, drift=None)
    assert a == b
    assert a.rstrip().endswith("it has not been tested against its own claim.")
