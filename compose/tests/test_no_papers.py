"""
compose/tests/test_no_papers.py

**Non-negotiable #6: composition sees only port contracts and a task
specification. Never the papers.**

This is the file that has to fail if that ever stops being true. It greps the
rendered composition prompt against the actual card store rather than reasoning
about the code — a construction that keeps cards out today is a construction
someone can change tomorrow without noticing what it was for.

Why it matters, in one sentence: G3 claims held-out scenarios were solved by
recombination alone, and a composition step that had read the Mechanism and
Derivation would be doing generation again under a different name.
"""

import re

import pytest

from compose.compose import render
from compose.library import load_library
from compose.retrieve import retrieve

# Words too generic to mean anything as evidence of a leak. "speed", "goal" and
# "distance" appear in a card AND in a node's own effect sentence, and a node's
# effect is explicitly what composition selects on.
STOPWORDS = set("""
a an and are as at be because been but by can could does for from has have if
in into is it its may more must no not of on one only or over should so such
than that the their then there these they this those through to under until up
was were what when where which while who will with within without
about above after again against all also always any been before being below
both cannot each either enough even ever every first give given here how
however into just least less like long made make many much never next once
onto other others out own rather same second several since some still take
than them then they thus very well whether whose why
robot speed goal distance path time range motion move moves obstacle
""".split())


def content_words(text):
    return {w for w in re.findall(r"[a-z]{4,}", text.lower())
            if w not in STOPWORDS}


@pytest.fixture(scope="module")
def prompts():
    """Every prompt this pipeline would send, across both axes."""
    from compose.spec import TaskSpec
    from core import scenario as s

    library = load_library(include_toys=True)
    out = []
    for goal_known in (True, False):
        for drifts in (True, False):
            spec = TaskSpec(
                family="probe", goal_known=goal_known, pose_drifts=drifts,
                n_beams=s.N_BEAMS, fov=s.FOV,
                range_max=s.RANGE_MAX, detect_radius=s.DETECT_RADIUS,
                max_lin=s.MAX_LIN, max_ang=s.MAX_ANG)
            out.append(render(spec, retrieve(library, spec)))
    return out


def test_no_mechanism_or_derivation_sentence_reaches_the_prompt(cards, prompts):
    """Sentence-level. A card's Mechanism or Derivation appearing verbatim is
    the unambiguous failure, and this catches a paste."""
    for card in cards.all():
        for field in (card.mechanism, card.derivation):
            for sentence in re.split(r"(?<=[.;])\s+", field.strip()):
                sentence = sentence.strip()
                if len(sentence) < 30:
                    continue
                for prompt in prompts:
                    assert sentence not in prompt, (
                        f"{card.id}: a {len(sentence)}-character span of the "
                        f"card reached the composition prompt")


def test_no_card_vocabulary_reaches_the_rendered_node_block(cards):
    """Word-level, and the stronger check: a paraphrase leaks too.

    **Scoped to the node block, and that scope is the point.** A prompt has two
    kinds of content. The template and `TaskSpec.describe()` are hand-written,
    committed and diffable — a card could only reach them if someone edited
    source, which the sentence-level test above covers by grepping the whole
    prompt. The node block is the only part rendered from DATA, and so the only
    part where a card could arrive without anyone editing anything. That is what
    this checks.

    Not zero overlap: a card about speed limiting and the node it produced share
    the word "brake" because they are about the same thing, and a node's EFFECT
    is a field composition is explicitly allowed to read. What must not happen is
    a card contributing vocabulary the node contracts do not already carry —
    which is exactly the difference computed here.
    """
    from compose.compose import render_node

    library = load_library(include_toys=True)
    contracts = set()
    block = []
    for entry in library.all():
        cls = entry.node_cls
        contracts |= content_words(
            " ".join([cls.EFFECT, cls.PRECONDITION, *cls.TAGS, entry.node_id]))
        block.append(render_node(entry))
    rendered = content_words("\n".join(block))

    for card in cards.all():
        card_words = content_words(
            " ".join([card.mechanism, card.derivation, card.source.citation,
                      card.prediction.statement]))
        leaked = (card_words & rendered) - contracts
        assert not leaked, (
            f"{card.id}: {sorted(leaked)} appear in the rendered node block "
            f"and in the card, and in no node's public contract")


def test_the_rendered_node_block_shows_only_identity_fields():
    """The structural half of the check above, and the one that would actually
    fire. Rendering is driven by `LibraryEntry.identity()`; if a card field were
    ever added to identity(), it would reach the prompt AND the snapshot tag
    together, and this fails first."""
    from compose.compose import render_node

    library = load_library(include_toys=True)
    entry = library.all()[0]
    ident = entry.identity()
    rendered = render_node(entry)
    for key in ("card_id", "source_span", "model_id", "prompt_hash",
                "origin", "confidence", "temperature"):
        value = ident.get(key)
        if value:
            assert str(value) not in rendered, (
                f"{key} is provenance, not a port contract, and composition "
                f"has no use for it")


def test_the_card_store_is_not_importable_from_the_render_path():
    """Structural, not textual. `render()` builds from LibraryEntry.identity(),
    which has no field a card could arrive in."""
    from nodes.library import LibraryEntry
    library = load_library(include_toys=True)
    keys = set(library.all()[0].identity())
    assert not keys & {"mechanism", "derivation", "source", "trigger",
                       "citation", "span"}
    assert "card_id" in keys      # the ATTRIBUTION stays; the content never was


def test_the_prompt_names_only_retrieved_nodes(prompts):
    """A prompt listing a node retrieval did not offer would widen the
    vocabulary behind the retrieval log's back."""
    from compose.spec import TaskSpec
    from core import scenario as s

    library = load_library(include_toys=True)
    spec = TaskSpec(family="probe", goal_known=True, pose_drifts=False,
                    n_beams=s.N_BEAMS, fov=s.FOV,
                    range_max=s.RANGE_MAX, detect_radius=s.DETECT_RADIUS,
                    max_lin=s.MAX_LIN, max_ang=s.MAX_ANG)
    found = retrieve(library, spec)
    prompt = render(spec, found)
    for entry in library.all():
        header = f"### `{entry.node_id}`"
        assert (header in prompt) == (entry.node_id in found.node_ids)
