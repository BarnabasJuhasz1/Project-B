"""
generate/cardgen.py

**The card call — chunk 42. This is where the conditions differ, and it is now
the only place they do.**

Generation used to be one call: a condition's prompt, carrying either a human's
extracted card (B), a citation (A) or nothing (D), produced a leaf node
directly. `ProjectB_pipeline.drawio` has always shown something else — three
`LLM Writing Card` boxes, one per arm, feeding a single `Node` step — and this
module is that step.

    A   the citation, and nothing else        -> a card written from memory
    B   the source document, in full          -> a card written from the paper
    D   nothing at all                        -> a card written from scratch

Each arm writes a card in the **frozen six-field schema**, and
`generate/prompts/node.md` — one file, shared by every arm — turns that card
into a node. After this call the pipeline cannot tell the arms apart, which is
what makes "the conditions differ by exactly one thing" structural rather than a
diff test over three node prompts.

WHY THE REAL LOADER PARSES THE RESULT
-------------------------------------
`cards.store.card_from_dict`, not a laxer copy. A reconstruction that would not
load as a card is a failure of the reconstruction, not something this module
papers over — and the failures are informative: the first citations probe had
two arms name `localisation_failures`, a metric no controller can move, and the
loader's refusal was the result. Same rule here.

⚠️ **A GENERATED CARD IS NOT A PRINCIPLE CARD.** It lives under
`generate/cards/`, `cards.store` refuses to load that directory, and nothing in
it is counted in the corpus's provenance number. `cards/*.yaml` remains the
human-extracted corpus: the citation A is pointed at, the document B is handed,
and the verified-span answer key both are checked against.

SIM-FREE.
"""

import json
import re
from dataclasses import dataclass
from pathlib import Path

from generate import backends, corpus

PROMPTS = Path(__file__).resolve().parent / "prompts"
CARDS_DIR = Path(__file__).resolve().parent / "cards"

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# WHICH ARM'S CARD CALL SEES WHAT. The single source of truth; `generator.py`
# re-exports these. Anything asking "does this condition get the paper?" imports
# one of them rather than testing `condition == "b"`, because that test was
# scattered across four modules and each copy was a place the ablation could
# drift.
CITATION_CONDITIONS = ("a",)      # the reference alone
PAPER_CONDITIONS = ("b",)         # the source document, in full
CORPUS_CONDITIONS = ("a", "b")    # arms that enumerate over the card store

# What an arm with no source must put in `source.citation`. The schema requires
# a non-empty citation — a principle card is an extraction, and an extraction
# from nowhere is a contradiction — so the no-source arm says so in words rather
# than being handed a special case in the loader. It never reaches a prompt:
# `generator.render_node` does not render `source` at all.
# NO COLON, deliberately. The prompt asks the model to write this line into a
# YAML field, and `citation: (no source: ...)` is a scanner error — "mapping
# values are not allowed here" — so the arm with no source would have every card
# refused by the loader for a punctuation mistake the prompt told it to make.
# Found while building the replay fixture, which is what a fixture is for.
NO_SOURCE = "(no source - the principle was devised, not extracted)"

SOURCE_BLOCK = re.compile(
    r"<!-- ---- SOURCE BLOCK BEGIN ---- -->\n"
    r"(?P<block>.*?)"
    r"<!-- ---- SOURCE BLOCK END ---- -->\n",
    re.S)

YAML_FENCE = re.compile(r"```yaml\n(?P<card>.*?)```", re.S)

# The header on every written card. It sits one directory away from the real
# corpus and is the same shape, so anything that opens one is told what it is
# holding before it reads a field.
HEADER = (
    "# ⚠️ MODEL-WRITTEN CARD — NOT A PRINCIPLE CARD.\n"
    "# Written by condition {condition}'s card call ({source_note}).\n"
    "# Nobody has verified anything in this file. The span is a CLAIM.\n"
    "# cards.store refuses to load this directory. See generate/cardgen.py.\n")

SOURCE_NOTE = {
    "citation": "given the reference alone",
    "paper": "given the source document in full",
    "none": "given no source at all",
}


class CardGenerationError(RuntimeError):
    """The card call could not produce a card at all. NOT a rejection — a
    rejection is a card that was judged and found wanting, and `admit.py`
    records those."""


def treatment(condition):
    """What this arm's card call is given: 'citation', 'paper' or 'none'.

    Derived in one place so the prompt, the metadata, the filename header and
    the node's origin cannot disagree about which treatment was applied.
    """
    if condition in PAPER_CONDITIONS:
        return "paper"
    if condition in CITATION_CONDITIONS:
        return "citation"
    return "none"


def prompt_path(condition):
    path = PROMPTS / f"card_{condition}.md"
    if not path.is_file():
        raise CardGenerationError(
            f"no card prompt for condition {condition!r} at {path}. Every arm "
            f"writes its own card; an arm with no card prompt cannot generate.")
    return path


def source_block(condition):
    """Just the block that varies. What the diff test compares, and what the
    write-up quotes."""
    match = SOURCE_BLOCK.search(prompt_path(condition).read_text(encoding="utf-8"))
    if not match:
        raise CardGenerationError(
            f"card_{condition}.md has no source block markers. They are what "
            f"makes 'the card prompts differ only here' checkable rather than "
            f"asserted.")
    return match.group("block")


def shared_body(condition):
    """The card prompt with the source block removed. Must be identical across
    A, B and D — see generate/tests/test_prompts.py."""
    return SOURCE_BLOCK.sub("", prompt_path(condition).read_text(encoding="utf-8"))


def render(condition, task_text, card=None, paper=None):
    """The exact text sent to the model. Hashed, recorded and replayable.

    `card` is the CORPUS entry this arm is pointed at — the source of A's
    citation and the key naming B's PDF. It is never rendered wholesale: A sees
    `source.citation` and B sees the document, and neither sees the human's
    mechanism, trigger, derivation or prediction. That is the change chunk 42
    makes — the human extraction is no longer an input to generation.
    """
    text = prompt_path(condition).read_text(encoding="utf-8")

    if "{{CITATION}}" in text:
        if card is None:
            raise CardGenerationError(
                f"condition {condition} is pointed at a reference and none was "
                f"supplied. The citation is this arm's entire treatment.")
        text = text.replace("{{CITATION}}", card.source.citation.strip())
    elif "{{PAPER}}" in text:
        if paper is None:
            raise CardGenerationError(
                f"condition {condition} is handed the source document and none "
                f"was supplied. See generate/corpus.py — the corpus is an input "
                f"and is not committed, so this arm cannot run without it.")
        text = text.replace("{{PAPER}}", paper.text)
    elif card is not None or paper is not None:
        raise CardGenerationError(
            f"a source was supplied for condition {condition}, whose card "
            f"prompt has no slot for one. A source reaching the arm that is "
            f"supposed to have none would void the ablation silently.")

    return text.replace("{{TASK}}", task_text)


def extract_card(response_text, condition, where="<generated card>"):
    """The single fenced yaml block, parsed by the REAL loader.

    **No block is a result, not an error.** The card prompts invite an honest
    "I do not know this paper" outside the fence, and that answer belongs in the
    denominator next to every other outcome — the same way
    `generator.extract_code` treats a model that declines to emit a node.
    """
    blocks = YAML_FENCE.findall(response_text)
    if not blocks:
        return None
    if len(blocks) > 1:
        raise CardGenerationError(
            f"the response contains {len(blocks)} fenced yaml blocks and the "
            f"prompt asked for one; picking one would be the harness choosing "
            f"the card")

    import yaml
    from cards.store import card_from_dict

    data = yaml.safe_load(blocks[0])
    if not isinstance(data, dict):
        raise CardGenerationError(
            f"the yaml block is a {type(data).__name__}, not a card")

    # The no-source arm is TOLD to write NO_SOURCE and usually does. When it
    # leaves the citation empty instead, the schema would refuse the card — for
    # a good reason that does not apply here, since this arm has no source by
    # construction. Filled in rather than rejected, and the substitution is
    # recorded on the GeneratedCard so it is visible rather than silent.
    substituted = False
    if treatment(condition) == "none":
        source = data.get("source")
        if not isinstance(source, dict):
            source = {}
            data["source"] = source
        if not str(source.get("citation") or "").strip():
            source["citation"] = NO_SOURCE
            substituted = True

    return card_from_dict(data, where=where), substituted


@dataclass(frozen=True)
class GeneratedCard:
    """One model-written card, and everything needed to attribute it.

    `ref_id` names the CORPUS entry this arm was pointed at — the paper B read
    or the reference A was given — and is None for the arm with no source. It is
    not a claim that the corpus entry's content reached the model: for A it
    emphatically did not, and `treatment` is the field that says which.
    """

    card: object                 # a cards.schema.Card
    condition: str
    task: str
    treatment: str               # 'citation' | 'paper' | 'none'
    prompt_hash: str             # of the REAL prompt. Keys the transcript.
    model_id: str
    ref_id: str = None
    paper_sha256: str = None
    citation_substituted: bool = False
    # The grid row key — `generator.plan_hash`. Equal to `prompt_hash` for every
    # arm whose prompt this machine can render without the corpus, and different
    # for the arm that is handed a document: that prompt carries 50k characters
    # of PDF, so hashing it would make the grid uncomputable on a machine with
    # no corpus. Both are recorded; the stem uses this one, so the card file and
    # its staged candidate always share a name.
    key: str = None

    @property
    def stem(self):
        return (f"{self.condition}__{self.ref_id or self.task}__"
                f"{self.key or self.prompt_hash}")

    def metadata(self):
        return {
            "condition": self.condition,
            "task": self.task,
            "treatment": self.treatment,
            "card_prompt_hash": self.prompt_hash,
            "card_row_key": self.key or self.prompt_hash,
            "card_model_id": self.model_id,
            "ref_id": self.ref_id,
            "paper_sha256": self.paper_sha256,
            "generated_card_id": self.card.id,
            "generated_node_name": self.card.node_name,
            "citation_substituted": self.citation_substituted,
        }

    def as_dict(self):
        c = self.card
        return {
            "id": c.id,
            "node_name": c.node_name,
            "source": {"citation": c.source.citation,
                       "span": c.source.span,
                       # NEVER "verified": nobody has opened anything for this
                       # file. `Verification` has one honest value here.
                       "verified": "unverified"},
            "mechanism": c.mechanism,
            "trigger": c.trigger,
            "derivation": c.derivation,
            "prediction": {"statement": c.prediction.statement,
                           "metric": c.prediction.metric,
                           "baseline": c.prediction.baseline,
                           "min_effect": c.prediction.min_effect,
                           "cost_metric": c.prediction.cost_metric},
            "confidence": c.confidence.value,
            "generated": self.metadata(),
        }

    def write(self, directory=None):
        """Commit the card. One file per call, under the arm that wrote it."""
        import yaml
        directory = Path(directory or CARDS_DIR) / self.condition
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{self.stem}.yaml"
        header = HEADER.format(condition=self.condition.upper(),
                               source_note=SOURCE_NOTE[self.treatment])
        path.write_text(
            header + yaml.safe_dump(self.as_dict(), sort_keys=False,
                                    allow_unicode=True),
            encoding="utf-8")
        return path


def load_generated(path):
    """Read a written card back. The inverse of `write`, and the reason the node
    call is replayable with no corpus: the card is the committed artefact."""
    import yaml
    from cards.store import card_from_dict

    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    meta = data.pop("generated", {}) or {}
    card = card_from_dict(data, where=str(path))
    return GeneratedCard(
        card=card,
        condition=meta.get("condition", ""),
        task=meta.get("task", ""),
        treatment=meta.get("treatment", "none"),
        prompt_hash=meta.get("card_prompt_hash", ""),
        model_id=meta.get("card_model_id", ""),
        ref_id=meta.get("ref_id"),
        paper_sha256=meta.get("paper_sha256"),
        citation_substituted=bool(meta.get("citation_substituted")),
        key=meta.get("card_row_key"))


def write_refusal(response_text, condition, task, ref_id, key,
                  directory=None):
    """A call that produced no card, kept under a different extension.

    Staged rather than discarded for the same reason a node refusal is: G1
    reports a rate and a rate needs a denominator. `ls generate/cards/a/` then
    shows at a glance which citations the model declined to reconstruct.
    """
    directory = Path(directory or CARDS_DIR) / condition
    directory.mkdir(parents=True, exist_ok=True)
    stem = f"{condition}__{ref_id or task}__{key}"
    path = directory / f"{stem}.refusal.md"
    path.write_text(response_text, encoding="utf-8")
    return path


def write_card(condition, task, task_text, backend, card=None, key=None,
               directory=None, transcripts=None, record=True):
    """One card call: prompt -> model -> a committed card, or a recorded refusal.

    Returns `(GeneratedCard | None, prompt_hash, response)`. A None card is an
    outcome, not an exception — see `extract_card`.
    """
    paper = None
    if treatment(condition) == "paper":
        if card is None:
            raise CardGenerationError(
                f"condition {condition} reads the source document and no "
                f"corpus entry was supplied to name it.")
        paper = corpus.load(card.id)

    prompt = render(condition, task_text, card=card, paper=paper)
    prompt_hash = backends.prompt_hash(prompt)
    response = backend.complete(prompt)

    if record:
        # ⚠️ THE PAPER IS REDACTED FROM THE COMMITTED TRANSCRIPT and the hash
        # beside it is the hash of the REAL prompt. out/corpus/ is gitignored
        # because the PNAS copy carries the downloader's IP address, and a
        # transcript carrying the full text would commit the corpus through the
        # back door. A machine holding the PDF re-renders the prompt
        # byte-identically, so replay still finds this file.
        stored = prompt if paper is None else prompt.replace(paper.text,
                                                             paper.redaction())
        backends.write_transcript(
            prompt, response,
            meta={"kind": "card", "condition": condition, "task": task,
                  "treatment": treatment(condition),
                  "ref_id": card.id if card else None,
                  "paper_sha256": paper.sha256 if paper else None,
                  "paper_redacted": paper is not None,
                  **backend.describe()},
            directory=transcripts, prompt_text=stored)

    key = key or prompt_hash
    parsed = extract_card(response.text, condition)
    if parsed is None:
        write_refusal(response.text, condition, task,
                      card.id if card else None, key, directory=directory)
        return None, prompt_hash, response

    parsed_card, substituted = parsed
    generated = GeneratedCard(
        card=parsed_card,
        condition=condition,
        task=task,
        treatment=treatment(condition),
        prompt_hash=prompt_hash,
        model_id=response.model_id,
        ref_id=card.id if card else None,
        paper_sha256=paper.sha256 if paper else None,
        citation_substituted=substituted,
        key=key)
    generated.write(directory)
    return generated, prompt_hash, response
