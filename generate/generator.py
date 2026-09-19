"""
generate/generator.py

Task family (+ whatever the arm's source treatment supplies) -> a candidate
node, staged, never straight into the library.

**TWO CALLS, NOT ONE — chunk 42.** Generation used to be a single call whose
prompt differed per condition. It is now:

    1. the CARD call   generate/cardgen.py, one prompt per arm, differing only
                       inside a delimited SOURCE BLOCK
    2. the NODE call   generate/prompts/node.md, ONE prompt for every arm, fed
                       the card the first call wrote and nothing identifying

This is what `ProjectB_pipeline.drawio` has shown since 2026-09-02 and what the
code never did. The point of it is that the independent variable now lives
entirely in step 1: after the card exists, the pipeline cannot tell A's card
from B's from D's, so "the conditions differ by exactly one thing" is structural
rather than a diff test over three node prompts. `generate/tests/test_prompts.py`
asserts both halves — that the three card prompts are identical outside their
source block, and that the node prompt is a single shared file.

The superseded prompts `condition_{a,b,d}.md` are **kept and never edited**.
Every committed transcript was rendered from them, the hash is over the rendered
prompt, and editing one would orphan its own record.

ONE CARD PER NODE CALL. §4 says "one card + the node contract spec", and one
card per call is what makes a source attributable to exactly one node — which is
what G1's provenance fraction is a fraction OF.

STAGING, NOT ADMISSION. Nothing here decides anything. The generator writes a
candidate and two transcripts; `generate/admit.py` runs the harness and decides.
Splitting them is what lets a rejected candidate be kept with its reason rather
than thrown away — G1 reports a unit pass rate, and a pass rate needs a
denominator.

WHAT THE NODE PROMPT DOES NOT CONTAIN, AND WHY IT MATTERS
---------------------------------------------------------
No scenario. Not a dev scenario, not a held-out one, not lab42's floorplan. The
prompt describes a task FAMILY and the frozen contract, and the model builds its
own micro-world to test against. `generate/` cannot import
`core.scenarios_held_out` (chunk 11's AST guard), and the candidate itself
cannot import `core.scenario` at all (the allowlist in admit.py).

No source, either. `render_node` renders the card's mechanism, trigger,
derivation, prediction and node_name — and **not** its citation, span or
confidence. Those are provenance, CLAUDE.md's mapping puts them in node
metadata rather than in the body, and withholding them is what makes the
rendered node prompt blind to which arm wrote the card. The arm that has no
source at all would otherwise announce itself in a citation line.
"""

import json
import re
from pathlib import Path

from generate import backends, cardgen
# Re-exported: CLAUDE.md points here for "who sees what", and `cardgen` is where
# the card call — the only step that differs by condition — actually reads them.
from generate.cardgen import (CITATION_CONDITIONS,  # noqa: F401
                              CORPUS_CONDITIONS, PAPER_CONDITIONS)

PROMPTS = Path(__file__).resolve().parent / "prompts"
STAGING = Path(__file__).resolve().parent / "staging"

# THE NODE CALL'S PROMPT — `modifier.md` since chunk 52.
#
# `node.md` asked for a leaf that DRIVES, and every node it produced seized the
# whole Twist. Growth round 1 measured what that costs: of eleven admitted
# nodes, none improved the tree, six fired on 87.6-100% of ticks and silenced
# the base entirely, and `fixed` was 0 in every row. `modifier.md` asks for a
# node that REVISES the base's command — see nodes/contract.py, chunk 52.
#
# Still ONE FILE FOR EVERY ARM, which is the property that matters: the
# independent variable lives entirely in the card call, and `render_node` has no
# condition parameter.
NODE_PROMPT = PROMPTS / "modifier.md"

# SUPERSEDED, KEPT, NEVER EDITED — same discipline as `condition_{a,b,d}.md`.
# Every v2 transcript was rendered from this file and the hash is over the
# rendered prompt, so editing it would orphan its own record. `generate/base.py`
# also splices its contract section, deliberately: the base was written against
# THIS text and is frozen, so it must keep being describable by it.
V2_NODE_PROMPT = PROMPTS / "node.md"

# THE v1 RECORDS, MOVED ASIDE 2026-09-08 — not deleted.
#
# `generate/staging/` and `generate/transcripts/` hold the CURRENT pipeline's
# records and nothing else, so `ls` answers "what is this experiment made of"
# without a version filter. The one-call records are intact under `archive/`,
# and everything that legitimately studies them — chunk 39's paired A/B table,
# the eight committed compositions, the replay tests over the committed
# transcripts — addresses them through these two constants.
#
# They are a READ path. Nothing generates into the archive, and nothing in the
# live pipeline reads from it: `pass_rate`, `batch.plan` and `compose.library`
# all default to the live directories.
ARCHIVE = Path(__file__).resolve().parent / "archive"
V1_STAGING = ARCHIVE / "v1-single-call" / "staging"
V1_TRANSCRIPTS = ARCHIVE / "v1-single-call" / "transcripts"

# THE v2 RECORDS, MOVED ASIDE 2026-09-14 by chunk 52 — again, not deleted.
#
# ⚠️ THIS IS A DEVIATION FROM CHUNK 52 AS WRITTEN, and the reason is mechanical.
# The chunk said the eleven v2 candidates would "remain in generate/staging/",
# and they cannot: `plan_hash` is a pure function of (condition, task, card),
# the cards are NOT regenerated, so a v3 candidate lands on the identical stem.
# `batch.plan()` would see a rejection already recorded for that stem and skip
# the row forever — and "a rejection is never retried" is the rule G1's
# denominator depends on. Two experiments cannot share a key.
#
# Nothing is lost: the records are intact, `pass_rate(pipeline_version=...)`
# still reads them, and they still back growth round 1 and the findings drawn
# from it. See generate/archive/README.md.
V2_STAGING = ARCHIVE / "v2-exclusive-control" / "staging"
V2_TRANSCRIPTS = ARCHIVE / "v2-exclusive-control" / "transcripts"

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

CONDITIONS = ("a", "b", "d")

# Which generation pipeline produced a record. Bumped by chunk 42, and the
# reason it exists rather than a hand-applied `retired_reason`: every candidate
# on disk before 42 was produced by a ONE-CALL pipeline whose prompts are no
# longer reachable, and pooling those with two-call records would compute one
# pass rate over two different treatments and label it with the condition's
# letter. `admit.pass_rate` buckets on this and refuses to pool. Nothing is
# deleted and nothing is rewritten.
# ⚠️ BUMPED BY CHUNK 52 (2026-09-14). v2 and v3 share the card call byte for
# byte — the cards are not regenerated — and differ in the NODE call: v2 asked
# for a leaf that drives and wins the tick, v3 asks for a modifier that revises
# the base's command inside a `Layer`. Those are different treatments of the
# same card, so a pass rate pooled over both would be one number over two
# experiments wearing one condition's letter. `pass_rate` buckets on this.
PIPELINE_VERSION = "v3-layered"

# The capability-driven retrieval query, per §5: "the task specification is the
# retrieval query, the corpus is asked what mechanisms are known for this class
# of problem".
#
# These are derived from the FROZEN CONTRACT — chunk 11a's two axes, goal
# disclosure and pose drift — and not from any scenario. That is deliberate and
# it is the reason it is not leakage: generation is written against the contract
# by design, whereas a task spec written from a scenario list would be tailoring
# the prompt to the benchmark. Held constant across A, B and D.
#
# The four entries are the complete 2x2 of those two axes — goal disclosed or
# not, pose drifting or not. Chunk 29 took the fourth cell because it is the one
# growth lever that is CONDITION-NEUTRAL: it lifts A, B and D by the same factor.
# A fifth family would have to be argued from the contract in writing, and it may
# never be argued from the held-out split — that is leakage and it voids G3.
#
# **The card call is task-conditioned too**, and all three card prompts carry
# {{TASK}} in their SHARED body. The arm with no source needs a problem to
# invent against, and a prompt carrying the task for one arm and not another
# would differ outside the source block — which is the one property this design
# is built to keep. So A and B write one card per (paper x task family), and the
# reachable ceiling is unchanged at |cards| x |TASKS| for them and |TASKS| for D.
TASKS = {
    "navigate_to_goal": (
        "Reach a goal whose position is known, in an indoor space with no map. "
        "Walls and furniture occlude; the route is not a straight line and the "
        "obstruction is often not visible until the robot is close to it."
    ),
    "search_for_goal": (
        "Reach a goal whose position is NOT disclosed. `obs.goal` is None until "
        "the robot has been close enough to see the goal with nothing in the "
        "way, after which it latches. Until then the task is to cover ground "
        "that has not been covered, in an indoor space with no map."
    ),
    "navigate_under_drift": (
        "Reach a goal in an indoor space with no map, while `obs.pose_estimate` "
        "drifts: a seeded random walk is added to it, so the believed pose "
        "wanders away from the true one over the episode and "
        "`obs.pose_error_estimate` reports how wrong it is believed to be. The "
        "scan is unaffected — what degrades is the belief about where the robot "
        "is, not what it can see."
    ),
    "search_under_drift": (
        "Reach a goal whose position is NOT disclosed, in an indoor space with "
        "no map, while `obs.pose_estimate` drifts. `obs.goal` is None until the "
        "robot has been close enough to see the goal with nothing in the way, "
        "after which it latches; throughout, a seeded random walk is added to "
        "the pose estimate, and `obs.pose_error_estimate` reports how wrong it "
        "is believed to be. The scan is unaffected. The two are not independent "
        "here: covering ground that has not been covered requires a record of "
        "where the robot has already been, and that record can only be kept in "
        "the drifting frame, so it decays while it is being written."
    ),
}

# SUPERSEDED by chunk 42, kept because it still reads the prompts that produced
# every committed v1 transcript. `condition_{a,b,d}.md` are no longer rendered.
LITERATURE_BLOCK = re.compile(
    r"<!-- ---- LITERATURE BLOCK BEGIN ---- -->\n"
    r"(?P<block>.*?)"
    r"<!-- ---- LITERATURE BLOCK END ---- -->\n",
    re.S)

# The model is asked for one fenced python block. Anything else in the response
# is prose, and prose is kept in the transcript rather than parsed.
CODE_FENCE = re.compile(r"```(?:python|py)?\n(?P<code>.*?)```", re.S)


class GenerationError(RuntimeError):
    """The pipeline could not produce a candidate at all. NOT a rejection —
    a rejection is a candidate that was judged and found wanting, and it is
    recorded as an outcome by admit.py."""


def task_text(task):
    if task not in TASKS:
        raise KeyError(f"no task {task!r}. Known: {sorted(TASKS)}")
    return TASKS[task]


def check_condition(condition):
    if condition not in CONDITIONS:
        raise KeyError(
            f"no condition {condition!r}. The ablation has {CONDITIONS} on the "
            f"generation path; C is an off-the-shelf Nav2 stack and is not "
            f"generated at all.")
    return condition


# --- v1, superseded. Kept so the committed transcripts stay readable. --------

def prompt_path(condition):
    """The SUPERSEDED one-call prompt for this condition. Nothing renders these
    any more; `cardgen.prompt_path` is the live one."""
    return PROMPTS / f"condition_{check_condition(condition)}.md"


def literature_block(condition):
    text = prompt_path(condition).read_text(encoding="utf-8")
    match = LITERATURE_BLOCK.search(text)
    if not match:
        raise GenerationError(
            f"condition_{condition}.md has no literature block markers.")
    return match.group("block")


def _card_yaml_v1(card):
    """The card as the SUPERSEDED one-call prompt rendered it. Frozen."""
    pr = card.prediction
    return "\n".join([
        f"id: {card.id}",
        f"node_name: {card.node_name}",
        "",
        "source:",
        f"  citation: {card.source.citation.strip()}",
        "",
        f"mechanism: {card.mechanism.strip()}",
        "",
        f"trigger: {card.trigger.strip()}",
        "",
        f"derivation: {card.derivation.strip()}",
        "",
        "prediction:",
        f"  statement: {pr.statement.strip()}",
        f"  metric: {pr.metric}",
        f"  baseline: {pr.baseline}",
        f"  min_effect: {pr.min_effect}",
        f"  cost_metric: {pr.cost_metric or '(none claimed)'}",
    ])


def render_v1(condition, task, card=None):
    """The SUPERSEDED one-call prompt, rendered exactly as it used to be.

    Kept, and frozen, for one reason: the transcript hash is over the RENDERED
    prompt, so this function is the only thing that can still say which prompt a
    committed v1 transcript was taken against. Delete it and every record in
    `generate/transcripts/` taken before chunk 42 becomes an unverifiable blob.

    It is not on any live path. `generate()` does not call it, `batch` does not
    call it, and it must never acquire a new behaviour — a change here would
    silently reattribute history.
    """
    check_condition(condition)
    text = prompt_path(condition).read_text(encoding="utf-8")
    if "{{CARD}}" in text:
        if card is None:
            raise GenerationError(
                f"condition {condition} takes a card and none was supplied.")
        text = text.replace("{{CARD}}", _card_yaml_v1(card))
    elif "{{CITATION}}" in text:
        if card is None:
            raise GenerationError(
                f"condition {condition} takes a citation and no card was "
                f"supplied to take it from.")
        text = text.replace("{{CITATION}}",
                            f"citation: {card.source.citation.strip()}")
    elif card is not None:
        raise GenerationError(
            f"a card was supplied for condition {condition}, whose v1 prompt "
            f"has no slot for one.")
    return text.replace("{{TASK}}", task_text(task))


# --- the node call ----------------------------------------------------------

def card_yaml(card):
    """The card as the NODE call sees it.

    Rendered from the loaded Card rather than by pasting a file, so a field the
    schema does not carry cannot reach the model — a stray YAML key would be an
    uncontrolled input.

    **`source`, `span` and `confidence` are deliberately NOT rendered**, and
    under chunk 42 this is load-bearing rather than tidy. CLAUDE.md's card->node
    mapping puts Trigger on the precondition, Mechanism + Derivation in the node
    body, Prediction in the unit test, and Source + Confidence in NODE METADATA
    — not in the prompt that writes the body. All of it still reaches the node:
    `admit.py` sets them from the generated card object itself.

    Withholding `source` is also what makes the rendered node prompt blind to
    the arm. A card written with no source at all says so in its citation line;
    rendering that line would tell the node call which condition it was serving,
    and the whole point of one shared node prompt is that it cannot know.
    """
    p = card.prediction
    lines = [
        f"id: {card.id}",
        f"node_name: {card.node_name}",
        "",
        f"mechanism: {card.mechanism.strip()}",
        "",
        f"trigger: {card.trigger.strip()}",
        "",
        f"derivation: {card.derivation.strip()}",
        "",
        "prediction:",
        f"  statement: {p.statement.strip()}",
        f"  metric: {p.metric}",
        f"  baseline: {p.baseline}",
        f"  min_effect: {p.min_effect}",
        f"  cost_metric: {p.cost_metric or '(none claimed)'}",
    ]
    return "\n".join(lines)


def render_node(card, task):
    """The exact node prompt sent to the model. Hashed, recorded, replayable.

    ONE FILE FOR EVERY CONDITION. There is no `condition` parameter and there
    must never be one: the moment this function can tell the arms apart, the
    ablation acquires a second place to differ and `test_prompts.py` stops
    meaning anything.
    """
    text = NODE_PROMPT.read_text(encoding="utf-8")
    if "{{CARD}}" not in text:
        raise GenerationError(
            f"{NODE_PROMPT.name} has no {{{{CARD}}}} slot. The node call is fed "
            f"a card and nothing else identifying; a prompt with no slot for "
            f"one would silently generate from the task alone.")
    return text.replace("{{CARD}}", card_yaml(card)).replace(
        "{{TASK}}", task_text(task))


def extract_code(response_text):
    """The single fenced python block, or None if the model emitted none.

    Not "the first block" and not "the longest": a response with two blocks
    means the model did not follow the output contract, and quietly picking one
    would admit a node assembled by the harness rather than by the model.

    **NO BLOCK IS A RESULT, NOT AN ERROR.** Discovered on the first run against
    the deliberately-bad fixture card, which is what the fixture is for: the
    model declined to emit a node at all and explained why the card could not be
    implemented honestly. That is a finding about the card, and it belongs in
    the denominator of G1's pass rate next to every other rejection. Raising
    here would have thrown it away.
    """
    blocks = CODE_FENCE.findall(response_text)
    if not blocks:
        return None
    if len(blocks) > 1:
        raise GenerationError(
            f"the response contains {len(blocks)} fenced python blocks and the "
            f"output contract asks for one; picking one would be the harness "
            f"choosing the candidate")
    return blocks[0]


# --- the grid key -----------------------------------------------------------

def plan_hash(condition, task, card=None):
    """The row key: a pure function of (condition, task, corpus entry).

    The node prompt hash cannot be this key any more — it depends on a card that
    does not exist until the model writes one — so the CARD prompt hash takes
    over as the stem, the node id and the resume key. It is still a pure
    function of the frozen inputs, so `--dry-run` stays the real code path minus
    the calls and `staging/` stays the only state.

    **The paper is stood in for by its id, not read.** Hashing 50k characters of
    PDF text would make the grid uncomputable on a machine with no corpus, and
    `out/corpus/` is gitignored and never committed. For A and D the stand-in
    changes nothing and this hash IS the transcript hash; for B the transcript
    is keyed on the real prompt and the record carries both. A replaced PDF
    therefore keeps its row and loses its transcript — loudly, at replay, which
    is the right place to find out.
    """
    check_condition(condition)
    return backends.prompt_hash(
        cardgen.render(condition, task_text(task), card=card,
                       paper=_PaperStandIn(card) if card and
                       cardgen.treatment(condition) == "paper" else None))


class _PaperStandIn:
    """What `plan_hash` hashes in place of a document it will not read."""

    def __init__(self, card):
        self.text = f"(source document for {card.id}, supplied at call time)"


class Candidate:
    """One staged candidate: the source, and everything needed to attribute it.

    **`card_id` and `citation_id` are different facts and must never be merged.**
    `card_id` means the source's CONTENT reached the model — under chunk 42 that
    is the paper itself (condition B). `citation_id` means only the reference did
    (condition A). They name the same store entry, which is the point — A and B
    are pointed at the same paper — but a node built from a pointer was written
    by a model that never read the document, and recording it under `card_id`
    would hand A B's provenance for free and quietly inflate G1's numerator.
    `nodes.contract` keeps them apart as `Origin.CARD` and `Origin.CITATION`.

    Both are None for D by construction, and that is not a gap — G1's provenance
    fraction needs "invented" and "nobody recorded it" to be different values.

    `generated_card_id` is the third fact and it is new: which model-written card
    this node was actually built from. Every arm has one now, including D.
    """

    def __init__(self, condition, task, source, prompt, response,
                 card_id=None, node_id=None, citation_id=None,
                 generated=None, key=None):
        if card_id and citation_id:
            raise GenerationError(
                f"{condition}/{task}: a candidate cannot be both card-derived "
                f"and citation-only. Those are two conditions.")
        self.condition = condition
        self.task = task
        # None when the model emitted no code. `response.text` is then the whole
        # artefact, and admit() turns it into a recorded NO_CANDIDATE_EMITTED
        # rejection rather than a crash.
        self.source = source
        self.prompt = prompt
        self.response = response
        self.card_id = card_id
        self.citation_id = citation_id
        self.generated = generated
        # The card prompt hash, not the node one. See `plan_hash`.
        self.key = key or (generated.prompt_hash if generated else None)
        self.node_id = node_id or f"gen_{condition}_{self.key}"

    @property
    def prompt_hash(self):
        """The NODE prompt's hash. Provenance for the second call; not the stem.
        `NodeMetadata.prompt_hash` is this one — it is the prompt that wrote the
        code."""
        return backends.prompt_hash(self.prompt)

    @property
    def stem(self):
        # The reference names the stem for A as well as B: both enumerate over
        # the store, so the task alone would collide across five cards.
        return (f"{self.condition}__"
                f"{self.card_id or self.citation_id or self.task}__"
                f"{self.key}")

    def metadata(self):
        meta = {
            "node_id": self.node_id,
            "emitted_code": self.source is not None,
            "condition": self.condition,
            "task": self.task,
            "card_id": self.card_id,
            "citation_id": self.citation_id,
            "model_id": self.response.model_id,
            "backend": self.response.backend,
            "prompt_hash": self.prompt_hash,
            "card_prompt_hash": self.key,
            "temperature": self.response.temperature,
            "sampling_note": self.response.sampling_note,
            "pipeline_version": PIPELINE_VERSION,
        }
        if self.generated is not None:
            meta.update(self.generated.metadata())
            # `generated.metadata()` carries the card call's own condition and
            # task. They are the same values; asserting rather than trusting,
            # because a mismatch would mean a node was built from another arm's
            # card and nothing else would notice.
            if (self.generated.condition != self.condition
                    or self.generated.task != self.task):
                raise GenerationError(
                    f"card was written for "
                    f"{self.generated.condition}/{self.generated.task} but the "
                    f"node is being staged as {self.condition}/{self.task}. A "
                    f"node built from another arm's card is not this arm's node.")
        return meta

    def write(self, directory=None):
        """Stage the candidate: the module, and a sidecar carrying provenance.

        The sidecar is separate from the module on purpose. Provenance in a
        docstring is provenance the model could have written; provenance in a
        file the model never saw is provenance the pipeline recorded.
        """
        directory = Path(directory or STAGING)
        directory.mkdir(parents=True, exist_ok=True)
        # `self.stem`, never a second copy of the formula. This line used to
        # recompute it, which silently disagreed with the property the moment
        # `citation_id` existed: the module landed under one stem while
        # `admit()` wrote the verdict under another. A split pair looks PENDING
        # to `batch.plan()` while already carrying a recorded verdict, so the
        # row is re-run — and "a rejection is never retried" is the rule G1's
        # denominator depends on.
        stem = self.stem
        if self.source is None:
            # A refusal is staged too, under a different extension so that
            # nothing mistakes it for a module and so that `ls staging/` shows
            # at a glance which calls produced no node.
            module = directory / f"{stem}.refusal.md"
            module.write_text(self.response.text, encoding="utf-8")
        else:
            module = directory / f"{stem}.py"
            module.write_text(self.source, encoding="utf-8")
        (directory / f"{stem}.json").write_text(
            json.dumps(self.metadata(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8")
        return module


def write_call_failure(stem, exc, condition, task, key, prompt,
                       directory=None):
    """Record a model call that produced nothing usable.

    Written beside the staged candidates as `<stem>.error.json`, under an
    extension that is neither `.py` (a module), `.json` (a candidate) nor
    `.refusal.md` (the model declining, which IS a response). Nothing loads it
    and nothing counts it — it exists so that a call which cost real money and
    real wall clock is not indistinguishable from a call nobody made.

    The payload the CLI returned is kept verbatim where there was one: on a
    runaway generation the token counts are the finding.
    """
    directory = Path(directory or STAGING)
    directory.mkdir(parents=True, exist_ok=True)
    payload = getattr(exc, "payload", {}) or {}
    usage = payload.get("usage") or {}
    record = {
        "condition": condition,
        "task": task,
        "key": key,
        "stem": stem,
        "kind": "node",
        "prompt_hash": backends.prompt_hash(prompt),
        "prompt_chars": len(prompt),
        "error": str(exc),
        "stop_reason": payload.get("stop_reason"),
        "duration_api_ms": payload.get("duration_api_ms"),
        "total_cost_usd": payload.get("total_cost_usd"),
        "usage": usage,
        "pipeline_version": PIPELINE_VERSION,
    }
    path = directory / f"{stem}.error.json"
    path.write_text(json.dumps(record, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
    return path


def generate(condition, task, backend, card=None, node_id=None,
             staging=None, transcripts=None, record=True, cards_dir=None):
    """Two calls: the arm writes a card, then the shared node prompt implements
    it.

    `card` is the CORPUS entry this arm is pointed at, and it is consumed by the
    card call alone — A takes its citation, B takes it as the key naming a PDF,
    D ignores it. **The human extraction never reaches a prompt**, which is the
    whole of chunk 42.

    `record=False` is for tests replaying a committed transcript, where writing
    one back would overwrite the fixture with itself.
    """
    check_condition(condition)
    text = task_text(task)
    key = plan_hash(condition, task, card)

    generated, _card_hash, card_response = cardgen.write_card(
        condition, task, text, backend, card=card, key=key,
        directory=cards_dir, transcripts=transcripts, record=record)

    treatment = cardgen.treatment(condition)
    card_id = card.id if (card and treatment == "paper") else None
    citation_id = card.id if (card and treatment == "citation") else None

    if generated is None:
        # No card, so no node call. Staged as a refusal with the CARD call's
        # response as the artefact — the model declining to reconstruct a paper
        # it does not know is the honest answer the prompt invites, and it
        # belongs in G1's denominator rather than being retried into existence.
        candidate = Candidate(condition, task, None, "", card_response,
                              card_id=card_id, citation_id=citation_id,
                              node_id=node_id, key=key)
        candidate.write(staging)
        return candidate

    return node_call(condition, task, generated, backend, card_id=card_id,
                     citation_id=citation_id, key=key, node_id=node_id,
                     staging=staging, transcripts=transcripts, record=record)


def node_call(condition, task, generated, backend, card_id=None,
              citation_id=None, key=None, node_id=None, staging=None,
              transcripts=None, record=True):
    """THE SECOND CALL, on its own: one committed card -> one staged candidate.

    Split out of `generate()` by chunk 52 so that the node call can be re-made
    WITHOUT re-making the card call. That is not a convenience — it is what makes
    the regeneration honest. The layered pipeline changed the second call and
    nothing else, so A's and B's cards must be reused byte for byte; writing new
    ones would change the independent variable at the same time as the execution
    model and leave the comparison attributable to neither.

    `scripts/regenerate_modifiers.py` is the caller that does that.
    """
    # ⚠️ `generated.key` FIRST, and the bug that put this comment here: for the
    # arm handed a document, the card prompt carries 50k characters of PDF, so
    # its `prompt_hash` is NOT the grid row key — `plan_hash` hashes a stand-in
    # instead. Defaulting to `prompt_hash` staged six B candidates under stems
    # their own cards did not share, so `load_generated_card` could not find the
    # card the node was built from and `admit_staged` failed on a missing file.
    # `GeneratedCard.stem` uses `key or prompt_hash`; so does this, or the two
    # disagree for exactly one arm.
    key = key or generated.key or generated.prompt_hash
    node_prompt = render_node(generated.card, task)
    try:
        node_response = backend.complete(node_prompt)
    except backends.BackendError as exc:
        # ⚠️ A node call that fails leaves NO transcript, because there is no
        # response to record — so without this the row looks exactly like one
        # nobody ever ran, and `batch.plan()` (which resumes from staging/)
        # would re-attempt it forever, paying again each time.
        #
        # Measured 2026-09-09: condition D's `navigate_to_goal` node call
        # returned no fenced code after 136,423 output tokens, 134,242 of them
        # thinking — 29.7 minutes and $3.71 for nothing. That is a fact about
        # the GENERATION STEP, not about the machine it ran on, and it belongs
        # in the record beside the refusals.
        #
        # ⚠️ This is an ERROR record, deliberately NOT a staged refusal: it does
        # not decide whether a runaway call counts in G1's denominator. That is
        # a pre-registration question and it is the user's, not this function's.
        # The row stays PENDING to `batch.plan()`.
        if record:
            write_call_failure(generated.stem, exc, condition=condition,
                               task=task, key=key, prompt=node_prompt,
                               directory=staging)
        raise

    if record:
        backends.write_transcript(
            node_prompt, node_response,
            meta={"kind": "node", "condition": condition, "task": task,
                  "card_id": card_id, "citation_id": citation_id,
                  "generated_card_id": generated.card.id,
                  "card_prompt_hash": generated.prompt_hash,
                  **backend.describe()},
            directory=transcripts)

    source = extract_code(node_response.text)
    candidate = Candidate(condition, task, source, node_prompt, node_response,
                          card_id=card_id, citation_id=citation_id,
                          node_id=node_id, generated=generated, key=key)
    candidate.write(staging)
    return candidate
