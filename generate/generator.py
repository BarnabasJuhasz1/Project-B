"""
generate/generator.py

Card (+ the node contract spec) -> a candidate node, staged, never straight into
the library.

This is the only place conditions A, B and D differ, and the difference is one
file: `generate/prompts/condition_{a,b,d}.md`. Everything else on the path —
the contract spec, the task specification, the output format, the harness, the
locomotion policy — is byte-identical across them, and
`generate/tests/test_prompts.py` asserts that by stripping the literature block
and diffing what is left. If the three ever drift apart anywhere else, the
ablation measures that drift instead of the corpus.

ONE CARD PER CALL. §4 says "one card + the node contract spec", and one card per
call is what makes a source span attributable to exactly one node — which is
what G1's provenance fraction is a fraction OF. It also makes coverage a clean
count: distinct admissible primitives per condition, one card at a time.

STAGING, NOT ADMISSION. Nothing here decides anything. The generator writes a
candidate and a transcript to `generate/staging/`; `generate/admit.py` runs the
harness over it and decides. Splitting them is what lets a rejected candidate be
kept with its reason rather than thrown away — G1 reports a unit pass rate, and
a pass rate needs a denominator.

WHAT THE PROMPT DOES NOT CONTAIN, AND WHY IT MATTERS
----------------------------------------------------
No scenario. Not a dev scenario, not a held-out one, not lab42's floorplan. The
prompt describes a task FAMILY and the frozen contract, and the model builds its
own micro-world to test against. `generate/` cannot import
`core.scenarios_held_out` (chunk 11's AST guard), and the candidate itself
cannot import `core.scenario` at all (the allowlist in admit.py) — so a
generated node cannot be coupled to a specific scenario even by accident.
"""

import json
import re
from pathlib import Path

from generate import backends

PROMPTS = Path(__file__).resolve().parent / "prompts"
STAGING = Path(__file__).resolve().parent / "staging"

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

CONDITIONS = ("a", "b", "d")

# The capability-driven retrieval query, per §5: "the task specification is the
# retrieval query, the corpus is asked what mechanisms are known for this class
# of problem".
#
# These are derived from the FROZEN CONTRACT — chunk 11a's two axes, goal
# disclosure and pose drift — and not from any scenario. That is deliberate and
# it is the reason it is not leakage: generation is written against the contract
# by design, whereas a task spec written from a scenario list would be tailoring
# the prompt to the benchmark. Held constant across A, B and D.
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
}

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


def prompt_path(condition):
    if condition not in CONDITIONS:
        raise KeyError(
            f"no condition {condition!r}. The ablation has {CONDITIONS} on the "
            f"generation path; C is an off-the-shelf Nav2 stack and is not "
            f"generated at all.")
    return PROMPTS / f"condition_{condition}.md"


def literature_block(condition):
    """Just the block that varies. What the diff test compares, and what the
    write-up quotes."""
    text = prompt_path(condition).read_text(encoding="utf-8")
    match = LITERATURE_BLOCK.search(text)
    if not match:
        raise GenerationError(
            f"condition_{condition}.md has no literature block markers. They "
            f"are what makes 'the prompts differ only here' checkable rather "
            f"than asserted.")
    return match.group("block")


def shared_body(condition):
    """The prompt with the literature block removed. Must be identical across
    A, B and D — see generate/tests/test_prompts.py."""
    text = prompt_path(condition).read_text(encoding="utf-8")
    return LITERATURE_BLOCK.sub("", text)


def card_yaml(card):
    """The card as the model sees it: the six fields, verbatim, no commentary.

    Rendered from the loaded Card rather than by pasting the source file, so a
    field the schema does not carry cannot reach the model — the pipeline is
    source-agnostic, and a stray YAML key would be an uncontrolled input.

    **`span` and `confidence` are deliberately NOT rendered.** CLAUDE.md's
    card->node mapping puts Trigger on the precondition, Mechanism + Derivation
    in the node body, Prediction in the unit test, and Source + Confidence in
    NODE METADATA — not in the prompt that writes the body. Both still reach the
    node: admit.py sets source_span and confidence from the Card object itself,
    so nothing is lost by withholding them here.

    The practical reason is stronger than the tidiness one. These two fields
    change when a citation is VERIFIED, which is clerical work on provenance,
    not new information about the mechanism. With them in the prompt, verifying
    a card changes its prompt hash and orphans its committed transcript - and
    that really happened, to limit_speed_dwa, on the first four verifications.
    With ~20 cards still to verify it would have happened ~19 more times.

    `citation` STAYS. It identifies the document, and a citation that changes is
    a different source, so invalidating the transcript is then correct.
    """
    p = card.prediction
    lines = [
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
        f"  statement: {p.statement.strip()}",
        f"  metric: {p.metric}",
        f"  baseline: {p.baseline}",
        f"  min_effect: {p.min_effect}",
        f"  cost_metric: {p.cost_metric or '(none claimed)'}",
    ]
    return "\n".join(lines)


def render(condition, task, card=None):
    """The exact text sent to the model. Hashed, recorded, and replayable."""
    if task not in TASKS:
        raise KeyError(f"no task {task!r}. Known: {sorted(TASKS)}")
    text = prompt_path(condition).read_text(encoding="utf-8")

    if "{{CARD}}" in text:
        if card is None:
            raise GenerationError(
                f"condition {condition} takes a card and none was supplied. "
                f"That is condition B's entire independent variable.")
        text = text.replace("{{CARD}}", card_yaml(card))
    elif card is not None:
        raise GenerationError(
            f"a card was supplied for condition {condition}, whose prompt has "
            f"no slot for one. A card reaching A or D would void the ablation "
            f"silently — B is the condition with the corpus.")

    return text.replace("{{TASK}}", TASKS[task])


def extract_code(response_text):
    """The single fenced python block, or None if the model emitted none.

    Not "the first block" and not "the longest": a response with two blocks
    means the model did not follow the output contract, and quietly picking one
    would admit a node assembled by the harness rather than by the model.

    **NO BLOCK IS A RESULT, NOT AN ERROR.** Discovered on the first run against
    the deliberately-bad fixture card, which is what the fixture is for: the
    model declined to emit a node at all and explained why the card could not be
    implemented honestly — that a vague Mechanism forces the model to invent the
    body and let the card take credit for it. That is a finding about the card,
    and it belongs in the denominator of G1's pass rate next to every other
    rejection. Raising here would have thrown it away.
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


class Candidate:
    """One staged candidate: the source, and everything needed to attribute it.

    `card_id` is None for A and D by construction, and that is not a gap — G1's
    provenance fraction needs "invented" and "nobody recorded it" to be
    different values, which is exactly what `nodes.contract.Origin` exists for.
    """

    def __init__(self, condition, task, source, prompt, response,
                 card_id=None, node_id=None):
        self.condition = condition
        self.task = task
        # None when the model emitted no code. `response.text` is then the whole
        # artefact, and admit() turns it into a recorded NO_CANDIDATE_EMITTED
        # rejection rather than a crash.
        self.source = source
        self.prompt = prompt
        self.response = response
        self.card_id = card_id
        self.node_id = node_id or f"gen_{condition}_{backends.prompt_hash(prompt)}"

    @property
    def prompt_hash(self):
        return backends.prompt_hash(self.prompt)

    @property
    def stem(self):
        return f"{self.condition}__{self.card_id or self.task}__{self.prompt_hash}"

    def metadata(self):
        return {
            "node_id": self.node_id,
            "emitted_code": self.source is not None,
            "condition": self.condition,
            "task": self.task,
            "card_id": self.card_id,
            "model_id": self.response.model_id,
            "backend": self.response.backend,
            "prompt_hash": self.prompt_hash,
            "temperature": self.response.temperature,
            "sampling_note": self.response.sampling_note,
        }

    def write(self, directory=None):
        """Stage the candidate: the module, and a sidecar carrying provenance.

        The sidecar is separate from the module on purpose. Provenance in a
        docstring is provenance the model could have written; provenance in a
        file the model never saw is provenance the pipeline recorded.
        """
        directory = Path(directory or STAGING)
        directory.mkdir(parents=True, exist_ok=True)
        stem = f"{self.condition}__{self.card_id or self.task}__{self.prompt_hash}"
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


def generate(condition, task, backend, card=None, node_id=None,
             staging=None, transcripts=None, record=True):
    """One generation call: prompt -> model -> staged candidate.

    `record=False` is for tests replaying a committed transcript, where writing
    one back would overwrite the fixture with itself.
    """
    prompt = render(condition, task, card)
    response = backend.complete(prompt)

    if record:
        backends.write_transcript(
            prompt, response,
            meta={"condition": condition, "task": task,
                  "card_id": card.id if card else None,
                  **backend.describe()},
            directory=transcripts)

    source = extract_code(response.text)
    candidate = Candidate(condition, task, source, prompt, response,
                          card_id=card.id if card else None, node_id=node_id)
    candidate.write(staging)
    return candidate
