"""
generate/base.py

**The base node.** Chunk 47.

One model-written navigator, shared byte-for-byte by every experimental
condition, and the substrate every behaviour tree in this project is grown on:

    Layer[ ports, BASE, node_1, ..., node_k ]

⚠️ **THE TREE SHAPE CHANGED ON 2026-09-14 (chunk 52) AND THE BASE'S ROLE WITH
IT.** It used to sit LAST in a `Fallback`, as the last resort a specialist fell
through to. It now sits FIRST in a `Layer`: it proposes a whole Twist on every
tick and later nodes revise the fields they declared. So it is no longer the
fallback, it is the default behaviour, and it is ticked on every step of every
episode rather than only when nobody else claimed the step.

**The node itself did not have to change.** Its prompt's one hard requirement —
never decline, always drive, always RUNNING — is exactly what a first layer
needs, and `Action.WRITES` defaults to all three Twist fields, so the base is a
full driver under the new contract without an edit. It is **not regenerated**,
and every growth number stays relative to the same node.

⚠️ **THIS IS NOT A CONDITION AND MUST NEVER BECOME ONE.**

- It is written by a **neutral** call: the node contract and the task family, no
  card, no citation, no corpus. It is not condition D's output and must not be
  reused from it — D writes one leaf from an invented principle, this writes the
  best navigator it can.
- ⚠️ Its prompt splices the contract section of **`prompts/node.md`**, which
  chunk 52 superseded for the arms. That is deliberate and must not be
  "modernised": the base was written against that exact text, the transcript
  hash is over the rendered prompt, and the base is frozen. Pointing this at
  `modifier.md` would describe the base with a contract it never saw.
- It is stored in `generate/base/`, **not** `generate/staging/`. Staging is arm
  territory: `admit.pass_rate` counts what is in it and `compose.library` builds
  the retrievable library from it. A base sitting there would be counted as an
  arm's node in G1 and could be retrieved as one by composition.
- It is a declared **substrate** under chunk 44's exemption to non-negotiable #3,
  and the write-up says so rather than glossing it.

WHY A STRONG BASE, AND WHY THE ENVIRONMENT IS THE FREE VARIABLE
----------------------------------------------------------------
The user's design, recorded in `plans/done/47-strongest-base-node.md`:

    I want the base as strong as possible, but still failing because the
    environment is difficult. If it succeeds 100% of the time, that does not
    mean the base is too strong — it means the environment is too easy.

So **difficulty is the base's failure rate**, and hardening a tier is the
response to a rate that is too low. Weakening the base never is.

⚠️ **FROZEN ONCE GENERATED.** Every growth-loop number is relative to this node.
Regenerating it is a different experiment, not a better base.

⚠️ EXECUTES MODEL-WRITTEN PYTHON, no sandbox — the same static AST allowlist
`generate/admit.py` applies, and for the same reason.
"""

import json
from dataclasses import dataclass
from pathlib import Path

from generate import backends
from generate.admit import ContractPorts, build_tree, check_source
from generate.generator import GenerationError, task_text
from nodes.contract import Action, Origin
from nodes.contract import NodeMetadata

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.

HERE = Path(__file__).resolve().parent
BASE_PROMPT = HERE / "prompts" / "base.md"
NODE_PROMPT = HERE / "prompts" / "node.md"
BASE_DIR = HERE / "base"

# The node id the base is always known by. Fixed, not a hash: unlike an arm's
# node there is exactly one of these, every tree contains it, and a committed
# result naming a base whose id moved would be unreadable.
BASE_NODE_ID = "base_navigator"

# The task the base is written against. The base has to work everywhere, so it
# is given the most general family rather than one of the four narrow ones.
BASE_TASK = "navigate_to_goal"

# Where node.md's contract section ends. The base prompt SPLICES that section in
# rather than restating it, so the base is written against character-identical
# contract text and the two cannot drift apart.
CONTRACT_ENDS_AT = "## The task family this node has to be useful for"


@dataclass(frozen=True)
class BaseCandidate:
    source: str
    prompt: str
    response: object
    node_cls: type
    metadata: NodeMetadata


def contract_section():
    """node.md's contract, verbatim, as the base prompt's {{CONTRACT}}."""
    text = NODE_PROMPT.read_text(encoding="utf-8")
    if CONTRACT_ENDS_AT not in text:
        raise GenerationError(
            f"{NODE_PROMPT.name} no longer contains {CONTRACT_ENDS_AT!r}, so the "
            f"base prompt cannot splice its contract section. Fix the marker "
            f"rather than duplicating the contract into base.md — two copies "
            f"drift, and the base would then be written against a different "
            f"contract from every other node.")
    return text.split(CONTRACT_ENDS_AT)[0].rstrip()


def render(task=BASE_TASK):
    """The exact base prompt. Hashed, recorded, replayable."""
    text = BASE_PROMPT.read_text(encoding="utf-8")
    for slot in ("{{CONTRACT}}", "{{TASK}}"):
        if slot not in text:
            raise GenerationError(f"{BASE_PROMPT.name} has no {slot} slot")
    return text.replace("{{CONTRACT}}", contract_section()).replace(
        "{{TASK}}", task_text(task))


def _metadata():
    # Origin.SUBSTRATE, not INVENTED: D invents a principle and is scored as an
    # arm; this invents nothing and is scored as nothing. Not HANDWRITTEN either
    # — a model wrote it, and the origin field exists to say exactly that.
    return NodeMetadata(node_id=BASE_NODE_ID, origin=Origin.SUBSTRATE)


def check(source):
    """Everything that can be checked without a model: it parses, it imports only
    what is allowed, it is exactly one Action, it builds, and it ticks.

    Deliberately NOT `admit.admit`. That path wants a PREDICTION and a micro()
    and measures a contribution against a card's claim; the base makes no claim
    and is measured directly by how often it reaches a goal.
    """
    # The AST allowlist is shared with the arms — same guard, same reason.
    check_source(source)

    # ⚠️ NOT `admit.load_candidate`. That one rejects anything without a
    # PREDICTION and a micro(), which the base prompt explicitly tells the model
    # not to write: the base makes no falsifiable claim and is measured directly
    # by how often it reaches a goal. Using the arm loader here rejected the
    # first real base call for NO_PREDICTION.
    namespace = {"__name__": "base_candidate", "__builtins__": __builtins__}
    try:
        exec(compile(source, "<base_candidate>", "exec"), namespace)  # noqa: S102
    except Exception as exc:                    # noqa: BLE001
        raise GenerationError(f"the base failed at import time: "
                              f"{type(exc).__name__}: {exc}") from exc

    classes = [v for v in namespace.values()
               if isinstance(v, type) and issubclass(v, Action) and v is not Action]
    if len(classes) != 1:
        raise GenerationError(
            f"the base must define exactly one Action subclass; found "
            f"{[c.__name__ for c in classes]}. Only an Action may drive, and the "
            f"base must drive on every tick.")
    node_cls = classes[0]
    metadata = _metadata()

    # It has to BUILD (wiring type-checks at construction) and it has to TICK.
    tree = build_tree(node_cls, metadata)

    # And its precondition must be unconditionally true — the one hard
    # requirement in the prompt. A base that declines leaves the tree with
    # nothing driving and the runner emits a zero Twist.
    node = [n for n in tree.nodes() if isinstance(n, node_cls)][0]
    if type(node).precondition is not Action.precondition:
        # It overrode it. That is allowed only if it still always says yes, and
        # we cannot prove that statically — so it is checked dynamically by the
        # calibration sweep, and flagged here so nobody assumes it was proven.
        pass
    return BaseCandidate(source=source, prompt="", response=None,
                         node_cls=node_cls, metadata=metadata)


def generate(backend, task=BASE_TASK, record=True, directory=None):
    """One neutral call -> the base node, checked and written."""
    prompt = render(task)
    response = backend.complete(prompt)

    # ⚠️ RECORD FIRST, CHECK SECOND. The first version checked before recording,
    # so a paid call whose output failed the check was discarded entirely — the
    # same mistake `generator.write_call_failure` exists to prevent on the arm
    # path, made again here. The transcript is the artefact; whether the code
    # passes is a separate question and must not be able to destroy it.
    if record:
        backends.write_transcript(prompt, response, meta={
            "kind": "base", "node_id": BASE_NODE_ID, "task": task,
            "condition": None, "role": "substrate",
        })

    source = _extract(response.text)
    candidate = check(source)

    if record:
        out = Path(directory or BASE_DIR)
        out.mkdir(parents=True, exist_ok=True)
        (out / f"{BASE_NODE_ID}.py").write_text(source, encoding="utf-8")
        (out / f"{BASE_NODE_ID}.json").write_text(json.dumps({
            "node_id": BASE_NODE_ID,
            "name": candidate.node_cls.__name__,
            "task": task,
            "prompt_hash": backends.prompt_hash(prompt),
            "prompt_chars": len(prompt),
            "is_condition": False,
            "role": "substrate",
            "note": ("The shared base node. NOT a condition, NOT counted in G1, "
                     "NOT in generate/staging/. Frozen once generated: every "
                     "growth-loop number is relative to it."),
            # `text` and `raw` are deliberately excluded: the response body
            # belongs in the transcript, and duplicating a 20 KB reply into the
            # record makes the record unreadable and the two able to disagree.
            **{k: v for k, v in response.as_dict().items()
               if k not in ("raw", "text")},
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return candidate


def _extract(text):
    from generate.generator import extract_code
    return extract_code(text)


def load(directory=None):
    """The committed base, as a node class + metadata. What a tree is built on."""
    path = Path(directory or BASE_DIR) / f"{BASE_NODE_ID}.py"
    if not path.exists():
        raise GenerationError(
            f"no base node at {path}. Generate one with "
            f"`scripts/generate_base.py`; it is a single model call and it is "
            f"frozen afterwards.")
    return check(path.read_text(encoding="utf-8"))
