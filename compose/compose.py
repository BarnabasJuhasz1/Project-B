"""
compose/compose.py

TaskSpec + retrieved port contracts -> a `nodes.tree.Tree`.

THE ONE DECISION THIS FILE MAKES
---------------------------------
**Composition returns a JSON tree over node ids. It never returns Python.**

G3 measures the fraction of held-out scenarios solved by **recombination alone,
no new primitives**. If composition emitted code, "no new primitives" would be a
rule the model was asked to respect, checkable only by reading what it wrote and
arguing about whether a helper function counts. Emitting a plan whose entire
vocabulary is a list of retrieved node ids makes it structural: there is no
channel through which a new primitive could arrive. A plan naming an id that was
not retrieved is rejected, by `build()`, with the reason recorded.

That also makes the repair loop honest. Chunk 16 requires repair to be allowed to
**rewire, never to create**, and the two to be separately logged. Since the
vocabulary of a repair is the same retrieved set, a repair *cannot* create; the
log distinguishes them by `kind`, and `created_primitives` is in every record and
is structurally always empty. It is recorded anyway, because a reader of the G3
table should be able to check the claim rather than be told it.

WHAT THE PROMPT CONTAINS
------------------------
The task spec's `describe()`, and `LibraryEntry.identity()` rendered per node —
kind, ports, effect, precondition, tags. Nothing else. Mechanism, Derivation and
Source are not reachable from a LibraryEntry, so #6 holds by construction; a
test in `compose/tests/` greps the rendered prompt against every card in the
store anyway, because a construction someone could later change is not a
guarantee.

    python scripts/compose_tree.py --scenario doorway_traverse --backend replay
    python -c "import controllers; print(controllers.make('composed:<id>'))"
"""

import json
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

from nodes.contract import Kind, NodeMetadata, Origin
from nodes.tree import Fallback, Sequence, Tree, WiringError

from compose import retrieve as retrieve_mod
from compose.library import load_library
from compose.spec import TaskSpec

PROMPTS = Path(__file__).resolve().parent / "prompts"
RECORDS = Path(__file__).resolve().parent / "compositions"

# One fenced json block, the same shape as the generator's code fence and for
# the same reason: two blocks means the output contract was not followed, and
# picking one would be the harness choosing the composition.
JSON_FENCE = re.compile(r"```(?:json)?\n(?P<body>.*?)```", re.S)

COMPOSITES = {"sequence": Sequence, "fallback": Fallback}


class CompositionError(Exception):
    """A plan that cannot become a tree. Carries a reason code, so that a G3
    miss can be classified without reading prose."""

    def __init__(self, reason, detail=""):
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


# --- the prompt ------------------------------------------------------------

def render_node(entry):
    """One node's public contract, exactly as `LibraryEntry.identity()` has it.

    Rendered from identity() rather than from the class so that what the model
    reads and what the snapshot tag hashes are the same fields. If a field is
    ever added to identity(), it appears here and in the tag together.
    """
    ident = entry.identity()
    ports = ", ".join(f"{name} ({ptype}, {direction})"
                      for name, ptype, direction in ident["ports"]) or "none"
    return "\n".join([
        f"### `{ident['node_id']}`",
        f"- kind: **{ident['kind']}**",
        f"- ports: {ports}",
        f"- effect: {ident['effect']}",
        f"- precondition: {ident['precondition']}",
        f"- tags: {', '.join(ident['tags']) or 'none'}",
    ])


def render(spec, retrieval):
    """The exact text sent to the model. Hashed, recorded, replayable."""
    text = (PROMPTS / "compose.md").read_text(encoding="utf-8")
    nodes = "\n\n".join(render_node(e) for e in retrieval.entries)
    if not nodes:
        raise CompositionError(
            "EMPTY_RETRIEVAL",
            "retrieval returned no nodes, so there is nothing to compose from. "
            "That is a fact about the library or the query, not a model "
            "failure, and it is recorded as one.")
    return text.replace("{{TASK}}", spec.describe()).replace("{{NODES}}", nodes)


def parse_plan(response_text):
    """The single fenced json block, parsed. Raises with a reason code."""
    blocks = JSON_FENCE.findall(response_text)
    if not blocks:
        raise CompositionError(
            "NO_PLAN_EMITTED",
            "the response contains no fenced json block; the recorded "
            "transcript is what it returned instead")
    if len(blocks) > 1:
        raise CompositionError(
            "MULTIPLE_PLANS",
            f"{len(blocks)} fenced json blocks and the output contract asks "
            f"for one")
    try:
        plan = json.loads(blocks[0])
    except json.JSONDecodeError as exc:
        raise CompositionError("MALFORMED_PLAN", str(exc)) from exc
    if not isinstance(plan, dict) or "tree" not in plan:
        raise CompositionError("MALFORMED_PLAN",
                               "the object has no 'tree' key")
    if "rationale" not in plan:
        # Not cosmetic. The rationale is the only place a composition says WHY,
        # and the first real run showed a model reasoning about a node it had
        # deliberately left out — which is the kind of thing a G3 post-mortem
        # needs and no other field records.
        raise CompositionError("MALFORMED_PLAN",
                               "the object has no 'rationale' key")
    return plan


# --- plan -> tree ----------------------------------------------------------

def _composite_metadata(node_id):
    """Provenance for a composite.

    HANDWRITTEN, and the word is doing work. A composite emits nothing of its
    own — `Composite.EFFECT` is literally "control flow only" — so it carries no
    mechanism and no source, and G1's provenance fraction counts library
    entries, which composites never become. The fact that a *model* chose this
    control flow is recorded where it belongs: in the composition record, with
    the prompt hash that produced it.
    """
    return NodeMetadata(node_id=node_id, origin=Origin.HANDWRITTEN)


def _build_node(spec_node, allowed, counter, path="tree"):
    if not isinstance(spec_node, dict):
        raise CompositionError("MALFORMED_PLAN", f"{path} is not an object")
    kind = spec_node.get("type")

    if kind in COMPOSITES:
        children = spec_node.get("children")
        if not isinstance(children, list) or not children:
            raise CompositionError(
                "MALFORMED_PLAN",
                f"{path}: a {kind} needs a non-empty 'children' list")
        index = counter[kind] = counter.get(kind, 0)
        counter[kind] = index + 1
        built = [_build_node(c, allowed, counter, f"{path}.children[{i}]")
                 for i, c in enumerate(children)]
        return COMPOSITES[kind](
            _composite_metadata(f"composed_{kind}_{index}"), built)

    if kind in ("leaf", "decorator"):
        node_id = spec_node.get("node")
        entry = allowed.get(node_id)
        if entry is None:
            raise CompositionError(
                "UNRETRIEVED_NODE",
                f"{path}: {node_id!r} is not in the retrieved set "
                f"{sorted(allowed)}. Composition's vocabulary is the retrieved "
                f"library and nothing else — that is what makes 'no new "
                f"primitives' mechanical rather than a promise.")
        declared = entry.node_cls.KIND
        if kind == "decorator":
            if declared is not Kind.DECORATOR:
                raise CompositionError(
                    "KIND_MISMATCH",
                    f"{path}: {node_id!r} is a {declared.value} and cannot wrap "
                    f"a child")
            child = spec_node.get("child")
            if child is None:
                raise CompositionError(
                    "MALFORMED_PLAN",
                    f"{path}: a decorator wraps exactly one 'child'")
            return entry.node_cls(
                entry.metadata,
                _build_node(child, allowed, counter, f"{path}.child"))
        if declared is Kind.DECORATOR:
            raise CompositionError(
                "KIND_MISMATCH",
                f"{path}: {node_id!r} is a decorator and needs a child; use "
                f"type 'decorator'")
        return entry.build()

    raise CompositionError(
        "MALFORMED_PLAN",
        f"{path}: unknown type {kind!r}; known: sequence, fallback, decorator, "
        f"leaf")


def build(plan, retrieval, snapshot=None):
    """Plan -> Tree, or a CompositionError with a reason code.

    `Tree.__init__` runs `check_wiring`, so a plan whose ports do not connect
    fails HERE — at composition time, before an episode is spent on it. That is
    chunk 12's decision and this is what it was for: a tree that failed mid
    episode would score a timeout and look like a bad idea rather than a bad
    wiring, and G3 counts solved scenarios.
    """
    allowed = {e.node_id: e for e in retrieval.entries}
    root = _build_node(plan["tree"], allowed, {})
    try:
        tree = Tree(root, snapshot=snapshot)
    except WiringError as exc:
        raise CompositionError("WIRING_FAILED", str(exc)) from exc
    if not any(n.KIND is Kind.ACTION for n in tree.nodes()):
        raise CompositionError(
            "NO_ACTION",
            "the tree contains no Action, so nothing in it can emit a Twist "
            "and the robot cannot move")
    return tree


# --- the record ------------------------------------------------------------

@dataclass
class Composition:
    """One composition attempt: what was asked, what was offered, what came
    back, and what happened to it. Written whether or not it succeeded — a G3
    denominator needs the failures."""

    composition_id: str
    spec: TaskSpec
    retrieval: retrieve_mod.Retrieval
    snapshot: str
    prompt: str
    response: object
    plan: Optional[dict] = None
    tree: Optional[Tree] = None
    ok: bool = False
    reason: str = ""
    detail: str = ""
    kind: str = "initial"                 # "initial" or "repair"
    repair_of: Optional[str] = None
    include_toys: bool = False

    @property
    def prompt_hash(self):
        from generate import backends
        return backends.prompt_hash(self.prompt)

    def as_dict(self):
        return {
            "composition_id": self.composition_id,
            "kind": self.kind,
            "repair_of": self.repair_of,
            "ok": self.ok,
            "reason": self.reason,
            "detail": self.detail,
            # Non-negotiable #8: every reported result names the frozen library
            # snapshot tag it was produced from.
            "library_snapshot": self.snapshot,
            "include_toys": self.include_toys,
            "spec": self.spec.as_dict(),
            "retrieval": self.retrieval.as_dict(),
            "prompt_hash": self.prompt_hash,
            "model_id": self.response.model_id,
            "backend": self.response.backend,
            "temperature": self.response.temperature,
            "sampling_note": self.response.sampling_note,
            "plan": self.plan,
            "tree_nodes": [n.metadata.node_id for n in self.tree.nodes()]
                          if self.tree is not None else None,
            # Structurally always empty: the plan's vocabulary is a list of
            # retrieved ids and there is no channel through which a primitive
            # could be created. Recorded anyway so a reader of the G3 table can
            # check the claim instead of being asked to believe it.
            "created_primitives": [],
        }

    def write(self, directory=None):
        directory = Path(directory or RECORDS)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{self.composition_id}.json"
        path.write_text(json.dumps(self.as_dict(), indent=2, sort_keys=True)
                        + "\n", encoding="utf-8")
        return path


def compose(spec, library, backend, retrieval=None, snapshot=None,
            include_toys=False, kind="initial", repair_of=None,
            record=True, records=None, transcripts=None):
    """One composition call: spec -> retrieval -> model -> tree.

    Never raises for a plan's own faults; a bad plan is a recorded failure with
    a reason code, exactly as a bad candidate is a recorded rejection.
    """
    from generate import backends as backends_mod

    retrieval = retrieval or retrieve_mod.retrieve(library, spec)
    snapshot = snapshot or library.snapshot()
    prompt = render(spec, retrieval)
    response = backend.complete(prompt)

    if record:
        backends_mod.write_transcript(
            prompt, response,
            meta={"step": "compose", "family": spec.family,
                  "axes": list(spec.axes), "library_snapshot": snapshot,
                  "kind": kind, "repair_of": repair_of,
                  **backend.describe()},
            directory=transcripts)

    composition_id = _composition_id(spec, kind,
                                     backends_mod.prompt_hash(prompt))
    result = Composition(composition_id=composition_id, spec=spec,
                         retrieval=retrieval, snapshot=snapshot, prompt=prompt,
                         response=response, kind=kind, repair_of=repair_of,
                         include_toys=include_toys)
    try:
        result.plan = parse_plan(response.text)
        result.tree = build(result.plan, retrieval, snapshot=snapshot)
        result.ok = True
        result.reason = "composed"
    except CompositionError as exc:
        result.reason = exc.reason
        result.detail = exc.detail

    if record:
        result.write(records)
    return result


def repair(previous, failure_signature, backend, library=None, **kwargs):
    """Re-compose after a failure, from the SAME library and the SAME retrieval.

    Rewiring only. The vocabulary is the previous retrieval's node ids, so this
    cannot create a primitive even if it wanted to — which is what G3's "no new
    primitives generated" needs, and why the check is not a rule anyone has to
    remember. It is logged as `kind: "repair"` with `repair_of` naming what it
    is repairing, so the two are separable in the record.
    """
    spec = TaskSpec(**{**previous.spec.as_dict(),
                       "failure_signature": failure_signature})
    return compose(spec, library, backend, retrieval=previous.retrieval,
                   snapshot=previous.snapshot,
                   include_toys=previous.include_toys, kind="repair",
                   repair_of=previous.composition_id, **kwargs)


def _composition_id(spec, kind, prompt_hash):
    axes = "_".join(spec.axes)
    prefix = "r_" if kind == "repair" else ""
    return f"{prefix}{spec.family}__{axes}__{prompt_hash}"


# --- rebuilding a recorded composition -------------------------------------

def load_composition(composition_id, records=None, staging=None, cards=None):
    """Rebuild the Tree a recorded composition produced.

    This is what `controllers.make("composed:<id>")` calls, and it is why a
    composed tree runs under `run_episode.py` with no runner change and nothing
    registered by hand.

    The library is reloaded and its snapshot re-checked against the record. A
    mismatch is fatal: non-negotiable #8 says a reported result names the
    library it came from, and rebuilding a tree against a library that has grown
    since would produce a result whose named snapshot is a lie.
    """
    records = Path(records or RECORDS)
    path = records / f"{composition_id}.json"
    if not path.exists():
        raise CompositionError(
            "NO_SUCH_COMPOSITION",
            f"{path} does not exist. Compositions are committed; a missing one "
            f"means the id is wrong or the record was never written.")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not data.get("ok"):
        raise CompositionError(
            "COMPOSITION_FAILED",
            f"{composition_id} was recorded as {data['reason']} and never "
            f"produced a tree")

    library = load_library(staging=staging,
                           include_toys=data.get("include_toys", False),
                           cards=cards)
    if library.snapshot() != data["library_snapshot"]:
        raise CompositionError(
            "SNAPSHOT_MISMATCH",
            f"the record names {data['library_snapshot']} and the library "
            f"loads as {library.snapshot()}. The library is append-only (#7), "
            f"so it has grown; re-compose against the current snapshot rather "
            f"than running an old plan under a new tag.")

    retrieved = data["retrieval"]["retrieved"]
    retrieval = retrieve_mod.Retrieval(
        entries=[library.get(node_id) for node_id in retrieved],
        query_tags=tuple(data["retrieval"]["query_tags"]),
        library_size=data["retrieval"]["library_size"])
    return build(data["plan"], retrieval, snapshot=data["library_snapshot"])


def available(records=None):
    """The ids of every successfully composed tree on disk, sorted."""
    records = Path(records or RECORDS)
    if not records.is_dir():
        return []
    out = []
    for path in sorted(records.glob("*.json")):
        if json.loads(path.read_text(encoding="utf-8")).get("ok"):
            out.append(path.stem)
    return out
