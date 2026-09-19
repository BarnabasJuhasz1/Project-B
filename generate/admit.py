"""
generate/admit.py

Staged candidate -> the chunk-13 harness -> the library, or a recorded
rejection.

Non-negotiable #5: a node is admissible only if it passes its unit scenario in
isolation, pass or fail decided automatically. Nothing in this file asks a human
anything, and nothing in it can be talked round by a well-written docstring.

**A REJECTION IS AN OUTCOME, NOT AN ERROR.** Every rejected candidate is kept,
with its reason, in `generate/staging/`. G1 reports a unit pass rate and a pass
rate needs a denominator; and a rejected node is the most informative artefact
the pipeline produces — it says what the generator got wrong, in a form that can
be counted per condition.

That has a consequence for the exception handling, and it is not a style
choice: **`admit()` must never let a candidate's fault escape as an
exception.** An exception propagates to `batch.run()`, which counts the row
`failed` and writes no result — so the candidate is generated, never judged, and
absent from the pass rate entirely. `Rejected` covers every failure mode someone
anticipated; the catch-all in `admit()` covers the ones nobody did, under the
reason `CANDIDATE_RAISED`, with the traceback kept so the record can be
attributed. See the clause itself for the caveat.

THE IMPORT ALLOWLIST IS THE LOAD-BEARING CHECK
-----------------------------------------------
A candidate may import `math`, `numpy`, `core.observation`, `nodes.contract` and
`harness.micro`. Nothing else. Three things fall out of one rule:

- **"No coupling to a specific scenario" becomes mechanical.** A node that
  cannot import `core.scenario` cannot name lab42, cannot read the dev set, and
  cannot reach the sealed split however it is prompted. Chunk 11's AST guard
  covers `generate/`'s own source; this covers the code `generate/` produces,
  which the guard cannot see because it does not exist when the guard runs.
- **Ground truth stays out.** `core.geometry` is the map — occupancy grids,
  raycasts against the true world. A node that could import it could read the
  answer instead of sensing it, and would pass its unit scenario brilliantly.
- **It bounds what the executed code can reach.** Which brings us to:

⚠️ **THIS FILE EXECUTES MODEL-WRITTEN PYTHON, AND THERE IS NO SANDBOX.** The
allowlist is a static AST check applied before the first byte runs, so it stops
`import os` and it stops a candidate reaching the filesystem through an import.
It is not a defence against a determined adversary — `exec` of untrusted code
never is — and it is not claimed to be one. It is a defence against a model
confidently doing something careless, which is the failure mode that actually
occurs. Stated here rather than discovered later.

WHAT ADMISSION ASKS, AND WHAT IT STOPPED ASKING
------------------------------------------------
⚠️ **This section replaces the `KIND_NOT_TESTABLE_ALONE` note that stood here
until chunk 37.** That rejection was correct about efficacy and wrong as a gate:
only an Action moves a navigation metric alone, so gating on a moved metric made
two of the description's three leaf kinds unreachable and left every published
port orphaned. It is gone.

Admission now asks **one question: does this node RUN?** It builds, its ports
are satisfied, and it survives a rollout in a world `harness.micro` owns. That
is a VALIDITY gate, it is stated as one in non-negotiable #5, and it is not
evidence the node does anything.

**Whether it HELPS is measured somewhere else, on a tree, against the shared
base**, by `scripts/measure_growth.py` over `core.worldgen`'s suite — chunk 44,
promoted to non-negotiable #9. The candidate's own micro-world used to carry
that job and no longer does: the generator writes both the mechanism and the
world that judges it, and of seven measured contributions none cleared the floor
while three worlds came back malformed.

CHUNK 52: A MODIFIER IS ADMITTED IN THE TREE IT WILL LIVE IN
-------------------------------------------------------------
A candidate is now usually a **modifier** — an `Action` whose `WRITES` is
narrowed to the Twist fields it may revise. It cannot drive; it adjusts a
command a driver proposed. Alone it has nothing to revise and `PortView` raises,
so `build_tree` puts it where it belongs:

    Layer[ ContractPorts, BASE, modifier ]

The base is the shared, frozen substrate, identical in every arm, so it cancels
from B-A here exactly as it does in the growth loop. `PortView`'s field scope
does the rest: a candidate that declared `("vx",)` and writes `wz` is rejected
by the contract rather than by a reviewer noticing.
"""

import ast
import dataclasses
import json
import traceback
from pathlib import Path

from cards.store import CONTROLLER_INDEPENDENT, metric_keys
from harness.unit import UnitScenario, run_unit, run_validity
from nodes.contract import (Action, Condition, Confidence, Decorator,
                            Direction, Kind, NodeMetadata, Origin, Port,
                            PortType, Status)
from nodes.toys import toy_metadata
from nodes.tree import Layer, Sequence, Tree, WiringError

from generate.generator import PIPELINE_VERSION, STAGING

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# What a candidate may import. See the module docstring — this is the check the
# other guarantees hang off.
ALLOWED_IMPORTS = frozenset({
    "math", "numpy",
    "core.observation",
    "nodes.contract",
    "harness.micro",
})

# The three leaf base classes a candidate may subclass, by the name it will use.
BASE_NAMES = {"Condition": Condition, "Action": Action, "Decorator": Decorator}

# Ports the harness will supply to a candidate that declares them as inputs.
# Deliberately tiny, and every one of them is a RESTATEMENT OF `Observation`
# with no navigation decision in it: `goal_known` is `obs.goal is not None`,
# `pose` is `obs.pose_estimate`, `ranges` is `obs.ranges`.
#
# The restraint is the point. A provider that computed, say, a bearing to the
# nearest gap would be doing the node's job for it, identically across every
# condition, and every generated node would inherit a hand-written mechanism
# nobody pre-registered. An input the harness cannot supply this cheaply is a
# rejection, not a new provider.
PROVIDED_PORTS = {
    "goal_known": PortType.FLAG,
    "pose": PortType.POSE,
    "ranges": PortType.RANGES,
}


class Rejected(Exception):
    """A candidate that was judged and found wanting. Carries the machine
    readable reason so rejections can be counted by kind per condition."""

    def __init__(self, reason, detail=""):
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason
        self.detail = detail


class AdmissionResult:
    """What happened to one candidate. Written next to it in staging."""

    def __init__(self, node_id, admitted, reason, detail="", unit=None,
                 metadata=None):
        self.node_id = node_id
        self.admitted = admitted
        self.reason = reason
        self.detail = detail
        self.unit = unit
        self.metadata = metadata or {}

    def __bool__(self):
        return self.admitted

    def as_dict(self):
        out = {
            "node_id": self.node_id,
            "admitted": self.admitted,
            "reason": self.reason,
            "detail": self.detail,
            "metadata": self.metadata,
        }
        if self.unit is not None:
            out["unit"] = {
                "metric": self.unit.metric,
                "candidate": self.unit.candidate,
                "baseline": self.unit.baseline,
                "effect": self.unit.effect,
                "required": self.unit.required,
            }
        return out

    def __repr__(self):
        verdict = "ADMITTED" if self.admitted else f"REJECTED({self.reason})"
        return f"<{self.node_id} {verdict}>"


# --- the static half: everything decidable without running anything --------

def _imported_modules(tree):
    """Every module name the source imports, at any depth.

    `ast.walk` rather than `tree.body`, because an import inside a function is
    still an import — and it is exactly where a candidate would put one if it
    were trying not to be noticed.
    """
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                names.add("." * node.level + (node.module or ""))
            else:
                names.add(node.module or "")
    return names


def _allowed(module):
    """`numpy.linalg` is allowed by `numpy`; `core.scenario` is not by `core`.

    Prefix matching on dotted components, never on characters — otherwise
    `core.observation_hack` would pass as a prefix of nothing and
    `mathematics` would pass as a prefix of `math`.
    """
    parts = module.split(".")
    return any(".".join(parts[:i]) in ALLOWED_IMPORTS
               for i in range(1, len(parts) + 1))


def check_source(source):
    """Parse, check imports, and find the one node class. Nothing runs yet."""
    try:
        tree = ast.parse(source)
    except SyntaxError as exc:
        raise Rejected("SYNTAX_ERROR", f"line {exc.lineno}: {exc.msg}") from exc

    denied = sorted(m for m in _imported_modules(tree) if not _allowed(m))
    if denied:
        raise Rejected(
            "IMPORT_DENIED",
            f"{denied} — a candidate may import only "
            f"{sorted(ALLOWED_IMPORTS)}. See generate/admit.py on why this is "
            f"the check the other guarantees hang off.")

    classes = [node for node in tree.body
               if isinstance(node, ast.ClassDef)
               and any(isinstance(b, ast.Name) and b.id in BASE_NAMES
                       for b in node.bases)]
    if not classes:
        raise Rejected(
            "NO_NODE",
            "no class subclassing Condition, Action or Decorator at module "
            "level")
    if len(classes) > 1:
        raise Rejected(
            "MULTIPLE_NODES",
            f"{[c.name for c in classes]} — one card, one call, one node. Two "
            f"nodes in one candidate makes a source span attributable to "
            f"neither, and G1's provenance fraction is a fraction of nodes.")
    return classes[0].name


def load_candidate(source, module_name="candidate"):
    """Execute the candidate and hand back what it defined.

    `check_source` has already run; this raises for what only running can find.
    """
    class_name = check_source(source)
    namespace = {"__name__": module_name, "__builtins__": __builtins__}
    try:
        exec(compile(source, f"<{module_name}>", "exec"), namespace)  # noqa: S102
    except Exception as exc:                     # noqa: BLE001 — recorded, not raised
        raise Rejected("IMPORT_TIME_ERROR",
                       f"{type(exc).__name__}: {exc}") from exc

    node_cls = namespace.get(class_name)
    if not (isinstance(node_cls, type)
            and issubclass(node_cls, (Condition, Action, Decorator))):
        raise Rejected("NO_NODE",
                       f"{class_name} is not a Condition, Action or Decorator "
                       f"once the module has run")

    prediction = namespace.get("PREDICTION")
    if not isinstance(prediction, dict):
        raise Rejected(
            "NO_PREDICTION",
            "PREDICTION is missing or is not a dict. A candidate that makes no "
            "falsifiable claim cannot be admitted or rejected automatically, "
            "which is the one thing §4 requires of this step.")

    # ⚠️ OPTIONAL SINCE CHUNK 52, AND THAT IS A DELIBERATE WEAKENING.
    #
    # A micro-world was the place a node's claimed effect was measured. Two
    # findings retired it. Chunk 37 measured that the generator writes both the
    # mechanism AND the world that judges it, so 3 of 7 measured contributions
    # came back MALFORMED and none cleared the floor. Chunk 44 then moved
    # contribution to the TREE, on a shared suite nobody generating a node has
    # seen — which is #9 promoted to the admission criterion.
    #
    # Under the layered pipeline a modifier cannot drive alone anyway: its
    # micro-world would have to contain the shared base to mean anything, and a
    # world the model wrote around a base it was never shown is not evidence.
    # So the modifier prompt does not ask for one, `contribution` records
    # `measured: False` with a reason, and `scripts/measure_growth.py` is where
    # the number now comes from. A candidate that supplies a micro() anyway is
    # still measured on it — nothing is thrown away.
    micro_fn = namespace.get("micro")
    if micro_fn is not None and not callable(micro_fn):
        raise Rejected("NO_MICRO",
                       f"micro is a {type(micro_fn).__name__}, not a callable")

    return node_cls, prediction, micro_fn, namespace


def _node_metadata(candidate_meta, node_id, card=None, citation_of=None,
                   generated_card=None):
    """Provenance, built by the pipeline rather than by the model.

    Chunk 12 left `model_id`, `prompt_hash` and `temperature` optional and said
    this chunk would enforce them, "because it is the chunk that knows what
    generated the node". This is that enforcement.
    """
    for field in ("model_id", "prompt_hash"):
        if not candidate_meta.get(field):
            raise Rejected(
                "PROVENANCE_INCOMPLETE",
                f"{field} is missing. A library that cannot say which model "
                f"produced it cannot be regenerated.")
    if "temperature" not in candidate_meta:
        raise Rejected("PROVENANCE_INCOMPLETE", "temperature was not recorded")
    if (candidate_meta["temperature"] is None
            and not candidate_meta.get("sampling_note")):
        raise Rejected(
            "PROVENANCE_INCOMPLETE",
            "temperature is None with no sampling_note saying why. None is "
            "ambiguous on its own — 'the model does not accept the parameter' "
            "and 'nobody recorded it' are different facts and only one of them "
            "is honest.")

    if card is not None and citation_of is not None:
        raise Rejected(
            "PROVENANCE_INCOMPLETE",
            "a candidate was handed both a card and a citation. Those are "
            "conditions B and A; one node cannot be both.")

    # THREE ORIGINS, THREE DIFFERENT CLAIMS ABOUT THE SAME STORE ENTRY.
    #   CARD     — the extracted content reached the model (B)
    #   CITATION — only the reference did; the mechanism is the model's own
    #              recall wearing the paper's name (A)
    #   INVENTED — no corpus at all (D)
    # Everything a human produced by reading the paper — the span, the
    # extraction confidence, the mechanism family — belongs to CARD alone. A
    # citation node inherits NONE of it, and `nodes.contract.NodeMetadata`
    # raises if one is set anyway, so this cannot be got wrong quietly.
    if card is not None:
        origin = Origin.CARD
    elif citation_of is not None:
        origin = Origin.CITATION
    else:
        origin = Origin.INVENTED
    # WHOSE SPAN AND CONFIDENCE THESE ARE — CHUNK 42 MOVED THEM.
    #
    # They used to come from the HUMAN card, because before 42 that card was the
    # node's input. It no longer is: every arm writes its own card and the node
    # is implemented from that. Carrying the human's verified span onto a node
    # built from a model-written one would hand the arm provenance it did not
    # earn — the exact failure `Origin.CITATION` was invented to prevent, one
    # step further down the pipeline.
    #
    # So the node carries the GENERATED card's claim, and `span_verified` stays
    # False because nobody has checked it. For B that claim has an answer key —
    # B read the document, and `cards/VERIFICATION.md` says where the mechanism
    # actually is — and scoring the agreement is a follow-up, not this chunk.
    # For A it is a claim about a paper A never saw, which is the confabulation
    # the provenance axis exists to detect and is worth recording for that
    # reason alone.
    span = confidence = None
    span_verified = False
    if generated_card is not None:
        span = generated_card.source.span or None
        confidence = Confidence(generated_card.confidence.value)
    elif card is not None:
        # A PRE-CHUNK-42 RECORD, and it keeps the provenance it actually had.
        # These candidates were generated in ONE call from the human card, so
        # that card's span really is the node's provenance and its verification
        # really does apply — `is_traceable` was true for them and stays true.
        # Rewriting them to the new rule would restate history: it would report
        # a node as untraceable when the traceable extraction is precisely what
        # produced it. The new accounting binds the new pipeline, and
        # `pass_rate` refuses to pool the two.
        span = card.source.span or None
        confidence = Confidence(card.confidence.value)
        span_verified = bool(card.is_traceable)
    return NodeMetadata(
        node_id=node_id,
        origin=origin,
        card_id=card.id if card else (citation_of.id if citation_of else None),
        generated_card_id=generated_card.id if generated_card else None,
        source_span=span,
        confidence=confidence,
        # False for every card the MODEL wrote, which is every card the current
        # pipeline produces. See nodes.contract.NodeMetadata.is_traceable for
        # what that costs and why the cost is the honest answer.
        span_verified=span_verified,
        # `card.family` is the card's own mechanism_family, defaulting to its
        # node_name. None for A and D: an invented principle pairs with nothing,
        # and NodeMetadata.family then falls back to the node_id, so every
        # invented node counts as its own mechanism. A citation node is the same
        # case — the family is an extraction decision it was never shown.
        mechanism_family=card.family if card else None,
        model_id=candidate_meta["model_id"],
        prompt_hash=candidate_meta["prompt_hash"],
        temperature=candidate_meta["temperature"],
    )


# --- the tree the candidate is tested in -----------------------------------

class ContractPorts(Condition):
    """Publishes the handful of ports that are pure restatements of
    `Observation`, so that a candidate declaring one can be tested alone.

    HANDWRITTEN, and it is not navigation: there is no decision anywhere in
    `tick`, only three assignments. See PROVIDED_PORTS on why it must stay that
    way. It is identical across every condition, so it cannot differentiate
    them.
    """

    PORTS = tuple(Port(name, port_type, Direction.OUT)
                  for name, port_type in PROVIDED_PORTS.items())
    PRECONDITION = "always"
    EFFECT = "publishes goal_known, pose and ranges straight from Observation"
    TAGS = ("harness", "scaffold")

    def tick(self, obs, ports):
        ports["goal_known"] = obs.goal is not None
        ports["pose"] = obs.pose_estimate
        ports["ranges"] = obs.ranges
        return Status.SUCCESS


def build_tree(node_cls, metadata):
    """Wrap one candidate leaf in the smallest tree that is a Controller.

    `ContractPorts` ticks first so a candidate that declared `goal_known` can
    read it. A candidate declaring an input the harness does not provide fails
    `Tree`'s construction-time wiring check, and that is reported as a rejection
    rather than as a crash.
    """
    # CHUNK 37: A CONDITION OR DECORATOR IS NO LONGER REFUSED HERE.
    #
    # `KIND_NOT_TESTABLE_ALONE` was correct about efficacy and wrong as a gate:
    # only an Action moves a navigation metric alone, so gating on a moved
    # metric made two of the description's three leaf kinds unreachable. The
    # visible cost was that NOTHING IN THE LIBRARY READS A PORT — every
    # published output is orphaned, because every producer had to be an Action
    # that also drove. Admission now asks whether the node RUNS; whether it
    # HELPS is `harness/ablate.py`'s question, asked of a tree.
    #
    # A Decorator needs a child to be constructible at all, so it is given one.
    # `DriveToGoal` is a handwritten fixture and NOT NAVIGATION — it is here so
    # a decorator has something real to govern while being ticked, and it can
    # never reach a benchmark condition (#3), the same standing rule
    # `ContractPorts` above is under.
    #
    # CHUNK 52: A MODIFIER IS VALIDATED IN A LAYER, ON THE SHARED BASE.
    #
    # A modifier declares a narrowed `WRITES` and revises a command someone else
    # proposed. Alone it has nothing to revise, `PortView.cap` raises, and the
    # node would be rejected for the shape that makes it a modifier. So it is
    # wrapped in the tree it is actually going to live in:
    #
    #     Layer[ ContractPorts, BASE, modifier ]
    #
    # Using the real base rather than a toy driver is the point: it is the same
    # substrate in every arm, so it cancels from B-A exactly as it does in the
    # growth loop, and admission asks "does this run in the context it will run
    # in" rather than "does it run beside a fixture". The base is frozen
    # (`generate/base.py`), so this coupling cannot move underneath a recorded
    # admission.
    #
    # ⚠️ EVERY candidate gets the base, not just the ones a narrowed `WRITES`
    # identifies as modifiers. Gating on `is_modifier` was tried first and was
    # wrong within the hour: two generated nodes declared all three fields (a
    # dynamic window over vx/wz that also pins vy to zero) and then revised the
    # pending command with `cap`/`adjust` like any modifier. They got the
    # baseless tree, read a command nobody had proposed, and were rejected for
    # the harness's shape rather than their own. Whether a node drives or
    # revises is a fact about its `tick`, not about its declaration, and the
    # layered tree is correct for both: a driver placed after the base simply
    # overwrites the whole command, which is exactly what it does in the growth
    # tree that measures it.
    #
    # The base itself is the one exception — it IS the base, and nesting it
    # inside a tree with itself would tick it twice.
    with_base = metadata.origin is not Origin.SUBSTRATE
    try:
        if node_cls.KIND is Kind.DECORATOR:
            from nodes.toys import DriveToGoal
            leaf = node_cls(metadata, DriveToGoal(toy_metadata("unit_child")))
        else:
            leaf = node_cls(metadata)
        if with_base:
            return Tree(Layer(toy_metadata("unit_layer"), [
                ContractPorts(toy_metadata("unit_ports")),
                _base_leaf(),
                leaf,
            ]))
        return Tree(Sequence(toy_metadata("unit_scaffold"), [
            ContractPorts(toy_metadata("unit_ports")),
            leaf,
        ]))
    except WiringError as exc:
        raise Rejected("UNSATISFIED_PORT", str(exc)) from exc
    except ValueError as exc:
        raise Rejected("CONTRACT_VIOLATION", str(exc)) from exc


def _base_leaf():
    """The shared base node, as a leaf. Imported lazily and on purpose.

    `generate.base` imports `ContractPorts`, `build_tree` and `check_source`
    from this module, so a module-level import here is a cycle. It is also
    genuinely a runtime dependency rather than a structural one: admission only
    needs the base when the candidate turns out to be a modifier.
    """
    from generate import base as basemod
    loaded = basemod.load()
    return loaded.node_cls(loaded.metadata)


# --- the prediction --------------------------------------------------------

def _unit_from_prediction(prediction, micro, node_id):
    """Conditions A and D supply their own claim; it still has to be one."""
    for field in ("metric", "baseline"):
        if not str(prediction.get(field, "")).strip():
            raise Rejected("NO_PREDICTION", f"PREDICTION.{field} is empty")

    metric = prediction["metric"]
    if metric not in metric_keys():
        raise Rejected(
            "UNSCOREABLE_PREDICTION",
            f"{metric!r} is not produced by core.metrics.score(); known: "
            f"{sorted(metric_keys())}")
    if metric in CONTROLLER_INDEPENDENT:
        raise Rejected(
            "UNSCOREABLE_PREDICTION",
            f"{metric!r} cannot be moved by a controller, so a prediction "
            f"about it is not falsifiable about the node — see "
            f"cards.store.CONTROLLER_INDEPENDENT.")

    return UnitScenario(
        node_id=node_id, micro=micro, metric=metric,
        baseline=prediction["baseline"],
        min_effect=float(prediction.get("min_effect", 0.0) or 0.0),
        note=str(prediction.get("cost_metric", "")))


def _prediction_matches_card(prediction, card):
    """Did the node restate its card's claim, or quietly substitute one?

    Asked of every arm since chunk 42, because every arm now has a card its node
    was written from. It used to be asked of B alone.

    Recorded, NOT fatal. The card's prediction is authoritative either way, so a
    drifted restatement changes nothing about what gets tested; and rejecting on
    it would conflate "the mechanism does not work" with "the model copied a
    number wrong", which are different findings and G1's pass rate is about the
    first. Whether B restates its card faithfully is worth a number of its own.
    """
    p = card.prediction
    return (prediction.get("metric") == p.metric
            and prediction.get("baseline") == p.baseline
            and float(prediction.get("min_effect", 0.0) or 0.0) == p.min_effect)


# --- the measurement, which decides nothing --------------------------------

def _measure_contribution(prediction, micro_fn, card, seeds, node_id,
                          tree_factory, extra):
    """Run the candidate's own unit scenario and RECORD what it said.

    **This function cannot reject anything**, and that is the whole of chunk 37.
    Every path returns; a candidate whose world is malformed, whose prediction
    names no scoreable metric, or whose effect is zero is recorded as such and
    still admitted on validity.

    The effect floor is kept and reported as `required`. It is a **reporting
    line**, not a threshold — `harness/unit.py`'s docstring says why it stopped
    being one.
    """
    if micro_fn is None:
        return None, {
            "measured": False,
            "reason": "no micro() — the modifier prompt does not ask for one. "
                      "Contribution is measured on the shared growth suite by "
                      "scripts/measure_growth.py, against the same base, which "
                      "is chunk 44's decision and non-negotiable #9."}
    try:
        micro = micro_fn()
    except Exception as exc:                     # noqa: BLE001
        return None, {"measured": False,
                      "reason": f"micro() raised {type(exc).__name__}: {exc}"}

    # Explicitly, rather than letting `run_unit` fold it into a UnitResult: a
    # malformed world means the contribution is UNMEASURED, which is a different
    # fact from an effect of zero, and the two were previously one reason code.
    try:
        micro.validate()
    except ValueError as exc:
        return None, {"measured": False,
                      "reason": f"the candidate's own micro-world is "
                                f"malformed: {exc}"}

    # WHICH CARD IS AUTHORITATIVE — CHUNK 42 MADE THE ARMS SYMMETRIC.
    #
    # `card` here is the card the node was IMPLEMENTED FROM, and under 42 every
    # arm has one it wrote itself. So every arm is scored against its own
    # card's prediction, and the special case this function used to carry — a
    # citation node scored against its own PREDICTION while B was scored
    # against the human card's — is gone, because the asymmetry that made it
    # necessary is gone.
    #
    # The human-extracted card reaches nothing here. Scoring any arm against it
    # would hand that arm content it was never shown, which is what the old
    # warning was about; it is now impossible rather than guarded.
    try:
        if card is not None:
            extra["prediction_restated_faithfully"] = _prediction_matches_card(
                prediction, card)
            # The card is authoritative where there is one: the claim under
            # test is the card's, not the node's restatement of it.
            unit = card.unit_scenario(micro, node_id=node_id, seeds=seeds)
        else:
            unit = _unit_from_prediction(prediction, micro, node_id)
            if seeds is not None:
                unit = dataclasses.replace(unit, seeds=tuple(seeds))
    except Rejected as exc:
        return None, {"measured": False,
                      "reason": f"{exc.reason}: {exc.detail}"}

    try:
        result = run_unit(tree_factory, unit)
    except Exception as exc:                     # noqa: BLE001
        # The traceback, for the same reason CANDIDATE_RAISED keeps one: this
        # clause cannot tell a candidate's fault from a harness bug hit while
        # measuring one, and the frame that raised is what lets a reader decide.
        return None, {
            "measured": False,
            "reason": f"run_unit raised {type(exc).__name__}: {exc}\n\n"
                      f"{traceback.format_exc()}"}

    return result, {
        "measured": True,
        "metric": result.metric,
        "candidate": result.candidate,
        "baseline": result.baseline,
        "effect": result.effect,
        "required": result.required,
        "cleared_floor": bool(result.admitted),
        "reason": result.reason,
    }


# --- the decision ----------------------------------------------------------

def admit(candidate_meta, source, card=None, library=None, seeds=None,
          citation_of=None, generated_card=None):
    """Judge one staged candidate. Returns an AdmissionResult; never raises for
    a candidate's own faults."""
    node_id = candidate_meta["node_id"]
    extra = {}

    # The model emitted no code. Not a crash and not a harness failure: it is a
    # finding about the card, and it belongs in G1's denominator alongside every
    # other rejection. See generate/generator.extract_code.
    if source is None:
        return AdmissionResult(
            node_id, False, "NO_CANDIDATE_EMITTED",
            "the model returned no fenced python block; the staged .refusal.md "
            "is what it returned instead",
            metadata=candidate_meta)

    try:
        node_cls, prediction, micro_fn, _ = load_candidate(source, node_id)
        metadata = _node_metadata(candidate_meta, node_id, card,
                                  citation_of=citation_of,
                                  generated_card=generated_card)
        # Record the provenance the node was actually given, in the staged
        # result. Without this the origin exists only inside the library, so a
        # rejected candidate has no recorded provenance at all and G1's
        # fraction cannot be recomputed from `generate/staging/` — which is the
        # complete record it is supposed to be. Additive: older records simply
        # do not carry these keys.
        extra["origin"] = metadata.origin.value
        extra["is_traceable"] = bool(metadata.is_traceable)
        extra["mechanism_family"] = metadata.mechanism_family
        extra["source_span"] = metadata.source_span
        extra["confidence"] = (metadata.confidence.value
                               if metadata.confidence else None)
        extra["generated_card_id"] = metadata.generated_card_id
        extra["span_verified"] = bool(metadata.span_verified)

        tree_factory = lambda: build_tree(node_cls, metadata)
        # Built once here so that UNSATISFIED_PORT and CONTRACT_VIOLATION keep
        # their own reason codes. `run_validity` would report them as a generic
        # INVALID_NODE, and "this node declares a port nothing provides" is a
        # far more actionable rejection than "it would not build".
        tree_factory()

        # ---- THE GATE (chunk 37): does this node RUN? ----------------------
        # In a world `harness.micro` owns, NEVER the candidate's. A generator
        # that writes a broken world would otherwise fail a node that is fine —
        # which is what happened to two of condition D's four candidates, both
        # filed as UNIT_SCENARIO_FAILED without a node ever being ticked.
        validity = run_validity(tree_factory, node_id)
        extra["validity"] = {"valid": validity.valid, "reason": validity.reason}
        # Diagnostics, never criteria. They separate "did not help" from "never
        # fired", which the old single number could not.
        extra["precondition_fired"] = {
            "ticks": validity.ticks,
            "passed": validity.precondition_passed,
            "drove": validity.drove,
            "fraction": validity.fired_fraction,
        }
        if not validity.valid:
            raise Rejected("INVALID_NODE", validity.reason)

        # ---- THE MEASUREMENT: recorded, and it decides nothing -------------
        # The GENERATED card, never the corpus entry. See _measure_contribution.
        result, extra["contribution"] = _measure_contribution(
            prediction, micro_fn, generated_card, seeds, node_id, tree_factory,
            extra)

    except Rejected as exc:
        return AdmissionResult(node_id, False, exc.reason, exc.detail,
                               metadata={**candidate_meta, **extra})
    except Exception as exc:                     # noqa: BLE001 — recorded, not raised
        # A CANDIDATE FAULT THAT NOBODY ANTICIPATED IS STILL A CANDIDATE FAULT.
        # Everything above raises `Rejected` for a failure mode someone thought
        # of in advance; this catches the ones nobody did. It exists because one
        # did occur: `b/navigate_to_goal/path_integration_ants` built its
        # micro-world with `drift=0.6` -- a bare float where a `DriftSpec`
        # belongs -- which `MicroScenario` accepts (the field is typed `object`)
        # and `validate()` does not type, so it survived every check here and
        # died five frames down in `core.drift.DriftModel.__init__`.
        #
        # WITHOUT THIS CLAUSE THE CANDIDATE LEAVES G1's DENOMINATOR ALTOGETHER.
        # The exception propagated to `batch.run()`'s generic handler, which
        # counts a row `failed` and writes no `.result.json` -- so the candidate
        # was generated, was never judged, and appeared in no pass rate. A
        # malformed micro-world is a REJECTION: the generator was asked to write
        # a node and a world to test it in, and it wrote a world that does not
        # run. Being wrong has to be counted, and it is counted here.
        #
        # ⚠️ THE HONEST CAVEAT: this clause cannot tell a candidate's fault from
        # a HARNESS BUG raised while judging one, and it will record the second
        # as the first. That is why the full traceback goes into `detail` rather
        # than just the message -- the record names the file and line that
        # raised, so a reader can attribute it. A run of `CANDIDATE_RAISED`
        # rejections all pointing at the same harness frame is a bug report, not
        # a pass rate, and the traceback is what makes that visible.
        return AdmissionResult(
            node_id, False, "CANDIDATE_RAISED",
            f"{type(exc).__name__}: {exc}\n\n{traceback.format_exc()}",
            metadata={**candidate_meta, **extra})

    # Valid is admitted. Whether the node cleared the old effect floor is in
    # `metadata["contribution"]`, reported and not acted on.
    contribution = extra.get("contribution", {})
    if contribution.get("measured"):
        detail = (f"valid; {contribution['metric']} effect "
                  f"{contribution['effect']:.3f} against a reported floor of "
                  f"{contribution['required']:.3f}")
    else:
        detail = (f"valid; contribution not measured "
                  f"({contribution.get('reason', 'no reason recorded')})")

    outcome = AdmissionResult(node_id, True, "admitted", detail, unit=result,
                              metadata={**candidate_meta, **extra})

    if library is not None:
        library.add(node_cls, metadata)
    return outcome


def load_generated_card(stem, meta, directory=None):
    """The MODEL-WRITTEN card a staged candidate was implemented from.

    Filed under the same stem as the candidate, which is why
    `cardgen.GeneratedCard.stem` keys on the grid row key rather than on its own
    prompt hash: the two must always agree or a node would be judged against
    somebody else's card.

    None for a pre-chunk-42 record, and that is not an error — those candidates
    were generated in one call and never had a card of their own.
    `_node_metadata` then produces exactly the metadata it always did, minus a
    span it never legitimately had.
    """
    from generate import cardgen

    condition = meta.get("condition")
    if not condition:
        return None
    path = (Path(directory or cardgen.CARDS_DIR) / condition / f"{stem}.yaml")
    if not path.is_file():
        return None
    return cardgen.load_generated(path).card


def admit_staged(stem, library=None, staging=None, cards=None, seeds=None):
    """Judge a candidate already on disk, by the stem `Candidate.write` used.

    The result is written back beside it, so `generate/staging/` is a complete
    record of what the generator produced and what happened to it — which is
    what G1's pass rate is computed from.
    """
    staging = Path(staging or STAGING)
    meta = json.loads((staging / f"{stem}.json").read_text(encoding="utf-8"))
    source = (staging / f"{stem}.py").read_text(encoding="utf-8")

    card = citation_of = None
    if meta.get("card_id") or meta.get("citation_id"):
        if cards is None:
            from cards.store import CardStore
            cards = CardStore.load()
        if meta.get("card_id"):
            card = cards.get(meta["card_id"])
        else:
            citation_of = cards.get(meta["citation_id"])

    result = admit(meta, source, card=card, library=library, seeds=seeds,
                   citation_of=citation_of,
                   generated_card=load_generated_card(stem, meta))
    (staging / f"{stem}.result.json").write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return result


def pass_rate(staging=None, cards=None, pipeline_version=None):
    """G1's unit pass rate, per condition, computed from what is on disk.

    **ONE PIPELINE AT A TIME — chunk 42.** `pipeline_version` selects which
    generation pipeline's records to count, defaulting to the current one.
    Records from any other version are named under `_superseded_pipeline` and
    counted nowhere: a v1 candidate was generated in one call from a human card,
    a v2 candidate writes its own card first, and a rate averaged over both
    would be one number over two treatments wearing one condition's letter.
    Pass `pipeline_version="v1-single-call"` to read the historical records —
    they are still real evidence about a real question.

    Reported rather than asserted, and with the rejection reasons broken out:
    "12 of 20 admitted" and "12 of 20 admitted, 6 of the 8 rejections were
    malformed micro-worlds" are very different findings about a generator.

    **Fixture candidates are excluded, and the rule is mechanical**: a candidate
    whose `card_id` does not resolve in the card store was generated from a card
    that is not part of the corpus, so it is a test fixture and counting it
    would put a deliberately-bad card into the number G1 reports. The
    excluded ids are returned rather than silently dropped — a count that
    quietly ignores things is how a pass rate stops meaning anything.
    """
    staging = Path(staging or STAGING)
    if cards is None:
        from cards.store import CardStore
        try:
            cards = CardStore.load()
        except Exception:                        # noqa: BLE001
            cards = None

    by_condition = {}
    excluded = []
    superseded = []
    retired = []
    superseded_pipeline = []
    for path in sorted(staging.glob("*.result.json")):
        result = json.loads(path.read_text(encoding="utf-8"))
        meta = result["metadata"]
        # EITHER column. A condition-A candidate names its reference in
        # `citation_id`, not `card_id`, so checking only the latter would let a
        # candidate generated from a TEST FIXTURE into the number G1 reports —
        # which is exactly what this rule exists to prevent, and it would have
        # been invisible because the fixture ids look like real ones.
        ref_id = meta.get("card_id") or meta.get("citation_id")
        if ref_id and cards is not None and ref_id not in cards:
            excluded.append(ref_id)
            continue
        # A re-recorded generation. The old record stays on disk - it is real
        # evidence and the denominator may not be edited after the fact - but
        # counting both would report one mechanism as two admitted primitives,
        # which is exactly the number G1 is a coverage claim about.
        if meta.get("superseded_by"):
            superseded.append(f"{path.stem} -> {meta['superseded_by']}")
            continue
        # A record produced under a DEFINITION OF THE CONDITION that no longer
        # exists. Different from `superseded_by`, which is the same prompt run
        # again: here the prompt is not reachable at all any more, because the
        # condition it belonged to was redefined. Pooling it with the current
        # records would compute one pass rate over two different treatments and
        # label the result with the condition's letter, which is precisely the
        # confound the letters exist to prevent. The record stays on disk — it
        # is real evidence about a real question — and it is named here rather
        # than dropped silently.
        if meta.get("retired_reason"):
            retired.append(f"{path.stem}: {meta['retired_reason']}")
            continue
        # A record produced by an EARLIER GENERATION PIPELINE — chunk 42.
        #
        # The same argument as `retired_reason` one level up: every candidate
        # staged before 42 came from a ONE-CALL pipeline, where the model was
        # handed a human's card (B), a citation (A) or nothing (D) and wrote a
        # node straight from it. Those prompts are not reachable any more. Under
        # 42 each arm writes its own card first and a single shared node prompt
        # implements it, so a v1 and a v2 record with the same letter on them
        # were produced under different treatments and averaging the two would
        # compute one pass rate over both.
        #
        # Done MECHANICALLY rather than by hand-editing `retired_reason` into
        # every record, which is what 2026-09-06 did for four of them: the
        # version is stamped at generation time, so this cannot be forgotten for
        # a record and cannot be applied to the wrong one. Nothing is deleted
        # and nothing is rewritten — the v1 records stay on disk and are named
        # in the return value.
        version = meta.get("pipeline_version", "v1-single-call")
        if version != (pipeline_version or PIPELINE_VERSION):
            superseded_pipeline.append(f"{path.stem}: {version}")
            continue
        condition = meta.get("condition", "?")
        bucket = by_condition.setdefault(condition, {
            "admitted": 0, "rejected": 0, "reasons": {},
            # The OLD rule, kept alongside the new one rather than overwritten.
            # A redefinition that silently replaces its own denominator is
            # indistinguishable from an improvement, and this one is not an
            # improvement — see plans/doing/37.
            "legacy_admitted": 0, "legacy_rejected": 0,
            "contribution_measured": 0, "contribution_cleared_floor": 0,
        })

        # `admitted_v2` is written by scripts/rejudge.py for records taken under
        # the old rule; a record judged today carries only `admitted`, which
        # already means validity.
        verdict = result.get("admitted_v2", result["admitted"])
        if verdict:
            bucket["admitted"] += 1
        else:
            bucket["rejected"] += 1
            reason = result.get("reason_v2", result["reason"])
            bucket["reasons"][reason] = bucket["reasons"].get(reason, 0) + 1

        if result["admitted"]:
            bucket["legacy_admitted"] += 1
        else:
            bucket["legacy_rejected"] += 1

        contribution = meta.get("contribution") or {}
        if contribution.get("measured"):
            bucket["contribution_measured"] += 1
            if contribution.get("cleared_floor"):
                bucket["contribution_cleared_floor"] += 1

    for bucket in by_condition.values():
        total = bucket["admitted"] + bucket["rejected"]
        # G1's reported rate. Under chunk 37 this is a VALIDITY rate: the
        # fraction of candidates that run. It is not the old efficacy rate and
        # must never be reported as one.
        bucket["pass_rate"] = bucket["admitted"] / total if total else 0.0
        legacy_total = bucket["legacy_admitted"] + bucket["legacy_rejected"]
        bucket["legacy_pass_rate"] = (bucket["legacy_admitted"] / legacy_total
                                      if legacy_total else 0.0)
        measured = bucket["contribution_measured"]
        # Of the candidates whose own unit scenario could be run at all, how
        # many moved their claimed metric past the (now purely reported) floor.
        bucket["contribution_rate"] = (
            bucket["contribution_cleared_floor"] / measured if measured else None)
    if excluded:
        by_condition["_excluded_fixtures"] = sorted(excluded)
    if superseded:
        by_condition["_superseded"] = sorted(superseded)
    if retired:
        by_condition["_retired"] = sorted(retired)
    if superseded_pipeline:
        # Named, never dropped. A reader who expects 21 records and is shown 4
        # has to be able to see where the other 17 went, and "an earlier
        # pipeline produced them" is the answer.
        by_condition["_superseded_pipeline"] = sorted(superseded_pipeline)
    by_condition["_pipeline_version"] = pipeline_version or PIPELINE_VERSION
    return by_condition
