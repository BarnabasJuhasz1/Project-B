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

WHAT THIS CHUNK CANNOT ADMIT YET, AND WHY THAT IS A FINDING
------------------------------------------------------------
Only **Actions** can be judged today, and a Condition or Decorator candidate is
rejected with `KIND_NOT_TESTABLE_ALONE` rather than quietly passed.

The reason is structural, not lazy. A unit scenario measures a navigation metric
over a rollout, and a metric can only move if something emits a Twist. An Action
does. A Condition publishes a Flag and a Decorator governs a child — neither
moves the robot, so neither can move `goal_reached` or `collisions` on its own.
Testing one means pairing it with a fixed consumer and comparing against the
same tree with the candidate replaced by a stand-in, which is a composition, and
composition is chunk 16. All five pilot cards produce Actions, so nothing is
blocked today; the rejection reason is what hands chunk 16 the problem stated
precisely instead of leaving it to be rediscovered.
"""

import ast
import dataclasses
import json
from pathlib import Path

from cards.store import CONTROLLER_INDEPENDENT, metric_keys
from harness.unit import UnitScenario, run_unit
from nodes.contract import (Action, Condition, Confidence, Decorator,
                            Direction, Kind, NodeMetadata, Origin, Port,
                            PortType, Status)
from nodes.toys import toy_metadata
from nodes.tree import Sequence, Tree, WiringError

from generate.generator import STAGING

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

    micro_fn = namespace.get("micro")
    if not callable(micro_fn):
        raise Rejected("NO_MICRO",
                       "micro() is missing — there is no world to test on")

    return node_cls, prediction, micro_fn, namespace


def _node_metadata(candidate_meta, node_id, card=None):
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

    origin = Origin.CARD if card is not None else Origin.INVENTED
    return NodeMetadata(
        node_id=node_id,
        origin=origin,
        card_id=card.id if card else None,
        # The card's span, not the citation. Empty while the card is
        # unverified, which is what makes the node untraceable — see
        # cards/VERIFICATION.md. Substituting the citation here would turn G1's
        # provenance fraction into a measure of fluency, invisibly.
        source_span=(card.source.span or None) if card else None,
        confidence=Confidence(card.confidence.value) if card else None,
        # `card.family` is the card's own mechanism_family, defaulting to its
        # node_name. None for A and D: an invented principle pairs with nothing,
        # and NodeMetadata.family then falls back to the node_id, so every
        # invented node counts as its own mechanism.
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
    if node_cls.KIND is not Kind.ACTION:
        raise Rejected(
            "KIND_NOT_TESTABLE_ALONE",
            f"{node_cls.__name__} is a {node_cls.KIND.value}. A unit scenario "
            f"measures a navigation metric over a rollout, and only an Action "
            f"emits a Twist — a Condition or Decorator cannot move a metric by "
            f"itself. Testing one means pairing it with a fixed consumer, "
            f"which is a composition; see plans/todo/16-composition.md.")
    try:
        return Tree(Sequence(toy_metadata("unit_scaffold"), [
            ContractPorts(toy_metadata("unit_ports")),
            node_cls(metadata),
        ]))
    except WiringError as exc:
        raise Rejected("UNSATISFIED_PORT", str(exc)) from exc
    except ValueError as exc:
        raise Rejected("CONTRACT_VIOLATION", str(exc)) from exc


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
    """Did condition B restate its card's claim, or quietly substitute one?

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


# --- the decision ----------------------------------------------------------

def admit(candidate_meta, source, card=None, library=None, seeds=None):
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
        metadata = _node_metadata(candidate_meta, node_id, card)

        try:
            micro = micro_fn()
        except Exception as exc:                 # noqa: BLE001
            raise Rejected("MALFORMED_MICRO",
                           f"micro() raised {type(exc).__name__}: {exc}") from exc

        if card is not None:
            extra["prediction_restated_faithfully"] = _prediction_matches_card(
                prediction, card)
            # The card is authoritative: the metric, the baseline and the
            # minimum effect are the corpus's claim, not the model's restatement
            # of it. Otherwise condition B could pass by predicting something
            # easier than the card did.
            unit = card.unit_scenario(micro, node_id=node_id, seeds=seeds)
        else:
            unit = _unit_from_prediction(prediction, micro, node_id)
            if seeds is not None:
                unit = dataclasses.replace(unit, seeds=tuple(seeds))

        tree_factory = lambda: build_tree(node_cls, metadata)
        tree_factory()          # fail here, as a rejection, not inside run_unit

        result = run_unit(tree_factory, unit)

    except Rejected as exc:
        return AdmissionResult(node_id, False, exc.reason, exc.detail,
                               metadata={**candidate_meta, **extra})

    outcome = AdmissionResult(
        node_id, result.admitted,
        "admitted" if result.admitted else "UNIT_SCENARIO_FAILED",
        result.reason, unit=result,
        metadata={**candidate_meta, **extra})

    if outcome.admitted and library is not None:
        library.add(node_cls, metadata)
    return outcome


def admit_staged(stem, library=None, staging=None, cards=None, seeds=None):
    """Judge a candidate already on disk, by the stem `Candidate.write` used.

    The result is written back beside it, so `generate/staging/` is a complete
    record of what the generator produced and what happened to it — which is
    what G1's pass rate is computed from.
    """
    staging = Path(staging or STAGING)
    meta = json.loads((staging / f"{stem}.json").read_text(encoding="utf-8"))
    source = (staging / f"{stem}.py").read_text(encoding="utf-8")

    card = None
    if meta.get("card_id"):
        if cards is None:
            from cards.store import CardStore
            cards = CardStore.load()
        card = cards.get(meta["card_id"])

    result = admit(meta, source, card=card, library=library, seeds=seeds)
    (staging / f"{stem}.result.json").write_text(
        json.dumps(result.as_dict(), indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return result


def pass_rate(staging=None, cards=None):
    """G1's unit pass rate, per condition, computed from what is on disk.

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
    for path in sorted(staging.glob("*.result.json")):
        result = json.loads(path.read_text(encoding="utf-8"))
        meta = result["metadata"]
        card_id = meta.get("card_id")
        if card_id and cards is not None and card_id not in cards:
            excluded.append(card_id)
            continue
        # A re-recorded generation. The old record stays on disk - it is real
        # evidence and the denominator may not be edited after the fact - but
        # counting both would report one mechanism as two admitted primitives,
        # which is exactly the number G1 is a coverage claim about.
        if meta.get("superseded_by"):
            superseded.append(f"{path.stem} -> {meta['superseded_by']}")
            continue
        condition = meta.get("condition", "?")
        bucket = by_condition.setdefault(
            condition, {"admitted": 0, "rejected": 0, "reasons": {}})
        if result["admitted"]:
            bucket["admitted"] += 1
        else:
            bucket["rejected"] += 1
            reason = result["reason"]
            bucket["reasons"][reason] = bucket["reasons"].get(reason, 0) + 1
    for bucket in by_condition.values():
        total = bucket["admitted"] + bucket["rejected"]
        bucket["pass_rate"] = bucket["admitted"] / total if total else 0.0
    if excluded:
        by_condition["_excluded_fixtures"] = sorted(excluded)
    if superseded:
        by_condition["_superseded"] = sorted(superseded)
    return by_condition
