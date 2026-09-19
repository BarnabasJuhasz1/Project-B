"""
runs/spec.py

Which controller **is** condition A, B, C and D, written down and committed.

This is a small file carrying a large claim. `composed:9c21b87c` is a hash, not
a name: it does not say which condition produced the tree, which library
snapshot it came from, or whether anyone meant to evaluate it. A sweep that
took controller names off the command line would make "which artefact was
condition B" a fact about somebody's shell history.

So the binding is a committed JSON file, validated on load, and the sweep reads
it. The write-up cites it.

    from runs.spec import load_spec
    spec = load_spec()
    print([c.label for c in spec.conditions])

    python scripts/run_sweep.py --spec runs/specs/rehearsal.json --dry-run

Sim-free.
"""

import json
import os
from dataclasses import dataclass
from typing import Optional, Tuple

import controllers

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# The four conditions of G2, in the order the write-up presents them. A spec may
# omit one -- condition C does not exist yet, and PRE-REGISTRATION.md §9 says an
# absent condition is reported as absent rather than substituted for -- but it
# may not invent a fifth, because #10's identical-episode-count argument is
# stated over these four.
CONDITION_LABELS = ("A", "B", "C", "D")

# What each condition is, for the log and the report header. Not decoration:
# a reader of a sweep log should not have to open the description to know what
# B minus D is testing.
CONDITION_MEANING = {
    "A": "naive LLM, node contract given, no literature",
    "B": "literature-informed LLM, card store available",
    "C": "human baseline, off-the-shelf tuned Nav2",
    "D": "invented principles, same model, no corpus",
}

# Where a spec lives by default.
DEFAULT_SPEC = os.path.join("runs", "specs", "evaluation.json")


# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Condition:
    """One condition's binding to a runnable controller."""

    label: str
    controller: str
    library_snapshot: Optional[str] = None
    note: str = ""

    @property
    def meaning(self):
        return CONDITION_MEANING.get(self.label, "")


@dataclass(frozen=True)
class SweepSpec:
    conditions: Tuple[Condition, ...]
    name: str = ""
    reportable: bool = True

    def condition(self, label):
        for c in self.conditions:
            if c.label == label:
                return c
        raise KeyError(f"no condition {label!r} in spec {self.name!r}")


def load_spec(path=None):
    """Read and validate a sweep spec. Raises rather than warning.

    Every check here is one that would otherwise surface as a wrong number
    rather than as an error:

    - **A scaffolding controller is refused.** Non-negotiable #3 says
      scaffolding must never enter a benchmark condition, and
      `controllers.scaffolding()` is the list. A sweep that quietly evaluated
      `scaffold_sweep` as condition A would produce a complete, plausible,
      meaningless table.
    - **A `layer:<id>` binding is refused.** The growth pairing is the shared
      base plus one leaf: a probe for `scripts/measure_growth.py`, never an
      arm. The base is exempt from #3 as a declared substrate, which is exactly
      what makes it part of every arm and an arm by itself in none.
    - **A bare `node:<id>` binding is refused.** It resolves — that is chunk
      36's point — but it is one leaf plus `ContractPorts`, which is evidence
      about that leaf and never a condition.
    - **An unresolvable controller is refused before the simulator boots**, the
      same discipline `run_episode.py`'s argparse validators use.
    - **A duplicate controller across two conditions is refused.** Two
      conditions bound to the same artefact produce a gap of exactly zero, and
      the interesting question is whether that was intended.
    - **`library_snapshot` is required on a reportable spec** (#8).
    """
    path = DEFAULT_SPEC if path is None else path
    full = path if os.path.isabs(path) else os.path.join(REPO_ROOT, path)
    with open(full) as fh:
        raw = json.load(fh)

    name = raw.get("name") or os.path.basename(path)
    reportable = bool(raw.get("reportable", True))
    conditions = []
    seen = {}
    for entry in raw.get("conditions", []):
        label = entry["label"]
        if label not in CONDITION_LABELS:
            raise ValueError(
                f"{path}: unknown condition {label!r}. G2 has four conditions "
                f"{CONDITION_LABELS}; a fifth would break the identical-episode"
                f"-count argument in non-negotiable #10.")
        controller = entry["controller"]
        if controller.startswith(controllers.LAYER_PREFIX):
            raise ValueError(
                f"{path}: condition {label} is bound to {controller!r}, which "
                f"is the GROWTH PAIRING — Layer[ContractPorts, BASE, node]: "
                f"the shared substrate plus ONE leaf. It is a probe, not an "
                f"arm. Refused for the same reason a bare "
                f"{controllers.NODE_PREFIX}<id> is: the base is exempt from #3 "
                f"as a declared substrate, which makes it part of every arm "
                f"and therefore an arm by itself in no sweep. A condition is a "
                f"composed tree ({controllers.COMPOSED_PREFIX}<id>) or a "
                f"registered controller.")
        if controller.startswith(controllers.NODE_PREFIX):
            raise ValueError(
                f"{path}: condition {label} is bound to {controller!r}, which "
                f"is ONE LEAF plus the harness's port scaffold — the tree the "
                f"admission decision was taken on, not a navigator. It is "
                f"refused for the same reason scaffolding is (#3): a sweep "
                f"would score it and produce a complete, plausible, "
                f"meaningless table. A condition is a composed tree "
                f"({controllers.COMPOSED_PREFIX}<id>) or a registered "
                f"controller.")
        if controller in controllers.scaffolding():
            raise ValueError(
                f"{path}: condition {label} is bound to {controller!r}, which "
                f"is scaffolding. Non-negotiable #3: scaffolding must never "
                f"enter a benchmark condition. It has no obstacle awareness "
                f"and no generation step, and a sweep would score it anyway.")
        try:
            controllers.make(controller)
        except Exception as exc:
            raise ValueError(
                f"{path}: condition {label}'s controller {controller!r} does "
                f"not resolve: {exc}") from exc
        if controller in seen:
            raise ValueError(
                f"{path}: conditions {seen[controller]} and {label} are both "
                f"bound to {controller!r}. Their gap is zero by construction; "
                f"if that is intended, say so in a note and use two names.")
        seen[controller] = label
        snapshot = entry.get("library_snapshot")
        if reportable and not snapshot:
            raise ValueError(
                f"{path}: condition {label} has no library_snapshot. "
                f"Non-negotiable #8: every reported result names the frozen "
                f"snapshot tag it came from. Set \"reportable\": false for a "
                f"rehearsal.")
        conditions.append(Condition(label=label, controller=controller,
                                    library_snapshot=snapshot,
                                    note=entry.get("note", "")))

    if not conditions:
        raise ValueError(f"{path}: no conditions")
    # Stable, and the order the write-up presents them in.
    conditions.sort(key=lambda c: CONDITION_LABELS.index(c.label))
    return SweepSpec(conditions=tuple(conditions), name=name,
                     reportable=reportable)
