"""
compose/library.py

Materialise the frozen node library that composition retrieves from.

THERE IS NO SERIALISED LIBRARY FORMAT, AND THAT IS DELIBERATE
--------------------------------------------------------------
`nodes.library.Library` holds node *classes*, and a class is not JSON. The
durable record is `generate/staging/` — every candidate, its provenance sidecar
and its admission verdict, all committed — and the library is what you get by
re-loading the admitted ones. So the library cannot drift from the record of how
it was produced: there is nothing for it to drift from.

**Loading re-derives, it never re-decides.** The verdict in
`<stem>.result.json` is the admission decision, taken once by
`generate/admit.py` with the harness in the loop, and committed. Re-running the
unit scenario here would be a second decision that could disagree with the
recorded one, and G1's pass rate is computed from the record. Re-checking the
*source* is different and does happen: `check_source` runs again on every load,
because the import allowlist is a safety barrier and a barrier you only check
once is a barrier for the honest.

⚠️ **THIS EXECUTES MODEL-WRITTEN PYTHON, with no sandbox** — the same warning
`generate/admit.py` carries, for the same reason, and it is worth repeating at
the second door into the same room.

WHAT IS ALWAYS IN, AND WHAT IS OPT-IN
--------------------------------------
`ContractPorts` is always in: it publishes `goal_known`, `pose` and `ranges`
straight from `Observation`, and it is the same class `generate/admit.py` uses,
so what a node could read at admission is exactly what it can read at
composition. See CONTRACT_PORTS_ID below.


`include_toys=True` adds the handwritten fixture leaves. They are NOT navigation
and must never appear in a benchmark condition (non-negotiable #3); they are in
so that the composition path can be exercised end to end while the real library
is one node long. Making it an argument rather than a default means a composed
benchmark condition containing a toy is something someone had to ask for, the
same shape as `controllers.available()` excluding scaffolding.
"""

import json
from pathlib import Path

from nodes.library import Library

REPO_ROOT = Path(__file__).resolve().parent.parent
STAGING = REPO_ROOT / "generate" / "staging"


class LibraryLoadError(RuntimeError):
    """A committed admission record could not be turned back into a node.

    Raised rather than skipped. A library that quietly drops an entry it cannot
    load is a library whose snapshot tag means "whatever loaded today", and #8
    requires the tag to name what a result was produced from.
    """


# The fixture leaves, and the ids they are admitted under. Handwritten, so their
# metadata says HANDWRITTEN and G1 counts them as untraceable — see
# nodes/toys.py on why a fixture claiming a source span would corrupt the one
# number the contract exists to make countable.
# The ports every candidate was generated and admitted against. NOT optional and
# NOT a toy.
#
# `generate/admit.py` publishes `goal_known`, `pose` and `ranges` from
# `Observation` through `ContractPorts`, so a node reading one of them can be
# tested alone. Composition has to offer the same three or the two steps
# disagree about what ports exist: a node admitted while reading `ranges` would
# be UNSATISFIABLE at composition time, and its unit scenario would have proved
# nothing about the run — which is the failure this repo keeps guarding against.
#
# The class is imported rather than re-declared, deliberately. One definition
# means the harness and composition cannot drift apart; two would mean they
# could, silently, and only a generated node's wiring failure would say so.
CONTRACT_PORTS_ID = "observation_ports"

TOY_LEAVES = (
    ("toy_is_goal_known", "nodes.toys:IsGoalKnown"),
    ("toy_drive_to_goal", "nodes.toys:DriveToGoal"),
    ("toy_sweep_for_goal", "nodes.toys:SweepForGoal"),
    ("toy_avoid_obstacles", "nodes.toys:AvoidObstacles"),
)


def admitted_stems(staging=None):
    """The stems of every candidate the recorded verdict admitted, sorted.

    Sorted by stem rather than left in directory order so that the library is
    built in the same sequence on every machine — `Library.snapshot()` hashes
    sorted contents and does not care, but `Library.all()` is insertion order
    and the composition record quotes it.
    """
    staging = Path(staging or STAGING)
    stems = []
    for path in sorted(staging.glob("*.result.json")):
        record = json.loads(path.read_text(encoding="utf-8"))
        if not record.get("admitted"):
            continue
        # A re-recorded generation. The record stays on disk - it is evidence of
        # a real generation and G1's denominator may not be edited after the
        # fact - but the library it produced is superseded by the re-record, and
        # admitting both puts ONE MECHANISM IN THE LIBRARY TWICE. That matters
        # more here than in the pass rate: this is the library G3's "solved by
        # recombination alone" is measured against, so a duplicate inflates the
        # coverage count and gives retrieval two names for one behaviour.
        # `generate/admit.py:pass_rate` skips the same records for the same
        # reason; the two must agree or the library and the number describing it
        # would be about different sets.
        if record.get("metadata", {}).get("superseded_by"):
            continue
        stems.append(path.name[: -len(".result.json")])
    return stems


def _add_contract_ports(library):
    from generate.admit import ContractPorts

    from nodes.toys import toy_metadata
    library.add(ContractPorts, toy_metadata(CONTRACT_PORTS_ID))


def _add_toys(library):
    import importlib

    from nodes.toys import toy_metadata
    for node_id, target in TOY_LEAVES:
        module_name, _, attr = target.partition(":")
        node_cls = getattr(importlib.import_module(module_name), attr)
        library.add(node_cls, toy_metadata(node_id))


def load_library(staging=None, include_toys=False, cards=None):
    """Rebuild the append-only library from the committed admission records.

    Returns the Library. `library.snapshot()` is the tag that goes into
    `manifest["library_snapshot"]` and into every composition record.
    """
    from generate import admit as admit_mod

    staging = Path(staging or STAGING)
    library = Library()

    # First, always. Everything else may read the ports it publishes, and
    # retrieval's port closure needs a producer to find.
    _add_contract_ports(library)

    if include_toys:
        # Toys first, so their ids are stable regardless of how many generated
        # nodes exist. A snapshot tag is over sorted contents, so ordering does
        # not change the tag; it changes what a reader sees at the top of a log.
        _add_toys(library)

    for stem in admitted_stems(staging):
        meta = json.loads((staging / f"{stem}.json").read_text(encoding="utf-8"))
        source = (staging / f"{stem}.py").read_text(encoding="utf-8")

        card = None
        if meta.get("card_id"):
            if cards is None:
                from cards.store import CardStore
                cards = CardStore.load()
            if meta["card_id"] not in cards:
                # A candidate generated from a test fixture card. It was never
                # part of the corpus, `pass_rate()` excludes it from G1's
                # numbers for the same reason, and it must not be retrievable.
                continue
            card = cards.get(meta["card_id"])

        try:
            node_cls, _, _, _ = admit_mod.load_candidate(source, meta["node_id"])
            metadata = admit_mod._node_metadata(meta, meta["node_id"], card)
            library.add(node_cls, metadata)
        except Exception as exc:                 # noqa: BLE001
            raise LibraryLoadError(
                f"{stem}: admitted on the record but will not load now: "
                f"{type(exc).__name__}: {exc}. The library is append-only (#7) "
                f"and a snapshot tag must name what a result came from, so "
                f"this is a hard failure rather than a skipped entry."
            ) from exc

    return library
