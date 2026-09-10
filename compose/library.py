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


def staging_dirs(staging=None):
    """Normalise `staging` to a tuple of directories to read, in priority order.

    Accepts None (the live directory), one path, or several. Several is what a
    HISTORICAL RECONSTRUCTION needs: the 2026-09-08 archive move put the
    one-call records under `generate/archive/v1-single-call/`, and the eight
    committed compositions name nodes that live there — so rebuilding one has to
    look in both places, while `load_library()` with no argument keeps reading
    the live directory alone and therefore stays a clean slate.

    Earlier directories win on a stem collision. There are none today and there
    should never be, since a stem carries a prompt hash; the rule exists so that
    if one ever appears the live record is the one that counts.
    """
    if staging is None:
        return (STAGING,)
    if isinstance(staging, (str, Path)):
        return (Path(staging),)
    return tuple(Path(d) for d in staging)


def admitted_stems(staging=None):
    """The stems of every candidate the recorded verdict admitted, sorted.

    Sorted by stem rather than left in directory order so that the library is
    built in the same sequence on every machine — `Library.snapshot()` hashes
    sorted contents and does not care, but `Library.all()` is insertion order
    and the composition record quotes it.
    """
    stems, seen = [], set()
    paths = [q for d in staging_dirs(staging) for q in d.glob("*.result.json")]
    for path in sorted(paths, key=lambda q: q.name):
        if path.name in seen:
            continue
        seen.add(path.name)
        record = json.loads(path.read_text(encoding="utf-8"))
        # Chunk 37's verdict where one exists, the original otherwise. A record
        # judged today carries only `admitted`, which already means validity;
        # `admitted_v2` is what `scripts/rejudge.py` wrote onto records taken
        # under the old rule. `generate/admit.py:pass_rate` reads the same field
        # in the same order — the two MUST agree, or the library and the number
        # describing it would be about different sets.
        if not record.get("admitted_v2", record.get("admitted")):
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
        # A node generated under a definition of its condition that no longer
        # exists. It must leave the library for the same reason it leaves the
        # pass rate: the library is per-condition evidence, and a node whose
        # condition was redefined is evidence about the retired treatment. It
        # stays on disk, and `generate/admit.py:pass_rate` names it in
        # `_retired` — the two must agree, or the library and the number
        # describing it would be about different sets.
        if record.get("metadata", {}).get("retired_reason"):
            continue
        stems.append(path.name[: -len(".result.json")])
    return stems


def _add_contract_ports(library):
    from generate.admit import ContractPorts

    from nodes.toys import toy_metadata
    library.add(ContractPorts, toy_metadata(CONTRACT_PORTS_ID))


def _add_toys(library, keep=None):
    import importlib

    from nodes.toys import toy_metadata
    for node_id, target in TOY_LEAVES:
        if keep is not None and node_id not in keep:
            continue
        module_name, _, attr = target.partition(":")
        node_cls = getattr(importlib.import_module(module_name), attr)
        library.add(node_cls, toy_metadata(node_id))


class LibraryReconstructionError(RuntimeError):
    """A historical library named by a record cannot be rebuilt today.

    Distinct from `LibraryLoadError`, which means an admitted node will not
    load. This means the SET is unrecoverable: an id the record listed is no
    longer an admitted, non-superseded, non-retired entry, so the library that
    composition saw cannot be reassembled from what is committed.
    """


def load_library(staging=None, include_toys=False, cards=None, only=None):
    """Rebuild the append-only library from the committed admission records.

    Returns the Library. `library.snapshot()` is the tag that goes into
    `manifest["library_snapshot"]` and into every composition record.

    `only` RESTRICTS the build to a set of node ids, and is how a HISTORICAL
    library is rebuilt. It exists because #7 (append-only) and #8 (every result
    names its snapshot) together mean that **every admission invalidates every
    composition recorded against the smaller library** — a cost that compounds,
    not a one-off. A composition record already carries `retrieval.considered`,
    the full list of ids the library held at composition time, so the historical
    library is exactly reconstructible from committed data with no model call.

    ⚠️ `only` never *adds* anything and never re-decides an admission: it is a
    filter over the same records `load_library()` reads, so a node that was
    never admitted cannot be conjured by naming it. Every requested id must
    materialise, and a missing one raises rather than yielding a smaller library
    that would hash to a tag naming a set it is not.
    """
    from generate import admit as admit_mod

    # NOT `Path(staging or STAGING)`: `staging` may name several directories
    # since the 2026-09-08 archive move, and coercing a tuple here raised a
    # TypeError from inside pathlib that said nothing about staging at all.
    library = Library()
    keep = None if only is None else set(only)

    def wanted(node_id):
        return keep is None or node_id in keep

    # First, always. Everything else may read the ports it publishes, and
    # retrieval's port closure needs a producer to find.
    if wanted(CONTRACT_PORTS_ID):
        _add_contract_ports(library)

    if include_toys:
        # Toys first, so their ids are stable regardless of how many generated
        # nodes exist. A snapshot tag is over sorted contents, so ordering does
        # not change the tag; it changes what a reader sees at the top of a log.
        _add_toys(library, keep)

    dirs = staging_dirs(staging)

    def _find(stem, suffix):
        for directory in dirs:
            path = directory / f"{stem}{suffix}"
            if path.exists():
                return path
        raise LibraryLoadError(
            f"{stem}{suffix} is in none of {[str(d) for d in dirs]}. The "
            f"verdict was found but the artefact it describes was not, which "
            f"means the record and the module were separated.")

    for stem in admitted_stems(staging):
        meta = json.loads(_find(stem, ".json").read_text(encoding="utf-8"))
        # Filtered BEFORE the source is read or executed. A historical
        # reconstruction must not run — or be broken by — a candidate admitted
        # after the composition it is rebuilding.
        if not wanted(meta["node_id"]):
            continue
        source = _find(stem, ".py").read_text(encoding="utf-8")

        # EITHER column, and they mean different things. `card_id` means the
        # extracted content produced this node (B); `citation_id` means only the
        # reference did (A). Reading just `card_id` here would rebuild every
        # condition-A node as `Origin.INVENTED`, losing the fact that it was
        # pointed at a paper at all — and `by_origin` is what G1's provenance
        # numbers are read off.
        card = citation_of = None
        ref_id = meta.get("card_id") or meta.get("citation_id")
        if ref_id:
            if cards is None:
                from cards.store import CardStore
                cards = CardStore.load()
            if ref_id not in cards:
                # A candidate generated from a test fixture card. It was never
                # part of the corpus, `pass_rate()` excludes it from G1's
                # numbers for the same reason, and it must not be retrievable.
                continue
            if meta.get("card_id"):
                card = cards.get(ref_id)
            else:
                citation_of = cards.get(ref_id)

        try:
            node_cls, _, _, _ = admit_mod.load_candidate(source, meta["node_id"])
            # The MODEL-WRITTEN card too, since chunk 42: it carries the span
            # and the extraction confidence the node's metadata is built from,
            # so a reconstruction that skipped it would rebuild every generated
            # node without them and `by_origin`/`traceable_count` would silently
            # disagree with the record they were admitted under. None for a
            # pre-42 candidate, which never had one.
            metadata = admit_mod._node_metadata(
                meta, meta["node_id"], card, citation_of=citation_of,
                generated_card=admit_mod.load_generated_card(stem, meta))
            library.add(node_cls, metadata)
        except Exception as exc:                 # noqa: BLE001
            raise LibraryLoadError(
                f"{stem}: admitted on the record but will not load now: "
                f"{type(exc).__name__}: {exc}. The library is append-only (#7) "
                f"and a snapshot tag must name what a result came from, so "
                f"this is a hard failure rather than a skipped entry."
            ) from exc

    if keep is not None:
        missing = sorted(keep - {entry.node_id for entry in library.all()})
        if missing:
            raise LibraryReconstructionError(
                f"{missing} were in the library when this composition was "
                f"recorded and are not admitted entries today. The usual cause "
                f"is a SUPERSEDED generation: `admitted_stems` drops it so that "
                f"one mechanism is not in the library twice, which is right for "
                f"the live library and is exactly what makes the historical one "
                f"unrecoverable. Returning a smaller library would produce a "
                f"snapshot tag naming a set it is not, which is the lie #8 "
                f"exists to prevent.")

    return library
