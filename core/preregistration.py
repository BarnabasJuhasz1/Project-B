"""
core/preregistration.py

The list of things that were frozen before the first evaluation run, and the
machinery that makes freezing them mean something.

`PRE-REGISTRATION.md` is the document. This module is its enforcement: it
enumerates every constant the document claims to have fixed, reads the values
the document records, and compares them against the values the code actually
holds. `core/tests/test_preregistration.py` fails on any disagreement, so
**changing a constant without editing the document is a failing test**. Without
that, the document is a comment.

Two properties are checked, not one, and the second is the one with teeth:

  1. the digest recorded in the document matches the digest of the live values;
  2. every recorded VALUE matches the live value, name by name.

(1) alone could be satisfied by pasting in a new digest. (2) cannot -- it forces
whoever moves a constant to write the new number into the document, next to the
rationale, where a reader will see it.

    from core.preregistration import digest, verify
    print(digest())
    verify()          # raises with a diff, or returns the values

Sim-free. No simulator, no GPU, no network.
"""

import hashlib
import importlib
import os
import re

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# The document. At the repo root, not under plans/, because it is not a plan --
# it is the claim the plans were executed under.
DOCUMENT = "PRE-REGISTRATION.md"

# The fenced block inside it that carries the values. Named rather than
# positional so the document can be reordered freely.
BLOCK_LANGUAGE = "prereg-constants"

# Every constant frozen by chunk 17, dotted. ORDER IS PART OF THE DIGEST, so
# appending is an amendment and reordering is a spurious failure -- add at the
# end and date the amendment, exactly as the document says.
#
# What belongs here: anything whose value changes what a reported number MEANS.
# What does not: anything that only changes how long a run takes, or how a
# result is printed. `--fps` is not here; `GOAL_RADIUS` is.
FROZEN = (
    # Geometry. The SPL denominator and what counts as passable are both
    # downstream of these, so they move every efficiency number at once.
    "core.geometry.GRID_RESOLUTION",
    "core.geometry.ROBOT_RADIUS",
    "core.geometry.ARC_STEP_DEG",

    # Scoring thresholds. What success, a collision, a near miss, a stuck span
    # and a localisation failure ARE.
    "core.metrics.GOAL_RADIUS",
    "core.metrics.FALL_HEIGHT",
    "core.metrics.COLLISION_CLEARANCE",
    "core.metrics.NEAR_MISS_CLEARANCE",
    "core.metrics.EVENT_RELEASE",
    "core.metrics.EVENT_RELEASE_EPS",
    "core.metrics.STUCK_WINDOW",
    "core.metrics.STUCK_DISPLACEMENT",
    "core.metrics.STUCK_COMMAND_EPS",
    "core.metrics.LOC_FAIL_THRESHOLD",

    # The task. Sensor suite, actuation limits and the disclosure rule are part
    # of the task specification and are identical in every condition -- if a
    # condition could move one, the ablation would measure the sensor.
    "core.scenario.N_BEAMS",
    "core.scenario.FOV",
    "core.scenario.RANGE_MAX",
    "core.scenario.MAX_LIN",
    "core.scenario.MAX_ANG",
    "core.scenario.DETECT_RADIUS",
    "core.scenario.DEFAULT_SCENE",

    # Admission. The bar a generated node clears to enter the library, and
    # therefore what G1's pass rate is a pass rate OF.
    "harness.unit.MIN_EFFECT_RATE",
    "harness.unit.MIN_EFFECT_REL",

    # The decision rule itself.
    "core.stats.RESAMPLING_UNIT",
    "core.stats.N_RESAMPLES",
    "core.stats.CI_METHOD",
    "core.stats.CI_LEVEL",
    "core.stats.BOOTSTRAP_SEED",
    "core.stats.X",
    "core.stats.SEED_OFFSETS",
)

# Artefacts that are frozen by content rather than by value. Hashed into the
# same digest so that swapping the scene or the split is as loud as moving a
# threshold. Paths are repo-relative.
FROZEN_FILES = (
    "core/scenarios_held_out.py",
    "scenes/lab42_v1.usd",
)


# ---------------------------------------------------------------------------


def _resolve(dotted):
    module, _, name = dotted.rpartition(".")
    return getattr(importlib.import_module(module), name)


def _sha256_file(path):
    h = hashlib.sha256()
    with open(os.path.join(REPO_ROOT, path), "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def values():
    """Every frozen constant, live, in FROZEN order. Files as `sha256:<hex>`."""
    out = {name: _resolve(name) for name in FROZEN}
    for path in FROZEN_FILES:
        out[path] = "sha256:" + _sha256_file(path)
    return out


def render(vals=None):
    """The canonical text form. This is what gets hashed and what the document
    carries, so the two cannot describe different things."""
    vals = values() if vals is None else vals
    return "".join(f"{name} = {value!r}\n" for name, value in vals.items())


def digest(vals=None):
    """sha256 over render(). The single number the document records."""
    return hashlib.sha256(render(vals).encode()).hexdigest()


def document_path():
    return os.path.join(REPO_ROOT, DOCUMENT)


def parse_document(text=None):
    """The values the document claims, from its ```prereg-constants block.

    Returns (values, recorded_digest). Values are parsed with ast.literal_eval,
    so the block is data and cannot execute anything.
    """
    import ast

    if text is None:
        with open(document_path()) as fh:
            text = fh.read()

    blocks = re.findall(r"^```" + BLOCK_LANGUAGE + r"\n(.*?)^```",
                        text, re.M | re.S)
    if len(blocks) != 1:
        raise ValueError(
            f"{DOCUMENT} must contain exactly one ```{BLOCK_LANGUAGE} block, "
            f"found {len(blocks)}")

    vals = {}
    for line in blocks[0].splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, _, literal = line.partition(" = ")
        if not _:
            raise ValueError(f"unparseable constant line in {DOCUMENT}: {line!r}")
        vals[name] = ast.literal_eval(literal)

    found = re.findall(r"^prereg-digest:\s*`?([0-9a-f]{64})`?\s*$", text, re.M)
    if len(found) != 1:
        raise ValueError(
            f"{DOCUMENT} must carry exactly one `prereg-digest: <sha256>` "
            f"line, found {len(found)}")
    return vals, found[0]


def verify():
    """Raise unless the code and the document agree. Returns the live values.

    The error message is the diff, because the failure this produces is
    someone tuning a threshold mid-project, and what they need to see is which
    one and by how much -- not that a hash changed.
    """
    live = values()
    recorded, recorded_digest = parse_document()

    problems = []
    for name in set(live) | set(recorded):
        if name not in recorded:
            problems.append(f"  {name}: frozen in code, ABSENT from {DOCUMENT}")
        elif name not in live:
            problems.append(f"  {name}: in {DOCUMENT}, no longer frozen in code")
        elif live[name] != recorded[name]:
            problems.append(
                f"  {name}: code {live[name]!r} != document {recorded[name]!r}")
    if problems:
        raise AssertionError(
            "the code and PRE-REGISTRATION.md disagree about a frozen "
            "constant. Either revert the change, or amend the document with a "
            "DATED APPEND -- never a silent edit.\n" + "\n".join(sorted(problems)))

    if digest(live) != recorded_digest:
        raise AssertionError(
            f"every value agrees but the digest does not: code "
            f"{digest(live)} != document {recorded_digest}. FROZEN's order is "
            f"part of the digest; reordering it is the usual cause.")
    return live
