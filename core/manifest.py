"""
core/manifest.py

Provenance for one run. "Every reported result names the frozen library snapshot
tag it was produced from" is non-negotiable #8, and the reproducibility
requirement in the description's §6 needs more than the tag: the code, the scene
bytes, the simulator build and the machine.

The scene bytes matter more than they look. scenes/lab42_v1.usd is an 18 MB
binary crate tracked in git, it has already been rewritten once by a rebuild, and
nothing about the file says whether it still matches build_scene.py. Hashing what
was actually opened closes that gap: two runs with the same usd_sha256 saw the
same room, whatever the diff says.

Every external probe degrades to None rather than raising. No git repo, no
nvidia-smi, no container — none of those may be the reason a run fails. A missing
field is a smaller problem than a lost episode.

    python -c "import core.manifest as m, json; print(json.dumps(m.capture(scene='scenes/lab42_v1.usd'), indent=2))"
"""

import hashlib
import importlib.metadata
import json
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# Set by the container entrypoint to the pinned image reference. Absent outside
# the container, which is itself worth recording: a local authoring run and a
# reported run must be distinguishable at a glance.
CONTAINER_IMAGE_ENV = "PROJECT_B_CONTAINER_IMAGE"

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Read in 1 MiB blocks; the scene is 18 MB and slurping it costs nothing but is
# an unnecessary spike when a batch runner hashes it once per episode.
_HASH_BLOCK = 1 << 20


# ---------------------------------------------------------------------------

def _run(argv):
    """Command stdout, stripped. None on any failure whatsoever."""
    if shutil.which(argv[0]) is None:
        return None
    try:
        out = subprocess.run(argv, cwd=REPO_ROOT, capture_output=True,
                             text=True, timeout=10)
    except (OSError, subprocess.SubprocessError):
        return None
    return out.stdout.strip() if out.returncode == 0 else None


def sha256_file(path):
    """None if the file is missing — the runner reports that itself, loudly."""
    try:
        digest = hashlib.sha256()
        with open(path, "rb") as fh:
            for block in iter(lambda: fh.read(_HASH_BLOCK), b""):
                digest.update(block)
        return digest.hexdigest()
    except OSError:
        return None


def _git():
    commit = _run(["git", "rev-parse", "HEAD"])
    status = _run(["git", "status", "--porcelain"])
    return {
        "git_commit": commit,
        # None, not False, when there is no repo: "clean" would be a lie.
        "git_dirty": None if status is None else bool(status),
        "git_branch": _run(["git", "rev-parse", "--abbrev-ref", "HEAD"]),
    }


def _gpu():
    out = _run(["nvidia-smi",
                "--query-gpu=name,driver_version",
                "--format=csv,noheader"])
    if not out:
        return {"gpu_name": None, "driver_version": None}
    name, _, driver = out.splitlines()[0].partition(",")
    return {"gpu_name": name.strip() or None, "driver_version": driver.strip() or None}


def _isaacsim_version():
    """Read the distribution metadata. Deliberately NOT `import isaacsim` — that
    would boot a simulator and break the sim-free guarantee of this package."""
    for dist in ("isaacsim", "isaacsim-core"):
        try:
            return importlib.metadata.version(dist)
        except importlib.metadata.PackageNotFoundError:
            continue
    return None


# Characters that are legal in a controller name but not in a filename on every
# machine a checkout might land on. A composed controller is named
# `composed:<id>`, and a colon is illegal on NTFS, was the path separator on
# classic macOS, and makes `host:path` ambiguous to scp and rsync. A repo that
# cannot be checked out on Windows is a real defect for a shared research repo,
# so the substitution happens once, here, rather than being remembered.
RUN_ID_SUBSTITUTIONS = {":": "-", "/": "-", os.sep: "-"}


def run_id(scenario_id, controller, seed):
    """The identity of one EPISODE: what its committed result is named after.

    One definition with three callers -- `scripts/run_episode.py`, the batch
    runner, and `core/tests/test_results_integrity.py`. Two copies of this
    would be two answers to "what is this episode called", and the integrity
    test exists precisely because a result that disagrees with its own name is
    unfindable by anything that trusts either one.

    The manifest keeps the controller's REAL name; only the filename is
    substituted. Going the other way -- storing the sanitised name -- would
    lose which composition actually ran.

        >>> run_id("doorway_traverse", "reactive_avoid", 1001)
        'doorway_traverse__reactive_avoid__seed1001'
        >>> run_id("held_x", "composed:9c21b87c", 2000)
        'held_x__composed-9c21b87c__seed2000'
    """
    safe = str(controller)
    for bad, good in RUN_ID_SUBSTITUTIONS.items():
        safe = safe.replace(bad, good)
    return f"{scenario_id}__{safe}__seed{seed}"


def _prereg_tag():
    """The most recent `prereg-*` tag reachable from HEAD, or None.

    None before the tag exists, which is the honest state during epic 02 and
    the reason `results/README.md` lists it as un-reportable rather than
    letting a null slide through.
    """
    return _run(["git", "describe", "--tags", "--abbrev=0", "--match", "prereg-*"])


def _prereg_digest():
    """sha256 over every frozen constant, from core.preregistration.

    Imported inside the function: core/manifest.py is imported by scripts that
    boot the simulator, and there is no reason to drag harness/ and core.stats
    into that import graph for a field that costs microseconds to compute here.
    """
    try:
        from core.preregistration import digest
        return digest()
    except Exception:
        # A manifest that cannot be written is worse than one missing a field.
        return None


def capture(scene, scenario_id=None, seed=None, controller=None,
            library_snapshot=None, extra=None):
    """Everything needed to tell two runs apart, or to prove they are the same.

    `library_snapshot` is the frozen node-library tag from non-negotiable #8. It
    is None until Week 3 builds the library, and it is in the schema now so that
    no result file ever exists without a slot for it.
    """
    scene_path = scene if os.path.isabs(scene) else os.path.join(REPO_ROOT, scene)
    manifest = {
        "started_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "scenario_id": scenario_id,
        "seed": seed,
        "controller": controller,
        "library_snapshot": library_snapshot,
        "usd_path": os.path.relpath(scene_path, REPO_ROOT),
        "usd_sha256": sha256_file(scene_path),
        # The sealed held-out split, hashed on every run rather than only on
        # held-out ones. A dev result that names the split it did NOT run
        # against is what lets you prove, later, that the split already existed
        # and had this content while development was happening. Recording it
        # only on held-out runs would throw that evidence away.
        "held_out_sha256": sha256_file(
            os.path.join(REPO_ROOT, "core", "scenarios_held_out.py")),
        # The pre-registration this result ran under. Two fields, because they
        # fail in different ways: the TAG says which commit of
        # PRE-REGISTRATION.md was in force, and the DIGEST says what the frozen
        # constants actually were when the robot ran. A tag can be moved and a
        # working tree can be edited ahead of a tag; the digest cannot be
        # either. A result whose digest does not match the tagged document was
        # scored under constants nobody registered.
        "prereg_tag": _prereg_tag(),
        "prereg_digest": _prereg_digest(),
        "isaacsim_version": _isaacsim_version(),
        "python_version": sys.version.split()[0],
        "platform": platform.platform(),
        "container_image": os.environ.get(CONTAINER_IMAGE_ENV),
        # Runs outside the pinned container are for authoring and debugging
        # only; this is the flag that says a number must not be reported.
        "in_container": os.path.exists("/.dockerenv"),
        "sim_seconds": None,
        "wall_seconds": None,
    }
    manifest.update(_git())
    manifest.update(_gpu())
    if extra:
        manifest.update(extra)
    return manifest


def write(manifest, path):
    with open(path, "w") as fh:
        json.dump(manifest, fh, indent=2, sort_keys=True)
        fh.write("\n")
