"""
core/tests/test_manifest.py

The degradation tests are the important ones. A manifest that raises when
nvidia-smi is missing turns a provenance nicety into a run-killer, and it would
fail exactly where it hurts — on a CI box or a machine set up mid-experiment.
"""

import hashlib
import json
import os
import subprocess

import pytest

from core import manifest as M

SCENE = "scenes/lab42_v1.usd"

REQUIRED = {
    "started_utc", "scenario_id", "seed", "controller", "library_snapshot",
    "usd_path", "usd_sha256", "isaacsim_version", "python_version", "platform",
    "container_image", "in_container", "sim_seconds", "wall_seconds",
    "git_commit", "git_dirty", "git_branch", "gpu_name", "driver_version",
    "held_out_sha256", "prereg_tag", "prereg_digest",
}


def test_every_field_is_present():
    assert REQUIRED <= set(M.capture(scene=SCENE))


def test_manifest_is_json_serialisable():
    m = M.capture(scene=SCENE, scenario_id="doorway_traverse", seed=1001,
                  controller="reactive_avoid")
    assert json.loads(json.dumps(m)) == m


def test_usd_hash_matches_the_bytes_on_disk():
    """The whole point: two runs with the same usd_sha256 saw the same room,
    whatever the 18 MB binary diff says."""
    path = os.path.join(M.REPO_ROOT, SCENE)
    expected = hashlib.sha256(open(path, "rb").read()).hexdigest()
    assert M.capture(scene=SCENE)["usd_sha256"] == expected


def test_missing_scene_hashes_to_none_rather_than_raising():
    assert M.capture(scene="scenes/does_not_exist.usd")["usd_sha256"] is None


def test_the_pre_registration_is_recorded_on_every_run():
    """Non-negotiable #8's sibling. A result has to be able to prove which
    pre-registration it ran under, and the digest is the half that cannot be
    faked by moving a tag."""
    from core.preregistration import digest
    m = M.capture(scene=SCENE)
    assert m["prereg_digest"] == digest()
    assert "prereg_tag" in m, "the slot exists before the tag does"


def test_library_snapshot_slot_exists_before_the_library_does():
    """Non-negotiable #8. The field is in the schema now so no result file ever
    exists without a place to name the snapshot it came from."""
    assert M.capture(scene=SCENE)["library_snapshot"] is None
    assert M.capture(scene=SCENE, library_snapshot="lib-v0")["library_snapshot"] == "lib-v0"


# --- degradation -----------------------------------------------------------

def test_gpu_probe_degrades_to_none(monkeypatch):
    monkeypatch.setattr(M.shutil, "which", lambda name: None)
    m = M.capture(scene=SCENE)
    assert m["gpu_name"] is None and m["driver_version"] is None
    assert m["git_commit"] is None          # same path, same degradation


def test_a_failing_subprocess_degrades_to_none(monkeypatch):
    def boom(*a, **k):
        raise OSError("no such binary")
    monkeypatch.setattr(M.subprocess, "run", boom)
    assert M.capture(scene=SCENE)["gpu_name"] is None


def test_git_dirty_is_none_not_false_without_a_repo(monkeypatch):
    """'clean' would be a lie when there is no repo to be clean about."""
    monkeypatch.setattr(M, "_run", lambda argv: None)
    assert M.capture(scene=SCENE)["git_dirty"] is None


def test_container_image_comes_from_the_environment(monkeypatch):
    monkeypatch.setenv(M.CONTAINER_IMAGE_ENV, "nvcr.io/nvidia/isaac-sim:6.0.1")
    assert M.capture(scene=SCENE)["container_image"] == "nvcr.io/nvidia/isaac-sim:6.0.1"


def test_isaac_version_is_read_not_imported():
    """Importing isaacsim here would boot a simulator and break the sim-free
    guarantee. importlib.metadata reads the dist-info instead."""
    import ast
    tree = ast.parse(open(os.path.join(M.REPO_ROOT, "core", "manifest.py")).read())
    # AST, not a substring search: the module's own docstring says the words
    # "import isaacsim" while explaining why it does not do it.
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported |= {a.name.split(".")[0] for a in node.names}
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported.add(node.module.split(".")[0])
    assert "isaacsim" not in imported
    assert M._isaacsim_version() == "6.0.1.0"


def test_write_produces_readable_json(tmp_path):
    path = tmp_path / "manifest.json"
    M.write(M.capture(scene=SCENE), path)
    assert json.loads(path.read_text())["usd_path"] == SCENE
