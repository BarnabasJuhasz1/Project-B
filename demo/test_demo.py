"""demo/test_demo.py — the invariants. `python demo/test_demo.py`, or pytest."""
import sys, pathlib, json, hashlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from demo import render, world, rollout

# The pilot's prompt hashes, as recorded in every pilot transcript name. If
# either moves, the pilot cannot be pooled with anything generated after it.
PILOT_HASHES = {"arm0": "c6a5b5e6eac0def5", "armB": "4045efb3e514b981"}
PAPER_TXT_SHA256 = "1294dc34b73f9b2a0b72c7cad6bff9e8b1dd3172e0049d841192674ff0f9843a"
MANIFEST = render.HERE / "controls" / "manifest.json"


def _skip(msg):
    """pytest.skip under pytest; a plain SKIP line under the __main__ runner."""
    try:
        import pytest
    except ImportError:
        raise _Skip(msg)
    pytest.skip(msg)


class _Skip(Exception):
    pass


def _prompt_hash(arm):
    return hashlib.sha256(render.render(arm).encode()).hexdigest()[:16]


def test_arms_differ_only_in_the_paper_block():
    assert render.ARMS == ("arm0", "armB", "armX")
    ok, detail = render.check(paper="PLACEHOLDER " * 1000)
    assert ok, detail


def test_all_three_real_renders_identical_outside_the_block():
    for arm in ("armB", "armX"):
        if not render.DOCUMENTS[arm].is_file():
            _skip(f"{render.DOCUMENTS[arm].name} absent -- see render.paper_text()")
    ok, detail = render.check()
    assert ok, detail
    assert "arm0, armB, armX" in detail


def test_pilot_prompt_hashes_unchanged():
    """armX must not have moved arm0 or armB by a byte."""
    assert _prompt_hash("arm0") == PILOT_HASHES["arm0"]
    if not render.DOCUMENTS["armB"].is_file():
        _skip("demo/paper.txt absent; arm0's hash was checked")
    assert hashlib.sha256(render.DOCUMENTS["armB"].read_bytes()).hexdigest() \
        == PAPER_TXT_SHA256, "demo/paper.txt changed"
    assert _prompt_hash("armB") == PILOT_HASHES["armB"]


def test_manifest_pins_the_pilot_hashes_and_armx():
    m = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for arm, h in PILOT_HASHES.items():
        assert m["prompt_sha256_16"][arm] == h
    assert m["prompt_sha256_16"]["armX"], "armX prompt hash not recorded"
    if not render.DOCUMENTS["armX"].is_file():
        _skip("feng2015.txt absent -- python demo/controls/build_feng2015.py")
    text = render.paper_text(render.DOCUMENTS["armX"])
    assert hashlib.sha256(text.encode()).hexdigest() == m["truncated_sha256"]
    assert len(text) == m["truncated_chars"]
    assert _prompt_hash("armX") == m["prompt_sha256_16"]["armX"]


def test_armx_tokens_within_half_a_percent_of_armb():
    """⚠️ Until a Claude count exists this passes on an OFFLINE ESTIMATE
    (proxy tokenizers, see controls/MANIFEST.md), not a measurement. Once the
    calibration count is recorded as `armX_measured`, that is what is tested."""
    t = json.loads(MANIFEST.read_text(encoding="utf-8"))["claude_input_tokens"]
    x = t["armX_measured"] if t["armX_measured"] is not None else t["armX_estimate"]
    assert abs(x / t["armB"] - 1) <= t["tolerance"], (x, t["armB"])


def test_feng_text_is_regenerated_not_committed():
    ignored = (render.HERE / "controls" / ".gitignore").read_text().split()
    assert "feng2015.txt" in ignored, "licence unclear (R10): keep it out of git"


def test_feng_cut_is_a_paragraph_boundary():
    from demo.controls import build_feng2015 as feng
    try:
        pdf = feng.find_pdf()
    except FileNotFoundError:
        _skip("Feng PDF not on this machine")
    m = json.loads(MANIFEST.read_text(encoding="utf-8"))
    full = feng.extract(pdf)
    assert hashlib.sha256(full.encode()).hexdigest() == m["full_extraction_sha256"]
    lines = full.split("\n")
    assert m["cut_lines"] - 1 in feng.paragraph_ends(lines)
    assert lines[m["cut_lines"] - 1].rstrip().endswith(m["cut_after_text"])


def test_world_guarantees_hold():
    r = world.verify()
    assert r["ok_open_solvable"], "short route missing"
    assert r["ok_blocked_solvable"], "NO second route — lap 2 is impossible"
    assert r["ok_door_matters"], f"detour only {r['detour_cost_m']:.2f} m"
    assert r["ok_start_free"] and r["ok_goal_free"]


def test_bypass_is_actually_navigable_not_just_solvable():
    """⚠️ The regression this exists for: the first world's bypass had two
    0.30 m slots. geodesic_distance called it solvable and EVERY failing
    controller in BOTH arms stalled at one of them, so the demo measured
    slot-threading rather than rerouting."""
    r = world.verify()
    assert r["ok_bypass_navigable"], (
        f"bypass narrows to {r['bypass_min_width_m']} m of centre-free width "
        f"at y={r['bypass_tightest_at_y']}")


def test_the_door_is_the_only_difference_between_the_two_worlds():
    extra = set(world.BLOCKED_WORLD.rects) - set(world.OPEN_WORLD.rects)
    assert extra == {tuple(map(float, world.DOOR))}, extra


def test_import_allowlist_refuses_ground_truth():
    bad = rollout.check_imports("from core.geometry import clearance\n")
    assert bad, "core.geometry must be refused — it is ground truth"
    assert not rollout.check_imports("import numpy as np\nimport heapq\n")


def test_door_shuts_only_after_the_first_out_and_back():
    assert rollout.LEG_BLOCKED == (False, False, True, True)
    assert len(rollout.LEG_TARGETS) == len(rollout.LEG_BLOCKED)


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn(); print(f"PASS {name}")
            except _Skip as exc:
                print(f"SKIP {name}: {exc}")
            except AssertionError as exc:
                fails += 1; print(f"FAIL {name}: {exc}")
    print(f"\n{fails} failure(s)")
    sys.exit(1 if fails else 0)
