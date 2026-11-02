"""demo/test_demo.py — the invariants. `python demo/test_demo.py`, or pytest."""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from demo import render, world, rollout


def test_arms_differ_only_in_the_paper_block():
    ok, detail = render.check(paper="PLACEHOLDER " * 1000)
    assert ok, detail


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
            except AssertionError as exc:
                fails += 1; print(f"FAIL {name}: {exc}")
    print(f"\n{fails} failure(s)")
    sys.exit(1 if fails else 0)
