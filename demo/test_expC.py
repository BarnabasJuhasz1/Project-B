"""
demo/test_expC.py -- Exp C's invariants, sim-free.

    pytest demo/test_expC.py -q
"""
import pathlib
import re
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from demo import expC
from demo.rollout import load_controller


def test_originals_are_byte_identical_to_the_recorded_hashes():
    for k, sha in expC.ORIGINAL_SHA256.items():
        assert expC.sha256(expC.original_path(k)) == sha, k


def test_the_first_16_hex_are_exp3s_controller_sha():
    # The unpatched comparison rows are keyed on these.
    exp3 = {"k0": "dc3622b547c3568e", "k1": "51b58b5eb74b4dd5",
            "k2": "d8f855696060c384", "k3": "0bf3f73571fcd251",
            "k4": "c0422969be1bba1f", "k6": "69411dd5e493f373",
            "k7": "70ca68a952daff95"}
    assert {k: v[:16] for k, v in expC.ORIGINAL_SHA256.items()} == exp3


def test_committed_copies_are_the_original_plus_the_block():
    for k in expC.EXP4_LABEL:
        got = expC.copy_path(k).read_text(encoding="utf-8")
        assert got == expC.build(k), k
        assert got.startswith(expC.original_path(k).read_text(encoding="utf-8"))
        assert got.count(expC.BANNER) == 1


def test_block_is_identical_across_copies_except_the_hook():
    def strip(k):
        b = expC.block(k)
        b = b.replace(f"demo/arms/armB/{k}.py", "demo/arms/armB/K.py")
        b = b.replace(expC.ORIGINAL_SHA256[k], "SHA")
        return b.split("# Drop the cached route; keep the map.")[0]
    ref = strip("k0")
    for k in expC.EXP4_LABEL:
        assert strip(k) == ref, k


def test_one_threshold_one_window_everywhere():
    for k in expC.EXP4_LABEL:
        src = expC.copy_path(k).read_text(encoding="utf-8")
        assert f"_EXPC_PROGRESS_M = {expC.PROGRESS_M!r}" in src
        assert f"_EXPC_WINDOW_S = {expC.WINDOW_S!r}" in src


def test_seven_controllers_and_k5_is_not_one_of_them():
    assert sorted(expC.EXP4_LABEL) == ["k0", "k1", "k2", "k3", "k4", "k6", "k7"]
    assert set(expC.HOOKS) == set(expC.EXP4_LABEL)
    assert sum(v == "incremental_only" for v in expC.EXP4_LABEL.values()) == 5


def test_patched_copies_never_reach_an_arm():
    # run.py / physics_sweep.py / analyse.py glob demo/arms/<arm>/k*.py.
    arms = expC.HERE / "arms"
    assert arms not in expC.COPIES.parents
    from demo import physics_sweep
    paths = {str(p) for _, _, p in physics_sweep.discover(["arm0", "armB"])}
    assert not any("interventions" in p for p in paths)


def test_copies_pass_the_import_gate_and_build():
    from demo.rollout import PatrolScenario
    for k in expC.EXP4_LABEL:
        cls = load_controller(expC.copy_path(k))
        ctl = cls()
        sc = PatrolScenario(seed=0)
        ctl.reset(sc, sc.rng())
        assert ctl.expc_fallback_fires == [] and ctl.expc_errors == []


def _monitor():
    ns = {"Controller": object}
    src = expC.block("k0")
    exec(compile(src, "<block>", "exec"), ns)
    return ns["_ExpCNoProgress"]()


def test_trigger_fires_after_the_window_without_progress():
    m = _monitor()
    assert not m.update(0.0, 0.0, 0.0, 10.0, 0.0)
    for i in range(1, 500):                       # 9.98 s, no progress
        assert not m.update(i * 0.02, 0.0, 0.0, 10.0, 0.0)
    assert m.update(10.0, 0.0, 0.0, 10.0, 0.0)    # 10 s: fire
    assert not m.update(10.02, 0.0, 0.0, 10.0, 0.0)  # and restart


def test_trigger_does_not_fire_while_progress_is_made():
    m = _monitor()
    for i in range(0, 1500):                      # 0.1 m/s toward the goal
        t = i * 0.02
        assert not m.update(t, 0.1 * t, 0.0, 100.0, 0.0)


def test_small_progress_does_not_count():
    m = _monitor()
    m.update(0.0, 0.0, 0.0, 10.0, 0.0)
    fired = [m.update(i * 0.02, 0.4 * min(1.0, i / 400), 0.0, 10.0, 0.0)
             for i in range(1, 501)]
    assert fired[-1] is True and sum(fired) == 1   # 0.4 m < 0.5 m


def test_trigger_restarts_on_a_new_target():
    m = _monitor()
    m.update(0.0, 0.0, 0.0, 10.0, 0.0)
    for i in range(1, 400):
        m.update(i * 0.02, 0.0, 0.0, 10.0, 0.0)
    assert not m.update(8.0, 0.0, 0.0, 0.0, 10.0)  # goal flipped at 8 s
    assert not m.update(17.9, 0.0, 0.0, 0.0, 10.0)
    assert m.update(18.0, 0.0, 0.0, 0.0, 10.0)


def test_hooks_only_drop_route_state():
    # Nothing a hook does may touch the map: no log-odds, occupancy or cost
    # grid is named on its left-hand side.
    for k, hook in expC.HOOKS.items():
        targets = re.findall(r"self\.(\w+)(?:\[[^\]]*\])?\s*=", hook)
        assert set(targets) <= {"_fields", "planners", "fields", "hcache",
                                "h", "pending_burst"}, (k, targets)


def _row(k, rep, legs, fires=""):
    return {"arm": "armB", "k": k, "rep": str(rep), "legs": str(legs),
            "fires": str(fires)}


def test_analysis_pairs_on_k_and_rep_and_reads_the_verdict():
    ks = ["k0", "k1", "k3"]
    u = [_row(k, r, l) for k, ls in (("k0", (2, 2, 2)), ("k1", (2, 2, 4)),
                                     ("k3", (4, 4, 4)))
         for r, l in enumerate(ls)]
    p = [_row(k, r, l, 3) for k, ls in (("k0", (4, 4, 2)), ("k1", (2, 4, 4)),
                                        ("k3", (4, 4, 4)))
         for r, l in enumerate(ls)]
    a = expC.analyse(u, p, ks)
    assert a["per"]["k0"]["unpatched"] is False and a["per"]["k0"]["patched"]
    assert a["rescued"] == 2 and a["broken"] == 0
    assert a["verdict"] == "INCONCLUSIVE"
    assert a["episodes_up"] == 3 and a["episodes_down"] == 0


def test_analysis_ignores_reps_missing_from_either_side():
    u = [_row("k0", 0, 2), _row("k0", 1, 2)]
    p = [_row("k0", 0, 4), _row("k0", 1, 4), _row("k0", 2, 4)]
    a = expC.analyse(u, p, ["k0"])
    assert a["per"]["k0"]["reps"] == ["0", "1"]


def test_verdict_table_and_sign_test():
    assert expC.verdict(3, 0) == "SUPPORTS"
    assert expC.verdict(5, 1) == "HARMS"
    assert expC.verdict(1, 0) == "DOES NOT SUPPORT"
    assert expC.verdict(2, 0) == "INCONCLUSIVE"
    assert abs(expC.sign_test(5, 0) - 0.0625) < 1e-12
    assert expC.sign_test(0, 0) == 1.0
