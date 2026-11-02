"""
demo/test_exp_ad.py -- Exp A / Exp D apparatus, sim-free.

    python -m pytest demo/test_exp_ad.py -q
"""
import json
import pathlib
import sys

import pytest

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from demo import exp3, exp_ad, exp_ad_report


def test_clock_limited_counts_match_the_prereg():
    """prereg/expD.md names 2 of 8 arm0 (k0, k6) and 7 of 8 armB (not k3)."""
    got = {(a, k) for a, k, p in exp_ad.controllers() if exp_ad.clock_limited(p)}
    assert {k for a, k in got if a == "arm0"} == {"k0", "k6"}
    assert {k for a, k in got if a == "armB"} == set(exp_ad.KS) - {"k3"}


def test_orders_cover_the_design_and_interleave_arms():
    a, d = exp_ad.job_order("A"), exp_ad.job_order("D")
    assert len(a) == 48 and len(d) == 16
    assert {(x[0], x[1], x[3]) for x in a} == {
        (arm, k, r) for arm in exp_ad.ARMS for k in exp_ad.KS for r in (0, 1, 2)}
    assert {x[3] for x in d} == {0}
    for order in (a, d):                       # never 4 of one arm in a row
        arms = [x[0] for x in order]
        assert all(len(set(arms[i:i + 4])) > 1 for i in range(len(arms) - 3))


def test_commands_reuse_exp3_offsets_and_only_A_opens_the_door():
    for rep in (0, 1, 2):
        cmd = exp_ad.command("A", "c.py", rep, "t.csv.gz")
        seed, dx, dy, dyaw = exp3.rep_condition(rep)
        assert cmd[cmd.index("--seed") + 1] == str(seed)
        assert float(cmd[cmd.index("--start-dx") + 1]) == dx
        assert float(cmd[cmd.index("--start-dyaw") + 1]) == dyaw
        assert "--door-never-shuts" in cmd
    assert "--door-never-shuts" not in exp_ad.command("D", "c.py", 0, "t.csv.gz")


def test_cpu_limit_check():
    assert exp_ad.cpu_limit_ok("D", "200000 100000")
    assert not exp_ad.cpu_limit_ok("D", "max 100000")
    assert not exp_ad.cpu_limit_ok("D", None)
    assert exp_ad.cpu_limit_ok("A", "max 100000") and exp_ad.cpu_limit_ok("A", None)
    assert not exp_ad.cpu_limit_ok("A", "200000 100000")


def test_parse_result_json():
    js = {"legs": 2, "total": 4, "success": False, "leg_times": [20.1, 30.2],
          "sim_seconds": 180.0, "contacts": 3, "step_ms_med": 1.2,
          "cpu_max": "200000 100000", "raised": None}
    row = exp_ad.parse("noise\nRESULT_JSON " + json.dumps(js), "")
    assert row["legs"] == 2 and row["leg_times"] == "[20.1, 30.2]"
    assert row["cpu_max"] == "200000 100000" and row["note"] == ""
    assert exp_ad.parse("crashed", "")["legs"] is None


def test_profile_check_needs_both_sources_performance():
    assert exp_ad.profile_ok(("performance", "performance"))
    assert not exp_ad.profile_ok(("performance", "balanced"))
    assert not exp_ad.profile_ok(("", "performance"))


def _rows(legs_of, rep_range=(0, 1, 2), **extra):
    out = []
    for arm in exp_ad.ARMS:
        for k in exp_ad.KS:
            for rep in rep_range:
                n = legs_of(arm, k, rep)
                out.append({"arm": arm, "k": k, "rep": str(rep), "legs": str(n),
                            "leg_times": json.dumps([20.0] * n), "contacts": "0",
                            "track_v_med": "0.07", "step_ms_med": "1.0",
                            "step_ms_p95": "2.0", "wall_seconds": "100",
                            "trace": "", "prereg_commit": "abc",
                            "cpu_max": "200000 100000", **extra})
    return out


def test_report_a_counts_and_reading():
    open_ = _rows(lambda a, k, r: 4)                       # everyone solves open
    blocked = _rows(lambda a, k, r: 4 if a == "arm0" else 2)
    md, c = exp_ad_report.report_a(open_, blocked)
    assert c["armB"]["open"] == 8 and c["armB"]["blocked"] == 0
    assert "tied to the blocked route" in md


def test_report_d_degraded_and_reading(tmp_path):
    e3 = _rows(lambda a, k, r: 4, rep_range=(0,))
    d = _rows(lambda a, k, r: 2 if exp_ad.clock_limited(
        REPO / "demo" / "arms" / a / f"{k}.py") else 4, rep_range=(0,))
    md = exp_ad_report.report_d(d, e3, [], tmp_path / "x.csv", tmp_path / "y.csv")
    assert "| degraded vs Exp 3 rep 0 | 9/9 | 0/7 |" in md
    assert "supported: slowing hurts" in md
