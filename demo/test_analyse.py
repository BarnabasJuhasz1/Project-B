"""demo/test_analyse.py — the pre-registered analysis and the power script,
on the pilot data and on synthetic fixtures. `python demo/test_analyse.py`, or
pytest. SIM-FREE, calls nothing, ~5 s."""
import csv
import hashlib
import json
import math
import pathlib
import sys

import numpy as np
import pytest

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from demo import analyse as A          # noqa: E402
from demo import power as PW           # noqa: E402

PILOT = HERE / "fixtures" / "pilot_physics.csv"
PILOT_SHA256 = "e4986fc6129da6ab6e23ea3fd510d8c7f1d6e3069c348f97beac27b78d050e55"


# --- the pilot --------------------------------------------------------------------

def test_pilot_fixture_is_the_committed_physics_csv():
    """The fixture is a verbatim copy of demo/out/physics.csv (2026-10-03). If
    the original is on disk it must still be byte-identical."""
    assert hashlib.sha256(PILOT.read_bytes()).hexdigest() == PILOT_SHA256
    orig = HERE / "out" / "physics.csv"
    if orig.exists():
        assert orig.read_bytes() == PILOT.read_bytes()


def test_pilot_reproduces_7_of_8_vs_2_of_8_p_0041():
    r = A.pilot(PILOT)
    assert (r["arms"]["arm0"]["x"], r["arms"]["arm0"]["n"]) == (7, 8)
    assert (r["arms"]["armB"]["x"], r["arms"]["armB"]["n"]) == (2, 8)
    assert r["p"] == pytest.approx(0.0406, abs=5e-4)
    assert r["mean_legs"]["arm0"] == pytest.approx(3.75)
    assert r["mean_legs"]["armB"] == pytest.approx(2.375)
    # The legs >= 3 rule physics_sweep.py prints agrees on this data (plan §0).
    assert r["legs_ge_3"] == {"arm0": 7, "armB": 2}


# --- intervals and tests, against published values ---------------------------------

def test_wilson_matches_newcombe_1998():
    lo, hi = A.wilson(81, 263)
    assert (round(lo, 4), round(hi, 4)) == (0.2553, 0.3662)
    assert A.wilson(0, 10)[0] == 0.0
    assert A.wilson(10, 10)[1] == pytest.approx(1.0)


def test_newcombe_method_10_matches_newcombe_1998():
    lo, hi = A.newcombe(56, 70, 48, 80)
    assert (round(lo, 4), round(hi, 4)) == (0.0524, 0.3339)


def test_holm_steps_down_and_stops():
    r = A.holm({"a": 0.01, "b": 0.04, "c": 0.03})
    assert r["a"] == (pytest.approx(0.03), True)
    assert r["c"] == (pytest.approx(0.06), False)
    assert r["b"] == (pytest.approx(0.06), False)   # monotone, never below c


def test_holm_agrees_with_power_scripts_vectorised_holm():
    rng = np.random.default_rng(0)
    for _ in range(500):
        ps = rng.uniform(0, 0.08, 3)
        ref = A.holm(dict(zip("xyz", ps)))
        vec = PW.holm_reject(ps)
        assert [ref[n][1] for n in "xyz"] == list(vec)


def test_kappa():
    assert A.cohen_kappa(list("aabb"), list("aabb")) == 1.0
    assert A.cohen_kappa(list("abab"), list("aabb")) == pytest.approx(0.0)


# --- the controller verdict --------------------------------------------------------

def _eps(legs):
    return {r: {"legs": l, "leg_times": [20.0] * l, "contacts": 1.0,
                "path_m": 80.0, "track_v_med": 0.05, "note": ""}
            for r, l in enumerate(legs)}


@pytest.mark.parametrize("legs,ok", [([4, 4, 4], True), ([4, 4, 2], True),
                                     ([4, 2, 2], False), ([3, 3, 3], False),
                                     ([2, 4, 4], True)])
def test_majority_of_three_all_four_trips(legs, ok):
    assert A.verdict(_eps(legs))["success"] is ok


def test_leg2_never_completed_is_infinite_not_zero():
    v = A.verdict(_eps([2, 2, 4]))
    assert v["leg2_time"] == math.inf
    assert A.verdict(_eps([4, 4, 2]))["leg2_time"] == 20.0


# --- synthetic fixtures -----------------------------------------------------------

def write_physics(path, k, success, reps=3, pilot=True, drop=(), extra=()):
    """success: {arm: number of fresh controllers that succeed}. A succeeding
    controller scores [4, 4, 2] or [4, 4, 4]; a failing one [2, 2, 4] or
    [2, 1, 2] -- so the majority rule, not any single rep, decides."""
    rows = []
    def add(arm, kk, legs, times=None):
        for rep, l in enumerate(legs[:reps]):
            if (arm, kk, rep) in drop:
                continue
            rows.append({"arm": arm, "k": f"k{kk}", "rep": rep, "legs": l,
                         "total": 4, "leg_times": json.dumps(times or [21.0] * l),
                         "contacts": 2 if arm == "armB" else 1, "path_m": 90.0,
                         "track_v_med": 0.06, "note": ""})
    for arm in A.ARMS:
        for i in range(k):
            kk = A.FRESH_START[arm] + i
            ok = i < success[arm]
            add(arm, kk, ([4, 4, 2] if i % 2 else [4, 4, 4]) if ok
                else ([2, 2, 4] if i % 2 else [2, 1, 2]))
    if pilot:
        ref = A.episodes(A.read_rows(PILOT), single_rep=True)
        for (arm, kk), e in ref.items():
            l = e[0]["legs"]
            add(arm, kk, [l, l, l], times=e[0]["leg_times"])
    rows += list(extra)
    rows.append({"arm": "(base)", "k": "reference", "rep": 0, "legs": 3})
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["arm", "k", "rep", "legs", "total",
                                           "leg_times", "contacts", "path_m",
                                           "track_v_med", "note"])
        w.writeheader(); w.writerows(rows)
    return path


def run(tmp_path, k, success, **kw):
    phys = write_physics(tmp_path / "physics.csv", k, success, **kw)
    return A.analyse(k, phys, kinematic=None, labels=None, reference=PILOT,
                     transcripts=tmp_path)


def test_content_pattern_is_read_as_content(tmp_path):
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22})
    t = o["primary"]["tests"]
    assert o["primary"]["arms"]["armB"]["x"] == 6
    assert t["H1"]["holm_reject"] and t["H2"]["holm_reject"]
    assert not t["H3"]["holm_reject"]
    assert o["primary"]["reading"] == "CONTENT"
    assert "worse" in o["primary"]["reading_text"]
    assert o["status"] == "CONFIRMATORY"


def test_length_pattern_is_read_as_length(tmp_path):
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 7})
    assert o["primary"]["reading"] == "LENGTH"


def test_null_is_read_as_not_replicated(tmp_path):
    o = run(tmp_path, 24, {"arm0": 18, "armB": 17, "armX": 18})
    assert o["primary"]["reading"] == "NOT_REPLICATED"


def test_unclear_pattern_makes_no_claim(tmp_path):
    # H1 rejected, armX halfway: neither H2 nor H3 survives.
    o = run(tmp_path, 30, {"arm0": 28, "armB": 8, "armX": 18})
    t = o["primary"]["tests"]
    assert t["H1"]["holm_reject"]
    assert o["primary"]["reading"] == "NO_CLAIM"


def test_primary_uses_fresh_only_and_secondary_pools(tmp_path):
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22})
    assert o["primary"]["arms"]["arm0"]["n"] == 24
    pooled = o["secondary"]["pooled"]
    assert pooled["arm0"]["n"] == 32 and pooled["armB"]["n"] == 32
    assert pooled["arm0"]["x"] == 22 + 7 and pooled["armB"]["x"] == 6 + 2
    assert "S5 pooled arm0 vs armB" in o["secondary"]["tests"]
    # the (base) reference row never enters any test
    assert all(a in A.ARMS for a in o["primary"]["arms"])


def test_rep0_check_reproduces_the_pilot(tmp_path):
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22})
    r = o["exp3"]["rep0_check"]
    assert (r["match"], r["n"]) == (16, 16)


def test_secondary_family_is_holm_corrected_and_complete(tmp_path):
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22})
    s = o["secondary"]
    names = set(s["tests"])
    for h in ("H1", "H2", "H3"):
        assert f"S1 mean trips {h}" in names and f"S2 leg-2 time {h}" in names
        assert f"S3 coll/m {h}" in names
    assert any("S4" in u for u in s["unavailable"])        # no kinematic.csv given
    for t in s["tests"].values():
        assert t["p_holm"] >= t["p"] - 1e-12


def test_bootstrap_and_agreement(tmp_path):
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22})
    b = o["exp3"]["bootstrap"]["H1"]
    assert b["ci"][0] <= b["diff"] <= b["ci"][1] < 0
    ag = o["exp3"]["agreement"]["arm0"]
    assert ag["n"] == 24 and ag["agree"] == 11 + 1      # 11 x [4,4,4] + one [2,1,2]
    # deterministic: the seed is fixed
    assert run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22}
               )["exp3"]["bootstrap"]["H1"] == b


def test_incomplete_set_is_refused(tmp_path):
    with pytest.raises(A.Incomplete):
        run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22},
            drop={("armX", 5, 2)})


def test_allow_incomplete_is_stamped_exploratory(tmp_path):
    phys = write_physics(tmp_path / "physics.csv", 24,
                         {"arm0": 22, "armB": 6, "armX": 22},
                         drop={("armX", 5, 2)})
    o = A.analyse(24, phys, reference=PILOT, transcripts=tmp_path,
                  allow_incomplete=True)
    assert o["status"].startswith("EXPLORATORY")


def test_apparatus_failure_row_is_not_scored(tmp_path):
    """A row with empty legs (NO RESULT LINE) is dropped -> the set is short."""
    extra = [{"arm": "armX", "k": "k99", "rep": 0, "legs": "", "note": "NO RESULT LINE"}]
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22}, extra=extra)
    assert o["primary"]["arms"]["armX"]["n"] == 24


def test_duplicate_episode_is_refused(tmp_path):
    extra = [{"arm": "arm0", "k": "k8", "rep": 0, "legs": 4}]
    with pytest.raises(ValueError, match="duplicate"):
        run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22}, extra=extra)


def test_generation_failure_counts_as_zero_trip_controller(tmp_path):
    drop = {("armB", 31, r) for r in range(3)}
    (tmp_path / "armB__k31__FAILED.json").write_text("{}")
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22}, drop=drop)
    s = o["primary"]["arms"]["armB"]
    assert s["n"] == 24 and s["generation_failures"] == 1
    assert any("no path left out" in u for u in o["secondary"]["unavailable"])


def test_missing_controller_without_failed_transcript_is_refused(tmp_path):
    drop = {("armB", 31, r) for r in range(3)}
    with pytest.raises(A.Incomplete):
        run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22}, drop=drop)


def test_kinematic_secondary(tmp_path):
    phys = write_physics(tmp_path / "physics.csv", 24, {"arm0": 22, "armB": 6, "armX": 22})
    kin = write_physics(tmp_path / "kinematic.csv", 24,
                        {"arm0": 20, "armB": 19, "armX": 20}, pilot=False)
    o = A.analyse(24, phys, kinematic=kin, reference=PILOT, transcripts=tmp_path)
    assert o["secondary"]["tests"]["S4 kinematic H1"]["p"] > 0.5


def test_exp4_labels_kappa_e10_and_association(tmp_path):
    phys = write_physics(tmp_path / "physics.csv", 24, {"arm0": 22, "armB": 6, "armX": 22})
    rows = []
    for arm in A.ARMS:
        for i in range(24):
            kk = A.FRESH_START[arm] + i
            ok = i < {"arm0": 22, "armB": 6, "armX": 22}[arm]
            pt = ("incremental_only" if (arm == "armB" and not ok)
                  else "full_every_step")
            for rater in ("r1", "r2", "consensus"):
                lab = pt
                if rater == "r2" and arm == "armX" and i == 0:
                    lab = "full_periodic"          # one disagreement
                rows.append({"blind_id": f"{arm}{kk}", "rater": rater,
                             "planner_type": lab, "map": "persistent_grid",
                             "velocity_selection": "path_following",
                             "stuck_handling": "none"})
    lab_path = tmp_path / "code_labels.csv"
    with open(lab_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
    key_path = tmp_path / "key.csv"
    with open(key_path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["blind_id", "arm", "k"]); w.writeheader()
        for arm in A.ARMS:
            for i in range(24):
                kk = A.FRESH_START[arm] + i
                w.writerow({"blind_id": f"{arm}{kk}", "arm": arm, "k": f"k{kk}"})
    o = A.analyse(24, phys, labels=lab_path, code_key=key_path,
                  reference=PILOT, transcripts=tmp_path)
    x = o["exp4"]
    assert x["e10"]["arm0"]["x"] == 0
    assert x["e10"]["armB"]["x"] == 18
    assert x["assoc_all"]["incremental_only"] == (0, 18)
    assert x["assoc_all"]["p"] < 1e-6
    assert x["assoc_armB"]["p"] < 1e-3
    assert 0.9 < x["kappa"]["planner_type"][0] < 1.0
    assert len(x["unlabelled"]) == 16                     # the pilot, uncoded here


def test_unknown_planner_label_is_refused(tmp_path):
    p = tmp_path / "l.csv"
    p.write_text("arm,k,planner_type\narm0,k8,D* Lite\n")
    with pytest.raises(ValueError, match="planner_type"):
        A.read_labels(p)


def test_cli_without_k_refuses_confirmatory(tmp_path):
    phys = write_physics(tmp_path / "physics.csv", 24, {"arm0": 22, "armB": 6, "armX": 22})
    if A.K_CONFIRMATORY is None:
        with pytest.raises(SystemExit):
            A.main(["--physics", str(phys), "--pilot-csv", str(PILOT)])


def test_format_report_runs(tmp_path):
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22})
    text = A.format_report(o)
    assert "READING: CONTENT" in text and "EXPLORATORY" in text


# --- power --------------------------------------------------------------------------

def test_power_pilot_at_n8_and_size_under_null():
    assert PW.power_two_arm(8, 0.875, 0.25) == pytest.approx(0.649, abs=2e-3)
    # Fisher is conservative: the size under any common rate is <= alpha.
    for p in (0.3, 0.5, 0.75, 0.9):
        assert PW.power_two_arm(24, p, p) <= 0.05


def test_power_three_arm_sums_to_one_and_matches_two_arm_bound():
    r = PW.three_arm(24, 0.85, 0.5, 0.85)
    assert sum(r[k] for k in ("content", "length", "not_replicated", "other")) \
        == pytest.approx(1.0)
    # Holm power sits between the Bonferroni and the unadjusted single test.
    assert (PW.power_two_arm(24, 0.85, 0.5, 0.05 / 3) - 1e-9 <= r["H1"]
            <= PW.power_two_arm(24, 0.85, 0.5) + 1e-9)


def test_power_reading_logic_matches_analyse(tmp_path):
    """power.three_arm and analyse.interpret must define 'content' alike: one
    synthetic outcome, scored both ways."""
    o = run(tmp_path, 24, {"arm0": 22, "armB": 6, "armX": 22})
    t = PW.p_table(24, 24)
    ps = np.array([t[6, 22], t[6, 22], t[22, 22]])
    rej = PW.holm_reject(ps)
    assert list(rej) == [o["primary"]["tests"][h]["holm_reject"]
                         for h in ("H1", "H2", "H3")]


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-q"]))
