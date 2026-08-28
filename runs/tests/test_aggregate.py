"""
runs/tests/test_aggregate.py

The aggregation, on synthetic result files. What is under test is the set of
refusals -- the places where scoring what happens to be on disk would produce a
number that looks valid and is not.

    pytest runs/tests/test_aggregate.py -q
"""

import json
import os

import pytest

from runs.aggregate import (CAVEATS, Incomplete, PER_FAMILY, PRIMARY,
                            SECONDARIES, collect, report, secondary,
                            secondary_by_family, verdict)
from runs.design import DEV, sweep_design
from runs.spec import Condition, SweepSpec


def spec():
    return SweepSpec((Condition("B", "reactive_avoid", "lib-v1"),
                      Condition("D", "goal_seeker", "lib-v1")),
                     name="test", reportable=False)


def write_results(tmp_path, plan, success_for, extra=None):
    """One result file per episode. `success_for(episode) -> bool`."""
    for ep in plan.episodes:
        ok = success_for(ep)
        result = {
            "scenario": ep.scenario, "seed": ep.seed, PRIMARY: ok,
            "spl": 0.9 if ok else 0.0,
            "collisions": 0 if ok else 3,
            "stuck_events": 0 if ok else 2,
            "time_to_goal": 12.0 if ok else None,
            "localisation_failures": 1,
        }
        result.update((extra or {}))
        payload = {"result": result,
                   "manifest": {"prereg_digest": "d" * 64,
                                "library_snapshot": "lib-v1"}}
        (tmp_path / ep.result_name).write_text(json.dumps(payload))


def test_a_missing_result_is_refused(tmp_path):
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: True)
    os.remove(tmp_path / plan.episodes[0].result_name)
    with pytest.raises(Incomplete, match="different benchmarks"):
        collect(spec(), plan, results_dir=str(tmp_path), split=DEV)


def test_the_refusal_says_which_condition_is_short(tmp_path):
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: True)
    for ep in [e for e in plan.episodes if e.condition == "D"][:2]:
        os.remove(tmp_path / ep.result_name)
    with pytest.raises(Incomplete, match=r"D: 2 missing"):
        collect(spec(), plan, results_dir=str(tmp_path), split=DEV)


def test_a_complete_set_collects(tmp_path):
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: e.condition == "B")
    got = collect(spec(), plan, results_dir=str(tmp_path), split=DEV)
    assert got.rate("B") == 1.0
    assert got.rate("D") == 0.0


def test_the_verdict_comes_from_core_stats(tmp_path):
    """Not recomputed here. A second implementation of the rule is a second
    rule, and only one of them was pre-registered."""
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: e.condition == "B")
    got = collect(spec(), plan, results_dir=str(tmp_path), split=DEV)
    v = verdict(got)
    assert v.gap == 1.0
    assert v.fires


def test_spl_is_only_available_per_family():
    """PRE-REGISTRATION.md §5. Pooling it across four families produces a
    number that means nothing in particular, so the API does not offer it."""
    assert "spl" in PER_FAMILY
    assert "spl" in CAVEATS


def test_time_to_goal_is_conditioned_on_success(tmp_path):
    """A fast mean over two arrivals is not better than a slower one over
    twelve, so the n travels with the mean."""
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: e.condition == "B")
    got = collect(spec(), plan, results_dir=str(tmp_path), split=DEV)
    mean, n, note = secondary(got, "B", "time_to_goal")
    assert mean == pytest.approx(12.0) and n == len(plan) // 2
    mean_d, n_d, _ = secondary(got, "D", "time_to_goal")
    assert mean_d is None and n_d == 0
    assert "success" in note


def test_a_never_arriving_condition_reports_none_not_zero(tmp_path):
    """0.0 would read as 'instant'."""
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: False)
    got = collect(spec(), plan, results_dir=str(tmp_path), split=DEV)
    assert secondary(got, "B", "time_to_goal")[0] is None


def test_per_family_breaks_out_by_family(tmp_path):
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: True)
    got = collect(spec(), plan, results_dir=str(tmp_path), split=DEV)
    per = secondary_by_family(got, "B", "spl")
    assert set(per) == {e.family for e in plan.episodes}


def test_mixed_prereg_digests_are_flagged_loudly(tmp_path):
    """Episodes scored under different constants are not comparable, and the
    report has to say so rather than averaging them."""
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: True)
    victim = plan.episodes[0]
    payload = json.loads((tmp_path / victim.result_name).read_text())
    payload["manifest"]["prereg_digest"] = "e" * 64
    (tmp_path / victim.result_name).write_text(json.dumps(payload))
    got = collect(spec(), plan, results_dir=str(tmp_path), split=DEV)
    assert "MORE THAN ONE PRE-REGISTRATION DIGEST" in report(got)


def test_the_report_names_every_secondary_and_no_others(tmp_path):
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: e.condition == "B")
    text = report(collect(spec(), plan, results_dir=str(tmp_path), split=DEV))
    for metric in SECONDARIES:
        assert metric in text
    assert "near_misses" not in text, "an unnamed secondary reached the report"


def test_a_non_firing_rule_is_not_read_as_a_null(tmp_path):
    """PRE-REGISTRATION.md §2: the rule is conservative and not firing is not
    evidence the corpus does not help. The report has to say so itself."""
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: False)
    text = report(collect(spec(), plan, results_dir=str(tmp_path), split=DEV))
    assert "conservative" in text


def test_localisation_failures_is_a_manipulation_check_in_the_report(tmp_path):
    plan = sweep_design(spec(), splits=(DEV,))
    write_results(tmp_path, plan, lambda e: True)
    text = report(collect(spec(), plan, results_dir=str(tmp_path), split=DEV))
    assert "MANIPULATION CHECKS" in text
    assert "localisation_failures" in text
    assert "never compared" in text
