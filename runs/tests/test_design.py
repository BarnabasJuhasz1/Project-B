"""
runs/tests/test_design.py

The episode set. The properties here are the ones that, if broken, produce a
complete and plausible table that means nothing.

    pytest runs/tests/ -q
"""

import pytest

from core.stats import SEED_OFFSETS, seeds_for
from runs.design import DEV, HELD_OUT, Episode, SweepPlan, sweep_design
from runs.spec import Condition, SweepSpec


def spec(*labels):
    names = {"A": "goal_seeker", "B": "reactive_avoid", "D": "scaffold_sweep"}
    return SweepSpec(tuple(Condition(l, names[l]) for l in labels),
                     name="test", reportable=False)


def test_the_plan_is_balanced_across_conditions():
    """Non-negotiable #10, as a property of the design rather than a hope."""
    plan = sweep_design(spec("A", "B"))
    assert plan.is_balanced()
    counts = set(plan.per_condition().values())
    assert len(counts) == 1


def test_the_count_is_scenarios_times_seeds_times_conditions():
    plan = sweep_design(spec("A", "B"))
    assert plan.per_split()[HELD_OUT] == 12 * len(SEED_OFFSETS) * 2
    assert plan.per_split()[DEV] == 3 * len(SEED_OFFSETS) * 2
    assert len(plan) == 15 * len(SEED_OFFSETS) * 2


def test_seeds_come_from_the_pre_registered_rule():
    """Not a list typed here. If `core.stats.SEED_OFFSETS` moves, this moves,
    and `test_preregistration.py` makes moving it require a document edit."""
    plan = sweep_design(spec("A"), splits=(DEV,))
    for scenario in {e.scenario for e in plan.episodes}:
        seeds = {e.seed for e in plan.episodes if e.scenario == scenario}
        base = min(seeds)
        assert seeds == set(seeds_for(base))


def test_replicate_zero_is_the_scenarios_own_seed():
    """PRE-REGISTRATION.md §3: replicate 0 must be the sealed episode itself."""
    from core.scenario import DEV_SCENARIOS
    plan = sweep_design(spec("A"), splits=(DEV,))
    for s in DEV_SCENARIOS:
        assert any(e.scenario == s.id and e.seed == s.seed for e in plan.episodes)


def test_the_order_is_scenario_major_condition_minor():
    """So that an interrupted sweep leaves a BALANCED partial set. Condition-
    major would leave one condition finished and the rest untouched."""
    plan = sweep_design(spec("A", "B"), splits=(DEV,))
    first_four = plan.episodes[:4]
    assert {e.condition for e in first_four[:2]} == {"A", "B"}
    assert first_four[0].scenario == first_four[1].scenario


def test_held_out_runs_before_dev():
    """An interrupted sweep should have spent its time on the set the decision
    rule actually reads."""
    plan = sweep_design(spec("A"))
    splits = [e.split for e in plan.episodes]
    assert splits.index(DEV) > max(i for i, s in enumerate(splits) if s == HELD_OUT)


def test_run_ids_are_unique():
    plan = sweep_design(spec("A", "B"))
    ids = [e.run_id for e in plan.episodes]
    assert len(ids) == len(set(ids))


def test_a_composed_controller_gets_a_filename_safe_run_id():
    """A colon is illegal on NTFS and ambiguous to scp. core.manifest.run_id()
    is the single place that deals with it."""
    ep = Episode("B", "composed:9c21b87c", "held_x", 2000, HELD_OUT, "nav", 120.0)
    assert ":" not in ep.run_id
    assert ep.result_name.endswith(".json")


def test_worst_case_sim_seconds_matches_the_pre_registration():
    """PRE-REGISTRATION.md §3 quotes 1920 s of held-out sim per condition per
    seed. If the split's limits ever move, that sentence is wrong and this
    fails rather than the document quietly going stale."""
    plan = sweep_design(spec("A"), splits=(HELD_OUT,))
    per_seed = plan.worst_case_sim_seconds() / len(SEED_OFFSETS)
    assert per_seed == pytest.approx(1920.0)


def test_an_unbalanced_plan_is_detected():
    plan = sweep_design(spec("A", "B"))
    truncated = SweepPlan(plan.episodes[:3])
    assert not truncated.is_balanced()


def test_every_episode_carries_its_split_and_family():
    """The aggregation reports per family and never pools dev with held-out;
    it can only do that if the label rides on the episode."""
    for ep in sweep_design(spec("A")).episodes:
        assert ep.split in (HELD_OUT, DEV)
        assert ep.family
