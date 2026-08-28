"""
core/tests/test_stats.py

The decision rule, tested on synthetic outcomes.

`core.stats.decide()` is the thing that will be pointed at in Week 5 and asked
"did B beat D". These tests pin the properties that make its answer mean
something: the resampling unit is the scenario, the two conditions are resampled
together, and a comparison over different scenario sets is refused rather than
computed.

    pytest core/tests/test_stats.py -q
"""

import pytest

from core.stats import (X, EpisodeOutcome, SEED_OFFSETS, decide,
                        outcomes_from_results, seeds_for)


def outcomes(pattern, seeds=3, prefix="s"):
    """`pattern` is one success flag per scenario, repeated over seeds."""
    return [EpisodeOutcome(f"{prefix}{i:02d}", 2000 + k, bool(v))
            for i, v in enumerate(pattern)
            for k in range(seeds)]


def test_a_clean_sweep_fires_the_rule():
    v = decide(outcomes([1] * 12), outcomes([0] * 12))
    assert v.gap == 1.0
    assert v.ci_disjoint
    assert v.fires


def test_identical_conditions_do_not_fire():
    same = [1, 1, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0]
    v = decide(outcomes(same), outcomes(same))
    assert v.gap == 0.0
    assert not v.fires
    assert not v.paired_excludes_zero


def test_a_gap_below_x_cannot_fire_however_tight_the_intervals():
    """X is a floor on what is worth claiming, applied before the intervals."""
    b = [1] * 3 + [0] * 9          # 0.25
    d = [0] * 12                   # 0.00
    v = decide(outcomes(b), outcomes(d))
    assert 0 < v.gap < X
    assert not v.fires


def test_the_resampling_unit_is_the_scenario_not_the_episode():
    """Adding seeds to an outcome that does not vary within a scenario must not
    narrow the interval. If it does, the bootstrap is counting one episode many
    times, which is exactly what the undrifted half of the split would do."""
    pattern = [1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0]
    narrow = decide(outcomes(pattern, seeds=1), outcomes([0] * 12, seeds=1))
    wide = decide(outcomes(pattern, seeds=10), outcomes([0] * 12, seeds=10))
    assert narrow.b_ci == pytest.approx(wide.b_ci)
    assert narrow.n_episodes_b * 10 == wide.n_episodes_b


def test_seeds_that_do_vary_within_a_scenario_are_averaged_not_dropped():
    b = [EpisodeOutcome("s00", 1, True), EpisodeOutcome("s00", 2, False),
         EpisodeOutcome("s01", 1, True), EpisodeOutcome("s01", 2, True)]
    d = [EpisodeOutcome(o.scenario, o.seed, False) for o in b]
    v = decide(b, d)
    assert v.b_rate == pytest.approx(0.75)      # (0.5 + 1.0) / 2


def test_a_comparison_over_different_scenarios_is_refused():
    """#10: identical episode counts across conditions. A condition scored over
    ten of twelve scenarios is a different benchmark, not a weaker result."""
    b = outcomes([1] * 12)
    d = [o for o in outcomes([0] * 12) if o.scenario != "s11"]
    with pytest.raises(ValueError, match="different benchmarks"):
        decide(b, d)


def test_the_bootstrap_is_reproducible():
    b, d = outcomes([1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0]), outcomes([1] + [0] * 11)
    assert decide(b, d).b_ci == decide(b, d).b_ci


def test_both_conditions_are_resampled_on_the_same_indices():
    """The pairing. If the two were resampled independently, a perfectly paired
    advantage would still show a paired interval straddling zero at n = 12."""
    b = outcomes([1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0])
    d = outcomes([1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0])
    v = decide(b, d)
    assert v.paired_diff == pytest.approx(4 / 12)
    assert v.paired_ci[0] > 0.0


def test_the_seed_rule_is_an_offset_from_the_scenarios_own_seed():
    assert seeds_for(2007) == tuple(2007 + off for off in SEED_OFFSETS)
    assert seeds_for(2007)[0] == 2007, "replicate 0 must be the sealed episode"


def test_outcomes_are_built_from_score_dicts_unchanged():
    rows = [{"scenario": "s00", "seed": 2000, "goal_reached": True,
             "spl": 0.9, "collisions": 0}]
    got = outcomes_from_results(rows)
    assert got == [EpisodeOutcome("s00", 2000, True)]


def test_the_verdict_reports_everything_even_when_it_does_not_fire():
    v = decide(outcomes([1] * 2 + [0] * 10), outcomes([0] * 12))
    assert not v.fires
    for field in (v.gap, v.b_rate, v.d_rate, v.paired_diff):
        assert field is not None
    assert "does not fire" in v.summary()


def test_an_empty_condition_raises_rather_than_scoring_zero():
    with pytest.raises(ValueError):
        decide([], outcomes([0] * 12))


def test_the_summary_names_the_pair_it_is_actually_about():
    """B-D is the headline, but the same rule is applied to B-A, which is
    confounded with principle articulation. A summary that always printed "D"
    would mislabel the contrast in the write-up."""
    b, a = outcomes([1] * 12), outcomes([0] * 12)
    v = decide(b, a, labels=("B", "A"))
    assert v.labels == ("B", "A")
    assert "A 0.000" in v.summary()
    assert "D " not in v.summary()
    assert decide(b, a).labels == ("B", "D")
