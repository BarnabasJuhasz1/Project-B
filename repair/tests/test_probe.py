"""repair/probe.py — is there anywhere this node could help?

⚠️ Almost everything here avoids a real rollout. One trial costs ~40 s (a
~10 s node rollout plus up to two ~30 s base rollouts), so the tests prime the
base cache instead. The one test that does roll out is marked `slow`.
"""
from __future__ import annotations

import pytest

from repair import probe as P


# --------------------------------------------------------------------------- #
# the axes — the known-good answer for this chunk


def test_the_ant_card_puts_drift_on_the_table():
    """⚠️ THE ONE THAT MATTERS. `to_scenario()` never sets `drift`, and this
    card's prediction opens "On scenarios where pose_estimate drifts". If the
    probe does not derive the drift axis from the card, it cannot find the case
    that motivated the whole chunk."""
    from repair.evidence import card_block
    assert "drift" in P.axes_for(card_block("gen_b_44d12df4d25b437f"))


def test_a_card_with_no_axis_words_gets_geometry_only():
    assert P.axes_for("a node that caps forward speed by clearance") == ()


def test_axes_are_derived_from_the_card_not_the_node_id():
    """The probe must not know which arm it is probing either."""
    import inspect
    params = set(inspect.signature(P.axes_for).parameters)
    assert params == {"card_text"}


def test_named_axes_are_tried_before_plain_geometry():
    """⚠️ Ordering is not cosmetic. The budget is small and a base rollout
    costs ~30 s, so trying undrifted geometry first on a node whose whole claim
    is about drift spends the budget before reaching the question."""
    plan = P.candidates(("drift",), seeds=range(4000, 4003))
    assert plan[0].drift is not None
    assert plan[-1].drift is None


def test_the_ladder_moves_one_knob():
    """sigma_yaw is pinned at half sigma_xy, so a result is attributable."""
    c = P.Candidate("maze", 4000, drift=0.10)
    s = c.scenario()
    assert s.drift.sigma_xy == 0.10
    assert s.drift.sigma_yaw == 0.05


# --------------------------------------------------------------------------- #
# a varied axis renames the scenario


def test_an_axis_change_renames_the_scenario():
    """⚠️ The same discipline `--goal-hidden` and `+cap60s` follow. A drifted
    world is NOT the same scenario as its undrifted twin and must never share
    an id with it — a committed result keyed on the plain id would otherwise be
    overwritten by a run under different conditions."""
    assert P.Candidate("maze", 4000).label == "gen-maze-04000"
    assert P.Candidate("maze", 4000, drift=0.10).label == "gen-maze-04000+drift0.10"
    assert P.Candidate("maze", 4000, goal_known=False).label == "gen-maze-04000+hidden"
    assert P.Candidate("maze", 4000, drift=0.05).label != P.Candidate("maze", 4000).label


def test_the_scenario_id_matches_the_label():
    c = P.Candidate("maze", 4000, drift=0.05)
    assert c.scenario().id == c.label


def test_the_undrifted_world_really_has_no_drift():
    """The premise of the whole chunk, asserted rather than assumed."""
    assert P.Candidate("maze", 4000).scenario().drift is None


# --------------------------------------------------------------------------- #
# ⚠️ the overfitting guards


def test_probe_and_confirmation_seeds_cannot_collide():
    """Repairing against the worlds you then score on is fitting the test set."""
    assert not (set(P.PROBE_SEEDS) & set(P.CONFIRM_SEEDS))


def test_the_probe_cannot_reach_already_measured_seeds():
    """2000-2019 and 3000-3019 are spent: growth round 2 and its replication.
    A probe that returned one of those would be "finding" a world the node has
    already been measured on."""
    assert not (set(P.PROBE_SEEDS) & P.MEASURED_SEEDS)
    assert not (set(P.CONFIRM_SEEDS) & P.MEASURED_SEEDS)


def test_the_default_plan_never_leaves_the_probe_range():
    for c in P.candidates(("drift", "goal_known"), seeds=None)[:200]:
        assert c.seed in P.PROBE_SEEDS


def test_the_whole_search_is_recorded_not_just_the_hit():
    """Same discipline as `compose/retrieve.py`: a miss must be attributable."""
    trials = (
        P.Trial(P.Candidate("maze", 4000, drift=0.05), False, "trigger never fired"),
        P.Trial(P.Candidate("maze", 4001, drift=0.05), False,
                "base cannot solve this world even without the axis"),
    )
    log = P.ProbeResult("n", None, trials, ("drift",), 12.0, "UNTESTABLE").log()
    assert "trigger never fired" in log
    assert "even without the axis" in log
    assert "UNTESTABLE" in log
    assert log.count("gen-maze-") == 2, "every candidate considered must appear"


# --------------------------------------------------------------------------- #
# the load-bearing-axis test — the defect this chunk's first run found


class _NeverCalled:
    """A runner that fails loudly if anything tries to roll out.

    ⚠️ `__iter__` is defined explicitly: implicit dunder lookup goes to the
    TYPE, not through `__getattr__`, so a tripwire relying on `__getattr__`
    alone would raise a confusing TypeError on tuple-unpacking instead of
    saying what went wrong — or, worse, read as a different failure.
    """

    def __getattr__(self, name):
        raise AssertionError(f"a cache hit must not roll out (touched {name})")

    def __iter__(self):
        raise AssertionError("a cache hit must not roll out (unpacked runner)")


def test_an_axis_that_changes_nothing_is_rejected():
    """⚠️ THE DEFECT THE FIRST REAL RUN FOUND. The probe returned
    `gen-maze-04000+drift0.05` as discriminating because the base failed there
    — but the base fails on seed 4000 **with or without drift**. The axis
    contributed nothing and the probe had found a hard world wearing a drift
    label. A repair tested against it would have been tested against geometry."""
    cache = {"gen-maze-04000": {"goal_reached": False, "path_length": 5.0}}
    state, why = P.axis_effect(P.Candidate("maze", 4000, drift=0.05),
                               False, cache, _NeverCalled())
    assert state == "unsolvable"
    assert "even without the axis" in why


def test_an_axis_that_takes_the_task_away_is_accepted():
    """The world the ant card's own comparison asks for: the base can do it
    when its pose is good, and drift is what removes that."""
    cache = {"gen-maze-04002": {"goal_reached": True, "path_length": 20.0}}
    state, why = P.axis_effect(P.Candidate("maze", 4002, drift=0.05),
                               False, cache, _NeverCalled())
    assert state == "axis_breaks"
    assert "takes it away" in why


def test_plain_geometry_has_no_axis_to_be_load_bearing():
    state, why = P.axis_effect(P.Candidate("maze", 4000), False, {}, _NeverCalled())
    assert state is None and "no axis" in why


def test_a_base_that_copes_is_not_an_axis_effect():
    """⚠️ THE SECOND DEFECT. Seed 4005: the base solves it BOTH ways (27.5 m
    clean, 40.3 m under drift). The first fix checked only the clean run and
    then claimed "the axis takes it away", which was simply false. Half a check
    with a whole claim attached is worse than no check."""
    cache = {"gen-maze-04005": {"goal_reached": True, "path_length": 27.5}}
    state, why = P.axis_effect(P.Candidate("maze", 4005, drift=0.05),
                               True, cache, _NeverCalled())
    assert state == "base_copes"
    assert "takes it away" not in why


# --------------------------------------------------------------------------- #
# one definition of "did it fire"


def test_the_floors_are_the_verdict_rules_floors():
    """⚠️ "Did it fire" must mean ONE thing across the loop. Two definitions
    would let a node read as INERT in the verdict and active in the probe."""
    from repair import verdict as V
    assert P.FIRE_FLOOR is V.FIRE_FLOOR
    assert P.CHANGED_FLOOR is V.CHANGED_FLOOR


def test_untestable_says_it_is_a_statement_about_the_search():
    r = P.ProbeResult("n", None, (), ("drift",), 1.0,
                      "UNTESTABLE within this budget.")
    assert r.untestable
    assert "budget" in r.note


# --------------------------------------------------------------------------- #


@pytest.mark.slow
def test_a_real_probe_finds_a_drift_world_for_the_ant_node():
    """The expensive one. ~40 s per trial, less on cache hits."""
    r = P.probe("gen_b_44d12df4d25b437f", budget=4)
    assert r.axes == ("drift",)
    assert r.trials, "a probe that ran nothing proves nothing"
    if r.found:
        assert r.found.drift is not None


def test_a_legacy_cache_entry_still_answers_the_probes_question():
    """⚠️ Caching the FULL metric set for the verdict rule must not make the
    probe re-roll. An entry written before `score` was cached still says
    whether the base solved the world, which is all the probe reads — routing
    it through `base_score` cost 20 rollouts to learn what the cache knew."""
    cache = {"gen-maze-04000": {"goal_reached": True, "path_length": 5.0}}
    reached, hit = P.base_result(P.Candidate("maze", 4000), cache, _NeverCalled())
    assert reached is True and hit is True


# --------------------------------------------------------------------------- #
# resolution — a REPAIRED node is not in the live library


def test_a_repair_round_resolves_and_is_not_a_keyerror(staged_round):
    """⚠️ THE ONE THAT KILLED A LIVE RUN. `probe()` resolved its node through
    `load_library().get`, which only knows `generate/staging/`. Repair rounds
    stage under `out/repair/<lineage>/round-NN/`, so the first round that ever
    chose PROBE_ENVIRONMENT died on `KeyError: no node '…__r01' in the library`
    — mid-round-2, after the diagnosis call had already been paid for.

    `budget=0` resolves the node and plans the search without rolling anything
    out, which is the whole of what regressed.
    """
    r = P.probe(staged_round, budget=0)
    assert "could not be resolved" not in r.note
    assert r.axes == ("drift",), "the parent's card still supplies the axes"


def test_an_unresolvable_node_is_a_RECORDED_outcome_not_a_traceback():
    """A probe that cannot resolve its node loses the loop a round, not the
    run — the rule `run_round` already applies to a failed regeneration. It
    still carries `entry_for`'s reason, which is what separates "no such round"
    from "that round staged two nodes"."""
    r = P.probe("gen_b_44d12df4d25b437f__r99", budget=0)
    assert r.untestable and r.trials == ()
    assert "could not be resolved" in r.note


def test_the_ladder_is_reachable_at_a_real_budget():
    """⚠️ THE LADDER USED TO BE UNREACHABLE PAST ITS FIRST RUNG. `candidates`
    was sigma-major over PROBE_SEEDS, which is 500 wide, so rung 0.10 sat at
    index 500 and rung 0.20 at index 1000. A live probe at the loop's default
    budget of 6 tested nothing but a body-length error and reported UNTESTABLE.

    The ladder's own comment says the rungs are a body length, a corridor width
    and a doorway; a search that cannot reach two of them is not a search."""
    plan = P.candidates(("drift",))
    reached = {c.drift for c in plan[:len(P.DRIFT_LADDER)]}
    assert reached == set(P.DRIFT_LADDER), (
        f"a budget of {len(P.DRIFT_LADDER)} must reach every rung, got {reached}")
