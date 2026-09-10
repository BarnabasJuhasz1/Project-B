"""
harness/tests/test_ablate.py

Leave-one-out node contribution. The measurement that replaced chunk 37's gate.

Every test here is SIM-FREE and none touches the sealed split — the ablation
runs on the DEV scenarios only, deliberately, because it feeds back into which
nodes get built and running it on the held-out set is the leak G3 exists to
prevent.
"""

import pytest

from core.scenario import DEV_SCENARIOS
from harness.ablate import ablatable, ablate, contribution
from nodes.tree import Composite, Tree

COMPOSED = "composed:navigate_to_goal__goal_known_no_drift__56b3466284b7d9e8"


@pytest.fixture
def tree_factory():
    import controllers
    return lambda: controllers.make(COMPOSED)


def test_the_standin_preserves_the_trees_shape(tree_factory):
    """**The load-bearing property.** `Sequence` short-circuits on anything that
    is not SUCCESS and `Fallback` on anything that is not FAILURE, so DELETING a
    child silently promotes every child after it — the ablated tree would then
    differ from the intact one in two ways and the measurement would be
    uninterpretable. Replacing keeps the shape identical."""
    intact = tree_factory()
    ablated = ablate(tree_factory(), "toy_avoid_obstacles")

    assert len(list(ablated.nodes())) == len(list(intact.nodes()))
    composites = lambda t: [n.metadata.node_id for n in t.nodes()
                            if isinstance(n, Composite)]
    assert composites(ablated) == composites(intact), (
        "the composite ids moved, so the control flow is not the same tree")


def test_ablating_a_node_that_is_not_in_the_tree_raises(tree_factory):
    """Silently ablating nothing would report a contribution of zero for a node
    that was never there, which is indistinguishable from a real zero."""
    with pytest.raises(KeyError):
        ablate(tree_factory(), "not_in_this_tree")


def test_ablation_returns_a_tree_that_still_runs(tree_factory):
    assert isinstance(ablate(tree_factory(), "toy_is_goal_known"), Tree)


def test_a_node_that_never_fires_is_reported_as_never_fired_not_as_useless(
        tree_factory):
    """**The case the old gate could not express**, and the reason this file
    exists: a node may be in the tree and never get the chance to act.

    All three dev scenarios disclose the goal, so the Fallback never reaches
    `toy_sweep_for_goal`. Its contribution is zero — but `never_fired_in` says
    WHY, and "did not help" and "never triggered" are different findings that
    the single admission number conflated.
    """
    out = contribution(tree_factory, list(DEV_SCENARIOS), seeds=(0,),
                       node_ids=["toy_sweep_for_goal"])
    entry = out["toy_sweep_for_goal"]
    assert entry["mean_delta"] == 0.0
    assert entry["never_fired_in"] == entry["n"] == len(DEV_SCENARIOS)
    assert all(v["ticks"] == 0 for v in entry["per_scenario"].values())


def test_a_load_bearing_node_shows_a_positive_contribution(tree_factory):
    """The control for the test above. Same tree, same scenarios, a node that
    does fire — the measurement has to separate them or it separates nothing."""
    out = contribution(tree_factory, list(DEV_SCENARIOS), seeds=(0,),
                       node_ids=["toy_avoid_obstacles"])
    entry = out["toy_avoid_obstacles"]
    assert entry["mean_delta"] > 0.0
    assert entry["helped_in"] == entry["n"]
    assert entry["never_fired_in"] == 0


def test_the_per_scenario_breakdown_is_kept_not_just_the_mean(tree_factory):
    """A node that changes nothing on eleven scenarios and rescues the twelfth
    is exactly what this exists to find, and a mean hides it."""
    out = contribution(tree_factory, list(DEV_SCENARIOS), seeds=(0,),
                       node_ids=["toy_avoid_obstacles"])
    per = out["toy_avoid_obstacles"]["per_scenario"]
    assert len(per) == len(DEV_SCENARIOS)
    for row in per.values():
        assert {"intact", "ablated", "delta", "ticks", "fired", "drove"} <= set(row)


def test_ablatable_skips_the_harness_own_scaffold(tree_factory):
    """Composites and stand-ins are not nodes anyone generated, so measuring
    their contribution would report on the harness rather than the library."""
    ids = ablatable(tree_factory())
    assert ids
    assert not any(i.startswith(("standin_", "composed_", "unit_")) for i in ids)
