"""
harness/tests/test_harness.py

The admission decision, tested with a node whose effect can be switched off.

That is the shape the Done-when asks for and it is the right one: "a toy node
with a real effect passes; the same node with its effect disabled fails" is a
much stronger statement than two different nodes giving two different answers,
because everything except the mechanism is held constant.
"""

import dataclasses

import pytest

import controllers
from core.geometry import LAB42, occupancy_grid
from core.observation import Command
from nodes.contract import Action, Direction, Status
from nodes.toys import GOAL_KNOWN_IN, IsGoalKnown, toy_metadata
from nodes.tree import Sequence, Tree

from harness.gate import Gate
from harness.micro import MicroScenario, room
from harness.unit import (MIN_EFFECT_RATE, UnitScenario, required_effect,
                          run_unit)
from nodes.library import Library



# --- a node whose mechanism can be switched off ----------------------------

class SteerPastObstacle(Action):
    """Drive at the goal, steering around whatever is in the way.

    NOT NAVIGATION and never a benchmark condition. A fixture, and deliberately
    a thin one: it delegates to `reactive_avoid` when its mechanism is on and to
    `goal_seeker` when it is off, rather than hand-rolling a gap follower here.

    Two reasons. The reference controllers are already known — one reaches the
    goal, the other walks into things — so the fixture's behaviour is not
    another thing to debug. And it makes "the same node with its effect
    disabled" literally a swap of the mechanism, with the ports, the
    precondition, the metadata and the tree all held constant.
    """

    PORTS = (GOAL_KNOWN_IN,)
    PRECONDITION = "the goal has been disclosed"
    EFFECT = "steers around an obstacle between the robot and the goal"
    TAGS = ("avoid",)

    MECHANISM = True

    def __init__(self, metadata):
        super().__init__(metadata)
        self._inner = (controllers.make("reactive_avoid") if self.MECHANISM
                       else controllers.make("goal_seeker"))

    def reset(self, scenario, rng):
        self._inner.reset(scenario, rng)

    def precondition(self, obs, ports):
        return ports["goal_known"]

    def tick(self, obs, ports):
        ports.drive(self._inner.step(obs))
        return Status.RUNNING


class SteerPastObstacleDisabled(SteerPastObstacle):
    """The same node with its claimed mechanism switched off."""

    MECHANISM = False


def _tree(node_cls, metadata):
    return Tree(Sequence(toy_metadata("unit_seq"), [
        IsGoalKnown(toy_metadata("unit_cond")),
        node_cls(metadata),
    ]))


def _micro(seed=9001):
    """A corridor with a box the robot must go around. `goal_seeker` walks into
    the box; a node that steers does not."""
    return MicroScenario(
        id="corridor", world=room(6.0, 4.0, boxes=[(2.0, -0.1, 2.2, 2.5)]),
        start=(1.0, 1.0, 0.0), goal=(5.0, 3.0), time_limit=30.0, seed=seed)


def _unit(**kw):
    base = dict(node_id="steer_past_obstacle", micro=_micro(),
                metric="goal_reached", baseline="goal_seeker")
    base.update(kw)
    return UnitScenario(**base)


# --- the core claim --------------------------------------------------------

def test_a_node_with_a_real_effect_is_admitted():
    result = run_unit(lambda: _tree(SteerPastObstacle, toy_metadata("n")), _unit())
    assert result.admitted, result.reason
    assert result.effect >= result.required


def test_the_same_node_with_its_effect_disabled_is_not():
    """Everything else held constant — same world, same seeds, same baseline,
    same ports. Only the mechanism changes."""
    result = run_unit(
        lambda: _tree(SteerPastObstacleDisabled, toy_metadata("n")), _unit())
    assert not result.admitted
    assert "below the required" in result.reason


# --- the generator cannot mark its own homework ----------------------------

def test_a_scenario_whose_baseline_already_succeeds_is_malformed():
    """The most important check here. A generator that writes both the node and
    its test can write a test that was already true; this is what catches it,
    and it rejects even a genuinely good node rather than award a free mark."""
    easy = dataclasses.replace(_micro(), id="empty", world=room(6.0, 4.0))
    result = run_unit(lambda: _tree(SteerPastObstacle, toy_metadata("n")),
                      _unit(micro=easy))
    assert not result.admitted
    assert result.reason.startswith("MALFORMED")


def test_the_effect_floor_cannot_be_lowered_by_the_scenario():
    """A unit scenario may demand MORE than the floor; never less."""
    assert required_effect(_unit(min_effect=0.0)) == MIN_EFFECT_RATE
    assert required_effect(_unit(min_effect=-5.0)) == MIN_EFFECT_RATE
    assert required_effect(_unit(min_effect=0.9)) == 0.9


def test_a_prediction_naming_no_real_metric_is_rejected():
    """'Prediction becomes the unit test' only works if the prediction names
    something measurable. A card whose Prediction is not falsifiable produces a
    node that cannot be admitted, which is the filter working."""
    result = run_unit(lambda: _tree(SteerPastObstacle, toy_metadata("n")),
                      _unit(metric="elegance"))
    assert not result.admitted
    assert "not falsifiable" in result.reason


def test_a_malformed_micro_world_is_rejected_not_scored():
    inside_a_wall = dataclasses.replace(_micro(), start=(2.1, 1.0, 0.0))
    result = run_unit(lambda: _tree(SteerPastObstacle, toy_metadata("n")),
                      _unit(micro=inside_a_wall))
    assert not result.admitted
    assert "malformed micro-world" in result.reason


# --- the micro-world itself ------------------------------------------------

def test_a_micro_world_does_not_disturb_lab42():
    """The hazard the World refactor was written to prevent: the geometry caches
    used to key on resolution alone, so two worlds at the same resolution would
    have shared a grid — and a micro-world would have silently changed what
    core.scenario.validate() considers solvable."""
    before = occupancy_grid()
    occupancy_grid(world=_micro().world)
    assert occupancy_grid() is before
    assert _micro().world is not LAB42


def test_a_micro_world_is_never_a_benchmark_scenario():
    from core.scenario import all_scenarios
    ids = {s.id for s in all_scenarios(None)} | {s.id for s in all_scenarios("dev")}
    assert _micro().id not in ids
    assert _micro().split == "unit"


def test_micro_sensing_matches_the_benchmark():
    """If a unit scenario could choose a better sensor, a node could pass by
    being handed one."""
    from core.scenario import N_BEAMS, RANGE_MAX, MAX_LIN
    m = _micro()
    assert (m.n_beams, m.range_max, m.max_lin) == (N_BEAMS, RANGE_MAX, MAX_LIN)


# --- the promotion gate ----------------------------------------------------

def test_promotion_admits_and_tags():
    gate = Gate(Library(), _tree)
    md = toy_metadata("steer_past_obstacle")
    result = gate.promote(SteerPastObstacle, md, _unit())
    assert result.promoted, result.reason
    assert result.snapshot.startswith("lib-001-")
    assert "steer_past_obstacle" in gate.library
    assert gate.run_regression() == []


def test_promotion_refuses_a_node_that_fails_its_unit_scenario():
    gate = Gate(Library(), _tree)
    result = gate.promote(SteerPastObstacleDisabled,
                          toy_metadata("disabled"), _unit(node_id="disabled"))
    assert not result.promoted
    assert "failed its unit scenario" in result.reason
    assert len(gate.library) == 0, "a refused candidate must not be in the library"


def test_promotion_refuses_a_duplicate_id():
    gate = Gate(Library(), _tree)
    md = toy_metadata("steer_past_obstacle")
    assert gate.promote(SteerPastObstacle, md, _unit()).promoted
    again = gate.promote(SteerPastObstacle, md, _unit())
    assert not again.promoted and "append-only" in again.reason


def test_a_regression_rolls_the_candidate_back_and_the_incumbent_wins():
    """#9, demonstrated rather than asserted: a candidate that breaks an
    incumbent's unit scenario is not admitted, and the library is left exactly
    as it was — snapshot tag included."""
    gate = Gate(Library(), _tree)
    assert gate.promote(SteerPastObstacle,
                        toy_metadata("incumbent"),
                        _unit(node_id="incumbent")).promoted
    before_snapshot = gate.library.snapshot()
    before_ids = [e.node_id for e in gate.library.all()]

    # Force a regression: swap the incumbent's recorded unit scenario for one
    # its node cannot pass, exactly as a genuinely interfering promotion would.
    gate.units["incumbent"] = _unit(node_id="incumbent", min_effect=99.0)

    result = gate.promote(SteerPastObstacle, toy_metadata("newcomer"),
                          _unit(node_id="newcomer"))
    assert not result.promoted
    assert result.regressions and "incumbent" in result.regressions[0]
    assert [e.node_id for e in gate.library.all()] == before_ids
    assert gate.library.snapshot() == before_snapshot


def test_extra_seeds_only_matter_when_something_is_stochastic():
    """Why the default is one seed. A deterministic micro-world driven by a
    deterministic node gives the same episode every time, so extra seeds cost
    rollouts and buy nothing — and implying an average where there is none is
    worse than not averaging."""
    one = run_unit(lambda: _tree(SteerPastObstacle, toy_metadata("n")), _unit())
    many = run_unit(lambda: _tree(SteerPastObstacle, toy_metadata("n")),
                    _unit(seeds=(0, 1, 2, 3, 4)))
    assert one.candidate == many.candidate
    assert one.baseline == many.baseline


def test_the_gate_reports_what_g1_needs():
    gate = Gate(Library(), _tree)
    gate.promote(SteerPastObstacle, toy_metadata("n1"), _unit(node_id="n1"))
    report = gate.report()
    assert report["node_count"] == 1
    assert report["units_recorded"] == 1
    assert report["traceable_fraction"] == 0.0     # handwritten fixture
