"""
compose/tests/test_spec.py

What composition is allowed to know about the task.
"""

import dataclasses
import re

import pytest

from compose.spec import SCENARIO_FIELDS_WITHHELD, TaskSpec
from core.scenario import DEV_SCENARIOS, Scenario, load


def test_the_withheld_fields_are_not_taskspec_fields():
    """The identifying half of a Scenario has no home on a TaskSpec.

    Asserted against the dataclass rather than against a rendered string,
    because a field that exists can be rendered by accident later.
    """
    names = {f.name for f in dataclasses.fields(TaskSpec)}
    assert not names & SCENARIO_FIELDS_WITHHELD


def test_every_withheld_field_is_really_a_scenario_field():
    """A deny-list naming something that does not exist is a deny-list that has
    stopped tracking what it denies."""
    names = {f.name for f in dataclasses.fields(Scenario)}
    assert SCENARIO_FIELDS_WITHHELD <= names


def numbers_in(text):
    """Every number the rendered task description contains."""
    return {float(n) for n in re.findall(r"-?\d+(?:\.\d+)?", text)}


def disclosed_numbers(spec):
    """Every number a TaskSpec is allowed to put in front of the model.

    Computed from the spec's own fields rather than listed, so a field added to
    TaskSpec widens this automatically and a number that appears WITHOUT a field
    behind it is what fails.
    """
    return {float(v) for v in spec.as_dict().values()
            if isinstance(v, (int, float)) and not isinstance(v, bool)}


@pytest.mark.parametrize("scenario", DEV_SCENARIOS, ids=lambda s: s.id)
def test_a_rendered_spec_contains_no_number_it_did_not_disclose(scenario):
    """The dev-set version of the leak check. The held-out version lives in
    core/tests/test_compose_leak.py, which is allowed to import the split.

    Checking the whole number SET rather than looking for the coordinates is
    deliberate. `blind_corner`'s goal is (8.0, 6.0) and `range_max` is 6.0, so a
    substring search for a coordinate finds a legitimate sensor parameter and
    fails on it; and more importantly, a coordinate is only one of the things
    that could leak. Every number in the text has to have a disclosed field
    behind it — that is the property, and it catches the class.
    """
    spec = TaskSpec.from_scenario(scenario)
    text = spec.describe()
    assert scenario.id not in text
    # Rounding: describe() prints to 0, 1 or 2 decimals, so a disclosed value
    # may appear in several forms. Compare against every rounding of every
    # disclosed field rather than the raw values alone.
    allowed = {round(v, d) for v in disclosed_numbers(spec) for d in (0, 1, 2)}
    assert numbers_in(text) <= allowed


def test_describe_is_a_function_of_the_axes_only():
    """Two scenarios in the same 2 x 2 cell with the same sensor configuration
    must render identically. If they do not, `describe()` is carrying something
    the axes do not — which is the shape every leak in this pipeline has."""
    a, b = load("doorway_traverse"), load("blind_corner")
    assert a.goal_known == b.goal_known and (a.drift is None) == (b.drift is None)
    # Their time limits differ (120 s and 45 s) and they still render
    # identically, which is the point: `time_limit` is not a TaskSpec field.
    assert a.time_limit != b.time_limit
    assert (TaskSpec.from_scenario(a).describe()
            == TaskSpec.from_scenario(b).describe())


def test_the_axes_change_what_the_model_reads():
    """The counterpart. A spec whose axes differ must describe differently, or
    goal_known and drift are being carried without being disclosed."""
    base = TaskSpec.from_scenario(load("doorway_traverse"))
    hidden = dataclasses.replace(base, goal_known=False)
    drifting = dataclasses.replace(base, pose_drifts=True)
    assert base.describe() != hidden.describe() != drifting.describe()
    assert "NOT disclosed" in hidden.describe()
    assert "DRIFTS" in drifting.describe()
    assert "exact" in base.describe()


def test_the_query_grows_with_the_axes():
    base = TaskSpec.from_scenario(load("doorway_traverse"))
    assert "search" not in base.tags()
    assert "search" in dataclasses.replace(base, goal_known=False).tags()
    assert "drift" in dataclasses.replace(base, pose_drifts=True).tags()


def test_the_query_names_no_mechanism():
    """A task spec that said 'homing' or 'frontier' would have told composition
    which mechanism to reach for, which is the corpus leaking in through the
    retrieval query rather than through the prompt."""
    banned = {"homing", "frontier", "path-integration", "integration",
              "reorient", "geometric", "area-restricted", "dwa"}
    for goal_known in (True, False):
        for drifts in (True, False):
            spec = dataclasses.replace(
                TaskSpec.from_scenario(load("doorway_traverse")),
                goal_known=goal_known, pose_drifts=drifts)
            assert not set(spec.tags()) & banned


def test_a_repair_carries_the_failure_and_nothing_else():
    base = TaskSpec.from_scenario(load("doorway_traverse"))
    repaired = dataclasses.replace(
        base, failure_signature="stalled against the divider")
    assert "THIS IS A REPAIR" in repaired.describe()
    assert "stalled against the divider" in repaired.describe()
    assert base.as_dict()["failure_signature"] is None


def test_a_spec_is_frozen():
    """A spec edited after retrieval would make the retrieval log a record of
    something the model never saw."""
    spec = TaskSpec.from_scenario(load("doorway_traverse"))
    with pytest.raises(dataclasses.FrozenInstanceError):
        spec.family = "something_else"
