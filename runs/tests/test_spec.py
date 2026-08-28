"""
runs/tests/test_spec.py

The condition -> controller binding. Every check here refuses something that
would otherwise produce a complete, plausible, meaningless table.

    pytest runs/tests/test_spec.py -q
"""

import json

import pytest

import controllers
from runs.spec import CONDITION_LABELS, load_spec


def write(tmp_path, payload):
    path = tmp_path / "spec.json"
    path.write_text(json.dumps(payload))
    return str(path)


def test_the_rehearsal_spec_loads_and_is_not_reportable():
    spec = load_spec("runs/specs/rehearsal.json")
    assert not spec.reportable
    assert [c.label for c in spec.conditions] == ["A", "B"]


def test_scaffolding_is_refused_as_a_condition(tmp_path):
    """Non-negotiable #3. `scaffold_sweep` has no obstacle awareness and no
    generation step; a sweep would score it perfectly happily."""
    path = write(tmp_path, {"reportable": False, "conditions": [
        {"label": "A", "controller": "scaffold_sweep"}]})
    with pytest.raises(ValueError, match="scaffolding"):
        load_spec(path)


def test_every_scaffolding_name_is_refused(tmp_path):
    for name in controllers.scaffolding():
        path = write(tmp_path, {"reportable": False, "conditions": [
            {"label": "A", "controller": name}]})
        with pytest.raises(ValueError, match="scaffolding"):
            load_spec(path)


def test_an_unresolvable_controller_is_refused_before_the_simulator(tmp_path):
    path = write(tmp_path, {"reportable": False, "conditions": [
        {"label": "A", "controller": "no_such_controller"}]})
    with pytest.raises(ValueError, match="does not resolve"):
        load_spec(path)


def test_two_conditions_cannot_share_a_controller(tmp_path):
    """Their gap is zero by construction, and the interesting question is
    whether that was intended."""
    path = write(tmp_path, {"reportable": False, "conditions": [
        {"label": "A", "controller": "goal_seeker"},
        {"label": "B", "controller": "goal_seeker"}]})
    with pytest.raises(ValueError, match="both"):
        load_spec(path)


def test_a_fifth_condition_is_refused(tmp_path):
    path = write(tmp_path, {"reportable": False, "conditions": [
        {"label": "E", "controller": "goal_seeker"}]})
    with pytest.raises(ValueError, match="four conditions"):
        load_spec(path)


def test_a_reportable_spec_must_name_a_library_snapshot(tmp_path):
    """Non-negotiable #8."""
    path = write(tmp_path, {"reportable": True, "conditions": [
        {"label": "A", "controller": "goal_seeker"}]})
    with pytest.raises(ValueError, match="library_snapshot"):
        load_spec(path)


def test_conditions_come_back_in_write_up_order(tmp_path):
    path = write(tmp_path, {"reportable": False, "conditions": [
        {"label": "B", "controller": "reactive_avoid"},
        {"label": "A", "controller": "goal_seeker"}]})
    assert [c.label for c in load_spec(path).conditions] == ["A", "B"]
    assert list(CONDITION_LABELS) == ["A", "B", "C", "D"]
