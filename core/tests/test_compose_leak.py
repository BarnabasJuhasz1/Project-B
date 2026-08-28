"""
core/tests/test_compose_leak.py

Does a TaskSpec built from a HELD-OUT scenario leak the held-out scenario?

WHY THIS FILE IS HERE AND NOT IN compose/tests/
------------------------------------------------
Chunk 11's AST guard forbids anything under `compose/` from importing
`core.scenarios_held_out`, and it makes no exception for tests. That is
deliberate and worth keeping: an exemption for tests is a hole shaped exactly
like a test, and the seal is stronger with no hole in it at all. So the one
check that must import the split lives here, next to the split's other tests,
where importing it is what the module is for.

The plan for chunk 16 anticipated this — "in a test that is allowed to import it
precisely because a test is not the generation path" — and this is where that
permission is exercised.

WHAT IT ASSERTS, AND WHY IT IS THE RIGHT SHAPE
-----------------------------------------------
Not "the id does not appear" alone. Every NUMBER in the rendered task
description has to have a disclosed TaskSpec field behind it. A start
coordinate, a goal coordinate and a seed are all numbers, and so is anything
else a future field might carry, so checking the whole number set catches the
class rather than three instances of it.

Failure messages here are deliberately counts and field names, never scenario
contents: a test that printed the split on failure would leak it into whatever
was reading the output.
"""

from collections import defaultdict

from compose.spec import TaskSpec
from compose.tests.test_spec import disclosed_numbers, numbers_in
from core.scenarios_held_out import HELD_OUT_SCENARIOS


def test_a_held_out_spec_discloses_no_number_it_does_not_declare():
    """The leak check. One assertion per scenario, reported by index rather than
    by id so a failure says which one without naming it."""
    for index, scenario in enumerate(HELD_OUT_SCENARIOS):
        spec = TaskSpec.from_scenario(scenario)
        text = spec.describe()
        allowed = {round(v, d) for v in disclosed_numbers(spec) for d in (0, 1, 2)}
        undisclosed = numbers_in(text) - allowed
        assert not undisclosed, (
            f"held-out scenario #{index}: {len(undisclosed)} number(s) in the "
            f"rendered task description have no TaskSpec field behind them")


def test_a_held_out_spec_names_no_scenario():
    for index, scenario in enumerate(HELD_OUT_SCENARIOS):
        text = TaskSpec.from_scenario(scenario).describe()
        assert scenario.id not in text, f"held-out scenario #{index} named"
        assert str(scenario.seed) not in text, f"held-out scenario #{index} seed"


def test_a_spec_cannot_tell_two_scenarios_in_one_family_apart():
    """The property G3 actually needs.

    Chunk 11's split is three scenarios per family. If composition could
    distinguish them, it could be tailored to an episode rather than to a task,
    and "solved by recombination alone" would be measuring something else. So
    every scenario in a family must render to the same task description.

    Reported as a count per family, never as the differing text.
    """
    by_family = defaultdict(set)
    for scenario in HELD_OUT_SCENARIOS:
        by_family[scenario.family].add(TaskSpec.from_scenario(scenario).describe())
    for family, rendered in by_family.items():
        assert len(rendered) == 1, (
            f"family {family!r} renders {len(rendered)} distinct task "
            f"descriptions, so a TaskSpec can tell its scenarios apart")


def test_the_axes_are_all_a_spec_carries_about_the_split():
    """A spec is the 2 x 2 cell plus the sensor configuration. Two families in
    the same cell would be indistinguishable, and that is fine — but a family
    must not map to more than one cell, or `axes` is not the family's axes."""
    cells = defaultdict(set)
    for scenario in HELD_OUT_SCENARIOS:
        cells[scenario.family].add(TaskSpec.from_scenario(scenario).axes)
    for family, seen in cells.items():
        assert len(seen) == 1, f"family {family!r} spans {len(seen)} cells"


def test_the_ast_guard_actually_covers_compose():
    """The guard in test_held_out.py skips a package directory that does not
    exist, so it was vacuous for `compose/` until this chunk created it. This
    asserts it is no longer vacuous — that the guard names the package AND finds
    files in it — because a guard that silently checks nothing is worse than no
    guard, and it looks identical in a green test run.
    """
    import os

    from core.tests.test_held_out import SEALED_FROM

    assert "compose" in SEALED_FROM
    root = os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), "compose")
    found = [name for _, _, names in os.walk(root)
             for name in names if name.endswith(".py")]
    assert len(found) > 1, "the guard would walk an empty package"
