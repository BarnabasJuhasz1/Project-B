"""
core/tests/test_held_out.py

The seal, such as a test can enforce it.

Three properties, in descending order of how much they buy:

**Nothing on the generation path imports the split.** Checked by AST over
controllers/, nodes/, generate/ and compose/, the same way test_import_guard
checks for simulator imports. This is the half of sealing that is mechanical.

**The split is the shape it claims to be.** Twelve scenarios, a genuinely
crossed 2 x 2, every search start actually undisclosed. `validate()` cannot
catch the last one — whether a goal is already visible from the start is a
property of the PAIR — and neither can a family label, which is a string
somebody typed.

**The split has not changed.** Its SHA-256 is recorded below. Editing the split
is then a failing test rather than a silent re-definition of what G3 measured.

What none of this buys is protection against the split having been in an LLM's
context during generation. That is procedural, it is written into
core/scenarios_held_out.py and results/README.md, and no test can see it.
"""

import ast
import dataclasses
import os
import sys
from collections import Counter

import pytest

from core.manifest import sha256_file
from core.scenario import all_scenarios, load, validate
from core.scenarios_held_out import HELD_OUT_FAMILIES, HELD_OUT_SCENARIOS

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SPLIT_PATH = os.path.join(REPO_ROOT, "core", "scenarios_held_out.py")

# The sealed split, as of `git tag split-v1`. If this test fails, the split was
# edited: either that was deliberate and G3's denominator has changed and must
# be re-declared, or it was an accident. It is never a number to just update.
SPLIT_SHA256 = "00b3312b0755cc94d252ccb2eb6a096cf0867aae19ef61c6c71cb31572870a63"

# Directories that must never reach the split. compose/ sees port contracts and
# a task specification, never a scenario (non-negotiable #6); generate/ sees one
# card and the node contract.
SEALED_FROM = ("controllers", "nodes", "harness", "cards", "generate", "compose")

EXPECTED = 12
PER_FAMILY = 3


# --- the shape -------------------------------------------------------------

def test_the_split_is_twelve_scenarios_three_per_family():
    """Asserted by count because an off-by-one here silently changes G3's
    denominator, and a fraction with the wrong denominator still looks fine."""
    assert len(HELD_OUT_SCENARIOS) == EXPECTED
    counts = Counter(s.family for s in HELD_OUT_SCENARIOS)
    assert set(counts) == set(HELD_OUT_FAMILIES)
    assert all(counts[f] == PER_FAMILY for f in HELD_OUT_FAMILIES), counts


def test_the_two_axes_are_actually_crossed():
    """The family label is a string somebody typed; goal_known and drift are the
    things that make a scenario what it says it is. This test exists because the
    first draft of the split had all six search scenarios silently disclosing
    their goal — goal_known defaults to True — so the 2 x 2 was a 2 x 1 and every
    count-based check still passed."""
    expected = {
        ("navigate_to_goal", True, False),
        ("search_unmapped", False, False),
        ("return_degraded", True, True),
        ("search_degraded", False, True),
    }
    actual = {(s.family, s.goal_known, s.drift is not None)
              for s in HELD_OUT_SCENARIOS}
    assert actual == expected

    cells = Counter((s.goal_known, s.drift is not None) for s in HELD_OUT_SCENARIOS)
    assert set(cells.values()) == {PER_FAMILY}, cells


def test_ids_and_seeds_are_unique_and_do_not_collide_with_dev():
    ids = [s.id for s in HELD_OUT_SCENARIOS]
    assert len(set(ids)) == len(ids)

    seeds = [s.seed for s in HELD_OUT_SCENARIOS]
    assert seeds == [2000 + i for i in range(1, EXPECTED + 1)], (
        "held-out seeds are 2000 + position, stated as a rule so that a seed "
        "added or moved after someone saw the results is visible by inspection")
    assert not set(seeds) & {s.seed for s in all_scenarios()}


def test_no_held_out_route_repeats_a_dev_route():
    dev = {(s.start[:2], s.goal) for s in all_scenarios()}
    for s in HELD_OUT_SCENARIOS:
        assert (s.start[:2], s.goal) not in dev, s.id


def test_every_scenario_marks_itself_held_out():
    assert all(s.split == "held_out" for s in HELD_OUT_SCENARIOS)


# --- runnability -----------------------------------------------------------

@pytest.mark.parametrize("scenario", HELD_OUT_SCENARIOS, ids=lambda s: s.id)
def test_geometry_validates_like_a_dev_scenario(scenario):
    """In bounds, start and goal free, goal reachable, positive time limit, and
    the goal-disc clearance guard from chunk 09.

    Drift is stripped first, because validate() also refuses a DriftSpec until
    chunk 13 lands — that gate is asserted separately below. Stripping it here
    keeps this test about the geometry, which is what has to be right *now*.
    """
    validate(dataclasses.replace(scenario, drift=None))


@pytest.mark.parametrize(
    "scenario", [s for s in HELD_OUT_SCENARIOS if not s.goal_known],
    ids=lambda s: s.id)
def test_a_search_goal_is_not_already_visible_from_the_start(scenario):
    """A search task the robot solves on its first observation is not a search
    task. validate() cannot catch this: it is a property of the start/goal pair,
    not of either one alone."""
    assert not scenario.disclosed_at(scenario.start[:2])


@pytest.mark.parametrize(
    "scenario", [s for s in HELD_OUT_SCENARIOS if s.drift is not None],
    ids=lambda s: s.id)
def test_degraded_scenarios_load_and_carry_a_real_drift_model(scenario):
    """Until chunk 13 these refused to load, so a degraded family could not
    quietly run with drift disabled. That gate has served its purpose and the
    assertion is now its inverse: they load, and what they load is a real
    DriftModel rather than the null object.

    Whether each one actually crosses LOC_FAIL_THRESHOLD inside its time limit
    is asserted in core/tests/test_drift.py, where the model lives.
    """
    from core.drift import DriftModel, make_drift
    loaded = load(scenario.id)
    assert loaded.drift is not None
    assert isinstance(make_drift(loaded), DriftModel)


# --- the seal --------------------------------------------------------------

def test_all_scenarios_never_returns_a_held_out_scenario():
    """A batch runner that iterates "everything" must not quietly evaluate
    against the split it is supposed to be held out from."""
    held = {s.id for s in HELD_OUT_SCENARIOS}
    for split in (None, "dev"):
        assert not {s.id for s in all_scenarios(split)} & held
    assert {s.id for s in all_scenarios("held_out")} == held


def test_a_held_out_scenario_still_loads_by_name():
    """Week 5 has to be able to run these. Naming one is the explicit act the
    seal permits; getting one from all_scenarios() is the accident it forbids."""
    picked = next(s for s in HELD_OUT_SCENARIOS if s.drift is None)
    assert load(picked.id).id == picked.id


def _is_split_import(node):
    if isinstance(node, ast.Import):
        return any(a.name.startswith("core.scenarios_held_out")
                   for a in node.names)
    if isinstance(node, ast.ImportFrom):
        return (node.module or "").startswith("core.scenarios_held_out")
    return False


def _imports_the_split(path, module_level_only=False):
    """Does this file import the sealed split, and if so, where?

    `module_level_only` is the difference between "reaches the split" and
    "drags the split in behind it". core/scenario.py legitimately does the
    first and must never do the second.
    """
    tree = ast.parse(open(path, encoding="utf-8").read(), filename=path)
    nodes = tree.body if module_level_only else ast.walk(tree)
    return any(_is_split_import(node) for node in nodes)


def test_nothing_on_the_generation_path_imports_the_split():
    """The mechanical half of sealing. core/scenario.py imports it lazily, from
    inside a function, so that importing core.scenario does not transitively
    pull the split into everything — which would make this check vacuous."""
    offenders = []
    for package in SEALED_FROM:
        root = os.path.join(REPO_ROOT, package)
        if not os.path.isdir(root):
            continue                       # nodes/, generate/, compose/ come later
        for dirpath, _, filenames in os.walk(root):
            for name in filenames:
                if name.endswith(".py"):
                    path = os.path.join(dirpath, name)
                    if _imports_the_split(path):
                        offenders.append(os.path.relpath(path, REPO_ROOT))
    assert not offenders, (
        "these import the sealed held-out split: " + ", ".join(offenders) +
        ". G3 measures scenarios never seen during generation; a generation "
        "path that can read them measures nothing.")


def test_core_scenario_reaches_the_split_only_lazily():
    """core.scenario must be able to load a held-out scenario by name, so it
    does import the split — but from inside a function.

    If it imported at module level, every importer of core.scenario would
    transitively import the split, and the guard above would be checking a
    condition that nothing could violate and nothing could satisfy. The laziness
    is what keeps the seal a real constraint rather than a decoration."""
    path = os.path.join(REPO_ROOT, "core", "scenario.py")
    assert _imports_the_split(path), "load() can no longer reach held-out ids"
    assert not _imports_the_split(path, module_level_only=True)


def test_the_runner_can_reach_the_split_without_advertising_it():
    """Two properties that pull in opposite directions, so both are asserted.

    Week 5 must be able to run a held-out scenario by name — an earlier draft of
    this chunk built `--scenario` from `choices=[s.id for s in all_scenarios()]`
    and, the moment all_scenarios() stopped returning held-out scenarios, made
    the entire split unrunnable.

    And `--help` must not print the ids. `scripts/seal_split.py` goes out of its
    way not to print them either: an id like `held_search_deep_right` describes
    the task, and pasting the split anywhere a model can see it is the one
    failure mode sealing cannot survive.
    """
    import subprocess
    out = subprocess.run(
        [sys.executable, os.path.join("scripts", "run_episode.py"), "--help"],
        cwd=REPO_ROOT, capture_output=True, text=True, timeout=180)
    assert out.returncode == 0, out.stderr[-2000:]
    leaked = [s.id for s in HELD_OUT_SCENARIOS if s.id in out.stdout]
    assert not leaked, f"--help printed held-out ids: {leaked}"
    assert "--scenario" in out.stdout


def test_the_split_has_not_changed():
    """A changed split is a failing test, not a silent re-definition of what G3
    measured. If this fails, decide whether the change was deliberate before
    touching the constant."""
    assert sha256_file(SPLIT_PATH) == SPLIT_SHA256


def test_every_run_records_the_split_hash():
    """So a result can prove which split it scored against — including dev
    results, which is what evidences that the split predates development."""
    from core.manifest import capture
    assert capture(scene="scenes/lab42_v1.usd")["held_out_sha256"] == SPLIT_SHA256
