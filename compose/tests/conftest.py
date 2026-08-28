"""
compose/tests/conftest.py

Shared fixtures. **No test in this package calls a model**, for the reasons
`generate/tests/conftest.py` sets out: every model-dependent test replays a
committed transcript from `generate/transcripts/`.

**No test in this package imports `core.scenarios_held_out`.** Chunk 11's AST
guard covers `compose/` including its tests, and it is a stronger seal for
having no exemption in it. The one test that must import the split — proving a
TaskSpec built from a held-out scenario leaks nothing — lives in
`core/tests/test_compose_leak.py` instead.
"""

import pytest

# The committed compositions, and what each is for. All eight were produced by
# real `claude -p` calls against claude-opus-5 and are replayable from
# `generate/transcripts/`.
#
# The three live ones were RE-COMPOSED on 2026-08-28 against lib-006-23c57a77269e.
# Nothing about composition changed; the library did. `limit_speed_dwa` was
# re-generated when its card was verified, so the node the retrieval offers is
# now `gen_b_d6ae93648c7b4f25` and the prompt that names it hashes differently.
# The lib-006-a05b0b267dd4 records are kept — they are evidence, and G3 needs a
# denominator — but they are archived, not current.

# The headline: composes, and the tree reaches the goal on doorway_traverse.
NAVIGATE_ID = "navigate_to_goal__goal_known_no_drift__56b3466284b7d9e8"
# A search-shaped task, composed from the axes rather than from a scenario.
SEARCH_ID = "search_for_goal__goal_hidden_no_drift__2b74bf7b40d6ed9a"
# Its repair, from the same library and the same retrieval.
REPAIR_ID = "r_search_for_goal__goal_hidden_no_drift__6004b2deaf87a659"

# --- kept as evidence, not as working compositions --------------------------
# A composition that SUCCEEDED and produced a tree that fails the scenario: the
# model put a driving Action ahead of another in a Sequence, where RUNNING
# short-circuits, and starved the second one. See the chunk 16 Outcome.
# NOT EXECUTABLE ANY MORE, and the reason is worth reading: the node it starves
# on IS the superseded gen_b_1f2fe8016b1441dc, so rebuilding the tree would need
# a library that no longer exists. The record is intact and the finding is read
# off it structurally — see test_a_composition_can_succeed_and_still_not_work.
STARVED_ID = "navigate_to_goal__goal_known_no_drift__6f9686b026c3f472"
# A recorded MULTIPLE_PLANS rejection: the model split one object across two
# fenced blocks. Its prompt no longer renders — the template was tightened in
# response — and it stays auditable because the transcript holds the prompt.
SPLIT_PLAN_ID = "navigate_to_goal__goal_known_no_drift__e516fff9d3ec23c2"


@pytest.fixture(scope="session")
def cards():
    """The card store. Used ONLY by test_no_papers.py, to check that none of
    this reaches the composition prompt. Nothing on the composition path can
    reach a card; the test reaches one so it can prove that."""
    from cards.store import CardStore
    return CardStore.load()


@pytest.fixture(scope="session")
def library():
    """The frozen library WITH the fixture leaves.

    Toys are in because the generated library is one node long and a composition
    of one node is not a composition. They are NOT navigation and never a
    benchmark condition — see nodes/toys.py.
    """
    from compose.library import load_library
    return load_library(include_toys=True)


@pytest.fixture(scope="session")
def generated_only():
    """The library as an evaluation would see it: no fixtures."""
    from compose.library import load_library
    return load_library(include_toys=False)


@pytest.fixture
def spec():
    from compose.spec import TaskSpec
    from core.scenario import load
    return TaskSpec.from_scenario(load("doorway_traverse"))


@pytest.fixture
def search_spec():
    """Goal hidden. Built from the axes rather than from a scenario, which is
    the same construction a held-out family gets."""
    from compose.spec import TaskSpec
    from core import scenario as s
    return TaskSpec(family="search_for_goal", goal_known=False,
                    pose_drifts=False, n_beams=s.N_BEAMS,
                    fov=s.FOV, range_max=s.RANGE_MAX,
                    detect_radius=s.DETECT_RADIUS, max_lin=s.MAX_LIN,
                    max_ang=s.MAX_ANG)


@pytest.fixture
def found(library, spec):
    from compose.retrieve import retrieve
    return retrieve(library, spec)


@pytest.fixture
def replay():
    from generate.backends import ReplayBackend
    return ReplayBackend()
