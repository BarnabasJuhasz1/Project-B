"""
nodes/tests/test_diagram.py

The behaviour-tree pane. Sim-free: it draws with PIL into a numpy array, so it
is testable without a GPU, and `core/tests/test_import_guard.py` already refuses
anything in `nodes/` that reaches for a simulator.

The rule these tests hold: **drawing is read-only.** A recorded run and a
headless run must be the same run (`scripts/run_episode.py` says so in its
docstring and `nodes/tests/test_trace.py` checks the trace half). This is the
other half — the diagram reads the tick log every frame, and if it could touch
it, the video would be changing the evidence.
"""

import numpy as np
import pytest

from nodes.diagram import (BOX_MAX_WIDTH, STATE_COLORS, TreeDiagram,
                           banner_height_for)
from nodes.toys import search_tree, toy_tree


@pytest.fixture(params=["toy_tree", "search_tree"])
def tree(request):
    return {"toy_tree": toy_tree, "search_tree": search_tree}[request.param]()


@pytest.fixture
def diagram(tree):
    return TreeDiagram(tree, width=1200)


def test_every_box_is_inside_the_banner(diagram):
    """A half-drawn node reads as a rendering bug, and this pane is only worth
    having if it can be trusted at a glance."""
    for box in diagram.boxes:
        assert 0 <= box.x0 < box.x1 <= diagram.width
        assert 0 <= box.y0 < box.y1 <= diagram.height


def test_boxes_are_capped_rather_than_filling_their_slot(tree):
    """On a 2560 px banner a three-leaf tree would otherwise draw three 800 px
    lozenges and read as a colour chart. The space between boxes is what carries
    the structure."""
    wide = TreeDiagram(tree, width=2562)
    for box in wide.boxes:
        assert box.x1 - box.x0 <= BOX_MAX_WIDTH + 1


def test_a_parent_is_centred_over_its_children(diagram):
    for box in diagram.boxes:
        kids = [b for b in diagram.boxes if b.parent is box]
        if kids:
            assert box.cx == pytest.approx(sum(k.cx for k in kids) / len(kids))


def test_the_layout_is_deterministic(tree):
    """The banner is drawn once per frame from a layout solved once per episode.
    An unstable layout would make the tree jitter under a still robot."""
    runs = [[(b.node_id, b.x0, b.y0) for b in TreeDiagram(tree, width=900).boxes]
            for _ in range(3)]
    assert runs[0] == runs[1] == runs[2]


def test_render_returns_a_frame_of_the_declared_size(diagram):
    rgb = diagram.render(t=1.0)
    assert rgb.shape == (diagram.height, diagram.width, 3)
    assert rgb.dtype == np.uint8


def test_banner_height_agrees_with_the_diagram_it_predicts(tree):
    """run_episode.py sizes the frame from this BEFORE it can build the diagram,
    and ffmpeg is started at that size. A disagreement is a crash mid-run."""
    assert banner_height_for(tree) == TreeDiagram(tree, width=800).height


# --- the states -------------------------------------------------------------

def test_an_empty_log_claims_no_activity(diagram):
    """Before the first tick the honest answer is that nothing has happened.
    Defaulting to 'active' would show a tree working in the settling frames."""
    assert TreeDiagram.states_from(None) == {}

    from nodes.trace import TickLog
    assert TreeDiagram.states_from(TickLog()) == {}


def test_states_come_from_the_latest_step_only(tree):
    """The pane answers 'what is the tree doing NOW'. Folding earlier steps in
    would light up every node that has ever driven and answer a different
    question — that one is `TickLog.summary()`."""
    from core.scenario import load
    scenario = load("doorway_traverse")
    tree.enable_trace()
    tree.reset(scenario, scenario.rng())

    from core.tests.kinematic import rollout
    rollout(scenario, tree)

    log = tree.tick_log
    last_step = log.rows[-1][0]
    states = TreeDiagram.states_from(log)
    on_last_step = {r[3] for r in log.rows if r[0] == last_step}
    assert set(states) == on_last_step
    assert set(states.values()) <= set(STATE_COLORS)


def test_rendering_never_touches_the_log(tree):
    """THE invariant. The video is not evidence and must not be able to become
    evidence by editing what it is drawn from."""
    from core.scenario import load
    from core.tests.kinematic import rollout
    scenario = load("doorway_traverse")
    tree.enable_trace()
    rollout(scenario, tree)

    log = tree.tick_log
    before = list(log.rows)
    diagram = TreeDiagram(tree, width=1000)
    for t in (0.0, 1.5, 9.44):
        diagram.render(log, t=t)
    assert log.rows == before
    assert len(log.rows) == len(before)


def test_a_failing_condition_is_not_drawn_like_a_passing_one(tree):
    """Found by watching a hidden-goal run: `toy_is_goal_known` returned FAILURE
    on all 6000 ticks, which is exactly why the goal branch behind it stayed
    dark — and the pane drew it the same blue as a Condition that passes. The
    one fact the diagram existed to show was the one it hid."""
    from nodes.trace import TickLog

    log = TickLog()
    log.begin_step(0.0)

    class _Meta:
        node_id = "probe"

    class _Node:
        metadata = _Meta()
        KIND = type("K", (), {"value": "condition"})()

    class _Status:
        def __init__(self, value):
            self.value = value

    log.record(_Node(), precondition=True, status=_Status("failure"), drove=False)
    assert TreeDiagram.states_from(log) == {"probe": "failed"}

    log.rows.clear()
    log.record(_Node(), precondition=True, status=_Status("success"), drove=False)
    assert TreeDiagram.states_from(log) == {"probe": "active"}

    assert STATE_COLORS["failed"][0] != STATE_COLORS["active"][0]
    assert STATE_COLORS["failed"][0] != STATE_COLORS["gated"][0]


def test_the_node_that_drove_is_drawn_differently_from_one_that_did_not(tree):
    """The whole point of the pane, asserted on pixels rather than on intent."""
    diagram = TreeDiagram(tree, width=1000)
    leaf = next(b for b in diagram.boxes
                if not any(x.parent is b for x in diagram.boxes))

    def centre(states):
        rgb = diagram.render(states=states)
        return tuple(rgb[(leaf.y0 + leaf.y1) // 2, int(leaf.cx)])

    drove = centre({leaf.node_id: "drove"})
    idle = centre({})
    gated = centre({leaf.node_id: "gated"})
    assert drove != idle and drove != gated and idle != gated
    assert drove == STATE_COLORS["drove"][0]
