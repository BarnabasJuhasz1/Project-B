"""
core/tests/test_slam_graph.py

Chunk 23's back end: the pose graph and the Gauss-Newton over it.

The plan named three one-line mistakes with three-hour debugging sessions behind
them — angle wrapping inside the residual, an information matrix that makes
metres and radians commensurable, and a fixed anchor. Each has a test here,
under its own heading, and each was written to FAIL against the mistake rather
than merely to pass against the fix.

SIM-FREE. No simulator, no GPU.
"""

import math

import numpy as np
import pytest

from core.slam import scan as scan_mod
from core.slam.graph import (CLOSURE, CLOSURE_INFO, ODOM_INFO, PoseGraph)


# ---- below this line is data, not code.

# A square, closing on itself. Four poses, each a quarter turn on from the last.
SQUARE = [(0.0, 0.0, 0.0),
          (2.0, 0.0, math.pi / 2),
          (2.0, 2.0, math.pi),
          (0.0, 2.0, -math.pi / 2)]

PERTURB = (0.30, -0.20, 0.10)


# ---- below this line is code, not data.


def chain(truth, guess=None, closure=None, robust=True):
    """A graph over `truth` with exact odometry edges and an optional closure.

    `guess` is what the nodes are INITIALISED to; the edges are always built
    from `truth`, so the optimiser's job is to undo the perturbation. Node 0 is
    the anchor and is never perturbed — perturbing it would test nothing, since
    it cannot move.
    """
    g = PoseGraph()
    for k, pose in enumerate(truth):
        g.add_node(pose if (k == 0 or guess is None) else guess[k])
    for k in range(len(truth) - 1):
        g.add_edge(k, k + 1, scan_mod.relative(truth[k], truth[k + 1]))
    if closure is not None:
        i, j = closure
        g.add_edge(i, j, scan_mod.relative(truth[i], truth[j]),
                   info=CLOSURE_INFO, robust=robust, kind=CLOSURE)
    return g


def perturbed(truth, delta=PERTURB):
    return [(p[0] + delta[0], p[1] + delta[1], p[2] + delta[2]) for p in truth]


def max_error(graph, truth):
    """Worst position error over all nodes, in metres."""
    return max(math.hypot(p[0] - t[0], p[1] - t[1])
               for p, t in zip(graph.poses, truth))


# -- the synthetic graph with a known answer -------------------------------
# "If this fails, nothing downstream is interpretable." -- chunk 23.

def test_exact_odometry_and_one_exact_closure_recovers_the_truth():
    g = chain(SQUARE, perturbed(SQUARE), closure=(0, 3))
    assert g.chi2() > 1.0                    # the perturbation is visible
    summary = g.solve()
    assert summary["singular"] == 0
    assert summary["chi2_after"] < 1e-12
    assert max_error(g, SQUARE) < 1e-9


def test_a_graph_with_no_closure_is_already_consistent():
    """An odometry chain explains itself: the residuals are zero by
    construction, so a solve is a no-op. This is why `_optimise()` runs on an
    accepted closure and not on every finished submap."""
    g = chain(SQUARE)
    assert g.chi2() == pytest.approx(0.0, abs=1e-12)
    before = list(g.poses)
    g.solve()
    assert g.poses == before


def test_solve_is_a_noop_on_a_graph_too_small_to_optimise():
    g = PoseGraph()
    g.add_node((1.0, 2.0, 0.3))
    summary = g.solve()
    assert summary["iterations"] == 0
    assert g.poses == [(1.0, 2.0, 0.3)]


# -- mistake 1: angle wrapping ---------------------------------------------

def test_a_loop_with_a_net_rotation_of_two_pi_is_not_spun_round():
    """The named failure: unwrapped, the closure residual on θ is 2π and the
    optimiser rotates the trajectory once round to "fix" a zero error."""
    n = 8
    truth = [(math.cos(2 * math.pi * k / n) * 2.0,
              math.sin(2 * math.pi * k / n) * 2.0,
              scan_mod.wrap_angle(2 * math.pi * k / n)) for k in range(n)]
    g = chain(truth, perturbed(truth), closure=(0, n - 1))
    g.solve()
    assert max_error(g, truth) < 1e-8
    for p, t in zip(g.poses, truth):
        assert abs(scan_mod.wrap_angle(p[2] - t[2])) < 1e-8


def test_residual_theta_is_wrapped_into_the_residual():
    """Two poses a hair either side of ±π: the θ residual must be the small
    angle between them, never the ~2π one."""
    g = PoseGraph()
    g.add_node((0.0, 0.0, math.pi - 0.01))
    g.add_node((1.0, 0.0, -math.pi + 0.01))
    g.add_edge(0, 1, (1.0, 0.0, 0.0))
    e = g.residuals()[0]
    assert abs(e[2]) == pytest.approx(0.02, abs=1e-9)


def test_jacobian_matches_a_central_difference():
    """Analytic against numeric, deliberately AWAY from the wrap — a central
    difference across ±π is wrong by 2π and would fail a correct Jacobian."""
    g = chain(SQUARE, perturbed(SQUARE), closure=(0, 3))
    analytic = g.jacobian().toarray()
    numeric = np.zeros_like(analytic)
    h = 1e-6
    for col in range(3 * len(g)):
        k, a = divmod(col, 3)
        for sign, side in ((+1, 0), (-1, 1)):
            saved = g.poses[k]
            bumped = list(saved)
            bumped[a] += sign * h
            g.poses[k] = tuple(bumped)
            numeric[:, col] += sign * g.residuals().reshape(-1) / (2 * h)
            g.poses[k] = saved
    assert np.max(np.abs(analytic - numeric)) < 1e-6


# -- mistake 2: metres and radians ------------------------------------------

def test_information_makes_metres_and_radians_commensurable():
    """A 0.1 rad error and a 0.1 m error must not contribute equally: the
    information diagonals are the exchange rate, and they are not 1:1."""
    assert ODOM_INFO[2] != ODOM_INFO[0]
    assert CLOSURE_INFO[2] != CLOSURE_INFO[0]

    g = PoseGraph()
    g.add_node((0.0, 0.0, 0.0))
    g.add_node((1.0, 0.0, 0.0))
    g.add_edge(0, 1, (1.0 - 0.1, 0.0, 0.0))          # 0.1 m out
    lin = g.chi2()

    g2 = PoseGraph()
    g2.add_node((0.0, 0.0, 0.0))
    g2.add_node((1.0, 0.0, 0.0))
    g2.add_edge(0, 1, (1.0, 0.0, -0.1))              # 0.1 rad out
    ang = g2.chi2()
    assert ang == pytest.approx(lin * ODOM_INFO[2] / ODOM_INFO[0])


def test_a_weakly_informed_closure_moves_the_graph_less():
    """The score-scaled information in `localiser._closure_info` only means
    something if the solver actually respects the weight.

    Measured on CONFLICTING constraints, because that is the only place a
    weight can be observed: with an exact closure the odometry chain already
    agrees with it and both weightings recover the truth to solver precision.
    So the closure here is deliberately wrong by half a metre, and the question
    is how far it drags the trajectory off the odometry-consistent answer.
    """
    def dragged(scale):
        g = chain(SQUARE, closure=None)
        bad = scan_mod.relative(SQUARE[0], SQUARE[3])
        g.add_edge(0, 3, (bad[0] + 0.5, bad[1], bad[2]),
                   info=tuple(v * scale for v in CLOSURE_INFO),
                   robust=False, kind=CLOSURE)
        g.solve()
        return max_error(g, SQUARE)

    assert dragged(0.01) < dragged(1.0) < dragged(100.0)


# -- mistake 3: the fixed anchor -------------------------------------------

def test_the_anchor_never_moves():
    g = chain(SQUARE, perturbed(SQUARE), closure=(0, 3))
    g.solve()
    assert g.poses[0] == SQUARE[0]


def test_the_solution_does_not_translate_freely():
    """Without a fixed anchor the normal equations are singular in three
    dimensions and any rigid transform of the answer is equally optimal. With
    one, translating the whole graph raises chi2 — the frame is pinned."""
    g = chain(SQUARE, closure=(0, 3))
    assert g.chi2() == pytest.approx(0.0, abs=1e-12)
    shifted = chain(SQUARE, [(p[0] + 1.0, p[1], p[2]) for p in SQUARE],
                    closure=(0, 3))
    assert shifted.chi2() > 1.0
    shifted.solve()
    assert max_error(shifted, SQUARE) < 1e-9


def test_solving_is_not_singular_on_the_shapes_the_localiser_builds():
    """A plain chain, no closure, badly initialised: the reduced system must
    still be well posed. `SlamLocaliser` only solves on a closure, but a
    singular flag here would mean the anchor was not doing its job."""
    g = chain(SQUARE, perturbed(SQUARE))
    summary = g.solve()
    assert summary["singular"] == 0
    assert max_error(g, SQUARE) < 1e-9


# -- Huber ------------------------------------------------------------------

def test_huber_bounds_the_damage_from_a_bad_closure():
    """Inject a closure that is a metre wrong and assert the trajectory error
    stays within a stated multiple of the un-injected case."""
    clean = chain(SQUARE, perturbed(SQUARE), closure=(0, 3))
    clean.solve()
    clean_err = max_error(clean, SQUARE)

    def with_bad_closure(robust):
        g = chain(SQUARE, perturbed(SQUARE), closure=(0, 3), robust=robust)
        bad = scan_mod.relative(SQUARE[0], SQUARE[2])
        g.add_edge(0, 2, (bad[0] + 1.0, bad[1] + 1.0, bad[2]),
                   info=CLOSURE_INFO, robust=robust, kind=CLOSURE)
        g.solve()
        return max_error(g, SQUARE)

    robust_err = with_bad_closure(True)
    naive_err = with_bad_closure(False)
    # The kernel must actually bind, and it must bind in the right direction.
    assert robust_err < naive_err
    # The stated bound. Not "the closure is ignored" -- Huber down-weights, it
    # does not reject, and claiming rejection would be the wrong claim to make.
    assert robust_err < clean_err + 0.5


def test_odometry_edges_are_not_robustified():
    """A bad odometry edge means the front end failed and must surface. If the
    kernel were applied to odometry too, the same injection would be absorbed
    and the failure would be invisible."""
    g = chain(SQUARE, closure=(0, 3))
    for edge in g.edges:
        assert edge.robust == (edge.kind == CLOSURE)

    errors = g.residuals()
    errors[0] = (5.0, 5.0, 0.0)                 # a huge odometry residual
    w = g.weights(errors)
    assert w[0:3] == pytest.approx(np.array(ODOM_INFO))


def test_huber_downweights_only_beyond_delta():
    g = chain(SQUARE, closure=(0, 3))
    small = g.residuals()
    small[-1] = (1e-4, 0.0, 0.0)                # whitened norm well under delta
    assert g.weights(small)[-3:] == pytest.approx(np.array(CLOSURE_INFO))
    big = g.residuals()
    big[-1] = (2.0, 0.0, 0.0)
    assert np.all(g.weights(big)[-3:] < np.array(CLOSURE_INFO))


# -- construction and determinism ------------------------------------------

def test_an_edge_to_a_node_that_does_not_exist_is_refused():
    g = PoseGraph()
    g.add_node((0.0, 0.0, 0.0))
    with pytest.raises(IndexError):
        g.add_edge(0, 1, (1.0, 0.0, 0.0))


def test_a_self_loop_is_refused():
    g = PoseGraph()
    g.add_node((0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="self-loop"):
        g.add_edge(0, 0, (0.0, 0.0, 0.0))


def test_the_iteration_count_is_fixed_not_a_tolerance():
    """The whole determinism argument: the same graph solved twice runs the
    same number of iterations, whatever the residual does."""
    a = chain(SQUARE, perturbed(SQUARE), closure=(0, 3)).solve()
    b = chain(SQUARE, perturbed(SQUARE), closure=(0, 3)).solve()
    assert a["iterations"] == b["iterations"]
    assert a == b


def test_two_identical_graphs_optimise_bitwise_identically():
    a = chain(SQUARE, perturbed(SQUARE), closure=(0, 3))
    b = chain(SQUARE, perturbed(SQUARE), closure=(0, 3))
    a.solve()
    b.solve()
    assert a.poses == b.poses


def test_edge_order_is_the_insertion_order():
    """`solve()` sums over edges in list order, and floating-point addition is
    not associative, so a set or a dict here would make the answer depend on
    hash ordering."""
    g = chain(SQUARE, closure=(0, 3))
    assert [(e.i, e.j) for e in g.edges] == [(0, 1), (1, 2), (2, 3), (0, 3)]
