"""
core/tests/test_geometry.py

Every expected number here is derived by hand from core.scene_spec.WALLS and
written into the test with the derivation next to it. That is the point: these
are the values the SPL denominator and the collision metric are built on, and a
regression in them would move every reported score quietly.

Wall half-thickness is 0.075 m throughout (WALL_THICKNESS = 0.15).
"""

import math

import numpy as np
import pytest

from core.geometry import (
    ARC_STEP_DEG, GRID_RESOLUTION, ROBOT_RADIUS,
    clearance, geodesic_distance, geodesic_field, occupancy_grid, raycast,
    segment_clearance, shortest_path, shortest_path_length,
)
from core.metrics import GOAL_RADIUS as GOAL_R
from core.scenario import all_scenarios
from core.scene_spec import GO2_START, GOAL

SOUTH, WEST, NORTH, EAST = -math.pi / 2, math.pi, math.pi / 2, 0.0


# --- raycast ---------------------------------------------------------------

def test_ray_south_hits_south_perimeter():
    # (3, 4) is open floor in the west room. South perimeter centre y=0, inner
    # face y=0.075, so 4.0 - 0.075 = 3.925.
    assert raycast((3.0, 4.0), 0.0, [SOUTH], 20.0)[0] == pytest.approx(3.925)


def test_ray_east_hits_divider_face():
    # (3, 1.5) faces the divider at x=6; inner face x=5.925, so 5.925 - 3.0.
    assert raycast((3.0, 1.5), 0.0, [EAST], 20.0)[0] == pytest.approx(2.925)


def test_ray_beyond_max_range_is_inf():
    # North perimeter is 6.425 m away from (3, 1.5); a 1 m beam sees nothing.
    assert math.isinf(raycast((3.0, 1.5), 0.0, [NORTH], 1.0)[0])


def test_yaw_rotates_the_beam():
    """`angles` are in the robot frame, so yaw must turn them."""
    facing_east = raycast((3.0, 1.5), 0.0, [0.0], 20.0)[0]
    turned_to_east = raycast((3.0, 1.5), EAST - WEST, [WEST], 20.0)[0]
    assert turned_to_east == pytest.approx(facing_east)


def test_enclosed_room_returns_on_every_bearing():
    """lab42 is a closed box, so a full sweep from open floor never sees sky."""
    sweep = raycast((3.0, 4.0), 0.0, np.linspace(-math.pi, math.pi, 73), 50.0)
    assert np.isfinite(sweep).all()


def test_ray_from_inside_a_wall_is_zero():
    """Documented behaviour: a sensor buried in geometry reports contact, not a
    negative range. (6, 4) is inside the divider, which spans y in [0, 5]."""
    assert raycast((6.0, 4.0), 0.0, [SOUTH], 20.0)[0] == 0.0


# --- clearance -------------------------------------------------------------

def test_clearance_inside_a_wall_is_zero():
    assert clearance((6.0, 2.0)) == 0.0


def test_clearance_to_nearest_face():
    # (3, 4): west perimeter face at x=0.075 and divider face at x=5.925 are
    # both 2.925 away, and both nearer than the south face at 3.925.
    assert clearance((3.0, 4.0)) == pytest.approx(2.925)


def test_clearance_is_vectorised():
    pts = np.array([[3.0, 4.0], [6.0, 2.0]])
    assert clearance(pts) == pytest.approx([2.925, 0.0])


# --- occupancy grid --------------------------------------------------------

def test_grid_shape_matches_the_room():
    grid = occupancy_grid()
    assert grid.shape == (160, 240)          # 8 m / 0.05, 12 m / 0.05
    assert grid.resolution == GRID_RESOLUTION


def test_start_and_goal_are_free():
    grid = occupancy_grid()
    assert grid.is_free(GO2_START[:2])
    assert grid.is_free(GOAL)


def test_inflation_widens_the_walls():
    """A point 0.2 m clear of the divider is free space but not free for a
    0.30 m robot centre."""
    near_wall = (5.925 - 0.2, 2.0)
    assert clearance(near_wall) == pytest.approx(0.2)
    assert not occupancy_grid().is_free(near_wall)


# --- geodesic --------------------------------------------------------------

def test_geodesic_exceeds_euclidean_and_is_plausible():
    d = geodesic_distance(GO2_START[:2], GOAL)
    euclid = math.hypot(GOAL[0] - GO2_START[0], GOAL[1] - GO2_START[1])
    assert euclid == pytest.approx(10.2956, abs=1e-3)
    # The divider blocks the straight line; the legal route goes through the
    # doorway at y in [5, 8] and past the blind corner.
    assert d > euclid
    assert d < 15.0


def test_geodesic_is_symmetric_enough():
    """Same corridor either way. Not bitwise equal — the goal cell seeds the
    field, so the two runs quantise differently — but within a cell or two."""
    there = geodesic_distance(GO2_START[:2], GOAL)
    back = geodesic_distance(GOAL, GO2_START[:2])
    assert there == pytest.approx(back, abs=4 * GRID_RESOLUTION)


def test_sealed_room_is_unreachable():
    """The divider (x=6, y in [0, 5]) and the blind-corner wall (y=5,
    x in [6, 12]) close off x in [6.075, 11.925], y in [0.075, 4.925] against
    the south and east perimeters. There is no door. If this ever starts
    passing a finite distance, the floorplan changed."""
    assert math.isinf(geodesic_distance((9.0, 2.5), GOAL))


def test_goal_inside_geometry_yields_an_empty_field():
    """core.scenario turns this into a load failure rather than a bogus SPL."""
    assert not np.isfinite(geodesic_field((6.0, 2.0))).any()


def test_diagonals_do_not_leak_through_walls():
    """Walls are 3 cells thick at 0.05 m, so an 8-connected search that allowed
    corner-cutting could still not tunnel — but the doorway-free sealed room is
    the real check, and this pins the diagonal cost itself."""
    # Straight-line distance across open floor in the west room, no obstacles
    # between: (1.0, 1.0) to (4.0, 4.0) is a pure diagonal, 3*sqrt(2) = 4.243.
    d = geodesic_distance((1.0, 1.0), (4.0, 4.0))
    assert d == pytest.approx(3.0 * math.sqrt(2.0), abs=0.1)


def test_robot_radius_is_what_the_grid_used():
    assert ROBOT_RADIUS == 0.30


# --- the taut path: the SPL denominator ------------------------------------
#
# geodesic_distance() above can only step at multiples of 45 degrees, so it
# overstates any other heading. These pin the continuous replacement.

# doorway_traverse, start to goal POINT, no stop_radius. Cross-checked against a
# constrained polyline optimisation (scipy SLSQP, 80 waypoints, 12 clearance
# samples per segment), which lands at 10.594350 while still violating clearance
# by 4.5e-4 m — two independent methods agreeing to 1.2e-4 m. Not a value to
# adjust if it drifts: if this moves, the floorplan or the geometry moved.
TAUT_DOORWAY = 10.5942


def test_taut_path_matches_the_independently_optimised_value():
    assert shortest_path_length(GO2_START[:2], GOAL) == pytest.approx(
        TAUT_DOORWAY, abs=1e-3)


def test_taut_path_is_never_longer_than_the_grid_path():
    """The grid is a restricted-heading version of the same problem, so it can
    never beat the continuous answer. Also the regression test for the bias
    being removed at all: strict inequality means the grid really was over."""
    for scenario in all_scenarios():
        taut = shortest_path_length(scenario.start[:2], scenario.goal)
        grid = geodesic_distance(scenario.start[:2], scenario.goal)
        assert taut <= grid + 1e-9
        assert taut < grid, f"{scenario.id}: expected the grid to overstate"


def test_taut_path_beats_the_grid_by_the_measured_margin():
    """The number that motivated replacing the denominator at all."""
    taut = shortest_path_length(GO2_START[:2], GOAL)
    grid = geodesic_distance(GO2_START[:2], GOAL)
    assert grid - taut == pytest.approx(0.4769, abs=1e-3)
    assert (grid - taut) / taut == pytest.approx(0.045, abs=2e-3)


@pytest.mark.parametrize("a, b", [
    ((1.0, 1.0), (4.0, 3.0)),          # west room, off-diagonal heading
    ((2.0, 6.0), (5.0, 7.0)),          # north strip
    ((8.0, 6.0), (11.0, 7.0)),         # east of the doorway
])
def test_open_floor_is_exactly_euclidean(a, b):
    """THE regression test for the whole change. With nothing in the way the
    shortest path is the straight line, to the last bit. The 8-connected grid
    fails this for every heading that is not a multiple of 45 degrees — it
    returns 3.828 for the first case against a true 3.606, and no amount of
    grid refinement fixes it."""
    assert shortest_path_length(a, b) == pytest.approx(math.hypot(b[0] - a[0],
                                                                 b[1] - a[1]),
                                                       abs=1e-9)


def test_the_returned_path_is_actually_walkable():
    """A length nobody can check is worth nothing. Every segment of the polyline
    must clear ROBOT_RADIUS along its whole length, not merely at its ends."""
    for scenario in all_scenarios():
        path = np.array(shortest_path(scenario.start[:2], scenario.goal))
        assert len(path) >= 2
        gaps = segment_clearance(path[:-1], path[1:])
        assert gaps.min() >= ROBOT_RADIUS - 1e-9
        assert path[0] == pytest.approx(np.array(scenario.start[:2]))
        assert path[-1] == pytest.approx(np.array(scenario.goal))


def test_taut_path_is_exactly_symmetric():
    """Unlike the grid, which seeds its field at the goal cell and so quantises
    the two directions differently. `return_home` is `doorway_traverse` reversed
    and the two must now score against an identical denominator."""
    there = shortest_path_length(GO2_START[:2], GOAL)
    back = shortest_path_length(GOAL, GO2_START[:2])
    assert there == back


def test_arc_sampling_has_converged_at_the_default_step():
    """ARC_STEP_DEG trades cost against an overstatement that is bounded by
    R*(1/cos(step/2) - 1) per rounded corner. Refining it 8x may not move the
    answer by as much as a tenth of a millimetre, or the default is too coarse
    to call the result continuous."""
    coarse = shortest_path_length(GO2_START[:2], GOAL)
    fine = shortest_path_length(GO2_START[:2], GOAL, step_deg=ARC_STEP_DEG / 8.0)
    assert coarse >= fine                      # sampling only ever overstates
    assert coarse - fine < 1e-4


def test_sealed_room_has_no_taut_path_either():
    """The reachability check core.scenario runs is the grid one, but the two
    must not disagree about the sealed room."""
    assert math.isinf(shortest_path_length((9.0, 2.5), GOAL))


def test_endpoint_inside_inflated_geometry_is_unreachable():
    """A goal the robot's body could never occupy has no denominator, and
    core.scenario turns that into a load failure."""
    assert math.isinf(shortest_path_length(GO2_START[:2], (6.0, 2.0)))


def test_stop_radius_shortens_by_exactly_that_much():
    """The SPL denominator measures the journey the episode actually ends, and
    the episode ends at GOAL_RADIUS. Subtracting a constant cannot change which
    path is shortest, which is why this is exact rather than a re-search."""
    full = shortest_path_length(GO2_START[:2], GOAL)
    for r in (0.0, 0.5, 1.25):
        assert shortest_path_length(GO2_START[:2], GOAL,
                                    stop_radius=r) == pytest.approx(full - r)


def test_stop_radius_agrees_with_measuring_to_the_goal_disc():
    """Subtracting the radius assumes the shortest route to the goal DISC leaves
    the same taut path, just stopping early. That holds while the disc is clear
    of geometry — which core.scenario.validate enforces — and this is the check
    that it does hold here, by measuring to sampled points on the disc directly."""
    ring = [(GOAL[0] + GOAL_R * math.cos(t), GOAL[1] + GOAL_R * math.sin(t))
            for t in np.linspace(0.0, 2.0 * math.pi, 181)]
    direct = min(shortest_path_length(GO2_START[:2], q) for q in ring)
    assert shortest_path_length(GO2_START[:2], GOAL,
                                stop_radius=GOAL_R) == pytest.approx(direct, abs=1e-6)


# --- segment_clearance, which every edge of the graph is built on ----------

def test_segment_clearance_is_zero_through_a_wall():
    """Straight across the divider at y = 2."""
    assert segment_clearance(np.array([5.0, 2.0]), np.array([7.0, 2.0])) == 0.0


def test_segment_clearance_catches_a_wall_the_endpoints_miss():
    """Both ends in open floor, the middle through the divider. Sampling only
    the endpoints would call this free, which is why the test is analytic."""
    assert segment_clearance(np.array([3.0, 2.0]), np.array([9.0, 2.0])) == 0.0


def test_segment_clearance_matches_dense_sampling():
    """The analytic vertex-pair argument against brute force, on segments that
    run near walls without touching them."""
    rng = np.random.default_rng(0)
    a = rng.uniform([0.5, 0.5], [11.5, 7.5], size=(300, 2))
    b = rng.uniform([0.5, 0.5], [11.5, 7.5], size=(300, 2))
    analytic = segment_clearance(a, b)
    t = np.linspace(0.0, 1.0, 4001)[None, :, None]
    sampled = clearance(a[:, None, :] + (b - a)[:, None, :] * t).min(axis=1)
    assert np.all(analytic <= sampled + 1e-9)          # never optimistic
    assert np.allclose(analytic, sampled, atol=2e-3)   # and not pessimistic
