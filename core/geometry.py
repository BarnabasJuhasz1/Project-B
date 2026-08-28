"""
core/geometry.py

Everything the rest of the project needs to know about where the walls are,
computed analytically from core.scene_spec and nothing else.

Three consumers, none of which should need a simulator:

  * the range sensor behind Observation.ranges          -> raycast()
  * the collision and near-miss metrics                 -> clearance()
  * the SPL denominator, i.e. the shortest legal path   -> geodesic_distance()

lab42's obstacles are six axis-aligned boxes whose geometry is known exactly,
so all three are closed-form or a small graph search. That buys determinism
(no GPU sensor pipeline, no physics contact query, no frame-rate dependence)
and it buys speed: the whole module imports and answers in milliseconds.

raycast() is deliberately shaped like a 2D lidar so that swapping in an RTX
lidar later changes nothing above it but the two fields it fills.

    python -c "from core.geometry import geodesic_distance as g; print(g((1.5,1.5),(10.5,6.5)))"
"""

import heapq
import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np

from core.scene_spec import bounds, wall_rects


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# Occupancy grid cell size. 0.05 m over the lab42 bounds is 240 x 160 = 38400
# cells: fine enough that the 0.15 m walls are three cells thick and cannot be
# tunnelled through, small enough that Dijkstra over the whole room is instant.
GRID_RESOLUTION = 0.05

# Inflation radius for the occupancy grid. The Go2 is roughly 0.65 x 0.31 m in
# plan; 0.30 m is a circumscribing radius, so an inflated-free path is passable
# at any heading. Conservative on purpose — the geodesic is the *denominator*
# of SPL, and understating it would inflate every controller's score.
ROBOT_RADIUS = 0.30

# Angular step for sampling the rounded corners of the inflated obstacles, used
# by shortest_path_length(). A taut path bends only where the inflated boundary
# curves, which on axis-aligned boxes is the quarter-arc rounding off each wall
# corner; sampling those arcs turns a continuous problem into a finite graph.
#
# Samples sit at radius ROBOT_RADIUS / cos(step/2), not ROBOT_RADIUS, so that
# the CHORD between two adjacent samples is tangent to the true inflated
# boundary rather than cutting inside it. Every graph edge is therefore a path
# the robot could actually walk, and the length is overstated by at most
# R * (1/cos(step/2) - 1) per rounded corner — 4.6e-5 m at 2 degrees. That is
# five orders of magnitude below the 0.51 m octile bias it replaces, and it errs
# in the conservative direction for an SPL denominator.
ARC_STEP_DEG = 2.0


# ---------------------------------------------------------------------------

class Grid:
    """Occupancy over the room, with the origin and resolution that place it.

    `occupied[iy, ix]` is True where the robot centre may not go. Row-major with
    y first, so the array prints the same way up as the floorplan reads.
    """

    __slots__ = ("occupied", "origin", "resolution")

    def __init__(self, occupied, origin, resolution):
        self.occupied = occupied
        self.origin = origin              # (min_x, min_y) of cell (0, 0)
        self.resolution = resolution

    @property
    def shape(self):
        return self.occupied.shape        # (ny, nx)

    def world_to_cell(self, xy):
        """(x, y) -> (ix, iy), clamped to the grid."""
        ix = int((xy[0] - self.origin[0]) / self.resolution)
        iy = int((xy[1] - self.origin[1]) / self.resolution)
        ny, nx = self.occupied.shape
        return (min(max(ix, 0), nx - 1), min(max(iy, 0), ny - 1))

    def cell_to_world(self, ix, iy):
        """(ix, iy) -> the (x, y) centre of that cell."""
        return (self.origin[0] + (ix + 0.5) * self.resolution,
                self.origin[1] + (iy + 0.5) * self.resolution)

    def is_free(self, xy):
        ix, iy = self.world_to_cell(xy)
        return not bool(self.occupied[iy, ix])


@dataclass(frozen=True)
class World:
    """The geometry a query is answered against: axis-aligned boxes and bounds.

    Every function in this module used to read `core.scene_spec` directly, which
    meant the whole module could only ever answer questions about lab42. The
    unit-scenario harness needs purpose-built micro-worlds — a handful of boxes
    that isolate one node's claimed effect — and they must be measured by THIS
    code, not by a second implementation, or a node that passes its unit
    scenario has been tested against different geometry from the benchmark.

    Frozen and hashable because it is part of every cache key. That is not a
    detail: the caches used to key on resolution alone, so two different worlds
    at the same resolution would have silently shared a grid, and a micro-world
    would have poisoned the occupancy grid that `core.scenario.validate()`
    decides solvability with.
    """

    rects: Tuple[Tuple[float, float, float, float], ...]
    bounds: Tuple[float, float, float, float]
    name: str = "unnamed"


LAB42 = World(rects=tuple(tuple(map(float, r)) for r in wall_rects()),
              bounds=tuple(map(float, bounds())), name="lab42")


def _world(world):
    """None means lab42 — so every existing call site keeps its meaning and the
    benchmark cannot accidentally be measured against a micro-world."""
    return LAB42 if world is None else world


def raycast(origin_xy, yaw, angles, max_range, world=None):
    """Range returns for beams at `angles` (robot frame) from `origin_xy`.

    Returns a float array the shape of `angles`, in metres, with `inf` where the
    beam leaves `max_range` without hitting a wall. `inf` rather than `max_range`
    so a controller can tell "nothing out there" from "a wall exactly at the
    sensor limit" — LimitSpeedToVisibleRange turns on precisely that difference.

    Slab test against each axis-aligned wall box, vectorised over beams. An
    origin inside a wall returns 0.0 for beams that start in it, which is what
    a real sensor buried in geometry would effectively report.
    """
    angles = np.atleast_1d(np.asarray(angles, dtype=float))
    # `bearings`, not `world`: the parameter now owns that name.
    bearings = angles + yaw
    dx = np.cos(bearings)
    dy = np.sin(bearings)
    px, py = float(origin_xy[0]), float(origin_xy[1])

    best = np.full(angles.shape, np.inf)

    # Division by a zero component is meant to produce +-inf here: a beam
    # parallel to a slab either misses it or is inside it for all t, and the
    # inf/-inf pair encodes exactly that. numpy's warning is noise.
    with np.errstate(divide="ignore", invalid="ignore"):
        for x0, y0, x1, y1 in _world(world).rects:
            tx0, tx1 = (x0 - px) / dx, (x1 - px) / dx
            ty0, ty1 = (y0 - py) / dy, (y1 - py) / dy
            tmin = np.maximum(np.minimum(tx0, tx1), np.minimum(ty0, ty1))
            tmax = np.minimum(np.maximum(tx0, tx1), np.maximum(ty0, ty1))

            # nan appears when a component is 0/0, i.e. the origin sits exactly
            # on a slab boundary and the beam is parallel to it. Treat as a miss.
            tmin = np.nan_to_num(tmin, nan=np.inf)
            tmax = np.nan_to_num(tmax, nan=-np.inf)

            hit = (tmax >= np.maximum(tmin, 0.0))
            dist = np.where(tmin >= 0.0, tmin, 0.0)
            best = np.where(hit & (dist < best), dist, best)

    best = np.where(best <= max_range, best, np.inf)
    return best


def clearance(xy, world=None):
    """Distance from a point to the nearest wall surface. 0.0 inside a wall.

    Vectorised: pass (2,) for one point or (..., 2) for many. This is the signal
    the collision and near-miss metrics threshold, so it is analytic rather than
    a physics contact query — a contact report depends on solver substeps, this
    does not.
    """
    p = np.asarray(xy, dtype=float)
    px, py = p[..., 0], p[..., 1]
    best = np.full(px.shape, np.inf)
    for x0, y0, x1, y1 in _world(world).rects:
        # Per-axis overhang; zero on an axis means the point is within the slab.
        ox = np.maximum(np.maximum(x0 - px, px - x1), 0.0)
        oy = np.maximum(np.maximum(y0 - py, py - y1), 0.0)
        best = np.minimum(best, np.hypot(ox, oy))
    return best if best.ndim else float(best)


_GRID_CACHE = {}


def occupancy_grid(resolution=GRID_RESOLUTION, inflate=ROBOT_RADIUS, world=None):
    """Rasterise the walls, inflated by `inflate`, over the room bounds.

    Configuration space, not workspace: a cell is occupied if the robot *centre*
    placed there would touch a wall. That is what makes the Dijkstra path below
    a path the robot could actually walk.

    Cached — every scenario on this floorplan wants the same grid.
    """
    world = _world(world)
    key = (world, round(resolution, 9), round(inflate, 9))
    if key in _GRID_CACHE:
        return _GRID_CACHE[key]

    min_x, min_y, max_x, max_y = world.bounds
    nx = int(round((max_x - min_x) / resolution))
    ny = int(round((max_y - min_y) / resolution))

    xs = min_x + (np.arange(nx) + 0.5) * resolution
    ys = min_y + (np.arange(ny) + 0.5) * resolution
    gx, gy = np.meshgrid(xs, ys)                       # both (ny, nx)

    occupied = clearance(np.stack([gx, gy], axis=-1), world) <= inflate

    grid = Grid(occupied, (min_x, min_y), resolution)
    _GRID_CACHE[key] = grid
    return grid


# Dijkstra over an 8-connected grid. The diagonal cost is sqrt(2), not 1:
# with unit diagonals every path through open floor is understated by up to
# ~30%, and SPL would then be systematically generous to every controller.
_NEIGHBOURS = [(dx, dy, math.hypot(dx, dy))
               for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)]

_FIELD_CACHE = {}


def geodesic_field(goal_xy, grid=None, world=None):
    """Distance from every free cell to `goal_xy`, in metres. `inf` if unreachable.

    Dijkstra rather than A*: the whole field is wanted, not one path. It is also
    what makes an unreachable goal loud — the start cell comes back `inf`, and
    core.scenario refuses to load a scenario rather than reporting a meaningless
    SPL for a task nobody could have solved.

    Diagonal steps may not cut a corner between two occupied cells; without that
    check the field leaks through the 0.15 m walls diagonally.
    """
    grid = grid if grid is not None else occupancy_grid(world=world)
    key = (round(float(goal_xy[0]), 6), round(float(goal_xy[1]), 6),
           id(grid))
    if key in _FIELD_CACHE:
        return _FIELD_CACHE[key]

    occ = grid.occupied
    ny, nx = occ.shape
    res = grid.resolution
    dist = np.full(occ.shape, np.inf)

    gix, giy = grid.world_to_cell(goal_xy)
    if occ[giy, gix]:
        # A goal inside inflated geometry has no cell to seed from. Leave the
        # field all-inf; the caller turns that into a scenario load failure.
        _FIELD_CACHE[key] = dist
        return dist

    dist[giy, gix] = 0.0
    heap = [(0.0, gix, giy)]
    while heap:
        d, ix, iy = heapq.heappop(heap)
        if d > dist[iy, ix]:
            continue
        for dx, dy, step in _NEIGHBOURS:
            jx, jy = ix + dx, iy + dy
            if not (0 <= jx < nx and 0 <= jy < ny) or occ[jy, jx]:
                continue
            if dx and dy and (occ[iy, jx] or occ[jy, ix]):
                continue                      # no squeezing between two corners
            nd = d + step * res
            if nd < dist[jy, jx]:
                dist[jy, jx] = nd
                heapq.heappush(heap, (nd, jx, jy))

    _FIELD_CACHE[key] = dist
    return dist


def geodesic_distance(start_xy, goal_xy, grid=None, world=None):
    """Grid path length from start to goal, `inf` if unreachable.

    NOT the SPL denominator any more — see shortest_path_length(). Movement here
    is restricted to multiples of 45 degrees, so this overestimates the true
    continuous shortest path: 11.0711 m against 10.5942 m on `doorway_traverse`,
    a 4.5% overstatement that does NOT shrink with resolution, because
    digitisation bias is a property of the connectivity and not of the cell
    size. On open floor with nothing in the way it is worse still — 3.828 m for
    a true 3.606 m over (1, 1) -> (4, 3), 6.2% — because a pure octile step
    cannot represent an arbitrary heading at all.

    Kept because it is what proves a scenario is solvable at all: the field is
    exhaustive over the room, so an unreachable goal comes back `inf` and
    core.scenario refuses to load. The visibility graph answers one query and
    cannot make that guarantee as cheaply.
    """
    grid = grid or occupancy_grid()
    field = geodesic_field(goal_xy, grid)
    ix, iy = grid.world_to_cell(start_xy)
    return float(field[iy, ix])


# ---------------------------------------------------------------------------
# The taut path: exact continuous shortest path, and the SPL denominator.
#
# geodesic_distance() above cannot travel at an arbitrary angle, so it overstates
# every route that is not a multiple of 45 degrees. As the denominator of SPL
# that is not merely conservative — it compresses the metric's discriminating
# range, and any controller within the bias band scores exactly 1.0. The
# functions below remove that bias by solving the continuous problem instead.
#
# Free space for the robot centre is the plane minus the wall boxes grown by
# ROBOT_RADIUS, i.e. minus a union of ROUNDED rectangles. A shortest path
# between two points in such a space is taut: straight segments that bend only
# where they touch a curved piece of the boundary, which here is the quarter-arc
# rounding off each wall corner. Sampling those arcs (see ARC_STEP_DEG) reduces
# it to a shortest path over a finite visibility graph.
# ---------------------------------------------------------------------------

# Slack on the "is this point far enough from a wall" test. Nodes and edges sit
# at exactly ROBOT_RADIUS from a corner by construction, so an exact >= test is
# at the mercy of the last bit of a hypot.
_FREE_EPS = 1e-9


def _point_rect_distance(points, rect):
    """Distance from each point to an axis-aligned box. 0.0 inside it."""
    x0, y0, x1, y1 = rect
    px, py = points[..., 0], points[..., 1]
    ox = np.maximum(np.maximum(x0 - px, px - x1), 0.0)
    oy = np.maximum(np.maximum(y0 - py, py - y1), 0.0)
    return np.hypot(ox, oy)


def _point_segment_distance(point, a, b):
    """Distance from one point to each segment a[i] -> b[i]."""
    ab = b - a
    denom = (ab * ab).sum(axis=-1)
    safe = np.where(denom > 0.0, denom, 1.0)
    t = np.clip(((point - a) * ab).sum(axis=-1) / safe, 0.0, 1.0)
    t = np.where(denom > 0.0, t, 0.0)
    closest = a + t[..., None] * ab
    return np.linalg.norm(point - closest, axis=-1)


def _segment_hits_rect(a, b, rect):
    """Slab test: does each segment a[i] -> b[i] touch the box?"""
    x0, y0, x1, y1 = rect
    lo = np.array([x0, y0])
    hi = np.array([x1, y1])
    d = b - a

    # A zero direction component is meant to produce +-inf: the segment is
    # parallel to that slab and is either inside it for all t (-inf, +inf) or
    # outside it for all t (+inf, +inf) / (-inf, -inf). Both encode correctly in
    # the min/max below. 0/0 on a segment starting exactly on a slab face is nan
    # and is widened to "no constraint".
    with np.errstate(divide="ignore", invalid="ignore"):
        inv = 1.0 / d
        t0 = (lo - a) * inv
        t1 = (hi - a) * inv
    tlo = np.nan_to_num(np.minimum(t0, t1), nan=-np.inf)
    thi = np.nan_to_num(np.maximum(t0, t1), nan=np.inf)

    tmin = tlo.max(axis=-1)
    tmax = thi.min(axis=-1)
    return (tmax >= np.maximum(tmin, 0.0)) & (tmin <= 1.0)


def segment_clearance(a, b, world=None):
    """Smallest wall distance anywhere along each segment. 0.0 where it crosses.

    Analytic, not sampled. A segment and an axis-aligned box are both convex, so
    when they are disjoint the closest pair always involves a vertex of one and
    a point on the other — checking the two segment endpoints against the box and
    the box's four corners against the segment is therefore exhaustive.

    Vectorised over segments: `a` and `b` are (..., 2).
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    best = np.full(a.shape[:-1], np.inf)
    for rect in _world(world).rects:
        x0, y0, x1, y1 = rect
        d = np.minimum(_point_rect_distance(a, rect), _point_rect_distance(b, rect))
        for corner in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            d = np.minimum(d, _point_segment_distance(np.array(corner), a, b))
        d = np.where(_segment_hits_rect(a, b, rect), 0.0, d)
        best = np.minimum(best, d)
    return best if best.ndim else float(best)


def _corner_arc_nodes(radius, step_deg, world=None):
    """Sample the quarter-arc that rounds off each wall corner.

    Only the OUTWARD quarter of each corner is sampled: past that the inflated
    boundary is a straight side parallel to the wall, where a taut path does not
    bend. Points buried inside another inflated wall, or outside the room, are
    dropped — they are not on the boundary of the free space.
    """
    step = math.radians(step_deg)
    n = max(1, int(math.ceil(math.radians(90.0) / step)))
    # Circumscribing radius: puts the CHORDS on the true inflated boundary
    # rather than inside it, so every edge built from these nodes is walkable.
    r = radius / math.cos(0.5 * math.radians(90.0) / n)

    world = _world(world)
    min_x, min_y, max_x, max_y = world.bounds
    out = []
    for x0, y0, x1, y1 in world.rects:
        quarters = (((x0, y0), 180.0), ((x1, y0), 270.0),
                    ((x1, y1), 0.0), ((x0, y1), 90.0))
        for (cx, cy), start_deg in quarters:
            angles = np.radians(start_deg + np.linspace(0.0, 90.0, n + 1))
            out.append(np.stack([cx + r * np.cos(angles),
                                 cy + r * np.sin(angles)], axis=-1))

    pts = np.concatenate(out, axis=0)
    inside = ((pts[:, 0] >= min_x) & (pts[:, 0] <= max_x) &
              (pts[:, 1] >= min_y) & (pts[:, 1] <= max_y))
    return pts[inside & (clearance(pts, world) >= radius - _FREE_EPS)]


_VISGRAPH_CACHE = {}


def _visibility_graph(radius=ROBOT_RADIUS, step_deg=ARC_STEP_DEG, world=None):
    """(nodes, weights) over the sampled corner arcs. Cached — it is fixed by
    the floorplan, so start and goal are the only per-query work."""
    world = _world(world)
    key = (world, round(radius, 9), round(step_deg, 9))
    if key in _VISGRAPH_CACHE:
        return _VISGRAPH_CACHE[key]

    nodes = _corner_arc_nodes(radius, step_deg, world)
    n = len(nodes)
    weights = np.full((n, n), np.inf)
    if n:
        i, j = np.triu_indices(n, k=1)
        visible = segment_clearance(nodes[i], nodes[j], world) >= radius - _FREE_EPS
        length = np.linalg.norm(nodes[i] - nodes[j], axis=-1)
        weights[i[visible], j[visible]] = length[visible]
        weights[j[visible], i[visible]] = length[visible]
        np.fill_diagonal(weights, 0.0)

    _VISGRAPH_CACHE[key] = (nodes, weights)
    return nodes, weights


def _dijkstra(weights, source, target):
    """Path length and node sequence over a dense weight matrix."""
    n = len(weights)
    dist = np.full(n, np.inf)
    prev = np.full(n, -1, dtype=int)
    dist[source] = 0.0
    seen = np.zeros(n, dtype=bool)
    heap = [(0.0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if seen[u]:
            continue
        seen[u] = True
        if u == target:
            break
        for v in np.flatnonzero(np.isfinite(weights[u]) & ~seen):
            nd = d + weights[u, v]
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (float(nd), int(v)))

    if not np.isfinite(dist[target]):
        return math.inf, []
    path, u = [], target
    while u != -1:
        path.append(u)
        u = prev[u]
    return float(dist[target]), path[::-1]


def shortest_path(start_xy, goal_xy, radius=ROBOT_RADIUS, step_deg=ARC_STEP_DEG,
                  world=None):
    """The taut path from start to goal as a polyline. `[]` if unreachable.

    Exposed because a length on its own cannot be checked. A test can walk this
    polyline and assert every segment clears `radius`, which is the property that
    makes the length trustworthy as an SPL denominator.
    """
    start = np.asarray(start_xy, dtype=float)[:2]
    goal = np.asarray(goal_xy, dtype=float)[:2]
    corners, corner_w = _visibility_graph(radius, step_deg, world)

    n = len(corners)
    nodes = np.concatenate([corners, start[None, :], goal[None, :]], axis=0)
    s, g = n, n + 1

    weights = np.full((n + 2, n + 2), np.inf)
    weights[:n, :n] = corner_w
    np.fill_diagonal(weights, 0.0)

    # Endpoints inside inflated geometry have no legal edge at all; the caller
    # turns the resulting inf into a scenario load failure rather than a score.
    for idx, point in ((s, start), (g, goal)):
        if clearance(point[None, :], world)[0] < radius - _FREE_EPS:
            return []

    for idx, point in ((s, start), (g, goal)):
        if n:
            reach = segment_clearance(np.broadcast_to(point, corners.shape),
                                      corners, world) >= radius - _FREE_EPS
            length = np.linalg.norm(corners - point, axis=-1)
            weights[idx, :n] = np.where(reach, length, np.inf)
            weights[:n, idx] = weights[idx, :n]

    if segment_clearance(start, goal) >= radius - _FREE_EPS:
        direct = float(np.linalg.norm(goal - start))
        weights[s, g] = weights[g, s] = direct

    _, order = _dijkstra(weights, s, g)
    return [tuple(nodes[i]) for i in order]


def shortest_path_length(start_xy, goal_xy, radius=ROBOT_RADIUS,
                         step_deg=ARC_STEP_DEG, stop_radius=0.0, world=None):
    """Length of the taut path from start to goal. `inf` if unreachable.

    **The SPL denominator.** Continuous, so unlike geodesic_distance() it is not
    biased by the 45-degree steps of a grid.

    `stop_radius` shortens the answer to the distance needed to get *within*
    `stop_radius` of the goal, which is what a success criterion of "within
    GOAL_RADIUS of the goal" actually asks of the robot. Subtracting a constant
    cannot change which path is shortest, so this is exact — provided the disc of
    that radius around the goal lies in free space, since otherwise the last
    stretch of the straight run-in might not be walkable. Callers are expected to
    hold that (core.scenario.validate checks it).
    """
    path = shortest_path(start_xy, goal_xy, radius, step_deg, world)
    if not path:
        return math.inf
    total = float(sum(math.dist(path[i], path[i + 1])
                      for i in range(len(path) - 1)))
    return max(0.0, total - stop_radius)
