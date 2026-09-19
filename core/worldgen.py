"""
core/worldgen.py

**Seeded, randomised worlds.** Chunk 45, epic 05.

lab42 is one hand-authored floorplan, and every node ever generated has been
aimed at its one divider, one doorway and one blind corner. This produces
`core.geometry.World`s from a seed instead: a whole distribution of rooms, in
three difficulty tiers, each reproducible from `(tier, seed)` alone.

⚠️ **A WORLD IS NOT A SCENARIO.** This module produces geometry and candidate
start/goal pairs. Turning those into `core.scenario.Scenario`s — with time
limits, seeds, drift and goal disclosure — is chunk 46. Nothing here imports
`core.scenario`, and nothing here may import `core.scenarios_held_out`.

WHY IT IS SEEDED AND PROCEDURAL RATHER THAN HAND-AUTHORED
----------------------------------------------------------
Two reasons, and the second is declared in `PRE-REGISTRATION.md`'s 2026-09-13
amendment as a guard against a threat this epic introduces.

1. **It is the only available fix for the power problem.** `Scenario.seed`
   reaches only `rng()` and `drift_rng()`, so on an undrifted lab42 scenario
   every replicate seed is the *same episode* — measured on three controllers
   and confirmed under physics. Effective N is the scenario count, and the
   pre-registered rule sits at ~0.16 power. A layout that varies with the seed
   makes replicate seeds genuine samples.
2. **It stops the benchmark being shaped to fit the nodes that already exist.**
   Replacing the benchmark after building the pipeline on the old one is a
   researcher degree of freedom. A distribution drawn from a seed cannot be
   quietly tuned scenario by scenario the way twelve hand-written rooms can.

DIFFICULTY IS MEASURED, NEVER ASSERTED
---------------------------------------
A tier sets *generation parameters*, not a difficulty label. What comes back
carries `Difficulty`, computed from the geometry by `core.geometry` — the same
code that scores the benchmark, so a world cannot be "hard" by claim. Tiers are
therefore a hypothesis about difficulty that `measure()` can falsify, which is
what makes `tier_report()` worth running before a suite is frozen.

SIM-FREE. No isaacsim/omni/torch/pxr/carb.

Examples
--------
    P=~/miniforge3/envs/env_isaacsim/bin/python

    $P -c "
    from core.worldgen import generate, TIERS
    g = generate('cluttered', seed=7)
    print(g.world.name, len(g.world.rects), 'rects')
    print(g.difficulty)
    print(g.start, '->', g.goal)"

    # does a tier actually produce what it claims?
    $P -c "
    from core.worldgen import tier_report
    print(tier_report('maze', n=20))"
"""

import math
import zlib
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from core.geometry import (ROBOT_RADIUS, World, clearance, geodesic_distance,
                           occupancy_grid, shortest_path_length)
from core.scenario import Scenario

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.

# The room every generated world lives in. Same footprint as lab42 (12 x 8 m) on
# purpose: the Go2's turning circle, the lidar range and the 60 s episode cap
# were all calibrated against that scale, and changing the geometry AND the size
# at once would make a regression impossible to attribute.
ROOM = (0.0, 0.0, 12.0, 8.0)

# Matches core.scene_spec.WALL_THICKNESS. Walls are drawn as axis-aligned boxes
# because core.geometry.raycast is analytic against exactly that.
WALL_THICKNESS = 0.15

# A doorway must admit the robot at any heading, with margin. ROBOT_RADIUS
# circumscribes the body, so 2 * R is the bare minimum and would leave a path
# that only exists at one heading; the margin is what makes it walkable.
MIN_DOOR_WIDTH = 2.0 * ROBOT_RADIUS + 0.30       # 0.90 m
MIN_ROOM_SPAN = 4.0 * ROBOT_RADIUS + 0.60        # 1.80 m — do not partition below this

# A start or goal must stand this far off any wall. Larger than ROBOT_RADIUS so
# the robot is not spawned in contact, and so the run-in disc that
# shortest_path_length(stop_radius=...) assumes is genuinely free.
ENDPOINT_CLEARANCE = ROBOT_RADIUS + 0.25

# ⚠️ The GOAL needs more room than the start, and the number is not arbitrary:
# `core.scenario.validate` refuses a goal within ROBOT_RADIUS + GOAL_RADIUS of a
# wall, because `optimal_distance()` subtracts GOAL_RADIUS from the path and that
# is only the same thing as "distance to the goal DISC" when the whole disc is
# walkable. GOAL_RADIUS is 0.5, so the bar is 0.80 m; the margin makes a
# generated goal pass that check rather than fail it at suite-build time.
GOAL_CLEARANCE = ROBOT_RADIUS + 0.5 + 0.05

# What a generated pair must be worth measuring. A start and goal three metres
# apart in an empty corner tests nothing.
MIN_EUCLIDEAN = 6.0        # metres apart, straight line
MIN_TORTUOSITY = 1.05      # taut path / straight line — some detour required

# The tiers. `partitions` is how many interior walls are drawn; `door_frac` is
# how much of each wall is left open, as a fraction of its span; `clutter` is
# how many freestanding blocks are dropped in afterwards; `symmetry` mirrors the
# layout about the room's midline, which is the geometry-induced localisation
# stressor (two halves that scan alike defeat pose estimation without touching
# the estimator, so it needs no change to core/drift.py and inherits none of
# epic 03's ordering constraint).
#
# `detour` is how many candidate start/goal pairs are considered before keeping
# the most circuitous. ⚠️ It exists because `tier_report` FALSIFIED the first
# version of these tiers: over 25 seeds each, `cluttered` scored tortuosity
# 1.30+-0.25 and `maze` 1.28+-0.19 — indistinguishable. More partitions make
# rooms DENSER, not routes LONGER, and the endpoint sampler was taking the first
# pair 6 m apart regardless of how direct the route was.
#
# ⚠️ AND IT ONLY HALF WORKED, WHICH IS THE HONEST STATE. With `detour` the means
# do separate (cluttered 1.78+-0.44, maze 2.10+-0.45 at n=25) but the
# DISTRIBUTIONS OVERLAP HEAVILY — cluttered reaches 2.90, maze descends to 1.23,
# and tuning cluttered's detour over 2/3/4 moved it only 1.73 -> 1.87. So the
# tiers are NOT reliably ordered by tortuosity and the test does not assert that
# they are. What they DO order, strictly and tightly, is density and tightness:
# free_fraction 0.75/0.63/0.52, bottleneck 0.84/0.53/0.40 m, obstacles 4/11/20.
# Read the tiers as "how cluttered and how tight", not "how far around".
TIERS = {
    "open": dict(partitions=(1, 2), door_frac=(0.30, 0.45),
                 clutter=(0, 2), block=(0.4, 0.9), symmetry=0.0, detour=1),
    "cluttered": dict(partitions=(2, 4), door_frac=(0.16, 0.28),
                      clutter=(3, 7), block=(0.3, 1.1), symmetry=0.0, detour=6),
    "maze": dict(partitions=(4, 6), door_frac=(0.10, 0.18),
                 clutter=(4, 9), block=(0.3, 1.2), symmetry=0.5, detour=40),
}

# Give up on a seed after this many failed attempts and report it, rather than
# looping forever on a tier whose parameters cannot produce a solvable room.
MAX_ATTEMPTS = 60

# `core.geometry`'s caches are keyed on the World and never evict. One world was
# fine; a thousand generated ones is a thousand occupancy grids, three thousand
# cached arrays, and eventually the machine. Callers generating in bulk must
# bound it — see `clear_geometry_caches`.
CACHE_SOFT_LIMIT = 64


@dataclass(frozen=True)
class Difficulty:
    """What the geometry actually is, measured by the benchmark's own code."""
    geodesic_length: float      # grid path; what difficulty is screened on
    taut_length: float          # the SPL denominator. NaN unless with_taut=True — see measure()
    euclidean: float            # straight-line start -> goal
    tortuosity: float           # geodesic / euclidean; 1.0 is a clear run
    free_fraction: float        # of the room, how much the robot centre may occupy
    min_corridor: float         # BOTTLENECK radius: the largest robot that still fits
    n_obstacles: int

    def __str__(self):
        return (f"geo={self.geodesic_length:.2f}m tort={self.tortuosity:.2f} "
                f"free={self.free_fraction:.2f} corridor={self.min_corridor:.2f}m "
                f"obstacles={self.n_obstacles}")


@dataclass(frozen=True)
class GeneratedWorld:
    """A world, a start, a goal, and what makes it hard. Chunk 46 turns this
    into a Scenario; this module deliberately stops short of that."""
    world: World
    start: Tuple[float, float]
    goal: Tuple[float, float]
    tier: str
    seed: int
    difficulty: Difficulty
    attempts: int               # how many draws it took; a tier needing many is badly tuned


class WorldgenError(RuntimeError):
    """A tier could not produce a solvable world for this seed inside
    MAX_ATTEMPTS. Raised rather than returned: silently substituting an easier
    room would bias the distribution toward exactly the rooms the suite exists
    to avoid."""


def clear_geometry_caches():
    """Drop `core.geometry`'s memo tables.

    They are keyed on the World and unbounded, which is right for one floorplan
    and wrong for a generated distribution. Call this between batches. It is
    correctness-neutral — the caches are pure memos — and only ever costs
    recomputation.
    """
    from core import geometry
    for name in ("_GRID_CACHE", "_FIELD_CACHE", "_VISGRAPH_CACHE"):
        cache = getattr(geometry, name, None)
        if isinstance(cache, dict):
            cache.clear()


def _boundary_walls():
    """The room's own four walls, as boxes lying inside ROOM."""
    x0, y0, x1, y1 = ROOM
    t = WALL_THICKNESS
    return [(x0, y0, x1, y0 + t), (x0, y1 - t, x1, y1),
            (x0, y0, x0 + t, y1), (x1 - t, y0, x1, y1)]


def _split_wall(rng, cell, vertical, door_frac):
    """One interior wall across `cell`, with a gap in it.

    Returns (rects, sub_cell_a, sub_cell_b) or None when the cell is too small
    to divide without producing a room the robot cannot turn around in.
    """
    x0, y0, x1, y1 = cell
    span = (x1 - x0) if vertical else (y1 - y0)
    cross = (y1 - y0) if vertical else (x1 - x0)
    if span < 2 * MIN_ROOM_SPAN or cross < MIN_ROOM_SPAN:
        return None

    # Where to cut. Kept away from the ends so neither child is a sliver.
    lo, hi = x0 + MIN_ROOM_SPAN, x1 - MIN_ROOM_SPAN
    if not vertical:
        lo, hi = y0 + MIN_ROOM_SPAN, y1 - MIN_ROOM_SPAN
    if hi <= lo:
        return None
    cut = float(rng.uniform(lo, hi))

    # The gap. door_frac is a fraction of the wall's own length, floored at
    # MIN_DOOR_WIDTH so a tight tier cannot generate an impassable doorway —
    # an unsolvable room is not a hard room, it is a wasted draw.
    door = max(MIN_DOOR_WIDTH, cross * float(rng.uniform(*door_frac)))
    door = min(door, cross - 2 * WALL_THICKNESS)
    if door < MIN_DOOR_WIDTH:
        return None
    gap_lo = float(rng.uniform(y0 + WALL_THICKNESS, y1 - WALL_THICKNESS - door)) \
        if vertical else \
        float(rng.uniform(x0 + WALL_THICKNESS, x1 - WALL_THICKNESS - door))

    t = WALL_THICKNESS
    if vertical:
        rects = [(cut, y0, cut + t, gap_lo), (cut, gap_lo + door, cut + t, y1)]
        return rects, (x0, y0, cut, y1), (cut + t, y0, x1, y1)
    rects = [(x0, cut, gap_lo, cut + t), (gap_lo + door, cut, x1, cut + t)]
    return rects, (x0, y0, x1, cut), (x0, cut + t, x1, y1)


def _partition(rng, params):
    """Recursively cut the room into rooms joined by doorways.

    Recursive splitting rather than random wall placement because it produces
    the motif that makes navigation hard in a building: rooms connected by
    single apertures, so a greedy controller heading straight at the goal meets
    a wall and must commit to a doorway that leads away from it. Random walls
    mostly produce clutter, which is a different and easier problem.
    """
    rects = list(_boundary_walls())
    cells = [(WALL_THICKNESS, WALL_THICKNESS,
              ROOM[2] - WALL_THICKNESS, ROOM[3] - WALL_THICKNESS)]
    target = int(rng.integers(params["partitions"][0], params["partitions"][1] + 1))

    made = 0
    guard = 0
    while made < target and cells and guard < 40:
        guard += 1
        # Split the largest cell: keeps the room sizes even instead of shaving
        # slivers off one corner.
        cells.sort(key=lambda c: (c[2] - c[0]) * (c[3] - c[1]), reverse=True)
        cell = cells.pop(0)
        vertical = (cell[2] - cell[0]) >= (cell[3] - cell[1])
        out = _split_wall(rng, cell, vertical, params["door_frac"])
        if out is None:
            out = _split_wall(rng, cell, not vertical, params["door_frac"])
        if out is None:
            continue
        new_rects, a, b = out
        rects.extend(new_rects)
        cells.extend([a, b])
        made += 1
    return rects, cells


def _add_clutter(rng, rects, cells, params):
    """Freestanding blocks. These make local minima and blind corners."""
    n = int(rng.integers(params["clutter"][0], params["clutter"][1] + 1))
    placed = 0
    for _ in range(n * 6):
        if placed >= n or not cells:
            break
        cell = cells[int(rng.integers(0, len(cells)))]
        w = float(rng.uniform(*params["block"]))
        h = float(rng.uniform(*params["block"]))
        if (cell[2] - cell[0]) < w + 2 * MIN_DOOR_WIDTH:
            continue
        if (cell[3] - cell[1]) < h + 2 * MIN_DOOR_WIDTH:
            continue
        bx = float(rng.uniform(cell[0] + MIN_DOOR_WIDTH, cell[2] - w - MIN_DOOR_WIDTH))
        by = float(rng.uniform(cell[1] + MIN_DOOR_WIDTH, cell[3] - h - MIN_DOOR_WIDTH))
        rects.append((bx, by, bx + w, by + h))
        placed += 1
    return rects


def _mirror(rects, probability, rng):
    """Mirror the layout about x = 6 with `probability`.

    ⚠️ This is the LOCALISATION stressor, and it is geometric on purpose. Two
    halves that scan alike produce genuine pose ambiguity without touching
    `core/drift.py` or the localiser — so it inherits none of epic 03's ordering
    constraint, which forbids changing the drift model before the first
    evaluation run.
    """
    if probability <= 0.0 or rng.random() > probability:
        return rects, False
    mid = 0.5 * (ROOM[0] + ROOM[2])
    out = list(rects)
    for (x0, y0, x1, y1) in rects:
        if x1 <= mid:
            out.append((2 * mid - x1, y0, 2 * mid - x0, y1))
    return out, True



def _components(grid):
    """Label the free space, **4-connected**.

    ⚠️ WHY NOT `geodesic_distance`: connectivity is a reachability question and
    Dijkstra answers a much harder one. Endpoint sampling asked it up to 40
    times per draw, each on a fresh goal field, and a single `maze` world cost
    ~50 s. Labelling the free space once is O(cells).

    ⚠️ WHY 4-CONNECTED AND NOT 8, honestly. `core.geometry.geodesic_field`
    forbids a diagonal step that cuts the corner between two occupied cells —
    "without that check the field leaks through the 0.15 m walls diagonally" —
    and 8-connected labelling permits exactly those pinches. 4-connectivity is
    STRICTLY CONSERVATIVE against the geodesic: every 4-connected route is one
    the geodesic also admits, so a world accepted here is accepted by
    `core.scenario.validate` downstream. It can in principle reject a layout
    whose only route needs a diagonal, which at 0.05 m cells and a 0.90 m
    minimum doorway does not arise — a real corridor is eighteen cells wide.

    ⚠️ **Correction, and it matters because the wrong cause was written here
    first.** This was changed from 8 to 4 while chasing generated worlds that
    `geodesic_distance` called unsolvable. **Connectivity was not the cause.**
    `core.geometry.geodesic_distance` was dropping its `world` argument
    (`grid = grid or occupancy_grid()`), so it answered about LAB42 for every
    generated world. Fixed 2026-09-13 in `core/geometry.py`; unsolvable worlds
    went 32/48 -> 0/48. 4-connectivity is kept on the conservative argument
    above, NOT because it fixed anything.
    """
    from scipy import ndimage
    structure = np.array([[0, 1, 0],
                          [1, 1, 1],
                          [0, 1, 0]], dtype=bool)     # 4-connected
    labels, n = ndimage.label(~grid.occupied, structure=structure)
    return labels, n


def _sample_endpoints(rng, world, grid, detour=1):
    """A start and a goal that are far apart, clear of walls, and connected.

    Returns None if this layout cannot supply such a pair, which is a fact about
    the layout and a reason to redraw it rather than to relax the requirement.
    """
    labels, n = _components(grid)
    if n == 0:
        return None
    # Work inside the LARGEST component. A layout whose free space is split by a
    # wall with no doorway is not unsolvable — it is two rooms — and sampling
    # across the split would reject a perfectly good layout for the wrong reason.
    sizes = np.bincount(labels.ravel())
    sizes[0] = 0                                   # label 0 is the occupied set
    biggest = int(sizes.argmax())
    free = np.argwhere(labels == biggest)
    if len(free) < 2:
        return None

    # Clearance for every candidate at once; the per-point call inside a loop
    # was the second-largest cost after the Dijkstras.
    pts = np.array([grid.cell_to_world(int(c), int(r)) for r, c in free])
    clear = clearance(pts, world)
    starts = pts[clear >= ENDPOINT_CLEARANCE]
    goals = pts[clear >= GOAL_CLEARANCE]        # stricter — see GOAL_CLEARANCE
    if len(starts) < 1 or len(goals) < 1:
        return None

    # Pick the GOAL once and build its geodesic field once; then every candidate
    # start is a table lookup rather than another Dijkstra. That is what makes a
    # detour bias of 40 cost the same as a bias of 1.
    from core.geometry import geodesic_field
    best = None
    for _ in range(12):
        goal = tuple(goals[int(rng.integers(0, len(goals)))])
        field = geodesic_field(goal, grid=grid, world=world)
        for _ in range(max(1, detour)):
            a = tuple(starts[int(rng.integers(0, len(starts)))])
            euclid = math.dist(a, goal)
            if euclid < MIN_EUCLIDEAN:
                continue
            ix, iy = grid.world_to_cell(a)
            geo = float(field[iy, ix])
            # Same component, so this is finite except at the grid's own edge
            # cases; guard anyway rather than propagate an inf into a ratio.
            if not math.isfinite(geo):
                continue
            tort = geo / euclid
            if best is None or tort > best[0]:
                best = (tort, a, goal)
        if best is not None and detour <= 1:
            break
    return (best[1], best[2]) if best else None


def _bottleneck_radius(world, start, goal, lo=ROBOT_RADIUS, hi=1.60, tol=0.02):
    """Largest inflation radius at which start and goal are still connected.

    Monotone in the radius, so bisection is exact to `tol`. `lo` starts at
    ROBOT_RADIUS because a route the real robot cannot pass is not a route; a
    return of `lo` means "only just passable", and larger means slack.
    """
    def connected(radius):
        grid = occupancy_grid(inflate=radius, world=world)
        labels, n = _components(grid)
        if n == 0:
            return False
        ia = grid.world_to_cell(start)
        ib = grid.world_to_cell(goal)
        la, lb = labels[ia[1], ia[0]], labels[ib[1], ib[0]]
        return bool(la != 0 and la == lb)

    if not connected(lo):
        return float("nan")          # not even passable by this robot
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if connected(mid) else (lo, mid)
    return float(lo)


def measure(world, start, goal, grid=None, with_bottleneck=True, with_taut=False):
    """Difficulty, computed by the benchmark's own geometry code.

    ⚠️ `with_taut` is OFF by default and that is a measured decision, not a
    preference. `core.geometry.shortest_path_length` — the SPL denominator —
    builds a visibility graph over arc samples at every obstacle corner, so its
    cost is quadratic in obstacle count. Profiled 2026-09-13 on one `maze`
    world with 22 rects:

        occupancy_grid       23.9 ms
        _sample_endpoints   314.2 ms
        geodesic_distance    87.5 ms
        _bottleneck_radius  105.2 ms
        shortest_path_length  43,536 ms   <-- everything else is noise

    lab42 has 7 rects and pays ~1 s. A generated world has three times that and
    pays forty seconds. Difficulty screening therefore uses the GEODESIC, which
    `core.geometry` documents as ~4.5% longer than the taut path on
    `doorway_traverse` (45-degree steps), and the exact denominator is left to
    whoever actually scores an episode.

    ⚠️ **This is a problem for the epic, not just for this function.** SPL is a
    named secondary in `PRE-REGISTRATION.md`, and scoring an episode on a
    generated world will pay that 43 s once per world. Recorded in
    `plans/05-epic-environment-suite.md`.
    """
    grid = grid if grid is not None else occupancy_grid(world=world)
    euclid = float(math.dist(start, goal))
    free = float((~grid.occupied).mean())
    geo = float(geodesic_distance(start, goal, grid=grid, world=world))
    taut = float(shortest_path_length(start, goal, world=world)) if with_taut \
        else float("nan")

    # Narrowest squeeze on the route, as a BOTTLENECK RADIUS: the largest robot
    # that could still get from start to goal at all.
    #
    # ⚠️ Two obvious versions are both meaningless, and both were written first.
    # (a) minimum clearance over free cells — `occupancy_grid` DEFINES free as
    # clearance > ROBOT_RADIUS, so the answer is always ~0.30 m. (b) minimum
    # clearance along the taut path — `shortest_path` is taut around the
    # INFLATED boundary, so it hugs every rounded corner at exactly
    # ROBOT_RADIUS, and the answer is again always ~0.30 m. Every tier reported
    # 0.30 m under both, which is what exposed them.
    #
    # Binary search on the inflation radius is the real question: at what body
    # size does the route close? Monotone, so bisection is exact to the
    # tolerance, and it is cheap now that connectivity is a component label.
    corridor = (_bottleneck_radius(world, start, goal)
                if with_bottleneck else float("nan"))

    return Difficulty(
        geodesic_length=geo,
        taut_length=taut,
        euclidean=euclid,
        tortuosity=(geo / euclid) if euclid > 0 and math.isfinite(geo) else float("inf"),
        free_fraction=free,
        min_corridor=corridor,
        n_obstacles=len(world.rects) - 4,          # less the four boundary walls
    )


def generate(tier, seed, require_detour=True):
    """One reproducible world for `(tier, seed)`.

    Redraws internally until the layout is solvable and worth measuring; the
    draw sequence is a pure function of the seed, so the same call always
    returns the same world. Raises `WorldgenError` rather than returning an
    easier room.
    """
    if tier not in TIERS:
        raise KeyError(f"unknown tier {tier!r}. Known: {sorted(TIERS)}")
    params = TIERS[tier]

    for attempt in range(1, MAX_ATTEMPTS + 1):
        # One stream per attempt, derived from the seed, so attempt N is
        # reproducible without replaying attempts 1..N-1.
        # ⚠️ NOT `hash(tier)`. Python randomises str hashing per process
        # (PYTHONHASHSEED), so `abs(hash(tier))` made `generate(tier, seed)`
        # return a DIFFERENT WORLD IN EVERY PROCESS — 21, 24 and 31 rects on
        # three consecutive runs of the same call. That silently invalidated the
        # module's central claim, the chunk 47 base calibration, and a growth
        # measurement, and the test missed it because it compared two calls
        # inside ONE process. crc32 is stable across processes and versions.
        tier_key = zlib.crc32(tier.encode("utf-8"))
        rng = np.random.default_rng([tier_key, int(seed), attempt])

        rects, cells = _partition(rng, params)
        rects = _add_clutter(rng, rects, cells, params)
        rects, mirrored = _mirror(rects, params["symmetry"], rng)

        name = f"{tier}-{seed:05d}" + ("-sym" if mirrored else "")
        world = World(rects=tuple(tuple(map(float, r)) for r in rects),
                      bounds=tuple(map(float, ROOM)), name=name)

        grid = occupancy_grid(world=world)
        pair = _sample_endpoints(rng, world, grid, detour=params["detour"])
        if pair is None:
            continue
        start, goal = pair
        # Cheap screen first. The bottleneck search costs ~40x the rest of the
        # draw, and a world rejected for tortuosity never needs it.
        diff = measure(world, start, goal, grid=grid, with_bottleneck=False)
        if not math.isfinite(diff.geodesic_length):
            continue
        if require_detour and diff.tortuosity < MIN_TORTUOSITY:
            continue
        diff = measure(world, start, goal, grid=grid, with_bottleneck=True)
        return GeneratedWorld(world=world, start=start, goal=goal, tier=tier,
                              seed=int(seed), difficulty=diff, attempts=attempt)

    raise WorldgenError(
        f"tier {tier!r} produced no solvable world for seed {seed} in "
        f"{MAX_ATTEMPTS} attempts. That is a badly tuned tier, not a bad seed: "
        f"loosen door_frac or clutter in TIERS rather than retrying.")


def batch(tier, seeds, bound_caches=True):
    """Generate many, keeping `core.geometry`'s caches bounded as we go."""
    out = []
    for i, seed in enumerate(seeds):
        if bound_caches and i and i % CACHE_SOFT_LIMIT == 0:
            clear_geometry_caches()
        out.append(generate(tier, seed))
    return out


def tier_report(tier, n=20, start_seed=1000):
    """Does this tier produce what it claims? Difficulty, aggregated.

    The point of the module's "difficulty is measured, never asserted" rule: run
    this before freezing a suite, and if `open` and `maze` overlap, the tiers are
    a label and not a fact.
    """
    worlds = batch(tier, range(start_seed, start_seed + n))
    def stat(f):
        v = np.array([f(w.difficulty) for w in worlds], dtype=float)
        return f"{v.mean():.2f}+-{v.std():.2f}"
    return {
        "tier": tier, "n": len(worlds),
        "taut_length": stat(lambda d: d.taut_length),
        "tortuosity": stat(lambda d: d.tortuosity),
        "free_fraction": stat(lambda d: d.free_fraction),
        "min_corridor": stat(lambda d: d.min_corridor),
        "n_obstacles": stat(lambda d: d.n_obstacles),
        "attempts": stat(lambda d: 0) if not worlds else
                    f"{np.mean([w.attempts for w in worlds]):.2f}",
    }


# ---------------------------------------------------------------------------
# A generated world, as something the rollout can actually drive.

@dataclass(frozen=True)
class GeneratedScenario(Scenario):
    """A `core.scenario.Scenario` that lives on a generated world.

    ⚠️ **No frozen-contract change.** Chunk 11a closed `Scenario`'s field list,
    and `Scenario.world` was deliberately left a derived PROPERTY returning
    LAB42 so that `harness.micro.MicroScenario` could answer differently. This
    is the same move: one property override, no new benchmark field, nothing
    sealed or hashed changes.

    `scene` names the USD `scripts/export_world.py` writes for this world
    (chunk 53, 2026-09-15). Sim-free rollouts never load it; a physics run needs
    it exported first, and pointing this at lab42's USD instead would silently
    run the wrong geometry. Until 2026-09-15 no exporter existed and this
    docstring said so.

        python scripts/export_world.py --tier maze --seed 2004
    """

    gen_world: object = None        # the core.geometry.World; see `world` below

    @property
    def world(self):
        return self.gen_world


def to_scenario(g, time_limit=120.0, goal_known=True, split="dev", seed=None):
    """`GeneratedWorld` -> a scenario the kinematic rollout and the runner accept.

    ⚠️ Deliberately does NOT call `core.scenario.validate`. Validation calls
    `optimal_distance()` -> `shortest_path_length`, which costs ~43 s on a
    generated world (epic 05, chunk 45 outcome), and chunk 47's calibration
    needs `goal_reached` over hundreds of worlds and no denominator at all.
    A scenario destined for the FROZEN SUITE must be validated — that is chunk
    46's job, once, with the denominator computed and committed alongside it.
    """
    sx, sy = g.start
    gx, gy = g.goal
    yaw = math.atan2(gy - sy, gx - sx)      # face the goal; no free heading bonus
    return GeneratedScenario(
        id=f"gen-{g.tier}-{g.seed:05d}",
        family=f"gen_{g.tier}",
        split=split,
        start=(float(sx), float(sy), float(yaw)),
        goal=(float(gx), float(gy)),
        time_limit=float(time_limit),
        seed=int(g.seed if seed is None else seed),
        scene=f"scenes/generated/{g.world.name}.usd",
        goal_known=goal_known,
        gen_world=g.world,
    )
