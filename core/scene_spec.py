"""
core/scene_spec.py

The lab42 floorplan, as data. This is the single definition — build_scene.py
imports it to author the USD, and the geometry, scenario and metric code
imports it to reason about the same room without a simulator.

It used to live at the top of scripts/build_scene.py, which constructs
SimulationApp and calls parse_args() at module import. Reading the floorplan
from there meant booting a simulator and having argv eaten, so nothing could
depend on it. Hence this module: no isaacsim, no omni, no torch, no argparse.

    python -c "from core.scene_spec import bounds; print(bounds())"
    -> (0.0, 0.0, 12.0, 8.0), in well under a second, no simulator banner.
"""

# ---------------------------------------------------------------------------
# Scene definition. Everything below this line is data, not code.
# ---------------------------------------------------------------------------

WALL_HEIGHT = 1.5       # metres
WALL_THICKNESS = 0.15

# (centre_x, centre_y, length, yaw_degrees)
# Room spans x in [0, 12], y in [0, 8].
WALLS = [
    (6.0, 0.0, 12.0, 0.0),    # south perimeter
    (6.0, 8.0, 12.0, 0.0),    # north perimeter
    (0.0, 4.0, 8.0, 90.0),    # west perimeter
    (12.0, 4.0, 8.0, 90.0),   # east perimeter
    (6.0, 2.5, 5.0, 90.0),    # divider, leaves a doorway at y in [5, 8]
    (9.0, 5.0, 6.0, 0.0),     # creates a blind corner at (6, 5)
]

GO2_START = (1.5, 1.5, 0.55)  # spawned slightly high so it drops and settles
GOAL = (10.5, 6.5)            # reachable only via the doorway and blind corner

# Lighting. Part of the scene spec, not decoration: RTX camera and lidar
# conditions depend on it, so it must be identical across scene variants.
DOME_INTENSITY = 700.0        # ambient fill, lifts the shadows
KEY_INTENSITY = 1500.0        # single directional light, gives shape
KEY_ROTATION = (-50.0, 0.0, 25.0)

CEILING_HEIGHT = 2.8
CEILING_RADIUS = 0.25
CEILING_INTENSITY = 40000.0
CEILING_LIGHTS = [           # (x, y), roughly one per quadrant
    (3.0, 2.0), (3.0, 6.0),
    (9.0, 2.0), (9.0, 6.5),
]


# ---------------------------------------------------------------------------

def _along_x(cx, cy, yaw_deg):
    """True if the wall runs along x, False if along y.

    Every wall in lab42 is axis-aligned, which is what lets the raycast and the
    occupancy grid stay analytic and exact rather than meshing anything. A wall
    at some other yaw would need an oriented-box representation throughout the
    geometry code, so this raises rather than letting the rest of the module
    quietly describe a rectangle wider than the wall it came from.
    """
    turns = yaw_deg / 90.0
    if abs(turns - round(turns)) > 1e-9:
        raise ValueError(
            f"wall at ({cx}, {cy}) has yaw {yaw_deg} deg; the geometry code "
            "only handles axis-aligned walls. Add oriented-box support to "
            "core/geometry.py before authoring one at an angle.")
    return round(turns) % 2 == 0


def wall_segments():
    """Wall centrelines as ((x0, y0), (x1, y1)) pairs.

    The zero-thickness skeleton of the floorplan. Useful for framing and for
    reading the layout; obstacle tests want wall_rects() instead, which has the
    thickness the robot actually collides with.
    """
    segments = []
    for cx, cy, length, yaw_deg in WALLS:
        half = 0.5 * length
        if _along_x(cx, cy, yaw_deg):
            segments.append(((cx - half, cy), (cx + half, cy)))
        else:
            segments.append(((cx, cy - half), (cx, cy + half)))
    return segments


def wall_rects():
    """Walls as axis-aligned 2D boxes, (min_x, min_y, max_x, max_y).

    build_scene.py gives each wall scale (length, WALL_THICKNESS, WALL_HEIGHT)
    and rotates it about z, so in plan view a wall is a rectangle `length` by
    WALL_THICKNESS.
    """
    rects = []
    for cx, cy, length, yaw_deg in WALLS:
        along_x = _along_x(cx, cy, yaw_deg)
        half_x = 0.5 * (length if along_x else WALL_THICKNESS)
        half_y = 0.5 * (WALL_THICKNESS if along_x else length)
        rects.append((cx - half_x, cy - half_y, cx + half_x, cy + half_y))
    return rects


def bounds():
    """Room extent as (min_x, min_y, max_x, max_y), from the wall centrelines.

    Centrelines, not rects: the perimeter walls straddle the boundary, so the
    rects would report x in [-0.075, 12.075] and the occupancy grid would carry
    a ring of cells outside the building.

    recorder.py:82 hardcodes the same numbers as FALLBACK_SCENE_BOUNDS. That
    duplication is deliberate for now — recorder.py is imported only after
    SimulationApp exists and is not on the benchmark path, so it is left alone.
    """
    xs = [p[0] for seg in wall_segments() for p in seg]
    ys = [p[1] for seg in wall_segments() for p in seg]
    return (min(xs), min(ys), max(xs), max(ys))
