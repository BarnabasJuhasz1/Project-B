"""
demo/maze2.py

THE SECOND DEMO MAZE -- Mindfire Exp F (review item E2). Hand-authored, like
the first (`demo/world.py`), and that is a declared fact about it: no number
from it may be pooled with anything from `core.worldgen`, nor with the first
maze's numbers. See demo/EXPF.md and prereg/expF.md.

WHAT IS HELD FIXED, AND WHY
---------------------------
The bounds (14 x 10 m), HOME, GOAL and the spawn yaw (0.6 rad) are the first
maze's, imported rather than re-typed. Every one of the 16 controllers sizes its
map from `scenario.start` / `scenario.goal` plus a margin, so keeping the two
endpoints identical means every controller builds the same grid on both mazes,
and the ONLY thing that differs between Exp 3 and Exp F is the wall layout. The
door still shuts between legs 1 and 2, while the robot is at home, by the same
code path (`demo/rollout.py`, `demo/record_isaac.py`).

THE TOPOLOGY, AND HOW IT DIFFERS FROM THE FIRST MAZE
---------------------------------------------------
Two full-width dividing walls cut the building into three bands.

    top band     y 6.75 .. 9.85   GOAL at the far right end
    ---- upper divider, y 6.60 -- two openings: WEST (x 1.70..3.10) and the
                                   DOOR (x 8.70..10.30) on the home->goal line
    middle band  y 3.55 .. 6.60
    ---- lower divider, y 3.40 -- ONE opening, x 4.00..5.50, shared by both routes
    bottom band  y 0.15 .. 3.40   HOME at the far left end

Short route: home -> lower opening -> DOOR -> goal. With the door shut the robot
must stay in the middle band, turn its back on the goal, run ~6 m west to the
WEST opening, and come back east along the whole top band.

First maze: the routes split AT HOME (one goes through a central room, the
other round its outside), and shutting the door turns the room into a
cul-de-sac the robot has to back out of. Here the routes SHARE a prefix (the
single lower opening), split in the middle band, and the blocked route is
not a dead-end room but the near end of a corridor whose other exit is behind
the robot. Same premise -- the shut door sits on the straight line home->goal,
so a controller steering on goal bearing is pulled back to it -- on a different
graph.

WHAT verify() GUARANTEES, measured by core.geometry
--------------------------------------------------
The first maze's checks, unchanged in meaning, plus what the first maze checked
by eye or not at all:

1-4. short route open; long route shut; detour > 3 m; start and goal free --
     exactly `demo.world.verify()`.
5.   `ok_bypass_navigable`: the long route exists with 0.30 m of margin either
     side ON TOP of ROBOT_RADIUS, i.e. >= 0.60 m of free width for the robot
     centre everywhere along it -- the first maze's 0.60 m criterion, measured
     over the whole route instead of one hand-picked window (the first maze's
     `bypass_bottleneck()` scans x 0.2..3.4 only; that window does not exist
     here).
6.   `ok_door_on_short_route`: a shortest open-door route passes through the
     door rect, so the block really cuts it.
7.   `ok_spawn_free_all_reps`: the Go2's CIRCUMSCRIBED radius (0.383 m, body
     0.70 x 0.31 m) clears every wall at HOME plus every Exp 3 start offset.

    $ python demo/maze2.py           # the checks
    $ python demo/maze2.py --plot    # demo/out/expF/maze2.png

SIM-FREE.
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np

from core.geometry import (World, ROBOT_RADIUS, geodesic_distance,
                           geodesic_field, occupancy_grid, clearance)
from demo.world import BOUNDS, T, HOME, GOAL

# ---- below this line is data, not code.

NAME = "maze2"
SPAWN_YAW = 0.6                   # the first maze's, see demo/record_isaac.py
GO2_CIRCUMSCRIBED = 0.383         # m; 0.70 x 0.31 m body (CLAUDE.md, physics-transfer)
NAVIGABLE_MARGIN = 0.30           # m either side on top of ROBOT_RADIUS (= 0.60 m width)

LOWER_Y = 3.40                    # lower divider, y LOWER_Y .. LOWER_Y + T
UPPER_Y = 6.60                    # upper divider, y UPPER_Y .. UPPER_Y + T

# The upper divider's EAST opening, on the home->goal line. The one rect that
# changes, so "what the door does" has exactly one definition -- as in world.py.
DOOR = (8.70, UPPER_Y, 10.30, UPPER_Y + T)

_WALLS = [
    # ---- boundary (as the first maze)
    (0.0, 0.0, 14.0, T), (0.0, 10.0 - T, 14.0, 10.0),
    (0.0, 0.0, T, 10.0), (14.0 - T, 0.0, 14.0, 10.0),

    # ---- lower divider: ONE opening, x 4.00..5.50, shared by both routes.
    (T, LOWER_Y, 4.00, LOWER_Y + T),
    (5.50, LOWER_Y, 14.0 - T, LOWER_Y + T),

    # ---- upper divider: WEST opening x 1.70..3.10 (the detour), then the wall,
    # then x 8.70..10.30 -> DOOR, then the wall to the east boundary. East of
    # the door the middle band is a dead-end pocket.
    (T, UPPER_Y, 1.70, UPPER_Y + T),
    (3.10, UPPER_Y, 8.70, UPPER_Y + T),
    (10.30, UPPER_Y, 14.0 - T, UPPER_Y + T),

    # ---- partial walls. They carry none of the topology (verify() is measured
    # with them in): one stub in the middle band and one in the top band, so
    # the long route winds a little rather than being two straight runs.
    (6.40, UPPER_Y - 1.40, 6.40 + T, UPPER_Y),        # hangs from the upper divider
    (5.20, 8.60, 5.20 + T, 10.0 - T),                 # hangs from the north wall

    # ---- clutter, so the recording reads as a building rather than a diagram.
    # ⚠️ PLACED BY verify(), NOT BY EYE. The first draft had a block at
    # (3.40, 7.80) that left a 1.16 m gap to the top-band stub -- a 0.54 m slot
    # for the robot centre, under the 0.60 m rule -- and one at (3.30, 4.50)
    # beside the lower opening. Both were moved before any controller ran.
    # ⚠️ CALIBRATED ON THE REFERENCE CONTROLLERS ONLY, as the first maze was
    # (demo/README.md, "Calibration decisions"). The bottom-band block first sat
    # at (2.30, 1.80), ON the home->goal line: the frozen base was deflected
    # east along the lower divider and missed the opening, failing LEG 0 with
    # the door open (0/4) -- so the door was not the discriminator. Moved off
    # the line, the base does legs 0-1 (26.5 s, 57.7 s) and then fails at the
    # shut door, 2/4, the same pattern as on the first maze (27.0 s, 28.7 s,
    # 2/4); reactive_avoid scores 2/4 (first maze 1/4). None of the 16 arm
    # controllers was run on any draft of this maze.
    (0.90, 2.40, 1.55, 3.05),                         # bottom band, OFF the line
    (7.60, 1.20, 8.30, 1.90),
    (11.20, 1.60, 11.90, 2.30),
    (7.90, 4.00, 8.55, 4.65),                         # middle band
    (11.60, 4.40, 12.30, 5.10),                       # the dead-end pocket
    (9.60, 8.80, 10.25, 9.45),                        # top band
    (7.70, 7.40, 8.40, 8.05),
    (12.40, 7.20, 13.05, 7.75),
]

OPEN_WORLD = World(rects=tuple(tuple(map(float, r)) for r in _WALLS),
                   bounds=BOUNDS, name="demo-maze2-open")
BLOCKED_WORLD = World(rects=tuple(tuple(map(float, r)) for r in _WALLS + [DOOR]),
                      bounds=BOUNDS, name="demo-maze2-blocked")


# ---- code.

def route_margin(world, a=HOME, b=GOAL, hi=0.6, tol=0.005):
    """The largest extra margin m such that a route a -> b exists for a disc of
    radius ROBOT_RADIUS + m. Free width for the robot CENTRE along the tightest
    point of the best route is then >= 2 m.

    ⚠️ The general form of the first maze's `bypass_bottleneck()`: that one
    scans a hand-picked window and would miss a slot elsewhere. This one is a
    bisection over inflated occupancy grids, so it sees the whole route.
    """
    def ok(m):
        g = occupancy_grid(inflate=ROBOT_RADIUS + m, world=world)
        return geodesic_distance(a, b, grid=g, world=world) < float("inf")
    if not ok(0.0):
        return 0.0
    lo = 0.0
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def door_on_shortest_route(open_world=None, door=None, slack=0.15):
    """True if a shortest open-world route home -> goal passes through `door`:
    min over the door's cells of d(home, c) + d(c, goal) is within `slack` m of
    the open-world distance (one grid cell is 0.05 m). Measured on the robot-
    centre grid, so it is about routes the robot can take."""
    open_world = OPEN_WORLD if open_world is None else open_world
    door = DOOR if door is None else door
    grid = occupancy_grid(world=open_world)
    fh = geodesic_field(HOME, grid, world=open_world)
    fg = geodesic_field(GOAL, grid, world=open_world)
    total = fh + fg
    ys = grid.origin[1] + (np.arange(grid.occupied.shape[0]) + 0.5) * grid.resolution
    xs = grid.origin[0] + (np.arange(grid.occupied.shape[1]) + 0.5) * grid.resolution
    x0, y0, x1, y1 = door
    iy = (ys >= y0) & (ys <= y1)
    ix = (xs >= x0) & (xs <= x1)
    through = float(np.min(total[np.ix_(iy, ix)])) if iy.any() and ix.any() else float("inf")
    best = geodesic_distance(HOME, GOAL, grid=grid, world=open_world)
    return through, best, through - best <= slack


def spawn_clearances(reps=None):
    """Clearance of every Exp 3 spawn point to the nearest wall, metres."""
    from demo import exp3
    reps = exp3.REP_CONDITIONS if reps is None else reps
    out = {}
    for rep, (_, dx, dy, _) in sorted(reps.items()):
        out[rep] = float(clearance((HOME[0] + dx, HOME[1] + dy), OPEN_WORLD))
    return out


def verify(open_world=None, blocked_world=None):
    """The guarantees, measured by core.geometry."""
    open_world = OPEN_WORLD if open_world is None else open_world
    blocked_world = BLOCKED_WORLD if blocked_world is None else blocked_world
    out = {}
    for name, w in (("open", open_world), ("blocked", blocked_world)):
        out[f"{name}_home_to_goal"] = geodesic_distance(HOME, GOAL, world=w)
        out[f"{name}_goal_to_home"] = geodesic_distance(GOAL, HOME, world=w)
    out["home_clear"] = float(clearance(HOME, open_world)) - ROBOT_RADIUS
    out["goal_clear"] = float(clearance(GOAL, open_world)) - ROBOT_RADIUS
    o, b = out["open_home_to_goal"], out["blocked_home_to_goal"]
    out["detour_cost_m"] = b - o
    out["open_route_margin_m"] = round(route_margin(open_world), 3)
    out["blocked_route_margin_m"] = round(route_margin(blocked_world), 3)
    out["blocked_min_width_m"] = round(2 * out["blocked_route_margin_m"], 3)
    through, best, on = door_on_shortest_route(open_world)
    out["via_door_m"] = round(through, 3)
    sp = spawn_clearances()
    out["spawn_clearance_min_m"] = round(min(sp.values()), 3)
    out["ok_open_solvable"] = o < float("inf")
    out["ok_blocked_solvable"] = b < float("inf")
    out["ok_door_matters"] = (b - o) > 3.0
    out["ok_door_on_short_route"] = on
    out["ok_bypass_navigable"] = out["blocked_route_margin_m"] >= NAVIGABLE_MARGIN
    out["ok_start_free"] = out["home_clear"] > 0.05
    out["ok_goal_free"] = out["goal_clear"] > 0.05
    out["ok_spawn_free_all_reps"] = min(sp.values()) > GO2_CIRCUMSCRIBED + 0.05
    out["all_ok"] = all(v for k, v in out.items() if k.startswith("ok_"))
    return out


def plot(path, worlds=None, door=None, title_prefix="maze2"):
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    worlds = worlds or (OPEN_WORLD, BLOCKED_WORLD)
    door = door or DOOR
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    for ax, title, w in zip(axes, ("legs 0-1 - door open", "legs 2-3 - door shut"),
                            worlds):
        for (x0, y0, x1, y1) in w.rects:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fc="0.35", ec="0.2"))
        ax.add_patch(Rectangle((door[0], door[1]), door[2] - door[0],
                               door[3] - door[1],
                               fc="crimson" if w is worlds[1] else "none",
                               ec="crimson", ls="-" if w is worlds[1] else "--",
                               zorder=5))
        # the geodesic route, so the picture shows what verify() measured
        grid = occupancy_grid(world=w)
        field = geodesic_field(GOAL, grid, world=w)
        p = np.array(HOME, float)
        pts = [p.copy()]
        for _ in range(2000):
            ix, iy = grid.world_to_cell(p)
            best, nxt = field[iy, ix], None
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    jx, jy = ix + dx, iy + dy
                    if 0 <= jy < field.shape[0] and 0 <= jx < field.shape[1] \
                            and field[jy, jx] < best:
                        best, nxt = field[jy, jx], (jx, jy)
            if nxt is None:
                break
            p = np.array(grid.cell_to_world(*nxt), float)
            pts.append(p)
        pts = np.array(pts)
        d = geodesic_distance(HOME, GOAL, world=w)
        ax.plot(pts[:, 0], pts[:, 1], "-", color="tab:orange", lw=2,
                label=f"shortest route {d:.2f} m")
        ax.plot(*HOME, "o", color="tab:blue", ms=12, label="home")
        ax.plot(*GOAL, "*", color="tab:green", ms=20, label="goal")
        ax.set_xlim(0, 14); ax.set_ylim(0, 10); ax.set_aspect("equal")
        ax.set_title(f"{title_prefix}: {title}"); ax.legend(loc="lower right")
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout(); fig.savefig(path, dpi=110)
    return path


if __name__ == "__main__":
    r = verify()
    for k, v in r.items():
        print(f"{k:26} {v}")
    if "--plot" in sys.argv:
        out = pathlib.Path(__file__).resolve().parent / "out" / "expF" / "maze2.png"
        print(f"-> {plot(out)}")
