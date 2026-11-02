"""
demo/world.py

THE DEMO WORLD. Hand-authored, and that is a declared fact about it rather than
a detail: it was built so the demo's premise is guaranteed to hold. No number
from it may be pooled with anything from `core.worldgen`'s seeded distribution.
See demo/README.md.

THE TOPOLOGY, AND WHY IT IS THIS ONE
------------------------------------
A central room sits across the direct line from home to the goal. The short
route goes IN through its near opening and OUT through its far one. `DOOR`
closes the far one from lap 2 onward, which turns the room into a cul-de-sac: to
reach the goal the robot must recognise the block, come back out the way it came
in, and take the long way round the outside.

That is chosen deliberately over a simple short-way/long-way fork:

* The shut door sits almost exactly on the straight line home -> goal, so a
  controller that steers on goal bearing is pulled back to it again and again.
  That is the base's documented failure mode -- it re-enters an unexitable
  detour 4 to 7 times per episode (`out/evidence/base-failure-mode/`) -- and it
  is what the demo has to make visible.
* Backing out of a room is legible on video in a way "took the other corridor"
  is not.

WHAT verify() GUARANTEES, measured by core.geometry and never asserted in prose
-----------------------------------------------------------------------------
1. A short route exists with the door open.
2. A longer route exists with the door shut -- so lap 2 is possible and a
   failure is about the controller, not the world.
3. The detour is real (> 3 m), so success and failure are not a coin toss on
   the time budget.
4. Both routes fit the robot: `occupancy_grid` inflates by ROBOT_RADIUS, so a
   route that does not fit returns inf and check 2 fails.

The clutter and staggered partial walls carry none of the topology. They are
there so the recording reads as a room rather than a diagram.

    $ python demo/world.py          # prints the checks
    $ python demo/world.py --plot   # writes demo/out/world.png

SIM-FREE.
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from core.geometry import World, ROBOT_RADIUS, geodesic_distance, clearance

# ---- below this line is data, not code.

BOUNDS = (0.0, 0.0, 14.0, 10.0)
T = 0.15

HOME = (1.10, 1.10)
GOAL = (12.90, 8.90)

# The central room's far exit. DOOR fills it from lap 2 on. Declared as the one
# rect that changes, so "what the door does" has exactly one definition.
DOOR = (9.20, 6.75, 11.00, 6.75 + T)

_WALLS = [
    # ---- boundary
    (0.0, 0.0, 14.0, T), (0.0, 10.0 - T, 14.0, 10.0),
    (0.0, 0.0, T, 10.0), (14.0 - T, 0.0, 14.0, 10.0),

    # ---- the central room, x 3.0..11.0, y 3.10..6.90.
    # NEAR opening in the bottom wall, x 3.80..4.80 -- how the robot gets in.
    (3.00, 3.10, 3.60, 3.10 + T),
    (5.20, 3.10, 11.00, 3.10 + T),
    (3.00, 3.10, 3.00 + T, 6.90),             # left wall
    (10.85, 3.10, 11.00, 6.30),               # right wall, stops below the mouth
    (3.00, 6.75, 9.20, 6.75 + T),             # top wall, left of the far opening
    # x in [9.20, 11.00] of the top wall is the FAR OPENING -> DOOR closes it.

    # ---- blocks the bottom-right bypass, so the detour cannot simply cut
    # round the near end of the room.
    (11.00, 3.10, 14.0 - T, 3.10 + T),

    # ---- the left bypass, made to zigzag. This is what gives the long route
    # its length: without it the detour costs under 2 m and the demo turns on
    # the time budget rather than on the geometry.
    # ⚠️ THE GAPS ARE 1.40 m AND THAT IS LOAD-BEARING. At 0.90 m -- the first
    # version -- the robot CENTRE had a 0.30 m slot with 0.15 m either side,
    # and EVERY failing controller in BOTH arms stalled at one of these two
    # points having already turned into the bypass. The demo was measuring
    # slot-threading, not rerouting. `verify()` missed it because
    # geodesic_distance inflates by ROBOT_RADIUS and calls any positive-width
    # centre path solvable. `ok_bypass_navigable` is the check that now
    # refuses it.
    (T, 5.30, 1.60, 5.30 + T),
    (1.55, 7.30, 3.00, 7.30 + T),

    # ---- the top crossing, also made to wind.
    (5.90, 8.40, 5.90 + T, 10.0 - T),
    (7.90, 8.40, 10.60, 8.40 + T),

    # ---- clutter. Carries none of the topology; it is here so the scene reads
    # as a room rather than a diagram.
    (1.70, 2.40, 2.45, 3.00),
    (5.10, 4.30, 5.80, 5.05),                 # inside the central room
    (7.30, 4.00, 7.95, 4.65),                 # inside the central room
    (6.40, 1.30, 7.10, 2.05),
    (10.40, 1.35, 11.10, 2.10),
    (12.30, 1.55, 12.95, 2.20),
    (3.90, 8.85, 4.55, 9.50),
    (12.40, 5.20, 13.10, 5.90),
    (1.60, 8.20, 2.25, 8.85),
]

OPEN_WORLD = World(rects=tuple(tuple(map(float, r)) for r in _WALLS),
                   bounds=BOUNDS, name="demo-patrol-open")
BLOCKED_WORLD = World(rects=tuple(tuple(map(float, r)) for r in _WALLS + [DOOR]),
                      bounds=BOUNDS, name="demo-patrol-blocked")


def bypass_bottleneck(samples=180):
    """The tightest free width available to the robot CENTRE anywhere along the
    left bypass corridor, with the door shut.

    ⚠️ This exists because `geodesic_distance` cannot answer it: it reports a
    route through a one-cell-wide slot as perfectly solvable, and a reactive
    controller with a 0.65 m body cannot drive one.
    """
    import numpy as np
    worst = (float("inf"), None)
    for y in np.linspace(4.0, 9.0, samples):
        best, start = 0.0, None
        for x in np.linspace(0.2, 3.4, samples):
            if float(clearance((x, y), BLOCKED_WORLD)) - ROBOT_RADIUS > 0:
                start = x if start is None else start
                best = max(best, x - start)
            else:
                start = None
        if best < worst[0]:
            worst = (best, round(float(y), 2))
    return worst


def verify():
    """The guarantees, measured by core.geometry."""
    out = {}
    for name, w in (("open", OPEN_WORLD), ("blocked", BLOCKED_WORLD)):
        out[f"{name}_home_to_goal"] = geodesic_distance(HOME, GOAL, world=w)
        out[f"{name}_goal_to_home"] = geodesic_distance(GOAL, HOME, world=w)
    out["home_clear"] = clearance(HOME, OPEN_WORLD) - ROBOT_RADIUS
    out["goal_clear"] = clearance(GOAL, OPEN_WORLD) - ROBOT_RADIUS
    o, b = out["open_home_to_goal"], out["blocked_home_to_goal"]
    out["detour_cost_m"] = b - o
    width, at_y = bypass_bottleneck()
    out["bypass_min_width_m"] = round(width, 3)
    out["bypass_tightest_at_y"] = at_y
    # 0.60 m of free width for the CENTRE, i.e. 0.30 m of margin either side on
    # top of the robot's own radius. Below this the demo measures threading.
    out["ok_bypass_navigable"] = width >= 0.60
    out["ok_open_solvable"] = o < float("inf")
    out["ok_blocked_solvable"] = b < float("inf")
    out["ok_door_matters"] = (b - o) > 3.0
    out["ok_start_free"] = out["home_clear"] > 0.05
    out["ok_goal_free"] = out["goal_clear"] > 0.05
    out["all_ok"] = all(v for k, v in out.items() if k.startswith("ok_"))
    return out


if __name__ == "__main__":
    r = verify()
    for k, v in r.items():
        print(f"{k:24} {v}")
    if "--plot" in sys.argv:
        import matplotlib; matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))
        for ax, (title, w) in zip(axes, (("lap 1 - door open", OPEN_WORLD),
                                         ("lap 2 - door shut", BLOCKED_WORLD))):
            for (x0, y0, x1, y1) in w.rects:
                ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                                       fc="0.35", ec="0.2"))
            ax.add_patch(Rectangle((DOOR[0], DOOR[1]), DOOR[2] - DOOR[0],
                                   DOOR[3] - DOOR[1], fc="crimson", ec="crimson",
                                   zorder=5))
            ax.plot(*HOME, "o", color="tab:blue", ms=12, label="home")
            ax.plot(*GOAL, "*", color="tab:green", ms=20, label="goal")
            ax.set_xlim(0, 14); ax.set_ylim(0, 10); ax.set_aspect("equal")
            ax.set_title(title); ax.legend(loc="upper left")
        out_dir = pathlib.Path(__file__).resolve().parent / "out"
        out_dir.mkdir(exist_ok=True)
        fig.tight_layout(); fig.savefig(out_dir / "world.png", dpi=110)
        print(f"-> {out_dir / 'world.png'}")
