#!/usr/bin/env python
"""
scripts/plot_kinematic.py

**Kinematic rollouts, drawn.** The fast loop's picture.

`core/tests/kinematic.py` rolls a controller round a scenario in ~0.3 s where
physics takes ~21 s headless and ~240 s with an mp4 — 35x and 400x. That makes
it the right tool for triage: does this node fire at all, does the tree drive
into a wall, is this A/B pair even comparable. What it did not have was a way to
LOOK at the result, and a trajectory you cannot see is a number you have to
trust.

This draws one figure per scenario, with every controller you name overlaid on
the same axes — which is the A vs B view: arm A's node and arm B's node, same
world, same start, same goal, two paths.

    out/kinematic/<scenario>.png

⚠️ **THIS IS NOT A BENCHMARK PATH AND NO NUMBER FROM IT MAY BE REPORTED.**
`kinematic.py` says it first and means it: the robot is a point that turns and
translates exactly as commanded, walls are impassable rather than physical, and
**the frozen locomotion policy is absent entirely** — the thing that actually
decides whether a Twist becomes motion. Nothing here can see a fall, a gait
failure or a contact dynamic. Use it to decide what is worth simulating; take
every reported number from `scripts/run_episode.py` inside the container.

SIM-FREE
--------
No isaacsim, omni, pxr, torch or carb. It runs on a laptop with no GPU, and it
runs on the output of a machine that has one.

Examples
--------
    P=~/miniforge3/envs/env_isaacsim/bin/python

    # every dev scenario, one reference controller
    $P scripts/plot_kinematic.py --controller reactive_avoid

    # the A vs B view: the limit_speed_dwa pair, overlaid
    $P scripts/plot_kinematic.py \
         --controller node:gen_a_fac8c7b94fe6b1c6 \
         --controller node:gen_b_c2c95bcb965e342e

    # one scenario, three controllers, somewhere else
    $P scripts/plot_kinematic.py --scenario blind_corner \
         --controller goal_seeker --controller reactive_avoid \
         --out /tmp/look

    # what can I name?
    $P -c "import controllers; print(controllers.available());
           print(controllers.library_nodes())"
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _scenario_id(value):
    """Validate before anything heavy imports, the same discipline the
    simulator scripts use — there is no SimulationApp here, but a typo should
    still not survive until after the rollouts."""
    from core.scenario import DEV_SCENARIOS
    known = [s.id for s in DEV_SCENARIOS]
    if value not in known:
        raise argparse.ArgumentTypeError(
            f"unknown dev scenario {value!r}. Known: {known}")
    return value


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Draw kinematic rollouts. NOT a benchmark path.")
    p.add_argument("--scenario", action="append", dest="scenarios",
                   type=_scenario_id, metavar="ID",
                   help="repeatable; default is every DEV scenario. The "
                        "held-out split is deliberately unreachable here.")
    p.add_argument("--controller", action="append", dest="controllers",
                   metavar="NAME", required=True,
                   help="repeatable. Any name run_episode.py accepts: a "
                        "registry name, node:<node_id>, composed:<id>. Two or "
                        "more are overlaid on the same axes.")
    p.add_argument("--out", default=None, metavar="DIR",
                   help="output directory; default out/kinematic/. Relative "
                        "paths resolve under Repo/ regardless of where you "
                        "launched from.")
    p.add_argument("--estimate", action="store_true",
                   help="also draw the pose ESTIMATE, dashed. Only says "
                        "anything on a scenario that drifts.")
    p.add_argument("--dpi", type=int, default=DPI)
    return p.parse_args(argv)


# Everything above this line is code; below it is data, not code.
# ---------------------------------------------------------------------------

# One colour per controller, in the order they were named on the command line.
# Chosen to stay distinguishable in greyscale print and to keep red free: red
# means CONTACT here, and a controller that happened to be drawn red would read
# as a collision the whole way along.
TRACK_COLOURS = ("#1f77b4", "#2ca02c", "#9467bd", "#8c564b", "#17becf")

CONTACT_COLOUR = "#d62728"      # red, and reserved for it
WALL_COLOUR = "#3f3f3f"
INFLATED_COLOUR = "#c8c8c8"     # where the robot CENTRE may not go
OPTIMAL_COLOUR = "#8a8a8a"      # the taut path: the SPL denominator
GOAL_COLOUR = "#e6a700"

FIGSIZE = (11.0, 7.2)           # lab42 is 12 x 8 m; this keeps it near 1:1
DPI = 140

# A contact marker every step would be a solid red line along the wall the
# robot is scraping. One in this many is enough to see WHERE it happened.
CONTACT_STRIDE = 5


def _draw_world(ax, world):
    """The room: inflated no-go region underneath, true wall boxes on top."""
    from core.geometry import occupancy_grid

    grid = occupancy_grid(world=world)
    min_x, min_y, max_x, max_y = world.bounds
    # imshow rather than pcolormesh: the grid is already row-major with y
    # first, which is the orientation Grid's docstring promises.
    ax.imshow(grid.occupied, origin="lower", cmap="Greys", vmin=0, vmax=3.0,
              extent=(grid.origin[0],
                      grid.origin[0] + grid.shape[1] * grid.resolution,
                      grid.origin[1],
                      grid.origin[1] + grid.shape[0] * grid.resolution),
              interpolation="nearest", zorder=0)

    from matplotlib.patches import Rectangle
    for (x0, y0, x1, y1) in world.rects:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               facecolor=WALL_COLOUR, edgecolor="none",
                               zorder=1))
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)
    ax.set_aspect("equal")


def _draw_scenario(ax, scenario, world):
    """Start, goal, the detection radius when the goal is withheld, and the
    taut path the SPL denominator is measured along."""
    from matplotlib.patches import Circle
    from core.geometry import shortest_path

    sx, sy, _yaw = scenario.start
    gx, gy = scenario.goal

    path = shortest_path((sx, sy), (gx, gy), world=world)
    if len(path):
        ax.plot([p[0] for p in path], [p[1] for p in path],
                color=OPTIMAL_COLOUR, linewidth=1.4, linestyle=(0, (6, 4)),
                zorder=2, label="taut optimal (SPL denominator)")

    ax.plot([sx], [sy], marker="o", markersize=9, color="black",
            markerfacecolor="white", zorder=6, label="start")
    ax.plot([gx], [gy], marker="*", markersize=18, color=GOAL_COLOUR,
            markeredgecolor="black", markeredgewidth=0.6, zorder=6,
            label="goal")

    # Only meaningful on a search scenario: until the robot is inside this
    # circle with line of sight, Observation.goal is None. Drawing it explains
    # a trajectory that wanders before turning decisive.
    if not scenario.goal_known:
        ax.add_patch(Circle((gx, gy), scenario.detect_radius, fill=False,
                            edgecolor=GOAL_COLOUR, linestyle=":",
                            linewidth=1.2, zorder=5))


def _draw_track(ax, trace, label, colour, draw_estimate):
    """One controller's path, with contacts called out."""
    x = trace.column("x")
    y = trace.column("y")
    ax.plot(x, y, color=colour, linewidth=2.0, zorder=4, label=label)

    contact = trace.column("contact").astype(bool)
    if contact.any():
        ax.plot(x[contact][::CONTACT_STRIDE], y[contact][::CONTACT_STRIDE],
                linestyle="none", marker="x", markersize=6,
                color=CONTACT_COLOUR, zorder=7)

    # Where it stopped. On a timeout this is the interesting end of the line.
    ax.plot([x[-1]], [y[-1]], marker="s", markersize=6, color=colour,
            markeredgecolor="black", markeredgewidth=0.5, zorder=6)

    if draw_estimate:
        ax.plot(trace.column("est_x"), trace.column("est_y"), color=colour,
                linewidth=1.1, linestyle=(0, (2, 3)), alpha=0.8, zorder=3)


def _summary(scored):
    """The one-line verdict that goes under the title and into stdout."""
    reached = "REACHED" if scored["goal_reached"] else "failed"
    spl = scored.get("spl")
    return (f"{reached}  t={scored.get('time_to_goal')}  "
            f"spl={spl:.3f}  collisions={scored.get('collisions')}  "
            f"min_clear={scored.get('min_clearance'):.2f}m"
            if spl is not None else reached)


def main(argv=None):
    args = _parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")           # no display on the container, and none needed
    import matplotlib.pyplot as plt

    import controllers
    from core.geometry import LAB42
    from core.metrics import score
    from core.scenario import DEV_SCENARIOS
    from core.tests.kinematic import rollout

    out_dir = Path(args.out) if args.out else REPO_ROOT / "out" / "kinematic"
    if not out_dir.is_absolute():
        out_dir = REPO_ROOT / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    wanted = args.scenarios or [s.id for s in DEV_SCENARIOS]
    scenarios = [s for s in DEV_SCENARIOS if s.id in wanted]

    print(f"[plot_kin] ⚠ kinematic rollouts — NOT a benchmark path, "
          f"no number here is reportable")
    print(f"[plot_kin] {len(scenarios)} scenario(s) x "
          f"{len(args.controllers)} controller(s) -> {out_dir}")

    written = []
    for scenario in scenarios:
        fig, ax = plt.subplots(figsize=FIGSIZE)
        _draw_world(ax, LAB42)
        _draw_scenario(ax, scenario, LAB42)

        lines = []
        for i, name in enumerate(args.controllers):
            colour = TRACK_COLOURS[i % len(TRACK_COLOURS)]
            try:
                controller = controllers.make(name)
            except KeyError as exc:
                # A bad controller name should not cost the other figures.
                print(f"[plot_kin]   {scenario.id}: {name}: {exc}")
                continue
            trace = rollout(scenario, controller)
            scored = score(trace, scenario)
            summary = _summary(scored)
            _draw_track(ax, trace, f"{name} — {summary}", colour,
                        args.estimate)
            lines.append(f"{name}: {summary}")
            print(f"[plot_kin]   {scenario.id:18} {name:34} {summary}")

        ax.set_title(f"{scenario.id}  ({scenario.family}, split={scenario.split}, "
                     f"goal_known={scenario.goal_known})\n"
                     f"kinematic rollout — not a benchmark path",
                     fontsize=10)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.legend(loc="upper left", fontsize=7, framealpha=0.9)

        path = out_dir / f"{scenario.id}.png"
        fig.tight_layout()
        fig.savefig(path, dpi=args.dpi)
        plt.close(fig)
        written.append(path)

    for path in written:
        print(f"[plot_kin] wrote {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
