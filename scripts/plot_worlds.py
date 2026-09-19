#!/usr/bin/env python
"""
scripts/plot_worlds.py

**Generated worlds, drawn.** Chunk 45's eyes.

`core/worldgen.py` reports a tier's difficulty as numbers, and numbers are the
right thing to assert on — but they cannot tell you that a "maze" is really one
big room with furniture, or that every layout has its doorway in the same place.
This draws a contact sheet: one row per tier, one column per seed, with the
start, the goal and the taut route on each.

    out/worlds/<tier>.png          one tier, several seeds
    out/worlds/contact.png         every tier, side by side

⚠️ SIM-FREE, and NOT a benchmark path — it draws geometry, scores nothing.

Examples
--------
    P=~/miniforge3/envs/env_isaacsim/bin/python

    $P scripts/plot_worlds.py                       # all three tiers, 6 seeds
    $P scripts/plot_worlds.py --tier maze --n 12
    $P scripts/plot_worlds.py --seeds 1,2,3 --route  # + the geodesic route
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _parse_args(argv=None):
    p = argparse.ArgumentParser(description="Draw generated worlds. Sim-free.")
    p.add_argument("--tier", action="append", dest="tiers",
                   help="repeatable; default is every tier")
    p.add_argument("--n", type=int, default=6, help="how many seeds per tier")
    p.add_argument("--seeds", help="explicit comma-separated seeds; overrides --n")
    p.add_argument("--start-seed", type=int, default=1000)
    p.add_argument("--route", action="store_true",
                   help="overlay the taut path. ⚠ SLOW — the visibility graph is "
                        "quadratic in obstacle count and costs ~40 s on a maze "
                        "world. See the chunk 45 outcome in the epic.")
    p.add_argument("--out", default=None)
    p.add_argument("--dpi", type=int, default=DPI)
    return p.parse_args(argv)


# Everything above this line is code; below it is data, not code.
# ---------------------------------------------------------------------------

WALL_COLOUR = "#3f3f3f"
FREE_COLOUR = "#ffffff"
BLOCKED_COLOUR = "#d9d9d9"      # inflated: where the robot CENTRE may not go
START_COLOUR = "#1f77b4"
GOAL_COLOUR = "#e6a700"
ROUTE_COLOUR = "#8a8a8a"

PANEL = 3.1                     # inches per panel; lab42 is 12 x 8 m, so 3:2
DPI = 130


def _draw(ax, g, route=False):
    from matplotlib.patches import Rectangle
    from core.geometry import occupancy_grid

    grid = occupancy_grid(world=g.world)
    ax.imshow(grid.occupied, origin="lower", cmap="Greys", vmin=0, vmax=3.0,
              extent=(grid.origin[0],
                      grid.origin[0] + grid.shape[1] * grid.resolution,
                      grid.origin[1],
                      grid.origin[1] + grid.shape[0] * grid.resolution),
              interpolation="nearest", zorder=0)
    for (x0, y0, x1, y1) in g.world.rects:
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               facecolor=WALL_COLOUR, edgecolor="none", zorder=1))

    if route:
        from core.geometry import shortest_path
        poly = shortest_path(g.start, g.goal, world=g.world)
        if len(poly):
            ax.plot([p[0] for p in poly], [p[1] for p in poly],
                    color=ROUTE_COLOUR, linewidth=1.4, linestyle=(0, (5, 3)),
                    zorder=2)

    ax.plot([g.start[0]], [g.start[1]], marker="o", markersize=7,
            color=START_COLOUR, markeredgecolor="black", markeredgewidth=0.5,
            zorder=4)
    ax.plot([g.goal[0]], [g.goal[1]], marker="*", markersize=14,
            color=GOAL_COLOUR, markeredgecolor="black", markeredgewidth=0.5,
            zorder=4)

    d = g.difficulty
    ax.set_title(f"{g.tier}/{g.seed}  tort={d.tortuosity:.2f} "
                 f"free={d.free_fraction:.2f}\nbottleneck={d.min_corridor:.2f}m "
                 f"obs={d.n_obstacles}", fontsize=7)
    ax.set_xticks([]); ax.set_yticks([])
    ax.set_aspect("equal")


def main(argv=None):
    args = _parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    from core.worldgen import TIERS, generate, clear_geometry_caches

    tiers = args.tiers or list(TIERS)
    if args.seeds:
        seeds = [int(s) for s in args.seeds.split(",")]
    else:
        seeds = list(range(args.start_seed, args.start_seed + args.n))

    out_dir = Path(args.out) if args.out else REPO_ROOT / "out" / "worlds"
    if not out_dir.is_absolute():
        out_dir = REPO_ROOT / out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    print(f"[plot_worlds] {len(tiers)} tier(s) x {len(seeds)} seed(s) -> {out_dir}")

    def sheet(rows, path, ncol):
        """One figure. `rows` is [(label, [GeneratedWorld, ...]), ...]."""
        fig, axes = plt.subplots(len(rows), ncol,
                                 figsize=(PANEL * ncol, PANEL * 0.78 * len(rows)),
                                 squeeze=False)
        for r, (_label, worlds) in enumerate(rows):
            for c in range(ncol):
                ax = axes[r][c]
                if c < len(worlds):
                    _draw(ax, worlds[c], route=args.route)
                else:
                    ax.axis("off")
        fig.tight_layout()
        fig.savefig(path, dpi=args.dpi)
        plt.close(fig)
        print(f"[plot_worlds] wrote {path}")

    # Generated once and reused for both the per-tier sheets and the contact
    # sheet: a maze world is ~250 ms and drawing it twice would double the cost
    # of the only expensive part.
    by_tier = {}
    for tier in tiers:
        worlds = []
        for seed in seeds:
            g = generate(tier, seed)
            worlds.append(g)
            print(f"[plot_worlds]   {tier:10} seed={seed:5} {g.difficulty}")
        by_tier[tier] = worlds
        # The geometry caches are keyed on the world and never evict; a sheet
        # makes one grid per panel plus the bottleneck bisection's several.
        clear_geometry_caches()

    # Per tier: one row of `per_row`, wrapped, so a 12-seed sheet is readable.
    per_row = min(len(seeds), 6)
    for tier, worlds in by_tier.items():
        rows = [(tier, worlds[i:i + per_row])
                for i in range(0, len(worlds), per_row)]
        sheet(rows, out_dir / f"{tier}.png", per_row)

    # And all tiers side by side, which is the one that shows the tiers ARE
    # different rather than telling you they are.
    if len(by_tier) > 1:
        ncol = min(len(seeds), 6)
        sheet([(t, w[:ncol]) for t, w in by_tier.items()],
              out_dir / "contact.png", ncol)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
