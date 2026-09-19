#!/usr/bin/env python
"""One row of `<tier>-worlds.csv`, opened up: why did this node break THIS world?

`scripts/measure_growth.py` says a node `broke` a world. This says how. It
re-runs the two rollouts that produced the row — the base alone, and
`Layer[ContractPorts, BASE, node]` — on the same generated world, and draws:

    trajectories.png  both paths over the geometry, contacts, where each stopped
    timeline.png      vx / wz / clearance / goal distance against time, for both,
                      with the node's precondition shaded where it fired
    episode.mp4       the two runs animated side by side, same clock
    README.md         what question this answers and how to reproduce it

⚠️ KINEMATIC, NOT PHYSICS, and therefore NOT A BENCHMARK PATH. This is the
rollout that produced the growth row, which is the point: it is the run being
explained, not a second opinion on it. The frozen locomotion policy is absent.
A physics version of this case needs three things that do not exist yet — a USD
export for generated worlds, a scenario binding `core.scenario.load()` resolves,
and a `controllers.make()` spec for the layered pairing (chunk 46 and after).

Reproducing the case that motivated this script:

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P scripts/debug_growth_case.py --tier maze --seed 2004 \
        --node gen_b_44d12df4d25b437f

    # the row it explains, from out/growth/maze-worlds.csv:
    # maze,2004,gen-maze-02004,gen_b_44d12df4d25b437f,B,1,0,broke,0.9375,0.9375,...
    grep gen_b_44d12df4d25b437f out/growth/maze-worlds.csv | grep ,2004,
"""
from __future__ import annotations

import argparse
import math
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Explain one growth row. KINEMATIC, not a benchmark path.")
    p.add_argument("--tier", default="maze")
    p.add_argument("--seed", type=int, required=True)
    p.add_argument("--node", required=True, help="node_id, e.g. gen_b_44d1...")
    p.add_argument("--time-limit", type=float, default=120.0,
                   help="must match the growth run being explained (120 s)")
    p.add_argument("--out", default=None,
                   help="default out/growth-debug/<tier>-<seed>-<node>/")
    p.add_argument("--fps", type=int, default=25)
    p.add_argument("--no-video", action="store_true",
                   help="skip episode.mp4; the two PNGs are the cheap half")
    return p.parse_args(argv)


# Everything above this line is code; below it is data, not code.
# ---------------------------------------------------------------------------

# Shared with scripts/plot_kinematic.py on purpose: two pictures of the same
# rollout that disagreed about which line was the robot would be worse than no
# picture at all.
BASE_COLOUR = "#1f77b4"
PAIR_COLOUR = "#d62728"
CONTACT_COLOUR = "#d62728"
WALL_COLOUR = "#3f3f3f"
OPTIMAL_COLOUR = "#8a8a8a"
GOAL_COLOUR = "#e6a700"
FIRE_SHADE = "#ffd9d9"


def _growth_build():
    """`scripts/measure_growth.py:_build`, loaded by path.

    measure_growth is a script rather than a package module, so it is imported
    the only way a script can be. Worth the ugliness: the alternative is a
    second copy of the layering rule.
    """
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "_measure_growth", REPO_ROOT / "scripts" / "measure_growth.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module._build


def _rollout(tree, scenario, node_id):
    """One rollout, plus the node's own tick summary. Trace is byte-identical
    whether or not tracing is on — `nodes/trace.py` is inert by construction."""
    from core.metrics import GOAL_RADIUS, score
    from core.tests.kinematic import rollout

    log = tree.enable_trace()
    trace = rollout(scenario, tree)
    x, y = trace.column("x")[-1], trace.column("y")[-1]
    reached = math.dist((x, y), scenario.goal) <= GOAL_RADIUS
    return trace, score(trace, scenario), reached, log.summary().get(node_id, {})


def _fired_mask(tree_log_rows, n):
    """Per-step 'did the precondition pass', from the tick log."""
    import numpy as np
    mask = np.zeros(n, dtype=bool)
    for step in tree_log_rows:
        if 0 <= step < n:
            mask[step] = True
    return mask


def main(argv=None):
    args = _parse_args(argv)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.patches import Circle, Rectangle

    from compose.library import load_library
    from core.geometry import shortest_path
    from core.worldgen import generate, to_scenario
    from generate import base as basemod
    from generate.admit import build_tree

    g = generate(args.tier, args.seed)
    scenario = to_scenario(g, time_limit=args.time_limit)
    world = g.world
    base = basemod.load()

    entries = [e for e in load_library().all()
               if e.metadata.node_id == args.node]
    if not entries:
        raise SystemExit(f"[debug] unknown node {args.node!r}. "
                         f"See `controllers.library_nodes()`.")
    entry = entries[0]

    # ⚠️ The pairing is IMPORTED from the growth script, never re-declared.
    # If it were rebuilt here, this could explain a tree the growth row never
    # measured — and the two would drift apart silently the next time the
    # layering changes. `_build` is the one definition of "base + node".
    pair_tree = _growth_build()(entry.node_cls, entry.metadata, base)

    out = Path(args.out) if args.out else (
        REPO_ROOT / "out" / "growth-debug"
        / f"{args.tier}-{args.seed}-{args.node}")
    out.mkdir(parents=True, exist_ok=True)

    print(f"[debug] {args.tier}/{args.seed}  node={args.node}", flush=True)
    print(f"[debug] {world.name}  {g.difficulty}", flush=True)

    b_trace, b_score, b_hit, _ = _rollout(
        build_tree(base.node_cls, base.metadata), scenario, args.node)
    p_trace, p_score, p_hit, p_summary = _rollout(
        pair_tree, scenario, args.node)

    for label, sc, hit in (("BASE", b_score, b_hit), ("PAIR", p_score, p_hit)):
        print(f"[debug] {label}: reached={hit} "
              f"path={sc['path_length']:.2f} m  coll={sc['collisions']} "
              f"minclear={sc['min_clearance']:.3f} "
              f"stuck={sc['stuck_events']}", flush=True)
    print(f"[debug] node ticks: {p_summary}", flush=True)

    gx, gy = scenario.goal
    sx, sy, _yaw = scenario.start
    taut = shortest_path((sx, sy), (gx, gy), world=world)

    def draw_world(ax):
        for (x0, y0, x1, y1) in world.rects:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                                   facecolor=WALL_COLOUR, edgecolor="none",
                                   zorder=1))
        if len(taut):
            ax.plot([p[0] for p in taut], [p[1] for p in taut],
                    color=OPTIMAL_COLOUR, lw=1.3, ls=(0, (6, 4)), zorder=2,
                    label="taut optimal")
        ax.plot([sx], [sy], "o", ms=9, color="black", mfc="white", zorder=6,
                label="start")
        ax.plot([gx], [gy], "*", ms=17, color=GOAL_COLOUR, mec="black",
                mew=0.6, zorder=6, label="goal")
        from core.metrics import GOAL_RADIUS
        ax.add_patch(Circle((gx, gy), GOAL_RADIUS, fill=False,
                            edgecolor=GOAL_COLOUR, ls=":", lw=1.1, zorder=5))
        x0, y0, x1, y1 = world.bounds
        ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)
        ax.set_aspect("equal"); ax.grid(alpha=0.15, zorder=0)

    # ---- trajectories.png
    fig, ax = plt.subplots(figsize=(9, 6.4))
    draw_world(ax)
    for trace, colour, label, hit in (
            (b_trace, BASE_COLOUR, "base alone", b_hit),
            (p_trace, PAIR_COLOUR, f"base + {args.node}", p_hit)):
        x, y = trace.column("x"), trace.column("y")
        ax.plot(x, y, color=colour, lw=2.0, zorder=4,
                label=f"{label} — {'REACHED' if hit else 'failed'}")
        contact = trace.column("contact").astype(bool)
        if contact.any():
            ax.plot(x[contact][::20], y[contact][::20], "x", ms=6,
                    color=CONTACT_COLOUR, zorder=7)
        ax.plot([x[-1]], [y[-1]], "s", ms=7, color=colour, mec="black",
                mew=0.5, zorder=6)
    ax.set_title(f"{args.tier}/{args.seed} — {args.node}\n"
                 f"base {'REACHED' if b_hit else 'failed'}, "
                 f"pair {'REACHED' if p_hit else 'failed'}", fontsize=10)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout(); fig.savefig(out / "trajectories.png", dpi=140)
    plt.close(fig)

    # ---- timeline.png
    fig, axes = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for trace, colour, label in ((b_trace, BASE_COLOUR, "base"),
                                 (p_trace, PAIR_COLOUR, "pair")):
        t = trace.column("t")
        gd = np.hypot(trace.column("x") - gx, trace.column("y") - gy)
        axes[0].plot(t, gd, color=colour, lw=1.4, label=label)
        axes[1].plot(t, trace.column("vx"), color=colour, lw=1.0, label=label)
        axes[2].plot(t, trace.column("wz"), color=colour, lw=1.0, label=label)
        axes[3].plot(t, trace.column("clearance"), color=colour, lw=1.0,
                     label=label)
    from core.metrics import GOAL_RADIUS
    axes[0].axhline(GOAL_RADIUS, color=GOAL_COLOUR, ls=":", lw=1.1)
    axes[0].set_ylabel("distance to goal (m)")
    axes[1].set_ylabel("vx (m/s)")
    axes[2].set_ylabel("wz (rad/s)")
    axes[3].set_ylabel("clearance (m)"); axes[3].set_xlabel("t (s)")
    for a in axes:
        a.grid(alpha=0.2); a.legend(fontsize=8, loc="upper right")
    axes[0].set_title(f"{args.tier}/{args.seed} — {args.node}", fontsize=10)
    fig.tight_layout(); fig.savefig(out / "timeline.png", dpi=140)
    plt.close(fig)

    # ---- episode.mp4
    if not args.no_video:
        _animate(out, args, world, draw_world, b_trace, p_trace, b_hit, p_hit)

    (out / "README.md").write_text(_readme(args, g, b_score, p_score,
                                           b_hit, p_hit, p_summary),
                                   encoding="utf-8")
    print(f"[debug] wrote {out}", flush=True)
    return 0


def _animate(out, args, world, draw_world, b_trace, p_trace, b_hit, p_hit):
    """Both runs, same clock, side by side -> mp4 through ffmpeg."""
    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.animation import FFMpegWriter

    n = max(len(b_trace.column("t")), len(p_trace.column("t")))
    stride = max(1, int(round(50 / args.fps)))     # traces are 50 Hz
    frames = range(0, n, stride)

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.4))
    artists = []
    for ax, trace, colour, label, hit in (
            (axes[0], b_trace, BASE_COLOUR, "base alone", b_hit),
            (axes[1], p_trace, PAIR_COLOUR, f"base + {args.node}", p_hit)):
        draw_world(ax)
        ax.set_title(f"{label} — {'REACHED' if hit else 'failed'}", fontsize=10)
        line, = ax.plot([], [], color=colour, lw=2.0, zorder=4)
        dot, = ax.plot([], [], "o", ms=8, color=colour, mec="black", mew=0.6,
                       zorder=8)
        artists.append((trace, line, dot))
    clock = fig.text(0.5, 0.97, "", ha="center", fontsize=10)

    writer = FFMpegWriter(fps=args.fps, bitrate=2400)
    path = out / "episode.mp4"
    with writer.saving(fig, str(path), dpi=110):
        for k in frames:
            for trace, line, dot in artists:
                x, y = trace.column("x"), trace.column("y")
                i = min(k, len(x) - 1)
                line.set_data(x[:i + 1], y[:i + 1])
                dot.set_data([x[i]], [y[i]])
            clock.set_text(f"t = {k / 50.0:6.2f} s")
            writer.grab_frame()
    plt.close(fig)
    print(f"[debug] wrote {path} ({len(list(frames))} frames)", flush=True)


def _readme(args, g, b_score, p_score, b_hit, p_hit, p_summary):
    d = g.difficulty
    return f"""# {args.tier}/{args.seed} — why `{args.node}` broke this world

**Question.** `out/growth/{args.tier}-worlds.csv` records this (node × world) as
`broke`: the base reached the goal and the base+node pair did not. Why?

**Answer.** See `trajectories.png` and `timeline.png`; `episode.mp4` animates both.

| | base alone | base + node |
|---|---|---|
| goal reached | **{b_hit}** | **{p_hit}** |
| path length | {b_score['path_length']:.2f} m | {p_score['path_length']:.2f} m |
| collisions | {b_score['collisions']} | {p_score['collisions']} |
| near misses | {b_score['near_misses']} | {p_score['near_misses']} |
| min clearance | {b_score['min_clearance']:.3f} m | {p_score['min_clearance']:.3f} m |
| stuck events | {b_score['stuck_events']} | {p_score['stuck_events']} |

Node tick summary: `{p_summary}`

World `{g.world.name}` (a pure function of `({args.tier}, {args.seed})`):
{d}

⚠️ **Kinematic, not physics, and not a benchmark path.** This is the rollout that
produced the growth row — the run being explained, not a second opinion on it.

## Reproduce

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P scripts/debug_growth_case.py --tier {args.tier} --seed {args.seed} \\
    --node {args.node}
```
"""


if __name__ == "__main__":
    raise SystemExit(main())
