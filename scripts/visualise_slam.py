#!/usr/bin/env python
"""
scripts/visualise_slam.py

**A SLAM run, drawn.** Chunk 24b.

Chunks 21-23 verified the pipeline the only way a pipeline can be verified — a
number in a test, a number in `slam.csv`. That answers every question that is a
scalar and none of the questions that are a shape:

    does the map look like lab42?
    does the estimate track the truth, or wander off down a corridor?
    what does the trajectory do at the instant a loop closure lands?
    is the uncertainty proxy anywhere near the realised error?

This draws four things from one run directory and answers all four:

    map.png       the map, the truth, the estimate and the pose graph, one frame
    error.png     realised error vs the PREDICTED proxy, match quality, closures
    closures.png  the graph alone, and where every rejected proposal died
    slam.mp4      all of it, over time, so the map SNAPS when a loop closes

SIM-FREE, AND DELIBERATELY DOWNSTREAM
--------------------------------------
Nothing here imports isaacsim, omni, pxr or torch, and nothing here is a fourth
pane in `scripts/recorder.py`. Two reasons and the second is the decisive one:

1. `recorder.py` costs ~0.9 s per frame because it renders RTX cameras. This
   view needs no renderer, and coupling it to one would put a 40-minute price on
   a picture that takes seconds.
2. **A container run gets the same treatment as a local one.** Reported episodes
   are produced on rented hardware with no display; this runs afterwards, on a
   laptop, against the committed artefacts. A pane inside the video would be
   reachable only by re-simulating.

⚠️ **THIS DRAWS GROUND TRUTH, AND THAT IS ALLOWED HERE BECAUSE IT IS NOT IN THE
LOOP.** Every consumer of `Observation` is forbidden ground truth by contract.
This one reads `trace.csv` after the episode is over, scores nothing, and is
imported by nothing — `core/metrics.py` has never heard of it. Comparing the
estimate against the truth is the entire point of the picture, and the reason it
is safe is that the comparison happens strictly downstream of the run.

WHAT IT READS
-------------
One run directory, as `scripts/run_episode.py` leaves it:

    trace.csv      ground truth and the estimate, one row per CONTROL step
    slam.csv       the localiser's diagnostics, one row per control step
    slam_map.npz   the map and the graph history  (needs `--slam-map` at run time)
    manifest.json  provenance, for the titles

`slam_map.npz` is the only one that is not written by default. Without it the
error panels still draw and the map ones say so rather than inventing a map.

The two CSVs are joined on `slam.csv`'s own `scans` column, which counts scans
taken so far and is exactly what `core/slam/record.py` keys its history on.
There is no second time base anywhere in this file.

    $P scripts/visualise_slam.py out/runs/doorway_traverse+slam__2026-09-04T12-00-00
    $P scripts/visualise_slam.py <run_dir> --only map,error
    $P scripts/visualise_slam.py <run_dir> --only mp4 --speed 8 --fps 20
    $P scripts/visualise_slam.py <run_dir> --out out/evidence/slam-visualisation
"""

import argparse
import csv
import json
import math
import os
import subprocess
import sys

# argparse before the heavy imports, matching the house rule in run_episode.py:
# a malformed flag is rejected before anything expensive is loaded. There is no
# SimulationApp here, but matplotlib's first import is ~1 s and the habit is
# worth keeping uniform across scripts/.
parser = argparse.ArgumentParser(
    description="Draw a SLAM run from its run directory. Sim-free.")
parser.add_argument("run_dir", help="out/runs/<run_id>__<utc>, or any "
                                    "directory holding trace.csv + slam.csv")
parser.add_argument("--out", default=None, metavar="DIR",
                    help="where the artefacts go (default: the run directory)")
parser.add_argument("--only", default="map,error,closures,mp4", metavar="LIST",
                    help="comma-separated subset of map,error,closures,mp4")
parser.add_argument("--fps", type=int, default=20,
                    help="mp4 frame rate (default 20)")
parser.add_argument("--speed", type=float, default=4.0, metavar="X",
                    help="mp4 playback speed against simulated time "
                         "(default 4x). A 120 s episode at 4x is 30 s of video")
parser.add_argument("--max-points", type=int, default=40000, metavar="N",
                    help="thin the cloud to about N points FOR THE ANIMATION "
                         "only; the stills always draw every point")
parser.add_argument("--dpi", type=int, default=140)
args = parser.parse_args()

import numpy as np                                              # noqa: E402

import matplotlib                                               # noqa: E402
matplotlib.use("Agg")                                           # no display
import matplotlib.pyplot as plt                                 # noqa: E402
from matplotlib.patches import Circle, Rectangle                # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core import scene_spec                                     # noqa: E402
from core.scenario import DEV_SCENARIOS                         # noqa: E402
from core.slam.matcher import (MAX_SPREAD, MAX_SPREAD_THETA,    # noqa: E402
                               MIN_SCORE)
from core.slam.record import RecordedMap                        # noqa: E402

# ---- below this line is data, not code -------------------------------------

# One palette, used by all four artefacts, so the same thing is the same colour
# whichever picture you are looking at. Truth is green and the estimate is
# orange throughout — the single most important distinction in every panel.
C_TRUTH = "#2e8b57"
C_EST = "#e07b39"
C_MAP = "#2b3a55"
C_WALL = "#b9bec9"
C_NODE = "#3f6fa8"
C_ODOM = "#8fa8c4"
C_CLOSURE = "#c0392b"
C_UNCERT = "#8e6bbf"
C_GRID = "#e3e6ec"
C_TEXT = "#33383f"

# The map is drawn as the points the pipeline actually holds, not as an
# occupancy grid: `core/slam/likelihood.py` builds a blurred field PER SUBMAP
# and there is no global grid anywhere in the localiser. Rasterising one for the
# picture would be drawing a data structure the SLAM does not have.
POINT_SIZE = 0.6
POINT_ALPHA = 0.35

# core/metrics.py's own threshold, redrawn here so the error panel shows the
# line the score is actually read against rather than a decorative one.
LOC_FAIL_THRESHOLD = 1.0

# ---- below this line is code, not data -------------------------------------


def log(msg):
    print(f"[vis_slam] {msg}", flush=True)


def read_csv(path):
    """A csv of floats -> a dict of numpy columns. Ints stay readable as floats;
    every consumer below either compares or plots, and neither cares."""
    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        return {}
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}


def load_run(run_dir):
    """Everything the pictures are drawn from, or a clear reason there is not.

    Deliberately strict about `trace.csv` and forgiving about everything else:
    a run with no trace is not a run, whereas a run with no map is a run made
    before `--slam-map` existed and the error panels are still worth drawing.
    """
    trace_path = os.path.join(run_dir, "trace.csv")
    if not os.path.exists(trace_path):
        sys.exit(f"[vis_slam] no trace.csv in {run_dir}: that is not a run "
                 f"directory. Expected out/runs/<run_id>__<utc>/")
    trace = read_csv(trace_path)

    slam_path = os.path.join(run_dir, "slam.csv")
    slam = read_csv(slam_path) if os.path.exists(slam_path) else {}
    if not slam:
        sys.exit(f"[vis_slam] no slam.csv in {run_dir}. That run had no "
                 f"localiser with diagnostics -- a drift or perfect run writes "
                 f"none by design. Re-run with --localisation slam.")
    if len(slam["t"]) != len(trace["t"]):
        sys.exit(f"[vis_slam] slam.csv has {len(slam['t'])} rows and trace.csv "
                 f"has {len(trace['t'])}. Both are written one row per control "
                 f"step by the same loop, so they cannot disagree unless one "
                 f"was edited.")

    map_path = os.path.join(run_dir, "slam_map.npz")
    smap = RecordedMap.load(map_path) if os.path.exists(map_path) else None

    manifest = {}
    man_path = os.path.join(run_dir, "manifest.json")
    if os.path.exists(man_path):
        with open(man_path, encoding="utf-8") as fh:
            manifest = json.load(fh)
    return trace, slam, smap, manifest


def errors(trace):
    """Realised position error and heading error. GROUND TRUTH — see the module
    docstring for why that is legitimate here and nowhere a controller looks."""
    pos = np.hypot(trace["x"] - trace["est_x"], trace["y"] - trace["est_y"])
    yaw = np.abs((trace["yaw"] - trace["est_yaw"] + math.pi)
                 % (2 * math.pi) - math.pi)
    return pos, np.degrees(yaw)


def title_for(manifest, run_dir):
    """The scenario id, from whichever of the two manifest shapes is present.

    `run_episode.py` writes a flat `scenario_id`; `compare_localisers.py
    --save-run` writes a nested `scenario.id`. The directory name is the last
    resort and is wrong as soon as the artefacts are copied out of the run
    directory into an evidence folder, which is exactly what they are for.
    """
    scen = manifest.get("scenario", {})
    name = (manifest.get("scenario_id")
            or (scen.get("id") if isinstance(scen, dict) else None))
    return name or os.path.basename(run_dir.rstrip("/")).split("__")[0]


def goal_for(manifest):
    """The goal, for the marker. `None` when it cannot be known honestly.

    `run_episode.py`'s manifest does not carry the goal, so it is looked up from
    the scenario id with the `+noise` / `+slam` / `+cap60s` suffixes stripped —
    those rename a scenario without moving its goal.

    ⚠️ **Only the DEV scenarios are searched, never `core.scenario.load()`.**
    load() would resolve a held-out id, and a rendering script is not a place to
    reach into the sealed split for a decoration. A held-out run therefore draws
    no goal marker, which is the right failure.
    """
    scen = manifest.get("scenario", {})
    if isinstance(scen, dict) and scen.get("goal"):
        return scen["goal"]
    name = (manifest.get("scenario_id") or "").split("+")[0]
    for dev in DEV_SCENARIOS:
        if dev.id == name:
            return dev.goal
    return None


# -- shared drawing ----------------------------------------------------------


def draw_walls(ax):
    """lab42, as the room the run happened in.

    Drawn from `core.scene_spec.wall_rects()`, the same data `build_scene.py`
    exports to USD, so the reference frame in the picture is the reference frame
    of the episode. It is a BACKDROP: the SLAM never sees it, and it is drawn
    pale so that a map point sitting on a wall is legible as a match and a map
    point sitting off one is legible as an error.
    """
    for (x0, y0, x1, y1) in scene_spec.wall_rects():
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                               facecolor=C_WALL, edgecolor="none", zorder=1))
    bx0, by0, bx1, by1 = scene_spec.bounds()
    ax.set_xlim(bx0 - 0.6, bx1 + 0.6)
    ax.set_ylim(by0 - 0.6, by1 + 0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")


def draw_graph(ax, poses, edge_i, edge_j, edge_kind, nodes=True, lw=1.0,
               closure_alpha=0.5):
    """The pose graph: the odometry chain, and the closures that pull it around.

    Closures are drawn as ARCS rather than straight lines, and not for
    decoration — a closure between submap 3 and submap 21 is a chord across the
    room and a straight chord is indistinguishable from a trajectory segment.
    The bow makes it obvious that it is a constraint and not a path.
    """
    for i, j, kind in zip(edge_i, edge_j, edge_kind):
        if i >= len(poses) or j >= len(poses):
            continue
        p, q = poses[i], poses[j]
        if kind == 0:
            ax.plot([p[0], q[0]], [p[1], q[1]], color=C_ODOM, lw=lw,
                    zorder=3, solid_capstyle="round")
        else:
            mx, my = 0.5 * (p[0] + q[0]), 0.5 * (p[1] + q[1])
            dx, dy = q[0] - p[0], q[1] - p[1]
            bow = 0.18
            cx, cy = mx - bow * dy, my + bow * dx
            t = np.linspace(0.0, 1.0, 24)[:, None]
            curve = ((1 - t) ** 2 * np.array([p[0], p[1]])
                     + 2 * (1 - t) * t * np.array([cx, cy])
                     + t ** 2 * np.array([q[0], q[1]]))
            ax.plot(curve[:, 0], curve[:, 1], color=C_CLOSURE, lw=lw,
                    alpha=closure_alpha, zorder=4)
    if nodes and len(poses):
        # Coloured by submap INDEX, not uniformly, because the thing a closure
        # needs is a revisit: two nodes of very different colour sitting on top
        # of each other is the robot coming back to somewhere it has been, and
        # that is what the picture should make obvious.
        return ax.scatter(poses[:, 0], poses[:, 1], s=18,
                          c=np.arange(len(poses)), cmap="viridis", zorder=5,
                          edgecolors="white", linewidths=0.4)
    return None


def draw_robot(ax, pose, color, label=None, size=0.32):
    """A heading triangle, because a dot cannot be wrong about its yaw and the
    estimate's yaw error is half of what the picture is for."""
    x, y, th = pose
    pts = np.array([[size, 0.0], [-0.5 * size, 0.55 * size],
                    [-0.5 * size, -0.55 * size]])
    c, s = math.cos(th), math.sin(th)
    rot = np.array([[c, -s], [s, c]])
    tri = pts @ rot.T + np.array([x, y])
    return ax.fill(tri[:, 0], tri[:, 1], color=color, zorder=8,
                   edgecolor="white", linewidth=0.6, label=label)[0]


# -- the four artefacts ------------------------------------------------------


def figure_map(trace, slam, smap, manifest, out_path, run_dir):
    """The one picture. Map, truth, estimate, graph, closures, final error."""
    fig, ax = plt.subplots(figsize=(11.0, 7.6))
    draw_walls(ax)

    if smap is not None:
        pts = smap.points_at(smap.n_scans)
        if len(pts):
            ax.scatter(pts[:, 0], pts[:, 1], s=POINT_SIZE, c=C_MAP,
                       alpha=POINT_ALPHA, linewidths=0, zorder=2,
                       label=f"SLAM map ({len(pts):,} points)")
        draw_graph(ax, smap.final_poses, smap.edge_i, smap.edge_j,
                   smap.edge_kind, lw=0.7, closure_alpha=0.38)

    ax.plot(trace["x"], trace["y"], color=C_TRUTH, lw=2.0, zorder=6,
            label="ground truth")
    ax.plot(trace["est_x"], trace["est_y"], color=C_EST, lw=1.6, ls="--",
            zorder=7, label="SLAM estimate")

    draw_robot(ax, (trace["x"][-1], trace["y"][-1], trace["yaw"][-1]), C_TRUTH)
    draw_robot(ax, (trace["est_x"][-1], trace["est_y"][-1],
                    trace["est_yaw"][-1]), C_EST)
    ax.scatter([trace["x"][0]], [trace["y"][0]], s=70, marker="o",
               facecolor="white", edgecolor=C_TEXT, zorder=9, label="start")

    goal = goal_for(manifest)
    if goal:
        ax.add_patch(Circle((goal[0], goal[1]), 0.5, facecolor="none",
                            edgecolor=C_TRUTH, ls=":", lw=1.4, zorder=6))
        ax.scatter([goal[0]], [goal[1]], marker="*", s=150, color=C_TRUTH,
                   zorder=9, label="goal")

    pos_err, yaw_err = errors(trace)
    closures = int(slam["closure_edges"][-1]) if "closure_edges" in slam else 0
    subs = int(slam["submaps"][-1]) if "submaps" in slam else 0
    ax.set_title(
        f"{title_for(manifest, run_dir)}   —   {trace['t'][-1]:.1f} s, "
        f"{subs} submaps, {closures} closure edges\n"
        f"final error {pos_err[-1]:.3f} m / {yaw_err[-1]:.1f}°   ·   "
        f"mean {pos_err.mean():.3f} m   ·   max {pos_err.max():.3f} m",
        fontsize=11, color=C_TEXT)
    ax.legend(loc="upper left", fontsize=8, framealpha=0.92)
    fig.tight_layout()
    fig.savefig(out_path, dpi=args.dpi)
    plt.close(fig)
    log(f"map -> {out_path}")


def figure_error(trace, slam, smap, manifest, out_path, run_dir):
    """Four stacked panels on one time base. The top one is the honest one.

    Panel A puts the REALISED error next to `error_estimate()`, which
    `core/slam/localiser.py` documents as a proxy — accumulated dead-reckoning
    uncertainty, not a marginal covariance, and explicitly not calibrated.
    Nothing in this repo has ever plotted the two against each other, and a card
    in chunk 26 (`IsLocalisationDegraded`) is going to read the proxy. Whatever
    the panel says about it is a finding, including if it says the proxy is out
    by an order of magnitude.
    """
    t = trace["t"]
    pos_err, yaw_err = errors(trace)
    fig, axes = plt.subplots(4, 1, figsize=(11.0, 9.4), sharex=True,
                             gridspec_kw={"height_ratios": [3, 2, 2, 2]})

    solves = []
    if smap is not None and len(smap.hist_scan):
        scan_at = slam.get("scans")
        for s in smap.solve_scans():
            hit = np.nonzero(scan_at >= s)[0]
            if len(hit):
                solves.append(t[hit[0]])

    ax = axes[0]
    for x in solves:
        ax.axvline(x, color=C_CLOSURE, lw=0.7, alpha=0.35, zorder=1)
    ax.plot(t, pos_err, color=C_EST, lw=1.6, label="realised error |est − true|")
    if "uncertainty" in slam:
        ax.plot(t, slam["uncertainty"], color=C_UNCERT, lw=1.4, ls="--",
                label="error_estimate() — the PREDICTED proxy")
    ax.axhline(LOC_FAIL_THRESHOLD, color=C_TEXT, lw=0.8, ls=":",
               label=f"LOC_FAIL_THRESHOLD = {LOC_FAIL_THRESHOLD} m")
    ax.set_ylabel("metres")
    ax.legend(fontsize=8, loc="upper left", ncol=2)
    if "uncertainty" in slam:
        # The number the chunk exists to produce. `error_estimate()` is
        # documented as an uncalibrated proxy; this says by how much, on this
        # run, rather than leaving the reader to eyeball two curves.
        live = pos_err > 0
        ratio = float(np.mean(slam["uncertainty"][live] / pos_err[live]))
        ax.text(0.99, 0.94,
                f"proxy / realised: mean {ratio:.1f}x   "
                f"(peak proxy {slam['uncertainty'].max():.2f} m, "
                f"peak realised {pos_err.max():.2f} m)",
                transform=ax.transAxes, ha="right", va="top", fontsize=8,
                color=C_UNCERT)
    ax.set_title(
        f"{title_for(manifest, run_dir)} — localisation over time"
        + (f"   (red lines: {len(solves)} pose-graph solves)" if solves else ""),
        fontsize=11, color=C_TEXT)

    ax = axes[1]
    for x in solves:
        ax.axvline(x, color=C_CLOSURE, lw=0.7, alpha=0.35, zorder=1)
    ax.plot(t, yaw_err, color=C_EST, lw=1.4)
    ax.set_ylabel("heading err (°)")

    ax = axes[2]
    # All three factors `converged` is the AND of, each against its own
    # threshold. Plotting the score alone is how chunk 24b first misread a run
    # whose matches were being refused on ROTATIONAL spread: the score looked
    # healthy, the translational ambiguity looked small, and the panel could not
    # say what was wrong because the deciding number was not in the file.
    if "match_score" in slam:
        ax.plot(t, slam["match_score"], color=C_NODE, lw=1.2,
                label="match score")
        ax.axhline(MIN_SCORE, color=C_NODE, lw=0.7, ls=":")
    if "match_ambiguity" in slam:
        ax.plot(t, slam["match_ambiguity"], color=C_CLOSURE, lw=1.0,
                alpha=0.7, label="ambiguity (m)")
        ax.axhline(MAX_SPREAD, color=C_CLOSURE, lw=0.7, ls=":")
    if "match_ambiguity_theta" in slam:
        # Scaled onto the same axis by its own threshold, so "at 1.0 it is
        # exactly at the limit" reads the same way for all three.
        # Clipped, not rescaled: on a failing run this runs well above its
        # limit and an autoscaled axis would squash the score and the
        # translational ambiguity into the floor. The line sitting flat along
        # the top reads correctly as "off the scale", which it is.
        ax.plot(t, np.minimum(slam["match_ambiguity_theta"] / MAX_SPREAD_THETA,
                              1.5),
                color=C_UNCERT, lw=1.0, alpha=0.8,
                label="heading spread / limit (clipped)")
        ax.axhline(1.0, color=C_UNCERT, lw=0.7, ls=":")
    if "match_converged" in slam:
        # `converged` means score >= MIN_SCORE, which matcher.py is careful to
        # say is not "the optimiser terminated". Shaded because a run of
        # non-converged matches is the failure mode that matters and a dotted
        # line at zero is easy to miss.
        bad = slam["match_converged"] < 0.5
        ax.fill_between(t, 0, 1, where=bad, color=C_CLOSURE, alpha=0.12,
                        transform=ax.get_xaxis_transform(),
                        label="match not converged")
    ax.set_ylabel("match")
    ax.set_ylim(-0.04, 1.55)
    ax.legend(fontsize=8, loc="upper center", ncol=4, framealpha=0.9)

    ax = axes[3]
    if "closures_considered" in slam:
        ax.plot(t, slam["closures_considered"], color=C_ODOM, lw=1.4,
                label="closures considered")
    if "closures_accepted" in slam:
        ax.plot(t, slam["closures_accepted"], color=C_CLOSURE, lw=1.6,
                label="accepted")
    if "submaps" in slam:
        ax.plot(t, slam["submaps"], color=C_NODE, lw=1.2, ls="--",
                label="submaps")
    ax.set_ylabel("count")
    ax.set_xlabel("simulated seconds")
    ax.legend(fontsize=8, loc="upper left", ncol=3)

    for a in axes:
        a.grid(color=C_GRID, lw=0.6)
        a.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(out_path, dpi=args.dpi)
    plt.close(fig)
    log(f"error -> {out_path}")


def figure_closures(trace, slam, smap, manifest, out_path, run_dir):
    """The graph on its own, and where every proposal that died, died.

    The right panel is `loop.py`'s four gates as a bar chart. It is the only
    view that distinguishes "the loop was there and we never saw it" from "we
    saw it and refused it", which is the distinction `ClosureLog` was built to
    preserve and which a single `closures_accepted` number destroys.
    """
    fig, (ax, bx) = plt.subplots(
        1, 2, figsize=(13.6, 6.2), gridspec_kw={"width_ratios": [2.2, 1.0]})

    draw_walls(ax)
    if smap is not None:
        dots = draw_graph(ax, smap.final_poses, smap.edge_i, smap.edge_j,
                          smap.edge_kind, lw=1.0, closure_alpha=0.45)
        if dots is not None:
            fig.colorbar(dots, ax=ax, fraction=0.035, pad=0.02,
                         label="submap index (time)")
        # Numbers only while they can be read. On a two-lap trajectory the
        # second lap's nodes land on the first lap's and the labels collide
        # into illegibility -- which is itself the reason the colourbar is
        # there, so above the threshold the colour carries the ordering alone.
        if len(smap.final_poses) <= 12:
            for k, p in enumerate(smap.final_poses):
                ax.annotate(str(k), (p[0], p[1]), fontsize=7, color=C_TEXT,
                            xytext=(4, 4), textcoords="offset points",
                            zorder=6)
        n_cl = int((smap.edge_kind == 1).sum())
        ax.set_title(f"pose graph: {len(smap.final_poses)} submap nodes, "
                     f"{len(smap.edge_i) - n_cl} odometry edges, "
                     f"{n_cl} closures", fontsize=10, color=C_TEXT)
    else:
        ax.set_title("no slam_map.npz — re-run with --slam-map", fontsize=10)

    last = {k: v[-1] for k, v in slam.items()}
    gates = sorted(k for k in last if k.startswith("closures_rejected_"))
    labels = ["accepted"] + [k[len("closures_rejected_"):] for k in gates]
    values = [last.get("closures_accepted", 0.0)] + [last[k] for k in gates]
    colors = [C_TRUTH] + [C_CLOSURE] * len(gates)
    if "closures_suppressed_travel" in last:
        # Chunk 22a. Its own bar in its own colour, BELOW a separator, because
        # it is not a rejection: nothing here has looked at a scan. Collapsing
        # it into the gates would make an unmatched candidate read as a refused
        # match, which is the distinction `ClosureLog` exists to preserve.
        labels.append("suppressed: no travel")
        values.append(last["closures_suppressed_travel"])
        colors.append(C_ODOM)
    y = np.arange(len(labels))
    bx.barh(y, values, color=colors, alpha=0.85)
    bx.set_yticks(y)
    bx.set_yticklabels(labels, fontsize=9)
    bx.invert_yaxis()
    for i, v in enumerate(values):
        bx.text(v, i, f" {int(v)}", va="center", fontsize=9, color=C_TEXT)
    bx.set_xlabel("candidates")
    supp = int(last.get("closures_suppressed_travel", 0))
    bx.set_title(f"{int(last.get('closures_considered', 0))} proposals matched, "
                 f"by outcome" + (f"\n+ {supp} suppressed before matching"
                                  if supp else ""),
                 fontsize=10, color=C_TEXT)
    bx.grid(axis="x", color=C_GRID, lw=0.6)
    bx.set_axisbelow(True)

    fig.tight_layout()
    fig.savefig(out_path, dpi=args.dpi)
    plt.close(fig)
    log(f"closures -> {out_path}")


def animate(trace, slam, smap, manifest, out_path, run_dir):
    """The map growing, the estimate tracking, and the map SNAPPING on a closure.

    Artists are created once and their data replaced per frame rather than the
    axes being cleared and redrawn: clearing costs more than everything else
    put together at 200 000 points, and the frame is written straight into an
    ffmpeg rawvideo pipe — the same pipe `scripts/recorder.py` uses, so there is
    one way of making a video in this repo rather than two.
    """
    if smap is None:
        log("no slam_map.npz, skipping mp4 (re-run the episode with --slam-map)")
        return
    t = trace["t"]
    control_dt = float(t[1] - t[0]) if len(t) > 1 else 0.02
    stride = max(1, int(round(args.speed / (args.fps * control_dt))))
    frames = range(0, len(t), stride)
    thin = smap.decimate(max(1, len(smap.points) // max(args.max_points, 1)))
    scan_at = slam.get("scans", np.zeros(len(t)))
    pos_err, _ = errors(trace)

    # 12.8 x 6.4 inches at matplotlib's default 100 dpi is 1280 x 640, and
    # both are EVEN, which yuv420p requires. args.dpi is deliberately not used
    # here: it would silently change the video's dimensions and could make them
    # odd, and ffmpeg's failure for that is obscure.
    fig = plt.figure(figsize=(12.8, 6.4), dpi=100)
    gs = fig.add_gridspec(2, 2, width_ratios=[2.6, 1.0],
                          height_ratios=[1.0, 1.0], wspace=0.16, hspace=0.42,
                          left=0.05, right=0.985, top=0.92, bottom=0.09)
    ax = fig.add_subplot(gs[:, 0])
    ex = fig.add_subplot(gs[0, 1])
    cx = fig.add_subplot(gs[1, 1])
    draw_walls(ax)

    cloud = ax.scatter([], [], s=POINT_SIZE, c=C_MAP, alpha=POINT_ALPHA,
                       linewidths=0, zorder=2)
    live = ax.scatter([], [], s=5.0, c=C_CLOSURE, alpha=0.95, linewidths=0,
                      zorder=6, label="current scan")
    truth_line, = ax.plot([], [], color=C_TRUTH, lw=2.0, zorder=6,
                          label="ground truth")
    est_line, = ax.plot([], [], color=C_EST, lw=1.6, ls="--", zorder=7,
                        label="SLAM estimate")
    graph_lines = []
    ax.legend(loc="lower right", fontsize=8, framealpha=0.92)
    heading_truth = draw_robot(ax, (t[0], 0, 0), C_TRUTH)
    heading_est = draw_robot(ax, (t[0], 0, 0), C_EST)

    ex.plot(t, pos_err, color=C_EST, lw=1.2)
    if "uncertainty" in slam:
        ex.plot(t, slam["uncertainty"], color=C_UNCERT, lw=1.0, ls="--")
    ex.axhline(LOC_FAIL_THRESHOLD, color=C_TEXT, lw=0.7, ls=":")
    ex.set_title("error (m): realised vs predicted", fontsize=9, color=C_TEXT)
    ex.grid(color=C_GRID, lw=0.5)
    err_cursor = ex.axvline(t[0], color=C_TEXT, lw=1.0)

    if "closures_accepted" in slam:
        cx.plot(t, slam["closures_accepted"], color=C_CLOSURE, lw=1.4)
    if "submaps" in slam:
        cx.plot(t, slam["submaps"], color=C_NODE, lw=1.0, ls="--")
    cx.set_title("closures accepted / submaps", fontsize=9, color=C_TEXT)
    cx.set_xlabel("simulated seconds")
    cx.grid(color=C_GRID, lw=0.5)
    cl_cursor = cx.axvline(t[0], color=C_TEXT, lw=1.0)

    title = ax.set_title("", fontsize=11, color=C_TEXT)
    fig.canvas.draw()
    w, h = fig.canvas.get_width_height()

    proc = _start_ffmpeg(out_path, w, h, args.fps)
    log(f"mp4: {len(list(frames))} frames at {args.fps} fps, "
        f"{args.speed:g}x, {w}x{h}")
    try:
        for step in frames:
            k = int(scan_at[step])
            poses = thin.poses_at(k)
            pts = thin.points_at(k, poses)
            cloud.set_offsets(pts if len(pts) else np.zeros((0, 2)))
            # From the FULL map, not the decimated one: this is ~180 points
            # and thinning it would leave a scatter of survivors that reads as
            # a bad scan rather than as the current one.
            live_pts = _live_scan(smap, k, poses)
            live.set_offsets(live_pts if len(live_pts) else np.zeros((0, 2)))
            truth_line.set_data(trace["x"][:step + 1], trace["y"][:step + 1])
            est_line.set_data(trace["est_x"][:step + 1],
                              trace["est_y"][:step + 1])

            for artist in graph_lines:
                artist.remove()
            graph_lines.clear()
            shown = smap.edge_scan <= k
            before = set(ax.lines) | set(ax.collections)
            draw_graph(ax, poses, smap.edge_i[shown], smap.edge_j[shown],
                       smap.edge_kind[shown], lw=0.9)
            graph_lines.extend(
                [a for a in ax.lines if a not in before]
                + [a for a in ax.collections if a not in before])

            _move_robot(heading_truth, (trace["x"][step], trace["y"][step],
                                        trace["yaw"][step]))
            _move_robot(heading_est, (trace["est_x"][step], trace["est_y"][step],
                                      trace["est_yaw"][step]))
            err_cursor.set_xdata([t[step], t[step]])
            cl_cursor.set_xdata([t[step], t[step]])
            title.set_text(
                f"{title_for(manifest, run_dir)}   t={t[step]:6.2f} s   "
                f"scan {k}   submaps {len(poses)}   "
                f"closures {int(slam.get('closure_edges', scan_at)[step])}   "
                f"error {pos_err[step]:.3f} m")

            fig.canvas.draw()
            proc.stdin.write(np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
                             .tobytes())
    finally:
        proc.stdin.close()
        code = proc.wait()
        plt.close(fig)
    if code != 0:
        sys.exit(f"[vis_slam] ffmpeg exited {code}; {out_path} is unusable")
    log(f"mp4 -> {out_path}")


def _live_scan(smap, scan, poses):
    """Just the points measured at scan `scan`, so the beam pattern is visible.

    Drawn on top of the accumulated cloud in the closure colour. Without it the
    animation shows a map appearing with no visible cause; with it, the sensor
    is on screen.
    """
    if not len(poses):
        return np.zeros((0, 2))
    keep = smap.point_scan == scan
    if not keep.any():
        return np.zeros((0, 2))
    k = int(smap.point_submap[keep][0])
    if k >= len(poses):
        return np.zeros((0, 2))
    from core.slam import scan as scan_mod
    return scan_mod.transform(smap.points[keep], tuple(poses[k]))


def _move_robot(patch, pose, size=0.32):
    x, y, th = pose
    pts = np.array([[size, 0.0], [-0.5 * size, 0.55 * size],
                    [-0.5 * size, -0.55 * size]])
    c, s = math.cos(th), math.sin(th)
    patch.set_xy(pts @ np.array([[c, -s], [s, c]]).T + np.array([x, y]))


def _start_ffmpeg(out_path, width, height, fps):
    """The same rawvideo pipe scripts/recorder.py opens. Kept identical on
    purpose: one encoder invocation in the repo, one set of playback quirks."""
    cmd = [
        "ffmpeg", "-y", "-hide_banner", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{width}x{height}", "-r", str(fps), "-i", "-",
        "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "20",
        "-pix_fmt", "yuv420p", out_path,
    ]
    return subprocess.Popen(cmd, stdin=subprocess.PIPE)


def main():
    run_dir = args.run_dir.rstrip("/")
    out_dir = args.out or run_dir
    os.makedirs(out_dir, exist_ok=True)
    want = {w.strip() for w in args.only.split(",") if w.strip()}
    unknown = want - {"map", "error", "closures", "mp4"}
    if unknown:
        sys.exit(f"[vis_slam] --only: unknown {', '.join(sorted(unknown))}")

    trace, slam, smap, manifest = load_run(run_dir)
    log(f"{len(trace['t'])} control steps, {trace['t'][-1]:.1f} simulated "
        f"seconds" + (f", {smap.n_scans} scans, {smap.n_submaps} submaps"
                      if smap is not None else ", no map recorded"))

    if "map" in want:
        figure_map(trace, slam, smap, manifest,
                   os.path.join(out_dir, "map.png"), run_dir)
    if "error" in want:
        figure_error(trace, slam, smap, manifest,
                     os.path.join(out_dir, "error.png"), run_dir)
    if "closures" in want:
        figure_closures(trace, slam, smap, manifest,
                        os.path.join(out_dir, "closures.png"), run_dir)
    if "mp4" in want:
        animate(trace, slam, smap, manifest,
                os.path.join(out_dir, "slam.mp4"), run_dir)


if __name__ == "__main__":
    main()
