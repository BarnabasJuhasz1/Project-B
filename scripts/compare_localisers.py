#!/usr/bin/env python
"""
scripts/compare_localisers.py

**Does SLAM beat the random walk?** Chunk 23's headline Done-when, made
reproducible. SIM-FREE: no simulator, no GPU, no Isaac Sim.

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P scripts/compare_localisers.py                       # both trajectories
    $P scripts/compare_localisers.py --trajectory loop     # just the loop
    $P scripts/compare_localisers.py --sigma 0.06          # a gentler insult

WHY THE TRAJECTORY IS REPLAYED AND NOT DRIVEN
---------------------------------------------
Driving each localiser in closed loop confounds two things: how well it
estimates, and how the controller reacted to the estimate it was given. A worse
localiser sends the robot somewhere else, so the two runs are no longer the same
episode and the error columns are not comparable — measured, and the effect is
large: `reactive_avoid` reaches the goal in 14.5 s under a perfect pose and
times out at 40 s under SLAM, which makes SLAM's error look worse purely because
it was integrated over 2.8x as long.

So the trajectory is generated ONCE under `PerfectLocaliser` and then replayed
open-loop through each localiser. Same path, same duration, same scans, same
seed. This measures the ESTIMATOR. What a degraded estimate does to a
controller's success is a different question, and it is the benchmark's.

TWO TRAJECTORIES, BECAUSE THEY ANSWER DIFFERENT QUESTIONS
---------------------------------------------------------
**traverse** — `doorway_traverse` under `reactive_avoid`. One way, never
returns. There is no loop, so there is nothing to close, and SLAM's error can
only be held down by tracking.

**loop** — two laps of a rectangle in lab42's left room, ending where it began.
This is the trajectory epic 03 exists for: *what the robot chose to do* changes
how well it knows where it is, and a robot that returns somewhere it has been
can correct its accumulated error.

⚠️ THE THREE ROWS ARE NOT THREE SETTINGS OF ONE KNOB
-----------------------------------------------------
`odometry` and `slam` share a noise process — the same corrupted deltas, the
same rng — so `slam` minus `odometry` is exactly what the matching and the graph
buy. That subtraction is the honest one.

`drift` is a DIFFERENT error process at the same nominal sigma: `core/drift.py`
adds a random walk to the true pose, whereas odometry integrates corrupted
deltas, and integration compounds where addition does not. At sigma = 0.12 the
two are not equally severe and the table shows it. Chunk 25 is where the two are
calibrated against each other; until then, read `drift` as a landmark and not as
a controlled baseline.
"""

import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

# ---- below this line is data, not code.

# The rectangle the loop trajectory drives, in lab42's left room. Inside the
# divider at x = 6 with better than a metre of clearance at every corner, so the
# path is about the LOCALISER and never about whether the robot fits.
LOOP_CORNERS = ((1.5, 1.5), (4.5, 1.5), (4.5, 6.5), (1.5, 6.5))
LOOP_SPEED = 0.5           # m/s, matching the reference controllers
LOOP_TURN = 0.8            # rad/s, turning on the spot at each corner
LOOP_LAPS = 2              # one lap builds the map, the second one closes on it

# The default insult. DRIFT_MODERATE's own numbers, so the drift row is that
# preset exactly rather than something near it.
SIGMA_XY = 0.12            # metres per root-second
SIGMA_YAW_RATIO = 0.5      # sigma_yaw = sigma_xy / 2, the sealed split's ratio

DT = 0.02                  # the control step, matching core/tests/kinematic.py

# ---- below this line is code, not data.


# The standing height the frozen policy actually holds, measured in CLAUDE.md.
# Written into a saved replay's trace because core/metrics.py reads z against
# FALL_HEIGHT: a replay writing z=0 would score every step as a fall.
REPLAY_Z = 0.213


def log(msg):
    print(f"[compare_localisers] {msg}", flush=True)


def traverse_poses(scenario_id):
    """`reactive_avoid` on a dev scenario, driven under a perfect pose."""
    import dataclasses as dc

    from controllers.reactive_avoid import ReactiveAvoid
    from core.scenario import LocalisationSpec, load
    from core.tests.kinematic import rollout

    scenario = dc.replace(load(scenario_id), id="truth", sensor_noise=NOISE,
                          drift=None,
                          localisation=LocalisationSpec(kind="perfect"))
    return [(r[1], r[2], r[4]) for r in rollout(scenario, ReactiveAvoid()).rows]


def trace_poses(path):
    """The TRUE poses out of a recorded run's `trace.csv`.

    The third trajectory source, and the one that answers a question the other
    two cannot: **is a poor estimate caused by the physics, or by the path the
    physics produced?** Replaying a recorded episode's own true poses through
    the localiser with no simulator in the loop separates the two — if the error
    reproduces, the trajectory is the cause and the physics contributed nothing
    beyond driving it.

    ⚠️ It replays ground truth, which is legitimate for the same reason
    `scripts/visualise_slam.py` may draw it: strictly downstream of a finished
    episode, scoring nothing. It is also why this is a DIAGNOSTIC and not a
    result — the poses came from one run of one controller on one seed.
    """
    import csv

    with open(path, newline="", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        sys.exit(f"[compare_localisers] {path} has no rows")
    return [(float(r["x"]), float(r["y"]), float(r["yaw"])) for r in rows]


def loop_poses(laps=LOOP_LAPS):
    """Two laps of LOOP_CORNERS: drive the edges, turn on the spot at each."""
    x, y = LOOP_CORNERS[0]
    yaw = 0.0
    out = []
    for _ in range(laps):
        for k in range(len(LOOP_CORNERS)):
            tx, ty = LOOP_CORNERS[(k + 1) % len(LOOP_CORNERS)]
            target = math.atan2(ty - y, tx - x)
            while abs(_wrap(target - yaw)) > LOOP_TURN * DT:
                yaw = _wrap(yaw + math.copysign(LOOP_TURN * DT,
                                                _wrap(target - yaw)))
                out.append((x, y, yaw))
            yaw = target
            while math.hypot(tx - x, ty - y) > LOOP_SPEED * DT:
                x += LOOP_SPEED * DT * math.cos(yaw)
                y += LOOP_SPEED * DT * math.sin(yaw)
                out.append((x, y, yaw))
    return out


def _wrap(theta):
    return (theta + math.pi) % (2 * math.pi) - math.pi


def replay(poses, scenario, localiser, blind=False, collect=None):
    """Feed `poses` through `localiser`. Returns the per-step position error.

    `blind` passes an EMPTY scan, which is how the odometry row is produced: the
    same Tracker, the same noise, the same rng stream, and nothing ever matched.

    `collect` is a list that receives one `(t, pose, estimate, clearance,
    diagnostics)` tuple per step, for `--save-run`. Off by default and appended
    to rather than returned, so the normal path's loop is unchanged.
    """
    from core.geometry import clearance
    from core.sensor_noise import make_sensor_noise

    sensor = make_sensor_noise(scenario)
    angles = scenario.beam_angles()
    empty = np.zeros((0,))
    errs = []
    for step, (x, y, yaw) in enumerate(poses):
        if blind:
            ranges, beams = empty, empty
        else:
            ranges = sensor.scan((x, y), yaw, angles, scenario.range_max,
                                 scenario.world)
            beams = angles
        localiser.update(DT, (x, y, yaw), ranges, beams)
        est = localiser.estimate()
        ex, ey, _ = est
        errs.append(math.hypot(x - ex, y - ey))
        if collect is not None:
            collect.append((step * DT, (x, y, yaw), est,
                            float(clearance((x, y), scenario.world)),
                            localiser.diagnostics()))
    return errs


def save_run(out_dir, scenario, localiser, collected):
    """Write the replay as a RUN DIRECTORY, so visualise_slam.py can draw it.

    ⚠️ **This is a replay, not an episode**, and the directory says so: the
    scenario id carries `_replay`, no `result.json` is written and the manifest
    says what produced it -- so nothing can mistake it for something
    `scripts/rescore.py` or the sweep should look at.

    It exists because the loop trajectory is the only thing in this repo that
    drives far enough to close a loop, and until chunk 24b there was no way to
    LOOK at what the closures did. It also means the SLAM pictures are
    reproducible with no GPU and no simulator at all, which is the property
    that makes them usable in a write-up.

    The commands in `trace.csv` are derived from the pose deltas rather than
    invented: under the ideal kinematics this replay assumes, the body-frame
    delta over one step IS the command that produced it. Writing zeros would be
    a trace claiming the robot was stationary while it moved.
    """
    import csv
    import json

    from core.metrics import EpisodeTrace
    from core.observation import Command
    from core.slam import scan as scan_mod

    os.makedirs(out_dir, exist_ok=True)
    trace = EpisodeTrace()
    slam_rows = []
    for k, (t, pose, est, gap, diag) in enumerate(collected):
        nxt = collected[k + 1][1] if k + 1 < len(collected) else pose
        dx, dy, dyaw = scan_mod.relative(pose, nxt)
        command = Command(dx / DT, dy / DT, dyaw / DT)
        # z is the standing height the frozen policy actually holds (CLAUDE.md
        # measured ~0.213 m), not 0: core/metrics.py reads z against
        # FALL_HEIGHT and a replay that writes 0 would score as a fall.
        trace.append(t, (pose[0], pose[1], REPLAY_Z, pose[2]), command, gap,
                     gap <= 0.0, est, True)
        if diag:
            slam_rows.append(dict(diag, t=t))
    trace.write_csv(os.path.join(out_dir, "trace.csv"))

    if slam_rows:
        columns = sorted(slam_rows[0])
        with open(os.path.join(out_dir, "slam.csv"), "w", newline="",
                  encoding="utf-8") as fh:
            writer = csv.DictWriter(fh, fieldnames=columns)
            writer.writeheader()
            writer.writerows(slam_rows)
    recorder = getattr(localiser, "recorder", None)
    if recorder is not None and recorder.scans:
        recorder.save_npz(os.path.join(out_dir, "slam_map.npz"))
    with open(os.path.join(out_dir, "manifest.json"), "w",
              encoding="utf-8") as fh:
        json.dump({"scenario": {"id": scenario.id,
                                "goal": list(scenario.goal),
                                "start": list(scenario.start)},
                   "produced_by": "scripts/compare_localisers.py --save-run",
                   "note": "A REPLAY of a fixed trajectory, not a simulated "
                           "episode. Not scoreable and not reportable."},
                  fh, indent=2)
    log(f"  run directory -> {out_dir}")
    log(f"  draw it:  $P scripts/visualise_slam.py {out_dir}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[2])
    ap.add_argument("--trajectory", choices=("traverse", "loop", "both"),
                    default="both")
    ap.add_argument("--from-trace", default=None, metavar="TRACE_CSV",
                    help="replay the TRUE poses recorded in a run's trace.csv "
                         "instead of a synthetic trajectory. Answers 'is the "
                         "estimate poor because of the physics, or because of "
                         "the path the physics drove?' -- if the error "
                         "reproduces here, the physics contributed nothing "
                         "beyond the trajectory. Overrides --trajectory")
    ap.add_argument("--seeds", default=None, metavar="S1,S2,...",
                    help="repeat over these scenario seeds and report mean +- "
                         "sd. ⚠️ Localiser error is stochastic and a "
                         "single-seed ranking is not a ranking: on one seed "
                         "odometry beat SLAM here, and over five it did not")
    ap.add_argument("--scenario", default="doorway_traverse",
                    help="the dev scenario the traverse is driven on")
    ap.add_argument("--sigma", type=float, default=SIGMA_XY,
                    help=f"metres per root-second, default {SIGMA_XY}. Applied "
                         f"identically to the drift model and to the odometry "
                         f"SLAM integrates")
    ap.add_argument("--laps", type=int, default=LOOP_LAPS)
    # ⚠️ Defaults to DEFAULT_SCAN_RATE, which chunk 24a MOVED from 10 Hz to 5.
    # The table committed in out/evidence/localisers/ was produced before that
    # move and needs `--scan-rate 10` to reproduce. The flag exists so the
    # committed numbers stay reachable rather than becoming a claim nobody can
    # check -- the rate is a property of the apparatus, and an evidence folder
    # that cannot be re-run is a screenshot.
    ap.add_argument("--scan-rate", type=float, default=0.0, metavar="HZ",
                    help="localiser scan rate; 0 means DEFAULT_SCAN_RATE "
                         "(5 Hz since chunk 24a). Pass 10 to reproduce the "
                         "table committed in out/evidence/localisers/")
    ap.add_argument("--save-run", default=None, metavar="DIR",
                    help="write the SLAM row as a run directory (trace.csv, "
                         "slam.csv, slam_map.npz), one subdirectory per "
                         "trajectory, for scripts/visualise_slam.py. Turns on "
                         "the localiser's map recording, which is inert")
    args = ap.parse_args()

    import dataclasses as dc

    from core.scenario import DriftSpec
    from core.localisation import make_localiser
    from core.scenario import LocalisationSpec, load

    slam = LocalisationSpec(kind="slam", odom_sigma_lin=args.sigma,
                            odom_sigma_ang=args.sigma * SIGMA_YAW_RATIO,
                            scan_rate=args.scan_rate)
    drift = DriftSpec(sigma_xy=args.sigma,
                      sigma_yaw=args.sigma * SIGMA_YAW_RATIO)

    seeds = ([int(v) for v in args.seeds.split(",")] if args.seeds else [None])
    wanted = (("recorded",) if args.from_trace
              else ("traverse", "loop") if args.trajectory == "both"
              else (args.trajectory,))
    for label in wanted:
        poses = (trace_poses(args.from_trace) if label == "recorded"
                 else traverse_poses(args.scenario) if label == "traverse"
                 else loop_poses(args.laps))
        log(f"{label}: {len(poses)} steps, {len(poses) * DT:.1f} sim seconds")
        base = dc.replace(load(args.scenario), id=f"{label}_replay",
                          time_limit=len(poses) * DT, sensor_noise=NOISE,
                          drift=None)
        if len(seeds) > 1:
            log(f"  {len(seeds)} seeds: {seeds}")
        print(f"  {'localiser':10s} {'final':>8s} {'mean':>8s} {'max':>8s} "
              f"{'wall':>7s}")
        for name in ("odometry", "drift", "slam"):
            finals, means, maxes = [], [], []
            wall = 0.0
            localiser = collected = None
            for seed in seeds:
                seeded = base if seed is None else dc.replace(base, seed=seed)
                scenario = (dc.replace(seeded, drift=drift) if name == "drift"
                            else dc.replace(seeded, localisation=slam))
                localiser = make_localiser(scenario)
                saving = bool(args.save_run) and name == "slam" and seed is None
                if saving:
                    localiser.enable_recording()
                localiser.reset(scenario, scenario.localisation_rng())
                collected = [] if saving else None
                t0 = time.perf_counter()
                errs = replay(poses, scenario, localiser,
                              blind=(name == "odometry"), collect=collected)
                wall += time.perf_counter() - t0
                finals.append(errs[-1])
                means.append(float(np.mean(errs)))
                maxes.append(max(errs))
            if len(seeds) > 1:
                print(f"  {name:10s} {np.mean(finals):8.3f} "
                      f"{np.mean(means):8.3f} {np.mean(maxes):8.3f} {wall:7.1f}"
                      f"   +-{np.std(finals):.3f} / +-{np.std(means):.3f} "
                      f"over {len(seeds)} seeds", flush=True)
            else:
                print(f"  {name:10s} {finals[0]:8.3f} {means[0]:8.3f} "
                      f"{maxes[0]:8.3f} {wall:7.1f}", flush=True)
            if saving:
                save_run(os.path.join(args.save_run, label), scenario,
                         localiser, collected)
            if name == "slam":
                d = localiser.diagnostics()
                log(f"  submaps={d['submaps']} "
                    f"closures={d['closures_accepted']}/"
                    f"{d['closures_considered']} solves={d['solves']} "
                    f"solve_wall={localiser.total_solve_seconds:.3f}s")


# Chosen once, here, so all three rows see the same sensor: the matcher must not
# be fed core.geometry.raycast's EXACT ranges, or it reports an accuracy that is
# a property of the raycast rather than of SLAM. NOISE_MODERATE is chunk 20's.
from core.sensor_noise import NOISE_MODERATE as NOISE       # noqa: E402


if __name__ == "__main__":
    main()
