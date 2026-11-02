"""
demo/record_isaac.py

ONE CONTROLLER, THE FULL PATROL, UNDER PHYSICS, RECORDED.

This is the demo's video path. It drives the real Go2 articulation with the
frozen locomotion policy through all four legs and moves `/World/Door` into
place when the short route shuts.

    python demo/record_isaac.py demo/arms/arm0/k0.py --out demo/out/arm0.mp4
    python demo/record_isaac.py demo/arms/armB/k1.py --out demo/out/armB.mp4
    # then, side by side:
    python demo/record_isaac.py --stack demo/out/arm0.mp4 demo/out/armB.mp4
    # Exp 3: a perturbed rep with a per-step trace, no video
    python demo/record_isaac.py demo/arms/arm0/k0.py --no-record --seed 1 \
        --start-dx -0.088 --start-dy 0.077 --start-dyaw 0.023 \
        --trace demo/out/v2/traces/arm0__k0__r1.csv.gz
    # check the flags and the import gate without booting the simulator
    python demo/record_isaac.py demo/arms/arm0/k0.py --seed 1 --dry-run

⚠️ **EXP 3 FLAGS (Mindfire_Experiment_Plan.md §3).** `--seed` goes to
`PatrolScenario(seed=...)`; `--start-dx/-dy/-dyaw` are added to HOME and the
spawn yaw 0.6. With all four at their defaults the episode is the ORIGINAL
condition, byte for byte the loop that produced demo/out/physics.csv -- the
trace, the timing and the load sampling are all outside the control loop's
arithmetic, so they cannot move the trajectory. `--trace` writes one gzip CSV
row per control step (`demo/exp3.py` TRACE_COLUMNS). The `RESULT` line is
unchanged so old parsers still work; a `RESULT_JSON` line follows it with the
episode summary (contacts, path, step timing, tracking error, load average).

⚠️ **EXP A / EXP D (prereg/expA.md, prereg/expD.md).** `--door-never-shuts` is
the open-maze control: the door stays open on all four legs. The summary also
carries `cpu_max`, this process's cgroup CPU limit, so an Exp D episode proves
the slow-machine limit was active while it ran. Neither touches the default
path.

⚠️ **HOW THE DOOR IS TOGGLED.** `/World/Door` is translated 50 m below the floor
for legs 0-1 and back to its authored height for legs 2-3. Moving it removes
both its collision and its appearance in one operation, so the physics scene and
`demo/world.py`'s `OPEN_WORLD` / `BLOCKED_WORLD` cannot drift apart.

⚠️ **THE LIDAR IS ANALYTIC**, cast by `core.geometry.raycast` against the same
two `World`s the kinematic rollout uses -- exactly as `scripts/run_episode.py`
does. What physics supplies here is the POSE: the robot moves because the frozen
policy turned a Twist into a gait, not because a unicycle was integrated. That
is the whole reason this path exists and the kinematic one is not reportable.

⚠️ Rendering dominates wall clock at ~0.9 s/frame and never advances physics, so
the trajectory is identical with or without `--record`. A 180 s episode at 10
fps is ~27 minutes of wall clock. Use `--fps` as the cost lever.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time


def _offset(limit):
    """argparse type: a float within +-limit. Validated BEFORE the simulator
    boots, as every flag here is -- a typo must not cost a 20 s startup."""
    def parse(text):
        v = float(text)
        if not math.isfinite(v) or abs(v) > limit:
            raise argparse.ArgumentTypeError(f"{text} is outside +-{limit}")
        return v
    return parse


def _trace_path(text):
    if not (text.endswith(".csv.gz") or text.endswith(".csv")):
        raise argparse.ArgumentTypeError("--trace must end in .csv.gz or .csv")
    return text


parser = argparse.ArgumentParser()
parser.add_argument("controller", nargs="?", help="path to a controller .py")
parser.add_argument("--out", default=None)
parser.add_argument("--scene", default="scenes/demo-patrol.usd")
parser.add_argument("--fps", type=float, default=10.0)
parser.add_argument("--res", default="1280x720")
parser.add_argument("--no-record", action="store_true")
parser.add_argument("--gui", action="store_true")
parser.add_argument("--labels", nargs=2, metavar=("LTEXT", "RTEXT"),
                    help="captions for the two panes of --stack")
parser.add_argument("--stack", nargs=2, metavar=("LEFT", "RIGHT"),
                    help="ffmpeg two videos side by side; runs no simulator")
# ---- Exp 3. Defaults are the original condition.
parser.add_argument("--seed", type=int, default=0,
                    help="PatrolScenario seed (default 0, the original)")
parser.add_argument("--start-dx", type=_offset(0.5), default=0.0,
                    help="m added to HOME x")
parser.add_argument("--start-dy", type=_offset(0.5), default=0.0,
                    help="m added to HOME y")
parser.add_argument("--start-dyaw", type=_offset(math.pi), default=0.0,
                    help="rad added to the spawn yaw 0.6")
parser.add_argument("--trace", type=_trace_path, default=None,
                    help="per-step trace, gzip CSV (demo/exp3.py)")
# ---- Exp A (prereg/expA.md). Default off = the original door schedule.
parser.add_argument("--door-never-shuts", action="store_true",
                    help="OPEN-MAZE CONTROL: the door stays open on every "
                         "leg, so the short route is never blocked")
parser.add_argument("--dry-run", action="store_true",
                    help="validate flags, load the controller, print the "
                         "spawn pose and exit before the simulator boots")
args = parser.parse_args()

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

# ---- the stacking path needs no simulator, so it returns before the boot.
if args.stack:
    import subprocess
    left, right = args.stack
    out = args.out or os.path.join("demo", "out", "side-by-side.mp4")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    # ⚠️ LABELLED BY BEHAVIOUR, NOT BY ARM, and that is deliberate. These two
    # controllers were picked AFTER seeing their physics results, one per side,
    # and the arm-level comparison is a null (and kinematic). A pane captioned
    # "arm 0" vs "arm B" would read as a corpus claim the data does not carry.
    # Override with --labels if you have a caption the evidence supports.
    ltext, rtext = (args.labels if args.labels else
                    ("SOLVES - reroutes after the door shuts (4/4 trips)",
                     "FAILS - never finds the way round (2/4 trips)"))
    def esc(t):
        return t.replace("\\", "").replace(":", "\\:").replace("'", "")
    draw = ("drawtext=text='{}':x=20:y=20:fontsize=34:fontcolor=white:"
            "box=1:boxcolor=black@0.55:boxborderw=8")
    cmd = ["ffmpeg", "-y", "-i", left, "-i", right, "-filter_complex",
           f"[0:v]{draw.format(esc(ltext))}[l];"
           f"[1:v]{draw.format(esc(rtext))}[r];"
           "[l][r]vstack=inputs=2", "-c:v", "libx264", "-crf", "23",
           "-pix_fmt", "yuv420p", out]
    print("[record]", " ".join(cmd))
    subprocess.run(cmd, check=True)
    print(f"[record] wrote {out}")
    sys.exit(0)

if not args.controller:
    sys.exit("a controller path is required unless --stack is used")

from demo.world import HOME, GOAL, OPEN_WORLD, BLOCKED_WORLD
from demo.rollout import (LEG_TARGETS, LEG_BLOCKED, TIME_LIMIT, ARRIVE_RADIUS,
                          N_BEAMS, RANGE_MAX, MAX_LIN, MAX_ANG,
                          PatrolScenario, load_controller)
from core.scene_spec import GO2_START
from demo import exp3, exp_ad

SPAWN_YAW = 0.6
SPAWN = (HOME[0] + args.start_dx, HOME[1] + args.start_dy,
         SPAWN_YAW + args.start_dyaw)

if args.dry_run:
    load_controller(args.controller)          # the import gate, sim-free
    print("[record] DRY RUN " + json.dumps({
        "controller": args.controller, "seed": args.seed,
        "spawn": [round(v, 4) for v in SPAWN],
        "trace": args.trace, "record": not args.no_record,
        "door_never_shuts": args.door_never_shuts,
        "cpu_max": exp_ad.cgroup_cpu_max()}))
    sys.exit(0)

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": not args.gui})

import numpy as np
from core.geometry import ROBOT_RADIUS, clearance, raycast
from core.observation import Observation, clamp

sys.path.insert(0, os.path.join(REPO, "scripts"))
from go2_policy import open_episode

DOOR_PRIM = "/World/Door"
DOOR_HIDDEN_Z = -50.0
SETTLE_SECONDS = 2.0


def log(*a):
    print("[record]", *a, flush=True)


def door_setter():
    """Returns set_door(shut: bool).

    Moving the prim removes its collision and its appearance together, so the
    physics scene cannot disagree with `demo/world.py`. It goes through the
    same `SingleXFormPrim`/`XFormPrim` ladder `scripts/export_world.py` uses
    rather than editing xform ops by hand -- a FixedCuboid's op order is the
    isaacsim API's business, and AddTranslateOp on top of it silently stacks a
    second translate.
    """
    import omni.usd
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(DOOR_PRIM)
    if not prim or not prim.IsValid():
        raise RuntimeError(
            f"{DOOR_PRIM} is not in {args.scene}. Re-export with "
            f"demo/export_demo_world.py -- the door must be its own prim.")
    try:
        from isaacsim.core.prims import SingleXFormPrim as _XForm
    except ImportError:
        from isaacsim.core.prims import XFormPrim as _XForm
    handle = _XForm(DOOR_PRIM)
    here, _ = handle.get_world_pose()
    here = np.array(here, dtype=float).reshape(-1)[:3]
    shut_pos = here.copy()
    open_pos = np.array([here[0], here[1], DOOR_HIDDEN_Z])

    def set_door(shut):
        handle.set_world_pose(position=shut_pos if shut else open_pos)
    return set_door


def main():
    wall0 = time.time()
    load_samples = [os.getloadavg()[0]]
    with open(args.controller, "rb") as fh:
        controller_sha = hashlib.sha256(fh.read()).hexdigest()[:16]
    cls = load_controller(args.controller)
    controller = cls()
    scenario = PatrolScenario(seed=args.seed)
    angles = scenario.beam_angles()

    episode = open_episode(os.path.join(REPO, args.scene),
                           (SPAWN[0], SPAWN[1], GO2_START[2], SPAWN[2]), log=log)
    set_door = door_setter()
    set_door(False)

    for _ in range(int(SETTLE_SECONDS / episode.control_dt)):
        episode.drive((0.0, 0.0, 0.0))
    settled = episode.pose()
    log(f"settled at {settled[:3]}")

    recorder = None
    if not args.no_record and not args.gui:
        from recorder import Recorder
        w, h = (int(v) for v in args.res.lower().split("x"))
        # ABSOLUTE on purpose: recorder.resolve_out_path() anchors a RELATIVE
        # path under <repo>/out/, so "demo/out/x.mp4" would land in
        # out/demo/out/x.mp4 and the file would not be where the log says.
        out = os.path.abspath(args.out or os.path.join(
            REPO, "demo", "out",
            os.path.basename(args.controller).replace(".py", "") + ".mp4"))
        os.makedirs(os.path.dirname(out), exist_ok=True)
        recorder = Recorder(out, resolution=(w, h), fps=args.fps)
        log(f"recording -> {recorder.out_path} at {args.fps} fps")

    controller.reset(scenario, scenario.rng())      # ONCE, as in the rollout
    dt = episode.control_dt
    t, leg, leg_start, leg_times = 0.0, 0, 0.0, []
    next_frame, door_shut = 0.0, False
    # Exp 3 bookkeeping. Collected in memory and written after the loop, so no
    # file I/O happens between two control steps.
    trace_rows, poses, commands, raised = [], [], [], None
    next_load = 5.0

    while t < TIME_LIMIT and leg < len(LEG_TARGETS):
        # Exp A: with --door-never-shuts the schedule is all-open, so the door
        # prim stays 50 m below the floor and the lidar sees OPEN_WORLD.
        blocked = LEG_BLOCKED[leg] and not args.door_never_shuts
        if blocked != door_shut:
            door_shut = blocked
            set_door(door_shut)
            log(f"t={t:6.1f}s  leg {leg}: door {'SHUT' if door_shut else 'open'}")

        world = BLOCKED_WORLD if door_shut else OPEN_WORLD
        pose = episode.pose()
        x, y, yaw = pose[0], pose[1], pose[3]
        gap = float(clearance((x, y), world)) - ROBOT_RADIUS

        obs = Observation(
            t=t,
            ranges=raycast((x, y), yaw, angles, RANGE_MAX, world),
            range_angles=angles, range_max=RANGE_MAX,
            pose_estimate=(x, y, yaw), pose_error_estimate=0.0,
            goal=LEG_TARGETS[leg], contact=gap <= 0.0)
        t_step = time.perf_counter()
        try:
            command = clamp(controller.step(obs), MAX_LIN, MAX_ANG)
        except Exception as exc:
            log(f"controller raised at t={t:.1f}s: {exc!r}")
            raised = repr(exc)
            break
        step_ms = (time.perf_counter() - t_step) * 1000.0
        twist = command.as_tuple()
        poses.append((x, y, yaw))
        commands.append(twist)
        trace_rows.append([t, leg, int(door_shut), x, y, yaw, *twist, gap,
                           int(obs.contact), step_ms, None, None])
        if t >= next_load:
            load_samples.append(os.getloadavg()[0])
            next_load += 5.0
        # ⚠️ THE FRAME FLAG HAS TO REACH drive(). `Episode.drive` defaults to
        # render=False and only calls world.render() when told to, so driving
        # without it draws NOTHING, recorder.capture() captures an empty buffer
        # and ffmpeg is handed zero frames -- which writes no file at all while
        # still reporting a path. Same order as scripts/run_episode.py: aim the
        # camera, drive with the flag, then capture.
        want_frame = recorder is not None and t >= next_frame
        if want_frame:
            recorder.follow(np.array(pose[:3]))
        episode.drive(command.as_tuple(), render=want_frame or args.gui)
        if want_frame:
            recorder.capture()
            next_frame += 1.0 / args.fps

        tx, ty = LEG_TARGETS[leg]
        if math.hypot(x - tx, y - ty) <= ARRIVE_RADIUS:
            leg_times.append(round(t - leg_start, 2))
            log(f"t={t:6.1f}s  LEG {leg} COMPLETE in {leg_times[-1]}s")
            leg += 1
            leg_start = t
        t += dt

    if recorder is not None:
        recorder.close()
        log(f"video: {recorder.out_path}")
    log(f"RESULT {os.path.basename(args.controller)}: "
        f"legs {leg}/{len(LEG_TARGETS)}  times {leg_times}  t={t:.1f}s")

    # ---- Exp 3: the trace and the summary, all after the episode.
    end = episode.pose()
    poses.append((end[0], end[1], end[3]))
    for row, (ev, ew) in zip(trace_rows,
                             exp3.tracking_errors(poses, commands, dt)):
        row[12], row[13] = ev, ew
    if args.trace:
        trace_path = os.path.abspath(args.trace)
        os.makedirs(os.path.dirname(trace_path), exist_ok=True)
        exp3.write_trace(trace_path, trace_rows)
        log(f"trace: {trace_path} ({len(trace_rows)} steps)")
    load_samples.append(os.getloadavg()[0])
    summary = {
        "controller": args.controller, "controller_sha": controller_sha,
        "seed": args.seed,
        "start_offset": [args.start_dx, args.start_dy, args.start_dyaw],
        "spawn": [round(v, 4) for v in SPAWN],
        "settled": [round(settled[0], 4), round(settled[1], 4),
                    round(settled[3], 4)],
        "legs": leg, "total": len(LEG_TARGETS),
        "success": exp3.is_success(leg), "leg_times": leg_times,
        "sim_seconds": round(t, 1), "raised": raised,
        "control_dt": dt,
        **exp3.summarise(trace_rows, dt),
        "load1_start": round(load_samples[0], 2),
        "load1_mean": round(sum(load_samples) / len(load_samples), 2),
        "load1_max": round(max(load_samples), 2),
        "wall_seconds_in_process": round(time.time() - wall0, 1),
        "trace": args.trace,
        "door_never_shuts": args.door_never_shuts,
        "cpu_max": exp_ad.cgroup_cpu_max(),
    }
    print(exp3.RESULT_JSON_PREFIX + json.dumps(summary), flush=True)
    sim_app.close()


if __name__ == "__main__":
    main()
