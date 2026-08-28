"""
drive_go2.py

Loads the scene built by build_scene.py and drives the Go2 using the frozen
locomotion policy that ships with Isaac Sim 6.0.

The only thing this script sends the robot is a Twist: (vx, vy, wz).
That is the frozen interface. Everything above it is your experiment.

    python drive_go2.py --scene scenes/lab42_v1.usd --gui
    python drive_go2.py --scene scenes/lab42_v1.usd --demo waypoints

Check the class name first:
    SP=$(python -c "import site; print(site.getsitepackages()[0])")
    ls "$SP"/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/
"""

import argparse
import inspect
import math
import os
import sys


def parse_bounds(text):
    """(min_x, min_y, max_x, max_y) for --top-bounds. An argparse type, so a
    malformed value is rejected before the simulator spends 20s starting up."""
    if not text:
        return None
    parts = [float(v) for v in text.replace(" ", "").split(",")]
    if len(parts) != 4:
        raise argparse.ArgumentTypeError(
            f"needs four numbers, MINX,MINY,MAXX,MAXY, got {text!r}")
    if parts[2] <= parts[0] or parts[3] <= parts[1]:
        raise argparse.ArgumentTypeError(
            f"max must exceed min on both axes, got {text!r}")
    return tuple(parts)


parser = argparse.ArgumentParser()
parser.add_argument("--scene", default="scenes/lab42_v1.usd")
parser.add_argument("--gui", action="store_true")
parser.add_argument("--demo", choices=["forward", "square", "waypoints"],
                    default="forward")
parser.add_argument("--seconds", type=float, default=30.0)
parser.add_argument("--record", metavar="PATH",
                    help="render an mp4 of the run instead of watching it live. "
                         "Relative paths go under the repo's out/ whatever "
                         "directory you launch from, and a timestamp is "
                         "inserted before the extension, so --record walk "
                         "writes out/walk_20260822-141530.mp4 and earlier runs "
                         "are kept")
parser.add_argument("--fps", type=int, default=30,
                    help="frame rate of the recording")
parser.add_argument("--res", default="1280x720",
                    help="resolution of each pane, WIDTHxHEIGHT. The recording "
                         "is two panes wide unless --no-topdown")
parser.add_argument("--no-topdown", dest="topdown", action="store_false",
                    help="record only the chase camera, without the top-down "
                         "pane to its left")
parser.add_argument("--top-zoom", type=float, default=1.0,
                    help="zoom of the top-down pane. 1.0 fits the whole "
                         "floorplan, higher crops into the middle of it")
parser.add_argument("--top-bounds", metavar="MINX,MINY,MAXX,MAXY",
                    type=parse_bounds,
                    help="frame the top-down pane on these world bounds "
                         "instead of measuring the scene")
parser.add_argument("--focal", type=float, default=24.0,
                    help="recording camera focal length in mm, lower is wider")
parser.add_argument("--cam-offset", default="-1,3,2.4",
                    help="chase camera offset from the robot, in world axes. "
                         "Must keep the camera inside the room, the walls are solid")
parser.add_argument("--no-follow", dest="follow", action="store_false",
                    help="keep the camera still instead of tracking the robot")
args = parser.parse_args()

# Recording exists precisely to avoid the interactive viewport, which renders
# well below real time here.
if args.record and args.gui:
    print("[drive_go2] --record turns off --gui, the viewport is not needed")
    args.gui = False

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.usd
import torch
from isaacsim.core.api import World

# The Go2 policy class. Name may differ; see the ls command in the docstring.
_policy_cls = None
for module_name, class_name in [
    ("isaacsim.robot.policy.examples.robots.go2", "Go2FlatTerrainPolicy"),
    ("isaacsim.robot.policy.examples.robots.unitree", "UnitreeGo2FlatTerrainPolicy"),
    ("isaacsim.robot.policy.examples.robots", "Go2FlatTerrainPolicy"),
]:
    try:
        _policy_cls = getattr(__import__(module_name, fromlist=[class_name]),
                              class_name)
        print(f"[drive_go2] using {module_name}.{class_name}")
        break
    except (ImportError, AttributeError):
        continue

if _policy_cls is None:
    sys.exit("Could not find the Go2 policy class. Run the ls command in the "
             "docstring and set it explicitly.")


# --- simulation rates -------------------------------------------------------
# Match these to the shipped standalone example. Policies are trained at a
# specific control rate and behave badly if you change it.
PHYSICS_DT = 1.0 / 200.0
RENDER_DT = 1.0 / 50.0

GO2_START = (1.5, 1.5, 0.55)
MAX_LIN = 0.8      # m/s, keep modest indoors
MAX_ANG = 1.0      # rad/s


# --- velocity sources -------------------------------------------------------
# Each returns (vx, vy, wz) given elapsed time and current pose.
# These are SCAFFOLDING for smoke-testing the locomotion layer.
# They are NOT navigation and must not appear in any benchmark condition.

def demo_forward(t, pose):
    return (0.5, 0.0, 0.0)


def demo_square(t, pose):
    phase = int(t // 4) % 2
    return (0.5, 0.0, 0.0) if phase == 0 else (0.0, 0.0, 0.8)


WAYPOINTS = [(4.0, 1.5), (5.0, 6.5), (10.5, 6.5)]


def demo_waypoints(t, pose, _state={"i": 0}):
    """Deliberately naive: turn to face the waypoint, then walk at it.

    This is a stand-in so you can confirm the robot goes where told. It has no
    obstacle awareness and will walk into walls. Replacing this with something
    that does not is the entire point of the project.
    """
    i = _state["i"]
    if i >= len(WAYPOINTS):
        return (0.0, 0.0, 0.0)

    (x, y, yaw) = pose
    gx, gy = WAYPOINTS[i]
    dx, dy = gx - x, gy - y
    dist = math.hypot(dx, dy)

    if dist < 0.3:
        _state["i"] = i + 1
        return (0.0, 0.0, 0.0)

    heading_err = math.atan2(dy, dx) - yaw
    heading_err = (heading_err + math.pi) % (2 * math.pi) - math.pi

    if abs(heading_err) > 0.25:
        return (0.0, 0.0, float(np.clip(2.0 * heading_err, -MAX_ANG, MAX_ANG)))
    return (min(MAX_LIN, 0.6 * dist), 0.0,
            float(np.clip(1.0 * heading_err, -MAX_ANG, MAX_ANG)))


SOURCES = {"forward": demo_forward, "square": demo_square,
           "waypoints": demo_waypoints}


def parse_resolution(text):
    width, _, height = text.lower().partition("x")
    return (int(width), int(height))


def parse_offset(text):
    parts = [float(v) for v in text.replace(" ", "").split(",")]
    if len(parts) != 3:
        raise ValueError(f"--cam-offset needs three numbers, got {text!r}")
    return tuple(parts)



def make_command(policy):
    """Allocate the command tensor the policy consumes, once.

    forward() writes the command straight into a torch observation tensor, so
    passing a numpy array raises TypeError. The shipped examples keep one
    persistent tensor on the robot's device and overwrite it in place, which
    also avoids allocating on every physics step.
    """
    device = torch.device(str(policy.robot._device))
    return torch.zeros(3, dtype=torch.float32, device=device)


def base_pose(policy):
    """Return (x, y, yaw) of the robot base."""
    robot = policy.robot
    if hasattr(robot, "get_world_poses"):
        # 6.0 articulations are batched and return warp arrays; take row 0.
        positions, quats = robot.get_world_poses()
        pos = np.asarray(positions.numpy())[0]
        quat = np.asarray(quats.numpy())[0]
    else:
        pos, quat = robot.get_world_pose()
    w, x, y, z = quat
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (float(pos[0]), float(pos[1]), yaw)


def main():
    scene_path = os.path.abspath(args.scene)
    if not os.path.exists(scene_path):
        sys.exit(f"Scene not found: {scene_path}. Run build_scene.py first.")

    omni.usd.get_context().open_stage(scene_path)
    world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDER_DT,
                  stage_units_in_meters=1.0)

    # The exported scene already has a /World/Go2 reference. Remove it so the
    # policy controller can own the articulation cleanly.
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath("/World/Go2"):
        stage.RemovePrim("/World/Go2")

    # The constructor signature differs between releases, so pass only the
    # arguments this build actually accepts.
    sig = inspect.signature(_policy_cls.__init__)
    accepts_kwargs = any(p.kind is inspect.Parameter.VAR_KEYWORD
                         for p in sig.parameters.values())

    candidate = {
        "prim_path": "/World/Go2",
        "name": "Go2",
        "position": np.array(GO2_START),
    }
    kwargs = candidate if accepts_kwargs else {
        k: v for k, v in candidate.items() if k in sig.parameters
    }
    dropped = sorted(set(candidate) - set(kwargs))
    if dropped:
        print(f"[drive_go2] constructor does not take {dropped}, omitting")

    policy = _policy_cls(**kwargs)

    # If position was not a constructor argument, place the robot afterwards.
    if "position" not in kwargs:
        try:
            policy.robot.set_world_pose(position=np.array(GO2_START))
            print(f"[drive_go2] start pose set post-construction to {GO2_START}")
        except Exception as exc:
            print(f"[drive_go2] could not set start pose: {exc}")

    world.reset()
    policy.initialize()

    recorder = None
    if args.record:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from recorder import Recorder
        recorder = Recorder(args.record,
                            resolution=parse_resolution(args.res),
                            fps=args.fps,
                            focal_length=args.focal,
                            top_down=args.topdown,
                            top_zoom=args.top_zoom,
                            scene_bounds=args.top_bounds)

    velocity_source = SOURCES[args.demo]
    command = make_command(policy)
    steps = int(args.seconds / PHYSICS_DT)
    cam_offset = parse_offset(args.cam_offset)
    next_frame = 0.0
    t = 0.0

    for step in range(steps):
        if args.gui and not sim_app.is_running():
            break

        pose = base_pose(policy)
        vx, vy, wz = velocity_source(t, pose)
        command[0], command[1], command[2] = vx, vy, wz

        # ---- the frozen interface. Nothing below this line is experimental.
        policy.forward(PHYSICS_DT, command)

        # Rendering is the expensive part, so only do it on the steps that
        # actually produce a frame.
        want_frame = recorder is not None and t >= next_frame
        if want_frame and args.follow:
            positions, _ = policy.robot.get_world_poses()
            recorder.follow(np.asarray(positions.numpy())[0], offset=cam_offset)

        world.step(render=want_frame or args.gui)

        if want_frame:
            recorder.capture()
            next_frame += 1.0 / args.fps

        t += PHYSICS_DT

        if step % 400 == 0:
            print(f"t={t:5.1f}s  pose=({pose[0]:5.2f}, {pose[1]:5.2f}, "
                  f"{math.degrees(pose[2]):6.1f}deg)  cmd=({vx:.2f}, {vy:.2f}, {wz:.2f})")

    if recorder is not None:
        recorder.close()

    print(f"[drive_go2] finished at {base_pose(policy)}")
    sim_app.close()


if __name__ == "__main__":
    main()