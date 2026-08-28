"""
walk_forward.py

Smoke test for the frozen locomotion interface. Loads the exported lab42 scene,
spawns the Go2 with its pretrained flat-terrain policy, lets it settle, then
walks it forward in a straight line.

This proves exactly one thing: that policy.forward(dt, [vx, vy, wz]) produces a
stable gait on this machine. It is not navigation and must never appear in a
benchmark condition.

    python walk_forward.py                      # GUI, 0.5 m/s forward
    python walk_forward.py --headless           # no window, pose to stdout
    python walk_forward.py --vx 0.3 --seconds 6


    python walk_forward.py --record 1 --vx 0.5 --seconds 12

The robot starts at (1.5, 1.5) and walks toward +x. The divider wall sits at
x = 6.0, so at the default speed and duration it stops roughly 0.3 m short of
it. Raise --seconds and it will walk into the wall, which is expected.
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
parser.add_argument("--headless", action="store_true",
                    help="no viewport, just pose printed to stdout")
parser.add_argument("--vx", type=float, default=0.5,
                    help="forward speed in m/s once settled")
parser.add_argument("--seconds", type=float, default=9.0,
                    help="total sim seconds, including the settle phase")
parser.add_argument("--settle", type=float, default=1.5,
                    help="seconds of zero command to let the drop absorb")
parser.add_argument("--no-hold", dest="hold", action="store_false",
                    help="close the GUI immediately instead of holding it open")
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
# well below real time here. Holding a window open would defeat that.
if args.record and not args.headless:
    print("[walk] --record implies --headless, the viewport is not needed")
    args.headless = True

# SimulationApp must be constructed before any other isaacsim or omni import.
from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": args.headless})

import numpy as np
import omni.usd
import torch
from isaacsim.core.api import World

try:
    from isaacsim.core.utils.viewports import set_camera_view
except ImportError:
    set_camera_view = None

# Policy class location has moved between releases.
_policy_cls = None
for module_name, class_name in [
    ("isaacsim.robot.policy.examples.robots.go2", "Go2FlatTerrainPolicy"),
    ("isaacsim.robot.policy.examples.robots.unitree", "UnitreeGo2FlatTerrainPolicy"),
    ("isaacsim.robot.policy.examples.robots", "Go2FlatTerrainPolicy"),
]:
    try:
        _policy_cls = getattr(__import__(module_name, fromlist=[class_name]),
                              class_name)
        print(f"[walk] using {module_name}.{class_name}")
        break
    except (ImportError, AttributeError):
        continue

if _policy_cls is None:
    sim_app.close()
    sys.exit("Could not find the Go2 policy class. Check the shipped "
             "isaacsim.robot.policy.examples.robots package and set it "
             "explicitly.")


# --- constants --------------------------------------------------------------

GO2_START = (1.5, 1.5, 0.40)   # nominal stance is ~0.32, so this is a short drop
FALLBACK_PHYSICS_DT = 1.0 / 200.0
FALLBACK_DECIMATION = 4
FALL_HEIGHT = 0.15             # base below this means the policy lost the gait
LOG_EVERY = 0.5                # sim seconds between pose prints


def parse_resolution(text):
    width, _, height = text.lower().partition("x")
    return (int(width), int(height))


def parse_offset(text):
    parts = [float(v) for v in text.replace(" ", "").split(",")]
    if len(parts) != 3:
        raise ValueError(f"--cam-offset needs three numbers, got {text!r}")
    return tuple(parts)



def make_command(vx, vy, wz, policy):
    """Build a command the policy can consume.

    forward() writes the command straight into a torch observation tensor, so a
    numpy array raises TypeError. It has to be a torch tensor on the robot's
    own device.
    """
    device = torch.device(str(policy.robot._device))
    return torch.tensor([vx, vy, wz], dtype=torch.float32, device=device)


def base_pose(policy):
    """Return (x, y, z, yaw) of the robot base. Isaac quats are (w, x, y, z)."""
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
    return (float(pos[0]), float(pos[1]), float(pos[2]), yaw)


def main():
    scene_path = os.path.abspath(args.scene)
    if not os.path.exists(scene_path):
        sys.exit(f"Scene not found: {scene_path}. Run build_scene.py first.")

    print(f"[walk] opening {scene_path}")
    omni.usd.get_context().open_stage(scene_path)
    for _ in range(20):
        sim_app.update()

    # World is a singleton. Clear it so the dt arguments below actually apply.
    World.clear_instance()
    world = World(physics_dt=FALLBACK_PHYSICS_DT,
                  rendering_dt=FALLBACK_PHYSICS_DT * FALLBACK_DECIMATION,
                  stage_units_in_meters=1.0)

    # If the exported scene already contains a Go2 reference, drop it. The
    # policy controller needs to own the articulation itself.
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath("/World/Go2"):
        print("[walk] removing pre-existing /World/Go2 from the scene")
        stage.RemovePrim("/World/Go2")
        for _ in range(5):
            sim_app.update()

    # The constructor signature differs between releases, so pass only the
    # arguments this build actually accepts.
    sig = inspect.signature(_policy_cls.__init__)
    print(f"[walk] policy signature: {sig}")
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
        print(f"[walk] constructor does not take {dropped}, omitting")

    print("[walk] constructing policy, first run downloads the checkpoint")
    policy = _policy_cls(**kwargs)

    # If position was not a constructor argument, place the robot afterwards.
    if "position" not in kwargs:
        try:
            policy.robot.set_world_pose(position=np.array(GO2_START))
            print(f"[walk] start pose set post-construction to {GO2_START}")
        except Exception as exc:
            print(f"[walk] could not set start pose: {exc}")

    # Take the control rates from the policy's own env config rather than
    # hardcoding them. A policy trained at one rate misbehaves at another.
    physics_dt = float(getattr(policy, "_dt", FALLBACK_PHYSICS_DT))
    decimation = int(getattr(policy, "_decimation", FALLBACK_DECIMATION))
    control_hz = 1.0 / (physics_dt * decimation)
    print(f"[walk] physics dt {physics_dt:.4f}s, decimation {decimation}, "
          f"control rate {control_hz:.1f} Hz")
    world.set_simulation_dt(physics_dt=physics_dt,
                            rendering_dt=physics_dt * decimation)

    if set_camera_view is not None and not args.headless:
        set_camera_view(eye=[GO2_START[0] - 3.0, GO2_START[1] - 3.0, 2.5],
                        target=[GO2_START[0] + 2.0, GO2_START[1], 0.3])

    world.reset()
    policy.initialize()

    recorder = None
    if args.record:
        sys.path.insert(0, os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))))
        from recorder import Recorder
        recorder = Recorder(args.record,
                            resolution=parse_resolution(args.res),
                            fps=args.fps,
                            focal_length=args.focal,
                            top_down=args.topdown,
                            top_zoom=args.top_zoom,
                            scene_bounds=args.top_bounds)

    state = {"t": 0.0, "fallen": False}

    # Built once. Allocating a tensor per physics step would cost more than the
    # policy itself at 200 Hz.
    zero_command = make_command(0.0, 0.0, 0.0, policy)
    walk_command = make_command(args.vx, 0.0, 0.0, policy)

    def on_physics_step(step_size):
        """Called once per physics step. This is the only place a command is sent."""
        command = zero_command if state["t"] < args.settle else walk_command

        # ---- the frozen interface. Nothing below this line is experimental.
        policy.forward(step_size, command)
        state["t"] += step_size

    world.add_physics_callback("go2_control", callback_fn=on_physics_step)

    print(f"[walk] settling for {args.settle:.1f}s, then {args.vx:.2f} m/s forward")
    next_log = 0.0
    next_frame = 0.0
    cam_offset = parse_offset(args.cam_offset)

    while sim_app.is_running() and state["t"] < args.seconds:
        # Rendering is the expensive part, so only do it on the physics steps
        # that actually produce a frame.
        want_frame = recorder is not None and state["t"] >= next_frame
        if want_frame and args.follow:
            x, y, z, _ = base_pose(policy)
            recorder.follow((x, y, z), offset=cam_offset)

        world.step(render=want_frame or not args.headless)

        if want_frame:
            recorder.capture()
            next_frame += 1.0 / args.fps

        if state["t"] >= next_log:
            x, y, z, yaw = base_pose(policy)
            print(f"t={state['t']:5.2f}s  x={x:6.2f} y={y:6.2f} z={z:5.3f} "
                  f"yaw={math.degrees(yaw):7.1f}deg")
            next_log += LOG_EVERY

            if state["t"] > args.settle + 0.5 and z < FALL_HEIGHT and not state["fallen"]:
                state["fallen"] = True
                print(f"[walk] WARNING base height {z:.3f} is below "
                      f"{FALL_HEIGHT}. The policy has lost the gait.")

    if recorder is not None:
        recorder.close()

    x, y, z, yaw = base_pose(policy)
    travelled = math.hypot(x - GO2_START[0], y - GO2_START[1])
    print(f"[walk] finished at x={x:.2f} y={y:.2f} z={z:.3f} "
          f"yaw={math.degrees(yaw):.1f}deg")
    print(f"[walk] travelled {travelled:.2f} m in {state['t']:.1f}s "
          f"({'FELL' if state['fallen'] else 'stayed up'})")

    if args.hold and not args.headless and sim_app.is_running():
        print("[walk] holding the viewport open with a zero command. "
              "Close the window to exit.")
        world.remove_physics_callback("go2_control")

        def hold_step(step_size):
            policy.forward(step_size, zero_command)

        world.add_physics_callback("go2_hold", callback_fn=hold_step)
        while sim_app.is_running():
            world.step(render=True)

    sim_app.close()


if __name__ == "__main__":
    main()