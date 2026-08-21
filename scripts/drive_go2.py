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
import math
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--scene", default="scenes/lab42_v1.usd")
parser.add_argument("--gui", action="store_true")
parser.add_argument("--demo", choices=["forward", "square", "waypoints"],
                    default="forward")
parser.add_argument("--seconds", type=float, default=30.0)
args = parser.parse_args()

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.usd
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


def base_pose(policy):
    """Return (x, y, yaw) of the robot base."""
    pos, quat = policy.robot.get_world_pose()
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

    policy = _policy_cls(
        prim_path="/World/Go2",
        name="Go2",
        position=np.array(GO2_START),
    )

    world.reset()
    policy.initialize()

    velocity_source = SOURCES[args.demo]
    steps = int(args.seconds / PHYSICS_DT)
    t = 0.0

    for step in range(steps):
        if args.gui and not sim_app.is_running():
            break

        pose = base_pose(policy)
        vx, vy, wz = velocity_source(t, pose)

        # ---- the frozen interface. Nothing below this line is experimental.
        policy.forward(PHYSICS_DT, np.array([vx, vy, wz]))

        world.step(render=args.gui)
        t += PHYSICS_DT

        if step % 400 == 0:
            print(f"t={t:5.1f}s  pose=({pose[0]:5.2f}, {pose[1]:5.2f}, "
                  f"{math.degrees(pose[2]):6.1f}deg)  cmd=({vx:.2f}, {vy:.2f}, {wz:.2f})")

    print(f"[drive_go2] finished at {base_pose(policy)}")
    sim_app.close()


if __name__ == "__main__":
    main()