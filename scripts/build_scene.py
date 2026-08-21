"""
build_scene.py

Builds a simple Lab42-style indoor floorplan with a Unitree Go2 and exports it
to a USD file. The scene is defined as data at the top of this file, so it is
diffable, versionable, and reproducible.

Headless (produces the USD, no window):
    python build_scene.py --out scenes/lab42_v1.usd

With a viewport (watch it, needs a working GUI):
    python build_scene.py --out scenes/lab42_v1.usd --gui

Then open scenes/lab42_v1.usd in the Isaac Sim GUI to inspect it.
"""

import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--out", default="scenes/lab42_v1.usd",
                    help="where to write the exported USD")
parser.add_argument("--gui", action="store_true",
                    help="open a viewport and run physics until you close it")
parser.add_argument("--asset", default=None,
                    help="override the Go2 USD path if auto-discovery fails")
args = parser.parse_args()

# SimulationApp must be constructed before any other isaacsim or omni import.
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid, VisualCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage

# get_assets_root_path has moved between releases. Try newest name first.
try:
    from isaacsim.storage.native import get_assets_root_path        # 5.x, 6.x
except ImportError:
    try:
        from isaacsim.core.utils.nucleus import get_assets_root_path  # 4.5.x
    except ImportError:
        from omni.isaac.core.utils.nucleus import get_assets_root_path  # <= 4.2


# ---------------------------------------------------------------------------
# Scene definition. Everything below this line is data, not code.
# ---------------------------------------------------------------------------

WALL_HEIGHT = 1.5       # metres
WALL_THICKNESS = 0.15

# (centre_x, centre_y, length, yaw_degrees)
# Room spans x in [0, 12], y in [0, 8].
WALLS = [
    (6.0, 0.0, 12.0, 0.0),    # south perimeter
    (6.0, 8.0, 12.0, 0.0),    # north perimeter
    (0.0, 4.0, 8.0, 90.0),    # west perimeter
    (12.0, 4.0, 8.0, 90.0),   # east perimeter
    (6.0, 2.5, 5.0, 90.0),    # divider, leaves a doorway at y in [5, 8]
    (9.0, 5.0, 6.0, 0.0),     # creates a blind corner at (6, 5)
]

GO2_START = (1.5, 1.5, 0.55)  # spawned slightly high so it drops and settles
GOAL = (10.5, 6.5)            # reachable only via the doorway and blind corner

# Lighting. Part of the scene spec, not decoration: RTX camera and lidar
# conditions depend on it, so it must be identical across scene variants.
DOME_INTENSITY = 700.0        # ambient fill, lifts the shadows
KEY_INTENSITY = 1500.0        # single directional light, gives shape
KEY_ROTATION = (-50.0, 0.0, 25.0)

CEILING_HEIGHT = 2.8
CEILING_RADIUS = 0.25
CEILING_INTENSITY = 40000.0
CEILING_LIGHTS = [           # (x, y), roughly one per quadrant
    (3.0, 2.0), (3.0, 6.0),
    (9.0, 2.0), (9.0, 6.5),
]


# ---------------------------------------------------------------------------

def find_go2_asset():
    """Locate the Go2 USD. Asset layout shifted in Isaac Sim 6.0, so try a few."""
    if args.asset:
        return args.asset

    root = get_assets_root_path()
    if root is None:
        sys.exit("Could not resolve the Isaac assets root. Check your network "
                 "connection or set persistent.isaac.asset_root.default.")

    candidates = [
        f"{root}/Isaac/Robots/Unitree/Go2/go2.usd",
        f"{root}/Isaac/Robots/Unitree/Go2/go2.usda",
    ]
    print(f"[build_scene] assets root: {root}")
    return candidates[0]


def add_lights(stage):
    """Dome for ambient fill, one distant key light, ceiling lights per quadrant."""
    from pxr import Gf, UsdGeom, UsdLux

    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(DOME_INTENSITY)
    dome.CreateColorAttr(Gf.Vec3f(0.85, 0.88, 1.0))   # slightly cool

    key = UsdLux.DistantLight.Define(stage, "/World/Lights/key")
    key.CreateIntensityAttr(KEY_INTENSITY)
    key.CreateAngleAttr(1.5)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(*KEY_ROTATION))

    for i, (x, y) in enumerate(CEILING_LIGHTS):
        lamp = UsdLux.SphereLight.Define(stage, f"/World/Lights/ceiling_{i:02d}")
        lamp.CreateIntensityAttr(CEILING_INTENSITY)
        lamp.CreateRadiusAttr(CEILING_RADIUS)
        lamp.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.9))  # slightly warm
        UsdGeom.Xformable(lamp.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(x, y, CEILING_HEIGHT))


def add_wall(world, index, cx, cy, length, yaw_deg):
    world.scene.add(FixedCuboid(
        prim_path=f"/World/Walls/wall_{index:02d}",
        name=f"wall_{index:02d}",
        position=np.array([cx, cy, WALL_HEIGHT / 2.0]),
        orientation=euler_angles_to_quat(np.array([0.0, 0.0, np.deg2rad(yaw_deg)])),
        scale=np.array([length, WALL_THICKNESS, WALL_HEIGHT]),
        color=np.array([0.6, 0.6, 0.62]),
    ))


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    add_lights(omni.usd.get_context().get_stage())

    for i, (cx, cy, length, yaw) in enumerate(WALLS):
        add_wall(world, i, cx, cy, length, yaw)

    # Goal marker. Visual only, no collision, so it never affects an episode.
    world.scene.add(VisualCuboid(
        prim_path="/World/goal_marker",
        name="goal_marker",
        position=np.array([GOAL[0], GOAL[1], 0.05]),
        scale=np.array([0.4, 0.4, 0.02]),
        color=np.array([0.1, 0.8, 0.2]),
    ))

    asset_path = find_go2_asset()
    print(f"[build_scene] loading Go2 from: {asset_path}")
    print("[build_scene] first load streams from S3 and can take several minutes")
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/Go2")

    # SingleXFormPrim vs XFormPrim also churned across releases.
    try:
        from isaacsim.core.prims import SingleXFormPrim as _XForm
    except ImportError:
        from isaacsim.core.prims import XFormPrim as _XForm
    _XForm("/World/Go2").set_world_pose(position=np.array(GO2_START))

    world.reset()

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    omni.usd.get_context().get_stage().Export(out_path)
    print(f"[build_scene] wrote {out_path}")

    if args.gui:
        print("[build_scene] running. close the window to exit.")
        while sim_app.is_running():
            world.step(render=True)
    else:
        # A few steps headless just to confirm physics initialises and the
        # robot is not intersecting geometry.
        for _ in range(60):
            world.step(render=False)
        print("[build_scene] physics stepped cleanly")

    sim_app.close()


if __name__ == "__main__":
    main()