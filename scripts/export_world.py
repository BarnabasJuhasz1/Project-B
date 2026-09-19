"""
export_world.py

A `core.worldgen` generated world -> a USD `run_episode.py` can load, so that a
growth-suite world can be run under physics and recorded.

⚠️ **The lighting, wall height, wall colour and goal marker are lab42's, taken
from `core/scene_spec.py` and NOT redeclared here.** CLAUDE.md: "Lighting is
part of the scene spec, not decoration. RTX camera and lidar conditions depend
on it, so it must be identical across scene variants." A generated world with
its own lights would not be comparable with lab42 or with another generated
world, and the difference would be invisible in every number.

What DOES differ from build_scene.py, necessarily:

  - Obstacles come from `world.rects` — axis-aligned (x0, y0, x1, y1) boxes of
    arbitrary footprint — rather than from `WALLS`' (cx, cy, length, yaw)
    centrelines. A generated obstacle is a block, not a thin wall, so its
    thickness is its own, not `WALL_THICKNESS`.
  - Start and goal come from the generated world, not from `GO2_START`/`GOAL`.
  - The Go2 spawns at the scenario start, at lab42's spawn height, so it drops
    and settles the same way.

The written USD is named after the world, so `(tier, seed)` still names the
environment completely:

    python export_world.py --tier maze --seed 2004
    # -> scenes/generated/maze-02004.usd

    python scripts/run_episode.py --scenario gen-maze-02004 \
        --controller layer:gen_b_44d12df4d25b437f --max-seconds 120

⚠️ NOT a benchmark scene. The generated suite is not sealed, not in
`PRE-REGISTRATION.md`, and `all_scenarios()` deliberately never returns one.
"""

import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--tier", default="maze")
parser.add_argument("--seed", type=int, required=True)
parser.add_argument("--out", default=None,
                    help="default scenes/generated/<tier>-<seed>.usd")
parser.add_argument("--gui", action="store_true",
                    help="open a viewport and run physics until you close it")
parser.add_argument("--asset", default=None,
                    help="override the Go2 USD path if auto-discovery fails")
args = parser.parse_args()

# Sim-free, so it is safe above the SimulationApp line — same derivation as
# build_scene.py.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core.scene_spec import (
    WALL_HEIGHT, DOME_INTENSITY, KEY_INTENSITY, KEY_ROTATION,
    CEILING_HEIGHT, CEILING_RADIUS, CEILING_INTENSITY, CEILING_LIGHTS,
    GO2_START,
)
from core.worldgen import generate, to_scenario

_gen = generate(args.tier, args.seed)
_scenario = to_scenario(_gen)
_out = args.out or os.path.join("scenes", "generated", f"{_gen.world.name}.usd")

# SimulationApp must be constructed before any other isaacsim or omni import.
from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid, VisualCuboid
from isaacsim.core.utils.stage import add_reference_to_stage

try:
    from isaacsim.storage.native import get_assets_root_path        # 5.x, 6.x
except ImportError:
    try:
        from isaacsim.core.utils.nucleus import get_assets_root_path  # 4.5.x
    except ImportError:
        from omni.isaac.core.utils.nucleus import get_assets_root_path  # <= 4.2

# ---------------------------------------------------------------------------
# Everything below this line is code; the scene DATA is core/scene_spec.py and
# core/worldgen.py, deliberately.

WALL_COLOUR = np.array([0.6, 0.6, 0.62])     # build_scene.py's, matched


def find_go2_asset():
    """Locate the Go2 USD. Same ladder as build_scene.py — see its docstring."""
    if args.asset:
        return args.asset
    root = get_assets_root_path()
    if root is None:
        sys.exit("[export_world] could not resolve the Isaac assets root.")
    print(f"[export_world] assets root: {root}")
    return f"{root}/Isaac/Robots/Unitree/Go2/go2.usd"


def add_lights(stage):
    """lab42's lighting, verbatim. See the module docstring for why."""
    from pxr import Gf, UsdGeom, UsdLux

    dome = UsdLux.DomeLight.Define(stage, "/World/Lights/dome")
    dome.CreateIntensityAttr(DOME_INTENSITY)
    dome.CreateColorAttr(Gf.Vec3f(0.85, 0.88, 1.0))

    key = UsdLux.DistantLight.Define(stage, "/World/Lights/key")
    key.CreateIntensityAttr(KEY_INTENSITY)
    key.CreateAngleAttr(1.5)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(*KEY_ROTATION))

    for i, (x, y) in enumerate(CEILING_LIGHTS):
        lamp = UsdLux.SphereLight.Define(stage, f"/World/Lights/ceiling_{i:02d}")
        lamp.CreateIntensityAttr(CEILING_INTENSITY)
        lamp.CreateRadiusAttr(CEILING_RADIUS)
        lamp.CreateColorAttr(Gf.Vec3f(1.0, 0.96, 0.9))
        UsdGeom.Xformable(lamp.GetPrim()).AddTranslateOp().Set(
            Gf.Vec3d(x, y, CEILING_HEIGHT))


def add_block(world, index, x0, y0, x1, y1):
    """One generated rect as a fixed collider.

    Axis-aligned, so no rotation: `core.geometry`'s raycast and occupancy grid
    treat `world.rects` as axis-aligned boxes, and a USD block at any other yaw
    would be a different world from the one every sim-free number was computed
    on.
    """
    cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
    sx, sy = max(x1 - x0, 1e-3), max(y1 - y0, 1e-3)
    world.scene.add(FixedCuboid(
        prim_path=f"/World/Walls/block_{index:03d}",
        name=f"block_{index:03d}",
        position=np.array([cx, cy, WALL_HEIGHT / 2.0]),
        scale=np.array([sx, sy, WALL_HEIGHT]),
        color=WALL_COLOUR,
    ))


def main():
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()
    add_lights(omni.usd.get_context().get_stage())

    rects = list(_gen.world.rects)
    print(f"[export_world] {_gen.world.name}: {len(rects)} rects  "
          f"{_gen.difficulty}")
    for i, (x0, y0, x1, y1) in enumerate(rects):
        add_block(world, i, x0, y0, x1, y1)

    # Goal marker. Visual only, no collision, so it never affects an episode —
    # build_scene.py's rule and its dimensions.
    gx, gy = _scenario.goal
    world.scene.add(VisualCuboid(
        prim_path="/World/goal_marker", name="goal_marker",
        position=np.array([gx, gy, 0.05]),
        scale=np.array([0.4, 0.4, 0.02]),
        color=np.array([0.1, 0.8, 0.2]),
    ))

    asset_path = find_go2_asset()
    print(f"[export_world] loading Go2 from: {asset_path}")
    add_reference_to_stage(usd_path=asset_path, prim_path="/World/Go2")

    try:
        from isaacsim.core.prims import SingleXFormPrim as _XForm
    except ImportError:
        from isaacsim.core.prims import XFormPrim as _XForm
    sx, sy, _yaw = _scenario.start
    # lab42's spawn HEIGHT, the generated world's spawn POSITION: the drop-and-
    # settle behaviour the runner depends on is a property of the height.
    _XForm("/World/Go2").set_world_pose(
        position=np.array([sx, sy, GO2_START[2]]))

    world.reset()

    out_path = os.path.abspath(_out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    omni.usd.get_context().get_stage().Export(out_path)
    print(f"[export_world] wrote {out_path}")

    if args.gui:
        print("[export_world] running. close the window to exit.")
        while sim_app.is_running():
            world.step(render=True)
    else:
        for _ in range(60):
            world.step(render=False)
        print("[export_world] physics stepped cleanly")

    sim_app.close()


if __name__ == "__main__":
    main()
