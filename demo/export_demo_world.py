"""
demo/export_demo_world.py

THE DEMO WORLD -> USD, with the door as a prim that can be toggled at runtime.

Modelled on `scripts/export_world.py` and importing lab42's lighting, wall
height and wall colour from `core/scene_spec.py` for the same reason that file
gives: **lighting is spec, not decoration**, and a scene carrying its own lights
is not comparable with any other scene in this project.

⚠️ **THE DOOR IS ONE NAMED PRIM, `/World/Door`.** Everything else is
`/World/Walls/block_NNN`. `demo/record_isaac.py` drops the door's collision and
visibility for legs 0-1 and restores them for legs 2-3, so a single USD carries
both configurations and the two are guaranteed to differ by exactly the rect
that `demo/world.py` says they differ by.

⚠️ NOT a benchmark scene. It is hand-authored (see demo/README.md) and appears
in no split, no PRE-REGISTRATION constant and no committed result.

    python demo/export_demo_world.py
    # -> scenes/demo-patrol.usd
"""
import argparse
import os
import sys

parser = argparse.ArgumentParser()
parser.add_argument("--out", default=os.path.join("scenes", "demo-patrol.usd"))
parser.add_argument("--gui", action="store_true")
parser.add_argument("--asset", default=None)
args = parser.parse_args()

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from core.scene_spec import (
    WALL_HEIGHT, DOME_INTENSITY, KEY_INTENSITY, KEY_ROTATION,
    CEILING_HEIGHT, CEILING_RADIUS, CEILING_INTENSITY, CEILING_LIGHTS,
    GO2_START,
)
from demo.world import OPEN_WORLD, DOOR, HOME, GOAL

from isaacsim import SimulationApp
sim_app = SimulationApp({"headless": not args.gui})

import numpy as np
import omni.usd
from isaacsim.core.api import World as SimWorld
from isaacsim.core.api.objects import FixedCuboid, VisualCuboid
from isaacsim.core.utils.stage import add_reference_to_stage

try:
    from isaacsim.storage.native import get_assets_root_path
except ImportError:
    from isaacsim.core.utils.nucleus import get_assets_root_path

# ---- below this line is code; the scene DATA is core/scene_spec.py + demo/world.py

WALL_COLOUR = np.array([0.6, 0.6, 0.62])
DOOR_COLOUR = np.array([0.75, 0.15, 0.15])       # so the video reads at a glance


def add_lights(stage):
    """lab42's lighting, verbatim."""
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


def add_block(sim, path, name, rect, colour):
    x0, y0, x1, y1 = rect
    sim.scene.add(FixedCuboid(
        prim_path=path, name=name,
        position=np.array([0.5 * (x0 + x1), 0.5 * (y0 + y1), WALL_HEIGHT / 2.0]),
        scale=np.array([max(x1 - x0, 1e-3), max(y1 - y0, 1e-3), WALL_HEIGHT]),
        color=colour))


def main():
    sim = SimWorld(stage_units_in_meters=1.0)
    sim.scene.add_default_ground_plane()
    add_lights(omni.usd.get_context().get_stage())

    rects = list(OPEN_WORLD.rects)
    print(f"[export_demo] {len(rects)} static rects + 1 door")
    for i, r in enumerate(rects):
        add_block(sim, f"/World/Walls/block_{i:03d}", f"block_{i:03d}", r, WALL_COLOUR)

    # THE DOOR. One prim, named, toggled at runtime by demo/record_isaac.py.
    add_block(sim, "/World/Door", "door", DOOR, DOOR_COLOUR)

    # Markers. Visual only, no collision, so they never affect an episode.
    for name, (mx, my), colour in (("goal_marker", GOAL, (0.1, 0.8, 0.2)),
                                   ("home_marker", HOME, (0.2, 0.45, 0.95))):
        sim.scene.add(VisualCuboid(
            prim_path=f"/World/{name}", name=name,
            position=np.array([mx, my, 0.05]),
            scale=np.array([0.4, 0.4, 0.02]), color=np.array(colour)))

    asset = args.asset
    if asset is None:
        root = get_assets_root_path()
        if root is None:
            sys.exit("[export_demo] could not resolve the Isaac assets root.")
        asset = f"{root}/Isaac/Robots/Unitree/Go2/go2.usd"
    print(f"[export_demo] loading Go2 from: {asset}")
    add_reference_to_stage(usd_path=asset, prim_path="/World/Go2")

    try:
        from isaacsim.core.prims import SingleXFormPrim as _XForm
    except ImportError:
        from isaacsim.core.prims import XFormPrim as _XForm
    # lab42's spawn HEIGHT, the demo world's spawn POSITION -- the drop-and-
    # settle the runner depends on is a property of the height.
    _XForm("/World/Go2").set_world_pose(
        position=np.array([HOME[0], HOME[1], GO2_START[2]]))

    sim.reset()
    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    omni.usd.get_context().get_stage().Export(out_path)
    print(f"[export_demo] wrote {out_path}")

    if args.gui:
        while sim_app.is_running():
            sim.step(render=True)
    else:
        for _ in range(60):
            sim.step(render=False)
        print("[export_demo] physics stepped cleanly")
    sim_app.close()


if __name__ == "__main__":
    main()
