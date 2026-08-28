"""
scripts/go2_policy.py

Sim-side helpers for standing a Go2 up under the frozen locomotion policy.

IMPORT THIS ONLY AFTER SimulationApp EXISTS. It pulls in omni and torch at
import time, exactly like recorder.py, so importing it at the top of a module
defeats the whole ordering rule.

Everything here already existed, duplicated between drive_go2.py and
walk_forward.py. It is lifted rather than written a third time, with the two
things walk_forward.py gets right and drive_go2.py does not:

  * World.clear_instance() before construction, so the dt arguments apply at all
    (World is a singleton; a second World() call silently returns the first)
  * control rate derived from policy._dt / policy._decimation rather than
    hardcoded 200/50 Hz. A policy trained at one rate misbehaves at another.

drive_go2.py and walk_forward.py are deliberately left untouched. They work,
CLAUDE.md records them as locomotion smoke tests, and porting them onto this
module is a separate follow-up.
"""

import inspect
import math
import os

import numpy as np
import omni.kit.app
import omni.usd
import torch
from isaacsim.core.api import World

# Used only if the policy does not expose its own rates.
FALLBACK_PHYSICS_DT = 1.0 / 200.0
FALLBACK_DECIMATION = 4

# The scene ships with a /World/Go2 reference; the policy controller has to own
# the articulation itself, so the reference is removed before construction.
GO2_PRIM = "/World/Go2"


def _pump():
    """One app update. Stage loading and prim removal are asynchronous, so the
    next line cannot assume they finished. Goes through omni.kit.app rather than
    the caller's SimulationApp so this module never has to be handed one."""
    omni.kit.app.get_app().update()


def resolve_policy_class(log=print):
    """Find the Go2 flat-terrain policy. Module path churned across releases.

    On 6.0.1 it is
    isaacsim.robot.policy.examples.robots.go2.Go2FlatTerrainPolicy.
    If this ever fails, list what is actually shipped:

        SP=$(python -c "import site; print(site.getsitepackages()[0])")
        ls "$SP"/isaacsim/exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/
    """
    for module_name, class_name in [
        ("isaacsim.robot.policy.examples.robots.go2", "Go2FlatTerrainPolicy"),
        ("isaacsim.robot.policy.examples.robots.unitree", "UnitreeGo2FlatTerrainPolicy"),
        ("isaacsim.robot.policy.examples.robots", "Go2FlatTerrainPolicy"),
    ]:
        try:
            cls = getattr(__import__(module_name, fromlist=[class_name]), class_name)
            log(f"using {module_name}.{class_name}")
            return cls
        except (ImportError, AttributeError):
            continue
    raise RuntimeError(
        "Could not find the Go2 policy class. Run the ls command in "
        "scripts/go2_policy.py's docstring and set it explicitly.")


def yaw_to_quat(yaw):
    """Isaac quaternions are (w, x, y, z)."""
    return np.array([math.cos(0.5 * yaw), 0.0, 0.0, math.sin(0.5 * yaw)])


def make_command(policy):
    """Allocate the command tensor the policy consumes, once.

    forward() writes the command straight into a torch observation tensor, so a
    numpy array raises TypeError. The shipped examples keep one persistent
    tensor on the robot's own device and overwrite it in place, which also
    avoids allocating on every physics step — at 200 Hz that costs more than
    the policy does.
    """
    return torch.zeros(3, dtype=torch.float32,
                       device=torch.device(str(policy.robot._device)))


def base_pose(policy):
    """(x, y, z, yaw) of the robot base."""
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


class Episode:
    """A stood-up robot on an open stage, with the rates it wants to be driven at."""

    def __init__(self, world, policy, physics_dt, decimation):
        self.world = world
        self.policy = policy
        self.physics_dt = physics_dt
        self.decimation = decimation
        self.command = make_command(policy)

    @property
    def control_dt(self):
        return self.physics_dt * self.decimation

    def drive(self, twist, render=False):
        """Hold one Twist for a whole control step.

        The policy is a *locomotion* policy: it runs at physics rate and expects
        the command to be held across the decimation window, not re-decided
        every physics step.

        Physics always steps with render=False, and a frame is drawn afterwards
        by world.render(). world.step(render=True) does NOT advance the same
        amount of simulated time as world.step(render=False) — passing the flag
        through here made a recorded run reach the goal in 11.0 s where the same
        seed unrecorded took 15.44 s. A recording has to depict the run that was
        scored, so rendering must not touch the physics at all.
        """
        self.command[0], self.command[1], self.command[2] = twist
        for _ in range(self.decimation):
            # ---- the frozen interface. Nothing below this line is experimental.
            self.policy.forward(self.physics_dt, self.command)
            self.world.step(render=False)
        if render:
            self.world.render()

    def pose(self):
        return base_pose(self.policy)


def open_episode(scene_path, start, log=print):
    """Open `scene_path`, spawn the Go2 at `start` = (x, y, z, yaw), initialise.

    Returns an Episode. Raises if the scene is missing — the caller should not
    have to guess why the robot is standing in an empty world.
    """
    scene_path = os.path.abspath(scene_path)
    if not os.path.exists(scene_path):
        raise FileNotFoundError(
            f"Scene not found: {scene_path}. Run build_scene.py first.")

    log(f"opening {scene_path}")
    omni.usd.get_context().open_stage(scene_path)
    for _ in range(20):
        _pump()

    # World is a singleton: without this, a second World() in the same process
    # returns the first one and the dt arguments below are silently ignored.
    # This is exactly the drive_go2.py gotcha recorded in CLAUDE.md.
    World.clear_instance()
    world = World(physics_dt=FALLBACK_PHYSICS_DT,
                  rendering_dt=FALLBACK_PHYSICS_DT * FALLBACK_DECIMATION,
                  stage_units_in_meters=1.0)

    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(GO2_PRIM):
        log(f"removing pre-existing {GO2_PRIM} from the scene")
        stage.RemovePrim(GO2_PRIM)
        for _ in range(5):
            _pump()

    x, y, z, yaw = start
    policy_cls = resolve_policy_class(log)

    # The constructor signature differs between releases, so pass only what this
    # build actually accepts.
    sig = inspect.signature(policy_cls.__init__)
    accepts_kwargs = any(p.kind is inspect.Parameter.VAR_KEYWORD
                         for p in sig.parameters.values())
    candidate = {
        "prim_path": GO2_PRIM,
        "name": "Go2",
        "position": np.array([x, y, z]),
        "orientation": yaw_to_quat(yaw),
    }
    kwargs = (candidate if accepts_kwargs
              else {k: v for k, v in candidate.items() if k in sig.parameters})
    dropped = sorted(set(candidate) - set(kwargs))
    if dropped:
        log(f"constructor does not take {dropped}, omitting")

    log("constructing policy, first run downloads the checkpoint")
    policy = policy_cls(**kwargs)

    if "position" not in kwargs or "orientation" not in kwargs:
        try:
            policy.robot.set_world_pose(position=np.array([x, y, z]),
                                        orientation=yaw_to_quat(yaw))
            log(f"start pose set post-construction to {(x, y, z, yaw)}")
        except Exception as exc:                       # noqa: BLE001
            log(f"could not set start pose: {exc}")

    # Take the rates from the policy's own env config. A policy trained at one
    # control rate misbehaves at another, and these are not ours to pick.
    physics_dt = float(getattr(policy, "_dt", FALLBACK_PHYSICS_DT))
    decimation = int(getattr(policy, "_decimation", FALLBACK_DECIMATION))
    log(f"physics dt {physics_dt:.4f}s, decimation {decimation}, "
        f"control rate {1.0 / (physics_dt * decimation):.1f} Hz")
    world.set_simulation_dt(physics_dt=physics_dt,
                            rendering_dt=physics_dt * decimation)

    world.reset()
    policy.initialize()
    return Episode(world, policy, physics_dt, decimation)
