"""
demo/rollout.py

THE PATROL ROLLOUT. Four one-way trips, and the door shuts halfway.

    leg 0   home -> goal     door OPEN
    leg 1   goal -> home     door OPEN
    leg 2   home -> goal     door SHUT      <- the discovery
    leg 3   goal -> home     door SHUT

`obs.goal` holds whichever point is currently being driven to, and flips on
arrival. ⚠️ **`reset()` is called ONCE, before leg 0.** The controller is never
reset between legs -- that is the whole experiment. Anything it learned about
the building on legs 0-1 is still on the instance when it meets the shut door on
leg 2, if it chose to keep anything.

The door shuts while the robot is at HOME, between legs, so it can never close
on top of the robot.

⚠️ **KINEMATIC, AND NOT A BENCHMARK PATH.** The frozen locomotion policy is
absent; the robot turns and translates exactly as commanded. This is the fast
loop -- one episode in ~2 s against ~20 minutes recorded under physics. It is
for deciding which controllers are worth recording, never for a reported number.
It reuses `core.geometry`'s raycast and clearance and `core.observation`'s
contract so the sensor model and the interface are the benchmark's, not a second
implementation.

SIM-FREE.
"""
import sys, pathlib, math, ast
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import numpy as np

from core.geometry import ROBOT_RADIUS, clearance, raycast
from core.observation import Observation, clamp
from demo.world import OPEN_WORLD, BLOCKED_WORLD, HOME, GOAL

# ---- below this line is data, not code.

CONTROL_DT = 0.02             # 50 Hz, as core/tests/kinematic.py
ARRIVE_RADIUS = 0.40          # as core.metrics' goal test
N_BEAMS = 181
RANGE_MAX = 6.0
MAX_LIN, MAX_ANG = 0.8, 1.0
BLOCK_MARGIN = 0.55           # as core/tests/kinematic.py: corners may overlap
TIME_LIMIT = 180.0            # the budget. ~3x the taut total even on the long route;
                              # see demo/README.md for how it was calibrated.

# Which world each leg runs in. THE DOOR SHUTS AT LEG 2.
LEG_TARGETS = (GOAL, HOME, GOAL, HOME)
LEG_BLOCKED = (False, False, True, True)

# What a model-written controller is allowed to import.
#
# ⚠️ **ALLOW-BY-DEFAULT OVER THE STDLIB, DENY A SHORT LIST.** The first version
# of this was an explicit allowlist and it REFUSED 4 OF 6 GENERATED CONTROLLERS
# over `import time` -- a harmless module that no allowlist author thinks of and
# that models reach for constantly. A gate that rejects most of the samples is
# not measuring the arms, it is measuring the gate, and it very nearly produced
# a null result that was entirely my own bug.
#
# What this must actually stop is narrow: the controller reading GROUND TRUTH
# (core.geometry knows where the walls are; demo.world knows where the door is),
# and the obvious process/filesystem escapes. That is what BANNED and DENIED
# cover, and nothing else needs covering -- `generate/gate.py` is the strict
# version of this idea and it guards a path where a result gets reported.
import sys as _sys

DENIED_STDLIB = {
    "os", "subprocess", "socket", "importlib", "shutil", "ctypes",
    "multiprocessing", "pickle", "marshal", "signal", "pty", "tempfile",
    "urllib", "http", "ftplib", "webbrowser", "runpy",
}
EXTRA_ALLOWED = {"numpy", "core.observation"}
ALLOWED_IMPORTS = (set(_sys.stdlib_module_names) - DENIED_STDLIB) | EXTRA_ALLOWED

# Ground truth, by any spelling. These are the imports that would let a
# controller read the answer instead of sensing it.
BANNED_SUBSTRINGS = ("core.geometry", "core.worldgen", "demo.world",
                     "core.scenario", "core.scene_spec", "core.metrics")


class PatrolScenario:
    """What the controller is handed as `scenario`. Deliberately tiny, and it
    holds NO geometry -- the world reaches the controller only through the
    lidar, exactly as in the benchmark."""

    def __init__(self, seed=0, time_limit=None):
        self.start = HOME
        self.goal = GOAL
        self.home = HOME
        self.time_limit = TIME_LIMIT if time_limit is None else float(time_limit)
        self.range_max = RANGE_MAX
        self.max_lin = MAX_LIN
        self.max_ang = MAX_ANG
        self.seed = seed

    def rng(self):
        return np.random.default_rng(self.seed)

    def beam_angles(self):
        return np.linspace(-math.pi, math.pi, N_BEAMS, endpoint=False)


def check_imports(source, where="<controller>"):
    """Static AST allowlist. Returns a list of reasons; empty means fine."""
    bad = []
    for node in ast.walk(ast.parse(source)):
        names = []
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            names = [node.module or ""]
        for n in names:
            root = n.split(".")[0]
            if any(b in n for b in BANNED_SUBSTRINGS):
                bad.append(f"{where}: banned import {n!r}")
            elif n not in ALLOWED_IMPORTS and root not in ALLOWED_IMPORTS:
                bad.append(f"{where}: import {n!r} is not on the allowlist")
    return bad


def load_controller(path):
    """Execute a model-written file and return its Controller class."""
    path = pathlib.Path(path)
    source = path.read_text(encoding="utf-8")
    bad = check_imports(source, where=path.name)
    if bad:
        raise ImportError("; ".join(bad))
    ns = {"__name__": f"demo_controller_{path.stem}"}
    exec(compile(source, str(path), "exec"), ns)
    for key in ("Controller", "PatrolController", "Go2Controller", "BaseController"):
        if key in ns and isinstance(ns[key], type):
            return ns[key]
    raise AttributeError(f"{path} defines no Controller class")


def rollout(controller, seed=0, dt=CONTROL_DT, record=False, time_limit=None,
            start_offset=(0.0, 0.0, 0.0), noise=None):
    """Drive one patrol episode. Returns a result dict (+ a trace if asked).

    ⚠️ `time_limit` is the budget for the WHOLE EPISODE, not per leg. The
    four legs run in sequence and share it: a controller that spends the
    budget failing leg 2 never starts leg 3. It is overridable because a
    budget tight enough to punish the LONGER-BUT-CORRECT route would be
    measuring speed rather than capability.

    `start_offset` = (dx, dy, dyaw) is added to HOME and the spawn yaw 0.6 --
    Exp 3's perturbation (demo/exp3.py), mirrored here so the kinematic and
    physics reps start from the same pose. The default is the original.

    `noise`, if given, is an object with `sample() -> (dvx, dvy, dwz)` called
    once per step and ADDED to the Twist that is integrated -- never to what
    the controller is told it commanded, and never to its observation. Exp E
    (prereg/expE.md) uses it to inject the tracking error measured under
    physics (`demo/tracking.py`). None, the default, is the original rollout."""
    limit = TIME_LIMIT if time_limit is None else float(time_limit)
    scenario = PatrolScenario(seed=seed, time_limit=limit)
    angles = scenario.beam_angles()
    controller.reset(scenario, scenario.rng())          # ONCE, for the episode

    dx, dy, dyaw = start_offset
    x, y, yaw = HOME[0] + dx, HOME[1] + dy, 0.6 + dyaw
    leg = 0
    leg_start_t = 0.0
    legs_done, leg_times, collisions, path_length = [], [], 0, 0.0
    was_touching = False
    trace = []

    for step in range(int(limit / dt)):
        t = step * dt
        if leg >= len(LEG_TARGETS):
            break
        world = BLOCKED_WORLD if LEG_BLOCKED[leg] else OPEN_WORLD
        target = LEG_TARGETS[leg]

        gap = float(clearance((x, y), world)) - ROBOT_RADIUS
        touching = gap <= 0.0
        if touching and not was_touching:
            collisions += 1
        was_touching = touching

        obs = Observation(
            t=t,
            ranges=raycast((x, y), yaw, angles, RANGE_MAX, world),
            range_angles=angles,
            range_max=RANGE_MAX,
            pose_estimate=(x, y, yaw),     # no drift in this demo
            pose_error_estimate=0.0,
            goal=target,
            contact=touching,
        )
        try:
            command = clamp(controller.step(obs), MAX_LIN, MAX_ANG)
        except Exception as exc:                     # a crash is a result
            return {"legs_completed": leg, "crashed": repr(exc),
                    "leg_times": leg_times, "collisions": collisions,
                    "path_length": path_length, "duration": t,
                    "trace": trace if record else None}

        if record:
            trace.append((t, x, y, yaw, leg, command.vx, command.vy, command.wz,
                          gap, int(touching)))

        vx, vy, wz = command.vx, command.vy, command.wz
        if noise is not None:
            dvx, dvy, dwz = noise.sample()
            vx, vy, wz = vx + dvx, vy + dvy, wz + dwz
        nx = x + (vx * math.cos(yaw) - vy * math.sin(yaw)) * dt
        ny = y + (vx * math.sin(yaw) + vy * math.cos(yaw)) * dt
        if float(clearance((nx, ny), world)) > ROBOT_RADIUS * BLOCK_MARGIN:
            path_length += math.hypot(nx - x, ny - y)
            x, y = nx, ny
        yaw = (yaw + wz * dt + math.pi) % (2 * math.pi) - math.pi

        if math.hypot(x - target[0], y - target[1]) <= ARRIVE_RADIUS:
            leg_times.append(round(t - leg_start_t, 2))
            legs_done.append(leg)
            leg += 1
            leg_start_t = t

    return {"legs_completed": leg, "crashed": None, "leg_times": leg_times,
            "collisions": collisions, "path_length": round(path_length, 1),
            "duration": round(min(t, limit), 2), "time_limit": limit,
            "trace": trace if record else None}


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("controller", help="path to a controller .py")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    cls = load_controller(a.controller)
    r = rollout(cls(), seed=a.seed)
    r.pop("trace", None)
    print(r)
