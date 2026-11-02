"""
demo/mazes.py

WHICH MAZE. A name -> (open world, blocked world, door rect, scene) table, so
the rollout, the physics runner and the exporter resolve a maze the same way.

    patrol   demo/world.py   scenes/demo-patrol.usd   THE ORIGINAL. The default
                                                       everywhere, so every
                                                       existing command means
                                                       what it meant.
    maze2    demo/maze2.py   scenes/demo-maze2.usd    Mindfire Exp F (review E2)

⚠️ HOME, GOAL and the spawn yaw are SHARED, and `get()` refuses a maze that
disagrees: `PatrolScenario` hands the controller `start` / `goal` and every
controller sizes its map from them, so a maze with different endpoints would
change two things at once.

SIM-FREE.
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from demo import world as _patrol
from demo import maze2 as _maze2

DEFAULT = "patrol"

MAZES = {
    "patrol": dict(open=_patrol.OPEN_WORLD, blocked=_patrol.BLOCKED_WORLD,
                   door=_patrol.DOOR, home=_patrol.HOME, goal=_patrol.GOAL,
                   scene="scenes/demo-patrol.usd", spawn_yaw=0.6),
    "maze2": dict(open=_maze2.OPEN_WORLD, blocked=_maze2.BLOCKED_WORLD,
                  door=_maze2.DOOR, home=_maze2.HOME, goal=_maze2.GOAL,
                  scene="scenes/demo-maze2.usd", spawn_yaw=_maze2.SPAWN_YAW),
}


def get(name=None):
    name = DEFAULT if name is None else name
    if name not in MAZES:
        raise KeyError(f"unknown maze {name!r}; have {sorted(MAZES)}")
    m = MAZES[name]
    ref = MAZES[DEFAULT]
    for key in ("home", "goal", "spawn_yaw"):
        if m[key] != ref[key]:
            raise ValueError(f"maze {name!r} moves {key}: {m[key]} vs {ref[key]}")
    return m
