"""
harness/micro.py

A micro-world: a handful of boxes, a start, a goal, and a seed.

WHY IT IS NOT A Scenario. `MicroScenario` deliberately does not subclass or
extend `core.scenario.Scenario`, and `Scenario` deliberately did not grow a
`world` field to accommodate it — chunk 11a declared that field list closed. A
micro-world is not a benchmark task and must never be mistaken for one: it never
appears in `all_scenarios()`, it carries `split="unit"`, and nothing scored here
may be reported. It duck-types the surface `core.tests.kinematic.rollout` and
`core.metrics.score` actually use, and no more.

WHY IT IS MEASURED BY core.geometry. The whole point is that a node which passes
its unit scenario has been tested against the *same* geometry code the benchmark
runs on. A second implementation of clearance or raycast — however careful —
would mean a passing unit scenario says nothing about the benchmark. So
`core.geometry` was parameterised by `World` rather than reimplemented, and this
module supplies a `World` made of boxes.

THE LIMITATION, STATED PLAINLY. A micro-world is rolled out kinematically:
a point robot that turns and translates exactly as commanded, with the frozen
locomotion policy absent entirely. **A node that passes here may still fail under
physics.** The unit scenario is a NECESSARY condition for admission, never a
sufficient one, and the write-up has to say so.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from core.drift import make_drift            # noqa: F401  (re-exported for callers)
from core.geometry import World, segment_clearance
from core.metrics import GOAL_RADIUS
from core.scenario import (DETECT_RADIUS, FOV, MAX_ANG, MAX_LIN, N_BEAMS,
                           RANGE_MAX)

import math


def room(width, height, boxes=(), name="micro"):
    """A closed rectangular room of `width` x `height` with `boxes` inside.

    Walls are built the way core.scene_spec builds lab42's: 0.15 m thick, laid
    just outside the bounds, so a micro-world is the same KIND of thing the
    benchmark scene is and not a special case the geometry has to know about.
    """
    t = 0.075
    walls = (
        (0.0, -t, width, t),                 # south
        (0.0, height - t, width, height + t),  # north
        (-t, 0.0, t, height),                # west
        (width - t, 0.0, width + t, height),  # east
    )
    return World(rects=walls + tuple(tuple(map(float, b)) for b in boxes),
                 bounds=(0.0, 0.0, float(width), float(height)), name=name)


@dataclass(frozen=True)
class MicroScenario:
    """One purpose-built world that isolates one node's claimed effect."""

    id: str
    world: World
    start: Tuple[float, float, float]
    goal: Tuple[float, float]
    time_limit: float
    seed: int

    goal_known: bool = True
    detect_radius: float = DETECT_RADIUS
    drift: Optional[object] = None

    # Sensing is the benchmark's, not a per-test choice. If a unit scenario
    # could pick a better sensor, a node could pass by being handed one.
    n_beams: int = N_BEAMS
    fov: float = FOV
    range_max: float = RANGE_MAX
    max_lin: float = MAX_LIN
    max_ang: float = MAX_ANG

    family: str = "unit"
    split: str = "unit"

    def beam_angles(self):
        half = 0.5 * self.fov
        full = abs(self.fov - 2.0 * math.pi) < 1e-9
        return np.linspace(-half, half, self.n_beams, endpoint=not full)

    def rng(self):
        return np.random.default_rng(self.seed)

    def drift_rng(self):
        return np.random.default_rng([self.seed, 0xD41F7])

    def disclosed_at(self, pose_xy):
        """Identical rule to Scenario.disclosed_at, against this world."""
        if self.goal_known:
            return True
        gx, gy = self.goal
        x, y = float(pose_xy[0]), float(pose_xy[1])
        if math.hypot(gx - x, gy - y) > self.detect_radius:
            return False
        return float(segment_clearance(np.array([x, y]), np.array([gx, gy]),
                                       self.world)) > 0.0

    def optimal_distance(self):
        """The SPL denominator for this micro-world, by the same function the
        benchmark uses."""
        from core.geometry import shortest_path_length
        return shortest_path_length(self.start[:2], self.goal,
                                    stop_radius=GOAL_RADIUS, world=self.world)

    def validate(self):
        """The subset of core.scenario.validate that means anything here.

        A malformed micro-world is worse than a failing node: it produces a
        confident pass/fail about nothing. Raises rather than returning False.
        """
        from core.geometry import ROBOT_RADIUS, clearance, occupancy_grid
        min_x, min_y, max_x, max_y = self.world.bounds
        for label, (x, y) in (("start", self.start[:2]), ("goal", self.goal)):
            if not (min_x <= x <= max_x and min_y <= y <= max_y):
                raise ValueError(
                    f"micro {self.id!r}: {label} ({x}, {y}) is outside "
                    f"{self.world.bounds}")
        grid = occupancy_grid(world=self.world)
        for label, (x, y) in (("start", self.start[:2]), ("goal", self.goal)):
            if not grid.is_free((x, y)):
                raise ValueError(
                    f"micro {self.id!r}: {label} ({x}, {y}) is inside a box or "
                    f"within the robot radius of one")
        if not math.isfinite(self.optimal_distance()):
            raise ValueError(
                f"micro {self.id!r}: goal {self.goal} is unreachable from "
                f"{self.start[:2]}")
        if float(clearance(np.asarray(self.goal, dtype=float),
                           self.world)) < ROBOT_RADIUS + GOAL_RADIUS:
            raise ValueError(
                f"micro {self.id!r}: goal {self.goal} is too close to a wall for "
                f"the whole GOAL_RADIUS disc to be reachable, so "
                f"optimal_distance() would understate the denominator")
        if self.time_limit <= 0:
            raise ValueError(f"micro {self.id!r}: time_limit must be positive")
        if not self.goal_known and self.disclosed_at(self.start[:2]):
            raise ValueError(
                f"micro {self.id!r}: the goal is already visible from the start, "
                f"so this is not a search task")
        return self
