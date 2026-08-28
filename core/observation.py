"""
core/observation.py

The interface every benchmark condition is written against: what a controller
sees, what it may send, and the two methods it must have.

Fixing this now, before anything is written against it, is the point. Conditions
A, B, C and D all get the same three types, so a difference in their scores is a
difference in the code the LLM wrote and not a difference in what it was handed.

Two properties matter more than the field list.

**Ground truth never enters Observation.** The runner holds the true pose
separately and uses it only for scoring. `pose_estimate` is exactly the true pose
today, so nothing behaves differently yet — but drift can later be injected into
that one field and every controller, `IsLocalisationDegraded` included, becomes
measurable with no rewrites anywhere else. If controllers were handed the true
pose, the localisation-failure metric could never mean anything.

That principle now covers the goal as well, which is the second and last
application of it. `goal` is Optional: on a scenario that withholds it, the
controller is handed None until it has actually found the thing. The runner
still knows where the goal is the whole time and still scores against it —
`Scenario.goal` is unchanged and required. Withholding it from the controller is
what makes search a task rather than a navigation problem with extra steps.

**reset() exists so episode state lives on the instance.** demo_waypoints keeps
its route index in a mutable default argument (drive_go2.py:135), so a second
episode in the same process starts mid-route. A batch runner over a fixed seed
set does exactly that. Making reset() part of the protocol turns a thing to
remember into a thing a test can fail on.
"""

from dataclasses import dataclass
from typing import Optional, Protocol, Tuple

import numpy as np


@dataclass(frozen=True)
class Observation:
    """One control-step view of the world. Frozen: a controller must not edit
    what it was handed and have that leak into the next step or the trace."""

    t: float                    # sim seconds since episode start
    ranges: np.ndarray          # N-beam 2D scan, metres, inf where no return
    range_angles: np.ndarray    # beam bearings in the ROBOT frame, radians
    range_max: float            # what "as far as I can see" means
    pose_estimate: Tuple[float, float, float]   # (x, y, yaw) — the BELIEF
    pose_error_estimate: float  # scalar localisation-confidence proxy, metres
    contact: bool               # is the robot touching something right now

    # (x, y) in the same frame as pose_estimate, or None meaning "not
    # disclosed": the goal exists and the episode is still scored against it,
    # but this controller has not found it yet.
    #
    # It LATCHES. Once the runner discloses the goal it stays disclosed for the
    # rest of the episode, even if the goal goes back out of sight. The
    # precondition a node conditions on is "has the goal been found", which is
    # monotonic, and making it monotonic here means no node has to carry its own
    # memory of a fact the contract already established.
    #
    # A controller written against a scenario that always discloses the goal
    # still has to handle None, because the same controller is run on every
    # scenario in the split. Handle it explicitly; do not assume a tuple.
    goal: Optional[Tuple[float, float]]


@dataclass(frozen=True)
class Command:
    """The frozen interface. The only thing anything ever sends the robot.

    Below this line is the inherited, pretrained, never-retrained locomotion
    policy; above it is the entire experiment.
    """

    vx: float
    vy: float
    wz: float

    def as_tuple(self):
        return (self.vx, self.vy, self.wz)


STOP = Command(0.0, 0.0, 0.0)


class Controller(Protocol):
    """What the runner requires of anything driving the robot.

    Deliberately two methods. A generator emitting a behaviour tree, a hand
    written baseline and a Nav2 bridge all fit; nothing about the shape of the
    thing above the Twist is assumed.
    """

    def reset(self, scenario, rng: np.random.Generator) -> None:
        """Clear all episode state. Called once before every episode.

        `rng` is seeded from the scenario, so a controller that wants randomness
        must draw it from here and never from the global np.random.
        """
        ...

    def step(self, obs: Observation) -> Command:
        """One control step. Called every `decimation` physics steps, matching
        how the policy actually consumes commands — not every physics step."""
        ...


def clamp(command: Command, max_lin: float, max_ang: float) -> Command:
    """Saturate a Twist to the scenario's limits.

    Applied by the runner, not trusted to the controller: a generated node that
    returns 40 m/s should be scored on what the robot did, not rejected, and
    every condition must be saturated identically.
    """
    return Command(
        float(np.clip(command.vx, -max_lin, max_lin)),
        float(np.clip(command.vy, -max_lin, max_lin)),
        float(np.clip(command.wz, -max_ang, max_ang)),
    )
