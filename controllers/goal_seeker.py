"""
controllers/goal_seeker.py

Turn to face the goal, then walk at it. No use of `ranges` at all.

NOT NAVIGATION, and it must never appear in a benchmark condition. It exists to
prove that the failure half of the scoring code fires: on `doorway_traverse` it
walks into the divider and should produce collisions, stuck events and a
timeout. A harness whose reference controller always succeeds proves nothing.

This is demo_waypoints (drive_go2.py:135-160) with three changes, each of which
is the point:

  * the waypoint list is gone — it drives at obs.goal, so it is scenario-agnostic
  * it reads obs.pose_estimate, the robot's belief, not the true articulation pose
  * its state lives on the instance and is cleared by reset(), not in a mutable
    default argument that survives into the next episode

It HOLDS STILL when the goal has not been disclosed (obs.goal is None), because
it has no way to look for anything: it never reads `ranges`. That is a correct
refusal, not a search behaviour, and it stays that way deliberately — inventing
exploration is what the generated conditions are for, and a reference controller
that could search would quietly become a fourth baseline nobody pre-registered.
"""

import math

from core.observation import STOP, Command


class GoalSeeker:

    # Face the goal to within this before walking. Wide enough that the robot
    # does not stall trying to null a heading error the gait cannot hold.
    HEADING_TOLERANCE = 0.25      # rad
    TURN_GAIN = 2.0
    APPROACH_GAIN = 0.6

    def __init__(self):
        self._scenario = None
        self._steps = 0

    def reset(self, scenario, rng):
        self._scenario = scenario
        self._steps = 0

    def step(self, obs):
        self._steps += 1
        if obs.goal is None:
            return STOP                # see the module docstring
        x, y, yaw = obs.pose_estimate
        dx, dy = obs.goal[0] - x, obs.goal[1] - y
        dist = math.hypot(dx, dy)

        err = math.atan2(dy, dx) - yaw
        err = (err + math.pi) % (2 * math.pi) - math.pi

        if abs(err) > self.HEADING_TOLERANCE:
            return Command(0.0, 0.0, self.TURN_GAIN * err)
        return Command(self.APPROACH_GAIN * dist, 0.0, err)
