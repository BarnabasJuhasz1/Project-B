"""
controllers/reactive_avoid.py

Steer down the most goal-aligned direction that is clear enough to walk. A
textbook gap-follower (vector field histogram, in the usual naming), written to
be generic: it reads nothing but `Observation` and knows nothing about lab42.

NOT NAVIGATION in the sense this project means it, and it must never appear in a
benchmark condition. It is the half of the harness validation that is expected to
*succeed*, which is what proves the success and SPL paths fire and that a
scenario is solvable at all.

It is also, deliberately, close to what a frontier model writes when asked
plainly to "program a Go2 to navigate to a goal": instantaneous, greedy,
optimising alignment with the straight line to the goal, with no memory of where
it has been and no mechanism for what to do when the gradient runs out. That is
condition A's expected shape, so having it here as a reference point is useful
rather than a strawman.

The first version of this was a potential-field sum of repulsion vectors. It
parked permanently at (5.36, 3.47) on `doorway_traverse` — nose against the
divider, attraction and repulsion balanced, which is the local minimum every
potential field has. Gap-following has no such fixed point as long as some
direction is clear, and it is the honest baseline: a model asked for this cold
would write one of these two.

Like goal_seeker it HOLDS STILL when the goal has not been disclosed
(obs.goal is None). It has the sensing to explore and deliberately does not:
every steering decision it makes is a function of `goal_bearing`, so without a
goal it has no objective, and bolting one on would turn the success-side
reference controller into an unregistered search baseline. A search behaviour is
something a condition has to produce, not something the harness supplies.
"""

import math

import numpy as np

from core.observation import Command


class ReactiveAvoid:

    # Half-width of the cone checked around each candidate direction. Roughly
    # what the body sweeps at 1 m, so a "clear" direction is clear for the whole
    # robot and not just for an infinitely thin ray.
    CONE = math.radians(25.0)
    # A direction is admissible if nothing in its cone is nearer than this. The
    # cap against distance-to-goal matters near the end of an episode, when the
    # wall behind the goal would otherwise make every direction inadmissible.
    CLEAR_NEEDED = 1.0         # m
    # Beams beyond this do not make a direction any more attractive; the point
    # is to walk towards the goal, not towards open space.
    INFLUENCE = 2.0            # m
    TURN_GAIN = 1.5
    # Above this heading error, turn in place rather than arc round.
    FORWARD_CONE = math.pi / 3
    MIN_SPEED = 0.15           # m/s once committed to moving
    # Directions this close to the most open one count as equally open.
    TIE_TOLERANCE = 0.05       # m

    def __init__(self):
        self._scenario = None
        self._steps = 0

    def reset(self, scenario, rng):
        self._scenario = scenario
        self._steps = 0

    def step(self, obs):
        self._steps += 1
        if obs.goal is None:
            return Command(0.0, 0.0, 0.0)      # see the module docstring
        x, y, yaw = obs.pose_estimate
        dx, dy = obs.goal[0] - x, obs.goal[1] - y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            return Command(0.0, 0.0, 0.0)

        goal_bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2 * math.pi) - math.pi

        angles = np.asarray(obs.range_angles, dtype=float)
        # inf means "nothing out there", which for choosing a direction is the
        # same as "clear to the sensor limit".
        ranges = np.nan_to_num(np.asarray(obs.ranges, dtype=float),
                               posinf=obs.range_max)

        # Only the forward hemisphere. The gait walks forwards; picking a
        # direction behind the robot means turning about anyway, and the turn is
        # what the heading error below already produces.
        forward = np.abs(angles) <= math.pi / 2
        candidates = angles[forward]
        if candidates.size == 0:
            return Command(0.0, 0.0, self.TURN_GAIN * goal_bearing)

        # Minimum range within CONE of each candidate: how far the robot could
        # actually walk that way.
        delta = np.abs(((candidates[:, None] - angles[None, :] + math.pi)
                        % (2 * math.pi)) - math.pi)
        in_cone = delta <= self.CONE
        cone_min = np.where(in_cone, ranges[None, :], np.inf).min(axis=1)

        needed = min(self.CLEAR_NEEDED, dist)
        admissible = cone_min > needed

        offset = np.abs(((candidates - goal_bearing + math.pi)
                         % (2 * math.pi)) - math.pi)

        if admissible.any():
            # Among the walkable directions, the one pointing most at the goal.
            best = int(np.argmin(np.where(admissible, offset, np.inf)))
        else:
            # Boxed in: turn towards the most open direction rather than pick
            # the least-bad one to walk into. Ties are broken towards the goal,
            # not by beam index — with a uniformly blocked front every cone_min
            # is equal, and argmax alone would turn whichever way the beam array
            # happens to be ordered.
            openest = cone_min >= cone_min.max() - self.TIE_TOLERANCE
            best = int(np.argmin(np.where(openest, offset, np.inf)))
            return Command(0.0, 0.0, self.TURN_GAIN * float(candidates[best]))

        heading_err = float(candidates[best])
        wz = self.TURN_GAIN * heading_err
        if abs(heading_err) > self.FORWARD_CONE:
            return Command(0.0, 0.0, wz)

        # Speed falls off with heading error and with how far the chosen
        # direction is actually clear. The second term is a crude stand-in for
        # LimitSpeedToVisibleRange — crude on purpose: it uses the nearest
        # return in the cone, not the free distance along the intended path, and
        # it has no notion of an occlusion boundary at all.
        speed = math.cos(heading_err) * min(1.0, float(cone_min[best]) / self.INFLUENCE)
        return Command(max(self.MIN_SPEED, speed), 0.0, wz)
