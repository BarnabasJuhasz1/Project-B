"""
controllers/scaffold_sweep.py

Drive forward; turn away when something gets close. Never reads `obs.goal`.

**NOT NAVIGATION, and it must never appear in a benchmark condition.** It is the
search-family equivalent of `scripts/testing/walk_forward.py`: a scaffold that
makes the robot *move* so that machinery around it can be exercised. It has no
goal, no memory, no coverage guarantee and no termination condition — it wanders
until the episode runs out.

WHY IT EXISTS. Chunk 11a made `Observation.goal` optional and the runner latches
disclosure once the goal comes within DETECT_RADIUS with a clear line of sight.
Nothing could test that latch under physics, because every controller in this
repo correctly holds still when the goal is withheld — inventing exploration is
what the generated conditions are *for*. So the transition False -> True had
never executed in `run_episode.py`, only in the kinematic rollout, and those are
two separate latch lines over one shared rule.

This makes the robot move without giving it a search behaviour: it ignores
`obs.goal` entirely, disclosed or not, so it can never be mistaken for a
baseline that solves the search family. Watch `goal_known` in the trace, not the
score. The score will be a timeout, and that is correct.
"""

import numpy as np

from core.observation import Command


class ScaffoldSweep:

    TURN_CLEARANCE = 1.2      # start turning when the way ahead is closer than this
    CRUISE = 0.6              # m/s
    TURN_RATE = 0.9           # rad/s
    FORWARD_CONE = 0.6        # rad either side of straight ahead

    def __init__(self):
        self._turning = 0
        self._sign = 1.0

    def reset(self, scenario, rng):
        self._turning = 0
        # Which way it peels off is drawn from the scenario's rng, never from
        # global np.random, so a scaffold run is still reproducible from a seed.
        self._sign = 1.0 if rng.random() < 0.5 else -1.0

    def step(self, obs):
        # obs.goal is deliberately never read. That is the whole point: this
        # cannot become a search baseline by accident.
        angles = np.asarray(obs.range_angles, dtype=float)
        ranges = np.asarray(obs.ranges, dtype=float)
        ahead = np.abs(angles) <= self.FORWARD_CONE
        clear = np.nanmin(np.where(ahead, ranges, np.inf))

        if self._turning > 0:
            self._turning -= 1
            return Command(0.0, 0.0, self._sign * self.TURN_RATE)

        if clear < self.TURN_CLEARANCE:
            # Turn towards whichever side has more room, then commit to it for a
            # while — re-deciding every step is how a reactive controller ends up
            # oscillating in a corner instead of leaving it.
            left = np.nanmax(np.where(angles > self.FORWARD_CONE, ranges, -np.inf))
            right = np.nanmax(np.where(angles < -self.FORWARD_CONE, ranges, -np.inf))
            self._sign = 1.0 if left >= right else -1.0
            self._turning = 25          # ~0.5 s at 50 Hz
            return Command(0.0, 0.0, self._sign * self.TURN_RATE)

        return Command(self.CRUISE, 0.0, 0.0)
