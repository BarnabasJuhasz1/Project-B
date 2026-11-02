# Write the best possible navigation controller for a Unitree Go2

You are writing a Python class that drives a Unitree Go2 quadruped around an
indoor space it has **no map of**. Your controller is the entire experiment: the
locomotion layer below you is inherited, pretrained and frozen, and the only
thing anything ever sends the robot is a Twist.

## The task the robot has to perform

The robot **shuttles back and forth between two fixed points** in the building —
call them *home* and *the goal*. It does this repeatedly, for as long as the
episode lasts.

`obs.goal` always holds **the point you are currently heading for**. When you
arrive there, the harness switches `obs.goal` to the other point and the robot
must turn round and drive back. You do not have to detect arrival or manage the
sequence yourself; just always drive toward whatever `obs.goal` currently says.

So over one episode the robot covers the same building **several times, in both
directions**. The episode ends when the time budget runs out, and you are scored
on **how many of these one-way trips you complete**.

The building is unmapped, and the robot has never been in it before. Nothing
tells you the layout in advance. You learn what you learn from the lidar.

## What the robot can sense, once per control step

```
t                    sim seconds since the episode started
ranges               181 lidar beams over a full circle, metres, inf where no return
range_angles         beam bearings in the ROBOT frame, radians
range_max            6.0 — what "as far as I can see" means
pose_estimate        (x, y, yaw) — where the robot believes it is
pose_error_estimate  scalar metres — how wrong that belief is believed to be
contact              bool — is the robot touching something right now
goal                 (x, y) — THE POINT CURRENTLY BEING DRIVEN TO
```

Ground truth is never observable. `pose_estimate` is the only position
information you get.

## What the robot can be sent

A `Command(vx, vy, wz)` — metres/second and radians/second in the robot frame,
saturated by the harness to `|vx|, |vy| <= 0.8` and `|wz| <= 1.0`. You cannot
touch the gait, the joints or the policy. Everything you do lives above that line.

## What you must write

One Python class named `Controller`, with exactly two methods:

```python
class Controller:
    def reset(self, scenario, rng):
        """Clear ALL episode state. Called once before the episode.
        `scenario` exposes .start, .goal, .time_limit, .range_max,
        .max_lin, .max_ang. `rng` is a seeded numpy Generator — if you
        want randomness, draw it from here and never from global np.random."""

    def step(self, obs):
        """One control step at 50 Hz. Return a Command."""
```

## Rules

- **Imports: only the Python standard library and `numpy`**, plus
  `from core.observation import Command` (and `Observation` if you want the type).
  Nothing else. In particular nothing from `core.geometry` — that module holds
  ground truth and using it is cheating.
- **All episode state lives on the instance and is cleared in `reset()`.** No
  module-level mutable state, no mutable default arguments.
- `step()` is called at 50 Hz and must return quickly. It must never raise.
- Write the whole controller in one file. No placeholders, no `TODO`, no
  abstract methods left unimplemented — this code is executed exactly as you
  write it.

## How you are scored

**One-way trips completed inside the time budget** is the primary number.
Collisions and distance travelled are recorded but do not decide the result.

Return **one fenced ```python block** containing the complete file, and nothing
else that looks like Python.
