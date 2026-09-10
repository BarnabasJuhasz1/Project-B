"""
core/tests/kinematic.py

An idealised unicycle rollout, for tests only.

THIS IS NOT A SIMULATOR AND NOT A BENCHMARK PATH. No number produced here may
be reported. The robot is a point that turns and translates exactly as
commanded, walls are impassable rather than physical, and the frozen locomotion
policy — the thing that actually decides whether a Twist becomes motion — is
absent entirely.

What it is good for: asserting, in milliseconds and without a GPU, that a
reference controller has the behaviour it claims. "goal_seeker is expected to
fail, reactive_avoid is expected to succeed" is the entire basis for trusting
the scoring code, and it is worth being an assertion rather than a comment.
The real numbers come from scripts/run_episode.py inside the container.
"""

import math

from core.localisation import make_localiser
from core.sensor_noise import make_sensor_noise
from core.geometry import ROBOT_RADIUS, clearance, raycast
from core.metrics import EpisodeTrace
from core.observation import Observation, clamp

CONTROL_DT = 0.02          # 50 Hz, the rate the runner derives from the policy
# The robot may approach a wall until its circumscribing circle is this far
# inside it. ROBOT_RADIUS circumscribes a 0.65 x 0.31 m body, so a little
# overlap is the body's corners, not a wall strike.
BLOCK_MARGIN = 0.55


def rollout(scenario, controller, dt=CONTROL_DT):
    """Drive `controller` round `scenario` under ideal kinematics. Returns an
    EpisodeTrace that core.metrics.score() consumes unchanged."""
    controller.reset(scenario, scenario.rng())
    angles = scenario.beam_angles()
    x, y, yaw = scenario.start
    trace = EpisodeTrace()
    # The disclosure latch, applying scenario.disclosed_at() exactly as
    # run_episode.py does. It has to be the same rule in both places: a node
    # whose unit scenario discloses the goal on a different step from the
    # benchmark is a node whose unit scenario proves nothing about the run.
    goal_known = scenario.goal_known
    # Built through make_localiser() so a unit scenario and the benchmark
    # resolve the pose estimate identically -- the same argument make_drift()
    # already made, one level up. With no `localisation` set this IS the drift
    # model it always was, wrapped.
    localiser = make_localiser(scenario)
    localiser.reset(scenario, scenario.localisation_rng())
    # NoNoise.scan() IS raycast(), so a scenario without sensor_noise takes the
    # code path it always took -- see core/sensor_noise.py.
    sensor = make_sensor_noise(scenario)
    # Micro-worlds are measured by THIS code, not a second implementation, or a
    # node that passes its unit scenario was tested against different geometry
    # from the benchmark.
    world = scenario.world

    for step in range(int(scenario.time_limit / dt)):
        t = step * dt
        gap = float(clearance((x, y), world)) - ROBOT_RADIUS
        goal_known = goal_known or scenario.disclosed_at((x, y))
        # The scan is cast BEFORE the localiser updates, because a localiser
        # matches the scan it is about to be judged on. `raycast` is a pure
        # function of pose, so hoisting it changes nothing for the drift path.
        ranges = sensor.scan((x, y), yaw, angles, scenario.range_max, world)
        localiser.update(dt, (x, y, yaw), ranges, angles)
        estimate = localiser.estimate()
        obs = Observation(
            t=t,
            ranges=ranges,
            range_angles=angles,
            range_max=scenario.range_max,
            pose_estimate=estimate,
            pose_error_estimate=localiser.error_estimate(),
            goal=scenario.goal if goal_known else None,
            contact=gap <= 0.0,
        )
        command = clamp(controller.step(obs), scenario.max_lin, scenario.max_ang)
        trace.append(t, (x, y, 0.55, yaw), command, gap, gap <= 0.0,
                     estimate, goal_known)

        nx = x + (command.vx * math.cos(yaw) - command.vy * math.sin(yaw)) * dt
        ny = y + (command.vx * math.sin(yaw) + command.vy * math.cos(yaw)) * dt
        if float(clearance((nx, ny), world)) > ROBOT_RADIUS * BLOCK_MARGIN:
            x, y = nx, ny          # otherwise the wall simply stops it
        yaw = (yaw + command.wz * dt + math.pi) % (2 * math.pi) - math.pi

        if math.hypot(x - scenario.goal[0], y - scenario.goal[1]) <= 0.4:
            break

    return trace


def command_sequence(scenario, controller, steps=200):
    """The first `steps` Twists a controller issues, for reset() determinism."""
    controller.reset(scenario, scenario.rng())
    angles = scenario.beam_angles()
    x, y, yaw = scenario.start
    out = []
    goal_known = scenario.goal_known
    world = scenario.world
    for step in range(steps):
        goal_known = goal_known or scenario.disclosed_at((x, y))
        obs = Observation(
            t=step * CONTROL_DT,
            ranges=raycast((x, y), yaw, angles, scenario.range_max, world),
            range_angles=angles,
            range_max=scenario.range_max,
            pose_estimate=(x, y, yaw),
            pose_error_estimate=0.0,
            goal=scenario.goal if goal_known else None,
            contact=False,
        )
        command = controller.step(obs)
        out.append(command.as_tuple())
        # Advance so the controller sees a changing world; if it were fed the
        # same observation forever, stale state would never show up.
        x += command.vx * math.cos(yaw) * CONTROL_DT
        y += command.vx * math.sin(yaw) * CONTROL_DT
        yaw += command.wz * CONTROL_DT
    return out
