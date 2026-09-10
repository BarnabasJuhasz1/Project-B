# Write one behaviour-tree leaf node

You are writing a single leaf node for the behaviour tree of a Unitree Go2
quadruped navigating an indoor space it has no map of.

Below the node is an inherited, pretrained, **frozen** locomotion policy. The
only thing anything ever sends the robot is a Twist — `(vx, vy, wz)`. You never
touch the policy, the gait, or the joints. You decide what Twist to ask for and
when.

Return **one Python module**, in one fenced ```python block, and nothing else
that looks like code. Everything you want to say in prose, say outside the
fence. What the module must contain is set out in "What to return" at the end.

## The node contract

A leaf is one of three kinds, and the kind is the class you subclass:

- **`Condition`** — answers one question about the world. Never emits a Twist.
- **`Action`** — does one thing. The only kind that may emit a Twist.
- **`Decorator`** — wraps exactly one child and governs whether the child's
  Twist happens. Never emits one of its own.

Your class declares, as class attributes:

| Attribute | What it is |
|---|---|
| `PORTS` | a tuple of `Port` — the typed interface. See below. |
| `PRECONDITION` | one sentence: when this node applies |
| `EFFECT` | one sentence: what this node changes |
| `TAGS` | a tuple of short strings, for retrieval |

and implements:

```python
def reset(self, scenario, rng):    # clear per-episode state. optional.
def precondition(self, obs, ports) -> bool:   # default: always True
def tick(self, obs, ports) -> Status:         # Status.SUCCESS / FAILURE / RUNNING
```

**One precondition and one effect.** A node that needs two of either is two
nodes. This is not a style rule: a unit scenario has to isolate one claimed
effect, and it cannot isolate two.

**No coupling to a specific scenario.** Your node will be run on worlds you have
never seen. It may not name a room, a coordinate, a doorway or a map. Anything
it knows about the world it learns through `obs` at tick time.

### Ports

Ports are how nodes pass information to each other. They are **not** a
blackboard: at tick time you are handed a `PortView` restricted to exactly the
ports you declared, and touching anything else raises immediately.

```python
Port(name, PortType.X, Direction.IN)    # you read it:   value = ports["name"]
Port(name, PortType.X, Direction.OUT)   # you write it:  ports["name"] = value
```

The port vocabulary is **closed**. These seven types and no others:

| `PortType` | Meaning |
|---|---|
| `POSE` | `(x, y, yaw)` — a belief about where the robot is, never ground truth |
| `BEARING` | radians, robot frame unless the port name says otherwise |
| `SPEED` | m/s, non-negative |
| `RANGES` | an array of beam returns, metres |
| `CONFIDENCE` | `[0, 1]` — how much to trust something |
| `FLAG` | a boolean fact about the episode |
| `REGION` | an area of the floor, `(x, y, radius)` |

Reading an IN port that nothing has written yet raises. Declare only what you
use.

**The Twist is not a port.** An `Action` emits one with
`ports.drive(Command(vx, vy, wz))`. A `Condition` or `Decorator` that calls
`drive()` raises — that is what makes the three kinds a real distinction rather
than a label.

### What to import

Exactly these, and nothing else. `Command` lives in `core.observation`, not
alongside the node contract — the Twist is the frozen interface, deliberately
outside the port system.

```python
import math
import numpy as np

from core.observation import Command
from nodes.contract import (Action, Condition, Decorator, Direction, Port,
                            PortType, Status)
from harness.micro import MicroScenario, room
```

Import only what you use.

### What your node can see

`tick` is handed an `Observation`, frozen, one per control step:

```python
obs.t                   # sim seconds since the episode started
obs.ranges              # np.ndarray, 181 beams, metres, inf where no return
obs.range_angles        # np.ndarray, beam bearings in the ROBOT frame, radians
obs.range_max           # 6.0 — what "as far as I can see" means
obs.pose_estimate       # (x, y, yaw) — the BELIEF about our pose, not the truth
obs.pose_error_estimate # scalar, metres — how wrong we think the belief is
obs.contact             # bool — are we touching something right now
obs.goal                # (x, y), or None meaning NOT DISCLOSED
```

Two of these are traps and both are deliberate:

- **`pose_estimate` is a belief.** On some scenarios a seeded random walk is
  added to it and it drifts away from the truth. Ground truth is never in
  `Observation`; the runner keeps it for scoring only.
- **`obs.goal` may be `None`**, meaning the goal exists and the episode is still
  scored against it, but this robot has not found it yet. It **latches**: once
  disclosed it stays disclosed. Your node runs on scenarios that withhold the
  goal and on scenarios that do not, so **handle `None` explicitly**. Do not
  assume a tuple.

The scan is 181 beams over a full circle, so `obs.ranges[0]` is not the front —
find the beam you want through `obs.range_angles`.

Speeds are saturated by the runner to `|vx|, |vy| <= 0.8 m/s` and
`|wz| <= 1.0 rad/s`. Asking for more is not an error; it is just clipped.

## The task family this node has to be useful for

{{TASK}}

## What to base the node on

A principle card has been written for this task. **Implement this principle and
nothing else.**

```yaml
{{CARD}}
```

The fields mean what they say, and each one has a job:

- **mechanism** — what happens, and why it helps. This is the node body.
- **trigger** — when it applies. This is the precondition.
- **derivation** — how the principle becomes a rule for a Go2 with a 181-beam 2D
  scan. Where the derivation and your instincts disagree, follow the derivation:
  it is what the card is for.
- **prediction** — the falsifiable claim. **Restate it verbatim** in
  `PREDICTION`: the metric, the baseline and the minimum effect are the card's,
  not yours, and the micro-world you build must be one where that claim is the
  thing under test.
- **node_name** — what the leaf is called. Use it as the class name.

You are not told where this principle came from, and you do not need to know.
A card is a card: extracted from a paper, recalled from one, or devised from
nothing, it is implemented the same way and judged by the same harness.

Implement exactly this principle. If you believe a different mechanism would
score better, that belief is not what is being measured here — a node that
quietly implements something else makes the card untestable.

## The unit scenario

A node is admitted to the library only if it passes a **unit scenario** in
isolation, decided automatically with no human in the loop. You write that
scenario too, and the harness enforces three things you cannot talk it out of:

1. There is a **fixed baseline** — a named reference controller, run over the
   same world as your node.
2. **The baseline must actually fail.** A scenario whose baseline already
   succeeds is rejected as malformed, not passed. It is not testing your node,
   it is testing nothing.
3. **The effect must clear a floor fixed in advance** — half a rate point for
   `goal_reached`, 10% relative for everything else. You may demand more of
   yourself. You may never demand less.

So build a micro-world in which the mechanism you are implementing is the thing
that makes the difference, and in which a robot without it fails.

```python
from harness.micro import MicroScenario, room

world = room(width, height, boxes=[(x0, y0, x1, y1), ...])   # axis-aligned rects
MicroScenario(id="...", world=world,
              start=(x, y, yaw), goal=(gx, gy),
              time_limit=seconds, seed=0,
              goal_known=True)      # False makes it a search task
```

The room is closed by walls 0.15 m thick laid just outside the bounds. Boxes are
`(min_x, min_y, max_x, max_y)`. The robot is a disc of radius 0.3 m and is
rolled out kinematically — it turns and translates exactly as commanded.

A micro-world is rejected if the start or the goal is inside a box or within the
robot radius of one, if the goal is unreachable, if the goal is too close to a
wall, or if `goal_known=False` but the goal is already visible from the start.
Leave room.

The two baselines you may name:

| Name | What it does |
|---|---|
| `goal_seeker` | walks straight at the goal. No sensing at all. Walks into walls. |
| `reactive_avoid` | gap follower: steers at the widest gap that makes progress. |

The metrics the harness can score, and which direction is better:

| Metric | Better |
|---|---|
| `goal_reached` | higher (it is a rate over seeds, 0.0 or 1.0 on one seed) |
| `spl` | higher (path efficiency) |
| `min_clearance` | higher (closest approach to anything, metres) |
| `collisions`, `near_misses` | lower |
| `stuck_events`, `time_stuck`, `recovery_time` | lower |
| `time_to_goal`, `path_length` | lower |

`localisation_failures`, `optimal_distance`, `scenario` and `seed` are **not
available** — no controller can move them, so a prediction about one is not
falsifiable about your node.

## Rules the candidate is checked against

Your module is rejected, automatically, if any of these is false. The rejection
is recorded with its reason; it is not a catastrophe, but it is a wasted node.

- It parses, and it imports **only** from this list:
  `math`, `numpy`, `core.observation`, `nodes.contract`, `harness.micro`.
  Nothing else — no `os`, no file access, no `core.scenario`, no `core.geometry`.
  Your node learns about the world through `obs`, and that restriction is what
  makes "no coupling to a specific scenario" true rather than promised.
- It defines **exactly one** `Condition`, `Action` or `Decorator` subclass.
- Every port type is one of the seven. `EFFECT` and `PRECONDITION` are non-empty.
- The micro-world validates, the baseline fails on it, and the effect clears the
  floor.

## What to return

One fenced ```python block. It contains the three things below, plus anything
"What to base the node on" additionally asked for:

1. **One node class** — a subclass of `Condition`, `Action` or `Decorator`.
2. **`PREDICTION`** — a module-level dict, the falsifiable claim:

   ```python
   PREDICTION = {
       "metric": "goal_reached",     # what gets better
       "baseline": "goal_seeker",    # what it gets better than
       "min_effect": 0.5,            # by at least how much
       "cost_metric": "time_to_goal" # what it makes WORSE. Required, and honest.
   }
   ```

   `cost_metric` may be `""` only if you genuinely believe the mechanism costs
   nothing, and you should be suspicious of that belief.

3. **`def micro(seed=0)`** — returns the `MicroScenario` the prediction is
   tested on.

Put a short docstring on the class saying what mechanism it implements and why
it should work. Write it as you would write code someone else has to maintain:
comments where a choice is not obvious, and no comments restating the line below
them.
