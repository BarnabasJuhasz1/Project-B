# Write one behaviour-tree node that ADJUSTS an existing navigator

You are writing a single node for the behaviour tree of a Unitree Go2 quadruped
navigating an indoor space it has no map of.

Below the node is an inherited, pretrained, **frozen** locomotion policy. The
only thing anything ever sends the robot is a Twist — `(vx, vy, wz)`. You never
touch the policy, the gait, or the joints.

**You are not writing the navigator.** A navigator already exists, it already
drives, and it will keep driving whether or not your node does anything. Your
node sits above it and revises what it asked for.

Return **one Python module**, in one fenced ```python block, and nothing else
that looks like code. Everything you want to say in prose, say outside the
fence. What the module must contain is set out in "What to return" at the end.

## How the tree runs: a layered command pipeline

Every node in the tree ticks, **every step, in order**. Nothing short-circuits
anything.

    Layer[ ports, BASE, node_1, node_2, ... ]
             ^      ^      ^
             |      |      `- each may revise the fields it declared
             |      `-------- proposes a whole command, every tick
             `--------------- publishes a few facts from the sensors

The **base** is a general indoor navigator. It gets the robot most of the way
most of the time, it handles the awkward cases, and it never declines to act.
Each later node may then adjust individual fields of what the base proposed.

This is subsumption: a competent low layer, refined by narrow higher ones. Your
node is one of the higher ones.

**So the thing you are writing is small.** It does one thing to one part of the
command, under one condition, and the rest of the time it stays out of the way.

## The one hard rule: you declare which fields you may write

Your class declares `WRITES` — the Twist fields it is allowed to touch:

```python
WRITES = ("vx",)            # may bound or set forward speed, nothing else
WRITES = ("wz",)            # may steer, but may not change speed
WRITES = ("vx", "wz")       # both. Only if the mechanism genuinely needs both.
```

This is **enforced, not advisory.** Writing a field you did not declare raises
immediately and the node is rejected. Declare the smallest set your mechanism
actually needs — a node that declares all three is claiming to be a navigator,
and it will be judged as one.

Two ways to write, and they are the only two:

```python
ports.cap(vx=0.35)       # min(|pending|, 0.35), sign kept. Never speeds up.
ports.adjust(wz=0.8)     # replaces the field outright. Last writer wins.
```

Prefer `cap` whenever the mechanism is a limit, a bound, or a "no faster than"
— it composes with other caps in any order, which `adjust` does not.

And you can see what the base asked for:

```python
pending = ports.command          # a Command(vx, vy, wz). Never None when you
                                 # tick, because the base ticks before you.
if pending.vx > my_limit:
    ports.cap(vx=my_limit)
```

**Do not call `ports.drive(...)`.** That writes the whole command and is the
base's job; it raises for a node that narrowed `WRITES`.

## Declining is a first-class outcome, and it is free

Return `Status.FAILURE` from `precondition`, or simply write nothing in `tick`,
and the base's command passes through **byte-identical**. The robot behaves
exactly as if your node were not in the tree.

That matters more than it sounds. Your node's contribution is measured by adding
it to the tree and re-running the same worlds: it can only be credited for the
episodes it changes, and it is blamed for every episode it breaks. A node that
acts only when its trigger genuinely holds is a good node. A node that fires on
every tick has replaced the base rather than supported it, and it will be
measured as having done so.

## The node contract

A leaf is one of three kinds, and the kind is the class you subclass:

- **`Condition`** — answers one question about the world and publishes it as a
  port. Never touches the Twist.
- **`Action`** — the only kind that may touch the Twist. With a narrowed
  `WRITES` it is a *modifier*, which is what you are writing.
- **`Decorator`** — wraps exactly one child and governs whether the child runs.
  Never touches the Twist itself.

Your class declares, as class attributes:

| Attribute | What it is |
|---|---|
| `WRITES` | the Twist fields this node may write. See above. |
| `PORTS` | a tuple of `Port` — the typed interface between nodes. See below. |
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
nodes. This is not a style rule: contribution is measured by adding one node at
a time, and a node with two effects is attributable to neither.

**No coupling to a specific scenario.** Your node will be run on procedurally
generated worlds you have never seen. It may not name a room, a coordinate, a
doorway or a map. Anything it knows about the world it learns through `obs` at
tick time.

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

Three ports are always published before your node ticks, so you may declare any
of them as inputs: `goal_known` (FLAG), `pose` (POSE), `ranges` (RANGES).
Anything else you declare as an input must be published by another node earlier
in the Layer, and if nothing does, the tree refuses to build.

Publish an OUT port when the quantity you computed is genuinely useful to
someone else — a limit, a bearing, a confidence. Do not publish your own
internal state.

### What to import

Exactly these, and nothing else.

```python
import math
import numpy as np

from core.observation import Command
from nodes.contract import (Action, Condition, Decorator, Direction, Port,
                            PortType, Status)
```

Import only what you use. You will usually not need `Command` at all: a modifier
revises fields, it does not construct commands.

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
- **trigger** — when it applies. This is the precondition, and under the layered
  pipeline it is doing real work: outside the trigger your node must leave the
  base's command alone.
- **derivation** — how the principle becomes a rule for a Go2 with a 181-beam 2D
  scan. Where the derivation and your instincts disagree, follow the derivation:
  it is what the card is for.
- **prediction** — the falsifiable claim. **Restate it verbatim** in
  `PREDICTION`: the metric, the baseline and the minimum effect are the card's,
  not yours.
- **node_name** — what the leaf is called. Use it as the class name.

You are not told where this principle came from, and you do not need to know.
A card is a card: extracted from a paper, recalled from one, or devised from
nothing, it is implemented the same way and judged by the same harness.

If you believe a different mechanism would score better, that belief is not what
is being measured here — a node that quietly implements something else makes the
card untestable.

**If the principle genuinely cannot be expressed as an adjustment to an existing
navigator, say so in prose and write the closest honest version you can.** Some
mechanisms are whole navigators and this pipeline cannot represent them; that is
a finding about the corpus, and it is more useful than a node that pretends.

## How this node will be judged

Two steps, both automatic, neither of which you can talk round.

1. **Validity.** Your module is loaded, wired into
   `Layer[ports, BASE, your node]`, and ticked over a rollout. It has to build,
   its ports have to be satisfiable, and it must not raise. Writing a Twist
   field you did not declare fails here.
2. **Contribution.** The tree above is run over a suite of randomly generated
   indoor worlds and compared, on identical worlds, against the base alone. What
   is reported is how many worlds your node **fixed** (the base failed, the pair
   succeeded) and how many it **broke** (the reverse).

You do not write the worlds and you have never seen them. There is no
micro-world to build and no baseline to name — that is deliberate, because a
generator that writes both the mechanism and the world that judges it is
grading its own homework.

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
- `WRITES` names only `vx`, `vy` and `wz`, and the node writes nothing outside
  it.

## What to return

One fenced ```python block. It contains exactly two things:

1. **One node class** — normally an `Action` with a narrowed `WRITES`.
2. **`PREDICTION`** — a module-level dict, the card's falsifiable claim:

   ```python
   PREDICTION = {
       "metric": "goal_reached",     # what gets better
       "baseline": "base",           # what it gets better than
       "min_effect": 0.5,            # by at least how much
       "cost_metric": "time_to_goal" # what it makes WORSE. Required, and honest.
   }
   ```

   `cost_metric` may be `""` only if you genuinely believe the mechanism costs
   nothing, and you should be suspicious of that belief.

**No `micro()`.** Do not write one; it is not read.

Put a short docstring on the class saying what mechanism it implements, which
Twist field it revises and why, and when it declines. Write it as you would
write code someone else has to maintain: comments where a choice is not obvious,
and no comments restating the line below them.
