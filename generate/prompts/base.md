{{CONTRACT}}

## The task family this node has to be useful for

{{TASK}}

## What this node is

**This is the BASE node.** It is not derived from any principle, any paper or any
citation, and it is not one arm's work. It is the substrate every behaviour tree
in this project is built on: one node, shared byte-for-byte by every experimental
condition, and the thing every later node is added *on top of*.

So the instruction is simply:

> **Write the best indoor navigator you can, as one leaf.**

No card constrains you. Use whatever you judge works — reactive gap following,
potential fields, a dynamic window, a wall-follower with a bug-algorithm escape,
a hybrid. You are not being asked to demonstrate a mechanism; you are being asked
to *get the robot to the goal* as reliably as you can, in a cluttered indoor
space it has no map of.

Write it as though it is the only node in the tree, because at first it will be.

## The one hard requirement: it must never decline to act

The tree is a `Fallback`, and this node sits **last**:

    Fallback[ specialist_1, ..., specialist_k, BASE ]

Specialists ahead of it may return `FAILURE` when their precondition does not
hold, and control then falls through to you. **You are the last resort.** So:

- **`precondition` must return `True` unconditionally.** Do not override it at
  all, or override it to `return True`. A base that declines leaves the tree with
  nothing driving, the runner emits a zero Twist, and the robot stands still
  until the episode times out.
- **`tick` must always `drive(...)`** and return `Status.RUNNING`. Never return
  `SUCCESS` or `FAILURE` from the base: `SUCCESS` short-circuits the tree with no
  Twist written, and `FAILURE` means "nobody handled this step".

Handle every case yourself, including the awkward ones: no goal disclosed yet,
the goal directly behind a wall, a dead end, being in contact with something, a
scan that is entirely `inf`, and being stuck with no progress for a long time.
There is nothing behind you to catch any of it.

## What good looks like here

The environments are procedurally generated indoor layouts: rooms joined by
doorways roughly 0.9 m wide, freestanding furniture-sized blocks, dead ends, and
routes that require committing to an opening that leads *away* from the goal
before it leads towards it. Straight-line pursuit fails on all of them.

The measure is **how often the robot reaches the goal**, over many different
layouts. Path efficiency matters much less than arriving at all. A base that
gets there slowly and awkwardly on 60% of layouts is far better than one that
takes the optimal path on 30% and jams against a wall on the rest.

Two failure modes are worth designing against explicitly, because they are the
common ones:

- **Local minima.** Heading at the goal drives into the wall between you and it.
  Something has to make the robot commit to an aperture, and keep committing long
  enough to get through rather than oscillating at the threshold.
- **Livelock.** Two nearly equal openings, or a corner that looks equally good
  from both sides, and the robot dithers in place forever. Hysteresis, commitment
  with a timer, or a remembered decision all help; the node may keep state on
  itself across ticks, and `reset()` is where you clear it.

## Rules the candidate is checked against

Your module is rejected, automatically, if any of these is false.

- It parses, and it imports **only** from this list:
  `math`, `numpy`, `core.observation`, `nodes.contract`, `harness.micro`.
  Nothing else — no `os`, no file access, no `core.scenario`, no `core.geometry`.
  Your node learns about the world through `obs`, and that restriction is what
  makes "no coupling to a specific scenario" true rather than promised.
- It defines **exactly one** `Action` subclass. Not a `Condition`, not a
  `Decorator` — only an `Action` may emit a Twist, and this node must emit one on
  every tick.
- Every port type is one of the seven. `EFFECT` and `PRECONDITION` are non-empty.

## What to return

One fenced ```python block, containing:

1. **One `Action` subclass**, the base node. Name the class `BaseNavigator`.
2. Nothing else that is required — no `PREDICTION`, no `micro()`. Those belong to
   nodes that implement a principle and are tested against a claim. This node
   makes no claim; it is measured directly by how often it reaches the goal.

Say whatever you want about your design in prose outside the fence.
