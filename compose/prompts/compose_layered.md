# Compose a behaviour tree

You are assembling a behaviour tree for a quadruped robot from a **fixed set of
existing nodes**. You are not writing code, and you are not designing new
behaviours. Everything the tree can do is already in the list below; your job is
to decide which of those nodes to use and how to arrange them.

## The task

{{TASK}}

## The nodes available to you

Each entry is a node's complete public contract: its kind, its typed ports, the
one thing it does, and when it applies. There is nothing else to know about a
node — no implementation, no rationale, no source. Two nodes with the same
ports are interchangeable as far as wiring is concerned and are told apart only
by their effect.

{{NODES}}

## How a tree is built

Four composites and one leaf form:

- **`layer`** — tick **every** child, in order, whatever each one returns.
  Position is priority and later is higher: the first child proposes a whole
  motion command, and each later child may revise the parts of it that its own
  contract says it writes. **This is how a node supports the robot's existing
  behaviour instead of taking it over**, and it is the arrangement most trees
  should be built around.
- **`sequence`** — tick children in order until one does not SUCCEED. FAILURE or
  RUNNING from any child stops the sequence and is returned. This is "and then".
- **`fallback`** — tick children in order until one does not FAIL. SUCCESS or
  RUNNING from any child stops it and is returned. This is "or else", and it is
  the recovery idiom: a branch that fails hands over to the next one.
- **`decorator`** — wraps exactly one child and governs whether or how often the
  child's behaviour happens. Only a node whose kind is `decorator` may be used
  this way.
- **`leaf`** — one node from the list, ticked directly.

Two rules the tree must satisfy, both checked mechanically before it runs:

1. **Every `in` port must be written by a node that appears EARLIER in tick
   order.** Tick order is depth-first, left to right. A node that reads
   `goal_known` must be preceded somewhere earlier in the tree by a node that
   writes `goal_known`, with the same port type.
2. **A node whose kind is `condition` or `decorator` never moves the robot.**
   Only an `action` emits motion. A tree containing no action cannot drive.

A node's own precondition gates it: when the precondition is false the node
returns FAILURE without doing anything. That is what makes a `fallback` of two
actions with opposite preconditions take turns rather than fight.

**Two actions in one `sequence` almost never both run**, and two in a
`fallback` never do. An action that is driving returns RUNNING, which stops a
sequence and short-circuits a fallback, so the second action ticks only on the
steps where the first did not act. If you put two actions in either of those and
expect them to combine, they will not: one of them will be starved.

**Put them in a `layer` instead.** Every child ticks, and an action that revises
part of the command adjusts what an earlier one asked for. Each node's contract
says which parts of the motion it writes; it writes those and nothing else,
enforced, so two nodes revising different parts cannot conflict. A node whose
precondition is false in a layer changes nothing at all — the command passes
through it untouched — so adding one is safe even where it rarely applies.

**The general-purpose driver goes FIRST in the layer**, because everything after
it revises what it proposed. A node that revises must have something to revise:
put one before any driver and the tree is rejected.

## What to return

**Exactly one fenced `json` block, containing exactly one JSON object with both
keys — `rationale` and `tree` — inside it.** Not one block per key. Not a block
for the plan and another for anything else. If your response contains a second
fenced block of any kind, the composition is rejected without being read, and
the same is true if the single block is missing either key. Write whatever prose
you like outside the block; just do not fence it as `json`. No Python anywhere.

```json
{
  "rationale": "One or two sentences on why this arrangement, in terms of the task and the nodes' effects.",
  "tree": {
    "type": "fallback",
    "children": [
      {"type": "sequence", "children": [
        {"type": "leaf", "node": "some_condition_id"},
        {"type": "leaf", "node": "some_action_id"}
      ]},
      {"type": "leaf", "node": "another_action_id"}
    ]
  }
}
```

Object forms:

- `{"type": "layer", "children": [...]}` — one or more children, ticked in
  order, all of them.
- `{"type": "sequence", "children": [...]}` — one or more children.
- `{"type": "fallback", "children": [...]}` — one or more children.
- `{"type": "decorator", "node": "<id>", "child": {...}}` — exactly one child.
- `{"type": "leaf", "node": "<id>"}`

**`node` must be an id from the list above, spelled exactly.** Any other value is
rejected and the composition fails. If the available nodes cannot do the task,
return the best tree they can form and say so in `rationale` — do not invent a
node, and do not describe one you wish existed as if it were there.
