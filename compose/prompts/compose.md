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

Three composites and one leaf form:

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

**Two actions in one `sequence` almost never both run.** An action that is
driving returns RUNNING, and RUNNING stops the sequence — so the second action
ticks only on the steps where the first returned SUCCESS, which for a driving
action usually means only when it has finished. If you want one action to modify
another's motion, that has to happen through a port the second one READS; if the
second one declares no such input port, putting them in a sequence does not
combine them, it starves the second one. The contracts above tell you what each
node reads and writes, and that is the only channel through which two nodes
affect each other.

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

- `{"type": "sequence", "children": [...]}` — one or more children.
- `{"type": "fallback", "children": [...]}` — one or more children.
- `{"type": "decorator", "node": "<id>", "child": {...}}` — exactly one child.
- `{"type": "leaf", "node": "<id>"}`

**`node` must be an id from the list above, spelled exactly.** Any other value is
rejected and the composition fails. If the available nodes cannot do the task,
return the best tree they can form and say so in `rationale` — do not invent a
node, and do not describe one you wish existed as if it were there.
