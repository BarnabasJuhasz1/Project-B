# Diagnose one behaviour-tree node

You are reviewing one node in a robot navigation system. It has already been
measured. Your job is to say **why** it behaved as it did, and what should be
done about it.

## How the system works

A robot is driven by a `Layer` of nodes. Every node ticks, every step, in order.

1. A shared **base navigator** runs first and proposes a complete velocity
   command `(vx, vy, wz)`. The base has lidar, avoids obstacles, and can solve
   many worlds on its own. **The base is identical in every experiment and is
   never modified.**
2. Later nodes **revise** that command, and may only touch the fields they
   declared in `WRITES`. A node that declines leaves the base's command
   byte-identical.

The node below was added after the base and measured against **the base alone**
on the same worlds. It is generated from a principle card extracted from
research literature.

## What you must not assume

- **A worse number is not automatically a broken node.** A node may correctly
  trade one metric for another it actually claims, or may have been measured
  somewhere its capability was never on the table.
- **The research paper may simply not transfer.** Papers are theoretical and
  often omit what a robot needs. "Implemented faithfully, does not help here"
  is a legitimate conclusion and one of the more useful ones. Do not force a
  repair that the evidence does not support.
- **The card may ask for something the node did not do.** This is the check
  nothing automatic can make: read the card's own Trigger, Mechanism and
  Derivation, and say whether the node actually implements them. Cards
  frequently state a precondition, a priority, or an applicability condition
  that the implementation quietly ignores.

---

{{BUNDLE}}

---

## What to return

Return **exactly one fenced `json` block** and nothing else. No prose outside
it.

```json
{
  "cause": "One paragraph. What is actually wrong, or actually right, stated mechanically. Refer to specific lines, ports, numbers or trajectory rows.",
  "card_departures": [
    "Each place the node does NOT do what its own card says. Quote the card. Empty list if it is faithful."
  ],
  "evidence": [
    "Specific pointers into the material above that support the cause."
  ],
  "action": "REGENERATE_NODE",
  "action_reason": "One sentence on why this action and not the others.",
  "guidance": "If REGENERATE_NODE: what the replacement must do differently, as instructions to whoever writes it. Empty string otherwise.",
  "confidence": "high"
}
```

`action` must be exactly one of:

| value | when |
|---|---|
| `REGENERATE_NODE` | the card is sound; the implementation is not |
| `PROBE_ENVIRONMENT` | this was measured somewhere its capability was not on the table |
| `ACCEPT` | it behaved as the card predicts; the outcome is correct |
| `NOT_TRANSFERABLE` | faithful and competent, and the principle does not carry over here |

`confidence` must be exactly one of: `high`, `medium`, `low`.
