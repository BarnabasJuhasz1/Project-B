# Rewrite a node that a previous attempt got wrong

A node was generated from the principle card below, measured on a suite of
procedurally generated worlds, and it **made the robot worse**. You are writing
the replacement.

The card is not in question. The card is fixed and you must still implement it.
What failed was the implementation.

## The previous attempt

```python
{{PREVIOUS_SOURCE}}
```

## Why it failed

This is a review of the code above against the card it was generated from,
written by someone who read both alongside the measured trajectories.

{{DIAGNOSIS}}

## ⚠️ Ports the base publishes — read these

The previous attempt could not have known about these; they were not documented
when it was written. **The shared base publishes three ports on every tick**,
before any later node runs, and you may declare any of them as `Direction.IN`:

| Port | Type | Meaning |
|---|---|---|
| `nav_heading` | `BEARING` | the direction the base has decided to travel, robot frame |
| `nav_stuck` | `FLAG` | the base believes it is stuck or off its plan |
| `nav_progress` | `CONFIDENCE` | 1.0 falling toward 0.0 as time passes with no progress |

⚠️ **Why this matters more than it looks.** The base gates its own forward speed
on being aligned with `nav_heading`, and raises its turn rate to a floor until it
is. A node that overwrites `wz` without reading `nav_heading` deletes the base's
ability to reach the alignment its own speed gate is waiting for — the robot then
stands still, in open space, indefinitely. Taking `wz` and merely *capping* `vx`
produces exactly this deadlock.

If your node steers, prefer a **bounded correction** to `nav_heading` over a
replacement of it, and **decline** when `nav_stuck` is true — a node with no
obstacle competence must yield to the component that has it.

---

Everything below is the standard node specification. It is unchanged, and it
governs. Where it and this page disagree, this page is the addition, not a
replacement.

---

{{NODE_PROMPT}}
