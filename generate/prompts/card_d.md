# Write one principle card

You are writing a **principle card**: one mechanism, stated so precisely that
somebody who has never seen your sources could implement it and somebody else
could prove it wrong.

The card is not the implementation. A later step turns it into a behaviour-tree
leaf, and that step will be given **your card and nothing else** — not this
prompt, not your reasoning, not whatever you were given below. So everything the
implementer needs has to survive inside the six fields.

## The system the principle has to fit

A Unitree Go2 quadruped navigating an indoor space it has no map of. Below the
behaviour tree sits an inherited, pretrained, **frozen** locomotion policy: the
only thing anything ever sends the robot is a Twist — `(vx, vy, wz)`, saturated
to `|vx|, |vy| <= 0.8 m/s` and `|wz| <= 1.0 rad/s`. Nothing you write touches the
policy, the gait or the joints.

What the robot can sense, once per control step:

```
t                    sim seconds since the episode started
ranges               181 lidar beams over a full circle, metres, inf where no return
range_angles         beam bearings in the ROBOT frame, radians
range_max            6.0 — what "as far as I can see" means
pose_estimate        (x, y, yaw) — the BELIEF about our pose, never ground truth
pose_error_estimate  scalar metres — how wrong we believe that belief is
contact              bool — are we touching something right now
goal                 (x, y), or None meaning NOT DISCLOSED
```

Two of those are traps. `pose_estimate` **drifts** on some scenarios — a seeded
random walk is added to it — and ground truth is never observable. `goal` may be
`None`, meaning the goal exists and the episode is still scored against it but
this robot has not found it yet; it latches once seen.

## The task family this principle has to be useful for

{{TASK}}

<!-- ---- SOURCE BLOCK BEGIN ---- -->
## What to base the principle on

Nothing is supplied. No paper, no reference, no summary.

Devise the principle yourself, from what you already know about making a legged
robot navigate an indoor space it has no map of. Name the mechanism, state the
condition it applies under, and derive it into a rule for these sensors.

Set `source.citation` to the single line
`(no source - the principle was devised, not extracted)`, leave `span` empty, and
do not attribute the mechanism to a paper. If your principle happens to resemble
published work, that resemblance is a finding for somebody else to make, not a
citation for you to claim.
<!-- ---- SOURCE BLOCK END ---- -->

## What a card is

Six fields. `mechanism`, `trigger` and `derivation` are prose. `prediction` is
**structured**, because a prediction that cannot be decided automatically cannot
become a unit test — and this one will be run as one, with no human in the loop.

```yaml
id: <short_snake_case_identifier>
node_name: <TheLeafThisWouldProduce>       # CamelCase, verb-first

source:
  citation: <the reference this principle comes from, or the single line
             "(no source - the principle was devised, not extracted)" if it
             comes from none>
  span: <the section, page, figure or equation number where the mechanism is
         stated — the part that makes a threshold checkable. LEAVE EMPTY if you
         cannot point at a location you are sure of.>

mechanism: <what happens, and why it helps. The finding, not the implementation.>
trigger: <the condition under which it applies -> becomes the precondition port>
derivation: <how that becomes a rule for THIS robot and THESE sensors. Numbers
             belong here, with their units, and so does the reasoning that got
             you to them.>

prediction:
  statement: <the falsifiable claim, in words>
  metric: <what gets better>
  baseline: <goal_seeker or reactive_avoid>
  min_effect: <a number: the smallest effect that would count as support>
  cost_metric: <the metric this mechanism is expected to make WORSE>

confidence: <high | medium | low>
```

### `prediction.metric` — the closed list

The metric must be one the scorer actually computes, and one a **controller can
move**. These and no others:

| metric | better |
|---|---|
| `goal_reached` | higher (a rate over seeds) |
| `spl` | higher (path efficiency) |
| `min_clearance` | higher (closest approach to anything, metres) |
| `collisions`, `near_misses` | lower |
| `stuck_events`, `time_stuck`, `recovery_time` | lower |
| `time_to_goal`, `path_length` | lower |

`localisation_failures`, `optimal_distance`, `scenario` and `seed` are **not
available**: no controller can move them, so a prediction naming one is not
falsifiable about the node and the card is rejected at load.

`cost_metric` is the honest counterpart — the metric this mechanism is expected
to make worse. A mechanism that buys clearance with time should say so. It may
be empty only if you genuinely believe the mechanism costs nothing, and you
should be suspicious of that belief.

### `confidence` — criteria about the SOURCE, not about how sure you feel

- **high** — the mechanism *and* its quantitative form are stated explicitly in
  a source you can point at; the span points at the equation, threshold or
  exponent itself.
- **medium** — the mechanism is stated explicitly somewhere, but the
  quantitative form is adapted: different units, a different body scale, a
  different sensor. The number is yours; the mechanism is not.
- **low** — the mechanism is qualitative and the derivation is entirely yours.
  Still usable, but it is your claim, and the card must not read otherwise.

## Rules

- **One mechanism.** A card describing two is two cards. This is not a style
  rule: the unit scenario that decides admissibility has to isolate one claimed
  effect, and it cannot isolate two.
- **No coupling to a specific scenario.** The leaf will run on worlds nobody has
  seen. The card may not name a room, a coordinate, a doorway or a map.
- **Do not invent a `span`.** An empty span is a result and costs nothing. A
  section number reconstructed from the shape of such papers is a fabrication,
  it will be checked against the document, and it is the single failure this
  pipeline exists to detect.
- **Do not invent numbers.** If a source states a threshold, an exponent or a
  closed form, give it and say where. If it does not, derive one, say the
  derivation is yours, and set `confidence: low`.

## What to return

One fenced ```yaml block containing exactly one card, and nothing else that
looks like YAML. Prose outside the fence — including, if it applies, an honest
statement of what you did not know.
