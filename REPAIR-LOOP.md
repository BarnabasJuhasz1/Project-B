# REPAIR-LOOP.md — the autonomous diagnose-and-repair pipeline

**Written 2026-09-16, on the user's instruction. Everything else is on hold until the core of
this is running — see `plans/hold/` and `plans/README.md`.**

⚠️ **This document is the WHY and the WHAT. The HOW is `plans/06-epic-repair-loop.md` and its
chunks.** Read this before touching any of them: the chunks reference these definitions rather
than repeating them.

---

## The thing that happened, and why hand-fixing it is worthless

On 2026-09-15 the node `gen_b_44d12df4d25b437f` (arm B, ant path integration, Müller & Wehner
1988) was run under physics on generated maze world 2004. It was admitted as valid. It fired on
**97.4%** of ticks. The robot **froze in open space** — 46 cm of clearance, zero contacts, zero
collisions — for the last 52 seconds of a 120 second episode, inside a box 4 cm × 3 cm, and
timed out having made **no net progress toward the goal**. The base alone solves that world.

⚠️ **2026-09-18, and it does not weaken the case — it sharpens it.** *"The base alone solves that
world"* is a **kinematic** fact. Under physics the base **also fails** on 2004: it times out at
120 s having walked 30.1 m against a 16.2 m optimum
(`results/gen-maze-02004__base__seed2004.json`, 2026-09-15, uncapped). The growth pairing is
kinematic throughout and internally consistent, so every verdict stands for the suite it was
measured on — but no prose anywhere may say the base *solves* 2004 without saying kinematically.
Recorded in the 2026-09-18 CORRECTION of `plans/done/58-repair-loop.md`; the repaired node's own
physics run is `out/evidence/nodes/gen_b_44d12df4d25b437f__r01/physics-2004/`.

The diagnosis took a session. It is worth writing down what the diagnosis actually *was*,
because it is the shape of every diagnosis this loop has to produce:

1. The node's trigger is a **latch, not a gate** — `_travel >= 1.0 m` accumulates monotonically
   and nothing inside an episode can reset it, so once armed it can never disarm.
2. The believed position ran to (−2.1, 12.7), **off the map**, while the robot sat at
   (9.67, 0.94). That is not a coding error: it is the paper's own model doing the thing the
   paper says it does. The card said so — *"a known, signed, systematic error, largest for
   near-reversals"* — and named the condition under which it is affordable: *"left and right
   turns are equally frequent in nature and their opposite-signed errors cancel."* **A maze
   corridor does not deliver that. Nothing checked.**
3. The node overwrote the base's steering (`ports.adjust` is last-writer-wins) while only
   *capping* forward speed — and the cap sat at 0.80 and never bit once. The base's rule is
   "don't move until you are pointed at the gap you chose, and turn hard until you are." The
   node destroyed the turn and left the gate. **99% of the 4,447 slow steps had the base's
   turn-in-place floor overwritten.** The base withheld forward speed pending an alignment it
   was no longer permitted to reach. That is the freeze.

Three separate defects, at three different layers — trigger design, applicability, and
arbitration — and only one of them is a bug in the ordinary sense.

**Fixing this node by hand fixes this node, in this environment, once.** The point of the
project is a pipeline. So the pipeline has to be able to do what that session did.

---

## What the loop must actually decide

The naive loop is "measure, if worse then regenerate, repeat". It is wrong, and it is wrong in
a way that would quietly destroy the experiment. **A worse number is not a broken node.**

Four distinct situations produce a worse number:

**1. The node is genuinely broken.** It stops the robot doing the task, in a way nothing in its
card claims or predicts. The case above. Repair.

### ⚠️ The case that proves the point, found 2026-09-16 while writing this

`to_scenario()` in `core/worldgen.py` never sets `drift`, and `Scenario.drift` defaults to
`None`. **Every world in the growth suite hands the controller a perfect pose estimate.**

The ant node's card predicts, in its own words:

> *"**On scenarios where pose_estimate drifts**, a robot steering to a goal bearing reconstructed
> from this Eq.-3 integrator ... reaches the goal on a strictly larger fraction of seeds than the
> same robot steering to the bearing recomputed each step from pose_estimate directly, because a
> 1/l_n-damped accumulator fed by proprioception does not inherit the position random walk."*

The mechanism exists to survive a hazard. **The hazard was switched off.** The node has been
measured, ranked, called a regression and written up as evidence for B−A across twenty worlds,
none of which contain the condition its card names in its first clause.

This is not the geometry being wrong for the mechanism — it is subtler and worse. It is the kind
of thing a human notices once, in one session, on one node, having read the card and the world
generator in the same hour. **There are fifteen other nodes and nobody has read their cards
against the worlds they were scored on.** That is the argument for this epic in one paragraph.

⚠️ It also means `PROBE_ENVIRONMENT`'s search axes are **not** just (tier, seed). Drift, goal
disclosure and sensor noise are scenario axes that a card can predicate on, and the probe has to
be able to turn on the hazard a card names.

**2. The node did nothing, because there was nothing here for it to do.** Its trigger never
fired, or fired and changed nothing. That is not evidence of a defect — a geometric
reorientation node in a corridor with no corners has correctly declined. **You cannot judge it
until you have put it somewhere its capability is actually on the table.** Repairing it here
would be repairing it against noise.

**3. The node made a metric worse and that was the point.** The card's Prediction names the
metric it moves and the metric it spends. A collision-avoidance node that stops the robot short
of an unreachable goal has *succeeded* — `goal_reached` fell, and the alternative was a wreck.
A legibility node produces a longer path by construction and scores negatively on SPL. **Judging
these on goal-reach is judging them on a claim they never made.** This is already a live problem
and already measured: 6 of the 16 nodes in growth round 2 predict `collisions`, not goal-reach,
and round 1 scored `goal_reached` only.

**4. The node is faithful, competent, and the paper does not transfer.** Research papers are
theoretical, and are frequently missing exactly the thing a robot needs. It is entirely possible
that implementing a paper to 100% fidelity never produces a positive result on this benchmark.
**That is a finding, and it is one of the more interesting findings the project can produce.**
It must be recorded as data, with the evidence that led to it — never as a loop failure, never
as something to keep grinding against until the budget runs out.

A loop that cannot tell these four apart is not a repair loop. It is a random walk with a model
call in it.

---

### ⚠️ The second case that proves the point, found 2026-09-16 while building chunk 58

The shared base publishes three ports every tick — `nav_heading` (the direction it has chosen),
`nav_stuck` and `nav_progress`. **`generate/prompts/modifier.md` never mentions them.** It tells
every model:

> *"Three ports are always published before your node ticks, so you may declare any of them as
> inputs: `goal_known` (FLAG), `pose` (POSE), `ranges` (RANGES)."*

So **no generated node in any arm can ever read what the base intends** — not because any model
failed, but because none was told the ports exist.

Verified directly: a node declaring `nav_heading` and `nav_stuck` as IN ports **builds and ticks
correctly**, the values are live and vary, and `nav_stuck` does go true. The capability is
complete and works. It was simply never advertised.

This reframes the arbitration failure that motivated the layered pipeline. A node that overwrites
`wz` without consulting the base is not a node that ignored the base's intent — it is a node that
**had no way to know the base had one**. Every node in the library was written blind to it.

Three consequences:

1. It is **shared across all arms**, so it does not bias B−A. It caps what *every* arm could
   achieve, which is different and worse: the ceiling was set by the prompt, not by the corpus.
2. ⚠️ **It directly shapes `REGENERATE_NODE`.** A repair prompt that does not advertise these
   ports will reproduce the same class of defect, and the loop will burn its budget rediscovering
   it. Chunk 58's repair prompt must carry them.
3. It is the strongest argument yet for the deferred `REVISE_SHARED_PROMPT` action — and a clean
   example of why that action is worth having, since no amount of per-node repair fixes a ceiling
   set in a shared prompt.

⚠️ **Raise this in `generate/FINDINGS.md` before any G1 number is reported.** It bears on what the
library's node count actually measures.

## The verdict taxonomy

⚠️ **Verdicts are produced by RULES OVER MEASUREMENTS, in code. The model proposes a CAUSE and
an ACTION; it does not get to declare the verdict.** Same discipline as `core/stats.py:decide()`
being the rule and the prose being a description of it. An agent that both diagnoses and grades
is marking its own homework, and the taxonomy stops being falsifiable the moment it can.

| Verdict | Mechanical definition | What the loop does |
|---|---|---|
| **HELPED** | The node's **own claimed metric** (card `prediction.metric`) moved past its own `min_effect`, in the claimed direction, **while still doing the task** | Exit. Promote. |
| **INERT** | Fired rarely, or fired and left the command byte-identical (`changed` ≈ 0) | **Not a defect.** → environment probe |
| **MISJUDGED** | Scored metric worse, but the claimed metric improved and the trade is the one the card declared | Re-score against the claim. Escalate to a **metric gap**, not a node repair |
| **REGRESSED** | Worse, with no claim in the card covering it | Repair |
| **DEGENERATE** | A severe subtype of REGRESSED: the tree stopped doing the task at all — base succeeded on the same world, augmented tree made no net progress, and the failure is not a collision | Repair, highest priority |
| **UNTESTABLE** | No world in reach both fires the trigger and leaves the base headroom | Terminal. Record with the search that was done |
| **NOT-TRANSFERABLE** | Repaired to fidelity, budget spent, no effect found anywhere reachable | Terminal. **This is a result, not an error** |

Two rules that are not negotiable:

- ⚠️ **INERT IS NOT A SUCCESS.** The cheapest way to make a node stop hurting is to make it
  decline everywhere. A loop that is scored on "no longer regresses" will discover this
  immediately and every node will converge on doing nothing. A node that helps nothing has not
  been repaired.
- ⚠️ **Say `n` out loud.** Every verdict carries its sample size. A single-seed result that
  looked like a finding in chunk 22a did not survive five seeds.

---

## The repair action space

| Action | In the core? | Cost | Notes |
|---|---|---|---|
| `REGENERATE_NODE` | **yes** | one model call | Re-make the node call from **the same card**. `scripts/regenerate_modifiers.py` already does exactly this. Leaves the independent variable untouched |
| `PROBE_ENVIRONMENT` | **yes** | sim-free, seconds | Search `core.worldgen`'s (tier, seed) space for a world that fires the trigger and leaves the base headroom |
| `ACCEPT` / `NOT_TRANSFERABLE` / `UNTESTABLE` | **yes** | free | Terminal verdicts, with a written reason. The loop must be able to stop and say why |
| `REGENERATE_CARD` | **deferred** — see below | one model call | ⚠️ Moves the independent variable |
| `REVISE_SHARED_PROMPT` | **deferred** | full library regeneration | The only structurally fair fix across arms, and the largest blast radius |

### Deferred: `REGENERATE_CARD` — a documented extension, not built

**Flagged as supplementary on the user's instruction, 2026-09-16. Do not implement in the core.**

It is the right fix for a real defect class, and there is already a confirmed instance. The
ant card flagged its own uncertainty honestly:

> *"The paper also gives a step-length update, Eq. 1b, but that line is not legibly reproduced
> in the text I was given, so I use the unambiguous l_{n+1} = l_n + 1 and flag that this differs
> from the paper if Eq. 1b contains a turn-dependent term."*

The card was telling the truth. `pdftotext` destroyed that equation —
`out/corpus/path_integration_ants.e9f40cbbd48a.txt` carries the `[1b]` label with its body gone.
So the defect is in **corpus extraction**, upstream of the card, and no amount of node
regeneration reaches it.

⚠️ **Why it is still deferred:** cards are reused byte for byte precisely so that the execution
model and the independent variable never move at the same time. Regenerating a card cannot be an
edit — it has to land as a **new row with its own `plan_hash` and its own lineage**, because
`plan_hash` is a pure function of (condition, task, card) and a changed card is a different row
key. That is a second experiment, and it deserves its own chunk.

---

## Arm-blindness — decided 2026-09-16, and load-bearing

⚠️ **The repair loop is a SECOND GENERATION STAGE, and it can eat B−A without anyone noticing.**

If the loop can tell which arm it is repairing, it can — deliberately or not — try harder for
one of them, and B−A stops measuring extraction and starts measuring how hard the loop worked.
The existing pipeline solves this structurally: `render_node(card, task)` has **no condition
parameter**, and a test pins that. The repair loop gets the same treatment.

**The diagnosing agent never sees the condition.** This is not a convention, it is a scrub with
a test behind it. The obvious tell is the card's `source.citation` field — it names the paper,
which names the arm. Less obvious: `source.verified`, `confidence`, and the whole `generated:`
block (`condition`, `treatment`, `ref_id`, `paper_sha256`).

The projection already exists and is already correct: **`generator.card_yaml` renders mechanism,
trigger, derivation, prediction and node_name only** — precisely the fields the card→node table
calls the node body, and precisely not the metadata. The evidence bundle reuses it rather than
inventing a second scrubber.

**Budget.** A fixed number of repair rounds per node, identical across arms, declared before any
cross-arm run. Deferred only in the sense that the *number* is set once the first runs show what
a repair actually costs — the mechanism ships in the core, the constant is written down before
it is used for a comparison.

⚠️ **Consequence to declare in the write-up:** the node prompt is no longer byte-identical
across nodes, because a repair round appends a node-specific diagnosis. It remains identical in
*procedure* — same prompt, same blind bundle, same budget, same rules — and that is the claim
being made. It is weaker than the claim the first-generation prompt supports, and it is stated
rather than glossed.

---

## The traps

**1. The loop is gradient descent on its own test set.** Repairing against the worlds you then
score on is fitting the test set, and it is the same argument that produced the dev/held-out
split. The loop gets a **repair set** and a **confirmation set**, disjoint seeds, and the
confirmation set is looked at **once**.

⚠️ **`PROBE_ENVIRONMENT` makes this materially worse**, because it lets the loop go shopping for
a world where its node happens to win. Guard: a probed world is **committed before** the node is
repaired against it, and confirmation seeds are drawn from a range the probe cannot reach.

**2. The sealed split is not in play, at any point.** `core/scenarios_held_out.py` must never
enter this loop, by the same AST guard that protects generation and composition. Generated
worlds are already safe by construction: `all_scenarios()` never returns one, they are in no
split and appear in no pre-registration constant.

**3. Repair oscillation.** Node → repaired → back to something behaviourally identical to where
it started. Every round records a behaviour signature; a repair that reproduces an earlier
signature terminates the lineage rather than spending the rest of the budget rediscovering it.

**4. A verdict from one world is not a verdict.**

**5. The model marks its own homework.** Addressed by the rules/proposal split above. Worth
restating because it is the easiest thing to erode: the moment a convenient wrapper lets the
diagnosis text set the verdict, the taxonomy is decorative.

---

## What "working" means — the exit conditions

A node leaves the loop in exactly one of these states, and **all five are data**:

| Exit | Meaning |
|---|---|
| `HELPED` | Cleared its own claim on the confirmation set, at a declared `n` |
| `ACCEPT` | Valid, harmless, does something somewhere — kept without a positive result on this suite |
| `UNTESTABLE` | No reachable world discriminates it. The search is recorded |
| `NOT_TRANSFERABLE` | Faithful, repaired, and the paper does not carry over. **A finding** |
| `UNREPAIRED` | Budget exhausted. Recorded with every round's evidence |

None of them is an exception, a crash, or a retry. A loop that can only terminate on success
will grind forever against a paper that does not transfer, and will report that grinding as
progress.

---

## The first run

**Arm B, task `navigate_to_goal`, node `gen_b_44d12df4d25b437f`** — the ant path-integration node
diagnosed above.

Chosen deliberately: the diagnosis already exists in full, so **the loop's first output can be
checked against a known-good answer**. That is the only cheap way to find out whether the loop
reasons correctly rather than merely runs. If it does not independently find the latch, the
arbitration conflict, or the unchecked applicability condition, the loop is not working — and
that is a much clearer signal than any aggregate.
