# Chunk 15 findings

## 0. What the four real generation calls produced

Four `claude -p` calls against `claude-opus-5`, one task family
(`navigate_to_goal`), transcripts committed under `generate/transcripts/`:

| Condition | Input | Node | Outcome |
|---|---|---|---|
| **B** | `limit_speed_dwa` | `LimitSpeedToVisibleRange` | **ADMITTED** — `collisions` 0.0 vs baseline 2.0, effect 1.0 ≥ 0.5 |
| **B** | `vague_adaptive_motion` (fixture) | `MoveAdaptively` | **ADMITTED** — see §1. Excluded from the pass rate. |
| **A** | no literature | `FollowBoundaryToGoal` | **REJECTED** — `UNIT_SCENARIO_FAILED`, effect 0.000 |
| **D** | invented principle | `CommittedDetour` | **REJECTED** — `UNIT_SCENARIO_FAILED`, effect 0.000 |

**Do not read an effect into this.** N = 1 per condition. Both rejections were
checked by hand and are genuine node failures rather than harness artefacts:
A's node travels 27.8 m in its own 60 s world (optimal 8.06 m) without arriving;
D's node holds `wz` at the 1.0 rad/s ceiling indefinitely and circles, covering
0.32 m. Both designed a world their own node could not solve, which is worth
recording on its own — the generator is not reliably able to test what it wrote.

Condition D's `PRINCIPLE` block came back correctly populated (mechanism,
trigger, derivation), so the "D produces cards-without-sources rather than
nodes-without-cards" design works as intended.


## 1. The admission harness cannot detect card infidelity — a validity threat to condition B

**The deliberately-bad card was ADMITTED.** `generate/tests/fixtures/vague_adaptive_motion.yaml`
has a contentless Mechanism ("move intelligently, adapting to the environment"), an
unconditional Trigger, and a Derivation that restates the Mechanism. The pipeline
was run on it twice, with identical inputs:

| Run | Prompt hash | Outcome |
|---|---|---|
| 1 | `45a8af…` (first call) | **NO_CANDIDATE_EMITTED** — the model refused, and said why |
| 2 | `45a8af4b86a63990` | **ADMITTED** — `goal_reached` 1.000 vs baseline 0.000 |

Run 1's refusal diagnosed run 2 in advance, and is worth quoting in the write-up:

> If I write a node body, every line of it is my invention wearing the card's name.
> […] Whichever I picked would be the thing under test, and the card would get
> credited for a result it did not predict. The scenario would be well-formed and
> the number would clear the floor, and the number would mean nothing.

Run 2 then did exactly that. The admitted node's own `EFFECT` describes clearance-weighted
heading selection plus a commitment-to-boundary-following mode switch on stalled progress —
**none of which appears anywhere on the card**.

**Why the harness cannot catch this.** `harness/unit.py` tests whether the NODE works, not
whether the node implements the CARD. Those are different claims and only the first is
mechanical. A model handed a vague mechanism fills the gap with a real one, and the real one
passes on its own merits.

**Why this threatens G2.** B−D is supposed to separate genuine information gain from the model
restating what it already knew. If B's nodes are partly model-invented mechanisms attributed to
cards, B's advantage is manufactured by the attribution rather than by the corpus — and it would
look identical in the numbers. **The experiment design already contains the mitigation**: it is
precisely why the plan says B−D matters most, and why `limit_speed_dwa` was written as the
internal control. What is needed is to say so explicitly, not to add machinery.

**For chunk 17.** Card fidelity must be declared: either (a) a stated limitation, with B−D as the
control that absorbs it, or (b) a pre-registered fidelity check. It cannot be left implicit.

**For chunk 16.** A cheap partial check exists and belongs with composition: run each admitted
node on the OTHER admitted nodes' micro-worlds. A node implementing a separable mechanism should
degrade gracefully; one tuned to a self-designed trap should collapse.

## 2. The generator designs its own micro-world, so the effect floor can always be cleared

`harness/unit.py`'s requirement 2 — the baseline must actually fail — was written to stop a
generator writing a test that was already true. It has a second effect nobody intended: it
**incentivises an adversarial world**. The model is free to construct a trap that defeats
`reactive_avoid`, and then any competent node clears the floor. Both the good card and the bad
card did this. The requirement is still right; it is just not sufficient, and the write-up should
not present a cleared effect floor as evidence the mechanism generalises.

## 3. Generation is reproducible, not repeatable, and temperature cannot fix it

`temperature` was removed from the Messages API on Opus 5, Fable 5, Sonnet 5, Opus 4.8 and 4.7 —
passing it is a 400. Chunk 15's "temperature 0 produces a byte-identical candidate" is therefore
unavailable on any current model. The two runs of the bad card above are the demonstration: same
prompt, same model, two different outcomes.

Replaced by the transcript: every call writes `generate/transcripts/<prompt_hash>.json` and
`ReplayBackend` rebuilds a library byte-for-byte from committed files with no model, no key and
no network. That is a stronger reproducibility property than temperature 0 would have given, and
it is the one a container run should use.

## 4. Chunks 12 and 14 disagreed about source spans

`nodes/contract.py` required a CARD-origin node to carry a source span; chunk 14 deliberately
left every pilot card's span empty, because there is no corpus on this machine. A condition-B
node was therefore unconstructible, and both ways out were lies — substitute the citation for the
span, or record a corpus node as INVENTED.

Fixed in `nodes/contract.py` by separating **origin** (where the node came from) from
**traceability** (whether anyone opened the paper). A CARD node must name its card and confidence;
the span stays optional, and `is_traceable` still demands it. An unverified card now yields a node
that is CARD and untraceable — the honest state, counted in G1's denominator and not its numerator.

---

## ⚠️ 2026-09-06 — condition A above is the RETIRED naive arm

Everything in §0 and §1 describes conditions as they were defined before 2026-09-06, when **A
received no literature at all**. A is now the **citation arm**: it is handed a card's citation
and none of its content. See `PRE-REGISTRATION.md`'s 2026-09-06 append.

Read the findings with that substitution in mind:

- §0's A row (`FollowBoundaryToGoal`, REJECTED) is evidence about the **naive arm**, not about
  condition A. So are the four A records generated on 2026-09-05; all are marked
  `retired_reason` in `generate/staging/` and excluded from the pass rate and the library.
- **§1 and §3 are unaffected and still bind.** Card infidelity is a property of the harness, not
  of a condition's inputs, and it now applies to A as well — a citation node's mechanism is the
  model's own recall wearing the paper's name, and nothing here can check it against the paper.
  If anything, **A needs the caveat more than B does**: B's node can at least be diffed against a
  card someone wrote, and A's cannot be diffed against anything.
- §2 is unaffected: the generator still designs its own micro-world under every condition.

**A new finding is now possible and has not been measured.** A's prompt asks the model to say so
when it does not recognise a reference. Whether it does, and whether an unrecognised reference
produces a worse node than a recognised one, is exactly the confabulation question the provenance
axis exists for — and it is answerable from A's transcripts alone, with no episodes.

## ⚠️ 2026-09-08 — generation is now TWO calls, and the findings above are about the first pipeline

Everything above was produced by a **one-call** pipeline: a condition's prompt carried a card, a
citation or nothing, and the model wrote a node straight from it. Chunk 42 split that in two —
each arm writes its own principle card, then **one shared node prompt** implements it. See
`PRE-REGISTRATION.md`'s 2026-09-08 append and `plans/done/42-card-first-generation.md`.

Read the findings above with that in mind:

- **§1 (card infidelity) is unaffected in substance and sharper in form.** The harness still
  cannot check a node against the card it claims to implement. What changed is that *every* arm
  now has a card to be unfaithful to, including D — so the threat is symmetric across the
  ablation instead of being B's alone, and `admit.py` records
  `prediction_restated_faithfully` for all three.
- **§2 is unaffected.** The generator still designs its own micro-world under every condition.
- **§3 is unaffected and now covers a chain.** Replaying the card call reproduces the card, which
  renders the node prompt, which replays the node call. `generate/tests/test_replay.py` exercises
  the whole chain against a hand-built fixture pair with no model.
- **§0's table is a v1 table.** `pass_rate` refuses to pool pipelines; read those numbers with
  `pass_rate(pipeline_version="v1-single-call")`.

### What the split makes newly measurable, and what it costs

**Newly measurable.** The card is now an artefact for every arm, committed under
`generate/cards/<condition>/`. So "what did each arm actually believe the principle *was*" is
readable directly, per row, without inferring it from the node's code — and A's card against B's
card, for the same paper, is `generate/probe.py`'s `B − E` diff computed on the live pipeline
rather than in a side experiment. **The probe and A's card call now produce the same kind of
artefact**; whether the probe should be folded into A is a real simplification and is deliberately
not taken here.

**What it costs, stated plainly.** Two model calls per row instead of one, and a second place a
row can fail: a card call that produces no card stages a refusal and never reaches the node call.
That refusal belongs in G1's denominator — the card prompts invite an honest "I do not know this
paper" — but it means A's pass rate now mixes *two* failure modes, "would not write a card" and
"wrote a card that would not admit", and the reason code is what separates them.

### The finding the split produced before a single model call was spent

The no-source arm was told to write `(no source: the principle was devised, not extracted)` into
`source.citation`. **That is a YAML scanner error** — `mapping values are not allowed here` — so
every card D was *instructed* to write would have been refused by the loader, and the arm would
have shown a 0% card rate for a punctuation mistake the prompt asked for. Caught while building
the replay fixture, which is what a fixture is for. The sentinel now uses a dash.

## ⚠️ 2026-09-12 — the model authors the mechanism, the trigger AND the world that judges them

Finding 2 said the generator designs its own micro-world, so the effect floor can always be
cleared. Ten B rows and five A rows later the failure is the **opposite one**, and worse.

**Nothing has ever cleared the contribution floor.** Seven nodes across A and B have had
contribution measured; `contribution_rate` is 0.0 in both arms. Three were flagged
`MALFORMED: the fixed baseline already succeeds on this micro-world, so the scenario cannot
distinguish the node from doing nothing`; the rest scored effect 0.000. One node
(`gen_b_56349a5b3270ba5f`) was ADMITTED with its precondition firing **0/600 ticks**, having
never driven. **Admission is a validity check and nothing more.**

### Preconditions are per-node and model-written — and that is the problem

`Node.precondition` (`nodes/contract.py:396`) defaults to `return True`; **all 15 generated
nodes override it**, so there is no shared trigger. They differ enormously — B's
`limit_speed_dwa` is `obs.goal is not None and not obs.contact`, while A's tests for an
obstacle inside `TRIGGER_RANGE` in the forward half-plane, and A's `reorient_geometry` is
stateful with a threshold and a cooldown.

⚠️ **`precondition_fired` is therefore NOT comparable across nodes.** Each fraction was
measured in that node's *own* micro-world, written by the same model that wrote the trigger.
A's `limit_speed_dwa` has a genuinely selective trigger and still fired 600/600, because the
world it was given always has an obstacle in range. A's `reorient_geometry` fired 0/600
because `pose_error_estimate` never crossed its threshold in the world it was handed — not
because the node is inert.

**Consequence for any A/B comparison:** "A's node never fires, B's always does" cannot be read
off these numbers. Two self-authored worlds are not a common measurement. Fire fractions and
contribution effects only mean something in a **shared world neither arm's model wrote**, and
rolling the pairs through the three dev scenarios kinematically would establish that for free,
sim-free, before any Isaac Sim time is spent.

### First shared-world measurement of an A/B pair, 2026-09-12

`scripts/plot_kinematic.py` now does the shared-world check the paragraph above asks for.
The `limit_speed_dwa` pair — same paper, one arm given the citation and one the PDF — rolled
kinematically through the three dev scenarios, a world **neither model wrote**:

| scenario | A `gen_a_fac8c7b94fe6b1c6` | B `gen_b_c2c95bcb965e342e` |
|---|---|---|
| doorway_traverse | failed | REACHED, spl 0.598 |
| blind_corner | failed (1 collision) | REACHED, spl 0.381 |
| return_home | failed | failed |

**A 0/3, B 2/3.** Both had scored `precondition_fired = 1.0` and `contribution effect 0.000` in
their own micro-worlds, i.e. indistinguishable — the shared world separates them immediately.
B's paths are inefficient (a wide loop on `blind_corner`), so this is not a quality verdict.

⚠️ **n = 3, kinematic, no locomotion policy, dev split. Not reportable, and not a substitute
for physics** — it is exactly the triage the finding above calls for, and it says the
`limit_speed_dwa` pair is worth spending Isaac Sim time on.

## ⚠️ 2026-09-14 — no generated node MODULATES. Preconditions are binary, and B−A is zero.

Growth round 1: each admitted node placed in `Fallback[specialist, BASE]` against the base
alone, 20 `maze` worlds, paired on seeds, kinematic. Base alone 10/20 = 50%.

| node | arm | rate | delta | fixed | broke |
|---|---|---|---|---|---|
| gen_b_44d12df4 | B | 0% | −50% | 0 | 10 |
| gen_b_56349a5b | B | 50% | +0% | 0 | 0 |
| gen_b_78bda1e5 | B | 25% | −25% | 0 | 5 |
| gen_b_8fec33b6 | B | 0% | −50% | 0 | 10 |
| gen_b_b561d9c7 | B | 50% | +0% | 0 | 0 |
| gen_b_c2c95bcb | B | 0% | −50% | 0 | 10 |
| gen_b_dc50c330 | B | 50% | +0% | 0 | 0 |
| gen_a_c4bc4dc7 | A | 50% | +0% | 0 | 0 |
| gen_a_d17f79d4 | A | 0% | −50% | 0 | 10 |
| gen_a_f5f830d1 | A | 50% | +0% | 0 | 0 |
| gen_a_fac8c7b9 | A | 0% | −50% | 0 | 10 |

**Not one node improved the tree. Not one fixed a single world the base failed. `fixed` is 0
in every row, for both arms.**

### The mechanism, measured rather than inferred

Precondition fire rate inside the growth tree, one maze world, 6000 ticks:

| fire rate | nodes | base drove | result |
|---|---|---|---|
| **0.0%** | 5 | 6000 | inert — the base does everything, delta exactly +0% |
| **87.6-100%** | 6 | 0-742 | **replaces** the base — the tree scores what the specialist scores alone |

**The distribution is bimodal with nothing in between.** A precondition that is always true does
not add a refinement to the base — it takes first refusal every tick, returns RUNNING, and the
base becomes dead code. A precondition that is never true does nothing at all.

**No node modulates.** The whole `Fallback[specialist, base]` architecture presupposes
specialists that fire *sometimes* — on the situations their mechanism addresses — and not one
of eleven generated nodes has such a precondition.

### ⚠️ This is a GENERATOR-level result, not an arm effect

A and B produce the identical pattern: 2/4 of A's nodes inert and 2/4 replacing, 3/7 of B's
inert and 4/7 replacing. **B−A is zero on every column, including `fixed`.** Whatever the
literature bought, it did not buy a precondition that discriminates.

The likely cause is upstream, in the card: `trigger` becomes the precondition, and a trigger
written as "applies when navigating to a goal" compiles to `True`. The card prompts ask for a
trigger but nothing tests that the trigger *selects*. `precondition_fired` was already known to
be 1.0 or 0.0 in the micro-worlds; this shows it is the same in a shared world, so it is a
property of the nodes and not of the worlds they were tested in.

### What this does NOT show

That the mechanisms are worthless. A node that replaces the base is being asked to navigate
alone, which is not what it was written for. The finding is that **the current nodes cannot be
composed with a base**, which is a fact about their preconditions and about the generation
prompt — not about dynamic-window velocity selection or path integration.

---

## ⚠️ 2026-09-14 — the execution model was the problem, and the library was regenerated

**Chunk 52 acted on the finding above.** The section immediately preceding this one measured
that no generated node modulates — preconditions are binary, six of eleven nodes fired on
87.6-100% of ticks and silenced the base entirely, `fixed` was 0 in every row and B−A was
exactly zero. This records what was done about it and what it cost.

### What changed

The node call now asks for a **modifier**, not a driver.

| | v2 (`prompts/node.md`) | v3 (`prompts/modifier.md`) |
|---|---|---|
| tree shape | `Fallback[specialist, BASE]` | `Layer[ports, BASE, node]` |
| who ticks | the first child that does not fail | **everybody, every step** |
| what the node writes | the whole Twist, via `drive()` | only the fields it declared in `WRITES`, via `cap()` / `adjust()` |
| declining | the base drives instead | the base's command passes through **byte-identical** |
| micro-world | the model writes one | **not asked for** — contribution is measured on the shared growth suite |

⚠️ **The CARD call did not change and every committed card is reused byte for byte.** That is
deliberate and it is the only way the comparison stays readable: the independent variable must
not move at the same time as the execution model. The v3 records differ from the v2 records in
the second call and nothing else.

### What this cost, and what it did not

- `PIPELINE_VERSION` is `v3-layered`. `pass_rate` refuses to pool v1, v2 and v3 — three
  pipelines, three treatments of a card, three denominators.
- The eleven v2 candidates and their fifteen node transcripts moved to
  `generate/archive/v2-exclusive-control/`, intact, with their verdicts. **Their pass-rate
  denominators are unchanged.** They had to move rather than stay: `plan_hash` is a pure
  function of (condition, task, card) and the cards are unchanged, so a v3 candidate lands on
  the identical stem. Two experiments cannot share a row key.
- The **base is NOT regenerated**. It is a full driver, it was already valid under the new
  contract, and it is frozen — every growth number is relative to it.

### The first regenerated node, and the honest thing it says

`a / limit_speed_dwa` came back as an `Action` with `WRITES = ("vx", "wz")` — narrowed, but
using `adjust` rather than `cap`, and firing whenever there is anything within 2.5 m ahead. The
model said so in prose rather than hiding it:

> This card is, honestly, a whole navigator wearing a modifier's coat: DWA *selects* a command
> rather than revising one.

That is a **finding about the card, not a failure of the prompt**, and it is the kind of thing
the prompt now explicitly invites. Not every mechanism in this corpus is a modifier. A dynamic
window is a local planner; a speed limit is a cap. The pipeline can now represent the second
kind, and it makes the first kind *visible* — `WRITES` and the `changed` column say which one a
node turned out to be, where under v2 every node looked the same.

### What this does NOT show

Nothing about whether the corpus helps. The structural change is not a result; the number that
decides whether it was worth making is **`fixed` in `scripts/measure_growth.py`**, which was 0
in every row of round 1. Until it moves, the layered pipeline is a better-specified way of
measuring the same nothing.

### ⚠️ The D runaway was the ASK, not the arm — and D has now produced a node

⚠️ **D WAS FROZEN ON 2026-09-15 and no further D work is to be done** until A and B work on at least one benchmark case — see CLAUDE.md, "The conditions". Everything below stands as a record; none of it is an invitation to generate more D rows. D's single node is **n = 1 against A's and B's 15** and cannot be read as an arm effect.

Condition D had never produced an admissible node: 4 attempts under v1, all rejected, and its
single v2 node call **failed outright after 35 minutes, ~158k thinking tokens and $4.69**,
returning no code at all. That call is why `backends.CLI_BUDGET_USD` exists.

Under `prompts/modifier.md`, the same card, the same model, the same arm:

| | v2 (`node.md`) | v3 (`modifier.md`) |
|---|---|---|
| cost | **$4.69** | **$0.334** |
| output tokens | ~158,000 | 8,037 |
| thinking tokens | ~157,900 | 3,517 |
| result | no fenced block, exited non-zero | **ADMITTED** |

The plausible reading, stated as a reading and not a proof: the old prompt asked an arm with no
source at all to invent a principle *and* write a whole navigator from it, with a micro-world to
test it in. That is an open-ended design problem, and the model spent 158k tokens on it. The new
prompt asks for one adjustment to one field of a command an existing navigator already produces,
and does not ask for a world at all. **A narrower ask is cheaper and it completes.**

⚠️ **n = 1 on the failure and n = 1 on the success.** This is not a measured effect of prompt
scope on cost. It is one very expensive failure and one cheap success on the same row, worth
recording because the failure is what drained a session's budget and the explanation on offer at
the time — "arm D runs away" — now looks wrong.

**Whole regeneration: 16 node calls, $7.17, mean $0.45, dearest $0.66.** For comparison, the
dearest successful v2 node call was $1.36. A further five calls returned
`terminal_reason: api_error` with 0 tokens and $0 at 5-way concurrency; all five succeeded on
retry at 2-way, so that is a concurrency artefact and not a prompt problem.

⚠️ **Five `.error.json` files therefore sit in `generate/staging/` beside a SUCCESSFUL
`.result.json` for the same row.** They are kept, not tidied away — a call that happened is a
call that happened — but read them with `total_cost_usd` and `pipeline_version` in hand: these
five are `$0`, `0` tokens, `v3-layered`. The one that matters, D's `$4.69` / 159,389-token
runaway, is a **v2 record and lives in `archive/v2-exclusive-control/staging/`** under the same
filename. Same stem, same extension, three orders of magnitude apart; the version stamp is what
tells them apart.

---

## ⚠️ 2026-09-14 — growth round 2: `fixed` moved, once, and it was arm D

The layered pipeline's whole justification was that `fixed` was 0 in every row of round 1. Round
2, same base, same generator, same 20 `maze` worlds (seeds 2000-2019), base alone **10/20 = 50%**,
now `Layer[ports, BASE, node]`:

| | nodes | improved | regressed | net-zero | **`fixed`** |
|---|---|---|---|---|---|
| **A** | 5 | 0 | 3 | 2 | **0** |
| **B** | 10 | 0 | 4 | 6 | **0** |
| **D** | 1 | **1** | 0 | 0 | **2** |

`gen_d_88c49cee31e01477` — `SteerThroughWidestAdmissibleGap` — scored **12/20 = 60%** against
the base's 10/20: **2 fixed, 0 broken.**

### It replicated on fresh worlds, and got bigger

Seeds 3000-3019, twenty worlds neither the node nor anyone else had been run on:

| | base alone | + the D node |
|---|---|---|
| seeds 2000-2019 | 10/20 | **12/20** — fixed 2, broke 0 |
| seeds 3000-3019 | 8/20 | **12/20** — fixed 4, broke 0 |
| **pooled** | **18/40 (45%)** | **24/40 (60%)** — **fixed 6, broke 0** |

**Six worlds fixed and none broken, across forty paired worlds.** Exact McNemar on 6/0 gives
p ≈ 0.03 two-sided. ⚠️ **That is not a G2 number and may never be reported as one**: it is
kinematic, not physics; it is the growth tier, not the sealed split; X = 0.30 was derived on
lab42 and does not transfer; and +15 points pooled is well under it anyway.

### What the `fire` / `changed` columns say, and it is the real finding

Round 1 could only report a fire rate, which conflated "never ran" with "ran and did nothing".
The two columns separate them, and the library falls into **four** groups:

| pattern | nodes | outcome |
|---|---|---|
| never fires (`fire` 0-2%) | 4 | +0%, by definition |
| fires ~100%, changes **nothing** | 3 | +0% — a cap whose bound is never the binding one |
| fires ~95-100%, changes ~100% | 6 | **every one regresses**, -10% to -50% |
| **fires 62%, changes 57%** | **1** | **+10%, and it is the only node that helps** |

**Selectivity is the variable.** A node that acts on every tick has replaced the base whatever
its `WRITES` says; a node that never acts cannot help. The one node in the library with a
precondition that is genuinely sometimes-true *and* an effect that genuinely bites is the one
that improves the tree. The layered contract makes that visible and makes it safe — `broke = 0`
is only meaningful because a declining modifier provably leaves the command byte-identical — but
**it does not make the generator produce selective nodes.**

### ⚠️ B−A is STILL exactly zero, and the one hit is in the arm with no corpus

Neither corpus arm produced a node that fixed a single world. The only improvement came from the
arm that was handed **nothing at all** and invented its principle. Read that carefully:

- It is **n = 1 node** against **n = 15**, and D's ceiling is 4 rows against A's and B's 20. D has
  had one node measured, ever. This is not evidence that inventing beats reading.
- The likelier reading is about the *task*, not the arm. The D card's principle — steer through
  the widest admissible gap when the goal is occluded — is **a navigation mechanism sized to a
  modifier**. The corpus cards are speed limits (caps that never bind on a base that is already
  cautious) and path integration (heading substitutions that fight the base every tick). Whether
  a mechanism helps here depends on whether it is *selective*, and nothing in the pipeline
  selects for that.
- ⚠️ And one confound is worth naming: **the arms are not at comparable row counts** (A 5, B 10,
  D 1), which chunk 42's outcome already flags. Nothing here may be read as an arm comparison.

### What this does and does not license

**Does:** the layered pipeline is no longer unfalsified. A node can now improve the incumbent
tree without replacing it, and one has. `scripts/prune_obsolete.py`'s gate passes.

**Does not:** it says nothing about the corpus. Fourteen of sixteen nodes still contribute
nothing or actively harm, `fixed` is 0 in both corpus arms, and the mechanism that worked is the
one whose shape happened to fit. The next question is whether **selectivity can be prompted for**
— not whether the corpus helps.

---

## ⚠️ 2026-09-15 — the nodes were being judged on a claim they never made

Growth round 2 scored **`goal_reached` and nothing else**, and then read six "+0%" rows as
"this node does nothing". That was wrong for six of the sixteen nodes:

| predicted metric | nodes | families |
|---|---|---|
| `collisions` | **6** | the entire `limit_speed_dwa` / `limit_speed_ics` set |
| `goal_reached` | 10 | `path_integration_*`, `reorient_geometry`, D |

A speed cap that prevents collisions at no cost to arrivals scores exactly +0% on goal-reach
and looks inert. `scripts/measure_growth.py --metrics` now scores the full set through the real
`core.metrics.score()` — the SPL denominator is a ~22 s visibility-graph solve per world, paid
once up front and then cached for all seventeen controllers, which is the resolution epic 05
already recorded.

### Did each node move the metric its own card predicts?

Same 20 maze worlds, base 10/20:

| | nodes |
|---|---|
| **improved its own claimed metric** | **3** |
| made it worse | 5 |
| no change at all | 8 |

### ⚠️ And two of those three "wins" are a robot that stopped

`collisions` is lower-is-better, so **a robot that does not move scores perfectly.**

| node | arrivals | path (m) | collisions | **collisions / metre** |
|---|---|---|---|---|
| base alone | 0.50 | 30.0 | 0.40 | **0.0133** |
| `gen_b_c2c95bcb965e342e` | **0.00** | **6.2** | 0.30 | **0.0481** — 3.6x WORSE |
| `gen_b_c380e7a8cfb32b78` | **0.15** | 27.8 | 0.00 | 0.0000, with near-misses **doubled** (4.2 → 8.4) |

`c2c95bcb`'s win **reverses entirely** once normalised: it walks 6.2 m instead of 30.0, reaches
the goal in **0 of 20 worlds**, and collides more often per metre travelled than the base does.
`c380e7a8` keeps a genuine zero but loses 70% of arrivals and doubles near-misses — it is
creeping, not avoiding.

**So the honest count is: zero of sixteen nodes improved their own claimed metric while still
doing the task.** The only real improvement anywhere in the library remains the D node on
`goal_reached`, and it *costs* collisions (0.40 → 1.70) and clearance (0.126 → 0.037) to buy it.

### The methodological finding, which outlives this round

⚠️ **A lower-is-better COUNT metric is not comparable between controllers with different
arrival rates.** `collisions`, `near_misses` and `stuck_events` all reward stopping. This is not
hypothetical — it produced an apparent second win in the first table that measured it, and the
sign flipped under normalisation.

`measure_growth.py` now reports every count metric **per metre travelled** beside the raw count,
and flags when the two disagree (`<-- REVERSES the raw count`). Neither number replaces the
other: per-metre alone would flatter a node that crawls safely and never arrives, which is why
`goal_reached` stays primary. ⚠️ **This applies to `PRE-REGISTRATION.md` §5's named
secondaries too** — collisions and near-misses are on that list, and the same normalisation
question arises there. Raised here; not settled, because settling it is an amendment.
