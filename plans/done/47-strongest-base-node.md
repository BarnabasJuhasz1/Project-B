# 47 — The strongest base node, and difficulty calibrated against it

**Epic:** [05](../05-epic-environment-suite.md) · **Depends on:** 45 · **Needs Isaac Sim:** later
**Status: PLANNED 2026-09-13, user's design.** SIM-FREE up to the physics confirmation.

⚠️ The epic's provisional chunk table listed 47 as "geometry-induced localisation
difficulty". That becomes **51**. This chunk takes 47 because everything in chunk 44's
growth loop waits on it.

---

## The principle, in the user's words

> I want the worlds to be too hard for the base node, because then we can actually see the
> other nodes helping and improving the success rate. I want the base node to be **as strong
> as possible**, but still fail because the environment is very difficult. **If it succeeds
> 100% of the time, that does not mean the base is too strong — it means the environment is
> too easy.**

**This inverts which knob is the free variable.** The base is maximised; the *environment* is
tuned until the base fails. Difficulty stops being a property of geometry statistics and
becomes a property measured against a fixed, maximally strong reference:

> **Difficulty is the base node's failure rate. A tier is calibrated, not designed.**

Chunk 45 measured difficulty as tortuosity, free fraction, bottleneck radius and obstacle
count. Those stay — they are cheap, sim-free and diagnostic — but **they are now proxies.**
The definition is the base's failure rate, and where the two disagree the proxies are wrong.

## Why a strong base is the RIGHT choice, not a compromise

The corpus supplies **corrections to a navigator, not navigators**.
`LimitSpeedToVisibleRange`, `MaintainHomingVector` and `ReorientToGeometry` all presuppose
something that already drives. Bolt them onto a robot that cannot avoid a wall and both arms
will independently produce obstacle avoidance first, the arms will converge, and B−A will
measure basic competence rather than what the literature bought.

## ⚠️ The base IS the null hypothesis, and that is worth exploiting

CLAUDE.md's claim under test is that literature-derived principles beat *"what a frontier
model produces alone"*. **A maximally strong, cold, LLM-written navigator is exactly that
thing.** So:

- The base tree scored on its own is a meaningful reference number, and it is very close to
  the **retired naive arm** that the write-up otherwise has to say does not exist.
- It costs nothing extra: the growth loop's **round-0 incumbent measurement IS that number**.
  Record it as such rather than discarding it.

## ⚠️ The threat this introduces, declared before building

Tuning worlds until the base fails is **adversarial selection of the environment against the
base**. Two consequences, and one hard rule:

- Absolute success rates on this suite will be low by construction and **may not be reported
  as "how well the system navigates"** — only as differences between arms on a common suite.
- **The environment may be tuned against the SHARED BASE ONLY, never against any arm's
  nodes.** Tuning against A's or B's nodes would shape the benchmark to the thing being
  measured. This is the same guard the 2026-09-13 pre-registration amendment already declares
  for procedural generation, extended to the calibration loop.

## What "strongest an LLM can create" means operationally

One node, model-written, **shared byte-for-byte by every arm**, under chunk 44's #3 exemption
as a declared *substrate*.

- It is **not** condition D's output and must never be. D writes one leaf from an invented
  principle; this is a separate, neutral call whose only instruction is to navigate as well as
  possible. Reusing D's node would make the substrate one arm's work.
- Prompted with the node contract and the task, and **no card, no citation, no corpus** — the
  same blindness `node.md` already has, minus the card.
- Generated once, committed, and frozen. A regenerated base is a different experiment: every
  growth-loop number is relative to it.
- Precondition must be **unconditionally true**. It is the last resort in
  `Fallback[specialist..., base]`; a base that declines leaves the tree with nothing driving
  and the robot receives `STOP` (chunk 44's freeze trap).

## The calibration loop

1. Generate the base. Commit it.
2. Score it over each tier, kinematically, over many seeds.
3. Read the failure rate:
   - **~0% failure** -> the environment is too easy. Harden the tiers. **Do not weaken the base.**
   - **~100% failure** -> nothing to improve on; a growth loop cannot climb from zero. Soften.
   - **a middle band** -> usable. The target band is a decision, recorded below.
4. Re-measure. Freeze the suite only when the band is hit.

**Target band: 30-70% success for the growth tier.** Below 30% the loop has too few successes
to avoid regressing; above 70% too little headroom. ⚠️ This number is a judgement and is
recorded here so it is not quietly changed later.

## Build

1. `core/worldgen.py` -> a scenario that rolls out on a generated world.
   ⚠️ **No frozen-contract change needed**: `core.scenario.Scenario.world` is already a
   derived PROPERTY returning `LAB42`, left that way by 11a precisely so
   `harness.micro.MicroScenario` could answer differently. A subclass overriding `world` is
   enough.
2. A neutral base-node prompt + a generation path that is **not** a condition.
3. `scripts/measure_base.py` — the calibration loop, sim-free, per tier.
4. Harden or soften the tiers until the band is hit. Record every iteration.
5. Physics confirmation of the base on a sample, before anything is frozen.

## Done when

The base is committed and frozen; its failure rate per tier is measured and recorded; a growth
tier sits in the target band; and the round-0 number is written down as the naive-arm
reference.

## Not in scope

The growth loop itself (chunk 44). Moving obstacles (48/49). Any arm's nodes — this chunk
touches the substrate only.


## Progress 2026-09-13 — the harness is built and the worlds are already close

Built:

- `core.worldgen.GeneratedScenario` + `to_scenario()`. ⚠️ **No frozen-contract change was
  needed**: `Scenario.world` is a derived property (11a left it that way for
  `MicroScenario`), so a subclass overriding it is enough. A kinematic rollout on a generated
  world works.
- `core/scenario.py` made **world-aware at five call sites** — `disclosed_at`'s line-of-sight
  test, `optimal_distance`, and three checks in `validate`. All were using the LAB42 default.
  Behaviour-preserving today (every current scenario's `world` IS LAB42); required the moment
  a scenario is not lab42. Without it, a generated world's goal would be "seen" through its
  walls. 473 core tests pass.
- `scripts/measure_base.py` — the calibration loop.

### Interim reference: `reactive_avoid`, n=20 per tier, kinematic

⚠️ `reactive_avoid` is a hand-written gap follower and a **placeholder**, not the base. It is
a reference controller and never a condition; this is calibration, not scoring.

| tier | success | verdict |
|---|---|---|
| open | **70%** | in band, top edge |
| cluttered | **55%** | in band, ideal |
| maze | **20%** | too hard for a growth tier; good as a stretch tier |

**This falsifies the guess made from the contact sheet** that `open` is trivial — at 70% it is
not. The pictures showed low tortuosity and I inferred "too easy"; the measurement disagrees,
which is the whole point of defining difficulty as a failure rate.

**What it predicts.** A genuinely strong base should beat a hand-written gap follower, so every
number above moves UP. The likely landing: `open` -> too easy and needs hardening,
`cluttered` -> upper band or above, **`maze` -> into the 30-70% band and becomes the growth
tier.** So the tiers are already roughly calibrated for a strong base, and the expected
response is to harden `open` rather than to rebuild the generator.

### Next

Generate the base itself. Needs: a neutral prompt (node contract + task, no card, no citation,
no corpus), a generation path that is **not** a condition and not condition D, one model call,
committed and frozen. Then re-run the table above against it and harden whichever tier leaves
the band.

## The base is generated — 2026-09-13

`generate/base/base_navigator.py`, 538 lines, `claude-opus-5`, one neutral call, prompt hash
`128534500760ffe0`. Committed and **frozen**.

What it built, from its own design notes: corridor-clearance gap scoring (each of the 181
bearings treated as a candidate travel direction, with a swept-rectangle clearance so a 0.9 m
doorway reads as open); heading **commitment with hysteresis** against livelock; a Bug-style
**wall-following detour** when distance-to-goal stops improving; a timed **escape ladder** on
contact or no-motion; and a **novelty-driven exploration mode** on a decaying visit grid for
when the goal is not disclosed. It degrades to exploration on a bad pose and scales goal-trust
by `pose_error_estimate`. `precondition` returns `True`; `tick` is wrapped so that even an
unexpected fault still drives.

### Calibration, n=20 per tier, kinematic

| tier | `reactive_avoid` (placeholder) | **base** | verdict |
|---|---|---|---|
| open | 70% | **90%** | too easy — regression floor only |
| cluttered | 55% | **70%** | in band, top edge |
| maze | 20% | **40%** | **IN BAND — this is the growth tier** |

**The base beats the hand-written gap follower on every tier**, which is what "strongest an LLM
can create" was supposed to mean, and the prediction recorded above — that a real base moves
every number up and lifts `maze` into the band — held.

So under the user's framing: **`open` is too easy and would need hardening if it were ever a
growth tier.** It is not; it is a floor. `maze` is the growth tier and `cluttered` is usable.
**No tier needs rebuilding.**

### Decisions this settles

- The growth suite is **`maze`**. `cluttered` is the secondary. `open` is a regression floor
  that a healthy tree should never fail.
- No hardening round is needed, so chunk 45's generator stands as built.

### Still open

- The base has never run under **physics**. Everything above is kinematic, and
  `kinematic.py` omits the frozen locomotion policy entirely.
- `Origin.SUBSTRATE` was added to `nodes/contract.py` — a new enum value, not a new field.
  `INVENTED` would have made the base indistinguishable from condition D's node, and
  `HANDWRITTEN` would have been false about a model-written node in the one field that exists
  to record where code came from.

## ⚠️ CORRECTION 2026-09-13 — the first calibration was measured on non-reproducible worlds

**The table above (open 90% / cluttered 70% / maze 40%) is VOID.** So is the growth round
measured against it.

`core/worldgen.py` seeded its RNG with `abs(hash(tier))`, and **Python randomises str hashing
per process**. `generate('maze', 2000)` returned 21, 24 and 31 rects on three consecutive
runs — every process drew a different world. The same base scored 40%, 30% and 25% on three
runs of "the same" tier, which is how it was caught.

`test_same_seed_same_world` passed throughout because it compares two calls **inside one
process**. Fixed with `zlib.crc32(tier)`, verified identical across three interpreters, and
`test_same_seed_same_world_ACROSS_PROCESSES` now spawns real subprocesses.

### Calibration, re-measured on stable worlds (n=20, kinematic)

| tier | base success | verdict |
|---|---|---|
| open | **100%** | ⚠️ ENVIRONMENT TOO EASY — regression floor only |
| cluttered | **45%** | IN BAND |
| maze | **50%** | IN BAND |

Both `cluttered` and `maze` are usable growth tiers; `open` is not, and under the user's
framing that is a statement about the **environment**, not the base. `open` needs hardening if
it is ever to be more than a floor.

⚠️ **Every number in this project measured on a generated world before this fix is void**, and
that is: the first base calibration, and growth round 1. Nothing else had run on generated
worlds yet, so nothing committed is affected.

---

## Outcome — NOT RECORDED AT THE TIME, AND PARTIAL

⚠️ **This is a stub, written 2026-09-15 when the chunk was moved from `todo/` to `done/`.**
It is a reconstruction from the artefacts on disk, not a contemporaneous record. This chunk
was **executed without ever passing through `doing/`**, which is why no Outcome was written.
No Checks table exists and none can honestly be written.

⚠️ **One Done-when item is only partly met — see below. This chunk is moved as "executed",
not as "clean".**

**Moved on the following evidence, all verified 2026-09-15:**

| Done-when | State |
|---|---|
| The base is committed and frozen | ✅ `generate/base.py`, `generate/base/base_navigator.py` (21 KB) and `generate/base/base_navigator.json`, which records `role: substrate`, `is_condition: false`, `prompt_hash: 128534500760ffe0`, and the note *"Frozen once generated: every growth-loop number is relative to it."* |
| Its failure rate **per tier** is measured and recorded | ⚠️ **`maze` only.** `out/growth/maze.csv` records `BASE_ALONE` at 0.5000 over seeds 2000-2019 and `out/growth/maze-replication-seed3000.csv` at 0.4000 over seeds 3000-3019. **`open` and `cluttered` were never run against the base** — `out/growth/` holds no such file. And `out/` is gitignored, so even the `maze` number is not committed anywhere. |
| A growth tier sits in the target band (30-70%) | ✅ `maze` at 50% and 40%, both inside it. Established for one tier out of three. |
| The round-0 number is written down as the naive-arm reference | ⚠️ It exists as the `BASE_ALONE` row of the growth CSVs, which is what the plan said it would be (*"the growth loop's round-0 incumbent measurement IS that number"*). It is **not** written down anywhere durable or committed, and it is not named as the naive-arm reference in `PRE-REGISTRATION.md`. |

`scripts/measure_base.py` exists and is the committed tool that produces these numbers, so the
gap is unrun measurements rather than missing machinery.

**What this hands forward:** if the base's failure rate is going to serve as the difficulty
definition the plan makes it — *"Difficulty is the base node's failure rate. A tier is
calibrated, not designed."* — then `open` and `cluttered` still need measuring, and the
resulting table needs a committed home rather than a gitignored CSV. That is a small,
sim-free piece of work and it is not tracked by any open chunk.

**Not verified here:** anything about whether the base is in fact the *strongest* base
obtainable, which is the chunk's title and its central claim. No alternative base was
generated or compared. The claim rests on the prompt and the single generation.
