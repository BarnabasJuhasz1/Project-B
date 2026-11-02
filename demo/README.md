# `demo/` — the two-arm patrol demo (V2)

**2026-09-29.** A deliberately small, self-contained experiment: does handing a
model **one research paper** produce a materially better Go2 controller than the
same model gets without it, on a task where the environment changes underneath
the robot?

⚠️ **THIS IS NOT THE EPIC-07 PIPELINE AND SHARES NO NUMBERS WITH IT.** No cards,
no behaviour trees, no node contract, no retrieval, no composition, no
`generate/staging` row, no pass-rate denominator. It reuses exactly three things
from the repo and nothing else:

| reused | why |
|---|---|
| `core.geometry` (raycast, clearance, ROBOT_RADIUS) | the sensor model and the collision test must be the benchmark's, not a second implementation |
| `core.observation` (Observation, Command, clamp) | the frozen interface the whole project is written against |
| `generate.backends.ClaudeCliBackend` | it runs each call in a scratch dir with `--setting-sources ''`, so **this project's CLAUDE.md never reaches the model** |

## The two arms

|  | receives |
|---|---|
| **arm0** | a detailed description of the task, and nothing else |
| **armB** | the **identical** description, plus Stentz (1994) *"Optimal and Efficient Path Planning for Partially-Known Environments"* (D\*) in full |

⚠️ **Neither arm is told that the route gets blocked.** That is the point: the
question is whether the paper leads the model to build something that copes with
a change it was not warned about.

⚠️ **The arms differ ONLY inside a delimited paper block.** `demo/render.py`
strips the block from both renderings and asserts they are byte-identical;
`demo/generate.py` refuses to make a call if that check fails. This is the only
piece of the old pipeline's discipline that was worth keeping, and it is what
the whole comparison rests on.

## The task

The robot shuttles between **home** and **the goal**, four one-way trips.
`obs.goal` holds whichever point is currently being driven to and flips on
arrival. `reset()` is called **once**, before leg 0 — so anything a controller
learned on legs 0–1 is still on the instance when it meets the shut door.

    leg 0   home -> goal    door OPEN
    leg 1   goal -> home    door OPEN
    leg 2   home -> goal    door SHUT     <- the discovery
    leg 3   goal -> home    door SHUT

The door shuts while the robot is at home, between legs, so it can never close
on top of the robot.

## The world

⚠️ **HAND-AUTHORED, and that is a declared fact about it.** It was built so the
demo's premise is guaranteed to hold, and **no number from it may be pooled with
anything from `core.worldgen`'s seeded distribution.** `demo/world.py`'s
`verify()` checks all four guarantees against `core.geometry` — the benchmark's
own code — rather than asserting them:

| check | value |
|---|---|
| short route, door open | **15.03 m** |
| long route, door shut | **20.60 m** |
| detour cost | **5.57 m** |
| both routes fit the robot | yes (inflated by ROBOT_RADIUS) |

The shut door sits almost exactly on the straight line home → goal, so a
controller steering on goal bearing is pulled back to it repeatedly. That is the
base's documented failure mode (`out/evidence/base-failure-mode/`: it re-enters
an unexitable detour 4–7 times per episode), and making it visible is the
demo's job. The clutter and staggered partial walls carry none of the topology —
they are there so the recording reads as a room rather than a diagram.

## ⚠️ The premise was confirmed BEFORE any model call was made

The project's own frozen base (`generate/base/base_controller.py`, a model
written "asked plainly" artefact) on this patrol, 180 s budget:

    legs completed 2/4 — leg 0 in 27.0 s, leg 1 in 28.7 s, then leg 2 NEVER

It runs the out-and-back cleanly twice, meets the shut door, turns back, wanders
the whole building for the remaining ~125 s and never finds the long way round —
which exists and is only 5.6 m further. Picture: `demo/out/smoke-base.png`.

So the task discriminates, and it discriminates on the intended thing. Had it
not, the six generation calls would have been wasted.

## Calibration decisions, and what they cost

- **Budget 180 s.** ~3x the taut distance of all four legs even on the long
  route. Generous enough that a failure reads as "never found the way" rather
  than "ran out of clock".
- **Openings widened, two clutter blocks removed.** The first world was hard
  enough that `reactive_avoid` lost leg **1** — before the door ever shut. Legs
  0–1 have to be routine or the door is not the discriminator.
- **k = 3 per arm.** Epic 07 established that the same prompt produces
  opposite-sign effects across draws; k = 1 is not interpretable.

## Reproducing

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P demo/world.py --plot            # the world + its four checks
    $P demo/render.py                  # the arms are identical outside the block
    $P demo/generate.py --dry-run      # prompt sizes, calls nothing
    $P demo/generate.py --k 3          # 6 model calls
    $P demo/run.py --plot              # evaluate everything, contact sheets
    $P demo/test_demo.py               # the invariants

## What a null result here means

If armB does not beat arm0, that is **not** "literature does not help". It is
one paper, one task, one hand-built world, k = 3, kinematic. The honest claim
available from this demo is about *this* paper on *this* failure — and the
anchoring caveat from epic 07 still applies in the other direction: arm0 is not
handed working code here, so this is a cleaner test of the paper than the edit
arms were.

---

## RESULTS (2026-09-29, k = 3, one world, kinematic)

| arm | legs per sample | mean | solved the block |
|---|---|---|---|
| **arm0** — no paper | 4, 2, 4 | **3.33** | **2/3** |
| **armB** — Stentz 1994 | 2, 4, 2 | **2.67** | **1/3** |
| (base) reference | 2 | 2.00 | 0/1 |

⚠️ **THE RESULT IS AGAINST THE HYPOTHESIS, AND AT k = 3 IT IS ALSO NOISE.**
2/3 against 1/3 is a difference of one sample. Nothing here supports "the paper
helped" and nothing here supports "the paper hurt".

### ⚠️ WHY, AND THIS IS THE ACTUAL FINDING

**All three arm0 controllers independently built a persistent occupancy grid and
a global planner, with no paper and no prompting to do so** — a log-odds grid
plus, respectively, min-plus Bellman-Ford, a Dijkstra-style value function, and
a weighted wavefront. arm0 k2 additionally wrote a correlative scan matcher for
pose correction. All three explicitly noted that the map persists across trips.

So **there was no capability gap for the corpus to fill.** The task is
in-distribution: asked to shuttle repeatedly through an unmapped building, the
model already knows to map it and plan over the map.

**And D\*'s actual contribution does not bind here.** D\* is an *efficiency*
result — repair the plan locally instead of replanning globally, so that cost
stays constant as the environment grows (its Sec. 4, Table 1). In a 14 x 10 m
room at 50 Hz with numpy, a full wavefront replan every step is cheap. The paper
optimises a constraint this platform does not have, while its complexity is paid
in full.

⚠️ **armB did engage with the paper properly** — this is not a failure to read
it. All three cite PROCESS-STATE L1-L26, MODIFY-COST, the RAISE/LOWER split of
Sec. 2.3, the `k_min >= h(robot)` stopping rule and footnote 1's obstacle-cost
argument, and the transcriptions are faithful.

### What the pictures show

- `demo/out/armB-k1-seed0.png` — **the mechanism working.** Leg 2: into the
  room, door shut, back out, round the left and over the top, goal in 67.2 s.
  Leg 3: straight out on the long route in 40.8 s, **without re-trying the
  door**. That is the memory doing its job.
- `demo/out/armB-k0-seed0.png` — **the mechanism failing.** Leg 2 oscillates
  between the room and the lower-left, re-entering repeatedly, never taking the
  left-hand route. The blockage never propagated far enough to change the plan.

### Observation, not a result (n = 1 each)

The successful armB sample took **2 collisions**; the successful arm0 samples
took **27** and **11**. Suggestive, but this is one sample per arm and picking a
secondary metric after seeing the primary is exactly the post-hoc filtering the
pre-registration discipline exists to stop.

### ⚠️ The demo the brief asked for does not exist in this data

An "arm0 fails / armB succeeds" pair can only be produced by choosing arm0 k1
and armB k1 — 1 of 3 from each arm, selected after seeing the answer. That is
cherry-picking and it must not be presented as the arm comparison.

What IS honest and does show a large contrast: **base (2/4) vs armB k1 (4/4)** —
but that compares a controller written for a single trip against one written for
the patrol, so it demonstrates task-awareness, not the corpus.

### ⚠️ THE BUDGET IS NOT WHAT DECIDES IT (checked, 2026-09-29)

The 180 s budget is for the **whole episode**, not per leg: the four legs run in
sequence and share one clock, so a controller that spends the budget failing
leg 2 never starts leg 3. That raised a fair objection -- a budget tight enough
to punish the longer-but-correct route would be measuring speed, not capability.
armB k1 succeeds using 149 s of 180, which is only 31 s of margin.

Re-run at 180 / 240 / 300 s (`$CLAUDE_JOB_DIR/tmp/budget_sweep.py`):

| controller | 180s | 240s | 300s |
|---|---|---|---|
| arm0 k0 | 4/4 (109s) | 4/4 (114s) | 4/4 (109s) |
| arm0 k1 | 2/4 (180s) | 2/4 (240s) | 2/4 (300s) |
| arm0 k2 | 4/4 (152s) | 4/4 (152s) | 4/4 (152s) |
| armB k0 | 2/4 (180s) | 2/4 (240s) | 2/4 (300s) |
| armB k1 | 4/4 (150s) | 4/4 (150s) | 4/4 (148s) |
| armB k2 | 2/4 (180s) | 2/4 (240s) | 2/4 (300s) |
| (base) | 2/4 (180s) | 2/4 (240s) | 2/4 (300s) |

**Every verdict is identical at all three budgets.** The failures consume the
whole budget at every level -- they oscillate indefinitely rather than run out
of clock -- and the successes finish in 109-152 s with margin. So 180 s is a
fair budget and no result here is an artefact of it. **Checked and ruled out.**

### ⚠️ WHAT IS AND IS NOT A CLEAN COMPARISON

| pairing | task description | clean? |
|---|---|---|
| **arm0 vs armB** | identical; `render.check()` asserts the renders are byte-identical outside the paper block, and the transcripts confirm one prompt hash per arm (`c6a5b5e6…` / `4045efb3…`) across all k | **YES — differs only by the corpus** |
| base vs armB | the base was generated in epic 07 against `navigate_to_goal`: one trip, no shuttling, no revisiting. It has never seen `demo/task.md` | **NO — confounds task-awareness with the corpus** |

⚠️ Within an arm the k's are **three draws from one identical prompt** — same
hash, same character count. They carry no information difference at all.

So the problem with staging "no-paper fails / paper succeeds" is **not** a
confound, it is **selection**: arm0 is 4,2,4 and armB is 2,4,2, so that pairing
exists only by choosing 1 of 9 combinations after seeing the answer, against an
arm-level mean that runs the other way.

---

## ⚠️ CORRECTED RESULT (2026-09-30) — THE FIRST WORLD HAD A DEFECT

**Everything above this line that reports arm0 beating armB is WRONG, and the
cause was the world, not either arm.**

### How it was found

`diagnose_arms.py` asked, for each controller: on leg 2, did it ever enter the
bypass corridor, and how far up did it get? Every failing controller **in both
arms** had already turned into the bypass (leg-2 min x of 0.6-1.5 m) and then
stalled at y = 5.5-7.3. **Not one of them failed to reroute.**

Measuring the corridor explained it. The two zigzag walls -- added to lengthen
the detour -- left 0.90 m gaps, which after inflating by the 0.30 m robot radius
gave the robot CENTRE a **0.30 m slot with 0.15 m of margin either side**, at
y = 5.3 and y = 7.3. `verify()` passed it because `geodesic_distance` reports any
positive-width centre path as solvable. **Solvable is not navigable.**

So the demo was measuring slot-threading, not rerouting.

### The fix

Both gaps widened to 1.40 m: the tightest point on the bypass now gives the
centre **0.769 m** of free width. The detour is still **4.33 m** on a 15.03 m
route. `test_bypass_is_actually_navigable_not_just_solvable` and
`world.bypass_bottleneck()` exist so this cannot recur.

### The result on the fixed world (n = 8 per arm, 3 reps each, quiet machine)

| | k0 | k1 | k2 | k3 | k4 | k5 | k6 | k7 | solved | mean legs |
|---|---|---|---|---|---|---|---|---|---|---|
| **arm0** | 4 | 4 | 3 | 4 | 4 | 2 | 4 | 3 | **7/8** | **3.50** |
| **armB** | 2 | 4 | 4 | 4 | 4 | 2* | 4 | 4 | **6/8** | **3.50** |
| (base) | 2 | | | | | | | | 0/1 | 2.00 |

\* armB k5 is the only controller unstable across reps: `[2, 1, 2]`.

**Fisher exact two-sided p = 1.000. The arms are indistinguishable.**

| | before (broken world) | after (fixed world) |
|---|---|---|
| arm0 | 5/8, mean 3.25 | **7/8, mean 3.50** |
| armB | 3/8, mean 2.75 | **6/8, mean 3.50** |
| p | 0.619 | **1.000** |

⚠️ **The defect hurt armB MORE** (+3 on the fix against arm0's +2), so the
apparent "paper makes it worse" was manufactured entirely by the bad geometry.
A cost-halo explanation was checked and is NOT supported -- both arms inflate
obstacles at similar rates.

### What stands, and what does not

**Stands:** all eight arm0 controllers built a persistent occupancy grid and a
global planner with no paper and no prompting, and seven of eight reroute
correctly. There was no capability gap for the corpus to fill, and D\*'s
contribution -- constant replanning cost as the environment grows -- does not
bind in a 14 x 10 m room.

**Does not stand:** any claim that the paper helped or hurt. p = 1.000 at n = 8.

**Still true:** the demo the brief asked for does not exist in this data. Both
arms sit at mean 3.50 against the base's 2.00, and that gap is task-awareness
(the base was generated for a single trip) rather than the corpus.

### ⚠️ Repeatability, and why it is reported

16 of 17 controllers score identically across three reps on a quiet machine. But
8 of 16 generated controllers bound their planning with a WALL-CLOCK budget
(`time.perf_counter()`), and an evaluation taken while a generation call was
running scored armB k1 at 3/4 where every clean rep gives 4/4. **Arm numbers
come from a quiet machine**, and `demo/repeatability.py` is how that is checked.

---

## ⚠️ UNDER PHYSICS THE ANSWER IS DIFFERENT (2026-10-03)

**Every kinematic number above is a statement about an idealised unicycle.** Run
with the frozen locomotion policy in the loop, `demo/physics_sweep.py`, same 16
controllers, same world, 180 s budget:

| | k0 | k1 | k2 | k3 | k4 | k5 | k6 | k7 | solved | mean legs |
|---|---|---|---|---|---|---|---|---|---|---|
| **arm0** | 4 | 4 | 4 | 4 | 4 | 4 | 4 | 2 | **7/8** | **3.75** |
| **armB** | 2 | 2 | 2 | 4 | 4 | 2 | 1 | 2 | **2/8** | **2.38** |
| (base) | 3 | | | | | | | | 1/1 | 3.00 |

**Fisher exact two-sided p = 0.041.**

| | kinematic | physics |
|---|---|---|
| arm0 | 7/8, mean 3.50 | **7/8, mean 3.75** |
| armB | 6/8, mean 3.50 | **2/8, mean 2.38** |
| p | 1.000 | **0.041** |

⚠️ **THE KINEMATIC PROXY WAS NOT MERELY NOISY, IT WAS QUALITATIVELY WRONG.** Six
of sixteen controllers flipped verdict, and the flips are not symmetric: arm0
lost one and gained one, **armB lost four, all from success to failure**
(k1 4→2, k2 4→2, k6 4→1, k7 4→2). A proxy that reverses a p = 1.000 into a
p = 0.041 is not a fast loop for this question, and no kinematic number in this
directory should be read as a claim about the robot.

⚠️ This is the first non-null result the demo has produced and it runs **against
the hypothesis**: the arm handed the paper is substantially worse once the gait
is real. armB k6 (1/4) is worse than the frozen base (3/4).

### Mechanism: partly identified, not settled

The three armB controllers that implement D\* **purely incrementally**, with no
full-replan path at all (k0, k1, k2 — `full-replan` keyword count 0), fail
**3 of 3**. That is consistent with statefulness being the hazard: a repaired
search tree that is subtly stale keeps driving a plan the world no longer
supports, where a controller that recomputes the whole cost field each step
cannot hold a stale belief.

⚠️ **But it does not explain everything and must not be reported as settled.**
armB k5 and k6 carry substantial full-replan machinery and fail anyway, while
arm0's eight all recompute globally and seven succeed. A second candidate is
tracking error: arm0's controllers score candidate twists by local rollout
(a dynamic-window style), which tolerates the gait not going exactly where
commanded, whereas following D\* backpointers does not. **Neither has been
tested.** Testing it is a controlled ablation, not a grep.

### What the video shows, revised

`demo/out/side-by-side.mp4` pairs arm0 k5 (4/4) against armB k0 (2/4). When it
was recorded that looked like cherry-picking against a null. Under physics it is
**the modal outcome of each arm** — 7/8 of arm0 solve, 6/8 of armB do not — so
the panes may legitimately be labelled by arm. They are currently labelled by
behaviour; `--labels` changes it.

### Standing caveat

n = 8 controllers per arm on **one hand-authored world**. The sample generalises
over generation variance, not over environments: this is one draw from one
geometry, in no split, in no PRE-REGISTRATION constant. p = 0.041 is a statement
about this world.
