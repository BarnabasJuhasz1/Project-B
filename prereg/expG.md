---
created: 2026-11-02
experiment: Exp G, pilot-size armX
status: FROZEN at the commit that adds this file, before any armX generation call
approved: Barna, 2026-11-02 ("spin up as many agents as you want to start up all the experiments we can start"); budget ~$11, hard cap $16
---

# Exp G — pilot-size armX (length-matched off-topic document)

Written and committed **before the first armX generation call**. Anything below
that changes after this commit is a dated amendment at the end, never an edit.

## Question

The pilot (`demo/`, 2026-09-29 / 2026-10-03, not pre-registered) found under
Isaac Sim physics: **arm0 (no paper) 7/8** controllers complete all four trips,
**armB (Stentz 1994, D\*) 2/8** (Fisher two-sided p = 0.041).

**Does any long document in the prompt cause the physics drop, or only this
paper's content?** armX gets the same prompt block as armB, holding a
length-matched, off-topic paper: Feng et al. 2015, acoustic gas thermometry,
cut at 979 lines (`demo/controls/MANIFEST.md`). Prompt hash
`a848cebd7806d2cc`; it measured 26,872 first-turn input tokens against armB's
26,464 (+1.5 %).

- **Length / distraction (L):** armX behaves like armB, so a long document is enough.
- **Content (C):** armX behaves like arm0, so the drop needs D\*'s content.

⚠️ armX controls for length and distraction, not for relevance. A wrong but
relevant-looking navigation paper is a different control and is not run here
(plan R8).

## Design

| item | value |
|---|---|
| arm | `armX`, `demo/render.py`, prompt `a848cebd7806d2cc` (byte-identical to arm0/armB outside the paper block, `render.check()`) |
| samples | **k = 8**, `demo/arms/armX/k0.py … k7.py`, one independent call each, never redrawn (`generate.py` skips existing k) |
| generation path | `demo/generate.py` → `generate.backends.ClaudeCliBackend`: `claude -p --model claude-opus-5 --setting-sources '' --system-prompt <fixed> --output-format json --max-budget-usd 2.5 --strict-mcp-config`, scratch cwd, timeout 1800 s |
| pilot comparators | arm0 k0–k7, armB k0–k7, **their existing physics verdicts**. Not regenerated, not re-run here |
| world, budget | the demo's one world (door shuts before leg 2), 180 s for all four trips |
| kinematic | `demo/rollout.py`, seed 0, original start, every armX controller. Secondary, never the primary |
| physics | `demo/physics_sweep.py --arms armX --no-base --reps 3` (Exp 3 conditions, `demo/exp3.py`): rep 0 = the pilot's exact condition (seed 0, no offset); reps 1–2 = Exp 3's fixed seeds and start offsets, identical for every controller. One Isaac Sim process at a time (`flock /tmp/mindfire-isaac.lock`), platform profile `performance` required before every episode |
| per episode | trips (legs), time per trip, contacts, path length, tracking error (`track_v_*`, `track_wz_med`), controller step time (`step_ms_*`), load average, power profile, per-step trace |

### Disclosed differences from the pilot's generation (apparatus, not arm)

1. **CLI 2.1.302** (pilot: 2.1.278). The pilot's version is no longer installed.
2. **`--strict-mcp-config`** (Barna, 2026-11-02). The pilot ran without it and
   loaded no connectors either (0 of 16 session logs). Under 2.1.302, one of
   two calibration calls loaded the account's claude.ai connectors without it.
3. **Calls in flight:** the pilot ran 6 at a time. Here they run in batches of
   at most 4, so that the cost cap below can be enforced between batches.
4. **Date / model serving:** the model id is the same (`claude-opus-5`); the
   serving stack may have changed since 2026-09-29.

So armX is compared against a pilot generated **under a slightly different
apparatus**. That weakens every armX-vs-pilot contrast, and the write-up says so.

## Gates (checked before more calls are made)

- **R3 token gate:** each armX call's **first-turn** input tokens (from the CLI
  session log, not the JSON total) must be within **±3 %** of armB's 26,464
  (the tolerance Barna set on 2026-11-02). Outside it: stop generating and
  report.
- **Cost gate:** the next batch starts only if *spent so far + 2.50 × (calls
  in the batch)* ≤ **$16** (2.50 is each call's `--max-budget-usd` cap). If
  fewer than 8 controllers are affordable, k is what was generated, and that is
  reported.
- **Failures stay in the denominator:** a call that fails, times out or returns
  no fenced block is a 0-trip controller. The one exception: a call that dies
  at the **output-token cap** is retried once (the project's rule), and only
  if the cost gate allows it.
- **R4 (tools):** for every call, the number of turns and the tools invoked are
  read from the session log and reported. Calls that used tools are **not**
  excluded; the pilot's calls used tools too (8 of 16 were multi-turn).

## Primary outcome and analysis

**Unit:** the controller. **Success:** a controller completes **all 4 trips
within the 180 s budget** (`legs = 4`).

**Primary (comparable with the pilot):** armX successes in the **rep 0**
physics episode, the pilot's exact condition, out of 8. Two Fisher exact
tests, two-sided:

- **X0:** armX vs arm0 (7/8)
- **XB:** armX vs armB (2/8)

These are **Holm-corrected** over the two tests at α = 0.05. Uncorrected p,
Wilson 95 % intervals per arm, and Newcombe intervals on both differences are
reported beside them.

**Fixed in advance:** every possible armX result s / 8 and its p-values
(scipy `fisher_exact`, two-sided):

| armX s/8 | p X0 (vs 7/8) | p XB (vs 2/8) | Holm X0 | Holm XB | reading |
|---|---|---|---|---|---|
| 0 | 0.0014 | 0.467 | **0.003** | 0.467 | L |
| 1 | 0.0101 | 1.000 | **0.020** | 1.000 | L |
| 2 | 0.0406 | 1.000 | 0.081 | 1.000 | inconclusive (leans L) |
| 3 | 0.119 | 1.000 | 0.238 | 1.000 | inconclusive |
| 4 | 0.282 | 0.608 | 0.564 | 0.608 | inconclusive |
| 5 | 0.569 | 0.315 | 0.629 | 0.629 | inconclusive |
| 6 | 1.000 | 0.132 | 1.000 | 0.264 | inconclusive |
| 7 | 1.000 | 0.0406 | 1.000 | 0.081 | inconclusive (leans C) |
| 8 | 1.000 | 0.0070 | 1.000 | **0.014** | C |

**Interpretation, fixed in advance:**

| pattern | reading |
|---|---|
| X0 rejected (Holm), XB not | **L:** a long off-topic document is enough to produce the drop. The content claim is withdrawn |
| XB rejected (Holm), X0 not | **C:** the drop needs D\*'s content, not just a long document |
| neither rejected | **inconclusive at n = 8.** Report the counts and intervals. No directional claim; "leans" may only be stated with its interval |

⚠️ At n = 8 per arm, only 3 of the 9 possible outcomes decide anything. A
non-rejection is **not** evidence that two arms are equal.

⚠️ The comparators are the **pilot's** counts. They were found after the
kinematic-vs-physics flip and were not pre-registered (winner's curse, forking
paths). Exp G inherits that weakness, and the write-up states it.

## Secondary (exploratory, labelled as such)

1. **Majority of 3 reps:** a controller succeeds if it completes 4 trips in at
   least 2 of its 3 reps. armX against arm0 and armB on the same rule, using
   the Exp 3 rows (`demo/out/v2/physics.csv`) **once Exp 3 has finished all 3
   reps for arm0 and armB**. Fisher, two-sided, uncorrected, labelled
   exploratory.
2. **Mean trips per controller** (rep 0, and over reps), with Mann–Whitney U
   against arm0 and armB.
3. **Kinematic success** (legs = 4, seed 0) for armX against arm0 and armB,
   all three re-run now by the same `demo/run.py` into
   `demo/out/v2/expG/kinematic.csv`, with Fisher. The rollout is
   deterministic, so arm0/armB should reproduce the pilot's kinematic counts.
   `demo/out/results.csv` predates the corrected world and is not used. The kinematic simulator is
   **not** a proxy for physics: the pilot flipped there.
4. Per-episode collisions, tracking error, controller step time (wall-clock-
   budgeted controllers, R5) and time per trip, described per arm.

## Outputs

- `demo/arms/armX/k*.py`, `demo/transcripts/armX__k*__a848cebd7806d2cc.json`
- `demo/out/v2/expG/kinematic.csv`, `demo/out/v2/expG/physics.csv`
  (+ `traces/`, `manifest.json`, `sweep.log`)
- `demo/controls/manifest.json` → `expG`: CLI version, first-turn tokens,
  turns, tools and cost per call
- results appended to this file as a dated **Outcome** section

The pilot's files (`demo/arms/arm0`, `demo/arms/armB`, `demo/out/physics.csv`,
`demo/out/results.csv`) and Exp 3's outputs are **not touched**.

## Note for PREREG-v2 (not frozen; not edited here)

Exp G occupies armX **k0–k7**, the same indices the pilot occupies for
arm0/armB. If the confirmatory campaign runs later, `analyse.FRESH_START["armX"]`
must become **8**, as it is for arm0 and armB. Otherwise these 8 would be
counted as fresh confirmatory samples.

---

## Outcome (appended 2026-11-03; nothing above was edited)

**Generation (2026-11-02):** 8/8 calls produced a controller. Total **$11.92**
($1.22–1.99 per call), under the $16 cap. First-turn input tokens were
26,869–26,872 (+1.5 % vs armB, inside the ±3 % gate), and no call loaded MCP.
R4: k3 ran 4 turns with tools (Bash ×2, Write) in its scratch directory; the
other 7 were single-turn. Details: `demo/controls/manifest.json` → `expG`.

**Physics (2026-11-03):** 24 episodes, `demo/out/v2/expG/physics.csv`.
Every episode ran on AC with profile `performance`; `--require-ac` was added
to `physics_sweep.py` after the laptop went to battery twice while the profile
still read `performance`. Load average during the episodes was 6–8.6, mostly
Isaac itself. Analysis output: `demo/out/v2/expG/analysis.txt`
(`demo/expG_analyse.py --require-complete`, unchanged since the pre-registration
commit).

### Primary (rep 0 = the pilot's condition, all four trips)

| arm | success | Wilson 95 % |
|---|---|---|
| arm0 (pilot) | 7/8 | [0.53, 0.98] |
| armB (pilot) | 2/8 | [0.07, 0.59] |
| **armX** | **0/8** | [0.00, 0.32] |

- **X0**, armX vs arm0: diff −0.875, Newcombe [−0.98, −0.40], Fisher p = 0.0014, **Holm p = 0.0028, REJECTED**
- **XB**, armX vs armB: diff −0.250, Newcombe [−0.59, +0.12], Fisher p = 0.467, Holm p = 0.467, not rejected

**Pre-registered reading: L.** A long off-topic document is enough to produce
the physics drop, so **the content claim is withdrawn**. armX is at least as
bad as armB. It is **not** evidence that armX equals armB: the interval on the
difference runs from −0.59 to +0.12.

### Secondary (exploratory)

- Trips per controller at rep 0: armX k0–k7 = 1, 2, 2, 0, 2, 0, 3, 1. Mean
  1.38, against arm0 3.75 (MWU p = 0.0013) and armB 2.38 (MWU p = 0.11).
  The door shuts after the first two trips (`LEG_BLOCKED`). **4 of 8 armX
  controllers fail before it does** (k0, k3, k5, k7: under 2 trips, on the
  open route); 3 stop exactly where the door blocks the short route, which is
  the pilot's armB failure mode. So part of armX's failure is broader than
  armB's.
- Majority of 3 reps: armX 1/8 (k6 only, 4/4 in reps 1–2), arm0 4/8, armB
  2/8 (Exp 3 rows). Fisher armX vs arm0 p = 0.28, vs armB p = 1.0. ⚠️ Under
  Exp 3's three reps, arm0 itself falls from 7/8 to 4/8, so the pilot's arm0
  comparator is optimistic.
- Kinematic (seed 0, same `run.py` run): armX 1/8, arm0 5/8 (p = 0.12),
  armB 6/8 (p = 0.041). The kinematic simulator also separates armX, unlike
  arm0 vs armB.
- armX per episode: contacts mean 2.3 (median 0), path 44 m (median 34 m),
  controller step time median 1.1 ms (p95 4.1 ms). These controllers are not
  starved by a planning budget.

### Caveats

The comparators are the unregistered pilot. armX was generated on CLI 2.1.302
with `--strict-mcp-config` and the pilot on 2.1.278 without it, so an apparatus
effect cannot be excluded. That needs fresh arm0 samples under the same CLI,
which is the confirmatory campaign. One world, local laptop, not in the
container. n = 8.
