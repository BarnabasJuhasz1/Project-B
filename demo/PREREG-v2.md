---
created: 2026-11-02
status: DRAFT — NOT FROZEN, NOT TAGGED
---

# `demo/PREREG-v2.md` — pre-registration of the Mindfire confirmatory study

> ⚠️ **DRAFT. Nothing here is frozen until Barna approves it, the open decisions in
> §11 are filled in, and the commit is tagged** (`mindfire-prereg-v2`, proposed).
> Until then any line may change. After the tag, a change is a **dated append** to
> §12, never an edit.

Source of the design: `Mindfire_Experiment_Plan.md` (2026-10-28), §2–§5. This
document fixes what that plan leaves to be fixed. The analysis is **code**, not
prose: `demo/analyse.py` is the analysis, and where this text and that file
disagree, the file at the tagged commit wins and the disagreement is reported.
Power: `demo/power.py`. Tests of both: `demo/test_analyse.py`.

## 1. What is already known, and that it was not pre-registered

The **pilot**: 16 controllers (`demo/arms/{arm0,armB}/k0..k7.py`), one physics
episode each (`demo/out/physics.csv`, 2026-10-03). Success = all four trips:
**arm0 7/8, armB 2/8, Fisher exact two-sided p = 0.0406** (reproduced by
`python demo/analyse.py --pilot`). Under the kinematic simulator the same 16 gave
7/8 vs 6/8, p = 1.000.

⚠️ The pilot was **not** pre-registered, and the physics comparison was chosen
**after** the kinematic result came out null (forking paths). It generated the
hypothesis and is never part of the confirmatory test.

## 2. Hypotheses

Confirmatory, on **fresh** controllers only (§3), at the controller level:

| | contrast | reading |
|---|---|---|
| **H1** replication | armB ≠ arm0 | the pilot difference replicates |
| **H2** content vs length | armB ≠ armX | the paper's content, not a long document, drives it |
| **H3** length / distraction | armX ≠ arm0 | any long document is enough |

All two-sided. The pilot direction (armB worse) is stated as the expectation; it
is **not** used to make the tests one-sided.

## 3. Design and sample size

| | value |
|---|---|
| arms | `arm0` (task only), `armB` (task + Stentz 1994, `paper.txt`), `armX` (task + length-matched Feng et al. 2015, `controls/feng2015.txt`) |
| prompts | `prompt.md` and `task.md` **unchanged**; the three renders byte-identical outside the paper block (`render.check()`) |
| prompt hashes | arm0 `c6a5b5e6eac0def5`, armB `4045efb3e514b981`, armX **`<to be filled from the armX build>`** |
| model | `claude-opus-5` via `ClaudeCliBackend`, CLI version recorded per transcript. If not served: all three arms generated with the successor, pilot reported separately and never pooled (plan R2) |
| **k per arm (fresh)** | ⚠️ **DECISION FOR BARNA: 24 or 30** (§11). Recommended **30**. Written into `analyse.K_CONFIRMATORY` at the freeze |
| fresh k indices | arm0 `k8..k(7+k)`, armB `k8..k(7+k)`, armX `k0..k(k-1)` (`analyse.FRESH_START`) |
| generation order | all 3k jobs shuffled with a fixed seed **`<shuffle seed, to be fixed>`**, `--jobs 6`, 1800 s timeout |
| physics reps | 3 per controller (Exp 3): rep 0 = seed 0, offset (0, 0, 0), the original condition; rep 1 = seed 1, (−0.088 m, +0.077 m, +0.023 rad); rep 2 = seed 2, (+0.038 m, −0.069 m, +0.066 rad). Same offsets for every controller. Drawn from `PERTURB_SEED = 3003`; episode order from `ORDER_SEED = 3` (`demo/exp3.py` on branch `mindfire-exp3`; **verify against the committed file at the freeze**) |
| budget | 180 s for the whole four-leg episode |
| environment | the one hand-authored world (`demo/world.py`, `BLOCKED_WORLD` after leg 1), frozen Go2 policy, `pose_error_estimate = 0.0` |

### Power (exact enumeration, `demo/power.py`)

Controller-level success probabilities; Fisher two-sided; "Holm" is the exact
power of H1 under Holm over H1–H3 with armX = arm0 (the content scenario).

| true rates arm0 vs armB | k = 24 single test | k = 24 **Holm** | k = 30 single test | k = 30 **Holm** |
|---|---|---|---|---|
| 0.875 vs 0.25 (pilot point estimate) | 0.996 | 0.989 | 0.999 | 0.998 |
| 0.85 vs 0.50 (plausible shrunk) | 0.668 | **0.506** | 0.777 | **0.648** |
| 0.875 vs 0.5625 (half the pilot gap) | 0.599 | — | 0.709 | — |
| 0.80 vs 0.55 (small) | 0.356 | 0.211 | 0.444 | 0.297 |

- Probability the **content** reading fires when it is true (shrunk): **0.38 at
  k = 24, 0.51 at k = 30**, 0.73 at k = 40.
- Smallest k giving 80 % Holm power on H1 at the shrunk effect: **k = 40**.
- Smallest gap detectable at 80 % (single test, arm0 = 0.875): 0.40 at k = 24,
  0.35 at k = 30.
- Family-wise error under the null (all arms 0.75): 0.006–0.008. Fisher is
  conservative at these sizes.
- ⚠️ The plan's §2.4 table (~0.7 at k = 24, ~0.8 at k = 30) is the **single,
  uncorrected** test. The Holm correction it also specifies costs 0.13–0.16 of
  power at the shrunk effect. Both are true; the design as written is
  under-powered for a shrunk effect at either k.
- Assumes per-controller verdicts are measured without error. If Exp 3 shows reps
  disagreeing often, the effective effect shrinks and all of these are optimistic.

## 4. Outcome and success definition

- **Unit of analysis: the controller** (one independent generation call).
  Episodes of one controller are never treated as independent samples.
- **Episode success:** all **4** trips completed within the 180 s budget
  (`legs == 4`). `physics_sweep.py`'s printed `legs >= 3` is **not** the rule.
- **Controller success (primary outcome):** episode success in **at least 2 of
  its 3** physics reps.

## 5. Exclusions and missing data

| case | handling |
|---|---|
| generation call fails, times out, or returns no code block | **stays in the denominator** as a 0-trip controller in every rep (`<arm>__k<k>__FAILED.json`) |
| call dies at the **output-token cap** | retried **once**; retry count reported per arm. A second cap failure is a 0-trip controller |
| controller fails the import gate or raises in `step()` | counts as what it achieved (legs completed before the raise). It is the model's output |
| episode produces no `RESULT` line (simulator crash, apparatus) | **re-run**, never scored as 0. Two failed re-runs of one episode stop the sweep for diagnosis |
| episode run while a generation call or another simulator was active | re-run; the original stays in the log and is not analysed |
| fewer than 3 valid reps for any confirmatory controller | `analyse.py` **refuses** (`Incomplete`). No confirmatory number is produced from a partial set |
| armX samples generated before a failed token-match check (plan R3: first armX transcripts' input tokens > 1 % from armB's 25,314) | generation stops, the control is re-truncated, the affected armX samples are **reported but replaced** and the replacement count is stated |
| the `(base)` reference row | descriptive only, never tested |
| pilot 16 | excluded from H1–H3; enter S5 (pooled), Exp 3 and Exp 4 |

No controller is excluded for any reason based on its outcome.

## 6. Primary analysis (`analyse.primary`)

1. Per arm: successes / k, Wilson 95 % interval.
2. H1–H3: Fisher exact, two-sided, on fresh controllers; difference with a
   Newcombe hybrid-score 95 % interval (method 10).
3. Holm across H1, H2, H3, family-wise α = 0.05.
4. **Interpretation table, applied mechanically** (`analyse.interpret`):

| pattern | reading |
|---|---|
| H1 not rejected | **NOT_REPLICATED.** Report the pilot as a pilot and the confirmatory null as the result |
| H1 and H2 rejected, H3 not, armB differs from arm0 and armX in the same direction | **CONTENT:** the paper's content makes the controllers worse (or better) under physics |
| H1 and H3 rejected, H2 not | **LENGTH:** a long document, not this paper, is enough. The content claim is withdrawn |
| anything else | **NO_CLAIM:** report the intervals, no directional claim |

"armX close to arm0" in the plan is operationalised as **H3 not rejected**. ⚠️ A
non-rejected comparison is never reported as equality; it is reported with its
interval (≈ ±0.25 wide at these k).

## 7. Secondary analyses — exploratory, Holm within the family (`analyse.secondary`)

On the fresh controllers, for each of the three contrasts:

- **S1** mean trips per controller (mean legs over its 3 reps), Mann–Whitney U.
- **S2** leg-2 completion time, median over reps, a never-completed leg = +∞
  (ranked worst), Mann–Whitney U.
- **S3** collisions per metre (contacts summed over reps / path summed over reps),
  Mann–Whitney U. Controllers with zero path are left out and counted.
- **S4** kinematic success (`demo/out/v2/kinematic.csv`, same majority rule),
  Fisher.

Plus **S5**: arm0 vs armB pooled, pilot + fresh (8 + k each), primary outcome,
Fisher. One Holm correction over every test computed (up to 13). A test that
cannot be computed is listed as unavailable, never dropped silently.

## 8. Exp 3 analyses — descriptive (`analyse.exp3`)

- Per-episode success per arm, difference with a **cluster bootstrap over
  controllers** (10,000 resamples, seed 20261102, percentile interval).
- Within-controller agreement: controllers whose 3 reps all agree, per arm.
- Tracking error per arm: median over controllers of the per-controller median
  `track_v_med` (|realised − commanded| planar velocity, from Exp 3's traces).
- **Rep 0 vs the pilot** (`demo/out/physics.csv`): legs and leg times row by row.
  A mismatch is reported; it does not stop the analysis, but it means the pilot
  table itself carries evaluation noise and says so in the write-up.

## 9. Exp 4 analyses — exploratory, correlational (`analyse.exp4`)

Labels per `demo/CODING.md` (frozen on branch `mindfire-exp4`), coded blind,
joined to (arm, k) only after coding. Rows with `rater == "consensus"` are the
labels; the two raters give Cohen's κ per dimension.

- Label distributions per arm (descriptive, primary for this experiment).
- **E10:** fraction of arm0 controllers with any incremental repair
  (`incremental_only` or `incremental_with_fallback`), Wilson interval; same for
  armB and armX.
- **Association:** physics success for `incremental_only` vs any planner with a
  full-replan path (`full_every_step`, `full_periodic`,
  `incremental_with_fallback`), Fisher two-sided, over **all** labelled
  controllers (pilot + fresh, every arm) and within armB. `no_global_planner`
  and `unclassifiable` are on neither side. Not corrected; labelled exploratory.
  It cannot establish cause.

## 10. Stopping rules

- **No interim test of H1–H3.** No arm-level tally of fresh physics results is
  computed, and no decision about the campaign depends on outcomes, until every
  confirmatory episode exists.
- Generation runs to the full k in every arm. The sweep runs every episode.
- Allowed stops, all on the **apparatus**, never on outcomes:
  1. `claude-opus-5` not served at the start → plan R2 before any call.
  2. Rep 0 of the pilot fails to reproduce `physics.csv` (step 1) → stop and
     diagnose before the fresh sweep; report what was found.
  3. armX token mismatch > 1 % (plan R3) → §5.
  4. Two failed re-runs of one episode → stop, fix, resume.
  5. Cost exceeds the worst case in plan §2.6 by more than 25 % → stop and ask;
     the samples generated so far are kept and nothing is re-drawn.
- If the campaign ends short of k for reasons outside these rules, the primary
  test is **not run**; the partial data are analysed with
  `--allow-incomplete`, every number marked EXPLORATORY.

## 11. Open decisions — Barna

1. **k = 24 or 30 per arm** (or 40). Recommendation: **30**. At the plausible
   shrunk effect it gives 0.65 Holm power on H1 against 0.51 at 24, for about
   +$25–40 and +4 GPU-hours. It still falls short of 80 %; that needs k = 40
   (~$170 typical / ~$265 worst, ~31 GPU-h) and is the honest option if the
   budget allows.
2. **Multiplicity procedure.** The plan says Holm over H1–H3 and `analyse.py`
   implements that. An alternative that also controls family-wise error: test H1
   alone at 0.05 and, only if it is rejected, Holm over H2/H3 (gatekeeping; the
   interpretation table needs H1 first anyway). At k = 30, shrunk effect: H1
   power 0.78 vs 0.65, content reading 0.58 vs 0.51. **Keep the plan unless you
   choose otherwise;** switching is a change to `analyse.holm` usage plus a test.
3. The generation **shuffle seed** and the **armX prompt hash** (from the armX
   build).
4. Container or local for the v2 sweep (plan §3.3), stated before it starts.
5. Approve this document, then commit and tag it **before the first generation
   call**. Optional OSF mirror.

## 12. Amendments

*(dated appends only, after the tag)*
