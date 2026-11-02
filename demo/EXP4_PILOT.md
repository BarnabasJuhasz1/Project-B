# Exp 4 — blinded code analysis, PILOT 16 (2026-11-02)

Steps 2 of `Mindfire_Experiment_Plan.md` §4–§5: rubric frozen, features
extracted, pilot set blinded and coded by rater 1. **No second rater yet**: the
LLM-judge prompt is prepared (`demo/JUDGE_PROMPT.md`) and has **not** been run.
No κ exists. Everything below is one rater.

| file | what |
|---|---|
| `demo/CODING.md` | the rubric, committed (`637bdd0`) before any controller was read in detail |
| `demo/code_features.py` → `demo/out/v2/code_features.csv` | static AST / regex features, keyed by (arm, k), counted on the header-stripped body |
| `demo/blind_controllers.py` → `demo/out/v2/blind/pilot/` | 16 blinded files (header stripped, random ids). **Not committed**: regenerable from the key's seed |
| `demo/out/v2/blind_keys/pilot_key.csv` | blind id → arm, k, source and blinded sha256, seed |
| `demo/out/v2/code_labels_pilot.csv` | rater 1's labels, by blind id, with the deciding method named in `notes` |
| `demo/test_code_features.py` | 22 tests (detectors, header split, blinding invariants, the real pilot set) |

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P -m pytest demo/test_code_features.py -q
    $P demo/code_features.py
    $P demo/blind_controllers.py --set pilot --seed <seed from the key> --force   # rebuilds the identical blinded set

## Label distributions per arm (rater 1, n = 8 per arm)

| dimension | arm0 | armB |
|---|---|---|
| planner type | full_periodic **8** | incremental_only **5**, incremental_with_fallback **2**, full_periodic **1** |
| map | persistent_grid 8 | persistent_grid 8 |
| velocity selection | rollout_scored **4**, path_following 4 | path_following **8** |
| stuck handling | explicit_recovery 8 | explicit_recovery 8 |
| claims D\* | no 8 | yes 8 |
| D\* variant | na 8 | stentz1994 **4**, other 4 |

- **E10 — arm0 with any incremental-repair machinery: 0 of 8.** All eight
  recompute a global field from scratch, gated (a step counter, a period, a
  forced flag, or an anytime wave restarted after it converges). None does it
  literally every step, so the README's "recompute globally" is right and
  "every step" would not be.
- **armB with incremental repair: 7 of 8.** The eighth (k5) keeps a field per
  goal but resets it from scratch whenever a new obstacle cell appears, and only
  propagates cost *decreases* incrementally — coded `full_periodic`, marked
  BORDERLINE in its notes.
- **The four Stentz-faithful armB files** (k0, k1, k2, k7: heap OPEN keyed on
  `k`, `h`/`k`/`t`/`b`, PROCESS-STATE L1–L25, MODIFY-COST re-OPENing CLOSED
  states) all cut the stopping rule short with a wall-clock budget
  (`ds_stop_rule = partial`). The other four call vectorised Bellman sweeps "D\*"
  but keep no `k` and no OPEN list (`dstar_variant = other`).
- ⚠️ **One disagreement with the pilot README.** It lists armB k0, k1, k2 as
  "purely incremental, `full-replan` keyword count 0". Rater 1 codes **k2 as
  `incremental_with_fallback`**: `_step` calls `reset_tree()` when the traced
  backpointers are corrupted. A keyword count cannot see a fallback that is not
  called "replan".
- `code_features.rollout_scoring` agrees with the rater's `velocity_selection`
  on **16 of 16** files (the only static feature with a one-to-one rubric
  counterpart).

### Exploratory only — not a test, n = 16, pilot

Joined with the README's single-run physics trips (4/4 = success):
`incremental_only` 2 of 5 succeed; every type with a full-replan path 7 of 11.
Stentz-faithful D\* 0 of 4; D\*-named sweeps 2 of 4. This is the pilot that
generated the hypothesis, one physics run per controller, one rater; it may
inform `PREREG-v2.md` and must not be reported as a finding.

## Disclosures

- ⚠️ **Rater 1 is the agent that wrote the rubric and the blinding script, and
  was not blind to the arm-level story**: it had read the pilot README (which
  says armB builds D\* and arm0 does not) and, while designing the header
  stripper, the headers of arm0 k0, armB k0 and armB k4. It recognised arm0 k0's
  blinded file (noted in that row). Blinding is nominal for rater 1 on this
  pilot; the second rater is what makes κ meaningful.
- ⚠️ **Residual leak**: 8 of 16 blinded files still carry D\* vocabulary in the
  body (comments and identifiers). Header stripping alone does not blind armB.
  `--strip-comments` removes `#` comments; identifiers stay because the D\*
  checklist needs them.
- Two codes are flagged for the second rater: armB k5 (`full_periodic`, could be
  read as `incremental_with_fallback`) and armB k7 (`incremental_with_fallback`
  on one narrow trigger: a new D\* tree when the goal cell turns lethal).
- Coded from the code only; nothing was executed.
