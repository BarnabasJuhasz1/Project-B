# Exp 4 — blinded code analysis, PILOT 16 (2026-11-02)

Step 2 of `Mindfire_Experiment_Plan.md` §4–§5: the rubric was frozen, features
were extracted, and the pilot set was blinded and coded by **two raters**. Rater 1
is the agent that wrote the rubric (`claude-opus-5-5`). Rater 2 is an LLM judge
(`claude-sonnet-5-5`), run once per blinded file through `demo/JUDGE_PROMPT.md`
on 2026-11-02 with the user's approval. Disagreements were resolved into
**consensus** rows, each with a logged reason. The distributions below are the
consensus labels.

| file | what |
|---|---|
| `demo/CODING.md` | the rubric, committed (`637bdd0`) before any controller was read in detail |
| `demo/code_features.py` → `demo/out/v2/code_features.csv` | static AST / regex features, keyed by (arm, k), counted on the header-stripped body |
| `demo/blind_controllers.py` → `demo/out/v2/blind/pilot/` | 16 blinded files (header stripped, random ids). **Not committed**: regenerable from the key's seed |
| `demo/out/v2/blind_keys/pilot_key.csv` | blind id → arm, k, source and blinded sha256, seed |
| `demo/out/v2/code_labels_pilot.csv` | rater 1's labels, by blind id, with the deciding method named in `notes` |
| `demo/test_code_features.py` | 22 tests (detectors, header split, blinding invariants, the real pilot set) |
| `demo/run_judge.py` | rater 2: one `claude -p` call per blinded file, in an empty scratch directory, with `--tools ""`, `--strict-mcp-config` and `--setting-sources ""`, under a fixed system prompt and a $5 hard cap |
| `demo/out/v2/judge/pilot/<blind_id>.json` | every judge call verbatim: prompt sha256, argv, CLI version, the CLI's JSON payload (usage, cost) and the parsed row |
| `demo/out/v2/code_labels_pilot_rater2.csv` | rater 2's labels |
| `demo/out/v2/code_labels_pilot_consensus.csv` | **rater 1 + rater 2 + `consensus` rows**, the format `analyse.py` reads (`--labels … --code-key demo/out/v2/blind_keys/pilot_key.csv`) |
| `demo/out/v2/code_disagreements_pilot.csv` | every disagreement: both codes, the consensus code and the reason |

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P -m pytest demo/test_code_features.py -q
    $P demo/code_features.py
    $P demo/blind_controllers.py --set pilot --seed <seed from the key> --force   # rebuilds the identical blinded set
    $P demo/run_judge.py --set pilot --write-only     # rater-2 CSV from the committed records, no calls

(`P="env -u PYTHONHOME -u PYTHONPATH ~/miniforge3/envs/env_isaacsim/bin/python"` on
this machine: the inherited environment sets `PYTHONHOME` and Python does not
start without unsetting it.) The blinded set was rebuilt in `Repo/` from the
key's seed before the judge ran. The regenerated key, including every blinded
file's sha256, is byte-identical to the committed one, so the judge coded the
same bytes rater 1 did.

## Inter-rater agreement (16 files, unweighted Cohen's κ)

| dimension | κ | agreement |
|---|---|---|
| planner type | **1.00** | 16/16 |
| map | undefined (one category) | 16/16 |
| velocity selection | **1.00** | 16/16 |
| stuck handling | undefined (one category) | 16/16 |
| claims D\* | **1.00** | 16/16 |
| D\* variant | **0.80** | 14/16 |
| `ds_open_keyed_on_k` | 1.00 | 16/16 |
| `ds_h_and_k` | 1.00 | 16/16 |
| `ds_raise_lower` | 0.90 | 15/16 |
| `ds_modify_cost` | 1.00 | 16/16 |
| `ds_stop_rule` | 1.00 | 16/16 |
| `ds_backpointers` | 0.71 | 13/16 |

`map` and `stuck_handling` take a single value in both raters' codes on every
file, so κ is undefined there (`analyse.cohen_kappa` returns 1.0 when chance
agreement is 1). Report them as 100 % agreement with no κ. The two codes
flagged for the second rater (armB k5 `full_periodic`, armB k7
`incremental_with_fallback`) were **both confirmed** independently.

**The six disagreements and how they were resolved**
(`code_disagreements_pilot.csv` has the full reasons):

| file (arm k) | column | rater 1 | rater 2 | consensus | why |
|---|---|---|---|---|---|
| c53162f95 (armB k3) | D\* variant | other | stentz1994 | **other** | no `k`, no OPEN list (both raters' checklist agrees); rater 2 coded the same structure `other` elsewhere |
| c6538702a (armB k4) | D\* variant | other | stentz1994 | **other** | same |
| c6538702a (armB k4) | backpointers | partial | yes | **partial** | argmin of `h` alone, no arc cost: neither `b()` nor the `c + g` analogue |
| cef0231d9 (armB k6) | backpointers | partial | yes | **yes** | rater 1 was wrong: `_carrot` takes argmin of `h + c` |
| cf90a0a80 (armB k5) | raise/lower | partial | no | **no** | a cost increase resets the field; no RAISE state is ever processed |
| cf90a0a80 (armB k5) | backpointers | partial | no | **no** | heading = argmin over a lookahead ring; no pointer or successor walk |

All six are on D\*-checklist columns of armB files. **No disagreement touches
planner type, map, velocity selection or stuck handling**, so every headline
number below is the same under either rater.

## Label distributions per arm (consensus, n = 8 per arm)

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
generated the hypothesis, with one physics run per controller. It may inform
`PREREG-v2.md` and must not be reported as a finding.

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
- ⚠️ **Rater 2 is blind to the file→arm mapping but not to the design.** It
  saw only `CODING.md` (verbatim, as its rubric), one blinded file and the fixed
  prompt. It did not see the key, the README, the unblinded arms, rater 1's
  labels or the features. But `CODING.md` itself names arm0, armB, armX and
  Stentz, and the D\* terms left in the bodies point to armB just as they did
  for rater 1. So κ here measures agreement on what the code does, **not**
  agreement between raters who could not tell the arms apart. On 8 of 16 files
  (all of armB) the arm is effectively visible to any reader.
- ⚠️ **Raters are two models from one family** (Opus 5.5 and Sonnet 5.5), not two
  people. Shared training could inflate agreement. A human second rater would
  be the stronger check.
- **Cost: 17 judge calls for 16 files, about $1.26 in total.** The first batch
  ran in a foreground shell with a 2-minute timeout, which killed it after 12
  recorded calls. A 13th call, for `ce52dcec0`, had already reached the API: its
  re-run recorded `cache_read_input_tokens = 14,535` against 335 created, so
  the killed call wrote that prompt to the cache. That call left no record and
  no label, and its cost is not in the CLI's accounting here; at the per-call
  rate seen (≈ $0.07–0.09) it is ≈ $0.08. **Recorded cost: $1.18** (16 calls,
  `total_cost_usd`). One coded call per file: the killed call produced nothing,
  so no file was coded twice and no call was repeated to change an answer.
- Coded from the code only; nothing was executed.
