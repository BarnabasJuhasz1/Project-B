<!--
demo/JUDGE_PROMPT.md — the fixed prompt for Exp 4's SECOND RATER (an LLM judge).

PREPARED, NOT RUN. No call has been made with it. Running it is a paid API call
and needs the user's go-ahead (Mindfire_Experiment_Plan.md §4.2, §4.4).

How it is meant to be used, once approved:
  - one call per BLINDED file (demo/out/v2/blind/<set>/<blind_id>.py), fresh
    context each time, the same model and settings for every file;
  - {{RUBRIC}} is replaced by demo/CODING.md byte for byte, {{CODE}} by the
    blinded file byte for byte, {{BLIND_ID}} by its id; nothing else changes;
  - the rendered prompt's sha256 is recorded per file, and the reply is stored
    verbatim beside the parsed row;
  - the judge never sees the key, the arm names, the features CSV, rater 1's
    labels or any other file.
Rater 1 and the judge are compared with Cohen's kappa per dimension;
disagreements are resolved by discussion and logged, never by re-running the
judge until it agrees.
Everything above this comment block is stripped before rendering.
-->
You are coding one Python navigation controller against a fixed rubric. The
controller was written for a Unitree Go2 that shuttles between two points in an
unmapped building, using a 2-D lidar and a pose estimate, and sends only a
velocity command `(vx, vy, wz)` every 20 ms.

Read the rubric first, then the code, then answer. Code what the code is
**written to do** — its structure — not what you guess it does at runtime. Apply
each decision rule as written, including its precedence and exclusion rules.
Where a file mixes mechanisms, the rubric says which one wins. If no category
fits, answer `unclassifiable` and say why in `notes`.

Comments and identifiers in the code may name a paper or algorithm. Do not
take a name as evidence: a file that calls something "D\*" is coded on what that
code does, and the faithfulness checklist exists to check exactly that.

## The rubric

{{RUBRIC}}

## The controller (blind id `{{BLIND_ID}}`)

```python
{{CODE}}
```

## Your answer

Reply with **one JSON object and nothing else**, with exactly these keys and only
the codes the rubric defines for each:

```json
{
  "blind_id": "{{BLIND_ID}}",
  "planner_type": "full_every_step | full_periodic | incremental_only | incremental_with_fallback | no_global_planner | unclassifiable",
  "map": "persistent_grid | none | unclassifiable",
  "velocity_selection": "rollout_scored | path_following | reactive | unclassifiable",
  "stuck_handling": "explicit_recovery | timeout | none | unclassifiable",
  "claims_dstar": "yes | no",
  "dstar_variant": "stentz1994 | dstar_lite | other | na",
  "ds_open_keyed_on_k": "yes | partial | no | na",
  "ds_h_and_k": "yes | partial | no | na",
  "ds_raise_lower": "yes | partial | no | na",
  "ds_modify_cost": "yes | partial | no | na",
  "ds_stop_rule": "yes | partial | no | na",
  "ds_backpointers": "yes | partial | no | na",
  "notes": "for every non-obvious label, the function or method name that decided it"
}
```
