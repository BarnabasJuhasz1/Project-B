# `demo/CODING.md` — the Exp 4 code-analysis rubric

**Frozen 2026-11-02, before any controller was read in detail.** Exp 4 of
`Mindfire_Experiment_Plan.md` (§4.2). The five dimensions and their categories
are copied **exactly** from §4.2. What this file adds is a *decision rule* for
each one, so two raters reading the same file reach the same label. A change to
any category or decision rule after this commit is a **dated append** at the
bottom, never an edit, and labels coded under the old text are re-coded.

⚠️ **This is a static reading of the code, not of its behaviour.** A label says
what the code is written to do. Whether a branch ever runs on the patrol world is
a runtime question and belongs to Exp 3's traces, not here.

⚠️ **Code the blinded file only** (`demo/blind_controllers.py` output). The
header comment is stripped there because the prompt asks armB and armX to name
their source at the top. Body comments and identifiers are *not* stripped and
can still reveal the arm (e.g. `PROCESS-STATE`, `rhs`). That residual leak is
declared, not hidden: the D\* checklist below cannot be coded without reading
D\* vocabulary.

## Unit, output, and the escape hatch

- One row per controller file, in `demo/out/v2/code_labels*.csv`, keyed by the
  **blind id**. The key (blind id → arm, k) lives in a separate file and is
  joined only after coding is finished.
- Columns: `blind_id, rater, planner_type, map, velocity_selection,
  stuck_handling, claims_dstar, dstar_variant, ds_open_keyed_on_k, ds_h_and_k,
  ds_raise_lower, ds_modify_cost, ds_stop_rule, ds_backpointers, notes`.
- **One label per dimension.** Where a file contains several mechanisms, the
  decision rule says which one wins. If none of the categories fits, write
  `unclassifiable` and say why in `notes`. That is an escape hatch, not a sixth
  category, and every use of it is reported.
- `notes` must cite the function or method name that decided each non-obvious
  label, so a second rater can check the reading.

## 1. Planner type

| code | category (§4.2) | decision rule |
|---|---|---|
| `full_every_step` | full replan every step | A global cost-to-go field or path to the goal is recomputed **from scratch** on every call to `step()` (or every control step it is not skipped by a gate). |
| `full_periodic` | periodic full replan | From-scratch global recomputation, but **gated**: a timer, a step counter, a map-changed flag, a path-blocked / path-invalid test, a wall-clock budget, or a search sliced over several steps and then restarted. Any gate at all puts it here, however often it fires in practice. |
| `incremental_only` | incremental only | The planner keeps its search state (values, keys, OPEN list) **across steps** and, when sensed costs change, **repairs only the affected states** (D\*, Focused D\*, D\* Lite, LPA\*, or a hand-rolled equivalent). There is **no** code path that discards that state and recomputes from scratch, except the cases excluded below. |
| `incremental_with_fallback` | incremental with a full-replan fallback | As `incremental_only`, plus **at least one** code path that throws the search state away and recomputes from scratch, triggered by something other than the excluded cases: no path found, inconsistency detected, stuck, a repair budget exceeded, a timer. |
| `no_global_planner` | no global planner | Nothing computes a route or cost-to-go to `obs.goal` over a map. Includes pure reactive and pure goal-bearing steering, even if a map is built. |

Excluded from "fallback" (they do not move an `incremental_only` file to
`incremental_with_fallback`): initialisation in `reset()`, and re-initialising
the search **because `obs.goal` switched** between legs. Both are a new search,
not a recovery from a failed repair. Record a goal-switch reinitialisation in
`notes` as `reinit_on_goal_switch`.

A resumable search that is spread over several steps but **restarts from
scratch** each time it completes is `full_periodic`, not incremental: reuse of
the *previous* search's values after a cost change is what makes it incremental.

If a file holds both an incremental and a full planner and either can drive,
code by which runs on the **nominal** path (no stuck flag, path found): the
incremental one with the full one reachable → `incremental_with_fallback`.

## 2. Map

| code | category (§4.2) | decision rule |
|---|---|---|
| `persistent_grid` | persistent occupancy or log-odds grid | A grid (array or dict keyed by cell) of occupancy, hit counts, log-odds or cost, written from lidar and **kept on the instance across steps**. |
| `none` | none | No such grid, or one rebuilt from the current scan only. |

If the grid is cleared when `obs.goal` switches, it is still `persistent_grid`;
note `grid_cleared_on_goal_switch`. A non-grid memory (a list of blocked edges,
a visited-cell set with no occupancy) is `none`, with the memory named in
`notes`.

## 3. Velocity selection

What produces the `Command` on the **nominal** path (not in a stuck or recovery
mode).

| code | category (§4.2) | decision rule |
|---|---|---|
| `rollout_scored` | rollout-scored | Several candidate twists `(v, w)` (or `(vx, vy, wz)`) are **forward-simulated** over a horizon and the best-scoring one is sent (DWA, trajectory sampling). Scoring against a global cost field still counts here. |
| `path_following` | pure pursuit or backpointer following | The command is computed **geometrically** from a path, a waypoint, backpointers, or the gradient / steepest descent of a cost-to-go field: pure pursuit, a heading P-controller toward a lookahead point, carrot following. A single safety slow-down on top does not change this. |
| `reactive` | reactive | The command is computed from the **current scan and goal bearing** alone, with no path or field consulted: gap following, potential fields, vector field histograms. |

A file that steers by path following and then picks among a handful of heading
offsets **without** forward-simulating them is `path_following`. Forward
simulation over a horizon is the line.

## 4. Stuck handling

| code | category (§4.2) | decision rule |
|---|---|---|
| `explicit_recovery` | explicit recovery | A dedicated recovery behaviour with its own commands: back up, rotate in place, an escape state or mode in a state machine. |
| `timeout` | timeout | Lack of progress is detected (timer, progress window, contact counter) and the reaction is a **planning-level** change only: force a replan, mark cells blocked, inflate costs, drop a waypoint. No dedicated manoeuvre. |
| `none` | none | No progress / stuck detection. |

Precedence when several are present: `explicit_recovery` > `timeout` > `none`.

## 5. D\* faithfulness checklist

Coded for **every file that claims D\***, whatever its arm. `claims_dstar` =
`yes` if any identifier or comment in the blinded file names D\*, D-star,
Stentz, D\* Lite, LPA\*, Focused D\*, or uses the D\* state vocabulary (`RAISE`,
`LOWER`, `PROCESS-STATE`, `MODIFY-COST`, `rhs`, `k_m`). Otherwise `no`, and every
checklist item is `na`.

`dstar_variant`: `stentz1994` (h / k / RAISE / LOWER), `dstar_lite` (g / rhs /
two-part key), `other` (named but neither), `na`.

Each item is `yes` / `partial` / `no` / `na`. Code the Stentz-1994 item where the
file follows the paper; for a D\* Lite file, code the **functional analogue** in
the second column and say so in `notes`.

| column | item (§4.2) | Stentz 1994 | D\* Lite analogue |
|---|---|---|---|
| `ds_open_keyed_on_k` | OPEN list keyed on `k` | priority queue ordered by `k(X)` = min of `h` over its time on OPEN | queue ordered by `[min(g,rhs)+h+k_m, min(g,rhs)]` |
| `ds_h_and_k` | `h` and `k` values maintained | both stored per state, `k` updated on (re)insertion | `g` and `rhs` stored per state |
| `ds_raise_lower` | RAISE and LOWER states handled | `PROCESS-STATE` distinguishes `k < h` (RAISE) from `k = h` (LOWER) and propagates each | over-consistent (`g > rhs`) and under-consistent (`g < rhs`) handled separately |
| `ds_modify_cost` | `MODIFY-COST` on sensed changes | a sensed cost change re-inserts the affected state(s) on OPEN | `UpdateVertex` on the endpoints of each changed edge, `k_m` updated |
| `ds_stop_rule` | the `k_min ≥ h(robot)` stopping rule | repair runs until `k_min ≥ h(robot state)` (or OPEN empty) | `ComputeShortestPath` runs while `top key < key(start)` or `rhs(start) ≠ g(start)` |
| `ds_backpointers` | consistent backpointers after repair | `b(X)` pointers updated in `PROCESS-STATE` and followed to drive | next cell = argmin over successors of `c + g`, recomputed after repair |

`partial` means the item is present but altered in a way that matters to it: a
budget that can stop repair before the stopping rule holds, a queue keyed on `h`
instead of `k`, RAISE / LOWER merged into one branch. Say which in `notes`.

## Derived numbers this rubric feeds (§4.3)

- **E10:** arm0 controllers with *any* incremental-repair machinery =
  `planner_type ∈ {incremental_only, incremental_with_fallback}`.
- **Association test (exploratory):** physics success for `incremental_only`
  against every planner type with a full-replan path (`full_every_step`,
  `full_periodic`, `incremental_with_fallback`). `no_global_planner` is excluded
  from both sides.

## Amendments

*(dated appends only)*
