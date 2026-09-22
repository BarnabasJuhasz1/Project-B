# What the growth numbers mean, and which worlds they were measured on

> ⚠️ **THE MEASURED UNIT CHANGED ON 2026-09-21.** `measure_growth.py` pairs
> `Layer[ports, BASE, node]` against the base; the live experiment pairs **a whole edited
> controller** against the same base, on the same 20 worlds, through
> `scripts/measure_controller.py` (`plans/todo/61-paired-measurement.md`).
>
> **Everything below still applies, and four rules carry over unchanged and are not optional:**
> 1. **The full metric set**, never `goal_reached` alone — 6 of 16 nodes predicted `collisions`.
> 2. **Each card's own claimed metric is marked.**
> 3. ⚠️ **Every COUNT metric is reported PER METRE**, beside the raw count, with a flag when the
>    two disagree. A robot that stops scores perfectly on collisions, and that already produced
>    an apparent win whose sign reversed.
> 4. ⚠️ **The drift axis is switched ON where a card names it.** `to_scenario()` never set
>    `drift`; epic 06 fixed it and `out/growth/maze+drift0.10.csv` is the result.
>
> ⚠️ **Growth CSVs are AGGREGATES** — `fixed = 2` never says *which* two worlds — and **nothing
> here is a benchmark number**: these are kinematic rollouts with the frozen locomotion policy
> absent.

Written 2026-09-15. Covers every number in `out/growth/*.csv` and the tables in
`generate/FINDINGS.md` from 2026-09-14 onward.

⚠️ **None of this is a benchmark number.** `scripts/measure_growth.py` is a *kinematic*
rollout (`core/tests/kinematic.py`), not Isaac Sim — the frozen locomotion policy is absent.
It is triage: which nodes fire, which pairs are comparable, what is worth spending physics on.
Every reportable number comes from `nvcr.io/nvidia/isaac-sim:6.0.1`, per §6 of the description.

---

## 1. The six growth columns

`out/growth/<tier>.csv`, one row per node, written by `scripts/measure_growth.py`. Every row
compares **`Layer[ContractPorts, BASE, node]`** against **the base alone**, paired on the same
worlds. A `BASE_ALONE` row gives the reference.

| column | meaning | read it for |
|---|---|---|
| `rate` | fraction of worlds where the **pair** reached the goal | the headline |
| `delta` | `rate − base_rate` | net effect, and it hides as much as it shows |
| `fixed` | worlds where **the base failed and the pair succeeded** | the number that matters — it is what "this node helps" means |
| `broke` | the reverse: base succeeded, pair failed | the cost side |
| `fire` | fraction of **ticks** whose `precondition()` returned True | did it ever run? |
| `changed` | fraction of ticks where the command **actually differed** afterwards | did running it do anything? |
| `n` | worlds in the row (20) | — |

**`fixed` and `broke` are not derivable from `delta`.** A node at `delta = 0` may have fixed
five worlds and broken five. Read those two columns, never the delta alone.

**`fire` and `changed` separate the two null results.** A node with `fire` high and `changed`
zero is running every tick and doing nothing — a cap whose bound is never the binding one.
That looks identical to a node that never fires if you only read `rate`. `changed` is possible
only because a declining node provably leaves the base's command byte-identical
(`nodes/contract.py`, chunk 52), which is also what makes `fixed`/`broke` attributable at all.

`fire` and `changed` are per **tick**, not per world. `rate`, `fixed`, `broke` are per world.

A row may instead carry `UNSATISFIED_PORT` in the `n` column: the node reads a port only
another specialist publishes, so it cannot sit in a two-leaf tree. That is a fact about the
node, recorded rather than skipped.

### The four groups this produced

Over the 16 committed nodes, `fire`/`changed` came out bimodal with nothing between:

| pattern | nodes | outcome |
|---|---|---|
| never fires (`fire` 0-2%) | 4 | +0%, by definition |
| fires ~100%, `changed` ≈ 0 | 3 | +0% — runs, does nothing |
| fires ~100%, `changed` ~100% | 6 | **every one regresses**, −10% to −50% |
| fires 62%, changes 57% | 1 | **+10%, the only node that helps** |

**Selectivity is the variable**, and nothing in `generate/prompts/modifier.md` selects for it.

---

## 2. The full metric set — `--metrics`

Without `--metrics`, **only `goal_reached` is scored.** That was wrong for 6 of the 16 nodes,
whose cards predict `collisions` — the whole speed-limiter family, judged on a claim they never
made. `scripts/measure_growth.py --metrics` scores the real `core.metrics.score()` and writes
`out/growth/<name>-metrics.csv`, one row per (node × metric):

`tier, node_id, arm, metric, base, pair, delta, direction, is_own_claim, n_base, n_pair`

`is_own_claim` marks the metric **that node's own card predicted**, read off the staged
module's `PREDICTION`.

### The eight reported

| metric | direction | note |
|---|---|---|
| `goal_reached` | higher | **primary, always** |
| `collisions` | lower | `contact` is an analytic proxy — `clearance − ROBOT_RADIUS ≤ 0`, a 0.30 m circle round a 0.65 × 0.31 m body. No physics contact sensor is wired to the Go2. |
| `near_misses` | lower | |
| `min_clearance` | higher | |
| `stuck_events` | lower | |
| `time_to_goal` | lower | undefined on failures — conditioned on success, read next to the success rate |
| `path_length` | lower | |
| `spl` | higher | denominator is the omniscient taut path; near-meaningless for search families |

### The ten `score()` keys NOT reported here

`scenario`, `seed`, `termination`, `spl_uncapped`, `optimal_distance`, `recovery_time`,
`time_stuck`, `localisation_failures`, `duration`, `control_steps`.

`localisation_failures` is excluded **deliberately**: `cards.store.CONTROLLER_INDEPENDENT` says
no controller can move it, so a delta would be noise wearing a name.

⚠️ **Three of these are gaps, not decisions.** `PRE-REGISTRATION.md` §5 names `stuck_events`
*"count, with `time_stuck`"* and says `spl_uncapped` *"is reported alongside `spl`"* — and
`spl_uncapped > 1` is a **fault signal**, not rounding. `recovery_time` is half of a named
secondary too. None of the three is in `REPORTED_METRICS`. Growth rows are therefore not a
superset of the pre-registered secondaries.

### ⚠️ Count metrics are also reported per metre

`collisions`, `near_misses` and `stuck_events` are lower-is-better **counts**, so **a robot that
stops scores perfectly.** This is not hypothetical: one node cut collisions 0.40 → 0.30 and
looked like the library's second win. It had walked **6.2 m instead of 30.0** and reached the
goal in **0 of 20** worlds — per metre it collides **3.6× more** than the base and the win
reverses. Each is now printed per metre beside the raw count, flagged `<-- REVERSES the raw
count` when the two disagree.

Neither figure replaces the other: per-metre alone flatters a node that crawls safely and never
arrives. `goal_reached` stays primary.

---

## 3. Which environments, exactly

A world is a **pure function of `(tier, seed)`** — `core.worldgen.generate(tier, seed)` — so
naming those two names the environment completely. Room is 12 × 8 m, same as lab42.

| run | file | tier | seeds | base |
|---|---|---|---|---|
| **growth round 2** (the A/B table) | `out/growth/maze.csv` | `maze` | **2000-2019** | 10/20 = 50% |
| the `--metrics` re-score | `out/growth/maze-metrics-run-metrics.csv` | `maze` | **2000-2019** | 10/20 = 50% |
| replication, fresh worlds | `out/growth/maze-replication-seed3000.csv` | `maze` | **3000-3019** | 8/20 = 40% |

Defaults: `--tier maze --n 20 --start-seed 2000 --time-limit 120`. Seeds are
`range(start_seed, start_seed + n)`, contiguous, no gaps. `cluttered` (base ~70%) and `open`
(~90%) exist but **have never been run against the base** — `maze` is the only calibrated tier.

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P -c "from core.worldgen import generate; w=generate('maze',2013); print(w.start,w.goal,w.difficulty)"
$P scripts/plot_worlds.py --n 5          # a contact sheet, one row per tier
```

### Per-world attribution — `<stem>-worlds.csv`

The summary CSV is an aggregate: `fixed = 2` says two worlds flipped, never **which two**. Since
2026-09-15 every run also writes `<stem>-worlds.csv`, **one row per (node × world)**:

`tier, seed, scenario_id, node_id, arm, base_reached, pair_reached, outcome, fire, changed,
ticks, geodesic, tortuosity, min_corridor, claim_metric, base_claim, pair_claim`

| column | note |
|---|---|
| `outcome` | **`fixed` / `broke` / `both_ok` / `both_fail`** — the four cells of the paired 2×2, named rather than left to be recomputed from two booleans |
| `fire`, `changed` | **per world**, not pooled. The aggregate hides real spread: one node's fire rate ranges 0.33-0.82 across worlds while its summary row reads 62%. |
| `ticks` | the rollout's length. **`ticks` at the cap means the episode timed out** (120 s × 50 Hz = 6000). |
| `geodesic`, `tortuosity`, `min_corridor` | that world's **measured** difficulty, carried inline so a row reads as "fails on *that* geometry" with nothing to regenerate |
| `claim_metric` | the metric this node's own card predicted |
| `base_claim`, `pair_claim` | its per-world value — **filled only with `--metrics`**. Empty means not scored, or genuinely `None` (e.g. `time_to_goal` on a failure): empty is never 0, because "did not arrive" is not "arrived instantly". |

The `BASE_ALONE` rows come first, one per world, so the base is inspectable on its own.

Rows are **buffered per node and flushed after it completes**, so a node that raises partway
writes no rows and this file never disagrees with the summary about which nodes were measured.

```bash
# which worlds did this node fix, and what do they look like?
awk -F, '$4=="gen_d_88c49cee31e01477" && $8=="fixed"' out/growth/maze-worlds.csv
```

### The 20 maze worlds, seeds 2000-2019

Difficulty is **measured by `core.geometry`, never asserted**. `corridor m` is the bottleneck
radius — the largest robot that still fits; the Go2 proxy is a 0.30 m circle, so **every one of
these worlds has a bottleneck tighter than the robot's own radius**, which is what makes the
tier hard. `tortuosity` is geodesic ÷ euclidean; 1.0 is a clear run.

| seed | start | goal | geodesic m | tortuosity | free | corridor m | obstacles |
|---|---|---|---|---|---|---|---|
| **2000** | (9.2, 4.7) | (3.1, 3.9) | 11.92 | 1.94 | 0.56 | 0.44 | 16 |
| **2001** | (7.3, 7.3) | (1.2, 7.0) | 17.95 | 2.94 | 0.50 | 0.42 | 20 |
| **2002** | (8.7, 6.4) | (1.8, 2.5) | 20.65 | 2.60 | 0.54 | 0.42 | 19 |
| **2003** | (2.9, 1.2) | (5.4, 6.9) | 9.28 | 1.50 | 0.44 | 0.35 | 33 |
| **2004** | (9.6, 1.7) | (2.6, 5.9) | 17.72 | 2.18 | 0.55 | 0.42 | 18 |
| **2005** | (4.7, 1.6) | (10.4, 5.2) | 9.76 | 1.45 | 0.59 | 0.35 | 12 |
| **2006** | (0.9, 1.0) | (3.2, 6.9) | 11.68 | 1.84 | 0.49 | 0.42 | 22 |
| **2007** | (1.3, 0.7) | (1.9, 6.7) | 8.08 | 1.34 | 0.52 | 0.42 | 23 |
| **2008** | (1.0, 7.1) | (1.4, 1.0) | 13.53 | 2.23 | 0.56 | 0.43 | 18 |
| **2009** | (6.8, 6.5) | (4.1, 1.1) | 11.23 | 1.85 | 0.45 | 0.42 | 28 |
| **2010** | (10.1, 7.3) | (3.7, 6.2) | 16.27 | 2.49 | 0.51 | 0.42 | 17 |
| **2011** | (9.4, 1.7) | (4.7, 5.7) | 10.62 | 1.70 | 0.43 | 0.41 | 26 |
| **2012** | (9.9, 6.9) | (3.9, 6.7) | 12.32 | 2.05 | 0.60 | 0.54 | 14 |
| **2013** | (8.0, 6.7) | (5.5, 1.1) | 13.92 | 2.26 | 0.50 | 0.38 | 23 |
| **2014** | (7.5, 5.6) | (1.6, 1.9) | 10.01 | 1.43 | 0.45 | 0.32 | 27 |
| **2015** | (2.2, 7.2) | (1.3, 1.2) | 10.15 | 1.67 | 0.48 | 0.44 | 29 |
| **2016** | (4.5, 3.4) | (11.0, 1.2) | 18.53 | 2.71 | 0.59 | 0.42 | 12 |
| **2017** | (5.0, 4.0) | (10.9, 5.1) | 15.45 | 2.57 | 0.51 | 0.31 | 22 |
| **2018** | (7.3, 3.3) | (2.3, 7.0) | 11.52 | 1.85 | 0.46 | 0.43 | 25 |
| **2019** | (1.3, 7.1) | (11.0, 6.5) | 13.52 | 1.39 | 0.50 | 0.42 | 20 |

⚠️ **Tiers order on density and tightness, NOT on tortuosity**, and the tier distributions
overlap (`core/worldgen.py`). A high-tortuosity `cluttered` world can be harder than a low one
here.

---

## 4. Where the numbers and the nodes live

| what | where |
|---|---|
| the growth tables | `out/growth/<tier>.csv` (summary), `-worlds.csv` (per node × world), `-metrics.csv` (per node × metric) — ⚠️ **gitignored**, not committed |
| the narrative + findings | `generate/FINDINGS.md`, entries from 2026-09-14 |
| the nodes themselves | `generate/staging/<arm>__<card>__<hash>.py` |
| the admission record | same stem, `.json` (task, card_id, `pipeline_version`) and `.result.json` |
| the card each was written from | `generate/cards/<arm>/<stem>.yaml` |
| the shared base they layer onto | `generate/base/base_navigator.py` — frozen |
| the one node prompt, identical per arm | `generate/prompts/modifier.md` |

```bash
$P scripts/measure_growth.py --tier maze --n 20              # goal_reached only
$P scripts/measure_growth.py --tier maze --n 20 --metrics    # + the full set, ~22 s/world once
```

`--metrics` pays one ~22 s visibility-graph solve per world for the SPL denominator, **once**:
`core.geometry` caches it on the world and every later controller reuses it.

---

## 5. The A/B navigation grid these numbers describe

Both arms have exactly **five `navigate_to_goal` nodes, paired one per card**, all on maze
seeds 2000-2019 against a base at 10/20. (B's other five nodes are `search_for_goal`; the stem
carries the **card**, not the task, so filter on the JSON's `task` field.)

| card | arm | class | rate | delta | broke | fire | changed |
|---|---|---|---|---|---|---|---|
| `limit_speed_dwa` | A | `SelectVelocityByDynamicWindow` | 0.00 | −0.50 | 10 | 1.000 | 1.000 |
| `limit_speed_dwa` | B | `SelectDynamicWindowVelocity` | 0.00 | −0.50 | 10 | 0.948 | 0.948 |
| `limit_speed_ics` | A | `FilterInevitableCollisionStates` | 0.40 | −0.10 | 2 | 0.916 | 0.018 |
| `limit_speed_ics` | B | `LimitSpeedToStoppableSeenDistance` | 0.50 | +0.00 | 0 | 0.997 | 0.004 |
| `path_integration_ants` | A | `SteerByPathIntegratedGoalVector` | 0.00 | −0.50 | 10 | 1.000 | 1.000 |
| `path_integration_ants` | B | `SteerByPathIntegratedGoalVector` | 0.00 | −0.50 | 10 | 0.971 | 0.971 |
| `path_integration_mammals` | A | `SpiralSearchGoalUncertaintyDisc` | 0.50 | +0.00 | 0 | 0.000 | 0.000 |
| `path_integration_mammals` | B | `SpiralSearchAtBelievedGoal` | 0.50 | +0.00 | 0 | 0.020 | 0.019 |
| `reorient_geometry` | A | `ReorientToBoundaryGeometry` | 0.50 | +0.00 | 0 | 1.000 | 0.000 |
| `reorient_geometry` | B | `ReanchorGoalToVisibleShape` | 0.50 | +0.00 | 0 | 1.000 | 0.000 |

**`fixed` is 0 for all ten. B−A is exactly zero on every column.**
