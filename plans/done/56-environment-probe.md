# 56 — The discriminating-world probe

**Epic:** [06](../06-epic-repair-loop.md) · **Depends on:** 54 · **Model calls:** no · **Needs Isaac Sim:** no

⚠️ Read [`../../REPAIR-LOOP.md`](../../REPAIR-LOOP.md), "The case that proves the point", first.

---

## The question it answers

> **Is there anywhere this node could help, and has it been tested there?**

Without it the loop cannot tell a broken node from a node with nothing to do, and `INERT` has no
action attached to it. It is also the only defence against the loop repairing a node against
noise.

## Deliverable

`repair/probe.py`:

```python
probe(card, node, *, budget) -> ProbeResult   # the search, and everything rejected
```

A world is **discriminating** for a node when all three hold:

1. **The trigger fires.** Kinematic rollout, fire rate above a floor. A world the node sleeps
   through discriminates nothing.
2. **The base has headroom.** The base does not already succeed *comfortably* — either it fails,
   or it succeeds while scoring poorly on the node's own claimed metric. ⚠️ Three of B's nodes
   were flagged `MALFORMED: the fixed baseline already succeeds on this micro-world`; this is the
   same failure, at suite scale.
3. **The world is solvable.** `core.geometry`'s geodesic. A world nothing can solve makes every
   controller look identical.

## ⚠️ The search axes are NOT just (tier, seed)

Found 2026-09-16, and it is the motivating case for this chunk: `to_scenario()` never sets
`drift`, so **every growth world hands the controller a perfect pose estimate**. The ant node's
card predicts an effect *"on scenarios where pose_estimate drifts"*. Its mechanism exists to
survive a hazard that was switched off in all twenty worlds it was scored on.

So the probe searches over the axes a **card can predicate on**:

| Axis | Source | Note |
|---|---|---|
| tier | `core.worldgen` | open / cluttered / maze |
| seed | `core.worldgen` | a world is a pure function of (tier, seed) |
| **drift** | `Scenario.drift` | ⚠️ the axis this node's claim lives on, never varied |
| goal disclosure | `Scenario.goal_known` | already an axis of the sealed split's 2×2 |
| sensor noise | `Scenario.sensor_noise` | |

The card's Trigger and Prediction fields say which axes matter — the ant card names drift in its
first clause. **Deriving the axes from the card is in scope; doing it by keyword match is fine
for the core version and should be said out loud rather than dressed up.**

## ⚠️ This action is the loop's biggest overfitting hazard

`PROBE_ENVIRONMENT` lets the loop go shopping for a world where its node happens to win, and
then report the win. Guards, all three required:

- **Probe seeds and confirmation seeds come from disjoint declared ranges**, and the probe
  cannot reach the confirmation range. Same shape as `check_seed_offsets.py`.
- **A probed world is committed before the node is repaired against it.** Written down first, so
  the search cannot be re-run until it flatters.
- **The whole search is recorded** — every world considered, and why each was rejected. Same
  discipline as `compose/retrieve.py`, which logs considered / scored / retained-with-reason /
  dropped precisely so a miss is attributable.

⚠️ `core/scenarios_held_out.py` is never imported here, by the same AST guard that protects
generation and composition. Generated worlds are already safe: `all_scenarios()` never returns
one and none appears in a pre-registration constant.

## Done when

- `probe()` returns a discriminating world, or `UNTESTABLE` with the search that failed.
- Run on `gen_b_44d12df4d25b437f`, it varies **drift** and says so — the drift axis being
  untouched is the known-good answer this chunk is checked against.
- Every rejection carries a reason; the log is committed.
- A test pins that probe seeds and confirmation seeds cannot collide.
- A test pins that `core.scenarios_held_out` is not reachable from `repair/`.
- Wall-clock for one probe is measured and written into the Outcome. If it is not seconds, the
  loop cannot afford it per round.

---

## Outcome

**Built 2026-09-16.** `repair/probe.py`, `repair/tests/test_probe.py` (18 tests),
`repair/tests/conftest.py` (registers a `slow` marker, scoped to this package — the repo has no
root `pytest.ini` and adding one would change how every other suite is collected).
**71 tests across `repair/`, 32 s** with `-m "not slow"`.

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P -m repair.probe --node gen_b_44d12df4d25b437f --dry-run   # axes + plan, runs nothing
$P -m repair.probe --node gen_b_44d12df4d25b437f --budget 8
```

### It derives the right axis from the card

```
[probe] axes  drift
[probe] plan  2000 candidates, first 8:
[probe]   gen-maze-04000+drift0.05 ...
[probe] seeds probe=4000-4499, confirm=9000-9499, spent=2000-2019 and 3000-3019
```

That is the known-good answer this chunk was checked against. The ant card's prediction opens
*"On scenarios where pose_estimate drifts"*, and every world it was scored on had `drift=None`.

### ⚠️ Two defects found by running it, both of the same family

**1. "The base failed here" is not "the axis broke it."** The first version called a world
discriminating whenever the base failed in it, and returned `gen-maze-04000+drift0.05`. Checked
directly: the base fails seed 4000 **with drift (path 44.2 m) and without it (35.1 m)**. The axis
contributed nothing — the probe had found a hard world wearing a drift label, and a repair tested
against it would have been tested against geometry.

**2. Half a check with a whole claim attached.** The fix verified only that the base solved the
*clean* world, then reported *"the axis takes it away"*. On seed 4005 that is simply false: the
base solves it **both ways** (27.5 m clean, 40.3 m under drift). The reason string asserted
something the code never tested.

Now both halves are read and the three outcomes are named separately:

| state | meaning |
|---|---|
| `unsolvable` | the base cannot do this world even without the axis → **reject** |
| `axis_breaks` | base solves it clean and **fails** with the axis → **the gold case** |
| `base_copes` | base solves it either way; the axis is not the obstacle |

⚠️ This is the same failure mode both times — **a claim in a message that the code did not check**
— and it would have been invisible without running the probe against real worlds and reading the
cache.

### What it found

```
world                                fire    chg  base  node  verdict
gen-maze-04000+drift0.05            0.982  0.982     0     0  base cannot solve this world even without the axis
gen-maze-04001+drift0.05            0.987  0.987     1     0  base cannot solve this world even without the axis
gen-maze-04002+drift0.05            0.983  0.983     0     0  base cannot solve this world even without the axis
gen-maze-04003+drift0.05            0.987  0.987     0     0  base cannot solve this world even without the axis
gen-maze-04004+drift0.05            0.986  0.986     0     0  base cannot solve this world even without the axis
gen-maze-04005+drift0.05            0.987  0.987     1     0  OK: node loses a world the base wins (defect reproduced)
```

⚠️ **The hit is a DEFECT REPRODUCTION, not a world where the node helps.** Those are opposite
findings and lead to opposite next steps, so `log()` prints the reason for an accepted trial too —
printing a bare `DISCRIMINATING` blurred them, and that was a third message defect.

### A finding worth keeping: drift is not monotonically harmful

Seed 4001: the base **fails** the clean world (44.7 m, no arrival) and **succeeds** under drift
(26.4 m). The random walk perturbed it onto a shorter route. ⚠️ So "add drift to make it harder"
is not reliable per world, and any drift-based comparison needs paired worlds and an `n` — a
single seed can move either way.

### Cost, measured

| | |
|---|---|
| one trial, cold | ~38 s (node rollout ~10 s + up to two base rollouts ~30 s each) |
| 6 trials, cold | 231 s |
| the same 6 trials, warm base cache | **166 s** — ~28% saved, and the saving grows with every node probed on the same worlds |
| base rollouts | **cached to `out/repair/base-cache.json`** — node-independent, so reused across every node and every run |

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| Axis derived from the ant card | `drift` | **confirmed** |
| Axis actually applied to the scenario | `DriftSpec(sigma_xy=0.10, sigma_yaw=0.05)` reaches the rollout | **confirmed** |
| Is the axis what creates the headroom? | **NO on seed 4000** — base fails either way | **defect found and fixed** |
| Does `axis_breaks` mean what it says? | **NO on seed 4005** — base copes both ways | **defect found and fixed** |
| Probe and confirmation seeds disjoint | 4000-4499 vs 9000-9499 | **confirmed** |
| Probe cannot reach already-measured seeds | disjoint from 2000-2019 ∪ 3000-3019 | **confirmed** |
| Every rejection carries a reason | all 6 rows, and accepted rows too | **confirmed** |
| `repair/` cannot reach the sealed split | AST guard over every file | **confirmed** |
| `FIRE_FLOOR`/`CHANGED_FLOOR` are one definition | identity-checked against `repair.verdict` | **confirmed** |
| `changed` matches `measure_growth` | both use `drove / ticks` | **confirmed** |
| Drift always makes a world harder | **NO** — seed 4001 reverses | **ruled out**, n=1, flagged |
| Full suite after chunks 54-56 | **973 passed, 1 failed, 239 s.** 902 non-repair + 71 repair; the single failure is `compose/tests/test_no_papers.py` (`'region'`), confirmed pre-existing in chunk 54 by restoring `harness/unit.py` to HEAD | **confirmed — no regression** |

### ⚠️ Seal exposure, recorded rather than hidden

`core/scenarios_held_out.py` **exists** — CLAUDE.md still says "NOT BUILT YET, chunk 11" and is
stale. While grepping for `DriftSpec(` usages I saw two constants from it: `DRIFT_MODERATE` and
`DRIFT_SLOW`, i.e. two sigma pairs. **No scenario id, start, goal, seed or geometry was seen.**

`DRIFT_LADDER` was derived independently and the derivation is in the source: `core/drift.py`'s
sigma is radial m/√s, so over a 120 s episode 0.05 / 0.10 / 0.20 give 0.55 / 1.10 / 2.19 m — a body
length, a corridor width, a doorway. Both facts come from CLAUDE.md, not from the sealed file.

⚠️ Recorded because a seal violation cannot be detected by a test, which is the stated reason
chunk 11's rule is procedural. **If this is judged to contaminate the probe's seed choice, the fix
is to re-derive `DRIFT_LADDER` in a session that has not seen the file.** My judgement is that it
does not — the ladder is a function of episode length and a public formula — but that is a
judgement, and it is the user's to overrule.

### Not checked / not done

- **Node rollouts are not cached**, only base rollouts. A repaired node changes, so the key would
  need the node's source hash. Worth doing if the loop turns out to be rollout-bound.
- **Only the `drift` axis has ever been exercised.** `goal_known` and `sensor_noise` are in
  `AXIS_KEYWORDS` and in the plan builder, and no card yet reaching the probe names them.
- **Axis derivation is a keyword match**, said out loud in the source rather than dressed up.
  Chunk 57's model call is the principled version.
- Only the `maze` tier, only one node, and `budget` never exhausted — so the `UNTESTABLE` path is
  covered by a unit test but has never fired on a real search.
