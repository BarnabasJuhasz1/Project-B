# 54 — The verdict taxonomy as code

**Epic:** [06](../06-epic-repair-loop.md) · **Depends on:** — · **Model calls:** no · **Needs Isaac Sim:** no

⚠️ Read [`../../REPAIR-LOOP.md`](../../REPAIR-LOOP.md) first. This chunk implements the taxonomy
defined there.

---

## Why this is first

Every other chunk in the epic either feeds this rule or acts on its output. Built later, the
taxonomy ends up encoded implicitly in the loop's control flow, where it cannot be unit-tested,
cannot be reported, and cannot be changed without changing the loop.

**Same discipline as `core/stats.py:decide()`** — the function IS the rule and the prose in
`REPAIR-LOOP.md` is a description of it.

## Deliverable

`repair/verdict.py`, sim-free, no model calls:

```python
classify(node_id, rows, card, base_rows) -> Verdict
```

`Verdict` carries: the label, the reason in one sentence, **`n`**, the claimed metric and its
measured delta, and the per-world evidence that drove it.

## The rule, with precedence fixed in advance

⚠️ **Precedence is declared here, before the first run.** Several of these conditions can hold
at once, and a precedence chosen after seeing a tie is a story about whoever ran the loop —
exactly the argument chunk 44 makes about its own tie-break.

1. **INERT** — `fire` below a floor, **or** `changed` ≈ 0 across every world. Checked first
   because a node that never acted cannot have caused anything, and every downstream comparison
   on it is noise.
2. **DEGENERATE** — ∃ a world where the base reached the goal, the augmented tree did not, and
   the failure is *degenerate*: net displacement far below the base's, no contact, and time
   spent stuck above a floor. The robot stopped doing the task without hitting anything.
3. **HELPED** — the node's **own claimed metric** improved by at least the card's own
   `min_effect`, in the claimed direction, **and task integrity is preserved**.
4. **MISJUDGED** — the *scored* metric is worse, but the **claimed** metric improved past
   `min_effect` and task integrity is preserved. The collision-avoidance-node case.
5. **REGRESSED** — worse, with no claim in the card covering it.
6. **ACCEPT** — none of the above; valid, acted, changed nothing measurable either way.

`UNTESTABLE` and `NOT_TRANSFERABLE` are **not produced here** — they are loop-level terminal
states (chunk 58), because both are statements about a *search* having been exhausted, not about
one measurement.

## Two rules that carry most of the weight

**Count metrics are compared per metre.** `collisions`, `near_misses` and `stuck_events` are
lower-is-better counts, and a robot that stops scores perfectly on all three. This is not
hypothetical: growth round 2 produced a node that cut collisions 0.40 → 0.30 while walking 6.2 m
instead of 30.0 and reaching the goal in **0 of 20 worlds** — per metre it collided **3.6× more**
and the win reversed. `METRICS.md` already carries the rule; `classify()` applies it, and the
raw count never decides a verdict on its own.

**Task integrity is a precondition on HELPED, not a bonus.** Defined mechanically: the augmented
tree's goal-reach rate has not fallen relative to the base, **and** its net displacement is
within a declared fraction of the base's. Without this, "improved its claimed metric" and "the
robot stopped" are the same row.

## ⚠️ The taxonomy's hard edge, stated rather than hidden

The user's motivating example — *the goal is effectively unreachable without a collision, the
base reaches it anyway and wrecks the robot, and the node correctly refuses* — **cannot be
classified mechanically in this version.** Deciding it needs to know that the goal was
unreachable safely, which nothing computes.

The core version does not guess. Such a case lands as `MISJUDGED` and the **reason string says
which check could not be made**, so the diagnosis call (chunk 57) is handed the ambiguity
explicitly rather than inheriting a confident wrong label. Mechanising it is a follow-up and
probably wants a safe-corridor computation in `core/geometry.py`.

## ⚠️ What this chunk must not do

Take a model call. `classify()` is a pure function over measurements. The moment a diagnosis
string can influence a verdict, the taxonomy stops being falsifiable and the loop is marking its
own homework.

## Done when

- `classify()` exists, is pure, and is imported by nothing that takes a model call.
- Replaying the **committed** `out/growth/maze-worlds.csv` rows reproduces a verdict for every
  one of the 16 measured nodes, and the distribution is written into the Outcome.
- `gen_b_44d12df4d25b437f` classifies as **DEGENERATE** — base reached, pair did not, 4 cm of
  net displacement over the last 52 s, zero contacts. If the rule does not catch the one case we
  have fully diagnosed, the rule is wrong.
- The per-metre rule is exercised by a test built from the round-2 node that "won" by stopping.
- Precedence is covered by a test per branch, including at least one row where two conditions
  hold at once.
- Every `Verdict` carries `n`.

---

## Outcome

**Built 2026-09-16.** `repair/verdict.py` (the rule + loaders + a CLI),
`repair/tests/test_verdict.py` — **28 tests, 0.15 s**, no simulator, no model call.

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P -m repair.verdict --tier maze                          # all 16 committed nodes
$P -m repair.verdict --node gen_b_44d12df4d25b437f        # one
```

### The committed table, classified

| Verdict | n | Nodes |
|---|---|---|
| **INERT** | 8 | 4 never fire, 3 fire ~100% and change 0.00%, 1 fires 3.9% |
| **DEGENERATE** | 3 | `gen_b_44d12df4d25b437f`, `gen_b_c2c95bcb965e342e`, `gen_a_d17f79d4734acb2d` |
| **REGRESSED** | 3 | |
| **MISJUDGED** | 1 | `gen_b_c380e7a8cfb32b78` |
| **ACCEPT** | 1 | `gen_d_88c49cee31e01477` |
| **HELPED** | **0** | — |

⚠️ **Zero HELPED across sixteen nodes.** Pinned by a test so a change to the rule has to be an
argued change rather than a silent one.

### What differed from the plan

**1. ⚠️ `min_effect` is RELATIVE, not absolute — the plan assumed absolute and the first
implementation was wrong.** `harness/unit.py:_effect()` has been the project's definition all
along: absolute difference for a rate metric (`goal_reached`), and `(baseline − candidate)/|baseline|`
for every other. Four of the sixteen nodes claim `collisions` with `min_effect` 0.3–0.5, which
reads as an impossible demand against a base that collides 0.40 times per episode, and as an
ordinary one once read as "cut them by 30–50%".

The wrong version **produced a wrong verdict**: `gen_b_c380e7a8cfb32b78` came out REGRESSED and is
MISJUDGED. Fixed by delegating to the existing function rather than writing a second one — which
needed a one-line public alias, `harness/unit.py: effect = _effect`. **The only edit outside this
chunk**, non-behavioural, no caller moved.

**2. The floors are NOT the harness's floors, deliberately.** `MIN_EFFECT_RATE` is 0.5, calibrated
for a micro-world whose baseline was built to fail outright. On a shared suite where the base
already reaches 50% the same number means "solve every remaining world". Chunk 37/44 scrapped that
floor as an admission criterion, and importing it here would reinstate it in a context it was never
calibrated for. `FLOOR_RATE = FLOOR_REL = 0.10`, and a card may demand more, never less.

**3. DEGENERATE is a whole-node judgement with one per-world precondition, not the per-world test
the plan described.** `<tier>-worlds.csv` carries no path length, so "lost a world the base won" is
per-world (`broke > 0`) while "did not crash" and "shows the stopped signature" are read from the
aggregate. Said out loud in the docstring rather than dressed up; the reason string names the seeds.

### What was discovered

**1. ⚠️ The project's only positive node does not clear its own card's floor.** `gen_d_88c49cee31e01477`
moved `goal_reached` +0.10 on seeds 2000–2019 against a stated `min_effect` of **0.15** → **ACCEPT,
not HELPED**. On the replication set (seeds 3000–3019) it moved +0.20 → **HELPED**. Both are real
committed measurements at n = 20. **The verdict flips between two world sets of twenty**, which is
`n` biting exactly where CLAUDE.md warns it does. Pinned as a test.

This bears on CLAUDE.md's description of D as "the one hit" — it is a hit on one world set and not
on the other, and it has never been read against its own prediction until now. ⚠️ Recorded only;
**no work was done on D and none is proposed** — the arm is frozen and this is a reading of an
existing committed row.

**2. ⚠️ An A-arm node claims `collisions` and makes them 13× worse.** `gen_a_fac8c7b94fe6b1c6`:
0.40 → **5.30** collisions while travelling 4.99 m against the base's 29.96 m — **79× worse per
metre**. Checked specifically because −78.56 looked like a small-denominator artifact; it is not.

**3. METRICS.md's per-metre case is caught before per-metre is needed.** `gen_b_c2c95bcb965e342e`
is the node that cut collisions 0.40 → 0.30 and "looked like the library's second win" — it walked
6.2 m instead of 30.0. DEGENERATE fires on it at precedence 2, before the claim is ever evaluated.
The per-metre arithmetic is still tested independently and reproduces METRICS.md's 3.6× figure.

**4. The taxonomy earns its keep on real data.** One genuine MISJUDGED exists:
`gen_b_c380e7a8cfb32b78` took collisions to **zero** while `goal_reached` fell 0.35. Under a
goal-reach-only reading it is a regression; it is a speed limiter being judged on a claim it never
made. That is the distinction this chunk was built for, and it was not visible in any existing table.

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| Known-good case classifies DEGENERATE | `gen_b_44d12df4d25b437f`, reason names seed 2004 | **confirmed**, n=20 |
| All 16 committed nodes classify, none raises | 16/16, distribution above | **confirmed**, n=20 each |
| `min_effect` semantics | **relative** for all but `goal_reached`; first implementation wrong | **corrected** — one verdict flipped |
| METRICS.md per-metre reversal reproduces | ratio 3.57 ∈ (3.3, 3.9) | **confirmed** |
| −78.56 outlier is not a divide-by-near-zero artifact | collisions 0.40→5.30, path 29.96→4.99 | **ruled out** |
| `harness/unit.py` alias is non-behavioural | full suite: **930 passed**, 214 s | **confirmed** |
| The one suite failure is not mine | `compose/tests/test_no_papers.py::test_no_card_vocabulary_reaches_the_rendered_node_block` reproduces with `harness/unit.py` restored to HEAD | **ruled out — PRE-EXISTING** |
| `RATE_REGIME` agrees with `harness.unit.RATE_METRICS` | equal | **confirmed** |
| `repair/` is sim-free and cannot reach the sealed split | AST guard over every file | **confirmed** |
| A `Verdict` carries no arm tell | scanned for condition letters, citation, author names | **confirmed** |
| ⚠️ **MISJUDGED is threshold-sensitive** | `gen_b_c380e7a8cfb32b78` stuck ratio **1.420** vs ceiling **1.50** — a 5% margin. At 1.4 it becomes REGRESSED | **inconclusive — flagged** |

⚠️ **The sensitivity above is the weakest part of this chunk.** The single MISJUDGED verdict in the
committed table depends on `INTEGRITY_STUCK_RATIO` being 1.5 rather than 1.4. The constant was
chosen before the data was seen, which is the right order, but it decides a verdict on a 5% margin
and it should be treated as pre-registration material when the loop is first used across arms.

### Not checked

- **No verdict has been validated against physics.** Every number here comes from kinematic
  rollouts, which `measure_growth.py` says is not a benchmark path. Kinematic-vs-physics agreement
  was measured excellent on `goal_reached` (6/6) and poor on path shape for FAILING runs — and
  `path_length` is an input to both the integrity test and the degeneracy test. ⚠️ **This is the
  single largest open risk in the rule** and it wants a physics spot-check on one DEGENERATE case.
- **The HELPED branch is not exercised by any real committed measurement**, because none exists.
  It is tested synthetically and via the D replication reconstruction.
- `UNTESTABLE` and `NOT_TRANSFERABLE` are absent by design — chunk 58 owns them.
- Only the `maze` tier has been classified; `open` and `cluttered` have no committed growth table.

### ⚠️ One pre-existing failure in the suite, NOT caused by this chunk

```
FAILED compose/tests/test_no_papers.py::test_no_card_vocabulary_reaches_the_rendered_node_block
AssertionError: reorient_geometry: ['region'] appear in the rendered node block
                and in the card, and in no node's public contract
```

**930 passed, 1 failed, 214 s.** Verified pre-existing by restoring `harness/unit.py` to HEAD and
re-running: it fails identically. The test imports only from `compose/`, `core/` and `nodes/` —
nothing this chunk touches.

It is a **real finding about non-negotiable #6** ("composition sees only port contracts and a task
specification, never the papers"): the word *region* reaches the rendered node block from the
`reorient_geometry` card without appearing in any node's public contract. Left alone — diagnosing
it is not this chunk's job and silencing it would be worse — but it should not be left unowned.
Not filed as a chunk, because everything is on hold; recorded here so it is not rediscovered.

### A note on `git status`

`plans/doing/33-…` and `plans/doing/43-…` show as **deletions** in `git diff --stat` because they
were moved to the untracked `plans/hold/`. Both files are intact. The move becomes a rename on the
first commit that includes `plans/hold/`.
