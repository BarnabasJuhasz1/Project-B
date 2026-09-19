# 58 — The loop

**Epic:** [06](../06-epic-repair-loop.md) · **Depends on:** 54, 55, 56, 57 · **Model calls:** yes · **Needs Isaac Sim:** no

---

## The deliverable

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P scripts/repair_node.py --node gen_b_44d12df4d25b437f --rounds 3
$P scripts/repair_node.py --node gen_b_44d12df4d25b437f --dry-run   # the plan, calls nothing
```

`repair/loop.py` + `scripts/repair_node.py`. One node, one arm, one task, to a terminal verdict,
with every round on disk.

## One round

1. **Measure** — `measure_growth` on the current repair set. Kinematic.
2. **Classify** — `repair/verdict.py` (54). Mechanical. A rule, not a model.
3. **Terminal?** → stop and record.
4. **Assemble** — the blind bundle (55).
5. **Diagnose** — one model call (57). Proposes a cause and an action.
6. **Act** — `REGENERATE_NODE` (via `regenerate_modifiers.py`) or `PROBE_ENVIRONMENT` (56).
7. Round + 1, until the budget is spent.

## Terminal states — all five are data, none is an error

| Exit | Produced by |
|---|---|
| `HELPED` | 54, confirmed on the confirmation set at declared `n` |
| `ACCEPT` | 54 or 57 |
| `UNTESTABLE` | 56 exhausted its search |
| `NOT_TRANSFERABLE` | 57, after at least one `REGENERATE_NODE` round |
| `UNREPAIRED` | budget spent |

⚠️ **A loop that can only terminate on success will grind forever against a paper that does not
transfer, and report the grinding as progress.** `NOT_TRANSFERABLE` requires at least one real
repair attempt first, so it cannot be reached by giving up on round one.

## ⚠️ INERT is not an exit

The cheapest way to stop a node regressing is to make it decline everywhere. A loop rewarded for
"no longer regresses" finds this immediately, and every node converges on doing nothing while the
table fills with clean rows. `INERT` routes to `PROBE_ENVIRONMENT`; it never terminates the loop
on its own.

## ⚠️ Repair/confirmation separation

Repairing against the worlds you then score on is fitting the test set. Disjoint declared seed
ranges; the **confirmation set is looked at once**, when a terminal verdict is proposed. The
probe cannot reach the confirmation range (56).

## ⚠️ Oscillation

Every round records a **behaviour signature** — fire rate, changed rate, per-world outcome
vector. A repair whose signature matches an earlier round in the same lineage terminates the
lineage rather than spending the remaining budget rediscovering it.

## Provenance — the lineage is the record

⚠️ `plan_hash` is a pure function of (condition, task, card), so **every repair round produces
the identical stem**. Two rounds cannot share a row key, for exactly the reason chunk 52 moved v2
aside. Each round gets a `repair_round` and a `parent_node_id`, and superseded rounds are
archived, never overwritten — `generate/archive/README.md` is the pattern and it is already
carrying two superseded pipelines.

**Never retry a rejection.** `generate/batch.py` refuses to, because a pass rate needs a
denominator; the same applies to a repair round.

## Resumability

Resume from what is on disk, never from a progress file — `sweep.py` resumes from `results/`,
`batch.py` from `staging/`. ⚠️ And the rehearsal's own worst bug applies here too: **five
episodes exited 0, wrote nothing, and were recorded as `ok`.** Exit 0 is necessary; the result
file is sufficient.

## Not in scope

Physics as a gate (the loop is kinematic; physics confirms a terminal verdict through commands
that already exist), the library-wide sweep, arm A, `REGENERATE_CARD`, `REVISE_SHARED_PROMPT`.

## Done when

- The command above runs end to end on `gen_b_44d12df4d25b437f` and reaches a terminal verdict.
- `--dry-run` prints the plan and calls nothing.
- Every round is on disk: bundle, transcript, diagnosis, action, measurement, verdict.
- A killed run resumes without repeating a model call.
- ⚠️ **The Outcome records whether the loop independently found any of the three known defects**
  — the trigger latch, the `adjust`/gate arbitration conflict, the unchecked turn-symmetry
  condition. A clean termination that found none of them is a **failure of the epic**, and saying
  so in the Outcome is the point of having picked a fully-diagnosed node first.
- A **Checks** section: check → outcome → verdict, with `n` wherever the result is stochastic,
  and an explicit note of what was not checked.

---

## Outcome

**Done 2026-09-17.** The command runs end to end and reaches a terminal verdict. Four live runs
were needed to get one valid one, and the three failures are the substance of this Outcome.

### The run that counts

    $P scripts/repair_node.py --node gen_b_44d12df4d25b437f --rounds 3

| # | node | verdict | action | outcome |
|---|---|---|---|---|
| 1 | `…44d12df4d25b437f` | `DEGENERATE` | `REGENERATE_NODE` | → `__r01` |
| 2 | `…__r01` | **`ACCEPT`** | `REGENERATE_NODE` | → `__r02` |
| 3 | `…__r02` | `INERT` | `REGENERATE_NODE` | → `__r03` |

`EXIT: UNREPAIRED — budget of 3 round(s) spent.` 1380 s wall clock, three distinct behaviour
signatures (`0e5defb63c15`, `d5a337ea4464`, `e36a7a1c1c69`), every round on disk under
`out/repair/gen_b_44d12df4d25b437f/`.

### ⚠️ The epic's success criterion: MET, on round 1

The loop independently identified the arbitration conflict, in the detail the hand diagnosis of
2026-09-16 found it:

> *"The node deadlocks the base. Every armed tick it calls `ports.adjust(wz=wz)` with a turn rate
> computed purely from its own reconstructed bearing, never reading `nav_heading` or `nav_stuck`,
> so it is the last writer on wz and overrides the base's avoidance turn entirely (5625 of 6000
> ticks drove on seed 2004). But its only lever on translation is `ports.cap(vx=v_limit)`, which
> can lower the base's vx and never raise it. The base gates its own forward speed on being
> aligned with `nav_heading`; with wz overwritten the base can never reach that alignment..."*

Of the three known defects: **#2 (arbitration) found in full**; **#1 (the monotone trigger latch)
found in substance** — *"It is a self-locking loop"*, *"the 1.0 m arming gate is crossed almost
immediately and the node is effectively always on... without a single decline"* — and **#3 (the
unchecked turn-symmetry condition) found as the `drift: None` mismatch**, which is the stronger
form of the same point.

**And one nobody found by hand:** `_p_hat` uses `r = self._l * _UNIT`, where `l` counts *path
travelled*, so the reconstructed radius grows with arc length rather than net displacement —
*"in a maze with tortuosity 2.18 the phantom position runs off at roughly twice the true
displacement and the bearing inverts."*

The repair then narrowed `WRITES` to `("wz",)` and biased off `nav_heading` instead of mastering
it. **Round 1 `DEGENERATE` → round 2 `ACCEPT`: the loop removed the regression.**

### ⚠️ THE FIFTH LIVE FAILURE, AND THE ONLY ONE THAT NEVER RAISED

A complete 3-round run finished green, wrote every record, and exited `UNREPAIRED` — and every
verdict after round 1 was an artefact.

`entry_for` resolved a repair-round id by taking `entries[0]` from that round's library.
`ContractPorts` — the harness's own port shim, always present and exempt by design — sorts first.
**So the loop measured the shim instead of the repair, every round.** The shim does nothing:

    fire 0.0%  ->  INERT  ->  two rounds with the IDENTICAL signature
               ->  the oscillation guard  ->  EXIT: UNREPAIRED

Round 1 is unaffected — it measures the original node through the committed CSV, not through
`entry_for` — so the diagnosis quality looked right and masked the rest. Re-measured after the
fix, the round-1 repair fires on **25.5%** of ticks on seed 2004 and ends at 71.1 s; the run had
recorded it as 0.0% and `INERT`.

`entry_for` now filters the shim and **raises on ambiguity**. A resolver that guesses which entry
was meant will eventually guess wrong silently, which is exactly what happened.

⚠️ **A pre-existing test was passing only because of this bug.**
`test_a_repaired_node_is_not_looked_up_in_the_growth_csv` stubbed `load_round`, but `_measure`
calls `entry_for`; on a round directory that does not exist the old code returned the shim
instead of raising, so the missing stub never showed.

⚠️ The polluted round is archived, not deleted —
`out/repair/archive/shim-measured-2026-09-17/`, with a README. It is a repair written against a
diagnosis of the shim's behaviour.

### ⚠️ The lineage carries the LATEST node, never the BEST one

Round 2 reached `ACCEPT` and the loop spent its remaining budget moving away from it, ending on
`__r03` with `UNREPAIRED`. Nothing in `run()` remembers that `__r01` was the best node in the
lineage. **A budget can therefore end on a worse node than one the loop already had, and the
recorded exit will not say so.** Not fixed here: "keep the best" is a change to the terminal-state
rules, which are the part of this epic that is pre-registration material.

### The naming bug that killed the first full run

Round 2 is handed round 1's id, and `new_id = f"{node_id}__r{number:02d}"` compounded to
`…__r01__r02`. Round 3 then died in `entry_for` on `int('01__r02')`, after four model calls. It
also split one lineage across two top-level directories. Rounds are now named and staged off
`lineage_root()`; `entry_for` uses `rpartition` and refuses a non-digit suffix readably.

### Resumability — it was not implemented at all

The Done-when asks that a killed run resume without repeating a model call. `regenerate` called
the model unconditionally and `diagnose` had no resume path. Both now resume:

- `regenerate` returns an existing round when its `.result.json` is present — **that file and
  not the directory or the `.py`**, because it is written last. The sweep rehearsal's own worst
  bug applies here: five episodes exited 0, wrote nothing, and were recorded as `ok`.
- `diagnose` replays a transcript whose prompt hash already exists, and **does not rewrite it**.
  Writing it back moves the mtime and nothing else, so the file then claims a call was made when
  none was. Observed: one transcript's mtime moved three times on 2026-09-17 while its
  `session_id` and cost never changed.

Measured: with both in place, round 1 of a re-run takes **31.4 s** and spends nothing.

⚠️ This **mitigates but does not close** chunk 57's open follow-up. A genuinely new diagnosis of
a changed node still overwrites nothing, but a re-call with the same prompt and a different model
would still replace the record. Keying by `(prompt_hash, timestamp)` is still not built.

### ⚠️ `INERT` routes by the model's choice, not by the rule

`REPAIR-LOOP.md` says *"`INERT` routes to `PROBE_ENVIRONMENT`; it never terminates the loop on
its own."* Only the second clause is enforced in code — `_terminal_for` excludes `INERT` — while
the action comes from the diagnosis call. On both `INERT` rounds the model chose
`REGENERATE_NODE`, and on round 2 it gave its reason for declining to probe:

> *"The empty drift axis alone would argue for PROBE_ENVIRONMENT, but re-measuring under drift
> would only reproduce the failure, because the l-ratchet and the unconditional wz override are
> drift-independent."*

That is a better decision than the rule would have made, which is an argument for leaving it with
the model — but it is a **gap between the document and the code**, and it should be settled
deliberately rather than left implicit. Raised, not settled.

### ⚠️ `--dry-run` DESTROYED the record of the run it was checking

Found by running the documented command, immediately after the run it destroyed:

    $P scripts/repair_node.py --node gen_b_44d12df4d25b437f --dry-run

`_write` fired after every round, dry or not, so a dry run clobbered `loop.json`. The chunk
advertises `--dry-run` as *"the plan, calls nothing"* — **calling nothing and writing nothing are
different promises and only one of them was made.** Fixed; a dry run now writes nothing at all.

`out/repair/gen_b_44d12df4d25b437f/loop.json` is a **reconstruction**, marked as one in the file
itself. `action` and `action_reason` were re-parsed from the diagnosis transcripts and are exact;
the verdicts, signatures and timings were transcribed from the run's stdout. The round
directories were never at risk and are originals.

⚠️ **A second test was propped up by this bug.**
`test_every_round_is_written_before_the_next_one_starts` used `dry_run=True` for convenience, so
it silently stopped testing anything the moment a dry run correctly stopped writing. It now
drives a real round that terminates on its verdict, which needs no model call and exercises the
write path a real run takes. **Two of the six bugs in this chunk were each holding up a test that
would otherwise have caught something** — worth stating as a pattern, not as two incidents.

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| `--dry-run` calls nothing | prints the plan, classifies round 1, stops before the diagnosis call | confirmed |
| `--dry-run` **writes** nothing | it used to clobber `loop.json`; now leaves a completed record untouched | confirmed, after a fix |
| End to end to a terminal verdict | `UNREPAIRED`, budget spent, 3 rounds, 1380 s | confirmed |
| Every round on disk | `loop.json` + `round-01..03`, written after **every** round | confirmed |
| Resume without repeating a model call | round 1 of a re-run: 31.4 s, $0 spent, identical `session_id` | confirmed, n = 2 re-runs |
| Loop finds the known defects | 2 of 3 in full, the third in a stronger form, plus a new one | confirmed |
| Oscillation guard | fired on the artefact run (identical signatures); on the valid run all 3 signatures differ | confirmed — ⚠️ and its first firing was a FALSE POSITIVE caused by the shim bug |
| `NOT_TRANSFERABLE` refused on round 1 | never proposed in any run | **not exercised** |
| `PROBE_ENVIRONMENT` path | never proposed in any run | **not exercised** |
| `HELPED` / confirmation-set path | never reached | **not exercised** |
| Node source executes under the AST allowlist | `check_source` passed on all 3 repairs | confirmed |
| Repaired node keeps its parent's provenance | `pipeline_version: v3-layered-repaired`, `parent_node_id`, `repair_round` all set | confirmed |
| Physics | not run — the loop is kinematic by design | **not checked** |

**Not checked, and worth saying:** three of the five exits have never been produced by a real
run. `UNREPAIRED` and the verdict path to `ACCEPT` are the only ones exercised. The budget of 3
was never enough to reach a confirmation set, so `CONFIRM_SEEDS` has not been read once.

**Model spend:** ~$4.0 across four runs on 2026-09-17 (13 transcripts total, $5.89 including
2026-09-16's). The valid run cost ~$2.0.

---

## ⚠️ CORRECTION — 2026-09-18. Round 2 was NOT `ACCEPT`, and this Outcome's headline is wrong

**A dated append, never an edit.** Everything above is left exactly as written on 2026-09-17,
including the claim this section retracts. What follows is what the confirmation pass found.

### The claim that does not hold

> *"Round 1 `DEGENERATE` → round 2 `ACCEPT`: the loop removed the regression."* — line 142 above.

Under the **fixed** classifier, `gen_b_44d12df4d25b437f__r01` is **`REGRESSED`**:

    goal_reached fell -0.2000 with no claim covering it
    (claimed goal_reached moved -0.2000, floor 0.1)   n=20

It loses **four worlds the base wins** — seeds **2007, 2009, 2011, 2014** — and fixes none. The
loop **reduced** the regression (10 broke → 4, −0.50 → −0.20). It did not remove it.

### Why the record says otherwise

`repair/loop.py:measure_live` built its `Measurement` with `metrics={}`, so
`pair_of("goal_reached")` returned `(None, None)`, `primary_worse` was always False, and
**`DEGENERATE`, `REGRESSED` and `HELPED` were all unreachable for anything the loop repaired** —
only `INERT` and `ACCEPT` could come back. That bug is **already fixed**, and `repair/loop.py:350`
carries a comment naming this node and this number. It is **not among the six bugs this Outcome
lists**, and the recorded run was never re-derived against the fix.

⚠️ **This is the shim bug's exact shape a second time**: a plausible verdict, produced by a
resolver/measurement defect, that nothing outside the loop had checked. Chunk 58 found the first
instance and stated the lesson; the second instance was sitting inside the same table.

### What was checked, and how

| Check | Outcome | Verdict |
|---|---|---|
| Independent re-measure — `measure_growth.py --staging <round-01>`, a path that never calls `entry_for` | base **10/20** (matches the committed table exactly); `__r01` rate 0.30, delta −0.20, fixed 0, **broke 4** | **confirmed**, n = 20 worlds |
| The verdict rule fed that measurement — `python -m repair.verdict --worlds-csv … --metrics-csv …` | **`REGRESSED`** | **confirmed** |
| Loop's own `measure_live`, re-run today under the fixed code | `fixed 0, broke 4, both_ok 6, both_fail 10` → **`REGRESSED`** | **confirmed** — the two measurement paths AGREE; only the record is stale |
| `claim_for` resolves the card's claim for a repair-round id | `metric='goal_reached', min_effect=0.1, direction='higher', source='card'` for both parent and `__r01` | **ruled out** as a cause |
| **Physics** — the row this Outcome marked *"not checked"* | `layer:gen_b_44d12df4d25b437f__r01` on `gen-maze-02004`: 32.71 m walked, `time_stuck` 14.10 s, 8 stuck events, 0 collisions, **goal not reached** | **confirmed**, n = 1 episode |
| Did the repair remove the FREEZE? | original node 8.27 m / 30.42 s stuck; `__r01` 32.71 m / 14.10 s, stuck *less* than the base's 17.74 s. Fires 896/6000 ticks (14.9%), drives 652 (10.9%) | **confirmed** — the freeze is gone, and the node is **selective**, n = 1 |
| Is the repair a BENEFIT under physics? | nobody reaches the goal on 2004 | **confirmed negative**, n = 1 |
| Is `__r01`'s regression an artefact of the missing drift hazard? | **not checked here** — the 2026-09-17 drift run (`out/evidence/nodes/gen_b_44d12df4d25b437f__r01/`) found 1 fixed / 1 broke at drift 0.10, which is noise at n = 20 | **inconclusive, and already recorded** |

Full evidence, both legs, with the reproducing commands:
`out/evidence/nodes/gen_b_44d12df4d25b437f__r01/README.md`.

### ⚠️ Two further findings from the same pass

**1. Under physics the base does NOT solve `gen-maze-02004`.** `goal_reached: false`,
`termination: timeout`, 30.1 m against a 16.2 m optimum
(`results/gen-maze-02004__base__seed2004.json`, 2026-09-15, uncapped, 6000 steps). It solves it
**kinematically**, which is where every claim about this world comes from. So `REPAIR-LOOP.md:15`,
`plans/done/55-evidence-bundle.md:128` and round 1's own `DEGENERATE` reason describe a kinematic
fact in physics-flavoured prose. The pairing is internally consistent and the verdict stands for
the suite it was measured on — but **"a world the base alone solves" is false under the frozen
locomotion policy**, and it is recorded in no document. `measure_growth.py`'s docstring claim of
6/6 kinematic↔physics agreement on `goal_reached` was measured on **lab42, n = 1 per cell**, never
on a generated maze.

**2. `fire` and `changed` have two different definitions.** `measure_growth.py` computes them
tick-weighted (Σfired / Σticks); `measure_live` takes an unweighted mean of per-world rates. On
identical rollouts that is 0.1398 / 0.0987 against 0.1856 / 0.1282, because per-world ticks run
1442–6000 and an early-finishing world is up-weighted by the second. It flips nothing here — both
clear `FIRE_FLOOR = 0.05` comfortably — but `classify()` compares `measure_live`'s number against
floors calibrated on the growth table's number, and both columns feed `loop.signature_of()`, the
oscillation guard. **A node near the floor can be classified differently depending on which path
measured it.** Raised, not settled.

### What this changes, and what it does not

- ⚠️ **The epic's success criterion is UNAFFECTED.** It asks that the loop independently identify
  one of the three hand-diagnosed defects. It identified all three plus one nobody found, and the
  diagnosis transcripts are on disk. That claim rests on the diagnosis text, not on a verdict.
- ⚠️ **The lineage-best problem is now WORSE than recorded.** Finding 2 of the epic says the loop
  ended on `__r03` having passed through an `ACCEPT` at round 2. There was no `ACCEPT`: **every
  round of the only valid run regressed or was inert**, so the lineage never held a node worth
  keeping and "keep the best" would have had nothing to keep.
- **`plans/06-epic-repair-loop.md` lines 111 and 120 carry the retracted claim** and now carry a
  dated correction pointing here. `plans/README.md` does **not** state the verdict at all — what
  it carries, at line 34, is the *"a world the base alone solves"* claim that finding 1 below
  contradicts. Both are corrected by appending rather than by rewriting, for the reason the top
  of this section gives.

### Tooling added to make the confirmation possible at all

A repaired node was unreachable from every committed tool, because rounds stage under
`out/repair/<lineage>/round-NN/` and **all rounds reuse the parent's stem** (`plan_hash` is a pure
function of (condition, task, card), and a repair re-makes neither).

- `scripts/measure_growth.py --staging <dir>` — repeatable; reads the library from those
  directories instead of `generate/staging/`. Passed straight through to `load_library`, so the
  flag cannot drift from the default it overrides.
- `controllers/_library_entry` delegates a `__rNN` id to `repair.evidence.entry_for`, so
  `layer:<id>__rNN` runs under physics. ⚠️ Adding the round directories to
  `RECONSTRUCTION_STAGING` would have been a four-way stem collision resolved by directory order —
  precisely the "resolver that guesses" this chunk already paid for once.
- Three tests in `core/tests/test_controllers.py` (42 pass): a repair round resolves to the repair
  and **never to the `ContractPorts` shim**; an unreachable round **raises** rather than guessing;
  and a repair round stays out of `library_nodes()`, because it is a run output and not an
  admitted member.
