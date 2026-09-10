# 41 — The harness and the runner do not share a timebase

**Status:** todo. SIM-FREE to diagnose and to fix; **one physics run to confirm.**
**Blocked by:** nothing. **Found by:** chunk 36, diagnosing why B fails under physics.

---

## The defect, in two lines

```python
core/tests/kinematic.py:61     t = step * dt        # exact: t - W lands ON a sample
scripts/run_episode.py:627     t += control_dt      # accumulated: drifts ~1e-12 s
```

Nominally the same timestamps. The difference reaches **3.78e-12 s** over a 60 s
episode — and that is enough to silently disable a node's mechanism between
admission and evaluation.

## The evidence it was found on

`gen_b_4dc13c6a972bb06b` (`ReorientToGeometry`, condition B, Cheng 1986) keeps a
1.2 s window of pose history and asks whether it has moved:

```python
cutoff = obs.t - self.STUCK_WINDOW
while len(self._history) > 2 and self._history[0][0] < cutoff:
    self._history.pop(0)
...
if self._history[-1][0] - self._history[0][0] < self.STUCK_WINDOW:
    return False
```

The trim guarantees the span is **≤** `STUCK_WINDOW`; the test then demands it be
**≥** `STUCK_WINDOW`. So the predicate can only pass on **exact float equality**,
which needs `obs.t - 1.2` to land exactly on a stored timestamp. `STUCK_WINDOW`
is an exact multiple of the 0.02 s control step, so in exact arithmetic it always
ties. Measured, replaying the recorded episode's own belief trace:

| timebase | window test rejects | stuck verdicts | first fires |
|---|---|---|---|
| `step * dt` (the harness) | 1964 / 3000 | **804** | t = 33.2 s |
| `t += dt` (the runner) | **2997 / 3000** | **0** | never |

Instrumented directly, the same node: **8 stuck verdicts and four phase
transitions** on `doorway_traverse` under `core/tests/kinematic.py`, **1 verdict
and a goal reach** in its own unit micro-world, and **zero** under physics — where
it drove into the divider and stayed there for 49.6 of 60 s. Its paper's mechanism
has never executed in a physics episode.

## Why this is not one node's bug

This is **chunk 10's finding a second time.** 10 fixed a float knife edge inside
`core/metrics.py` — event counts flipping on a 6.7e-12 s timestamp difference,
`STUCK_WINDOW` being an exact multiple of the control step — and fixed it *in the
scorer*. The same edge exists in the **contract between the harness and the
runner**, where nothing has ever looked, and it is now confirmed to bite a
generated node.

The consequence for the project is the load-bearing part: **a node admitted on
the kinematic rollout can behave differently under `run_episode.py` for reasons
that have nothing to do with physics.** `harness/micro.py`'s docstring warns that
a node passing its unit scenario may fail under physics because the locomotion
policy is absent. It does not warn that the node may fail because **`t` is
computed differently**, and that is a defect rather than a caveat. Every
admission decision, every `harness/ablate.py` contribution, and every
`scripts/ab_compare.py` row is computed on the `step * dt` timebase; **the G2
sweep is not.**

## What to do

1. **One definition of the episode clock**, imported by both, the way
   `core.manifest.run_id()` is one definition with three callers. `step * dt` is
   the right one — it does not accumulate error and it is what the scorer's
   re-scoring path already assumes.
2. A test that the two agree **bit for bit** over a full-length episode, not
   approximately. Approximately is what is true today.
3. Re-run chunk 36's four episodes and check whether `ReorientToGeometry`'s
   stuck verdict now fires under physics. **Do not assume it recovers** — the
   mechanism firing is not the same as the mechanism working, and the honest
   outcome may be that it fires and still fails.
4. **Do not "fix" the node.** It is a committed condition-B artefact and G1's
   denominator; editing model-written code after the fact is not available.
   A window predicate that can only pass on float equality is worth recording as
   a *generation* finding in `generate/FINDINGS.md` — the model wrote a
   comparison that is exactly the tie the repo had already been bitten by once.

## ⚠️ Pre-registration

The timebase reaches `time_to_goal`, `stuck_events` and `recovery_time`, three of
the metrics `PRE-REGISTRATION.md` names. Changing it moves committed numbers, so
it is a **dated append** to the Amendments section, never an edit, and
`scripts/rescore.py` has to be run over `results/` afterwards with
`core/tests/test_results_integrity.py` green. **This is the user's call, not the
chunk's.**

## Done when

- One clock, imported by both, with a bit-for-bit test over a full episode.
- Chunk 36's four episodes re-run and the delta recorded either way.
- The generation finding is in `generate/FINDINGS.md`.
- A dated append to `PRE-REGISTRATION.md` **if the user approves it**, with the
  re-scoring done and the integrity test green.
