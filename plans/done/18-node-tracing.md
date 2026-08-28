# 18 — Node tracing

**Epic:** [02](../02-epic-node-library.md) · **Depends on:** 12 · **Needs Isaac Sim:** no
**Added on request**, outside the planned chunk sequence, after chunk 14.

## Why

A scored result says what the robot did. For a behaviour tree it says nothing about *why*, and
the why is the point: a tree that fails because its precondition never fired is a completely
different result from one that fails because its action was wrong, and `core/metrics.py` scores
them identically. G3 counts solved held-out scenarios, so "the library had the right node but the
tree never reached it" and "the node was wrong" are the two failures that must be told apart.

## Build

- `nodes/trace.py` — `TickLog`: per-step rows of `(step, t, depth, node_id, kind, precondition,
  status, drove)`, plus `summary()` and an ASCII `timeline()`.
- `nodes/tree.py` — `Tree.enable_trace()`, opt-in. `tick_node` records; `PortStore` carries the log.
- `scripts/run_episode.py` — enables tracing on any controller that offers it, writes `nodes.csv`
  into the run directory, and logs the summary and timeline.
- `nodes/toys.py` — `SweepForGoal` and `search_tree()`, a Fallback whose branches take turns.
- `nodes/tests/test_trace.py` — 7 tests.

## Outcome

**Done 2026-08-25.** 293 tests pass. `results/` untouched.

### The property that makes it trustworthy

**Tracing changes nothing.** `test_tracing_does_not_change_the_episode` asserts a traced and an
untraced run produce identical trace rows and identical scores, not merely similar ones. A
diagnostic that perturbs what it observes is worse than none, and a benchmark run must be
byte-identical whether or not anyone was watching. It is off by default, so a benchmark run
allocates nothing.

**Deliberately not part of `TRACE_COLUMNS`.** The number of nodes ticked varies per step, so it is
not a fixed-width row; `trace.csv` is what determinism is checked on and what `rescore.py` re-scores
from; and scoring must stay a pure function of the trace. A tick log is an explanation, not
evidence, and nothing in `score()` may ever read it.

### What it showed, in physics

`held_search_past_corner` + `toy_search_tree`, recorded: 1744 node ticks over 436 steps.

```
node                         kind       ticks    pre  drove  statuses
toy_is_goal_known            condition    436    436      0  failure=284 success=152
toy_goal_branch              decorator    436    436    152  failure=284 running=152
toy_sweep_for_goal           action       284    284    284  running=284
toy_drive_to_goal            action       152    152    152  running=152

toy_sweep_for_goal       |###############################################          |
toy_drive_to_goal        |                                              ###########|
```

The handover lands **exactly** on the disclosure latch: the sweep drives through step 283,
`DriveToGoal` takes over at step 284, and step 284 is where `goal_known` flips in `trace.csv`
(t = 5.68 s — the same step as the earlier headless run, so determinism holds with recording on).

That cross-check is the useful one. It ties three independently built things together — chunk
11a's latch, chunk 12's Fallback semantics, and this log — and it would have caught an off-by-one
in any of them.

### Design note

`test_exactly_one_node_drives_per_step` is worth keeping. The Twist is the frozen interface and
`PortView.drive()` is last-write-wins; if two actions ever recorded `drove` in one step, the port
contract's claim that only one Action decides the command would be quietly false.

`SweepForGoal` and `search_tree` are scaffolding and say so: both are in `controllers._SCAFFOLD`,
so `available()` never returns them and nothing can sweep them into an evaluation.

## Not in scope

Overlaying node activity on the recorded video — the recorder pipes raw frames to ffmpeg and text
overlay would mean touching it. The ASCII timeline plus `nodes.csv` covers the diagnostic need.
