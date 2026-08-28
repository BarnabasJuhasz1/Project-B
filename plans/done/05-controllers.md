# 05 — Reference controllers

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 03 · **Needs Isaac Sim:** no

## Why

The harness needs something to run before it can be trusted. Two controllers, one expected to
fail and one expected to succeed, exercise both halves of the scoring code: a harness where the
reference always succeeds proves nothing, and one where it always fails proves less.

Both are **harness-validation references, not benchmark conditions**, and both say so in their
docstrings — the convention `drive_go2.py:120-122` and `walk_forward.py:8-10` already follow.
Neither reads scenario-specific hints; both work from `Observation` alone.

## Build

- `controllers/__init__.py` — name → factory registry, so `run_episode.py --controller <name>`
  resolves without an import chain per controller.
- `controllers/goal_seeker.py` — turn to face `obs.goal`, then walk at it. No use of `ranges`.
  Essentially `demo_waypoints` (`drive_go2.py:135-160`) with the waypoint list removed, the
  ground-truth pose swapped for `obs.pose_estimate`, and the route index moved onto the instance.
  Expected to walk into the divider on `doorway_traverse`.
- `controllers/reactive_avoid.py` — goal attraction plus repulsion from short `ranges` beams.
  Expected to reach the goal, which also sanity-checks that the scenario is solvable at all.

## Done when

`core/tests/test_controllers.py` passes, sim-free, by feeding hand-built `Observation`s:

- `goal_seeker` with the goal behind it commands rotation and near-zero `vx`; with the goal ahead
  it commands forward motion.
- `reactive_avoid` given a short beam dead ahead does not command full speed into it.
- **Both: run a scripted sequence, call `reset()`, run it again, and assert the two command
  sequences are identical.** This is the direct guard against the `drive_go2.py:135`
  mutable-default bug, and the reason `reset()` is in the protocol at all.
- Neither module touches global `np.random` or `random`.

## Outcome

Done 2026-08-22. `controllers/` (registry + two references), `core/tests/kinematic.py`,
`core/tests/test_controllers.py`. Whole suite: **82 passed in 10.2 s**.

Under the idealised unicycle rollout, on all three dev scenarios:

| controller | doorway_traverse | blind_corner | return_home |
|---|---|---|---|
| `goal_seeker` | timeout, 1 collision, 1 stuck | timeout, 1 collision, 1 stuck | timeout, 1 collision, 1 stuck |
| `reactive_avoid` | goal @ 14.4 s | goal @ 6.3 s | goal @ 14.5 s |

Both halves of the scoring code are exercised, and the dev set is demonstrably solvable.

### Differed from the plan

- **`reactive_avoid` is a gap follower, not a potential field.** The potential-field version the
  chunk describes was written first and **parked permanently at (5.36, 3.47)** on
  `doorway_traverse` — nose against the divider, attraction and repulsion balanced. That is the
  local minimum every potential field has, and it is not a defensible "expected to succeed"
  reference. Rewritten as a vector-field-histogram gap follower: pick the most goal-aligned
  direction whose ±25° cone is clear. Still generic, still greedy, still no memory — still exactly
  the shape condition A is expected to produce.
- **`core/tests/kinematic.py` was added**, which the chunk did not call for. It is a point-robot
  unicycle rollout over `core.geometry`, used only to assert the two docstring claims
  ("expected to fail" / "expected to succeed") rather than leave them as comments. Its docstring
  says in capitals that it is not a simulator and no number from it may be reported.
- Two extra tests beyond the chunk's list: `test_goal_seeker_ignores_ranges_entirely` (it is the
  *no-sensing* reference; if it starts reacting to beams the failure metrics lose their baseline)
  and an AST check for mutable default arguments — the shape of the `drive_go2.py:135` bug rather
  than only its symptom.
- The import guard now covers `controllers/` as well as `core/`.

### Two defects found by writing the tests

- **Arbitrary tie-break.** When the front is uniformly blocked, every direction has the same
  cone-minimum and `np.argmax` returned index 0 — so the robot turned whichever way the beam array
  happened to be ordered. Now ties within `TIE_TOLERANCE` are broken towards the goal.
  `test_boxed_in_turns_towards_the_goal_not_by_beam_order` pins it.
- **The stuck metric had a blind spot at exactly its own threshold.** The first `reactive_avoid`
  had a `max(0.05, speed)` floor, and `STUCK_COMMAND_EPS` is also 0.05 — so a controller crawling
  at its floor against a wall was classified as "commanded to stop", not stuck. That is why the
  parked run above scored `stuck_events == 0` while being visibly stuck for 110 s. `MIN_SPEED` is
  now 0.15. **The threshold collision is still latent for any future controller**, and is worth
  raising when the metric constants are frozen for pre-registration.

### Note for chunk 07

`reactive_avoid` clips the doorway corner at (5.72, 5.13) — a circumscribed-circle gap of −0.06 m,
scored as one collision on every dev scenario. `ROBOT_RADIUS = 0.30` circumscribes a 0.65 × 0.31 m
body, so that is probably the corner of the bounding circle and not a wall strike. Whether it is a
real contact is a question only the simulator can answer; compare against the physics `contact`
flag in chunk 07 before concluding the metric is too strict.
