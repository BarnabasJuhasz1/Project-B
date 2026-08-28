# 07 — Episode runner

**Epic:** [00](../00-epic-reproducible-episode.md) · **Depends on:** 01–06 · **Needs Isaac Sim:** yes

## Why

The chunk the epic exists for: one scenario, one controller, headless, scored, with provenance.
Everything before this is sim-free scaffolding; this is where it meets the simulator.

## Build

### `scripts/go2_policy.py` — sim-side helpers, imported only after `SimulationApp`

Three things already work and are currently duplicated between `drive_go2.py` and
`walk_forward.py`. Lift them here rather than writing a third copy:

- the policy-class probe ladder (`drive_go2.py:89-105`) — resolves to
  `isaacsim.robot.policy.examples.robots.go2.Go2FlatTerrainPolicy` on 6.0.1
- `base_pose()` (`drive_go2.py:180-193`) — handles 6.0's batched warp arrays
- `make_command()` (`drive_go2.py:168-177`) — preallocated torch tensor on `robot._device`

Plus the two things `walk_forward.py` gets right and `drive_go2.py` does not:
`World.clear_instance()` before construction, and deriving the control rate from `policy._dt` /
`policy._decimation` (`walk_forward.py:237-244`) rather than hardcoding 200/50 Hz.

Leave `drive_go2.py` and `walk_forward.py` untouched — they work, CLAUDE.md records them as smoke
tests, and refactoring them onto this module is a separate follow-up.

### `scripts/run_episode.py` — the entry point

Follows the repo's established shape: argparse at module level (so a bad flag is rejected before
the 20 s boot), then `SimulationApp`, then the omni imports.

```
--scenario doorway_traverse --controller goal_seeker [--seed N] [--record PATH] [--gui]
```

Loop, once per control step:
1. read true pose, build `Observation` — `ranges` from `geometry.raycast`, `pose_estimate`
   currently equal to the true pose, `clearance` kept out of the observation and recorded on the
   trace for scoring only
2. `controller.step(obs)` → `Command`
3. write the Twist into the preallocated tensor, `policy.forward(dt, command)` for `decimation`
   physics steps
4. append to the `EpisodeTrace`
5. terminate on goal, timeout, or fall

Then `score()`, `capture()`, and write:
- `results/<scenario>__<controller>__seed<N>.json` — scored result plus manifest, committed
- `out/runs/<run_id>/trace.csv` — gitignored (`**/runs/` already covers it)

`--record` wires to the existing `recorder.Recorder` exactly as `drive_go2.py:264-273` does. Off
by default: benchmark runs must not pay for rendering.

## Done when

```
python scripts/run_episode.py --scenario doorway_traverse --controller goal_seeker --headless
```

completes and writes a `result.json` showing `goal_reached: false` and `collisions > 0` — the
naive controller walking into the divider, scored correctly.

## Outcome

Done 2026-08-22. `scripts/go2_policy.py`, `scripts/run_episode.py`. Both dev-set references run
headless end to end, scored, with provenance.

```
python scripts/run_episode.py --scenario doorway_traverse --controller goal_seeker
  -> goal_reached false, termination timeout, collisions 1, stuck_events 2, time_stuck 78.8s
     jammed against the divider at (5.60, 3.87), gap 0.025 m, for the last ~80 s

python scripts/run_episode.py --scenario doorway_traverse --controller reactive_avoid
  -> goal_reached true, time_to_goal 15.44 s, path 10.63 m vs 11.07 m optimal, spl 1.0
     15.4 sim seconds in 23.6 wall seconds
```

Both halves of the scoring code fire against the real simulator, not just synthetic traces.
Rates derived from the policy: physics dt 0.005 s, decimation 4, **50 Hz control**.
Writes `results/<run_id>.json` (result + manifest) and `out/runs/<run_id>/trace.csv`
(gitignored by the existing `**/runs/` rule).

### The blocking find: FALL_HEIGHT was inside the normal gait envelope

The first run terminated `"fell"` after 0.9 s, every time. It was not falling. Measured on the
6.0.1 Go2 base prim under the frozen policy:

| | base z |
|---|---|
| standing still | ~0.213 (min 0.211 over 3 s) |
| walking 0.5 m/s | ~0.145 (**min 0.128** over 5 s) |
| turning 1 rad/s | ~0.182 (min 0.144 over 3 s) |

`FALL_HEIGHT = 0.15` sits *inside* that envelope. **`core/metrics.py` now uses 0.08**, well below
the walking minimum and unambiguously a collapsed posture, with the measurements recorded in the
comment.

⚠️ **`scripts/testing/walk_forward.py:137` still has 0.15 and is therefore reporting a false
failure** — on this machine it prints `WARNING base height 0.138 is below 0.15. The policy has lost
the gait` and `travelled 1.97 m in 6.0s (FELL)` while walking perfectly well. The chunk says to
leave that file alone, so it is untouched, but the locomotion smoke test currently fails a healthy
gait and that should be fixed before anyone uses it to judge the policy.

Better still, **a fall is a tilt, not a height.** Base roll/pitch is a far more robust criterion
than base z, which varies by 40% across a gait cycle. That needs orientation on the trace and is
deliberately not done here — raise it when the metric constants are frozen for pre-registration.

### Differed from the plan

- **A settle phase was added before episode t = 0.** The robot is spawned at 0.40 m (walk_forward's
  short drop, not build_scene's 0.55) and given 1.5 s of zero command before the clock starts, so
  the drop is not charged against the time limit and no controller is ever asked what to do about a
  falling robot. If the base is still below `FALL_HEIGHT` after settling, the run aborts with a
  message pointing at `walk_forward.py`.
- **`Episode.drive()` holds one Twist across the whole decimation window.** The policy runs at
  physics rate and expects the command held, not re-decided every physics step.
- **`_pump()` goes through `omni.kit.app.get_app().update()`** rather than the caller's
  `SimulationApp`. The first version reached into `sys.modules["__main__"].sim_app`, which couples
  the helper to how the entry point happens to name its variable.
- The runner terminates on the same conditions `score()` uses, so the trace does not run past the
  outcome — but `score()` remains the only thing that decides what happened.
- Start yaw is now honoured: `yaw_to_quat()` feeds the constructor's `orientation` argument, which
  6.0.1 accepts. It does **not** accept `name`, so that is dropped (already handled by the ladder).

### Known limitation

**`Observation.contact` and the trace's `contact` column are the analytic clearance test, not a
physics contact report.** `gap = clearance(x, y) - ROBOT_RADIUS <= 0`. Chunk 05 said to compare the
two in this chunk; that was not done, because it needs contact sensors added to the Go2 prim.
Consequence: `reactive_avoid` scores exactly 1 collision on every scenario, always at the doorway
corner (min clearance −0.069 m here), and whether the body actually touched the divider is still
unverified. `ROBOT_RADIUS = 0.30` circumscribes a 0.65 × 0.31 m body, so a small negative gap is
probably the bounding circle's corner. **This needs settling before the collision count is reported
as a secondary metric.**
