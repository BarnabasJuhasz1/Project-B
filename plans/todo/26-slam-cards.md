# 26 — The cards SLAM unlocks

**Epic:** [03](../03-epic-slam.md) · **Depends on:** 25 · **Needs Isaac Sim:** no

## Why

The whole point. Epic 03 is infrastructure, and infrastructure that unlocks nothing is cost
without benefit. This chunk cashes it in: the mechanisms that were blocked, deferred or testable
only in a degraded form become cards, nodes, and unit scenarios.

It is also the chunk that tests the epic's own premise. If SLAM lands and no card scores
differently because of it, that is a finding worth as much as the alternative — and it is the
kind of finding this project is built to be able to state.

## Decided

**The new sealed split is authored here, and the procedural rule from chunk 11 applies in full.**

A SLAM cycle needs held-out scenarios whose axes include localisation. Chunk 11's seal is half
technical and half procedural, and the procedural half is binding: **the session that seals the
new split must never be the session that generates against it.** Nothing in the repo can detect a
violation.

The existing split at `00b3312b…` is **untouched** — epic 03's decision 4, and chunk 11's Outcome
explains what re-sealing would cost. The new one is additive, sealed separately, tagged
separately.

**Four card families, in priority order.**

### 1. `MaintainHomingVector` — arbitration, finally

`cards/path_integration_mammals.yaml` predicts that path integration is **arbitrated against
other cues rather than used alone**. Under an open-loop random walk there is no other cue to
arbitrate against, so the card could only be tested in the degraded form "replace the pose
estimate", which is what `path_integration_ants` already says. **The pairing was the point** —
chunk 14 wrote two cards with one `node_name` and different Derivations specifically to test
whether the Derivation field carries information — and until now the two collapsed onto the same
node.

With SLAM the arbitration is real: integrate the homing vector, weight it against the SLAM pose
by their respective uncertainties, and the card's claim is falsifiable exactly as written.
**This is the single strongest reason to build the epic.**

### 2. `IsLocalisationDegraded` — a Condition that finally gates something

Reads `pose_error_estimate`, which under SLAM rises between closures and drops when one lands.
⚠️ It is a **Condition**, and `generate/admit.py` currently rejects those with
`KIND_NOT_TESTABLE_ALONE` — a Condition emits no Twist and cannot move a navigation metric by
itself. So this card is **blocked on chunk 16's composition work**, not on this epic. Say so
rather than discovering it at admission time.

### 3. `DeviateToDisambiguate` — back in scope, on a different dependent variable

Ruled out of cycle 1 on cost, not merit: its dependent variable was an *observer's* inference
about intent, no metric measured belief, and a legible path is longer than an optimal one so it
scored negatively on SPL and would be rejected for a reason unrelated to whether the mechanism
works.

Under SLAM, a deviation that brings known geometry back into view **has a measurable payoff in
pose error**. Same mechanism, a dependent variable the harness already computes, and the SPL cost
is now a real trade-off rather than a scoring artefact — which is what `cost_metric` exists to
record. It was one of the strongest cards on the "would the model invent it anyway?" screen, so
it is worth the effort.

### 4. `RequireSensorAgreement` — still blocked, and it stays blocked

One modality. Cross-checking needs chunk 27's camera, and this chunk does not pretend otherwise.

**No card may claim novelty**, unchanged. `Card.__post_init__` enforces it and the reason is the
same as always: every source a model can suggest is inside its own training distribution.

**The unit scenarios must isolate the localiser, not the navigator.** A micro-world that tests
`MaintainHomingVector` has to be one where a robot with a good pose estimate succeeds and one
with a bad estimate fails — which means the fixed baseline is the same node with arbitration
switched off, not `goal_seeker`. Chunk 13's `SteerPastObstacle` / `SteerPastObstacleDisabled`
fixture is the pattern.

## Build

- `harness/micro.py` — micro-worlds carrying a `localisation` spec, and enough structure for a
  loop to be closeable. A featureless box gives a scan matcher nothing to match.
- New cards in `cards/`, with `cards/VERIFICATION.md` updated — and **still unverified** unless
  someone has actually opened the papers.
- `core/scenarios_held_out_slam.py` — the new sealed split, with 11's AST guard extended to it.
- Generated nodes via `scripts/generate_node.py`, per condition, recorded as always.

## Done when

- A micro-world exists in which a robot with a good pose estimate reaches the goal and one with a
  degraded estimate does not — the precondition for any of these cards being testable at all.
- `MaintainHomingVector` is generated from **both** path-integration cards, and the two nodes are
  compared. If they score identically the Derivation field carries no information, and that is
  reported.
- Every new card's Prediction names a metric `score()` computes and that a controller can move on
  a SLAM scenario — enforced at load by chunk 25's scenario-aware rule.
- The new split is sealed, hashed, tagged, and its hash recorded in the tests and every manifest.
- The AST guard covers the new split module, and `generate/` and `compose/` are checked against it.
- `IsLocalisationDegraded` is either admitted through chunk 16's composition path, or **recorded
  as blocked with the reason** — not quietly dropped.

## Not in scope

Running cycle 2's evaluation, and camera VO.
