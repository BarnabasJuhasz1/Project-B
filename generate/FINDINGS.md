# Chunk 15 findings

## 0. What the four real generation calls produced

Four `claude -p` calls against `claude-opus-5`, one task family
(`navigate_to_goal`), transcripts committed under `generate/transcripts/`:

| Condition | Input | Node | Outcome |
|---|---|---|---|
| **B** | `limit_speed_dwa` | `LimitSpeedToVisibleRange` | **ADMITTED** — `collisions` 0.0 vs baseline 2.0, effect 1.0 ≥ 0.5 |
| **B** | `vague_adaptive_motion` (fixture) | `MoveAdaptively` | **ADMITTED** — see §1. Excluded from the pass rate. |
| **A** | no literature | `FollowBoundaryToGoal` | **REJECTED** — `UNIT_SCENARIO_FAILED`, effect 0.000 |
| **D** | invented principle | `CommittedDetour` | **REJECTED** — `UNIT_SCENARIO_FAILED`, effect 0.000 |

**Do not read an effect into this.** N = 1 per condition. Both rejections were
checked by hand and are genuine node failures rather than harness artefacts:
A's node travels 27.8 m in its own 60 s world (optimal 8.06 m) without arriving;
D's node holds `wz` at the 1.0 rad/s ceiling indefinitely and circles, covering
0.32 m. Both designed a world their own node could not solve, which is worth
recording on its own — the generator is not reliably able to test what it wrote.

Condition D's `PRINCIPLE` block came back correctly populated (mechanism,
trigger, derivation), so the "D produces cards-without-sources rather than
nodes-without-cards" design works as intended.


## 1. The admission harness cannot detect card infidelity — a validity threat to condition B

**The deliberately-bad card was ADMITTED.** `generate/tests/fixtures/vague_adaptive_motion.yaml`
has a contentless Mechanism ("move intelligently, adapting to the environment"), an
unconditional Trigger, and a Derivation that restates the Mechanism. The pipeline
was run on it twice, with identical inputs:

| Run | Prompt hash | Outcome |
|---|---|---|
| 1 | `45a8af…` (first call) | **NO_CANDIDATE_EMITTED** — the model refused, and said why |
| 2 | `45a8af4b86a63990` | **ADMITTED** — `goal_reached` 1.000 vs baseline 0.000 |

Run 1's refusal diagnosed run 2 in advance, and is worth quoting in the write-up:

> If I write a node body, every line of it is my invention wearing the card's name.
> […] Whichever I picked would be the thing under test, and the card would get
> credited for a result it did not predict. The scenario would be well-formed and
> the number would clear the floor, and the number would mean nothing.

Run 2 then did exactly that. The admitted node's own `EFFECT` describes clearance-weighted
heading selection plus a commitment-to-boundary-following mode switch on stalled progress —
**none of which appears anywhere on the card**.

**Why the harness cannot catch this.** `harness/unit.py` tests whether the NODE works, not
whether the node implements the CARD. Those are different claims and only the first is
mechanical. A model handed a vague mechanism fills the gap with a real one, and the real one
passes on its own merits.

**Why this threatens G2.** B−D is supposed to separate genuine information gain from the model
restating what it already knew. If B's nodes are partly model-invented mechanisms attributed to
cards, B's advantage is manufactured by the attribution rather than by the corpus — and it would
look identical in the numbers. **The experiment design already contains the mitigation**: it is
precisely why the plan says B−D matters most, and why `limit_speed_dwa` was written as the
internal control. What is needed is to say so explicitly, not to add machinery.

**For chunk 17.** Card fidelity must be declared: either (a) a stated limitation, with B−D as the
control that absorbs it, or (b) a pre-registered fidelity check. It cannot be left implicit.

**For chunk 16.** A cheap partial check exists and belongs with composition: run each admitted
node on the OTHER admitted nodes' micro-worlds. A node implementing a separable mechanism should
degrade gracefully; one tuned to a self-designed trap should collapse.

## 2. The generator designs its own micro-world, so the effect floor can always be cleared

`harness/unit.py`'s requirement 2 — the baseline must actually fail — was written to stop a
generator writing a test that was already true. It has a second effect nobody intended: it
**incentivises an adversarial world**. The model is free to construct a trap that defeats
`reactive_avoid`, and then any competent node clears the floor. Both the good card and the bad
card did this. The requirement is still right; it is just not sufficient, and the write-up should
not present a cleared effect floor as evidence the mechanism generalises.

## 3. Generation is reproducible, not repeatable, and temperature cannot fix it

`temperature` was removed from the Messages API on Opus 5, Fable 5, Sonnet 5, Opus 4.8 and 4.7 —
passing it is a 400. Chunk 15's "temperature 0 produces a byte-identical candidate" is therefore
unavailable on any current model. The two runs of the bad card above are the demonstration: same
prompt, same model, two different outcomes.

Replaced by the transcript: every call writes `generate/transcripts/<prompt_hash>.json` and
`ReplayBackend` rebuilds a library byte-for-byte from committed files with no model, no key and
no network. That is a stronger reproducibility property than temperature 0 would have given, and
it is the one a container run should use.

## 4. Chunks 12 and 14 disagreed about source spans

`nodes/contract.py` required a CARD-origin node to carry a source span; chunk 14 deliberately
left every pilot card's span empty, because there is no corpus on this machine. A condition-B
node was therefore unconstructible, and both ways out were lies — substitute the citation for the
span, or record a corpus node as INVENTED.

Fixed in `nodes/contract.py` by separating **origin** (where the node came from) from
**traceability** (whether anyone opened the paper). A CARD node must name its card and confidence;
the span stays optional, and `is_traceable` still demands it. An unverified card now yields a node
that is CARD and untraceable — the honest state, counted in G1's denominator and not its numerator.
