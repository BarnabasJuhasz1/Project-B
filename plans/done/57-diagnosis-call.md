# 57 — The diagnosis call

**Epic:** [06](../06-epic-repair-loop.md) · **Depends on:** 55 · **Model calls:** yes · **Needs Isaac Sim:** no

---

## What it does

One model call. Blind bundle in, a **structured proposal** out:

```python
diagnose(bundle) -> Diagnosis     # cause, evidence, proposed action, confidence
```

⚠️ **It proposes. It does not decide.** The verdict came from `repair/verdict.py` before this
call was made, and the loop — not the model — chooses whether to act on the proposal. The model's
job is the part a rule cannot do: reading a card, a node, a trajectory and a set of outcomes
together, and saying *why*.

## The action vocabulary is closed

The proposal names one of exactly:

| Action | Meaning |
|---|---|
| `REGENERATE_NODE` | the card is fine, the implementation is not — re-make the node call with the diagnosis appended |
| `PROBE_ENVIRONMENT` | this was measured somewhere its capability was not on the table |
| `ACCEPT` | valid, acted, and the outcome is the one the card predicts |
| `NOT_TRANSFERABLE` | faithful and competent; the paper does not carry over here, and here is why |

⚠️ **`REGENERATE_CARD` is deliberately absent from the vocabulary in the core version** (user's
instruction, 2026-09-16). A model that can name it will name it, and there is nothing behind it.
When it lands it comes with its own chunk — see `REPAIR-LOOP.md`.

A free-text action is refused and the call is recorded as a failure, not silently coerced. The
same rule `compose/compose.py` applies to a plan whose vocabulary is the retrieved node ids and
nothing else — a closed vocabulary is what makes the output checkable.

## Reuse, not reinvention

`generate/backends.py` already provides everything: `ClaudeCliBackend` with a fresh session per
call (so nodes cannot leak into each other) and `--setting-sources ''` (so `CLAUDE.md` is not in
the model's context — which matters *enormously* here, since this CLAUDE.md names the arms, the
conditions and this very node). `ReplayBackend` is the reproducible path. Transcripts are keyed
by prompt hash and committed.

⚠️ **`--setting-sources ''` is load-bearing for blindness, not hygiene.** A diagnosing agent that
could read this repo's CLAUDE.md would know which arm every node belongs to, that D is frozen,
and that B−A is the headline. Pin it with a test.

`repair/prompts/diagnose.md` is **one prompt for every condition**, fed a blind bundle. It has no
condition parameter, and a test pins that — the same test `generate/prompts/modifier.md` already
has.

## What the prompt must ask for

The cause, in terms of the card. Specifically, whether the node's behaviour departs from **what
its own card says it should do** — because that is the check a rule cannot make and a reader can.
For the first target the known-good answer is that the ant card contains three instructions the
node did not follow:

- *"it must sit at lower priority than an avoidance leaf"* — it sits at highest priority
- *"the tree should fall through to whatever computes goal bearing directly from pose_estimate"*
  — the fallback ports are published and nothing reads them
- *"left and right turns are equally frequent ... and their opposite-signed errors cancel"* —
  the applicability condition is stated in the card and never tested

A diagnosis that finds none of these on a bundle containing both the card and the frozen
trajectory has not read them together, and the prompt needs work.

## Done when

- `diagnose()` returns a structured `Diagnosis` with a closed action, or a recorded failure.
- The transcript is committed and the call replays from it.
- A test pins that `render` has no condition parameter.
- A test pins `--setting-sources ''`.
- ⚠️ **Run live against `gen_b_44d12df4d25b437f` and write the verbatim proposal into the
  Outcome**, alongside the three known-good findings above, marked found / missed. This is the
  epic's success criterion and it is cheap to check exactly once.

---

## Outcome

**Built 2026-09-16.** `repair/prompts/diagnose.md`, `repair/diagnose.py`,
`repair/tests/test_diagnose.py` (29 tests, none of which calls a model), and the first live
transcript committed at `repair/transcripts/0b00747fffa64ce6.json`.

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P -m repair.diagnose --node gen_b_44d12df4d25b437f --auto-case --dry-run   # prompt only
$P -m repair.diagnose --node gen_b_44d12df4d25b437f --auto-case --replay    # committed
$P -m repair.diagnose --node gen_b_44d12df4d25b437f --auto-case             # ⚠️ REAL CALL
```

Prompt: 24,066 chars (~6,000 tokens). **One live call, `$0.43`**, against a $2.50 backstop.

---

## ⚠️ THE EPIC'S SUCCESS CRITERION: MET

The epic states that a loop terminating cleanly without finding any of the three hand-diagnosed
defects **has failed, however tidy its output**. Given only the blind bundle, the model found
**all three, plus a fourth nobody found by hand.**

**1. The arbitration conflict.** Verbatim:

> *"the base gates forward speed on being aligned with `nav_heading` and raises its turn rate to a
> floor until it is, and the node's last-writer-wins `wz` deletes that floor... the base can no
> longer rotate onto its plan, its speed gate never opens"*

**2. The self-latching trigger.**

> *"this is self-latching: the integrator is fed the executed command, so with speed ~0 no strides
> accrue, `l` and `phi` freeze, b stays ~0, and the node keeps commanding zero turn forever"*

**3. `l` is path length, not net displacement** — and it connected this to the card's *own* Eq. 1b
flag, which is the provenance point the whole corpus argument rests on:

> *"Faithful to the card's own text, but the card's flag was never honoured with a validity check."*

**4. ⚠️ AND THE DRIFT AXIS, UNPROMPTED.** It opened with it:

> *"The node is not being measured on its claim (the `drift` axis reads None, so `pose_estimate` is
> exact and a drift-immune bearing source has nothing to win), but that is not why it scored
> DEGENERATE"*

That is chunk 55's `conditions_block` doing exactly the job it was built for — and note it did
**not** stop there and call `PROBE_ENVIRONMENT`. It separated "measured in the wrong place" from
"broken anyway", and chose `REGENERATE_NODE` because *"that defect would ruin the node under drift
too, so it must be fixed before any environment question is worth asking."* That is the judgement
the taxonomy exists to enable, made correctly, unprompted.

### Two defects it found that I did not

- **`precondition()` overwrites `_last_cmd` with the BASE's proposal**, so the integrator is fed
  the base's command — including negative stuck-recovery reversals — not the node's own, which the
  card explicitly demands. A real card departure, missed by hand.
- **The SUCCESS test becomes unreachable** once `l*u > ||G|| + 0.5` (8.63 m against a 17.72 m
  geodesic), so the node can report SUCCESS from a radius test that can never be satisfied.

Four card departures, seven pieces of cited evidence, all traceable to the bundle.

---

## What differed from the plan

**⚠️ The first live call failed to parse, and the parser was wrong, not the model.** It returned a
complete, correct, four-departure diagnosis and omitted exactly one field — `confidence` — and the
entire $0.43 result was discarded.

The line now drawn: **a field the loop ACTS ON is required; a field that is only REPORTED is
optional.** `action`, `cause` and `action_reason` drive what happens next; `confidence` is metadata
for a human reading the log. It defaults to `"unstated"`, which is *in* the closed vocabulary — so
absent is fine, while **present-and-wrong is still refused**, because a model writing
`confidence: "pretty sure"` has not understood the schema.

This matters more than it looks: **a rejection is never retried**, so over-strictness here is
unrecoverable waste rather than safety.

The fix was validated by **replaying the committed transcript** — no second call was needed, which
is the whole reason transcripts exist.

**One edit outside the chunk:** `generate/backends.py` gained a `system_prompt` parameter,
defaulting to the existing constant. The default ends *"Return the code in a single fenced python
block"*, which on a diagnosis call instructs the model to do the wrong thing. All 175 `generate/`
tests pass unchanged.

## Checks

| Check | Outcome | Verdict |
|---|---|---|
| Does the model find the arbitration conflict? | quoted verbatim, names `nav_heading` and last-writer-wins | **confirmed**, n=1 |
| The self-latching trigger? | quoted verbatim | **confirmed**, n=1 |
| `l` as path length, tied to the card's Eq. 1b flag? | quoted verbatim | **confirmed**, n=1 |
| The drift axis, unprompted? | opened with it | **confirmed**, n=1 |
| Did it confuse "wrong place" with "broken"? | **no** — it separated them explicitly and justified the ordering | **ruled out**, n=1 |
| Does the bundle leak an arm? | `leaks()` empty on the exact prompt sent | **confirmed** |
| Prompt offers exactly the actions the parser accepts | test pins both directions | **confirmed** |
| `REGENERATE_CARD` refused by name | tested for all three deferred actions | **confirmed** |
| `--setting-sources ''` + scratch cwd | pinned by a source test | **confirmed** |
| Cost per call | **$0.43** against a $2.50 backstop | **confirmed**, n=1 |

## ⚠️ Not checked, and the honest limits

- **n = 1.** One node, one call, one model. The result is strong but it is a single sample, and a
  single-seed result that looked like a finding in chunk 22a did not survive five seeds. **Nothing
  here says the loop works in general** — only that it worked once, on the one case where a
  known-good answer existed to check it against.
- **The diagnosis has not been acted on.** Whether `REGENERATE_NODE` plus that guidance actually
  produces a better node is chunk 58, and is a completely separate question.
- **Repeatability is untested.** The same prompt may produce a different diagnosis; temperature-0
  determinism does not exist on this model family. The transcript is the record, not a guarantee.
- **The known-good answer was written by me, in this session.** The bundle was built by me knowing
  what the defects were. I did not deliberately steer the bundle toward them — the contract block's
  `nav_heading` warning was written in chunk 55 before this call — but **a genuinely blind test
  would have had someone else pick the target**, and this one did not. Stated because it is the
  kind of thing that is invisible later.

---

## Follow-up 2026-09-16: the generalisation check (n = 3, on the user's instruction)

⚠️ **57's result was n = 1.** Three more nodes were diagnosed to test the failure mode that
would silently wreck chunk 58: **an agent that always finds something to fix.** Eight of the
sixteen library nodes are INERT; if the agent proposes `REGENERATE_NODE` on those, the loop
becomes a repair-everything machine and burns its budget on nodes that were never broken.

**Predictions were written down before the calls were made.** Two of three completed; total
~$1.00.

| node | verdict | predicted | actual | |
|---|---|---|---|---|
| `gen_a_f5f830d140abf703` | INERT | not a reflexive repair | **`PROBE_ENVIRONMENT`** | ✅ held |
| `gen_b_c380e7a8cfb32b78` | MISJUDGED | recognise the declared trade | **`REGENERATE_NODE`** | ❌ **wrong, and I was the one who was wrong** |
| `gen_a_fac8c7b94fe6b1c6` | REGRESSED | `REGENERATE_NODE` | **backend failure** | — |

### ✅ The failure mode did not appear

On the INERT node it chose `PROBE_ENVIRONMENT` and reasoned correctly: `drift: None` makes the
card's sole trigger (a pose-error threshold) *"unsatisfiable by construction"*.

It then did two things beyond the ask:

- **It explained my own verdict's number.** The `fire = 100%` in the INERT reason string is an
  artifact — the precondition returns `err <= _ERR_TRUST`, which with `err == 0.0` is true on
  every tick, *"so the harness counts a 'fire' on every tick of a node that is, correctly, only
  ever recording a reference."* My rule reported the number; the model explained it.
- **It anticipated a wasted loop round.** It found a *second*, independent defect (a 1.0 m
  frontal `_blocked_ahead` veto) and warned it *"should be fixed before that re-run or it will
  report INERT a second time for a different reason."*

### ❌ My prediction was wrong, and the model was right

I expected `gen_b_c380e7a8cfb32b78` to be recognised as a declared collisions↔arrivals trade.
Instead it found a genuine implementation defect: **`MARGIN` is used twice** — as the corridor
half-width *and* as the standoff subtracted in the braking distance — so any corridor narrower
than ~0.70 m yields `v_out = 0` **for every heading**, in worlds whose narrowest corridor is
0.42 m. Rotating in place, the card's stated escape, can never lift it.

It identified the resulting limit cycle from the trajectory (*"the base pushes forward, the node
zeroes it, the base recovers by backing up (uncapped), and repeats"*), and used it to explain
another number of mine: why `changed` is 0.212 rather than ~1.0 despite a 0.992 fire rate —
*"the node binds only on the forward half of the cycle."*

⚠️ **So the model overrode my rule's label and was correct to.** This is the
rule-labels / model-explains split working exactly as designed, including the part where the
model is allowed to dissent. It also confirms chunk 54's flagged threshold sensitivity was real:
that verdict sat at a stuck ratio of **1.420 against a 1.50 ceiling**, and it was on the wrong
side of it.

### ⚠️ A real defect found: a backend failure lost the call entirely

The third call spent **$0.30 and 6,016 thinking tokens out of a 6,120-token output budget**,
emitted almost no text, and raised `BackendError`. Only `DiagnosisError` was being recorded, so
**the whole call vanished** — same family as the sweep rehearsal's "exited 0, wrote nothing".

Fixed: `repair.diagnose.write_call_failure` mirrors `generator.write_call_failure`, filing
`<hash>.error.json`. `ReplayBackend` reads `<hash>.json`, so a failure can never be replayed as
though it were a diagnosis — pinned by a test. The token counts are kept verbatim because on a
runaway **the token counts are the finding**: 6,016 thinking against 104 of output is a model
reasoning itself out of an answer, which wants a different response from a timeout or a refusal.

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| Does the agent reflexively propose repairs? | **NO** — `PROBE_ENVIRONMENT` on the INERT node | **ruled out**, n=2 completed |
| Does it explain numbers the rule only reports? | yes, twice (`fire=100%`, `changed=0.212`) | **confirmed**, n=2 |
| Can it dissent from the rule's label? | yes, and correctly | **confirmed**, n=1 |
| Is chunk 54's MISJUDGED threshold sensitivity real? | **yes** — 1.420 vs a 1.50 ceiling, wrong side | **confirmed** |
| Are backend failures recorded? | **NO, they were lost** | **defect found and fixed** |
| Cost per call | $0.43, ~$0.30, ~$0.30 | **confirmed**, n=3 |

### Not checked

- **The REGRESSED-by-crashing branch was never tested** — the one call that would have tested it
  is the one that failed. Whether the agent distinguishes *crashing* from *freezing* is still
  unknown, and it matters: they are different defects wanting different repairs.
- n = 2 completed. Still small.
- No diagnosis has been acted on. That is chunk 58.

---

## ⚠️ Correction 2026-09-16: the verbatim quotes above are no longer on disk

**A transcript keyed by prompt hash is NOT an immutable record of a particular call.**
`write_transcript` files under `prompt_hash(prompt)` and **overwrites**. The diagnosis prompt is
deterministic, so every later live run on the same node rewrites the same file.

`repair/transcripts/0b00747fffa64ce6.json` now holds a *different* diagnosis of the same node,
produced by a chunk 58 loop run. It is equally correct and finds the same defects — the deadlock,
the `cap`/`adjust` asymmetry, the missing `nav_heading` read, the untested drift axis — in
different words.

**Consequence, stated plainly: the quotations in this Outcome are accurate as of when they were
made, and are no longer verifiable from the repository.** The artefact that backed them has been
replaced. That is a real loss and it is recorded rather than quietly tidied.

`test_the_first_live_diagnosis_found_the_known_defects` has been weakened to assert the
SUBSTANCE is found rather than the phrasing, and its docstring carries this reason.

**The fix belongs in a follow-up, not here.** For generation this behaviour is tolerable — the
pipeline is re-derivable and a rewritten transcript describes the same deterministic step. For a
diagnosis it is not: "what the model said about this node on this date" is exactly the thing the
record is for. Options are to key diagnosis transcripts by `(prompt_hash, timestamp)`, or to copy
a transcript into `repair/tests/fixtures/` when it is first cited. **Neither is built.**
