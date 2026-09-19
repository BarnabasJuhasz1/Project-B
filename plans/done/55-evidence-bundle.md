# 55 — The arm-blind evidence bundle

**Epic:** [06](../06-epic-repair-loop.md) · **Depends on:** 54 · **Model calls:** no · **Needs Isaac Sim:** no

⚠️ Read [`../../REPAIR-LOOP.md`](../../REPAIR-LOOP.md), "Arm-blindness", first.

---

## What this is

Everything the diagnosing agent is allowed to see about one node, assembled into one object,
**with every trace of which arm produced it removed.**

## Deliverable

`repair/evidence.py`:

```python
bundle(node_id, rows, verdict, *, world=None) -> Bundle   # blind by construction
```

Contents:

| Part | Source | Note |
|---|---|---|
| The card, scrubbed | **`generator.card_yaml`** | mechanism / trigger / derivation / prediction / node_name — reuse it, do not write a second scrubber |
| The node source | `generate/staging/<stem>.py` | model-written Python, as text |
| The verdict | chunk 54 | label, reason, `n`, claimed-metric delta |
| Per-world outcomes | `<tier>-worlds.csv` | the rows behind the verdict, not the aggregate |
| One case, opened up | `scripts/debug_growth_case.py` | timeline, both trajectories, where each run stopped |
| The scenario axes in force | `core.worldgen.to_scenario` | ⚠️ including `drift=None` — see below |
| The port contract | `nodes/contract.py`, base's published ports | what the node *could* have read and did not |

## ⚠️ The scrub is the whole point, and it has a test behind it

The obvious tell is `source.citation` — it names the paper, which names the arm. The rest of the
leaks are quieter:

| Field | Leaks |
|---|---|
| `source.citation` | the paper, therefore the arm |
| `source.span`, `source.verified` | only the paper arm has a span to verify |
| `confidence` | arm-correlated in practice |
| `generated:` block | `condition`, `treatment`, `ref_id`, `paper_sha256` — explicit |
| the stem / filename | `b__path_integration_ants__…` begins with the arm letter |
| `node_id` | `gen_b_…` begins with the arm letter |

**The last two are the ones that will be missed.** Everything in the loop addresses nodes by id,
and the id carries the arm in plain text. The bundle refers to the node by an **opaque handle**
and the mapping is held outside the bundle.

⚠️ `generator.card_yaml` is already the correct projection and already excludes `source` and
`confidence` — it exists because the *generation* prompt had the same problem, and the card→node
table's reasoning ("they are metadata") applies unchanged. Reuse is not a shortcut here; a second
scrubber is a second thing to keep in sync.

## ⚠️ Include what was NOT varied

The bundle states the scenario axes the measurement actually used, including the ones left at
their defaults. Found 2026-09-16: `to_scenario()` never sets `drift`, so every growth world hands
the controller a **perfect pose estimate** — and the ant node's card predicts an effect
*"on scenarios where pose_estimate drifts"*. An agent shown only the geometry and the outcome
cannot possibly reach that. An agent shown `drift=None` beside a card whose first clause names
drift has a decent chance of it in one step.

This generalises: **the bundle reports the conditions of the test, not only its result.**

## Done when

- `bundle()` returns the same object for a node whether or not the caller knows its condition.
- A test asserts **no arm tell survives**: scan the rendered bundle for the condition letters in
  every position they appear (`gen_a_`, `gen_b_`, `gen_d_`, `a__`, `b__`, `d__`, `condition:`,
  `treatment:`, `citation`, every `ref_id`, every author surname in `cards/*.yaml`).
- The rendered bundle for `gen_b_44d12df4d25b437f` is committed as a fixture, so a future change
  that reopens a leak fails a diff rather than a reviewer's attention.
- The bundle names `drift=None` for a growth world.
- The bundle fits a single model call — measure it and write the token count into the Outcome.

---

## Outcome

**Built 2026-09-16.** `repair/evidence.py`, `repair/tests/test_evidence.py` (20 tests),
`repair/tests/fixtures/bundle-ant-node.md` (the committed rendered bundle).
**50 tests across `repair/`, 32 s.**

```bash
P=~/miniforge3/envs/env_isaacsim/bin/python
$P -m repair.evidence --node gen_b_44d12df4d25b437f --check-only
$P -m repair.evidence --node gen_b_44d12df4d25b437f --auto-case
```

### Size, measured

| | chars | ~tokens |
|---|---|---|
| bundle, no case | 18,274 | ~4,570 |
| bundle + one world opened up | 21,074 | ~5,270 |

Comfortably one model call. `--check-only` prints the size and the leak scan and costs milliseconds.

### The scrub

`leaks()` returns empty. The card block carries **zero** author surnames, no `citation`, no
`span`, no `verified`, no `confidence`, no `generated:` block. The node is addressed by an opaque
salted handle (`node-3cb6a0af73f4`) and the node id appears nowhere in the rendered text.

⚠️ **The salt is load-bearing.** A handle derived from the node id alone is brute-forceable —
three arms, one known plan hash, three guesses. Pinned by a test that computes the unsalted
digest and asserts it differs.

### What differed from the plan

**The plan said "reuse `generator.card_yaml`" and that was right, but not sufficient.** `card_yaml`
renders multi-line values **unindented**, so the rendered card block is not valid YAML — a prose
line containing a colon is indistinguishable from a top-level key. This is pre-existing and fine
(it is a prompt, not a document anything loads), but it means the allowlist cannot be checked by
parsing. It is checked field by field instead.

**The case block needed a shared clock.** Sampling each trace by its own index puts the two runs on
different clocks — the base stops when it reaches the goal, the pair runs to the time limit — and
the rows interleaved out of chronological order, with a `t=68.2` row printed after `t=115.0`. Both
runs are now sampled on one shared time axis and a finished run reads `(ended)`.

### What was discovered

**1. The case block shows the defect at a glance, which is the whole bet of this chunk.** On seed
2004: the base walks from (9.58, 1.73) to the goal by t = 68.4 s, while the node-augmented tree
sits at (9.5, 2.5) with `vx ≈ 0.00` and `wz ≈ 0.00` for the entire 120 s at 0.33–0.40 m clearance —
open space, nothing near it. A model that cannot reach "it froze in the open" from that table will
not reach it from anything.

**2. ⚠️ A known, deliberate, bounded leak: the node's own docstring names its paper.** The ant node
says *"Mueller & Wehner Eq. 3"*. Acceptable, for three reasons now recorded as a test rather than
left unstated:

- **A and B are pointed at the same paper**, so naming it cannot separate them — and A-vs-B is the
  headline.
- It separates {A, B} from D, which is weaker, is carried by the card's content anyway, and D is
  frozen.
- Scrubbing a model's own code comments would corrupt the artefact under diagnosis.

If it ever has to close, the fix is a scrubbed **copy** for the prompt, never an edit to the staged
file.

**3. The blindness claim is narrower than "blind" and is stated in the module docstring.** The
bundle carries no *label* of the arm. It necessarily carries the arm's *effect* — a card written
from a paper in full cites equations and page numbers, and that difference **is** the independent
variable. The claim is exactly: *the agent is never told the arm and cannot recover it from
metadata.* Not that a rich card is information-theoretically arm-free.

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| No arm tell in the rendered bundle | `leaks()` empty | **confirmed** |
| Committed fixture scanned independently | clean | **confirmed** |
| `bundle()` cannot be told the arm | no `condition`/`arm`/`treatment` parameter | **confirmed** (structural) |
| Author surnames reach the card block | none of 10 surnames across `cards/*.yaml` | **ruled out** |
| Handle is salted and not invertible | differs from the unsalted digest; two arms at one hash differ | **confirmed** |
| Bundle names the axes left at defaults | `drift: None` + "was NOT VARIED" present, and the card names drift | **confirmed** |
| Bundle fits one model call | 18.3 kB / 21.1 kB measured | **confirmed** |
| A case stays clean | rollout-generated text scanned too | **confirmed** |
| Node source names its paper | `Mueller`, `Wehner` present | **confirmed — accepted, bounded, tested** |

### ⚠️ Three of my own tests were wrong before they were right

Recorded because each was a *silent* failure mode, and two would have passed forever:

1. **A vacuous test.** The surname scan used `citation:\s*(.+)` — but `citation:` is a YAML
   **folded block** (`>-`), so the text is on the following indented lines and the regex captured
   nothing. It would have passed on an empty set. Caught only by an
   `assert surnames, "would pass vacuously"` guard written alongside it. **Every scan test wants
   that guard.**
2. **Prose read as schema.** The allowlist test parsed top-level keys and read the mechanism text
   as YAML keys (see above).
3. **A loose substring.** `"condition" in source` matched `PRECONDITION`, a contract keyword. Fixed
   by reusing `ARM_TELLS`, which is punctuated precisely for this reason — one definition, not two.

### Not checked

- **Whether a model can actually recover the arm from a rich card.** The scrub is verified against
  every *label*; the residual signal in card content is argued, not measured. Measuring it is a
  probe of its own — hand a model bundles and ask it to guess the arm — and is the natural
  follow-up if the blindness claim is ever challenged.
- The bundle has not been fed to a model. That is chunk 57.
- Only the `maze` tier and one node have been rendered.
