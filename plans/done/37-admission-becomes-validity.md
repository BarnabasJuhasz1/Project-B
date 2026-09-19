# 37 — Admission becomes a validity gate; efficacy becomes a measurement

**Status:** todo. SIM-FREE.
**Blocks:** 38 (the stand-in ablation reports the contribution this stops gating on).
**Touches a non-negotiable and a pre-registered number.** See "Sign-off" below.

## Why

The gate does two jobs and only one is defensible.

**Validity** — does this parse, import only what is allowed, declare one class,
wire into a tree and run without raising? Mechanical, cheap, scenario-independent,
and the thing that actually protects the library. The import allowlist inside it
is load-bearing for the seal.

**Efficacy in isolation** — does this node, *alone*, move a whole-task metric in
a world *it designed for itself*? This is the source of every pathology measured
so far, and on 2026-09-07 the last piece of evidence arrived:

- It **forces breadth**. Only an Action can move a metric alone, so a narrow
  mechanism can only pass by being packaged with a carrier — which is the
  two-effect violation, the orphan port and the starvation, all one cause.
- **Measured**: `ExtendedSurfaceDetour` (A, Cheng 1986) with the paper's
  boundary representation ablated **still reaches the goal** on both dev
  scenarios (SPL 0.934 / 0.951, +1 collision each). Six support layers do the
  work; the paper is one layer of seven.
- **And admission could not have caught that**: the same ablated node **fails
  its own unit scenario**, effect 0.000. The micro-world was *honest* — the
  mechanism really is load-bearing there — but it is **not representative**, and
  no improvement to micro-worlds fixes that, because the generator picks them.
- It **rejects Conditions and Decorators outright** (`KIND_NOT_TESTABLE_ALONE`),
  making two of the description's three leaf kinds unreachable. Nothing in the
  library reads a port, and this is why.
- **Half of condition D's "0 of 4" never ran a node**: two rejections are the
  harness refusing the generator's *world* (goal unreachable; goal too close to
  a wall), filed under the same reason code as node failure.

## Decided

**Demote efficacy from a gate to a measurement. Keep validity as the gate.**

Rejected: removing admission entirely. It would discard the only per-condition
quantitative comparison in the project, move `LibraryLoadError` from generation
time to composition time, and delete a pre-registered number rather than
redefining it. Redefining is honest; deleting throws away a denominator.

## The design

### The validity gate runs in a HARNESS-OWNED world

The single most important detail. A node is validated in a world
`harness/micro.py` owns and commits, **never the one the candidate built** —
otherwise a generator that writes a broken world fails a node that is fine,
which is exactly what happened to two of D's four.

`harness/micro.py` gains `validity_world()`: one committed room with a couple of
boxes, start and goal reachable, defined as data above the divider. It is not a
benchmark scenario and says so.

### What is still a rejection

Everything decidable about the candidate itself: `SYNTAX_ERROR`,
`IMPORT_DENIED`, `NO_NODE`, `MULTIPLE_NODES`, `IMPORT_TIME_ERROR`,
`NO_PREDICTION` (the dict is missing or malformed — a node must still make a
claim), `NO_MICRO`, `PROVENANCE_INCOMPLETE`, `UNSATISFIED_PORT`,
`CONTRACT_VIOLATION`, `NO_CANDIDATE_EMITTED`, `CANDIDATE_RAISED`, and the new
`INVALID_NODE` (raised while ticking the harness-owned world).

### What stops being a rejection

| was | becomes |
|---|---|
| `UNIT_SCENARIO_FAILED` | recorded: `contribution.effect`, no gate |
| `MALFORMED_MICRO` | recorded: `contribution.measured = false`, with the reason |
| `UNSCOREABLE_PREDICTION` | recorded: contribution unmeasurable from the node's own claim |
| `KIND_NOT_TESTABLE_ALONE` | **gone** — a Condition or Decorator is valid or not, like anything else. A Decorator is validated wrapping a no-op stand-in child. |

### What gets recorded on every candidate

Additive keys, so older records simply lack them:

- `validity`: passed / the reason
- `contribution`: `{measured, metric, candidate, baseline, effect, required}` —
  `required` kept as a **reporting line**, not a threshold
- `precondition_fired`: ticks and the fraction its precondition passed in the
  harness world. Sharply diagnostic — measured 0/6000 for
  `LimitSpeedToOcclusionBoundary` against 6000/6000 for `ReorientToGeometry`.

## Re-judging the existing 21 records

**The old verdicts are not rewritten.** They are re-judged deterministically from
committed sources — no model calls — and **both** verdicts are stored side by
side (`admitted` stays; `admitted_v2` and `validity` are added), so the old
denominator survives and the change is auditable rather than retroactive.

⚠️ **Expected effect: D goes from 0 admitted to as many as 4, and the library
from 7 nodes to as many as 11.** This must be reported as a **redefinition of
what admissible means**, never as condition D improving. The plan's Outcome says
so and so must the write-up.

## Build

1. `harness/micro.py`: `validity_world()`, committed as data.
2. `harness/unit.py`: `run_validity(tree_factory, world, ticks)` -> `ValidityResult`.
   `run_unit` keeps computing the effect and stops being the decision.
3. `generate/admit.py`: `admit()` decides on validity; records contribution and
   `precondition_fired`. `build_tree` accepts a Condition or Decorator.
4. `generate/admit.py:pass_rate()` reports **both** rates per condition:
   `validity_rate` and (over rows where it was measured) `contribution_rate`.
5. Tests: a narrow node that moves nothing is ADMITTED and recorded as
   zero-contribution; a node that raises mid-tick is REJECTED; a Condition is
   ADMITTED; a candidate whose own micro-world is malformed is ADMITTED with
   `contribution.measured=false`; the harness-owned world is used, not the
   candidate's.
6. `scripts/rejudge.py`: re-judge every staged record under both rules, print the
   diff, write `admitted_v2` back. Idempotent, no model calls.

## Sign-off REQUIRED before anything is reported

Two documents change meaning and **neither is edited without the user saying so**:

- **Non-negotiable #5** in `CLAUDE.md` — "A node is admissible only if it passes
  its unit scenario in isolation" becomes admissible-on-validity, with the unit
  scenario reported rather than gating.
- **`PRE-REGISTRATION.md`** — G1 currently reads "≥20 admissible behaviour-tree
  nodes, each passing an isolated unit scenario", and reports a unit pass rate.
  That becomes a validity rate plus a contribution distribution. A **dated
  append**, never an edit. `MIN_EFFECT_RATE` / `MIN_EFFECT_REL` stop being
  thresholds and become reporting lines.

Consequence to state in the append: **G1's coverage claim then rests entirely on
the distinct-mechanism count and on tree contribution (chunk 38), not on the node
count**, because reaching 20 becomes much easier.

## Known cost, accepted

A node that ticks harmlessly and does nothing is now admitted. It is inert, it
occupies a retrieval slot, and its contribution reads zero. This is the right
error to make — far better than rejecting real mechanisms for not being
standalone navigators — but it is a real change in what the library means.

## Done when

- Validity gates; efficacy is recorded.
- Conditions and Decorators can be admitted.
- The 21 records carry both verdicts and the diff is reported.
- The full sim-free suite is green.
- The PRE-REGISTRATION append and the #5 rewording are **drafted and awaiting
  sign-off**, not landed.

---

## Outcome — 2026-09-07

Built and executed. **The code is done; the two document changes are drafted
below and NOT landed.**

### What was built

- `harness/micro.py`: `validity_scenario()` — a committed, harness-owned world.
- `harness/unit.py`: `run_validity()` -> `ValidityResult`, plus a docstring
  section saying why the three requirements stopped gating. `run_unit` is
  unchanged and still computes the effect.
- `generate/admit.py`: validity is the gate; `_measure_contribution()` records
  the rest and **cannot reject anything**. `build_tree` now accepts a Condition
  or a Decorator (given a fixture child to govern). `pass_rate()` reports the
  validity rate, the legacy rate and the contribution rate side by side.
- `scripts/rejudge.py`: re-judges every staged record under both rules, prints
  the diff, writes `admitted_v2` **additively**. No model calls, idempotent.
- `compose/library.py`: `admitted_stems` reads `admitted_v2` where present, so
  the library and the number describing it stay about the same set.

### The finding that matters most, and it is not the one the plan predicted

**Validity admits 14 of 14. Every candidate ever generated runs.**

| condition | validity (new) | legacy (old) | contribution |
|---|---|---|---|
| A | 5/5 = **1.00** | 4/5 = 0.80 | 4/5 = 0.80 |
| B | 5/5 = **1.00** | 3/5 = 0.60 | 3/4 = 0.75 |
| D | 4/4 = **1.00** | 0/4 = 0.00 | 0/2 = 0.00 |

The plan predicted "hitting 20 becomes much easier". It is stronger than that:
**the validity gate discriminates nothing on this corpus, so G1's pass rate
under the new rule is a constant 1.00 and carries no information.** The gate is
a floor that catches a node which crashes — `test_a_node_that_raises_mid_rollout_fails_VALIDITY`
proves it fires — and no real candidate has ever crashed. That is itself worth
reporting: **models reliably generate code that runs; what they do not reliably
generate is code that helps.**

So the plan's caveat is the whole story, not a footnote: **G1's coverage claim
now rests entirely on the distinct-mechanism count and on chunk 38's tree
contribution.** The node count is close to free.

**The contribution rate is the number that survives, and its denominators are
now honest.** D is 0 of **2**, not 0 of 4 — the two whose contribution could not
be measured were the harness refusing the generator's world, and they have left
the denominator instead of being counted as node failures. B is 3 of 4 for the
same reason (the `drift=0.6` candidate is unmeasurable, not failing).

### Counts

- **Condition D has 4 admitted nodes for the first time.** Chunk 29's "Done
  when" — a non-zero admitted count for every condition — is now met, **by
  redefinition, not by D improving.** Any write-up that does not say so is
  wrong.
- Library **12 -> 19 entries**, `lib-012-9a766931282c` -> `lib-019-ed0c5cc44084`.
  By origin: card 5, citation 5, invented 4, handwritten 5.
- Two previously-rejected A/B nodes rejoin: `DynamicWindowDrive` (A/dwa),
  `MaintainHomingVector` and `ArbitrateHomingVector` (B).

### Checks

| Check | Outcome | Verdict |
|---|---|---|
| Does validity use the harness world, not the candidate's? | A candidate whose `micro()` raises outright is ADMITTED with `contribution.measured=false`; pinned by a test | confirmed — the D failure mode is fixed at the source |
| Does the gate still catch a broken node? | A node raising mid-tick is rejected `INVALID_NODE`, with the exception in the reason | confirmed |
| Are the specific rejection reasons preserved? | `tree_factory()` is called before `run_validity`, so `UNSATISFIED_PORT` and `CONTRACT_VIOLATION` keep their codes rather than collapsing into `INVALID_NODE` | **regression found during the change and fixed**; two tests pin it |
| Is the traceback still kept when a measurement blows up? | Yes, in `contribution.reason` | **regression found and fixed** |
| Can a Condition be admitted? | Yes. So can a Decorator, given a fixture child | confirmed, both pinned |
| Does the firing rate separate "never fired" from "did not help"? | A node whose precondition is always false: ticks>0, passed=0, drove=0 | confirmed, pinned |
| Are the old verdicts preserved? | `rejudge.py` writes `admitted_v2` additively; `admitted`/`reason` untouched; `pass_rate` reports both | confirmed |
| Was the exclusion list read from the right place? | **No, at first.** `retired_reason` lives in the RESULT metadata, not the candidate sidecar, so the first run counted 19 records instead of 14 and silently included four retired naive-arm records | **bug found and fixed before `--apply`** |
| Did growing the library by 7 nodes break the committed compositions? | **No.** All compose tests pass | the reconstruction work earlier the same day did exactly what it was built for — this is the standing cost eliminated, demonstrated under a 7-node growth |
| Full sim-free suite | **757 passed, 1 failed**, 235 s | the 1 is the pre-existing `no_papers` port-name gap |
| Is that failure new? | No — same root cause, different node: `MaintainHomingVector` declares `home_bearing`, and the test's `contracts` set omits port names while `render_node` renders them. Counting port names as contract clears every card to zero | pre-existing, still unfixed by choice |
| Behaviour under physics | **Not checked.** Nothing here ran a simulator | outstanding — chunk 36 |
| Is the contribution rate stable? | **Not checked and not checkable.** n=1 per cell | inconclusive by construction, FINDINGS #3 |
| Import allowlist | Not weakened, not extended | confirmed |
| Leakage | No sealed file opened; every grep named its directories | chunk 11's rule honoured |

### ⚠️ AWAITING SIGN-OFF — drafted, not landed

**1. `CLAUDE.md` non-negotiable #5.** Currently:

> **A node is admissible only if it passes its unit scenario in isolation.** The
> unit scenario is a purpose-built micro-world that isolates one node's claimed
> effect and tests it against a fixed baseline. Pass or fail, decided
> automatically.

Proposed:

> **A node is admissible only if it runs.** Validity — it parses, imports only
> what the allowlist permits, wires into a tree and ticks a harness-owned world
> without raising — is decided automatically and is the gate. Its **unit
> scenario still runs and is still recorded**, as a contribution measurement
> against a fixed baseline, but it no longer decides admission: the micro-world
> is designed by the generator, and a mechanism can be ablated out of a node
> that passes it while the node still solves real scenarios. Whether a node
> *helps* is measured in a tree, over scenarios the generator did not design.

**2. `PRE-REGISTRATION.md` — a DATED APPEND, never an edit.** G1 currently reads
"≥20 admissible behaviour-tree nodes, each passing an isolated unit scenario"
and reports a unit pass rate. The append must state:

- what admissible now means, and that the reported rate is a **validity** rate;
- that `MIN_EFFECT_RATE` / `MIN_EFFECT_REL` are now **reporting lines, not
  thresholds**;
- that the old rate is retained per condition as `legacy_pass_rate`, computed
  from untouched records, so the redefinition is auditable;
- that **D's rise from 0 to 4 is a redefinition and not a result**;
- that **the validity rate is 1.00 for every condition**, so G1's coverage claim
  rests on the distinct-mechanism count and on tree contribution;
- that G2's decision rule, X, the seed set, the episode count, the sealed split
  and the secondary metrics are **untouched**.
