"""repair/verdict.py — **DOES THIS NODE WORK?** The taxonomy, as code.

`classify()` IS the rule. The prose in `REPAIR-LOOP.md` is a description of this
function, the same way `PRE-REGISTRATION.md` describes `core.stats.decide()`.

⚠️ **THIS MODULE NEVER TAKES A MODEL CALL.** It is a pure function over
measurements that already exist. The diagnosing agent (chunk 57) proposes a
CAUSE and an ACTION; it does not get to declare the verdict. An agent that both
diagnoses and grades is marking its own homework, and the taxonomy stops being
falsifiable the moment it can.

⚠️ **A WORSE NUMBER IS NOT A BROKEN NODE.** Four distinct situations produce
one, and a loop that cannot tell them apart is a random walk with a model call
in it:

  * the node stopped the robot doing the task            -> DEGENERATE
  * the node had nothing to do here                      -> INERT
  * the node spent a metric it never claimed             -> MISJUDGED
  * the node is fine and the paper does not transfer     -> not decided here

The last one is a statement about a SEARCH having been exhausted, not about one
measurement, so `UNTESTABLE` and `NOT_TRANSFERABLE` are loop-level terminal
states (chunk 58) and are deliberately absent from `Label`.

Usage — reproduce the committed growth table's verdicts:

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P -c "from repair.verdict import classify_node; \
           print(classify_node('gen_b_44d12df4d25b437f'))"

    $P -m repair.verdict --tier maze          # every node in out/growth/maze.csv
"""
from __future__ import annotations

import csv
import re
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# ⚠️ PRECEDENCE IS DECLARED HERE, BEFORE THE FIRST RUN. Several of these
# conditions can hold on the same node at once, and a precedence chosen after
# seeing a tie is a story about whoever ran the loop -- the same argument chunk
# 44 makes about its own tie-break. The order in `Label` IS the precedence and
# `classify()` walks it top to bottom.


class Label(str, Enum):
    """The six verdicts one measurement can produce, in precedence order."""

    INERT = "INERT"
    DEGENERATE = "DEGENERATE"
    HELPED = "HELPED"
    MISJUDGED = "MISJUDGED"
    REGRESSED = "REGRESSED"
    ACCEPT = "ACCEPT"


# A node that never acted cannot have caused anything, and every downstream
# comparison on it is noise -- which is why INERT is checked FIRST rather than
# last. Round 2's library split into "never fires" (4), "fires and changes
# nothing" (3) and "fires ~100% and changes ~100%" (6), so BOTH floors are
# load-bearing: `fire` alone reads a node that fires on every tick and revises
# nothing as active.
FIRE_FLOOR = 0.05          # fraction of ticks the precondition passed
CHANGED_FLOOR = 0.01       # fraction of ticks the command actually moved

# "The robot stopped doing the task." Not a judgement call: a node is
# DEGENERATE when it lost a world the base won, did NOT lose it by crashing,
# and shows the signature of having stopped.
DEGENERATE_STUCK_RATIO = 1.5    # pair stuck_events / base, above which it stalled
DEGENERATE_PATH_RATIO = 0.75    # pair path_length / base, below which it stopped short

# Task integrity gates HELPED and MISJUDGED. ⚠️ It is a PRECONDITION, not a
# bonus. Without it, "improved its claimed metric" and "the robot stopped" are
# the same row: growth round 2 produced a node that cut collisions 0.40 -> 0.30
# while walking 6.2 m instead of 30.0 and arriving in 0 of 20 worlds.
INTEGRITY_PATH_RATIO = 0.75
INTEGRITY_STUCK_RATIO = 1.5

# A card that states no floor still has to clear something, or any noise counts
# as a win. Mirrors `harness.unit`'s two-regime structure, because the EFFECT
# SIZE has two regimes (see `_improvement`): absolute for a rate, relative for
# everything else.
#
# ⚠️ THESE ARE NOT `harness.unit`'s FLOORS AND MUST NOT BE. `MIN_EFFECT_RATE` is
# 0.5, calibrated for a MICRO-WORLD whose baseline was built to fail outright —
# there, "+0.5 goal-reach" means "the node solves what the baseline could not".
# On a shared suite where the base already reaches 50%, the same number means
# "solve every remaining world", which is a different and far harder demand. The
# micro-world effect floor was SCRAPPED as an admission criterion by chunk 37/44
# and importing it here would quietly reinstate it in a context it was never
# calibrated for.
FLOOR_RATE = 0.10          # absolute, for goal_reached
FLOOR_REL = 0.10           # relative, for every other metric

# A card may demand MORE than the floor, never less — the same rule
# `harness.unit.required_effect` applies, for the same reason: the generator
# writes the card, so a generator-supplied floor of 0.0 is the generator
# marking its own homework.
DEFAULT_MIN_EFFECT = FLOOR_REL

# The primary metric. `goal_reached` stays primary even though every count
# metric is also read per metre -- per-metre alone flatters a node that crawls
# safely and never arrives (METRICS.md section 2).
PRIMARY = "goal_reached"

# ⚠️ COUNT METRICS ARE COMPARED PER METRE TRAVELLED, ALWAYS. These three are
# lower-is-better counts, so a robot that stops scores perfectly on all of
# them. The raw count never decides a verdict on its own. METRICS.md carries
# the rule and the case that produced it.
PER_METRE = ("collisions", "near_misses", "stuck_events")

# Which way is better. Mirrors scripts/measure_growth.REPORTED_METRICS; a
# metric absent here cannot be claimed, which is deliberate -- an unknown
# direction is not a claim, it is a typo.
# Which metrics are RATES, and therefore compared by absolute difference rather
# than relative to the baseline. Restated from `harness.unit.RATE_METRICS` at
# import time below, never hard-coded twice.
RATE_REGIME = frozenset({"goal_reached"})

DIRECTION = {
    "goal_reached": "higher",
    "collisions": "lower",
    "near_misses": "lower",
    "min_clearance": "higher",
    "stuck_events": "lower",
    "time_to_goal": "lower",
    "path_length": "lower",
    "spl": "higher",
}

GROWTH_DIR = REPO_ROOT / "out" / "growth"


# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Claim:
    """What the node's own card says it moves, and by how much.

    ⚠️ Each arm is scored against ITS OWN generated card's prediction. The
    asymmetry where B was scored against a human card is gone, because the
    human card no longer reaches a prompt.
    """

    metric: Optional[str]
    min_effect: float
    direction: Optional[str]
    source: str                     # "card" | "node" | "missing"

    @property
    def stated(self) -> bool:
        return self.metric is not None and self.direction is not None

    @property
    def floor(self) -> float:
        """What this node actually has to clear: its own demand, or the floor
        for the metric's regime, whichever is larger."""
        base = FLOOR_RATE if self.metric in RATE_REGIME else FLOOR_REL
        return max(base, float(self.min_effect))

    @property
    def relative(self) -> bool:
        """Is `min_effect` a fraction of the baseline rather than an absolute
        difference? True for everything except a rate metric."""
        return self.metric not in RATE_REGIME


@dataclass(frozen=True)
class WorldOutcome:
    """One (node x world) row of `<tier>-worlds.csv`."""

    seed: int
    scenario_id: str
    base_reached: bool
    pair_reached: bool
    outcome: str                    # fixed | broke | both_ok | both_fail
    fire: float
    changed: float


@dataclass(frozen=True)
class Measurement:
    """Everything `classify()` is allowed to look at, for one node.

    ⚠️ Carries no condition letter and no citation. The node id does carry the
    arm (`gen_b_...`) and that is fine HERE -- this module is not the blind
    component. Chunk 55's bundle is, and a `Verdict` is built to be safe to put
    in it: label, reason, n, metric, delta, and nothing else.
    """

    node_id: str
    tier: str
    worlds: tuple
    metrics: dict                   # metric -> (base, pair) means, either may be None
    fire: float
    changed: float

    @property
    def n(self) -> int:
        return len(self.worlds)

    def pair_of(self, metric):
        return self.metrics.get(metric, (None, None))

    def counts(self):
        c = {"fixed": 0, "broke": 0, "both_ok": 0, "both_fail": 0}
        for w in self.worlds:
            if w.outcome in c:
                c[w.outcome] += 1
        return c


@dataclass(frozen=True)
class Verdict:
    """A label, why it was reached, and the sample size behind it.

    ⚠️ `n` is not optional. A single-seed result that looked like a finding in
    chunk 22a did not survive five seeds, and the D node's own verdict FLIPS
    between two world sets of twenty (see the module's __main__).
    """

    node_id: str
    label: Label
    reason: str
    n: int
    tier: str
    claim_metric: Optional[str] = None
    claim_delta: Optional[float] = None      # signed, POSITIVE MEANS BETTER
    claim_min_effect: Optional[float] = None
    per_metre: bool = False
    counts: dict = field(default_factory=dict)
    unchecked: tuple = ()                    # checks that could not be made

    def __str__(self):
        head = f"{self.node_id}  {self.label.value}  (n={self.n}, {self.tier})"
        return f"{head}\n    {self.reason}"


# ---------------------------------------------------------------------------
# the rule


def _ratio(pair, base):
    """pair/base, or None when base is zero or either side is missing."""
    if pair is None or base is None or abs(base) < 1e-12:
        return None
    return pair / base


def _improvement(metric, base, pair):
    """Signed improvement in `metric`. POSITIVE MEANS BETTER, whichever way the
    metric points. None when either side was never observed.

    ⚠️ **DELEGATES TO `harness.unit.effect`, WHICH IS THE ONE DEFINITION OF
    EFFECT SIZE IN THIS REPO.** It is NOT a plain difference: absolute for a
    rate metric (`goal_reached`), and RELATIVE TO THE BASELINE for every other.
    That distinction is load-bearing here — four of the sixteen committed nodes
    claim `collisions` with a `min_effect` of 0.3-0.5, which reads as an
    impossible demand against a base that collides 0.40 times per episode and
    as an ordinary one once it is read as "cut them by 30-50%", which is what
    the card meant and what the harness has always computed.
    """
    if base is None or pair is None or metric not in DIRECTION:
        return None
    from harness.unit import effect
    return effect(metric, pair, base)


def _claim_improvement(m: Measurement, claim: Claim):
    """The improvement in the node's OWN claimed metric, per metre where the
    metric is a count.

    ⚠️ The per-metre form is not a second opinion, it is THE comparison for a
    count metric. A lower-is-better count is not comparable across controllers
    with different arrival rates, and the one apparent win it produced reversed
    sign under it.
    """
    if not claim.stated:
        return None, False
    base, pair = m.pair_of(claim.metric)
    if claim.metric not in PER_METRE:
        return _improvement(claim.metric, base, pair), False

    b_path, p_path = m.pair_of("path_length")
    if base is None or pair is None or not b_path or not p_path:
        return _improvement(claim.metric, base, pair), False
    return _improvement(claim.metric, base / b_path, pair / p_path), True


def task_integrity(m: Measurement):
    """Is the robot still doing the task at all?

    Deliberately independent of the primary metric, because MISJUDGED is
    DEFINED as "the primary got worse" -- a definition that also demanded the
    primary hold up would be empty. What it tests is the SIGNATURE OF HAVING
    STOPPED: distance travelled collapsed, or time spent stuck rose.

    Returns (ok, reason, unchecked).
    """
    unchecked = []
    b_path, p_path = m.pair_of("path_length")
    b_stuck, p_stuck = m.pair_of("stuck_events")

    path_r = _ratio(p_path, b_path)
    stuck_r = _ratio(p_stuck, b_stuck)

    if path_r is None:
        unchecked.append("path_length missing: could not test whether it still travels")
    elif path_r < INTEGRITY_PATH_RATIO:
        return False, (f"travelled {path_r:.2f}x the base's distance "
                       f"(floor {INTEGRITY_PATH_RATIO})"), tuple(unchecked)

    if stuck_r is None:
        unchecked.append("stuck_events missing: could not test whether it stalled")
    elif stuck_r > INTEGRITY_STUCK_RATIO:
        return False, (f"stuck {stuck_r:.2f}x as often as the base "
                       f"(ceiling {INTEGRITY_STUCK_RATIO})"), tuple(unchecked)

    return True, "still travelling and not stalling", tuple(unchecked)


def _degenerate(m: Measurement):
    """Did the node stop the robot doing the task, without crashing it?

    Three things together, and all three are needed:

      1. it LOST a world the base WON -- `broke` > 0. A node that only ever
         failed where the base already failed has not broken anything.
      2. it did NOT lose by crashing. A node that collides more is a different
         defect with a different repair, and calling both DEGENERATE would send
         the loop after the wrong cause.
      3. it shows the signature of having STOPPED: stuck_events up, or distance
         travelled collapsed.

    ⚠️ The per-world rows carry no path length, so 2 and 3 are read from the
    AGGREGATE while 1 is per-world. Said out loud rather than dressed up: this
    is a whole-node judgement with one per-world precondition, and the reason
    string says which worlds triggered it.
    """
    broke = [w for w in m.worlds if w.outcome == "broke"]
    if not broke:
        return None, ()

    unchecked = []
    b_col, p_col = m.pair_of("collisions")
    if b_col is None or p_col is None:
        unchecked.append("collisions missing: could not rule out crashing")
    elif p_col > b_col + 1e-9:
        return None, tuple(unchecked)       # it crashed; that is REGRESSED, not this

    b_path, p_path = m.pair_of("path_length")
    b_stuck, p_stuck = m.pair_of("stuck_events")
    path_r = _ratio(p_path, b_path)
    stuck_r = _ratio(p_stuck, b_stuck)

    stalled = stuck_r is not None and stuck_r >= DEGENERATE_STUCK_RATIO
    short = path_r is not None and path_r <= DEGENERATE_PATH_RATIO
    if stuck_r is None and path_r is None:
        unchecked.append("path_length and stuck_events both missing: "
                         "could not test for the stopped signature")
        return None, tuple(unchecked)

    if not (stalled or short):
        return None, tuple(unchecked)

    seeds = ", ".join(str(w.seed) for w in broke[:5])
    more = f" (+{len(broke) - 5} more)" if len(broke) > 5 else ""
    how = []
    if stalled:
        how.append(f"stuck {stuck_r:.2f}x as often")
    if short:
        how.append(f"travelled {path_r:.2f}x the distance")
    return (f"lost {len(broke)} world(s) the base won [seeds {seeds}{more}] "
            f"without colliding more, and {' and '.join(how)}"), tuple(unchecked)


def classify(m: Measurement, claim: Claim) -> Verdict:
    """The rule. Walks `Label` in declared precedence order.

    ⚠️ `UNTESTABLE` and `NOT_TRANSFERABLE` are NOT produced here. Both are
    statements about a search having been exhausted, which one measurement
    cannot support. Chunk 58 owns them.
    """
    unchecked = []
    counts = m.counts()

    def out(label, reason, delta=None, per_metre=False):
        return Verdict(node_id=m.node_id, label=label, reason=reason, n=m.n,
                       tier=m.tier, claim_metric=claim.metric,
                       claim_delta=delta, claim_min_effect=claim.floor,
                       per_metre=per_metre, counts=counts,
                       unchecked=tuple(unchecked))

    # 1. INERT ---------------------------------------------------------------
    # First, because a node that never acted cannot have caused anything.
    if m.fire < FIRE_FLOOR:
        return out(Label.INERT,
                   f"fired on {m.fire:.1%} of ticks (floor {FIRE_FLOOR:.0%}) — "
                   f"nothing here triggered it")
    if m.changed < CHANGED_FLOOR:
        return out(Label.INERT,
                   f"fired on {m.fire:.1%} of ticks but changed the command on "
                   f"{m.changed:.2%} (floor {CHANGED_FLOOR:.0%}) — it acted and "
                   f"did nothing")

    # 2. DEGENERATE ----------------------------------------------------------
    why, deg_unchecked = _degenerate(m)
    unchecked.extend(deg_unchecked)
    if why:
        return out(Label.DEGENERATE, why)

    # the claim, and whether the robot is still doing the task ----------------
    delta, per_metre = _claim_improvement(m, claim)
    if not claim.stated:
        unchecked.append("the node states no claimed metric: "
                         "HELPED and MISJUDGED cannot be reached")
    elif delta is None:
        unchecked.append(f"claimed metric {claim.metric!r} was never observed "
                         f"on both sides: HELPED and MISJUDGED cannot be reached")

    integrity_ok, integrity_why, integrity_unchecked = task_integrity(m)
    unchecked.extend(integrity_unchecked)

    b_prim, p_prim = m.pair_of(PRIMARY)
    prim_delta = _improvement(PRIMARY, b_prim, p_prim)
    primary_worse = prim_delta is not None and prim_delta < -1e-9
    if prim_delta is None:
        unchecked.append(f"{PRIMARY} was never observed on both sides")

    unit = "/m" if per_metre else ""
    # ⚠️ Say which arithmetic produced the number. A bare "+0.25 (floor 0.5)"
    # invites reading a relative effect as an absolute one.
    pct = " relative" if claim.stated and claim.relative else ""
    cleared = delta is not None and delta >= claim.floor

    # 3. HELPED --------------------------------------------------------------
    if cleared and integrity_ok and not primary_worse:
        return out(Label.HELPED,
                   f"moved its own claimed metric {claim.metric}{unit} by "
                   f"{delta:+.4f} (floor {claim.floor:g}{pct}) while {integrity_why}",
                   delta, per_metre)

    # 4. MISJUDGED -----------------------------------------------------------
    # Its claim held and the PRIMARY did not. That is the trade the card
    # declared, not a defect -- so the loop escalates a METRIC question, not a
    # node repair.
    if cleared and integrity_ok and primary_worse:
        return out(Label.MISJUDGED,
                   f"{PRIMARY} fell {prim_delta:+.4f}, but its own claimed "
                   f"metric {claim.metric}{unit} moved {delta:+.4f} "
                   f"(floor {claim.floor:g}{pct}) while {integrity_why} — judged "
                   f"on a claim it never made",
                   delta, per_metre)

    # ⚠️ THE TAXONOMY'S HARD EDGE. "The goal was unreachable without a
    # collision and the node correctly refused" needs to know the goal was
    # unreachable safely, and nothing computes that. The core version does not
    # guess: it says which check it could not make, so chunk 57 inherits the
    # ambiguity rather than a confident wrong label.
    if cleared and not integrity_ok:
        unchecked.append(
            f"claimed metric cleared its floor ({delta:+.4f}{unit}) but task "
            f"integrity failed: {integrity_why}. Whether stopping was CORRECT "
            f"here (an unreachable-without-collision goal) is not computable — "
            f"no safe-corridor measure exists")

    # 5. REGRESSED -----------------------------------------------------------
    if primary_worse:
        return out(Label.REGRESSED,
                   f"{PRIMARY} fell {prim_delta:+.4f} with no claim covering it"
                   + (f" (claimed {claim.metric}{unit} moved "
                      f"{delta:+.4f}, floor {claim.floor:g}{pct})"
                      if delta is not None else ""),
                   delta, per_metre)

    # 6. ACCEPT --------------------------------------------------------------
    return out(Label.ACCEPT,
               "acted, harmed nothing measurable, and did not clear its own "
               "claim" + (f" ({claim.metric}{unit} moved {delta:+.4f}, floor "
                          f"{claim.min_effect})" if delta is not None else ""),
               delta, per_metre)


# ---------------------------------------------------------------------------
# loading it from what is already on disk
#
# `classify()` is the rule and takes a `Measurement`. Everything below only
# builds one, from files `scripts/measure_growth.py` already writes. Kept
# separate so the rule can be tested without a CSV and a CSV format change
# cannot quietly alter a verdict.


def claim_for(node_id, staging=None, cards_dir=None) -> Claim:
    """What this node's own card claims, resolved card-first.

    ⚠️ The CARD is the claim; the node's `PREDICTION` is the card's claim
    restated by the model that wrote the node. They can disagree, and when they
    do the card wins — `REPAIR-LOOP.md` judges a node against "the card's own
    Prediction". The node is the fallback because a candidate can exist before
    anyone has matched it to a card file.

    ⚠️ `cards.store` REFUSES to load `generate/cards/` and that refusal is
    deliberate — a model-written card is never a principle card and no span in
    it is ever counted as traceable. So this reads the YAML directly and takes
    only the prediction block. It does not, and must not, make a generated card
    loadable as a real one.
    """
    from generate.generator import STAGING

    staging = Path(staging or STAGING)
    cards_dir = Path(cards_dir or REPO_ROOT / "generate" / "cards")

    stem = _stem_for(node_id, staging)
    if stem is not None:
        arm = stem.split("__")[0]
        card = cards_dir / arm / f"{stem}.yaml"
        if card.exists():
            got = _prediction_from_yaml(card.read_text(encoding="utf-8"))
            if got.get("metric"):
                return _claim(got, "card")

        node = staging / f"{stem}.py"
        if node.exists():
            got = _prediction_from_python(node.read_text(encoding="utf-8"))
            if got.get("metric"):
                return _claim(got, "node")

    return Claim(metric=None, min_effect=DEFAULT_MIN_EFFECT,
                 direction=None, source="missing")


def _claim(got, source) -> Claim:
    metric = got.get("metric")
    try:
        min_effect = float(got.get("min_effect", DEFAULT_MIN_EFFECT))
    except (TypeError, ValueError):
        min_effect = DEFAULT_MIN_EFFECT
    return Claim(metric=metric, min_effect=min_effect,
                 direction=DIRECTION.get(metric), source=source)


def lineage_root(node_id):
    """The node a repair lineage started from — every `__rNN` stripped.

    ⚠️ Defined HERE, at the bottom of the import graph, because `verdict` is
    imported by `evidence` which is imported by `loop`, and all three need the
    same answer. `repair.evidence` re-exports it.
    """
    return node_id.split("__r")[0]


def _stem_for(node_id, staging):
    """`gen_b_44d12…` -> `b__path_integration_ants__44d12…`.

    The node id is `gen_<arm>_<plan_hash>` and the stem is
    `<arm>__<task>__<plan_hash>`, so the hash identifies the row and the arm
    disambiguates nothing — but matching on BOTH is what stops a hash prefix
    collision silently resolving to another arm's node.

    ⚠️ RESOLVED FROM THE LINEAGE ROOT. A repaired node is not in
    `generate/staging/` and never will be, so `gen_b_…__r01` split on "_" gave
    `digest="r01"`, matched nothing, and `claim_for` returned
    `Claim(metric=None, source="missing")`. The consequence was not small:
    **`HELPED` and `MISJUDGED` are unreachable without a claim**, so the only
    verdict that terminates the loop on success could never be produced for
    anything the loop repaired.

    Resolving from the root is correct by construction, not a convenience:
    **the card is not regenerated** and is reused byte for byte, so a repair's
    claim IS its parent's claim.
    """
    parts = lineage_root(node_id).split("_")
    if len(parts) < 3 or parts[0] != "gen":
        return None
    arm, digest = parts[1], parts[-1]
    for path in sorted(Path(staging).glob(f"{arm}__*__{digest}.py")):
        return path.stem
    return None


def _prediction_from_yaml(text):
    """The `prediction:` block, without importing a YAML parser or the loader
    that refuses this directory. Only four scalar keys are wanted."""
    block = re.search(r"^prediction:\s*$(.*?)^\S", text, re.S | re.M)
    body = block.group(1) if block else text
    got = {}
    for key in ("metric", "min_effect", "baseline", "cost_metric"):
        found = re.search(rf"^\s+{key}:\s*(\S+)\s*$", body, re.M)
        if found:
            got[key] = found.group(1).strip().strip("'\"")
    return got


def _prediction_from_python(text):
    """The staged module's `PREDICTION` dict, read as text.

    Deliberately NOT by importing the module: `generate/admit.py` executes
    model-written Python behind an AST allowlist, and a verdict has no business
    doing that. Same reason `measure_growth.entry_metric` uses a regex.
    """
    block = re.search(r"PREDICTION\s*=\s*\{(.*?)\}", text, re.S)
    if not block:
        return {}
    body = block.group(1)
    got = {}
    for key in ("metric", "min_effect", "baseline", "cost_metric"):
        found = re.search(rf'["\']{key}["\']\s*:\s*([^,\n}}]+)', body)
        if found:
            got[key] = found.group(1).strip().strip("'\"")
    return got


def load_measurement(node_id, tier="maze", worlds_csv=None, metrics_csv=None):
    """Build a `Measurement` from `measure_growth.py`'s committed output.

    `worlds_csv` is `<tier>-worlds.csv` (one row per node x world) and
    `metrics_csv` is `<stem>-metrics.csv` (one row per node x metric). The
    second is optional: without it only INERT and the primary-metric branches
    can be reached, and `Verdict.unchecked` says so rather than a metric
    silently reading as absent.
    """
    worlds_csv = Path(worlds_csv or GROWTH_DIR / f"{tier}-worlds.csv")
    rows = []
    with open(worlds_csv, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            if r["node_id"] != node_id:
                continue
            rows.append(WorldOutcome(
                seed=int(r["seed"]),
                scenario_id=r["scenario_id"],
                base_reached=r["base_reached"] == "1",
                pair_reached=r["pair_reached"] == "1",
                outcome=r["outcome"],
                fire=float(r["fire"] or 0.0),
                changed=float(r["changed"] or 0.0),
            ))
    if not rows:
        raise KeyError(f"{node_id} has no rows in {worlds_csv}")

    metrics = {}
    path = _find_metrics_csv(tier) if metrics_csv is None else Path(metrics_csv)
    if path is not None and path.exists():
        with open(path, newline="", encoding="utf-8") as fh:
            for r in csv.DictReader(fh):
                if r["node_id"] != node_id or r["metric"].endswith("_per_metre"):
                    continue
                # ⚠️ `_per_metre` rows are SKIPPED on purpose. Not every
                # committed run has them (the rule postdates the first run
                # that needed it), so `_claim_improvement` recomputes the
                # ratio from path_length and every run is read the same way.
                base = float(r["base"]) if r["base"] else None
                pair = float(r["pair"]) if r["pair"] else None
                metrics[r["metric"]] = (base, pair)

    n = len(rows)
    return Measurement(
        node_id=node_id, tier=tier, worlds=tuple(rows), metrics=metrics,
        fire=sum(w.fire for w in rows) / n,
        changed=sum(w.changed for w in rows) / n,
    )


def _find_metrics_csv(tier):
    """The `--metrics` sidecar for `tier`, if one was ever written.

    ⚠️ It is not named after the tier — it is named after the RUN
    (`--name`), so `maze-metrics-run-metrics.csv` belongs to the same 20
    worlds as `maze.csv` and nothing in the filename says so. Checked by
    diffing the two summaries, not assumed.
    """
    exact = GROWTH_DIR / f"{tier}-metrics.csv"
    if exact.exists():
        return exact
    found = sorted(GROWTH_DIR.glob(f"{tier}*-metrics.csv"))
    return found[0] if found else None


def classify_node(node_id, tier="maze", **kw) -> Verdict:
    """Load and classify in one call. The ordinary entry point."""
    return classify(load_measurement(node_id, tier=tier, **kw),
                    claim_for(node_id))


def classify_tier(tier="maze", **kw):
    """Every node in `<tier>-worlds.csv`, in file order. BASE_ALONE is skipped:
    it is the comparison, not a candidate."""
    worlds_csv = Path(kw.pop("worlds_csv", None) or GROWTH_DIR / f"{tier}-worlds.csv")
    seen, out = [], []
    with open(worlds_csv, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            nid = r["node_id"]
            if nid == "BASE_ALONE" or nid in seen:
                continue
            seen.append(nid)
    for nid in seen:
        out.append(classify(
            load_measurement(nid, tier=tier, worlds_csv=worlds_csv, **kw),
            claim_for(nid)))
    return out


def _main(argv=None):
    import argparse
    import collections

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--tier", default="maze")
    p.add_argument("--node", default=None, help="one node id, else the whole tier")
    p.add_argument("--worlds-csv", default=None)
    p.add_argument("--metrics-csv", default=None)
    a = p.parse_args(argv)

    kw = {}
    if a.worlds_csv:
        kw["worlds_csv"] = a.worlds_csv
    if a.metrics_csv:
        kw["metrics_csv"] = a.metrics_csv

    verdicts = ([classify_node(a.node, tier=a.tier, **kw)] if a.node
                else classify_tier(a.tier, **kw))

    for v in verdicts:
        claim = f"{v.claim_metric}{'/m' if v.per_metre else ''}" if v.claim_metric else "—"
        print(f"[verdict] {v.node_id:26} {v.label.value:11} n={v.n:<3} "
              f"claim={claim}")
        print(f"[verdict]   {v.reason}")
        for u in v.unchecked:
            print(f"[verdict]   ⚠️ NOT CHECKED: {u}")
    tally = collections.Counter(v.label.value for v in verdicts)
    print(f"[verdict] {len(verdicts)} node(s): "
          + ", ".join(f"{k} {n}" for k, n in sorted(tally.items())))
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
