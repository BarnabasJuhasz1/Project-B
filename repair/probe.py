"""repair/probe.py — **IS THERE ANYWHERE THIS NODE COULD HELP?**

Without this the loop cannot tell a BROKEN node from a node with NOTHING TO DO,
`INERT` has no action attached to it, and every repair is a repair against
noise.

⚠️ **THE MOTIVATING CASE, FOUND 2026-09-16.** `core.worldgen.to_scenario()`
never sets `drift`, so every world in the growth suite hands the controller a
**perfect pose estimate**. The ant node's card predicts an effect *"on scenarios
where pose_estimate drifts"*. Its mechanism exists to survive a hazard that was
switched off in all twenty worlds it was scored on, ranked in, and written up
as B−A evidence from.

So the probe does NOT search only (tier, seed). It searches the axes a **card
can predicate on**, and it can turn on the hazard a card names.

A world is **discriminating** when all three hold:

  1. the node's trigger FIRES — a world it sleeps through discriminates nothing
  2. the base has HEADROOM — it does not already succeed comfortably
  3. the world is SOLVABLE — one nothing can solve makes every controller
     look identical

⚠️ **THIS IS THE LOOP'S BIGGEST OVERFITTING HAZARD.** It lets the loop go
shopping for a world where its node happens to win, and then report the win.
Three guards, all required, all below: disjoint declared seed ranges, the
probed world written down BEFORE any repair, and the whole search recorded.

Usage:

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P -m repair.probe --node gen_b_44d12df4d25b437f --dry-run   # the plan, runs nothing
    $P -m repair.probe --node gen_b_44d12df4d25b437f --budget 8
"""
from __future__ import annotations

import dataclasses
import json
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# ⚠️ DISJOINT, DECLARED, AND THE PROBE CANNOT REACH THE CONFIRMATION RANGE.
# Repairing against the worlds you then score on is fitting the test set — the
# same argument that produced the dev/held-out split, at a smaller scale.
#
# 2000-2019 and 3000-3019 are SPENT: growth round 2 and its replication. A
# probe that returned one of those would be "finding" a world the node has
# already been measured on.
MEASURED_SEEDS = frozenset(range(2000, 2020)) | frozenset(range(3000, 3020))
PROBE_SEEDS = range(4000, 4500)
CONFIRM_SEEDS = range(9000, 9500)

# ⚠️ DERIVED HERE, NOT COPIED FROM THE SEALED SPLIT. `core/drift.py`'s sigma is
# RADIAL m/sqrt(s), so realised error after t seconds is sigma*sqrt(t). Over a
# 120 s episode that is 0.55 / 1.10 / 2.19 m — a body length, a corridor width,
# and a doorway-scale error. The ladder is chosen from the episode length and
# that formula, both of which are stated in CLAUDE.md.
#
# `sigma_yaw` is held at half `sigma_xy` throughout so the ladder has ONE knob.
# A probe that varied two at once could not say which one mattered.
DRIFT_LADDER = (0.05, 0.10, 0.20)

# Which card words put an axis on the table. ⚠️ KEYWORD MATCH, and said out
# loud rather than dressed up: it is enough for the core version because a card
# that predicates on drift says "drift" or "pose_estimate" in its own first
# clause. Chunk 57's model call is the principled version.
AXIS_KEYWORDS = {
    "drift": ("drift", "pose_estimate", "dead reckon", "dead-reckon", "odometry",
              "proprioception", "localisation", "localization", "path integration"),
    "goal_known": ("not disclosed", "goal_known", "hidden", "undisclosed",
                   "search", "unknown goal", "line-of-sight", "line of sight"),
    "sensor_noise": ("sensor noise", "noisy", "range noise", "measurement noise"),
}

# A trigger that passes on fewer than this fraction of ticks has not been given
# anything to bite on. Same floor the verdict rule uses, and deliberately the
# same number: "did it fire" must mean one thing across the loop.
from repair.verdict import FIRE_FLOOR, CHANGED_FLOOR        # noqa: E402

# The base is node-independent, so its result for a world is reusable across
# every node and every run. A base rollout costs ~30 s and a pair ~10 s, so
# without this a probe is dominated by re-running the same baseline.
BASE_CACHE = REPO_ROOT / "out" / "repair" / "base-cache.json"

# `core.geometry`'s caches are keyed on the World and never evict.
CACHE_EVERY = 16


# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Candidate:
    """One point in the search space. A scenario is a pure function of this."""

    tier: str
    seed: int
    drift: Optional[float] = None       # sigma_xy; sigma_yaw is half of it
    goal_known: bool = True

    @property
    def label(self):
        """⚠️ An axis change RENAMES the scenario, the same discipline
        `--goal-hidden` and `+cap60s` follow. A drifted world is not the same
        scenario as its undrifted twin and must never share an id with it."""
        name = f"gen-{self.tier}-{self.seed:05d}"
        if self.drift:
            name += f"+drift{self.drift:.2f}"
        if not self.goal_known:
            name += "+hidden"
        return name

    def scenario(self):
        from core.scenario import DriftSpec
        from core.worldgen import generate, to_scenario

        s = to_scenario(generate(self.tier, self.seed), goal_known=self.goal_known)
        changes = {"id": self.label}
        if self.drift:
            changes["drift"] = DriftSpec(sigma_xy=self.drift,
                                         sigma_yaw=self.drift / 2.0)
        return dataclasses.replace(s, **changes)


@dataclass(frozen=True)
class Trial:
    """One candidate, tested. `ok` is the whole point; `why` is why not."""

    candidate: Candidate
    ok: bool
    why: str
    fire: float = 0.0
    changed: float = 0.0
    base_reached: Optional[bool] = None
    pair_reached: Optional[bool] = None
    seconds: float = 0.0


@dataclass(frozen=True)
class ProbeResult:
    node_id: str
    found: Optional[Candidate]
    trials: tuple
    axes: tuple
    seconds: float = 0.0
    note: str = ""

    @property
    def untestable(self):
        return self.found is None

    def log(self):
        """⚠️ THE WHOLE SEARCH, NOT JUST THE HIT. Same discipline as
        `compose/retrieve.py`, which logs considered / scored / retained /
        dropped precisely so a miss is attributable to the library or to the
        search."""
        head = f"{'world':<34}{'fire':>7}{'chg':>7}{'base':>6}{'node':>6}  verdict"
        lines = [f"axes varied: {', '.join(self.axes) if self.axes else '(none)'}",
                 f"budget spent: {len(self.trials)} trials, {self.seconds:.0f} s", "",
                 head, "-" * (len(head) + 24)]
        for t in self.trials:
            b = "-" if t.base_reached is None else int(t.base_reached)
            p = "-" if t.pair_reached is None else int(t.pair_reached)
            # ⚠️ An accepted trial prints its reason too. "DISCRIMINATING"
            # alone hides the difference between "the base has headroom here"
            # and "this REPRODUCES the defect" — which are opposite findings
            # and lead to opposite next steps.
            mark = f"OK: {t.why}" if t.ok else t.why
            lines.append(f"{t.candidate.label:<34}{t.fire:>7.3f}{t.changed:>7.3f}"
                         f"{str(b):>6}{str(p):>6}  {mark}")
        lines.append("")
        lines.append(f"result: {self.found.label if self.found else 'UNTESTABLE'}")
        if self.note:
            lines.append(self.note)
        return "\n".join(lines)


# ---------------------------------------------------------------------------
# which axes this card puts on the table


def axes_for(card_text) -> tuple:
    """Which scenario axes the card predicates on.

    ⚠️ Keyword match. A card that depends on drift says so in words — the ant
    card's prediction opens *"On scenarios where pose_estimate drifts"*. Good
    enough to find the case that motivated this chunk, and honest about what it
    is.
    """
    low = (card_text or "").lower()
    return tuple(axis for axis, words in AXIS_KEYWORDS.items()
                 if any(w in low for w in words))


def candidates(axes, tier="maze", seeds=None, ladder=DRIFT_LADDER):
    """The search space, ordered so the axes the CARD NAMES are tried first.

    ⚠️ Ordering is not cosmetic. The budget is small and a base rollout costs
    ~30 s, so trying undrifted geometry first on a node whose whole claim is
    about drift would spend the budget before reaching the question.
    """
    seeds = list(seeds if seeds is not None else PROBE_SEEDS)
    out = []
    if "drift" in axes:
        for sigma in ladder:
            for seed in seeds:
                out.append(Candidate(tier, seed, drift=sigma))
    if "goal_known" in axes:
        for seed in seeds:
            out.append(Candidate(tier, seed, goal_known=False))
    for seed in seeds:                       # plain geometry, always last
        out.append(Candidate(tier, seed))
    return out


# ---------------------------------------------------------------------------
# running one


def _load_cache():
    try:
        return json.loads(BASE_CACHE.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def _save_cache(cache):
    BASE_CACHE.parent.mkdir(parents=True, exist_ok=True)
    BASE_CACHE.write_text(json.dumps(cache, indent=1, sort_keys=True),
                          encoding="utf-8")


def _runner():
    from repair.evidence import _debug_module, _growth_build, _tree_of
    from generate import base as basemod
    return _debug_module(), _growth_build(), basemod.load(), _tree_of


def base_score(candidate, cache=None, runner=None):
    """The base's FULL scored result on this world. Cached.

    ⚠️ The cache used to hold `goal_reached` and `path_length` only, which is
    all the probe needs — but a VERDICT needs the whole metric set, and
    `loop.measure_live` was shipping `metrics={}` partly because there was no
    cheap way to get it. An entry written by the old format has no `score` key,
    so it is re-rolled once and upgraded in place rather than invalidated
    wholesale: a rollout is deterministic, so the upgrade cannot change any
    answer already given.

    Returns (score_dict, was_cached).
    """
    cache = _load_cache() if cache is None else cache
    key = candidate.label
    entry = cache.get(key)
    if entry is not None and "score" in entry:
        return entry["score"], True

    dbg, _, base, tree_of = runner or _runner()
    _, score, _, _ = dbg._rollout(tree_of(base), candidate.scenario(),
                                  base.metadata.node_id)
    score = dict(score)
    cache[key] = {"goal_reached": bool(score["goal_reached"]),
                  "path_length": float(score["path_length"]),
                  "score": score}
    return score, False


def base_result(candidate, cache=None, runner=None):
    """Did the base solve this world? Cached — it does not depend on the node.

    Rollouts are deterministic and a world is a pure function of its candidate,
    so the cache is a memo, not a shortcut.

    ⚠️ Reads `goal_reached` DIRECTLY rather than going through `base_score`.
    The probe needs the boolean and nothing else, and an entry written before
    the full metric set was cached still answers that question perfectly.
    Routing this through `base_score` would re-roll every legacy entry to
    compute a metric set this caller never reads — 20 rollouts to learn what
    the cache already knew.
    """
    cache = _load_cache() if cache is None else cache
    entry = cache.get(candidate.label)
    if entry is not None:
        return bool(entry["goal_reached"]), True

    score, hit = base_score(candidate, cache, runner)
    return bool(score["goal_reached"]), hit


def _without_axis(candidate):
    """The same geometry with every axis back at its default."""
    return Candidate(candidate.tier, candidate.seed)


def axis_effect(candidate, base_with_axis, cache, runner):
    """What does the varied axis actually DO to the base here?

    ⚠️ TWO DEFECTS THIS FUNCTION EXISTS BECAUSE OF, BOTH FOUND ON REAL RUNS.

    **First:** the probe's first version called a world discriminating whenever
    the base failed in it. It returned `gen-maze-04000+drift0.05` — and the
    base fails on seed 4000 **with or without drift**. The axis contributed
    nothing and the probe had found a hard world wearing a drift label. A
    repair tested against it would have been tested against geometry.

    **Second:** the fix checked only that the base solved the CLEAN world, then
    reported "the axis takes it away" — which on seed 4005 was simply false.
    The base solves 4005 both ways (27.5 m clean, 40.3 m under drift). Half a
    check with a whole claim attached to it is worse than no check.

    So both halves are read, and the three outcomes are named separately:

        "unsolvable"   the base cannot do this world even without the axis
        "axis_breaks"  the base solves it clean and FAILS with the axis  <- gold
        "base_copes"   the base solves it either way; the axis is not the obstacle

    Returns (state, why); state is None when no axis was varied.
    """
    plain = _without_axis(candidate)
    if plain.label == candidate.label:
        return None, "no axis varied"

    clean, _ = base_result(plain, cache, runner)
    if not clean:
        # ⚠️ "even without the axis", not "here anyway". Seed 4001 has the base
        # FAILING clean and SUCCEEDING under drift — drift perturbed it onto a
        # lucky route. "fails here anyway" would be plainly false for that row,
        # and a misleading rejection reason is worse than none: the log is the
        # only record of why a world was passed over.
        return "unsolvable", "base cannot solve this world even without the axis"
    if not base_with_axis:
        return "axis_breaks", "base solves it clean and the axis takes it away"
    return "base_copes", "base solves it with and without the axis"


def try_candidate(node_id, candidate, entry, cache, runner):
    """Test one world. ⚠️ THE NODE IS ROLLED OUT FIRST, ON PURPOSE.

    A candidate the node sleeps through is rejected without ever paying for the
    base — and the base is the expensive half (~30 s against ~10 s). Screening
    in the cheap direction is what makes a budget of a dozen affordable, and
    base results are cached across nodes and runs because the base does not
    depend on the node.
    """
    started = time.time()
    dbg, build, base, _ = runner
    scenario = candidate.scenario()

    _, p_score, _, summary = dbg._rollout(
        build(entry.node_cls, entry.metadata, base), scenario, node_id)

    ticks = max(1, int(summary.get("ticks", 1)))
    fire = float(summary.get("precondition_passed", 0)) / ticks
    # ⚠️ `changed` is `drove`, exactly as `scripts/measure_growth.py` computes
    # it — the tick log records `drove` as "the command object changed". Fire
    # high with changed zero is a node that runs and does nothing, which reads
    # identically to one that never fires if you only look at the rate. Two
    # definitions of `changed` in one loop would make those indistinguishable
    # again.
    changed = float(summary.get("drove", 0)) / ticks
    pair_reached = bool(p_score["goal_reached"])

    def done(ok, why, base_reached=None):
        return Trial(candidate, ok, why, fire, changed, base_reached,
                     pair_reached, time.time() - started)

    if fire < FIRE_FLOOR:
        return done(False, "trigger never fired")
    if changed < CHANGED_FLOOR:
        return done(False, "fires but changes nothing")

    base_reached, _ = base_result(candidate, cache, runner)
    state, why = axis_effect(candidate, base_reached, cache, runner)

    if state == "unsolvable":
        return done(False, why, base_reached)

    if state == "axis_breaks":
        # The gold case: the task is doable, and the hazard the card names is
        # exactly what removes it. This is the ant card's own comparison.
        return done(True, why, base_reached)

    if base_reached and not pair_reached:
        # The base wins and the node loses: discriminating in the WRONG
        # direction. Returned anyway — it REPRODUCES the defect on fresh
        # geometry, which is what a repair has to be tested against. ⚠️ Not the
        # same finding as "axis_breaks" and the log must not blur them.
        return done(True, "node loses a world the base wins (defect reproduced)",
                    base_reached)
    if base_reached and pair_reached:
        return done(False, "base already succeeds here", base_reached)
    return done(True, "base has headroom and the trigger fires", base_reached)


def probe(node_id, tier="maze", budget=8, seeds=None, card_text=None,
          ladder=DRIFT_LADDER) -> ProbeResult:
    """Search for a world that puts this node's capability on the table."""
    from core.worldgen import clear_geometry_caches
    from repair.evidence import card_block, entry_for

    started = time.time()
    # ⚠️ `entry_for`, NEVER `load_library().get` — a REPAIRED node is not in the
    # live library. Rounds stage under `out/repair/<lineage>/round-NN/` and all
    # of them reuse the parent's stem, so `__r01` resolved to a KeyError and
    # killed a live run mid-round-2 after the diagnosis call had been paid for.
    # `entry_for` is the one resolver every component that needs to RUN a node
    # goes through, and it is the component that already knows about rounds.
    try:
        entry = entry_for(node_id)
    except (KeyError, ValueError) as exc:
        # An unresolvable node is an OUTCOME, not a traceback — the same rule
        # `run_round` applies to a failed regeneration and to an unusable
        # diagnosis. The loop records UNTESTABLE and keeps its remaining budget.
        # ⚠️ It still says WHY: `entry_for` raises rather than guessing, and
        # that message is the only thing distinguishing "no such node" from
        # "the round staged two".
        return ProbeResult(node_id, None, (), (), time.time() - started,
                           f"{node_id} could not be resolved: {exc}")
    if entry is None:
        return ProbeResult(node_id, None, (), (), time.time() - started,
                           f"{node_id} is not in the live library")

    axes = axes_for(card_text if card_text is not None else card_block(node_id))
    plan = candidates(axes, tier=tier, seeds=seeds, ladder=ladder)
    cache, runner = _load_cache(), _runner()

    trials, found = [], None
    for i, cand in enumerate(plan[:budget]):
        trials.append(try_candidate(node_id, cand, entry, cache, runner))
        # ⚠️ SAVED AFTER EVERY TRIAL, not at the end. A base rollout costs ~30 s
        # and a killed probe would otherwise throw away everything it paid for
        # — the same discipline `measure_growth.py` writes its rows under, and
        # the same family as the sweep rehearsal's "exited 0, wrote nothing".
        _save_cache(cache)
        if trials[-1].ok:
            found = cand
            break
        if (i + 1) % CACHE_EVERY == 0:
            clear_geometry_caches()

    note = ("" if found else
            "UNTESTABLE within this budget. ⚠️ That is a statement about the "
            "search, not proof that no such world exists — widen the budget or "
            "the axes before reading it as NOT_TRANSFERABLE.")
    return ProbeResult(node_id, found, tuple(trials), axes,
                       time.time() - started, note)


# ---------------------------------------------------------------------------


def _main(argv=None):
    import argparse

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--node", required=True)
    p.add_argument("--tier", default="maze")
    p.add_argument("--budget", type=int, default=8)
    p.add_argument("--dry-run", action="store_true",
                   help="print the axes and the plan; run nothing")
    a = p.parse_args(argv)

    from repair.evidence import card_block

    axes = axes_for(card_block(a.node))
    plan = candidates(axes, tier=a.tier)

    if a.dry_run:
        print(f"[probe] node  {a.node}")
        print(f"[probe] axes  {', '.join(axes) if axes else '(none — geometry only)'}")
        print(f"[probe] plan  {len(plan)} candidates, first {a.budget}:")
        for c in plan[:a.budget]:
            print(f"[probe]   {c.label}")
        print(f"[probe] seeds probe={PROBE_SEEDS.start}-{PROBE_SEEDS.stop - 1}, "
              f"confirm={CONFIRM_SEEDS.start}-{CONFIRM_SEEDS.stop - 1}, "
              f"spent=2000-2019 and 3000-3019 (growth + replication)")
        print("[probe] ran nothing")
        return 0

    result = probe(a.node, tier=a.tier, budget=a.budget)
    print(result.log())
    return 0 if result.found else 2


if __name__ == "__main__":
    raise SystemExit(_main())
