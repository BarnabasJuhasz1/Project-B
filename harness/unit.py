"""
harness/unit.py

The admission decision. Non-negotiable #5: a node is admissible only if it
passes its unit scenario in isolation, decided automatically.

G1 reports the unit pass rate, so this file *is* the measurement.

THE PROBLEM THIS FILE EXISTS TO SOLVE
--------------------------------------
§4 says the card's Prediction becomes the unit test, which means the generator
emits the node AND its test. **A generator that writes both can write a test its
node trivially passes.** Nothing about asking it nicely fixes that.

So the generator proposes and the harness enforces three structural requirements
it cannot dodge:

1. **There must be a fixed baseline**, named, and run over the same world and the
   same seeds as the candidate.
2. **The baseline must actually fail.** A unit scenario whose baseline already
   succeeds is REJECTED AS MALFORMED, not passed. This is the single most
   important check here: it is the one that catches "I wrote a test that was
   already true".
3. **The effect must exceed a floor fixed in advance** — `MIN_EFFECT_RATE` /
   `MIN_EFFECT_REL` below, not a number the generator supplies. A unit scenario
   may demand MORE than the floor; it may never demand less.

A rejection is an outcome, not an error. Chunk 15 keeps rejected candidates with
their reason, because G1's pass rate needs a denominator and because a rejected
node is the most informative artefact the pipeline produces.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

from core.metrics import score
from core.tests.kinematic import rollout

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# PRE-REGISTRATION MATERIAL. Fixed before any candidate is judged, or the first
# borderline node sets the bar retroactively. Chunk 17 collects these.
#
# Calibrated on the two reference controllers, deliberately: their capabilities
# are known, neither is a generated node, and neither has an interest in the
# answer. On a corridor micro-world with one box, `reactive_avoid` reaches the
# goal on every seed and `goal_seeker` on none — a rate effect of 1.0. Half of
# that is a demanding but reachable bar for a node claiming a real mechanism.
MIN_EFFECT_RATE = 0.5      # for rate metrics: goal_reached
MIN_EFFECT_REL = 0.10      # for continuous metrics: 10% relative improvement

# Metrics where lower is better. Anything else is treated as higher-is-better.
LOWER_IS_BETTER = frozenset({
    "collisions", "near_misses", "stuck_events", "time_stuck",
    "recovery_time", "localisation_failures", "time_to_goal", "path_length",
})

RATE_METRICS = frozenset({"goal_reached"})


@dataclass(frozen=True)
class UnitScenario:
    """One node's claimed effect, stated as something that can be falsified.

    `metric` must be a key of `core.metrics.score()`. `min_effect` is what THIS
    scenario demands; the harness raises the demand to the floor if it is lower,
    and never lowers it.
    """

    node_id: str
    micro: object                       # harness.micro.MicroScenario
    metric: str
    baseline: str                       # a name resolvable by controllers.make
    min_effect: float = 0.0
    note: str = ""

    # ONE seed by default, and that is not laziness. A micro-world with no
    # drift, driven by a deterministic node, gives the same episode for every
    # seed — five of them is five identical rollouts, five times the cost, and a
    # misleading suggestion that something was averaged. Supply more seeds only
    # where there is something to average over: a DriftSpec on the micro-world,
    # or a node that draws from the rng it is handed.
    seeds: Tuple[int, ...] = (0,)


@dataclass(frozen=True)
class UnitResult:
    node_id: str
    admitted: bool
    reason: str
    metric: str = ""
    candidate: float = 0.0
    baseline: float = 0.0
    effect: float = 0.0
    required: float = 0.0

    def __bool__(self):
        return self.admitted


def _aggregate(controller_factory, unit):
    """Mean of `unit.metric` over the seeds, on the same micro-world."""
    import dataclasses
    values = []
    for seed in unit.seeds:
        micro = dataclasses.replace(unit.micro, seed=int(seed))
        result = score(rollout(micro, controller_factory()), micro)
        value = result[unit.metric]
        if value is None:
            # recovery_time is None when nothing was ever stuck, time_to_goal
            # when the goal was never reached. Both mean "the event did not
            # happen", which for a lower-is-better metric is the best case.
            value = 0.0 if unit.metric in LOWER_IS_BETTER else 0.0
        values.append(float(value))
    return float(np.mean(values))


def _effect(metric, candidate, baseline):
    """How much better the candidate is, in the metric's own terms."""
    if metric in RATE_METRICS:
        return candidate - baseline
    if metric in LOWER_IS_BETTER:
        if baseline == 0.0:
            return 0.0                  # nothing to improve on
        return (baseline - candidate) / abs(baseline)
    if baseline == 0.0:
        return float(candidate > 0.0)
    return (candidate - baseline) / abs(baseline)


def required_effect(unit):
    """The floor, raised to whatever the scenario itself demands."""
    floor = MIN_EFFECT_RATE if unit.metric in RATE_METRICS else MIN_EFFECT_REL
    return max(floor, float(unit.min_effect))


def run_unit(candidate_factory, unit, baseline_factory=None):
    """Decide whether one node is admissible. No human in the loop.

    `candidate_factory` and `baseline_factory` are zero-argument callables
    returning a Controller — a Tree wrapping the node under test, and the fixed
    baseline. `baseline_factory` defaults to resolving `unit.baseline` through
    the controller registry.
    """
    import controllers

    if unit.metric not in _score_keys():
        return UnitResult(unit.node_id, False,
                          f"metric {unit.metric!r} is not produced by "
                          f"core.metrics.score(); a prediction that names no "
                          f"measurable quantity is not falsifiable")
    try:
        unit.micro.validate()
    except ValueError as exc:
        return UnitResult(unit.node_id, False, f"malformed micro-world: {exc}")

    if baseline_factory is None:
        try:
            baseline_factory = lambda: controllers.make(unit.baseline)
        except KeyError as exc:
            return UnitResult(unit.node_id, False, f"unknown baseline: {exc}")

    base = _aggregate(baseline_factory, unit)
    cand = _aggregate(candidate_factory, unit)
    effect = _effect(unit.metric, cand, base)
    required = required_effect(unit)

    # Requirement 2, and the one that matters most: if the baseline already
    # clears the bar, the scenario is not testing the node — it is testing
    # nothing, and passing it would inflate G1's pass rate with free marks.
    if _baseline_already_succeeds(unit, base, cand):
        return UnitResult(
            unit.node_id, False,
            "MALFORMED: the fixed baseline already succeeds on this "
            "micro-world, so the scenario cannot distinguish the node from "
            "doing nothing. Make the world harder, not the bar lower.",
            unit.metric, cand, base, effect, required)

    if effect < required:
        return UnitResult(
            unit.node_id, False,
            f"effect {effect:.3f} below the required {required:.3f}",
            unit.metric, cand, base, effect, required)

    return UnitResult(unit.node_id, True, "admitted",
                      unit.metric, cand, base, effect, required)


def _baseline_already_succeeds(unit, base, cand):
    if unit.metric in RATE_METRICS:
        return base >= 1.0
    if unit.metric in LOWER_IS_BETTER:
        return base <= 0.0
    return False


_SCORE_KEYS = None


def _score_keys():
    """The keys score() actually returns, discovered rather than hardcoded, so a
    new metric is usable the day it exists and a removed one fails loudly."""
    global _SCORE_KEYS
    if _SCORE_KEYS is None:
        from core.scenario import load
        import controllers
        sc = load("blind_corner")
        _SCORE_KEYS = frozenset(score(rollout(sc, controllers.make("goal_seeker")), sc))
    return _SCORE_KEYS
