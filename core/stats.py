"""
core/stats.py

The pre-registered decision rule, as code.

`PRE-REGISTRATION.md` states G2's rule in words; this module is the same rule in
a form that cannot be reinterpreted after the fact. Week 5's analysis calls it
rather than reimplementing it, which is the whole point — a rule that lives only
in prose is re-derived by whoever runs the analysis, and they will re-derive it
having already seen the numbers.

Sim-free, like the rest of core/. No simulator, no GPU, no I/O.

    from core.stats import EpisodeOutcome, decide
    verdict = decide(outcomes_b, outcomes_d)
    print(verdict.fires, verdict.gap, verdict.b_ci, verdict.d_ci)

The one design decision worth reading before using any of this: **the resampling
unit is the scenario, not the episode.** See RESAMPLING_UNIT below.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------
#
# PRE-REGISTRATION MATERIAL. Every constant here is hashed by
# core/tests/test_preregistration.py against a value recorded in
# PRE-REGISTRATION.md. Changing one without editing the document is a failing
# test, and after the tag `prereg-v1` it is an amendment rather than an edit.

# The unit resampled by the bootstrap.
#
# "scenario" means: resample the 12 held-out scenarios with replacement, then
# take every seed belonging to a drawn scenario. "episode" would resample the
# 12 x S episodes independently.
#
# Episode-level resampling is wrong here and not marginally so. Two facts make
# it wrong:
#
#   1. Episodes are CLUSTERED within scenarios. Success on
#      `held_navigate_*__seed2001` says a great deal about the same scenario at
#      seed 2002 and rather little about a different scenario. Treating them as
#      independent understates every interval.
#   2. On the six UNDRIFTED held-out scenarios the seeds are not merely
#      correlated, they are IDENTICAL. `Scenario.seed` reaches exactly two
#      places -- `rng()`, handed to the controller, and `drift_rng()`. A
#      controller that does not draw from its rng, run on a scenario with no
#      drift, produces a bit-identical episode at every seed. Measured on
#      `reactive_avoid`, `goal_seeker` and `toy_search_tree`: identical trace
#      hash across seeds 1002/7/999 with drift off, different with drift on.
#      Episode-level resampling would count one episode S times and shrink the
#      interval by sqrt(S) for free.
#
# The price is that the effective sample size is the number of SCENARIOS, and
# the split has twelve of them. That is the binding constraint on this
# experiment's power, and `scripts/power_analysis.py` quantifies it. It is
# recorded here rather than worked around, because the alternative is an
# interval that is narrow and wrong.
RESAMPLING_UNIT = "scenario"

# Bootstrap resamples. 10000 is enough that the 2.5th and 97.5th percentiles are
# stable to well under a point at any N this experiment produces; the whole
# analysis is milliseconds, so there is no reason to economise.
N_RESAMPLES = 10000

# Percentile intervals, not BCa. With twelve clusters the bias-correction and
# acceleration terms are themselves estimated from twelve numbers, and a
# correction estimated that badly is noise dressed as rigour. Percentile is the
# transparent choice and it is the one the description's wording implies.
CI_METHOD = "percentile"
CI_LEVEL = 0.95

# The bootstrap's own randomness. Fixed so that the analysis is reproducible
# bit-for-bit from the committed results, and fixed BEFORE any result exists so
# that it cannot be chosen to move a boundary.
BOOTSTRAP_SEED = 20260826

# G2's threshold: B's goal-reach rate must exceed D's by at least this much for
# the primary rule to fire.
#
# 0.30 is read off `scripts/power_analysis.py`, not chosen. Under the estimator
# above, at twelve scenarios, the disjoint-interval criterion needs a TRUE gap
# near 0.50 for 80% power and has roughly 0.2 power at 0.30. So the interval
# criterion, not X, is what binds; X's job is to be a floor on what is worth
# claiming that does not bind BEFORE the interval does. 0.30 is the largest
# value with that property, and a 30-point improvement in goal-reach rate is
# unambiguously worth claiming.
#
# The consequence is pre-registered rather than discovered: this rule is
# conservative, it is expected NOT to fire on a genuine 20-point corpus
# advantage, and a non-firing rule therefore may not be reported as "the
# corpus does not help". See PRE-REGISTRATION.md, "What this rule cannot do".
X = 0.30

# Replicate seeds, as an offset applied to each scenario's own sealed seed.
#
# Replicate 0 IS the sealed episode: offset 0 leaves `Scenario.seed` alone, so
# the first replicate of every held-out scenario is exactly the episode chunk
# 11 sealed and chunk 13 verified. The other two are perturbations of it.
#
# Three, not five or ten. `scripts/power_analysis.py` measures the whole seed
# axis as nearly inert: going from 3 replicates to 10 moves the rule's power by
# a few points and multiplies wall clock by three. It is nearly inert because
# on the six UNDRIFTED held-out scenarios extra seeds are not extra episodes at
# all -- they are the same episode recorded again (see RESAMPLING_UNIT). Seeds
# buy information on the drifted half and on any condition that draws from its
# rng, and that is all they buy.
#
# 20000 is skipped, and the reason is a stated rule rather than a preference:
# the offsets are the first three multiples of 10000 at which EVERY drifted
# held-out scenario's pose error crosses LOC_FAIL_THRESHOLD inside its own time
# limit. At 20000 one of the six never crosses, so drift would be nominally on
# and functionally absent for that episode -- a manipulation that did not take.
# The rule is safe from seed-shopping because drift is injected open-loop and is
# bit-identical across conditions (see cards/store.py's CONTROLLER_INDEPENDENT
# and plans/done/14-first-cards.md), so no choice of offset can favour any
# condition. Reproduce with scripts/check_seed_offsets.py.
SEED_OFFSETS = (0, 10000, 30000)
SEEDS_PER_SCENARIO = len(SEED_OFFSETS)


# ---------------------------------------------------------------------------


def seeds_for(scenario_seed):
    """The pre-registered replicate seeds for one scenario.

    A rule, not a list, so that "did anyone add a seed after seeing the
    results" is answerable by inspection rather than by memory.
    """
    return tuple(scenario_seed + off for off in SEED_OFFSETS)


@dataclass(frozen=True)
class EpisodeOutcome:
    """One scored episode, reduced to what the decision rule reads.

    `scenario` is the clustering key. `success` is `result["goal_reached"]`.
    Nothing else enters the primary rule -- deliberately, so that "we tried a
    few definitions of success" is not available later.
    """

    scenario: str
    seed: int
    success: bool


def _by_scenario(outcomes):
    """scenario id -> mean success over its seeds. The cluster summary."""
    acc = {}
    for o in outcomes:
        acc.setdefault(o.scenario, []).append(bool(o.success))
    return {k: float(np.mean(v)) for k, v in acc.items()}


def _percentile_ci(samples, level=CI_LEVEL):
    lo = 100.0 * (1.0 - level) / 2.0
    return (float(np.percentile(samples, lo)),
            float(np.percentile(samples, 100.0 - lo)))


@dataclass(frozen=True)
class Verdict:
    """The whole decision, with every number that produced it.

    Reported in full whichever way it comes out. `fires` False with a large
    `gap` and a wide interval is a different result from `fires` False with a
    gap of zero, and the write-up has to be able to tell them apart.
    """

    fires: bool
    gap: float
    # Which two conditions this verdict is about. Carried on the object rather
    # than assumed by the printer: the same rule is applied to B-D (the
    # headline), B-A (confounded with principle articulation) and B-C, and a
    # summary line that always said "B" and "D" would mislabel two of the three.
    b_rate: float
    d_rate: float
    b_ci: Tuple[float, float]
    d_ci: Tuple[float, float]
    ci_disjoint: bool
    paired_diff: float
    paired_ci: Tuple[float, float]
    paired_excludes_zero: bool
    n_scenarios: int
    n_episodes_b: int
    n_episodes_d: int
    # Which two conditions this verdict is about. Carried on the object rather
    # than assumed by the printer: the same rule is applied to B-D (the
    # headline), B-A (confounded with principle articulation) and B-C, and a
    # summary line that always said "B" and "D" would mislabel two of the three.
    labels: Tuple[str, str] = ("B", "D")

    def summary(self):
        treatment, control = self.labels
        return (
            f"{treatment} {self.b_rate:.3f} "
            f"[{self.b_ci[0]:.3f}, {self.b_ci[1]:.3f}]  "
            f"{control} {self.d_rate:.3f} "
            f"[{self.d_ci[0]:.3f}, {self.d_ci[1]:.3f}]  "
            f"gap {self.gap:+.3f} (X = {X:.2f})  "
            f"disjoint={self.ci_disjoint}  "
            f"paired {self.paired_diff:+.3f} "
            f"[{self.paired_ci[0]:+.3f}, {self.paired_ci[1]:+.3f}]  "
            f"=> {'FIRES' if self.fires else 'does not fire'}"
        )


def decide(outcomes_b, outcomes_d, x=None, n_resamples=None, seed=None,
           labels=("B", "D")):
    """G2's rule. Returns a Verdict; raises rather than guessing.

    The primary rule, exactly as pre-registered:

        B's goal-reach rate exceeds D's by at least X, AND the two 95 percent
        cluster-bootstrap confidence intervals do not overlap.

    The paired interval is computed on the SAME resampled scenario indices and
    reported alongside. It is a secondary, not a tie-breaker: if the two
    disagree, the primary rule governs the claim and the disagreement is
    reported. Deciding which one to believe after seeing them is the exact
    move a pre-registration exists to forbid.

    Both conditions must cover the same scenarios. That is non-negotiable #10
    and it is checked, not assumed -- a condition that crashed on two scenarios
    and was scored over the remaining ten is a comparison between two different
    benchmarks.
    """
    x = X if x is None else x
    n_resamples = N_RESAMPLES if n_resamples is None else n_resamples
    seed = BOOTSTRAP_SEED if seed is None else seed

    b, d = _by_scenario(outcomes_b), _by_scenario(outcomes_d)
    if not b:
        raise ValueError("no episodes for condition B")
    if set(b) != set(d):
        missing = sorted(set(b) ^ set(d))
        raise ValueError(
            "conditions do not cover the same scenarios, so the comparison is "
            f"between two different benchmarks. Differing: {missing}")

    scenarios = sorted(b)
    n = len(scenarios)
    bv = np.array([b[s] for s in scenarios])
    dv = np.array([d[s] for s in scenarios])

    rng = np.random.default_rng(seed)
    # One index draw per resample, applied to BOTH conditions. Drawing
    # separately would break the pairing that the paired interval reads, and
    # the two conditions genuinely did run the same scenarios -- resampling
    # them apart would model an experiment nobody performed.
    idx = rng.integers(0, n, size=(n_resamples, n))
    b_boot = bv[idx].mean(axis=1)
    d_boot = dv[idx].mean(axis=1)
    diff_boot = b_boot - d_boot

    b_ci, d_ci = _percentile_ci(b_boot), _percentile_ci(d_boot)
    paired_ci = _percentile_ci(diff_boot)
    disjoint = b_ci[0] > d_ci[1] or d_ci[0] > b_ci[1]
    gap = float(bv.mean() - dv.mean())

    return Verdict(
        fires=bool(gap >= x and disjoint),
        gap=gap,
        b_rate=float(bv.mean()),
        d_rate=float(dv.mean()),
        b_ci=b_ci,
        d_ci=d_ci,
        ci_disjoint=bool(disjoint),
        paired_diff=float((bv - dv).mean()),
        paired_ci=paired_ci,
        paired_excludes_zero=bool(paired_ci[0] > 0.0 or paired_ci[1] < 0.0),
        n_scenarios=n,
        n_episodes_b=len(outcomes_b),
        n_episodes_d=len(outcomes_d),
        labels=tuple(labels),
    )


def outcomes_from_results(results):
    """`[{"scenario": ..., "seed": ..., "goal_reached": ...}, ...]` -> outcomes.

    Takes `result` dicts straight from `core.metrics.score()`, so the analysis
    reads the committed files rather than a transcription of them.
    """
    return [EpisodeOutcome(scenario=r["scenario"], seed=r["seed"],
                           success=bool(r["goal_reached"]))
            for r in results]
