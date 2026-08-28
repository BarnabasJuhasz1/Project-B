"""
scripts/power_analysis.py

What effect size can G2's decision rule actually detect?

`plans/todo/17-pre-registration.md` carries a table of "smallest detectable X"
computed for two INDEPENDENT PROPORTIONS at the episode level:
`X > 1.96 / sqrt(N)`, N being episodes per condition. That table is wrong for
this experiment, in the unsafe direction, for two reasons:

  1. Episodes are clustered within scenarios, so the effective sample size is
     nearer the scenario count than the episode count.
  2. On the six undrifted held-out scenarios the replicate seeds are not merely
     correlated, they are IDENTICAL -- `Scenario.seed` reaches only `rng()` and
     `drift_rng()`, so a controller that ignores its rng on an undrifted
     scenario runs the same episode at every seed.

So the episode count is close to irrelevant and the SCENARIO count is the
binding constraint. This script measures that, by simulating the whole pipeline
-- scenario-level heterogeneity, degenerate seeds where the design makes them
degenerate, and `core.stats.decide()` itself as the estimator -- rather than
substituting a closed form for it.

    python scripts/power_analysis.py                     # the pre-registration table
    python scripts/power_analysis.py --reps 5000         # tighter, slower
    python scripts/power_analysis.py --n-scenarios 15    # dev + held-out

Sim-free. Nothing here is a result about the robot; it is a property of the
DESIGN, and it is committed so that the number in PRE-REGISTRATION.md can be
reproduced rather than taken on trust.
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from core.stats import EpisodeOutcome, decide          # noqa: E402


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# Between-scenario heterogeneity on the logit scale. This is the parameter the
# answer is most sensitive to, so it is swept rather than assumed.
#
#   0.0  every scenario equally hard -- the independent-proportions world, and
#        a fiction: `goal_seeker` reaches the goal on none of the dev scenarios
#        and `reactive_avoid` on all three.
#   1.5  moderate: a scenario's difficulty shifts the odds by ~4x either way.
#   3.0  strong: scenarios are close to solved-or-not, which is what the six
#        undrifted held-out scenarios will actually look like.
SIGMA_U = (0.0, 1.5, 3.0)

# The baseline condition's rate. D is the comparator that matters (B-D), and a
# rate near 0.5 is where a proportion's variance is largest, so it is the
# honest place to size a rule.
D_RATE = 0.40

# True B-D gaps to sweep, in points.
GAPS = (0.10, 0.15, 0.20, 0.25, 0.30, 0.40, 0.50)

# Replicate seeds per scenario to sweep.
SEED_COUNTS = (1, 3, 5, 10)

# Held-out split shape, from plans/done/11-held-out-split.md: 12 scenarios,
# half of them carrying a DriftSpec. Only the drifted half has any within-
# scenario randomness for a deterministic controller.
N_SCENARIOS = 12
FRACTION_DRIFTED = 0.5

# Power we would want before calling a rule usable. 0.8 is the convention and
# nothing here argues for a different one.
TARGET_POWER = 0.80

# Fewer resamples than core.stats.N_RESAMPLES: this runs the bootstrap tens of
# thousands of times, and a power estimate is itself a Monte Carlo average, so
# the inner precision does not need to be the reporting precision.
INNER_RESAMPLES = 800


# ---------------------------------------------------------------------------


def _sigmoid(z):
    return 1.0 / (1.0 + np.exp(-z))


_MU_CACHE = {}


def _solve_mu(target_rate, sigma_u, n=40_000, seed=0):
    """mu such that E[sigmoid(mu + u)] == target_rate, u ~ N(0, sigma_u^2).

    Bisection over a fixed quadrature sample rather than a closed form: the
    logistic-normal mean has none, and a fixed sample keeps the solve
    deterministic. Cached because the sweep asks for the same handful of
    (rate, sigma) pairs thousands of times.
    """
    key = (round(target_rate, 6), sigma_u, n, seed)
    if key in _MU_CACHE:
        return _MU_CACHE[key]
    u = np.random.default_rng(seed).normal(0.0, sigma_u, size=n) if sigma_u > 0 \
        else np.zeros(n)
    lo, hi = -25.0, 25.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if _sigmoid(mid + u).mean() < target_rate:
            lo = mid
        else:
            hi = mid
    _MU_CACHE[key] = 0.5 * (lo + hi)
    return _MU_CACHE[key]


def simulate(gap, sigma_u, n_seeds, reps, n_scenarios=N_SCENARIOS,
             fraction_drifted=FRACTION_DRIFTED, d_rate=D_RATE, seed=1):
    """Power of the primary rule and of the paired secondary, at one design point.

    One replication builds a whole four-condition-style dataset for B and D:
    scenario difficulties `u_i` are SHARED between the conditions, because both
    run the same scenarios -- that is the pairing, and modelling it away would
    flatter the paired interval.
    """
    rng = np.random.default_rng(seed)
    mu_d = _solve_mu(d_rate, sigma_u)
    mu_b = _solve_mu(min(d_rate + gap, 0.999), sigma_u)

    ids = [f"s{i:02d}" for i in range(n_scenarios)]
    n_drift = int(round(n_scenarios * fraction_drifted))
    drifted = set(ids[:n_drift])

    fires = paired = 0
    for _ in range(reps):
        u = rng.normal(0.0, sigma_u, size=n_scenarios) if sigma_u > 0 \
            else np.zeros(n_scenarios)
        out = {"b": [], "d": []}
        for cond, mu in (("b", mu_b), ("d", mu_d)):
            p = _sigmoid(mu + u)
            for i, sid in enumerate(ids):
                if sid in drifted:
                    # Drift gives every seed its own realisation.
                    draws = rng.random(n_seeds) < p[i]
                else:
                    # No drift and a deterministic controller: one episode,
                    # recorded n_seeds times. This is the whole point.
                    draws = np.repeat(rng.random() < p[i], n_seeds)
                for k, ok in enumerate(draws):
                    out[cond].append(EpisodeOutcome(sid, 2100 + k, bool(ok)))
        v = decide(out["b"], out["d"], n_resamples=INNER_RESAMPLES,
                   seed=int(rng.integers(1, 2**31 - 1)))
        fires += v.fires
        # The paired secondary as it would be applied: a gap of at least X and
        # an interval on the paired difference that excludes zero.
        paired += bool(v.gap >= _X() and v.paired_ci[0] > 0.0)
    return fires / reps, paired / reps


def _X():
    from core.stats import X
    return X


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--reps", type=int, default=1500,
                    help="replications per design point (default 1500)")
    ap.add_argument("--n-scenarios", type=int, default=N_SCENARIOS)
    ap.add_argument("--d-rate", type=float, default=D_RATE)
    args = ap.parse_args()

    print(f"[power] X = {_X():.2f}, D rate = {args.d_rate:.2f}, "
          f"{args.n_scenarios} scenarios, {args.reps} reps/cell")
    print(f"[power] rule 1 = gap >= X and the two 95% cluster-bootstrap CIs are disjoint")
    print(f"[power] rule 2 = gap >= X and the paired difference CI excludes 0 (secondary)")

    for sigma_u in SIGMA_U:
        print(f"\n[power] between-scenario sigma = {sigma_u}")
        header = "  true gap |" + "".join(f"  S={s:<2d} disjoint / paired" for s in SEED_COUNTS)
        print(header)
        print("  " + "-" * (len(header) - 2))
        for gap in GAPS:
            row = f"  {gap:>8.2f} |"
            for n_seeds in SEED_COUNTS:
                p1, p2 = simulate(gap, sigma_u, n_seeds, args.reps,
                                  n_scenarios=args.n_scenarios,
                                  d_rate=args.d_rate)
                row += f"   {p1:>5.2f} / {p2:>5.2f}      "
            print(row)

    print(f"\n[power] target power {TARGET_POWER:.0%}. A cell below it is a rule "
          f"that will not fire on a real effect of that size.")


if __name__ == "__main__":
    main()
