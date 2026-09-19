#!/usr/bin/env python
"""
scripts/benchmark_slam.py

**What does SLAM cost, and what does each scan rate buy?** Chunk 24a's cost
table. SIM-FREE: no simulator, no GPU, no Isaac Sim.

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P scripts/benchmark_slam.py                        # both trajectories
    $P scripts/benchmark_slam.py --trajectory loop
    $P scripts/benchmark_slam.py --rates 50 20 10 5 2
    $P scripts/benchmark_slam.py --budget-only          # just the arithmetic

WHY A COST NUMBER IS ALLOWED TO BE SIM-FREE
-------------------------------------------
Every *reported* number in this project comes from the pinned container, and
this one will too -- the script is committed so that chunk 24 can run it there.
But the localiser's cost is a **pure function of the scans and poses it is
handed**, not of what produced them: `SlamLocaliser.update()` takes a dt, a
pose, and an array of ranges, and cannot tell a physics step from a kinematic
one. Isaac Sim adds a constant that this script does not measure and does not
need to, because that constant is already known and is not a SLAM cost:
~0.9 s per rendered frame, and a 15 s episode in 21 s headless.

So the rate can be **decided here and confirmed there**, which is the point --
renting a GPU to discover a number that a laptop can measure is the expensive
way round.

⚠️ WHAT THIS SCRIPT DOES NOT ESTABLISH
---------------------------------------
Determinism under physics. That is chunk 24's, it needs the container, and no
amount of CPU-side measurement substitutes for it.

COST AND ACCURACY IN ONE TABLE, DELIBERATELY
---------------------------------------------
The scan rate is a **trade**, not a knob with a right setting: a slower sensor
is cheaper and knows less. A table of wall clock alone would let the rate be
chosen by cost and then justified by accuracy afterwards, which is choosing the
answer. Both columns are printed together, on the same trajectory, at the same
seed, so the trade is visible in one row.

The accuracy columns are produced exactly as `compare_localisers.py` produces
them -- one trajectory generated once under a perfect pose and **replayed**
open-loop through each configuration -- and the reason is that script's: driving
each localiser confounds the estimator with how a controller reacted to it, and
the effect is large enough to invert the answer. This script imports that
machinery rather than restating it.

THE PHASE BREAKDOWN
-------------------
`SlamLocaliser.cost()` splits an episode into matching, closure search, solving
and a remainder. The remainder is the interesting column: likelihood fields,
scan endpoints, submap accumulation and graph maintenance are the costs nobody
budgets for, and they are derived by subtraction so the parts and the total
cannot disagree.

⚠️ Timings are wall clock and therefore differ every run. They are read off
attributes and are **never** in `slam.csv`, which has to be byte-identical
across two runs of one episode. Cost and determinism are measured on the same
pipeline and reported through different channels; that is the only way to have
both. See `SlamLocaliser.cost()`.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np

# ---- below this line is data, not code.

# The rates put on trial, in Hz. Chosen to bracket the decision rather than to
# flatter it:
#
#   50  every control step. Chunk 23 measured this as unreachable (~2.2 s of
#       matching per simulated second) and it is here to be shown unreachable
#       rather than asserted so. It is the only rate that is not a sensor.
#   20  the fast end of real 2D lidar.
#   10  the middle of the real range, and what chunk 23 assumed.
#    5  DEFAULT_SCAN_RATE since chunk 24a: the slow end of real 2D lidar, and
#       measured 3.3-4.1x cheaper than 10 Hz at indistinguishable accuracy.
#    2  slower than any real sensor. Here as the floor: if accuracy survives it,
#       the accuracy is not coming from the scan rate.
CANDIDATE_RATES = (50.0, 20.0, 10.0, 5.0, 2.0)

# The insult, matching compare_localisers.py so the two tables are comparable.
SIGMA_XY = 0.12
SIGMA_YAW_RATIO = 0.5

# What a rate must not exceed to be affordable, as a fraction of real time.
# ⚠️ NOT a tuned number: at 1.0 the localiser costs a second of wall clock per
# simulated second, so an episode's SLAM cost equals its own duration and the
# sweep's wall clock doubles. Above that SLAM dominates the physics it is
# attached to, which is the point at which the cost is a property of the
# localiser rather than of the benchmark.
BUDGET_RATIO = 1.0

# ---- below this line is code, not data.


def log(msg):
    print(f"[benchmark_slam] {msg}", flush=True)


def _slam_spec(rate, sigma):
    from core.scenario import LocalisationSpec
    return LocalisationSpec(kind="slam", odom_sigma_lin=sigma,
                            odom_sigma_ang=sigma * SIGMA_YAW_RATIO,
                            scan_rate=rate)


def measure(poses, scenario, localiser, blind=False):
    """Replay `poses`, returning (errors, wall clock, cost dict or None)."""
    from compare_localisers import replay

    t0 = time.perf_counter()
    errs = replay(poses, scenario, localiser, blind=blind)
    wall = time.perf_counter() - t0
    cost = localiser.cost() if hasattr(localiser, "cost") else None
    return errs, wall, cost


def run_table(label, poses, base_scenario, rates, sigma):
    """One trajectory, every configuration. Returns rows for the budget."""
    import dataclasses as dc

    from core.localisation import make_localiser
    from core.scenario import DriftSpec, LocalisationSpec

    sim_seconds = len(poses) * DT
    log(f"{label}: {len(poses)} steps, {sim_seconds:.1f} sim seconds")

    header = (f"  {'localiser':>14s} {'wall':>7s} {'x real':>7s} "
              f"{'match':>7s} {'closure':>8s} {'solve':>7s} {'other':>7s} "
              f"{'scans':>6s} {'eff Hz':>7s} "
              f"{'final':>7s} {'mean':>7s} {'max':>7s}")
    print(header, flush=True)
    print("  " + "-" * (len(header) - 2), flush=True)

    rows = []
    # The two controls first. Neither matches anything, so both are here to
    # establish what the localiser machinery costs when it does no SLAM --
    # without them, "SLAM costs 0.4x real time" has no zero.
    controls = (("perfect", dc.replace(
                    base_scenario,
                    localisation=LocalisationSpec(kind="perfect"))),
                ("drift", dc.replace(
                    base_scenario,
                    drift=DriftSpec(sigma_xy=sigma,
                                    sigma_yaw=sigma * SIGMA_YAW_RATIO))))
    for name, scenario in controls:
        localiser = make_localiser(scenario)
        localiser.reset(scenario, scenario.localisation_rng())
        errs, wall, _ = measure(poses, scenario, localiser)
        print(f"  {name:>14s} {wall:7.2f} {wall / sim_seconds:7.3f} "
              f"{'-':>7s} {'-':>8s} {'-':>7s} {'-':>7s} {'-':>6s} "
              f"{'-':>7s} "
              f"{errs[-1]:7.3f} {np.mean(errs):7.3f} {max(errs):7.3f}", flush=True)

    for rate in rates:
        scenario = dc.replace(base_scenario,
                              localisation=_slam_spec(rate, sigma))
        localiser = make_localiser(scenario)
        localiser.reset(scenario, scenario.localisation_rng())
        errs, wall, cost = measure(poses, scenario, localiser)
        ratio = cost["total_seconds"] / sim_seconds
        # ⚠️ THE DELIVERED RATE, not the requested one, and they differ. A scan
        # lands on a control step, so a rate is realisable only as an integer
        # number of them -- 20 Hz asks for 2.5 steps and gets 3, which is
        # 16.7 Hz. And `_since_scan` accumulates in units of dt, so ten
        # additions of 0.02 reach 0.19999999999999998 and a 5 Hz request slips
        # to every 11th step. Both are deterministic; both mean the scan rate a
        # pre-registration names is not the one the apparatus ran at. Printed
        # rather than corrected -- see plans/done/24a-slam-cost-model.md for
        # why the fix belongs to chunk 25.
        effective = cost["scans"] / sim_seconds
        print(f"  {'slam @ %g Hz' % rate:>14s} {cost['total_seconds']:7.2f} "
              f"{ratio:7.3f} "
              f"{cost['match_seconds']:7.2f} {cost['closure_seconds']:8.2f} "
              f"{cost['solve_seconds']:7.2f} {cost['other_seconds']:7.2f} "
              f"{cost['scans']:6d} {effective:7.2f} "
              f"{errs[-1]:7.3f} {np.mean(errs):7.3f} {max(errs):7.3f}", flush=True)
        rows.append({"trajectory": label, "rate": rate, "ratio": ratio,
                     "effective": effective,
                     "final": errs[-1], "mean": float(np.mean(errs)),
                     "cost": cost})
    print(flush=True)
    return rows


def scaling(rate, sigma, laps, scenario_id):
    """Does the cost per simulated second stay CONSTANT as an episode lengthens?

    ⚠️ **The budget in `budget()` multiplies one ratio by the sweep's total
    sim-seconds, which is only valid if it does.** The traverse table gives a
    reason to doubt it: at the control rate, closure search costs twice what
    matching does, and a closure proposal is a match against every nearby
    finished submap -- so the per-round cost grows with the number of submaps,
    which grows with the episode. If that is quadratic, a budget extrapolated
    from a 15 s trajectory under-reads a 240 s one, and it under-reads it in
    the direction that matters.

    Matching is the linear part: one scan, one match, whatever came before.
    Closure search is the suspect part. They are reported separately so the
    answer says WHICH, and not merely whether.
    """
    import dataclasses as dc

    from compare_localisers import NOISE, loop_poses
    from core.localisation import make_localiser
    from core.scenario import load

    log(f"scaling at {rate:g} Hz: is cost per simulated second constant?")
    print(f"  {'laps':>5s} {'sim s':>7s} {'submaps':>8s} {'closed':>7s} "
          f"{'match':>7s} {'closure':>8s} {'solve':>7s} {'total':>7s} "
          f"{'x real':>7s} {'final':>7s} {'mean':>7s}", flush=True)
    print("  " + "-" * 87, flush=True)
    out = []
    for n in laps:
        poses = loop_poses(n)
        sim_seconds = len(poses) * DT
        sc = dc.replace(load(scenario_id), id=f"scale{n}_bench",
                        time_limit=sim_seconds, sensor_noise=NOISE, drift=None,
                        localisation=_slam_spec(rate, sigma))
        loc = make_localiser(sc)
        loc.reset(sc, sc.localisation_rng())
        errs, _, _ = measure(poses, sc, loc)
        c = loc.cost()
        d = loc.diagnostics()
        subs = d["submaps"]
        ratio = c["total_seconds"] / sim_seconds
        # ⚠️ ACCURACY IS IN THIS TABLE TOO, and it is not decoration. A slower
        # rate is cheaper AND takes fewer scans before a submap finishes, so it
        # reaches the submap count a closure needs later. On a SHORT episode
        # that can mean no closures at all -- which looks like "the rate is too
        # slow" when it is really "the episode is too short". The `closed`
        # column is what tells those two apart, and the whole rate decision
        # turns on it: see plans/done/24a-slam-cost-model.md.
        print(f"  {n:5d} {sim_seconds:7.1f} {subs:8d} "
              f"{d['closure_edges']:7d} "
              f"{c['match_seconds']:7.2f} "
              f"{c['closure_seconds']:8.2f} {c['solve_seconds']:7.2f} "
              f"{c['total_seconds']:7.2f} {ratio:7.3f} "
              f"{errs[-1]:7.3f} {np.mean(errs):7.3f}", flush=True)
        out.append((sim_seconds, ratio, errs[-1]))
    print(flush=True)
    first, last = out[0], out[-1]
    growth = last[1] / first[1] if first[1] else float("nan")
    span = last[0] / first[0]
    log(f"episode {span:.1f}x longer -> cost per simulated second {growth:.2f}x")
    if growth > 1.25:
        log("⚠️ SUPERLINEAR. A budget extrapolated from a short trajectory "
            "UNDER-reads a long one; see this function's docstring.")
    else:
        log("linear enough to extrapolate: the budget table is sound.")
    print(flush=True)


def budget(rows):
    """What the pre-registered episode set would cost, per rate.

    Derived from `runs.design` rather than typed, so it tracks the frozen
    constants instead of a number someone copied out of a table once.

    ⚠️ The episode COUNT is taken per-condition and multiplied by
    `runs.spec.CONDITION_LABELS`, not read off whichever spec happened to load.
    The only committed spec is `rehearsal.json`, which binds two conditions
    because the machinery needs two to compare -- so counting its episodes
    directly would under-budget the real sweep by half. G2 runs four.

    Held-out ids are never printed. The split is sealed and `runs/sweep.py`
    redacts them by default; a cost report has no business revealing what a
    sweep log will not.
    """
    from runs.design import sweep_design
    from runs.spec import CONDITION_LABELS

    plan = sweep_design(_counting_spec())
    conditions = len(CONDITION_LABELS)
    # One condition's worth, then scaled. Every condition runs the identical
    # episode set -- non-negotiable #10 -- so this division is exact and not an
    # average.
    first = plan.episodes[0].condition
    per_condition = [e for e in plan.episodes if e.condition == first]
    episodes = len(per_condition) * conditions
    sim_seconds = sum(e.time_limit for e in per_condition) * conditions

    log(f"budget: {len(per_condition)} episodes per condition x {conditions} "
        f"conditions = {episodes} episodes, {sim_seconds:.0f} sim-seconds")
    log("        WORST CASE: every episode running to its full time_limit. An "
        "episode that reaches the goal stops there.")
    log(f"        chunk 25 makes a PerfectLocaliser control MANDATORY for every "
        f"SLAM condition, so the SLAM hours below DOUBLE for a reported run.")

    if not rows:
        return

    # ⚠️ A RANGE, not a number, and the width of it is the finding. SLAM's cost
    # is dominated by closure search, a closure proposal is a wide match against
    # every finished submap within SEARCH_RADIUS, and how many of those there
    # are depends on WHERE THE ROBOT WENT. A one-way traverse proposes almost
    # nothing; a trajectory that revisits proposes against everything it has
    # already built. So the sweep's wall clock is a function of controller
    # behaviour -- which is the thing the experiment is measuring. Collapsing
    # that to a single number would hide the one property a scheduler needs.
    print(f"  {'rate':>10s} {'cheapest':>9s} {'dearest':>9s} "
          f"{'SLAM hours':>18s} {'+control':>10s} {'affordable':>11s}",
          flush=True)
    print("  " + "-" * 72, flush=True)
    for rate in sorted({r["rate"] for r in rows}, reverse=True):
        at = [r["ratio"] for r in rows if r["rate"] == rate]
        lo, hi = min(at), max(at)
        lo_h, hi_h = lo * sim_seconds / 3600.0, hi * sim_seconds / 3600.0
        # Judged on the DEAREST trajectory. A budget set on the cheap half is a
        # budget that is wrong exactly when it matters.
        verdict = "yes" if hi <= BUDGET_RATIO else "NO"
        print(f"  {'%g Hz' % rate:>10s} {lo:9.3f} {hi:9.3f} "
              f"{'%.1f - %.1f' % (lo_h, hi_h):>18s} "
              f"{'%.1f - %.1f' % (lo_h * 2, hi_h * 2):>10s} "
              f"{verdict:>11s}", flush=True)
    print(flush=True)
    log(f"affordable means <= {BUDGET_RATIO:g}x real time; see BUDGET_RATIO")
    log("this is the SLAM cost ALONE -- physics and the policy are on top, and "
        "are the same for every condition")
    # ⚠️ The extrapolation is the weak step, and it is weak in the UNSAFE
    # direction. The ratios above are measured on a 14.5 s and a 77.7 s
    # trajectory; the held-out scenarios carry time_limits of 120-240 s. Closure
    # search proposes against every finished submap within SEARCH_RADIUS, and in
    # a room smaller than the sensor range that candidate set GROWS with the
    # episode -- so cost per simulated second need not be constant, and a ratio
    # measured on a short trajectory would under-read a long one. `--scaling`
    # is the test of exactly this; run it before trusting a row above.
    log("⚠️ ratios are measured on trajectories SHORTER than the held-out "
        "episodes (120-240 s). Run --scaling before trusting the extrapolation")


def _counting_spec():
    """A spec to count episodes with. Any binding gives the same COUNT.

    The episode set is a function of the frozen constants; a spec supplies the
    condition BINDING, which a cost estimate does not care about. `rehearsal`
    is used because it is the one marked `reportable:false` -- a cost estimate
    should not have to load an evaluation binding to do arithmetic. Its two
    conditions are corrected for in `budget()`.
    """
    from runs.spec import load_spec
    return load_spec("runs/specs/rehearsal.json")


def main():
    ap = argparse.ArgumentParser(
        description="SLAM cost per scan rate, and the sweep budget")
    ap.add_argument("--trajectory", choices=("traverse", "loop", "both"),
                    default="both")
    ap.add_argument("--scenario", default="doorway_traverse",
                    help="the dev scenario the traverse is driven on")
    ap.add_argument("--rates", type=float, nargs="+", default=CANDIDATE_RATES,
                    help=f"scan rates in Hz, default {CANDIDATE_RATES}")
    ap.add_argument("--sigma", type=float, default=SIGMA_XY,
                    help=f"odometry noise, m/sqrt(s), default {SIGMA_XY}")
    ap.add_argument("--laps", type=int, default=2)
    ap.add_argument("--scaling", type=int, nargs="+", metavar="LAPS",
                    default=None,
                    help="instead of the cost table, run one rate over "
                         "increasing episode lengths and test whether cost per "
                         "simulated second is constant. e.g. --scaling 1 2 3 4")
    ap.add_argument("--budget-only", action="store_true",
                    help="skip the measurement and print the episode set's "
                         "size only")
    args = ap.parse_args()

    import dataclasses as dc

    from compare_localisers import (NOISE, loop_poses, traverse_poses)
    from core.scenario import load

    if args.budget_only:
        budget([])
        return

    if args.scaling:
        scaling(args.rates[0] if len(args.rates) == 1 else 10.0,
                args.sigma, args.scaling, args.scenario)
        return

    log(f"threads: OMP={os.environ.get('OMP_NUM_THREADS', 'unset')} "
        f"MKL={os.environ.get('MKL_NUM_THREADS', 'unset')} "
        f"-- measured irrelevant to the RESULT (chunk 24a), and left visible "
        f"because it is not irrelevant to the TIMING")

    wanted = (("traverse", "loop") if args.trajectory == "both"
              else (args.trajectory,))
    rows = []
    for label in wanted:
        poses = (traverse_poses(args.scenario) if label == "traverse"
                 else loop_poses(args.laps))
        base = dc.replace(load(args.scenario), id=f"{label}_bench",
                          time_limit=len(poses) * DT, sensor_noise=NOISE,
                          drift=None)
        rows += run_table(label, poses, base, args.rates, args.sigma)

    budget(rows)


# DT is the control step, and it is imported rather than restated so that a
# change to the kinematic rollout's step cannot silently make this table wrong.
from compare_localisers import DT                             # noqa: E402


if __name__ == "__main__":
    main()
