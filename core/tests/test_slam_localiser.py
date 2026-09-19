"""
core/tests/test_slam_localiser.py

Chunk 23's assembly: `SlamLocaliser` under `core.localisation.Localiser`.

The tests here are about WIRING and about the fences, not about accuracy. How
accurate the thing is depends on the scene, the trajectory and the noise, and
that measurement lives in `out/evidence/localisers/` where it can carry its
conditions with it — an accuracy threshold hidden in a unit test is a
pre-registered constant nobody registered.

⚠️ Every rollout here is short and every micro-world is small, deliberately.
One correlative match costs ~44 ms, so a full 120 s benchmark episode takes
minutes; a test suite that ran one would stop being run.

SIM-FREE. No simulator, no GPU.
"""

import dataclasses as dc
import math

import numpy as np
import pytest

from core.localisation import Localiser, make_localiser
from core.scenario import (DEV_SCENARIOS, LocalisationSpec, SensorNoiseSpec,
                           validate)
from core.sensor_noise import make_sensor_noise
from core.slam import scan as scan_mod
from core.slam.graph import CLOSURE, ODOMETRY
from core.slam.localiser import DEFAULT_SCAN_RATE, SlamLocaliser, _closure_info
from core.slam.loop import CLOSURE_MAX_AMBIGUITY, CLOSURE_MIN_SCORE

# ---- below this line is data, not code.

NOISE = SensorNoiseSpec(sigma_range_frac=0.005, dropout=0.02,
                        spurious_max_range=0.01, sigma_angle=0.002)
SLAM = LocalisationSpec(kind="slam", odom_sigma_lin=0.12, odom_sigma_ang=0.060)

DT = 0.02

# ---- below this line is code, not data.


def scenario(**overrides):
    base = dc.replace(DEV_SCENARIOS[0], id="slam_fixture", sensor_noise=NOISE,
                      localisation=SLAM)
    return dc.replace(base, **overrides) if overrides else base


def drive(localiser, sc, steps=120, speed=0.5, start=None):
    """Walk forward from `start` and feed the localiser. Returns the poses."""
    sensor = make_sensor_noise(sc)
    angles = sc.beam_angles()
    x, y, yaw = start or sc.start
    poses = []
    for _ in range(steps):
        x += speed * DT * math.cos(yaw)
        y += speed * DT * math.sin(yaw)
        poses.append((x, y, yaw))
        ranges = sensor.scan((x, y), yaw, angles, sc.range_max, sc.world)
        localiser.update(DT, (x, y, yaw), ranges, angles)
    return poses


def fresh(sc=None):
    sc = sc or scenario()
    loc = make_localiser(sc)
    loc.reset(sc, sc.localisation_rng())
    return loc, sc


# -- the protocol, and the chunk-19 socket ---------------------------------

def test_make_localiser_builds_a_slam_localiser():
    """The guard chunk 19 raised for is retired: a scenario asking for SLAM now
    loads and runs, which is chunk 23's first Done-when."""
    sc = scenario()
    validate(sc)
    assert isinstance(make_localiser(sc), SlamLocaliser)


def test_it_satisfies_the_localiser_protocol():
    loc, _ = fresh()
    assert isinstance(loc, Localiser)
    for name in ("reset", "update", "estimate", "error_estimate",
                 "diagnostics"):
        assert callable(getattr(loc, name))


def test_the_estimate_starts_at_the_scenario_start():
    """The one pose a robot is entitled to know exactly: where the episode is
    defined to begin. It is also the graph anchor."""
    loc, sc = fresh()
    assert loc.estimate() == tuple(float(v) for v in sc.start)
    assert loc._graph.poses[0] == tuple(float(v) for v in sc.start)


def test_update_before_reset_is_an_error_not_a_silent_zero():
    loc = SlamLocaliser(SLAM, np.random.default_rng(0))
    with pytest.raises(RuntimeError, match="before reset"):
        loc.update(DT, (0.0, 0.0, 0.0), np.zeros(0), np.zeros(0))


def test_diagnostics_are_flat_scalars():
    """`run_episode.py` writes them straight to a csv row, so a nested value
    would produce a file no one notices is malformed until scoring."""
    loc, _ = fresh()
    drive(loc, scenario(), steps=40)
    d = loc.diagnostics()
    assert d
    for key, value in d.items():
        assert isinstance(key, str)
        assert isinstance(value, (int, float)) and not isinstance(value, bool)


def test_diagnostics_carry_no_wall_clock():
    """⚠️ A deliberate departure from chunk 23's build list, which asked for
    solve wall-clock in diagnostics(). The same chunk requires slam.csv to be
    byte-identical across two runs, and a timing is different every run. The
    timing lives on the attributes below instead."""
    loc, _ = fresh()
    drive(loc, scenario(), steps=40)
    assert not any("second" in k or "wall" in k or "time" in k
                   for k in loc.diagnostics())
    assert hasattr(loc, "last_solve_seconds")
    assert hasattr(loc, "total_solve_seconds")


# -- the fences ------------------------------------------------------------

def test_zero_odometry_noise_is_refused():
    """The remaining way to configure SLAM into being a free ride: perfect dead
    reckoning makes the estimate exact, leaves a closure nothing to correct, and
    reports a suspiciously excellent number. `PerfectLocaliser` is the honest
    way to ask for that."""
    sc = scenario(localisation=LocalisationSpec(kind="slam"))
    loc = make_localiser(sc)
    with pytest.raises(ValueError, match="PERFECT dead reckoning"):
        loc.reset(sc, sc.localisation_rng())


def test_the_estimate_is_not_the_truth():
    """The property that distinguishes this from `PerfectLocaliser`, and the
    one thing a wiring bug would silently undo."""
    loc, sc = fresh()
    poses = drive(loc, sc, steps=120)
    ex, ey, _ = loc.estimate()
    x, y, _ = poses[-1]
    assert math.hypot(x - ex, y - ey) > 1e-6


def test_error_estimate_is_a_prediction_not_the_realised_error():
    """`error_estimate()` must never be the realised error, or
    `IsLocalisationDegraded` becomes a node that reads the answer. It is the
    accumulated dead-reckoning uncertainty: a function of TIME and the noise
    spec, and of nothing the robot measured."""
    loc, sc = fresh()
    poses = drive(loc, sc, steps=120)
    predicted = loc.error_estimate()
    x, y, _ = poses[-1]
    ex, ey, _ = loc.estimate()
    realised = math.hypot(x - ex, y - ey)
    assert predicted != pytest.approx(realised, abs=1e-9)
    # sigma * sqrt(t) since the last accepted closure, so it is bounded by the
    # whole-episode figure and cannot be reading the map.
    assert 0.0 <= predicted <= SLAM.odom_sigma_lin * math.sqrt(120 * DT) + 1e-9


def test_uncertainty_rises_while_driving():
    loc, sc = fresh()
    sensor = make_sensor_noise(sc)
    angles = sc.beam_angles()
    x, y, yaw = sc.start
    seen = []
    for _ in range(60):
        x += 0.5 * DT
        ranges = sensor.scan((x, y), yaw, angles, sc.range_max, sc.world)
        loc.update(DT, (x, y, yaw), ranges, angles)
        seen.append(loc.error_estimate())
    assert seen[-1] > seen[0]
    assert all(b >= a for a, b in zip(seen, seen[1:]))     # no closure yet


# -- the scan rate ---------------------------------------------------------

def test_the_default_scan_rate_is_not_every_control_step():
    """MEASURED, not chosen: one match costs ~44 ms and the control rate is
    50 Hz, so a scan per control step does not run in real time and a 60 s
    episode does not finish. Chunk 24 may move this; it may not move it back."""
    assert DEFAULT_SCAN_RATE > 0.0
    assert DEFAULT_SCAN_RATE < 50.0


def test_between_scans_the_estimate_is_held_not_dead_reckoned():
    """A stale estimate is the honest consequence of a slow sensor. Dead
    reckoning it forward would need the true pose on a step where no
    measurement was taken."""
    loc, sc = fresh()
    sensor = make_sensor_noise(sc)
    angles = sc.beam_angles()
    x, y, yaw = sc.start
    ranges = sensor.scan((x, y), yaw, angles, sc.range_max, sc.world)
    loc.update(DT, (x, y, yaw), ranges, angles)          # first scan: due
    held = loc.estimate()
    for _ in range(int(1.0 / DEFAULT_SCAN_RATE / DT) - 2):
        x += 0.5 * DT
        loc.update(DT, (x, y, yaw),
                   sensor.scan((x, y), yaw, angles, sc.range_max, sc.world),
                   angles)
        assert loc.estimate() == held
    assert loc.diagnostics()["scans"] == 1


def test_a_faster_scan_rate_takes_more_scans_over_the_same_drive():
    slow, _ = fresh(scenario(localisation=dc.replace(SLAM, scan_rate=5.0)))
    fast, sc = fresh(scenario(localisation=dc.replace(SLAM, scan_rate=20.0)))
    drive(slow, sc, steps=100)
    drive(fast, sc, steps=100)
    assert fast.diagnostics()["scans"] > slow.diagnostics()["scans"]


# -- the graph the front end builds ----------------------------------------

def test_every_submap_gets_a_node_and_consecutive_ones_an_odometry_edge():
    loc, sc = fresh()
    drive(loc, sc, steps=500)
    d = loc.diagnostics()
    assert d["submaps"] >= 2
    assert len(loc._graph) == d["submaps"]
    odom = [e for e in loc._graph.edges if e.kind == ODOMETRY]
    assert [(e.i, e.j) for e in odom] == [(k, k + 1)
                                          for k in range(d["submaps"] - 1)]


def test_a_closure_edge_joins_two_submap_nodes_not_the_robot():
    """A proposal is expressed between the ROBOT and an old submap, and the
    robot is not a node. If the conversion were skipped the edge indices would
    be wrong and the optimiser would fold the wrong pair together."""
    # ⚠️ The rate is PINNED here rather than taken from DEFAULT_SCAN_RATE, and
    # the skip this replaces is why. This test is about edge-index conversion,
    # not about the scan rate -- but it needs closures to exist to have anything
    # to check, and how many submaps 800 steps finishes is a function of the
    # rate. When chunk 24a moved the default from 10 Hz to 5, this drive dropped
    # from 8 submaps to 4, stopped accepting any closure, and the test began
    # SKIPPING ITSELF silently. A conditional skip is a test that quietly stops
    # testing, so the dependency is pinned and the assertion is now
    # unconditional.
    sc = scenario(localisation=dc.replace(SLAM, scan_rate=10.0))
    loc = make_localiser(sc)
    loc.reset(sc, sc.localisation_rng())
    drive(loc, sc, steps=800)
    closures = [e for e in loc._graph.edges if e.kind == CLOSURE]
    assert closures, "no closure accepted; this test needs one to check"
    for e in closures:
        assert 0 <= e.i < len(loc._graph)
        assert 0 <= e.j < len(loc._graph)
        assert e.i != e.j
        assert e.robust is True


def test_closures_are_proposed_only_when_a_submap_finishes():
    """The localiser's policy, not the tracker's: a proposal is a wider match
    against every nearby submap, so proposing per scan multiplies the episode's
    dominant cost by the candidate count."""
    loc, sc = fresh()
    drive(loc, sc, steps=300)
    d = loc.diagnostics()
    # At most one proposal round per finished submap, times the candidates it
    # had. Far fewer than one round per scan, which is the thing being asserted.
    assert d["closures_considered"] <= d["scans"]
    assert d["solves"] <= d["submaps"]


def test_optimising_carries_the_submaps_and_the_live_pose_together():
    """A closure that moves submap c must move the robot with it, or the
    estimate and the map disagree and the next match starts from a guess the
    map does not support."""
    loc, sc = fresh()
    drive(loc, sc, steps=400)
    current = loc._tracker.current
    local = scan_mod.relative(current.origin, loc._tracker.pose)
    assert loc.estimate() == pytest.approx(
        np.array(scan_mod.compose(current.origin, local)))
    assert loc._tracker.pose == loc.estimate()
    for k, submap in enumerate(loc._tracker.submaps + [current]):
        assert submap.origin == loc._graph.poses[k]


def test_closure_information_is_scaled_by_the_match_quality():
    """`matcher.py` says a barely-unique match deserves less weight than a
    sharp one. This is where that is spent, and the gates bound both factors so
    the weight lands in (0, score] and can never be zero."""
    sharp = _closure_info(1.0, 0.0)
    blunt = _closure_info(CLOSURE_MIN_SCORE, CLOSURE_MAX_AMBIGUITY * 0.9)
    assert all(b < s for s, b in zip(sharp, blunt))
    assert all(v > 0.0 for v in blunt)


# -- determinism -----------------------------------------------------------

def test_two_identical_runs_produce_identical_estimates():
    """The property `scripts/check_determinism.sh` checks one level up, and the
    reason there is no convergence tolerance anywhere in `core/slam`."""
    sc = scenario()
    out = []
    for _ in range(2):
        loc = make_localiser(sc)
        loc.reset(sc, sc.localisation_rng())
        drive(loc, sc, steps=200)
        out.append((loc.estimate(), loc.error_estimate(), loc.diagnostics()))
    assert out[0] == out[1]


def test_reset_starts_a_clean_episode():
    """A submap or an edge surviving into the next episode is a correlation
    between episodes that the four-condition ablation is not allowed to have."""
    sc = scenario()
    loc = make_localiser(sc)
    loc.reset(sc, sc.localisation_rng())
    drive(loc, sc, steps=300)
    dirty = loc.diagnostics()
    assert dirty["submaps"] > 1

    loc.reset(sc, sc.localisation_rng())
    clean = loc.diagnostics()
    assert clean["submaps"] == 1
    assert clean["scans"] == 0
    assert clean["graph_edges"] == 0
    assert clean["closure_edges"] == 0
    assert loc.estimate() == tuple(float(v) for v in sc.start)


def test_the_localiser_rng_is_its_own_stream():
    """A controller that draws more randomness must not change the pose
    estimate, or the four conditions are not comparable. `localisation_rng` is
    salted separately from `rng` and `drift_rng` for exactly this."""
    sc = scenario()
    a, b = make_localiser(sc), make_localiser(sc)
    a.reset(sc, sc.localisation_rng())
    b.reset(sc, sc.localisation_rng())
    sc.rng().normal(size=10_000)              # a greedy controller
    drive(a, sc, steps=150)
    drive(b, sc, steps=150)
    assert a.estimate() == b.estimate()


# -- cost, and the threading question chunk 24a settled ---------------------

def test_the_phase_breakdown_sums_to_the_total():
    """`cost()` derives `other` by subtraction, so the parts cannot disagree
    with the total. This test is what makes that claim worth relying on: if a
    future phase is timed but not subtracted, `other` silently absorbs it and
    the budget stops being attributable."""
    loc, sc = fresh()
    drive(loc, sc, steps=300)
    c = loc.cost()
    parts = (c["match_seconds"] + c["closure_seconds"]
             + c["solve_seconds"] + c["other_seconds"])
    assert parts == pytest.approx(c["total_seconds"], abs=1e-9)
    # And the phases really did run, or the sum is trivially satisfied.
    assert c["matches"] > 0 and c["updates"] >= c["scans"] > 0


def test_wall_clock_is_not_in_the_diagnostics():
    """`slam.csv` must be byte-identical across two runs of one episode and a
    timing differs every run. Cost and determinism are reported through
    different channels, and this is the fence between them."""
    loc, sc = fresh()
    drive(loc, sc, steps=120)
    assert not [k for k in loc.diagnostics() if "second" in k or "wall" in k]


def test_a_slower_scan_rate_takes_fewer_scans():
    """The rate has to be a real lever before chunk 24a can set a budget with
    it. At 50 Hz against a 0.02 s step every update scans; at 5 Hz far fewer do.

    ⚠️ **The numbers here are MEASURED, and 5 Hz does not deliver 5 Hz.** Two
    separate effects, both found by chunk 24a and neither of them a rounding
    nicety:

    **Quantisation.** A scan can only fall on a control step, so a rate is
    realisable only as an integer number of them. 20 Hz asks for 2.5 steps and
    gets 3, which is 16.7 Hz — 83% of nominal.

    **Float accumulation.** `_since_scan` is summed in units of `dt`, and ten
    additions of 0.02 land on 0.19999999999999998, just under the 0.2 s
    threshold — so a 5 Hz request slips to every 11th step, 4.53 Hz, 91% of
    nominal.

    Both are deterministic, so nothing about reproducibility is at stake. What
    is at stake is that the scan rate is **pre-registration material** for
    cycle 2, and a document saying "5 Hz" would be describing an apparatus that
    ran at 4.53 Hz. Chunk 24a reports this and does not fix it: carrying the
    remainder instead of resetting to zero would move every committed SLAM
    number in `out/evidence/localisers/`, and that is a `Decided` for chunk 25,
    not a change to make in passing. See plans/done/24a-slam-cost-model.md.
    """
    counts = {}
    for rate in (50.0, 5.0):
        sc = scenario(localisation=dc.replace(SLAM, scan_rate=rate))
        loc = make_localiser(sc)
        loc.reset(sc, sc.localisation_rng())
        drive(loc, sc, steps=200)
        counts[rate] = loc.cost()["scans"]
    assert counts[50.0] == 200            # the control rate is exactly reached
    assert counts[5.0] == 19              # 1 + floor(199/11), NOT 1 + 199/10


def test_the_pipeline_is_byte_identical_across_blas_thread_counts():
    """⚠️ **The measurement that replaced chunk 24's proposed assertion.**

    Chunk 24's plan says to assert `OMP_NUM_THREADS == 1` at runtime, on the
    grounds that "a parallel sparse solve is a non-deterministic sparse solve".
    Chunk 24a measured it instead and the premise does not hold *for this
    pipeline*: 1 thread and 22 produce identical bytes.

    The reason is structural rather than lucky, which is what makes it worth a
    test. Nothing in `core/slam` reaches a BLAS threading threshold — `spsolve`
    is SuperLU and single-threaded, and the only dense products are the 3x3
    blocks in `graph.py` and an `(N,2) @ (2,2)` in `scan.py`. There is no
    thread-count-dependent reduction to be non-deterministic.

    So this test is a **regression guard on the reason**, not a ritual: add a
    dense solve, a large `einsum` or a threaded factorisation and it fails,
    which is exactly when the assertion question should reopen.
    """
    threadpoolctl = pytest.importorskip("threadpoolctl")
    pools = threadpoolctl.threadpool_info()
    most = max((p.get("num_threads") or 1) for p in pools) if pools else 1
    if most < 2:
        pytest.skip(f"only {most} BLAS thread available; nothing to contrast")

    def run(limit):
        with threadpoolctl.threadpool_limits(limits=limit, user_api="blas"):
            loc, sc = fresh()
            poses = drive(loc, sc, steps=260)
            # The full-precision stream, not the rounded diagnostics: a
            # rounded comparison would pass on a pipeline that differs in the
            # sixth decimal, which is precisely the failure being excluded.
            return (poses, loc.estimate(), loc.error_estimate(),
                    loc.diagnostics())

    assert run(1) == run(most), (
        f"the SLAM pipeline is not reproducible across BLAS thread counts "
        f"(1 vs {most}). Something now reaches a threaded reduction; see this "
        f"test's docstring before pinning threads to hide it.")
