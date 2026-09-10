"""
core/tests/test_slam_record.py

Chunk 24b's recorder: `core/slam/record.py`, and the one property everything
drawn from it depends on.

⚠️ **The load-bearing test in this file is `test_recording_is_inert`.** The
whole reason `SlamLocaliser.enable_recording()` exists is to produce pictures
that are evidence about the run that was scored. A recorder that perturbed the
estimate — by drawing from the rng, by changing an order of operations, by
anything — would make every picture evidence about a DIFFERENT episode, and it
would do so silently. `nodes/trace.py` made the same promise about `trace.csv`
and this is the same test, one epic later.

The second property is that the map reads back exactly: `points_at()` at the end
of an episode must equal `Submap.global_points()`, bit for bit and not to some
tolerance. It is exact because both go through `scan.transform`, and the reason
that matters is that a renderer with its own rotation would drift from the
pipeline's in the last bit and nobody would ever find it.

SIM-FREE. No simulator, no GPU.
"""

import dataclasses as dc
import math
import os

import numpy as np

from core.localisation import make_localiser
from core.scenario import DEV_SCENARIOS, LocalisationSpec, SensorNoiseSpec
from core.sensor_noise import make_sensor_noise
from core.slam.matcher import MAX_SPREAD, MAX_SPREAD_THETA, MIN_SCORE
from core.slam.record import (EDGE_CLOSURE, EDGE_ODOMETRY, NODE_ADDED, SOLVED,
                              MapRecorder, RecordedMap)

# ---- below this line is data, not code.

NOISE = SensorNoiseSpec(sigma_range_frac=0.005, dropout=0.02,
                        spurious_max_range=0.01, sigma_angle=0.002)
SLAM = LocalisationSpec(kind="slam", odom_sigma_lin=0.12, odom_sigma_ang=0.060,
                        # Every control step, so a short rollout still fills
                        # submaps and exercises the graph. Cost is why the
                        # benchmark does not do this; 160 steps is why a test
                        # can.
                        scan_rate=50.0)
DT = 0.02

# ---- below this line is code, not data.


def scenario(**overrides):
    base = dc.replace(DEV_SCENARIOS[0], id="record_fixture", sensor_noise=NOISE,
                      localisation=SLAM)
    return dc.replace(base, **overrides) if overrides else base


def drive(localiser, sc, steps=160, speed=0.5):
    """Walk forward and feed the localiser. Returns the estimate at every step."""
    sensor = make_sensor_noise(sc)
    angles = sc.beam_angles()
    x, y, yaw = sc.start
    out = []
    for _ in range(steps):
        x += speed * DT * math.cos(yaw)
        y += speed * DT * math.sin(yaw)
        ranges = sensor.scan((x, y), yaw, angles, sc.range_max, sc.world)
        localiser.update(DT, (x, y, yaw), ranges, angles)
        out.append((localiser.estimate(), localiser.error_estimate(),
                    dict(localiser.diagnostics())))
    return out


# One correlative match costs ~44 ms, so the two rollouts below are the whole
# cost of this file. They are deterministic and no test mutates the localiser,
# so they are computed once and shared -- the alternative is a two-minute test
# file, and a test file that slow stops being run.
_RUNS = {}


def run(record):
    if record not in _RUNS:
        sc = scenario()
        loc = make_localiser(sc)
        if record:
            loc.enable_recording()
        loc.reset(sc, sc.localisation_rng())
        _RUNS[record] = (loc, drive(loc, sc))
    return _RUNS[record]


# -- the one that matters --------------------------------------------------

def test_recording_is_inert():
    """A recorded run IS the unrecorded run.

    Full precision on the pose stream, not `approx`: the project's determinism
    guarantee is bitwise, and a recorder that moved the estimate by an ulp would
    still have moved it.
    """
    plain, plain_steps = run(record=False)
    taped, taped_steps = run(record=True)

    assert [s[0] for s in plain_steps] == [s[0] for s in taped_steps]
    assert [s[1] for s in plain_steps] == [s[1] for s in taped_steps]
    assert [s[2] for s in plain_steps] == [s[2] for s in taped_steps]
    assert plain.diagnostics() == taped.diagnostics()


def test_recording_is_off_by_default():
    """It costs a copy of every scan point, so a sweep must not pay for it
    without asking."""
    loc, _ = run(record=False)
    assert loc.recorder is None


def test_reset_starts_a_new_recorder():
    """A submap surviving into the next episode is a correlation between
    episodes the ablation may not have, and so is a recording."""
    sc = scenario()
    loc = make_localiser(sc)
    loc.enable_recording()
    loc.reset(sc, sc.localisation_rng())
    drive(loc, sc, steps=60)
    first = loc.recorder
    assert first.scans
    loc.reset(sc, sc.localisation_rng())
    assert loc.recorder is not first
    assert loc.recorder.scans == 0


# -- the map reads back exactly --------------------------------------------

def test_points_at_the_end_are_the_submaps_own_points():
    """`points_at()` reproduces `Submap.global_points()` for the whole map.

    Exactly, not approximately. Both go through `scan.transform`, and the point
    of pinning it is that a renderer with its own open-coded rotation would
    differ in the last bit and the disagreement would never be found.
    """
    loc, _ = run(record=True)
    tracker = loc._tracker
    truth = np.concatenate(
        [s.global_points() for s in tracker.submaps + [tracker.current]])
    mine = loc.recorder.points_at(loc.recorder.scans)
    assert mine.shape == truth.shape
    assert np.array_equal(mine, truth)


def test_the_map_moves_when_the_graph_moves():
    """The same scan drawn against two different graph states lands in two
    different places — which is the correction a closure applies, and the only
    reason to store local points and a pose history rather than world points."""
    loc, _ = run(record=True)
    rec = loc.recorder
    moved = rec.points_at(rec.scans)
    # Shift every node by a metre in x and the whole map must come with it.
    poses = rec.poses_at(rec.scans).copy()
    poses[:, 0] += 1.0
    shifted = rec.points_at(rec.scans, poses)
    assert np.allclose(shifted[:, 0] - moved[:, 0], 1.0)
    assert np.allclose(shifted[:, 1], moved[:, 1])


def test_scan_indices_line_up_with_the_slam_csv_column():
    """The recorder's scan index IS `slam.csv`'s `scans` value on that step.

    That equality is the entire join between the two files. If it drifts, every
    picture is drawn against the wrong instant and nothing raises.
    """
    sc = scenario()
    loc = make_localiser(sc)
    loc.enable_recording()
    loc.reset(sc, sc.localisation_rng())
    sensor = make_sensor_noise(sc)
    angles = sc.beam_angles()
    x, y, yaw = sc.start
    for _ in range(80):
        x += 0.5 * DT * math.cos(yaw)
        y += 0.5 * DT * math.sin(yaw)
        loc.update(DT, (x, y, yaw),
                   sensor.scan((x, y), yaw, angles, sc.range_max, sc.world),
                   angles)
        assert loc.recorder.scans == loc.diagnostics()["scans"]


def test_poses_at_forward_fills_and_is_never_interpolated():
    """⚠️ Two snapshots can share one scan index, and `poses_at` must return the
    LATER of them.

    That is not a corner case: a scan that finishes a submap adds a node and,
    if the closure round that follows accepts anything, solves — both at the
    same scan. The map has to be drawn against the state the scan ENDED in, or
    every closure appears one scan late. The first version of this test asserted
    every snapshot was reachable and failed on exactly that pair.
    """
    # Built directly rather than driven, and that is the point: forward-fill is
    # a property of MapRecorder, not of the SLAM pipeline. An earlier version
    # drove a rollout and relied on it producing a coincident pair — then chunk
    # 22a's travel gate removed the degenerate closures that rollout was
    # getting, the pair stopped occurring, and the test's own guard caught it.
    # A unit test of a lookup should not depend on a matcher accepting a loop.
    rec = MapRecorder((0.0, 0.0, 0.0))
    rec.graph([(0.0, 0.0, 0.0)], NODE_ADDED)                  # scan 0
    rec.scan(0, np.zeros((0, 2)))                             # -> scan 1
    rec.scan(0, np.zeros((0, 2)))                             # -> scan 2
    added = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    solved = [(0.0, 0.0, 0.0), (1.5, 0.2, 0.1)]
    rec.graph(added, NODE_ADDED)                              # both at scan 2
    rec.graph(solved, SOLVED)

    ats = [at for at, _r, _p in rec._hist]
    assert ats == [0, 2, 2], "the fixture must contain a coincident pair"
    # ⚠️ THE PROPERTY: the LATER snapshot wins. A scan that finishes a submap
    # adds a node and, if the closure round that follows accepts anything,
    # solves — both at the same scan. The map has to be drawn against the state
    # the scan ENDED in, or every closure appears one scan late.
    assert np.array_equal(rec.poses_at(2), np.array(solved))
    # Forward-filled, never interpolated: before the pair, the scan-0 snapshot
    # stands unchanged.
    assert np.array_equal(rec.poses_at(1), np.array([(0.0, 0.0, 0.0)]))
    assert np.array_equal(rec.poses_at(99), np.array(solved))

    # And on a real rollout the same lookup agrees with the last snapshot at
    # every index it holds.
    loc, _ = run(record=True)
    live = loc.recorder
    last_at = {}
    for at, _reason, poses in live._hist:
        last_at[at] = poses
    for at, poses in last_at.items():
        assert np.array_equal(live.poses_at(at), poses)


# -- the graph is described, not just drawn --------------------------------

def test_every_odometry_edge_chains_consecutive_submaps():
    loc, _ = run(record=True)
    rec = loc.recorder
    odom = [e for e in rec._edges if e[2] == EDGE_ODOMETRY]
    assert odom
    for i, j, _kind, _at, *_info in odom:
        assert j == i + 1


def test_the_recorded_graph_matches_the_localisers_graph():
    """Not a second graph. The recorder is a view of the one that ran."""
    loc, _ = run(record=True)
    rec = loc.recorder
    assert rec.n_submaps == len(loc._graph)
    assert len(rec._edges) == len(loc._graph.edges)
    assert np.allclose(rec._final_poses, np.array(loc._graph.poses))
    kinds = [EDGE_CLOSURE if e.kind == "closure" else EDGE_ODOMETRY
             for e in loc._graph.edges]
    assert [e[2] for e in rec._edges] == kinds


def test_a_solve_is_recorded_as_a_solve():
    """`solve_scans()` is what the error plot marks; a node addition must not
    show up as one."""
    loc, _ = run(record=True)
    rec = loc.recorder
    assert len(rec.solve_scans()) == loc.diagnostics()["solves"]
    reasons = {r for _at, r, _p in rec._hist}
    assert reasons <= {NODE_ADDED, SOLVED}


# -- persistence -----------------------------------------------------------

def test_the_npz_round_trips_without_pickle(tmp_path):
    """`allow_pickle=False` on load is the point: a file that needs pickle to
    read is a file this repo should not be writing."""
    loc, _ = run(record=True)
    rec = loc.recorder
    path = str(tmp_path / "slam_map.npz")
    rec.save_npz(path)
    back = RecordedMap.load(path)

    assert back.n_submaps == rec.n_submaps
    assert back.n_scans == rec.scans
    assert np.array_equal(back.points_at(back.n_scans),
                          rec.points_at(rec.scans))
    assert back.closures() == [(int(a), int(b), int(c))
                               for a, b, c in rec.closures()]
    assert back.solve_scans() == rec.solve_scans()
    assert back.meta["scan_rate"] == str(SLAM.scan_rate)


def test_an_unrecognised_npz_says_so_rather_than_raising_a_keyerror(tmp_path):
    path = str(tmp_path / "not_a_map.npz")
    np.savez_compressed(path, points=np.zeros((3, 2)))
    try:
        RecordedMap.load(path)
    except ValueError as exc:
        assert "not a slam map file" in str(exc)
        assert "final_poses" in str(exc)
    else:                                              # pragma: no cover
        raise AssertionError("a file missing every array but one was accepted")


def test_decimate_thins_points_and_shares_everything_else(tmp_path):
    """The animation redraws the whole cloud per frame; the stills do not.
    Thinning must touch the points and nothing else."""
    loc, _ = run(record=True)
    path = str(tmp_path / "slam_map.npz")
    loc.recorder.save_npz(path)
    full = RecordedMap.load(path)
    thin = full.decimate(4)
    assert len(thin.points) == len(full.points[::4])
    assert thin.n_submaps == full.n_submaps
    assert np.array_equal(thin.final_poses, full.final_poses)
    assert thin.closures() == full.closures()
    assert full.decimate(1) is full


def test_an_empty_recorder_still_saves(tmp_path):
    """A localiser that was reset and never updated. The renderer's failure
    mode should be an empty picture, never a crash in numpy."""
    rec = MapRecorder((0.0, 0.0, 0.0))
    path = str(tmp_path / "empty.npz")
    rec.save_npz(path)
    back = RecordedMap.load(path)
    assert back.n_scans == 0
    assert len(back.points_at(0)) == 0
    assert os.path.exists(path)


# -- what the sidecar has to be able to explain ----------------------------

def test_the_diagnostics_carry_every_factor_of_the_convergence_verdict():
    """⚠️ A regression test for a DIAGNOSTIC gap, not for a behaviour.

    `MatchResult.converged` is the AND of three conditions — score,
    translational spread, rotational spread. `slam.csv` carried the first two,
    so on chunk 24b's physics episode 80% of the matches read as "not
    converged" beside a healthy score and a small translational ambiguity and
    nothing in the file could say why. The deciding number was the one that was
    missing.

    This asserts the file can explain its own verdict. It belongs with the
    recorder's tests because the recorder's whole job is producing artefacts
    somebody reads later, and an unexplainable column is the same class of
    failure as an unreadable one.
    """
    loc, _ = run(record=True)
    d = loc.diagnostics()
    for key in ("match_score", "match_ambiguity", "match_ambiguity_theta",
                "match_converged"):
        assert key in d, f"slam.csv lost {key}"

    tracker = loc._tracker
    m = tracker.last_match
    assert m is not None
    assert d["match_converged"] == int(
        m.score >= MIN_SCORE
        and m.ambiguity <= MAX_SPREAD
        and m.ambiguity_theta <= MAX_SPREAD_THETA), \
        "the verdict is no longer reconstructible from the columns beside it"
