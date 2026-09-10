"""
core/slam/tracker.py

**Odometry chaining and submap management.** Chunk 22's third piece.

The loop, once per scan:

    noisy odometry  ->  a guess at where we are now
    match the scan against the CURRENT submap  ->  correct the guess
    add the scan to the current submap
    submap full?  ->  finish it, start a new one at the corrected pose
    propose closures against FINISHED submaps  ->  hand them to the graph

This module **detects and proposes**; it never corrects the trajectory. Applying
a closure is the graph optimisation in chunk 23, and keeping the two apart is
what lets 22's gates be tested without a solver in the loop.

WHERE THE ERROR COMES FROM
--------------------------
`update()` is handed the true pose, which is the simulator standing in for wheel
encoders and an IMU. **That is not a free ride**, because the tracker never uses
it directly: it takes the *delta* between successive true poses, corrupts it with
`LocalisationSpec.odom_sigma_lin/ang`, and composes the result onto its own
estimate. The true pose therefore supplies the motion the robot actually made
and nothing else — no absolute position, no correction, no ground truth about
where the robot is.

That corruption is the whole experiment. Without it the estimate is exact, a
loop closure has nothing to correct, and the epic's claim that *the pose estimate
becomes a consequence of what the robot did* cannot be measured.

SIM-FREE.
"""

import time

import numpy as np

from core.slam import loop as loop_mod
from core.slam import scan as scan_mod
from core.slam.matcher import match
from core.slam.submap import SCANS_PER_SUBMAP, Submap

# ---- below this line is data, not code.

# How many scans a submap must hold before anything is matched against it. A
# thin submap is not a map: its field is one scan's outline, and the next scan
# is the same outline, so the match is a self-fit that any wall-sliding rotation
# satisfies. Measured at 1-3 scans: score 1.00 with a heading 0.6 rad wrong.
MIN_SCANS_TO_MATCH = 5

# ⚠️ A TIGHT BOOTSTRAP WINDOW WAS TRIED HERE AND MADE IT WORSE. Chunk 23.
#
# The reasoning was good and the measurement disagreed, so the measurement wins
# and this comment is the record. Submap 0 has no predecessor to fall back on,
# so its opening scans are placed by odometry alone and are then ADDED to it,
# smearing the one map every later pose is registered against: measured on a
# 14.5 s traverse, a ~0.04 rad yaw bias set in the first half second and never
# corrected, which at 0.5 m/s was most of the 0.40 m of position error the
# tracker had AT THAT POINT. (It is 0.22 m now -- the two fixes below did that,
# not the window.)
#
# The hypothesis was that chunk 22's failure came from the WINDOW rather than the
# thinness -- a 0.6 rad wall-slide is only reachable because the default +/-0.20
# rad window admits it -- so matching a thin submap inside three sigma of what
# odometry can justify should be safe. It was not: final error went from 0.40 m
# to 0.57 m, and chunk 22's own `test_a_thin_submap_is_not_matched_against`
# failed. A thin submap is not a map, at any window.
#
# What remains true is the cost. The bias is real, it is permanent, and on a
# one-way traverse there is no loop to close it out. That is chunk 25's problem
# and it is stated in chunk 23's Outcome rather than tuned away here.

# ---- below this line is code, not data.


class Tracker:
    """Dead reckoning corrected by scan matching, plus submaps and closures.

    Deterministic given the same scans, the same true poses and the same rng.
    Every source of variability is either fixed (submap boundaries are a scan
    count) or seeded (odometry noise comes from the localiser's own stream).
    """

    def __init__(self, initial_pose, rng, odom_sigma_lin=0.0,
                 odom_sigma_ang=0.0, scans_per_submap=SCANS_PER_SUBMAP):
        self.pose = tuple(float(v) for v in initial_pose)
        self._rng = rng
        self._odom_sigma_lin = float(odom_sigma_lin)
        self._odom_sigma_ang = float(odom_sigma_ang)
        self._scans_per_submap = scans_per_submap

        self._last_true = None
        self.submaps = []                 # finished only
        self.current = Submap(self.pose, index=0)
        self.log = loop_mod.ClosureLog()
        self.pending_closures = []        # handed to chunk 23's graph
        self.last_match = None
        # Accumulated dead-reckoning uncertainty since the last accepted
        # closure. The PREDICTED error, never the realised one -- the same rule
        # `core/drift.py` and `core/localisation.py` both state, and the single
        # easiest way for ground truth to re-enter Observation.
        #
        # Accumulated as a VARIANCE and reported as its square root. Summing
        # sigma*sqrt(dt) per step instead would overestimate by sqrt(N_steps) --
        # a random walk's error after T seconds is sigma*sqrt(T), not
        # sigma*sqrt(dt) added N times -- and gate 3 in `loop.py` budgets a
        # closure's correction against this number, so an inflated proxy would
        # quietly wave through the corrections it exists to refuse.
        self._uncertainty_var = 0.0

        # Wall clock, per phase. ⚠️ Deliberately NOT in `diagnostics()` and so
        # never in `slam.csv` -- chunk 23's rule, and the reason is that the
        # sidecar has to be byte-identical across two runs of one episode while
        # a timing differs every run by construction. Chunk 24a's cost model
        # reads these attributes; nothing else does.
        #
        # perf_counter() costs ~50 ns against a ~44 ms match, so the
        # instrumentation is four parts per million of what it measures and is
        # left permanently on rather than guarded by a flag.
        self.match_seconds = 0.0
        self.closure_seconds = 0.0
        self.matches = 0                  # scans actually matched, not scans
        self.closure_rounds = 0           # proposal rounds, not candidates

    # -- odometry ----------------------------------------------------------

    def _odometry(self, dt, true_pose):
        """The motion the robot made, corrupted. Never its absolute position."""
        if self._last_true is None:
            self._last_true = tuple(float(v) for v in true_pose)
            return (0.0, 0.0, 0.0)
        delta = scan_mod.relative(self._last_true, true_pose)
        self._last_true = tuple(float(v) for v in true_pose)
        if dt <= 0 or (self._odom_sigma_lin == 0.0
                       and self._odom_sigma_ang == 0.0):
            return delta
        # Per sqrt(second), matching DriftSpec's units so the two are directly
        # comparable when chunk 25 asks whether SLAM beats the random walk.
        # Split across the two translational axes the way core/drift.py does,
        # so sigma is RADIAL rather than per-axis.
        scale = np.sqrt(dt / 2.0)
        return (delta[0] + float(self._rng.normal(0.0, self._odom_sigma_lin * scale)),
                delta[1] + float(self._rng.normal(0.0, self._odom_sigma_lin * scale)),
                scan_mod.wrap_angle(
                    delta[2] + float(self._rng.normal(
                        0.0, self._odom_sigma_ang * np.sqrt(dt)))))

    # -- the loop ----------------------------------------------------------

    def update(self, dt, true_pose, points):
        """One scan. Returns the corrected pose estimate."""
        odom = self._odometry(dt, true_pose)
        guess = scan_mod.compose(self.pose, odom)
        self._uncertainty_var += self._odom_sigma_lin ** 2 * max(dt, 0.0)

        points = np.asarray(points, dtype=float).reshape(-1, 2)

        # ⚠️ A submap with only a scan or two in it is not a map, and matching
        # against one is worse than not matching at all. Measured in chunk 22:
        # at 1-3 scans the matcher returned score 1.00 with a heading 0.6 rad
        # wrong, because a single scan's field is exactly the shape the next
        # scan makes and any rotation sliding points along a wall fits it.
        #
        # Chunk 22 answered that by placing the first MIN_SCANS_TO_MATCH scans
        # of every submap on odometry alone, and chunk 23 found what that costs:
        # those uncorrected half-seconds recur at every submap boundary and
        # compound into the dominant error term of a long episode.
        #
        # So the rule is not "no map, no match" but **match against the
        # thickest map available** -- the previous submap, finished and
        # therefore thick by definition, while the current one fills. Odometry
        # alone is now only submap 0's opening scans. See `_match_target`.
        target = self._match_target()
        if target is not None and len(points) > 0:
            local_guess = scan_mod.relative(target.origin, guess)
            t0 = time.perf_counter()
            result = match(points, target.field(), local_guess)
            self.match_seconds += time.perf_counter() - t0
            self.matches += 1
            self.last_match = result
            if result.converged:
                self.pose = scan_mod.compose(target.origin, result.pose)
            else:
                # A match that cannot say where it is must not move the
                # estimate. Falling back to odometry is worse than a good match
                # and far better than a confident wrong one.
                self.pose = guess
        else:
            self.pose = guess

        if len(points) > 0:
            self.current.add(points, self.pose)

        if self.current.finished:
            self._finish_submap()
        return self.pose

    def _match_target(self):
        """Which submap this scan is matched against, or None for odometry only.

        The current submap once it is thick enough to be a map; otherwise the
        most recently finished one, which is thick by definition. `None` only
        while submap 0 is still filling, where there is no thick map anywhere
        and odometry is genuinely all there is.
        """
        if len(self.current) >= MIN_SCANS_TO_MATCH:
            return self.current
        if self.submaps:
            return self.submaps[-1]
        return None

    def _finish_submap(self):
        finished = self.current
        self.submaps.append(finished)
        self.current = Submap(self.pose, index=finished.index + 1)

    def propose_closures(self, points):
        """Run the closure gates against every eligible finished submap.

        Returns the accepted proposals, which chunk 23 turns into graph edges.
        Rejections are counted in `self.log` and reach `slam.csv`; they are the
        only thing distinguishing "the loop was there and we did not see it"
        from "we saw it and rejected it".
        """
        accepted = []
        t0 = time.perf_counter()
        self.closure_rounds += 1
        for submap in loop_mod.candidates(
                self.submaps, self.pose, self.current.index,
                uncertainty=self.uncertainty, log=self.log):
            proposal = self.log.record(
                loop_mod.propose(submap, points, self.pose,
                                 uncertainty=self.uncertainty))
            if proposal.accepted:
                accepted.append(proposal)
                # A closure is evidence the estimate is good again, so the
                # uncertainty budget resets. This is what makes `uncertainty`
                # the qualitative shape a controller can condition on:
                # monotonically rising between closures, dropping when one
                # lands. Chunk 23 promotes it to a graph-derived quantity.
                self._uncertainty_var = 0.0
        self.pending_closures.extend(accepted)
        self.closure_seconds += time.perf_counter() - t0
        return accepted

    # -- reporting ---------------------------------------------------------

    @property
    def uncertainty(self):
        """PREDICTED position uncertainty in metres. Never the realised error."""
        return float(np.sqrt(self._uncertainty_var))

    def diagnostics(self):
        d = {
            "submaps_finished": len(self.submaps),
            "submap_scans": len(self.current),
            "uncertainty": round(self.uncertainty, 6),
            "match_score": round(self.last_match.score, 6)
            if self.last_match else 0.0,
            "match_ambiguity": round(self.last_match.ambiguity, 6)
            if self.last_match else 0.0,
            # ⚠️ ADDED BY CHUNK 24b, and it is not a nice-to-have.
            #
            # `converged` is the AND of three things -- score >= MIN_SCORE,
            # translational spread <= MAX_SPREAD, rotational spread <=
            # MAX_SPREAD_THETA -- and until now `slam.csv` carried only the
            # first two. On the physics episode in
            # out/evidence/slam-visualisation/ that made 80% of the matches
            # read as "not converged" beside a healthy score and a small
            # translational ambiguity, with nothing in the file able to say
            # why. A diagnostics dict that cannot explain its own verdict is
            # the shape of bug this project keeps finding, so the third factor
            # is here now.
            "match_ambiguity_theta": round(self.last_match.ambiguity_theta, 6)
            if self.last_match else 0.0,
            "match_converged": int(bool(
                self.last_match and self.last_match.converged)),
        }
        d.update(self.log.as_dict())
        return d
