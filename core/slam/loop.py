"""
core/slam/loop.py

**Loop closure: proposing it, and — mostly — refusing it.**

Chunk 22's second piece, and **the entire point of building SLAM in this
project.** Loop closure is the mechanism by which *what the robot chose to do*
changes how well it knows where it is: a robot that returns to somewhere it has
been can correct its accumulated error, and one that never revisits cannot.
Every card epic 03 unlocks depends on that being true and measurable, and
`localisation_failures` stops being a manipulation check exactly here.

⚠️ A FALSE CLOSURE DOES NOT DEGRADE THE MAP, IT DESTROYS IT
------------------------------------------------------------
A wrong closure tells the graph that two different places are the same place.
The optimiser then folds them onto each other and **every pose after it is
wrong** — not slightly wrong, structurally wrong. Missing a true closure costs
some accuracy; accepting a false one costs the map. The gates below are
therefore deliberately conservative, and the asymmetry is the design.

lab42 is a loop-closure trap and that is useful: the divider at x = 6 and the
wall along y = 5 make two structurally similar corridors, so the benchmark scene
already contains a realistic false-closure hazard. Better tested against on
purpose than discovered during evaluation.

WHY GEOMETRY AND NOT APPEARANCE
-------------------------------
Candidates are finished submaps whose origin lies within a search radius of the
current estimate, inflated by the current uncertainty. There is no place
recognition descriptor and at this scale there does not need to be one.

*Rejected:* a scan-descriptor index (Scan Context and relatives). Necessary at
building scale, overkill in a 12 x 8 m room, and one more component to make
deterministic.

EVERY REJECTION IS COUNTED, BY GATE — AND SO IS EVERY SUPPRESSION
------------------------------------------------------------------
Same reasoning as `generate/admit.py`'s recorded rejections: *"the loop was
there and we did not see it"* and *"we saw it and rejected it"* are different
failures with different fixes, and nothing else can tell them apart. The
counters reach `slam.csv` through `SlamLocaliser.diagnostics()`.

⚠️ Chunk 22a added a third category and it is deliberately NOT a fifth gate.
`candidates()` drops a submap the robot has not travelled `MIN_SUBMAP_TRAVEL`
metres from, BEFORE any match runs — so it is neither "we did not see it" nor
"we saw it and rejected it", but *"we saw it and did not bother looking"*.
`ClosureLog.suppressed` counts those separately. Folding them into `rejected`
would make an unmatched candidate read as a refused match in every number
downstream, and folding them into nothing at all would erase the first
distinction this section exists to preserve.

SIM-FREE.
"""

import numpy as np

from core.slam import scan as scan_mod
from core.slam.matcher import MAX_SPREAD, MIN_SCORE, match

# ---- below this line is data, not code.

# How far from the current estimate a finished submap's origin may sit and still
# be considered. Roughly a room's width of travel: close enough that the two
# could plausibly overlap, far enough that a real return is not missed.
SEARCH_RADIUS = 3.0

# The search radius grows with the pose uncertainty, because that is exactly the
# quantity saying how far the estimate might have wandered. Bounded, or a badly
# drifted estimate would propose every submap in the map.
RADIUS_UNCERTAINTY_GAIN = 2.0
MAX_SEARCH_RADIUS = 6.0

# GATE 1. A closure must match better than ordinary tracking does. Set from
# chunk 21's noise sweep, where a good match scored 0.56-0.66 across a 16x range
# of sensor noise, and FIXED IN ADVANCE rather than tuned until a demo worked.
CLOSURE_MIN_SCORE = 0.55

# GATE 1b. Chunk 21's ambiguity measure, which is why that finding matters here:
# a featureless corridor scores WELL and means nothing, and a corridor is
# precisely what a false closure is proposed across.
CLOSURE_MAX_AMBIGUITY = 0.15

# GATE 2. A match landing within this many metres of the search window's edge
# wanted to go further and was clamped by the window, not chosen by the data.
WINDOW_MARGIN = 0.05

# GATE 3. How many multiples of the believed uncertainty a correction may claim.
# A closure proposing a 4 m correction when the estimate says 0.3 m of drift is
# asserting the estimate was wrong by an order of magnitude; far likelier the
# match is wrong.
MAX_CORRECTION_SIGMA = 3.0
MIN_CORRECTION_FLOOR = 0.30      # metres, so a confident estimate can still close

# A submap this recently finished is not a loop, it is the trajectory. Without
# this every submap would "close" against its immediate predecessor, which is
# odometry wearing a different name.
MIN_SUBMAP_GAP = 2

# ⚠️ AND THE INDEX GAP ABOVE IS A PROXY THAT FAILS WHEN THE ROBOT STOPS MOVING.
#
# A submap boundary is a pure function of scan count (`submap.py`, and for a
# good reason: it is what makes two identical runs cut them identically), so a
# STATIONARY robot finishes submaps without going anywhere. Chunk 24b measured
# it under physics: turning on the spot for 40 s produced **nine submaps inside
# a 0.13 x 0.19 m box**, and all four gates below correctly passed 37 closures
# between them. Every one was valid -- they are scans of the same place from the
# same place -- and every one was worthless, because a constraint saying "node
# 11 is where node 12 is" cannot correct an error accumulated between node 0 and
# node 6. An index gap of nine and a trajectory of 0.3 m is exactly the
# "odometry wearing a different name" the gap gate exists to refuse.
#
# So the gate is restated in the currency that does not lie: METRES TRAVELLED
# along the trajectory between the two submaps. This is not a new policy.
# Measured across both of chunk 24b's runs, ordinary driving covers 0.92-1.73 m
# per submap, so MIN_SUBMAP_GAP = 2 ALREADY means about two metres whenever the
# robot is moving; 2.0 leaves the moving case as it was and bites only where the
# index was always the wrong unit.
#
# The two populations separate cleanly, which is why a threshold exists at all:
#
#                          closures | travel: min  median     max
#   genuine lap-to-lap revisits  59 |        3.00   13.98   29.38
#   the stationary cluster       37 |        0.16    0.47    3.81
#
# At 2.0 the genuine set keeps 59/59 with 1.5x margin below its own minimum and
# the cluster keeps 1/37 -- its single real long-range edge.
#
# ⚠️ REJECTED: gating on the EUCLIDEAN separation of the two submaps, which is
# the obvious reading of "the closures were 10 cm apart" and is exactly wrong.
# On the genuine set the median separation is 2.50 m and the MINIMUM is 0.02 m:
# a revisit IS being in the same place, that is what the word means. A distance
# gate would discard real closures and leave the cluster's 3.81 m outlier.
MIN_SUBMAP_TRAVEL = 2.0          # metres of trajectory, not metres apart

# The closure search window. Wider than the tracking window in `matcher.py`,
# because a loop closure is correcting accumulated drift rather than one step of
# motion.
CLOSURE_WINDOW_XY = 0.60
CLOSURE_WINDOW_THETA = 0.40

REJECT_REASONS = ("score", "ambiguity", "window_edge", "correction_too_large")

# ---- below this line is code, not data.


class ClosureProposal:
    """A candidate constraint between the current pose and a finished submap.

    `accepted` is the verdict; `reason` is which gate refused it, or None. Both
    are recorded either way — a proposal that was seen and refused is evidence,
    and it is the only thing distinguishing a missed loop from a rejected one.
    """

    __slots__ = ("submap_index", "pose_in_submap", "score", "ambiguity",
                 "correction", "accepted", "reason")

    def __init__(self, submap_index, pose_in_submap, score, ambiguity,
                 correction, accepted, reason):
        self.submap_index = submap_index
        self.pose_in_submap = pose_in_submap
        self.score = score
        self.ambiguity = ambiguity
        self.correction = correction       # metres the closure would move us
        self.accepted = accepted
        self.reason = reason

    def __repr__(self):
        verdict = "ACCEPTED" if self.accepted else f"rejected:{self.reason}"
        return (f"ClosureProposal(submap={self.submap_index}, "
                f"score={self.score:.3f}, ambiguity={self.ambiguity:.3f}, "
                f"correction={self.correction:.3f}, {verdict})")


def travel_to_pose(submaps, pose):
    """Metres of trajectory from each finished submap's origin to `pose`.

    The polyline through the intervening origins, plus the last hop from the
    newest origin to where the robot believes it is now. Returned as a list
    parallel to `submaps`.

    ⚠️ It is a LOWER BOUND on the distance actually travelled, because origins
    are sampled every `SCANS_PER_SUBMAP` scans and a curve between two of them
    is longer than the chord. That is the safe direction: under-reading travel
    makes the gate refuse more, and `loop.py`'s whole design is the asymmetry
    that a missed closure costs accuracy while a bad one costs the map.

    Computed from the CURRENT origins, which the optimiser relocates. That is
    intended: the quantity being tested is how much trajectory the odometry
    chain already spans, and the chain is what the estimate says it is.
    """
    if not submaps:
        return []
    origins = [s.origin for s in submaps]
    # Distance from each origin forward to the end of the chain, accumulated
    # backwards so the whole array costs one pass rather than one per submap.
    tail = np.hypot(pose[0] - origins[-1][0], pose[1] - origins[-1][1])
    out = [0.0] * len(origins)
    out[-1] = float(tail)
    for k in range(len(origins) - 2, -1, -1):
        step = np.hypot(origins[k + 1][0] - origins[k][0],
                        origins[k + 1][1] - origins[k][1])
        out[k] = out[k + 1] + float(step)
    return out


def candidates(submaps, pose, current_index, uncertainty=0.0,
               search_radius=SEARCH_RADIUS, min_gap=MIN_SUBMAP_GAP,
               min_travel=MIN_SUBMAP_TRAVEL, log=None):
    """Finished submaps near `pose` that are old enough to be a loop.

    "Old enough" is two things, and the second is chunk 22a: at least `min_gap`
    submaps ago, AND at least `min_travel` metres of trajectory ago. See
    `MIN_SUBMAP_TRAVEL` for why the index alone is not enough — a stationary
    robot satisfies any index gap without going anywhere.

    Returned in ascending index order — a fixed order, so the acceptance
    decision cannot depend on iteration order. That matters more than it looks:
    the first accepted closure is the one applied, so an unordered candidate set
    would make the result depend on dict ordering.

    `log` is an optional `ClosureLog`. Suppressions are counted there rather
    than silently dropped, because this module's docstring promises that "the
    loop was there and we did not see it" stays distinguishable from "we saw it
    and rejected it" — and a filter that runs BEFORE the match would otherwise
    erase the first of those. It is a separate counter from the rejections, not
    a fifth reason: nothing here has looked at a scan.
    """
    radius = min(search_radius + RADIUS_UNCERTAINTY_GAIN * float(uncertainty),
                 MAX_SEARCH_RADIUS)
    travel = travel_to_pose(submaps, pose)
    out = []
    for k, submap in enumerate(submaps):
        if not submap.finished:
            continue
        if current_index - submap.index < min_gap:
            continue
        d = np.hypot(submap.origin[0] - pose[0], submap.origin[1] - pose[1])
        if d > radius:
            continue
        if travel[k] < min_travel:
            # Near enough and old enough by index, but the robot has not been
            # anywhere since. Counted, not silently dropped.
            if log is not None:
                log.suppress()
            continue
        out.append(submap)
    return sorted(out, key=lambda s: s.index)


def propose(submap, points, pose, uncertainty=0.0,
            window_xy=CLOSURE_WINDOW_XY, window_theta=CLOSURE_WINDOW_THETA,
            min_score=CLOSURE_MIN_SCORE, max_ambiguity=CLOSURE_MAX_AMBIGUITY,
            max_correction_sigma=MAX_CORRECTION_SIGMA):
    """Match `points` against one finished submap and run all three gates.

    Returns a `ClosureProposal` whether or not it passed. The gates are checked
    in a fixed order and the FIRST failure is the recorded reason, so a
    proposal that fails two gates is attributed to one of them consistently.
    """
    guess = scan_mod.relative(submap.origin, pose)
    result = match(points, submap.field(), guess,
                   window_xy=window_xy, window_theta=window_theta,
                   min_score=min_score, max_spread=max_ambiguity)

    # How far the closure would move the current estimate. This is the quantity
    # gate 3 tests, and it is measured in the submap frame where both poses are
    # expressed, so no global-frame error contaminates it.
    correction = float(np.hypot(result.pose[0] - guess[0],
                                result.pose[1] - guess[1]))

    def verdict(accepted, reason):
        return ClosureProposal(submap.index, result.pose, result.score,
                               result.ambiguity, correction, accepted, reason)

    # GATE 1 -- does the scan actually fit this submap?
    if result.score < min_score:
        return verdict(False, "score")

    # GATE 1b -- does it fit in only ONE place? Chunk 21's finding: a corridor
    # scores well everywhere along itself, and a corridor is exactly what a
    # false closure is proposed across.
    if result.ambiguity > max_ambiguity:
        return verdict(False, "ambiguity")

    # GATE 2 -- a match on the window edge wanted to go further, and was stopped
    # by the window rather than chosen by the data. Reject rather than clamp:
    # clamping would hand the graph a constraint the data does not support.
    dx = abs(result.pose[0] - guess[0])
    dy = abs(result.pose[1] - guess[1])
    dtheta = abs(scan_mod.wrap_angle(result.pose[2] - guess[2]))
    if (dx > window_xy - WINDOW_MARGIN or dy > window_xy - WINDOW_MARGIN
            or dtheta > window_theta - 0.02):
        return verdict(False, "window_edge")

    # GATE 3 -- is the correction plausible given how wrong we think we are?
    budget = max(MIN_CORRECTION_FLOOR,
                 max_correction_sigma * float(uncertainty))
    if correction > budget:
        return verdict(False, "correction_too_large")

    return verdict(True, None)


class ClosureLog:
    """Counters by gate, plus the accepted total. Reaches `slam.csv`.

    Deliberately a plain counter rather than a list of proposals: it goes into
    a per-step diagnostics dict, and an unbounded list would grow with the
    episode. The full proposals are available to tests, which is where the
    detail is actually wanted.
    """

    __slots__ = ("accepted", "rejected", "considered", "suppressed")

    def __init__(self):
        self.accepted = 0
        self.considered = 0
        self.rejected = {reason: 0 for reason in REJECT_REASONS}
        # Chunk 22a. Candidates dropped by `candidates()` for lack of travel,
        # BEFORE any match ran. Deliberately not one of `rejected`: a rejection
        # is a verdict on a scan and this is a verdict on the trajectory, and
        # collapsing them would make an unmatched candidate look like a refused
        # match in every number downstream.
        self.suppressed = 0

    def suppress(self):
        self.suppressed += 1

    def record(self, proposal):
        self.considered += 1
        if proposal.accepted:
            self.accepted += 1
        else:
            self.rejected[proposal.reason] += 1
        return proposal

    def as_dict(self):
        out = {"closures_accepted": self.accepted,
               "closures_considered": self.considered,
               "closures_suppressed_travel": self.suppressed}
        out.update({f"closures_rejected_{k}": v
                    for k, v in self.rejected.items()})
        return out
