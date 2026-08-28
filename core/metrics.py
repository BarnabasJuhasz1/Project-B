"""
core/metrics.py

Scoring is a pure function of a recorded trace. Nothing here knows how the trace
was produced, which is what lets it be unit-tested against synthetic traces in
milliseconds, re-scored after the fact when a threshold moves, and reused
unchanged by the Week 3 unit-scenario harness.

    from core.metrics import EpisodeTrace, score
    result = score(trace, scenario)

WARNING: every constant below is PRE-REGISTRATION MATERIAL. They may be tuned
this week. Once the first evaluation run happens they must not move, and the
frozen values must be written down alongside G2's decision threshold X. Changing
a threshold after seeing results turns an ablation into a search over thresholds.
"""

import csv
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

from core.geometry import ROBOT_RADIUS


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# Success. 0.5 m is about one body length: close enough to call it arrival,
# loose enough that the goal is not a test of the locomotion policy's precision.
#
# Also read by core.scenario.optimal_distance(), which measures the SPL
# denominator to this disc rather than to the goal point — the numerator stops
# here, so the denominator must too. Moving this constant moves both, which is
# the point: they cannot drift apart.
GOAL_RADIUS = 0.5

# Base height below which the robot has fallen.
#
# NOT walk_forward.py:137's 0.15. That value is inside the normal gait envelope
# for this asset and this policy, so it fails a robot that is walking perfectly
# well — walk_forward.py itself reports FELL after travelling 1.97 m in 6 s.
# Measured on the 6.0.1 Go2 base prim under the frozen policy:
#
#     standing still   z ~ 0.213      (min 0.211 over 3 s)
#     walking 0.5 m/s  z ~ 0.145      (min 0.128 over 5 s)
#     turning 1 rad/s  z ~ 0.182      (min 0.144 over 3 s)
#
# 0.08 is well under the walking minimum and unambiguously a collapsed posture.
# A tilt check on the base orientation would be a better fall criterion than
# height at all; that needs roll/pitch on the trace and is deliberately not done
# here. Flag it when these constants are frozen for pre-registration.
FALL_HEIGHT = 0.08

# Clearance in a trace is the gap between the robot's circumscribing circle
# (ROBOT_RADIUS) and the nearest wall surface, so it goes negative when the body
# is overlapping geometry. Both figures are gaps, not centre distances.
COLLISION_CLEARANCE = 0.05
NEAR_MISS_CLEARANCE = 0.25

# The signal must stay clear for this long before a new event of the same kind
# can start. Without it a controller oscillating either side of a threshold
# scores dozens of collisions for one scrape.
#
# A gap of EXACTLY EVENT_RELEASE closes the span. That is a decision, not a
# consequence: the constant is the duration after which the signal counts as
# released, so a gap that reaches it has released, and requiring one sample
# more would make the constant wrong by one sample. _events() implements it
# with a tolerance rather than a bare `>=`, because at 50 Hz control this is
# exactly 25 steps and the comparison would otherwise be decided by the last
# bits of an accumulated float. See EVENT_RELEASE_EPS below.
EVENT_RELEASE = 0.5

# Slack on that comparison. `t` reaches _events() either accumulated by
# repeated addition (the runner) or rounded to 6 dp (a trace read back from
# disk by scripts/rescore.py), and those differ by ~1e-12. Without the slack
# they gave DIFFERENT EVENT COUNTS for the same episode: measured on
# doorway_traverse__goal_seeker__seed1001, stuck_events 2 -> 1 and time_stuck
# 78.76 -> 111.36 s, a 41% swing from a 6.7e-12 s difference in timestamps.
#
# 1e-9 matches the guard _stuck_spans() has always carried, and is nine orders
# of magnitude below the shortest control step anyone would run, so it can only
# ever resolve an exact tie in the intended direction. It is not a threshold to
# tune: it exists to make an equality decidable.
EVENT_RELEASE_EPS = 1e-9

# Stuck: commanded to move, but did not. The window has to be long enough that
# turning in place on the spot does not read as stuck.
STUCK_WINDOW = 2.0
STUCK_DISPLACEMENT = 0.1
STUCK_COMMAND_EPS = 0.05

# Localisation failure. 1.0 m is roughly the doorway width: an error that large
# means the robot's belief puts it on the wrong side of a wall.
LOC_FAIL_THRESHOLD = 1.0


# ---------------------------------------------------------------------------

TRACE_COLUMNS = [
    "t", "x", "y", "z", "yaw",
    "vx", "vy", "wz",
    "clearance", "contact",
    "est_x", "est_y", "est_yaw",
    "goal_known",
]

# Columns stored as 0/1 rather than as a 6 dp float.
BOOL_COLUMNS = {"contact", "goal_known"}

# Columns added AFTER traces had already been recorded, mapped to the value
# that reproduces the behaviour of the runs that predate them. read_csv() fills
# from here, which is what keeps a committed result re-scorable across a change
# to the trace format — the alternative is that adding a column silently
# orphans every episode recorded before it, and re-scoring is the property
# this module opens by claiming.
#
# goal_known: every episode recorded before the goal could be withheld was run
# on a scenario that disclosed it from t=0, so True is not a guess.
TRACE_BACKFILL = {"goal_known": True}


@dataclass
class EpisodeTrace:
    """One row per CONTROL step. Ground truth lives here and nowhere else the
    controller can see."""

    rows: List[tuple] = field(default_factory=list)

    def append(self, t, pose, command, clearance, contact, pose_estimate,
               goal_known=True):
        """`goal_known` defaults True because that is what every scenario did
        before the goal could be withheld, and defaulting it keeps every
        existing caller and test correct. A scenario that withholds the goal
        must pass it explicitly — run_episode.py does."""
        x, y, z, yaw = pose
        self.rows.append((
            float(t), float(x), float(y), float(z), float(yaw),
            float(command.vx), float(command.vy), float(command.wz),
            float(clearance), bool(contact),
            float(pose_estimate[0]), float(pose_estimate[1]),
            float(pose_estimate[2]), bool(goal_known),
        ))

    def __len__(self):
        return len(self.rows)

    def column(self, name):
        return np.array([r[TRACE_COLUMNS.index(name)] for r in self.rows])

    def write_csv(self, path):
        with open(path, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(TRACE_COLUMNS)
            # repr-round to 6 dp so the determinism diff is not defeated by the
            # last bit of a float's decimal expansion.
            for row in self.rows:
                writer.writerow([f"{v:.6f}" if isinstance(v, float) else int(v)
                                 for v in row])

    @classmethod
    def read_csv(cls, path):
        """The inverse of write_csv(). It lives here, beside the writer, rather
        than in scripts/rescore.py, so the two cannot drift apart and so the
        round trip is testable without importing anything from scripts/.

        Columns are matched BY NAME, never by position: a trace written before a
        column existed is still readable, filled from TRACE_BACKFILL. A column
        this module does not know, or a missing one with no backfill, is an
        error — that trace predates something the backfill cannot reconstruct
        and the episode has to be re-run rather than re-scored.
        """
        trace = cls()
        with open(path, newline="") as fh:
            reader = csv.DictReader(fh)
            header = reader.fieldnames or []
            unknown = [c for c in header if c not in TRACE_COLUMNS]
            missing = [c for c in TRACE_COLUMNS if c not in header]
            unfillable = [c for c in missing if c not in TRACE_BACKFILL]
            if unknown or unfillable:
                raise ValueError(
                    f"{path}: trace columns cannot be reconciled with "
                    f"core.metrics.TRACE_COLUMNS. Unknown here: {unknown}. "
                    f"Missing with no backfill: {unfillable}. This trace "
                    f"predates a change to the format that TRACE_BACKFILL "
                    f"cannot reconstruct; re-run the episode.")
            for row in reader:
                trace.rows.append(tuple(
                    TRACE_BACKFILL[c] if c not in header
                    else bool(int(float(row[c]))) if c in BOOL_COLUMNS
                    else float(row[c])
                    for c in TRACE_COLUMNS))
        return trace


def _events(active, t, release=EVENT_RELEASE):
    """Rising edges of a boolean signal, with hysteresis.

    Returns [(t_start, t_end)]. An event ends only once `active` has been False
    continuously for `release` seconds, so one long scrape is one event and a
    signal chattering across a threshold is still one event.

    A clear gap of exactly `release` DOES end the event — see EVENT_RELEASE.
    `clear_since` is the first clear sample, so the shortest closing gap is
    `release/dt + 1` samples, and the comparison lands on exact equality
    whenever `release` is an integer multiple of the control step. It is at 50
    Hz, which is why EVENT_RELEASE_EPS is there: without it the answer depends
    on how the caller built `t`, and re-scoring a committed trace stops being
    safe. core/tests/test_metrics.py pins both sides of the boundary.
    """
    spans = []
    start = None
    clear_since = None
    for i, flag in enumerate(active):
        if flag:
            clear_since = None
            if start is None:
                start = t[i]
        elif start is not None:
            if clear_since is None:
                clear_since = t[i]
            elif t[i] - clear_since >= release - EVENT_RELEASE_EPS:
                spans.append((start, clear_since))
                start = None
                clear_since = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def _stuck_spans(t, x, y, speed_cmd):
    """Commanded to move but not moving, over a trailing STUCK_WINDOW.

    Detection lags the onset by the window length by construction — you cannot
    know the robot is stuck until it has failed to move for a while — so the
    reported recovery time is measured from detection, not from the true onset.
    """
    active = np.zeros(len(t), dtype=bool)
    for i in range(len(t)):
        # Same exact-tie hazard as _events(), and it needs the tolerance on the
        # SEARCH as well as on the check below. STUCK_WINDOW is an integer
        # multiple of the control step at every rate we run (100 steps at 50
        # Hz), so `t[i] - STUCK_WINDOW` lands on t[i - 100] by construction —
        # and lands the wrong side of it, in float, on 26% of samples at 50 Hz.
        # searchsorted then returns i-99, the window comes up one step short,
        # and the `continue` below silently drops the sample from the stuck
        # signal. Nudging the target down resolves every tie toward the fuller
        # window, which is the same direction EVENT_RELEASE_EPS resolves in.
        j = np.searchsorted(t, t[i] - STUCK_WINDOW - EVENT_RELEASE_EPS,
                            side="left")
        if t[i] - t[j] < STUCK_WINDOW - EVENT_RELEASE_EPS:
            continue                      # window not fully populated yet
        if np.any(speed_cmd[j:i + 1] <= STUCK_COMMAND_EPS):
            continue                      # it was told to stop, that is not stuck
        moved = np.hypot(x[j:i + 1] - x[i], y[j:i + 1] - y[i]).max()
        active[i] = moved < STUCK_DISPLACEMENT
    return _events(active, t)


def score(trace, scenario):
    """Score one episode. Returns a plain dict, JSON-ready.

    Everything is computed over the trace truncated at termination: a robot that
    tumbles past the goal marker has not reached the goal, and a path length
    that keeps accumulating after success would depress SPL for no reason.
    """
    if len(trace) == 0:
        raise ValueError("empty trace: the episode recorded no control steps")

    t = trace.column("t")
    x, y, z = trace.column("x"), trace.column("y"), trace.column("z")
    clear = trace.column("clearance")
    contact = trace.column("contact").astype(bool)
    est_x, est_y = trace.column("est_x"), trace.column("est_y")
    speed_cmd = np.hypot(trace.column("vx"), trace.column("vy"))

    gx, gy = scenario.goal
    to_goal = np.hypot(x - gx, y - gy)
    upright = z >= FALL_HEIGHT

    fell = np.flatnonzero(~upright)
    reached = np.flatnonzero((to_goal <= GOAL_RADIUS) & upright)
    fall_i = int(fell[0]) if fell.size else None
    goal_i = int(reached[0]) if reached.size else None

    if fall_i is not None and (goal_i is None or fall_i <= goal_i):
        termination, end, goal_reached = "fell", fall_i, False
    elif goal_i is not None:
        termination, end, goal_reached = "goal", goal_i, True
    else:
        termination, end, goal_reached = "timeout", len(t) - 1, False

    sl = slice(0, end + 1)
    steps = np.hypot(np.diff(x[sl]), np.diff(y[sl]))
    path_length = float(steps.sum())
    optimal = scenario.optimal_distance()

    # SPL. `optimal` is now the taut continuous path to the GOAL_RADIUS disc —
    # the same journey `path_length` measures — so the clamp below should never
    # bind on a legal path. It used to bind constantly: the old denominator was
    # the 8-connected grid geodesic all the way to the goal POINT, roughly 1.0 m
    # long on a 10.6 m route, which pinned every competent controller at exactly
    # 1.0 and cost the metric its discriminating range.
    #
    # The clamp stays, because a real path CAN still come in under `optimal` by
    # cutting a corner into a wall, and SPL above 1 is not meaningful. But it is
    # now a fault indicator rather than routine, so the unclamped ratio is
    # reported alongside: spl_uncapped > 1 means the robot took a shortcut no
    # legal path could take, which should show up as a collision too.
    spl_uncapped = (optimal / path_length) if goal_reached and path_length > 0 else 0.0
    if goal_reached and path_length == 0.0:
        spl_uncapped = 1.0                 # started inside the goal radius
    spl = min(1.0, spl_uncapped)

    collided = contact[sl] | (clear[sl] <= COLLISION_CLEARANCE)
    collision_spans = _events(collided, t[sl])

    # A near miss is an approach that did NOT become a collision. Counting the
    # near band raw would report every collision as a near miss too, and the two
    # numbers would move together and say nothing independent.
    near_spans = []
    for start, stop in _events(clear[sl] <= NEAR_MISS_CLEARANCE, t[sl]):
        window = (t[sl] >= start) & (t[sl] <= stop)
        if clear[sl][window].min() > COLLISION_CLEARANCE:
            near_spans.append((start, stop))

    stuck_spans = _stuck_spans(t[sl], x[sl], y[sl], speed_cmd[sl])
    recoveries = [stop - start for start, stop in stuck_spans]

    loc_error = np.hypot(est_x[sl] - x[sl], est_y[sl] - y[sl])
    loc_spans = _events(loc_error > LOC_FAIL_THRESHOLD, t[sl])

    return {
        "scenario": scenario.id,
        "seed": scenario.seed,
        "termination": termination,
        "goal_reached": bool(goal_reached),
        "time_to_goal": float(t[goal_i]) if goal_reached else None,
        "spl": float(spl),
        "spl_uncapped": float(spl_uncapped),
        "path_length": path_length,
        "optimal_distance": float(optimal),
        "collisions": len(collision_spans),
        "near_misses": len(near_spans),
        "stuck_events": len(stuck_spans),
        "recovery_time": float(np.mean(recoveries)) if recoveries else None,
        "time_stuck": float(sum(recoveries)),
        "localisation_failures": len(loc_spans),
        "min_clearance": float(clear[sl].min()),
        "duration": float(t[end]),
        "control_steps": end + 1,
    }
