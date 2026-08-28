"""
core/scenarios_held_out.py

THE SEALED HELD-OUT SPLIT. Twelve scenarios, never seen during generation.

G3 measures "the fraction of held-out scenarios solved by recombination alone,
no new primitives". That number means nothing if the library was developed
against these tasks, and unlike most mistakes this one cannot be repaired
afterwards by any amount of care — so this module is separate from
core/scenario.py precisely so that "did anything import it" is a one-line
check, enforced by core/tests/test_held_out.py.

WHAT SEALING DOES AND DOES NOT BUY YOU
--------------------------------------
It buys: nothing under controllers/, nodes/, generate/ or compose/ can import
this module without failing a test; core.scenario.load() reaches these only when
asked for one by name, and imports this module lazily so that merely importing
core.scenario does not pull the split in; every held-out run records this file's
SHA-256, so a result can prove which split it scored against.

It does NOT buy protection against an LLM having read this file in an earlier
session. Generation is LLM-driven and its context is not a sandbox. The
mitigation is procedural and no test can enforce it: **hold generation sessions
that have never had this module in context, and never paste it into a prompt.**
That limitation is written into results/README.md because a reviewer will ask.

THE DESIGN: A 2 x 2, NOT A LIST
--------------------------------
The two axes are the two things 11a's contract exposes, and they are crossed
rather than confounded, so each mechanism family's contribution is separable
from the results without running a separate ablation:

                    goal disclosed        goal withheld
    no drift        navigate_to_goal      search_unmapped
    drift on        return_degraded       search_degraded

`search_degraded` is the family that carries G3's actual claim. It is the only
one that cannot be solved by any single primitive: the robot must explore for an
undisclosed goal *while* its pose estimate drifts, so an exploration node and a
path-integration node have to be recombined. A library that solves the other
three and fails this one has demonstrated transfer but not composition, and
those are different results.

Three per family is the smallest count at which a per-family breakdown is
reportable rather than anecdotal. Twelve is the G3 denominator and it is an
input to chunk 17's episode-count arithmetic: 12 x the seed set is N per
condition, and N sets the smallest detectable X. **Do not change the count.**

    python -c "import core.scenarios_held_out as h; print(len(h.HELD_OUT_SCENARIOS))"
    python scripts/seal_split.py
"""

import math

from core.scenario import DriftSpec, Scenario

# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# Drift magnitudes. PRE-REGISTRATION MATERIAL, sealed with the split.
#
# A random walk's expected error grows as sigma * sqrt(t), and
# LOC_FAIL_THRESHOLD is 1.0 m, so the episode crosses into "localisation has
# failed" at t = (1.0 / sigma)^2 seconds. Both levels are sized so that crossing
# lands near the MIDDLE of the episode it is used in: early enough that the run
# has a degraded phase worth measuring, late enough that a controller which
# handles it has somewhere to demonstrate that.
#
#     MODERATE  (1.0 / 0.12)^2 =  69 s
#     SLOW      (1.0 / 0.09)^2 = 123 s
#
# The units are what make these implementation-independent: any correct seeded
# random walk in metres-per-root-second reproduces that arithmetic. If chunk 13
# finds the threshold is not actually crossed, that is a finding about the drift
# MODEL and it is fixed there. It is not a licence to edit this file — a split
# that gets tuned after sealing is not a split.
#
# sigma_yaw is scaled to put ~0.5 rad (29 deg) of heading error at the same
# crossing time, so the estimate degrades in bearing and position together.
DRIFT_MODERATE = DriftSpec(sigma_xy=0.12, sigma_yaw=0.060)
DRIFT_SLOW = DriftSpec(sigma_xy=0.09, sigma_yaw=0.045)

# Seeds are 2000 + the 1-based position in this list. Stated as a rule rather
# than as twelve arbitrary integers so that "was a seed added or moved after
# someone saw the results" is answerable by inspection. Dev holds 1001-1003, so
# the ranges cannot collide.
_SEED_BASE = 2000


HELD_OUT_SCENARIOS = [

    # -- navigate_to_goal ---------------------------------------------------
    # The control family. Goal disclosed, no drift: the same task the dev set
    # trains against, on routes it never saw. A library that fails HERE has not
    # transferred at all, so this is the family that distinguishes "the split is
    # hard" from "the library is empty".

    Scenario(
        id="held_nav_top_corridor", family="navigate_to_goal", split="held_out",
        # Straight along the top of the room and through the doorway. The dev
        # routes all approach the doorway from the south-west; this one never
        # goes near the divider's southern face.
        start=(1.0, 7.0, 0.0), goal=(11.0, 7.0),
        time_limit=120.0, seed=_SEED_BASE + 1,
    ),
    Scenario(
        id="held_nav_diagonal_up", family="navigate_to_goal", split="held_out",
        # Starts facing north, away from the goal: the first thing it must do
        # is turn, which doorway_traverse never requires.
        start=(2.0, 1.0, math.pi / 2), goal=(10.0, 6.0),
        time_limit=120.0, seed=_SEED_BASE + 2,
    ),
    Scenario(
        id="held_nav_right_to_left", family="navigate_to_goal", split="held_out",
        # East to west, ending deep in the left region rather than at its
        # corner. return_home ends at (1.5, 1.5); this ends at mid-height, so a
        # controller that memorised the dev endpoint gains nothing.
        start=(11.0, 6.0, math.pi), goal=(1.0, 3.0),
        time_limit=120.0, seed=_SEED_BASE + 3,
    ),

    # -- search_unmapped ----------------------------------------------------
    # Goal withheld. obs.goal is None until the robot is within DETECT_RADIUS
    # with a clear line of sight, and every start below is chosen so that is
    # false at t=0 — asserted in the tests, because validate() cannot catch it:
    # it is a property of the start/goal PAIR, not of either alone.
    #
    # Time limits are generous relative to the taut path because exploring is
    # not walking. A limit tuned to the optimal route would fail a competent
    # searcher for lack of time, which biases against exactly the mechanisms
    # the corpus is supposed to supply.

    Scenario(
        id="held_search_deep_right", family="search_unmapped", split="held_out",
        # The goal is behind the divider AND past the blind corner, so no
        # amount of looking from the left region finds it. 11.29 m taut.
        start=(1.0, 1.0, 0.0), goal=(11.0, 6.5),
        goal_known=False,
        time_limit=180.0, seed=_SEED_BASE + 4,
    ),
    Scenario(
        id="held_search_past_corner", family="search_unmapped", split="held_out",
        # Short: 4.60 m taut. The goal sits just past (6, 5), so this isolates
        # the corner from the traverse — the search equivalent of blind_corner.
        start=(2.0, 7.0, 0.0), goal=(7.0, 6.0),
        goal_known=False,
        time_limit=120.0, seed=_SEED_BASE + 5,
    ),
    Scenario(
        id="held_search_reverse", family="search_unmapped", split="held_out",
        # Searching westward, from the right region into the left. The left
        # region is the larger open space, so this is the coverage problem
        # rather than the occlusion one.
        start=(10.8, 7.2, math.pi), goal=(1.0, 4.0),
        goal_known=False,
        time_limit=180.0, seed=_SEED_BASE + 6,
    ),

    # -- return_degraded ----------------------------------------------------
    # Goal disclosed, pose estimate drifting. Isolates localisation from search:
    # the robot knows where it is going and cannot trust where it is.

    Scenario(
        id="held_return_long", family="return_degraded", split="held_out",
        start=(11.0, 6.0, math.pi), goal=(1.0, 1.0),
        drift=DRIFT_MODERATE,        # crosses 1.0 m at ~69 s of 150
        time_limit=150.0, seed=_SEED_BASE + 7,
    ),
    Scenario(
        id="held_return_short", family="return_degraded", split="held_out",
        # 5.58 m taut, and the limit is deliberately far longer than the walk
        # needs: the task here is the drift, not the distance.
        start=(9.0, 7.5, math.pi), goal=(3.0, 6.5),
        drift=DRIFT_MODERATE,        # crosses 1.0 m at ~69 s of 120
        time_limit=120.0, seed=_SEED_BASE + 8,
    ),
    Scenario(
        id="held_return_outbound", family="return_degraded", split="held_out",
        # The one degraded run that goes outbound rather than home, so the
        # family is not secretly "walk back the way you came".
        start=(0.8, 0.8, 0.0), goal=(11.0, 6.2),
        drift=DRIFT_MODERATE,        # crosses 1.0 m at ~69 s of 150
        time_limit=150.0, seed=_SEED_BASE + 9,
    ),

    # -- search_degraded ----------------------------------------------------
    # Both axes on, and the family G3's claim actually rests on. Not solvable by
    # any one primitive: explore to find a goal you were not given, while the
    # estimate you would explore with is decaying. Longest limits in the split,
    # and SLOW drift so the degraded phase lands mid-episode rather than before
    # the robot has covered any ground.

    Scenario(
        id="held_sd_far", family="search_degraded", split="held_out",
        start=(1.0, 1.0, 0.0), goal=(11.0, 7.0),
        goal_known=False,
        drift=DRIFT_SLOW,            # crosses 1.0 m at ~123 s of 240
        time_limit=240.0, seed=_SEED_BASE + 10,
    ),
    Scenario(
        id="held_sd_corner", family="search_degraded", split="held_out",
        start=(1.0, 6.0, 0.0), goal=(8.0, 6.5),
        goal_known=False,
        drift=DRIFT_MODERATE,        # crosses 1.0 m at ~69 s of 180
        time_limit=180.0, seed=_SEED_BASE + 11,
    ),
    Scenario(
        id="held_sd_reverse", family="search_degraded", split="held_out",
        start=(11.0, 7.0, math.pi), goal=(2.0, 2.0),
        goal_known=False,
        drift=DRIFT_SLOW,            # crosses 1.0 m at ~123 s of 240
        time_limit=240.0, seed=_SEED_BASE + 12,
    ),
]

HELD_OUT_FAMILIES = ("navigate_to_goal", "search_unmapped",
                     "return_degraded", "search_degraded")
