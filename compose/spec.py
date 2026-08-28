"""
compose/spec.py

`TaskSpec` — what composition is told about the task, and nothing else.

It is the retrieval query and the composition input, and it has one hard
requirement: **it must be expressible for a held-out scenario without leaking
the held-out module into the prompt.** A spec that carried a scenario's start
coordinates would let a composed tree be tailored to one held-out episode, and
G3's "solved by recombination alone" would be measuring memorisation.

WHAT IS IN, AND WHY EACH ONE IS LEGITIMATE
------------------------------------------
**`goal_known`** — is the goal disclosed at t = 0? The single most consequential
bit in the spec: it decides whether the composed tree needs an exploration
branch at all. It is a task property, not a mechanism.

**`pose_drifts`** — does the pose estimate degrade? Same argument.

Those two are exactly the axes of chunk 11's 2 x 2, so composition sees the
*shape* of each held-out family without ever seeing a held-out scenario. That is
the intended separation, not a hole in it.

**The sensor configuration** — beam count, field of view, maximum range,
detection radius. `core/scenario.py` already argues these belong to the task and
are identical in every condition; a tree that cannot see how far it can see
cannot reason about speed.

**The twist saturation** — the same argument, and it bounds what any Action can
actually achieve.

**`failure_signature`** — populated only on a repair, and it is the §5
failure-driven trigger. It describes what went wrong, never which scenario.

WHAT IS OUT
-----------
`id`, `start`, `goal`, `seed`, `scene`, `split` — and `time_limit`.
`SCENARIO_FIELDS_WITHHELD` names them so a test can assert it rather than a
reader having to notice.

**`time_limit` is out, and that is the second departure from chunk 16's
proposal.** The plan listed it, and it was in until the leak test measured it:
within three of the four held-out families the time limit is the ONLY field
that differs between the three scenarios, so a spec carrying it can tell an
episode from its family. That is precisely what G3 must not be able to do —
"solved by recombination alone" stops meaning anything if a tree could be
tailored to one episode. It is not merely unrendered, it is not a field: a spec
that holds something it never shows the model is a leak waiting for someone to
render it. Nothing is lost — no node reads a clock, the budget is enforced by
the runner, and the episode's time limit is recorded in the run manifest, which
is where a per-episode fact belongs.

**The start region is out, and that departs from what chunk 16 proposed.** The
plan's Decided section listed "start region" among the fields; the same plan's
Done-when forbids start coordinates. Any bucketing coarse enough not to identify
a scenario is also too coarse to compose differently against, and a tree that
conditioned on where it started would be coupled to a scenario, which
non-negotiable #4 forbids leaves from being. So it is dropped entirely rather
than blurred, and this comment is the record of the departure.

**The family name is in, and it is not identifying**: chunk 11's split is 12
scenarios in 4 families of 3, so a family cannot name an episode. It is carried
because a batch runner needs to group by it, NOT because the prose the model
reads is built from it — `describe()` below is a function of the axes alone, so
it cannot say more than `goal_known` and `pose_drifts` already say.
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# Scenario fields a TaskSpec must never carry. Asserted, not remembered:
# core/tests/test_compose_leak.py checks a rendered held-out spec against the
# real values, and compose/tests/test_spec.py checks the field list itself.
SCENARIO_FIELDS_WITHHELD = frozenset({"id", "start", "goal", "seed", "scene",
                                      "split", "time_limit"})

# Retrieval vocabulary, derived from the axes and NOT from any card. Every word
# here describes the task; none of them names a mechanism. "frontier",
# "path-integration" and "homing" were all considered and dropped for exactly
# that reason — a task spec that says "homing" has told composition which
# mechanism to reach for, which is the corpus leaking in through the query.
BASE_TAGS = ("navigate", "goal", "obstacle", "avoid", "drive", "speed",
             "clearance")
SEARCH_TAGS = ("search", "explore", "coverage", "unknown", "disclosure")
DRIFT_TAGS = ("localisation", "drift", "pose", "uncertainty")


@dataclass(frozen=True)
class TaskSpec:
    """One task, as composition sees it. Frozen: a spec that could be edited
    after retrieval would make the retrieval log a record of something else."""

    family: str
    goal_known: bool
    pose_drifts: bool
    n_beams: int
    fov: float
    range_max: float
    detect_radius: float
    max_lin: float
    max_ang: float

    # Populated only on a repair. See compose.compose.repair().
    failure_signature: Optional[str] = None

    @classmethod
    def from_scenario(cls, scenario, failure_signature=None):
        """Project a Scenario onto the fields above.

        Deliberately field-by-field rather than `dataclasses.asdict` minus a
        deny-list: a field added to Scenario later must be an explicit decision
        to disclose, not something that arrives in the prompt because nobody
        updated a set.
        """
        return cls(
            family=scenario.family,
            goal_known=scenario.goal_known,
            pose_drifts=scenario.drift is not None,
            n_beams=int(scenario.n_beams),
            fov=float(scenario.fov),
            range_max=float(scenario.range_max),
            detect_radius=float(scenario.detect_radius),
            max_lin=float(scenario.max_lin),
            max_ang=float(scenario.max_ang),
            failure_signature=failure_signature,
        )

    # -- the retrieval query -------------------------------------------------

    def tags(self):
        """The retrieval query, as a tag set. A function of the axes only."""
        tags = list(BASE_TAGS)
        if not self.goal_known:
            tags += list(SEARCH_TAGS)
        if self.pose_drifts:
            tags += list(DRIFT_TAGS)
        return tuple(sorted(set(tags)))

    # -- what the model reads ------------------------------------------------

    def describe(self):
        """The task in prose, generated FROM THE AXES rather than looked up.

        Not a table keyed by family name. A lookup table could hold a sentence
        that says more than the axes do — including, eventually, something a
        paper suggested — and nothing would catch it. Generating the text means
        the prose cannot carry information the fields do not.
        """
        lines = ["Reach a goal in an indoor space with no map. Walls and "
                 "furniture occlude; the route is not a straight line and an "
                 "obstruction is often not visible until the robot is close."]
        if self.goal_known:
            lines.append(
                "The goal position IS disclosed at the start: `obs.goal` is "
                "populated from the first step.")
        else:
            lines.append(
                "The goal position is NOT disclosed. `obs.goal` is None until "
                "the robot has been within "
                f"{self.detect_radius:.1f} m of the goal with nothing in the "
                "way, after which it latches and stays populated. Until then "
                "the task is to cover ground that has not been covered.")
        if self.pose_drifts:
            lines.append(
                "`obs.pose_estimate` DRIFTS: the believed pose wanders away "
                "from the true one over the episode, and "
                "`obs.pose_error_estimate` reports how wrong it is believed to "
                "be. The scan is unaffected — what degrades is the belief "
                "about where the robot is, not what it can see.")
        else:
            lines.append("`obs.pose_estimate` is exact.")
        lines.append(
            f"The scan is {self.n_beams} beams over {self.fov:.2f} rad, to "
            f"{self.range_max:.1f} m. The Twist is saturated at "
            f"{self.max_lin:.2f} m/s and {self.max_ang:.2f} rad/s. The episode "
            f"has a fixed time limit, enforced by the runner.")
        if self.failure_signature:
            lines.append(
                "THIS IS A REPAIR. The previous composition failed like this: "
                + self.failure_signature.strip())
        return "\n\n".join(lines)

    def as_dict(self):
        """What goes in the composition record. Every field, so a reader can
        reconstruct exactly what composition was told."""
        return {
            "family": self.family,
            "goal_known": self.goal_known,
            "pose_drifts": self.pose_drifts,
            "n_beams": self.n_beams,
            "fov": self.fov,
            "range_max": self.range_max,
            "detect_radius": self.detect_radius,
            "max_lin": self.max_lin,
            "max_ang": self.max_ang,
            "failure_signature": self.failure_signature,
        }

    @property
    def axes(self):
        """The 2 x 2 cell this task sits in. What a G3 table is grouped by."""
        return ("goal_known" if self.goal_known else "goal_hidden",
                "drift" if self.pose_drifts else "no_drift")
