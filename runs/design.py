"""
runs/design.py

The episode set, derived rather than typed.

`PRE-REGISTRATION.md` fixes four conditions, fifteen scenarios and three seeds.
This module is the one place that turns those constants into a list of episodes,
and everything downstream -- the sweep, the resume check, the aggregation --
reads the same list. Non-negotiable #10 ("identical episode counts across
conditions") therefore holds because the same generator produced every
condition's episodes, not because somebody checked afterwards.

    from runs.design import sweep_design
    plan = sweep_design(spec)
    print(len(plan.episodes), plan.per_condition())

Sim-free. No simulator, no GPU, no subprocess, no I/O. It knows what should run
and nothing about running it, which is what makes the whole design testable in
milliseconds.

**This module imports the sealed split** -- it has to, since the split IS the
evaluation set. That is allowed: chunk 11's AST guard covers `controllers/`,
`nodes/`, `generate/` and `compose/`, the four places a model's context could
reach. `runs/` is downstream of generation and composition and never feeds them.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from core.manifest import run_id
from core.scenario import DEV_SCENARIOS, all_scenarios
from core.stats import seeds_for


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# The two splits, and which one the decision rule applies to.
#
# `PRE-REGISTRATION.md` §4: both are run and reported, separately and never
# pooled. Held-out is the headline and is the only set §2's rule is applied to;
# dev is descriptive and is labelled as the set the prompts were iterated
# against. Keeping the label on every episode is what makes "we report the one
# that worked" impossible to do by accident.
HELD_OUT = "held_out"
DEV = "dev"
DECISION_SPLIT = HELD_OUT


# ---------------------------------------------------------------------------


def _held_out_scenarios():
    """The sealed split, through `core.scenario`'s own door.

    `all_scenarios("held_out")` rather than importing
    `core.scenarios_held_out` directly. That function is deliberately the ONLY
    public way to get the split -- `all_scenarios()` with no argument never
    returns one, precisely so that "a batch runner that iterates everything"
    cannot evaluate against it by accident. Asking by name is the explicit act,
    and this module is the one place in the repo entitled to make it.
    """
    return all_scenarios("held_out")


@dataclass(frozen=True)
class Episode:
    """One (condition, scenario, seed) triple, and the name its result takes.

    `run_id` is computed here rather than by the caller so that the resume
    check, the subprocess invocation and the aggregation are all looking for the
    same filename.
    """

    condition: str
    controller: str
    scenario: str
    seed: int
    split: str
    family: str
    time_limit: float

    @property
    def run_id(self):
        return run_id(self.scenario, self.controller, self.seed)

    @property
    def result_name(self):
        return self.run_id + ".json"


@dataclass(frozen=True)
class SweepPlan:
    """Every episode a full sweep runs, in execution order."""

    episodes: Tuple[Episode, ...]

    def __len__(self):
        return len(self.episodes)

    def per_condition(self):
        out = {}
        for ep in self.episodes:
            out[ep.condition] = out.get(ep.condition, 0) + 1
        return out

    def per_split(self):
        out = {}
        for ep in self.episodes:
            out[ep.split] = out.get(ep.split, 0) + 1
        return out

    def for_split(self, split):
        return SweepPlan(tuple(e for e in self.episodes if e.split == split))

    def for_condition(self, condition):
        return SweepPlan(tuple(e for e in self.episodes if e.condition == condition))

    def worst_case_sim_seconds(self):
        """If every episode ran to timeout. The number the wall-clock estimate
        in PRE-REGISTRATION.md §3 is built on, recomputed from the actual plan
        rather than quoted."""
        return sum(e.time_limit for e in self.episodes)

    def is_balanced(self):
        """Every condition covers exactly the same (scenario, seed) pairs.

        #10, as a property of the plan rather than a hope about it. Checked here
        AND enforced again at aggregation, because a plan can be balanced and a
        sweep can still leave holes in it.
        """
        by_condition = {}
        for ep in self.episodes:
            by_condition.setdefault(ep.condition, set()).add((ep.scenario, ep.seed))
        sets = list(by_condition.values())
        return bool(sets) and all(s == sets[0] for s in sets)


def scenario_set(splits=(HELD_OUT, DEV)):
    """(scenario, split) pairs the sweep covers, in a stable order.

    Held-out first: it is the set the decision rule reads, so an interrupted
    sweep should have spent its time there rather than on the dev set.
    """
    out = []
    if HELD_OUT in splits:
        out += [(s, HELD_OUT) for s in _held_out_scenarios()]
    if DEV in splits:
        out += [(s, DEV) for s in DEV_SCENARIOS]
    return out


def sweep_design(spec, splits=(HELD_OUT, DEV)):
    """The full episode set, in execution order.

    **Scenario-major, condition-minor.** Determinism means the order cannot
    change any result, so it is chosen for what a PARTIAL sweep tells you:
    interrupted after two hours, this leaves a balanced set across all four
    conditions, where condition-major would leave one condition finished and
    three untouched. It also surfaces "condition D crashes on everything" in the
    first few minutes rather than in the last quarter of a ten-hour run.
    """
    episodes = []
    for scenario, split in scenario_set(splits):
        for seed in seeds_for(scenario.seed):
            for condition in spec.conditions:
                episodes.append(Episode(
                    condition=condition.label,
                    controller=condition.controller,
                    scenario=scenario.id,
                    seed=seed,
                    split=split,
                    family=scenario.family,
                    time_limit=scenario.time_limit,
                ))
    return SweepPlan(tuple(episodes))
