"""
runs/aggregate.py

Committed results -> the pre-registered verdict and the named secondaries.

    from runs.aggregate import collect, report
    table = collect(spec, plan, results_dir="results")
    print(report(table))

Two rules from `PRE-REGISTRATION.md` are enforced here rather than remembered:

1. **§2 -- the decision rule is `core.stats.decide()`.** This module does not
   compute a confidence interval. It assembles outcomes and calls that function,
   because a second implementation of the rule is a second rule.
2. **§5 -- SPL is reported per family and never pooled.** The denominator is the
   omniscient taut path, which is near-meaningless for a search family, so a
   pooled SPL is a number that means nothing in particular. The pooling is not
   available in the API, not merely discouraged.

And one from non-negotiable #10: **an unbalanced set raises.** A condition that
crashed on two scenarios is not a weaker result, it is a different benchmark.

Sim-free. Reads JSON, does arithmetic.
"""

import json
import os
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np

from core.stats import X, EpisodeOutcome, decide
from runs.design import DECISION_SPLIT, DEV, HELD_OUT

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# The primary metric. One key, no alternatives held in reserve.
PRIMARY = "goal_reached"

# The four named secondaries, PRE-REGISTRATION.md §5. Naming them here rather
# than deriving them from score()'s keys is the point: reporting all eighteen
# and highlighting the significant ones is multiple comparisons by another name.
SECONDARIES = ("spl", "collisions", "stuck_events", "time_to_goal")

# Secondaries that must be broken out by family rather than pooled, and why.
PER_FAMILY = {
    "spl": "the denominator is the omniscient taut path, so it is "
           "near-meaningless for a search family",
}

# Secondaries defined only on some episodes. Reported conditioned on that, and
# always next to the rate it is conditioned on -- a fast mean time-to-goal over
# two successes is not better than a slower one over twelve.
CONDITIONED_ON_SUCCESS = ("time_to_goal", "spl")

# Reported, never compared. PRE-REGISTRATION.md §5: drift is injected open-loop
# into a separate RNG stream, so no controller can influence this and a
# controller can lower it only by finishing sooner. It answers "did drift bite",
# and reporting it as a performance secondary would let "condition B localises
# better" be read out of a number that cannot show it.
MANIPULATION_CHECKS = ("localisation_failures",)

# Everything else score() returns. Printed as a table, marked descriptive, with
# no test run on any of it.
DESCRIPTIVE_NOTE = ("descriptive only -- no claim rests on these and no "
                    "significance test is run on them")

# Caveats that travel with a figure wherever it is printed, PRE-REGISTRATION.md
# §5. Attached here so that a report cannot show the number without them.
CAVEATS = {
    "collisions": "`contact` is an ANALYTIC proxy (clearance - ROBOT_RADIUS "
                  "<= 0); no physics contact sensor is wired to the Go2",
    "spl": "per family only; pooling across families is meaningless",
}


# ---------------------------------------------------------------------------


class Incomplete(Exception):
    """The set is not the set that was pre-registered. Raised, not warned:
    scoring what happens to be on disk is how an ablation quietly becomes a
    comparison between two different benchmarks."""


def _load(results_dir, name):
    full = results_dir if os.path.isabs(results_dir) \
        else os.path.join(REPO_ROOT, results_dir)
    with open(os.path.join(full, name)) as fh:
        return json.load(fh)


@dataclass
class ConditionTable:
    label: str
    controller: str
    library_snapshot: Optional[str]
    results: Dict[str, dict] = field(default_factory=dict)   # run_id -> result
    families: Dict[str, str] = field(default_factory=dict)   # run_id -> family


@dataclass
class Collected:
    spec_name: str
    split: str
    conditions: Dict[str, ConditionTable]
    prereg_digests: set
    library_snapshots: Dict[str, Optional[str]]

    def outcomes(self, label):
        table = self.conditions[label]
        return [EpisodeOutcome(scenario=r["scenario"], seed=r["seed"],
                               success=bool(r[PRIMARY]))
                for r in table.results.values()]

    def rate(self, label):
        vals = [bool(r[PRIMARY]) for r in self.conditions[label].results.values()]
        return float(np.mean(vals)) if vals else float("nan")


def collect(spec, plan, results_dir="results", split=None):
    """Read every result the plan names, for one split. Raises if any is absent.

    `split` defaults to the decision split. Dev and held-out are collected
    separately and never pooled -- PRE-REGISTRATION.md §4 -- so this takes one
    split rather than offering a way to merge them.
    """
    split = DECISION_SPLIT if split is None else split
    wanted = [e for e in plan.episodes if e.split == split]
    if not wanted:
        raise Incomplete(f"the plan contains no {split} episodes")

    conditions, missing, digests, snapshots = {}, [], set(), {}
    for condition in spec.conditions:
        table = ConditionTable(condition.label, condition.controller,
                               condition.library_snapshot)
        for episode in (e for e in wanted if e.condition == condition.label):
            try:
                payload = _load(results_dir, episode.result_name)
            except FileNotFoundError:
                missing.append((condition.label, episode.run_id))
                continue
            table.results[episode.run_id] = payload["result"]
            table.families[episode.run_id] = episode.family
            manifest = payload.get("manifest", {})
            digests.add(manifest.get("prereg_digest"))
            snapshots.setdefault(condition.label, manifest.get("library_snapshot"))
        conditions[condition.label] = table

    if missing:
        by_condition = defaultdict(list)
        for label, run in missing:
            by_condition[label].append(run)
        lines = "\n  ".join(
            f"{label}: {len(runs)} missing" for label, runs in sorted(by_condition.items()))
        raise Incomplete(
            f"{len(missing)} of {len(wanted)} {split} results are missing, so "
            f"the conditions did not run the same episodes. Non-negotiable #10 "
            f"requires identical episode counts; scoring the remainder would "
            f"compare two different benchmarks.\n  {lines}\n"
            f"Run `scripts/run_sweep.py` again -- it resumes.")

    return Collected(spec_name=spec.name, split=split, conditions=conditions,
                     prereg_digests=digests, library_snapshots=snapshots)


def verdict(collected, b="B", d="D"):
    """G2's rule, via `core.stats.decide()`. Never reimplemented here.

    The labels ride along so the summary line names the pair it is actually
    about. B-D is the headline; B-A is confounded with principle articulation
    and is reported as such, and a line that called A "D" would hide that.
    """
    return decide(collected.outcomes(b), collected.outcomes(d), labels=(b, d))


def secondary(collected, label, metric):
    """One secondary for one condition. Returns (mean, n, note) or (None, 0, ...).

    `None` where the metric is undefined for every episode -- `time_to_goal` on
    a condition that never arrived -- rather than 0.0, which would read as
    "instant" instead of "never".
    """
    vals = []
    for r in collected.conditions[label].results.values():
        if metric in CONDITIONED_ON_SUCCESS and not r[PRIMARY]:
            continue
        v = r.get(metric)
        if v is not None:
            vals.append(float(v))
    note = ("over successes only" if metric in CONDITIONED_ON_SUCCESS else "")
    if not vals:
        return None, 0, note
    return float(np.mean(vals)), len(vals), note


def secondary_by_family(collected, label, metric):
    """The same, split by task family. The only way `spl` may be reported."""
    buckets = defaultdict(list)
    table = collected.conditions[label]
    for run_id, r in table.results.items():
        if metric in CONDITIONED_ON_SUCCESS and not r[PRIMARY]:
            continue
        v = r.get(metric)
        if v is not None:
            buckets[table.families[run_id]].append(float(v))
    return {fam: (float(np.mean(v)), len(v)) for fam, v in sorted(buckets.items())}


def report(collected, b="B", d="D"):
    """The whole reading, as text. Every number carries its caveat."""
    lines = []
    labels = sorted(collected.conditions)
    lines.append(f"spec {collected.spec_name!r}, split {collected.split}, "
                 f"conditions {', '.join(labels)}")

    digests = {x for x in collected.prereg_digests if x}
    if len(digests) > 1:
        lines.append("  ⚠️ RESULTS SPAN MORE THAN ONE PRE-REGISTRATION DIGEST. "
                     "Some episodes were scored under constants others were "
                     "not. This comparison is not valid as it stands.")
    elif not digests:
        lines.append("  ⚠️ no prereg_digest on any result: scored under "
                     "constants nobody registered.")
    else:
        lines.append(f"  pre-registration digest {list(digests)[0][:16]}…")
    for label in labels:
        snap = collected.library_snapshots.get(label)
        lines.append(f"  {label}: {collected.conditions[label].controller}  "
                     f"n={len(collected.conditions[label].results)}  "
                     f"snapshot={snap or 'NONE (#8 unmet)'}")

    lines.append("")
    lines.append(f"PRIMARY -- goal-reach rate (X = {X:.2f})")
    for label in labels:
        lines.append(f"  {label}  {collected.rate(label):.3f}")

    if b in collected.conditions and d in collected.conditions:
        v = verdict(collected, b, d)
        lines.append("")
        lines.append(f"DECISION RULE {b} - {d}")
        lines.append(f"  {v.summary()}")
        if not v.fires and v.paired_excludes_zero:
            lines.append("  reading: a consistent per-scenario advantage that "
                         "the pre-registered rule cannot resolve at this many "
                         "scenarios. Suggestive and underpowered, NOT the "
                         "headline (PRE-REGISTRATION.md §2).")
        elif not v.fires:
            lines.append("  reading: null. And note the rule is conservative — "
                         "not firing is NOT evidence the corpus does not help.")
        elif v.fires and not v.paired_excludes_zero:
            lines.append("  reading: should not occur. Investigate before "
                         "claiming anything (PRE-REGISTRATION.md §2).")

    lines.append("")
    lines.append("SECONDARIES -- the four named in advance")
    for metric in SECONDARIES:
        if metric in PER_FAMILY:
            lines.append(f"  {metric} (per family: {PER_FAMILY[metric]})")
            for label in labels:
                per = secondary_by_family(collected, label, metric)
                shown = ", ".join(f"{fam} {m:.3f} (n={n})"
                                  for fam, (m, n) in per.items()) or "no data"
                lines.append(f"    {label}  {shown}")
        else:
            row = []
            for label in labels:
                mean, n, note = secondary(collected, label, metric)
                row.append(f"{label} " + ("--" if mean is None else f"{mean:.3f}")
                           + f" (n={n})")
            suffix = CAVEATS.get(metric, "")
            _, _, note = secondary(collected, labels[0], metric)
            lines.append(f"  {metric:<14} " + "  ".join(row)
                         + (f"   [{note}]" if note else "")
                         + (f"   [{suffix}]" if suffix else ""))

    lines.append("")
    lines.append("MANIPULATION CHECKS -- reported, never compared")
    for metric in MANIPULATION_CHECKS:
        row = []
        for label in labels:
            mean, n, _ = secondary(collected, label, metric)
            row.append(f"{label} " + ("--" if mean is None else f"{mean:.2f}"))
        lines.append(f"  {metric:<24} " + "  ".join(row)
                     + "   [did drift bite? A controller can lower this only "
                       "by finishing sooner]")
    return "\n".join(lines)
