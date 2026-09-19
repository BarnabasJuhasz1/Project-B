"""repair/evidence.py — everything the diagnosing agent is allowed to see.

One node, one object, **with every LABEL of which arm produced it removed.**

⚠️ **WHY BLINDNESS IS LOAD-BEARING.** The repair loop is a second generation
stage. If it can tell which arm it is repairing it can — deliberately or not —
try harder for one of them, and B−A stops measuring extraction and starts
measuring how hard the loop worked. The existing pipeline solves this
structurally: `render_node(card, task)` has no condition parameter and a test
pins it. This gets the same treatment.

⚠️ **WHAT BLINDNESS CAN AND CANNOT DO — READ THIS BEFORE TRUSTING IT.**

The bundle carries **no label** of the arm: no condition letter, no
`treatment`, no `ref_id`, no `paper_sha256`, no `source`, no `span`, no
`confidence`, and no id or filename with the arm letter in it.

It necessarily carries the arm's **effect**. A card written from a paper in
full reads differently from one written from a citation alone — it cites
equations, page numbers and fitted constants — and that difference IS the
independent variable. Scrubbing it would destroy the thing being diagnosed.

So the claim is exactly: **the agent is never TOLD the arm, and cannot recover
it from metadata.** It is not that the arm is information-theoretically
unrecoverable from a rich card. That is the same position the node prompt has
always been in, and it is stated rather than glossed.

⚠️ **A and B are pointed at the SAME paper**, so naming the paper's subject
does not separate them — it separates {A, B} from D, which is a weaker leak and
one the card content carries anyway.

Usage:

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P -m repair.evidence --node gen_b_44d12df4d25b437f          # render it
    $P -m repair.evidence --node gen_b_44d12df4d25b437f --case 2004
"""
from __future__ import annotations

import hashlib
import importlib.util
import textwrap
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from repair.verdict import (DIRECTION, Verdict, claim_for, classify,
                            lineage_root, load_measurement)  # noqa: F401

REPO_ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

# ⚠️ THE HANDLE MUST NOT BE INVERTIBLE. A node id is `gen_<arm>_<plan_hash>`
# and the stem is `<arm>__<task>__<plan_hash>`, so the arm is one letter in
# plain text -- and that is the leak that gets missed, because everything in
# the loop addresses nodes by id.
#
# A handle derived from the id alone would be brute-forceable: there are three
# arms, so an agent that knew the hash function and the plan hash could try all
# three and compare. The salt removes that. It is a fixed constant rather than
# a per-run random so a bundle is reproducible and a committed fixture stays
# valid; the agent never sees this file (`--setting-sources ''`), so a fixed
# salt it cannot read is as good as a random one it cannot read.
HANDLE_SALT = "repair-bundle-v1"
HANDLE_LEN = 12

# What a card is allowed to contribute. Not a list of fields to strip -- a list
# of fields to KEEP, because a denylist silently passes anything added later.
# `generator.card_yaml` already renders exactly these and no more, so the
# allowlist is expressed by reusing it rather than by filtering after the fact.
CARD_FIELDS_RENDERED = ("id", "node_name", "mechanism", "trigger",
                        "derivation", "prediction")

# The arm letters, in every position they appear in this repo.
ARM_TELLS = ("gen_a_", "gen_b_", "gen_d_", "a__", "b__", "d__",
             "condition:", "condition=", "treatment:", "treatment=",
             "ref_id", "paper_sha256", "citation:", "span:", "verified:",
             "confidence:", "card_row_key", "card_prompt_hash")

# How much of one case to show. A full 6000-step trace is ~700 kB and would
# dominate a prompt; the SHAPE is what a diagnosis needs -- where it went, when
# it stopped, and what the base did instead.
CASE_SAMPLES = 24


# ---------------------------------------------------------------------------


def handle_for(node_id: str) -> str:
    """An opaque, stable, non-invertible name for a node."""
    digest = hashlib.sha256((HANDLE_SALT + node_id).encode("utf-8")).hexdigest()
    return f"node-{digest[:HANDLE_LEN]}"


@dataclass(frozen=True)
class Bundle:
    """What the diagnosing agent sees. Nothing else reaches it."""

    handle: str
    card: str
    node_source: str
    verdict: Verdict
    worlds: str
    conditions: str
    contract: str
    case: Optional[str] = None

    def render(self) -> str:
        parts = [
            f"# {self.handle}", "",
            "## The principle card this node was built from", "",
            "```yaml", self.card.strip(), "```", "",
            "## The node, as generated", "",
            "```python", self.node_source.strip(), "```", "",
            "## What it scored", "",
            self._verdict_block(), "",
            "## Per-world outcomes", "",
            self.worlds.strip(), "",
            "## The conditions the measurement ran under", "",
            self.conditions.strip(), "",
            "## The contract: what this node could have read", "",
            self.contract.strip(),
        ]
        if self.case:
            parts += ["", "## One world, opened up", "", self.case.strip()]
        return "\n".join(parts) + "\n"

    def _verdict_block(self):
        v = self.verdict
        out = [f"verdict: {v.label.value}",
               f"reason:  {v.reason}",
               f"n:       {v.n} worlds",
               f"outcomes: " + ", ".join(f"{k} {n}" for k, n in sorted(v.counts.items()))]
        if v.claim_metric:
            delta = "not measured" if v.claim_delta is None else f"{v.claim_delta:+.4f}"
            out.append(f"its own claim: {v.claim_metric}"
                       f"{'/m' if v.per_metre else ''} moved {delta} "
                       f"against a floor of {v.claim_min_effect:g}")
        for u in v.unchecked:
            out.append(f"NOT CHECKED: {u}")
        return "\n".join(out)

    def leaks(self):
        """Every arm tell found in the rendered bundle. Empty is the contract.

        ⚠️ Scans the WHOLE rendered text, the node source included. A node
        source that names its own paper is not an A-vs-B tell (both arms are
        pointed at the same paper) but an explicit `condition:` anywhere is,
        and this is what catches it.
        """
        text = self.render().lower()
        return tuple(t for t in ARM_TELLS if t.lower() in text)


# ---------------------------------------------------------------------------
# building one


# ⚠️ ONE RESOLVER FOR "GIVE ME THIS NODE", AND IT LIVES HERE ON PURPOSE.
# `repair/loop.py` imports evidence, so the resolver cannot live there without
# a cycle — and it has to be shared, because the alternative is what actually
# happened: FOUR separate live failures, each a different component assuming a
# node id names a member of the committed library. A repaired node never is
# one. Fixing the fourth call site would only have waited for the fifth.
WORK = REPO_ROOT / "out" / "repair"


def round_dir(node_id, number):
    """Where one repair round is staged.

    ⚠️ ROUND-NUMBERED, and not for tidiness. `plan_hash` is a pure function of
    (condition, task, card) and the card is NOT regenerated, so every round
    produces the IDENTICAL stem — two rounds cannot share a row key, the same
    reason chunk 52 moved the v2 records aside. Rounds live OUTSIDE
    `generate/staging/`, which is the experiment's record: a repair round is a
    candidate, not a library member, until it earns it.
    """
    return WORK / node_id / f"round-{number:02d}"


def entry_for(node_id):
    """The library entry for a node id — library member OR repair round.

    Every component that needs to RUN a node goes through this.
    """
    from compose.library import load_library

    if "__r" not in node_id:
        return load_library().get(node_id)

    # ⚠️ rpartition, and the digit check, are both load-bearing. A compounded
    # id reached here once (`..._r01__r02`) and died on `int('01__r02')` three
    # rounds into a live run — an unreadable traceback for a naming bug. The
    # root cause is fixed in `loop.regenerate`; this says so out loud if
    # anything ever reintroduces it.
    parent, _, suffix = node_id.rpartition("__r")
    if not suffix.isdigit():
        raise ValueError(
            f"{node_id!r} is not a repair-round id: expected one '__rNN' "
            f"suffix off the lineage root, got {suffix!r}")
    # ⚠️ THE FIFTH LIVE FAILURE, AND THE WORST, BECAUSE IT NEVER RAISED.
    #
    # `ContractPorts` is ALWAYS in a loaded library — it is the harness's own
    # port shim, exempt by design (`compose/library.py`) — and it sorts first.
    # So `entries[0]` returned the SHIM for every repaired node, and the loop
    # measured the shim instead of the repair. It does nothing, so it fired on
    # 0.0% of ticks; that is INERT; two rounds of it produce the IDENTICAL
    # behaviour signature; and the oscillation guard then terminated the
    # lineage as UNREPAIRED. A whole 3-round run, every round green, every
    # verdict an artefact.
    #
    # The lesson is the one the comment above already draws: a resolver that
    # GUESSES which entry was meant will eventually guess wrong silently.
    # Ambiguity raises.
    from compose.library import CONTRACT_PORTS_ID

    lib = load_library(staging=round_dir(parent, int(suffix)))
    entries = [e for e in lib.all()
               if getattr(e, "node_id", None) != CONTRACT_PORTS_ID
               and e.node_cls.__name__ != "ContractPorts"]
    if not entries:
        raise KeyError(f"round {suffix} of {parent} staged nothing loadable")
    if len(entries) > 1:
        raise KeyError(
            f"round {suffix} of {parent} staged {len(entries)} nodes "
            f"({[e.node_cls.__name__ for e in entries]}) — a repair round is "
            f"ONE node call and must stage exactly one")
    return entries[0]


def _stem_for(node_id):
    """The staging stem. ⚠️ A repaired node shares its PARENT's stem — the card
    is not regenerated, so `plan_hash` and therefore the stem are unchanged."""
    from repair.verdict import _stem_for as stem
    from generate.generator import STAGING
    return stem(lineage_root(node_id), STAGING)


def card_block(node_id) -> str:
    """The card, rendered EXACTLY as the node call saw it.

    ⚠️ Reuses `generator.card_yaml` rather than filtering a YAML file. It
    already excludes `source`, `span` and `confidence`, it already exists
    because the NODE prompt had this identical problem, and a second scrubber
    is a second thing to keep in sync. Reuse is the point, not a shortcut.
    """
    from generate.cardgen import load_generated
    from generate.generator import card_yaml

    stem = _stem_for(node_id)
    if stem is None:
        return "# no card found for this node"
    path = REPO_ROOT / "generate" / "cards" / stem.split("__")[0] / f"{stem}.yaml"
    if not path.exists():
        return "# no card found for this node"
    return card_yaml(load_generated(path).card)


def node_source(node_id) -> str:
    stem = _stem_for(node_id)
    if stem is None:
        return "# no staged source for this node"
    from generate.generator import STAGING
    path = Path(STAGING) / f"{stem}.py"
    return path.read_text(encoding="utf-8") if path.exists() else "# missing"


def worlds_block(measurement) -> str:
    """One row per world. ⚠️ NOT the aggregate: `fixed = 2` never says WHICH
    two, and a diagnosis needs the geometry that broke, not the mean."""
    head = f"{'seed':>6} {'base':>5} {'node':>5} {'outcome':<10} {'fire':>6} {'changed':>8}"
    rows = [head, "-" * len(head)]
    for w in measurement.worlds:
        rows.append(f"{w.seed:>6} {int(w.base_reached):>5} {int(w.pair_reached):>5} "
                    f"{w.outcome:<10} {w.fire:>6.3f} {w.changed:>8.3f}")
    return "\n".join(rows)


def conditions_block(tier="maze", seed=None) -> str:
    """⚠️ THE CONDITIONS OF THE TEST, NOT ONLY ITS RESULT — INCLUDING THE AXES
    LEFT AT THEIR DEFAULTS.

    Found 2026-09-16: `core.worldgen.to_scenario()` never sets `drift`, so every
    world in the growth suite hands the controller a PERFECT POSE ESTIMATE. A
    card can predicate its whole claim on drift, and an agent shown only the
    geometry and the outcome cannot possibly notice. An agent shown
    `drift: None` beside such a card has a chance of it in one step.
    """
    from core.worldgen import generate, to_scenario

    s = to_scenario(generate(tier, int(seed if seed is not None else 2000)))
    lines = [
        "Every world below was generated by core.worldgen and scored kinematically.",
        "",
        f"  tier                : {tier}",
        f"  drift               : {s.drift}",
        f"  localisation        : {getattr(s, 'localisation', None)}",
        f"  goal_known          : {s.goal_known}",
        f"  sensor_noise        : {getattr(s, 'sensor_noise', None)}",
        f"  time_limit          : {s.time_limit} s",
        "",
        "⚠️ An axis reading `None` was NOT VARIED. `drift: None` means the pose",
        "estimate handed to the controller is exact — there is no localisation",
        "error of any kind in these worlds. If this node's claim depends on an",
        "axis that reads None, it has not been tested against its own claim.",
    ]
    return "\n".join(lines)


def contract_block() -> str:
    """What the node could have read, and how it is allowed to act.

    A diagnosis needs to know what was AVAILABLE and unused, not only what was
    used -- "it never read the base's chosen heading" is only a finding if the
    heading was there to read.
    """
    from nodes.contract import PortType

    lines = [
        "A node sits in a Layer. The shared base proposes a whole Twist every",
        "tick; later nodes revise only the fields they declared in WRITES.",
        "",
        "Two combinators and no more:",
        "  ports.cap(vx=...)     min() of magnitude, sign kept",
        "  ports.adjust(vx=...)  LAST WRITER WINS — it replaces the field",
        "",
        "A node that declines leaves the base's command byte-identical.",
        "",
        "Port types available: " + ", ".join(sorted(p.name for p in PortType)),
        "",
        "Ports the shared base publishes every tick, readable by any later node:",
        "  nav_heading   BEARING  the direction the base has chosen to travel",
        "  nav_stuck     FLAG     the base believes it is stuck or off its plan",
        "  nav_progress  SCALAR   1.0 falling to 0.0 as time passes with no progress",
        "",
        "⚠️ The base gates its own forward speed on being aligned with",
        "nav_heading, and raises its turn rate to a floor until it is. A node",
        "that overwrites wz without reading nav_heading removes the base's",
        "ability to reach the alignment its own speed gate is waiting for.",
    ]
    return "\n".join(lines)


def case_block(node_id, tier="maze", seed=None, samples=CASE_SAMPLES) -> str:
    """One world, opened up: the base and the node-augmented tree, side by side.

    ⚠️ Re-runs the SAME pairing `measure_growth.py` measured, by importing its
    `_build` rather than re-declaring it — the same rule `debug_growth_case.py`
    follows, and for the same reason: a bundle that explained a tree the row
    never measured would be worse than no bundle.

    KINEMATIC. Not a benchmark path, and it says so in the rendered text.
    """
    from core.worldgen import generate, to_scenario
    from generate import base as basemod

    try:
        entry = entry_for(node_id)
    except KeyError:
        return f"# {node_id} could not be resolved; no case could be built"
    if entry is None:
        return f"# {node_id} is not in the live library; no case could be built"

    g = generate(tier, int(seed))
    scenario = to_scenario(g)
    build = _growth_build()
    base = basemod.load()

    dbg = _debug_module()
    b_trace, b_score, _, _ = dbg._rollout(
        _tree_of(base), scenario, base.metadata.node_id)
    p_trace, p_score, _, summary = dbg._rollout(
        build(entry.node_cls, entry.metadata, base), scenario, node_id)

    lines = [
        f"world: {scenario.id}  (tier {tier}, seed {seed})",
        f"start {tuple(round(v, 2) for v in scenario.start[:2])} -> "
        f"goal {tuple(round(v, 2) for v in scenario.goal)}",
        f"geodesic {g.difficulty.geodesic_length:.2f} m, "
        f"tortuosity {g.difficulty.tortuosity:.2f}, "
        f"narrowest corridor {g.difficulty.min_corridor:.2f} m, "
        f"{g.difficulty.n_obstacles} obstacles",
        "",
        f"{'':<22}{'BASE ALONE':>14}{'BASE + NODE':>14}",
        f"{'goal reached':<22}{str(bool(b_score['goal_reached'])):>14}"
        f"{str(bool(p_score['goal_reached'])):>14}",
        f"{'path length (m)':<22}{b_score['path_length']:>14.2f}"
        f"{p_score['path_length']:>14.2f}",
        f"{'collisions':<22}{b_score['collisions']:>14}{p_score['collisions']:>14}",
        f"{'stuck events':<22}{b_score['stuck_events']:>14}{p_score['stuck_events']:>14}",
        f"{'min clearance (m)':<22}{b_score['min_clearance']:>14.3f}"
        f"{p_score['min_clearance']:>14.3f}",
        "",
        "Trajectories, sampled evenly over each run:",
        "",
        _trajectory_table(b_trace, p_trace, samples),
    ]
    if summary:
        lines += ["", "The node's own ticks:", textwrap.indent(str(summary).strip(), "  ")]
    lines += ["",
              "⚠️ KINEMATIC ROLLOUT. The frozen locomotion policy is absent, so no",
              "number here is reportable. It is the rollout that produced the row."]
    return "\n".join(lines)


def _trajectory_table(b_trace, p_trace, samples):
    """Both runs on ONE shared clock.

    ⚠️ Sampling each trace by its own index puts them on different clocks: the
    base stops when it reaches the goal, the pair runs to the time limit, and
    the rows interleave out of chronological order. The question the table has
    to answer is "where was each one AT THE SAME MOMENT", so the time axis is
    shared and a run that has ended shows as ended.
    """
    import numpy as np

    def at(trace, t):
        ts = trace.column("t")
        if len(ts) == 0 or t > ts[-1] + 1e-9:
            return None
        i = int(np.searchsorted(ts, t, side="left"))
        i = min(i, len(ts) - 1)
        return (trace.column("x")[i], trace.column("y")[i],
                trace.column("vx")[i], trace.column("wz")[i],
                trace.column("clearance")[i])

    end = max(b_trace.column("t")[-1], p_trace.column("t")[-1])
    grid = [end * k / (samples - 1) for k in range(samples)]

    head = (f"{'t':>6} | {'base x':>7}{'base y':>7}{'base vx':>8} | "
            f"{'node x':>7}{'node y':>7}{'node vx':>8}{'node wz':>8}{'clear':>7}")
    out = [head, "-" * len(head)]
    for t in grid:
        bb, pp = at(b_trace, t), at(p_trace, t)
        lhs = (f"{bb[0]:>7.2f}{bb[1]:>7.2f}{bb[2]:>8.2f}" if bb
               else f"{'(ended)':>22}")
        rhs = (f"{pp[0]:>7.2f}{pp[1]:>7.2f}{pp[2]:>8.2f}{pp[3]:>8.2f}{pp[4]:>7.2f}"
               if pp else f"{'(ended)':>38}")
        out.append(f"{t:>6.1f} | {lhs} | {rhs}")

    out.append("")
    out.append(f"base ended at t={b_trace.column('t')[-1]:.1f}s, "
               f"({b_trace.column('x')[-1]:.2f}, {b_trace.column('y')[-1]:.2f})")
    out.append(f"node ended at t={p_trace.column('t')[-1]:.1f}s, "
               f"({p_trace.column('x')[-1]:.2f}, {p_trace.column('y')[-1]:.2f})")
    return "\n".join(out)


def _tree_of(base):
    from nodes.tree import Tree, Layer
    from nodes.toys import toy_metadata
    from generate.admit import ContractPorts
    return Tree(Layer(toy_metadata("base_layer"), [
        ContractPorts(toy_metadata("base_ports")),
        base.node_cls(base.metadata),
    ]))


def _debug_module():
    """`scripts/debug_growth_case.py`, loaded by path — it is a script, not a
    package module, and its `_rollout` is the one definition of "run this pair
    and score it"."""
    spec = importlib.util.spec_from_file_location(
        "_dbg", REPO_ROOT / "scripts" / "debug_growth_case.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _growth_build():
    return _debug_module()._growth_build()


def _growth_module():
    """`scripts/measure_growth.py`, loaded by path.

    ⚠️ Loaded rather than reimplemented so `REPORTED_METRICS` and `_mean` have
    ONE definition. A repaired node and a committed one must be scored by the
    same arithmetic or their verdicts are not comparable — and the verdict rule
    is what decides whether a repair is kept.
    """
    spec = importlib.util.spec_from_file_location(
        "_growth", REPO_ROOT / "scripts" / "measure_growth.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def aggregate_metrics(base_scores, pair_scores):
    """Per-world scored dicts -> the `Measurement.metrics` mapping.

    ⚠️ THE BUG THIS EXISTS BECAUSE OF. `loop.measure_live` built its
    `Measurement` with `metrics={}`, so `pair_of("goal_reached")` returned
    `(None, None)` for EVERY repaired node. `primary_worse` was then always
    False and the `DEGENERATE`, `REGRESSED` and `HELPED` branches were all
    unreachable: a repaired node could only ever come back `INERT` or
    `ACCEPT`.

    It was not a silent near-miss either. `gen_b_…__r01` was classified
    **ACCEPT — "acted, harmed nothing measurable"** while losing four of the
    ten worlds the base solves. The verdict was not wrong about the data; it
    had no data.

    Shape matches `load_measurement`: `{metric: (base_mean, pair_mean)}`, the
    mean taken over the worlds where the value is not None, which is what
    `measure_growth._mean` does and why it is reused rather than rewritten.
    """
    g = _growth_module()
    metrics = {}
    for metric, _direction in g.REPORTED_METRICS:
        base, _nb = g._mean(base_scores, metric)
        pair, _np = g._mean(pair_scores, metric)
        metrics[metric] = (base, pair)
    return metrics


def bundle(node_id, tier="maze", case_seed=None, verdict=None,
           measurement=None) -> Bundle:
    """Assemble everything the diagnosing agent may see, blind by construction.

    `case_seed` opens one world up. Passing None skips it — the rest of the
    bundle is pure file reads and costs milliseconds, while a case runs two
    kinematic rollouts.
    """
    m = measurement if measurement is not None else load_measurement(node_id, tier=tier)
    v = verdict if verdict is not None else classify(m, claim_for(node_id))

    return Bundle(
        handle=handle_for(node_id),
        card=card_block(node_id),
        node_source=node_source(node_id),
        verdict=v,
        worlds=worlds_block(m),
        conditions=conditions_block(tier, seed=m.worlds[0].seed if m.worlds else None),
        contract=contract_block(),
        case=None if case_seed is None else case_block(node_id, tier, case_seed),
    )


def worst_case_seed(measurement):
    """The world most worth opening up: one the node LOST that the base won."""
    for outcome in ("broke", "both_fail", "fixed", "both_ok"):
        for w in measurement.worlds:
            if w.outcome == outcome:
                return w.seed
    return None


def _main(argv=None):
    import argparse

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--node", required=True)
    p.add_argument("--tier", default="maze")
    p.add_argument("--case", type=int, default=None,
                   help="seed of one world to open up; 'auto' picks the worst")
    p.add_argument("--auto-case", action="store_true")
    p.add_argument("--check-only", action="store_true",
                   help="print the leak scan and the size, not the bundle")
    a = p.parse_args(argv)

    m = load_measurement(a.node, tier=a.tier)
    seed = a.case
    if a.auto_case and seed is None:
        seed = worst_case_seed(m)

    b = bundle(a.node, tier=a.tier, case_seed=seed, measurement=m)
    text = b.render()
    leaks = b.leaks()

    if a.check_only:
        print(f"[evidence] handle    {b.handle}")
        print(f"[evidence] rendered  {len(text)} chars, ~{len(text) // 4} tokens")
        print(f"[evidence] case      {'seed ' + str(seed) if seed else 'none'}")
        print(f"[evidence] leaks     {leaks if leaks else 'NONE'}")
        return 1 if leaks else 0

    print(text)
    if leaks:
        print(f"\n⚠️ ARM TELLS PRESENT: {leaks}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
