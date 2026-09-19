"""repair/loop.py — the autonomous diagnose-and-repair loop. Chunk 58.

One node, one arm, one task, to a **terminal verdict**, with every round on disk.

    measure -> classify -> terminal? -> bundle -> diagnose -> act -> repeat

⚠️ **ALL FIVE EXITS ARE DATA. NONE IS AN ERROR.** A loop that can only terminate
on success will grind forever against a paper that does not transfer, and report
the grinding as progress.

⚠️ **INERT IS NOT AN EXIT.** The cheapest way to stop a node regressing is to
make it decline everywhere. A loop rewarded for "no longer regresses" finds this
immediately and every node converges on doing nothing while the table fills with
clean rows. INERT routes to PROBE_ENVIRONMENT; it never terminates the loop.

⚠️ **THE REPAIR PROMPT ADVERTISES PORTS THE GENERATION PROMPT DID NOT.** The base
publishes `nav_heading`, `nav_stuck` and `nav_progress` every tick, and
`generate/prompts/modifier.md` never mentions them — so no node in the library
could read them, and the arbitration failure that motivated the layered pipeline
is a PROMPT OMISSION rather than a model failure. `repair/prompts/repair_node.md`
documents them.

    Consequence to declare: a repaired node had information the original did not.
    It is applied IDENTICALLY IN EVERY ARM, so it cannot bias B−A — but a
    repaired node and an unrepaired one are not comparable with each other, and
    the write-up has to say so.

Usage:

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P scripts/repair_node.py --node gen_b_44d12df4d25b437f --dry-run
    $P scripts/repair_node.py --node gen_b_44d12df4d25b437f --rounds 3
"""
from __future__ import annotations

import hashlib
import json
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

REPAIR_PROMPT = Path(__file__).resolve().parent / "prompts" / "repair_node.md"
WORK = REPO_ROOT / "out" / "repair"

# ⚠️ THE DECLARED PER-NODE BUDGET. Fixed, and identical across arms — without
# that, B−A measures how long we kept going. Set to 3 on the user's instruction
# (2026-09-16): one diagnosis plus one node call per round, so ~6 model calls.
# ⚠️ It is PRE-REGISTRATION MATERIAL and must be written into a dated append
# before any cross-arm run.
DEFAULT_ROUNDS = 3

# ⚠️ Repairing against the worlds you then score on is fitting the test set.
# `probe.PROBE_SEEDS` (4000-4499) and `probe.CONFIRM_SEEDS` (9000-9499) are
# already disjoint; the loop repairs on the growth tier and confirms on the
# confirmation range, which it looks at ONCE, when a terminal verdict is
# proposed.
CONFIRM_N = 10


class Exit(str, Enum):
    """How a node leaves the loop. ⚠️ All five are data."""

    HELPED = "HELPED"                        # cleared its own claim on confirmation
    ACCEPT = "ACCEPT"                        # valid, harmless, kept without a win
    UNTESTABLE = "UNTESTABLE"                # no reachable world discriminates it
    NOT_TRANSFERABLE = "NOT_TRANSFERABLE"    # faithful, repaired, does not carry over
    UNREPAIRED = "UNREPAIRED"                # budget spent


# ---------------------------------------------------------------------------


@dataclass
class Round:
    """One pass. Everything that happened, on disk."""

    number: int
    node_id: str
    verdict_label: str = ""
    verdict_reason: str = ""
    signature: str = ""
    action: str = ""
    action_reason: str = ""
    diagnosis_hash: str = ""
    outcome: str = ""
    seconds: float = 0.0
    # ⚠️ THE AXIS THIS ROUND WAS MEASURED ON, recorded per round and not just
    # per run. Round 1 of this lineage was measured clean and later rounds may
    # not be, so a run-level field alone would mislabel it. None means the
    # growth suite as committed: a PERFECT POSE ESTIMATE.
    drift: Optional[float] = None

    def as_dict(self):
        return dict(self.__dict__)


@dataclass
class LoopResult:
    node_id: str
    exit: Optional[Exit]
    reason: str
    rounds: list = field(default_factory=list)
    seconds: float = 0.0
    drift: Optional[float] = None
    # ⚠️ WHERE THIS RUN PICKED THE LINEAGE UP. A re-run entered at `…__r01`
    # numbers its rounds from 2, so its `rounds` list does NOT describe round 1
    # — and round-01/ is still sitting in the lineage directory. Without this
    # field the live record silently omits the round everything after it was
    # built on, and the only account of it is in out/repair/archive/.
    continues_from: Optional[str] = None

    def log(self):
        head = f"{'#':>2}  {'verdict':<12}{'action':<20}{'outcome'}"
        lines = [f"node: {self.node_id}", "", head, "-" * (len(head) + 20)]
        for r in self.rounds:
            lines.append(f"{r.number:>2}  {r.verdict_label:<12}{r.action:<20}"
                         f"{r.outcome}")
        lines += ["", f"EXIT: {self.exit.value if self.exit else 'NONE'}",
                  self.reason]
        return "\n".join(lines)


def signature_of(measurement) -> str:
    """A behaviour fingerprint, for oscillation detection.

    ⚠️ A repair can land back on something behaviourally identical to a round
    it already tried. Without this the loop spends the rest of its budget
    rediscovering it. Built from the per-world outcome vector plus rounded fire
    and changed rates — what the node DOES, not what its source says.
    """
    parts = [f"{w.seed}:{w.outcome}" for w in measurement.worlds]
    parts.append(f"fire={measurement.fire:.2f}")
    parts.append(f"changed={measurement.changed:.2f}")
    return hashlib.sha256("|".join(parts).encode("utf-8")).hexdigest()[:12]


def render_repair(card, task, previous_source, diagnosis) -> str:
    """The repair prompt. ⚠️ ONE FILE FOR EVERY CONDITION, no condition
    parameter — the same rule `render_node` and `diagnose.render` follow.

    The standard node specification is SPLICED IN from
    `generate/prompts/modifier.md` rather than copied, so the contract has one
    definition and a change there reaches repairs automatically.
    """
    from generate.generator import render_node

    text = REPAIR_PROMPT.read_text(encoding="utf-8")
    for slot in ("{{PREVIOUS_SOURCE}}", "{{DIAGNOSIS}}", "{{NODE_PROMPT}}"):
        if slot not in text:
            raise RuntimeError(f"{REPAIR_PROMPT.name} has no {slot} slot")
    return (text
            .replace("{{PREVIOUS_SOURCE}}", previous_source.strip())
            .replace("{{DIAGNOSIS}}", diagnosis.strip())
            .replace("{{NODE_PROMPT}}", render_node(card, task)))


from repair.evidence import entry_for, lineage_root, round_dir  # noqa: E402,F401


# ---------------------------------------------------------------------------
# one round


# The growth tier's seeds. ⚠️ A REPAIRED NODE IS SCORED ON THE SAME WORLDS AS
# ITS PREDECESSOR, or the comparison between rounds means nothing.
GROWTH_SEEDS = range(2000, 2020)


def _measure(node_id, tier="maze", seeds=None, drift=None):
    """Score this node against the base on the growth tier.

    ⚠️ TWO PATHS, AND THE SECOND IS NOT OPTIONAL. `load_measurement` reads
    `<tier>-worlds.csv`, which only has rows for nodes `measure_growth.py` has
    already been run over. A REPAIRED node has none — the second live run of
    this loop died on exactly that, after a successful regeneration:

        KeyError: 'gen_b_…__r01 has no rows in …/maze-worlds.csv'

    So a repaired node is measured by actually rolling it out, on the same
    seeds, through the same pairing and the same cached base results.

    ⚠️ **AND THE COMMITTED CSV IS UNDRIFTED.** `core.worldgen.to_scenario()`
    never sets `drift`, so every row in `<tier>-worlds.csv` was measured with a
    PERFECT POSE ESTIMATE. When `drift` is requested, the CSV cannot answer the
    question and the short-circuit above must NOT be taken — for the original
    node either. Reading it would silently return the clean number and label it
    with the drifted axis, which is the exact mistake epic 06 exists because of:
    a node judged on a suite that does not contain the condition its card names.
    """
    from repair.verdict import load_measurement

    if "__r" not in node_id and drift is None:
        return load_measurement(node_id, tier=tier)

    return measure_live(entry_for(node_id), node_id,
                        seeds or GROWTH_SEEDS, tier=tier, drift=drift)


def _terminal_for(label):
    """Which verdicts END the loop, and which only route it.

    ⚠️ INERT IS NOT HERE. It routes to PROBE_ENVIRONMENT. See the module
    docstring: a loop that accepts INERT as an exit will make every node
    decline everywhere and call it success.
    """
    return {"HELPED": Exit.HELPED}.get(label)


def _write_probe_log(node_id, number, result):
    """The probe's whole search, beside the lineage it belongs to.

    Named for the ROUND, not the node: every round of a lineage reuses the
    parent's stem (`plan_hash` is a pure function of (condition, task, card)),
    so a file named after the node would be overwritten by the next probe.
    """
    path = WORK / lineage_root(node_id) / f"probe-round-{number:02d}.log"
    path.parent.mkdir(parents=True, exist_ok=True)
    head = (f"node: {node_id}\nround: {number}\n"
            f"found: {result.found.label if result.found else '(none)'}\n")
    path.write_text(f"{head}\n{result.log()}\n\n{result.note}\n",
                    encoding="utf-8")
    return path


def run_round(node_id, number, budget, tier="maze", backend=None,
              case_seed=None, seen=None, dry_run=False, drift=None):
    """Measure, classify, and — if it is not terminal — diagnose and act.

    Returns (Round, exit_or_None, next_node_id).
    """
    from repair.diagnose import Action, DiagnosisError, diagnose
    from repair.evidence import bundle as make_bundle, worst_case_seed
    from repair.verdict import claim_for, classify

    started = time.time()
    rec = Round(number=number, node_id=node_id, drift=drift)

    measurement = _measure(node_id, tier=tier, drift=drift)
    verdict = classify(measurement, claim_for(node_id))
    rec.verdict_label = verdict.label.value
    rec.verdict_reason = verdict.reason
    rec.signature = signature_of(measurement)

    # ⚠️ OSCILLATION. A repair whose behaviour matches an earlier round in this
    # lineage terminates it rather than spending the rest of the budget
    # rediscovering the same node.
    if seen is not None and rec.signature in seen:
        rec.outcome = "signature repeats an earlier round"
        rec.seconds = time.time() - started
        return rec, Exit.UNREPAIRED, node_id
    if seen is not None:
        seen.add(rec.signature)

    done = _terminal_for(verdict.label.value)
    if done is not None:
        rec.outcome = f"terminal: {verdict.reason}"
        rec.seconds = time.time() - started
        return rec, done, node_id

    if dry_run:
        rec.outcome = "dry run: stopped before the diagnosis call"
        rec.seconds = time.time() - started
        return rec, None, node_id

    seed = case_seed if case_seed is not None else worst_case_seed(measurement)
    b = make_bundle(node_id, tier=tier, case_seed=seed, measurement=measurement,
                    verdict=verdict)
    if b.leaks():
        rec.outcome = f"ABORTED: bundle leaks {b.leaks()}"
        rec.seconds = time.time() - started
        return rec, Exit.UNREPAIRED, node_id

    try:
        d = diagnose(b, backend=backend)
    except DiagnosisError as exc:
        # ⚠️ NEVER RETRIED. A rejection is an outcome with a denominator, the
        # same rule `generate/batch.py` applies.
        rec.outcome = f"diagnosis unusable: {exc}"
        rec.seconds = time.time() - started
        return rec, None, node_id

    rec.action = d.action.value
    rec.action_reason = d.action_reason
    rec.diagnosis_hash = d.prompt_hash

    if d.action is Action.ACCEPT:
        rec.outcome = "the model judges the outcome correct"
        rec.seconds = time.time() - started
        return rec, Exit.ACCEPT, node_id

    if d.action is Action.NOT_TRANSFERABLE:
        # ⚠️ REQUIRES AT LEAST ONE REAL REPAIR ATTEMPT FIRST, so it cannot be
        # reached by giving up on round one.
        if number <= 1:
            # ⚠️ The note must SURVIVE the regeneration below, which sets
            # `outcome` itself. An earlier version overwrote it, and the record
            # then showed a plain regeneration with no trace of the refusal —
            # caught by its own test.
            refused = ("NOT_TRANSFERABLE proposed on round 1 and REFUSED "
                       "(it requires a repair attempt first); ")
            rec.action = Action.REGENERATE_NODE.value
        else:
            rec.outcome = "faithful, repaired, and it does not carry over"
            rec.seconds = time.time() - started
            return rec, Exit.NOT_TRANSFERABLE, node_id

    if d.action is Action.PROBE_ENVIRONMENT:
        from repair.probe import probe
        result = probe(node_id, tier=tier, budget=budget)
        # ⚠️ THE SEARCH IS WRITTEN DOWN, WHICH IS WHAT `UNTESTABLE` MEANS.
        # REPAIR-LOOP.md defines the exit as "No reachable world discriminates
        # it. The search is recorded" — and the first real UNTESTABLE recorded
        # `result.note[:80]`, which truncated mid-word and threw away all six
        # trials. Which worlds were tried, which rung, and WHY each was passed
        # over is the entire evidential content of that exit: without it the
        # verdict is unfalsifiable, and this epic has already shipped two
        # plausible verdicts that were measurement artefacts.
        log_path = _write_probe_log(node_id, number, result)
        if result.found is None:
            rec.outcome = (f"no discriminating world found in "
                           f"{len(result.trials)} trial(s); search recorded in "
                           f"{log_path.name}")
            rec.seconds = time.time() - started
            return rec, Exit.UNTESTABLE, node_id
        rec.outcome = (f"discriminating world: {result.found.label}"
                       f"; search recorded in {log_path.name}")
        rec.seconds = time.time() - started
        return rec, None, node_id

    # REGENERATE_NODE ------------------------------------------------------
    refused = locals().get("refused", "")
    guidance = d.guidance or d.cause
    try:
        new_id, staged = regenerate(node_id, f"{d.cause}\n\n{guidance}", number,
                                    backend=backend)
    except Exception as exc:            # a failed repair is an OUTCOME
        rec.outcome = f"{refused}regeneration failed: {exc}"
        rec.seconds = time.time() - started
        return rec, None, node_id

    rec.outcome = f"{refused}regenerated -> {new_id} ({staged.name})"
    rec.seconds = time.time() - started
    return rec, None, new_id


# ---------------------------------------------------------------------------
# regeneration, and measuring something that has no CSV yet


def measure_live(entry, node_id, seeds, tier="maze", drift=None):
    """Score a node by ACTUALLY RUNNING IT, with no committed CSV to read.

    ⚠️ `repair.verdict.load_measurement` reads `<tier>-worlds.csv`, which only
    exists for nodes `measure_growth.py` has already been run over. A REPAIRED
    node has no rows, so the loop cannot classify it without rolling it out.

    Uses the same pairing and the same base-result cache as the probe, so a
    repaired node and its predecessor are scored identically.
    """
    from repair.evidence import aggregate_metrics
    from repair.probe import Candidate, base_score, _load_cache, _save_cache, _runner
    from repair.verdict import Measurement, WorldOutcome

    cache, runner = _load_cache(), _runner()
    dbg, build, base, _ = runner
    rows = []
    base_scores, pair_scores = {}, {}
    for seed in seeds:
        cand = Candidate(tier, int(seed), drift=drift)
        b_score, _ = base_score(cand, cache, runner)
        _save_cache(cache)
        base_reached = bool(b_score["goal_reached"])
        _, score, _, summary = dbg._rollout(
            build(entry.node_cls, entry.metadata, base), cand.scenario(), node_id)
        ticks = max(1, int(summary.get("ticks", 1)))
        pair_reached = bool(score["goal_reached"])
        outcome = ("fixed" if pair_reached and not base_reached else
                   "broke" if base_reached and not pair_reached else
                   "both_ok" if base_reached else "both_fail")
        rows.append(WorldOutcome(
            seed=int(seed), scenario_id=cand.label,
            base_reached=base_reached, pair_reached=pair_reached,
            outcome=outcome,
            fire=float(summary.get("precondition_passed", 0)) / ticks,
            changed=float(summary.get("drove", 0)) / ticks))
        base_scores[seed], pair_scores[seed] = b_score, dict(score)

    n = len(rows)
    # ⚠️ NOT `metrics={}`. It was, and that made three of the five verdicts
    # unreachable for anything the loop repaired — `pair_of(PRIMARY)` returned
    # `(None, None)`, so `primary_worse` was always False and only INERT and
    # ACCEPT could ever come back. `gen_b_…__r01` was called
    # "harmed nothing measurable" while losing four worlds the base wins.
    return Measurement(node_id=node_id, tier=tier, worlds=tuple(rows),
                       metrics=aggregate_metrics(base_scores, pair_scores),
                       fire=sum(r.fire for r in rows) / n,
                       changed=sum(r.changed for r in rows) / n)


def regenerate(node_id, diagnosis_text, number, backend=None, record=True):
    """Re-make the NODE call with the diagnosis attached. One model call.

    ⚠️ **THE CARD IS NOT REGENERATED** and is reused byte for byte. The card
    call is the independent variable, and moving it at the same time as the
    implementation would leave any difference attributable to neither. That is
    the same rule `scripts/regenerate_modifiers.py` follows.

    Returns (new_node_id, staged_dir) or raises.
    """
    from generate.admit import check_source
    from generate.backends import (ClaudeCliBackend, prompt_hash,
                                   write_transcript)
    from generate.cardgen import load_generated
    from generate.generator import STAGING, extract_code
    from repair.evidence import _stem_for, node_source

    stem = _stem_for(node_id)
    if stem is None:
        raise RuntimeError(f"no staged source for {node_id}")
    arm = stem.split("__")[0]

    # ⚠️ RESUME FROM WHAT IS ON DISK, NEVER FROM A PROGRESS FILE — the rule
    # `sweep.py` follows from `results/` and `batch.py` from `staging/`. A
    # killed run must not pay for this round's model call twice.
    #
    # ⚠️ And the sweep rehearsal's own worst bug applies here: five episodes
    # exited 0, wrote nothing, and were recorded as `ok`. So the marker is the
    # RESULT FILE, which `_stage` writes LAST — a round dir holding a half
    # written `.py` and no `.result.json` is an interrupted call, not a
    # finished one, and is redone.
    root = lineage_root(node_id)
    done = round_dir(root, number)
    if (done / f"{stem}.result.json").exists():
        return f"{root}__r{number:02d}", done

    # ⚠️ THE TASK COMES FROM THE COMMITTED METADATA, NOT FROM THE STEM. The
    # stem is `<arm>__<CARD id>__<plan_hash>` — its middle segment is the card,
    # not the task. The first live run of this loop died on exactly that:
    # `no task 'path_integration_ants'. Known: ['navigate_to_goal', ...]`.
    # Two different names that both look like a task is a trap the filename
    # invites, so it is read from the one place that states it.
    meta_parent = json.loads(
        (Path(STAGING) / f"{stem}.json").read_text(encoding="utf-8"))
    task = meta_parent["task"]

    card_path = REPO_ROOT / "generate" / "cards" / arm / f"{stem}.yaml"
    card = load_generated(card_path).card

    prompt = render_repair(card, task, node_source(node_id), diagnosis_text)
    backend = backend or ClaudeCliBackend()
    response = backend.complete(prompt)

    if record:
        write_transcript(prompt, response,
                         {"step": "repair", "node_id": node_id,
                          "round": number, "task": task},
                         directory=Path(__file__).resolve().parent / "transcripts")

    source = extract_code(response.text)
    if not source:
        raise RuntimeError("the repair call returned no fenced python block")
    check_source(source)                      # the AST allowlist still applies

    # ⚠️ THE ROUND IS NAMED OFF THE LINEAGE ROOT, NEVER OFF ITS PARENT. Round 2
    # is handed round 1's id, so `f"{node_id}__r{number:02d}"` compounds into
    # `..._r01__r02` and round 3 dies in `entry_for` on
    # `int('01__r02')` — which is exactly how the first full 3-round run
    # ended, after four model calls. `repair_round` already carries the depth;
    # the id carries the lineage, and a lineage has one root. `root` and `out`
    # are the same two values the resume check above computed.
    out = done
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{stem}.py").write_text(source, encoding="utf-8")

    new_id = f"{root}__r{number:02d}"

    # ⚠️ INHERIT THE PARENT'S METADATA AND OVERRIDE ONLY WHAT CHANGED.
    # A hand-built dict loses provenance fields the library HARD-REQUIRES, and
    # loses them one at a time: the third live run of this loop died on
    #
    #   PROVENANCE_INCOMPLETE: temperature was not recorded
    #
    # after a successful regeneration AND a successful re-measurement. Copying
    # the parent keeps `temperature`, `sampling_note`, `card_id`, `ref_id`,
    # `treatment`, `paper_sha256` and `origin` correct by construction — and
    # they ARE still correct: a repair re-makes the node call, and the card call
    # that produced all of them is untouched.
    meta = dict(meta_parent)
    meta.update({
        "node_id": new_id,
        "parent_node_id": node_id,
        "repair_round": number,
        "emitted_code": True,
        "prompt_hash": prompt_hash(prompt),
        "model_id": response.model_id,
        "backend": response.backend,
        "pipeline_version": "v3-layered-repaired",
    })
    (out / f"{stem}.json").write_text(json.dumps(meta, indent=2, sort_keys=True),
                                      encoding="utf-8")
    (out / f"{stem}.result.json").write_text(
        json.dumps({"admitted": True, "node_id": new_id, "reason": "admitted",
                    "detail": "validity only; contribution is measured on the "
                              "shared suite, which is chunk 44's decision",
                    "metadata": meta}, indent=2, sort_keys=True),
        encoding="utf-8")
    return new_id, out


def load_round(node_id, number):
    """The repaired node from one round. Delegates to the shared resolver."""
    return entry_for(f"{node_id}__r{number:02d}")


# ---------------------------------------------------------------------------
# the loop


def first_round_number(node_id):
    """Where numbering CONTINUES for this node. 1, or one past its own round.

    ⚠️ A lineage can be re-entered part-way — `run("…__r01")` means "carry on
    from the node round 1 produced", not "start a second round 1". Without this
    `regenerate` would name the next node `…__r01` again and stage it over
    `round-01`, destroying the only valid round of the run this continues.
    """
    root = lineage_root(node_id)
    if node_id == root:
        return 1
    suffix = node_id[len(root):].lstrip("_").lstrip("r")
    if not suffix.isdigit():
        raise ValueError(
            f"{node_id!r} is not a repair-round id: expected one '__rNN' "
            f"suffix off {root!r}")
    return int(suffix) + 1


def recorded_drift(node_id):
    """The axis a lineage's existing `loop.json` was run on, and whether it has
    one at all. Returns (found, drift)."""
    path = WORK / lineage_root(node_id) / "loop.json"
    if not path.exists():
        return False, None
    data = json.loads(path.read_text(encoding="utf-8"))
    return True, data.get("drift")


def run(node_id, rounds=DEFAULT_ROUNDS, tier="maze", backend=None,
        probe_budget=6, dry_run=False, drift=None) -> LoopResult:
    """One node, to a terminal verdict or the end of its budget.

    ⚠️ Resumes from what is on disk, never from a progress file — the rule
    `sweep.py` and `batch.py` both follow. And the rehearsal's own worst bug
    applies: exit 0 is necessary, the RESULT FILE is sufficient.

    `drift` puts the hazard the card may predicate on ONTO the repair set. ⚠️ It
    changes what the lineage is an experiment about, so it is recorded per round
    and a lineage may not silently mix axes — see the guard below.
    """
    started = time.time()

    # ⚠️ ONE LINEAGE, ONE AXIS. A run repaired against drifted worlds and one
    # repaired against clean worlds are different experiments, and `round_dir`
    # keys off the node id alone — so without this they would interleave in one
    # directory under one `loop.json` and nothing would say which round saw
    # which hazard. Same argument `Candidate.label` makes by renaming a drifted
    # world, and chunk 52 makes by moving the v2 records aside. Deliberately
    # changing the axis means archiving the previous run first, which is what
    # `out/repair/archive/` is for.
    found, previous = recorded_drift(node_id)
    if found and previous != drift:
        raise ValueError(
            f"{lineage_root(node_id)}'s loop.json records drift={previous!r} "
            f"and this run asks for drift={drift!r}. A lineage may not mix "
            f"axes: archive the existing run under out/repair/archive/ with a "
            f"README, or re-run it on drift={previous!r}.")

    start = first_round_number(node_id)
    result = LoopResult(node_id=node_id, exit=None, reason="", drift=drift,
                        continues_from=node_id if start > 1 else None)
    seen, current = set(), node_id

    for number in range(start, start + rounds):
        rec, done, nxt = run_round(current, number, probe_budget, tier=tier,
                                   backend=backend, seen=seen, dry_run=dry_run,
                                   drift=drift)
        result.rounds.append(rec)
        # ⚠️ after EVERY round, not at the end — but NEVER on a dry run.
        #
        # `--dry-run` is documented as "the plan, calls nothing", and a reader
        # reasonably takes that to mean it changes nothing either. It did not:
        # `_write` clobbered `loop.json`, so running the advertised dry-run
        # command DESTROYED the completed run's record. Found 2026-09-17 by
        # doing exactly that, immediately after the run it destroyed.
        if not dry_run:
            _write(result)
        if done is not None:
            result.exit = done
            result.reason = rec.outcome
            break
        if dry_run:
            result.reason = "dry run: stopped before the first model call"
            break
        # A regenerated node must be RE-MEASURED before the next round can
        # classify it — it has no committed CSV rows.
        if nxt != current:
            current = nxt
        result.node_id = current

    if result.exit is None and not dry_run:
        result.exit = Exit.UNREPAIRED
        result.reason = f"budget of {rounds} round(s) spent"

    result.seconds = time.time() - started
    if not dry_run:
        _write(result)
    return result


def _write(result):
    """Every round on disk, incrementally."""
    out = WORK / lineage_root(result.node_id)
    out.mkdir(parents=True, exist_ok=True)
    (out / "loop.json").write_text(json.dumps({
        "node_id": result.node_id,
        "continues_from": result.continues_from,
        "drift": result.drift,
        "exit": result.exit.value if result.exit else None,
        "reason": result.reason,
        "seconds": round(result.seconds, 1),
        "rounds": [r.as_dict() for r in result.rounds],
    }, indent=2, sort_keys=True) + "\n", encoding="utf-8")
