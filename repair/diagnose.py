"""repair/diagnose.py — one model call: WHY did this node behave like that?

⚠️ **IT PROPOSES. IT DOES NOT DECIDE.** The verdict came from
`repair/verdict.py` — a rule over measurements — before this call was made, and
the loop, not the model, chooses whether to act on the proposal. The model's job
is the part a rule cannot do: reading a card, a node, a trajectory and a set of
outcomes *together* and saying why.

⚠️ **ONE PROMPT FOR EVERY CONDITION.** `render(bundle)` has no condition
parameter and a test pins that, exactly as `generator.render_node(card, task)`
does. The bundle it is fed is blind by construction (chunk 55).

⚠️ **`--setting-sources ''` IS LOAD-BEARING HERE, NOT HYGIENE.** A diagnosing
agent that could read this repo's CLAUDE.md would know which arm every node
belongs to, that D is frozen, and that B−A is the headline. `ClaudeCliBackend`
already runs from a scratch directory with settings disabled; a test pins it.

Usage:

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P -m repair.diagnose --node gen_b_44d12df4d25b437f --dry-run   # prompt only
    $P -m repair.diagnose --node gen_b_44d12df4d25b437f --replay    # committed transcript
    $P -m repair.diagnose --node gen_b_44d12df4d25b437f             # ⚠️ REAL CALL
"""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

REPO_ROOT = Path(__file__).resolve().parent.parent


# ---------------------------------------------------------------------------
# Everything below this line is data, not code.
# ---------------------------------------------------------------------------

PROMPT = Path(__file__).resolve().parent / "prompts" / "diagnose.md"
TRANSCRIPTS = Path(__file__).resolve().parent / "transcripts"


class Action(str, Enum):
    """⚠️ A CLOSED VOCABULARY. The same rule `compose/compose.py` applies to a
    plan whose vocabulary is the retrieved node ids and nothing else: a closed
    vocabulary is what makes the output checkable. A free-text action is
    REFUSED and recorded as a failure, never silently coerced into the nearest
    match."""

    REGENERATE_NODE = "REGENERATE_NODE"
    PROBE_ENVIRONMENT = "PROBE_ENVIRONMENT"
    ACCEPT = "ACCEPT"
    NOT_TRANSFERABLE = "NOT_TRANSFERABLE"


# ⚠️ `REGENERATE_CARD` IS DELIBERATELY ABSENT (user's instruction, 2026-09-16).
# A model that can name an action will name it, and there is nothing behind it:
# a changed card is a new `plan_hash` and therefore a new row, never an edit.
# When it lands it comes with its own chunk. See REPAIR-LOOP.md.
DEFERRED_ACTIONS = ("REGENERATE_CARD", "REVISE_PROMPT", "REVISE_SHARED_PROMPT")

# ⚠️ "unstated" IS IN THE VOCABULARY, and the first live call is why. The model
# returned a complete, correct, four-departure diagnosis and omitted exactly one
# field — `confidence` — and the whole thing was thrown away over it.
#
# The line that matters: **a field the loop ACTS ON is required; a field that is
# only REPORTED is optional.** `action`, `cause` and `action_reason` drive what
# happens next. `confidence` is metadata for a human reading the log. Discarding
# a paid-for, correct diagnosis over metadata is not strictness, it is waste —
# and because a rejection is never retried, it is unrecoverable waste.
CONFIDENCE = ("high", "medium", "low", "unstated")

# The default system prompt ends "Return the code in a single fenced python
# block", which on a diagnosis call is an instruction to do the wrong thing.
# ⚠️ A per-BACKEND setting, never a per-CONDITION one — varying it between arms
# would be a second place for the ablation to differ, invisible in a diff.
SYSTEM_PROMPT = (
    "You are reviewing generated code for a robotics research codebase against "
    "the specification it was generated from. Be concrete and mechanical. "
    "Return a single fenced json block and nothing else."
)

# Only what the loop acts on. See CONFIDENCE above.
REQUIRED_FIELDS = ("cause", "action", "action_reason")


class DiagnosisError(RuntimeError):
    """The model returned something that is not a diagnosis. A RESULT, not a
    crash: the loop records it and moves on, the same way `generate/admit.py`
    records a rejection rather than retrying it."""


# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Diagnosis:
    """What the model proposed. Not a verdict, and not a decision."""

    handle: str
    action: Action
    cause: str
    action_reason: str
    confidence: str
    card_departures: tuple = ()
    evidence: tuple = ()
    guidance: str = ""
    prompt_hash: str = ""
    model_id: str = ""
    raw: dict = field(default_factory=dict)

    def __str__(self):
        out = [f"{self.handle}  ->  {self.action.value}  ({self.confidence})",
               "", f"cause: {self.cause}", "",
               f"why this action: {self.action_reason}"]
        if self.card_departures:
            out += ["", "departures from its own card:"]
            out += [f"  - {d}" for d in self.card_departures]
        if self.evidence:
            out += ["", "evidence:"]
            out += [f"  - {e}" for e in self.evidence]
        if self.guidance:
            out += ["", f"guidance for the replacement: {self.guidance}"]
        return "\n".join(out)


def render(bundle) -> str:
    """The exact diagnosis prompt. Hashed, recorded, replayable.

    ⚠️ ONE FILE FOR EVERY CONDITION, and there is no `condition` parameter.
    The moment this function can tell the arms apart, the repair loop acquires
    a second place to differ.
    """
    text = PROMPT.read_text(encoding="utf-8")
    if "{{BUNDLE}}" not in text:
        raise DiagnosisError(
            f"{PROMPT.name} has no {{{{BUNDLE}}}} slot. The diagnosis call is "
            f"fed a blind bundle and nothing else; a prompt with no slot for "
            f"one would silently diagnose from the framing alone.")
    return text.replace("{{BUNDLE}}", bundle.render())


def parse(text, handle="") -> Diagnosis:
    """Model output -> a Diagnosis, or `DiagnosisError` saying what was wrong.

    ⚠️ Strict on purpose. Coercing a near-miss action into the nearest valid
    one would let the model invent actions the loop cannot perform, and the
    loop would then act on a guess about what it meant.
    """
    block = re.search(r"```json\s*(.*?)```", text, re.S)
    payload = block.group(1) if block else text
    try:
        data = json.loads(payload.strip())
    except ValueError as exc:
        raise DiagnosisError(f"response is not JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise DiagnosisError(f"response is {type(data).__name__}, not an object")

    missing = [f for f in REQUIRED_FIELDS if not str(data.get(f, "")).strip()]
    if missing:
        raise DiagnosisError(f"diagnosis is missing {missing}")

    raw_action = str(data["action"]).strip().upper()
    if raw_action in DEFERRED_ACTIONS:
        raise DiagnosisError(
            f"{raw_action} is a DEFERRED action with nothing behind it — the "
            f"prompt offers four and this is not one of them")
    try:
        action = Action(raw_action)
    except ValueError as exc:
        raise DiagnosisError(
            f"{raw_action!r} is not one of {[a.value for a in Action]}") from exc

    # Absent is fine and becomes "unstated"; PRESENT AND WRONG is still an
    # error. A model that writes confidence: "pretty sure" has not understood
    # the schema, and that is worth catching — unlike simply omitting it.
    raw_confidence = data.get("confidence")
    if raw_confidence is None or not str(raw_confidence).strip():
        confidence = "unstated"
    else:
        confidence = str(raw_confidence).strip().lower()
        if confidence not in CONFIDENCE:
            raise DiagnosisError(
                f"confidence {confidence!r} is not in {CONFIDENCE}")

    def listed(key):
        value = data.get(key) or []
        if isinstance(value, str):
            value = [value]
        return tuple(str(v).strip() for v in value if str(v).strip())

    return Diagnosis(
        handle=handle,
        action=action,
        cause=str(data["cause"]).strip(),
        action_reason=str(data["action_reason"]).strip(),
        confidence=confidence,
        card_departures=listed("card_departures"),
        evidence=listed("evidence"),
        guidance=str(data.get("guidance", "") or "").strip(),
        raw=data,
    )


def diagnose(bundle, backend=None, record=True) -> Diagnosis:
    """One model call. Blind bundle in, a structured proposal out.

    The transcript is written under `repair/transcripts/`, keyed by the prompt
    hash, exactly as generation's are — which is what replaces temperature-0
    determinism here too.
    """
    from generate.backends import (BackendError, ClaudeCliBackend,
                                   ReplayBackend, prompt_hash,
                                   write_transcript)

    prompt = render(bundle)
    digest = prompt_hash(prompt)

    # ⚠️ RESUME FROM WHAT IS ON DISK. A killed run must not pay for the same
    # diagnosis twice, and the diagnosis prompt is a pure function of the
    # bundle, so an existing transcript for THIS hash is this exact call. The
    # loop's other model call (`loop.regenerate`) resumes from its staged round
    # for the same reason; this is the other half of that Done-when.
    #
    # ⚠️ It also stops a re-run OVERWRITING the recorded diagnosis. Transcripts
    # are keyed by prompt hash and `write_transcript` replaces, so a second
    # live run on an unchanged node silently destroys the first one's wording —
    # which already happened to the transcript the epic's success criterion is
    # asserted against, on 2026-09-17.
    #
    # An explicit `backend=` always wins: a caller naming its backend means it.
    resumed = False
    if backend is None:
        replay = ReplayBackend(directory=TRANSCRIPTS)
        if replay.path_for(prompt).exists():
            backend, resumed = replay, True

    backend = backend or ClaudeCliBackend(system_prompt=SYSTEM_PROMPT)

    # ⚠️ A REPLAYED CALL DOES NOT REWRITE ITS OWN TRANSCRIPT. Writing it back
    # changes the mtime and nothing else, so the file then claims a call was
    # made at a time when none was — and mtime is how a reader reconstructs
    # what a run actually spent. Observed: this transcript's mtime moved three
    # times on 2026-09-17 while its `session_id` and cost never changed.
    record = record and not resumed

    try:
        response = backend.complete(prompt)
    except BackendError as exc:
        # ⚠️ A CALL THAT COST MONEY AND RETURNED NOTHING IS STILL EVIDENCE.
        # Found 2026-09-16: the third node of the generalisation check spent
        # $0.30 and 6,016 THINKING tokens out of a 6,120-token output budget,
        # emitted almost no text, and raised here — and because only
        # DiagnosisError was recorded, the whole call vanished. Same family as
        # the sweep rehearsal's "exited 0, wrote nothing", and the same fix
        # `generator.write_call_failure` already applies to generation: a call
        # nobody can see is indistinguishable from a call nobody made.
        #
        # Filed as `<hash>.error.json`, which `ReplayBackend` does not read —
        # it looks for `<hash>.json` — so a failure can never be replayed as
        # though it were a diagnosis.
        if record:
            write_call_failure(digest, exc, bundle.handle, prompt)
        raise
    try:
        result = parse(response.text, handle=bundle.handle)
    except DiagnosisError:
        if record:
            write_transcript(prompt, response,
                             {"step": "diagnose", "handle": bundle.handle,
                              "parsed": False},
                             directory=TRANSCRIPTS)
        raise

    result = Diagnosis(**{**result.__dict__,
                          "prompt_hash": digest,
                          "model_id": response.model_id})
    if record:
        write_transcript(prompt, response,
                         {"step": "diagnose", "handle": bundle.handle,
                          "parsed": True, "action": result.action.value},
                         directory=TRANSCRIPTS)
    return result


def write_call_failure(digest, exc, handle, prompt, directory=None):
    """Record a diagnosis call that produced nothing usable.

    Mirrors `generate.generator.write_call_failure`, deliberately: the payload
    the CLI returned is kept verbatim, because on a runaway the token counts
    ARE the finding — 6,016 thinking tokens against 104 of output says the
    model reasoned itself out of an answer, which is a different problem from a
    timeout or a refusal and wants a different response.
    """
    import json as _json

    directory = Path(directory or TRANSCRIPTS)
    directory.mkdir(parents=True, exist_ok=True)
    payload = getattr(exc, "payload", {}) or {}
    record = {
        "prompt_hash": digest,
        "handle": handle,
        "step": "diagnose",
        "parsed": False,
        "error": str(exc),
        "payload": payload,
        "usage": payload.get("usage") or {},
        "prompt_chars": len(prompt),
    }
    path = directory / f"{digest}.error.json"
    path.write_text(_json.dumps(record, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
    return path


def _main(argv=None):
    import argparse

    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--node", required=True)
    p.add_argument("--tier", default="maze")
    p.add_argument("--case", type=int, default=None,
                   help="seed of one world to open up in the bundle")
    p.add_argument("--auto-case", action="store_true")
    p.add_argument("--dry-run", action="store_true",
                   help="print the prompt and its hash; call nothing")
    p.add_argument("--replay", action="store_true",
                   help="use the committed transcript instead of a model")
    a = p.parse_args(argv)

    from repair.evidence import bundle as make_bundle, worst_case_seed
    from repair.verdict import load_measurement

    m = load_measurement(a.node, tier=a.tier)
    seed = a.case
    if a.auto_case and seed is None:
        seed = worst_case_seed(m)
    b = make_bundle(a.node, tier=a.tier, case_seed=seed, measurement=m)

    if a.dry_run:
        from generate.backends import prompt_hash
        text = render(b)
        print(f"[diagnose] handle      {b.handle}")
        print(f"[diagnose] prompt_hash {prompt_hash(text)}")
        print(f"[diagnose] prompt      {len(text)} chars, ~{len(text) // 4} tokens")
        print(f"[diagnose] leaks       {b.leaks() or 'NONE'}")
        print("[diagnose] called nothing")
        return 0

    backend = None
    if a.replay:
        from generate.backends import ReplayBackend
        backend = ReplayBackend(directory=TRANSCRIPTS)

    try:
        print(diagnose(b, backend=backend))
    except DiagnosisError as exc:
        print(f"[diagnose] FAILED: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
