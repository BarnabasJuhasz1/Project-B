"""
generate/probe.py

**Chunk 33 — the citations probe. Does human extraction earn its place?**

Condition B hands the model a card: a mechanism, a trigger, a derivation and a
structured prediction, extracted from a paper by someone who read it. That step
is expensive, and nothing in the pipeline has ever tested whether it buys
anything the model could not have recalled unaided.

This module asks. It renders a prompt carrying **a citation and nothing else**,
takes back a card in the frozen six-field schema, and diffs it field by field
against the human-extracted card for the same reference. `cards/VERIFICATION.md`
says which spans a human has actually checked, so four of the five citations
come with an answer key.

    B - D  =  everything the corpus buys        (pre-registered; needs episodes)
    E - D  =  what POINTING buys                (direction alone)
    B - E  =  what EXTRACTION buys              (the same paper, read or not)

**This is not a fifth condition and must not become one.** `runs/spec.py` refuses
a fifth by design — #10's identical-episode-count argument is stated over exactly
four — and adding one costs an amendment to a tagged pre-registration plus a full
extra sweep. So `CONDITIONS` is untouched, this prompt is not
`condition_e.md`, and the probe **stops at the card**: nothing here generates a
node, admits one, or reaches a library. The question it answers is
reconstruction fidelity, and that needs no episodes at all.

WHY THE SPAN IS THE HEADLINE
----------------------------
The project's third claim for a corpus is provenance: *"a model states a
threshold or an exponent with equal confidence whether or not it is
confabulating."* That is measurable here and nowhere else in the pipeline. A
reconstructed `span` that names a section which does not exist, or a `prediction`
whose numbers do not appear in the paper, is the confabulation the provenance
axis was invented to detect — caught against a verified span rather than against
another model's opinion.

⚠️ **A reconstructed card is never a real card.** It lives in `cards/probe/`,
which `cards.store` does not read, and a test proves the store cannot load one.
A probe card reaching condition B's retrieval would corrupt the headline
comparison silently.

SIM-FREE. No simulator, no GPU, no library, no episodes.
"""

import difflib
import re
from pathlib import Path

from generate import backends

PROMPT = Path(__file__).resolve().parent / "prompts" / "probe_card.md"
PROBE_DIR = Path(__file__).resolve().parent.parent / "cards" / "probe"

CITATION_BLOCK = re.compile(
    r"<!-- ---- CITATION BLOCK BEGIN ---- -->.*?"
    r"<!-- ---- CITATION BLOCK END ---- -->",
    re.DOTALL)
YAML_FENCE = re.compile(r"```yaml\n(.*?)```", re.DOTALL)

# The fields a reconstruction is scored on, and how. Below this line is data.
# ---------------------------------------------------------------------------
# Prose fields: compared by similarity, which is a weak signal and is reported
# as such — two correct statements of one mechanism can share few words.
PROSE_FIELDS = ("mechanism", "trigger", "derivation")
# Mechanical fields: right or wrong, no rubric needed. This is where the probe
# actually decides something.
EXACT_FIELDS = ("prediction.metric", "prediction.baseline",
                "prediction.cost_metric", "confidence")
NUMERIC_FIELDS = ("prediction.min_effect",)
# ---------------------------------------------------------------------------


class ProbeError(ValueError):
    pass


def render(citation):
    """The exact text sent to the model. Hashed, recorded and replayable, the
    same way `generator.render` is."""
    if not citation or not citation.strip():
        raise ProbeError("a probe needs a citation")
    text = PROMPT.read_text(encoding="utf-8")
    if "{{CITATION}}" not in text:
        raise ProbeError(f"{PROMPT.name} has no {{{{CITATION}}}} slot")
    return text.replace("{{CITATION}}", citation.strip())


def leaked(prompt, card):
    """Everything from the card that reached the prompt but should not have.

    The probe's whole claim is that the model was given a citation and nothing
    else. If the mechanism, the derivation or the span leaked in, E is not
    measuring recall — it is measuring transcription, and the result would be
    silently meaningless. Checked rather than asserted, on every call.
    """
    found = []
    body = prompt.lower()
    for name, value in (("mechanism", card.mechanism),
                        ("trigger", card.trigger),
                        ("derivation", card.derivation),
                        ("span", card.source.span),
                        ("prediction.statement", card.prediction.statement)):
        value = (value or "").strip()
        if not value:
            continue
        # A distinctive run of the field, not the whole thing: the model's own
        # output is not in the prompt, so any substantial overlap is a leak.
        probe = " ".join(value.split()[:8]).lower()
        if len(probe) > 20 and probe in body:
            found.append(name)
    return found


def extract_card(response_text):
    """The single fenced yaml block, parsed with the REAL loader.

    Parsed by `cards.store.card_from_dict`, so a reconstruction that would not
    load as a card is a failure of the reconstruction rather than something this
    module papers over. **No block is a result, not an error** — the prompt
    explicitly invites "I do not know this document", and that answer belongs in
    the denominator, the same way `generator.extract_code` treats a refusal.
    """
    blocks = YAML_FENCE.findall(response_text)
    if not blocks:
        return None
    if len(blocks) > 1:
        raise ProbeError(
            f"the response contains {len(blocks)} fenced yaml blocks and the "
            f"prompt asked for one; picking one would be the harness choosing "
            f"the answer")
    import yaml
    from cards.store import card_from_dict
    data = yaml.safe_load(blocks[0])
    if not isinstance(data, dict):
        raise ProbeError(f"the yaml block is a {type(data).__name__}, not a card")
    return card_from_dict(data, where="<probe reconstruction>")


def _get(card, dotted):
    obj = card
    for part in dotted.split("."):
        obj = getattr(obj, part)
    return obj.value if hasattr(obj, "value") else obj


def similarity(a, b):
    """Token-overlap ratio in [0, 1]. A WEAK signal, and reported as one: two
    correct statements of one mechanism can share very few words, so this ranks
    reconstructions, it does not score them. The exact fields decide."""
    a_words = re.findall(r"[a-z0-9]+", (a or "").lower())
    b_words = re.findall(r"[a-z0-9]+", (b or "").lower())
    if not a_words or not b_words:
        return 0.0
    return difflib.SequenceMatcher(None, a_words, b_words).ratio()


def diff(human, reconstructed):
    """One card against its reconstruction, field by field.

    The result is data, not a verdict. `span_checkable` is the field that
    matters most and it is deliberately NOT scored here: whether a reconstructed
    span is right is a question about a PDF, and this module has no PDF. It
    reports whether there is an answer key and what the two say, and a human
    closes it in `cards/probe/FINDINGS.md`.
    """
    if reconstructed is None:
        return {
            "card_id": human.id,
            "citation": human.source.citation,
            "reconstructed": False,
            # Deliberately does NOT say why. `diff()` is handed None and cannot
            # tell a refusal from a card that would not load, and those are
            # different results: one is the honest answer the prompt invites,
            # the other is a confident reconstruction failing a rule. `probe()`
            # knows which and overwrites this.
            "note": "no card was parsed from the response",
        }

    out = {
        "card_id": human.id,
        "citation": human.source.citation,
        "reconstructed": True,
        "node_name": {"human": human.node_name,
                      "model": reconstructed.node_name},
        "prose": {f: {"similarity": round(similarity(getattr(human, f),
                                                     getattr(reconstructed, f)), 3),
                      "human": getattr(human, f).strip(),
                      "model": getattr(reconstructed, f).strip()}
                  for f in PROSE_FIELDS},
        "exact": {},
        "numeric": {},
    }
    for field in EXACT_FIELDS:
        h, m = _get(human, field), _get(reconstructed, field)
        out["exact"][field] = {"human": h, "model": m, "match": h == m}
    for field in NUMERIC_FIELDS:
        h, m = _get(human, field), _get(reconstructed, field)
        out["numeric"][field] = {"human": h, "model": m,
                                 "match": abs(float(h) - float(m)) < 1e-9}

    # The provenance question, stated rather than answered. `is_traceable` means
    # a human has opened the paper, so it is the only thing here that can serve
    # as an answer key — and limit_speed_ics deliberately has none.
    out["span"] = {
        "human": human.source.span.strip(),
        "model": reconstructed.source.span.strip(),
        "has_answer_key": human.is_traceable,
        "model_offered_one": bool(reconstructed.source.span.strip()),
        # An invented span where there IS a key is the confabulation the
        # provenance axis exists to detect; where there is NO key it is
        # unfalsifiable and must not be counted either way.
        "checkable": human.is_traceable and bool(
            reconstructed.source.span.strip()),
    }
    out["exact_match_count"] = sum(
        v["match"] for v in out["exact"].values()) + sum(
        v["match"] for v in out["numeric"].values())
    out["exact_field_count"] = len(EXACT_FIELDS) + len(NUMERIC_FIELDS)
    return out


def probe(card, backend, record=True):
    """One citation -> one reconstruction -> one diff. Never raises for the
    model's own faults: a refusal is an outcome."""
    prompt = render(card.source.citation)
    escaped = leaked(prompt, card)
    if escaped:
        raise ProbeError(
            f"{card.id}: {escaped} from the human card reached the probe "
            f"prompt. E must see the citation and nothing else, or it measures "
            f"transcription rather than recall.")

    response = backend.complete(prompt)
    if record:
        backends.write_transcript(prompt, response,
                                  {"kind": "probe", "card_id": card.id})
    try:
        reconstructed = extract_card(response.text)
        error = None
    except (ProbeError, ValueError) as exc:
        reconstructed, error = None, f"{type(exc).__name__}: {exc}"

    result = diff(card, reconstructed)
    result["prompt_hash"] = backends.prompt_hash(prompt)
    result["model_id"] = getattr(response, "model_id", None)
    if error:
        result["error"] = error
    if reconstructed is None:
        # The two ways to produce no card are not one outcome, and collapsing
        # them loses the more interesting half. `extract_card` returns None only
        # when the response carries no yaml block at all — the "I do not know
        # this document" the prompt calls correct. Anything else RAISED, which
        # means the model did return a card and the REAL loader refused it.
        # That is a reconstruction failure with a reason, and the reason is the
        # result: both cards rejected on the first run named a prediction metric
        # no controller can move.
        result["outcome"] = "rejected" if error else "refused"
        result["note"] = (
            f"the model returned a card and the loader refused it — {error}"
            if error else
            "the model declined to reconstruct — no yaml block in the response")
    return result, reconstructed


# --- persistence -----------------------------------------------------------
#
# `cards/probe/` holds three files per citation: the reconstruction as YAML (so
# it is readable as a card even though nothing may load it as one), the diff as
# JSON (so FINDINGS.md is regenerated rather than hand-maintained), and the
# written result. Committed, because a probe with no record is an anecdote.

def write_result(result, reconstructed, directory=None):
    """One reconstruction and its diff, keyed by the card it reconstructs."""
    import json
    directory = Path(directory or PROBE_DIR)
    directory.mkdir(parents=True, exist_ok=True)
    card_id = result["card_id"]
    (directory / f"{card_id}.diff.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if reconstructed is not None:
        (directory / f"{card_id}.yaml").write_text(
            _as_yaml(reconstructed, card_id), encoding="utf-8")
    return directory / f"{card_id}.diff.json"


def _as_yaml(card, reconstructs):
    """The reconstruction, written back out as YAML.

    The header is not decoration. These files sit one directory away from real
    cards and are the same shape, so anything that reads one has to be told what
    it is holding before it reads a field.
    """
    import yaml
    data = {
        "id": card.id,
        "node_name": card.node_name,
        "source": {"citation": card.source.citation,
                   "span": card.source.span,
                   # NEVER carried over from the reconstruction: `verified`
                   # means a human opened the paper, and nobody has.
                   "verified": "unverified"},
        "mechanism": card.mechanism,
        "trigger": card.trigger,
        "derivation": card.derivation,
        "prediction": {"statement": card.prediction.statement,
                       "metric": card.prediction.metric,
                       "baseline": card.prediction.baseline,
                       "min_effect": card.prediction.min_effect,
                       "cost_metric": card.prediction.cost_metric},
        "confidence": card.confidence.value,
    }
    header = (
        "# ⚠️ MODEL RECONSTRUCTION — NOT A PRINCIPLE CARD.\n"
        "# Produced from the citation alone by generate/probe.py. Nobody has\n"
        "# opened this paper to write this file. The span is a GUESS.\n"
        f"# Reconstructs: cards/{reconstructs}.yaml\n"
        "# cards.store refuses to load this directory. See cards/probe/README.md.\n")
    return header + yaml.safe_dump(data, sort_keys=False, allow_unicode=True)


def load_results(directory=None):
    """Every diff on disk, newest schema wins. Sorted, so FINDINGS is stable."""
    import json
    directory = Path(directory or PROBE_DIR)
    out = []
    for path in sorted(directory.glob("*.diff.json")):
        try:
            out.append(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, ValueError):
            continue
    return out


def _rejected(result):
    """Did the model return a card the loader refused, as opposed to no card?

    Derived from `error` rather than read from a key, because `error` is the
    signal that was always there: `extract_card` returns None only for a
    response with no yaml block, and raises for everything else. That makes this
    correct for diffs written before `outcome` existed, which are on disk.
    """
    return not result.get("reconstructed") and bool(result.get("error"))


def _headline(results, got):
    """The one-line count, split the way the outcomes actually split.

    "N produced a card" read as though the rest were refusals. On the first real
    run both non-producers had in fact returned a card that the loader refused,
    and the summary hid it — so the headline names all three outcomes or none.
    """
    rejected = sum(1 for r in results if _rejected(r))
    refused = len(results) - len(got) - rejected
    parts = [f"{len(got)} produced a loadable card"]
    if rejected:
        parts.append(f"{rejected} returned one the loader refused")
    if refused:
        parts.append(f"{refused} declined to reconstruct")
    return f"{len(results)} citation(s) probed: " + ", ".join(parts) + "."


def write_findings(results, directory=None):
    """FINDINGS.md, generated from the diffs rather than maintained by hand.

    Deliberately states what it CANNOT conclude. The prose similarities are a
    weak signal, and a reconstructed span is unresolved until a human opens the
    paper — writing a verdict here that nobody checked would be the same
    confabulation the probe exists to detect, one level up.
    """
    directory = Path(directory or PROBE_DIR)
    directory.mkdir(parents=True, exist_ok=True)
    got = [r for r in results if r.get("reconstructed")]
    keyed = [r for r in got if r["span"]["has_answer_key"]]
    offered = [r for r in keyed if r["span"]["model_offered_one"]]

    lines = [
        "# Citations probe — findings",
        "",
        "**Generated by `scripts/probe_citations.py --report`. Do not hand-edit "
        "the tables**; the written verdicts below them are the part a human "
        "owns.",
        "",
        _headline(results, got),
        "",
        "## Mechanical fields",
        "",
        "The part that decides something: right or wrong, no rubric.",
        "",
        "| card | exact fields | node_name | confidence (human → model) |",
        "|---|---|---|---|",
    ]
    for r in sorted(got, key=lambda r: r["card_id"]):
        conf = r["exact"]["confidence"]
        lines.append(
            f"| `{r['card_id']}` | {r['exact_match_count']}/"
            f"{r['exact_field_count']} | "
            f"{'✅' if r['node_name']['human'] == r['node_name']['model'] else '❌'} "
            f"`{r['node_name']['model']}` | "
            f"{conf['human']} → {conf['model']} "
            f"{'✅' if conf['match'] else '❌'} |")

    lines += [
        "",
        "## Provenance — the headline",
        "",
        "*\"A model states a threshold or an exponent with equal confidence "
        "whether or not it is confabulating.\"* This is the only place in the "
        "pipeline that measures it.",
        "",
        f"- {len(keyed)} of {len(got)} reconstructions have an **answer key** "
        "(the human span is verified against the document).",
        f"- {len(offered)} of those {len(keyed)} offered a span of their own.",
        "",
        "| card | model span | human span (verified) | checked? |",
        "|---|---|---|---|",
    ]
    for r in sorted(got, key=lambda r: r["card_id"]):
        s = r["span"]
        model = (s["model"][:80] + "…") if len(s["model"]) > 80 else (s["model"] or "*(empty)*")
        state = ("**UNRESOLVED — open the paper**" if s["checkable"]
                 else "no answer key" if not s["has_answer_key"]
                 else "model offered none")
        lines.append(f"| `{r['card_id']}` | {model} | "
                     f"{'verified' if s['has_answer_key'] else '—'} | {state} |")

    lines += [
        "",
        "⚠️ **A span marked UNRESOLVED is not a result yet.** The probe records "
        "what the model claimed; whether it is true is a question about a PDF, "
        "and this file is generated by something with no PDF. Resolve each one "
        "by hand and write the verdict under *Written verdicts* below.",
        "",
        "## Prose fields",
        "",
        "Token-overlap similarity, which is a **weak** signal — two correct "
        "statements of one mechanism can share very few words. It ranks "
        "reconstructions; it does not score them.",
        "",
        "| card | mechanism | trigger | derivation |",
        "|---|---|---|---|",
    ]
    for r in sorted(got, key=lambda r: r["card_id"]):
        p = r["prose"]
        lines.append(f"| `{r['card_id']}` | {p['mechanism']['similarity']:.2f} | "
                     f"{p['trigger']['similarity']:.2f} | "
                     f"{p['derivation']['similarity']:.2f} |")

    # Split, because "no card" bundles the probe's most and least interesting
    # outcomes under one heading. A refusal is the prompt working; a rejection
    # is a fluent reconstruction failing a rule the real loader enforces.
    refused = [r for r in results
               if not r.get("reconstructed") and not _rejected(r)]
    rejected = [r for r in results if _rejected(r)]

    if rejected:
        lines += [
            "",
            "## Returned a card the loader refused",
            "",
            "The model produced a card in the frozen schema and "
            "`cards.store.card_from_dict` — the **real** loader, not a laxer "
            "one — rejected it. These are reconstruction failures **with a "
            "stated reason**, not refusals, and the reason is the result.",
            "",
        ]
        for r in sorted(rejected, key=lambda r: r["card_id"]):
            lines.append(f"- `{r['card_id']}` — {r.get('error')}")

    if refused:
        lines += [
            "",
            "## Declined to reconstruct",
            "",
            "No yaml block in the response. `generate/prompts/probe_card.md` "
            "invites *\"I do not know this document\"* explicitly, so this is "
            "a **correct** answer and belongs in the denominator, not in the "
            "errors.",
            "",
        ]
        for r in sorted(refused, key=lambda r: r["card_id"]):
            lines.append(f"- `{r['card_id']}` — {r.get('note') or r.get('error')}")

    lines += [
        "",
        "## Written verdicts",
        "",
        "*Owned by a human. One paragraph per citation: was the mechanism "
        "recalled, were the numbers real, and what does that say about "
        "`B − E`?*",
        "",
        "*(not yet written)*",
        "",
        "---",
        "",
        "**This is not a fifth condition.** No episode was run from anything "
        "here. See `plans/todo/33-citation-probe.md` for why the probe stops at "
        "the card.",
        "",
    ]
    path = directory / "FINDINGS.md"
    path.write_text("\n".join(lines), encoding="utf-8")
    return path
