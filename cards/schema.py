"""
cards/schema.py

The principle card: the interface between extraction and generation, and the
thing that keeps the pipeline source-agnostic. Six fields, from §6 of the
description: **Source, Mechanism, Trigger, Derivation, Prediction, Confidence.**

They map onto a leaf node like this, and the mapping is the whole point:

    Trigger                -> the precondition port
    Mechanism + Derivation -> the node body
    Prediction             -> the unit test
    Source + Confidence    -> node metadata

WHY A PREDICTION IS STRUCTURED RATHER THAN PROSE
------------------------------------------------
"Prediction becomes the unit test" is only mechanical if a Prediction names
something `core.metrics.score()` actually computes. A card whose Prediction is
prose produces a node nobody can admit or reject automatically — and §4 requires
that decision to be automatic. So a Prediction carries a metric key, a baseline
and a minimum effect, and `Card.unit_scenario()` turns it directly into the
`harness.unit.UnitScenario` that decides admissibility. **A card whose Prediction
is not falsifiable is rejected at load**, which is the filter working rather than
failing.

`cost_metric` is the honest counterpart: the metric this mechanism is expected to
make WORSE. LimitSpeedToVisibleRange buys clearance with time, and a card that
cannot say what it costs is a card that will be reported selectively.

WHY VERIFICATION IS A FIRST-CLASS STATE
----------------------------------------
G1 reports "the fraction carrying a source span with extraction confidence", and
provenance is one of the three advantages a corpus is claimed to supply over a
prompted model — the description's own framing is that "a model states a
threshold or an exponent with equal confidence whether or not it is
confabulating."

A source span written from memory is exactly that confabulation. So `verified`
is a separate, explicit state, an UNVERIFIED card is **never counted as
traceable**, and `span` may only be non-empty once a human has opened the paper.
The pipeline runs on unverified cards — the mechanisms are still falsifiable by
the harness — but the provenance number cannot be inflated by them.
"""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple


class Confidence(Enum):
    """Extraction confidence. Three points with written criteria, because a
    0–1 float nobody calibrates makes G1's fraction meaningless.

    HIGH   the mechanism AND its quantitative form are stated explicitly in the
           source; the span points at the equation, threshold or exponent itself.
    MEDIUM the mechanism is stated explicitly, but the quantitative form is
           adapted — different units, a different body scale, a different sensor.
           The number is ours; the mechanism is theirs.
    LOW    the mechanism is described qualitatively and the derivation is ours.
           Still usable, but it is our claim wearing their citation, and the
           write-up must not present it otherwise.
    """

    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


class Verification(Enum):
    """Has a human opened the paper and checked the span?

    UNVERIFIED is the honest default and the state every card starts in. It is
    not a formality: an unverified span is a claim about a document nobody has
    read, which is the failure mode the provenance axis exists to detect.
    """

    UNVERIFIED = "unverified"
    VERIFIED = "verified"


@dataclass(frozen=True)
class Source:
    """Where the principle came from.

    `citation` is what you would put in a bibliography. `span` is the page,
    section or equation — the part that makes a stated threshold checkable, and
    the part that may only be filled in by someone who has the paper open.
    """

    citation: str
    span: str = ""
    verified: Verification = Verification.UNVERIFIED
    verified_by: str = ""
    url: str = ""

    def __post_init__(self):
        if not self.citation.strip():
            raise ValueError("a Source needs a citation")
        if self.verified is Verification.VERIFIED:
            if not self.span.strip():
                raise ValueError(
                    f"{self.citation!r}: marked VERIFIED with no span. A "
                    f"verification that does not say WHERE is not a "
                    f"verification.")
            if not self.verified_by.strip():
                raise ValueError(
                    f"{self.citation!r}: marked VERIFIED with no verified_by. "
                    f"Provenance means someone is accountable for having "
                    f"looked.")

    @property
    def is_traceable(self):
        """Exactly what G1 reports the fraction of. Unverified is not traceable,
        however plausible the span looks."""
        return (self.verified is Verification.VERIFIED
                and bool(self.span.strip()))


@dataclass(frozen=True)
class Prediction:
    """The falsifiable claim, in a form the harness can decide automatically."""

    statement: str          # the claim in words, for the write-up
    metric: str             # a key of core.metrics.score()
    baseline: str           # a controller name the claim is made against
    min_effect: float = 0.0
    cost_metric: str = ""   # what this mechanism is expected to make WORSE

    def __post_init__(self):
        if not self.statement.strip():
            raise ValueError("a Prediction needs a statement")
        if not self.metric.strip():
            raise ValueError(
                f"{self.statement[:50]!r}: a Prediction must name a metric that "
                f"core.metrics.score() computes, or it cannot become a unit "
                f"test and the node it produces can never be admitted.")
        if self.min_effect < 0:
            raise ValueError("min_effect cannot be negative")


@dataclass(frozen=True)
class Card:
    """One principle, extracted from one source."""

    id: str
    node_name: str          # the leaf this card is meant to produce
    source: Source
    mechanism: str          # what the finding says happens
    trigger: str            # when it applies -> the precondition port
    derivation: str         # how it becomes a rule for this robot
    prediction: Prediction
    confidence: Confidence
    tags: Tuple[str, ...] = ()
    supplies: Tuple[str, ...] = ()   # novelty / coverage / provenance
    notes: str = ""

    ADVANTAGES = ("novelty", "coverage", "provenance")

    def __post_init__(self):
        for name in ("id", "node_name", "mechanism", "trigger", "derivation"):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"card {self.id!r}: {name} is empty")
        unknown = [s for s in self.supplies if s not in self.ADVANTAGES]
        if unknown:
            raise ValueError(
                f"card {self.id!r}: unknown claimed advantage {unknown}. G1 "
                f"reports these three separately and they are not the same "
                f"claim; {list(self.ADVANTAGES)}.")
        if "novelty" in self.supplies:
            raise ValueError(
                f"card {self.id!r}: claims NOVELTY. No hand-curated card can. "
                f"Novelty means 'published after the training cutoff', and "
                f"every source a model can suggest is by construction inside "
                f"its own training distribution. The novelty axis needs the "
                f"ingestion daemon polling recent venues (§5); saying so is a "
                f"finding, claiming it here would be a lie.")

    @property
    def is_traceable(self):
        return self.source.is_traceable

    def unit_scenario(self, micro, node_id=None, seeds=None):
        """Prediction -> the unit test that decides admissibility.

        This is the mapping §4 asks for, made mechanical rather than
        aspirational. The harness raises `min_effect` to its own floor if this
        card asks for less; a card can demand more, never less.
        """
        from harness.unit import UnitScenario
        kwargs = dict(
            node_id=node_id or self.node_name,
            micro=micro,
            metric=self.prediction.metric,
            baseline=self.prediction.baseline,
            min_effect=self.prediction.min_effect,
            note=f"{self.id}: {self.prediction.statement}",
        )
        if seeds is not None:
            kwargs["seeds"] = tuple(seeds)
        return UnitScenario(**kwargs)
