"""
cards/store.py

Load, list and retrieve principle cards.

Retrieval exists because non-negotiable #7 makes the library append-only and
monotonic: by Week 5 the store may hold far more than fits in one prompt, and
keeping the working set small at composition time is the mechanism that replaces
deletion.

**A card whose Prediction names no real metric is rejected at load**, not at
generation time and not at admission. The metric list is discovered from
`core.metrics.score()` rather than hardcoded, so a new metric is usable the day
it exists and a removed one fails loudly instead of silently never matching.
"""

import os
from pathlib import Path

import yaml

from cards.schema import (Card, Confidence, Prediction, Source, Verification)

CARDS_DIR = Path(__file__).resolve().parent


class CardError(ValueError):
    """A malformed card. Raised at load: a card that cannot become a unit test
    is not a card, and finding that out during evaluation is too late."""


# Keys score() returns that a CONTROLLER CANNOT MOVE. A prediction naming one of
# these is not falsifiable about the node, however precisely it is worded.
#
# `localisation_failures` is the interesting one, and it is measured rather than
# assumed: drift is injected open-loop and the runner records `drift.estimate()`
# straight into the trace, so the localisation error at a given step is
# bit-identical across every controller. goal_seeker and scaffold_sweep — which
# behave nothing alike — both score exactly 2 on a drifted doorway_traverse
# because both run the full 6000 steps; reactive_avoid scores 0 only because it
# finished in 798. **A controller can lower it only by finishing sooner.**
#
# It is still worth recording: as a MANIPULATION CHECK that drift actually bit.
# It is not a performance metric and must not be reported as one, or "condition
# B localises better" will be read out of a number that cannot show it.
# Chunk 17 has to say which of the two it is.
CONTROLLER_INDEPENDENT = frozenset({
    "localisation_failures",     # a property of the drift model and duration
    "optimal_distance",          # a property of the scenario
    "scenario", "seed",          # identifiers
})

_METRIC_KEYS = None


def metric_keys():
    """The keys core.metrics.score() actually returns.

    Discovered by scoring one real episode rather than hardcoded, so this list
    cannot drift from the metrics themselves — which is the failure that would
    let a card name a metric that quietly stopped existing.
    """
    global _METRIC_KEYS
    if _METRIC_KEYS is None:
        import controllers
        from core.metrics import score
        from core.scenario import load as load_scenario
        from core.tests.kinematic import rollout
        sc = load_scenario("blind_corner")
        _METRIC_KEYS = frozenset(score(rollout(sc, controllers.make("goal_seeker")), sc))
    return _METRIC_KEYS


def _card_from_dict(data, where):
    try:
        src = data["source"]
        pred = data["prediction"]
        card = Card(
            id=data["id"],
            node_name=data["node_name"],
            source=Source(
                citation=src["citation"],
                span=src.get("span", ""),
                verified=Verification(src.get("verified", "unverified")),
                verified_by=src.get("verified_by", ""),
                url=src.get("url", ""),
            ),
            mechanism=data["mechanism"],
            trigger=data["trigger"],
            derivation=data["derivation"],
            prediction=Prediction(
                statement=pred["statement"],
                metric=pred["metric"],
                baseline=pred["baseline"],
                min_effect=float(pred.get("min_effect", 0.0)),
                cost_metric=pred.get("cost_metric", ""),
            ),
            confidence=Confidence(data["confidence"]),
            tags=tuple(data.get("tags", ())),
            supplies=tuple(data.get("supplies", ())),
            notes=data.get("notes", ""),
            mechanism_family=data.get("mechanism_family", ""),
        )
    except KeyError as exc:
        raise CardError(f"{where}: missing field {exc}") from exc
    except (ValueError, TypeError) as exc:
        raise CardError(f"{where}: {exc}") from exc

    keys = metric_keys()
    for label, name in (("prediction.metric", card.prediction.metric),
                        ("prediction.cost_metric", card.prediction.cost_metric)):
        if name and name not in keys:
            raise CardError(
                f"{where}: {label} {name!r} is not produced by "
                f"core.metrics.score(). A prediction that names no measurable "
                f"quantity is not falsifiable, so the node it produces could "
                f"never be admitted or rejected automatically. "
                f"Known: {sorted(keys)}")
        if name in CONTROLLER_INDEPENDENT:
            raise CardError(
                f"{where}: {label} {name!r} cannot be moved by a controller, so "
                f"a prediction about it is not falsifiable ABOUT THE NODE. See "
                f"cards.store.CONTROLLER_INDEPENDENT for why - "
                f"localisation_failures in particular is identical across "
                f"controllers that behave nothing alike, and differs only with "
                f"episode length.")
    return card


def load_card(path):
    path = Path(path)
    with open(path, encoding="utf-8") as fh:
        data = yaml.safe_load(fh)
    if not isinstance(data, dict):
        raise CardError(f"{path.name}: expected one card per file, got {type(data)}")
    card = _card_from_dict(data, path.name)
    if card.id != path.stem:
        raise CardError(
            f"{path.name}: id {card.id!r} does not match the filename. One card "
            f"per file, named after its id, so a diff names the card it changed.")
    return card


class CardStore:
    """Every card in `cards/`. Loaded eagerly, because a malformed card should
    fail when the store is opened rather than when something reaches for it."""

    def __init__(self, cards):
        self._cards = list(cards)
        self._by_id = {c.id: c for c in self._cards}
        if len(self._by_id) != len(self._cards):
            raise CardError("duplicate card ids in the store")
        # One source, one node. Two cards sharing a node_name would produce two
        # nodes under one name, so the library could not say which source a node
        # came from - and provenance is the whole point of a card. Cards that
        # derive the SAME mechanism say so with `mechanism_family`, which is what
        # G1's coverage count is taken over.
        by_name = {}
        for c in self._cards:
            if c.node_name in by_name:
                raise CardError(
                    f"cards {by_name[c.node_name]!r} and {c.id!r} share "
                    f"node_name {c.node_name!r}. node_name is 1-1 with the card. "
                    f"If they are two derivations of one mechanism, give them "
                    f"distinct names and the same `mechanism_family`.")
            by_name[c.node_name] = c.id

    @classmethod
    def load(cls, directory=None):
        directory = Path(directory or CARDS_DIR)
        paths = sorted(p for p in directory.glob("*.yaml"))
        return cls(load_card(p) for p in paths)

    def __len__(self):
        return len(self._cards)

    def __iter__(self):
        return iter(self._cards)

    def __contains__(self, card_id):
        return card_id in self._by_id

    def all(self):
        return list(self._cards)

    def get(self, card_id):
        if card_id not in self._by_id:
            raise KeyError(f"no card {card_id!r}. Known: {sorted(self._by_id)}")
        return self._by_id[card_id]

    # -- retrieval ----------------------------------------------------------

    def by_tag(self, tag):
        return [c for c in self._cards if tag in c.tags]

    def by_metric(self, metric):
        """Cards predicting an effect on this metric. The failure-driven
        retrieval axis from §5: a failure signature names a metric, and this is
        what turns that into a query."""
        return [c for c in self._cards if c.prediction.metric == metric]

    def by_node(self, node_name):
        """The one card that produces this node. A list, not a card, because
        an unknown name is an empty answer rather than an error - but it can
        never hold more than one, and CardStore.__init__ is why."""
        return [c for c in self._cards if c.node_name == node_name]

    def by_family(self, family):
        """Every card deriving this mechanism, however many sources argue it.
        This is the retrieval axis that survives the corpus growing: asking for
        MaintainHomingVector should find the ant paper and the mammal paper."""
        return [c for c in self._cards if c.family == family]

    def tags(self):
        return sorted({t for c in self._cards for t in c.tags})

    # -- what G1 reports ----------------------------------------------------

    def provenance(self):
        total = len(self._cards)
        traceable = sum(c.is_traceable for c in self._cards)
        return {
            "card_count": total,
            "traceable_count": traceable,
            "traceable_fraction": (traceable / total) if total else 0.0,
            "unverified": sorted(c.id for c in self._cards if not c.is_traceable),
            "by_confidence": {
                level.value: sum(c.confidence is level for c in self._cards)
                for level in Confidence
            },
            # DISTINCT MECHANISMS, not distinct nodes - G1's coverage measure.
            # Taken over families, so N cards on one principle count once.
            "mechanisms": len({c.family for c in self._cards}),
            "nodes": len({c.node_name for c in self._cards}),
        }
