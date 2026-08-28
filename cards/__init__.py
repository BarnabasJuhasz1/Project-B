"""
cards/

The principle card store: the interface between extraction and generation.

Sim-free, like core/, controllers/, nodes/ and harness/, and it must never
import `core.scenarios_held_out` — a card is retrieved by a task specification,
never by a held-out scenario.
"""

from cards.schema import Card, Confidence, Prediction, Source, Verification
from cards.store import CardStore, CardError, load_card

__all__ = ["Card", "CardStore", "CardError", "Confidence", "Prediction",
           "Source", "Verification", "load_card"]
