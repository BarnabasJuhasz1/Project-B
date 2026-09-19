"""
harness/gate.py

Promotion. Non-negotiable #9: an addition is promoted only if it **beats the
incumbent on a frozen regression suite**, for both capability-driven and
failure-driven additions.

The regression suite is every previously admitted node's unit scenario. Adding a
node must not regress any of them.

HOW STRONG IS THAT, HONESTLY
-----------------------------
Today: weak, and worth saying so. The library holds independent leaves, so
admitting one cannot change another's result by any mechanism other than a bug —
shared module state, a poisoned geometry cache, a monkeypatch. Those are real
failure modes and this catches them, but it is not yet the interesting check.

It becomes the interesting check when decorators and composites arrive, because
then a promoted node genuinely can change how an existing one behaves: a
`RateLimitedRetry` wrapped around an existing action changes that action's unit
result. The suite is built now so that it is already running, already green, and
already the thing that fails, rather than being introduced at the point where it
would first have caught something.

The gate is also where the library and the unit scenarios stay in step. A node
in the library with no unit scenario is a node G1 cannot report a pass rate for,
so `promote()` is the only way in.

⚠️ **DORMANT SINCE 2026-09-14 (CHUNK 52), AND SAYING SO RATHER THAN LOOKING
ALIVE.** This file is the one that claims to implement non-negotiable #9, and #9
moved out from under it in two steps:

- **Chunk 44** promoted "beats the incumbent on a frozen suite" from a secondary
  check to THE admission criterion, and moved the measurement from a generated
  micro-world to a TREE on a shared suite.
- **Chunk 52** stopped asking the generator for a micro-world at all. A modifier
  revises the base's command and cannot drive alone, so a world it wrote around
  a base it was never shown is evidence about nothing.

So no node the current pipeline produces has a `UnitScenario`, and nothing in
the live path calls `promote()`. **`scripts/measure_growth.py` is what #9 means
now**: add one node to `Layer[ports, BASE, ...]`, re-run the same generated
worlds, and read `fixed` and `broke`.

This file is **not deleted** — the regression idea is right and the rewrite is
its own chunk (see `DELETION-CANDIDATES.md`, tier 3). What has to change is the
suite: "every admitted node's unit scenario" becomes "the growth suite, at the
incumbent tree's score". Its tests supply their own units and still pass, which
is why the dormancy is easy to miss and is written down here.
"""

from dataclasses import dataclass, field
from typing import Dict, List

from harness.unit import UnitResult, run_unit


@dataclass
class PromotionResult:
    node_id: str
    promoted: bool
    reason: str
    unit: UnitResult = None
    regressions: List[str] = field(default_factory=list)
    snapshot: str = ""

    def __bool__(self):
        return self.promoted


class Gate:
    """Holds a library, the unit scenario for every node in it, and the factory
    that turns a node into something drivable."""

    def __init__(self, library, tree_factory):
        """`tree_factory(node_cls, metadata) -> Controller` — how a leaf becomes
        something the rollout can drive. Injected rather than assumed, because
        chunk 16 composes trees differently from the trivial wrapper used here
        and the gate must not care."""
        self.library = library
        self.tree_factory = tree_factory
        self.units: Dict[str, object] = {}

    # -- the suite ----------------------------------------------------------

    def regression_suite(self):
        """Every admitted node's unit scenario, in admission order."""
        return [(entry, self.units[entry.node_id])
                for entry in self.library.all()
                if entry.node_id in self.units]

    def run_regression(self):
        """Re-run every admitted node against its own unit scenario.

        Returns the node ids that no longer pass. Empty is the only acceptable
        answer; anything else means promoting the last candidate broke an
        incumbent, and the incumbent wins.
        """
        failed = []
        for entry, unit in self.regression_suite():
            result = run_unit(
                lambda e=entry: self.tree_factory(e.node_cls, e.metadata), unit)
            if not result.admitted:
                failed.append(f"{entry.node_id}: {result.reason}")
        return failed

    # -- promotion ----------------------------------------------------------

    def promote(self, node_cls, metadata, unit):
        """Admit one node, or refuse it with a reason.

        Order matters. The candidate's own unit scenario is run FIRST, so a node
        that cannot pass in isolation never reaches the regression suite and
        never costs the time of running it.
        """
        node_id = metadata.node_id
        if node_id in self.library:
            return PromotionResult(
                node_id, False,
                f"{node_id!r} is already in the library, which is append-only "
                f"(#7). A revision is a new node with a new id.")

        result = run_unit(lambda: self.tree_factory(node_cls, metadata), unit)
        if not result.admitted:
            return PromotionResult(node_id, False,
                                   f"failed its unit scenario: {result.reason}",
                                   unit=result)

        # Provisionally admit, then check the incumbents. On any regression the
        # library is rolled back — the incumbent wins, which is what "beats the
        # incumbent on a frozen regression suite" means.
        entry = self.library.add(node_cls, metadata)
        self.units[node_id] = unit
        regressions = [r for r in self.run_regression()
                       if not r.startswith(f"{node_id}:")]
        if regressions:
            self._rollback(node_id)
            return PromotionResult(
                node_id, False,
                "promoting it regressed the frozen regression suite; the "
                "incumbent wins (#9)",
                unit=result, regressions=regressions)

        return PromotionResult(node_id, True, "promoted", unit=result,
                               snapshot=self.library.snapshot())

    def _rollback(self, node_id):
        """Undo a provisional admission.

        The library is append-only for everything that was ever ADMITTED (#7); a
        candidate that failed the regression suite was never admitted, and
        leaving it in would corrupt both the snapshot tag and G1's node count.
        This reaches into Library's internals deliberately and in exactly one
        place, rather than giving Library a public remove() that #7 forbids.
        """
        self.units.pop(node_id, None)
        self.library._entries = [e for e in self.library._entries
                                 if e.node_id != node_id]
        self.library._by_id.pop(node_id, None)

    # -- what G1 reports ----------------------------------------------------

    def report(self):
        provenance = self.library.provenance()
        provenance["units_recorded"] = len(self.units)
        return provenance
