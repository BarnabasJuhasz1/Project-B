"""
harness/

The unit-scenario harness: purpose-built micro-worlds, the pass/fail decision
that admits a node, and the promotion gate.

Non-negotiable #5 — "a node is admissible only if it passes its unit scenario in
isolation" — makes this package the *measurement*, not a convenience. G1 reports
the unit pass rate, so `run_unit` is where that number comes from.

Sim-free, like core/, controllers/ and nodes/. That is a hard requirement rather
than a preference here: a unit scenario runs once per generated candidate, and
you cannot boot a 20 s simulator per node.
"""
