"""Sim-free half of Project B.

Nothing in this package may import isaacsim, omni or torch. The floorplan, the
geometry, the controller contract and the metrics all have to be importable in
milliseconds, without a GPU, so that unit tests and the Week 3 node harness do
not pay a 20 s simulator boot per iteration. core/tests/test_import_guard.py
enforces it.
"""
