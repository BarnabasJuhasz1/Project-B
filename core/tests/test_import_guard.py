"""
core/tests/test_import_guard.py

core/, controllers/, nodes/, harness/ and cards/ are the sim-free half of the codebase: importable in milliseconds, on a
machine with no GPU, so unit tests and the Week 3 node harness do not pay a
20 s simulator boot per iteration. That property is easy to lose to one
convenience import, and it fails silently — the code still works, it just
becomes 400x slower to test and impossible to run outside the container.

So assert it. A subprocess imports every module in the package and reports what
landed in sys.modules.
"""

import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Importing any one of these is what makes a module require a simulator.
# torch is on the list because the frozen policy owns it: it lives below the
# Twist interface, and nothing above that line has any business touching it.
FORBIDDEN = ("isaacsim", "omni", "torch", "pxr", "carb")

PROBE = r"""
import importlib, pkgutil, sys
import cards, controllers, core, harness, nodes, runs

names = [m.name for m in pkgutil.walk_packages(core.__path__, "core.")
         if not m.name.startswith("core.tests")]
names += [m.name for m in pkgutil.walk_packages(controllers.__path__,
                                                "controllers.")]
names += [m.name for m in pkgutil.walk_packages(nodes.__path__, "nodes.")
          if not m.name.startswith("nodes.tests")]
names += [m.name for m in pkgutil.walk_packages(harness.__path__, "harness.")
          if not m.name.startswith("harness.tests")]
names += [m.name for m in pkgutil.walk_packages(cards.__path__, "cards.")
          if not m.name.startswith("cards.tests")]
names += [m.name for m in pkgutil.walk_packages(runs.__path__, "runs.")
          if not m.name.startswith("runs.tests")]
for name in names:
    importlib.import_module(name)

forbidden = {f}
hits = sorted(m for m in sys.modules
              if m.split(".")[0] in forbidden)
print("IMPORTED:" + ",".join(sorted(names)))
print("HITS:" + ",".join(hits))
"""


def _probe():
    proc = subprocess.run(
        [sys.executable, "-c", PROBE.format(f=repr(FORBIDDEN))],
        cwd=REPO_ROOT, capture_output=True, text=True,
    )
    assert proc.returncode == 0, proc.stderr
    lines = dict(line.split(":", 1) for line in proc.stdout.strip().splitlines()
                 if ":" in line)
    return lines


def test_core_imports_no_simulator_modules():
    lines = _probe()
    hits = [h for h in lines["HITS"].split(",") if h]
    assert not hits, (
        "the sim-free packages pulled in simulator modules: " + ", ".join(hits) +
        ". Move whatever needs them into scripts/, which is imported only after "
        "SimulationApp exists.")


def test_probe_actually_imported_something():
    """Guards the guard: an empty module list would pass the test above."""
    modules = [m for m in _probe()["IMPORTED"].split(",") if m]
    assert "core.scene_spec" in modules, modules
    assert "controllers.goal_seeker" in modules, modules
    # runs/ drives the simulator through subprocesses and must not import it
    # itself -- that is what lets --dry-run cost milliseconds.
    assert "runs.sweep" in modules, modules
