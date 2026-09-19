"""The autonomous diagnose-and-repair loop. Epic 06.

⚠️ Read `REPAIR-LOOP.md` before touching anything here: it holds the verdict
taxonomy, the repair action space, the arm-blindness decision and the traps.

SIM-FREE. Nothing here imports isaacsim, omni, torch, pxr or carb.
⚠️ Nothing here may import `core.scenarios_held_out`, by the same rule that
protects `generate/` and `compose/`. The seal is made of that.
"""
