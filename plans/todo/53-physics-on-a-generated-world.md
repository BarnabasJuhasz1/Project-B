> ⚠️ **ON HOLD since 2026-09-16, on the user's instruction.** All work is paused while
> the autonomous diagnose-and-repair loop is built — see [`../../REPAIR-LOOP.md`](../../REPAIR-LOOP.md)
> and [epic 06](../06-epic-repair-loop.md). Nothing here is cancelled or superseded; this file is
> intact and resumes where it stopped. Move it back to `plans/doing/` to restart.

# 53 — Physics on a generated world

**Epic:** [05](../05-epic-environment-suite.md) · **Depends on:** 45, 47, 52 · **Needs Isaac Sim:** yes
**Status: DOING — built 2026-09-15, on the user's request.**

---

## Why

`scripts/measure_growth.py` says `gen_b_44d12df4d25b437f` **broke** maze/2004, and
`scripts/debug_growth_case.py` explains it kinematically. The user asked to see the same
case under physics, recorded, with lab42's camera setup.

That was not possible. Three things were missing, and none of them was a flag:

| # | Gap | Where it was stated |
|---|---|---|
| 1 | No USD exists for a generated world | `core/worldgen.py:GeneratedScenario` — *"`scene` names a USD that DOES NOT EXIST YET"* |
| 2 | `core.scenario.load()` resolves committed scenarios only | CLAUDE.md — *"A WORLD IS NOT A SCENARIO"* |
| 3 | No `controllers.make()` spec for the growth pairing | `Layer[ports, BASE, node]` lived only inside `measure_growth._build` |

## Built

1. **`scripts/export_world.py`** — a generated world -> USD. ⚠️ **Lighting, wall height,
   wall colour and the goal marker are lab42's, imported from `core/scene_spec.py` and not
   redeclared.** CLAUDE.md: *"Lighting is part of the scene spec, not decoration."* A
   generated scene with its own lights would not be comparable with lab42 or with another
   generated world, and the difference would be invisible in every number. Obstacles come
   from `world.rects` as axis-aligned blocks of their own footprint — a generated obstacle
   is a block, not a thin wall.
2. **`core.scenario.load()` resolves `gen-<tier>-<seed>`**, via a lazy import of
   `core.worldgen` (the same pattern, and the same cycle reason, as `_held_out()`).
3. **`controllers.make("layer:<node_id>")`** -> the growth pairing, with
   `measure_growth._build` **imported, never re-declared**.
4. `_library_entry()` in `controllers/__init__.py`: one definition of "resolve this node id",
   shared by the `node:` and `layer:` branches.

## Decided: resolvable by name, never enumerated

`all_scenarios()` does **not** return generated scenarios, and `_BY_ID` is untouched. A
generated world is not a benchmark scenario: it is in no split, carries no sealed seed and
appears in no `PRE-REGISTRATION.md` constant. A batch runner that iterates "everything" must
not silently evaluate against it.

**This is exactly the rule the sealed split already follows** — `load()` resolves a held-out
id by name and `all_scenarios()` refuses to enumerate it — and it is adopted here for a
related reason. Naming one is an explicit act; listing them is what we avoid.

## Decided: `layer:` is refused as a condition binding

`runs/spec.py` refuses it, as it already refuses `node:` and scaffolding. The argument is
one step longer than for `node:` and worth writing down:

> The base is exempt from non-negotiable #3 **as a declared substrate** (chunk 44's single
> exemption). That is precisely what makes it part of every arm — and therefore an arm by
> itself in no sweep. `Layer[ContractPorts, BASE, node]` is the substrate plus one leaf: a
> probe for `measure_growth.py`, never a condition.

## Done when

- [x] `export_world.py` writes a loadable USD for `(tier, seed)`, lighting imported not redeclared.
- [x] `load("gen-maze-02004")` resolves and validates; `all_scenarios()` is unchanged.
- [x] `controllers.make("layer:<id>")` returns the same tree `measure_growth._build` does.
- [x] `runs/spec.py` refuses a `layer:` condition binding.
- [ ] Two physics episodes recorded on `gen-maze-02004`: base alone, and base+node.
- [ ] The sim-free suite is green with no committed record rewritten.

## Not in scope

- **Any reportable number.** These run locally, so every result carries `in_container:false`.
- **A frozen generated suite.** That is chunk 46. This resolves worlds one at a time, by name.
- **Moving obstacles** — chunk 49, which is what the epic's table means by sim-side work.
- **Drift or SLAM on a generated world.** Untouched and untested here.

## Risks known before the run

1. **Physics may not reproduce the kinematic verdict.** The base reaches the goal at t ≈ 68 s
   kinematically; under the frozen locomotion policy it may not. Chunk 36 measured
   kinematic->physics agreement as good on `goal_reached` (6/6) and poor on path shape for
   FAILING runs, so the base's success is the likelier half to survive.
2. **Rendering dominates.** ~0.9 s/frame fixed overhead. At 15 fps a 120 s episode is ~27 min
   and the pair times out at 120 s, so the two episodes together are ~45 min of wall clock.
3. **`validate()` costs ~17 s per resolution** — `optimal_distance()` is a visibility-graph
   solve, quadratic in obstacle count. Paid deliberately: a run whose SPL denominator was
   never computed cannot be scored.
