"""repair/verdict.py — the rule, tested against the COMMITTED growth table.

⚠️ These tests are deliberately built on real committed measurements rather
than on invented numbers wherever a real one exists. The taxonomy's job is to
reproduce judgements a human made by hand, and a synthetic fixture cannot check
that.
"""
from __future__ import annotations

import ast
import collections
from pathlib import Path

import pytest

from repair import verdict as V

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
GROWTH = REPO_ROOT / "out" / "growth"

pytestmark = pytest.mark.skipif(
    not (GROWTH / "maze-worlds.csv").exists(),
    reason="the committed growth table is not on this machine")


# --------------------------------------------------------------------------- #
# helpers: a Measurement without a CSV, so the RULE can be tested on its own


def _worlds(n=20, broke=0, fixed=0, fire=0.9, changed=0.9, base_ok=10):
    """n worlds with a stated outcome mix. Seeds mirror the real tier."""
    out, made_broke, made_fixed = [], 0, 0
    for i in range(n):
        base_reached = i < base_ok
        if base_reached and made_broke < broke:
            outcome, pair, made_broke = "broke", False, made_broke + 1
        elif not base_reached and made_fixed < fixed:
            outcome, pair, made_fixed = "fixed", True, made_fixed + 1
        elif base_reached:
            outcome, pair = "both_ok", True
        else:
            outcome, pair = "both_fail", False
        out.append(V.WorldOutcome(seed=2000 + i, scenario_id=f"gen-maze-{2000+i:05d}",
                                  base_reached=base_reached, pair_reached=pair,
                                  outcome=outcome, fire=fire, changed=changed))
    return tuple(out)


def _measure(metrics=None, **kw):
    w = _worlds(**{k: v for k, v in kw.items() if k in
                   ("n", "broke", "fixed", "fire", "changed", "base_ok")})
    return V.Measurement(
        node_id=kw.get("node_id", "gen_x_test"), tier="maze", worlds=w,
        metrics=metrics or {},
        fire=kw.get("fire", 0.9), changed=kw.get("changed", 0.9))


def _claim(metric="goal_reached", min_effect=0.1):
    return V.Claim(metric=metric, min_effect=min_effect,
                   direction=V.DIRECTION.get(metric), source="card")


# --------------------------------------------------------------------------- #
# the case the whole chunk was checked against


def test_the_diagnosed_node_is_degenerate():
    """⚠️ THE ONE THAT MATTERS. `gen_b_44d12df4d25b437f` froze the robot in open
    space — 46 cm clearance, 0 contacts — for the last 52 s of a 120 s episode
    on a world the base alone solves. If the rule does not catch the one case
    diagnosed by hand, the rule is wrong."""
    v = V.classify_node("gen_b_44d12df4d25b437f")
    assert v.label is V.Label.DEGENERATE
    assert v.n == 20
    assert "2004" in v.reason          # the world that was opened up under physics
    assert "without colliding more" in v.reason


def test_the_metrics_md_per_metre_case_is_caught():
    """The node that "won" by stopping: collisions 0.40 -> 0.30, which read as
    the library's second win. It had walked 6.2 m instead of 30.0 and reached
    the goal in 0 of 20 worlds.

    ⚠️ The raw count says BETTER. The rule must not."""
    m = V.load_measurement("gen_b_c2c95bcb965e342e")
    base_col, pair_col = m.pair_of("collisions")
    base_path, pair_path = m.pair_of("path_length")
    assert pair_col < base_col                     # the raw count improved
    assert pair_path < 7.0 and base_path > 29.0    # because it stopped

    # per metre it collides ~3.6x MORE, which is the number METRICS.md quotes
    ratio = (pair_col / pair_path) / (base_col / base_path)
    assert 3.3 < ratio < 3.9, ratio

    v = V.classify(m, V.claim_for("gen_b_c2c95bcb965e342e"))
    assert v.label is V.Label.DEGENERATE
    assert v.label is not V.Label.HELPED


def test_claim_improvement_uses_the_per_metre_form_for_counts():
    """A count metric is compared per metre, and the flag says so."""
    m = _measure(metrics={"collisions": (0.4, 0.3), "path_length": (30.0, 6.2)})
    delta, per_metre = V._claim_improvement(m, _claim("collisions", 0.1))
    assert per_metre is True
    assert delta < 0, "per metre this is WORSE; the raw count would say better"


# --------------------------------------------------------------------------- #
# precedence — one test per branch, in declared order


def test_inert_on_fire_floor():
    v = V.classify(_measure(fire=0.0, changed=0.0), _claim())
    assert v.label is V.Label.INERT and "nothing here triggered it" in v.reason


def test_inert_on_changed_floor_even_when_it_fires_every_tick():
    """⚠️ Both floors are load-bearing. Three committed nodes fire on ~100% of
    ticks and revise the command on 0.00%: `fire` alone reads them as active."""
    v = V.classify(_measure(fire=1.0, changed=0.0), _claim())
    assert v.label is V.Label.INERT and "acted and did nothing" in v.reason


def test_degenerate_requires_losing_a_world_the_base_won():
    """both_fail is not breaking anything."""
    metrics = {"goal_reached": (0.5, 0.0), "collisions": (0.4, 0.0),
               "path_length": (30.0, 6.0), "stuck_events": (2.5, 9.0)}
    assert V.classify(_measure(metrics=metrics, broke=5), _claim()).label \
        is V.Label.DEGENERATE
    assert V.classify(_measure(metrics=metrics, broke=0), _claim()).label \
        is not V.Label.DEGENERATE


def test_degenerate_excludes_crashing():
    """⚠️ A node that collides MORE is a different defect with a different
    repair. Calling both DEGENERATE sends the loop after the wrong cause."""
    crashed = {"goal_reached": (0.5, 0.0), "collisions": (0.4, 5.3),
               "path_length": (30.0, 5.0), "stuck_events": (2.5, 0.0)}
    v = V.classify(_measure(metrics=crashed, broke=8), _claim("collisions", 0.5))
    assert v.label is V.Label.REGRESSED


def test_helped_needs_task_integrity():
    """⚠️ INTEGRITY IS A PRECONDITION, NOT A BONUS. The same claimed improvement
    reads as HELPED when the robot still moves and does not when it stopped."""
    moving = {"goal_reached": (0.5, 0.5), "collisions": (0.4, 0.1),
              "path_length": (30.0, 29.0), "stuck_events": (2.5, 2.0)}
    stopped = dict(moving, path_length=(30.0, 6.0))
    c = _claim("collisions", 0.5)
    assert V.classify(_measure(metrics=moving), c).label is V.Label.HELPED
    assert V.classify(_measure(metrics=stopped), c).label is not V.Label.HELPED


def test_misjudged_is_the_primary_falling_while_the_claim_holds():
    metrics = {"goal_reached": (0.5, 0.15), "collisions": (0.4, 0.0),
               "path_length": (30.0, 27.8), "stuck_events": (2.5, 3.0)}
    v = V.classify(_measure(metrics=metrics), _claim("collisions", 0.5))
    assert v.label is V.Label.MISJUDGED
    assert "a claim it never made" in v.reason


def test_helped_and_misjudged_are_disjoint():
    """They differ on exactly one thing: whether the PRIMARY fell."""
    kept = {"goal_reached": (0.5, 0.5), "collisions": (0.4, 0.0),
            "path_length": (30.0, 29.0), "stuck_events": (2.5, 2.0)}
    lost = dict(kept, goal_reached=(0.5, 0.15))
    c = _claim("collisions", 0.5)
    assert V.classify(_measure(metrics=kept), c).label is V.Label.HELPED
    assert V.classify(_measure(metrics=lost), c).label is V.Label.MISJUDGED


def test_two_conditions_at_once_resolve_by_declared_precedence():
    """⚠️ A node can be INERT *and* look like it broke worlds. Precedence is
    declared in advance, so this is decided rather than discovered: a node that
    never acted cannot have caused anything."""
    metrics = {"goal_reached": (0.5, 0.0), "collisions": (0.4, 0.0),
               "path_length": (30.0, 6.0), "stuck_events": (2.5, 9.0)}
    v = V.classify(_measure(metrics=metrics, broke=10, fire=0.0, changed=0.0),
                   _claim())
    assert v.label is V.Label.INERT
    assert list(V.Label).index(V.Label.INERT) < list(V.Label).index(V.Label.DEGENERATE)


def test_regressed_is_the_fallthrough_for_uncovered_harm():
    metrics = {"goal_reached": (0.5, 0.3), "collisions": (0.4, 0.4),
               "path_length": (30.0, 29.0), "stuck_events": (2.5, 2.5)}
    assert V.classify(_measure(metrics=metrics), _claim()).label is V.Label.REGRESSED


def test_accept_when_nothing_moved_either_way():
    metrics = {"goal_reached": (0.5, 0.5), "collisions": (0.4, 0.4),
               "path_length": (30.0, 30.0), "stuck_events": (2.5, 2.5)}
    assert V.classify(_measure(metrics=metrics), _claim()).label is V.Label.ACCEPT


# --------------------------------------------------------------------------- #
# the effect size, and the floors


def test_effect_size_is_not_defined_twice():
    """⚠️ `harness.unit.effect` is THE definition: absolute for a rate metric,
    RELATIVE for every other. A second copy would drift and the two numbers
    would be reported as if they were comparable."""
    from harness import unit
    assert V._improvement("goal_reached", 0.5, 0.0) == unit.effect("goal_reached", 0.0, 0.5)
    assert V._improvement("collisions", 0.4, 0.3) == unit.effect("collisions", 0.3, 0.4)
    # and they genuinely differ in regime
    assert V._improvement("goal_reached", 0.4, 0.3) == pytest.approx(-0.1)
    assert V._improvement("collisions", 0.4, 0.3) == pytest.approx(0.25)


def test_rate_regime_agrees_with_the_harness():
    from harness.unit import RATE_METRICS
    assert V.RATE_REGIME == RATE_METRICS


def test_a_card_may_demand_more_than_the_floor_never_less():
    assert V.Claim("goal_reached", 0.01, "higher", "card").floor == V.FLOOR_RATE
    assert V.Claim("goal_reached", 0.40, "higher", "card").floor == 0.40
    assert V.Claim("collisions", 0.00, "lower", "card").floor == V.FLOOR_REL


def test_the_floor_is_not_the_harness_micro_world_floor():
    """⚠️ `MIN_EFFECT_RATE` is 0.5, calibrated for a micro-world whose baseline
    was built to FAIL. On a suite where the base already reaches 50% it means
    "solve every remaining world". Chunk 37/44 scrapped that floor as an
    admission criterion; importing it here would reinstate it."""
    from harness.unit import MIN_EFFECT_RATE
    assert V.FLOOR_RATE < MIN_EFFECT_RATE


# --------------------------------------------------------------------------- #
# properties every verdict must have


def test_every_committed_node_classifies_and_carries_n():
    verdicts = V.classify_tier("maze")
    assert len(verdicts) == 16
    for v in verdicts:
        assert v.n == 20, v
        assert v.reason.strip()
        assert isinstance(v.label, V.Label)


def test_the_committed_distribution_is_pinned():
    """⚠️ ZERO HELPED across sixteen nodes, which is the honest state. Pinned so
    that a change to the rule has to be an argued change, not a silent one."""
    tally = collections.Counter(v.label.value for v in V.classify_tier("maze"))
    assert dict(tally) == {"INERT": 8, "DEGENERATE": 3, "REGRESSED": 3,
                           "MISJUDGED": 1, "ACCEPT": 1}
    assert tally["HELPED"] == 0


def test_a_verdict_carries_no_arm_tell():
    """A `Verdict` goes into chunk 55's BLIND bundle. Its own fields must not
    name a condition. (`node_id` is handled there by an opaque handle; nothing
    else here may leak.)"""
    v = V.classify_node("gen_b_44d12df4d25b437f")
    leaky = f"{v.label} {v.reason} {v.claim_metric} {v.tier} {v.unchecked}"
    for tell in ("condition", "citation", "treatment", "arm",
                 "Wehner", "Müller", "path_integration_ants"):
        assert tell.lower() not in leaky.lower(), tell


def test_inert_is_never_a_success():
    """⚠️ The cheapest way to stop a node regressing is to make it decline
    everywhere. A loop scored on "no longer regresses" finds this immediately."""
    v = V.classify(_measure(fire=0.0, changed=0.0), _claim())
    assert v.label is V.Label.INERT
    assert v.label not in (V.Label.HELPED, V.Label.ACCEPT)


def test_the_verdict_flips_between_two_world_sets_of_twenty():
    """⚠️ SAY `n` OUT LOUD. The project's only positive node scores +0.10 on
    seeds 2000-2019 and +0.20 on seeds 3000-3019, against its own card's floor
    of 0.15. Both are real committed measurements at n=20. The verdict is a
    statement about a world set, not about a node."""
    claim = V.claim_for("gen_d_88c49cee31e01477")
    assert claim.floor == 0.15

    on_2000 = {"goal_reached": (0.50, 0.60), "collisions": (0.4, 0.4),
               "path_length": (30.0, 30.0), "stuck_events": (2.5, 2.5)}
    on_3000 = dict(on_2000, goal_reached=(0.40, 0.60))

    assert V.classify(_measure(metrics=on_2000, fixed=2), claim).label is V.Label.ACCEPT
    assert V.classify(_measure(metrics=on_3000, fixed=4), claim).label is V.Label.HELPED


def test_unchecked_records_what_could_not_be_tested():
    """⚠️ THE TAXONOMY'S HARD EDGE. "The goal was unreachable without a
    collision and the node correctly refused" is not computable. The rule must
    say so rather than guess."""
    stopped_but_safe = {"goal_reached": (0.5, 0.1), "collisions": (0.4, 0.0),
                        "path_length": (30.0, 5.0), "stuck_events": (2.5, 2.0)}
    v = V.classify(_measure(metrics=stopped_but_safe), _claim("collisions", 0.5))
    assert any("not computable" in u for u in v.unchecked), v.unchecked


def test_a_missing_metrics_file_is_declared_not_silently_absent():
    m = V.load_measurement("gen_b_44d12df4d25b437f", metrics_csv="/nonexistent.csv")
    v = V.classify(m, V.claim_for("gen_b_44d12df4d25b437f"))
    assert v.unchecked, "a verdict with no metrics must say what it could not check"


# --------------------------------------------------------------------------- #
# the seal, and sim-freeness


def _imports(path):
    tree = ast.parse(Path(path).read_text(encoding="utf-8"))
    out = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            out.update(a.name for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            out.add(node.module)
    return out


@pytest.mark.parametrize("path", sorted(
    str(p) for p in (REPO_ROOT / "repair").rglob("*.py")))
def test_repair_is_sim_free_and_sealed(path):
    """⚠️ `repair/` must never import `core.scenarios_held_out` — the same AST
    guard that protects generation and composition. The seal is made of that."""
    mods = _imports(path)
    for banned in ("isaacsim", "omni", "torch", "pxr", "carb"):
        assert not any(m == banned or m.startswith(banned + ".") for m in mods), \
            f"{path} imports {banned}"
    assert "core.scenarios_held_out" not in mods, f"{path} imports the SEALED split"
