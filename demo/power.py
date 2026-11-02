"""
demo/power.py

EXACT POWER FOR THE MINDFIRE CONFIRMATORY TEST, BY ENUMERATION. No simulation
noise, no normal approximation: every possible outcome of the binomial draws is
listed, weighted by its probability, and scored with the test the
pre-registration actually runs (`demo/analyse.py`).

What the confirmatory test is (Mindfire_Experiment_Plan.md §2.5, PREREG-v2.md):

    unit         the controller (one independent generation call)
    success      all 4 trips within 180 s in >= 2 of its 3 physics episodes
    tests        Fisher exact, two-sided, on the FRESH samples only
                 H1 armB != arm0, H2 armB != armX, H3 armX != arm0
    correction   Holm across the three, family-wise alpha = 0.05

So power here is power over CONTROLLER-level success probabilities. The three
physics reps only decide each controller's verdict; they add no sample size.
⚠️ The pilot's per-controller verdicts were 16/17 stable across reps on the
kinematic loop, which is why measurement noise is modelled as zero below.
If Exp 3 shows reps disagreeing often, the effective effect shrinks and these
numbers are optimistic.

Three readings are reported for each scenario:

    raw      power of one Fisher test at alpha = 0.05, no correction
             (what the plan's §2.4 table approximates)
    bonf     at alpha = 0.05/3 -- a LOWER BOUND on that hypothesis's Holm power
    holm     the exact Holm power of H1 (and H2), enumerating all three arms
             jointly, with armX's rate set by the scenario

and, for the three-arm scenarios, the probability that the pre-registered
interpretation table returns each reading.

⚠️ NOT A BENCHMARK PATH. Calls nothing, reads no data, boots no simulator.

    python demo/power.py                    # the full report, ~30 s
    python demo/power.py --k 24 30 --quick  # two-arm table only
"""
import argparse
import functools
import pathlib
import sys

import numpy as np
from scipy.stats import binom, fisher_exact

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

# ---- below this line is data, not code ------------------------------------

ALPHA = 0.05
N_HYP = 3                       # H1, H2, H3, Holm-corrected together

PILOT = {"arm0": (7, 8), "armB": (2, 8)}   # demo/out/physics.csv, legs == 4

# (label, p_arm0, p_armB). p_arm0 is held near the pilot's 0.875 throughout:
# arm0's rate is the one least exposed to the winner's curse, because the
# hypothesis was formed about armB.
TWO_ARM_SCENARIOS = [
    ("pilot point estimate", 0.875, 0.25),
    ("half the pilot gap", 0.875, 0.5625),
    ("plausible shrunk (plan)", 0.85, 0.50),
    ("small (plan)", 0.80, 0.55),
    ("one-third of the gap", 0.875, 0.667),
]

# Three-arm scenarios for the interpretation table: (label, p0, pB, pX).
THREE_ARM_SCENARIOS = [
    ("CONTENT, pilot size", 0.875, 0.25, 0.875),
    ("CONTENT, shrunk", 0.85, 0.50, 0.85),
    ("CONTENT, small", 0.80, 0.55, 0.80),
    ("LENGTH, shrunk (armX = armB)", 0.85, 0.50, 0.50),
    ("HALFWAY (armX between)", 0.85, 0.50, 0.675),
    ("NULL (all equal 0.75)", 0.75, 0.75, 0.75),
]

KS = (8, 24, 30, 32)

# ---- code ------------------------------------------------------------------


@functools.lru_cache(maxsize=None)
def p_table(n1, n2):
    """Two-sided Fisher p for every (x1, x2), x1 in 0..n1, x2 in 0..n2."""
    t = np.empty((n1 + 1, n2 + 1))
    for x1 in range(n1 + 1):
        for x2 in range(n2 + 1):
            t[x1, x2] = fisher_exact([[x1, n1 - x1], [x2, n2 - x2]],
                                     alternative="two-sided")[1]
    return t


def pmf(n, p):
    return binom.pmf(np.arange(n + 1), n, p)


def power_two_arm(n, p1, p2, alpha=ALPHA, n2=None):
    """P(Fisher two-sided p <= alpha) when x1 ~ Bin(n, p1), x2 ~ Bin(n2, p2)."""
    n2 = n if n2 is None else n2
    w = np.outer(pmf(n, p1), pmf(n2, p2))
    return float(w[p_table(n, n2) <= alpha + 1e-12].sum())


def holm_reject(ps, alpha=ALPHA):
    """Holm step-down. ps: array (..., m). Returns a boolean array, same shape.
    Vectorised over leading axes; this is the same rule analyse.holm applies."""
    ps = np.asarray(ps)
    m = ps.shape[-1]
    order = np.argsort(ps, axis=-1, kind="stable")
    sorted_p = np.take_along_axis(ps, order, axis=-1)
    thresh = alpha / (m - np.arange(m))
    ok = sorted_p <= thresh + 1e-12
    ok = np.cumprod(ok, axis=-1).astype(bool)   # stop at the first failure
    out = np.zeros_like(ok)
    np.put_along_axis(out, order, ok, axis=-1)
    return out


def three_arm(n, p0, pB, pX, alpha=ALPHA):
    """Exact joint enumeration over (x0, xB, xX). Returns Holm power per
    hypothesis and the probability of each interpretation-table reading."""
    t = p_table(n, n)
    x = np.arange(n + 1)
    X0, XB, XX = np.meshgrid(x, x, x, indexing="ij")
    w = (pmf(n, p0)[:, None, None] * pmf(n, pB)[None, :, None]
         * pmf(n, pX)[None, None, :])
    ps = np.stack([t[XB, X0], t[XB, XX], t[XX, X0]], axis=-1)  # H1, H2, H3
    rej = holm_reject(ps, alpha)
    h1, h2, h3 = rej[..., 0], rej[..., 1], rej[..., 2]
    same_dir = np.sign(XB - X0) == np.sign(XB - XX)
    content = h1 & h2 & ~h3 & same_dir
    length = h1 & h3 & ~h2
    no_rep = ~h1
    other = ~(content | length | no_rep)
    s = lambda m: float(w[m].sum())
    return {"H1": s(h1), "H2": s(h2), "H3": s(h3), "content": s(content),
            "length": s(length), "not_replicated": s(no_rep), "other": s(other)}


def three_arm_gatekeep(n, p0, pB, pX, alpha=ALPHA):
    """ALTERNATIVE PROCEDURE, NOT THE PLAN'S: H1 tested alone at alpha; only if
    it is rejected are H2 and H3 tested, Holm over the two at alpha. Controls
    the family-wise error (fixed-sequence gatekeeping) and spends no alpha on
    H2/H3 before H1 is in -- which the interpretation table requires anyway.
    Reported so the choice can be made on numbers; analyse.py implements the
    plan's Holm-over-three unless PREREG-v2.md says otherwise."""
    t = p_table(n, n)
    x = np.arange(n + 1)
    X0, XB, XX = np.meshgrid(x, x, x, indexing="ij")
    w = (pmf(n, p0)[:, None, None] * pmf(n, pB)[None, :, None]
         * pmf(n, pX)[None, None, :])
    h1 = t[XB, X0] <= alpha + 1e-12
    rest = holm_reject(np.stack([t[XB, XX], t[XX, X0]], axis=-1), alpha)
    h2, h3 = h1 & rest[..., 0], h1 & rest[..., 1]
    same_dir = np.sign(XB - X0) == np.sign(XB - XX)
    s = lambda m: float(w[m].sum())
    return {"H1": s(h1), "H2": s(h2), "H3": s(h3),
            "content": s(h1 & h2 & ~h3 & same_dir),
            "length": s(h1 & h3 & ~h2), "not_replicated": s(~h1)}


def assurance(n, alpha=ALPHA):
    """Power averaged over the pilot posterior (Beta(1+s, 1+f) per arm).
    ⚠️ Inherits the pilot's winner's curse in full; reported, never planned on."""
    a0, n0 = PILOT["arm0"]; aB, nB = PILOT["armB"]
    grid = np.linspace(0.0005, 0.9995, 400)
    from scipy.stats import beta
    w0 = beta.pdf(grid, 1 + a0, 1 + n0 - a0); w0 /= w0.sum()
    wB = beta.pdf(grid, 1 + aB, 1 + nB - aB); wB /= wB.sum()
    # Predictive distribution of x is beta-binomial; mixing pmfs over the grid.
    P0 = (w0[:, None] * binom.pmf(np.arange(n + 1)[None, :], n, grid[:, None])).sum(0)
    PB = (wB[:, None] * binom.pmf(np.arange(n + 1)[None, :], n, grid[:, None])).sum(0)
    w = np.outer(P0, PB)
    return float(w[p_table(n, n) <= alpha + 1e-12].sum())


def min_detectable(n, p0, target=0.80, alpha=ALPHA):
    """Largest armB rate below p0 that still gives `target` power. Bisection on a
    monotone region; returns (pB, gap)."""
    lo, hi = 0.0, p0
    if power_two_arm(n, p0, lo, alpha) < target:
        return None
    for _ in range(40):
        mid = (lo + hi) / 2
        if power_two_arm(n, p0, mid, alpha) >= target:
            lo = mid
        else:
            hi = mid
    return lo, p0 - lo


K_SEARCH_MAX = 80


def k_needed(p0, pB, pX, target=0.80, hyp="H1"):
    """Smallest k per arm (step 2) whose exact Holm power on `hyp` >= target.
    Fisher power is not monotone in k at the 1e-3 level, hence a linear scan."""
    for k in range(8, K_SEARCH_MAX + 1, 2):
        if three_arm(k, p0, pB, pX)[hyp] >= target:
            return k
    return None


def report(ks=KS, quick=False):
    lines = []
    out = lines.append
    out("=" * 78)
    out("TWO-ARM POWER, H1 (armB vs arm0), Fisher two-sided, exact enumeration")
    out("raw = alpha 0.05 | bonf = alpha 0.05/3, a lower bound on Holm power")
    out("-" * 78)
    hdr = f"{'scenario':28}{'p0':>6}{'pB':>7}" + "".join(
        f"{'k=' + str(k):>9}" for k in ks)
    out(hdr)
    for label, p0, pB in TWO_ARM_SCENARIOS:
        for tag, a in (("raw", ALPHA), ("bonf", ALPHA / N_HYP)):
            row = f"{(label if tag == 'raw' else '') :28}{p0:>6.3f}{pB:>7.3f}"
            row += "".join(f"{power_two_arm(k, p0, pB, a):>9.3f}" for k in ks)
            out(row + f"  {tag}")
    out("-" * 78)
    out("assurance over the pilot posterior (raw alpha; inherits winner's curse):")
    out("    " + "  ".join(f"k={k}: {assurance(k):.3f}" for k in ks))
    out("smallest gap detectable at 80% power, arm0 held at 0.875:")
    for k in ks:
        for tag, a in (("raw", ALPHA), ("bonf", ALPHA / N_HYP)):
            md = min_detectable(k, 0.875, alpha=a)
            out(f"    k={k:<3} {tag:5} " + (
                f"pB <= {md[0]:.3f}  (gap >= {md[1]:.3f})" if md else "none"))
    if quick:
        return "\n".join(lines)

    out("=" * 78)
    out("THREE-ARM, EXACT HOLM over H1/H2/H3, and the interpretation table")
    out("content = H1 & H2 & !H3 & same direction | length = H1 & H3 & !H2")
    out("-" * 78)
    for k in [k for k in ks if k in (24, 30)]:
        out(f"k = {k} per arm (fresh only)")
        out(f"  {'scenario':32}{'H1':>7}{'H2':>7}{'H3':>7}"
            f"{'content':>9}{'length':>8}{'no-rep':>8}{'other':>7}")
        for label, p0, pB, pX in THREE_ARM_SCENARIOS:
            r = three_arm(k, p0, pB, pX)
            out(f"  {label:32}{r['H1']:>7.3f}{r['H2']:>7.3f}{r['H3']:>7.3f}"
                f"{r['content']:>9.3f}{r['length']:>8.3f}"
                f"{r['not_replicated']:>8.3f}{r['other']:>7.3f}")
    out("-" * 78)
    out("ALTERNATIVE (not the plan's): gatekeeping, H1 at 0.05 first, then Holm")
    out("over H2/H3 at 0.05. Same columns.")
    for k in [k for k in ks if k in (24, 30)]:
        for label, p0, pB, pX in THREE_ARM_SCENARIOS:
            r = three_arm_gatekeep(k, p0, pB, pX)
            out(f"  k={k:<3}{label:29}{r['H1']:>7.3f}{r['H2']:>7.3f}{r['H3']:>7.3f}"
                f"{r['content']:>9.3f}{r['length']:>8.3f}{r['not_replicated']:>8.3f}")
    out("-" * 78)
    out("smallest k per arm giving 80% EXACT HOLM power on H1 (content scenarios):")
    for label, p0, pB, pX in THREE_ARM_SCENARIOS[:3]:
        k = k_needed(p0, pB, pX)
        out(f"  {label:32} k = {k if k else '> ' + str(K_SEARCH_MAX)}")
    out("=" * 78)
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--k", type=int, nargs="+", default=list(KS))
    ap.add_argument("--quick", action="store_true",
                    help="two-arm table only (skip the three-arm enumeration)")
    a = ap.parse_args(argv)
    print(report(tuple(a.k), a.quick))


if __name__ == "__main__":
    main()
