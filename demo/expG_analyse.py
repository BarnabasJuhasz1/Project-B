"""
demo/expG_analyse.py

EXP G's ANALYSIS, AS PRE-REGISTERED IN prereg/expG.md. Written and committed
BEFORE any armX controller exists. Calls no model, boots no simulator.

Primary: armX successes (legs == 4) in the physics rep-0 episode, the pilot's
exact condition, against the pilot's physics verdicts (demo/out/physics.csv:
arm0 7/8, armB 2/8). Fisher two-sided, Holm over the two tests at 0.05, with
Wilson and Newcombe intervals. Everything else is printed as EXPLORATORY.

    python demo/expG_analyse.py                 # whatever exists so far
    python demo/expG_analyse.py --require-complete
"""
import argparse
import csv
import pathlib
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from demo.analyse import wilson, newcombe, fisher, holm, mwu      # noqa: E402

DEMO = pathlib.Path(__file__).resolve().parent
PILOT_PHYS = DEMO / "out" / "physics.csv"          # the pilot, never written
EXP3_PHYS = DEMO / "out" / "v2" / "physics.csv"    # Exp 3: arm0/armB, 3 reps
EXPG = DEMO / "out" / "v2" / "expG"
K = 8
SUCCESS_LEGS = 4


def rows(path):
    if not pathlib.Path(path).exists():
        return []
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def legs(r):
    return None if r.get("legs") in ("", None) else int(r["legs"])


def show(name, x, n):
    lo, hi = wilson(x, n)
    return f"{name:6} {x}/{n}  Wilson95 [{lo:.2f}, {hi:.2f}]"


def contrast(label, x, n, cx, cn, cname):
    lo, hi = newcombe(x, n, cx, cn)
    p = fisher(x, n, cx, cn)
    print(f"  {label}: armX {x}/{n} vs {cname} {cx}/{cn}: diff {x / n - cx / cn:+.3f} "
          f"Newcombe95 [{lo:+.2f}, {hi:+.2f}]  Fisher p = {p:.4f}")
    return p


def majority(eps_by_ctrl, n_reps=3):
    """A controller succeeds if >= 2 of its 3 reps do; None if reps missing."""
    out = {}
    for key, eps in eps_by_ctrl.items():
        if len(eps) < n_reps or any(e is None for e in eps):
            continue
        out[key] = sum(e >= SUCCESS_LEGS for e in eps) * 2 > n_reps
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--require-complete", action="store_true")
    a = ap.parse_args(argv)

    pilot = {(r["arm"], r["k"]): int(r["legs"]) for r in rows(PILOT_PHYS)
             if r["arm"] in ("arm0", "armB")}
    pc = {arm: sum(v >= SUCCESS_LEGS for (ar, _), v in pilot.items() if ar == arm)
          for arm in ("arm0", "armB")}
    pn = {arm: sum(1 for (ar, _) in pilot if ar == arm) for arm in ("arm0", "armB")}
    assert (pc["arm0"], pc["armB"]) == (7, 2), f"pilot counts moved: {pc}"

    phys = rows(EXPG / "physics.csv")
    rep0 = {r["k"]: legs(r) for r in phys if r["arm"] == "armX" and r["rep"] == "0"}
    n_ctrl = len(list((DEMO / "arms" / "armX").glob("k*.py")))
    print(f"[expG] armX controllers on disk: {n_ctrl}; physics rep-0 episodes: "
          f"{sum(v is not None for v in rep0.values())}")
    if a.require_complete and (len(rep0) < n_ctrl or None in rep0.values()):
        raise SystemExit("[expG] incomplete: not every armX controller has a rep-0 result")

    # ---- PRIMARY ----------------------------------------------------------
    # A generation call that produced no controller (failed, timed out, no
    # fenced block) is a 0-trip controller IN the denominator (prereg).
    samples = {f.name.split("__")[1] for f in (DEMO / "transcripts").glob("armX__k*__*.json")}
    no_code = sorted(k for k in samples if not (DEMO / "arms" / "armX" / f"{k}.py").exists())
    x = sum(v is not None and v >= SUCCESS_LEGS for v in rep0.values())
    n = sum(v is not None for v in rep0.values()) + len(no_code)
    if no_code:
        print(f"[expG] samples with no controller, counted as 0 trips: {no_code}")
    if n == 0:
        print("[expG] no armX physics results yet -- nothing to test")
        return
    print("\nPRIMARY (physics, rep 0 = the pilot's condition, success = 4/4 trips)")
    for name, cx, cn in (("arm0", pc["arm0"], pn["arm0"]),
                         ("armB", pc["armB"], pn["armB"]), ("armX", x, n)):
        print("  " + show(name, cx, cn))
    ps = {"X0": contrast("X0", x, n, pc["arm0"], pn["arm0"], "arm0"),
          "XB": contrast("XB", x, n, pc["armB"], pn["armB"], "armB")}
    adj = holm(ps)
    for h, (p_adj, rej) in adj.items():
        print(f"  {h}: Holm p = {p_adj:.4f}  {'REJECTED' if rej else 'not rejected'}")
    r0, rb = adj["X0"][1], adj["XB"][1]
    reading = ("L: a long off-topic document is enough" if r0 and not rb else
               "C: the drop needs D*'s content" if rb and not r0 else
               "inconclusive at n = 8 (no directional claim)")
    print(f"  READING (prereg/expG.md): {reading}")

    # ---- SECONDARY, EXPLORATORY -------------------------------------------
    print("\nSECONDARY -- EXPLORATORY")
    by = {}
    for r in phys:
        by.setdefault(("armX", r["k"]), []).append(legs(r))
    for r in rows(EXP3_PHYS):
        if r["arm"] in ("arm0", "armB"):
            by.setdefault((r["arm"], r["k"]), []).append(legs(r))
    maj = majority(by)
    for arm in ("arm0", "armB", "armX"):
        v = [s for (ar, _), s in maj.items() if ar == arm]
        print(f"  majority-of-3, {show(arm, sum(v), len(v)) if v else arm + ' (incomplete)'}")
    mx = [s for (ar, _), s in maj.items() if ar == "armX"]
    for c in ("arm0", "armB"):
        v = [s for (ar, _), s in maj.items() if ar == c]
        if mx and v and len(v) == 8:
            contrast(f"maj X vs {c}", sum(mx), len(mx), sum(v), len(v), c)

    xs = [v for v in rep0.values() if v is not None]
    for c in ("arm0", "armB"):
        cs = [v for (ar, _), v in pilot.items() if ar == c]
        if xs:
            print(f"  mean trips rep 0: armX {statistics.mean(xs):.2f} vs {c} "
                  f"{statistics.mean(cs):.2f}  MWU p = {mwu(xs, cs):.4f}")

    # The kinematic comparators are re-run by the same `demo/run.py` into the
    # same CSV (deterministic at seed 0), NOT read from demo/out/results.csv,
    # which predates the corrected world (README 2026-09-30).
    kin = rows(EXPG / "kinematic.csv")
    kx = [int(r["legs"]) for r in kin if r["arm"] == "armX"]
    pk = kin
    if kx:
        kxs = sum(v >= SUCCESS_LEGS for v in kx)
        print(f"  kinematic (seed 0), {show('armX', kxs, len(kx))}")
        for c in ("arm0", "armB"):
            cs = [int(r["legs"]) for r in pk if r["arm"] == c and r["seed"] == "0"]
            ccount = sum(v >= SUCCESS_LEGS for v in cs)
            contrast(f"kin X vs {c}", kxs, len(kx), ccount, len(cs), c)

    if phys:
        print("\n  per-episode means (armX physics):")
        for col in ("contacts", "path_m", "track_v_med", "track_wz_med",
                    "step_ms_med", "step_ms_p95"):
            vals = [float(r[col]) for r in phys if r.get(col) not in ("", None)]
            if vals:
                print(f"    {col:13} mean {statistics.mean(vals):9.3f}  "
                      f"median {statistics.median(vals):9.3f}  (n={len(vals)})")


if __name__ == "__main__":
    main()
