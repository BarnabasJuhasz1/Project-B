"""
demo/exp_e.py

EXP E (plan item E7a): THE KINEMATIC ROLLOUT WITH THE GAIT'S MEASURED TRACKING
ERROR INJECTED. Pre-registered in prereg/expE.md, which was committed before
this script was first run.

    python demo/exp_e.py --fit      # fit the noise model from Exp 3's traces
    python demo/exp_e.py            # run, then print + write the summary

Design (prereg/expE.md):
* controllers: the 16 pilot controllers, arm0 and armB k0..k7, untouched;
  the frozen base as a reference row, outside every test.
* reps 0-2: the Exp 3 conditions (scenario seed + start offset, demo/exp3.py),
  noise seed NOISE_SEED + rep -- so every controller gets the same noise
  stream in a given rep, and the design stays paired.
* two conditions: `noise` (AR(1) residuals fitted by demo/tracking.py on the
  Exp 3 traces) and `none` (the same reps, no noise) as the reference.
* success = all four trips; primary = majority (>= 2 of 3) per controller;
  Fisher exact two-sided, arm0 vs armB, in the `noise` condition.

SIM-FREE. Executes model-written controllers through rollout.load_controller's
import gate, as demo/run.py does.
"""
import argparse
import csv
import json
import os
import pathlib
import statistics
import sys
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

import numpy as np

from demo import exp3, tracking
from demo.rollout import load_controller, rollout

# ---- below this line is data, not code.

NOISE_SEED = 5000
ARMS = ("arm0", "armB")
KS = tuple(f"k{i}" for i in range(8))
V2 = REPO / "demo" / "out" / "v2"
MODEL = V2 / "expE_noise_model.json"
OUT = V2 / "expE.csv"
FIELDS = ["condition", "arm", "k", "rep", "seed", "noise_seed", "legs",
          "success", "leg_times", "collisions", "path_length", "crashed",
          "wall_s", "load1"]

# ---- code.


def physics_traces():
    """Every non-quarantined Exp 3 trace, all arms and the base."""
    with open(V2 / "physics.csv") as fh:
        return [V2 / r["trace"] for r in csv.DictReader(fh)
                if r["trace"] and (V2 / r["trace"]).exists()]


def controllers(base=True):
    out = [(arm, k, REPO / "demo" / "arms" / arm / f"{k}.py")
           for arm in ARMS for k in KS]
    if base:
        out.append(("(base)", "reference",
                    REPO / "generate" / "base" / "base_controller.py"))
    return out


def run(model):
    rows = []
    for cond in ("noise", "none"):
        for arm, k, path in controllers():
            for rep in range(exp3.N_REPS):
                seed, dx, dy, dyaw = exp3.rep_condition(rep)
                noise = (tracking.AR1Noise(model, np.random.default_rng(
                    NOISE_SEED + rep)) if cond == "noise" else None)
                t0 = time.perf_counter()
                r = rollout(load_controller(path)(), seed=seed,
                            start_offset=(dx, dy, dyaw), noise=noise)
                row = {"condition": cond, "arm": arm, "k": k, "rep": rep,
                       "seed": seed, "noise_seed": NOISE_SEED + rep
                       if noise else "", "legs": r["legs_completed"],
                       "success": exp3.is_success(r["legs_completed"]),
                       "leg_times": json.dumps(r["leg_times"]),
                       "collisions": r["collisions"],
                       "path_length": r["path_length"],
                       "crashed": r["crashed"] or "",
                       "wall_s": round(time.perf_counter() - t0, 1),
                       "load1": round(os.getloadavg()[0], 2)}
                rows.append(row)
                print(f"[expE] {cond:5} {arm:7} {k:10} rep {rep} legs "
                      f"{row['legs']}/4 {row['crashed']}", flush=True)
    with OUT.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS)
        w.writeheader(); w.writerows(rows)
    return rows


def majority(rows, cond, arm):
    per = {}
    for r in rows:
        if r["condition"] == cond and r["arm"] == arm:
            per.setdefault(r["k"], []).append(str(r["success"]) == "True")
    return {k: sum(v) * 2 > len(v) for k, v in per.items()}, per


def physics_majority():
    per = {}
    with open(V2 / "physics.csv") as fh:
        for r in csv.DictReader(fh):
            per.setdefault((r["arm"], r["k"]), []).append(
                exp3.is_success(r["legs"]))
    return {key: sum(v) * 2 > len(v) for key, v in per.items()}


def summarise(rows):
    from scipy.stats import fisher_exact
    phys = physics_majority()
    out = ["# Exp E — kinematic rollout with measured tracking noise\n",
           "Pre-registered: prereg/expE.md. Success = all four trips. "
           "Primary = majority of 3 reps per controller, Fisher exact "
           "two-sided, arm0 vs armB, condition `noise`.\n",
           "| condition | arm0 maj-of-3 | armB maj-of-3 | Fisher p | arm0 "
           "episodes | armB episodes | agree with physics maj (16) |",
           "|---|---|---|---|---|---|---|"]
    res = {}
    for cond in ("noise", "none"):
        m0, e0 = majority(rows, cond, "arm0")
        mb, eb = majority(rows, cond, "armB")
        s0, sb = sum(m0.values()), sum(mb.values())
        p = fisher_exact([[s0, len(m0) - s0], [sb, len(mb) - sb]])[1]
        ep0 = sum(sum(v) for v in e0.values()); n0 = sum(len(v) for v in e0.values())
        epb = sum(sum(v) for v in eb.values()); nb = sum(len(v) for v in eb.values())
        agree = sum(1 for arm, m in (("arm0", m0), ("armB", mb))
                    for k, v in m.items() if phys.get((arm, k)) == v)
        res[cond] = (s0, sb, p)
        out.append(f"| {cond} | {s0}/{len(m0)} | {sb}/{len(mb)} | {p:.3f} | "
                   f"{ep0}/{n0} | {epb}/{nb} | {agree}/16 |")
    ph0 = sum(v for (a, _), v in phys.items() if a == "arm0")
    phb = sum(v for (a, _), v in phys.items() if a == "armB")
    pp = fisher_exact([[ph0, 8 - ph0], [phb, 8 - phb]])[1]
    out.append(f"| physics (Exp 3) | {ph0}/8 | {phb}/8 | {pp:.3f} | | | 16/16 |")
    s0, sb, p = res["noise"]
    passed = s0 > sb and p < 0.05
    out.append(f"\n**Pass criterion (prereg): arm0 > armB and p < 0.05 in "
               f"`noise` → {'MET' if passed else 'NOT MET'}.**\n")
    out.append("Per controller, legs in rep 0/1/2:\n")
    for cond in ("noise", "none"):
        for arm in ARMS + ("(base)",):
            cells = []
            for k in KS + ("reference",):
                v = [str(r["legs"]) for r in rows if r["condition"] == cond
                     and r["arm"] == arm and r["k"] == k]
                if v:
                    cells.append(f"{k} " + "/".join(v))
            out.append(f"- {cond} **{arm}**: " + ", ".join(cells))
    text = "\n".join(out)
    print(text)
    (V2 / "EXPE.md").write_text(text + "\n")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--fit", action="store_true",
                    help="fit and write the noise model, then stop")
    ap.add_argument("--summary-only", action="store_true")
    a = ap.parse_args(argv)
    if a.fit:
        model = tracking.fit_noise_model(physics_traces())
        MODEL.write_text(json.dumps(model, indent=1))
        print(json.dumps(model, indent=1))
        return
    if a.summary_only:
        with OUT.open() as fh:
            summarise(list(csv.DictReader(fh)))
        return
    model = json.loads(MODEL.read_text())
    summarise(run(model))


if __name__ == "__main__":
    main()
