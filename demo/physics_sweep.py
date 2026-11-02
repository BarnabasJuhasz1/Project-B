"""
demo/physics_sweep.py

EVERY CONTROLLER, UNDER PHYSICS. The arm comparison that is actually about the
robot rather than about an idealised unicycle.

⚠️ **WHY THIS EXISTS.** The kinematic fast loop is NOT a valid proxy here.
Spot-checking five controllers, THREE flipped verdict:

    arm0 k5   kinematic 2/4  ->  physics 4/4
    armB k1   kinematic 4/4  ->  physics 2/4
    (base)    kinematic 2/4  ->  physics 3/4

So every kinematic number in demo/README.md is a statement about a unicycle, and
the arm comparison has to be re-made with the frozen locomotion policy in the
loop before it means anything about this robot.

⚠️ **ONE CONTROLLER PER PROCESS, STRICTLY SEQUENTIAL.** Isaac Sim's `World` is a
singleton and 8 of the 16 generated controllers bound their planning with a
WALL-CLOCK budget, so two sims at once would change the behaviour of the very
runs being measured. This script therefore SHELLS OUT once per controller and
waits. ~4 min each, ~70 min for the set.

It resumes: a controller whose row is already in the CSV is skipped, so a killed
sweep keeps its work.

    python demo/physics_sweep.py
    python demo/physics_sweep.py --out demo/out/physics.csv
"""
import argparse
import csv
import json
import pathlib
import re
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

PY = sys.executable
RESULT_RE = re.compile(
    r"RESULT\s+(?P<name>\S+):\s+legs\s+(?P<legs>\d+)/(?P<total>\d+)\s+"
    r"times\s+(?P<times>\[[^\]]*\])\s+t=(?P<t>[\d.]+)")
FIELDS = ["arm", "k", "legs", "total", "leg_times", "sim_seconds",
          "wall_seconds", "note"]


def run_one(path):
    """One controller in its own process. Returns a row dict."""
    t0 = time.time()
    proc = subprocess.run(
        [PY, str(REPO / "demo" / "record_isaac.py"), str(path), "--no-record"],
        cwd=str(REPO), capture_output=True, text=True, timeout=1800)
    wall = round(time.time() - t0, 1)
    m = RESULT_RE.search(proc.stdout)
    if not m:
        tail = (proc.stdout or proc.stderr).strip().splitlines()[-3:]
        return {"legs": None, "total": None, "leg_times": "", "sim_seconds": "",
                "wall_seconds": wall,
                "note": "NO RESULT LINE: " + " | ".join(tail)[:200]}
    raised = "controller raised" in proc.stdout
    return {"legs": int(m["legs"]), "total": int(m["total"]),
            "leg_times": m["times"], "sim_seconds": float(m["t"]),
            "wall_seconds": wall,
            "note": "controller raised" if raised else ""}


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(REPO / "demo" / "out" / "physics.csv"))
    a = ap.parse_args(argv)

    jobs = []
    for arm in ("arm0", "armB"):
        for p in sorted((REPO / "demo" / "arms" / arm).glob("k*.py")):
            jobs.append((arm, p.stem, p))
    jobs.append(("(base)", "reference",
                 REPO / "generate" / "base" / "base_controller.py"))

    out_path = pathlib.Path(a.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    done = {}
    if out_path.exists():
        with out_path.open() as fh:
            for row in csv.DictReader(fh):
                done[(row["arm"], row["k"])] = row
        print(f"[phys] resuming: {len(done)} rows already in {out_path.name}")

    rows = []
    for arm, k, path in jobs:
        if (arm, k) in done:
            rows.append(done[(arm, k)])
            print(f"[phys] {arm:7} {k:10} (cached) legs {done[(arm,k)]['legs']}")
            continue
        print(f"[phys] {arm:7} {k:10} running...", flush=True)
        row = {"arm": arm, "k": k, **run_one(path)}
        rows.append(row)
        print(f"[phys] {arm:7} {k:10} legs {row['legs']}/{row['total']}  "
              f"times {row['leg_times']}  ({row['wall_seconds']}s wall) "
              f"{row['note']}", flush=True)
        with out_path.open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
            w.writeheader(); w.writerows(rows)

    print("\n" + "=" * 64)
    print(f"{'arm':10}{'n':>4}{'mean legs':>12}{'solved block':>15}")
    print("-" * 64)
    per_arm = {}
    for r in rows:
        if r["legs"] is not None:
            per_arm.setdefault(r["arm"], []).append(int(r["legs"]))
    for arm, v in per_arm.items():
        solved = sum(1 for x in v if x >= 3)
        print(f"{arm:10}{len(v):>4}{sum(v)/len(v):>12.2f}"
              f"{f'{solved}/{len(v)}':>15}")
    print("=" * 64)
    print("⚠ UNDER PHYSICS, frozen locomotion policy in the loop. Still NOT "
          "reportable\n  under PRE-REGISTRATION.md: one hand-authored world, "
          "not in any split.")
    print(f"-> {out_path}")


if __name__ == "__main__":
    main()
