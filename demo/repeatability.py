"""
demo/repeatability.py

⚠️ HOW REPEATABLE IS A PATROL SCORE? Run every controller `--reps` times and
report the spread.

THIS EXISTS BECAUSE THE SCORE TURNED OUT NOT TO BE DETERMINISTIC. 8 of the 15
generated controllers bound their per-step planning with a WALL-CLOCK budget
(`time.perf_counter()`), so the number of planner iterations they get depends on
what else the machine is doing. The same controller on the same seed scored 4/4
and then 3/4 on two runs, and the first k=8 evaluation was taken while a
generation call was still running.

⚠️ **THE EXPOSURE IS NOT EVEN ACROSS ARMS** -- 6 of 7 armB controllers use a
wall-clock budget against 2 of 8 for arm0 -- so machine load is not noise that
cancels, it is a bias with a direction. Any arm comparison has to be taken on a
quiet machine and reported with this spread beside it.

    python demo/repeatability.py --reps 3
"""
import sys, pathlib, argparse, statistics, csv
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from demo.rollout import load_controller, rollout
from demo.run import discover

HERE = pathlib.Path(__file__).resolve().parent


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--csv", default=str(HERE / "out" / "repeatability.csv"))
    a = ap.parse_args(argv)

    rows = discover()
    print(f"[rep] {len(rows)} controllers x {a.reps} reps, same seed each time")
    print(f"{'controller':22}{'legs per rep':>22}{'mean':>8}{'stable':>9}")
    print("-" * 61)
    out, per_arm = [], {}
    for arm, k, path in rows:
        legs = []
        for _ in range(a.reps):
            try:
                r = rollout(load_controller(path)(), seed=0)
                legs.append(r["legs_completed"])
            except Exception:
                legs.append(-1)
        stable = len(set(legs)) == 1
        mean = statistics.mean(legs)
        print(f"{arm + ' ' + k:22}{str(legs):>22}{mean:>8.2f}"
              f"{('yes' if stable else 'NO'):>9}", flush=True)
        out.append({"arm": arm, "k": k, "legs": legs, "mean": mean,
                    "stable": stable})
        per_arm.setdefault(arm, []).extend(legs)

    print("\n" + "=" * 61)
    print(f"{'arm':10}{'n (ctrl x rep)':>16}{'mean legs':>12}{'solved':>10}")
    print("-" * 61)
    for arm, v in per_arm.items():
        print(f"{arm:10}{len(v):>16}{statistics.mean(v):>12.2f}"
              f"{f'{sum(1 for x in v if x >= 3)}/{len(v)}':>10}")
    unstable = [f"{r['arm']} {r['k']}" for r in out if not r["stable"]]
    print("=" * 61)
    print(f"⚠ unstable across reps: {unstable or 'none'}")

    p = pathlib.Path(a.csv); p.parent.mkdir(parents=True, exist_ok=True)
    with p.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["arm", "k", "legs", "mean", "stable"])
        w.writeheader(); w.writerows(out)
    print(f"-> {p}")


if __name__ == "__main__":
    main()
