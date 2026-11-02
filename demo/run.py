"""
demo/run.py

EVALUATE EVERY GENERATED CONTROLLER ON THE PATROL, AND PRINT THE TABLE.

Paired by construction: there is one world and one seed set, and every
controller sees exactly the same four legs with the door shutting at the same
point. A difference between arms is the controller.

⚠️ **KINEMATIC. NOT A BENCHMARK PATH.** The frozen locomotion policy is absent.
This decides which controllers are worth recording under Isaac Sim; it is not a
reportable number and nothing here may be pooled with anything in `results/`.

⚠️ **A CRASH IS A RESULT, NOT AN ERROR.** A controller that raises scores the
legs it finished and the exception is printed. Quietly dropping it would turn k
into a best-of.

    $ python demo/run.py
    $ python demo/run.py --plot          # + one contact sheet per controller
    $ python demo/run.py --seeds 0,1,2   # the rollout is deterministic given a
                                         # seed; only a controller that draws
                                         # from rng varies across them
"""
import sys, pathlib, argparse, statistics, csv
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from demo.rollout import load_controller, rollout, LEG_TARGETS, TIME_LIMIT
from demo.render import ARMS

HERE = pathlib.Path(__file__).resolve().parent


def discover():
    """(arm, k, path) for every generated controller, plus the frozen base as a
    reference row. The base is NOT an arm -- it was written for a single-trip
    task and is here only to say what the substrate does."""
    rows = []
    for arm in ARMS:
        for p in sorted((HERE / "arms" / arm).glob("k*.py")):
            rows.append((arm, p.stem, p))
    base = HERE.parent / "generate" / "base" / "base_controller.py"
    if base.is_file():
        rows.append(("(base)", "reference", base))
    return rows


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--plot", action="store_true")
    ap.add_argument("--seeds", default="0")
    ap.add_argument("--csv", default="demo/out/results.csv")
    a = ap.parse_args(argv)
    seeds = [int(s) for s in a.seeds.split(",")]

    rows = discover()
    if not rows:
        raise SystemExit("no controllers found — run demo/generate.py first")

    print(f"[run] {len(rows)} controllers x {len(seeds)} seed(s), "
          f"budget {TIME_LIMIT}s, {len(LEG_TARGETS)} legs")
    out, per_arm = [], {}
    for arm, k, path in rows:
        for seed in seeds:
            try:
                cls = load_controller(path)
            except Exception as exc:                 # refused by the allowlist
                print(f"  {arm:8} {k:10} REFUSED: {exc}")
                out.append({"arm": arm, "k": k, "seed": seed, "legs": 0,
                            "note": f"refused: {exc}"})
                continue
            r = rollout(cls(), seed=seed, record=a.plot)
            note = r["crashed"] or ""
            out.append({"arm": arm, "k": k, "seed": seed,
                        "legs": r["legs_completed"],
                        "leg_times": r["leg_times"],
                        "collisions": r["collisions"],
                        "path_length": r["path_length"], "note": note})
            per_arm.setdefault(arm, []).append(r["legs_completed"])
            flag = "  <-- SOLVED THE BLOCK" if r["legs_completed"] >= 3 else ""
            print(f"  {arm:8} {k:10} seed{seed}  legs {r['legs_completed']}/4"
                  f"  times {r['leg_times']}  coll {r['collisions']}"
                  f"  {r['path_length']}m {note}{flag}")
            if a.plot:
                from demo.plot import draw
                draw(r, HERE / "out" / f"{arm}-{k}-seed{seed}",
                     title=f"{arm} {k} (seed {seed})")

    print("\n" + "=" * 62)
    print(f"{'arm':10}{'n':>4}{'mean legs':>12}{'solved block':>15}")
    print("-" * 62)
    for arm in list(ARMS) + ["(base)"]:
        v = per_arm.get(arm)
        if not v:
            continue
        solved = sum(1 for x in v if x >= 3)
        print(f"{arm:10}{len(v):>4}{statistics.mean(v):>12.2f}"
              f"{f'{solved}/{len(v)}':>15}")
    print("=" * 62)
    print("⚠ kinematic, not reportable. 'solved block' = reached the goal on "
          "leg 2,\n  which is the leg where the short route is shut.")

    csv_path = pathlib.Path(a.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    # A fixed field list: a REFUSED row carries fewer keys than a scored one,
    # and taking the header from the first row crashed on exactly that.
    fields = ["arm", "k", "seed", "legs", "leg_times", "collisions",
              "path_length", "note"]
    with csv_path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows([{f: row.get(f, "") for f in fields} for row in out])
    print(f"-> {csv_path}")


if __name__ == "__main__":
    main()
