"""
demo/baselines/run_expB.py

MINDFIRE EXP B: the two hand-written reference baselines, kinematic and under
physics, on exactly the harness the generated controllers were scored on.
Pre-registered in `prereg/expB.md`.

    python demo/baselines/run_expB.py kinematic        # 2 baselines x 3 reps, ~1.5 min
    python demo/baselines/run_expB.py physics --dry-run
    flock /tmp/mindfire-isaac.lock python demo/baselines/run_expB.py physics
    python demo/baselines/run_expB.py report --v2 ../Repo/demo/out/v2/physics.csv

⚠️ **THE BASELINES ARE NOT AN ARM.** They live in demo/baselines/, never in
demo/arms/, and nothing here writes to demo/out/v2/ or demo/out/physics.csv.
Output: demo/out/expB/{kinematic,physics}.csv and demo/out/expB/traces/.

⚠️ **SAME HARNESS, SAME CONDITIONS.** Physics episodes go through
`demo/record_isaac.py` via `physics_sweep.run_one` -- the identical runner,
world, door schedule, 180 s budget, analytic lidar and frozen policy -- with
Exp 3's three rep conditions (`demo/exp3.REP_CONDITIONS`): rep 0 the original
start pose, reps 1-2 the same perturbed start poses and seeds every Exp 3
controller got.

⚠️ **PHYSICS REFUSES TO RUN** unless `prereg/expB.md` is committed and clean
and neither baseline has changed since that commit. The commit hash goes in
every row. Before EACH episode `powerprofilesctl get` must say `performance`;
otherwise the run stops at that episode boundary (exit 3) and resumes later
from the CSV. Only one Isaac Sim process may run at a time: start this under
`flock /tmp/mindfire-isaac.lock`.

SIM-FREE except for the subprocesses `physics` starts.
"""
import argparse
import csv
import json
import os
import pathlib
import statistics
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(REPO))

from demo import exp3

# ---- below this line is data, not code.

BASELINES = (("dstar_lite", HERE / "dstar_lite.py"),
             ("astar_replan", HERE / "astar_replan.py"))
PREREG = "prereg/expB.md"
OUT = REPO / "demo" / "out" / "expB"
REQUIRED_PROFILE = "performance"
PLAN_EVERY = 10          # must equal the shared block's PLAN_EVERY (tested)
PHYS_FIELDS = ["order", "baseline", "rep", "seed", "dx", "dy", "dyaw",
               "legs", "total", "success", "leg_times", "sim_seconds",
               "wall_seconds", "contacts", "path_m",
               "track_v_med", "track_v_p95", "track_wz_med",
               "step_ms_med", "step_ms_p95", "step_ms_max",
               "plan_ms_med", "plan_ms_p95", "plan_ms_max", "plan_steps",
               "nonplan_ms_med", "load1_before", "load1_mean", "load1_max",
               "power_profile", "controller_sha", "prereg_commit", "git_head",
               "trace", "note"]
KIN_FIELDS = ["baseline", "rep", "seed", "dx", "dy", "dyaw", "legs",
              "success", "leg_times", "duration", "collisions", "path_m",
              "plans", "no_path", "errors", "tree_inits",
              "plan_ms_med", "plan_ms_p95", "plan_ms_max",
              "expanded_med", "expanded_p95", "expanded_max"]

# ---- code.


def git(*args):
    return subprocess.run(["git", *args], cwd=str(REPO), text=True,
                          capture_output=True).stdout.strip()


def power_profile():
    """`powerprofilesctl get`, run without the launcher's PYTHONHOME/PATH (the
    tool is a Python script and dies under them)."""
    env = {k: v for k, v in os.environ.items()
           if k not in ("PYTHONHOME", "PYTHONPATH")}
    try:
        return subprocess.run(["powerprofilesctl", "get"], text=True, env=env,
                              capture_output=True, timeout=20).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def q(values, p):
    v = sorted(values)
    return v[min(len(v) - 1, int(round(p * (len(v) - 1))))] if v else None


def rnd(v, n=2):
    return None if v is None else round(float(v), n)


def plan_step_rows(rows):
    """Indices of the trace rows on which the controller planned. The shared
    block plans when its tick counter is 0 mod PLAN_EVERY, and the counter
    restarts at 0 on the first step a new goal is seen -- which is the first
    row of each leg in the trace."""
    out, leg_start, prev_leg = [], 0, None
    for i, r in enumerate(rows):
        if r["leg"] != prev_leg:
            leg_start, prev_leg = i, r["leg"]
        if (i - leg_start) % PLAN_EVERY == 0:
            out.append(i)
    return out


def plan_timing(trace_path):
    rows = exp3.read_trace(trace_path)
    plan = set(plan_step_rows(rows))
    pm = [rows[i]["step_ms"] for i in sorted(plan)]
    other = [r["step_ms"] for i, r in enumerate(rows) if i not in plan]
    return {"plan_ms_med": rnd(statistics.median(pm)) if pm else None,
            "plan_ms_p95": rnd(q(pm, 0.95)), "plan_ms_max": rnd(max(pm, default=None)),
            "plan_steps": len(pm),
            "nonplan_ms_med": rnd(statistics.median(other)) if other else None}


def read_rows(path):
    if not pathlib.Path(path).exists():
        return []
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def write_rows(path, rows, fields, key):
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = pathlib.Path(str(path) + ".tmp")
    with tmp.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(sorted(rows, key=key))
    os.replace(tmp, path)


# ---- kinematic


def kinematic(a):
    from demo.rollout import load_controller, rollout
    rows = []
    for name, path in BASELINES:
        cls = load_controller(path)
        for rep in range(exp3.N_REPS):
            seed, dx, dy, dyaw = exp3.rep_condition(rep)
            ctrl = cls()
            r = rollout(ctrl, seed=seed, start_offset=(dx, dy, dyaw))
            st = ctrl.stats
            inits = [getattr(p, "inits", None) for p in ctrl.planners.values()]
            rows.append({
                "baseline": name, "rep": rep, "seed": seed, "dx": dx, "dy": dy,
                "dyaw": dyaw, "legs": r["legs_completed"],
                "success": exp3.is_success(r["legs_completed"]),
                "leg_times": json.dumps(r["leg_times"]),
                "duration": r["duration"], "collisions": r["collisions"],
                "path_m": r["path_length"], "plans": st["plans"],
                "no_path": st["no_path"], "errors": st["errors"],
                "tree_inits": json.dumps(inits) if inits[0] is not None else "",
                "plan_ms_med": rnd(statistics.median(st["plan_ms"])),
                "plan_ms_p95": rnd(q(st["plan_ms"], 0.95)),
                "plan_ms_max": rnd(max(st["plan_ms"])),
                "expanded_med": statistics.median(st["expanded"]),
                "expanded_p95": q(st["expanded"], 0.95),
                "expanded_max": max(st["expanded"])})
            print(f"[expB] kinematic {name:12} rep {rep}: legs {r['legs_completed']}/4 "
                  f"{r['leg_times']} collisions {r['collisions']} "
                  f"plan ms med {rows[-1]['plan_ms_med']} max {rows[-1]['plan_ms_max']}",
                  flush=True)
    out = pathlib.Path(a.out) / "kinematic.csv"
    write_rows(out, rows, KIN_FIELDS, key=lambda r: (r["baseline"], int(r["rep"])))
    print(f"-> {out}")


# ---- physics


def frozen_commit():
    """The commit that froze prereg/expB.md, after checking it is clean and the
    baselines have not moved since. Raises SystemExit otherwise."""
    commit = git("log", "-1", "--format=%H", "--", PREREG)
    if not commit:
        raise SystemExit(f"[expB] {PREREG} is not committed; physics refuses to run")
    if git("status", "--porcelain", "--", PREREG):
        raise SystemExit(f"[expB] {PREREG} has uncommitted changes")
    moved = git("diff", "--name-only", commit, "--",
                *[str(p.relative_to(REPO)) for _n, p in BASELINES])
    if moved:
        raise SystemExit(f"[expB] baselines changed since {commit[:12]}: {moved}")
    return commit


def physics(a):
    from demo import physics_sweep
    order = exp3.job_order([(name, "ref", path) for name, path in BASELINES])
    out = pathlib.Path(a.out) / "physics.csv"
    done = {(r["baseline"], r["rep"]): r for r in read_rows(out)}
    todo = [j for j in order if (j[0], str(j[3])) not in done]
    print(f"[expB] {len(order)} episodes planned, {len(order) - len(todo)} done, "
          f"{len(todo)} to run", flush=True)
    if a.dry_run:
        for i, (name, _k, _p, rep) in enumerate(order):
            print(f"  {i} {name:12} rep {rep} {exp3.rep_condition(rep)}")
        return 0
    commit = frozen_commit()
    head = git("rev-parse", "HEAD")
    print(f"[expB] prereg commit {commit}  HEAD {head}", flush=True)
    trace_dir = out.parent / "traces"
    trace_dir.mkdir(parents=True, exist_ok=True)
    rows = list(done.values())
    for i, (name, _k, path, rep) in enumerate(order):
        if (name, str(rep)) in done:
            continue
        prof = power_profile()
        if prof != REQUIRED_PROFILE:
            print(f"[expB] PAUSED before #{i} ({name} rep {rep}): "
                  f"powerprofilesctl get = {prof!r}, need {REQUIRED_PROFILE!r}. "
                  f"Resumable.", flush=True)
            return 3
        seed, dx, dy, dyaw = exp3.rep_condition(rep)
        trace = trace_dir / f"{name}__r{rep}.csv.gz"
        print(f"[expB] #{i} {name:12} rep {rep} (profile {prof}, load "
              f"{os.getloadavg()[0]:.2f}) running...", flush=True)
        row = {"order": i, "baseline": name, "rep": rep, "seed": seed,
               "dx": dx, "dy": dy, "dyaw": dyaw,
               **physics_sweep.run_one(path, rep, trace)}
        after = power_profile()
        row["power_profile"] = prof if after == prof else f"{prof}->{after}"
        row["prereg_commit"], row["git_head"] = commit, head
        if trace.exists():
            row.update(plan_timing(trace))
            row["trace"] = str(trace.relative_to(out.parent))
        rows.append(row)
        write_rows(out, rows, PHYS_FIELDS, key=lambda r: int(r["order"]))
        print(f"[expB] #{i} {name:12} rep {rep}: legs {row['legs']}/4 "
              f"times {row['leg_times']} contacts {row.get('contacts')} "
              f"plan ms med {row.get('plan_ms_med')} max {row.get('plan_ms_max')} "
              f"({row['wall_seconds']} s wall) {row.get('note') or ''}", flush=True)
    print(f"-> {out}")
    return 0


# ---- report


def report(a):
    phys = read_rows(pathlib.Path(a.out) / "physics.csv")
    kin = read_rows(pathlib.Path(a.out) / "kinematic.csv")
    print("## Kinematic (demo/rollout.py)\n")
    print("| baseline | rep | legs | leg times s | collisions | plan ms med / max |")
    print("|---|---|---|---|---|---|")
    for r in kin:
        print(f"| {r['baseline']} | {r['rep']} | {r['legs']} | {r['leg_times']} | "
              f"{r['collisions']} | {r['plan_ms_med']} / {r['plan_ms_max']} |")
    print("\n## Physics (demo/record_isaac.py)\n")
    print("| baseline | rep | legs | leg times s | contacts | track_v med | "
          "track_wz med | plan ms med / p95 / max | profile |")
    print("|---|---|---|---|---|---|---|---|---|")
    for r in sorted(phys, key=lambda r: (r["baseline"], int(r["rep"]))):
        print(f"| {r['baseline']} | {r['rep']} | {r['legs']} | {r['leg_times']} | "
              f"{r['contacts']} | {r['track_v_med']} | {r['track_wz_med']} | "
              f"{r['plan_ms_med']} / {r['plan_ms_p95']} / {r['plan_ms_max']} | "
              f"{r['power_profile']} |")
    for name, _p in BASELINES:
        mine = [r for r in phys if r["baseline"] == name and r["legs"] != ""]
        s = sum(exp3.is_success(r["legs"]) for r in mine)
        print(f"\n{name}: {s}/{len(mine)} episodes all four trips "
              f"({'SUCCEEDS' if s * 2 > len(mine) and len(mine) == 3 else 'fails' if len(mine) == 3 else 'incomplete'} by majority of 3)")
    if a.v2:
        v2 = [r for r in read_rows(a.v2) if r["legs"] not in ("", None)]
        print(f"\n## Exp 3 arms for comparison ({a.v2}, {len(v2)} episodes)\n")
        print("| arm | episodes all-4 | controllers maj-of-3 | mean legs | "
              "median contacts | median track_v med |")
        print("|---|---|---|---|---|---|")
        for arm in sorted({r["arm"] for r in v2}):
            mine = [r for r in v2 if r["arm"] == arm]
            per = {}
            for r in mine:
                per.setdefault(r["k"], []).append(exp3.is_success(r["legs"]))
            full = {k: v for k, v in per.items() if len(v) == exp3.N_REPS}
            maj = sum(sum(v) * 2 > len(v) for v in full.values())
            eps = sum(exp3.is_success(r["legs"]) for r in mine)
            legs = statistics.mean(int(r["legs"]) for r in mine)
            con = statistics.median(float(r["contacts"]) for r in mine
                                    if r["contacts"] not in ("", None))
            tv = statistics.median(float(r["track_v_med"]) for r in mine
                                   if r["track_v_med"] not in ("", None))
            print(f"| {arm} | {eps}/{len(mine)} | {maj}/{len(full)} "
                  f"(of {len(per)} with all reps) | {legs:.2f} | {con:g} | {tv:.3f} |")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("what", choices=("kinematic", "physics", "report"))
    ap.add_argument("--out", default=str(OUT))
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--v2", default=None,
                    help="Exp 3's demo/out/v2/physics.csv, for the comparison")
    a = ap.parse_args(argv)
    return {"kinematic": kinematic, "physics": physics, "report": report}[a.what](a) or 0


if __name__ == "__main__":
    sys.exit(main())
