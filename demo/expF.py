"""
demo/expF.py

MINDFIRE EXP F -- THE SAME 16 CONTROLLERS ON A SECOND MAZE (review item E2).
The pre-registration is `prereg/expF.md`; this file is the procedure it names.

    python demo/expF.py check                    # maze2's guarantees, sim-free
    python demo/export_demo_world.py --maze maze2   # -> scenes/demo-maze2.usd
    python demo/expF.py scene-check              # the USD against demo/maze2.py
    python demo/expF.py sanity                   # base controller, both simulators
    python demo/expF.py kinematic                # 16 x 3 reps, unicycle
    python demo/expF.py physics                  # 16 x 3 reps, Isaac Sim, resumable
    python demo/expF.py physics --dry-run        # the interleaved order, no boot
    python demo/expF.py report                   # Fisher per arm, both simulators

WHAT IS IDENTICAL TO EXP 3, AND WHY THAT IS THE POINT
----------------------------------------------------
The controllers (demo/arms/{arm0,armB}/k0..k7.py, unchanged, sha256 logged per
episode), the task, the 180 s budget, the door schedule, the reps
(`exp3.REP_CONDITIONS`: rep 0 = seed 0 and no offset, reps 1-2 = seeds 1-2 and
the same start offsets for every controller), the success rule (all four legs),
the interleaved order (`exp3.job_order`, `ORDER_SEED`), the trace format and the
runner (`demo/record_isaac.py`). The only thing that changes is `--maze maze2`.

⚠️ **ONE ISAAC SIM PROCESS ON THE LAPTOP AT A TIME.** This script does not take
the lock itself; run it as

    nohup flock /tmp/mindfire-isaac.lock python demo/expF.py physics > LOG &

⚠️ **POWER PROFILE.** Before every episode `powerprofilesctl get` must say
`performance`; otherwise the sweep stops at the episode boundary (resumable)
and says so. A low-power reboot halved episode speed once (2026-11-02), and
8 of 16 controllers bound their planning by wall clock.

⚠️ **The base controller is NOT in the sweep.** It is the sanity episode only
(`sanity`), so the 16-controller numbers are exactly the pre-registered set.
"""
import argparse
import csv
import hashlib
import json
import os
import pathlib
import platform
import shutil
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from demo import exp3
from demo import physics_sweep as ps

# ---- below this line is data, not code.

MAZE = "maze2"
ARMS = ("arm0", "armB")
OUT = REPO / "demo" / "out" / "expF"
PHYSICS_CSV = OUT / "physics.csv"
KINEMATIC_CSV = OUT / "kinematic.csv"
SANITY_CSV = OUT / "sanity.csv"
REQUIRED_PROFILE = "performance"
MAJORITY = 2                      # of 3 reps, as PREREG-v2 §4

FIELDS = ["order", "sim", "maze", "arm", "k", "rep", "seed", "dx", "dy", "dyaw",
          "legs", "total", "success", "leg_times", "sim_seconds",
          "wall_seconds", "contacts", "path_m",
          "step_ms_med", "step_ms_p95", "step_ms_max",
          "track_v_med", "track_v_p95", "track_wz_med",
          "load1_before", "load1_mean", "load1_max",
          "controller_sha", "power_profile", "trace", "note"]

# ---- code.


def power_profile():
    """`powerprofilesctl get`, or "" if it cannot be read. The Jarvis launcher
    leaks PYTHONHOME/PYTHONPATH and powerprofilesctl is a Python script."""
    env = {k: v for k, v in os.environ.items()
           if k not in ("PYTHONHOME", "PYTHONPATH")}
    exe = shutil.which("powerprofilesctl", path=env.get("PATH")) or "powerprofilesctl"
    try:
        return subprocess.run([exe, "get"], env=env, capture_output=True,
                              text=True, timeout=20).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def sha16(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()[:16]


def controllers():
    """(arm, k, path), the 16, sorted by arm then k. No base."""
    return ps.discover(list(ARMS), None, base=False)


def order(reps=range(exp3.N_REPS)):
    return exp3.job_order(controllers(), reps)


def read_rows(path):
    return ps.read_rows(path) if pathlib.Path(path).exists() else []


def write_rows(path, rows):
    path = pathlib.Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        for r in sorted(rows, key=lambda r: int(r["order"])):
            w.writerow(r)
    os.replace(tmp, path)


def manifest(name, extra=None):
    def git(*cmd):
        return subprocess.run(["git", *cmd], cwd=str(REPO), text=True,
                              capture_output=True).stdout.strip()
    m = {"started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "argv": sys.argv, "maze": MAZE,
         "git_head": git("rev-parse", "HEAD"),
         "git_branch": git("rev-parse", "--abbrev-ref", "HEAD"),
         "git_dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
         "prereg_commit": git("log", "-1", "--format=%H", "--", "prereg/expF.md"),
         "rep_conditions": exp3.REP_CONDITIONS,
         "order_seed": exp3.ORDER_SEED,
         "success_rule": f"legs >= {exp3.LEGS_FOR_SUCCESS}",
         "controllers": {f"{a}/{k}": sha16(p) for a, k, p in controllers()},
         "python": sys.version.split()[0], "host": platform.node(),
         "in_container": os.path.exists("/.dockerenv"), **(extra or {})}
    OUT.mkdir(parents=True, exist_ok=True)
    path = OUT / f"manifest-{name}.json"
    if path.exists():
        path = OUT / f"manifest-{name}.resume-{int(time.time())}.json"
    path.write_text(json.dumps(m, indent=1))
    return m


# ---- the maze check -----------------------------------------------------------

def cmd_check(a):
    from demo import maze2
    r = maze2.verify()
    for k, v in r.items():
        print(f"{k:26} {v}")
    print(f"-> {maze2.plot(OUT / 'maze2.png')}")
    if not r["all_ok"]:
        raise SystemExit("[expF] maze2 FAILS its checks")


def cmd_scene_check(a):
    """The exported USD against demo/mazes.py, block by block. Reads the file
    with pxr only (no SimulationApp), so it is seconds -- but it still runs
    under the Isaac lock, as every scene check does on this laptop.

    ⚠️ It does NOT use UsdGeom.BBoxCache. The cuboids' authored `extent` is
    already scaled and the xform scales it again (a 0.15 m wall read 0.022 m
    thick, a 14 m wall 196 m long), so a bbox check fails a correct scene. The
    rect is the cube's own corners, +-size/2, through its local-to-world
    matrix -- the geometry physics actually collides with. `--maze patrol` runs
    the same check on the first maze's scene, which is how the check itself is
    validated."""
    from pxr import Gf, Usd, UsdGeom
    from demo import mazes
    m = mazes.get(a.maze)
    path = REPO / m["scene"]
    stage = Usd.Stage.Open(str(path))
    tc = Usd.TimeCode.Default()

    def rect_of(prim):
        cubes = [q for q in Usd.PrimRange(prim) if q.IsA(UsdGeom.Cube)]
        if len(cubes) != 1:
            raise RuntimeError(f"{prim.GetPath()}: {len(cubes)} cubes")
        cube = cubes[0]
        h = float(UsdGeom.Cube(cube).GetSizeAttr().Get()) / 2.0
        M = UsdGeom.Xformable(cube).ComputeLocalToWorldTransform(tc)
        pts = [M.Transform(Gf.Vec3d(sx * h, sy * h, sz * h))
               for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
        xs, ys, zs = zip(*[(q[0], q[1], q[2]) for q in pts])
        return (min(xs), min(ys), max(xs), max(ys)), (min(zs), max(zs))

    walls = [p for p in stage.Traverse()
             if p.GetParent() and p.GetParent().GetPath() == "/World/Walls"]
    got = sorted(tuple(round(v, 3) for v in rect_of(p)[0]) for p in walls)
    want = sorted(tuple(round(v, 3) for v in r) for r in m["open"].rects)
    door = stage.GetPrimAtPath("/World/Door")
    door_rect = tuple(round(v, 3) for v in rect_of(door)[0]) if door else None
    checks = {
        "maze": a.maze, "scene": str(path.relative_to(REPO)),
        "size_mb": round(path.stat().st_size / 1e6, 1),
        "n_wall_blocks": len(walls), "n_rects": len(want),
        "ok_walls_match_world": got == want,
        "door_rect": door_rect,
        "ok_door_matches": door_rect == tuple(round(v, 3) for v in m["door"]),
        "ok_home_marker": bool(stage.GetPrimAtPath("/World/home_marker")),
        "ok_goal_marker": bool(stage.GetPrimAtPath("/World/goal_marker")),
    }
    checks["all_ok"] = all(v for k, v in checks.items() if k.startswith("ok_"))
    for k, v in checks.items():
        print(f"{k:22} {v}")
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"scene-check-{a.maze}.json").write_text(json.dumps(checks, indent=1))
    if not checks["all_ok"]:
        if got != want:
            print("missing:", sorted(set(want) - set(got)))
            print("extra:  ", sorted(set(got) - set(want)))
        raise SystemExit("[expF] scene does not match demo/maze2.py")


# ---- kinematic -----------------------------------------------------------------

class _Timed:
    """Wraps a controller and times step(). Outside the rollout's arithmetic,
    so the trajectory is the one rollout() alone would produce."""

    def __init__(self, inner):
        self.inner, self.ms = inner, []

    def reset(self, scenario, rng):
        return self.inner.reset(scenario, rng)

    def step(self, obs):
        t0 = time.perf_counter()
        try:
            return self.inner.step(obs)
        finally:
            self.ms.append((time.perf_counter() - t0) * 1000.0)


def kinematic_episode(path, rep, trace_path=None, maze=MAZE):
    from demo.rollout import load_controller, rollout, LEG_BLOCKED, CONTROL_DT
    seed, dx, dy, dyaw = exp3.rep_condition(rep)
    ctl = _Timed(load_controller(path)())
    t0 = time.time()
    r = rollout(ctl, seed=seed, record=True, start_offset=(dx, dy, dyaw),
                maze=maze)
    wall = round(time.time() - t0, 1)
    tr = r.pop("trace") or []
    # trace tuple: (t, x, y, yaw, leg, vx, vy, wz, gap, touching)
    poses = [(row[1], row[2], row[3]) for row in tr]
    commands = [(row[5], row[6], row[7]) for row in tr]
    errs = exp3.tracking_errors(poses, commands, CONTROL_DT)
    rows = []
    for row, ms, (ev, ew) in zip(tr, ctl.ms, errs):
        t, x, y, yaw, leg, vx, vy, wz, gap, touch = row
        rows.append([t, leg, int(LEG_BLOCKED[min(leg, 3)]), x, y, yaw, vx, vy,
                     wz, gap, touch, ms, ev, ew])
    if trace_path:
        pathlib.Path(trace_path).parent.mkdir(parents=True, exist_ok=True)
        exp3.write_trace(trace_path, rows)
    s = exp3.summarise(rows, CONTROL_DT)
    legs = r["legs_completed"]
    return {"legs": legs, "total": 4, "success": exp3.is_success(legs),
            "leg_times": json.dumps(r["leg_times"]),
            "sim_seconds": r["duration"], "wall_seconds": wall,
            # rollout() counts contact ONSETS exactly as summarise() does
            "contacts": r["collisions"], "path_m": r["path_length"],
            **{k: s[k] for k in ("step_ms_med", "step_ms_p95", "step_ms_max",
                                 "track_v_med", "track_v_p95", "track_wz_med")},
            "controller_sha": sha16(path),
            "note": ("controller raised: " + r["crashed"])[:200]
                    if r.get("crashed") else ""}


def cmd_kinematic(a):
    out = pathlib.Path(a.out or KINEMATIC_CSV)
    jobs = order()
    done = {(r["arm"], r["k"], r["rep"]): r for r in read_rows(out)}
    if not a.dry_run:
        m = manifest("kinematic")
        if not m["prereg_commit"]:
            raise SystemExit("[expF-kin] prereg/expF.md is not committed; "
                             "the 16 controllers run only after it is")
    rows = list(done.values())
    for i, (arm, k, path, rep) in enumerate(jobs):
        if (arm, k, str(rep)) in done:
            continue
        if a.dry_run:
            print(f"  {i:3} {arm} {k} rep {rep}")
            continue
        prof = power_profile()
        if prof != REQUIRED_PROFILE:
            print(f"[expF-kin] PAUSED before #{i}: power profile is {prof!r}. "
                  f"Resumable.", flush=True)
            raise SystemExit(3)
        seed, dx, dy, dyaw = exp3.rep_condition(rep)
        trace = out.parent / "traces-kinematic" / ps.trace_name(arm, k, rep)
        load = round(os.getloadavg()[0], 2)
        row = {"order": i, "sim": "kinematic", "maze": MAZE, "arm": arm, "k": k,
               "rep": rep, "seed": seed, "dx": dx, "dy": dy, "dyaw": dyaw,
               "load1_before": load, "power_profile": prof,
               **kinematic_episode(path, rep, trace)}
        row["trace"] = str(trace.relative_to(out.parent))
        rows.append(row)
        write_rows(out, rows)
        print(f"[expF-kin] #{i:2} {arm} {k} rep {rep}: legs {row['legs']}/4 "
              f"{row['leg_times']} contacts {row['contacts']} "
              f"({row['wall_seconds']}s) {row['note']}", flush=True)
    if not a.dry_run:
        print(f"-> {out}")


# ---- physics -------------------------------------------------------------------

def physics_command(path, rep, trace, maze=MAZE):
    seed, dx, dy, dyaw = exp3.rep_condition(rep)
    return [ps.PY, str(REPO / "demo" / "record_isaac.py"), str(path),
            "--maze", maze, "--no-record", "--seed", str(seed),
            "--start-dx", repr(dx), "--start-dy", repr(dy),
            "--start-dyaw", repr(dyaw), "--trace", str(trace)]


def physics_episode(path, rep, trace, maze=MAZE):
    load_before = round(os.getloadavg()[0], 2)
    t0 = time.time()
    try:
        proc = subprocess.run(physics_command(path, rep, trace, maze),
                              cwd=str(REPO), capture_output=True, text=True,
                              timeout=1800)
        stdout, stderr = proc.stdout, proc.stderr
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout.decode() if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = "TIMEOUT after 1800 s"
    return ps.row_from_output(stdout, stderr, round(time.time() - t0, 1),
                              load_before), stdout


def run_physics(jobs, out, sim_label, dry_run=False):
    # prereg §7: a SIMULATOR failure (no result line) is re-run once, and the
    # re-run is marked; a second failure stays in the file as missing.
    done = {(r["arm"], r["k"], r["rep"]): r for r in read_rows(out)
            if r["legs"] not in ("", None) or r["note"].startswith("RETRY")}
    todo = [j for j in jobs if (j[1][0], j[1][1], str(j[1][3])) not in done]
    print(f"[expF-phys] {len(jobs)} planned, {len(jobs) - len(todo)} done, "
          f"{len(todo)} to run -> {out}", flush=True)
    if dry_run:
        for i, (arm, k, _, rep) in jobs:
            print(f"  {i:3} {arm:6} {k:10} rep {rep}")
        return True
    failed_once = {(r["arm"], r["k"], r["rep"]) for r in read_rows(out)
                   if r["legs"] in ("", None)}
    rows = list(done.values())
    for i, (arm, k, path, rep) in todo:
        prof = power_profile()
        if prof != REQUIRED_PROFILE:
            print(f"[expF-phys] PAUSED before #{i}: power profile is {prof!r}, "
                  f"need {REQUIRED_PROFILE!r}. Resumable.", flush=True)
            return False
        seed, dx, dy, dyaw = exp3.rep_condition(rep)
        trace = out.parent / "traces" / ps.trace_name(arm, k, rep)
        print(f"[expF-phys] #{i:2} {arm} {k} rep {rep} (profile {prof}, load "
              f"{os.getloadavg()[0]:.2f}) running...", flush=True)
        res, stdout = physics_episode(path, rep, trace)
        after = power_profile()
        row = {"order": i, "sim": sim_label, "maze": MAZE, "arm": arm, "k": k,
               "rep": rep, "seed": seed, "dx": dx, "dy": dy, "dyaw": dyaw,
               **res,
               "power_profile": prof if after == prof else f"{prof}->{after}",
               "trace": (str(trace.relative_to(out.parent))
                         if trace.exists() else "")}
        if row.get("controller_sha") in (None, ""):
            row["controller_sha"] = sha16(path)
        if (arm, k, str(rep)) in failed_once:
            row["note"] = "RETRY (simulator failed once): " + (row["note"] or "")
        rows.append(row)
        write_rows(out, rows)
        print(f"[expF-phys] #{i:2} {arm} {k} rep {rep}: legs {row['legs']}/4 "
              f"{row['leg_times']} contacts {row.get('contacts')} "
              f"({row['wall_seconds']}s wall) {row['note']}", flush=True)
    return True


def cmd_physics(a):
    jobs = list(enumerate(order()))
    if not a.dry_run:
        prof = power_profile()
        if prof != REQUIRED_PROFILE:
            raise SystemExit(f"[expF-phys] refusing to start: profile {prof!r}")
        m = manifest("physics")
        if not m["prereg_commit"]:
            raise SystemExit("[expF-phys] prereg/expF.md is not committed; "
                             "the physics sweep runs only after it is")
    ok = run_physics(jobs, pathlib.Path(a.out or PHYSICS_CSV), "isaac",
                     a.dry_run)
    if not ok:
        raise SystemExit(3)


def cmd_sanity(a):
    """One base-controller episode on maze2 in each simulator. The base is not
    one of the 16, so this looks at no arm before the pre-registration."""
    base = ps.BASE
    rows = read_rows(SANITY_CSV)
    if a.sim in ("kinematic", "both"):
        r = {"order": 0, "sim": "kinematic", "maze": MAZE, "arm": "(base)",
             "k": "reference", "rep": 0, "seed": 0, "dx": 0.0, "dy": 0.0,
             "dyaw": 0.0, **kinematic_episode(
                 base, 0, OUT / "traces-sanity" / "kinematic__base__r0.csv.gz")}
        rows = [x for x in rows if x["sim"] != "kinematic"] + [r]
        write_rows(SANITY_CSV, rows)
        print(f"[expF-sanity] kinematic base: legs {r['legs']}/4 "
              f"{r['leg_times']} contacts {r['contacts']}")
    if a.sim in ("isaac", "both"):
        prof = power_profile()
        if prof != REQUIRED_PROFILE:
            raise SystemExit(f"[expF-sanity] profile {prof!r}, not running")
        trace = OUT / "traces-sanity" / "isaac__base__r0.csv.gz"
        res, stdout = physics_episode(base, 0, trace)
        (OUT / "sanity-isaac.log").write_text(stdout)
        r = {"order": 1, "sim": "isaac", "maze": MAZE, "arm": "(base)",
             "k": "reference", "rep": 0, "seed": 0, "dx": 0.0, "dy": 0.0,
             "dyaw": 0.0, **res, "power_profile": prof,
             "trace": str(trace.relative_to(OUT)) if trace.exists() else ""}
        rows = [x for x in rows if x["sim"] != "isaac"] + [r]
        write_rows(SANITY_CSV, rows)
        print(f"[expF-sanity] isaac base: legs {r['legs']}/4 {r['leg_times']} "
              f"contacts {r.get('contacts')} {r['note']}")


# ---- report --------------------------------------------------------------------

def summarise(rows):
    """{arm: {k: {rep: row}}}"""
    out = {}
    for r in rows:
        if r.get("legs") in ("", None):
            continue
        out.setdefault(r["arm"], {}).setdefault(r["k"], {})[int(r["rep"])] = r
    return out


def arm_table(rows, n_reps=exp3.N_REPS):
    from demo.analyse import fisher, wilson
    per = summarise(rows)
    res = {}
    for arm in ARMS:
        ctrls = per.get(arm, {})
        # prereg §5/§7: success in a strict majority of the reps that exist
        # (2 of 3 when complete; a tie with a missing rep counts as failure).
        complete = dict(ctrls)
        succ = {k: 2 * sum(exp3.is_success(v[r]["legs"]) for r in v) > len(v)
                for k, v in complete.items()}
        eps = [r for v in ctrls.values() for r in v.values()]
        res[arm] = {
            "n": len(complete), "x": sum(succ.values()),
            "per_k": {k: [int(ctrls[k][r]["legs"]) if r in ctrls[k] else None
                          for r in range(n_reps)] for k in sorted(
                              ctrls, key=lambda s: int(s[1:]))},
            "episodes": len(eps),
            "ep_success": sum(exp3.is_success(r["legs"]) for r in eps),
            "rep0": sum(exp3.is_success(v[0]["legs"]) for v in ctrls.values()
                        if 0 in v),
            "rep0_n": sum(1 for v in ctrls.values() if 0 in v),
            "wilson": wilson(sum(succ.values()), len(complete)) if complete else None,
        }
    a, b = res["arm0"], res["armB"]
    p = fisher(a["x"], a["n"], b["x"], b["n"]) if a["n"] and b["n"] else None
    p0 = (fisher(a["rep0"], a["rep0_n"], b["rep0"], b["rep0_n"])
          if a["rep0_n"] and b["rep0_n"] else None)
    return res, p, p0


def cmd_report(a):
    for label, path in (("KINEMATIC", KINEMATIC_CSV), ("ISAAC SIM PHYSICS", PHYSICS_CSV)):
        rows = read_rows(path)
        if not rows:
            print(f"{label}: no rows in {path}")
            continue
        res, p, p0 = arm_table(rows)
        print(f"\n== {label}, maze2, {len(rows)} episodes ({path.name})")
        for arm in ARMS:
            r = res[arm]
            per = "  ".join(f"{k}:{''.join('-' if x is None else str(x) for x in v)}"
                            for k, v in r["per_k"].items())
            w = r["wilson"]
            print(f"  {arm:5} controllers {r['x']}/{r['n']} "
                  f"(Wilson {w[0]:.2f}-{w[1]:.2f})" if w else f"  {arm}: incomplete",
                  f" episodes {r['ep_success']}/{r['episodes']}  rep0 "
                  f"{r['rep0']}/{r['rep0_n']}")
            print(f"        legs per rep: {per}")
        print(f"  PRIMARY Fisher two-sided, controllers (>= {MAJORITY}/3 reps "
              f"all four legs): p = {p if p is None else round(p, 4)}")
        print(f"  secondary, rep 0 only: p = {p0 if p0 is None else round(p0, 4)}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check")
    sc = sub.add_parser("scene-check")
    sc.add_argument("--maze", default=MAZE)
    k = sub.add_parser("kinematic")
    k.add_argument("--out"); k.add_argument("--dry-run", action="store_true")
    p = sub.add_parser("physics")
    p.add_argument("--out"); p.add_argument("--dry-run", action="store_true")
    s = sub.add_parser("sanity")
    s.add_argument("--sim", choices=("kinematic", "isaac", "both"), default="both")
    sub.add_parser("report")
    a = ap.parse_args(argv)
    {"check": cmd_check, "scene-check": cmd_scene_check, "kinematic": cmd_kinematic, "physics": cmd_physics,
     "sanity": cmd_sanity, "report": cmd_report}[a.cmd](a)


if __name__ == "__main__":
    main()
