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
runs being measured. This script therefore SHELLS OUT once per episode and
waits. ~4-5 min each.

⚠️ **EXP 3 (2026-11-02, Mindfire_Experiment_Plan.md §3).** Each controller now
runs `--reps` times (default 3) under the conditions in `demo/exp3.py`: rep 0
is the original condition and must reproduce demo/out/physics.csv; reps 1-2
perturb the start pose and the scenario seed, identically for every
controller. The order is INTERLEAVED (rep-major blocks, each a seeded shuffle
across arms), every episode writes a per-step gzip trace, and the machine's
load average is logged per episode -- because wall-clock-budgeted controllers,
mostly armB, are hurt by load (plan R5). Success is ALL FOUR legs.

⚠️ **IT NEVER WRITES demo/out/physics.csv.** That file is the pilot's result
and rep 0 is checked AGAINST it. The default output is demo/out/v2/.

It resumes: an (arm, k, rep) already in the CSV is skipped, so a killed sweep
keeps its work. An episode that produced no result line (the simulator, not
the controller, failed) is recorded with an empty `legs` and is NOT re-run
unless `--retry-failed` is given -- the project records failures rather than
quietly retrying them.

    python demo/physics_sweep.py --dry-run                 # the order, no boot
    python demo/physics_sweep.py --arms arm0,armB --ks k0,k1,k2,k3,k4,k5,k6,k7
    python demo/physics_sweep.py --compare-only            # rep 0 vs physics.csv
"""
import argparse
import csv
import json
import os
import pathlib
import platform
import re
import subprocess
import sys
import time

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from demo import exp3
from demo.render import ARMS

PY = sys.executable
RESULT_RE = re.compile(
    r"RESULT\s+(?P<name>\S+):\s+legs\s+(?P<legs>\d+)/(?P<total>\d+)\s+"
    r"times\s+(?P<times>\[[^\]]*\])\s+t=(?P<t>[\d.]+)")
FIELDS = ["order", "arm", "k", "rep", "seed", "dx", "dy", "dyaw",
          "legs", "total", "success", "leg_times", "sim_seconds",
          "wall_seconds", "contacts", "path_m",
          "step_ms_med", "step_ms_p95", "step_ms_max",
          "track_v_med", "track_v_p95", "track_wz_med",
          "load1_before", "load1_mean", "load1_max",
          "controller_sha", "power_profile", "trace", "note"]
PILOT_REFERENCE = REPO / "demo" / "out" / "physics.csv"
DEFAULT_OUT = REPO / "demo" / "out" / "v2" / "physics.csv"
BASE = REPO / "generate" / "base" / "base_controller.py"
PLATFORM_PROFILE = pathlib.Path("/sys/firmware/acpi/platform_profile")
POWER_SUPPLY = pathlib.Path("/sys/class/power_supply")


def on_ac():
    """True if any Mains supply reports online; None where there is none.

    ⚠️ Added 2026-11-03: the profile can read `performance` while the laptop
    runs on battery (seen twice on 2026-11-02/03), so the profile alone does
    not guarantee a fast machine.
    """
    mains = [d for d in POWER_SUPPLY.glob("*")
             if (d / "type").is_file() and (d / "type").read_text().strip() == "Mains"]
    if not mains:
        return None
    return any((d / "online").read_text().strip() == "1" for d in mains
               if (d / "online").is_file())


def power_profile():
    """The ACPI platform profile, or "" where the machine has none.

    ⚠️ Logged per episode because it moved under the sweep once: on 2026-11-02
    the laptop rebooted into `low-power` and episodes ran ~2x slower, which
    starves the wall-clock-budgeted controllers (plan R5). A file read, ~free.
    """
    try:
        return PLATFORM_PROFILE.read_text().strip()
    except OSError:
        return ""


def discover(arms, ks=None, base=True):
    """(arm, k, path) for every controller that will run."""
    jobs = []
    for arm in arms:
        for p in sorted((REPO / "demo" / "arms" / arm).glob("k*.py"),
                        key=lambda p: int(p.stem[1:])):
            if ks is None or p.stem in ks:
                jobs.append((arm, p.stem, p))
    if base:
        jobs.append(("(base)", "reference", BASE))
    return jobs


def trace_name(arm, k, rep):
    return f"{arm.strip('()')}__{k}__r{rep}.csv.gz"


def command(path, rep, trace):
    seed, dx, dy, dyaw = exp3.rep_condition(rep)
    return [PY, str(REPO / "demo" / "record_isaac.py"), str(path),
            "--no-record", "--seed", str(seed), "--start-dx", repr(dx),
            "--start-dy", repr(dy), "--start-dyaw", repr(dyaw),
            "--trace", str(trace)]


def run_one(path, rep, trace):
    """One episode in its own process. Returns a row dict."""
    load_before = round(os.getloadavg()[0], 2)
    t0 = time.time()
    try:
        proc = subprocess.run(command(path, rep, trace), cwd=str(REPO),
                              capture_output=True, text=True, timeout=1800)
        stdout, stderr = proc.stdout, proc.stderr
    except subprocess.TimeoutExpired as exc:
        stdout, stderr = (exc.stdout or ""), "TIMEOUT after 1800 s"
        stdout = stdout.decode() if isinstance(stdout, bytes) else stdout
    wall = round(time.time() - t0, 1)
    return row_from_output(stdout, stderr, wall, load_before)


def row_from_output(stdout, stderr, wall, load_before):
    """Parse one record_isaac.py run. RESULT_JSON is preferred; the old RESULT
    line is the fallback so a run that died after printing it still scores."""
    base = {"wall_seconds": wall, "load1_before": load_before}
    js = exp3.parse_result_json(stdout)
    if js is not None:
        return {**base, **{k: js.get(k) for k in (
            "legs", "total", "success", "sim_seconds", "contacts", "path_m",
            "step_ms_med", "step_ms_p95", "step_ms_max", "track_v_med",
            "track_v_p95", "track_wz_med", "load1_mean", "load1_max",
            "controller_sha")},
            "leg_times": json.dumps(js.get("leg_times", [])),
            "note": ("controller raised: " + js["raised"])[:200]
                    if js.get("raised") else ""}
    m = RESULT_RE.search(stdout or "")
    if m:
        legs = int(m["legs"])
        return {**base, "legs": legs, "total": int(m["total"]),
                "success": exp3.is_success(legs), "leg_times": m["times"],
                "sim_seconds": float(m["t"]),
                "note": "RESULT line only, no RESULT_JSON"}
    tail = (stdout or stderr or "").strip().splitlines()[-3:]
    return {**base, "legs": None, "total": None, "success": None,
            "leg_times": "", "sim_seconds": "",
            "note": "NO RESULT LINE: " + " | ".join(tail)[:200]}


def read_rows(path):
    if not pathlib.Path(path).exists():
        return []
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def write_rows(path, rows):
    """Atomic: a sweep killed mid-write must not lose the finished rows."""
    tmp = pathlib.Path(str(path) + ".tmp")
    with tmp.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: int(r["order"])))
    os.replace(tmp, path)


def write_manifest(out_dir, a, order):
    """What a reader needs to know the sweep's conditions without the code."""
    def git(*cmd):
        try:
            return subprocess.run(["git", *cmd], cwd=str(REPO), text=True,
                                  capture_output=True).stdout.strip()
        except OSError:
            return None
    manifest = {
        "started_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "argv": sys.argv, "git_head": git("rev-parse", "HEAD"),
        "git_branch": git("rev-parse", "--abbrev-ref", "HEAD"),
        "git_dirty": bool(git("status", "--porcelain", "--untracked-files=no")),
        "rep_conditions": exp3.REP_CONDITIONS,
        "perturb_seed": exp3.PERTURB_SEED, "order_seed": exp3.ORDER_SEED,
        "success_rule": f"legs >= {exp3.LEGS_FOR_SUCCESS}",
        "order": [[arm, k, rep] for arm, k, _, rep in order],
        "python": sys.version.split()[0], "host": platform.node(),
        "nproc": os.cpu_count(), "in_container": os.path.exists("/.dockerenv"),
    }
    path = out_dir / "manifest.json"
    if path.exists():                       # a resume keeps the first start
        path = out_dir / f"manifest.resume-{int(time.time())}.json"
    path.write_text(json.dumps(manifest, indent=1))


def report(rows, reference):
    print("\n" + "=" * 72)
    print(f"{'arm':10}{'episodes':>9}{'ep success':>12}{'ctrls':>7}"
          f"{'maj-of-reps':>13}{'all agree':>11}")
    print("-" * 72)
    per = {}
    for r in rows:
        if r["legs"] in ("", None):
            continue
        per.setdefault(r["arm"], {}).setdefault(r["k"], []).append(
            exp3.is_success(r["legs"]))
    for arm, ctrls in per.items():
        eps = [s for v in ctrls.values() for s in v]
        maj = sum(1 for v in ctrls.values() if sum(v) * 2 > len(v))
        agree = sum(1 for v in ctrls.values() if len(set(v)) == 1)
        print(f"{arm:10}{len(eps):>9}{f'{sum(eps)}/{len(eps)}':>12}"
              f"{len(ctrls):>7}{f'{maj}/{len(ctrls)}':>13}"
              f"{f'{agree}/{len(ctrls)}':>11}")
    print("=" * 72)
    print(f"success = legs >= {exp3.LEGS_FOR_SUCCESS}. maj-of-reps is the plan's "
          "primary outcome only once every\ncontroller has all its reps. "
          "UNDER PHYSICS, one hand-authored world, in_container false\nunless "
          "run in nvcr.io/nvidia/isaac-sim:6.0.1.")
    if reference.exists():
        cmp = exp3.compare_rep0(rows, read_rows(reference))
        if cmp:
            n = sum(1 for c in cmp if c[2] == "MATCH")
            print(f"\nrep 0 vs {reference.name}: {n}/{len(cmp)} MATCH")
            for arm, k, verdict, detail in cmp:
                if verdict != "MATCH":
                    print(f"  {verdict:7} {arm} {k}: {detail}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--arms", default=",".join(ARMS),
                    help=f"comma list (default render.ARMS = {','.join(ARMS)})")
    ap.add_argument("--ks", default=None,
                    help="comma list of k stems, e.g. k0,k1 (default: all)")
    ap.add_argument("--no-base", action="store_true",
                    help="leave out the frozen base's reference row")
    ap.add_argument("--reps", type=int, default=exp3.N_REPS,
                    help="reps 0..N-1 (default 3)")
    ap.add_argument("--only-rep", type=int, default=None,
                    help="run just this rep (e.g. 0 for the determinism check)")
    ap.add_argument("--reference", default=str(PILOT_REFERENCE))
    ap.add_argument("--retry-failed", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--require-profile", default=None,
                    help="e.g. performance: refuse to start, and pause at the "
                         "next episode boundary, if the platform profile is "
                         "anything else")
    ap.add_argument("--require-ac", action="store_true",
                    help="refuse to start, and pause at the next episode "
                         "boundary, if no mains supply is online")
    ap.add_argument("--compare-only", action="store_true")
    a = ap.parse_args(argv)

    out_path = pathlib.Path(a.out).resolve()
    if out_path == PILOT_REFERENCE.resolve():
        raise SystemExit(f"refusing to write {out_path}: it is the pilot result "
                         f"rep 0 is checked against. Use demo/out/v2/.")
    if not 1 <= a.reps <= exp3.N_REPS:
        raise SystemExit(f"--reps must be 1..{exp3.N_REPS}")
    reference = pathlib.Path(a.reference)

    if a.compare_only:
        report(read_rows(out_path), reference)
        return

    arms = [s for s in a.arms.split(",") if s]
    unknown = set(arms) - set(ARMS)
    if unknown:
        raise SystemExit(f"unknown arm(s) {sorted(unknown)}; known: {ARMS}")
    ks = set(a.ks.split(",")) if a.ks else None
    reps = [a.only_rep] if a.only_rep is not None else list(range(a.reps))
    order = exp3.job_order(discover(arms, ks, base=not a.no_base), reps)

    done = {(r["arm"], r["k"], r["rep"]): r for r in read_rows(out_path)}
    if a.retry_failed:
        done = {key: r for key, r in done.items() if r["legs"] not in ("", None)}
    todo = [j for j in order if (j[0], j[1], str(j[3])) not in done]
    print(f"[phys] {len(order)} episodes planned, {len(order) - len(todo)} "
          f"already in {out_path.name}, {len(todo)} to run")

    if a.dry_run:
        for i, (arm, k, path, rep) in enumerate(order):
            mark = "  (done)" if (arm, k, str(rep)) in done else ""
            print(f"  {i:3} {arm:7} {k:10} rep {rep}{mark}")
        return

    out_path.parent.mkdir(parents=True, exist_ok=True)
    trace_dir = out_path.parent / "traces"
    trace_dir.mkdir(exist_ok=True)
    write_manifest(out_path.parent, a, order)
    rows = list(done.values())
    for i, (arm, k, path, rep) in enumerate(order):
        if (arm, k, str(rep)) in done:
            continue
        before = power_profile()
        if a.require_profile and before != a.require_profile:
            print(f"[phys] PAUSED before #{i}: platform profile is {before!r}, "
                  f"--require-profile {a.require_profile!r}. Resumable.",
                  flush=True)
            break
        if a.require_ac and on_ac() is False:
            print(f"[phys] PAUSED before #{i}: on battery (no mains supply "
                  f"online), --require-ac. Resumable.", flush=True)
            break
        seed, dx, dy, dyaw = exp3.rep_condition(rep)
        trace = trace_dir / trace_name(arm, k, rep)
        print(f"[phys] #{i:3} {arm:7} {k:10} rep {rep} "
              f"(load {os.getloadavg()[0]:.2f}) running...", flush=True)
        row = {"order": i, "arm": arm, "k": k, "rep": rep, "seed": seed,
               "dx": dx, "dy": dy, "dyaw": dyaw,
               **run_one(path, rep, trace)}
        row["trace"] = (str(trace.relative_to(out_path.parent))
                        if trace.exists() else "")
        after = power_profile()
        row["power_profile"] = before if after == before else f"{before}->{after}"
        rows = [r for r in rows
                if (r["arm"], r["k"], str(r["rep"])) != (arm, k, str(rep))]
        rows.append(row)
        print(f"[phys] #{i:3} {arm:7} {k:10} rep {rep} legs "
              f"{row['legs']}/{row['total']}  times {row['leg_times']}  "
              f"({row['wall_seconds']}s wall, load {row.get('load1_mean')}) "
              f"{row['note']}", flush=True)
        write_rows(out_path, rows)

    report(rows, reference)
    print(f"-> {out_path}")


if __name__ == "__main__":
    main()
