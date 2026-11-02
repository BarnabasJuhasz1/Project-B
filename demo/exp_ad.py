"""
demo/exp_ad.py

EXP A AND EXP D: two physics follow-ups on the 16 pilot controllers, run with
Exp 3's apparatus (record_isaac.py, the frozen Go2 policy, exp3's start
offsets, the 180 s budget, success = all four trips). Pre-registered in
`prereg/expA.md` and `prereg/expD.md`, committed before the first episode.

    A   OPEN MAZE. The door never shuts, so the short route stays open on all
        four legs. 16 controllers x reps 0-2 (exp3.REP_CONDITIONS), 48
        episodes, interleaved across arms. Question: do the armB controllers
        fail only when they have to replan, or do they fail anyway?

    D   SLOW MACHINE. The original door schedule, rep 0 only (seed 0, no
        offset), 16 episodes, with the whole episode process inside a cgroup
        CPU limit (`CPU_QUOTA`, via systemd-run --user --scope). Question:
        does slowing the machine hurt the wall-clock-budgeted controllers more?
        Compared with Exp 3's rep 0 at normal speed.

⚠️ **THE CPU LIMIT IS CHECKED, NOT ASSUMED.** The runner reads its own cgroup's
`cpu.max` before every episode and refuses to start one unless it equals
`EXPERIMENTS["D"]["cpu_max"]`; record_isaac.py reports the value its own
process saw, and a mismatch stops the sweep. Exp A refuses to run under ANY
limit. The power profile is never changed by anything here: the runner reads it
(the ACPI file and `powerprofilesctl get`) before every episode and PAUSES,
resumably, if it is not `performance`. An episode during which it moved is
QUARANTINED to `physics_quarantine.csv` and rerun on resume.

⚠️ **ONE ISAAC PROCESS ON THE MACHINE.** Always run under the shared lock:

    nohup flock /tmp/mindfire-isaac.lock env -u PYTHONHOME -u PYTHONPATH \\
        $P -u demo/exp_ad.py A > demo/out/expA/sweep.log 2>&1 &
    nohup flock /tmp/mindfire-isaac.lock systemd-run --user --scope --quiet \\
        -p CPUQuota=200% env -u PYTHONHOME -u PYTHONPATH \\
        $P -u demo/exp_ad.py D > demo/out/expD/sweep.log 2>&1 &
    python demo/exp_ad.py A --dry-run      # the order + every check, no boot

It resumes: an (arm, k, rep) already in the CSV is skipped.
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

# ---- below this line is data, not code.

ARMS = ("arm0", "armB")
KS = tuple(f"k{i}" for i in range(8))
CPU_QUOTA = "200%"            # systemd CPUQuota: 2 CPUs of the 22 (Ultra 9 185H)
EXPERIMENTS = {
    "A": {"reps": (0, 1, 2), "door_never_shuts": True, "order_seed": 41,
          "cpu_max": "max", "prereg": "prereg/expA.md",
          "out": REPO / "demo" / "out" / "expA" / "physics.csv"},
    "D": {"reps": (0,), "door_never_shuts": False, "order_seed": 42,
          "cpu_max": "200000 100000", "prereg": "prereg/expD.md",
          "out": REPO / "demo" / "out" / "expD" / "physics.csv"},
}
REQUIRED_PROFILE = "performance"
EPISODE_TIMEOUT = 1200        # s. Was 3600 for the first A and D passes; a normal
                              # episode takes 60-330 s wall even at 2 CPUs, so 1200 s
                              # only ever fires on a hung simulator
# The same rule exp3_report.py marks budgets with.
BUDGET_RE = re.compile(r"time\.(perf_counter|time|monotonic)\s*\(")
FIELDS = ["order", "experiment", "arm", "k", "rep", "seed", "dx", "dy", "dyaw",
          "door_never_shuts", "legs", "total", "success", "leg_times",
          "sim_seconds", "wall_seconds", "contacts", "contact_steps", "path_m",
          "step_ms_med", "step_ms_p95", "step_ms_max",
          "track_v_med", "track_v_p95", "track_wz_med", "track_wz_p95",
          "load1_before", "load1_mean", "load1_max", "cpu_max",
          "power_profile", "clock_limited", "controller_sha",
          "prereg_commit", "trace", "note"]

# ---- code.

PLATFORM_PROFILE = pathlib.Path("/sys/firmware/acpi/platform_profile")


def cgroup_cpu_max(pid="self"):
    """This process's cgroup v2 `cpu.max` ("200000 100000" = 2 CPUs), "max"
    when the cgroup has the file but no limit, None when it has no file."""
    try:
        for line in pathlib.Path(f"/proc/{pid}/cgroup").read_text().splitlines():
            if line.startswith("0::"):
                f = pathlib.Path("/sys/fs/cgroup" + line[3:]) / "cpu.max"
                return f.read_text().strip() if f.exists() else None
    except OSError:
        pass
    return None


def cpu_limit_ok(experiment, value):
    want = EXPERIMENTS[experiment]["cpu_max"]
    if want == "max":                       # A: no limit of any kind
        return value in ("max", None) or value.startswith("max ")
    return value == want


def power_profiles():
    """(ACPI platform_profile, `powerprofilesctl get`). Read only."""
    try:
        acpi = PLATFORM_PROFILE.read_text().strip()
    except OSError:
        acpi = ""
    env = {k: v for k, v in os.environ.items()
           if k not in ("PYTHONHOME", "PYTHONPATH")}
    try:
        ppd = subprocess.run(["powerprofilesctl", "get"], env=env, text=True,
                             capture_output=True, timeout=20).stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        ppd = ""
    return acpi, ppd


def profile_ok(profiles):
    return all(p == REQUIRED_PROFILE for p in profiles)


def clock_limited(path):
    return bool(BUDGET_RE.search(pathlib.Path(path).read_text()))


def controllers():
    return [(arm, k, REPO / "demo" / "arms" / arm / f"{k}.py")
            for arm in ARMS for k in KS]


def job_order(experiment):
    e = EXPERIMENTS[experiment]
    return exp3.job_order(controllers(), e["reps"], seed=e["order_seed"])


def command(experiment, path, rep, trace):
    seed, dx, dy, dyaw = exp3.rep_condition(rep)
    cmd = [sys.executable, str(REPO / "demo" / "record_isaac.py"), str(path),
           "--no-record", "--seed", str(seed), "--start-dx", repr(dx),
           "--start-dy", repr(dy), "--start-dyaw", repr(dyaw),
           "--trace", str(trace)]
    if EXPERIMENTS[experiment]["door_never_shuts"]:
        cmd.append("--door-never-shuts")
    return cmd


def prereg_commit(experiment):
    """The commit that last touched the pre-registration, or "" if it is not
    committed -- in which case the sweep refuses to run."""
    rel = EXPERIMENTS[experiment]["prereg"]
    out = subprocess.run(["git", "log", "-1", "--format=%H", "--", rel],
                         cwd=str(REPO), text=True, capture_output=True)
    dirty = subprocess.run(["git", "status", "--porcelain", "--", rel],
                           cwd=str(REPO), text=True, capture_output=True)
    return "" if dirty.stdout.strip() else out.stdout.strip()


def parse(stdout, stderr):
    js = exp3.parse_result_json(stdout)
    if js is None:
        tail = (stdout or stderr or "").strip().splitlines()[-3:]
        return {"legs": None, "note": "NO RESULT_JSON: " + " | ".join(tail)[:200]}
    row = {k: js.get(k) for k in (
        "legs", "total", "success", "sim_seconds", "contacts",
        "contact_steps", "path_m", "step_ms_med", "step_ms_p95",
        "step_ms_max", "track_v_med", "track_v_p95", "track_wz_med",
        "track_wz_p95", "load1_mean", "load1_max", "controller_sha",
        "cpu_max", "door_never_shuts")}
    row["leg_times"] = json.dumps(js.get("leg_times", []))
    row["note"] = ("controller raised: " + js["raised"])[:200] if js.get("raised") else ""
    return row


def run_one(experiment, path, rep, trace):
    load_before = round(os.getloadavg()[0], 2)
    t0 = time.time()
    try:
        p = subprocess.run(command(experiment, path, rep, trace), cwd=str(REPO),
                           capture_output=True, text=True, timeout=EPISODE_TIMEOUT)
        stdout, stderr = p.stdout, p.stderr
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout.decode() if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = f"TIMEOUT after {EPISODE_TIMEOUT} s"
    return {"wall_seconds": round(time.time() - t0, 1),
            "load1_before": load_before, **parse(stdout, stderr)}


def read_rows(path):
    path = pathlib.Path(path)
    return list(csv.DictReader(path.open(newline=""))) if path.exists() else []


def write_rows(path, rows):
    tmp = pathlib.Path(str(path) + ".tmp")
    with tmp.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: int(r["order"])))
    os.replace(tmp, path)


def trace_name(arm, k, rep):
    return f"{arm}__{k}__r{rep}.csv.gz"


def write_manifest(out_dir, experiment, order, commit, argv):
    git = lambda *c: subprocess.run(["git", *c], cwd=str(REPO), text=True,
                                    capture_output=True).stdout.strip()
    e = EXPERIMENTS[experiment]
    m = {"experiment": experiment, "started_utc": time.strftime(
             "%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
         "argv": argv, "git_head": git("rev-parse", "HEAD"),
         "git_branch": git("rev-parse", "--abbrev-ref", "HEAD"),
         "prereg": e["prereg"], "prereg_commit": commit,
         "door_never_shuts": e["door_never_shuts"],
         "cpu_quota": CPU_QUOTA if experiment == "D" else None,
         "cpu_max_required": e["cpu_max"], "cpu_max_seen": cgroup_cpu_max(),
         "rep_conditions": {r: exp3.rep_condition(r) for r in e["reps"]},
         "order_seed": e["order_seed"],
         "success_rule": f"legs >= {exp3.LEGS_FOR_SUCCESS}",
         "order": [[a, k, r] for a, k, _, r in order],
         "python": sys.version.split()[0], "host": platform.node(),
         "nproc": os.cpu_count(), "in_container": os.path.exists("/.dockerenv")}
    path = out_dir / "manifest.json"
    if path.exists():
        path = out_dir / f"manifest.resume-{int(time.time())}.json"
    path.write_text(json.dumps(m, indent=1))


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("experiment", choices=sorted(EXPERIMENTS))
    ap.add_argument("--out", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--retry-failed", action="store_true",
                    help="rerun episodes with no result (the SIMULATOR failed, not the controller); the failed rows are kept in physics_simfail.csv")
    a = ap.parse_args(argv)
    e = EXPERIMENTS[a.experiment]
    out = pathlib.Path(a.out or e["out"]).resolve()
    tag = f"[exp{a.experiment}]"

    order = job_order(a.experiment)
    rows0 = read_rows(out)
    failed = [r for r in rows0 if r["legs"] in ("", None)]
    if a.retry_failed and failed and not a.dry_run:
        keep = out.parent / "physics_simfail.csv"
        write_rows(keep, read_rows(keep) + failed)
        rows0 = [r for r in rows0 if r not in failed]
        write_rows(out, rows0)
        print(f"{tag} retrying {len(failed)} simulator failures; kept in "
              f"{keep.name}", flush=True)
    elif a.retry_failed:
        rows0 = [r for r in rows0 if r not in failed]
    done = {(r["arm"], r["k"], r["rep"]): r for r in rows0}
    todo = [j for j in order if (j[0], j[1], str(j[3])) not in done]
    commit = prereg_commit(a.experiment)
    cpu, prof = cgroup_cpu_max(), power_profiles()
    print(f"{tag} {len(order)} planned, {len(order) - len(todo)} done, "
          f"{len(todo)} to run | prereg {e['prereg']} @ {commit or 'UNCOMMITTED'}"
          f" | cpu.max {cpu!r} (need {e['cpu_max']!r}) | profile {prof}",
          flush=True)
    if a.dry_run:
        for i, (arm, k, path, rep) in enumerate(order):
            mark = "  (done)" if (arm, k, str(rep)) in done else ""
            print(f"  {i:3} {arm:5} {k:3} rep {rep} clock={int(clock_limited(path))} "
                  f"{' '.join(command(a.experiment, path, rep, 'T.csv.gz')[3:])}{mark}")
        return 0
    if not commit:
        raise SystemExit(f"{tag} REFUSED: {e['prereg']} is not committed")
    if not cpu_limit_ok(a.experiment, cpu):
        raise SystemExit(f"{tag} REFUSED: cpu.max {cpu!r}, need {e['cpu_max']!r}")

    out.parent.mkdir(parents=True, exist_ok=True)
    (out.parent / "traces").mkdir(exist_ok=True)
    write_manifest(out.parent, a.experiment, order, commit, sys.argv)
    rows = list(done.values())
    for i, (arm, k, path, rep) in enumerate(order):
        if (arm, k, str(rep)) in done:
            continue
        before = power_profiles()
        if not profile_ok(before):
            print(f"{tag} PAUSED before #{i}: power profile {before}, need "
                  f"{REQUIRED_PROFILE!r}. Not changed by this script. Resumable.",
                  flush=True)
            return 2
        cpu = cgroup_cpu_max()
        if not cpu_limit_ok(a.experiment, cpu):
            print(f"{tag} STOPPED before #{i}: cpu.max {cpu!r}", flush=True)
            return 3
        seed, dx, dy, dyaw = exp3.rep_condition(rep)
        trace = out.parent / "traces" / trace_name(arm, k, rep)
        print(f"{tag} #{i:2} {arm} {k} rep {rep} (load "
              f"{os.getloadavg()[0]:.2f}, cpu.max {cpu}, profile {before[1]}) "
              f"running...", flush=True)
        row = {"order": i, "experiment": a.experiment, "arm": arm, "k": k,
               "rep": rep, "seed": seed, "dx": dx, "dy": dy, "dyaw": dyaw,
               "clock_limited": int(clock_limited(path)),
               "prereg_commit": commit, **run_one(a.experiment, path, rep, trace)}
        row["trace"] = str(trace.relative_to(out.parent)) if trace.exists() else ""
        after = power_profiles()
        row["power_profile"] = (before[1] if after == before
                                else f"{before}->{after}")
        print(f"{tag} #{i:2} {arm} {k} rep {rep} legs {row['legs']} times "
              f"{row.get('leg_times')} contacts {row.get('contacts')} step_ms "
              f"{row.get('step_ms_med')}/{row.get('step_ms_p95')} "
              f"({row['wall_seconds']}s wall) {row.get('note', '')}", flush=True)
        if after != before:
            q = out.parent / "physics_quarantine.csv"
            write_rows(q, read_rows(q) + [row])
            print(f"{tag} QUARANTINED #{i}: profile moved {before}->{after}; "
                  f"PAUSED. Resumable, it reruns.", flush=True)
            return 2
        if not cpu_limit_ok(a.experiment, row.get("cpu_max")):
            row["note"] = f"CPU LIMIT NOT SEEN BY THE EPISODE: {row.get('cpu_max')!r}"
            q = out.parent / "physics_quarantine.csv"
            write_rows(q, read_rows(q) + [row])
            print(f"{tag} STOPPED: {row['note']}", flush=True)
            return 3
        rows = [r for r in rows if (r["arm"], r["k"], str(r["rep"])) != (arm, k, str(rep))]
        rows.append(row)
        write_rows(out, rows)
    print(f"{tag} complete: {len(rows)} rows -> {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
