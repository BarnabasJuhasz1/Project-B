"""
demo/expC.py

EXP C: THE FULL-REPLAN FALLBACK ABLATION (review item E7b). The causal test of
Hypothesis 2: a plan that is only ever repaired locally goes stale under
physics, and recomputing it from scratch when the robot stops making progress
would rescue it.

⚠️ **THIS IS AN INTERVENTION ON MODEL-WRITTEN CODE.** Every file this script
writes under `demo/interventions/expC/` is a model's controller with a block
appended BY US. The patched copies are NOT samples of any arm: they never enter
an arm count, a Fisher test, a pass rate or `demo/arms/`. `run.py`,
`physics_sweep.py` and `analyse.py` glob `demo/arms/<arm>/k*.py` and cannot see
them. The originals stay byte-identical, and `check_originals()` proves it
against the hashes recorded below before anything runs.

⚠️ **THE PATCH IS ONE RULE, APPENDED, AND IDENTICAL IN EVERY COPY.** The model's
file is copied verbatim and a block is appended after it. The block subclasses
the model's `Controller` and, before every step, asks one shared no-progress
monitor (`_ExpCNoProgress`, `PROGRESS_M`, `WINDOW_S`) whether the robot has
stopped closing on its current target. When it fires, the controller's cached
ROUTE state (its D* trees, cost-to-goal fields) is discarded, and the model's
own code rebuilds it from scratch on its current map in the same step. The map,
the cost grid, penalties, recovery state and every other attribute are kept.
The ONLY per-controller text is the 1-5 lines naming where that controller
keeps its route (`HOOKS`) -- the controllers store it under different names,
so "drop the route" cannot be literally the same line in all seven.

⚠️ **WHEN THE MONITOR DOES NOT FIRE, THE PATCHED COPY IS THE ORIGINAL.** No rng
draw, no state the model's code reads, no change to the command. So a kinematic
episode with zero fires must reproduce the original's trajectory exactly, and
`kinematic` checks that.

⚠️ **THE TRIGGER ALSO FIRES ON LEGITIMATE DETOURS**, by construction: straight-
line distance to the target cannot fall while the robot backs out of the room
the shut door turned into a cul-de-sac. The unpatched Exp 3 traces show 20-50 s
without 0.5 m of straight-line progress on SUCCESSFUL leg-2 detours. So a
successful detour sees ~2-5 fallback firings. That is declared, not hidden: a
full replan on a correct map returns the same route, so a firing that hurts is
itself evidence (prereg/expC.md).

    python demo/expC.py make          # write the copies + diffs, check hashes
    python demo/expC.py check         # originals intact, copies == make()
    python demo/expC.py kinematic     # 7 x (orig, patched) x 3 reps, sim-free
    python demo/expC.py physics       # the Isaac runs (flock + nohup it)
    python demo/expC.py report        # paired against Exp 3's own rows

SIM-FREE except `physics`, which shells out to record_isaac.py once per episode.
"""
import argparse
import csv
import difflib
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent
sys.path.insert(0, str(REPO))

from demo import exp3

# ---- below this line is data, not code.

ARM = "armB"

# The seven armB controllers with incremental repair (Exp 4, EXP4_PILOT.md),
# with their CONSENSUS planner label. ⚠️ Only FIVE are `incremental_only`. k2
# and k7 already carry a NARROW from-scratch path -- k2 rebuilds a tree when its
# backpointer trace is corrupted, k7 when its goal cell turns lethal -- neither
# of which is a no-progress trigger. They are patched with the same rule and
# reported as their own stratum. armB k5 (`full_periodic`) is not patched.
EXP4_LABEL = {
    "k0": "incremental_only",
    "k1": "incremental_only",
    "k2": "incremental_with_fallback",
    "k3": "incremental_only",
    "k4": "incremental_only",
    "k6": "incremental_only",
    "k7": "incremental_with_fallback",
}

# sha256 of each ORIGINAL, as committed (the first 16 hex are Exp 3's
# controller_sha). check_originals() refuses to run if any has moved.
ORIGINAL_SHA256 = {
    "k0": "dc3622b547c3568e8a8eacc4ddd91545182cc2c50683e3aff99610a6319fe2ae",
    "k1": "51b58b5eb74b4dd5cfcbd3ab24ffe1884f43779499a99f4e39a4eeeda77199dd",
    "k2": "d8f855696060c3843f20e4ce89d1248cd27b88996d3aba97f256886c583c2994",
    "k3": "0bf3f73571fcd251f252c874525da8fb89318ac66c0c1a2938a7c71e2403f767",
    "k4": "c0422969be1bba1f986ee7b40243264ba46340ea4c8ef66784445fc06f77f6e9",
    "k6": "69411dd5e493f37363e7429ea600477122d89288cdcc95552698787c05dea2ef",
    "k7": "70ca68a952daff956aab772adbdfab4bf3e49db2b7d90640d6fdc099c70c8e5a",
}

# THE TRIGGER. One threshold, one window, used everywhere. Distance is the
# straight line from the pose estimate to obs.goal; time is SIM time (obs.t),
# so the trigger is deterministic and load cannot move it.
PROGRESS_M = 0.5        # progress = a new best distance at least this much lower
WINDOW_S = 10.0         # fire after this long without progress
GOAL_SWITCH_M = 0.5     # obs.goal moving further than this starts a new leg

# Where each controller keeps its ROUTE, and the from-scratch rebuild. Each is
# the model's own construction path: the next super().step() finds no route for
# the current goal and builds one exactly as it does for a goal it has never
# seen. k6 keeps one live field plus a per-goal cache, so its hook re-runs the
# no-cache branch of its own _switch_goal and requests its own planning burst.
HOOKS = {
    "k0": "self._fields = {}",
    "k1": "self.planners = {}",
    "k2": "self.planners = {}",
    "k3": "self.fields = {}",
    "k4": "self.fields = {}",
    "k6": ("self.hcache = {}\n"
           "if self.h is not None:\n"
           "    self.h = np.full((self.ny, self.nx), np.float32(_BIG), dtype=np.float32)\n"
           "    self.h[self.gj, self.gi] = np.float32(0.0)\n"
           "    self.pending_burst = True"),
    "k7": "self.planners = {}",
}

BANNER = "# ==== EXP C INTERVENTION: NOT MODEL-WRITTEN "

BLOCK = '''

{banner}{rule}
# Appended by demo/expC.py to a byte-identical copy of demo/arms/armB/{k}.py
# (sha256 {sha}).
# Everything ABOVE this banner is the model's file, unchanged.
# Everything below is ours and identical in every patched copy except the body
# of _expc_full_replan. This file is NOT an armB sample and never enters an arm
# count. See demo/expC.py and prereg/expC.md.
import math as _expc_math
import time as _expc_time

_EXPC_PROGRESS_M = {progress!r}
_EXPC_WINDOW_S = {window!r}
_EXPC_GOAL_SWITCH_M = {switch!r}


class _ExpCNoProgress:
    """True when the straight-line distance to the current target has not
    reached a new best, lower by _EXPC_PROGRESS_M, for _EXPC_WINDOW_S sim
    seconds. Restarts on a new target and after every firing."""

    def __init__(self):
        self.goal = None
        self.best = None
        self.t_best = None

    def update(self, t, x, y, gx, gy):
        d = _expc_math.hypot(gx - x, gy - y)
        if (self.goal is None or _expc_math.hypot(
                gx - self.goal[0], gy - self.goal[1]) > _EXPC_GOAL_SWITCH_M):
            self.goal, self.best, self.t_best = (gx, gy), d, t
            return False
        if d <= self.best - _EXPC_PROGRESS_M:
            self.best, self.t_best = d, t
            return False
        if t - self.t_best >= _EXPC_WINDOW_S:
            self.best, self.t_best = d, t
            return True
        return False


_ExpCModelController = Controller


class Controller(_ExpCModelController):
    """The model's controller plus the full-replan fallback. Nothing else."""

    def reset(self, scenario, rng):
        super().reset(scenario, rng)
        self._expc_monitor = _ExpCNoProgress()
        self.expc_fallback_fires = []
        self.expc_errors = []

    def step(self, obs):
        fired = False
        try:
            p, g = obs.pose_estimate, obs.goal
            fired = self._expc_monitor.update(
                float(obs.t), float(p[0]), float(p[1]), float(g[0]), float(g[1]))
        except Exception as exc:
            self.expc_errors.append(repr(exc)[:160])
        if not fired:
            return super().step(obs)
        t0 = _expc_time.perf_counter()
        try:
            self._expc_full_replan()
        except Exception as exc:
            self.expc_errors.append(repr(exc)[:160])
        cmd = super().step(obs)
        self.expc_fallback_fires.append({{
            "t": round(float(obs.t), 2),
            "goal": [round(float(obs.goal[0]), 2), round(float(obs.goal[1]), 2)],
            "dist": round(self._expc_monitor.best, 3),
            "step_ms": round((_expc_time.perf_counter() - t0) * 1000.0, 2)}})
        return cmd

    def _expc_full_replan(self):
        # Drop the cached route; keep the map. Per-controller (expC.HOOKS).
{hook}
'''

OUT = HERE / "interventions" / "expC"
COPIES = OUT / ARM
DIFFS = OUT / "diffs"
RESULTS = HERE / "out" / "expC"
EXP3_PHYSICS = HERE / "out" / "v2" / "physics.csv"
PREREG = REPO / "prereg" / "expC.md"

# ---- code.


def original_path(k):
    return HERE / "arms" / ARM / f"{k}.py"


def copy_path(k):
    return COPIES / f"{k}.py"


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def check_originals():
    """Every original still hashes to what was recorded. Raises otherwise."""
    bad = [k for k in EXP4_LABEL if sha256(original_path(k)) != ORIGINAL_SHA256[k]]
    if bad:
        raise SystemExit(f"[expC] ORIGINALS MOVED: {bad}. Nothing runs.")
    return True


def block(k):
    hook = "\n".join("        " + ln for ln in HOOKS[k].splitlines())
    return BLOCK.format(banner=BANNER, rule="=" * (79 - 2 - len(BANNER)),
                        k=k, sha=ORIGINAL_SHA256[k], progress=PROGRESS_M,
                        window=WINDOW_S, switch=GOAL_SWITCH_M, hook=hook)


def build(k):
    """The patched source: the original's bytes, then the block."""
    src = original_path(k).read_text(encoding="utf-8")
    if not src.endswith("\n"):
        src += "\n"
    return src + block(k)


def diff(k):
    a = original_path(k).read_text(encoding="utf-8").splitlines(keepends=True)
    b = build(k).splitlines(keepends=True)
    return "".join(difflib.unified_diff(
        a, b, fromfile=f"demo/arms/{ARM}/{k}.py",
        tofile=f"demo/interventions/expC/{ARM}/{k}.py"))


def make():
    check_originals()
    COPIES.mkdir(parents=True, exist_ok=True)
    DIFFS.mkdir(parents=True, exist_ok=True)
    rows = []
    for k in EXP4_LABEL:
        copy_path(k).write_text(build(k), encoding="utf-8")
        (DIFFS / f"{ARM}_{k}.diff").write_text(diff(k), encoding="utf-8")
        rows.append((k, EXP4_LABEL[k], ORIGINAL_SHA256[k][:16],
                     sha256(copy_path(k))[:16]))
    check_originals()
    for r in rows:
        print("[expC] {}  {:26} original {}  patched {}".format(*r))
    print(f"[expC] {len(rows)} copies -> {COPIES.relative_to(REPO)}, "
          f"diffs -> {DIFFS.relative_to(REPO)}; originals unchanged")


def check():
    """Originals intact; every committed copy is exactly build(k); every copy
    starts with the original's bytes; every copy passes the import gate."""
    from demo.rollout import load_controller
    check_originals()
    for k in EXP4_LABEL:
        got = copy_path(k).read_text(encoding="utf-8")
        if got != build(k):
            raise SystemExit(f"[expC] {copy_path(k)} is not build({k!r})")
        if not got.startswith(original_path(k).read_text(encoding="utf-8")):
            raise SystemExit(f"[expC] {k}: copy does not start with the original")
        load_controller(copy_path(k))
    print(f"[expC] check OK: {len(EXP4_LABEL)} originals byte-identical to "
          f"their recorded sha256, copies == build(), import gate passes")


# ---- the kinematic check (sim-free) -----------------------------------------

def _kin_one(path, rep):
    from demo.rollout import load_controller, rollout
    seed, dx, dy, dyaw = exp3.rep_condition(rep)
    ctl = load_controller(path)()
    t0 = time.perf_counter()
    r = rollout(ctl, seed=seed, record=True, start_offset=(dx, dy, dyaw))
    wall = time.perf_counter() - t0
    traj = hashlib.sha256(json.dumps(
        [tuple(round(v, 6) for v in row[:4]) for row in r["trace"]]
    ).encode()).hexdigest()[:16]
    fires = getattr(ctl, "expc_fallback_fires", None)
    return {"legs": r["legs_completed"], "leg_times": json.dumps(r["leg_times"]),
            "collisions": r["collisions"], "path_m": r["path_length"],
            "crashed": r["crashed"] or "", "traj_sha": traj,
            "fires": "" if fires is None else len(fires),
            "fire_times": "" if fires is None else json.dumps([f["t"] for f in fires]),
            "expc_errors": "" if fires is None else len(ctl.expc_errors),
            "wall_s": round(wall, 1)}


def kinematic(reps):
    check_originals()
    RESULTS.mkdir(parents=True, exist_ok=True)
    out = RESULTS / "kinematic.csv"
    rows = []
    for k in EXP4_LABEL:
        for rep in reps:
            for version, path in (("original", original_path(k)),
                                  ("patched", copy_path(k))):
                row = {"k": k, "rep": rep, "version": version,
                       **_kin_one(path, rep)}
                rows.append(row)
                print("[expC] kin {k} rep {rep} {version:8} legs {legs} "
                      "fires {fires} coll {collisions} traj {traj_sha} "
                      "{crashed}".format(**row), flush=True)
    with out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    # Verdicts: no crash, no error inside our block, and a patched episode
    # with zero firings must be the original's trajectory exactly.
    by = {(r["k"], r["rep"], r["version"]): r for r in rows}
    problems = []
    for k in EXP4_LABEL:
        for rep in reps:
            o, p = by[(k, rep, "original")], by[(k, rep, "patched")]
            if p["crashed"]:
                problems.append(f"{k} r{rep}: patched copy crashed {p['crashed']}")
            if p["expc_errors"]:
                problems.append(f"{k} r{rep}: {p['expc_errors']} errors in the block")
            if p["fires"] == 0 and p["traj_sha"] != o["traj_sha"]:
                problems.append(f"{k} r{rep}: 0 fires but trajectory differs")
    print(f"[expC] -> {out.relative_to(REPO)}")
    print("[expC] kinematic check: " + ("OK" if not problems else
                                        "PROBLEMS\n  " + "\n  ".join(problems)))
    return not problems


# ---- the physics runs ---------------------------------------------------------

PHYS_FIELDS = ["order", "k", "label", "rep", "seed", "dx", "dy", "dyaw",
               "legs", "success", "leg_times", "sim_seconds", "contacts",
               "path_m", "track_v_med", "track_v_p95", "track_wz_med",
               "step_ms_med", "step_ms_p95", "step_ms_max",
               "fires", "fire_times", "fire_step_ms", "expc_errors",
               "load1_before", "load1_mean", "load1_max", "wall_seconds",
               "power_profile", "controller_sha", "prereg_commit", "trace",
               "note"]


def power_profile():
    try:
        return subprocess.run(["powerprofilesctl", "get"], text=True,
                              capture_output=True, timeout=20).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


def prereg_commit():
    """The commit that first added prereg/expC.md, or "" if it is uncommitted."""
    r = subprocess.run(["git", "log", "--diff-filter=A", "--format=%H", "--",
                        str(PREREG.relative_to(REPO))], cwd=str(REPO),
                       text=True, capture_output=True)
    lines = r.stdout.split()
    return lines[-1] if lines else ""


def physics_order(reps):
    """Rep-major, and within a rep the order exp3.job_order gives one arm."""
    jobs = [(ARM, k, copy_path(k)) for k in EXP4_LABEL]
    return exp3.job_order(jobs, reps)


def read_csv(path):
    if not pathlib.Path(path).exists():
        return []
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def write_csv(path, rows, fields):
    tmp = pathlib.Path(str(path) + ".tmp")
    with tmp.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        w.writerows(sorted(rows, key=lambda r: int(r["order"])))
    os.replace(tmp, path)


def physics(reps, require_profile="performance"):
    check_originals()
    check()
    commit = prereg_commit()
    if not commit:
        raise SystemExit("[expC] prereg/expC.md is not committed. The design "
                         "is committed BEFORE the first physics episode.")
    RESULTS.mkdir(parents=True, exist_ok=True)
    out = RESULTS / "physics.csv"
    traces = RESULTS / "traces"
    traces.mkdir(exist_ok=True)
    rows = read_csv(out)
    done = {(r["k"], str(r["rep"])) for r in rows if r["legs"] not in ("", None)}
    order = physics_order(reps)
    print(f"[expC] {len(order)} episodes, {len(done)} done, prereg {commit[:12]}",
          flush=True)
    for i, (_, k, path, rep) in enumerate(order):
        if (k, str(rep)) in done:
            continue
        prof = power_profile()
        if require_profile and prof != require_profile:
            print(f"[expC] PAUSED before #{i}: power profile is {prof!r}, need "
                  f"{require_profile!r}. Resumable.", flush=True)
            return False
        seed, dx, dy, dyaw = exp3.rep_condition(rep)
        trace = traces / f"expC__{ARM}_{k}__r{rep}.csv.gz"
        cmd = [sys.executable, str(HERE / "record_isaac.py"), str(path),
               "--no-record", "--seed", str(seed), "--start-dx", repr(dx),
               "--start-dy", repr(dy), "--start-dyaw", repr(dyaw),
               "--trace", str(trace)]
        print(f"[expC] #{i} {k} rep {rep} running (profile {prof}, load "
              f"{os.getloadavg()[0]:.2f})", flush=True)
        load_before = round(os.getloadavg()[0], 2)
        t0 = time.time()
        try:
            proc = subprocess.run(cmd, cwd=str(REPO), capture_output=True,
                                  text=True, timeout=1800)
            stdout, stderr = proc.stdout, proc.stderr
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout.decode() if isinstance(exc.stdout, bytes) else (exc.stdout or "")
            stderr = "TIMEOUT after 1800 s"
        wall = round(time.time() - t0, 1)
        js = exp3.parse_result_json(stdout)
        row = {"order": i, "k": k, "label": EXP4_LABEL[k], "rep": rep,
               "seed": seed, "dx": dx, "dy": dy, "dyaw": dyaw,
               "wall_seconds": wall, "load1_before": load_before,
               "prereg_commit": commit,
               "trace": str(trace.relative_to(RESULTS)) if trace.exists() else ""}
        after = power_profile()
        row["power_profile"] = prof if after == prof else f"{prof}->{after}"
        if js is None:
            tail = (stdout or stderr or "").strip().splitlines()[-3:]
            row.update({"legs": "", "note": "NO RESULT_JSON: " + " | ".join(tail)[:200]})
        else:
            fb = js.get("fallback") or {}
            fires = fb.get("fires") or []
            row.update({key: js.get(key) for key in (
                "legs", "success", "sim_seconds", "contacts", "path_m",
                "track_v_med", "track_v_p95", "track_wz_med", "step_ms_med",
                "step_ms_p95", "step_ms_max", "load1_mean", "load1_max",
                "controller_sha")})
            row.update({
                "leg_times": json.dumps(js.get("leg_times", [])),
                "fires": len(fires) if fb else "",
                "fire_times": json.dumps([f["t"] for f in fires]),
                "fire_step_ms": json.dumps([f["step_ms"] for f in fires]),
                "expc_errors": len(fb.get("errors") or []) if fb else "",
                "note": ("controller raised: " + js["raised"])[:200]
                        if js.get("raised") else ""})
        rows = [r for r in rows if (r["k"], str(r["rep"])) != (k, str(rep))]
        rows.append(row)
        write_csv(out, rows, PHYS_FIELDS)
        print(f"[expC] #{i} {k} rep {rep} legs {row.get('legs')} fires "
              f"{row.get('fires')} times {row.get('leg_times')} ({wall}s wall, "
              f"profile {row['power_profile']}) {row.get('note', '')}", flush=True)
    print(f"[expC] -> {out.relative_to(REPO)}")
    return True


# ---- the report -----------------------------------------------------------------

def paired(exp3_rows, expc_rows):
    """(k, rep) -> (unpatched Exp 3 row, patched Exp C row), armB only."""
    base = {(r["k"], str(r["rep"])): r for r in exp3_rows if r["arm"] == ARM}
    out = {}
    for r in expc_rows:
        key = (r["k"], str(r["rep"]))
        if r.get("legs") in ("", None):
            continue
        out[key] = (base.get(key), r)
    return out


def controller_success(rows):
    """Majority of reps with legs == 4 (Exp 3's primary rule)."""
    s = [exp3.is_success(r["legs"]) for r in rows if r and r.get("legs") not in ("", None)]
    return (sum(s) * 2 > len(s)) if s else None, sum(s), len(s)


def sign_test(n_up, n_down):
    """Exact two-sided binomial test on discordant pairs (McNemar exact)."""
    from math import comb
    n = n_up + n_down
    if n == 0:
        return 1.0
    lo = min(n_up, n_down)
    return min(1.0, 2.0 * sum(comb(n, i) for i in range(lo + 1)) / 2 ** n)


# The pre-registered verdict (prereg/expC.md §5), as code.
RESCUE_FOR_SUPPORT = 3


def verdict(rescued, broken):
    if broken:
        return "HARMS"            # any unpatched success turned into a failure
    if rescued >= RESCUE_FOR_SUPPORT:
        return "SUPPORTS"
    if rescued <= 1:
        return "DOES NOT SUPPORT"
    return "INCONCLUSIVE"


def analyse(exp3_rows, expc_rows, ks=None):
    """The primary analysis: per controller, majority-of-3 success unpatched
    (its own Exp 3 rows) vs patched, paired on (k, rep). Only reps present in
    BOTH files count, so an incomplete sweep cannot pair unlike with unlike."""
    ks = list(ks or EXP4_LABEL)
    pairs = paired(exp3_rows, expc_rows)
    per, ep_up, ep_down = {}, 0, 0
    for k in ks:
        reps = sorted(rep for (kk, rep), (u, p) in pairs.items()
                      if kk == k and u is not None and u.get("legs") not in ("", None))
        us = [pairs[(k, r)][0] for r in reps]
        ps = [pairs[(k, r)][1] for r in reps]
        su, nu_s, nu = controller_success(us)
        sp, np_s, npp = controller_success(ps)
        per[k] = {"reps": reps, "unpatched": su, "patched": sp,
                  "unpatched_eps": f"{nu_s}/{nu}", "patched_eps": f"{np_s}/{npp}",
                  "legs_u": [int(u["legs"]) for u in us],
                  "legs_p": [int(p["legs"]) for p in ps],
                  "fires": [int(p["fires"]) if str(p.get("fires", "")).strip() else None
                            for p in ps]}
        for u, p in zip(us, ps):
            a, b = exp3.is_success(u["legs"]), exp3.is_success(p["legs"])
            ep_up += (not a) and b
            ep_down += a and not b
    rescued = sum(1 for v in per.values() if v["unpatched"] is False and v["patched"])
    broken = sum(1 for v in per.values() if v["unpatched"] and v["patched"] is False)
    return {"per": per, "rescued": rescued, "broken": broken,
            "n_unpatched_fail": sum(1 for v in per.values() if v["unpatched"] is False),
            "n_unpatched_success": sum(1 for v in per.values() if v["unpatched"]),
            "verdict": verdict(rescued, broken),
            "sign_p": sign_test(rescued, broken),
            "episodes_up": ep_up, "episodes_down": ep_down,
            "episode_sign_p": sign_test(ep_up, ep_down)}


def report():
    exp3_rows = read_csv(EXP3_PHYSICS)
    expc_rows = read_csv(RESULTS / "physics.csv")
    print("⚠️ INTERVENTION ON MODEL-WRITTEN CODE. Patched copies are not armB "
          "samples.\nUNDER PHYSICS, one world, in_container false. "
          "success = all 4 legs; controller = majority of its reps.\n")
    for name, ks in (("PRIMARY: all 7", list(EXP4_LABEL)),
                     ("incremental_only (5)", [k for k, v in EXP4_LABEL.items()
                                               if v == "incremental_only"]),
                     ("leg-2 failures k0,k1,k2,k7", ["k0", "k1", "k2", "k7"])):
        r = analyse(exp3_rows, expc_rows, ks)
        print(f"== {name}")
        for k, v in r["per"].items():
            print(f"  {k} {EXP4_LABEL[k]:26} reps {','.join(v['reps']) or '-':6} "
                  f"legs unpatched {v['legs_u']} -> patched {v['legs_p']}  "
                  f"fires {v['fires']}  ctrl {v['unpatched']} -> {v['patched']}")
        print(f"  rescued {r['rescued']}/{r['n_unpatched_fail']}, broken "
              f"{r['broken']}/{r['n_unpatched_success']}, verdict {r['verdict']}, "
              f"sign p {r['sign_p']:.3f}; episodes +{r['episodes_up']} "
              f"-{r['episodes_down']}, p {r['episode_sign_p']:.3f}\n")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=("make", "check", "kinematic", "physics",
                                    "report", "diff"))
    ap.add_argument("--reps", default="0,1,2")
    ap.add_argument("--k", default=None)
    ap.add_argument("--require-profile", default="performance")
    a = ap.parse_args(argv)
    reps = [int(r) for r in a.reps.split(",")]
    if a.cmd == "make":
        make()
    elif a.cmd == "check":
        check()
    elif a.cmd == "diff":
        print(diff(a.k or "k0"))
    elif a.cmd == "kinematic":
        sys.exit(0 if kinematic(reps) else 1)
    elif a.cmd == "physics":
        sys.exit(0 if physics(reps, a.require_profile or None) else 3)
    else:
        report()


if __name__ == "__main__":
    main()
