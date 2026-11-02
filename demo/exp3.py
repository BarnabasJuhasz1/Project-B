"""
demo/exp3.py

EXP 3's DESIGN AS DATA: the repetitions, the perturbations, the episode order,
the per-step trace and what is read off it. `Mindfire_Experiment_Plan.md` §3.

    rep 0   seed 0, start offset (0, 0, 0)   THE ORIGINAL CONDITION. Must
                                            reproduce demo/out/physics.csv.
    rep 1   seed 1, start offset REP_OFFSETS[1]
    rep 2   seed 2, start offset REP_OFFSETS[2]

⚠️ **WHY THE PERTURBATION EXISTS (plan R6).** Same-machine Isaac Sim is
deterministic, so three unperturbed reps of a controller with no wall-clock
budget would be three copies of one episode and the "3 reps" would be 1. The
start pose and the scenario seed are what make reps 1-2 different roll-outs.

⚠️ **THE SAME OFFSETS GO TO EVERY CONTROLLER**, so the design stays paired: rep
1 of arm0/k3 and rep 1 of armB/k3 start from the identical pose. The offsets
were drawn ONCE from `PERTURB_SEED` and are written out below as literals; a
test re-draws them and fails if the two ever disagree.

⚠️ **SUCCESS IS ALL FOUR LEGS** (`legs == 4`). `physics_sweep.py` used to print
"solved" at legs >= 3 while the report counted four trips; the plan fixes four.

SIM-FREE. Nothing here imports isaacsim, and `demo/test_exp3.py` tests all of
it with no simulator.
"""
import csv
import gzip
import io
import json
import math
import random
import statistics

# ---- below this line is data, not code.

PERTURB_SEED = 3003           # numpy default_rng; draws (dx, dy, dyaw) per rep
OFFSET_XY = 0.10              # U(-0.10, +0.10) m in x and in y
OFFSET_YAW = 0.15             # U(-0.15, +0.15) rad
ORDER_SEED = 3                # random.Random; the interleaved episode order
N_REPS = 3
LEGS_FOR_SUCCESS = 4          # all four trips, fixed in advance

# rep -> (scenario seed, dx m, dy m, dyaw rad). Rep 0 is the original
# condition and is NOT drawn. Reps 1-2: `draw_offsets()`, rounded to mm/mrad.
REP_CONDITIONS = {
    0: (0, 0.0, 0.0, 0.0),
    1: (1, -0.088, 0.077, 0.023),
    2: (2, 0.038, -0.069, 0.066),
}

# The per-step trace. One row per CONTROL step (50 Hz), gzip CSV, fixed
# precision so a 180 s episode is ~0.2 MB rather than ~2 MB.
#   door     0 open / 1 shut
#   gap      analytic clearance - ROBOT_RADIUS, m (contact is gap <= 0)
#   step_ms  the controller's own step() wall time -- wall-clock-budgeted
#            controllers (plan R5) are visible here
#   err_v    |realised - commanded| planar velocity over this step, m/s
#   err_wz   realised - commanded yaw rate over this step, rad/s
# err_* belong to the step whose command they judge; the last row has none.
TRACE_COLUMNS = ("t", "leg", "door", "x", "y", "yaw", "vx", "vy", "wz",
                 "gap", "contact", "step_ms", "err_v", "err_wz")
_FMT = ("{:.2f}", "{:d}", "{:d}", "{:.4f}", "{:.4f}", "{:.4f}", "{:.3f}",
        "{:.3f}", "{:.3f}", "{:.3f}", "{:d}", "{:.2f}", "{:.4f}", "{:.4f}")

# ---- code.


def draw_offsets(seed=PERTURB_SEED):
    """Re-draw the rep 1-2 offsets. Only the test calls this: the committed
    table above is what the sweep uses."""
    import numpy as np
    rng = np.random.default_rng(seed)
    out = {}
    for rep in range(1, N_REPS):
        dx, dy = rng.uniform(-OFFSET_XY, OFFSET_XY, 2)
        dyaw = rng.uniform(-OFFSET_YAW, OFFSET_YAW)
        out[rep] = (rep, round(float(dx), 3), round(float(dy), 3),
                    round(float(dyaw), 3))
    return out


def rep_condition(rep):
    """(seed, dx, dy, dyaw) for a rep."""
    if rep not in REP_CONDITIONS:
        raise KeyError(f"no rep {rep}; the design has reps "
                       f"{sorted(REP_CONDITIONS)}")
    return REP_CONDITIONS[rep]


def job_order(controllers, reps=range(N_REPS), seed=ORDER_SEED):
    """Interleave (arm, k, path) x rep into the order the sweep runs them.

    Rep-major blocks: every rep 0 runs first (so the determinism check against
    physics.csv reads early), and a controller's reps are a whole block apart
    rather than back to back.

    Within a block the order is a JITTERED STRATIFIED shuffle, not a plain one:
    each arm's controllers are shuffled, the j-th of an arm's n gets the
    position (j + u) / n with u ~ U(0, 1), and the block is sorted on it. Every
    arm is therefore spread evenly across the block -- a plain shuffle of 17
    put five of one arm in a row -- so slow drift on the machine (load,
    thermal) cannot line up with arm (plan R5). A block boundary that would put
    the same controller twice in a row is broken by swapping in the next job.
    """
    rng = random.Random(seed)
    order, controllers = [], list(controllers)
    arms = sorted({c[0] for c in controllers})
    for rep in reps:
        keyed = []
        for arm in arms:
            mine = [c for c in controllers if c[0] == arm]
            rng.shuffle(mine)
            for j, (a, k, path) in enumerate(mine):
                keyed.append(((j + rng.random()) / len(mine), (a, k, path, rep)))
        block = [job for _, job in sorted(keyed, key=lambda kv: kv[0])]
        if order and len(block) > 1 and block[0][:2] == order[-1][:2]:
            block[0], block[1] = block[1], block[0]
        order.extend(block)
    return order


def is_success(legs):
    return legs is not None and int(legs) >= LEGS_FOR_SUCCESS


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def tracking_errors(poses, commands, dt):
    """Per control step: (err_v m/s, err_wz rad/s), the gap between the Twist
    held over the step and what the gait actually did.

    poses[i] = (x, y, yaw) when command i was issued; commands[i] = (vx, vy,
    wz). Needs poses[i+1], so the last step has no error (None). The commanded
    body-frame velocity is rotated by the yaw at the START of the step -- the
    same integration the kinematic rollout uses, so this is exactly "how far
    physics departs from the unicycle" (review M1).
    """
    out = []
    for i in range(len(commands)):
        if i + 1 >= len(poses):
            out.append((None, None))
            continue
        x, y, yaw = poses[i]
        nx, ny, nyaw = poses[i + 1]
        vx, vy, wz = commands[i]
        cvx = vx * math.cos(yaw) - vy * math.sin(yaw)
        cvy = vx * math.sin(yaw) + vy * math.cos(yaw)
        err_v = math.hypot((nx - x) / dt - cvx, (ny - y) / dt - cvy)
        err_wz = wrap(nyaw - yaw) / dt - wz
        out.append((err_v, err_wz))
    return out


def write_trace(path, rows):
    """rows: sequences in TRACE_COLUMNS order (err_* may be None). Writes a
    gzip CSV with mtime 0, so the same episode gives the same bytes."""
    buf = io.StringIO()
    buf.write(",".join(TRACE_COLUMNS) + "\n")
    for r in rows:
        buf.write(",".join("" if v is None else f.format(v)
                           for f, v in zip(_FMT, r)) + "\n")
    data = buf.getvalue().encode()
    if str(path).endswith(".gz"):
        with open(path, "wb") as raw, gzip.GzipFile(
                fileobj=raw, mode="wb", mtime=0, filename="") as gz:
            gz.write(data)
    else:
        with open(path, "wb") as fh:
            fh.write(data)


def read_trace(path):
    """-> list of dicts, numbers parsed. For analysis and the tests."""
    opener = gzip.open if str(path).endswith(".gz") else open
    with opener(path, "rt") as fh:
        rows = []
        for r in csv.DictReader(fh):
            rows.append({k: (None if v == "" else
                             int(v) if k in ("leg", "door", "contact")
                             else float(v)) for k, v in r.items()})
        return rows


def _q(values, q):
    v = sorted(values)
    if not v:
        return None
    return v[min(len(v) - 1, int(round(q * (len(v) - 1))))]


def summarise(rows, dt):
    """Episode-level numbers read off a trace's rows (TRACE_COLUMNS order)."""
    idx = {c: i for i, c in enumerate(TRACE_COLUMNS)}
    contacts, prev, path = 0, 0, 0.0
    for i, r in enumerate(rows):
        if r[idx["contact"]] and not prev:
            contacts += 1
        prev = r[idx["contact"]]
        if i:
            path += math.hypot(r[idx["x"]] - rows[i - 1][idx["x"]],
                               r[idx["y"]] - rows[i - 1][idx["y"]])
    ms = [r[idx["step_ms"]] for r in rows]
    ev = [r[idx["err_v"]] for r in rows if r[idx["err_v"]] is not None]
    ew = [abs(r[idx["err_wz"]]) for r in rows if r[idx["err_wz"]] is not None]
    rnd = lambda v, n: None if v is None else round(v, n)
    return {
        "steps": len(rows),
        "contacts": contacts,
        "contact_steps": sum(1 for r in rows if r[idx["contact"]]),
        "path_m": round(path, 2),
        "step_ms_med": rnd(statistics.median(ms) if ms else None, 2),
        "step_ms_p95": rnd(_q(ms, 0.95), 2),
        "step_ms_max": rnd(max(ms) if ms else None, 2),
        "track_v_med": rnd(statistics.median(ev) if ev else None, 4),
        "track_v_p95": rnd(_q(ev, 0.95), 4),
        "track_wz_med": rnd(statistics.median(ew) if ew else None, 4),
        "track_wz_p95": rnd(_q(ew, 0.95), 4),
    }


RESULT_JSON_PREFIX = "RESULT_JSON "


def parse_result_json(stdout):
    """The last RESULT_JSON line record_isaac.py printed, or None."""
    for line in reversed((stdout or "").splitlines()):
        i = line.find(RESULT_JSON_PREFIX)
        if i >= 0:
            return json.loads(line[i + len(RESULT_JSON_PREFIX):])
    return None


def _times(s):
    if isinstance(s, list):
        return [float(x) for x in s]
    s = (s or "").strip()
    return [float(x) for x in json.loads(s)] if s else []


def compare_rep0(rows, reference_rows):
    """Rep 0 against the pilot's physics.csv, row by row.

    Match = same legs AND the same leg times AND the same sim_seconds. Leg
    times are rounded to 10 ms in both files, so equality is exact equality of
    the episode as far as either file can see it. Returns a list of
    (arm, k, verdict, detail) with verdict MATCH / DIFFER / MISSING.
    """
    ref = {(r["arm"], r["k"]): r for r in reference_rows}
    out = []
    for r in rows:
        if str(r.get("rep")) != "0":
            continue
        key = (r["arm"], r["k"])
        if key not in ref:
            out.append((*key, "MISSING", "no row in the reference"))
            continue
        a, b = r, ref[key]
        same = (str(a["legs"]) == str(b["legs"])
                and _times(a["leg_times"]) == _times(b["leg_times"])
                and float(a["sim_seconds"] or -1) == float(b["sim_seconds"] or -2))
        detail = (f"legs {a['legs']} vs {b['legs']}, times {a['leg_times']} "
                  f"vs {b['leg_times']}, t {a['sim_seconds']} vs "
                  f"{b['sim_seconds']}")
        out.append((*key, "MATCH" if same else "DIFFER", detail))
    return out
