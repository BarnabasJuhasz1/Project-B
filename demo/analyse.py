"""
demo/analyse.py

THE PRE-REGISTERED ANALYSIS FOR MINDFIRE (Exp 1-4), AS CODE. `PREREG-v2.md`
describes it; this file IS it. Written and committed before any confirmatory
data exists (Mindfire_Experiment_Plan.md §2.5, §5 step 4). Run it exactly as
committed on the confirmatory data; anything else is labelled exploratory.

What it computes, in the order the pre-registration lists it:

    PRIMARY (confirmatory)   per-controller success = all 4 trips in >= 2 of 3
                             physics reps. FRESH samples only. Fisher exact,
                             two-sided, Holm across
                                 H1 armB vs arm0   (replication)
                                 H2 armB vs armX   (content vs length)
                                 H3 armX vs arm0   (length / distraction)
                             + Wilson 95% per arm, Newcombe 95% per difference,
                             + the interpretation table, applied mechanically.
    SECONDARY (exploratory,  Holm within the family:
      Holm within family)      S1 mean trips per controller, Mann-Whitney U
                               S2 leg-2 time (median over reps; never = +inf), MWU
                               S3 collisions per metre, MWU
                               S4 kinematic success (same majority rule), Fisher
                             S1-S4 on each of the three contrasts, plus
                               S5 pooled arm0 vs armB, pilot + fresh, Fisher.
    EXP 3 (descriptive)      per-episode success, cluster bootstrap over
                             controllers (10,000, fixed seed); within-controller
                             agreement (3 of 3 reps agree); tracking error per
                             arm; rep 0 vs the pilot's physics.csv.
    EXP 4 (exploratory)      label distributions per arm; E10 (arm0 with any
                             incremental repair); Fisher on success for
                             incremental_only vs has-a-full-replan-path, over all
                             controllers and within armB; Cohen's kappa.
    TOOL USE (disclosure)    per arm: calls that attempted / executed a CLI
                             tool, CLI versions, success split by tool use, and
                             an EXPLORATORY tool-free re-run of H1-H3.
    PILOT (not pre-registered) legs == 4: physics 7/8 vs 2/8 (physics.csv, 1
                             rep); kinematic 5/8 vs 6/8 (repeatability.csv, 3
                             reps); tool attempts 5/8 vs 0/8.

⚠️ THE UNIT IS THE CONTROLLER. Reps of one controller are not independent and
never enter a Fisher test as separate samples.

⚠️ IT REFUSES AN INCOMPLETE SET. A confirmatory controller with fewer than 3
valid physics reps raises `Incomplete` -- re-run the episode, do not analyse
around it. `--allow-incomplete` exists for looking at a partial sweep and
stamps every number EXPLORATORY.

Inputs (all under demo/out/v2/ unless overridden):

    physics.csv      arm, k, rep, legs [, leg_times, contacts|collisions,
                     path_m|path_length, track_v_med, note]   -- Exp 3
    kinematic.csv    arm, k, rep, legs [, ...]                -- secondary S4
    code_labels.csv  arm, k (or blind_id + --code-key), planner_type, map,
                     velocity_selection, stuck_handling, ...  -- Exp 4
                     If a `rater` column exists: rows with rater == "consensus"
                     are the labels, the other two raters give kappa.
    tool_use.csv     arm, k, tool_calls, tools_executed, cli_version -- one row
                     per generation call, from its CLI session log (step 5)
    demo/transcripts/<arm>__k<k>__FAILED.json  a generation failure: counted as
                     a 0-trip controller in every rep (stays in the denominator).

    python demo/analyse.py --pilot                 # physics 7/8 vs 2/8 (p 0.041), kinematic 5/8 vs 6/8
    python demo/analyse.py --k 24                  # the confirmatory analysis
    python demo/analyse.py --k 24 --json out.json  # + a machine-readable copy
    python demo/analyse.py --k 24 --allow-incomplete   # a partial sweep, EXPLORATORY
"""
import argparse
import csv
import json
import math
import pathlib
import sys

import numpy as np
from scipy.stats import fisher_exact, mannwhitneyu, norm

DEMO = pathlib.Path(__file__).resolve().parent

# ---- below this line is data, not code -------------------------------------

ALPHA = 0.05
Z = float(norm.ppf(0.975))              # 1.959964 -- every 95% interval here

LEGS_FOR_SUCCESS = 4                    # "all four trips"; physics_sweep's >= 3 is NOT it
N_REPS = 3                              # Exp 3: rep 0 original, reps 1-2 perturbed
MAJORITY = 2                            # success in >= 2 of 3 reps

# ⚠️ DECISION FOR BARNA: k per arm, fresh. 24 or 30 (PREREG-v2.md §3).
# None means "not decided": the confirmatory run then requires --k.
K_CONFIRMATORY = None

PILOT_ARMS = ("arm0", "armB")
PILOT_KS = tuple(range(8))              # arm0/armB k0..k7 are the pilot
FRESH_START = {"arm0": 8, "armB": 8, "armX": 0}   # where fresh k's begin
ARMS = ("arm0", "armB", "armX")

# (name, a, b): the difference is always rate(a) - rate(b)
CONTRASTS = (("H1", "armB", "arm0"),
             ("H2", "armB", "armX"),
             ("H3", "armX", "arm0"))

BOOT_N = 10_000
BOOT_SEED = 20261102

# Exp 4 (demo/CODING.md §1). Codes are CODING.md's, verbatim.
PLANNER_TYPES = ("full_every_step", "full_periodic", "incremental_only",
                 "incremental_with_fallback", "no_global_planner",
                 "unclassifiable")
INCREMENTAL_ONLY = {"incremental_only"}
HAS_FULL_REPLAN = {"full_every_step", "full_periodic",
                   "incremental_with_fallback"}
ANY_INCREMENTAL = {"incremental_only", "incremental_with_fallback"}
LABEL_COLUMNS = ("planner_type", "map", "velocity_selection", "stuck_handling",
                 "claims_dstar", "dstar_variant")

# ---- code -------------------------------------------------------------------


class Incomplete(RuntimeError):
    """The data set the pre-registration describes is not all there."""


# --- intervals and tests ------------------------------------------------------

def wilson(x, n, z=Z):
    """Wilson score interval for x/n. (lo, hi); (nan, nan) for n == 0."""
    if n == 0:
        return (math.nan, math.nan)
    p = x / n
    den = 1 + z * z / n
    centre = (p + z * z / (2 * n)) / den
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return (max(0.0, centre - half), min(1.0, centre + half))


def newcombe(x1, n1, x2, n2, z=Z):
    """Newcombe (1998) hybrid-score interval, method 10, for p1 - p2."""
    p1, p2 = x1 / n1, x2 / n2
    l1, u1 = wilson(x1, n1, z)
    l2, u2 = wilson(x2, n2, z)
    d = p1 - p2
    return (d - math.sqrt((p1 - l1) ** 2 + (u2 - p2) ** 2),
            d + math.sqrt((u1 - p1) ** 2 + (p2 - l2) ** 2))


def fisher(x1, n1, x2, n2):
    return float(fisher_exact([[x1, n1 - x1], [x2, n2 - x2]],
                              alternative="two-sided")[1])


def holm(pvalues, alpha=ALPHA):
    """Holm step-down. dict name -> p  =>  dict name -> (p_adj, reject).
    p_adj is the Holm-adjusted p (monotone, capped at 1)."""
    items = sorted(pvalues.items(), key=lambda kv: (kv[1], kv[0]))
    m = len(items)
    out, running, still = {}, 0.0, True
    for i, (name, p) in enumerate(items):
        adj = min(1.0, max(running, (m - i) * p))
        running = adj
        still = still and p <= alpha / (m - i) + 1e-12
        out[name] = (adj, still)
    return out


def mwu(a, b):
    """Mann-Whitney U, two-sided, exact where scipy allows. a, b: lists."""
    if len(a) == 0 or len(b) == 0:
        return math.nan
    if len(set(a) | set(b)) == 1:
        return 1.0                       # every value tied: no evidence at all
    return float(mannwhitneyu(a, b, alternative="two-sided").pvalue)


def cohen_kappa(r1, r2):
    """Unweighted Cohen's kappa for two equal-length label lists."""
    n = len(r1)
    if n == 0:
        return math.nan
    cats = sorted(set(r1) | set(r2))
    po = sum(a == b for a, b in zip(r1, r2)) / n
    pe = sum((r1.count(c) / n) * (r2.count(c) / n) for c in cats)
    return 1.0 if pe == 1 else (po - pe) / (1 - pe)


# --- reading -------------------------------------------------------------------

def _k(v):
    """'k12' / '12' -> 12. Anything else (e.g. 'reference') -> None."""
    s = str(v).strip()
    s = s[1:] if s.startswith("k") else s
    return int(s) if s.isdigit() else None


def _float(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def _times(s):
    s = (s or "").strip()
    return [float(x) for x in json.loads(s)] if s else []


def read_rows(path):
    with open(path, newline="") as fh:
        return list(csv.DictReader(fh))


def episodes(rows, single_rep=False):
    """CSV rows -> {(arm, k): {rep: episode}}. Rows with no legs value are
    APPARATUS failures (no RESULT line) and are dropped here: they are re-run,
    never scored. A duplicate (arm, k, rep) raises."""
    out = {}
    for r in rows:
        k = _k(r["k"])
        if k is None or r["arm"] not in ARMS:
            continue                     # the (base) reference row, never tested
        if str(r.get("legs", "")).strip() in ("", "None"):
            continue
        rep = 0 if single_rep else int(r["rep"])
        eps = out.setdefault((r["arm"], k), {})
        if rep in eps:
            raise ValueError(f"duplicate episode {r['arm']} k{k} rep {rep}")
        contacts = _float(r.get("contacts", r.get("collisions")))
        path_m = _float(r.get("path_m", r.get("path_length")))
        eps[rep] = {
            "legs": int(r["legs"]),
            "leg_times": _times(r.get("leg_times")),
            "contacts": contacts,
            "path_m": path_m,
            "track_v_med": _float(r.get("track_v_med")),
            "note": r.get("note", ""),
        }
    return out


def generation_failures(transcripts=DEMO / "transcripts"):
    """{(arm, k)} with a FAILED transcript (demo/generate.py's record)."""
    out = set()
    for p in pathlib.Path(transcripts).glob("*__FAILED.json"):
        arm, k, _ = p.name.split("__", 2)
        if _k(k) is not None:
            out.add((arm, _k(k)))
    return out


def expected(k, sample):
    """The controllers a sample consists of. sample: 'fresh' | 'pilot'."""
    if sample == "pilot":
        return [(a, i) for a in PILOT_ARMS for i in PILOT_KS]
    return [(a, FRESH_START[a] + i) for a in ARMS for i in range(k)]


# --- the controller verdict ---------------------------------------------------

def verdict(eps, n_reps=N_REPS, majority=None):
    """{rep: episode} -> controller record. success = legs == 4 in >= 2 of 3
    (a strict majority of n_reps in general; 1 of 1 for the one-rep pilot)."""
    if majority is None:
        majority = MAJORITY if n_reps == N_REPS else n_reps // 2 + 1
    reps = sorted(eps)
    ok = [eps[r]["legs"] >= LEGS_FOR_SUCCESS for r in reps]
    legs = [eps[r]["legs"] for r in reps]
    leg2 = []
    for r in reps:
        t = eps[r]["leg_times"]
        leg2.append(t[2] if len(t) > 2 else math.inf)   # never completed = +inf
    c = [eps[r]["contacts"] for r in reps]
    m = [eps[r]["path_m"] for r in reps]
    cpm = (sum(c) / sum(m)) if (None not in c and None not in m
                                and sum(m) > 0) else None
    tv = [eps[r]["track_v_med"] for r in reps if eps[r]["track_v_med"] is not None]
    return {
        "n_reps": len(reps),
        "rep_success": ok,
        "success": sum(ok) >= majority,
        "mean_legs": float(np.mean(legs)),
        "leg2_time": float(np.median(leg2)),
        "coll_per_m": cpm,
        "track_v_med": float(np.median(tv)) if tv else None,
        "agree": len(set(ok)) == 1,
    }


def controllers(eps_by_ctrl, keys, failed=frozenset(), n_reps=N_REPS,
                allow_incomplete=False):
    """The controller table for `keys`. A key with a generation failure and no
    episodes is a 0-trip controller. Missing or short -> Incomplete."""
    out, problems = {}, []
    for key in keys:
        eps = eps_by_ctrl.get(key)
        if not eps and key in failed:
            eps = {r: {"legs": 0, "leg_times": [], "contacts": 0.0,
                       "path_m": 0.0, "track_v_med": None,
                       "note": "generation failed"} for r in range(n_reps)}
        if not eps:
            problems.append(f"{key[0]} k{key[1]}: no episodes")
            continue
        if len(eps) != n_reps:
            problems.append(f"{key[0]} k{key[1]}: {len(eps)}/{n_reps} reps")
            if not allow_incomplete:
                continue
        out[key] = {**verdict(eps, n_reps), "generation_failed": key in failed
                    and key not in eps_by_ctrl}
    if problems and not allow_incomplete:
        raise Incomplete("; ".join(problems[:10])
                         + (f" (+{len(problems) - 10} more)" if len(problems) > 10 else ""))
    return out, problems


def by_arm(ctrl):
    out = {}
    for (arm, _), v in sorted(ctrl.items()):
        out.setdefault(arm, []).append(v)
    return out


# --- analyses -------------------------------------------------------------------

def arm_summary(groups):
    out = {}
    for arm, vs in groups.items():
        x, n = sum(v["success"] for v in vs), len(vs)
        out[arm] = {"x": x, "n": n, "rate": x / n if n else math.nan,
                    "wilson": wilson(x, n),
                    "generation_failures": sum(v.get("generation_failed", False)
                                               for v in vs)}
    return out


def interpret(res):
    """The interpretation table of PREREG-v2.md §6, mechanically.
    'armX close to arm0' is operationalised as H3 NOT rejected, and the content
    reading additionally requires armB to differ from arm0 and armX in the SAME
    direction."""
    r = {h: res[h]["holm_reject"] for h in ("H1", "H2", "H3")}
    same_dir = (np.sign(res["H1"]["diff"]) == np.sign(res["H2"]["diff"])
                and res["H1"]["diff"] != 0)
    if not r["H1"]:
        return "NOT_REPLICATED", ("H1 not rejected: the pilot result did not "
                                  "replicate. Report the pilot as a pilot and "
                                  "the confirmatory null as the result.")
    if r["H2"] and not r["H3"] and same_dir:
        direction = "worse" if res["H1"]["diff"] < 0 else "better"
        return "CONTENT", (f"H1 and H2 rejected, H3 not: the paper's CONTENT "
                           f"makes the controllers {direction} under physics.")
    if r["H3"] and not r["H2"]:
        return "LENGTH", ("H1 and H3 rejected, H2 not: a long document, not "
                          "this paper, is enough. The content claim is withdrawn.")
    return "NO_CLAIM", ("No pre-registered pattern: report the intervals and "
                        "make no directional claim.")


def primary(groups):
    s = arm_summary(groups)
    res = {}
    for h, a, b in CONTRASTS:
        A, B = s[a], s[b]
        res[h] = {"a": a, "b": b, "diff": A["rate"] - B["rate"],
                  "newcombe": newcombe(A["x"], A["n"], B["x"], B["n"]),
                  "p": fisher(A["x"], A["n"], B["x"], B["n"])}
    for h, (adj, rej) in holm({h: res[h]["p"] for h in res}).items():
        res[h]["p_holm"], res[h]["holm_reject"] = adj, rej
    reading, text = interpret(res)
    return {"arms": s, "tests": res, "reading": reading, "reading_text": text}


def secondary(fresh, pooled, kin_fresh=None):
    """The exploratory family. Holm across every test that could be computed;
    an unavailable test is listed, never silently dropped."""
    tests, unavailable = {}, []
    for h, a, b in CONTRASTS:
        A, B = fresh.get(a, []), fresh.get(b, [])
        tests[f"S1 mean trips {h}"] = mwu([v["mean_legs"] for v in A],
                                          [v["mean_legs"] for v in B])
        tests[f"S2 leg-2 time {h}"] = mwu([v["leg2_time"] for v in A],
                                          [v["leg2_time"] for v in B])
        # A controller that never moved (generation failure, raised at t = 0)
        # has no collisions-per-metre; it is left out of S3 and counted.
        ca = [v["coll_per_m"] for v in A if v["coll_per_m"] is not None]
        cb = [v["coll_per_m"] for v in B if v["coll_per_m"] is not None]
        if not ca or not cb:
            unavailable.append(f"S3 coll/m {h}: contacts or path_m missing")
        else:
            tests[f"S3 coll/m {h}"] = mwu(ca, cb)
            dropped = len(A) + len(B) - len(ca) - len(cb)
            if dropped:
                unavailable.append(f"S3 coll/m {h}: {dropped} controller(s) "
                                   f"with no path left out")
        if kin_fresh is None:
            unavailable.append(f"S4 kinematic {h}: no kinematic.csv")
        else:
            KA, KB = kin_fresh.get(a, []), kin_fresh.get(b, [])
            xa, xb = sum(v["success"] for v in KA), sum(v["success"] for v in KB)
            tests[f"S4 kinematic {h}"] = fisher(xa, len(KA), xb, len(KB))
    P0, PB = pooled.get("arm0", []), pooled.get("armB", [])
    tests["S5 pooled arm0 vs armB"] = fisher(
        sum(v["success"] for v in PB), len(PB), sum(v["success"] for v in P0), len(P0))
    finite = {k: v for k, v in tests.items() if not math.isnan(v)}
    unavailable += [f"{k}: empty group" for k in tests if k not in finite]
    adj = holm(finite)
    return {"tests": {k: {"p": finite[k], "p_holm": adj[k][0],
                          "holm_reject": adj[k][1]} for k in finite},
            "unavailable": unavailable,
            "pooled": arm_summary({a: v for a, v in pooled.items()
                                   if a in PILOT_ARMS})}


def cluster_bootstrap(groups, a, b, n=BOOT_N, seed=BOOT_SEED):
    """Per-EPISODE success rate difference a - b, resampling CONTROLLERS within
    each arm. Percentile 95% interval."""
    rng = np.random.default_rng(seed)
    def cols(vs):                        # (successes, episodes) per controller
        return (np.array([sum(v["rep_success"]) for v in vs], float),
                np.array([len(v["rep_success"]) for v in vs], float))
    (sa, na), (sb, nb) = cols(groups[a]), cols(groups[b])
    point = sa.sum() / na.sum() - sb.sum() / nb.sum()
    ia = rng.integers(0, len(sa), (n, len(sa)))
    ib = rng.integers(0, len(sb), (n, len(sb)))
    d = sa[ia].sum(1) / na[ia].sum(1) - sb[ib].sum(1) / nb[ib].sum(1)
    lo, hi = np.percentile(d, [2.5, 97.5])
    return {"diff": float(point), "ci": (float(lo), float(hi))}


def exp3(groups, pilot_rep0=None, reference=None):
    out = {"per_episode": {}, "agreement": {}, "tracking": {}, "bootstrap": {}}
    for arm, vs in groups.items():
        eps = [s for v in vs for s in v["rep_success"]]
        out["per_episode"][arm] = {"x": sum(eps), "n": len(eps)}
        out["agreement"][arm] = {"agree": sum(v["agree"] for v in vs),
                                 "n": len(vs)}
        tv = [v["track_v_med"] for v in vs if v["track_v_med"] is not None]
        out["tracking"][arm] = (None if not tv else
                                {"median": float(np.median(tv)),
                                 "iqr": tuple(float(q) for q in
                                              np.percentile(tv, [25, 75])),
                                 "n": len(tv)})
    for h, a, b in CONTRASTS:
        if a in groups and b in groups:
            out["bootstrap"][h] = cluster_bootstrap(groups, a, b)
    if pilot_rep0 is not None and reference is not None:
        out["rep0_check"] = rep0_check(pilot_rep0, reference)
    return out


def rep0_check(eps_by_ctrl, reference):
    """Rep 0 of each pilot controller vs demo/out/physics.csv: legs and leg
    times must match. A mismatch is REPORTED, it does not stop the analysis."""
    rows = []
    for key, ref in sorted(reference.items()):
        got = eps_by_ctrl.get(key, {}).get(0)
        if got is None:
            rows.append((*key, "MISSING"))
        elif got["legs"] == ref[0]["legs"] and got["leg_times"] == ref[0]["leg_times"]:
            rows.append((*key, "MATCH"))
        else:
            rows.append((*key, f"DIFFER legs {got['legs']} vs {ref[0]['legs']}"))
    return {"rows": rows, "match": sum(r[2] == "MATCH" for r in rows),
            "n": len(rows)}


def read_labels(path, key_path=None):
    """-> (labels {(arm, k): row}, kappa {column: (kappa, n)})."""
    rows = read_rows(path)
    if key_path:
        key = {r["blind_id"]: (r["arm"], _k(r["k"])) for r in read_rows(key_path)}
        for r in rows:
            r["arm"], r["k"] = key[r["blind_id"]][0], str(key[r["blind_id"]][1])
    kappa = {}
    if rows and "rater" in rows[0]:
        raters = sorted({r["rater"] for r in rows} - {"consensus"})
        if len(raters) == 2:
            per = {rt: {(r["arm"], _k(r["k"])): r for r in rows if r["rater"] == rt}
                   for rt in raters}
            common = sorted(set(per[raters[0]]) & set(per[raters[1]]))
            for col in LABEL_COLUMNS:
                if col in rows[0]:
                    kappa[col] = (cohen_kappa([per[raters[0]][c][col] for c in common],
                                              [per[raters[1]][c][col] for c in common]),
                                  len(common))
        rows = [r for r in rows if r["rater"] == "consensus"]
    labels = {}
    for r in rows:
        pt = r["planner_type"].strip()
        if pt not in PLANNER_TYPES:
            raise ValueError(f"unknown planner_type {pt!r} (CODING.md §1)")
        labels[(r["arm"], _k(r["k"]))] = r
    return labels, kappa


def exp4(labels, ctrl):
    """ctrl: every controller with a physics verdict (pilot + fresh, all arms)."""
    dist = {}
    for (arm, _), r in sorted(labels.items()):
        for col in LABEL_COLUMNS:
            if col in r:
                d = dist.setdefault(col, {}).setdefault(arm, {})
                d[r[col]] = d.get(r[col], 0) + 1
    e10 = {}
    for arm in ARMS:
        ls = [r for (a, _), r in labels.items() if a == arm]
        if ls:
            x = sum(r["planner_type"] in ANY_INCREMENTAL for r in ls)
            e10[arm] = {"x": x, "n": len(ls), "wilson": wilson(x, len(ls))}

    def assoc(arm=None):
        inc, full = [], []
        for key, r in labels.items():
            if (arm and key[0] != arm) or key not in ctrl:
                continue
            if r["planner_type"] in INCREMENTAL_ONLY:
                inc.append(ctrl[key]["success"])
            elif r["planner_type"] in HAS_FULL_REPLAN:
                full.append(ctrl[key]["success"])
        if not inc or not full:
            return {"incremental_only": (sum(inc), len(inc)),
                    "has_full_replan": (sum(full), len(full)), "p": None}
        return {"incremental_only": (sum(inc), len(inc)),
                "has_full_replan": (sum(full), len(full)),
                "p": fisher(sum(inc), len(inc), sum(full), len(full))}
    unlabelled = sorted(set(ctrl) - set(labels))
    return {"distribution": dist, "e10": e10, "assoc_all": assoc(),
            "assoc_armB": assoc("armB"), "unlabelled": unlabelled}


def pilot_kinematic_episodes(path):
    """demo/out/repeatability.csv (one row per controller, `legs` a list of
    the 3 kinematic reps on the FIXED world) -> {(arm, k): {rep: episode}}.
    ⚠️ demo/out/results.csv is the FIRST, defective world and is never used."""
    out = {}
    for r in read_rows(path):
        k = _k(r["k"])
        if k is None or r["arm"] not in ARMS:
            continue
        out[(r["arm"], k)] = {i: {"legs": int(l), "leg_times": [],
                                  "contacts": None, "path_m": None,
                                  "track_v_med": None, "note": ""}
                              for i, l in enumerate(json.loads(r["legs"]))}
    return out


def _pilot_compare(eps, n_reps):
    ctrl, _ = controllers(eps, expected(None, "pilot"), n_reps=n_reps)
    g = {a: [v for (arm, _), v in sorted(ctrl.items()) if arm == a]
         for a in PILOT_ARMS}
    s = arm_summary(g)
    A, B = s["armB"], s["arm0"]
    legs3 = {a: sum(min(eps[(a, k)][r]["legs"] for r in eps[(a, k)]) >= 3
                    for k in PILOT_KS) for a in PILOT_ARMS}
    return {"arms": s,
            "diff": A["rate"] - B["rate"],
            "newcombe": newcombe(A["x"], A["n"], B["x"], B["n"]),
            "p": fisher(A["x"], A["n"], B["x"], B["n"]),
            "legs_ge_3": legs3,
            "mean_legs": {a: float(np.mean([v["mean_legs"] for v in g[a]])) for a in g}}


def pilot(path=DEMO / "out" / "physics.csv",
          kinematic=DEMO / "out" / "repeatability.csv", tool_use=None):
    """The pilot, success = legs == 4 throughout. NOT pre-registered.
    physics: one rep, success is that rep. kinematic: 3 reps, the same
    majority rule as v2 (only armB k5 is unstable, [2, 1, 2], and fails either
    way). Gives physics 7/8 vs 2/8 and kinematic 5/8 vs 6/8 -- the report's
    kinematic 7/8 counted legs >= 3 (Mindfire_Step0_Verification.md check 6)."""
    out = {"physics": _pilot_compare(episodes(read_rows(path), single_rep=True), 1)}
    if kinematic and pathlib.Path(kinematic).exists():
        out["kinematic"] = _pilot_compare(pilot_kinematic_episodes(kinematic), N_REPS)
    if tool_use and pathlib.Path(tool_use).exists():
        out["tool_use"] = tool_use_summary(read_tool_use(tool_use),
                                           expected(None, "pilot"))
    return out


# --- tool use per generation call: a recorded covariate, never a test of H1-H3 --

def read_tool_use(path):
    """arm, k, tool_calls, tools_executed [, cli_version] -> {(arm, k): row}.
    tool_calls = tool calls the model ATTEMPTED in the CLI session (denied or
    not); tools_executed = those that actually ran."""
    out = {}
    for r in read_rows(path):
        k = _k(r["k"])
        if k is None:
            continue
        out[(r["arm"], k)] = {"tool_calls": int(r["tool_calls"]),
                              "tools_executed": int(r.get("tools_executed") or 0),
                              "cli_version": r.get("cli_version", "")}
    return out


def tool_use_summary(tools, keys, ctrl=None):
    """Per arm: calls with >= 1 attempted tool call; pairwise Fisher
    (descriptive); CLI versions seen; and, given controller verdicts, success
    split by tool use. Missing calls are listed, never assumed tool-free."""
    per, missing = {}, []
    for key in keys:
        if key not in tools:
            missing.append(f"{key[0]} k{key[1]}")
            continue
        d = per.setdefault(key[0], {"used": 0, "n": 0, "executed": 0,
                                    "cli": set(), "succ_used": [0, 0],
                                    "succ_none": [0, 0]})
        t = tools[key]
        used = t["tool_calls"] > 0
        d["used"] += used; d["n"] += 1
        d["executed"] += t["tools_executed"] > 0
        d["cli"].add(t["cli_version"])
        if ctrl and key in ctrl:
            bucket = d["succ_used" if used else "succ_none"]
            bucket[0] += ctrl[key]["success"]; bucket[1] += 1
    for d in per.values():
        d["cli"] = sorted(d["cli"])
    tests = {}
    arms = sorted(per)
    for i, a in enumerate(arms):
        for b in arms[i + 1:]:
            tests[f"{a} vs {b}"] = fisher(per[a]["used"], per[a]["n"],
                                          per[b]["used"], per[b]["n"])
    return {"per_arm": per, "fisher": tests, "missing": missing}


def tool_free_sensitivity(fresh, tools):
    """EXPLORATORY: H1-H3 Fisher (unadjusted) among controllers whose call
    attempted no tool. Answers 'is the arm difference the tool asymmetry?'."""
    keep = {c: v for c, v in fresh.items()
            if c in tools and tools[c]["tool_calls"] == 0}
    s = arm_summary(by_arm(keep))
    res = {}
    for h, a, b in CONTRASTS:
        if a in s and b in s:
            A, B = s[a], s[b]
            res[h] = {"x": (A["x"], A["n"], B["x"], B["n"]),
                      "p": fisher(A["x"], A["n"], B["x"], B["n"])}
    return res


# --- the run ----------------------------------------------------------------------

def analyse(k, physics, kinematic=None, labels=None, code_key=None,
            reference=DEMO / "out" / "physics.csv",
            transcripts=DEMO / "transcripts", allow_incomplete=False,
            tool_use=None):
    failed = generation_failures(transcripts) if transcripts else set()
    eps = episodes(read_rows(physics))
    fresh, problems = controllers(eps, expected(k, "fresh"), failed,
                                  allow_incomplete=allow_incomplete)
    pilot_ctrl, p2 = controllers(eps, expected(k, "pilot"), failed,
                                 allow_incomplete=True)
    problems += [f"(pilot) {p}" for p in p2]
    fresh_g = by_arm(fresh)
    pooled_g = by_arm({**pilot_ctrl, **{c: v for c, v in fresh.items()
                                        if c[0] in PILOT_ARMS}})
    kin_g = None
    if kinematic and pathlib.Path(kinematic).exists():
        kin, kp = controllers(episodes(read_rows(kinematic)), expected(k, "fresh"),
                              failed, allow_incomplete=allow_incomplete)
        kin_g = by_arm(kin)
        problems += [f"(kinematic) {p}" for p in kp]
    ref = None
    if reference and pathlib.Path(reference).exists():
        ref = episodes(read_rows(reference), single_rep=True)
    out = {
        "status": "EXPLORATORY (incomplete data)" if (allow_incomplete and problems)
                  else "CONFIRMATORY",
        "k": k, "problems": problems,
        "primary": primary(fresh_g),
        "secondary": secondary(fresh_g, pooled_g, kin_g),
        "exp3": exp3({**fresh_g, **{f"pilot:{a}": v for a, v in
                                    by_arm(pilot_ctrl).items()}},
                     eps, ref),
    }
    if labels and pathlib.Path(labels).exists():
        lab, kappa = read_labels(labels, code_key)
        out["exp4"] = {**exp4(lab, {**pilot_ctrl, **fresh}), "kappa": kappa}
    if tool_use and pathlib.Path(tool_use).exists():
        tools = read_tool_use(tool_use)
        out["tool_use"] = {
            "fresh": tool_use_summary(tools, expected(k, "fresh"), fresh),
            "pilot": tool_use_summary(tools, expected(k, "pilot"), pilot_ctrl),
            "tool_free_sensitivity": tool_free_sensitivity(fresh, tools)}
    else:
        out["tool_use"] = None
    return out


# --- printing ---------------------------------------------------------------------

def _ci(t):
    return f"[{t[0]:+.3f}, {t[1]:+.3f}]"


def _rate(s):
    return f"{s['x']:>3}/{s['n']:<3} {s['rate']:.3f}  Wilson [{s['wilson'][0]:.3f}, {s['wilson'][1]:.3f}]"


def _format_tools(t, L, indent="  "):
    for a, d in sorted(t["per_arm"].items()):
        line = (f"{indent}{a:6} tool calls attempted in {d['used']}/{d['n']} calls,"
                f" executed in {d['executed']}/{d['n']}  CLI {','.join(d['cli'])}")
        if d["succ_used"][1] + d["succ_none"][1]:
            line += (f"  success with tools {d['succ_used'][0]}/{d['succ_used'][1]},"
                     f" without {d['succ_none'][0]}/{d['succ_none'][1]}")
        L.append(line)
    for name, p in t["fisher"].items():
        L.append(f"{indent}tool-use rate {name}: Fisher p {p:.4f} (descriptive)")
    if t["missing"]:
        L.append(f"{indent}⚠ no tool-use record for {len(t['missing'])} call(s)")


def format_pilot(r):
    L = ["PILOT -- NOT PRE-REGISTERED, found after the kinematic-vs-physics flip.",
         "Hypothesis-generating only. Success = all 4 legs (legs == 4).",
         "-" * 72]
    for tag, src in (("physics", "demo/out/physics.csv, 1 rep"),
                     ("kinematic", "demo/out/repeatability.csv, majority of 3 reps")):
        if tag not in r:
            continue
        x = r[tag]
        L.append(f"{tag.upper()} ({src})")
        for a in PILOT_ARMS:
            L.append(f"  {a:6} {_rate(x['arms'][a])}  mean legs {x['mean_legs'][a]:.2f}")
        L.append(f"  armB - arm0 = {x['diff']:+.3f}  Newcombe {_ci(x['newcombe'])}"
                 f"  Fisher exact two-sided p = {x['p']:.4f}")
        L.append(f"  (legs >= 3, NOT the rule: arm0 {x['legs_ge_3']['arm0']}/8, "
                 f"armB {x['legs_ge_3']['armB']}/8)")
    if "tool_use" in r:
        L.append("TOOL USE PER GENERATION CALL (disclosure, not a test of H1-H3)")
        _format_tools(r["tool_use"], L)
    return "\n".join(L)


def format_report(o):
    L = [f"MINDFIRE ANALYSIS -- {o['status']} -- k = {o['k']} fresh per arm", "=" * 72]
    if o["problems"]:
        L.append("problems: " + "; ".join(o["problems"][:8]))
    p = o["primary"]
    L.append("PRIMARY (fresh only; success = 4/4 trips in >= 2 of 3 physics reps)")
    for a in ARMS:
        if a in p["arms"]:
            s = p["arms"][a]
            L.append(f"  {a:6} {_rate(s)}  gen. failures {s['generation_failures']}")
    for h, t in p["tests"].items():
        L.append(f"  {h} {t['a']}-{t['b']}: {t['diff']:+.3f} Newcombe {_ci(t['newcombe'])}"
                 f"  Fisher p {t['p']:.4f}  Holm p {t['p_holm']:.4f}"
                 f"  {'REJECT' if t['holm_reject'] else 'not rejected'}")
    L.append(f"  READING: {p['reading']} -- {p['reading_text']}")
    L.append("-" * 72)
    s = o["secondary"]
    L.append("SECONDARY (EXPLORATORY; Holm within the family)")
    for name, t in s["tests"].items():
        L.append(f"  {name:28} p {t['p']:.4f}  Holm p {t['p_holm']:.4f}"
                 f"  {'REJECT' if t['holm_reject'] else ''}")
    for u in s["unavailable"]:
        L.append(f"  unavailable: {u}")
    for a, v in s["pooled"].items():
        L.append(f"  pooled {a:6} {_rate(v)}")
    L.append("-" * 72)
    e = o["exp3"]
    L.append("EXP 3 (descriptive)")
    for a, v in e["per_episode"].items():
        ag = e["agreement"][a]; tr = e["tracking"][a]
        L.append(f"  {a:12} episodes {v['x']}/{v['n']}  3-of-3 agree {ag['agree']}/{ag['n']}"
                 + (f"  track_v median {tr['median']:.3f} m/s" if tr else ""))
    for h, b in e["bootstrap"].items():
        L.append(f"  {h} per-episode diff {b['diff']:+.3f}  cluster bootstrap {_ci(b['ci'])}")
    if "rep0_check" in e:
        r = e["rep0_check"]
        L.append(f"  rep 0 reproduces pilot physics.csv: {r['match']}/{r['n']}")
        L += [f"    {a} k{k}: {v}" for a, k, v in r["rows"] if v != "MATCH"]
    if "exp4" in o:
        x = o["exp4"]
        L.append("-" * 72)
        L.append("EXP 4 (EXPLORATORY, correlational)")
        for col, d in x["distribution"].items():
            L.append(f"  {col}: " + " | ".join(f"{a} {dict(sorted(c.items()))}"
                                              for a, c in d.items()))
        for a, v in x["e10"].items():
            L.append(f"  E10 {a}: any incremental repair {v['x']}/{v['n']}"
                     f"  Wilson [{v['wilson'][0]:.3f}, {v['wilson'][1]:.3f}]")
        for tag in ("assoc_all", "assoc_armB"):
            v = x[tag]
            L.append(f"  {tag}: incremental_only {v['incremental_only']} vs "
                     f"has_full_replan {v['has_full_replan']}  Fisher p "
                     + ("n/a" if v["p"] is None else f"{v['p']:.4f}"))
        for col, (kp, n) in x["kappa"].items():
            L.append(f"  kappa {col}: {kp:.3f} (n = {n})")
        if x["unlabelled"]:
            L.append(f"  unlabelled controllers: {len(x['unlabelled'])}")
    L.append("-" * 72)
    L.append("TOOL USE PER GENERATION CALL (recorded covariate; disclosure)")
    t = o.get("tool_use")
    if t is None:
        L.append("  ⚠ unavailable: no tool_use.csv -- PREREG-v2.md §5a requires one")
    else:
        L.append(" fresh:"); _format_tools(t["fresh"], L)
        L.append(" pilot:"); _format_tools(t["pilot"], L)
        for h, v in t["tool_free_sensitivity"].items():
            L.append(f"  EXPLORATORY tool-free {h}: {v['x'][0]}/{v['x'][1]} vs "
                     f"{v['x'][2]}/{v['x'][3]}  Fisher p {v['p']:.4f}")
    return "\n".join(L)


def main(argv=None):
    v2 = DEMO / "out" / "v2"
    ap = argparse.ArgumentParser()
    ap.add_argument("--pilot", action="store_true",
                    help="reproduce the pilot (7/8 vs 2/8) and stop")
    ap.add_argument("--pilot-csv", default=str(DEMO / "out" / "physics.csv"))
    ap.add_argument("--pilot-kinematic", default=str(DEMO / "out" / "repeatability.csv"))
    ap.add_argument("--pilot-tools", default=str(DEMO / "fixtures" / "pilot_tool_use.csv"))
    ap.add_argument("--tool-use", default=str(v2 / "tool_use.csv"))
    ap.add_argument("--k", type=int, default=K_CONFIRMATORY)
    ap.add_argument("--physics", default=str(v2 / "physics.csv"))
    ap.add_argument("--kinematic", default=str(v2 / "kinematic.csv"))
    ap.add_argument("--labels", default=str(v2 / "code_labels.csv"))
    ap.add_argument("--code-key", default=None)
    ap.add_argument("--transcripts", default=str(DEMO / "transcripts"))
    ap.add_argument("--allow-incomplete", action="store_true")
    ap.add_argument("--json", default=None)
    a = ap.parse_args(argv)

    if a.pilot or not pathlib.Path(a.physics).exists():
        if not a.pilot:
            print(f"[analyse] no confirmatory data at {a.physics}; pilot only.\n")
        r = pilot(a.pilot_csv, a.pilot_kinematic, a.pilot_tools)
        print(format_pilot(r))
        if a.json:
            pathlib.Path(a.json).write_text(json.dumps(r, indent=2, default=str))
        return 0
    if a.k is None:
        ap.error("k is not fixed in K_CONFIRMATORY yet; pass --k (PREREG-v2.md §3)")
    o = analyse(a.k, a.physics, a.kinematic, a.labels, a.code_key,
                reference=a.pilot_csv, transcripts=a.transcripts,
                tool_use=a.tool_use,
                allow_incomplete=a.allow_incomplete)
    print(format_report(o))
    if a.k != K_CONFIRMATORY:
        print(f"\n⚠ --k {a.k} differs from the frozen K_CONFIRMATORY "
              f"({K_CONFIRMATORY}): NOT the pre-registered run.")
    if a.json:
        pathlib.Path(a.json).write_text(json.dumps(o, indent=2, default=str))
    return 0


if __name__ == "__main__":
    sys.exit(main())
