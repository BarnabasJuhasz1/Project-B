"""
demo/exp3_report.py

EXP 3, READ OFF THE SWEEP. Sim-free; reads demo/out/v2/ and prints (and
writes demo/out/v2/SUMMARY.md):

1. rep 0 against the pilot's demo/out/physics.csv, row by row, with each
   controller's wall-clock budget marked (`perf_counter` / `time.time` in its
   source), since that is the one way rep 0 can legitimately differ;
2. per arm: per-episode success, majority-of-3 success per controller,
   within-controller agreement (all reps the same verdict), success per rep;
3. tracking error from the traces (median over steps of err_v / |err_wz|,
   then summarised over episodes);
4. load average and wall time;
5. the quarantined low-power rows against their reruns.

Success = all four legs (demo/exp3.py). ⚠️ Descriptive. The pre-registered
test is `demo/analyse.py` / PREREG-v2, on fresh samples, not this pilot.

    python demo/exp3_report.py
"""
import csv
import pathlib
import re
import statistics
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from demo import exp3, tracking

V2 = REPO / "demo" / "out" / "v2"
PILOT = REPO / "demo" / "out" / "physics.csv"
BUDGET_RE = re.compile(r"time\.(perf_counter|time|monotonic)\s*\(")


def rows(path):
    return list(csv.DictReader(open(path))) if path.exists() else []


def source(arm, k):
    if arm == "(base)":
        return REPO / "generate" / "base" / "base_controller.py"
    return REPO / "demo" / "arms" / arm / f"{k}.py"


def budgeted(arm, k):
    return bool(BUDGET_RE.search(source(arm, k).read_text()))


def main():
    data = rows(V2 / "physics.csv")
    out = []
    p = out.append
    p(f"# Exp 3 summary — {len(data)} episodes\n")
    p("Success = all four legs. UNDER PHYSICS, local machine, in_container "
      "false. One hand-authored world. Descriptive only.\n")

    # 1. rep 0
    cmp = exp3.compare_rep0(data, rows(PILOT))
    ref = {(r["arm"], r["k"]): r for r in rows(PILOT)}
    n_match = sum(1 for c in cmp if c[2] == "MATCH")
    p(f"## 1. Rep 0 vs demo/out/physics.csv: {n_match}/{len(cmp)} exact\n")
    p("| arm | k | wall-clock budget | verdict | legs v2 / pilot | "
      "4-leg success same? | detail |")
    p("|---|---|---|---|---|---|---|")
    for arm, k, verdict, detail in sorted(cmp):
        r = next(x for x in data if (x["arm"], x["k"], x["rep"]) == (arm, k, "0"))
        same = exp3.is_success(r["legs"]) == exp3.is_success(ref[(arm, k)]["legs"])
        p(f"| {arm} | {k} | {'yes' if budgeted(arm, k) else 'no'} | {verdict} | "
          f"{r['legs']} / {ref[(arm, k)]['legs']} | {'yes' if same else 'NO'} | "
          f"{'' if verdict == 'MATCH' else detail} |")
    p("")

    # 2. per arm
    per = {}
    for r in data:
        per.setdefault(r["arm"], {}).setdefault(r["k"], {})[int(r["rep"])] = r
    p("## 2. Per arm\n")
    p("| arm | controllers | episodes success | maj-of-3 success | "
      "all 3 reps agree | rep 0 | rep 1 | rep 2 | mean legs |")
    p("|---|---|---|---|---|---|---|---|---|")
    for arm, ctrls in per.items():
        eps = [exp3.is_success(r["legs"]) for c in ctrls.values() for r in c.values()]
        maj = sum(1 for c in ctrls.values()
                  if sum(exp3.is_success(r["legs"]) for r in c.values()) * 2 > len(c))
        agree = sum(1 for c in ctrls.values()
                    if len({exp3.is_success(r["legs"]) for r in c.values()}) == 1)
        by_rep = []
        for rep in range(exp3.N_REPS):
            v = [exp3.is_success(c[rep]["legs"]) for c in ctrls.values() if rep in c]
            by_rep.append(f"{sum(v)}/{len(v)}")
        legs = [int(r["legs"]) for c in ctrls.values() for r in c.values()]
        p(f"| {arm} | {len(ctrls)} | {sum(eps)}/{len(eps)} | {maj}/{len(ctrls)} | "
          f"{agree}/{len(ctrls)} | {' | '.join(by_rep)} | "
          f"{sum(legs) / len(legs):.2f} |")
    p("\nPer controller (legs in rep 0 / 1 / 2):\n")
    for arm, ctrls in per.items():
        cells = []
        for k in sorted(ctrls, key=lambda s: (len(s), s)):
            cells.append(f"{k} " + "/".join(ctrls[k][r]["legs"]
                                            for r in sorted(ctrls[k])))
        p(f"- **{arm}**: " + ", ".join(cells))
    p("")

    # 3. tracking error -- demo/tracking.py has the definition
    from scipy.stats import mannwhitneyu
    p("## 3. Tracking error: commanded vs realised displacement per 50 Hz step\n")
    p("e_k = |p_{k+1} - (p_k + R(yaw_k)[vx, vy] dt)|, i.e. how far the frozen "
      "gait lands from where the held Twist would have put a unicycle. "
      "Per episode: the median over its steps.\n")
    ep = {}
    p("| arm | k | rep | legs | median e (mm/step) | mean e (mm/step) | "
      "median err_v (m/s) | median abs err_wz (rad/s) |")
    p("|---|---|---|---|---|---|---|---|")
    for arm, ctrls in per.items():
        for k in sorted(ctrls, key=lambda s: (len(s), s)):
            for rep in sorted(ctrls[k]):
                r = ctrls[k][rep]
                if not (r["trace"] and (V2 / r["trace"]).exists()):
                    continue
                e = tracking.episode_error(exp3.read_trace(V2 / r["trace"]))
                ep.setdefault(arm, []).append((k, e))
                p(f"| {arm} | {k} | {rep} | {r['legs']} | "
                  f"{e['disp_err_mm_med']:.2f} | {e['disp_err_mm_mean']:.2f} | "
                  f"{e['err_v_med']:.3f} | {e['err_wz_med']:.3f} |")
    p("\nPer arm (episode medians):\n")
    p("| arm | episodes | median e mm/step | IQR | median err_v m/s | IQR |")
    p("|---|---|---|---|---|---|")
    for arm, v in ep.items():
        d = [e["disp_err_mm_med"] for _, e in v]
        w = [e["err_v_med"] for _, e in v]
        q1, m, q3 = tracking.quartiles(d)
        w1, wm, w3 = tracking.quartiles(w)
        p(f"| {arm} | {len(v)} | {m:.2f} | [{q1:.2f}, {q3:.2f}] | {wm:.3f} | "
          f"[{w1:.3f}, {w3:.3f}] |")
    if "arm0" in ep and "armB" in ep:
        a0 = [e["disp_err_mm_med"] for _, e in ep["arm0"]]
        ab = [e["disp_err_mm_med"] for _, e in ep["armB"]]
        u, pv = mannwhitneyu(a0, ab, alternative="two-sided")
        c0 = [statistics.fmean(e["disp_err_mm_med"] for kk, e in ep["arm0"] if kk == k)
              for k in sorted({k for k, _ in ep["arm0"]})]
        cb = [statistics.fmean(e["disp_err_mm_med"] for kk, e in ep["armB"] if kk == k)
              for k in sorted({k for k, _ in ep["armB"]})]
        uc, pc = mannwhitneyu(c0, cb, alternative="two-sided")
        p(f"\narm0 vs armB, Mann-Whitney U two-sided: **controller level** "
          f"(mean of each controller's episode medians, n = {len(c0)} vs "
          f"{len(cb)}) U = {uc:.1f}, p = {pc:.3f}; episode level (n = "
          f"{len(a0)} vs {len(ab)}, NOT independent -- reps of one controller "
          f"are clustered) U = {u:.1f}, p = {pv:.3f}.")
    p("")

    # 4. machine
    f = lambda key: [float(r[key]) for r in data if r.get(key) not in ("", None)]
    wall = f("wall_seconds")
    p("## 4. Machine\n")
    p(f"- wall time: {sum(wall) / 3600:.2f} h over {len(wall)} episodes "
      f"(median {statistics.median(wall):.0f} s, range {min(wall):.0f}-{max(wall):.0f} s)")
    for key in ("load1_before", "load1_mean", "load1_max"):
        v = f(key)
        p(f"- {key}: median {statistics.median(v):.2f}, range "
          f"{min(v):.2f}-{max(v):.2f} (22 cores)")
    prof = {}
    for r in data:
        prof[r.get("power_profile") or "(not recorded)"] = \
            prof.get(r.get("power_profile") or "(not recorded)", 0) + 1
    p(f"- power profile per episode: {prof}")
    p("")

    # 5. low power
    low = rows(V2 / "physics_lowpower.csv")
    if low:
        p("## 5. Quarantined low-power rows vs their reruns\n")
        p("| arm | k | rep | low-power legs (wall s) | rerun legs (wall s) | "
          "leg times low-power | leg times rerun |")
        p("|---|---|---|---|---|---|---|")
        for q in low:
            r = per.get(q["arm"], {}).get(q["k"], {}).get(int(q["rep"]))
            if r:
                p(f"| {q['arm']} | {q['k']} | {q['rep']} | {q['legs']} "
                  f"({q['wall_seconds']}) | {r['legs']} ({r['wall_seconds']}) | "
                  f"{q['leg_times']} | {r['leg_times']} |")
        p("")
    text = "\n".join(out)
    print(text)
    (V2 / "SUMMARY.md").write_text(text + "\n")


if __name__ == "__main__":
    main()
