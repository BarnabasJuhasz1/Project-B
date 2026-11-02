"""
demo/exp_ad_report.py

EXP A AND EXP D, READ OFF THE SWEEPS EXACTLY AS prereg/expA.md AND
prereg/expD.md SAY. Sim-free. Writes demo/out/expA/SUMMARY.md and
demo/out/expD/SUMMARY.md and prints both.

    python demo/exp_ad_report.py --exp3 <path to Exp 3's v2/physics.csv>

Exp 3's sweep (the blocked-maze reference for A, the normal-speed reference
for D) lives in demo/out/v2/ of whichever tree ran it; it is READ, never
written. The pilot's demo/out/physics.csv is read the same way.

Unit of analysis: the controller. Success = all four trips (exp3.is_success).
"""
import argparse
import json
import pathlib
import statistics
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from scipy.stats import binomtest, fisher_exact

from demo import exp3
from demo.exp_ad import ARMS, EXPERIMENTS, KS, clock_limited, read_rows

PILOT = REPO / "demo" / "out" / "physics.csv"
EXP3 = REPO / "demo" / "out" / "v2" / "physics.csv"


def fisher(x1, n1, x2, n2):
    return float(fisher_exact([[x1, n1 - x1], [x2, n2 - x2]],
                              alternative="two-sided")[1])


def legs(r):
    return None if r is None or r.get("legs") in ("", None) else int(r["legs"])


def num(v):
    return None if v in ("", None) else float(v)


def med(v):
    v = [x for x in v if x is not None]
    return round(statistics.median(v), 2) if v else None


def by_ctrl(rows, rep=None):
    out = {}
    for r in rows:
        if r.get("arm") not in ARMS or legs(r) is None:
            continue
        if rep is not None and str(r["rep"]) != str(rep):
            continue
        out.setdefault((r["arm"], r["k"]), []).append(r)
    return out


def majority(eps):
    s = [exp3.is_success(legs(r)) for r in eps]
    return sum(s) * 2 > len(s)


def trace_step_ms(csv_path, row):
    """(mean, p95) of step_ms over the episode, from its trace, or None."""
    if not row or not row.get("trace"):
        return None
    p = pathlib.Path(csv_path).parent / row["trace"]
    if not p.exists():
        return None
    ms = [r["step_ms"] for r in exp3.read_trace(p)]
    return (round(statistics.fmean(ms), 2), exp3._q(ms, 0.95)) if ms else None


def fmt_p(p):
    return f"{p:.3f}"


def report_a(rows_a, rows_exp3):
    A, B3 = by_ctrl(rows_a), by_ctrl(rows_exp3)
    L = ["# Exp A: open maze (door never shuts)", "",
         f"Pre-registration: `prereg/expA.md` @ "
         f"`{(rows_a[0]['prereg_commit'] if rows_a else '?')}`. "
         f"{len(rows_a)} episodes. Success = all four trips; a controller "
         "succeeds if it does so in at least 2 of its 3 reps.", "",
         "| arm | k | clock | open legs r0/r1/r2 | open maj | blocked (Exp 3) legs "
         "r0/r1/r2 | blocked maj | trip times, first rep (s) | contacts | track_v med |"
         " step_ms med |", "|" + "---|" * 11]
    counts = {arm: {"open": 0, "blocked": 0, "n": 0, "ep": 0, "eps": 0,
                    "blocked_n": 0} for arm in ARMS}
    pairs = {arm: [] for arm in ARMS}
    for arm in ARMS:
        for k in KS:
            eps = sorted(A.get((arm, k), []), key=lambda r: int(r["rep"]))
            blk = sorted(B3.get((arm, k), []), key=lambda r: int(r["rep"]))
            om = majority(eps) if len(eps) == 3 else None
            bm = majority(blk) if len(blk) == 3 else None
            c = counts[arm]
            if om is not None:
                c["n"] += 1
                c["open"] += om
                c["ep"] += sum(exp3.is_success(legs(r)) for r in eps)
                c["eps"] += len(eps)
            if bm is not None:
                c["blocked_n"] += 1
                c["blocked"] += bm
            if om is not None and bm is not None:
                pairs[arm].append((om, bm))
            path = REPO / "demo" / "arms" / arm / f"{k}.py"
            L.append(
                f"| {arm} | {k} | {int(clock_limited(path))} | "
                f"{'/'.join(str(legs(r)) for r in eps) or '-'} | "
                f"{'-' if om is None else int(om)} | "
                f"{'/'.join(str(legs(r)) for r in blk) or '-'} | "
                f"{'-' if bm is None else int(bm)} | "
                f"{eps[0]['leg_times'] if eps else '-'} | "
                f"{sum(int(r['contacts'] or 0) for r in eps) if eps else '-'} | "
                f"{med(num(r['track_v_med']) for r in eps)} | "
                f"{med(num(r['step_ms_med']) for r in eps)} |")
    a0, aB = counts["arm0"], counts["armB"]
    L += ["", "## Primary (prereg §4)", "",
          "| | arm0 | armB | Fisher p (two-sided) |", "|---|---|---|---|"]
    if a0["n"] and aB["n"]:
        L.append(f"| open maze, controllers (majority of 3) | {a0['open']}/{a0['n']}"
                 f" | {aB['open']}/{aB['n']} | "
                 f"{fmt_p(fisher(a0['open'], a0['n'], aB['open'], aB['n']))} |")
        L.append(f"| open maze, episodes (descriptive) | {a0['ep']}/{a0['eps']} | "
                 f"{aB['ep']}/{aB['eps']} | {fmt_p(fisher(a0['ep'], a0['eps'], aB['ep'], aB['eps']))}"
                 " (episodes not independent) |")
    if a0["blocked_n"] and aB["blocked_n"]:
        L.append(f"| blocked maze, Exp 3 (reference) | {a0['blocked']}/{a0['blocked_n']}"
                 f" | {aB['blocked']}/{aB['blocked_n']} | "
                 f"{fmt_p(fisher(a0['blocked'], a0['blocked_n'], aB['blocked'], aB['blocked_n']))} |")
    L += ["", "## Open vs blocked, paired per controller (secondary)", ""]
    for arm in ARMS:
        p = pairs[arm]
        if not p:
            continue
        ob = sum(1 for o, b in p if o and not b)
        bo = sum(1 for o, b in p if b and not o)
        mc = binomtest(ob, ob + bo).pvalue if ob + bo else 1.0
        L.append(f"- {arm}: succeeds open only {ob}, blocked only {bo}, both "
                 f"{sum(1 for o, b in p if o and b)}, neither "
                 f"{sum(1 for o, b in p if not o and not b)} "
                 f"(exact McNemar p = {fmt_p(mc)}, n = {len(p)})")
    # the prereg's reading
    if aB["n"] == 8 and a0["n"] == 8:
        p = fisher(a0["open"], 8, aB["open"], 8)
        if aB["open"] >= 6 and p >= 0.05:
            r = ("armB succeed in the open maze: their failures are tied to the "
                 "blocked route (replanning).")
        elif aB["open"] <= 4:
            r = ("armB fail even with the route open: the failure is NOT "
                 "specific to replanning.")
        else:
            r = "mixed (armB 5/8, or 6+/8 with p < 0.05): no directional claim."
        L += ["", f"**Pre-registered reading (prereg §6):** {r}"]
    return "\n".join(L) + "\n", counts


def report_d(rows_d, rows_exp3, pilot_rows, exp3_csv, d_csv):
    D, E = by_ctrl(rows_d, rep=0), by_ctrl(rows_exp3, rep=0)
    P = {(r["arm"], r["k"]): r for r in pilot_rows}
    L = ["# Exp D: slow machine (cgroup CPU limit)", "",
         f"Pre-registration: `prereg/expD.md` @ "
         f"`{(rows_d[0]['prereg_commit'] if rows_d else '?')}`. "
         f"CPU limit cpu.max = `{rows_d[0]['cpu_max'] if rows_d else '?'}` "
         f"(seen by every episode process). Rep 0 condition (seed 0, no offset), "
         f"door schedule as in Exp 3.", "",
         "| arm | k | clock | legs pilot | legs Exp 3 r0 | legs D | Δ (D − Exp3) |"
         " degraded | trip times D (s) | trip times Exp 3 (s) | contacts D/E3 |"
         " track_v med D/E3 | step_ms med D/E3 | step_ms p95 D/E3 | step_ms mean D/E3 | wall s D/E3 |",
         "|" + "---|" * 16]
    tab = []
    for arm in ARMS:
        for k in KS:
            d = (D.get((arm, k)) or [None])[0]
            e = (E.get((arm, k)) or [None])[0]
            pl = P.get((arm, k))
            clk = int(clock_limited(REPO / "demo" / "arms" / arm / f"{k}.py"))
            ld, le = legs(d), legs(e)
            deg = None if ld is None or le is None else ld < le
            tab.append({"arm": arm, "k": k, "clock": clk, "ld": ld, "le": le,
                        "lp": legs(pl), "deg": deg, "d": d, "e": e,
                        "succ": None if ld is None else exp3.is_success(ld)})
            g = lambda r, f: "-" if r is None else r.get(f, "-")
            tm_d, tm_e = trace_step_ms(d_csv, d), trace_step_ms(exp3_csv, e)
            L.append(
                f"| {arm} | {k} | {clk} | {legs(pl)} | {le} | {ld} | "
                f"{'-' if deg is None else ld - le} | "
                f"{'-' if deg is None else int(deg)} | {g(d, 'leg_times')} | "
                f"{g(e, 'leg_times')} | {g(d, 'contacts')}/{g(e, 'contacts')} | "
                f"{g(d, 'track_v_med')}/{g(e, 'track_v_med')} | "
                f"{g(d, 'step_ms_med')}/{g(e, 'step_ms_med')} | "
                f"{g(d, 'step_ms_p95')}/{g(e, 'step_ms_p95')} | "
                f"{tm_d[0] if tm_d else '-'}/{tm_e[0] if tm_e else '-'} | "
                f"{g(d, 'wall_seconds')}/{g(e, 'wall_seconds')} |")
    done = [t for t in tab if t["deg"] is not None]
    L += ["", "## Primary (prereg §4)", ""]
    s = {arm: [t["succ"] for t in tab if t["arm"] == arm and t["succ"] is not None]
         for arm in ARMS}
    e3 = {arm: [exp3.is_success(t["le"]) for t in tab if t["arm"] == arm
                and t["le"] is not None] for arm in ARMS}
    L += ["| | arm0 | armB | Fisher p (two-sided) |", "|---|---|---|---|"]
    if all(s.values()):
        L.append(f"| slow machine, all four trips | {sum(s['arm0'])}/{len(s['arm0'])}"
                 f" | {sum(s['armB'])}/{len(s['armB'])} | "
                 f"{fmt_p(fisher(sum(s['arm0']), len(s['arm0']), sum(s['armB']), len(s['armB'])))} |")
    if all(e3.values()):
        L.append(f"| normal speed, Exp 3 rep 0 (reference) | {sum(e3['arm0'])}/{len(e3['arm0'])}"
                 f" | {sum(e3['armB'])}/{len(e3['armB'])} | "
                 f"{fmt_p(fisher(sum(e3['arm0']), len(e3['arm0']), sum(e3['armB']), len(e3['armB'])))} |")
    if done:
        dg = {arm: [t["deg"] for t in done if t["arm"] == arm] for arm in ARMS}
        if all(dg.values()):
            L.append(f"| degraded vs Exp 3 rep 0 (fewer trips) | {sum(dg['arm0'])}/{len(dg['arm0'])}"
                     f" | {sum(dg['armB'])}/{len(dg['armB'])} | "
                     f"{fmt_p(fisher(sum(dg['arm0']), len(dg['arm0']), sum(dg['armB']), len(dg['armB'])))} |")
        cl = [t["deg"] for t in done if t["clock"]]
        nc = [t["deg"] for t in done if not t["clock"]]
        L += ["", "| | clock-limited | not clock-limited | Fisher p (two-sided) |",
              "|---|---|---|---|"]
        if cl and nc:
            L.append(f"| degraded vs Exp 3 rep 0 | {sum(cl)}/{len(cl)} | {sum(nc)}/{len(nc)}"
                     f" | {fmt_p(fisher(sum(cl), len(cl), sum(nc), len(nc)))} |")
            dl = lambda grp: round(statistics.fmean(t["ld"] - t["le"] for t in grp), 2)
            L.append(f"| mean Δ trips | {dl([t for t in done if t['clock']])} | "
                     f"{dl([t for t in done if not t['clock']])} | — |")
        # the noise floor: did Exp 3 rep 0 already differ from the pilot?
        nz = [t for t in done if t["lp"] is not None]
        if nz:
            L.append(f"\nNoise floor at normal speed: Exp 3 rep 0 trips differ from "
                     f"the pilot's in {sum(t['le'] != t['lp'] for t in nz)}/{len(nz)} "
                     f"controllers. Degraded beyond BOTH normal-speed runs "
                     f"(D < min(pilot, Exp 3)): clock-limited "
                     f"{sum(t['ld'] < min(t['le'], t['lp']) for t in nz if t['clock'])}"
                     f"/{sum(1 for t in nz if t['clock'])}, not "
                     f"{sum(t['ld'] < min(t['le'], t['lp']) for t in nz if not t['clock'])}"
                     f"/{sum(1 for t in nz if not t['clock'])}.")
    L += ["", "## Planning time per step (secondary, prereg §5)", "",
          "| group | median of per-controller step_ms med, D | same, Exp 3 r0 | "
          "median ratio D/E3 | median ratio p95 |", "|---|---|---|---|---|"]
    for name, grp in (("clock-limited", [t for t in tab if t["clock"]]),
                      ("not clock-limited", [t for t in tab if not t["clock"]]),
                      ("arm0", [t for t in tab if t["arm"] == "arm0"]),
                      ("armB", [t for t in tab if t["arm"] == "armB"])):
        pr = [t for t in grp if t["d"] and t["e"] and num(t["e"]["step_ms_med"])]
        if not pr:
            continue
        L.append(f"| {name} | {med(num(t['d']['step_ms_med']) for t in pr)} | "
                 f"{med(num(t['e']['step_ms_med']) for t in pr)} | "
                 f"{med(num(t['d']['step_ms_med']) / num(t['e']['step_ms_med']) for t in pr)} | "
                 f"{med(num(t['d']['step_ms_p95']) / num(t['e']['step_ms_p95']) for t in pr if num(t['e']['step_ms_p95']))} |")
    if done and len(done) == 16:
        cl = [t["deg"] for t in done if t["clock"]]
        nc = [t["deg"] for t in done if not t["clock"]]
        p = fisher(sum(cl), len(cl), sum(nc), len(nc))
        rcl, rnc = sum(cl) / len(cl), sum(nc) / len(nc)
        if rcl > rnc and p < 0.05:
            r = "supported: slowing hurts the clock-limited controllers more (p < 0.05)."
        elif rcl > rnc:
            r = ("direction as predicted but not significant: no claim beyond the "
                 "counts (n = 9 vs 7 has little power, prereg §7).")
        else:
            r = "not supported: clock-limited controllers were not degraded more often."
        L += ["", f"**Pre-registered reading (prereg §6):** {r}"]
    return "\n".join(L) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--exp3", default=str(EXP3),
                    help="Exp 3's physics.csv (traces resolved beside it)")
    ap.add_argument("--pilot", default=str(PILOT))
    a = ap.parse_args(argv)
    rows3, pilot = read_rows(a.exp3), read_rows(a.pilot)
    for exp, fn in (("A", None), ("D", None)):
        out = EXPERIMENTS[exp]["out"]
        rows = read_rows(out)
        if not rows:
            print(f"[report] no Exp {exp} rows at {out}")
            continue
        if exp == "A":
            md, _ = report_a(rows, rows3)
        else:
            md = report_d(rows, rows3, pilot, a.exp3, out)
        md += (f"\nReference: Exp 3 = `{a.exp3}` ({len(rows3)} rows), pilot = "
               f"`{a.pilot}`. UNDER PHYSICS, one hand-authored world, local "
               f"Isaac Sim 6.0.1 (in_container false).\n")
        (out.parent / "SUMMARY.md").write_text(md)
        print(md)


if __name__ == "__main__":
    main()
