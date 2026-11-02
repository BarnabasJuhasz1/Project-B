"""
demo/controls/recut_from_calibration.py

THE ONE RE-CUT, from the 2026-11-02 calibration calls (MANIFEST.md, "The
calibration"). Calls no model and no API; reads `manifest.json["calibration"]`.

The two calls were not a clean pair. Under CLI 2.1.302 the armX session loaded
the account's claude.ai MCP connectors and the armB session did not, so armX's
first turn carried 7,504 characters of extra CLI overhead (MCP tool names, MCP
server instructions, three tool definitions). The measured difference is

    armX - armB (turn 1)  =  Claude(armX prompt - armB prompt)  +  E

and E, the extra overhead, is not counted by anything. It is ESTIMATED as
r * proxy(extra text), with r = Claude/proxy fitted on the CLI's OWN text in the
armB call (reminders + environment block + task, 1.444 legacy / 1.512 Qwen) --
the same ratio the paper block shows (1.473 / 1.480), so the Claude tokenizer
is ~1.47x both proxies across text types here.

Then Claude tokens per Feng line are anchored on the measured armX block (a
SECOND anchor, on Feng's own text, which is what the offline estimate lacked),
and the cut is moved to the paragraph boundary nearest zero deviation.

    PYTHONNOUSERSITE=1 ~/miniforge3/bin/python demo/controls/recut_from_calibration.py \\
        --tokenizer claude-legacy=<Xenova/claude-tokenizer tokenizer.json> \\
        --tokenizer qwen2.5=<Qwen2.5-7B-Instruct tokenizer.json>

SIM-FREE. Calls no model.
"""
import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from demo import render                                           # noqa: E402
from demo.controls import build_feng2015 as feng                   # noqa: E402
from demo.controls.estimate_tokens import load_tokenizers          # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tokenizer", action="append", required=True,
                    metavar="NAME=tokenizer.json")
    ap.add_argument("--pdf")
    ap.add_argument("--lo", type=int, default=900)
    ap.add_argument("--hi", type=int, default=1010)
    a = ap.parse_args()

    cal = feng.load_manifest()["calibration"]
    meas = cal["measured_turn1_input_tokens"]
    raw = meas["armX_cut813"] - meas["armB"]
    d_b = cal["claude_block_tokens_armB"]
    ex = cal["extra_overhead_in_armX_call"]
    counters = {n: c for n, c in load_tokenizers(a.tokenizer).items() if n != "chars"}

    full = feng.extract(feng.find_pdf(a.pdf))
    lines = full.split("\n")
    paras = set(feng.paragraph_ends(lines))
    base = {n: c(render.render("arm0")) for n, c in counters.items()}

    def block(n, cut):
        return counters[n](render.render("armX", paper=feng.truncate(full, cut))) - base[n]

    at_old = {n: block(n, cal["cut_lines_measured"]) for n in counters}
    delta, per_line = {}, {}
    for n in counters:
        e = ex["r"][n] * ex["proxy_tokens"][n]
        delta[n] = raw - e
        per_line[n] = (d_b + delta[n]) / at_old[n]     # Claude per proxy token, on Feng
        print(f"[recut] {n}: E = {ex['r'][n]} x {ex['proxy_tokens'][n]} = {e:.0f}; "
              f"armX - armB at cut {cal['cut_lines_measured']} = {delta[n]:+.0f} "
              f"({delta[n] / meas['armB']:+.2%})")

    best = None
    print("\n  cut  kind   chars  " + "  ".join(f"{n:>13s}" for n in counters) + "      mean")
    for i, line in enumerate(lines):
        cut = i + 1
        if not (a.lo <= cut <= a.hi and line.rstrip().endswith(".")):
            continue
        dev = {n: (delta[n] + per_line[n] * (block(n, cut) - at_old[n])) / meas["armB"]
               for n in counters}
        mean = sum(dev.values()) / len(dev)
        kind = "para" if i in paras else "sent"
        print(f"{cut:5d}  {kind}  {len(feng.truncate(full, cut)):6d}  "
              + "  ".join(f"{dev[n]:+13.2%}" for n in counters) + f"  {mean:+8.2%}")
        if kind == "para" and (best is None or abs(mean) < abs(best[1])):
            best = (cut, mean)
    print(f"\n[recut] nearest paragraph boundary: cut_lines={best[0]}, "
          f"estimated {best[1]:+.2%} vs armB (tolerance +-0.5 %)")


if __name__ == "__main__":
    main()
