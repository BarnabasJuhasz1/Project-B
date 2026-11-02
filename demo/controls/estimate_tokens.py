"""
demo/controls/estimate_tokens.py

WHERE TO CUT FENG SO armX MATCHES armB IN TOKENS -- an OFFLINE ESTIMATE.

⚠️ Claude's tokenizer is not public and no token-counting key was available
(2026-11-02), so this does NOT count Claude tokens. It anchors on the one exact
Claude measurement there is and transfers it through proxy tokenizers:

  1. ANCHOR. The pilot's single-turn CLI calls report exact input tokens:
     arm0 12,549 and armB 25,315 (median over the single-turn transcripts; the
     CLI overhead is identical across arms and varies by +-1). So the paper
     block, as rendered, is dB = 12,766 Claude tokens.
  2. TRANSFER. For each proxy tokenizer p, the block's proxy count is
     p(render(arm)) - p(render("arm0")). Assuming Claude/p is the same ratio on
     Feng's text as on D*'s, Claude(armX block) = dB * pX / pB.
  3. CUT. Among Feng's paragraph boundaries, take the one minimising the
     WORST relative error across the proxies (minimax), so no single proxy's
     quirks pick the cut.

The UNCERTAINTY is the proxies' disagreement at the chosen cut, and it is a
LOWER bound: two proxies agreeing does not make Claude agree with them. Feng is
dense with numerals, units and symbol glyphs that tokenise differently from
D*'s prose, which is exactly where tokenizers diverge. The plan's fallback --
one CALIBRATION CALL with the armX prompt, transcript to
demo/transcripts/calibration/, excluded from every denominator -- is the only
way to close it, and it needs Barna's approval (~$1.5). See MANIFEST.md.

    # needs the `tokenizers` package; env_isaacsim does not have it
    PYTHONNOUSERSITE=1 ~/miniforge3/bin/python demo/controls/estimate_tokens.py \\
        --tokenizer claude-legacy=<Xenova/claude-tokenizer tokenizer.json> \\
        --tokenizer qwen2.5=<tokenizer.json> \\
        --tokenizer gpt2=<whisper-tiny tokenizer.json> \\
        --vote claude-legacy,qwen2.5

SIM-FREE. Calls no model and no API.
"""
import argparse
import pathlib
import re
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))
from demo import render                                           # noqa: E402
from demo.controls import build_feng2015 as feng                   # noqa: E402

# ---- below this line is data, not code

# Exact Claude input tokens (input + cache_creation + cache_read), single-turn
# pilot calls, from demo/transcripts/*.json `raw.usage`, recorded 2026-11-02.
CLAUDE_ARM0 = 12549        # k4 12550, k6 12549, k7 12548
CLAUDE_ARMB = 25315        # k0-k5 25316, k6-k7 25315
TOLERANCE = 0.005          # plan §2.2: +-0.5 % of armB's token count

# ---- below this line is code


def load_tokenizers(specs):
    from tokenizers import Tokenizer
    out = {}
    for spec in specs:
        name, path = spec.split("=", 1)
        tok = Tokenizer.from_file(path)
        out[name] = (lambda t: lambda s: len(t.encode(s, add_special_tokens=False).ids))(tok)
    out["chars"] = len                      # the naive estimate, for contrast
    return out


_WS = re.compile(r"[ \t]{2,}|\n{2,}")


def excess_ws(text):
    """Characters beyond the first in every run of spaces or newlines."""
    return sum(len(r) - 1 for r in _WS.findall(text))


def ws_tokens(count, text):
    """Tokens a proxy spends on whitespace runs: its count on the raw text
    minus its count with every run collapsed to one space."""
    return count(text) - count(_WS.sub(" ", text))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tokenizer", action="append", default=[],
                    metavar="NAME=tokenizer.json")
    ap.add_argument("--vote", help="comma-separated proxies that choose the cut "
                    "(default: every --tokenizer)")
    ap.add_argument("--pdf")
    ap.add_argument("--show", type=int, default=6, help="neighbouring cuts shown")
    a = ap.parse_args()

    counters = load_tokenizers(a.tokenizer)
    voting = (a.vote.split(",") if a.vote
              else [n for n in counters if n != "chars"] or ["chars"])
    full = feng.extract(feng.find_pdf(a.pdf))
    lines = full.split("\n")
    d_b = CLAUDE_ARMB - CLAUDE_ARM0

    base = {n: c(render.render("arm0")) for n, c in counters.items()}
    blk_b = {n: c(render.render("armB")) - base[n] for n, c in counters.items()}
    print(f"[est] Claude block dB = {d_b} tokens (armB {CLAUDE_ARMB} - arm0 {CLAUDE_ARM0})")
    for n in counters:
        print(f"[est]   {n:8s} armB block {blk_b[n]:6d}  -> Claude/{n} = "
              f"{d_b / blk_b[n]:.4f}")

    rows = []
    for i in feng.paragraph_ends(lines):
        text = feng.truncate(full, i + 1)
        prompt = render.render("armX", paper=text)
        pred = {n: CLAUDE_ARM0 + d_b * (c(prompt) - base[n]) / blk_b[n]
                for n, c in counters.items()}
        err = {n: pred[n] / CLAUDE_ARMB - 1 for n in pred}
        rows.append((max(abs(err[n]) for n in voting), i + 1, len(text), pred, err))

    best = min(rows)
    k = [r[1] for r in rows].index(best[1])
    print(f"\n[est] {len(rows)} paragraph boundaries; cut chosen by minimax over "
          f"{voting}\n")
    print("  cut_lines   chars  " + "  ".join(f"{n:>17s}" for n in counters))
    for r in rows[max(0, k - a.show):k + a.show + 1]:
        mark = "->" if r is best else "  "
        print(f"{mark}{r[1]:8d} {r[2]:7d}  " + "  ".join(
            f"{r[3][n]:8.0f} ({r[4][n]:+6.2%})" for n in counters))
    _, cut, chars, pred, err = best
    ests = [pred[n] for n in voting]
    print(f"\n[est] CUT cut_lines={cut} ({chars} chars). Predicted armX: "
          f"{statistics.mean(ests):.0f} (proxies {min(ests):.0f}-{max(ests):.0f}); "
          f"armB {CLAUDE_ARMB}; tolerance +-{TOLERANCE * CLAUDE_ARMB:.0f}")
    within = all(abs(err[n]) <= TOLERANCE for n in voting)
    print(f"[est] every voting proxy within +-{TOLERANCE:.1%}: {within}. "
          f"chars alone would say {err['chars']:+.2%} -- why chars are not used.")

    # The uncertainty the proxies cannot see: what Claude charges for a run
    # of spaces or a blank line. D*'s layout text is column gaps (long space
    # runs); Feng's is double spacing (many blank lines). Proxies that price
    # the two differently disagree by tens of percent, so the DIRECTION of the
    # error is not identified offline -- only a Claude count can settle it.
    blk_b = render.BLOCK.search(render.render("armB")).group(0)
    blk_x = render.BLOCK.search(
        render.render("armX", paper=feng.truncate(full, cut))).group(0)
    print(f"\n[est] whitespace at cut_lines={cut}: excess whitespace chars "
          f"D* {excess_ws(blk_b) / len(blk_b):.1%}, Feng {excess_ws(blk_x) / len(blk_x):.1%}")
    for n, c in counters.items():
        if n == "chars":
            continue
        wb, wx = ws_tokens(c, blk_b), ws_tokens(c, blk_x)
        print(f"[est]   {n:13s} tokens on whitespace: D* {wb:5d} ({wb / c(blk_b):.1%})"
              f"  Feng {wx:5d} ({wx / c(blk_x):.1%})")

if __name__ == "__main__":
    main()
