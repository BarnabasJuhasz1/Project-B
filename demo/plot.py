"""
demo/plot.py

A patrol episode, DRAWN. One panel per leg, so "which leg did it lose" is a
thing you can see rather than infer from a number. The shut door is drawn in
red on the legs where it is shut.

SIM-FREE, scores nothing, and is downstream of everything -- it draws ground
truth, which is legitimate only because nothing imports it.

    $ python demo/plot.py <controller.py> --out demo/out/base
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from demo.world import OPEN_WORLD, BLOCKED_WORLD, DOOR, HOME, GOAL
from demo.rollout import LEG_BLOCKED, LEG_TARGETS, ARRIVE_RADIUS


def draw(result, out_path, title=""):
    trace = result["trace"]
    if not trace:
        raise ValueError("rollout(..., record=True) is needed to draw")
    n = len(LEG_TARGETS)
    # Seconds actually spent on each leg, from the trace. ⚠️ AN EMPTY PANEL IS
    # AMBIGUOUS AND THAT CAUSED REAL CONFUSION: a leg with no trace rows looks
    # like a run that was never recorded, when it means the PREVIOUS leg never
    # finished so this one never began. The budget accounting below makes the
    # difference explicit on the panel itself.
    spent = {}
    for row in trace:
        spent[row[4]] = spent.get(row[4], 0) + 1
    dt = 0.02
    started = set(spent)

    fig, axes = plt.subplots(1, n, figsize=(5.2 * n, 4.2))
    for leg, ax in enumerate(axes):
        world = BLOCKED_WORLD if LEG_BLOCKED[leg] else OPEN_WORLD
        began = leg in started
        for (x0, y0, x1, y1) in world.rects:
            ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0,
                                   fc="0.45" if began else "0.75",
                                   ec="0.3" if began else "0.6"))
        if LEG_BLOCKED[leg]:
            ax.add_patch(Rectangle((DOOR[0], DOOR[1]), DOOR[2] - DOOR[0],
                                   DOOR[3] - DOOR[1],
                                   fc="crimson" if began else "#e8a0a0",
                                   ec="crimson" if began else "#e8a0a0",
                                   zorder=6))
        pts = [(r[1], r[2]) for r in trace if r[4] == leg]
        if pts:
            ax.plot([p[0] for p in pts], [p[1] for p in pts], "-",
                    color="tab:blue", lw=1.4, zorder=4)
            ax.plot(pts[0][0], pts[0][1], "o", color="tab:blue", ms=7, zorder=5)
            ax.plot(pts[-1][0], pts[-1][1], "X", color="black", ms=11, zorder=7)
        tgt = LEG_TARGETS[leg]
        ax.add_patch(plt.Circle(tgt, ARRIVE_RADIUS,
                                fc="tab:green" if began else "#bfe0bf",
                                alpha=0.45, zorder=5))

        where = "->goal" if tgt == GOAL else "->home"
        head = f"leg {leg}: {where}{' | door SHUT' if LEG_BLOCKED[leg] else ''}"
        done = leg < result["legs_completed"]
        if done:
            sub, colour = f"reached in {result['leg_times'][leg]}s", "darkgreen"
        elif began:
            sub = (f"NOT reached — {spent[leg] * dt:.0f}s spent here,\n"
                   f"budget exhausted")
            colour = "darkred"
        else:
            blocker = max(started) if started else 0
            sub = (f"never started\n(leg {blocker} never finished)")
            colour = "0.45"
        ax.set_title(f"{head}\n{sub}", color=colour)
        ax.set_xlim(0, 14); ax.set_ylim(0, 10); ax.set_aspect("equal")
        ax.set_xticks([]); ax.set_yticks([])
        if not began:
            ax.set_facecolor("0.96")

    fig.suptitle((title + "  —  " if title else "")
                 + f"legs completed {result['legs_completed']}/{n}"
                 + f"   (budget {int(sum(spent.values()) * dt)}s used)",
                 y=0.99, fontsize=13)
    # suptitle first, then leave room for it: tight_layout() does not know
    # about a suptitle and the two titles collided.
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    out_path = pathlib.Path(out_path).with_suffix(".png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=100); plt.close(fig)
    return out_path


if __name__ == "__main__":
    import argparse
    from demo.rollout import load_controller, rollout
    ap = argparse.ArgumentParser()
    ap.add_argument("controller"); ap.add_argument("--out", default="demo/out/episode")
    ap.add_argument("--seed", type=int, default=0)
    a = ap.parse_args()
    res = rollout(load_controller(a.controller)(), seed=a.seed, record=True)
    print({k: v for k, v in res.items() if k != "trace"})
    print("->", draw(res, a.out, title=pathlib.Path(a.controller).name))
