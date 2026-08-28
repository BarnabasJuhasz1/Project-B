"""
nodes/trace.py

Which node ticked, when, with what status, and which one actually drove.

A scored trace says what the robot did. It says nothing about WHY, and for a
behaviour tree the why is the point: a tree that fails because its precondition
never fired is a completely different result from one that fails because its
action was wrong, and `core/metrics.py` scores them identically.

DELIBERATELY SEPARATE FROM EpisodeTrace
---------------------------------------
This does not touch `TRACE_COLUMNS`. Three reasons, all of them load-bearing:

- The number of nodes ticked varies per step, so it is not a fixed-width row.
- `trace.csv` is what determinism is checked on and what `rescore.py` re-scores
  from; adding a variable-width, purely diagnostic column to it would put the
  reproducibility claim at the mercy of a debugging feature.
- Scoring must stay a pure function of the trace. A tick log is an explanation,
  not evidence, and nothing in `score()` may ever read it.

It is OPT-IN (`Tree.enable_trace()`), so the default path allocates nothing and
a benchmark run is byte-identical whether or not anyone is watching.
"""

import csv
from collections import Counter

COLUMNS = ["step", "t", "depth", "node_id", "kind", "precondition", "status",
           "drove"]


class TickLog:
    """One episode's worth of node activity."""

    def __init__(self):
        self.rows = []
        self._step = -1
        self._t = 0.0
        self._depth = 0

    # -- recording ----------------------------------------------------------

    def begin_step(self, t):
        self._step += 1
        self._t = float(t)
        self._depth = 0

    def enter(self):
        self._depth += 1

    def leave(self):
        self._depth -= 1

    def record(self, node, precondition, status, drove):
        self.rows.append((
            self._step, round(self._t, 6), self._depth,
            node.metadata.node_id, node.KIND.value,
            bool(precondition), status.value, bool(drove),
        ))

    # -- reading ------------------------------------------------------------

    def __len__(self):
        return len(self.rows)

    @property
    def steps(self):
        return self._step + 1

    def write_csv(self, path):
        with open(path, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(COLUMNS)
            for row in self.rows:
                writer.writerow([f"{v:.6f}" if isinstance(v, float)
                                 else int(v) if isinstance(v, bool) else v
                                 for v in row])

    def summary(self):
        """Per node: how often it ticked, how often its precondition passed,
        how often it drove, and the spread of statuses.

        This is the diagnostic that matters. A node whose precondition never
        passes is dead weight the tree is carrying; a node that ticks every step
        and never drives is doing nothing; and an action that drives 100% of the
        time means everything behind it in a Fallback is unreachable.
        """
        out = {}
        for step, t, depth, node_id, kind, pre, status, drove in self.rows:
            entry = out.setdefault(node_id, {
                "kind": kind, "ticks": 0, "precondition_passed": 0,
                "drove": 0, "status": Counter(), "first_t": t, "last_t": t,
            })
            entry["ticks"] += 1
            entry["precondition_passed"] += int(pre)
            entry["drove"] += int(drove)
            entry["status"][status] += 1
            entry["last_t"] = t
        return out

    def format_summary(self, width=48):
        """A short table, for the runner's log."""
        lines = [f"{'node':28s} {'kind':9s} {'ticks':>6s} {'pre':>6s} "
                 f"{'drove':>6s}  statuses"]
        for node_id, e in self.summary().items():
            statuses = " ".join(f"{k}={v}" for k, v in sorted(e["status"].items()))
            lines.append(f"{node_id[:28]:28s} {e['kind']:9s} {e['ticks']:6d} "
                         f"{e['precondition_passed']:6d} {e['drove']:6d}  {statuses}")
        return "\n".join(lines)

    def timeline(self, node_id, columns=100):
        """An ASCII strip for one node over the episode.

        `#` the node drove, `+` it ticked and its precondition passed, `.` it
        ticked and was gated out, ` ` it did not tick at all. Reading which node
        was in charge when is otherwise a spreadsheet exercise.
        """
        if self.steps <= 0:
            return ""
        buckets = [" "] * columns
        for step, t, depth, nid, kind, pre, status, drove in self.rows:
            if nid != node_id:
                continue
            i = min(columns - 1, int(step * columns / max(1, self.steps)))
            mark = "#" if drove else ("+" if pre else ".")
            if buckets[i] == " " or mark == "#":
                buckets[i] = mark
        return "".join(buckets)
