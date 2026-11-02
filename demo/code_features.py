"""demo/code_features.py — static features of every demo controller (Exp 4).

SIM-FREE and DETERMINISTIC: AST and regex over the source text, nothing is
imported or executed, and the same files always give a byte-identical CSV. It
is `Mindfire_Experiment_Plan.md` §4.2's feature list made mechanical:

    lines of code, classes, functions        size
    persistent grid                          self.<attr> = np.zeros/full/ones/empty((h, w))
    priority queue                           heapq
    D* vocabulary                            rhs, k_m, RAISE, LOWER, PROCESS-STATE, MODIFY-COST
    full-replan paths                        a value field re-initialised OUTSIDE reset()
    wall-clock budgets                       time.perf_counter / time.time / time.monotonic
    rollout-scored velocity selection        candidate twists AND a velocity x horizon product

⚠️ These are FEATURES, not labels. The rubric in `demo/CODING.md` is applied by
a rater reading the code; a feature here is what a grep can establish, and the
point of having both is that they can disagree. Every count is taken on the
BODY — the file with its header comment stripped — because the prompt asks
armB (and armX) to cite their source in that header, so it lists exactly the
vocabulary being counted. `header_lines` is reported so the cut is visible.

⚠️ Features are computed on the ORIGINAL files, keyed by (arm, k). They are
mechanical, so nothing is gained by blinding them; the rater's labels are what
`demo/blind_controllers.py` blinds.

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P demo/code_features.py                        # demo/arms -> demo/out/v2/code_features.csv
    $P demo/code_features.py --arms /path/to/arms --out /tmp/f.csv
    $P demo/code_features.py --files a.py b.py      # arbitrary files, arm/k left blank
    $P -m pytest demo/test_code_features.py -q
"""
from __future__ import annotations

import argparse
import ast
import csv
import hashlib
import io
import pathlib
import re
import sys

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_ARMS = HERE / "arms"
DEFAULT_OUT = HERE / "out" / "v2" / "code_features.csv"

# ---- below this line is data, not code ------------------------------------

# D* state vocabulary (Stentz 1994 and D* Lite). Word-bounded, case as written
# in the paper where case carries meaning (RAISE / LOWER are upper case there;
# a lower-case `lower` is an ordinary English word and must not count).
DSTAR_VOCAB = {
    "rhs":           r"\brhs\b",
    "k_m":           r"\bk_?m\b",
    "RAISE":         r"\bRAISE\b",
    "LOWER":         r"\bLOWER\b",
    "PROCESS_STATE": r"(?i:\bprocess[-_ ]?state\b)",
    "MODIFY_COST":   r"(?i:\bmodify[-_ ]?cost\b)",
    # `D*` only with no space and followed by whitespace / punctuation, so the
    # multiplication `d * math.cos(a)` (6 false hits in arm0 on the first
    # draft of this pattern) never counts.
    "dstar_name":    r"\b(?:D|LPA)\*(?=[\s.,;:()'\"/-]|$)|(?i:\bd[-_]?star\b|\bstentz\b)",
}

# Free-text replan vocabulary. The pilot README's "full-replan keyword count"
# was never committed as code; this is it, stated, plus the AST detector below.
REPLAN_VOCAB = r"(?i)full[-_ ]?replan|from[-_ ]scratch|replan[-_ ]?from|rebuild[-_ ]?(?:the[-_ ])?field"

# Path-following vocabulary (a weak, supporting signal only).
PATH_FOLLOW_VOCAB = r"(?i)pure[-_ ]?pursuit|lookahead|look[-_]ahead|carrot|waypoint|backpointer|steepest|descen[dt]"

# A shape variable's name, e.g. `shp`, `shape`, `grid_shape`.
SHAPE_NAME = re.compile(r"(?i)^shp$|shape")

# Names that carry a TIME in a rollout (`wz * ts`, `cwz * HORIZON`, `v * dt`).
TIME_NAMES = re.compile(r"^(dt|ts|T|tt|t_?h|times?|horizon|HORIZON|\w*_dt|dt_\w*|\w*HORIZON\w*|\w*horizon\w*)$")
# Names that carry a commanded VELOCITY.
VEL_NAMES = re.compile(r"(?i)^_?c?(v|vx|vy|w|wz|om|omega|vel|vels|spd|speed|lin|ang)s?$")

# numpy constructors that allocate a grid.
ALLOC_FUNCS = {"zeros", "ones", "full", "empty", "zeros_like", "ones_like",
               "full_like", "empty_like"}

# Methods whose assignments are initialisation, not replanning.
INIT_METHODS = {"__init__", "reset"}

TIME_CALLS = ("perf_counter", "time", "monotonic", "process_time")

COLUMNS = [
    "arm", "k", "file", "sha256_16",
    "total_lines", "header_lines", "loc",
    "n_classes", "n_functions",
    "persistent_grid_attrs", "logodds_vocab",
    "uses_heapq", "heap_ops",
    *[f"dstar_{k}" for k in DSTAR_VOCAB], "dstar_vocab_total",
    "full_reinit_sites", "replan_vocab",
    "time_budget_calls",
    "candidate_grids", "rollout_vt_products", "rollout_integrations",
    "path_follow_vocab",
    "rollout_scoring",
]

# ---- header -----------------------------------------------------------------


def split_header(src: str) -> tuple[str, str]:
    """(header, body). The header is every leading blank line, `#` comment line
    and the module docstring, up to the first real statement. header + body is
    the original text exactly, so nothing else is touched."""
    lines = src.splitlines(keepends=True)
    try:
        tree = ast.parse(src)
    except SyntaxError:
        tree = None
    doc = None
    if tree and tree.body and isinstance(tree.body[0], ast.Expr) \
            and isinstance(getattr(tree.body[0], "value", None), ast.Constant) \
            and isinstance(tree.body[0].value.value, str):
        doc = (tree.body[0].lineno - 1, tree.body[0].end_lineno)  # [start, end)
    i = 0
    while i < len(lines):
        s = lines[i].strip()
        if not s or s.startswith("#"):
            i += 1
        elif doc and i == doc[0]:
            i = doc[1]
        else:
            break
    return "".join(lines[:i]), "".join(lines[i:])


# ---- features ---------------------------------------------------------------


def _loc(body: str) -> int:
    """Lines carrying code: not blank, not a pure comment line. Docstrings
    inside the body count, deliberately — telling them apart needs the
    tokenizer and buys nothing for a size feature."""
    return sum(1 for ln in body.splitlines() if ln.strip() and not ln.strip().startswith("#"))


def _comments(body: str) -> str:
    import tokenize
    out = []
    try:
        for tok in tokenize.generate_tokens(io.StringIO(body).readline):
            if tok.type == tokenize.COMMENT:
                out.append(tok.string)
    except (tokenize.TokenError, IndentationError):
        pass
    return "\n".join(out)


def _attr_target(node) -> str | None:
    """`self.x` or `self.x[...]` -> 'x'."""
    while isinstance(node, ast.Subscript):
        node = node.value
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
            and node.value.id == "self":
        return node.attr
    return None


def _np_alloc(call) -> str | None:
    """'zeros' etc. if `call` is np.<alloc>(...) / numpy.<alloc>(...)."""
    if isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) \
            and call.func.attr in ALLOC_FUNCS \
            and isinstance(call.func.value, ast.Name) \
            and call.func.value.id in ("np", "numpy"):
        return call.func.attr
    return None


def _is_2d_shape(call) -> bool:
    """A grid-sized allocation: np.zeros((h, w)), np.zeros(shp), np.zeros(a.shape),
    np.zeros(n * n) (a flattened grid), or np.zeros_like(self.other_grid).
    A literal int or a bare name like `K` is NOT a grid — that is a candidate or
    beam vector. Heuristic, and the attribute names are printed so a reader can
    see what it caught."""
    name = _np_alloc(call)
    if not name or not call.args:
        return False
    a = call.args[0]
    if name.endswith("_like"):
        return _attr_target(a) is not None
    return _grid_shape(a)


def _grid_shape(a) -> bool:
    if isinstance(a, (ast.Tuple, ast.List)):
        return len(a.elts) >= 2
    if isinstance(a, ast.Attribute):
        return a.attr == "shape" or bool(SHAPE_NAME.search(a.attr))
    if isinstance(a, ast.Name):
        return bool(SHAPE_NAME.search(a.id))
    if isinstance(a, ast.BinOp) and isinstance(a.op, ast.Mult):
        return not any(isinstance(x, ast.Constant) for x in (a.left, a.right))
    return False


def _whole_slice(sl) -> bool:
    if isinstance(sl, ast.Slice):
        return sl.lower is None and sl.upper is None and sl.step is None
    return isinstance(sl, ast.Constant) and sl.value is Ellipsis


def _big_or_inf(node) -> bool:
    """A fill value meaning 'unreached': inf, a name containing INF/BIG/UNREACHED,
    or a numeric constant >= 1e6."""
    src = ast.unparse(node)
    if re.search(r"(?i)\binf\b|np\.inf|math\.inf|float\(['\"]inf|big|unreach|_inf", src):
        return True
    try:
        return abs(float(ast.literal_eval(node))) >= 1e6
    except Exception:
        return False


def _method_of(tree) -> dict:
    """node -> name of the innermost enclosing function."""
    owner = {}

    def walk(node, fn):
        for child in ast.iter_child_nodes(node):
            f = child.name if isinstance(child, (ast.FunctionDef, ast.AsyncFunctionDef)) else fn
            owner[child] = f
            walk(child, f)
    walk(tree, None)
    return owner


def _names_in(node) -> set[str]:
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)} | \
           {n.attr for n in ast.walk(node) if isinstance(n, ast.Attribute)}


def _calls_trig(node) -> bool:
    for n in ast.walk(node):
        if isinstance(n, ast.Call):
            f = n.func
            name = f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", "")
            if name in ("cos", "sin"):
                return True
    return False


def _inside_loop(node, parents) -> bool:
    p = parents.get(node)
    while p is not None:
        if isinstance(p, (ast.For, ast.While, ast.comprehension, ast.ListComp,
                          ast.GeneratorExp)):
            return True
        if isinstance(p, (ast.FunctionDef, ast.AsyncFunctionDef)):
            return False
        p = parents.get(p)
    return False


def features(src: str) -> dict:
    header, body = split_header(src)
    tree = ast.parse(src)
    owner = _method_of(tree)
    parents = {c: p for p in ast.walk(tree) for c in ast.iter_child_nodes(p)}
    comments = _comments(body)

    f = {
        "sha256_16": hashlib.sha256(src.encode()).hexdigest()[:16],
        "total_lines": len(src.splitlines()),
        "header_lines": len(header.splitlines()),
        "loc": _loc(body),
        "n_classes": sum(isinstance(n, ast.ClassDef) for n in ast.walk(tree)),
        "n_functions": sum(isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))
                           for n in ast.walk(tree)),
    }

    # -- persistent grid: a self attribute allocated as a 2-D array.
    grids = set()
    for n in ast.walk(tree):
        if isinstance(n, (ast.Assign, ast.AnnAssign)):
            targets = n.targets if isinstance(n, ast.Assign) else [n.target]
            if n.value is not None and _is_2d_shape(n.value):
                for t in targets:
                    name = _attr_target(t) if not isinstance(t, ast.Subscript) else None
                    if name:
                        grids.add(name)
    f["persistent_grid_attrs"] = ";".join(sorted(grids))
    f["logodds_vocab"] = len(re.findall(r"(?i)log[-_ ]?odds|\blogodds\b|\bl_occ\b|\bl_free\b", body))

    # -- priority queue
    imports = set()
    for n in ast.walk(tree):
        if isinstance(n, ast.Import):
            imports |= {a.name for a in n.names}
        elif isinstance(n, ast.ImportFrom) and n.module:
            imports.add(n.module)
    f["uses_heapq"] = int("heapq" in imports)
    f["heap_ops"] = len(re.findall(r"\bheap(?:push|pop|ify|replace|pushpop)\b", body))

    # -- D* vocabulary (identifiers and comments alike; the body only)
    total = 0
    for key, pat in DSTAR_VOCAB.items():
        c = len(re.findall(pat, body))
        f[f"dstar_{key}"] = c
        total += c
    f["dstar_vocab_total"] = total

    # -- full replan: a whole value field initialised to "unreached" OUTSIDE
    # reset()/__init__ — `D = np.full((H, W), INF)`, `f[:] = INF`, `f.fill(INF)`
    # — in a method that runs during the episode. A from-scratch recompute has
    # to do one of these; an incremental repair never does. ⚠️ A site counts
    # whether or not its method is reachable, and a goal-switch re-init counts
    # too: the rubric's planner_type separates those, this does not.
    reinit = 0
    for n in ast.walk(tree):
        fn = owner.get(n)
        if fn is None or fn in INIT_METHODS:
            continue
        if isinstance(n, ast.Call) and _np_alloc(n) in ("full", "full_like") \
                and len(n.args) >= 2 and _big_or_inf(n.args[1]) \
                and (_np_alloc(n) == "full_like" or _grid_shape(n.args[0])):
            reinit += 1
        elif isinstance(n, ast.Assign) and len(n.targets) == 1:
            t = n.targets[0]
            if isinstance(t, ast.Subscript) and _whole_slice(t.slice) \
                    and _big_or_inf(n.value):
                reinit += 1
        elif isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                and n.func.attr == "fill" and n.args and _big_or_inf(n.args[0]):
            reinit += 1
    f["full_reinit_sites"] = reinit
    f["replan_vocab"] = len(re.findall(REPLAN_VOCAB, body))

    # -- wall-clock budgets
    f["time_budget_calls"] = len(re.findall(
        r"\btime\.(?:%s)\(\)" % "|".join(TIME_CALLS), body)) + len(re.findall(
        r"(?<![\w.])(?:perf_counter|monotonic)\(\)", body))

    # -- rollout scoring: a set of candidate twists (np.meshgrid /
    # itertools.product, or nested `for v in ...: for w in ...`) AND forward
    # simulation — a velocity multiplied by a horizon or timestep (`wz * ts`,
    # `cvx * HORIZON`), or a pose integrated with cos/sin and a timestep inside
    # a loop. Steering at a waypoint does neither.
    vt = integ = 0
    for n in ast.walk(tree):
        if isinstance(n, ast.BinOp) and isinstance(n.op, ast.Mult):
            l, r = _names_in(n.left), _names_in(n.right)
            if (any(TIME_NAMES.match(x) for x in l) and any(VEL_NAMES.match(x) for x in r)) or \
               (any(TIME_NAMES.match(x) for x in r) and any(VEL_NAMES.match(x) for x in l)):
                vt += 1
        if isinstance(n, (ast.AugAssign, ast.Assign)) and _calls_trig(n.value) \
                and any(TIME_NAMES.match(x) for x in _names_in(n.value)) \
                and _inside_loop(n, parents):
            integ += 1
    grid = 0
    for n in ast.walk(tree):
        if isinstance(n, ast.Call):
            name = n.func.attr if isinstance(n.func, ast.Attribute) else getattr(n.func, "id", "")
            if name in ("meshgrid", "product") and len(n.args) >= 2:
                grid += 1
        if isinstance(n, ast.For) and isinstance(n.target, ast.Name) \
                and VEL_NAMES.match(n.target.id) \
                and any(isinstance(c, ast.For) and c is not n
                        and isinstance(c.target, ast.Name) and VEL_NAMES.match(c.target.id)
                        for c in ast.walk(n)):
            grid += 1
    f["candidate_grids"] = grid
    f["rollout_vt_products"] = vt
    f["rollout_integrations"] = integ
    f["path_follow_vocab"] = len(re.findall(PATH_FOLLOW_VOCAB, body))
    f["rollout_scoring"] = int(grid > 0 and (vt + integ) > 0)
    return f


# ---- driver -----------------------------------------------------------------


def discover(arms_dir: pathlib.Path) -> list[tuple[str, str, pathlib.Path]]:
    """(arm, k, path) for every <arm>/k<n>.py, sorted by arm then numeric k."""
    out = []
    for p in arms_dir.glob("*/k*.py"):
        m = re.fullmatch(r"k(\d+)", p.stem)
        if m:
            out.append((p.parent.name, int(m.group(1)), p))
    return [(a, str(k), p) for a, k, p in sorted(out, key=lambda r: (r[0], r[1]))]


def rows_for(items) -> list[dict]:
    rows = []
    for arm, k, path in items:
        r = {"arm": arm, "k": k, "file": path.name if arm else str(path)}
        r.update(features(path.read_text()))
        rows.append(r)
    return rows


def write_csv(rows: list[dict], out: pathlib.Path) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLUMNS, lineterminator="\n")
        w.writeheader()
        for r in rows:
            w.writerow({c: r.get(c, "") for c in COLUMNS})


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--arms", type=pathlib.Path, default=DEFAULT_ARMS,
                    help="directory holding <arm>/k<n>.py")
    ap.add_argument("--files", type=pathlib.Path, nargs="+",
                    help="score these files instead (arm and k left blank)")
    ap.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    a = ap.parse_args(argv)
    items = [("", "", p) for p in sorted(a.files)] if a.files else discover(a.arms)
    if not items:
        print(f"[code_features] no controllers under {a.arms}", file=sys.stderr)
        return 1
    rows = rows_for(items)
    write_csv(rows, a.out)
    print(f"[code_features] {len(rows)} controllers -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
