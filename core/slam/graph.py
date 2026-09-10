"""
core/slam/graph.py

**The back end: a 2D pose graph, and the least squares that makes it consistent.**

Chunk 23. Chunk 22 *proposes* constraints; this is what applies them. Nodes are
submap origins, edges are relative-pose measurements — odometry between
consecutive submaps, loop closures between distant ones — and `solve()` moves the
nodes until the whole trajectory agrees with everything observed.

    node k        the origin pose of submap k, in the global frame
    odometry edge (k, k+1)   where the tracker thought it went
    closure edge  (s, c)     where a re-match says it actually is

WHY GAUSS-NEWTON WITH A FIXED ITERATION COUNT
---------------------------------------------
Not Levenberg-Marquardt with a convergence tolerance, and not
iterate-until-the-update-is-small. **A fixed count is the only formulation whose
cost AND whose output are both identical run to run**, and this epic spends
accuracy on determinism wherever the two trade (see `core/slam/__init__.py`). A
tolerance makes the iteration count a function of floating-point noise, and the
iteration count is a function the optimised poses depend on.

*Rejected:* g2o / GTSAM bindings. Faster, far better tested, and they drag a
compiled dependency into a `core/` that is deliberately importable with no
simulator and no build step — and their internal iteration behaviour is one more
thing to pin.

THE THREE ONE-LINE MISTAKES
---------------------------
Each of these is a one-line error with a three-hour debugging session behind it,
so each is stated here and each has a test:

1. **θ residuals are wrapped INSIDE the residual.** A loop with a net rotation
   of 2π has zero rotational error; unwrapped, it has an error of 2π and the
   optimiser spins the trajectory once round to "fix" it.
2. **The information matrix makes metres and radians commensurable.** They are
   different units; an unweighted least squares over both is a least squares
   over a meaningless sum. Every edge carries an explicit information diagonal
   and `ODOM_INFO` / `CLOSURE_INFO` below are where the two are traded off.
3. **The anchor node is FIXED.** Relative-pose measurements constrain nothing
   about the global frame, so without an anchor the normal equations are exactly
   singular in three dimensions and the solution is free to translate and rotate.
   Fixed by SOLVING A REDUCED SYSTEM — the anchor's columns are removed before
   the solve rather than damped with a large diagonal, because a large diagonal
   is a number someone has to choose and a removed column is not.

HUBER, ON CLOSURE EDGES ONLY
----------------------------
Chunk 22's three gates will not catch every bad closure, and a false closure does
not degrade the map, it destroys it — the optimiser folds two different places
onto each other and every pose after them is structurally wrong. The kernel
bounds the damage one bad edge can do instead of letting it dominate.

**Odometry edges are deliberately not robustified.** A bad odometry edge means
the front end failed, and that must surface rather than be quietly absorbed.

SIM-FREE.
"""

import numpy as np
from scipy.sparse import coo_matrix, csc_matrix, diags
from scipy.sparse.linalg import spsolve

from core.slam import scan as scan_mod

# ---- below this line is data, not code.

# Information (inverse covariance) diagonals, as (x, y, theta). These ARE the
# metres-vs-radians trade named above, so they are written as the sigmas they
# came from rather than as bare numbers:
#
#   odometry between consecutive submaps: ~0.10 m, ~0.05 rad  -> 100, 400
#   a loop closure across the room:       ~0.20 m, ~0.10 rad  ->  25, 100
#
# Closures are weighted BELOW odometry on purpose. Odometry over one submap is a
# couple of seconds of dead reckoning corrected by scan matching and is good; a
# closure is a match across accumulated drift and is the thing more likely to be
# wrong. The asymmetry is the same one the gates in `loop.py` express.
ODOM_INFO = (100.0, 100.0, 400.0)
CLOSURE_INFO = (25.0, 25.0, 100.0)

# Huber threshold, in units of the WHITENED residual norm (dimensionless, since
# the information matrix has already made the components commensurable). Beyond
# this an edge's influence grows linearly rather than quadratically.
HUBER_DELTA = 1.0

# Fixed, and fixed in advance. Ten is comfortably past where a graph this size
# stops moving; the point is that it is the same ten every run.
ITERATIONS = 10

# Below this the update is numerically indistinguishable from zero and further
# iterations only accumulate rounding. NOT a convergence tolerance -- the loop
# still runs its full ITERATIONS; this only skips APPLYING a null update, which
# is what keeps two runs bitwise identical rather than differing in the last bit
# after ten no-op additions.
NULL_UPDATE = 1e-12

ODOMETRY = "odometry"
CLOSURE = "closure"

# ---- below this line is code, not data.


class Edge:
    """One relative-pose measurement: "node j sits at `measurement` from node i".

    `info` is the diagonal of the information matrix, `robust` says whether the
    Huber kernel applies, and `kind` is carried for diagnostics only — nothing
    branches on it, because branching on a label rather than on `robust` is how
    a third edge kind silently gets no kernel.
    """

    __slots__ = ("i", "j", "measurement", "info", "robust", "kind")

    def __init__(self, i, j, measurement, info=ODOM_INFO, robust=False,
                 kind=ODOMETRY):
        self.i = int(i)
        self.j = int(j)
        self.measurement = tuple(float(v) for v in measurement)
        self.info = tuple(float(v) for v in info)
        self.robust = bool(robust)
        self.kind = kind

    def __repr__(self):
        return (f"Edge({self.i}->{self.j}, {self.kind}, "
                f"z=({self.measurement[0]:.3f}, {self.measurement[1]:.3f}, "
                f"{self.measurement[2]:.4f}), robust={self.robust})")


class PoseGraph:
    """Nodes, edges, and the Gauss-Newton that reconciles them.

    Node 0 is the anchor and never moves. That is not a convention a caller may
    override: every measurement here is relative, so the anchor is what ties the
    graph to the global frame, and `SlamLocaliser` anchors it at the scenario's
    start pose because that is the one pose a robot is entitled to know.
    """

    ANCHOR = 0

    def __init__(self, iterations=ITERATIONS, huber_delta=HUBER_DELTA):
        self.poses = []
        self.edges = []
        self.iterations = int(iterations)
        self.huber_delta = float(huber_delta)
        # Set by solve(); read by diagnostics(). Never wall-clock -- see
        # SlamLocaliser.diagnostics for why timing is kept out of slam.csv.
        self.last_solve = {}

    def __len__(self):
        return len(self.poses)

    # -- construction ------------------------------------------------------

    def add_node(self, pose):
        self.poses.append(tuple(float(v) for v in pose))
        return len(self.poses) - 1

    def add_edge(self, i, j, measurement, info=ODOM_INFO, robust=False,
                 kind=ODOMETRY):
        n = len(self.poses)
        if not (0 <= i < n and 0 <= j < n):
            raise IndexError(
                f"edge ({i}, {j}) references a node outside 0..{n - 1}; "
                f"nodes must exist before the edges that constrain them")
        if i == j:
            raise ValueError(
                f"edge ({i}, {j}) is a self-loop: it constrains a node against "
                f"itself, contributes a structurally zero residual and a "
                f"singular block")
        self.edges.append(Edge(i, j, measurement, info, robust, kind))
        return self.edges[-1]

    # -- the least squares -------------------------------------------------

    def residuals(self):
        """(E, 3) whitening-free errors, one row per edge.

        e = z^-1 (x_i^-1 x_j): the measurement's disagreement with the current
        poses, expressed in the MEASUREMENT's frame. θ is wrapped here — mistake
        1 above — by `scan.relative`, which wraps by construction.
        """
        out = np.zeros((len(self.edges), 3))
        for k, edge in enumerate(self.edges):
            predicted = scan_mod.relative(self.poses[edge.i], self.poses[edge.j])
            out[k] = scan_mod.relative(edge.measurement, predicted)
        return out

    def jacobian(self):
        """(3E, 3N) sparse ∂residual/∂pose.

        Analytic, not finite-differenced: a finite difference across the θ wrap
        is wrong by 2π exactly where the wrap bites, which is the case mistake 1
        is about. `core/tests/test_slam_graph.py` checks this against a central
        difference away from the wrap, which is the right place for that check.
        """
        rows, cols, vals = [], [], []
        for k, edge in enumerate(self.edges):
            A, B = self._blocks(edge)
            r0 = 3 * k
            for a in range(3):
                for b in range(3):
                    if A[a, b] != 0.0:
                        rows.append(r0 + a)
                        cols.append(3 * edge.i + b)
                        vals.append(A[a, b])
                    if B[a, b] != 0.0:
                        rows.append(r0 + a)
                        cols.append(3 * edge.j + b)
                        vals.append(B[a, b])
        return coo_matrix((vals, (rows, cols)),
                          shape=(3 * len(self.edges), 3 * len(self.poses))).tocsr()

    def _blocks(self, edge):
        """∂e/∂x_i and ∂e/∂x_j for one edge. The standard SE(2) pair.

        Derived for the parameterisation this class actually uses: the update is
        a plain addition on (x, y, θ) with a wrap, NOT a right-multiplied
        exponential, so these are derivatives with respect to the global
        coordinates directly. Mixing the two conventions gives an optimiser that
        converges slowly and looks merely badly tuned.
        """
        xi, yi, ti = self.poses[edge.i]
        xj, yj, _ = self.poses[edge.j]
        tz = edge.measurement[2]

        ci, si = np.cos(ti), np.sin(ti)
        cz, sz = np.cos(tz), np.sin(tz)
        # R_z^T and R_i^T, and d(R_i^T)/dθ_i.
        Rz_T = np.array([[cz, sz], [-sz, cz]])
        Ri_T = np.array([[ci, si], [-si, ci]])
        dRi_T = np.array([[-si, ci], [-ci, -si]])
        d = np.array([xj - xi, yj - yi])

        A = np.zeros((3, 3))
        A[:2, :2] = -Rz_T @ Ri_T
        A[:2, 2] = Rz_T @ (dRi_T @ d)
        A[2, 2] = -1.0

        B = np.zeros((3, 3))
        B[:2, :2] = Rz_T @ Ri_T
        B[2, 2] = 1.0
        return A, B

    def weights(self, errors=None):
        """(3E,) diagonal of the weighting matrix: information, times Huber.

        The kernel is applied as an IRLS reweighting — the standard, and the
        cheap one: an edge whose whitened residual norm exceeds `huber_delta`
        has its information scaled by `delta / norm`, so its contribution grows
        linearly rather than quadratically. Robust edges only, which in this
        project means closures only.
        """
        if errors is None:
            errors = self.residuals()
        w = np.zeros(3 * len(self.edges))
        for k, edge in enumerate(self.edges):
            info = np.array(edge.info)
            if edge.robust and self.huber_delta > 0.0:
                norm = float(np.sqrt(np.sum(info * errors[k] ** 2)))
                if norm > self.huber_delta:
                    info = info * (self.huber_delta / norm)
            w[3 * k:3 * k + 3] = info
        return w

    def chi2(self, errors=None):
        """Σ eᵀ Ω e over all edges, un-robustified. The number `solve()` reports
        before and after, and the only honest way to say whether it helped."""
        if errors is None:
            errors = self.residuals()
        total = 0.0
        for k, edge in enumerate(self.edges):
            total += float(np.sum(np.array(edge.info) * errors[k] ** 2))
        return total

    def solve(self, iterations=None):
        """Gauss-Newton, `iterations` times, anchor fixed. Returns a summary.

        Deterministic: a fixed iteration count, a fixed anchor, a fixed edge
        order, and `spsolve` (SuperLU), which is deterministic for a given
        matrix. ⚠️ **Set `OMP_NUM_THREADS=1`** if you need bitwise identity
        across machines — the BLAS underneath the sparse solve may otherwise
        reduce in a thread-count-dependent order. Same-machine identity, which
        is what this project guarantees, holds either way.
        """
        n = len(self.poses)
        iterations = self.iterations if iterations is None else int(iterations)
        summary = {"nodes": n, "edges": len(self.edges),
                   "iterations": 0, "chi2_before": 0.0, "chi2_after": 0.0,
                   "singular": 0}
        if n < 2 or not self.edges:
            # Nothing to optimise. Not an error: a graph with one submap in it
            # is the normal state for the first few seconds of every episode.
            self.last_solve = summary
            return summary

        # The reduced system: every dof except the anchor's three. Built once,
        # because the anchor never changes.
        free = np.array([3 * k + a for k in range(n) if k != self.ANCHOR
                         for a in range(3)])

        errors = self.residuals()
        summary["chi2_before"] = self.chi2(errors)

        for _ in range(iterations):
            J = self.jacobian()[:, free]
            W = diags(self.weights(errors))
            H = csc_matrix(J.T @ W @ J)
            b = J.T @ (W @ errors.reshape(-1))
            try:
                delta = spsolve(H, -b)
            except Exception:                       # pragma: no cover
                summary["singular"] = 1
                break
            if not np.all(np.isfinite(delta)):
                # A disconnected graph, or an edge set that leaves some node
                # unconstrained. Recorded rather than raised: an episode must
                # not die because the front end produced a degenerate graph, and
                # a counter in slam.csv is how that becomes visible.
                summary["singular"] = 1
                break
            summary["iterations"] += 1
            if float(np.max(np.abs(delta))) > NULL_UPDATE:
                self._apply(free, delta)
                errors = self.residuals()

        summary["chi2_after"] = self.chi2(errors)
        self.last_solve = summary
        return summary

    def _apply(self, free, delta):
        poses = np.array(self.poses, dtype=float).reshape(-1)
        poses[free] += delta
        poses = poses.reshape(-1, 3)
        self.poses = [(float(p[0]), float(p[1]), scan_mod.wrap_angle(p[2]))
                      for p in poses]
