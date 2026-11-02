"""demo/test_code_features.py — Exp 4's static features and blinding. SIM-FREE.

    P=~/miniforge3/envs/env_isaacsim/bin/python
    $P -m pytest demo/test_code_features.py -q

The detectors are pinned on small synthetic controllers written for the test,
so a test failure says which detector moved. The last two tests run over the
real pilot controllers when `demo/arms/` is present and skip otherwise.
"""
import csv
import pathlib
import sys
import textwrap

import pytest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from demo import blind_controllers as bc  # noqa: E402
from demo import code_features as cf  # noqa: E402

ARMS = pathlib.Path(__file__).resolve().parent / "arms"


def src(s: str) -> str:
    return textwrap.dedent(s).lstrip("\n")


# A full-replan, rollout-scored controller with a docstring header.
FULL = src('''
    """Map + wavefront + DWA. Uses nothing from any paper."""

    import math
    import time
    import numpy as np
    from core.observation import Command

    INF = 1e9

    class Controller:
        HORIZON = 1.0

        def reset(self, scenario, rng):
            self.lo = np.zeros((40, 30), dtype=np.float32)
            self.D = np.full((40, 30), INF)
            vx = np.array([0.0, 0.4, 0.8])
            wz = np.array([-1.0, 0.0, 1.0])
            VX, WZ = np.meshgrid(vx, wz)
            self.cvx, self.cwz = VX.ravel(), WZ.ravel()
            self.th = self.cwz * self.HORIZON

        def _plan(self, goal):
            t0 = time.perf_counter()
            D = np.full((40, 30), INF, dtype=np.float32)
            return D

        def step(self, obs):
            D = self._plan(obs.goal)
            return Command(0.0, 0.0, 0.0)
''')

# An incremental D* controller with a `#` header that names its source.
DSTAR = src('''
    # Built on D* (Stentz 1994): PROCESS-STATE, MODIFY-COST, RAISE / LOWER.
    # k_min >= h(robot) stopping rule.

    import heapq
    import numpy as np
    from core.observation import Command

    RAISE, LOWER = 1, 2

    class DStar:
        def __init__(self, n):
            self.h = np.full(n, np.inf)
            self.k = np.full(n, np.inf)
            self.open = []

        def process_state(self):
            k, x = heapq.heappop(self.open)
            return k

        def modify_cost(self, x, c):
            heapq.heappush(self.open, (self.h[x], x))

    class Controller:
        def reset(self, scenario, rng):
            self.p = DStar(100)

        def step(self, obs):
            # follow backpointers toward the goal
            return Command(0.4, 0.0, 0.0)
''')


# ---- header ------------------------------------------------------------------

def test_split_header_takes_a_docstring_and_nothing_after_it():
    h, b = cf.split_header(FULL)
    assert h + b == FULL
    assert "Uses nothing" in h and b.startswith("import math")


def test_split_header_takes_a_comment_block():
    h, b = cf.split_header(DSTAR)
    assert h + b == DSTAR
    assert "Stentz" in h and "Stentz" not in b and b.startswith("import heapq")


def test_split_header_takes_comments_then_a_docstring():
    s = "#!/usr/bin/env python\n# from the paper\n\n'''doc'''\n# more\nimport os\n"
    h, b = cf.split_header(s)
    assert b == "import os\n"


def test_split_header_leaves_a_headerless_file_alone():
    s = "import os\nx = 1\n"
    assert cf.split_header(s) == ("", s)


# ---- features ----------------------------------------------------------------

def test_counts_are_taken_on_the_body_not_the_header():
    f = cf.features(DSTAR)
    # "Stentz" / "D*" appear only in the stripped header; the two body hits
    # are the identifier `DStar`, which is a claim and must count
    assert f["dstar_dstar_name"] == 2
    assert f["dstar_RAISE"] == 1 and f["dstar_LOWER"] == 1
    assert f["dstar_PROCESS_STATE"] == 1 and f["dstar_MODIFY_COST"] == 1
    assert f["header_lines"] == 3  # two comment lines + the blank after them


def test_multiplication_by_d_is_not_dstar_vocabulary():
    s = "import math\nclass Controller:\n    def step(self, obs):\n        d = 1\n        x = d * math.cos(0)\n        y = d *2\n"
    assert cf.features(s)["dstar_vocab_total"] == 0
    assert cf.features("# uses D* here\nimport os\nx = 1  # D* Lite\n")["dstar_dstar_name"] == 1


def test_lower_case_lower_and_raise_are_english_not_dstar():
    s = "class Controller:\n    def step(self, obs):\n        # must never raise; lower the speed\n        return 0\n"
    assert cf.features(s)["dstar_vocab_total"] == 0


def test_heapq_and_heap_ops():
    f = cf.features(DSTAR)
    assert f["uses_heapq"] == 1 and f["heap_ops"] == 2
    assert cf.features(FULL)["uses_heapq"] == 0


def test_persistent_grid_needs_a_grid_shaped_self_attribute():
    f = cf.features(FULL)
    assert f["persistent_grid_attrs"] == "D;lo"
    # 1-D state vectors on DStar are not grids
    assert cf.features(DSTAR)["persistent_grid_attrs"] == ""
    s = "import numpy as np\nclass C:\n    def reset(self, s, r):\n        shp = (4, 4)\n        self.a = np.zeros(shp)\n        self.b = np.zeros(n * n)\n        self.c = np.zeros(3)\n        self.d = np.zeros(K)\n"
    assert cf.features(s)["persistent_grid_attrs"] == "a;b"


def test_full_reinit_counts_episode_methods_only():
    # reset()'s np.full is initialisation; _plan's is a from-scratch replan
    assert cf.features(FULL)["full_reinit_sites"] == 1
    # DStar.__init__'s np.full is initialisation
    assert cf.features(DSTAR)["full_reinit_sites"] == 0
    s = "import numpy as np\nBIG = 1e9\nclass C:\n    def plan(self):\n        self.f[:] = BIG\n        self.g.fill(np.inf)\n        self.h[...] = np.inf\n        z = np.full(xs, BIG)\n"
    assert cf.features(s)["full_reinit_sites"] == 3


def test_wall_clock_budget_calls():
    assert cf.features(FULL)["time_budget_calls"] == 1
    s = "from time import perf_counter\nimport time\nclass C:\n    def step(self, o):\n        a = perf_counter()\n        b = time.time()\n        c = time.monotonic()\n"
    assert cf.features(s)["time_budget_calls"] == 3


def test_rollout_needs_candidates_and_forward_simulation():
    f = cf.features(FULL)
    assert f["candidate_grids"] == 1 and f["rollout_vt_products"] >= 1
    assert f["rollout_scoring"] == 1
    assert cf.features(DSTAR)["rollout_scoring"] == 0
    # a pose integrated in a loop over nested v / w candidates
    loop = src('''
        import math
        class C:
            def step(self, obs):
                for v in (0.2, 0.4):
                    for w in (-1.0, 1.0):
                        x = 0.0
                        for _ in range(5):
                            x += v * math.cos(w) * dt
    ''')
    f = cf.features(loop)
    assert f["candidate_grids"] == 1 and f["rollout_integrations"] == 1
    assert f["rollout_scoring"] == 1
    # steering at a waypoint: trig, but no candidates and no horizon
    steer = "import math\nclass C:\n    def step(self, o):\n        b = math.atan2(1, 2)\n        return 0.5 * math.cos(b)\n"
    assert cf.features(steer)["rollout_scoring"] == 0


def test_features_are_deterministic_and_csv_is_byte_identical(tmp_path):
    for name, s in (("arm0", FULL), ("armB", DSTAR)):
        (tmp_path / "arms" / name).mkdir(parents=True)
        (tmp_path / "arms" / name / "k0.py").write_text(s)
    outs = []
    for i in range(2):
        out = tmp_path / f"f{i}.csv"
        assert cf.main(["--arms", str(tmp_path / "arms"), "--out", str(out)]) == 0
        outs.append(out.read_bytes())
    assert outs[0] == outs[1]
    rows = list(csv.DictReader(open(tmp_path / "f0.csv")))
    assert [r["arm"] for r in rows] == ["arm0", "armB"]
    assert list(rows[0]) == cf.COLUMNS


def test_discover_sorts_k_numerically(tmp_path):
    for k in (10, 2, 0):
        (tmp_path / "a").mkdir(exist_ok=True)
        (tmp_path / "a" / f"k{k}.py").write_text("x = 1\n")
    (tmp_path / "a" / "notes.py").write_text("x = 1\n")
    assert [k for _, k, _ in cf.discover(tmp_path)] == ["0", "2", "10"]


# ---- blinding ----------------------------------------------------------------

def _arms(tmp_path):
    root = tmp_path / "arms"
    for name, s in (("arm0", FULL), ("armB", DSTAR)):
        for k in range(3):
            (root / name).mkdir(parents=True, exist_ok=True)
            (root / name / f"k{k}.py").write_text(s + f"\nK_TAG = {k}\n")
    return root


def test_blinding_strips_the_header_and_nothing_else(tmp_path):
    root = _arms(tmp_path)
    out, key = tmp_path / "blind", tmp_path / "keys" / "key.csv"
    rows = bc.blind(cf.discover(root), out, key, seed=7)
    assert len(rows) == 6 and len({r["blind_id"] for r in rows}) == 6
    for r in rows:
        original = (root / r["arm"] / f"k{r['k']}.py").read_text()
        blinded = (out / f"{r['blind_id']}.py").read_text()
        assert blinded == cf.split_header(original)[1]
        assert "Stentz" not in blinded and "Uses nothing" not in blinded


def test_blind_ids_and_filenames_carry_no_arm(tmp_path):
    root = _arms(tmp_path)
    out = tmp_path / "blind"
    bc.blind(cf.discover(root), out, tmp_path / "k.csv", seed=1)
    for p in out.iterdir():
        assert not any(t in p.name for t in ("arm", "k0", "k1", "k2"))
    assert not list(out.glob("*.csv")), "the key must not sit beside the blinded files"


def test_blinding_is_reproducible_from_the_recorded_seed(tmp_path):
    root = _arms(tmp_path)
    a = bc.blind(cf.discover(root), tmp_path / "b1", tmp_path / "k1.csv", seed=42)
    b = bc.blind(cf.discover(root), tmp_path / "b2", tmp_path / "k2.csv", seed=42)
    c = bc.blind(cf.discover(root), tmp_path / "b3", tmp_path / "k3.csv", seed=43)
    assert a == b
    assert [r["blind_id"] for r in a] != [r["blind_id"] for r in c]


def test_blinding_refuses_to_redraw_a_started_set(tmp_path):
    root = _arms(tmp_path)
    out = tmp_path / "blind"
    bc.blind(cf.discover(root), out, tmp_path / "k.csv", seed=1)
    with pytest.raises(SystemExit):
        bc.blind(cf.discover(root), out, tmp_path / "k.csv", seed=2)


def test_blinding_refuses_a_key_inside_the_blinded_dir(tmp_path):
    root = _arms(tmp_path)
    with pytest.raises(SystemExit):
        bc.blind(cf.discover(root), tmp_path / "b", tmp_path / "b" / "key.csv", seed=1)


def test_strip_comments_keeps_code_and_docstrings():
    s = 'def f():\n    """doc"""\n    # gone\n    x = 1  # gone too\n    return x\n'
    assert bc.strip_comments(s) == 'def f():\n    """doc"""\n    x = 1\n    return x\n'


# ---- the real pilot set ------------------------------------------------------

needs_arms = pytest.mark.skipif(not (ARMS / "arm0").is_dir(), reason="demo/arms not present")


@needs_arms
def test_every_pilot_controller_parses_and_scores():
    items = cf.discover(ARMS)
    assert {a for a, _, _ in items} >= {"arm0", "armB"}
    rows = cf.rows_for(items)
    assert all(r["loc"] > 100 for r in rows)


@needs_arms
def test_no_pilot_header_survives_blinding():
    """armB is told to cite its source at the top. After stripping, no blinded
    pilot file may still OPEN with a comment or a docstring."""
    for _, _, p in cf.discover(ARMS):
        body = cf.split_header(p.read_text())[1]
        first = body.lstrip().splitlines()[0]
        assert not first.startswith(("#", '"""', "'''")), (p, first)
