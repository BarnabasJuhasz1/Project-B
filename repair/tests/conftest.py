"""Local pytest config for `repair/`.

Scoped to this package deliberately: the repo has no root `pytest.ini` and
adding one would change how every other suite is collected. Registering the
marker here keeps the change where it belongs.
"""


def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "slow: needs real kinematic rollouts (~40 s each). Deselect with "
        "-m 'not slow'.")


import shutil

import pytest


@pytest.fixture
def staged_round(tmp_path, monkeypatch):
    """A repair round on disk, built from a COMMITTED candidate.

    ⚠️ WHY THIS EXISTS. The two tests that pin `entry_for`'s behaviour used to
    resolve `…__r01` out of `out/repair/`, which is a RUN OUTPUT: gitignored,
    swept by `tidy_out.py`, and — as of 2026-09-19 — moved wholesale into
    `out/repair/archive/` the moment a lineage changes axis. Archiving a
    finished lineage turned both tests red, and on a fresh clone they had never
    been able to pass at all. A test that pins the worst bug this epic has had
    must not depend on scratch state that a legitimate housekeeping move can
    take away.

    The round is assembled from `generate/staging/`, which IS committed: three
    files under a `round-NN/` directory is the whole of what a round is on
    disk, so nothing about the resolver is being faked. `WORK` is repointed at
    `tmp_path`, so the fixture also cannot see — or damage — a real lineage.
    """
    from generate.generator import STAGING
    from repair import evidence as E

    stem, node = "b__path_integration_ants__44d12df4d25b437f", NODE
    monkeypatch.setattr(E, "WORK", tmp_path)
    d = tmp_path / node / "round-01"
    d.mkdir(parents=True)
    for ext in (".py", ".json", ".result.json"):
        shutil.copy(STAGING / f"{stem}{ext}", d / f"{stem}{ext}")
    return f"{node}__r01"


NODE = "gen_b_44d12df4d25b437f"
