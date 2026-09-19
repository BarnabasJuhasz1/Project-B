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
