import pytest

from coconut_cutter.design import optimize


@pytest.fixture(scope="session")
def design():
    d = optimize()
    assert d.ok, [c.name for c in d.failed()]
    return d


@pytest.fixture(scope="session")
def capacity(design):
    from coconut_cutter.line_sim import stations_needed
    return stations_needed(design, design.req.target_rate_per_h, design.req.hours_per_day, replications=20)
