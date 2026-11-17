import pytest

from coconut_harvester.design import optimize
from coconut_harvester.fleet import Plantation, fleet_needed


@pytest.fixture(scope="session")
def design():
    d = optimize()
    assert d.ok, [c.name for c in d.failed()]
    return d


@pytest.fixture(scope="session")
def plantation():
    return Plantation()


@pytest.fixture(scope="session")
def fleet(design, plantation):
    return fleet_needed(design, plantation)


@pytest.fixture(scope="session")
def iw():
    from coconut_harvester.inchworm import optimize as optimize_iw
    d = optimize_iw()
    assert d.ok, [c.name for c in d.failed()]
    return d


@pytest.fixture(scope="session")
def iw_fleet(iw, plantation):
    return fleet_needed(iw, plantation)
