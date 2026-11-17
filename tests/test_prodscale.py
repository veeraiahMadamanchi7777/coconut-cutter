"""Production-scale soak.

Default: the 100 ha reference estate over 10 fresh weather years.
``pytest -m prodscale``: a 1,000 ha estate (175,000 palms) and 20,000 closed-loop missions.
"""
import math
import random

import pytest

from coconut_harvester.fleet import Plantation, simulate_year
from coconut_harvester.robot_sim import run_mission


def _missions(design, n, seed):
    rng = random.Random(seed)
    for _ in range(n):
        t = design.pop.sample(rng)
        r = run_mission(design, t, design.pop.bunches(rng, 45), rng=rng)
        assert r.ok, (t, r)


def test_reference_estate_out_of_sample_years(design, fleet, plantation):
    """Fresh weather years the sizing never saw: the service level still holds."""
    years = [simulate_year(design, fleet.robots, plantation, seed=1000 + s) for s in range(10)]
    passed = sum(fleet.service.year_ok(y) for y in years)
    assert passed >= 8  # 90 % target; allow sampling noise on 10 fresh years
    for y in years:
        assert y.max_late <= fleet.service.hard_max_late
        assert y.harvests >= plantation.palms * (365 / plantation.cycle_days - 1.5)


@pytest.mark.prodscale
def test_1000_hectare_estate(design, fleet):
    big = Plantation(hectares=1000)
    robots = fleet.robots * 10
    y = simulate_year(design, robots, big, seed=7)
    print(f"\n1000 ha: {robots} robots, {y.harvests:,} harvests, {y.bunches:,} bunches, "
          f"P95 late {y.p95_late:.1f} d, max {y.max_late:.0f} d")
    assert y.harvests >= big.palms * (365 / big.cycle_days - 1.5)
    assert y.max_late <= fleet.service.hard_max_late


@pytest.mark.prodscale
def test_20000_closed_loop_missions(design):
    _missions(design, 20_000, seed=77)


def test_inchworm_reference_estate_out_of_sample_years(iw, iw_fleet, plantation):
    years = [simulate_year(iw, iw_fleet.robots, plantation, seed=2000 + s) for s in range(10)]
    assert sum(iw_fleet.service.year_ok(y) for y in years) >= 8
    assert all(y.max_late <= iw_fleet.service.hard_max_late for y in years)


@pytest.mark.prodscale
def test_inchworm_1000_hectare_estate(iw, iw_fleet):
    big = Plantation(hectares=1000)
    robots = iw_fleet.robots * 10
    y = simulate_year(iw, robots, big, seed=8)
    print(f"\ninchworm 1000 ha: {robots} robots, {y.harvests:,} harvests, P95 late {y.p95_late:.1f} d, "
          f"max {y.max_late:.0f} d")
    assert y.harvests >= big.palms * (365 / big.cycle_days - 1.5)
    assert y.max_late <= iw_fleet.service.hard_max_late


@pytest.mark.prodscale
def test_inchworm_5000_closed_loop_missions_wet_and_dry(iw):
    from coconut_harvester.inchworm_control import run_mission as iw_run
    rng = random.Random(55)
    for n in range(5000):
        t = iw.pop.sample(rng)
        r = iw_run(iw, t, iw.pop.bunches(rng, 45), wet=n % 2 == 1, rng=rng)
        assert r.ok and not r.fell, (t, r)
