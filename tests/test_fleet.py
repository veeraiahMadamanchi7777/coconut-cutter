"""Plantation-scale operations."""
import pytest

from coconut_harvester.fleet import FleetSize, Ops, Plantation, simulate_year
from coconut_harvester.tree import Dist, TreePopulation, Weather

SMALL = Plantation(hectares=10)


def test_deterministic(design):
    a = simulate_year(design, 2, SMALL, seed=3, days=120)
    b = simulate_year(design, 2, SMALL, seed=3, days=120)
    assert (a.harvests, a.bunches, a.lateness) == (b.harvests, b.bunches, b.lateness)


def test_every_palm_harvested_on_schedule_with_enough_robots(design):
    y = simulate_year(design, 3, SMALL, seed=1)
    rounds = 365 / SMALL.cycle_days
    assert y.harvests >= SMALL.palms * (rounds - 1.5)
    assert y.p95_late <= 3


def test_too_few_robots_falls_behind(design):
    y = simulate_year(design, 1, Plantation(hectares=25), seed=1)
    assert y.max_late > 30


def test_dry_climate_needs_fewer_robot_hours_than_monsoon(design):
    dry = simulate_year(design, 3, SMALL, weather=Weather(rain_prob=[0.0] * 12), seed=2)
    wet = simulate_year(design, 3, SMALL, weather=Weather(rain_prob=[0.6] * 12), seed=2)
    assert wet.lost_wet_hours > dry.lost_wet_hours == 0
    assert wet.max_late > dry.max_late


def test_trees_outside_envelope_go_to_manual_climber(design):
    giants = TreePopulation(d_base=Dist(400, 60, 300, 600))  # some trunks too fat for the ring
    y = simulate_year(design, 3, SMALL, seed=4, days=90, pop=giants)
    assert y.manual > 0


def test_battery_packs_never_starve_the_robot(design):
    y = simulate_year(design, 3, SMALL, seed=5)
    assert y.pack_waits == 0
    one_pack = simulate_year(design, 3, SMALL, ops=Ops(packs_per_robot=1, charge_h=3), seed=5)
    assert one_pack.pack_waits > 0


def test_plantation_fleet_meets_service_level(design, fleet, plantation):
    assert fleet.ok and len(fleet.results) == 30
    assert fleet.pass_frac >= fleet.service.year_pass_frac
    assert fleet.worst_max <= fleet.service.hard_max_late
    from concurrent.futures import ProcessPoolExecutor
    from coconut_harvester.fleet import Ops, _year
    from coconut_harvester.tree import Weather
    with ProcessPoolExecutor() as pool:
        ys = list(pool.map(_year, [(design, fleet.robots - 1, plantation, Ops(), Weather(), s) for s in range(30)]))
    smaller = FleetSize(fleet.robots - 1, ys, fleet.service)
    assert not smaller.ok  # fleet is not oversized
    for y in fleet.results:
        assert y.harvests >= plantation.palms * (365 / plantation.cycle_days - 1.5)
        assert y.manual == 0 and y.pack_waits == 0


def test_fast_path_energy_matches_design_model(design):
    """simulate_year inlines Design.energy_tree for speed; keep them identical."""
    import random
    rng = random.Random(8)
    ops = Ops()
    for _ in range(500):
        t = design.pop.sample(rng)
        peds = design.pop.bunches(rng, 45)
        elec = design.req.electronics_w / 3600
        fast = (design.energy_climb(t.height) + design.energy_descend(t.height) + elec * 2 * t.height / design.speed
                + sum(design.saw_w * (design.cut_time(p) + 3) / 3600 + 0.06 + elec * ops.aim_s for p in peds))
        assert fast == pytest.approx(design.energy_tree(t, peds, dwell_s=len(peds) * ops.aim_s), rel=1e-12)
