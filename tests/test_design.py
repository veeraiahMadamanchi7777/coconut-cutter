"""The sized robot against the tree population it must climb."""
import math
import random

import pytest
from hypothesis import HealthCheck, given, settings, strategies as st

from coconut_harvester.design import (BATTERIES, G, MOTORS, N_WHEELS, TIER_SPACINGS, WHEELS, Requirements,
                                      optimize, ridge_slope, size)
from coconut_harvester.tree import Dist, Tree, TreePopulation


def test_every_check_passes(design):
    assert design.ok, "\n".join(f"{c.name}: {c.value} vs {c.limit}" for c in design.failed())
    assert len(design.checks) >= 14


def test_catalogue_parts_only(design):
    assert design.wheel in WHEELS and design.motor in MOTORS
    assert design.battery_ah in BATTERIES and design.H in TIER_SPACINGS


def test_200k_trees_climbable_when_dry_and_holdable_when_wet(design):
    worst = math.inf
    for t in design.pop.stream(200_000, seed=11):
        assert design.clamps(t)
        assert design.can_climb(t, wet=False), t
        assert design.can_hold(t, wet=True), t
        worst = min(worst, design.traction_available(t.mu_dry) / design.traction_demand(t))
    assert worst >= design.req.sf_climb


def test_wet_bark_is_outside_the_climb_envelope_for_most_trees(design):
    """Documents why the controller forbids climbing wet bark."""
    wet_ok = sum(design.can_climb(t, wet=True) for t in design.pop.stream(5000, seed=2))
    assert wet_ok < 5000


@given(st.floats(50, 100), st.floats(0.5, 15), st.floats(20, 100))
def test_ridge_slope_properties(r, h, L):
    s = ridge_slope(r, h, L)
    assert 0 < s
    assert s <= math.pi * h / L + 1e-12
    assert ridge_slope(r, h * 0.5, L) <= s + 1e-12          # smaller ridge, gentler
    assert ridge_slope(r * 1.5, h, L) <= s + 1e-12          # bigger wheel, gentler
    assert ridge_slope(r, 0, L) == 0


def test_energy_is_physical(design):
    h = 15.0
    potential_wh = design.mass * G * h / 3600
    assert design.energy_climb(h) > potential_wh / design.eta_worm * 0.99  # worm losses at least
    assert 0 < design.energy_descend(h) < design.energy_climb(h)
    assert design.energy_climb(2 * h) == pytest.approx(2 * design.energy_climb(h))


def test_self_locking_drive_holds_without_power(design):
    assert design.eta_worm < 0.5


def test_heavier_requirements_cost_mass():
    light = optimize()
    heavy = optimize(Requirements(min_trees_per_charge=40, max_mass=60))
    assert heavy.battery_ah > light.battery_ah and heavy.mass > light.mass


def test_impossible_envelope_is_reported():
    slippery = TreePopulation(mu_dry=Dist(0.3, 0.02, 0.25, 0.35))
    with pytest.raises(RuntimeError):
        optimize(pop=slippery)
    d = size(Requirements(), slippery, WHEELS[0], H=0.6)
    assert not d.ok and d.failed()


def test_optimizer_returns_lightest_passing(design):
    for w in WHEELS:
        for H in TIER_SPACINGS:
            d = size(design.req, design.pop, w, H=H)
            if d.ok:
                assert d.mass >= design.mass - 1e-9


@settings(max_examples=40, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(mu_lo=st.floats(0.45, 0.65), ridge_hi=st.floats(6, 12), height_hi=st.floats(15, 26),
       lean_hi=st.floats(10, 25), ped_hi=st.floats(60, 85))
def test_any_passing_design_really_serves_its_population(mu_lo, ridge_hi, height_hi, lean_hi, ped_hi):
    pop = TreePopulation(mu_dry=Dist(0.7, 0.07, mu_lo, 0.9), ridge_h=Dist(5, 2, 1, ridge_hi),
                         height=Dist(12, 4, 4, height_hi), lean=Dist(5, 4, 0, lean_hi),
                         peduncle=Dist(50, 8, 35, ped_hi))
    try:
        d = optimize(Requirements(max_mass=60), pop)
    except RuntimeError:
        return
    rng = random.Random(1)
    for _ in range(2000):
        t = pop.sample(rng)
        assert d.can_climb(t, wet=False) and d.can_hold(t, wet=True)
        assert d.cut_time(pop.peduncle.hi) <= d.req.max_cut_time
        e_worst = d.energy_tree(t, [pop.peduncle.hi] * 4, 300) + d.descent_reserve(pop.height.hi)
        assert e_worst <= d.battery_wh * d.req.battery_dod
