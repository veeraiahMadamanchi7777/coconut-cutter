"""Production-scale soak tests.

Default run: one production week (~300k nuts) and a 1M-nut force Monte Carlo.
Full-year soak (~10M nuts): ``pytest -m prodscale``.
"""
import random
import time

import pytest

from coconut_cutter.line_sim import simulate_line


def test_one_production_week(design, capacity):
    n = capacity.stations
    days = 6
    r = simulate_line(design, n, days * design.req.hours_per_day, seed=2026)
    assert r.nuts >= design.req.target_rate_per_h * design.req.hours_per_day * days
    assert r.stalls == 0
    swaps = sum(s.swaps for s in r.per_station)
    assert swaps >= r.nuts // design.req.blade_resharpen_cuts - n  # crowns are being changed


def test_one_million_nuts_force_envelope(design):
    rng = random.Random(1_000_000)
    worst = max(design.required_force(design.pop.sample(rng), design.req.blade_dull_at_swap)
                for _ in range(1_000_000))
    assert worst * design.req.force_sf <= design.force_available


@pytest.mark.prodscale
def test_full_year_soak(design, capacity):
    n = capacity.stations
    t0 = time.perf_counter()
    hours = design.req.days_per_year * design.req.hours_per_day
    r = simulate_line(design, n, hours, seed=7)
    annual_target = design.req.target_rate_per_h * hours
    print(f"\nyear: {r.nuts:,} nuts on {n} stations in {time.perf_counter() - t0:.0f}s, "
          f"availability {r.availability:.3f}, jams {r.jams}, breakdowns "
          f"{sum(s.breakdowns for s in r.per_station)}")
    assert r.nuts >= annual_target
    assert r.stalls == 0
