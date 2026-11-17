"""Production-line simulation: correctness of the model and plant-scale capacity."""
from dataclasses import replace

import pytest

from coconut_cutter.design import Requirements, size
from coconut_cutter.line_sim import LineParams, simulate_line, stations_needed
from coconut_cutter.mechanics import MaterialModel


def test_deterministic_for_a_seed(design):
    a = simulate_line(design, 3, 8, seed=11)
    b = simulate_line(design, 3, 8, seed=11)
    assert a.nuts == b.nuts and a.jams == b.jams
    assert simulate_line(design, 3, 8, seed=12).nuts != a.nuts


def test_time_accounting_never_exceeds_horizon(design):
    r = simulate_line(design, 4, 16, seed=3)
    for s in r.per_station:
        assert 0 <= s.busy_s + s.down_s <= 16 * 3600 + 1e-6
    assert 0.8 < r.availability <= 1.0


def test_rate_bounded_by_ideal_cycle(design):
    lp = LineParams()
    mean_nut_machine = sum(design.machine_time(n) for n in design.pop.stream(20_000, seed=1)) / 20_000
    ideal_per_station = 3600 / (lp.load_mean + lp.unload_mean + mean_nut_machine)
    r = simulate_line(design, 1, 16, seed=4)
    assert 0.75 * ideal_per_station < r.rate_per_h < 1.05 * ideal_per_station


def test_throughput_scales_with_stations(design):
    rates = [simulate_line(design, n, 16, seed=5).rate_per_h for n in (1, 2, 4, 8)]
    assert rates == sorted(rates)
    assert rates[3] == pytest.approx(8 * rates[0], rel=0.08)


def test_no_stalls_at_plant_scale_with_sized_cylinder(design):
    r = simulate_line(design, 4, 16, seed=6)
    assert r.stalls == 0
    assert max(s.peak_force_max for s in r.per_station) <= design.force_available / design.req.force_sf


def test_undersized_machine_shows_stalls_and_lost_output():
    """The simulator must punish a bad design, not just the checks."""
    weak = size(Requirements(supply_min_mpa=0.30, force_sf=1.0))
    strong_mat = MaterialModel(q_shell=weak.mat.q_shell * 1.9)
    sim_design = replace(weak, mat=strong_mat)
    r = simulate_line(sim_design, 2, 16, seed=1)
    assert r.stalls > 0


def test_slower_operators_need_more_stations(design, capacity):
    fast = capacity
    slow = stations_needed(design, 2000, replications=8,
                           lp=LineParams(load_mean=4.0, unload_mean=2.5))
    assert slow.stations > fast.stations


def test_plant_target_met_on_bad_days(design, capacity):
    """P05 of 20 simulated production days meets the 2000 nuts/h plant target."""
    cap = capacity
    assert cap.p05_rate >= design.req.target_rate_per_h
    assert cap.stations <= 6  # sanity: a sensible floor footprint
    # one station fewer must NOT be enough, or we oversized the line
    fewer = [simulate_line(design, cap.stations - 1, 16, seed=s).rate_per_h for s in range(20)]
    assert sorted(fewer)[0] < design.req.target_rate_per_h


def test_simulation_terminates_for_every_station_offset(design):
    """Regression: float rounding at a break boundary once caused an infinite loop."""
    for n in (7, 8, 9):
        for seed in range(3):
            r = simulate_line(design, n, 8.0, seed=seed)
            assert r.nuts > 0


def test_compressor_keeps_up_with_line_running_flat_out(design, capacity):
    from coconut_cutter.design import plant_air
    air = plant_air(design, capacity.stations, max(capacity.rates))
    for seed in range(5):
        r = simulate_line(design, capacity.stations, 16, seed=100 + seed)
        demand = design.air_per_cycle_nl * r.rate_per_h / 60
        assert demand < air.compressor_fad_nl_min
