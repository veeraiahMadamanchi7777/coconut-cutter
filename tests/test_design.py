"""Verify the sized machine against the population it must serve."""
import random

import pytest
from hypothesis import HealthCheck, given, settings, strategies as st

from coconut_cutter.coconut import CoconutPopulation, Dist
from coconut_cutter.controller import Config
from coconut_cutter.design import ISO15552, Requirements, optimize, plant_air, size
from coconut_cutter.mechanics import MaterialModel, Punch, required_depth


def test_every_engineering_check_passes(design):
    assert design.checks and design.ok, "\n".join(f"{c.name}: {c.value} vs {c.limit}" for c in design.failed())


def test_catalogue_parts_only(design):
    assert design.bore in ISO15552 and design.rod == ISO15552[design.bore]
    assert design.stroke in (100, 125, 160, 200, 250, 320)


def test_monte_carlo_200k_nuts_dull_crown_never_stalls(design):
    """Every nut, with the crown at its dullest, is cut with ≥ the required force margin."""
    rng = random.Random(2026)
    worst = 0.0
    for _ in range(200_000):
        nut = design.pop.sample(rng)
        worst = max(worst, design.required_force(nut, design.req.blade_dull_at_swap))
    assert worst <= design.worst_peak_force  # envelope nut truly bounds the population
    assert design.force_available / worst >= design.req.force_sf


def test_every_nut_gets_a_clean_hole_without_gouging(design):
    for nut in design.pop.stream(100_000, seed=7):
        assert design.depth_cut >= required_depth(nut, design.punch) - 1e-9
        intrusion = design.depth_cut - nut.husk_top - nut.shell
        assert design.punch.min_overtravel - 1e-9 <= intrusion <= design.req.max_cavity_intrusion


def test_travel_always_within_stroke(design):
    shortest = min(design.pop.stream(100_000, seed=8), key=lambda c: c.height)
    assert design.travel(shortest) <= design.stroke
    assert design.pop.height.hi + design.req.load_clearance - design.pop.height.lo + design.depth_cut <= design.stroke


def test_stack_up_is_ordered_and_clears_tallest_nut(design):
    lv = design.levels()
    order = ["nut_bottom", "tallest_top", "tip_home", "carriage_bottom", "carriage_top",
             "arm_bottom", "arm_top", "cylinder_top"]
    vals = [lv[k] for k in order]
    assert vals == sorted(vals) and len(set(vals)) == len(vals)
    assert lv["tip_home"] - lv["tallest_top"] == design.req.load_clearance
    assert design.column_length == lv["arm_top"]


def test_controller_timeouts_cover_slowest_stroke_with_margin(design):
    cfg = Config()
    slowest = max(design.travel(n) for n in design.pop.stream(50_000, seed=1))
    assert cfg.extend_timeout >= 1.5 * slowest / design.kin.extend_speed
    assert cfg.retract_timeout >= 1.5 * slowest / design.kin.retract_speed
    assert cfg.blade_interval == design.req.blade_resharpen_cuts


def test_harder_coconuts_drive_a_bigger_cylinder():
    soft = size(mat=MaterialModel(q_shell=15))
    hard = size(mat=MaterialModel(q_shell=40))
    assert hard.bore > soft.bore
    assert hard.air_per_cycle_nl > soft.air_per_cycle_nl


def test_impossible_requirements_are_reported_not_hidden():
    d = size(mat=MaterialModel(q_husk=80, q_shell=400))
    assert not d.ok
    assert any("Cylinder force" in c.name for c in d.failed())
    with pytest.raises(RuntimeError):
        optimize(mat=MaterialModel(q_husk=80, q_shell=400))


def test_optimizer_returns_lowest_air_valid_design(design):
    for D in (32.0, 34.0, 36.0, 38.0, 40.0, 45.0):
        for e in (0.40, 0.45, 0.50, 0.60, 0.70):
            cand = size(punch=Punch(diameter=D, engagement=e))
            if cand.ok:
                assert cand.air_per_cycle_nl >= design.air_per_cycle_nl - 1e-9


def test_plant_air_scales_with_rate(design):
    a = plant_air(design, 4, 2000)
    b = plant_air(design, 8, 4000)
    assert b.demand_nl_min == pytest.approx(2 * a.demand_nl_min)
    assert a.compressor_fad_nl_min > a.demand_nl_min
    assert a.receiver_l > 0 and 1 < a.compressor_kw < 50


@settings(max_examples=60, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(
    q_shell=st.floats(10, 45), q_husk=st.floats(3, 12),
    husk_hi=st.floats(18, 35), p_min=st.floats(0.45, 0.65),
    dia=st.sampled_from([32.0, 36.0, 40.0]), eng=st.sampled_from([0.4, 0.5, 0.6]),
)
def test_any_design_that_passes_its_checks_really_serves_its_population(q_shell, q_husk, husk_hi, p_min, dia, eng):
    """Property: the checks are not just self-consistent: an 'ok' design never stalls
    on any sampled nut, and always cuts through, across random assumption sets."""
    pop = CoconutPopulation(husk_top=Dist(14.0, 4.0, 5.0, husk_hi))
    req = Requirements(supply_min_mpa=p_min, supply_max_mpa=max(0.7, p_min + 0.1))
    d = size(req, pop, Punch(diameter=dia, engagement=eng), MaterialModel(q_husk=q_husk, q_shell=q_shell))
    if not d.ok:
        return
    for nut in pop.stream(3000, seed=int(q_shell * 1000)):
        assert d.required_force(nut, req.blade_dull_at_swap) * req.force_sf <= d.force_available + 1e-6
        assert d.travel(nut) <= d.stroke
        assert d.depth_cut >= nut.husk_top + nut.shell + d.punch.min_overtravel - 1e-9
