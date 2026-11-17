import random

import pytest
from hypothesis import given, settings, strategies as st

from coconut_cutter.coconut import Coconut, CoconutPopulation
from coconut_cutter.mechanics import (MaterialModel, Punch, calibrate, cut_energy, force_at_depth,
                                      peak_force, required_depth)

nuts = st.builds(
    Coconut,
    diameter=st.floats(120, 220), height=st.floats(150, 270),
    husk_top=st.floats(3, 30), shell=st.floats(1, 7),
    strength=st.floats(0.5, 1.8), mass=st.floats(0.8, 2.5),
)
punches = st.builds(Punch, diameter=st.floats(25, 50), engagement=st.floats(0.3, 1.0))
mats = st.builds(MaterialModel, q_husk=st.floats(1, 40), q_shell=st.floats(1, 80),
                 mu=st.floats(0.05, 0.8), p_lat=st.floats(0.0, 0.2))


@given(nuts, punches, mats)
@settings(max_examples=300)
def test_analytic_peak_matches_dense_sweep(nut, punch, mat):
    depth = required_depth(nut, punch)
    zs = [depth * i / 4000 for i in range(4001)] + [nut.husk_top, nut.husk_top + nut.shell]
    sweep = max(force_at_depth(z, nut, punch, mat) for z in zs)
    assert peak_force(nut, punch, mat) == pytest.approx(sweep, rel=1e-12)


@given(nuts, punches, mats)
def test_force_non_negative_and_zero_before_contact(nut, punch, mat):
    assert force_at_depth(0.0, nut, punch, mat) == 0.0
    assert force_at_depth(-5.0, nut, punch, mat) == 0.0
    for z in (0.5, nut.husk_top, nut.husk_top + nut.shell, 100):
        assert force_at_depth(z, nut, punch, mat) >= 0.0


@given(nuts, punches, mats, st.floats(1.01, 3.0))
def test_harder_nut_needs_more_force(nut, punch, mat, k):
    harder = Coconut(**{**nut.__dict__, "strength": nut.strength * k})
    assert peak_force(harder, punch, mat) > peak_force(nut, punch, mat)


def test_energy_matches_hand_calculation():
    nut = Coconut(160, 200, husk_top=10, shell=2, strength=1, mass=1.4)
    punch = Punch(diameter=40, engagement=0.5, min_overtravel=8)
    mat = MaterialModel(q_husk=5, q_shell=20, mu=0.0, p_lat=0.0)
    P = punch.perimeter
    expected = (P * 0.5 * 5 * 10 + P * 0.5 * 20 * 2) / 1000  # J
    assert cut_energy(nut, punch, mat, required_depth(nut, punch), steps=20000) == pytest.approx(expected, rel=2e-3)


def test_calibration_recovers_true_material_from_noisy_rig_data():
    true = MaterialModel(q_husk=7.3, q_shell=26.1)
    punch = Punch()
    rng = random.Random(3)
    samples = []
    for nut in CoconutPopulation().stream(60, seed=9):
        nut = Coconut(**{**nut.__dict__, "strength": 1.0})
        depth = required_depth(nut, punch)
        trace = [(z, force_at_depth(z, nut, punch, true) * rng.gauss(1, 0.05))
                 for z in (depth * i / 200 for i in range(1, 201))]
        samples.append((nut, trace))
    fit = calibrate(samples, punch)
    assert fit.q_husk == pytest.approx(true.q_husk, rel=0.02)
    assert fit.q_shell == pytest.approx(true.q_shell, rel=0.02)


def test_calibration_rejects_traces_missing_a_layer():
    nut = Coconut(160, 200, 10, 2, 1, 1.4)
    with pytest.raises(ValueError):
        calibrate([(nut, [(1.0, 100.0), (5.0, 120.0)])], Punch())
