import pytest

from coconut_cutter.coconut import CoconutPopulation, Dist


def test_samples_respect_bounds_and_are_reproducible():
    pop = CoconutPopulation()
    a = list(pop.stream(20_000, seed=1))
    assert a == list(pop.stream(20_000, seed=1))
    for c in a:
        assert pop.diameter.lo <= c.diameter <= pop.diameter.hi
        assert pop.height.lo <= c.height <= pop.height.hi
        assert pop.husk_top.lo <= c.husk_top <= pop.husk_top.hi
        assert pop.shell.lo <= c.shell <= pop.shell.hi
        assert 0.6 <= c.mass <= 4.0  # sanity: real tender nuts are ~1-2 kg


def test_mean_mass_is_realistic():
    nuts = list(CoconutPopulation().stream(20_000, seed=2))
    assert 1.0 < sum(c.mass for c in nuts) / len(nuts) < 2.2


def test_invalid_distribution_rejected():
    with pytest.raises(ValueError):
        Dist(10, 1, 11, 20)


def test_fit_from_measurements_envelopes_every_measured_nut():
    measured = list(CoconutPopulation(shell=Dist(4.0, 1.0, 1.5, 8.0)).stream(300, seed=5))
    pop = CoconutPopulation.from_measurements(measured)
    for c in measured:
        assert pop.shell.lo <= c.shell <= pop.shell.hi
        assert pop.diameter.lo <= c.diameter <= pop.diameter.hi
    assert pop.shell.mean == pytest.approx(4.0, abs=0.2)
    with pytest.raises(ValueError):
        CoconutPopulation.from_measurements(measured[:10])
