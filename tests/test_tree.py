import random
from statistics import fmean

import pytest
from hypothesis import given, strategies as st

from coconut_harvester.tree import RAIN_PROB, Dist, TreePopulation, Weather


def test_samples_within_bounds_and_reproducible():
    pop = TreePopulation()
    a = list(pop.stream(20_000, seed=1))
    assert a == list(pop.stream(20_000, seed=1))
    for t in a:
        assert pop.height.lo <= t.height <= pop.height.hi
        assert pop.d_min <= t.d_top <= t.d_base <= pop.d_max
        assert 0 <= t.lean <= pop.lean.hi


@given(st.floats(0, 30))
def test_diameter_tapers_monotonically(z):
    t = TreePopulation().sample(random.Random(4))
    assert t.d_top <= t.diameter_at(z) <= t.d_base
    assert t.diameter_at(z) >= t.diameter_at(z + 1.0)


def test_bunch_count_matches_yield():
    pop, rng = TreePopulation(), random.Random(2)
    counts = [len(pop.bunches(rng, 45)) for _ in range(50_000)]
    assert fmean(counts) == pytest.approx(1.5, rel=0.03)
    stalks = [p for _ in range(5000) for p in pop.bunches(rng, 45)]
    assert all(pop.peduncle.lo <= p <= pop.peduncle.hi for p in stalks)


def test_survey_fit_envelopes_every_measured_tree():
    measured = list(TreePopulation(ridge_h=Dist(7, 2, 2, 14)).stream(200, seed=5))
    fit = TreePopulation.from_survey(measured)
    for t in measured:
        assert fit.ridge_h.lo <= t.ridge_h <= fit.ridge_h.hi
        assert fit.height.lo <= t.height <= fit.height.hi
    with pytest.raises(ValueError):
        TreePopulation.from_survey(measured[:10])


def test_monsoon_months_lose_more_hours():
    w, rng = Weather(), random.Random(1)
    jan = fmean(w.productive_hours(10, 7, rng) for _ in range(20_000))
    jul = fmean(w.productive_hours(190, 7, rng) for _ in range(20_000))
    assert jan > 6.5 and jul < 3.5
    assert len(RAIN_PROB) == 12
