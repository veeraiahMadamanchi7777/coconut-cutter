"""Coconut-palm population and weather model.

Values are design assumptions for tall-variety coconut palms in South-Indian
plantations (Kerala / Tamil Nadu / Karnataka). Replace them with a field
survey via ``TreePopulation.from_survey`` before release.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from statistics import fmean, pstdev
from typing import Iterable, List, Sequence


@dataclass(frozen=True)
class Dist:
    """Truncated normal: mean, sd, hard bounds [lo, hi]."""

    mean: float
    sd: float
    lo: float
    hi: float

    def __post_init__(self) -> None:
        if not (self.lo <= self.mean <= self.hi) or self.sd < 0:
            raise ValueError(f"invalid distribution {self}")

    def sample(self, rng: random.Random) -> float:
        if self.sd == 0:
            return self.mean
        for _ in range(1000):
            x = rng.gauss(self.mean, self.sd)
            if self.lo <= x <= self.hi:
                return x
        return self.mean


@dataclass(frozen=True)
class Tree:
    height: float        # m, ground to crown base (where the robot stops)
    d_base: float        # mm, trunk diameter at the mounting height (~1 m)
    d_top: float         # mm, trunk diameter under the crown
    lean: float          # deg from vertical
    ridge_h: float       # mm, tallest leaf-scar ridge on this trunk
    ridge_len: float     # mm, axial length of that ridge (gentler if longer)
    mu_dry: float        # wheel/bark friction, dry
    bunch_radius: float  # m, trunk axis to peduncle cut point

    def diameter_at(self, z: float) -> float:
        """Trunk diameter [mm] at height z [m] (linear taper above 1 m)."""
        if z <= 1.0 or self.height <= 1.0:
            return self.d_base
        f = min(1.0, (z - 1.0) / (self.height - 1.0))
        return self.d_base + (self.d_top - self.d_base) * f


@dataclass(frozen=True)
class TreePopulation:
    height: Dist = Dist(12.0, 4.0, 4.0, 22.0)
    d_base: Dist = Dist(310.0, 35.0, 230.0, 420.0)
    taper: Dist = Dist(0.80, 0.07, 0.62, 0.98)       # d_top / d_base
    lean: Dist = Dist(5.0, 4.0, 0.0, 20.0)
    ridge_h: Dist = Dist(5.0, 2.0, 1.0, 10.0)        # leaf-scar ridge height
    ridge_len: Dist = Dist(50.0, 12.0, 30.0, 90.0)   # axial length of the ridge profile
    mu_dry: Dist = Dist(0.70, 0.07, 0.50, 0.90)       # PU tread on dry bark
    wet_factor: float = 0.60                           # mu_wet = mu_dry * wet_factor
    bunch_radius: Dist = Dist(0.45, 0.08, 0.30, 0.70)
    peduncle: Dist = Dist(50.0, 8.0, 35.0, 75.0)       # mm, bunch stalk diameter
    bunch_mass: Dist = Dist(12.0, 4.0, 5.0, 25.0)      # kg
    bunches_per_month: float = 1.0                     # one new bunch ~ every month

    def sample(self, rng: random.Random) -> Tree:
        d0 = self.d_base.sample(rng)
        return Tree(
            height=self.height.sample(rng),
            d_base=d0,
            d_top=d0 * self.taper.sample(rng),
            lean=self.lean.sample(rng),
            ridge_h=self.ridge_h.sample(rng),
            ridge_len=self.ridge_len.sample(rng),
            mu_dry=self.mu_dry.sample(rng),
            bunch_radius=self.bunch_radius.sample(rng),
        )

    def stream(self, n: int, seed: int = 0):
        rng = random.Random(seed)
        for _ in range(n):
            yield self.sample(rng)

    def bunches(self, rng: random.Random, days_since_harvest: float) -> List[float]:
        """Peduncle diameters [mm] of bunches ready on one visit (Poisson count)."""
        lam = self.bunches_per_month * days_since_harvest / 30.0
        # Knuth Poisson sampler (lam is small)
        L, k, p = math.exp(-lam), 0, 1.0
        while True:
            p *= rng.random()
            if p <= L:
                break
            k += 1
        return [self.peduncle.sample(rng) for _ in range(k)]

    @property
    def d_min(self) -> float:
        return self.d_base.lo * self.taper.lo

    @property
    def d_max(self) -> float:
        return self.d_base.hi

    @classmethod
    def from_survey(cls, trees: Sequence[Tree], margin_sd: float = 3.0) -> "TreePopulation":
        """Fit to surveyed trees; bounds never tighter than the observed extremes."""
        if len(trees) < 50:
            raise ValueError("survey at least 50 palms")

        def fit(vals: Iterable[float], floor: float = 0.0) -> Dist:
            v = list(vals)
            m, s = fmean(v), pstdev(v)
            return Dist(m, s, max(floor, min(min(v), m - margin_sd * s)), max(max(v), m + margin_sd * s))

        base = cls()
        return cls(
            height=fit((t.height for t in trees), 1.0),
            d_base=fit((t.d_base for t in trees), 50.0),
            taper=fit((t.d_top / t.d_base for t in trees), 0.3),
            lean=fit((t.lean for t in trees), 0.0),
            ridge_h=fit((t.ridge_h for t in trees), 0.0),
            ridge_len=fit((t.ridge_len for t in trees), 5.0),
            mu_dry=fit((t.mu_dry for t in trees), 0.05),
            wet_factor=base.wet_factor,
            bunch_radius=fit((t.bunch_radius for t in trees), 0.1),
            peduncle=base.peduncle,
            bunch_mass=base.bunch_mass,
            bunches_per_month=base.bunches_per_month,
        )


# Probability that a given day has wet bark in the morning, by month (Kerala-like
# south-west + north-east monsoon). Index 0 = January.
RAIN_PROB = [0.05, 0.05, 0.10, 0.25, 0.40, 0.75, 0.75, 0.60, 0.45, 0.45, 0.30, 0.10]


@dataclass(frozen=True)
class Weather:
    rain_prob: Sequence[float] = tuple(RAIN_PROB)
    wet_day_dry_hours: float = 3.0   # productive hours left on a wet day (afternoon dry-out)

    def productive_hours(self, day_of_year: int, shift_h: float, rng: random.Random) -> float:
        month = min(11, int(day_of_year / 365.0 * 12))
        if rng.random() < self.rain_prob[month]:
            return min(shift_h, self.wet_day_dry_hours * rng.random())
        return shift_h
