"""Statistical model of the incoming tender-coconut population.

All numbers are *design assumptions* for trimmed ("polished") green tender
coconuts as sold in South-Asian markets. They are deliberately conservative
(wide spreads, heavy upper tails) and are meant to be replaced by measured
intake data — see ``CoconutPopulation.from_measurements``.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from statistics import fmean, pstdev
from typing import Iterable, Iterator, Sequence


@dataclass(frozen=True)
class Coconut:
    """One trimmed tender coconut, dimensions in mm, mass in kg."""

    diameter: float          # equatorial diameter of trimmed nut
    height: float            # base to tip of trimmed cone
    husk_top: float          # husk thickness remaining over the eye end
    shell: float             # tender shell thickness at the eye end
    strength: float          # multiplier on nominal cutting resistance (1.0 = nominal)
    mass: float


@dataclass(frozen=True)
class Dist:
    """Truncated normal distribution: mean, sd, hard [lo, hi] bounds."""

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
        # Rejection sampling keeps the shape inside the bounds (no pile-up at limits).
        for _ in range(1000):
            x = rng.gauss(self.mean, self.sd)
            if self.lo <= x <= self.hi:
                return x
        return min(max(self.mean, self.lo), self.hi)


@dataclass(frozen=True)
class CoconutPopulation:
    diameter: Dist = Dist(165.0, 14.0, 130.0, 210.0)
    height: Dist = Dist(205.0, 18.0, 160.0, 260.0)
    husk_top: Dist = Dist(14.0, 4.0, 5.0, 28.0)
    shell: Dist = Dist(3.0, 0.8, 1.2, 6.0)
    strength: Dist = Dist(1.0, 0.18, 0.6, 1.6)
    density: float = 0.95e-6  # kg/mm^3, whole tender nut (husk + water)
    # Trimmed nuts are a cylinder-and-cone, not an ellipsoid: volume = SHAPE·(π/6)·d²·h.
    shape: float = 0.60

    def _mass(self, d: float, h: float) -> float:
        return self.shape * math.pi / 6.0 * d * d * h * self.density

    def sample(self, rng: random.Random) -> Coconut:
        d = self.diameter.sample(rng)
        h = self.height.sample(rng)
        return Coconut(
            diameter=d,
            height=h,
            husk_top=self.husk_top.sample(rng),
            shell=self.shell.sample(rng),
            strength=self.strength.sample(rng),
            mass=self._mass(d, h),
        )

    def stream(self, n: int, seed: int = 0) -> Iterator[Coconut]:
        rng = random.Random(seed)
        for _ in range(n):
            yield self.sample(rng)

    def worst_case(self) -> Coconut:
        """Envelope nut: every parameter at its bound in the adverse direction."""
        d, h = self.diameter.hi, self.height.hi
        return Coconut(
            diameter=d,
            height=h,
            husk_top=self.husk_top.hi,
            shell=self.shell.hi,
            strength=self.strength.hi,
            mass=self._mass(d, h),
        )

    @classmethod
    def from_measurements(cls, nuts: Sequence[Coconut], margin_sd: float = 4.0) -> "CoconutPopulation":
        """Fit a population to measured intake samples.

        Bounds are set at ``mean ± margin_sd·sd`` but never tighter than the
        observed extremes, so the design always envelopes real data.
        """
        if len(nuts) < 30:
            raise ValueError("need at least 30 measured coconuts for a meaningful fit")

        def fit(values: Iterable[float], floor: float) -> Dist:
            v = list(values)
            m, s = fmean(v), pstdev(v)
            lo = max(floor, min(min(v), m - margin_sd * s))
            hi = max(max(v), m + margin_sd * s)
            return Dist(m, s, lo, hi)

        base = cls()
        return cls(
            diameter=fit((c.diameter for c in nuts), 1.0),
            height=fit((c.height for c in nuts), 1.0),
            husk_top=fit((c.husk_top for c in nuts), 0.0),
            shell=fit((c.shell for c in nuts), 0.1),
            strength=fit((c.strength for c in nuts), 0.05),
            density=base.density,
            shape=base.shape,
        )
