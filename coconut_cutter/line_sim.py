"""Discrete-event simulation of a multi-station opening line at plant scale.

Each station = one machine + one operator. Nuts are drawn from the
population model; every cycle's machine time and peak force come from the
*sized design*, so a design change shows up directly in throughput.

Losses modelled:
    * operator load / unload time (lognormal, per-operator skill spread)
    * crown dulling between swaps -> rising cut force -> stall/jam if it
      exceeds cylinder capacity
    * random jams (mis-seated nut, fibre wrap) + clear time
    * crown swap every ``blade_interval`` cuts
    * random machine breakdowns (exponential MTBF / MTTR)
    * staggered operator breaks
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from statistics import fmean, quantiles
from typing import List, Optional

from .coconut import CoconutPopulation
from .design import Design


@dataclass(frozen=True)
class LineParams:
    load_mean: float = 2.4        # s: pick nut from bin, seat in cup, press start
    unload_mean: float = 1.4      # s: lift opened nut to outfeed conveyor
    handling_cv: float = 0.30
    operator_skill_cv: float = 0.10
    jam_prob: float = 0.002       # per cycle, independent of force
    jam_clear_mean: float = 45.0  # s
    blade_swap_time: float = 360.0  # s, quick-change crown
    mtbf_h: float = 120.0         # operating hours between breakdowns
    mttr_h: float = 0.75
    break_minutes_per_shift: float = 30.0
    shift_h: float = 8.0


@dataclass
class StationResult:
    nuts: int = 0
    jams: int = 0
    stalls: int = 0
    swaps: int = 0
    breakdowns: int = 0
    busy_s: float = 0.0
    down_s: float = 0.0
    peak_force_max: float = 0.0


@dataclass
class LineResult:
    stations: int
    hours: float
    per_station: List[StationResult] = field(default_factory=list)

    @property
    def nuts(self) -> int:
        return sum(s.nuts for s in self.per_station)

    @property
    def rate_per_h(self) -> float:
        return self.nuts / self.hours

    @property
    def jams(self) -> int:
        return sum(s.jams for s in self.per_station)

    @property
    def stalls(self) -> int:
        return sum(s.stalls for s in self.per_station)

    @property
    def availability(self) -> float:
        tot = self.hours * 3600 * self.stations
        return 1.0 - sum(s.down_s for s in self.per_station) / tot


def _lognormal(rng: random.Random, mean: float, cv: float) -> float:
    sigma2 = math.log(1 + cv * cv)
    mu = math.log(mean) - sigma2 / 2
    return rng.lognormvariate(mu, math.sqrt(sigma2))


def simulate_station(design: Design, pop: CoconutPopulation, lp: LineParams,
                     hours: float, rng: random.Random, offset_frac: float) -> StationResult:
    r = StationResult()
    horizon = hours * 3600.0
    skill = max(0.6, rng.gauss(1.0, lp.operator_skill_cv))
    interval = design.req.blade_resharpen_cuts
    dull_end = design.req.blade_dull_at_swap
    cuts_on_blade = 0
    next_fail = rng.expovariate(1.0 / (lp.mtbf_h * 3600.0))
    run_time = 0.0  # operating seconds since last failure draw

    # Breaks: one block per shift, staggered across stations so the line
    # never loses every operator at once.
    shift = lp.shift_h * 3600.0
    brk = lp.break_minutes_per_shift * 60.0
    brk_start = (0.3 + 0.4 * offset_frac) * shift

    def in_break(t: float) -> Optional[float]:
        """End of the break containing ``t``, or None. Always > t (guarantees progress)."""
        end = math.floor(t / shift) * shift + brk_start + brk
        if end - brk <= t < end:
            return end
        return None

    t = 0.0
    while t < horizon:
        end = in_break(t)
        if end is not None:
            r.down_s += min(end, horizon) - t
            t = end
            continue
        if run_time >= next_fail:
            rep = rng.expovariate(1.0 / (lp.mttr_h * 3600.0))
            r.breakdowns += 1
            r.down_s += rep
            t += rep
            run_time = 0.0
            next_fail = rng.expovariate(1.0 / (lp.mtbf_h * 3600.0))
            continue
        if cuts_on_blade >= interval:
            r.swaps += 1
            r.down_s += lp.blade_swap_time
            t += lp.blade_swap_time
            cuts_on_blade = 0
            continue

        nut = pop.sample(rng)
        dull = 1.0 + (dull_end - 1.0) * cuts_on_blade / interval
        f = design.required_force(nut, dull)
        r.peak_force_max = max(r.peak_force_max, f)
        cycle = _lognormal(rng, lp.load_mean * skill, lp.handling_cv)
        mt = design.machine_time(nut)
        cycle += mt
        cycle += _lognormal(rng, lp.unload_mean * skill, lp.handling_cv)

        stalled = f > design.force_available
        jammed = not stalled and rng.random() < lp.jam_prob
        lost = _lognormal(rng, lp.jam_clear_mean, 0.5) if (stalled or jammed) else 0.0
        dt = cycle + lost
        if t + dt > horizon:
            break  # nut not finished inside the window: not counted
        if stalled:
            # Cylinder stalls on an out-of-spec nut; controller faults on extend
            # timeout; operator clears and rejects the nut (not counted as output).
            r.stalls += 1
        else:
            if jammed:
                r.jams += 1  # cleared and finished on the machine
            r.nuts += 1
            cuts_on_blade += 1
        r.down_s += lost
        r.busy_s += mt
        run_time += dt
        t += dt
    return r


def simulate_line(design: Design, stations: int, hours: float, seed: int = 0,
                  pop: Optional[CoconutPopulation] = None,
                  lp: LineParams = LineParams()) -> LineResult:
    pop = pop or design.pop
    res = LineResult(stations=stations, hours=hours)
    for k in range(stations):
        rng = random.Random((seed * 1_000_003) ^ (k * 7919 + 17))
        frac = k / max(1, stations - 1) if stations > 1 else 0.0
        res.per_station.append(simulate_station(design, pop, lp, hours, rng, frac))
    return res


@dataclass
class Capacity:
    stations: int
    p05_rate: float
    mean_rate: float
    rates: List[float]


def _rates(design: Design, n: int, hours: float, replications: int, lp: LineParams) -> List[float]:
    return [simulate_line(design, n, hours, seed=s, lp=lp).rate_per_h for s in range(replications)]


def _p05(rates: List[float]) -> float:
    return quantiles(rates, n=20, method="inclusive")[0]


def stations_needed(design: Design, target_rate: float, hours: float = 16.0,
                    replications: int = 20, max_stations: int = 40,
                    lp: LineParams = LineParams()) -> Capacity:
    """Smallest station count whose 5th-percentile day still meets ``target_rate``.

    Stations are independent, so the line rate is roughly n × the single-station
    rate; that gives a starting guess. The search then walks down while n-1
    still passes and up while n fails, so the answer is exact either way.
    """
    one = fmean(_rates(design, 1, hours, 4, lp))
    n = min(max_stations, max(1, math.ceil(target_rate / one)))
    rates = _rates(design, n, hours, replications, lp)
    if _p05(rates) >= target_rate:
        while n > 1:
            lower = _rates(design, n - 1, hours, replications, lp)
            if _p05(lower) < target_rate:
                break
            n, rates = n - 1, lower
        return Capacity(n, _p05(rates), fmean(rates), rates)
    while n < max_stations:
        n += 1
        rates = _rates(design, n, hours, replications, lp)
        if _p05(rates) >= target_rate:
            return Capacity(n, _p05(rates), fmean(rates), rates)
    raise RuntimeError(f"target {target_rate}/h not reachable with {max_stations} stations")
