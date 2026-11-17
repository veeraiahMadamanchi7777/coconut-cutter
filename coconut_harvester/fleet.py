"""Plantation-scale fleet simulation: a year of harvest rounds with real weather.

Every palm must be harvested every ``cycle_days``. Each robot works with a
two-person crew (operator + collector), moves tree to tree on a trolley,
swaps battery packs (charged on a field charger), breaks down now and then,
and cannot climb on wet bark. Palms outside the robot's envelope go to a
manual climber and are counted.
"""
from __future__ import annotations

import heapq
import math
import os
import random
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass, field
from statistics import fmean, quantiles
from typing import List, Optional

from .design import Design
from .tree import Tree, TreePopulation, Weather


@dataclass(frozen=True)
class Plantation:
    hectares: float = 100.0
    palms_per_ha: float = 175.0      # 7.5-8 m triangular spacing
    spacing_m: float = 8.0
    cycle_days: int = 45             # harvest round
    early_days: int = 3              # may harvest up to 3 days before due
    workdays: tuple = (0, 1, 2, 3, 4, 5)  # Mon-Sat
    shift_h: float = 7.0             # productive hours on a dry day

    @property
    def palms(self) -> int:
        return int(self.hectares * self.palms_per_ha)


@dataclass(frozen=True)
class Ops:
    trolley_speed: float = 0.7       # m/s between trees
    mount_s: float = 100.0           # lift onto trunk, close & latch ring
    dismount_s: float = 45.0
    handling_cv: float = 0.30
    aim_s: float = 30.0              # find & line up each bunch on the camera
    aim_cv: float = 0.5
    arm_s: float = 6.0               # deploy + stow
    swap_s: float = 120.0
    packs_per_robot: int = 3
    charge_h: float = 2.5
    mtbf_h: float = 200.0
    mttr_h: float = 2.0


@dataclass
class YearResult:
    robots: int
    harvests: int = 0
    manual: int = 0
    bunches: int = 0
    lateness: List[float] = field(default_factory=list)
    robot_hours: float = 0.0
    climb_hours: float = 0.0
    pack_waits: int = 0
    breakdowns: int = 0
    energy_kwh: float = 0.0
    lost_wet_hours: float = 0.0

    @property
    def p95_late(self) -> float:
        return quantiles(self.lateness, n=20, method="inclusive")[-1] if len(self.lateness) > 1 else 0.0

    @property
    def max_late(self) -> float:
        return max(self.lateness, default=0.0)

    @property
    def trees_per_robot_day(self) -> float:
        return self.harvests / max(1e-9, self.robot_hours) * 7.0


def _lnp(mean: float, cv: float):
    s2 = math.log(1 + cv * cv)
    return math.log(mean) - s2 / 2, math.sqrt(s2)


def _ln(rng: random.Random, mean: float, cv: float) -> float:
    return rng.lognormvariate(*_lnp(mean, cv))


def tree_visit(d: Design, tree: Tree, peduncles: List[float], ops: Ops, pl: Plantation,
               rng: random.Random) -> float:
    """Seconds for one complete visit (walk-in to walk-off), excluding swaps."""
    t = pl.spacing_m / ops.trolley_speed
    t += _ln(rng, ops.mount_s, ops.handling_cv) + _ln(rng, ops.dismount_s, ops.handling_cv)
    t += 2 * tree.height / d.speed * 1.08 + 6.0  # ridge slow-downs, clamp/release
    if peduncles:
        t += ops.arm_s + sum(_ln(rng, ops.aim_s, ops.aim_cv) + d.cut_time(p) + 1.0 for p in peduncles)
    return t


def simulate_year(d: Design, robots: int, pl: Plantation = Plantation(), ops: Ops = Ops(),
                  weather: Weather = Weather(), seed: int = 0, days: int = 365,
                  pop: Optional[TreePopulation] = None) -> YearResult:
    rng = random.Random(seed)
    pop = pop or d.pop
    trees = [pop.sample(rng) for _ in range(pl.palms)]
    climbable = [d.can_climb(t, wet=False) for t in trees]
    # Staggered start: palms are spread evenly over one round.
    due = [(i * pl.cycle_days / pl.palms, i) for i in range(pl.palms)]
    last = [dd - pl.cycle_days for dd, _ in due]
    heapq.heapify(due)
    res = YearResult(robots=robots)
    pack_wh = d.battery_wh * d.req.battery_dod
    reserve = d.descent_reserve(pop.height.hi)

    # Hot-loop constants (equivalent to Design.energy_tree / tree_visit; see test_fleet).
    mount_p, dism_p, aim_p = (_lnp(ops.mount_s, ops.handling_cv), _lnp(ops.dismount_s, ops.handling_cv),
                              _lnp(ops.aim_s, ops.aim_cv))
    walk = pl.spacing_m / ops.trolley_speed
    elec = d.req.electronics_w / 3600.0
    base_e = [d.energy_climb(t.height) + d.energy_descend(t.height) + elec * 2 * t.height / d.speed
              for t in trees]
    base_t = [walk + 2 * t.height / d.speed * 1.08 + 6.0 for t in trees]
    per_bunch_e = 0.06 + elec * ops.aim_s
    lnv = rng.lognormvariate

    # per-robot state
    charge = [pack_wh] * robots
    spare_ready = [[0.0] * (ops.packs_per_robot - 1) for _ in range(robots)]  # time packs are charged
    run_h = [0.0] * robots
    next_fail = [rng.expovariate(1 / ops.mtbf_h) for _ in range(robots)]

    for day in range(days):
        if day % 7 not in pl.workdays:
            continue
        hours = weather.productive_hours(day, pl.shift_h, rng)
        res.lost_wet_hours += (pl.shift_h - hours) * robots
        for r in range(robots):
            t = 0.0
            horizon = hours * 3600
            # packs left on charge overnight are full by morning
            spare_ready[r] = [0.0 for _ in spare_ready[r]]
            charge[r] = pack_wh
            while t < horizon and due and due[0][0] <= day + pl.early_days:
                if run_h[r] >= next_fail[r]:
                    rep = _ln(rng, ops.mttr_h * 3600, 0.5)
                    res.breakdowns += 1
                    t += rep
                    run_h[r] = 0.0
                    next_fail[r] = rng.expovariate(1 / ops.mtbf_h)
                    continue
                dd, i = due[0]
                tree = trees[i]
                if not climbable[i]:
                    heapq.heappop(due)
                    res.manual += 1
                    last[i] = day
                    heapq.heappush(due, (day + pl.cycle_days, i))
                    continue
                peds = pop.bunches(rng, day - last[i])
                cts = [d.cut_time(p) for p in peds]
                e = base_e[i] + sum(d.saw_w * (ct + 3.0) / 3600.0 + per_bunch_e for ct in cts)
                if charge[r] - e < reserve:
                    recharge = ops.charge_h * 3600 * (1 - max(0.0, charge[r]) / pack_wh)
                    if not spare_ready[r]:
                        # single pack: the robot waits while its own pack recharges
                        res.pack_waits += 1
                        t += 2 * ops.swap_s + recharge
                    else:
                        # swap in the soonest-charged spare; the flat pack goes on charge
                        k = min(range(len(spare_ready[r])), key=lambda j: spare_ready[r][j])
                        wait = max(0.0, spare_ready[r][k] - t)
                        if wait > 0:
                            res.pack_waits += 1
                        t += wait + ops.swap_s
                        spare_ready[r][k] = t + recharge
                    charge[r] = pack_wh
                    continue
                dt = base_t[i] + lnv(*mount_p) + lnv(*dism_p)
                if cts:
                    dt += ops.arm_s + sum(lnv(*aim_p) + ct + 1.0 for ct in cts)
                if t + dt > horizon:
                    break
                heapq.heappop(due)
                t += dt
                charge[r] -= e
                run_h[r] += dt / 3600
                res.energy_kwh += e / 1000
                res.climb_hours += 2 * tree.height / d.speed / 3600
                res.harvests += 1
                res.bunches += len(peds)
                res.lateness.append(max(0.0, day - dd))
                last[i] = day
                heapq.heappush(due, (day + pl.cycle_days, i))
            res.robot_hours += min(t, horizon) / 3600
    # palms still overdue at year end count as late too
    end = days - 1
    res.lateness.extend(end - dd for dd, _ in due if dd < end)
    return res


@dataclass(frozen=True)
class ServiceLevel:
    """Harvest-round service target for the fleet."""

    p95_late_max: float = 3.0    # days: 95 % of visits within 3 days of due
    max_late_max: float = 14.0   # days: no palm more than 2 weeks late ...
    year_pass_frac: float = 0.9  # ... in at least 9 of 10 weather years
    hard_max_late: float = 21.0  # and never more than 3 weeks late in any year

    def year_ok(self, y: YearResult) -> bool:
        return y.p95_late <= self.p95_late_max and y.max_late <= self.max_late_max


@dataclass
class FleetSize:
    robots: int
    results: List[YearResult]
    service: ServiceLevel = ServiceLevel()

    @property
    def pass_frac(self) -> float:
        return sum(self.service.year_ok(y) for y in self.results) / len(self.results)

    complete: bool = True

    def _failed(self) -> "FleetSize":
        self.complete = False
        return self

    @property
    def ok(self) -> bool:
        return (self.complete and self.pass_frac >= self.service.year_pass_frac
                and self.worst_max <= self.service.hard_max_late)

    @property
    def worst_p95(self) -> float:
        return max(r.p95_late for r in self.results)

    @property
    def worst_max(self) -> float:
        return max(r.max_late for r in self.results)


def _year(args) -> YearResult:
    d, n, pl, ops, weather, seed = args
    return simulate_year(d, n, pl, ops, weather, seed=seed)


def annual_capacity_per_robot(d: Design, pl: Plantation, ops: Ops, weather: Weather, seed: int = 99) -> float:
    """Harvests one robot completes in a year when it never runs out of due palms."""
    overloaded = Plantation(hectares=30, palms_per_ha=pl.palms_per_ha, spacing_m=pl.spacing_m,
                            cycle_days=pl.cycle_days, early_days=pl.cycle_days, workdays=pl.workdays,
                            shift_h=pl.shift_h)
    return simulate_year(d, 1, overloaded, ops, weather, seed=seed).harvests


def fleet_needed(d: Design, pl: Plantation = Plantation(), ops: Ops = Ops(), weather: Weather = Weather(),
                 service: ServiceLevel = ServiceLevel(), years: int = 30, max_robots: int = 400) -> FleetSize:
    """Smallest fleet meeting ``service`` over ``years`` simulated weather years.

    Ten years under-samples monsoon variability (a 10-year fit passed only ~65 %
    of fresh years), so 30 are used; a candidate is abandoned as soon as it has
    failed more years than the service level allows.
    """
    allowed_fails = math.floor((1 - service.year_pass_frac) * years + 1e-9)

    workers = max(1, min(os.cpu_count() or 1, 8))

    def trial(n: int) -> FleetSize:
        ys: List[YearResult] = []
        fails = 0
        with ProcessPoolExecutor(workers) as pool:
            for b in range(0, years, workers):
                seeds = range(b, min(years, b + workers))
                batch = list(pool.map(_year, [(d, n, pl, ops, weather, s) for s in seeds]))
                ys.extend(batch)
                fails += sum(not service.year_ok(y) for y in batch)
                if fails > allowed_fails or any(y.max_late > service.hard_max_late for y in batch):
                    return FleetSize(n, ys, service)._failed()
        return FleetSize(n, ys, service)

    need = pl.palms * 365 / pl.cycle_days
    n = max(1, min(max_robots, math.ceil(need / annual_capacity_per_robot(d, pl, ops, weather))))
    best = trial(n)
    if best.ok:
        while n > 1:
            lower = trial(n - 1)
            if not lower.ok:
                break
            best, n = lower, n - 1
        return best
    while n < max_robots:
        n += 1
        best = trial(n)
        if best.ok:
            return best
    raise RuntimeError("fleet target not reachable")
