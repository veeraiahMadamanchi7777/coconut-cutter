"""Parametric sizing of a ring-type coconut-palm climbing and harvesting robot.

Architecture (one robot):

    * Two clamp **tiers** (upper and lower rings) joined by three columns,
      spaced ``H`` apart. Each ring is hinged in two halves so it can be
      closed around the trunk at the foot of the tree.
    * Each tier has 3 swing-arm **wheel modules** at 120°, pressed radially
      onto the bark by a self-locking lead-screw actuator through series
      springs (the springs absorb leaf-scar ridges, the actuator follows the
      trunk taper under load-cell force control).
    * Every wheel is driven by a 24 V **worm gearmotor** (η < 0.5, so it is
      self-locking: loss of power cannot drop the robot).
    * A slewing **harvest arm** on the top ring carries a BLDC **circular saw**
      that cuts bunch stalks (peduncles); the bunch falls to the ground.
    * A ground **belay tether** is a fall arrest of last resort.
    * Pole camera and radio link to an operator handset.

Every function is pure; ``optimize`` searches catalogue combinations and
returns the lightest design that passes every check.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from typing import Dict, List, Optional, Sequence, Tuple

from .tree import Tree, TreePopulation

G = 9.81
N_PER_TIER = 3
TIERS = 2
N_WHEELS = N_PER_TIER * TIERS

# ---- catalogues -----------------------------------------------------------
# PU-tread V-profile wheels: diameter mm, width mm, load rating N, mass kg
WHEELS = [(100, 40, 1200, 0.35), (125, 45, 1800, 0.50), (150, 50, 2500, 0.70), (200, 60, 3500, 1.10)]
# 24 V worm gearmotors: name, rated torque Nm, rated output rpm, mass kg, efficiency
MOTORS = [("WG-10", 10, 60, 1.3, 0.45), ("WG-20", 20, 50, 1.9, 0.42), ("WG-35", 35, 40, 2.6, 0.40),
          ("WG-50", 50, 32, 3.4, 0.38), ("WG-80", 80, 25, 4.6, 0.35)]
# Self-locking lead-screw clamp actuators: rated force N, mass kg
CLAMP_ACTUATORS = [(1000, 1.0), (2500, 1.6), (4000, 2.3), (6000, 3.2)]
# BLDC saw motors: rated W, mass kg
SAW_MOTORS = [(150, 0.8), (250, 1.1), (400, 1.6), (600, 2.2)]
BLADES = [150, 185, 210, 235, 255]  # mm, TCT blades
# 24 V (7S) Li-ion packs, Ah
BATTERIES = [10, 15, 20, 30, 40]
V_PACK = 25.2
TIER_SPACINGS = [0.4, 0.5, 0.6, 0.7, 0.8, 1.0]  # m


@dataclass(frozen=True)
class Requirements:
    climb_speed: float = 0.25        # m/s minimum under load
    sf_climb: float = 1.3            # traction margin, dry bark, worst ridge
    sf_hold_wet: float = 1.5         # static hold margin on wet bark (stopped, rain arrives)
    c_rr: float = 0.03               # rolling resistance coefficient, PU on bark
    min_wheel_load_frac: float = 0.3  # no wheel may unload below this fraction of preload
    max_mass: float = 40.0           # kg, two-person carry between trees
    max_cut_time: float = 10.0       # s, thickest peduncle
    cut_energy: float = 80.0         # J/cm^3, green fibrous peduncle, TCT blade (conservative)
    kerf: float = 2.4                # mm
    feed_force: float = 60.0         # N, operator-controlled feed of saw into stalk
    min_trees_per_charge: float = 20.0
    battery_dod: float = 0.80
    descent_reserve_sf: float = 2.0  # always keep 2× the energy to come down
    electronics_w: float = 25.0      # compute, radio, camera, IMU, load cells
    eta_elec: float = 0.85           # motor + driver electrical efficiency
    wind_max: float = 10.0           # m/s, operations limit
    clamp_margin: float = 15.0       # mm extra radial travel
    tread_compliance: float = 3.0    # mm of ridge absorbed by the 20 mm PU tread under load
    tether_mbs: float = 15_000.0     # N, belay rope minimum breaking strength
    tether_sf: float = 10.0


@dataclass
class Check:
    name: str
    value: float
    limit: float
    unit: str
    ok: bool
    cmp: str = "≥"


@dataclass
class Design:
    req: Requirements
    pop: TreePopulation
    wheel: Tuple[int, int, int, float]
    motor: Tuple[str, int, int, float, float]
    H: float
    clamp_actuator: Tuple[int, float] = (0, 0.0)
    saw_motor: Tuple[int, float] = (0, 0.0)
    blade: int = 0
    battery_ah: int = 0
    mass: float = 0.0
    N0: float = 0.0              # N, clamp preload per wheel
    s_ridge: float = 0.0         # worst ridge slope the design climbs
    reach: float = 0.0           # m, trunk axis to saw
    spring_rate: float = 0.0     # N/mm per wheel
    clamp_travel: float = 0.0    # mm radial
    ring_inner_d: float = 0.0    # mm, ring clear opening
    mass_breakdown: Dict[str, float] = field(default_factory=dict)
    checks: List[Check] = field(default_factory=list)
    converged: bool = False

    # ---- derived ---------------------------------------------------------
    @property
    def ok(self) -> bool:
        return self.converged and all(c.ok for c in self.checks)

    def failed(self) -> List[Check]:
        return [c for c in self.checks if not c.ok]

    @property
    def weight(self) -> float:
        return self.mass * G

    @property
    def r_wheel(self) -> float:
        return self.wheel[0] / 2000.0  # m

    @property
    def speed(self) -> float:
        """Climb/descent speed at rated motor speed [m/s]."""
        return self.motor[2] / 60.0 * 2 * math.pi * self.r_wheel

    @property
    def eta_worm(self) -> float:
        return self.motor[4]

    @property
    def battery_wh(self) -> float:
        return self.battery_ah * V_PACK

    @property
    def saw_w(self) -> float:
        return float(self.saw_motor[0])

    # ---- per-tree physics (used by controller plant, sim and tests) -------
    def ridge_slope(self, h: float, length: float) -> float:
        return ridge_slope(self.r_wheel * 1000.0, h - self.req.tread_compliance, length)

    def traction_demand(self, tree: Tree, ridge: bool = True) -> float:
        """Axial force [N] the wheels must transmit while climbing ``tree``."""
        W = self.weight * math.cos(math.radians(tree.lean))
        s = self.ridge_slope(tree.ridge_h, tree.ridge_len) if ridge else 0.0
        return W + self.req.c_rr * N_WHEELS * self.N0 + N_PER_TIER * self.N0 * s

    def traction_available(self, mu: float) -> float:
        return mu * N_WHEELS * self.N0

    def can_climb(self, tree: Tree, wet: bool, margin: float = 1.1) -> bool:
        mu = tree.mu_dry * (self.pop.wet_factor if wet else 1.0)
        in_range = self.clamps(tree)
        return in_range and self.traction_available(mu) >= margin * self.traction_demand(tree)

    def can_hold(self, tree: Tree, wet: bool) -> bool:
        mu = tree.mu_dry * (self.pop.wet_factor if wet else 1.0)
        return self.traction_available(mu) >= self.weight

    def clamps(self, tree: Tree) -> bool:
        lo = self.ring_inner_d - 2 * self.clamp_travel
        return lo <= min(tree.d_base, tree.d_top) and max(tree.d_base, tree.d_top) <= self.ring_inner_d

    def cut_time(self, peduncle_mm: float) -> float:
        vol_cm3 = self.req.kerf * math.pi * peduncle_mm ** 2 / 4 / 1000.0
        p_cut = self.saw_w * 0.80 - 0.20 * self.saw_w  # 80 % efficiency minus idle losses
        return self.req.cut_energy * vol_cm3 / p_cut

    def energy_climb(self, height: float) -> float:
        """Wh to climb ``height`` m (worm efficiency, rolling resistance)."""
        F = self.weight + self.req.c_rr * N_WHEELS * self.N0
        return F * height / (self.eta_worm * self.req.eta_elec) / 3600.0

    def energy_descend(self, height: float) -> float:
        """Self-locking worm must be *driven* down: P_in = P_load(1/η - 2)."""
        back = max(0.0, 1.0 / self.eta_worm - 2.0)
        F = self.weight * back + self.req.c_rr * N_WHEELS * self.N0 / self.eta_worm
        return F * height / self.req.eta_elec / 3600.0

    def energy_tree(self, tree: Tree, peduncles: Sequence[float], dwell_s: float = 0.0) -> float:
        """Wh for one complete tree visit."""
        e = self.energy_climb(tree.height) + self.energy_descend(tree.height)
        e += sum(self.saw_w * (self.cut_time(p) + 3.0) / 3600.0 for p in peduncles)
        e += 0.06 * len(peduncles)  # slew + boom moves
        t = 2 * tree.height / self.speed + dwell_s
        e += self.req.electronics_w * t / 3600.0
        return e

    def descent_reserve(self, height: float) -> float:
        return self.req.descent_reserve_sf * (self.energy_descend(height)
                                              + self.req.electronics_w * height / self.speed / 3600)


def ridge_slope(r_mm: float, h: float, length: float) -> float:
    """Peak slope the wheel centre sees crossing a leaf-scar ridge.

    The smaller of a sharp step (wheel geometry) and a smooth half-cosine bump
    of height ``h`` over axial ``length`` (slope πh/L).
    """
    if h <= 0:
        return 0.0
    h = min(h, 0.9 * r_mm)
    step = math.sqrt(2 * r_mm * h - h * h) / (r_mm - h)
    bump = math.pi * h / length
    return min(step, bump)


def _pick(options, pred):
    for o in options:
        if pred(o):
            return o
    return None


def size(req: Requirements, pop: TreePopulation, wheel, motor_hint=None, H: float = 0.6,
         max_iter: int = 60) -> Design:
    """Size one configuration (wheel, tier spacing) to a converged mass."""
    d = Design(req=req, pop=pop, wheel=wheel, motor=motor_hint or MOTORS[0], H=H)
    d.reach = pop.bunch_radius.hi
    d.s_ridge = ridge_slope(wheel[0] / 2, pop.ridge_h.hi - req.tread_compliance, pop.ridge_len.lo)
    mu_c = pop.mu_dry.lo
    mu_w = pop.mu_dry.lo * pop.wet_factor
    lean = math.radians(pop.lean.hi)
    n, nn = N_PER_TIER, N_WHEELS

    # --- saw: blade must cut the thickest stalk in one pass; motor meets cut time
    flange_r, clear = 25.0, 10.0
    d.blade = _pick(BLADES, lambda b: b / 2 - flange_r - clear >= pop.peduncle.hi) or BLADES[-1]
    d.saw_motor = SAW_MOTORS[-1]
    for sm in SAW_MOTORS:
        d.saw_motor = sm
        if d.cut_time(pop.peduncle.hi) <= req.max_cut_time:
            break

    mass = 25.0
    for _ in range(max_iter):
        d.mass = mass
        W = mass * G
        # Tipping couple from the arm at full reach + body CG offset + wind on the crown arm.
        m_saw_unit = d.saw_motor[1] + 0.1 + (d.blade / 1000) ** 2 * 6 + 0.6  # motor, blade, guard
        m_boom = 1.5 * d.reach
        wind = 0.5 * 1.2 * req.wind_max ** 2 * 1.2 * 0.25
        M = G * (m_saw_unit * d.reach + m_boom * d.reach / 2 + 0.05 * mass) \
            + req.feed_force * d.reach + wind * (H + 0.3)
        F_tier = M / H + W * math.sin(lean) / 2
        dN = 2 * F_tier / n

        # --- clamp preload: max of climb traction, wet hold, and no-unload
        denom = nn * mu_c - req.sf_climb * (nn * req.c_rr + n * d.s_ridge)
        N_climb = req.sf_climb * W / denom if denom > 0 else float("inf")
        N_hold = req.sf_hold_wet * W / (nn * mu_w)
        N_unload = dN / (1 - req.min_wheel_load_frac)
        d.N0 = math.ceil(max(N_climb, N_hold, N_unload) / 10) * 10 if denom > 0 else float("inf")
        if not math.isfinite(d.N0):
            break

        # --- drive motor: torque on worst ridge, speed at rated rpm
        F_drive = req.sf_climb * (W + req.c_rr * nn * d.N0 + n * d.N0 * d.s_ridge)
        T_req = F_drive / nn * d.r_wheel
        d.motor = _pick(MOTORS, lambda m: m[1] >= T_req
                        and m[2] / 60 * 2 * math.pi * d.r_wheel >= req.climb_speed) or MOTORS[-1]

        # --- clamp actuator (2:1 linkage per tier), series spring, travel
        d.clamp_actuator = _pick(CLAMP_ACTUATORS, lambda a: a[0] >= 1.25 * n * (d.N0 + dN) / 2) \
            or CLAMP_ACTUATORS[-1]
        d.spring_rate = 0.25 * d.N0 / max(pop.ridge_h.hi, 1.0)
        d.clamp_travel = (pop.d_max - pop.d_min) / 2 + pop.ridge_h.hi + req.clamp_margin
        d.ring_inner_d = pop.d_max + 2 * req.clamp_margin

        # --- battery: smallest pack giving the trees-per-charge target on a mean tree
        mean_tree = Tree(pop.height.mean, pop.d_base.mean, pop.d_base.mean * pop.taper.mean,
                         pop.lean.mean, pop.ridge_h.mean, pop.ridge_len.mean, pop.mu_dry.mean,
                         pop.bunch_radius.mean)
        mean_bunches = [pop.peduncle.mean] * round(pop.bunches_per_month * 1.5)
        e_mean = d.energy_tree(mean_tree, mean_bunches, dwell_s=60.0)
        reserve = d.descent_reserve(pop.height.hi)
        d.battery_ah = _pick(BATTERIES, lambda ah: (ah * V_PACK * req.battery_dod - reserve) / e_mean
                             >= req.min_trees_per_charge) or BATTERIES[-1]

        # --- mass roll-up
        ring = 1.2 + 0.8 * (d.ring_inner_d / 300.0)
        mb = {
            "rings (2, hinged Al 6061)": TIERS * ring,
            "columns (3, Al tube)": 3 * H * 0.6,
            "wheel arms + springs": nn * 0.45,
            "wheels": nn * wheel[3],
            "drive gearmotors": nn * d.motor[3],
            "clamp actuators + load cells": TIERS * (d.clamp_actuator[1] + 0.2),
            "harvest arm (slew ring, boom, mast)": 1.5 + m_boom + 1.0,
            "saw unit (motor, blade, guard)": m_saw_unit,
            "battery": d.battery_ah * V_PACK / 140.0 + 0.3,
            "electronics, camera, radio": 1.5,
        }
        mb["fasteners & wiring (8 %)"] = 0.08 * sum(mb.values())
        new_mass = sum(mb.values())
        d.mass_breakdown = mb
        if abs(new_mass - mass) < 0.005:
            d.mass = new_mass
            d.converged = True
            break
        mass = new_mass

    _checks(d)
    return d


def _checks(d: Design) -> None:
    req, pop = d.req, d.pop
    C = d.checks.append
    C(Check("Sizing converged", float(d.converged), 1.0, "", d.converged))
    if not d.converged or not math.isfinite(d.N0):
        C(Check("Climb traction feasible with this wheel", 0.0, 1.0, "", False))
        return
    W = d.weight
    n, nn = N_PER_TIER, N_WHEELS
    mu_c = pop.mu_dry.lo
    mu_w = mu_c * pop.wet_factor
    worst = Tree(pop.height.hi, pop.d_base.hi, pop.d_base.hi * pop.taper.lo, pop.lean.hi,
                 pop.ridge_h.hi, pop.ridge_len.lo, mu_c, pop.bunch_radius.hi)

    climb_margin = d.traction_available(mu_c) / d.traction_demand(worst)
    C(Check("Climb traction, worst ridge, driest-worst bark", climb_margin, req.sf_climb, "x",
            climb_margin >= req.sf_climb - 1e-9))
    hold = d.traction_available(mu_w) / W
    C(Check("Static hold on wet bark", hold, req.sf_hold_wet, "x", hold >= req.sf_hold_wet - 1e-9))

    m_saw_unit = d.mass_breakdown["saw unit (motor, blade, guard)"]
    m_boom = 1.5 * d.reach
    wind = 0.5 * 1.2 * req.wind_max ** 2 * 1.2 * 0.25
    M = G * (m_saw_unit * d.reach + m_boom * d.reach / 2 + 0.05 * d.mass) \
        + req.feed_force * d.reach + wind * (d.H + 0.3)
    dN = 2 * (M / d.H + W * math.sin(math.radians(pop.lean.hi)) / 2) / n
    C(Check("Min wheel load / preload (arm at full reach, max lean)", (d.N0 - dN) / d.N0,
            req.min_wheel_load_frac, "", (d.N0 - dN) / d.N0 >= req.min_wheel_load_frac - 1e-9))
    peak_wheel = d.N0 * 1.25 + dN  # + spring rise on the tallest ridge
    C(Check("Peak wheel load vs wheel rating", peak_wheel, d.wheel[2], "N", peak_wheel <= d.wheel[2], "≤"))

    F_drive = req.sf_climb * d.traction_demand(worst)
    T_req = F_drive / nn * d.r_wheel
    C(Check("Gearmotor torque vs worst-ridge demand", d.motor[1] / T_req, 1.0, "x", d.motor[1] >= T_req))
    C(Check("Climb speed", d.speed, req.climb_speed, "m/s", d.speed >= req.climb_speed))
    C(Check("Worm drive self-locking (η < 0.5)", d.eta_worm, 0.5, "", d.eta_worm < 0.5, "<"))
    act_need = 1.25 * n * (d.N0 + dN) / 2
    C(Check("Clamp actuator force", d.clamp_actuator[0] / act_need, 1.0, "x", d.clamp_actuator[0] >= act_need))

    C(Check("Blade cuts thickest stalk in one pass", d.blade / 2 - 35.0, pop.peduncle.hi, "mm",
            d.blade / 2 - 35.0 >= pop.peduncle.hi))
    tcut = d.cut_time(pop.peduncle.hi)
    C(Check("Cut time, thickest stalk", tcut, req.max_cut_time, "s", tcut <= req.max_cut_time, "≤"))

    mean_tree = Tree(pop.height.mean, pop.d_base.mean, pop.d_base.mean * pop.taper.mean, pop.lean.mean,
                     pop.ridge_h.mean, pop.ridge_len.mean, pop.mu_dry.mean, pop.bunch_radius.mean)
    e_mean = d.energy_tree(mean_tree, [pop.peduncle.mean] * round(pop.bunches_per_month * 1.5), 60.0)
    tpc = (d.battery_wh * req.battery_dod - d.descent_reserve(pop.height.hi)) / e_mean
    C(Check("Trees per battery charge (mean tree)", tpc, req.min_trees_per_charge, "", tpc >= req.min_trees_per_charge))
    worst_e = d.energy_tree(worst, [pop.peduncle.hi] * 4, 300.0) + d.descent_reserve(pop.height.hi)
    C(Check("Full battery covers worst tree + reserve", d.battery_wh * req.battery_dod / worst_e, 1.0, "x",
            d.battery_wh * req.battery_dod >= worst_e))
    C(Check("Robot mass (two-person carry)", d.mass, req.max_mass, "kg", d.mass <= req.max_mass, "≤"))
    arrest = 2.0 * W  # taut belay, fall factor ≈ 0
    C(Check("Belay tether safety factor", req.tether_mbs / arrest, req.tether_sf, "x",
            req.tether_mbs / arrest >= req.tether_sf))


def optimize(req: Requirements = Requirements(), pop: TreePopulation = TreePopulation()) -> Design:
    """Lightest passing design over wheel size × tier spacing."""
    best: Optional[Design] = None
    for wheel in WHEELS:
        for H in TIER_SPACINGS:
            d = size(req, pop, wheel, H=H)
            if d.ok and (best is None or d.mass < best.mass - 1e-9):
                best = d
    if best is None:
        raise RuntimeError("no catalogue combination satisfies the requirements")
    return best
