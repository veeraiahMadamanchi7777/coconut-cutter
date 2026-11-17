"""Parametric sizing of a bench-mounted pneumatic tender-coconut opener.

Machine concept (one *station*):

    * C-frame welded from S355 rectangular hollow section (RHS).
    * Self-centering conical PU cup holds the nut, eye end up.
    * ISO 15552 double-acting cylinder drives a tool carriage downward.
    * Tool = serrated tubular crown punch (AISI 420, hardened) inside a
      spring-loaded hold-down/stripper foot. The foot lands on the nut
      first; a depth switch on the foot-to-punch travel ends the cut at a
      fixed depth *relative to the nut top*, so every nut size gets the same
      hole regardless of height.
    * A fixed knockout rod through the punch bore ejects the husk plug at
      the top of the return stroke.
    * Safety: interlocked guard + type 4 light curtain (or two-hand control
      on the manual variant), 5/3 closed-centre valve, dump valve, ISO 13849
      PL d.

Every function here is pure: given assumptions in, numbers out. Sizing
picks the smallest standard catalogue part that passes all checks; the test
suite then verifies the chosen design against a production-scale population.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from .coconut import Coconut, CoconutPopulation
from .mechanics import MaterialModel, Punch, peak_force

P_ATM = 0.101325  # MPa
E_STEEL = 200_000.0  # MPa
NU_STEEL = 0.3

# ISO 15552 bores [mm] -> piston-rod diameter [mm]
ISO15552 = {32: 12, 40: 16, 50: 20, 63: 20, 80: 25, 100: 25, 125: 32}
# ISO 15552 body length excluding stroke (cap to cap, typical catalogue) [mm]
ISO15552_BODY = {32: 94, 40: 105, 50: 106, 63: 121, 80: 128, 100: 138, 125: 160}

# ISO 15552 square end-cap / front-flange width [mm]
ISO15552_FLANGE = {32: 50, 40: 55, 50: 65, 63: 75, 80: 100, 100: 115, 125: 140}
FLANGE_OVERHANG = 10.0  # mm each side the flange may overhang the arm on its adapter plate

# Vertical stack-up constants [mm]
CUP_SEAT = 45.0         # base-plate top to nut bottom when seated
CARRIAGE = 40.0         # tool carriage plate + guide bushings
ROD_PROJECTION = 50.0   # rod end + flange below the arm, cylinder retracted
SHANK = 40.0            # punch shank clamped in carriage
STD_STROKES = [100, 125, 160, 200, 250, 320]

# Cold-formed RHS (EN 10219) h x b x t [mm], bending about h.
RHS_CATALOGUE: List[Tuple[int, int, float]] = [
    (60, 60, 4.0), (80, 80, 5.0), (100, 100, 5.0), (120, 120, 6.0),
    (150, 100, 6.0), (150, 150, 8.0), (200, 100, 8.0), (200, 200, 8.0),
]


@dataclass(frozen=True)
class Requirements:
    """What production needs from the machine."""

    target_rate_per_h: float = 2000.0      # plant throughput, nuts/hour
    hours_per_day: float = 16.0            # two shifts
    days_per_year: int = 300
    supply_min_mpa: float = 0.55           # gauge, worst case at the machine
    supply_max_mpa: float = 0.70           # gauge, regulator upper limit (stall case)
    force_sf: float = 1.5                  # cylinder force margin over worst nut
    buckling_sf: float = 3.5
    static_sf: float = 1.5                 # vs yield under stall load
    max_tool_deflection: float = 0.5      # mm at the punch axis
    load_clearance: float = 40.0          # mm above tallest nut for hand loading
    foot_lead: float = 10.0               # mm foot protrudes ahead of punch tip
    max_cavity_intrusion: float = 45.0    # mm punch may enter cavity
    blade_resharpen_cuts: int = 8000      # crown swap interval
    blade_dull_at_swap: float = 1.25      # resistance multiplier at end of interval


@dataclass(frozen=True)
class Kinematics:
    extend_speed: float = 300.0     # mm/s approach + cut (flow-controlled)
    retract_speed: float = 500.0    # mm/s
    valve_response: float = 0.03    # s per switching event
    dwell: float = 0.10             # s at depth (lets the plug shear cleanly)


@dataclass
class Check:
    name: str
    value: float
    limit: float
    unit: str
    ok: bool
    note: str = ""
    cmp: str = "≥"   # direction of the limit: value ≥ limit, or "≤" for maxima


@dataclass
class Design:
    req: Requirements
    pop: CoconutPopulation
    punch: Punch
    mat: MaterialModel
    kin: Kinematics

    worst_peak_force: float = 0.0
    depth_cut: float = 0.0
    stroke: int = 0
    spring_preload: float = 0.0
    spring_rate: float = 0.0
    bore: int = 0
    rod: int = 0
    force_available: float = 0.0
    force_stall: float = 0.0
    force_retract: float = 0.0
    punch_wall: float = 1.5
    punch_length: float = 0.0
    throat: float = 0.0
    column_length: float = 0.0
    rhs: Tuple[int, int, float] = (0, 0, 0.0)
    air_per_cycle_nl: float = 0.0
    light_curtain_distance: float = 0.0
    checks: List[Check] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.checks)

    def failed(self) -> List[Check]:
        return [c for c in self.checks if not c.ok]

    # ---- per-nut behaviour used by simulation & tests -------------------
    def travel(self, nut: Coconut) -> float:
        """Carriage travel [mm] for this nut until the depth switch trips."""
        top_tallest = self.pop.height.hi
        gap = (top_tallest + self.req.load_clearance) - nut.height  # punch tip to nut top at home
        # The foot lands first; the depth switch trips when the punch tip is
        # ``depth_cut`` below the nut top (foot compressed by lead + depth_cut).
        return gap + self.depth_cut

    def machine_time(self, nut: Coconut) -> float:
        """Powered cycle time [s] for one nut (excludes operator handling)."""
        k = self.kin
        t = self.travel(nut)
        return (
            k.valve_response + t / k.extend_speed + k.dwell
            + k.valve_response + t / k.retract_speed
        )

    def required_force(self, nut: Coconut, dull: float = 1.0) -> float:
        """Total cylinder load [N] at the force peak for this nut."""
        m = MaterialModel(
            q_husk=self.mat.q_husk * dull, q_shell=self.mat.q_shell * dull,
            mu=self.mat.mu, p_lat=self.mat.p_lat,
        )
        return peak_force(nut, self.punch, m) + self.spring_force_at_depth()

    def levels(self) -> Dict[str, float]:
        """Vertical stack-up, mm above the base-plate top, cylinder retracted.

        The cylinder is front-flange mounted on TOP of the upper arm; the rod
        passes down through the arm to the guided tool carriage.
        """
        L: Dict[str, float] = {}
        L["nut_bottom"] = CUP_SEAT
        L["tallest_top"] = CUP_SEAT + self.pop.height.hi
        L["tip_home"] = L["tallest_top"] + self.req.load_clearance
        L["carriage_bottom"] = L["tip_home"] + self.punch_length
        L["carriage_top"] = L["carriage_bottom"] + CARRIAGE
        L["arm_bottom"] = L["carriage_top"] + ROD_PROJECTION
        L["arm_top"] = L["arm_bottom"] + self.rhs[0]
        L["cylinder_top"] = L["arm_top"] + self.stroke + ISO15552_BODY.get(self.bore, 160)
        return L

    def spring_force_at_depth(self) -> float:
        return self.spring_preload + self.spring_rate * (self.req.foot_lead + self.depth_cut)


def _rhs_props(h: float, b: float, t: float) -> Tuple[float, float, float]:
    """Area [mm²], I [mm⁴], Z [mm³] of an RHS about its strong axis (sharp corners)."""
    A = b * h - (b - 2 * t) * (h - 2 * t)
    I = (b * h ** 3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12.0
    return A, I, I / (h / 2.0)


def _free_air(volume_mm3: float, p_gauge: float) -> float:
    """Free (normal) litres for a compressed volume."""
    return volume_mm3 * 1e-6 * (p_gauge + P_ATM) / P_ATM


# EN 1993-1-9 detail category 71 (fillet-welded attachment), constant-amplitude
# fatigue limit Δσ_D = 0.737·Δσ_C, partial factor γ_Mf = 1.35 (safe-life, high consequence).
FATIGUE_LIMIT = 0.737 * 71.0 / 1.35
YIELD_S355 = 355.0
YIELD_420_HARD = 1000.0  # MPa, AISI 420 hardened & tempered ~48 HRC (conservative)


def size(
    req: Requirements = Requirements(),
    pop: CoconutPopulation = CoconutPopulation(),
    punch: Punch = Punch(),
    mat: MaterialModel = MaterialModel(),
    kin: Kinematics = Kinematics(),
) -> Design:
    d = Design(req=req, pop=pop, punch=punch, mat=mat, kin=kin)
    worst = pop.worst_case()
    C = d.checks.append

    # --- 1. Cut depth: deep enough for the thickest nut, shallow enough for the thinnest.
    d.depth_cut = pop.husk_top.hi + pop.shell.hi + punch.min_overtravel
    intrusion = d.depth_cut - (pop.husk_top.lo + pop.shell.lo)
    C(Check("Cavity intrusion, thinnest-walled nut", intrusion, req.max_cavity_intrusion, "mm",
            intrusion <= req.max_cavity_intrusion, "punch must not gouge the tender meat", "≤"))

    # --- 2. Stroke: clearance over tallest nut down to full depth in shortest nut.
    need_stroke = (pop.height.hi - pop.height.lo) + req.load_clearance + d.depth_cut
    d.stroke = next(s for s in STD_STROKES if s >= need_stroke + 5)  # 5 mm end-cushion reserve
    C(Check("Stroke reserve", d.stroke - need_stroke, 5.0, "mm", d.stroke - need_stroke >= 5.0))

    # --- 3. Hold-down / stripper spring: must strip the plugged nut off the punch.
    strip = punch.perimeter * mat.mu * mat.p_lat * (worst.husk_top + worst.shell) * req.blade_dull_at_swap
    strip += worst.mass * 9.81
    d.spring_preload = math.ceil(2.0 * strip / 10.0) * 10.0
    d.spring_rate = 1.0  # N/mm, long soft die spring
    C(Check("Stripper spring vs strip force", d.spring_preload / strip, 2.0, "x",
            d.spring_preload >= 2.0 * strip))

    # --- 4. Peak cutting force (worst nut, dull crown at end of swap interval).
    d.worst_peak_force = d.required_force(worst, dull=req.blade_dull_at_swap)
    need = d.worst_peak_force * req.force_sf

    # --- 5. Cylinder: smallest ISO bore whose force at min pressure covers the need.
    eff = 0.90
    for bore, rod in ISO15552.items():
        area = math.pi / 4 * bore ** 2
        if area * req.supply_min_mpa * eff >= need:
            d.bore, d.rod = bore, rod
            break
    if not d.bore:
        d.bore, d.rod = max(ISO15552.items())
    A = math.pi / 4 * d.bore ** 2
    Aann = A - math.pi / 4 * d.rod ** 2
    d.force_available = A * req.supply_min_mpa * eff
    d.force_stall = A * req.supply_max_mpa  # no friction credit: worst for structure
    d.force_retract = Aann * req.supply_min_mpa * eff
    C(Check("Cylinder force margin (min supply, dull crown, worst nut)",
            d.force_available / d.worst_peak_force, req.force_sf, "x",
            d.force_available >= need))
    retract_need = 2.0 * (strip + 3.0 * 9.81)  # plug strip + ~3 kg tooling
    C(Check("Retract force margin", d.force_retract / retract_need, 1.0, "x",
            d.force_retract >= retract_need))

    # --- 6. Rod buckling: Euler case 1 (fixed-free, K=2), full stroke + mounting.
    L = d.stroke + 60.0
    I_rod = math.pi * d.rod ** 4 / 64
    P_cr = math.pi ** 2 * E_STEEL * I_rod / (2.0 * L) ** 2
    C(Check("Piston-rod buckling SF (stall)", P_cr / d.force_stall, req.buckling_sf, "x",
            P_cr / d.force_stall >= req.buckling_sf))

    # --- 7. Crown punch: axial stress & thin-shell local buckling under stall.
    t = d.punch_wall
    D = punch.diameter
    A_tube = math.pi / 4 * (D ** 2 - (D - 2 * t) ** 2)
    sigma = d.force_stall / A_tube
    C(Check("Punch axial stress SF (stall)", YIELD_420_HARD / sigma, req.static_sf, "x",
            YIELD_420_HARD / sigma >= req.static_sf))
    R = (D - t) / 2
    sigma_cr = 0.3 * E_STEEL * t / (R * math.sqrt(3 * (1 - NU_STEEL ** 2)))  # 0.3 knockdown
    C(Check("Punch shell-buckling SF (stall)", sigma_cr / sigma, req.static_sf, "x",
            sigma_cr / sigma >= req.static_sf))
    d.punch_length = d.depth_cut + req.foot_lead + SHANK
    plug_max = pop.husk_top.hi + pop.shell.hi
    bore_free = d.punch_length - 15.0  # knockout rod reaches to 15 mm of the crown at home
    C(Check("Punch bore holds largest plug", bore_free, plug_max, "mm", bore_free >= plug_max))

    # --- 8. C-frame: throat clears the biggest nut; pick smallest RHS that passes
    #        fatigue (normal cuts), static (stall/jam) and stiffness.
    clearance = 25.0
    for h, b, tt in RHS_CATALOGUE:
        throat = pop.diameter.hi / 2 + clearance + h / 2
        d.rhs = (h, b, tt)
        col_len = d.levels()["arm_top"]  # base plate to top of the upper arm
        _, I, Z = _rhs_props(h, b, tt)
        F_cut = d.worst_peak_force
        M_fat = F_cut * throat
        M_stall = d.force_stall * throat
        sig_fat = M_fat / Z  # stress range: zero -> peak each cycle
        sig_stall = M_stall / Z
        # gap opening: two cantilever arms + column rotation
        delta = (2 * F_cut * throat ** 3 / (3 * E_STEEL * I)
                 + F_cut * throat ** 2 * col_len / (E_STEEL * I))
        flange_ok = b >= ISO15552_FLANGE[d.bore] - 2 * FLANGE_OVERHANG
        if (flange_ok and sig_fat <= FATIGUE_LIMIT and YIELD_S355 / sig_stall >= req.static_sf
                and delta <= req.max_tool_deflection):
            d.rhs, d.throat, d.column_length = (h, b, tt), throat, col_len
            break
    else:
        h, b, tt = RHS_CATALOGUE[-1]
        d.rhs = (h, b, tt)
        d.throat = pop.diameter.hi / 2 + clearance + h / 2
        d.column_length = d.levels()["arm_top"]
    _, I, Z = _rhs_props(*d.rhs)
    sig_fat = d.worst_peak_force * d.throat / Z
    sig_stall = d.force_stall * d.throat / Z
    delta = (2 * d.worst_peak_force * d.throat ** 3 / (3 * E_STEEL * I)
             + d.worst_peak_force * d.throat ** 2 * d.column_length / (E_STEEL * I))
    C(Check("Frame weld stress range vs fatigue limit (EN 1993-1-9 cat 71)", sig_fat, FATIGUE_LIMIT,
            "MPa", sig_fat <= FATIGUE_LIMIT, "infinite life", "≤"))
    flange_min = ISO15552_FLANGE[d.bore] - 2 * FLANGE_OVERHANG
    C(Check("Arm width carries cylinder flange", d.rhs[1], flange_min, "mm", d.rhs[1] >= flange_min))
    C(Check("Frame static SF (stall)", YIELD_S355 / sig_stall, req.static_sf, "x",
            YIELD_S355 / sig_stall >= req.static_sf))
    C(Check("Tool deflection at worst cut", delta, req.max_tool_deflection, "mm",
            delta <= req.max_tool_deflection, "", "≤"))

    # --- 9. Air: both strokes at max regulator pressure, +15 % tubing dead volume.
    vol = (A + Aann) * d.stroke * 1.15
    d.air_per_cycle_nl = _free_air(vol, req.supply_max_mpa)

    # --- 10. Light curtain safety distance, ISO 13855: S = K·T + C, 14 mm resolution (C=0).
    T_curtain = 0.015
    T_stop = kin.valve_response + 0.05 + 0.02  # valve + cylinder decel + safety relay
    S = 2000.0 * (T_curtain + T_stop)
    if S > 500.0:
        S = max(500.0, 1600.0 * (T_curtain + T_stop))
    d.light_curtain_distance = max(100.0, S)

    return d


@dataclass
class PlantAir:
    demand_nl_min: float
    compressor_fad_nl_min: float
    compressor_kw: float
    receiver_l: float


def plant_air(design: Design, stations: int, rate_per_h: float) -> PlantAir:
    """Compressor + receiver for ``stations`` running at plant ``rate_per_h``."""
    demand = design.air_per_cycle_nl * rate_per_h / 60.0
    fad = demand * 1.3  # 30 % margin for leaks & growth
    kw = fad / 1000.0 * 6.5  # ~6.5 kW per m³/min FAD @ 7 bar (screw, typical)
    # Receiver: every station starts its extend stroke at once (worst burst).
    # V = (Q_burst - Q_compressor)·t · p_atm / Δp, with Δp = 1 bar between
    # compressor cut-out (8 bar) and regulator inlet (7 bar), ×3 for repeat bursts;
    # never smaller than the usual FAD/3 rule of thumb for load/unload screws.
    t_ext = design.stroke / design.kin.extend_speed
    A = math.pi / 4 * design.bore ** 2
    burst_nl = stations * _free_air(A * design.stroke * 1.15, design.req.supply_max_mpa)
    burst_excess = max(0.0, burst_nl - fad / 60.0 * t_ext)
    receiver = max(3.0 * burst_excess * P_ATM / 0.1, fad / 3.0)
    return PlantAir(demand, fad, kw, receiver)


def optimize(
    req: Requirements = Requirements(),
    pop: CoconutPopulation = CoconutPopulation(),
    mat: MaterialModel = MaterialModel(),
    kin: Kinematics = Kinematics(),
    min_hole: float = 32.0,
    min_engagement: float = 0.40,
) -> Design:
    """Pick the crown (hole size, shear angle) that passes every check with the
    lowest compressed-air use, tie-broken by larger hole then lighter frame.

    ``min_engagement`` reflects tooth manufacturability: steeper shear angles
    than ~0.4 engagement give fragile teeth on a 1.5 mm wall.
    """
    best: Optional[Design] = None
    best_key = None
    for D in (32.0, 34.0, 36.0, 38.0, 40.0, 45.0):
        if D < min_hole:
            continue
        for e in (0.40, 0.45, 0.50, 0.60, 0.70):
            if e < min_engagement:
                continue
            cand = size(req, pop, Punch(diameter=D, engagement=e), mat, kin)
            if not cand.ok:
                continue
            key = (round(cand.air_per_cycle_nl, 3), -D, cand.rhs[0] * cand.rhs[2], -e)
            if best_key is None or key < best_key:
                best, best_key = cand, key
    if best is None:
        raise RuntimeError("no catalogue combination satisfies the requirements")
    return best
