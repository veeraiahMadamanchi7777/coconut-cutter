"""Reimagined concept: inchworm climber with self-locking cantilever collars.

Why: the wheel-ring robot (``design.py``) holds on by friction under a fixed
clamp force. That makes it heavy (six motors, big clamp preload to survive
leaf-scar ridges), and it is grounded whenever the bark is wet, which in a
monsoon climate is a third of the year.

How this one works:

    * Two **collars**. Each is a rigid V-pad on one side of the trunk and a
      steel-cored strap around the other, with a cantilever arm sticking out
      on the pad side. Weight hung on the arm *cocks* the collar: the pad and
      strap bite at two points a height ``h`` apart, and the normal force is
      N = M/h, where M is the moment of the load about the trunk axis. Grip
      therefore grows with the load. It holds whenever 2·μ·M/h ≥ W, i.e. the
      lever L = M/W ≥ h/(2μ), wet or dry. (Hunters' climbing tree stands
      work exactly this way.)
    * One **electric cylinder** (BLDC + ball screw + spring-applied brake)
      joins the two arm tips. Climbing = release upper collar, extend,
      re-lock; release lower collar, retract, re-lock. Repeat.
    * Each collar has a **release servo** that adds slack to the strap. It is
      spring-return: with no power the collar locks. The controller never
      releases a collar unless the other one is verified locked.
    * A constant-force **strap take-up reel** follows the trunk taper.
    * Self-locking alone is NOT enough while harvesting: with the saw swung to
      the far side of the trunk the cocking moment nearly cancels. So the
      upper collar also has a **harvest tensioner** (self-locking lead screw on
      the strap) that clamps positively before the arm may deploy, and slackens
      again before the robot moves.
    * The **harvest arm and saw** sit on the upper collar (same saw sizing as
      the wheel concept); the battery box sits at the arm tip as a counterweight.
    * A ground **belay tether** is a fall arrest of last resort.

Leaf-scar ridges do not matter: a released collar has slack and simply
slides over them. Every function is pure; ``optimize`` searches catalogue
combinations for the lightest design that passes every check.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from .design import (BATTERIES, BLADES, G, SAW_MOTORS, V_PACK, Check, Requirements as WheelRequirements, _pick)
from .tree import Tree, TreePopulation

# ---- catalogues -----------------------------------------------------------
# 24 V BLDC electric cylinders (ball screw, spring brake):
# name, rated force N, no-load speed mm/s, motor W, base mass kg, mass per 100 mm stroke kg
CYLINDERS = [("EC-08", 800, 600, 300, 2.2, 0.35), ("EC-15", 1500, 450, 400, 2.8, 0.45),
             ("EC-25", 2500, 300, 450, 3.6, 0.55), ("EC-40", 4000, 200, 500, 4.6, 0.70)]
STROKES = [300, 400, 500, 600, 800]  # mm
# Square 6061-T6 tube for the cantilever arms: (side mm, wall mm, kg/m)
AL_TUBES = [(30, 2.0, 0.61), (40, 3.0, 1.20), (50, 3.0, 1.52), (60, 4.0, 2.42)]
LEVERS = [0.30, 0.35, 0.40, 0.45, 0.50, 0.55, 0.60]  # m, trunk axis to arm tip
PAD_SPANS = [0.10, 0.12, 0.15, 0.18]                 # m, vertical pad-to-strap bite distance h
# Self-locking lead-screw strap tensioners for harvest mode: rated strap tension N, mass kg
TENSIONERS = [(600, 0.6), (1000, 0.8), (1500, 1.0), (2500, 1.4)]
ARM_STOW_X = 0.20  # m, saw CG when stowed (parked over the cantilever)


@dataclass(frozen=True)
class Requirements:
    sf_lock: float = 1.5            # self-lock margin on the wettest, slipperiest bark
    min_speed: float = 0.15         # m/s average climb rate
    max_mass: float = 25.0          # kg, one-person lift (two-person mount still allowed)
    max_cut_time: float = 10.0
    cut_energy: float = 80.0        # J/cm^3
    kerf: float = 2.4
    feed_force: float = 60.0
    min_trees_per_charge: float = 20.0
    battery_dod: float = 0.80
    descent_reserve_sf: float = 2.0
    electronics_w: float = 25.0
    eta_screw: float = 0.85
    eta_elec: float = 0.85
    wind_max: float = 10.0
    strap_preload: float = 60.0     # N, take-up spring tension that starts the collar cocking
    strap_mbs: float = 8000.0       # N, steel-cored strap breaking strength
    strap_sf: float = 4.0
    pad_pressure_max: float = 0.6   # MPa, rubber pad on bark (no bark damage)
    pad_area: float = 60 * 90       # mm^2 per V-pad flank pair
    fatigue_range: float = 25.0 / 1.35  # MPa, EN 1999-1-3 cat. 25 welded Al, γMf 1.35
    max_deflection: float = 5.0     # mm at the arm tip
    lock_settle: float = 8.0        # mm lost per step while a collar bites
    transition_s: float = 0.25      # s per release or lock
    tether_mbs: float = 15_000.0
    tether_sf: float = 10.0
    tether_kg_per_m: float = 0.065  # 10 mm belay rope; the robot carries what hangs below it
    sway_tilt: float = 3.0          # deg, extra trunk tilt from wind sway at the operating wind limit
    lean_misalign: float = 6.0      # deg of lean that may act the wrong way (S-curve, mounting error)


@dataclass
class InchwormDesign:
    req: Requirements
    pop: TreePopulation
    lever: float
    pad_span: float
    tube: Tuple[int, float, float]
    cylinder: Tuple = CYLINDERS[0]
    stroke: int = 0
    saw_motor: Tuple[int, float] = (0, 0.0)
    blade: int = 0
    battery_ah: int = 0
    reach: float = 0.0
    mass: float = 0.0
    mass_breakdown: Dict[str, float] = field(default_factory=dict)
    m_upper: float = 0.0     # kg moving with the upper collar (collar, arm, saw, cylinder body)
    m_lower: float = 0.0     # kg moving with the lower collar (collar, battery box)
    cg_z_upper: float = 0.0  # m, CG below the upper collar when it carries the robot (worst stroke)
    cg_z_lower: float = 0.0  # m, CG above the lower collar when it carries the robot (worst stroke)
    tensioner: Tuple[int, float] = (0, 0.0)
    N_active: float = 0.0    # N, strap clamp force the tensioner applies in harvest mode
    lock_margin_wet: float = 0.0
    checks: List[Check] = field(default_factory=list)
    converged: bool = False

    concept = "Inchworm, self-locking collars"
    climbs_wet = True
    mount_s = 45.0      # strap round trunk, buckle, tension (no hinged ring, no wheel arms)
    dismount_s = 30.0

    # ------------------------------------------------------------ basics
    @property
    def ok(self) -> bool:
        return self.converged and all(c.ok for c in self.checks)

    def failed(self) -> List[Check]:
        return [c for c in self.checks if not c.ok]

    @property
    def weight(self) -> float:
        return self.mass * G

    @property
    def battery_wh(self) -> float:
        return self.battery_ah * V_PACK

    @property
    def saw_w(self) -> float:
        return float(self.saw_motor[0])

    # ------------------------------------------------------------ physics
    def cocking_moment(self, arm_x: float) -> float:
        """Moment [N·m] about the trunk axis on the UPPER collar when it alone carries
        the robot, with the saw arm's CG at signed horizontal offset ``arm_x`` [m]
        (+ = cantilever side). Battery box hangs off the lower collar via the cylinder
        at the arm tip; everything except the saw unit acts at the lever."""
        m_saw = self.mass_breakdown.get("saw unit (motor, blade, guard)", 0.0)
        m_boom = self.mass_breakdown.get("harvest arm (slew ring, boom)", 0.0)
        rest = self.mass - m_saw - m_boom
        return G * (rest * self.lever * 0.85 + (m_saw + 0.5 * m_boom) * arm_x) - self.req.feed_force * max(0, -arm_x)

    def tether_weight(self, height: float) -> float:
        """Weight [N] of belay rope hanging from the robot at ``height`` m."""
        return self.req.tether_kg_per_m * max(0.0, height) * G

    def lock_ratio(self, mu: float, arm_x: float, carried: Optional[float] = None,
                   active: float = 0.0, lean: float = 0.0, height: float = 0.0) -> float:
        """Friction capacity / axial load for one collar holding the robot.

        ``carried=None``: the UPPER collar holds everything (lower one moving, or harvesting).
        ``carried=W``: the LOWER collar holds everything (upper one moving).
        ``active``: strap clamp force from the harvest tensioner.
        ``lean`` [deg]: trunk lean. Mounting rule: the arms go on the side the trunk
        leans toward. Then the CG that hangs BELOW the upper collar swings back toward
        the trunk and takes lever away (W·|z|·sinθ), while the CG ABOVE the lower collar
        swings outward and helps. Only ``lean_misalign`` degrees are assumed to act
        the wrong way on the lower collar (S-curved trunk, mounting error).
        ``height`` [m]: adds the weight of belay rope hanging from the arm tip.
        """
        Wt = self.tether_weight(height)
        th = math.radians(lean)
        if carried is None:
            W = self.weight + Wt
            M = (self.cocking_moment(arm_x) + Wt * self.lever) * math.cos(th) \
                - self.weight * self.cg_z_upper * math.sin(th)
        else:
            W = carried + Wt
            tm = math.radians(min(lean, self.req.lean_misalign))
            M = (carried * self.lever * 0.85 + Wt * self.lever) * math.cos(tm) \
                - carried * self.cg_z_lower * math.sin(tm)
        N = max(0.0, active + M / self.pad_span) if active else max(0.0, M) / self.pad_span
        return 2 * mu * N / (W * math.cos(th))

    def harvest_ratio(self, mu: float, lean: float = 0.0, height: float = 0.0) -> float:
        """Upper collar, tensioner engaged, saw on the worst (far) side."""
        return self.lock_ratio(mu, -self.reach, active=self.N_active, lean=lean, height=height)

    def climb_ratio(self, mu: float, lean: float = 0.0, height: float = 0.0) -> float:
        """Self-lock with the saw stowed: the worse of the two collars."""
        up = self.lock_ratio(mu, ARM_STOW_X, lean=lean, height=height)
        lo = self.lock_ratio(mu, 0.0, carried=self.weight, lean=lean, height=height)
        return min(up, lo)

    def holds(self, tree: Tree, wet: bool) -> bool:
        mu = tree.mu_dry * (self.pop.wet_factor if wet else 1.0)
        lean = tree.lean + self.req.sway_tilt
        return (self.fits(tree) and self.harvest_ratio(mu, lean, tree.height) >= 1.15
                and self.climb_ratio(mu, lean, tree.height) >= 1.15)

    def fits(self, tree: Tree) -> bool:
        """Inside the design envelope: trunk diameter, height and lean."""
        return (self.pop.d_min <= min(tree.d_base, tree.d_top)
                and max(tree.d_base, tree.d_top) <= self.pop.d_max
                and tree.height <= self.pop.height.hi and tree.lean <= self.pop.lean.hi)

    def can_climb(self, tree: Tree, wet: bool) -> bool:
        return self.holds(tree, wet)

    def can_hold(self, tree: Tree, wet: bool) -> bool:
        return self.holds(tree, wet)

    def speed_at(self, force: float) -> float:
        """Cylinder speed [m/s] at a given load (power-limited above the knee)."""
        _, F_r, v0, P, _, _ = self.cylinder
        v_pow = self.req.eta_screw * P / max(force, 1.0)
        return min(v0 / 1000.0, v_pow)

    @property
    def step_length(self) -> float:
        return (self.stroke - self.req.lock_settle) / 1000.0

    def _step_time(self, up: bool) -> float:
        drag = 2 * self.pop.mu_dry.hi * self.req.strap_preload * 0.3  # released collar sliding
        heavy = self.m_upper * G + drag     # extend: lifts / lowers the upper assembly
        light = self.m_lower * G + drag     # retract: lifts / lowers the lower assembly
        s = self.step_length
        v_a = self.speed_at(heavy) if up else self.speed_at(drag) * 0.8
        v_b = self.speed_at(light) if up else self.speed_at(drag) * 0.8
        return s / v_a + s / v_b + 4 * self.req.transition_s

    @property
    def speed(self) -> float:
        """Average climb rate [m/s]."""
        return self.step_length / self._step_time(up=True)

    @property
    def descend_speed(self) -> float:
        return self.step_length / self._step_time(up=False)

    def steps(self, height: float) -> int:
        return math.ceil(height / self.step_length)

    def climb_time(self, tree: Tree) -> float:
        return self.steps(tree.height) * self._step_time(True)

    def descend_time(self, tree: Tree) -> float:
        return self.steps(tree.height) * self._step_time(False)

    def cut_time(self, peduncle_mm: float) -> float:
        vol_cm3 = self.req.kerf * math.pi * peduncle_mm ** 2 / 4 / 1000.0
        return self.req.cut_energy * vol_cm3 / (self.saw_w * 0.6)

    def energy_climb(self, height: float) -> float:
        drag = 2 * self.pop.mu_dry.hi * self.req.strap_preload * 0.3
        F = self.weight + 2 * drag
        e = F * height / (self.req.eta_screw * self.req.eta_elec)
        e += self.tether_weight(height) * height / 2 / (self.req.eta_screw * self.req.eta_elec)  # rope
        e += self.steps(height) * 4 * 5.0  # release servos, J per transition
        return e / 3600.0

    def energy_descend(self, height: float) -> float:
        # Ball screw is back-drivable: lowering is a controlled brake; motor runs at
        # ~25 % of lift power, no regeneration assumed.
        return 0.25 * self.weight * height / self.req.eta_elec / 3600.0 \
            + self.steps(height) * 4 * 5.0 / 3600.0

    def energy_tree(self, tree: Tree, peduncles: Sequence[float], dwell_s: float = 0.0) -> float:
        e = self.energy_climb(tree.height) + self.energy_descend(tree.height)
        e += sum(self.saw_w * (self.cut_time(p) + 3.0) / 3600.0 for p in peduncles)
        e += 0.06 * len(peduncles)
        t = self.climb_time(tree) + self.descend_time(tree) + dwell_s
        e += self.req.electronics_w * t / 3600.0
        return e

    def energy_base(self, tree: Tree) -> float:
        """energy_tree with no bunches and no dwell (fleet fast path)."""
        return self.energy_tree(tree, [], 0.0)

    def descent_reserve(self, height: float) -> float:
        t = self.steps(height) * self._step_time(False)
        return self.req.descent_reserve_sf * (self.energy_descend(height) + self.req.electronics_w * t / 3600)


def size(req: Requirements, pop: TreePopulation, lever: float, pad_span: float, tube,
         max_iter: int = 60) -> InchwormDesign:
    d = InchwormDesign(req=req, pop=pop, lever=lever, pad_span=pad_span, tube=tube)
    d.reach = pop.bunch_radius.hi
    flange_r, clear = 25.0, 10.0
    d.blade = _pick(BLADES, lambda b: b / 2 - flange_r - clear >= pop.peduncle.hi) or BLADES[-1]
    for sm in SAW_MOTORS:
        d.saw_motor = sm
        if d.cut_time(pop.peduncle.hi) <= req.max_cut_time:
            break
    h_max = pop.height.hi
    mean_tree = Tree(pop.height.mean, pop.d_base.mean, pop.d_base.mean * pop.taper.mean, pop.lean.mean,
                     pop.ridge_h.mean, pop.ridge_len.mean, pop.mu_dry.mean, pop.bunch_radius.mean)
    mass = 18.0
    for _ in range(max_iter):
        d.mass = mass
        # cylinder: stroke & model giving the target average speed with force margin
        best = None
        for cyl in CYLINDERS:
            for st in STROKES:
                d.cylinder, d.stroke = cyl, st
                if cyl[1] < 1.5 * mass * G:
                    continue
                if d.speed >= req.min_speed:
                    m = cyl[4] + cyl[5] * st / 100
                    if best is None or m < best[0]:
                        best = (m, cyl, st)
        if best is None:
            d.cylinder, d.stroke = CYLINDERS[-1], STROKES[-1]
        else:
            _, d.cylinder, d.stroke = best
        m_cyl = d.cylinder[4] + d.cylinder[5] * d.stroke / 100
        m_saw_unit = d.saw_motor[1] + 0.1 + (d.blade / 1000) ** 2 * 6 + 0.6
        m_boom = 1.5 * d.reach + 1.0
        arm_len = lever + d.pop.d_max / 2000 + 0.1
        e_mean = d.energy_tree(mean_tree, [pop.peduncle.mean] * round(pop.bunches_per_month * 1.5), 60.0)
        reserve = d.descent_reserve(h_max)
        d.battery_ah = _pick(BATTERIES, lambda ah: (ah * V_PACK * req.battery_dod - reserve) / e_mean
                             >= req.min_trees_per_charge) or BATTERIES[-1]
        collar = 2 * arm_len * tube[2] + 0.9 + 0.4 + 0.4 + 0.35  # arm tubes, V-pad, strap, reel, release servo
        # harvest tensioner: clamp so the upper collar alone holds the robot, wet, saw far side
        mu_wet = pop.mu_dry.lo * pop.wet_factor
        th = math.radians(pop.lean.hi + req.sway_tilt)
        Wt = d.tether_weight(h_max)
        M_far = ((d.cocking_moment(-d.reach) if d.mass_breakdown else 0.0) + Wt * lever) * math.cos(th) \
            - mass * G * d.cg_z_upper * math.sin(th)
        d.N_active = max(0.0, req.sf_lock * (mass * G + Wt) * math.cos(th) / (2 * mu_wet)
                         - M_far / pad_span) * 1.03
        d.tensioner = _pick(TENSIONERS, lambda t: t[0] >= 1.2 * d.N_active) or TENSIONERS[-1]
        mb = {
            "upper collar (arm, V-pad, strap, reel, servo)": collar,
            "harvest strap tensioner": d.tensioner[1],
            "lower collar (arm, V-pad, strap, reel, servo)": collar,
            "electric cylinder + brake": m_cyl,
            "harvest arm (slew ring, boom)": m_boom,
            "saw unit (motor, blade, guard)": m_saw_unit,
            "battery": d.battery_ah * V_PACK / 140.0 + 0.3,
            "electronics, camera, radio": 1.5,
        }
        mb["fasteners & wiring (8 %)"] = 0.08 * sum(mb.values())
        new = sum(mb.values())
        d.mass_breakdown = mb
        d.m_upper = mb["upper collar (arm, V-pad, strap, reel, servo)"] + m_boom + m_saw_unit + m_cyl * 0.6 \
            + 1.5 + mb["fasteners & wiring (8 %)"] / 2
        d.m_lower = new - d.m_upper
        # Axial CG offsets at full extension (worst case), from the layout.
        sep = 0.56 + d.stroke / 1000.0
        fast = mb["fasteners & wiring (8 %)"] / 2
        up_parts = [(collar + d.tensioner[1] + 1.5 + fast, 0.0), (m_boom, 0.35), (m_saw_unit, 0.60), (m_cyl, -0.30)]
        lo_parts = [(collar + fast, 0.0), (mb["battery"], -0.13)]
        d.cg_z_upper = -(sum(m * z for m, z in up_parts) + sum(m * (z - sep) for m, z in lo_parts)) / new
        d.cg_z_lower = (sum(m * (z + sep) for m, z in up_parts) + sum(m * z for m, z in lo_parts)) / new
        if abs(new - mass) < 0.005:
            d.mass, d.converged = new, True
            break
        mass = new
    _checks(d)
    return d


def _tube_props(side: float, wall: float) -> Tuple[float, float]:
    I = (side ** 4 - (side - 2 * wall) ** 4) / 12.0
    return I, I / (side / 2)


def _checks(d: InchwormDesign) -> None:
    req, pop = d.req, d.pop
    C = d.checks.append
    C(Check("Sizing converged", float(d.converged), 1.0, "", d.converged))
    if not d.converged:
        return
    mu_wet = pop.mu_dry.lo * pop.wet_factor
    W = d.weight
    lean, H = pop.lean.hi + req.sway_tilt, pop.height.hi
    # Climbing: pure self-lock, saw stowed, either collar alone carrying the robot,
    # at the top of the tallest tree (most rope), max lean plus wind sway.
    r_c = d.climb_ratio(mu_wet, lean, H)
    d.lock_margin_wet = r_c
    C(Check(f"Self-lock while climbing: wet bark, {pop.lean.hi:g}° lean + sway, {H:g} m up", r_c, req.sf_lock,
            "x", r_c >= req.sf_lock))
    r_v = d.climb_ratio(mu_wet, 0.0, 0.0)
    C(Check("Self-lock while climbing: wet bark, vertical trunk", r_v, req.sf_lock, "x", r_v >= req.sf_lock))
    # Harvesting: tensioner engaged, saw on the far side (cocking moment fights the grip).
    r_h = d.harvest_ratio(mu_wet, lean, H)
    C(Check(f"Harvest hold: wet bark, saw on far side, {pop.lean.hi:g}° lean + sway, {H:g} m up", r_h,
            req.sf_lock, "x", r_h >= req.sf_lock))
    C(Check("Harvest tensioner rating", d.tensioner[0] / max(d.N_active, 1.0), 1.2, "x",
            d.tensioner[0] >= 1.2 * d.N_active))
    M = max(d.cocking_moment(+d.reach), W * d.lever * 0.85)
    N = M / d.pad_span + d.N_active
    T = N  # strap carries the reaction opposite the pad
    C(Check("Strap tension safety factor", req.strap_mbs / T, req.strap_sf, "x", req.strap_mbs / T >= req.strap_sf))
    p = N / req.pad_area
    C(Check("Pad pressure on bark", p, req.pad_pressure_max, "MPa", p <= req.pad_pressure_max, "≤"))
    I, Z = _tube_props(d.tube[0], d.tube[1])
    M = max(d.cocking_moment(+d.reach), W * d.lever * 0.85)
    sig = M * 1000 / Z / 2  # two arm tubes share the moment
    C(Check("Arm stress range vs Al weld fatigue", sig, req.fatigue_range, "MPa", sig <= req.fatigue_range, "≤"))
    L = d.lever * 1000
    defl = (M / d.lever) / 2 * L ** 3 / (3 * 69_000 * I)
    C(Check("Arm tip deflection", defl, req.max_deflection, "mm", defl <= req.max_deflection, "≤"))
    F_need = 1.5 * (d.m_upper * G + d.tether_weight(pop.height.hi) + 2 * pop.mu_dry.hi * req.strap_preload * 0.3)
    C(Check("Cylinder force margin", d.cylinder[1] / F_need, 1.0, "x", d.cylinder[1] >= F_need))
    C(Check("Average climb speed", d.speed, req.min_speed, "m/s", d.speed >= req.min_speed))
    take_up = math.pi * (pop.d_max - pop.d_max * pop.taper.lo) + 50
    C(Check("Strap take-up reel covers worst taper", 600.0, take_up, "mm", 600.0 >= take_up))
    slack = 25.0
    C(Check("Release slack clears tallest ridge", slack, 2 * pop.ridge_h.hi, "mm", slack >= 2 * pop.ridge_h.hi))
    C(Check("Blade cuts thickest stalk in one pass", d.blade / 2 - 35.0, pop.peduncle.hi, "mm",
            d.blade / 2 - 35.0 >= pop.peduncle.hi))
    tcut = d.cut_time(pop.peduncle.hi)
    C(Check("Cut time, thickest stalk", tcut, req.max_cut_time, "s", tcut <= req.max_cut_time, "≤"))
    mean_tree = Tree(pop.height.mean, pop.d_base.mean, pop.d_base.mean * pop.taper.mean, pop.lean.mean,
                     pop.ridge_h.mean, pop.ridge_len.mean, pop.mu_dry.mean, pop.bunch_radius.mean)
    e_mean = d.energy_tree(mean_tree, [pop.peduncle.mean] * round(pop.bunches_per_month * 1.5), 60.0)
    tpc = (d.battery_wh * req.battery_dod - d.descent_reserve(pop.height.hi)) / e_mean
    C(Check("Trees per battery charge (mean tree)", tpc, req.min_trees_per_charge, "", tpc >= req.min_trees_per_charge))
    worst = Tree(pop.height.hi, pop.d_base.hi, pop.d_base.hi * pop.taper.lo, pop.lean.hi, pop.ridge_h.hi,
                 pop.ridge_len.lo, pop.mu_dry.lo, pop.bunch_radius.hi)
    worst_e = d.energy_tree(worst, [pop.peduncle.hi] * 4, 300.0) + d.descent_reserve(pop.height.hi)
    C(Check("Full battery covers worst tree + reserve", d.battery_wh * req.battery_dod / worst_e, 1.0, "x",
            d.battery_wh * req.battery_dod >= worst_e))
    C(Check("Robot mass (one-person lift)", d.mass, req.max_mass, "kg", d.mass <= req.max_mass, "≤"))
    arrest = 2.0 * (W + d.tether_weight(pop.height.hi))
    C(Check("Belay tether safety factor", req.tether_mbs / arrest, req.tether_sf, "x",
            req.tether_mbs / arrest >= req.tether_sf))


def optimize(req: Requirements = Requirements(), pop: TreePopulation = TreePopulation()) -> InchwormDesign:
    best: Optional[InchwormDesign] = None
    for lever in LEVERS:
        for h in PAD_SPANS:
            for tube in AL_TUBES:
                d = size(req, pop, lever, h, tube)
                if d.ok and (best is None or d.mass < best.mass - 1e-9):
                    best = d
    if best is None:
        raise RuntimeError("no catalogue combination satisfies the requirements")
    return best
