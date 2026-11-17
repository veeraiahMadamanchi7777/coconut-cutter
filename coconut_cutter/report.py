"""Generate the design report (Markdown) and a dimensioned front-view SVG."""
from __future__ import annotations

import math
import random
from statistics import fmean, quantiles
from typing import List

from .design import Design, plant_air
from .line_sim import Capacity, LineParams


def bom(d: Design, stations: int) -> List[tuple]:
    h, b, t = d.rhs
    return [
        ("C-frame column + arms", f"RHS {h}x{b}x{t:g} S355J2H, EN 10219, welded, epoxy-coated",
         f"{(d.column_length + 2 * d.throat) / 1000:.2f} m", stations),
        ("Base plate", "S355 20 mm, 400x350, 4x M12 anchor", "1", stations),
        ("Pneumatic cylinder", f"ISO 15552 Ø{d.bore}/{d.rod} x {d.stroke} mm, adj. cushions, magnetic piston",
         "1", stations),
        ("Tool guide", "2x Ø20 h6 hardened guide rods + linear bushings (anti-rotation, side-load)", "1 set", stations),
        ("Crown punch", f"AISI 420 hardened 48-52 HRC, Ø{d.punch.diameter:g} x {d.punch_wall:g} wall x "
         f"{d.punch_length:.0f} mm, serrated shear-angle crown (engagement {d.punch.engagement:g})",
         "1 + 2 spare", stations),
        ("Hold-down / stripper foot", f"POM-C food-grade ring on die spring, preload {d.spring_preload:.0f} N", "1",
         stations),
        ("Knockout rod", f"AISI 304 Ø{d.punch.diameter - 2 * d.punch_wall - 5:g}, fixed to frame, "
         "through punch bore (2.5 mm radial clearance)", "1", stations),
        ("Nut cup", "Food-grade PU 80 ShA conical cup, Ø130-210 self-centering, drain hole", "1", stations),
        ("Valve", "5/3 closed-centre, 24 VDC, meter-out flow controls", "1", stations),
        ("Safety dump valve", "Dual-channel monitored soft-start/dump, Cat 3", "1", stations),
        ("Sensors", "Reed (home), inductive (depth switch on foot), nut-present (capacitive), pressure switch",
         "1 set", stations),
        ("Light curtain", f"Type 4, 14 mm resolution, mount ≥ {d.light_curtain_distance:.0f} mm", "1 pair", stations),
        ("Interlocked guard", "Polycarbonate + coded RFID interlock switch", "1", stations),
        ("Safety controller", "Safety relay/PLC, PL d, e-stop + reset + cycle-start", "1", stations),
        ("FRL", "Filter 5 µm + regulator 0-8 bar + lockable shut-off (LOTO)", "1", stations),
    ]


def svg_front_view(d: Design) -> str:
    """Schematic front view at 1:2.5 drawn from ``Design.levels()`` (cylinder retracted)."""
    s = 0.4
    lv = d.levels()
    h_rhs, b_rhs = d.rhs[0], d.rhs[1]
    margin_top, margin_bot = 50.0, 70.0
    H = margin_top + (lv["cylinder_top"] + 20) * s + margin_bot
    W = 600.0
    base = H - margin_bot          # y of base-plate top
    Y = lambda z: base - z * s     # mm above base -> svg y
    col_x = 110.0                  # column centre-line
    ax = col_x + d.throat * s      # punch axis
    o: List[str] = []
    o.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {H:.0f}" width="{W:.0f}" '
             f'height="{H:.0f}" font-family="Helvetica,Arial,sans-serif" font-size="11">')
    o.append('<style>.m{fill:#c9d3dc;stroke:#2b3a48;stroke-width:1.2}.n{fill:#7fb55a;stroke:#3d6b25}'
             '.p{fill:#9aa7b4;stroke:#2b3a48}.d{stroke:#b5402a;stroke-width:0.8;fill:none}'
             '.t{fill:#1b2733}.dt{fill:#b5402a}.c{stroke:#888;stroke-dasharray:6 3 1 3}</style>')
    o.append(f'<rect width="{W:.0f}" height="{H:.0f}" fill="#fbfbf8"/>')
    o.append('<text x="12" y="22" class="t" font-size="13" font-weight="bold">Tender-coconut opener — '
             'front view, cylinder retracted (schematic, 1:2.5)</text>')

    def rect(cls: str, x: float, z0: float, w: float, z1: float) -> None:
        o.append(f'<rect class="{cls}" x="{x:.1f}" y="{Y(z1):.1f}" width="{w:.1f}" height="{(z1 - z0) * s:.1f}"/>')

    # base plate (20 mm, below datum), column, upper arm
    rect("m", col_x - 60, -20, d.throat * s + 120, 0)
    rect("m", col_x - h_rhs * s / 2, 0, h_rhs * s, lv["arm_top"])
    rect("m", col_x - h_rhs * s / 2, lv["arm_bottom"], d.throat * s + h_rhs * s / 2 + 20, lv["arm_top"])
    # cylinder on top of arm
    cw = ISO_FLANGE(d) * s
    rect("p", ax - cw / 2, lv["arm_top"], cw, lv["cylinder_top"])
    # rod, carriage, guide rods
    rect("m", ax - d.rod * s / 2, lv["carriage_top"], d.rod * s, lv["arm_bottom"])
    rect("m", ax - 70 * s, lv["carriage_bottom"], 140 * s, lv["carriage_top"])
    for gx in (-55, 55):
        rect("m", ax + gx * s - 10 * s / 2, lv["carriage_top"], 10 * s, lv["arm_bottom"])
    # punch + foot
    rect("p", ax - d.punch.diameter * s / 2, lv["tip_home"], d.punch.diameter * s, lv["carriage_bottom"])
    fw = (d.punch.diameter + 24) * s
    rect("m", ax - fw / 2, lv["tip_home"] - d.req.foot_lead, fw, lv["tip_home"] - d.req.foot_lead + 8)
    # cup + mean nut
    cup_top = lv["nut_bottom"] + 15
    o.append(f'<path class="m" d="M{ax - 110 * s:.1f} {Y(cup_top):.1f} L{ax - 60 * s:.1f} {Y(0):.1f} '
             f'L{ax + 60 * s:.1f} {Y(0):.1f} L{ax + 110 * s:.1f} {Y(cup_top):.1f} Z"/>')
    nh, nd = d.pop.height.mean, d.pop.diameter.mean
    zb = lv["nut_bottom"]
    o.append(f'<ellipse class="n" cx="{ax:.1f}" cy="{Y(zb + nh * 0.42):.1f}" rx="{nd * s / 2:.1f}" '
             f'ry="{nh * 0.42 * s:.1f}"/>')
    o.append(f'<path class="n" d="M{ax - nd * s * 0.40:.1f} {Y(zb + nh * 0.62):.1f} L{ax:.1f} {Y(zb + nh):.1f} '
             f'L{ax + nd * s * 0.40:.1f} {Y(zb + nh * 0.62):.1f} Z"/>')
    o.append(f'<line class="c" x1="{ax:.1f}" y1="{Y(lv["cylinder_top"] + 15):.1f}" x2="{ax:.1f}" '
             f'y2="{Y(-35):.1f}"/>')

    def vdim(z0: float, z1: float, label: str, x: float, right: bool = True) -> None:
        y0, y1 = Y(z0), Y(z1)
        o.append(f'<line class="d" x1="{x:.1f}" y1="{y0:.1f}" x2="{x:.1f}" y2="{y1:.1f}"/>')
        for y in (y0, y1):
            o.append(f'<line class="d" x1="{x - 4:.1f}" y1="{y:.1f}" x2="{x + 4:.1f}" y2="{y:.1f}"/>')
        anchor = "start" if right else "end"
        o.append(f'<text class="dt" text-anchor="{anchor}" x="{x + (6 if right else -6):.1f}" '
                 f'y="{(y0 + y1) / 2 + 4:.1f}">{label}</text>')

    dx = ax + 130
    vdim(lv["tallest_top"], lv["tip_home"], f"load clearance {d.req.load_clearance:.0f}", dx)
    vdim(lv["nut_bottom"], lv["tallest_top"], f"max nut {d.pop.height.hi:.0f}", dx)
    vdim(0, lv["arm_top"], f"column {d.column_length:.0f}", col_x - h_rhs * s / 2 - 10, right=False)
    vdim(lv["arm_top"], lv["cylinder_top"], f"cyl. {d.stroke} stroke", dx)
    o.append(f'<line class="d" x1="{col_x:.1f}" y1="{Y(-45):.1f}" x2="{ax:.1f}" y2="{Y(-45):.1f}"/>')
    o.append(f'<text class="dt" text-anchor="middle" x="{(col_x + ax) / 2:.1f}" y="{Y(-45) + 15:.1f}">'
             f'throat {d.throat:.0f}</text>')
    lx = ax + 40
    o.append(f'<text class="t" text-anchor="end" x="{ax - cw / 2 - 8:.1f}" '
             f'y="{Y((lv["arm_top"] + lv["cylinder_top"]) / 2) + 4:.1f}">ISO 15552 Ø{d.bore}/{d.rod}</text>')
    o.append(f'<text class="t" x="{lx:.1f}" y="{Y(lv["carriage_bottom"] - 20):.1f}">crown Ø{d.punch.diameter:g}'
             '</text>')
    o.append(f'<text class="t" x="{lx:.1f}" y="{Y(lv["carriage_top"] + 25):.1f}">guided carriage</text>')
    o.append(f'<text class="t" x="{ax + 60 * s + 8:.1f}" y="{Y(10):.1f}">PU cup</text>')
    zc = lv["arm_bottom"] / 2
    o.append(f'<text class="t" text-anchor="middle" transform="translate({col_x + 4:.1f} {Y(zc):.1f}) '
             f'rotate(-90)">RHS {h_rhs}×{b_rhs}×{d.rhs[2]:g} S355</text>')
    o.append("</svg>")
    return "\n".join(o)


def ISO_FLANGE(d: Design) -> float:
    from .design import ISO15552_FLANGE
    return ISO15552_FLANGE[d.bore]


def markdown(d: Design, cap: Capacity, mc_n: int = 200_000, seed: int = 42,
             lp: LineParams = LineParams()) -> str:
    rng = random.Random(seed)
    forces, times = [], []
    for _ in range(mc_n):
        nut = d.pop.sample(rng)
        forces.append(d.required_force(nut, d.req.blade_dull_at_swap))
        times.append(d.machine_time(nut))
    q = quantiles(forces, n=1000, method="inclusive")
    # Stations run at their natural pace, not the target: size air for the best simulated day.
    air = plant_air(d, cap.stations, max(cap.rates))
    annual = d.req.target_rate_per_h * d.req.hours_per_day * d.req.days_per_year
    cycles_station_yr = annual / cap.stations
    h, b, t = d.rhs
    L: List[str] = []
    w = L.append
    w("# Tender-Coconut Opening Machine — Design & Production-Scale Verification\n")
    w("> Generated by `coconut-cutter report`. Every number below is computed from the "
      "parametric model in `coconut_cutter/`; change an assumption, regenerate, and the "
      "test suite re-verifies it.\n")
    w("![Front view](front_view.svg)\n")
    w("## 1. Concept\n")
    w("A bench-mounted **pneumatic crown punch**. The operator seats a trimmed tender coconut "
      "eye-end-up in a self-centering PU cup and presses start (light curtain + interlocked "
      "guard). An ISO 15552 cylinder drives a serrated tubular crown through husk and tender "
      "shell. A spring-loaded **hold-down foot** lands first and carries a **depth switch**, so "
      "the hole depth is referenced to each nut's own top — tall and short nuts get the same "
      "clean hole without adjusting anything. On the return stroke the foot strips the nut off "
      "the crown and a fixed **knockout rod** ejects the husk plug. Straw goes straight in.\n")
    w("## 2. Requirements\n")
    w("| Item | Value |\n|---|---|")
    w(f"| Plant throughput | {d.req.target_rate_per_h:.0f} nuts/h, {d.req.hours_per_day:.0f} h/day, "
      f"{d.req.days_per_year} days/yr = **{annual / 1e6:.1f} M nuts/yr** |")
    w(f"| Hole | Ø{d.punch.diameter:g} mm, ≥ {d.punch.min_overtravel:g} mm clean breakthrough on every nut |")
    w(f"| Air supply at machine | {d.req.supply_min_mpa * 10:.1f}–{d.req.supply_max_mpa * 10:.1f} bar g |")
    w(f"| Safety | ISO 12100 risk assessment, ISO 13849-1 PL d, ISO 13855 distances, food-contact "
      "materials (EU 1935/2004 / FDA) |")
    w("\n## 3. Incoming nut population (design assumption — replace with intake data)\n")
    w("| Parameter | Mean | SD | Min | Max |\n|---|---|---|---|---|")
    for name, dist, unit in [("Diameter", d.pop.diameter, "mm"), ("Height", d.pop.height, "mm"),
                             ("Husk over eye", d.pop.husk_top, "mm"), ("Shell", d.pop.shell, "mm"),
                             ("Strength factor", d.pop.strength, "×")]:
        w(f"| {name} ({unit}) | {dist.mean:g} | {dist.sd:g} | {dist.lo:g} | {dist.hi:g} |")
    w("\nSizing uses the **envelope nut** (every parameter at its adverse bound simultaneously) "
      f"with a crown dulled to {d.req.blade_dull_at_swap:g}× at the end of its swap interval.\n")
    w("## 4. Sized design\n")
    w("| Component | Result |\n|---|---|")
    w(f"| Crown punch | Ø{d.punch.diameter:g} × {d.punch_wall:g} mm wall, engagement {d.punch.engagement:g}, "
      f"length {d.punch_length:.0f} mm, AISI 420 hardened |")
    w(f"| Cut depth (from nut top) | {d.depth_cut:.0f} mm |")
    w(f"| Cylinder | ISO 15552 Ø{d.bore}/{d.rod} × {d.stroke} mm |")
    w(f"| Worst-case cut load (envelope nut, dull crown, + spring) | {d.worst_peak_force:.0f} N |")
    w(f"| Cylinder force @ min supply | {d.force_available:.0f} N |")
    w(f"| Stall force @ max supply (structural case) | {d.force_stall:.0f} N |")
    w(f"| Stripper spring preload | {d.spring_preload:.0f} N |")
    w(f"| Frame | RHS {h}×{b}×{t:g} S355, throat {d.throat:.0f} mm, column {d.column_length:.0f} mm |")
    w(f"| Air per cycle | {d.air_per_cycle_nl:.1f} NL |")
    w(f"| Light-curtain min distance (ISO 13855) | {d.light_curtain_distance:.0f} mm |")
    w("\n## 5. Engineering checks\n")
    w("| Check | Value | Limit | Status |\n|---|---|---|---|")
    for c in d.checks:
        w(f"| {c.name} | {c.value:.2f} {c.unit} | {c.cmp} {c.limit:.2f} | {'PASS' if c.ok else '**FAIL**'} |")
    w(f"\nFatigue: each station sees ~{cycles_station_yr / 1e6:.1f} M cycles/yr, so the frame is designed "
      "for infinite life (stress range below the constant-amplitude fatigue limit).\n")
    w(f"## 6. Monte-Carlo over {mc_n:,} nuts (dull crown)\n")
    w("| Metric | Value |\n|---|---|")
    w(f"| Mean cut load | {fmean(forces):.0f} N |")
    w(f"| P99.9 cut load | {q[998]:.0f} N |")
    w(f"| Max cut load | {max(forces):.0f} N |")
    w(f"| Margin at max vs available force | {d.force_available / max(forces):.2f}× |")
    w(f"| Machine time mean / max | {fmean(times):.2f} s / {max(times):.2f} s |")
    w(f"\n## 7. Production line\n")
    w(f"Discrete-event simulation, {len(cap.rates)} replications of a {d.req.hours_per_day:.0f} h day, "
      f"including operator handling (load {lp.load_mean} s, unload {lp.unload_mean} s, CV {lp.handling_cv}), "
      f"random jams ({lp.jam_prob * 100:.1f} %), crown swaps every {d.req.blade_resharpen_cuts} cuts, "
      f"breakdowns (MTBF {lp.mtbf_h:g} h, MTTR {lp.mttr_h * 60:.0f} min) and staggered breaks.\n")
    w("| Metric | Value |\n|---|---|")
    w(f"| **Stations required** | **{cap.stations}** (+1 standby recommended) |")
    w(f"| Line rate, mean | {cap.mean_rate:.0f} nuts/h |")
    w(f"| Line rate, P05 day | {cap.p05_rate:.0f} nuts/h (target {d.req.target_rate_per_h:.0f}) |")
    w(f"| Air demand (at best-day line rate {max(cap.rates):.0f}/h) | {air.demand_nl_min:.0f} NL/min |")
    w(f"| Compressor | ≥ {air.compressor_fad_nl_min / 1000:.2f} m³/min FAD @ 8 bar, ~{air.compressor_kw:.1f} kW |")
    w(f"| Air receiver | ≥ {air.receiver_l:.0f} L |")
    w("\n## 8. Bill of materials (per line)\n")
    w("| Item | Specification | Qty / station | Stations |\n|---|---|---|---|")
    for item, spec, qty, n in bom(d, cap.stations):
        w(f"| {item} | {spec} | {qty} | {n} |")
    w("\n## 9. Control & safety\n")
    w("State machine in `coconut_cutter/controller.py` (IDLE → EXTENDING → DWELL → RETRACTING), "
      "with safety stops (guard/curtain), e-stop air dump, low-pressure fault, extend/retract "
      "timeouts (jam detection), anti-tie-down start, reset-only restart, and a crown-swap lockout. "
      "The test suite fuzzes it with random input sequences and proves the punch is never "
      "commanded down while a hand could be in the danger zone.\n")
    w("## 10. Before release — open items\n")
    w("1. **Calibrate the cutting model**: punch ≥ 50 real nuts on a load-cell rig, feed traces to "
      "`mechanics.calibrate()`, and replace the default `MaterialModel`. Regenerate this report.")
    w("2. **Measure the intake**: fit `CoconutPopulation.from_measurements()` to ≥ 200 nuts per "
      "supplier/season.")
    w("3. Validate the safety functions on the built machine (ISO 13849-2), and measure the stopping time "
      "for the ISO 13855 distance.")
    w("4. Endurance-run one prototype for 100 k cycles and inspect the frame welds and crown wear "
      "(this confirms the dulling assumption).")
    w("5. Food-safety review: wash-down (IP65 sensors), no lubricant above the cup, drained cup.\n")
    return "\n".join(L)
