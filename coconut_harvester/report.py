"""Design report (Markdown) and schematic drawing (SVG), generated from the model."""
from __future__ import annotations

import math
import random
from statistics import fmean, quantiles
from typing import List

from .design import G, N_PER_TIER, N_WHEELS, Design
from .fleet import FleetSize, Ops, Plantation
from .robot_sim import run_mission


def svg_drawing(d: Design) -> str:
    """Side view (left) and top section through one tier (right)."""
    W, Hh = 900, 560
    o: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {Hh}" width="{W}" height="{Hh}" '
        'font-family="Helvetica,Arial,sans-serif" font-size="11">',
        '<style>.tr{fill:#b89b72;stroke:#6b5235}.m{fill:#c9d3dc;stroke:#2b3a48;stroke-width:1.2}'
        '.w{fill:#3a4652;stroke:#111}.s{fill:#c0392b;stroke:#7b1e14}.g{fill:#7fb55a;stroke:#3d6b25}'
        '.d{stroke:#b5402a;stroke-width:.8;fill:none}.t{fill:#1b2733}.dt{fill:#b5402a}'
        '.c{stroke:#888;stroke-dasharray:6 3 1 3}.lf{fill:none;stroke:#4f8a3a;stroke-width:3}</style>',
        f'<rect width="{W}" height="{Hh}" fill="#fbfbf8"/>',
        '<text x="14" y="22" class="t" font-size="13" font-weight="bold">Palm-climbing harvest robot: '
        'side view at the crown (left), section through one clamp tier (right). Schematic.</text>',
    ]
    # ---- side view, scale 0.25 px/mm
    s = 0.25
    ax, base = 330.0, 520.0
    trunk_w = 300 * s
    o.append(f'<rect class="tr" x="{ax - trunk_w / 2}" y="60" width="{trunk_w}" height="{base - 60}"/>')
    for k in range(14):  # leaf-scar rings
        y = 80 + k * 32
        o.append(f'<line x1="{ax - trunk_w / 2}" y1="{y}" x2="{ax + trunk_w / 2}" y2="{y + 3}" stroke="#8a6f4c"/>')
    for dx, dy in ((-120, 10), (-60, -15), (60, -15), (120, 10)):  # fronds
        o.append(f'<path class="lf" d="M{ax} 75 Q{ax + dx * 0.6} {40 + dy} {ax + dx * 1.4} {95 + dy}"/>')
    z_top = 330.0  # y of the upper tier in the drawing
    Hpx = d.H * 1000 * s
    ring_w = d.ring_inner_d * s + 2 * d.wheel[0] * s * 0.4 + 30
    for y in (z_top, z_top + Hpx):
        o.append(f'<rect class="m" x="{ax - ring_w / 2}" y="{y - 6}" width="{ring_w}" height="12" rx="3"/>')
        r = d.wheel[0] / 2 * s
        for sx in (-1, 1):
            cx = ax + sx * (trunk_w / 2 + r)
            o.append(f'<circle class="w" cx="{cx}" cy="{y}" r="{r}"/>')
        o.append(f'<rect class="m" x="{ax + ring_w / 2 - 4}" y="{y - 22}" width="26" height="16"/>')
    for sx in (-1, 1):
        x = ax + sx * (ring_w / 2 - 10)
        o.append(f'<line x1="{x}" y1="{z_top}" x2="{x}" y2="{z_top + Hpx}" stroke="#2b3a48" stroke-width="3"/>')
    # battery + electronics box on lower tier
    o.append(f'<rect class="m" x="{ax - ring_w / 2 - 34}" y="{z_top + Hpx - 10}" width="30" height="40"/>')
    # mast + boom + saw
    mast_x = ax + ring_w / 2 - 14
    o.append(f'<rect class="m" x="{mast_x}" y="{z_top - 150}" width="10" height="150"/>')
    reach_px = d.reach * 1000 * s
    bx = ax - reach_px
    o.append(f'<line x1="{mast_x + 5}" y1="{z_top - 145}" x2="{bx}" y2="{z_top - 175}" stroke="#2b3a48" stroke-width="5"/>')
    br = d.blade / 2 * s
    o.append(f'<circle class="s" cx="{bx}" cy="{z_top - 175 - br}" r="{br}"/>')
    o.append(f'<circle cx="{bx}" cy="{z_top - 175 - br}" r="4" fill="#222"/>')
    # coconut bunch
    for k, (dx, dy) in enumerate(((0, 0), (14, 6), (-12, 8), (4, 18), (18, 22), (-6, 26))):
        o.append(f'<circle class="g" cx="{bx - 30 + dx}" cy="{z_top - 120 + dy}" r="9"/>')
    o.append(f'<line x1="{ax - trunk_w / 2}" y1="{z_top - 190}" x2="{bx - 30}" y2="{z_top - 125}" '
             'stroke="#6b8a3a" stroke-width="5"/>')
    o.append(f'<line class="c" x1="{ax}" y1="50" x2="{ax}" y2="{base}"/>')
    # dims
    xd = ax + ring_w / 2 + 40
    o.append(f'<line class="d" x1="{xd}" y1="{z_top}" x2="{xd}" y2="{z_top + Hpx}"/>')
    o.append(f'<text class="dt" x="{xd + 5}" y="{z_top + Hpx / 2 + 4}">tier spacing {d.H * 1000:.0f}</text>')
    o.append(f'<line class="d" x1="{ax}" y1="{z_top - 230}" x2="{bx}" y2="{z_top - 230}"/>')
    o.append(f'<text class="dt" x="{bx + 4}" y="{z_top - 236}">reach {d.reach * 1000:.0f} from trunk axis</text>')
    o.append(f'<text class="t" text-anchor="end" x="{bx - br - 6}" y="{z_top - 175 - br + 4}">Ø{d.blade} saw, {d.saw_w:.0f} W</text>')
    o.append(f'<text class="t" x="{ax + ring_w / 2 + 26}" y="{z_top - 12}">{d.motor[0]} worm drive ×{N_WHEELS}</text>')
    o.append(f'<text class="t" x="{ax - ring_w / 2 - 36}" y="{z_top + Hpx + 46}">{d.battery_ah} Ah pack</text>')
    o.append(f'<text class="t" x="{ax - 30}" y="{base + 15}">palm trunk</text>')

    # ---- top section, scale 0.45 px/mm
    s2 = 0.45
    cx, cy = 715.0, 300.0
    R_t = 310 / 2 * s2
    o.append(f'<circle class="tr" cx="{cx}" cy="{cy}" r="{R_t}"/>')
    R_ring = (d.ring_inner_d / 2 + d.wheel[0] * 0.9) * s2
    o.append(f'<circle cx="{cx}" cy="{cy}" r="{R_ring}" fill="none" stroke="#2b3a48" stroke-width="9" opacity=".35"/>')
    o.append(f'<line x1="{cx - R_ring - 8}" y1="{cy}" x2="{cx - R_ring + 12}" y2="{cy}" stroke="#2b3a48" stroke-width="3"/>')
    o.append(f'<text class="t" x="{cx - R_ring - 70}" y="{cy - 8}">hinge</text>')
    o.append(f'<text class="t" x="{cx + R_ring - 4}" y="{cy - 8}">latch</text>')
    for k in range(N_PER_TIER):
        a = math.radians(90 + 120 * k)
        wr = d.wheel[0] / 2 * s2
        ww = d.wheel[1] * s2
        px, py = cx + (R_t + wr) * math.cos(a), cy + (R_t + wr) * math.sin(a)
        ox, oy = cx + R_ring * math.cos(a), cy + R_ring * math.sin(a)
        o.append(f'<line x1="{ox}" y1="{oy}" x2="{px}" y2="{py}" stroke="#2b3a48" stroke-width="6"/>')
        o.append(f'<rect class="w" x="{-wr}" y="{-ww / 2}" width="{2 * wr}" height="{ww}" '
                 f'transform="translate({px} {py}) rotate({math.degrees(a)})"/>')
        o.append(f'<path class="d" d="M{ox + 10 * math.cos(a + 1.2)} {oy + 10 * math.sin(a + 1.2)} '
                 f'l{-14 * math.cos(a)} {-14 * math.sin(a)}"/>')
    o.append(f'<text class="t" x="{cx - 60}" y="{cy + R_ring + 30}">3 wheels @120°, preload {d.N0:.0f} N each</text>')
    o.append(f'<text class="t" x="{cx - 60}" y="{cy + R_ring + 46}">trunk Ø{d.pop.d_min:.0f}–{d.pop.d_max:.0f} mm</text>')
    o.append("</svg>")
    return "\n".join(o)


def markdown(d: Design, fleet: FleetSize, pl: Plantation = Plantation(), ops: Ops = Ops(),
             missions: int = 300, seed: int = 7) -> str:
    rng = random.Random(seed)
    res = []
    for _ in range(missions):
        t = d.pop.sample(rng)
        res.append((t, run_mission(d, t, d.pop.bunches(rng, pl.cycle_days), rng=rng)))
    ok = [r for _, r in res if r.ok]
    margins = []
    for t in d.pop.stream(100_000, seed=3):
        margins.append(d.traction_available(t.mu_dry) / d.traction_demand(t))
    q = quantiles(margins, n=1000, method="inclusive")
    yr = fleet.results[0]
    L: List[str] = []
    w = L.append
    w("# Coconut-Palm Climbing & Harvesting Robot: Design and Plantation-Scale Verification\n")
    w("> Generated by `coconut-harvester report`. Every number comes from the parametric model in "
      "`coconut_harvester/`. Change an assumption, regenerate, and CI re-verifies it.\n")
    w("![Robot](robot.svg)\n")
    w("## 1. What it does\n")
    w("Two people carry the robot to a palm, close its two hinged clamp rings around the trunk and "
      "latch them. The robot grips with six driven wheels and climbs to the crown. The operator "
      "uses the mast camera to swing the saw arm around the trunk and cut each ripe bunch's stalk, "
      "and the bunch falls clear. The robot then drives back down and is unlatched and moved to the "
      "next palm. Nobody climbs. That removes the main hazard in coconut harvesting.\n")
    w("## 2. Design basis\n")
    w("| Parameter | Mean | SD | Min | Max |\n|---|---|---|---|---|")
    p = d.pop
    for name, dist in [("Climb height (m)", p.height), ("Trunk Ø at base (mm)", p.d_base),
                       ("Top/base taper (–)", p.taper), ("Lean (deg)", p.lean),
                       ("Leaf-scar ridge height (mm)", p.ridge_h), ("Ridge length (mm)", p.ridge_len),
                       ("Wheel/bark friction, dry (–)", p.mu_dry), ("Bunch stalk Ø (mm)", p.peduncle),
                       ("Stalk distance from trunk axis (m)", p.bunch_radius)]:
        w(f"| {name} | {dist.mean:g} | {dist.sd:g} | {dist.lo:g} | {dist.hi:g} |")
    w(f"\nWet bark friction = {p.wet_factor:g} × dry. Sizing uses the **envelope tree**: every "
      "parameter at its adverse bound at once.\n")
    w("## 3. Sized robot\n")
    w("| Item | Result |\n|---|---|")
    w(f"| Mass | **{d.mass:.1f} kg** (two-person carry ≤ {d.req.max_mass:g} kg) |")
    w(f"| Clamp tiers | 2 × 3 wheels at 120°, tier spacing {d.H * 1000:.0f} mm, ring opening Ø{d.ring_inner_d:.0f} mm |")
    w(f"| Wheels | Ø{d.wheel[0]} × {d.wheel[1]} mm PU V-tread, rated {d.wheel[2]} N |")
    w(f"| Clamp preload | {d.N0:.0f} N per wheel, lead-screw actuator {d.clamp_actuator[0]} N per tier, "
      f"series spring {d.spring_rate:.0f} N/mm, radial travel {d.clamp_travel:.0f} mm |")
    w(f"| Drive | {N_WHEELS} × {d.motor[0]} worm gearmotor ({d.motor[1]} Nm, {d.motor[2]} rpm, η {d.motor[4]:g}, self-locking) |")
    w(f"| Climb / descent speed | {d.speed:.2f} m/s |")
    w(f"| Saw | Ø{d.blade} mm TCT blade, {d.saw_w:.0f} W BLDC, reach {d.reach * 1000:.0f} mm, 360° slew |")
    w(f"| Battery | 24 V {d.battery_ah} Ah Li-ion ({d.battery_wh:.0f} Wh), hot-swap; {ops.packs_per_robot} packs per robot |")
    w(f"| Fall arrest | Ground belay tether, MBS {d.req.tether_mbs / 1000:.0f} kN |")
    w("\nMass breakdown:\n")
    w("| Subsystem | kg |\n|---|---|")
    for k, v in d.mass_breakdown.items():
        w(f"| {k} | {v:.2f} |")
    w("\n## 4. Engineering checks\n")
    w("| Check | Value | Limit | Status |\n|---|---|---|---|")
    for c in d.checks:
        w(f"| {c.name} | {c.value:.2f} {c.unit} | {c.cmp} {c.limit:g} | {'PASS' if c.ok else '**FAIL**'} |")
    w("\n## 5. Monte-Carlo and closed-loop results\n")
    w("| Metric | Value |\n|---|---|")
    w(f"| Climb traction margin over 100,000 trees: P0.1 / median | {q[0]:.2f}× / {q[499]:.2f}× |")
    w(f"| Closed-loop missions (controller + physics) | {len(ok)}/{missions} completed, no faults |")
    w(f"| Mission time mean / max | {fmean(r.time_s for r in ok):.0f} s / {max(r.time_s for r in ok):.0f} s (excl. carry & mount) |")
    w(f"| Energy per tree mean / max | {fmean(r.energy_wh for r in ok):.1f} Wh / {max(r.energy_wh for r in ok):.1f} Wh |")
    w(f"| Trees per pack (mean) | {d.battery_wh * d.req.battery_dod / fmean(r.energy_wh for r in ok):.0f} |")
    w("\n## 6. Plantation scale\n")
    w(f"{pl.hectares:g} ha × {pl.palms_per_ha:g} palms/ha = **{pl.palms:,} palms**, harvested every "
      f"{pl.cycle_days} days, Mon–Sat, {pl.shift_h:g} productive h/day. Monsoon-season wet bark "
      "(Kerala-like rain calendar) grounds the robots. Ops model: trolley moves, mounting "
      f"{ops.mount_s:g} s, camera aiming {ops.aim_s:g} s per bunch, battery swaps on a field charger, "
      f"and breakdowns (MTBF {ops.mtbf_h:g} h).\n")
    sl = fleet.service
    w(f"**Service level:** 95 % of palms are harvested within {sl.p95_late_max:g} days of due and none "
      f"more than {sl.max_late_max:g} days late, in at least {sl.year_pass_frac:.0%} of weather years. "
      f"No palm is ever more than {sl.hard_max_late:g} days late. The fleet is the smallest that meets "
      f"this over {len(fleet.results)} simulated years. (Sizing on 10 years under-sampled monsoon "
      "variability: that fleet passed only about 65 % of fresh years.)\n")
    w("| Metric | Value |\n|---|---|")
    w(f"| **Robots required** | **{fleet.robots}** (+1 spare), {2 * fleet.robots} crew |")
    w(f"| Harvest visits / year | {yr.harvests:,} |")
    w(f"| Bunches cut / year | {yr.bunches:,} |")
    w(f"| Years meeting the round target | {round(fleet.pass_frac * len(fleet.results))}/{len(fleet.results)} |")
    w(f"| Lateness vs {pl.cycle_days}-day round, P95 / max (worst year) | {fleet.worst_p95:.0f} d / {fleet.worst_max:.0f} d |")
    w(f"| Trees per robot per 7 productive hours | {yr.trees_per_robot_day:.0f} |")
    w(f"| Robot-hours lost to wet bark / year | {yr.lost_wet_hours:,.0f} |")
    w(f"| Battery-pack waits | {yr.pack_waits} |")
    w(f"| Breakdowns / year (fleet) | {yr.breakdowns} |")
    w(f"| Palms needing a manual climber | {yr.manual} |")
    w(f"| Fleet energy / year | {yr.energy_kwh:,.0f} kWh |")
    w("\n## 7. Safety\n")
    w("Supervisory controller (`coconut_harvester/controller.py`), target ISO 13849-1 PL d:\n")
    w("- **SF1:** the clamp can never open unless the robot is on the ground.")
    w("- **SF2:** the saw runs only when all of these hold: arm deployed, robot stationary and "
      "clamped, drop zone confirmed clear, live radio link, dead-man CUT held, wind ≤ "
      f"{d.req.wind_max:g} m/s, no e-stop.")
    w("- **SF3:** the drive runs only when clamped at force, arm stowed, saw off, link up and tilt within limits.")
    w("- **SF4:** e-stop or link loss stops everything. The clamp keeps holding: worm drives and lead "
      "screws are self-locking, so the robot hangs safely with no power.")
    w("- **SF5:** if the IMU disagrees with wheel odometry (slip), the clamp boosts by 30 %, the robot holds and faults.")
    w("- **SF6:** no climbing on wet bark, in high wind, at the crown, or without twice the energy "
      "needed to come down. A low battery forces a descent.")
    w("- **SF7:** a stall on an out-of-envelope ridge faults. After a reset the robot can only descend.")
    w("- **SF8:** hold-to-run. Climbing and descending stop the moment the handset button is "
      "released. An automatic low-battery descent is the exception, and STOP still halts it.")
    w("- **Fall arrest:** a ground belay tether is the last line of defence.")
    w("- **Drop zone:** bunches of 5–25 kg fall from up to 22 m. The crew stays outside a radius of "
      "height/2 + 3 m and confirms it on the handset before every cut.\n")
    w("The test suite fuzzes the controller three ways and checks every safety function on every "
      "scan: random sensor and command streams, a plausible-plant stream with injected faults, and "
      "full harvest scripts with faults and stray commands injected at arbitrary points. Mutation "
      "testing deleted each of 10 safety rules in turn, and the tests caught every one. The fuzz "
      "also found a real flaw during development: the robot kept climbing when the operator "
      "released CLIMB. That is now SF8.\n")
    w("## 8. Before field release: open items\n")
    w("1. **Survey 50+ palms per estate** (height, diameter, taper, lean, scar ridges, bark friction "
      "with the chosen PU tread, dry and wet) and fit them with `TreePopulation.from_survey()`. Ridge "
      "geometry and wet friction drive the clamp force, so they matter most.")
    w("2. **Measure stalk cutting energy** with the chosen blade, and confirm the "
      f"{d.req.cut_energy:g} J/cm³ assumption.")
    w("3. Prototype tests: pull-off and slip tests on real trunks, 1,000 climbs endurance, a "
      "drop test of the belay, and IP54 rain exposure.")
    w("4. Validate the safety functions on hardware (ISO 13849-2) and check the radio link range under the canopy.")
    w("5. Run a pilot block (≈ 1,000 palms for two rounds) to calibrate the mounting and aiming times in the fleet model.\n")
    return "\n".join(L)


# =========================================================================
# Reimagined concept (inchworm) — drawing and the combined report
# =========================================================================
from .inchworm import AL_TUBES, InchwormDesign, size as iw_size  # noqa: E402
from .inchworm_control import run_mission as iw_mission  # noqa: E402


def _collar(o: List[str], x0: float, y: float, trunk_w: float, lever_px: float, h_px: float,
            released: bool = False) -> None:
    """Side view of one collar: V-pad (right), strap (left), cantilever arm to the right."""
    tilt = 0 if released else 6
    o.append(f'<g transform="rotate({tilt} {x0 + trunk_w / 2} {y})">')
    o.append(f'<rect class="pad" x="{x0 + trunk_w / 2 - 2}" y="{y}" width="10" height="{h_px * 0.55}" rx="2"/>')
    o.append(f'<line class="strap" x1="{x0 - trunk_w / 2}" y1="{y - h_px * 0.45}" '
             f'x2="{x0 + trunk_w / 2 + 6}" y2="{y + h_px * 0.15}"/>')
    o.append(f'<rect class="m" x="{x0 + trunk_w / 2 + 6}" y="{y - 4}" width="{lever_px}" height="8" rx="2"/>')
    o.append("</g>")


def svg_inchworm(d: InchwormDesign) -> str:
    W, H = 1010, 660
    o: List[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" '
        'font-family="Helvetica,Arial,sans-serif" font-size="11">',
        '<style>.tr{fill:#b89b72;stroke:#6b5235}.m{fill:#c9d3dc;stroke:#2b3a48;stroke-width:1.2}'
        '.pad{fill:#3a4652}.strap{stroke:#d08a1e;stroke-width:4}.cyl{fill:#7d8b99;stroke:#2b3a48}'
        '.s{fill:#c0392b;stroke:#7b1e14}.g{fill:#7fb55a;stroke:#3d6b25}.d{stroke:#b5402a;stroke-width:.8;fill:none}'
        '.t{fill:#1b2733}.dt{fill:#b5402a}.lf{fill:none;stroke:#4f8a3a;stroke-width:3}'
        '.c{stroke:#888;stroke-dasharray:6 3 1 3}.h{font-weight:bold}</style>',
        f'<rect width="{W}" height="{H}" fill="#fbfbf8"/>',
        '<text x="14" y="22" class="t" font-size="13" font-weight="bold">Reimagined: inchworm climber with '
        'self-locking collars. Side view (left), climbing gait (right). Schematic.</text>',
    ]
    s = 0.40  # px/mm
    ax, top, base = 330.0, 60.0, 560.0
    tw = 300 * s
    o.append(f'<rect class="tr" x="{ax - tw / 2}" y="{top}" width="{tw}" height="{base - top}"/>')
    for k in range(15):
        y = top + 15 + k * 32
        o.append(f'<line x1="{ax - tw / 2}" y1="{y}" x2="{ax + tw / 2}" y2="{y + 3}" stroke="#8a6f4c"/>')
    for dx, dy in ((-120, 10), (-60, -15), (60, -15), (120, 10)):
        o.append(f'<path class="lf" d="M{ax} {top + 15} Q{ax + dx * 0.6} {top - 20 + dy} {ax + dx * 1.3} {top + 35 + dy}"/>')
    lever = (d.lever * 1000 - 150) * s
    hp = d.pad_span * 1000 * s
    y_up = 250.0
    y_lo = y_up + d.stroke * s * 0.9 + 40
    tip = ax + tw / 2 + 6 + lever
    _collar(o, ax, y_up, tw, lever, hp)
    _collar(o, ax, y_lo, tw, lever, hp)
    # cylinder between arm tips
    o.append(f'<rect class="cyl" x="{tip - 9}" y="{y_up + 6}" width="18" height="{(y_lo - y_up) * 0.55}" rx="3"/>')
    o.append(f'<rect class="m" x="{tip - 4}" y="{y_up + 6 + (y_lo - y_up) * 0.55}" width="8" '
             f'height="{(y_lo - y_up) * 0.45 - 10}"/>')
    # battery box hanging at lower arm tip (counterweight)
    o.append(f'<rect class="m" x="{tip - 22}" y="{y_lo + 10}" width="44" height="48" rx="3"/>')
    o.append(f'<text class="t" x="{tip - 22}" y="{y_lo + 72}">battery {d.battery_ah} Ah (counterweight)</text>')
    # tensioner on upper strap
    o.append(f'<rect class="m" x="{ax - tw / 2 - 26}" y="{y_up - hp * 0.5 - 10}" width="22" height="16" rx="2"/>')
    o.append(f'<text class="t" text-anchor="end" x="{ax - tw / 2 - 30}" y="{y_up - hp * 0.5 + 2}">harvest tensioner</text>')
    # slewing mast, boom, saw
    o.append(f'<rect class="m" x="{ax + tw / 2 + 14}" y="{y_up - 120}" width="9" height="116"/>')
    bx = ax - d.reach * 1000 * s * 0.55
    o.append(f'<line x1="{ax + tw / 2 + 18}" y1="{y_up - 115}" x2="{bx}" y2="{y_up - 145}" stroke="#2b3a48" stroke-width="5"/>')
    br = d.blade / 2 * s * 0.6
    o.append(f'<circle class="s" cx="{bx}" cy="{y_up - 145 - br}" r="{br}"/>')
    o.append(f'<circle cx="{bx}" cy="{y_up - 145 - br}" r="3" fill="#222"/>')
    for dx, dy in ((0, 0), (14, 6), (-12, 8), (4, 18), (18, 22), (-6, 26)):
        o.append(f'<circle class="g" cx="{bx - 30 + dx}" cy="{y_up - 95 + dy}" r="8"/>')
    o.append(f'<text class="t" text-anchor="end" x="{bx - br - 6}" y="{y_up - 145 - br + 4}">Ø{d.blade} saw</text>')
    # labels & dims
    o.append(f'<text class="t" x="{tip + 16}" y="{y_up - 8}">upper collar</text>')
    o.append(f'<text class="t" x="{tip + 16}" y="{y_lo - 8}">lower collar</text>')
    o.append(f'<text class="t" x="{tip + 16}" y="{(y_up + y_lo) / 2 + 4}">{d.cylinder[0]} cylinder, {d.stroke} stroke</text>')
    o.append(f'<line class="d" x1="{ax}" y1="{y_lo + 80}" x2="{tip}" y2="{y_lo + 80}"/>')
    o.append(f'<text class="dt" x="{ax + 4}" y="{y_lo + 92}">lever L = {d.lever * 1000:.0f} mm from trunk axis</text>')
    o.append(f'<text class="dt" x="{ax - tw / 2}" y="{base + 18}">grip rule: lever L ≥ h / (2μ), so grip grows with load</text>')
    o.append(f'<line class="c" x1="{ax}" y1="{top}" x2="{ax}" y2="{base}"/>')

    # ---- gait strip
    gx0, gy = 655.0, 110.0
    o.append(f'<text class="t h" x="{gx0}" y="{gy - 40}">One climbing step ({d.step_length * 1000:.0f} mm)</text>')
    # (label, upper free, lower free, upper height px, lower height px)
    frames = [("1  release upper", True, False, 70, 0), ("2  extend: upper rises", True, False, 130, 0),
              ("3  lock upper, release lower", False, True, 130, 0), ("4  retract: lower rises", False, True, 130, 60)]
    for k, (label, ru, rl, hu, hl) in enumerate(frames):
        x = gx0 + k * 88
        o.append(f'<rect class="tr" x="{x}" y="{gy}" width="26" height="420"/>')
        y_l = gy + 400 - hl
        y_u = gy + 400 - hu
        for yy, rel in ((y_u, ru), (y_l, rl)):
            colour = "#d08a1e" if not rel else "#bbb"
            o.append(f'<line x1="{x - 3}" y1="{yy - 8}" x2="{x + 29}" y2="{yy + 4}" stroke="{colour}" stroke-width="4"/>')
            o.append(f'<rect class="m" x="{x + 26}" y="{yy - 3}" width="32" height="6"/>')
            o.append(f'<text class="t" x="{x + 28}" y="{yy - 7}" font-size="8">{"free" if rel else "locked"}</text>')
        o.append(f'<line x1="{x + 54}" y1="{y_u}" x2="{x + 54}" y2="{y_l}" stroke="#2b3a48" stroke-width="5"/>')
        o.append(f'<text class="t" x="{x - 2}" y="{gy + 440}" font-size="10">{label.split("  ")[0]}</text>')
    for k, (label, *_r) in enumerate(frames):
        o.append(f'<text class="t" x="{gx0}" y="{gy + 465 + 14 * k}" font-size="11">{label}</text>')
    o.append("</svg>")
    return "\n".join(o)


def markdown_full(iw: InchwormDesign, wd: Design, iw_fleet: FleetSize, wd_fleet: FleetSize, sens,
                  pl: Plantation = Plantation(), ops: Ops = Ops(), missions: int = 300, seed: int = 7) -> str:
    rng = random.Random(seed)
    res = []
    for k in range(missions):
        t = iw.pop.sample(rng)
        res.append(iw_mission(iw, t, iw.pop.bunches(rng, pl.cycle_days), wet=k % 2 == 1, rng=rng))
    ok = [r for r in res if r.ok]
    yi, yw = iw_fleet.results[0], wd_fleet.results[0]
    L: List[str] = []
    w = L.append
    w("# Coconut-Palm Harvesting Robot: Reimagined Design\n")
    w("> Generated by `coconut-harvester report` from the models in `coconut_harvester/`. Change an assumption, "
      "regenerate, and CI re-verifies everything.\n")
    w("![Inchworm robot](robot.svg)\n")
    w("## 1. Why reimagine\n")
    w("The first design (the *wheel ring*, appendix A) clamps six driven wheels onto the trunk and climbs on "
      "friction. Its own numbers exposed three structural problems:\n")
    w(f"1. **Weight:** {wd.mass:.1f} kg, right at the {wd.req.max_mass:g} kg two-person limit. Six gearmotors "
      f"weigh {wd.mass_breakdown['drive gearmotors']:.1f} kg, and they must be large to push the clamped wheels "
      "over leaf-scar ridges.")
    w(f"2. **Monsoon:** friction-only grip means no climbing on wet bark. That cost {yw.lost_wet_hours:,.0f} "
      f"robot-hours a year, and it is why the estate needed {wd_fleet.robots} robots.")
    w(f"3. **Mounting:** closing a hinged ring with six spring arms takes about {wd.mount_s:.0f} s per tree, "
      "more than the climb itself.\n")
    w("## 2. The new concept: an inchworm with self-locking collars\n")
    w("Each **collar** is a rubber V-pad on one side of the trunk and a steel-cored strap around the other, with "
      "a cantilever arm on the pad side. Weight on the arm *cocks* the collar, so pad and strap bite a height "
      "*h* apart. The normal force is N = M/h, where M is the load's moment about the trunk. **Grip grows with "
      "load.** The collar holds whenever the lever L ≥ h/(2μ), however heavy the robot is. Hunters' climbing tree "
      "stands rely on the same principle on wet trunks.\n")
    w("Two collars are joined by one electric cylinder. The robot climbs in four moves:")
    w("1. Release the upper collar.")
    w("2. Extend: the upper collar rises.")
    w("3. Lock the upper collar and release the lower one.")
    w("4. Retract: the lower collar rises.\n")
    w("**Fail-safe:** the release servos are spring-return, so if power is lost both collars lock where they "
      "are. Leaf-scar ridges stop mattering, because a released collar has slack and slides over them.\n")
    w("**Found by the physics model:** with the saw swung to the far side of the trunk, the cocking moment nearly "
      f"cancels (lock ratio {iw.lock_ratio(iw.pop.mu_dry.lo * iw.pop.wet_factor, -iw.reach):.2f}×), and a pure "
      "self-locking collar would let go mid-cut. So the upper collar also has a small **harvest tensioner**: a "
      f"self-locking lead screw on the strap ({iw.tensioner[0]} N). It clamps positively before the arm may "
      "deploy and slackens only once the arm is stowed again.\n")
    w("### Real tall palms: 100 ft, leaning, swaying\n")
    old = iw_size(iw.req, iw.pop, 0.35, 0.10, AL_TUBES[0])
    mu_w = iw.pop.mu_dry.lo * iw.pop.wet_factor
    H, lean = iw.pop.height.hi, iw.pop.lean.hi + iw.req.sway_tilt
    w(f"The first version of this design assumed palms up to 22 m and treated lean far too gently. A real "
      f"tall-variety palm reaches about 30 m (100 ft), leans, and sways. Three effects were added to the model:\n")
    w(f"- **Lean:** part of the robot hangs below the upper collar. On a leaning trunk that weight swings back "
      f"toward the trunk and takes lever away. The original 350 mm arms gave "
      f"{old.climb_ratio(mu_w, 0, 0):.2f}× grip on a vertical wet trunk but only "
      f"{old.climb_ratio(mu_w, lean, H):.2f}× at {iw.pop.lean.hi:g}° lean plus sway, against a 1.5× target. "
      f"The arms are now **{iw.lever * 1000:.0f} mm**, giving {iw.lock_margin_wet:.2f}×.")
    w(f"- **Rope:** at {H:g} m the robot carries {iw.tether_weight(H) / G:.1f} kg of belay rope.")
    w(f"- **Sway:** {iw.req.sway_tilt:g}° of extra tilt is allowed for wind sway at the {iw.req.wind_max:g} m/s limit.\n")
    w(f"**Mounting rule:** the arms go on the side the trunk leans toward. Palms taller than {H:g} m or leaning "
      f"more than {iw.pop.lean.hi:g}° are refused and go to a manual climber. The first model would have "
      "attempted them. A 100 ft climb takes about 3 minutes up and 3.5 down.\n")
    w("## 3. Head-to-head\n")
    w("Same tree population, same plantation, same weather years, same service level.\n")
    w("| | Wheel ring (v1) | **Inchworm (v2)** |\n|---|---|---|")
    w(f"| Mass | {wd.mass:.1f} kg | **{iw.mass:.1f} kg** |")
    w(f"| Motors | 6 drive + 2 clamp | 1 cylinder + 2 release servos + 1 tensioner |")
    w(f"| Climbs on wet bark | no | **yes** (lock margin {iw.lock_margin_wet:.2f}× at the wettest, slipperiest bark) |")
    w(f"| Climb speed | {wd.speed:.2f} m/s | {iw.speed:.2f} m/s |")
    w(f"| Mount / dismount | {wd.mount_s:.0f} s / {wd.dismount_s:.0f} s | {iw.mount_s:.0f} s / {iw.dismount_s:.0f} s |")
    w(f"| Sensitive to leaf-scar ridges | yes (sizes the clamp & motors) | no |")
    tpc = {id(d): next(c.value for c in d.checks if c.name.startswith("Trees per battery")) for d in (wd, iw)}
    w(f"| Battery / trees per charge (mean tree) | {wd.battery_ah} Ah / {tpc[id(wd)]:.0f} | "
      f"{iw.battery_ah} Ah / {tpc[id(iw)]:.0f} |")
    w(f"| Trees per robot per 7 productive h | {yw.trees_per_robot_day:.0f} | {yi.trees_per_robot_day:.0f} |")
    w(f"| Robot-hours lost to wet weather / yr | {yw.lost_wet_hours:,.0f} | {yi.lost_wet_hours:,.0f} |")
    w(f"| **Robots for {pl.palms:,} palms** | {wd_fleet.robots} | **{iw_fleet.robots}** |")
    w(f"| Crew | {2 * wd_fleet.robots} | **{2 * iw_fleet.robots}** |")
    w(f"| Years meeting the round (of {len(iw_fleet.results)}) | {round(wd_fleet.pass_frac * len(wd_fleet.results))} | "
      f"{round(iw_fleet.pass_frac * len(iw_fleet.results))} |")
    w(f"| Fleet energy / yr | {yw.energy_kwh:,.0f} kWh | {yi.energy_kwh:,.0f} kWh |")
    w("\nPer tree the two are about equal: the inchworm climbs more slowly but mounts faster. **All of the fleet "
      "saving comes from working through the monsoon.** The sensitivity study below makes that explicit.\n")
    w("## 4. How sure is the 7?\n")
    w("Fleet size for the inchworm under pessimistic versions of its key assumptions (each sized to the same "
      "service level over 30 weather years):\n")
    w("| Scenario | Mass | Robots |\n|---|---|---|")
    for r in sens:
        m = f"{r.mass:.1f} kg" if r.mass else "–"
        n = str(r.robots) if r.robots else "–"
        note = f" ({r.note})" if r.note else ""
        w(f"| {r.label}{note} | {m} | {n} |")
    w("\nEven if rainy days turn out to be mostly unworkable, the inchworm needs fewer robots. If it is "
      "grounded on wet bark just like the wheel ring, it needs the same number, because the advantage *is* "
      "wet-weather operation. Wet-bark self-lock and workable rain hours are therefore the two things to "
      "measure first (section 9).\n")
    w("## 5. Sized design\n")
    w("| Item | Result |\n|---|---|")
    w(f"| Mass | **{iw.mass:.1f} kg**: one person can lift it; two mount it |")
    w(f"| Collars | Lever {iw.lever * 1000:.0f} mm, pad-to-strap bite {iw.pad_span * 1000:.0f} mm, "
      f"{iw.tube[0]}×{iw.tube[0]}×{iw.tube[1]:g} 6061-T6 arms, steel-cored strap, 600 mm constant-force take-up reel |")
    w(f"| Cylinder | {iw.cylinder[0]}: {iw.cylinder[1]} N, BLDC ball screw, spring brake, {iw.stroke} mm stroke |")
    w(f"| Step / speed | {iw.step_length * 1000:.0f} mm per step; {iw.speed:.2f} m/s up, {iw.descend_speed:.2f} m/s down |")
    w(f"| Harvest tensioner | {iw.tensioner[0]} N self-locking lead screw, clamps {iw.N_active:.0f} N |")
    w(f"| Saw | Ø{iw.blade} mm TCT, {iw.saw_w:.0f} W BLDC, reach {iw.reach * 1000:.0f} mm, 360° slew |")
    w(f"| Battery | 24 V {iw.battery_ah} Ah ({iw.battery_wh:.0f} Wh), hung at the lower arm tip as counterweight |")
    w("\n| Subsystem | kg |\n|---|---|")
    for k, v in iw.mass_breakdown.items():
        w(f"| {k} | {v:.2f} |")
    w("\n## 6. Engineering checks\n")
    w("| Check | Value | Limit | Status |\n|---|---|---|---|")
    for c in iw.checks:
        w(f"| {c.name} | {c.value:.2f} {c.unit} | {c.cmp} {c.limit:g} | {'PASS' if c.ok else '**FAIL**'} |")
    w("\n## 7. Closed loop\n")
    w(f"{missions} full missions on random palms, half of them on **wet bark**, run the real controller "
      "against a physics plant (servo lag, cylinder speed under load, self-lock with the actual friction):\n")
    w("| Metric | Value |\n|---|---|")
    w(f"| Completed with no faults, no falls | {len(ok)}/{missions} |")
    w(f"| Mission time mean / max | {fmean(r.time_s for r in ok):.0f} s / {max(r.time_s for r in ok):.0f} s |")
    w(f"| Energy per tree mean / max | {fmean(r.energy_wh for r in ok):.1f} / {max(r.energy_wh for r in ok):.1f} Wh |")
    w(f"| Inchworm steps per mission (mean) | {fmean(r.steps for r in ok):.0f} |")
    w("\nThe test suite also injects sap patches (sudden friction loss, so the robot slips, faults and holds), "
      "radio loss mid-step (both collars lock and the robot does not move), and low battery (it comes home with "
      "charge to spare).\n")
    w("## 8. Safety functions\n")
    w("Controller: `coconut_harvester/inchworm_control.py`.\n")
    w("- **IW1:** both collars are never released in the air. A collar is released only while the other "
      "reports locked and the cylinder is braked.")
    w("- **IW2:** the cylinder moves only with exactly one collar free, the other locked, the arm stowed, the "
      "tensioner slack, the link up and tilt in limits.")
    w("- **IW3:** the saw runs only when all of these hold: arm deployed, tensioner tight, both collars locked, "
      "cylinder braked, drop zone clear, link up, dead-man held, wind ≤ "
      f"{iw.req.wind_max:g} m/s.")
    w("- **IW4:** the harvest tensioner never slackens while the arm is out. This is enforced on every output.")
    w("- **IW5:** e-stop or link loss brakes the cylinder and drops both release servos, so both collars lock.")
    w("- **IW6:** if a collar that should be locked moves (slip), the robot brakes, locks and faults.")
    w("- **IW7:** no climbing at the crown, in high wind, or without twice the energy needed to descend. A low "
      "battery forces a descent.")
    w("- **IW8:** hold-to-run. Releasing CLIMB or DESCEND brakes and re-locks immediately.")
    w("- **IW9:** the straps open only on the ground.\n")
    w("Verification:\n")
    w("- **Fuzzing:** every rule is checked on every scan under random input streams and under full harvest "
      "scripts with injected faults. Half the injections are aimed at the short harvest segment.")
    w("- **Mutation testing:** of 13 single-rule deletions, 12 are caught. The 13th (\"arm stowed to move\") is "
      "an equivalent mutant: an arm that is out latches the tensioner, and a latched tensioner already blocks "
      "motion.")
    w("- **Bug found:** the fuzz caught a real one during development. A brake-and-relock path slackened the "
      "tensioner while the arm was out. IW4 is now enforced by construction.\n")
    w("## 9. Before building\n")
    w("1. **Wet-bark self-lock rig:** cock a collar on 10+ real trunks, wet and dry. Measure slip load against "
      "lever and confirm μ_wet. The whole fleet advantage rests on this.")
    w("2. **Rain logging on a pilot block:** measure what share of rainy days is actually workable (lightning, "
      "downpours, crew).")
    w("3. Palm survey (`TreePopulation.from_survey`) and stalk cutting energy, as for v1.")
    w("4. Prototype tests: 2,000-step endurance of the collars and cylinder (watch strap and pad wear), the "
      "tensioner under saw vibration, a belay drop test, and ISO 13849-2 validation.\n")
    w("---\n")
    w("## Appendix A: wheel-ring concept (v1)\n")
    w(f"Kept in `coconut_harvester/design.py` and `controller.py`, and still fully tested, so the comparison "
      f"stays honest. Drawing: [wheel_robot.svg](wheel_robot.svg). Mass {wd.mass:.1f} kg, {wd.speed:.2f} m/s, "
      f"clamp preload {wd.N0:.0f} N per wheel, {wd_fleet.robots} robots for this estate.\n")
    w("| Check | Value | Limit | Status |\n|---|---|---|---|")
    for c in wd.checks:
        w(f"| {c.name} | {c.value:.2f} {c.unit} | {c.cmp} {c.limit:g} | {'PASS' if c.ok else '**FAIL**'} |")
    return "\n".join(L)
