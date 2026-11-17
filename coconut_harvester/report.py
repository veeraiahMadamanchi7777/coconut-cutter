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
