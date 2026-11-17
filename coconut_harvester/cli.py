"""``coconut-harvester report`` / ``coconut-harvester simulate``."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from .compare import sensitivity
from .design import optimize
from .fleet import Plantation, fleet_needed, simulate_year
from .inchworm import optimize as optimize_iw
from .report import markdown_full, svg_drawing, svg_inchworm


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="coconut-harvester")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("report", help="size robot + fleet, write docs/DESIGN.md and docs/robot.svg")
    r.add_argument("--out", default="docs")
    r.add_argument("--hectares", type=float, default=100.0)
    s = sub.add_parser("simulate", help="simulate one plantation year")
    s.add_argument("--robots", type=int, required=True)
    s.add_argument("--hectares", type=float, default=100.0)
    s.add_argument("--seed", type=int, default=0)
    s.add_argument("--concept", choices=["inchworm", "wheel"], default="inchworm")
    a = ap.parse_args(argv)

    pl = Plantation(hectares=a.hectares)
    if a.cmd == "report":
        wd, iw = optimize(), optimize_iw()
        for d in (wd, iw):
            if not d.ok:
                for c in d.failed():
                    print(f"FAIL [{d.concept}] {c.name}: {c.value:.2f} vs {c.limit}", file=sys.stderr)
                return 1
        wf, iwf = fleet_needed(wd, pl), fleet_needed(iw, pl)
        sens = sensitivity(iw, pl)
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "robot.svg").write_text(svg_inchworm(iw))
        (out / "wheel_robot.svg").write_text(svg_drawing(wd))
        (out / "DESIGN.md").write_text(markdown_full(iw, wd, iwf, wf, sens, pl))
        print(f"wrote {out}/DESIGN.md: inchworm {iw.mass:.1f} kg x {iwf.robots} robots "
              f"(wheel ring {wd.mass:.1f} kg x {wf.robots}) for {pl.palms:,} palms")
        return 0
    d = optimize_iw() if a.concept == "inchworm" else optimize()
    t0 = time.perf_counter()
    y = simulate_year(d, a.robots, pl, seed=a.seed)
    print(f"[{d.concept}] {a.robots} robots, {pl.palms:,} palms: {y.harvests:,} harvests, {y.bunches:,} bunches, "
          f"P95 late {y.p95_late:.1f} d, max late {y.max_late:.0f} d, {y.trees_per_robot_day:.0f} trees/robot-day "
          f"({time.perf_counter() - t0:.1f}s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
