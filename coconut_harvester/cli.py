"""``coconut-harvester report`` / ``coconut-harvester simulate``."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from .design import optimize
from .fleet import Plantation, fleet_needed, simulate_year
from .report import markdown, svg_drawing


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
    a = ap.parse_args(argv)

    d = optimize()
    if not d.ok:
        for c in d.failed():
            print(f"FAIL {c.name}: {c.value:.2f} vs {c.limit}", file=sys.stderr)
        return 1
    pl = Plantation(hectares=a.hectares)
    if a.cmd == "report":
        fleet = fleet_needed(d, pl)
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "robot.svg").write_text(svg_drawing(d))
        (out / "DESIGN.md").write_text(markdown(d, fleet, pl))
        print(f"wrote {out}/DESIGN.md: {d.mass:.1f} kg robot, {fleet.robots} robots for {pl.palms:,} palms")
        return 0
    t0 = time.perf_counter()
    y = simulate_year(d, a.robots, pl, seed=a.seed)
    print(f"{a.robots} robots, {pl.palms:,} palms: {y.harvests:,} harvests, {y.bunches:,} bunches, "
          f"P95 late {y.p95_late:.1f} d, max late {y.max_late:.0f} d, {y.trees_per_robot_day:.0f} trees/robot-day "
          f"({time.perf_counter() - t0:.1f}s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
