"""Command line: ``coconut-cutter report`` / ``coconut-cutter simulate``."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from .design import Requirements, optimize
from .line_sim import simulate_line, stations_needed
from .report import markdown, svg_front_view


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="coconut-cutter")
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("report", help="size the machine and write docs/DESIGN.md + front_view.svg")
    r.add_argument("--out", default="docs")
    r.add_argument("--rate", type=float, default=2000.0, help="target nuts/hour")
    r.add_argument("--replications", type=int, default=20)
    s = sub.add_parser("simulate", help="run the line for a long horizon (e.g. a full year)")
    s.add_argument("--stations", type=int)
    s.add_argument("--days", type=float, default=300)
    s.add_argument("--seed", type=int, default=0)
    a = ap.parse_args(argv)

    if a.cmd == "report":
        d = optimize(Requirements(target_rate_per_h=a.rate))
        if not d.ok:
            for c in d.failed():
                print(f"FAIL: {c.name}: {c.value:.2f} vs {c.limit}", file=sys.stderr)
            return 1
        cap = stations_needed(d, a.rate, d.req.hours_per_day, a.replications)
        out = Path(a.out)
        out.mkdir(parents=True, exist_ok=True)
        (out / "front_view.svg").write_text(svg_front_view(d))
        (out / "DESIGN.md").write_text(markdown(d, cap))
        print(f"wrote {out / 'DESIGN.md'}: Ø{d.bore} cylinder, crown Ø{d.punch.diameter:g}, "
              f"{cap.stations} stations, P05 {cap.p05_rate:.0f}/h")
        return 0

    d = optimize()
    n = a.stations or stations_needed(d, d.req.target_rate_per_h, d.req.hours_per_day, 10).stations
    hours = a.days * d.req.hours_per_day
    t0 = time.perf_counter()
    res = simulate_line(d, n, hours, seed=a.seed)
    dt = time.perf_counter() - t0
    print(f"{n} stations, {a.days:g} days: {res.nuts:,} nuts, {res.rate_per_h:.0f}/h, "
          f"availability {res.availability:.3f}, jams {res.jams}, stalls {res.stalls} "
          f"({res.nuts / dt:,.0f} nuts/s simulated)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
