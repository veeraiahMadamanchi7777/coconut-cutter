"""Head-to-head comparison of the two concepts and sensitivity of the reimagined one."""
from __future__ import annotations

import copy
from dataclasses import dataclass, replace
from typing import List, Optional

from . import design as wheel
from . import inchworm
from .fleet import FleetSize, Ops, Plantation, fleet_needed
from .tree import TALL_ESTATE, TreePopulation, Weather


@dataclass
class Row:
    label: str
    mass: Optional[float]
    robots: Optional[int]
    note: str = ""


def sensitivity(iw: inchworm.InchwormDesign, pl: Plantation = Plantation()) -> List[Row]:
    """How many inchworm robots under pessimistic versions of its key assumptions."""
    rows: List[Row] = []

    def run(label: str, d, ops: Ops = Ops(), weather: Weather = Weather(), note: str = "") -> None:
        f = fleet_needed(d, pl, ops, weather)
        rows.append(Row(label, d.mass, f.robots, note))

    run("Baseline assumptions", iw)
    run("Wet days only 20–50 % workable (vs 50–90 %)", iw, weather=Weather(wet_ok_fraction=(0.2, 0.5)))
    run("Mounting as slow as the wheel ring (100 s / 45 s)", iw, ops=Ops(mount_s=100.0, dismount_s=45.0))
    try:
        slick = inchworm.optimize(pop=replace(TreePopulation(), wet_factor=0.45))
        run("Wet bark 25 % slicker (μ_wet = 0.45 × dry)", slick, note="re-optimised design")
    except RuntimeError:
        rows.append(Row("Wet bark 25 % slicker (μ_wet = 0.45 × dry)", None, None, "no design passes"))
    tall = inchworm.optimize(iw.req, TALL_ESTATE)
    run("Old tall estate: most palms 20–28 m, tallest 31 m (100 ft)", tall, note="same robot")
    grounded = copy.copy(iw)
    grounded.climbs_wet = False
    run("Worst case: grounded on wet bark like the wheel robot", grounded,
        note="isolates the monsoon effect")
    return rows
