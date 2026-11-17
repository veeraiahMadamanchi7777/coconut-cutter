"""Station control logic (what goes into the safety PLC / safety relay + PLC).

Hardware it drives:
    * 5/3 closed-centre valve: ``extend`` and ``retract`` solenoids
      (both off = cylinder held, no motion).
    * Dump/soft-start valve ``air_on`` on the safety channel: off = all
      pressure exhausted, punch can be pushed up by hand.

Inputs are sampled each scan. Safety functions (ISO 13849 PL d target):
    SF1  Guard open / light curtain broken  -> stop motion (closed centre).
    SF2  E-stop                            -> dump air (STO-equivalent).
    SF3  Low supply pressure               -> stop, fault.
    SF4  Anti-tie-down: start must be released and re-pressed per cycle.
    SF5  Restart only via deliberate reset (no auto-restart after a stop).
The scan logic is deterministic and pure so it can be exhaustively fuzzed.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


class State(Enum):
    IDLE = auto()
    EXTENDING = auto()
    DWELL = auto()
    RETRACTING = auto()
    STOPPED = auto()      # safety stop, waiting for reset
    FAULT = auto()        # jam / timeout / pressure, waiting for reset
    ESTOP = auto()
    BLADE_DUE = auto()    # crown swap interval reached


@dataclass(frozen=True)
class Inputs:
    guard_closed: bool = True
    curtain_clear: bool = True
    estop_ok: bool = True
    pressure_ok: bool = True
    nut_present: bool = False
    at_home: bool = True
    at_depth: bool = False
    start: bool = False
    reset: bool = False
    blade_swapped: bool = False


@dataclass(frozen=True)
class Outputs:
    extend: bool = False
    retract: bool = False
    air_on: bool = False


@dataclass
class Config:
    extend_timeout: float = 1.5   # s; > slowest legitimate travel / speed
    retract_timeout: float = 1.5
    dwell: float = 0.10
    blade_interval: int = 8000


@dataclass
class Controller:
    cfg: Config = field(default_factory=Config)
    state: State = State.IDLE
    timer: float = 0.0
    cuts_since_swap: int = 0
    total_cuts: int = 0
    faults: int = 0
    _start_latched: bool = False   # start seen pressed; must release before next cycle

    def _safe_to_move(self, i: Inputs) -> bool:
        return i.guard_closed and i.curtain_clear and i.estop_ok and i.pressure_ok

    def step(self, i: Inputs, dt: float) -> Outputs:
        start_edge = i.start and not self._start_latched
        self._start_latched = i.start

        # SF2 has absolute priority.
        if not i.estop_ok:
            self.state = State.ESTOP
            self.timer = 0.0
            return Outputs(air_on=False)

        s = self.state
        if s in (State.EXTENDING, State.DWELL, State.RETRACTING):
            if not (i.guard_closed and i.curtain_clear):
                self.state, self.timer = State.STOPPED, 0.0
                return Outputs(air_on=True)
            if not i.pressure_ok:
                self.state, self.timer = State.FAULT, 0.0
                self.faults += 1
                return Outputs(air_on=True)

        self.timer += dt

        if s == State.ESTOP:
            # Recover only with a deliberate reset; then return home.
            if i.reset:
                self.state, self.timer = State.RETRACTING, 0.0
            return Outputs(air_on=False)

        if s in (State.STOPPED, State.FAULT):
            if i.reset and self._safe_to_move(i):
                self.state, self.timer = State.RETRACTING, 0.0
                return Outputs(retract=True, air_on=True)
            return Outputs(air_on=True)

        if s == State.BLADE_DUE:
            if i.blade_swapped and i.reset:
                self.cuts_since_swap = 0
                self.state, self.timer = State.IDLE, 0.0
            return Outputs(air_on=True)

        if s == State.IDLE:
            if (start_edge and i.nut_present and i.at_home and self._safe_to_move(i)):
                self.state, self.timer = State.EXTENDING, 0.0
                return Outputs(extend=True, air_on=True)
            return Outputs(air_on=True)

        if s == State.EXTENDING:
            if i.at_depth:
                self.state, self.timer = State.DWELL, 0.0
                return Outputs(air_on=True)
            if not i.nut_present:
                # Nut vanished mid-stroke (knocked out of cup): abort and go home.
                self.state, self.timer = State.RETRACTING, 0.0
                return Outputs(retract=True, air_on=True)
            if self.timer > self.cfg.extend_timeout:
                self.state, self.timer = State.FAULT, 0.0   # jam: stalled on a hard nut
                self.faults += 1
                return Outputs(air_on=True)
            return Outputs(extend=True, air_on=True)

        if s == State.DWELL:
            if self.timer >= self.cfg.dwell:
                self.cuts_since_swap += 1
                self.total_cuts += 1
                self.state, self.timer = State.RETRACTING, 0.0
                return Outputs(retract=True, air_on=True)
            return Outputs(air_on=True)

        if s == State.RETRACTING:
            if i.at_home:
                due = self.cuts_since_swap >= self.cfg.blade_interval
                self.state, self.timer = (State.BLADE_DUE if due else State.IDLE), 0.0
                return Outputs(air_on=True)
            if self.timer > self.cfg.retract_timeout:
                self.state, self.timer = State.FAULT, 0.0
                self.faults += 1
                return Outputs(air_on=True)
            return Outputs(retract=True, air_on=True)

        raise AssertionError(f"unhandled state {s}")  # pragma: no cover
