"""On-robot supervisory controller (runs on the safety MCU; the handset only *requests*).

Safety functions (target ISO 13849-1 PL d for SF1–SF4):

    SF1  Never release the clamp unless the robot is on the ground.
    SF2  Saw runs only with: arm deployed, robot stationary and clamped,
         drop zone confirmed clear, live radio link, dead-man CUT held,
         wind within limits, no e-stop.
    SF3  Drive only when clamped at force, arm stowed, saw off, link up,
         tilt within limits.
    SF4  E-stop or link loss -> stop drive, saw and arm; clamp holds
         (worm drives + self-locking clamp screws hold with zero power).
    SF5  Slip (IMU disagrees with wheel odometry) -> boost clamp, hold, fault.
    SF6  No climbing on wet bark, in high wind, at the crown, or without
         enough battery to come back down (2x reserve); low battery forces
         a descent.
    SF7  Stall on an out-of-envelope ridge -> fault; only descent allowed after.
    SF8  Hold-to-run: climbing/descending stops the moment the button is released
         (automatic low-battery descent excepted; STOP still halts it).

``step`` is pure apart from its own state, so it can be fuzzed exhaustively.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto


class State(Enum):
    GROUND = auto()       # on the ground; clamp open or closing
    CLAMPED = auto()      # gripping the trunk, stationary
    CLIMBING = auto()
    DESCENDING = auto()
    HARVEST = auto()      # arm deploying / deployed / stowing; saw may run
    FAULT = auto()
    ESTOP = auto()


class Cmd(Enum):
    NONE = auto()
    CLAMP = auto()
    CLIMB = auto()
    STOP = auto()
    DESCEND = auto()
    DEPLOY_ARM = auto()
    STOW_ARM = auto()
    CUT = auto()          # dead-man: must be held every scan
    RELEASE = auto()
    RESET = auto()


@dataclass(frozen=True)
class Inputs:
    cmd: Cmd = Cmd.NONE
    estop: bool = False
    comms_ok: bool = True
    clamp_force: float = 0.0     # N, minimum over all wheel load cells
    altitude: float = 0.0        # m, wheel odometry
    enc_vel: float = 0.0         # m/s, wheel odometry
    imu_vel: float = 0.0         # m/s, IMU/barometer fused
    tilt: float = 0.0            # deg, robot axis from vertical
    battery_wh: float = 250.0
    arm_stowed: bool = True
    arm_deployed: bool = False
    bark_wet: bool = False
    zone_clear: bool = False     # ground crew outside drop zone (confirmed on handset)
    at_crown: bool = False       # crown proximity sensor
    landed: bool = True          # bottom bumper switch
    wind: float = 0.0            # m/s, anemometer on the mast


@dataclass(frozen=True)
class Outputs:
    drive: int = 0               # -1 down, 0 hold, +1 up
    clamp_target: float = 0.0    # N per wheel; 0 = open
    saw: bool = False
    arm: int = 0                 # -1 stow, 0 hold, +1 deploy


@dataclass
class Config:
    N0: float = 530.0                # N clamp preload per wheel (from design)
    clamp_ok_frac: float = 0.85      # clamp considered made above this fraction of N0
    slip_boost: float = 1.3
    slip_tol: float = 0.08           # m/s IMU vs odometry disagreement
    stall_time: float = 3.0          # s commanded drive with no motion
    stall_vel: float = 0.02          # m/s
    tilt_max: float = 28.0           # deg (max lean 20° + sway)
    wind_max: float = 10.0
    min_harvest_alt: float = 2.0     # m, arm may not deploy near the ground crew
    landed_alt: float = 0.05
    reserve_wh_base: float = 2.0     # descent reserve = base + per_m * altitude
    reserve_wh_per_m: float = 0.5

    def reserve(self, alt: float) -> float:
        return self.reserve_wh_base + self.reserve_wh_per_m * max(0.0, alt)


@dataclass
class Controller:
    cfg: Config = field(default_factory=Config)
    state: State = State.GROUND
    clamp_requested: bool = False
    forced_descent: bool = False
    climb_blocked: bool = False      # set by a stall; cleared on landing
    slip: bool = False
    fault_reason: str = ""
    stall_timer: float = 0.0
    cuts: int = 0
    _cut_was_on: bool = False

    # ------------------------------------------------------------------ helpers
    def _clamped(self, i: Inputs) -> bool:
        return i.clamp_force >= self.cfg.clamp_ok_frac * self.cfg.N0

    def _on_ground(self, i: Inputs) -> bool:
        return i.landed and i.altitude <= self.cfg.landed_alt

    def _hold_target(self) -> float:
        return self.cfg.N0 * (self.cfg.slip_boost if self.slip else 1.0)

    def _move_ok(self, i: Inputs) -> bool:
        return (self._clamped(i) and i.arm_stowed and i.comms_ok and not i.estop
                and abs(i.tilt) <= self.cfg.tilt_max)

    def _climb_ok(self, i: Inputs) -> bool:
        return (self._move_ok(i) and not i.bark_wet and i.wind <= self.cfg.wind_max
                and not i.at_crown and not self.climb_blocked and not self.forced_descent
                and i.battery_wh > self.cfg.reserve(i.altitude))

    def _fault(self, reason: str) -> Outputs:
        self.state, self.fault_reason, self.stall_timer = State.FAULT, reason, 0.0
        return Outputs(clamp_target=self._hold_target())

    # ------------------------------------------------------------------ scan
    def step(self, i: Inputs, dt: float) -> Outputs:
        out = self._step(i, dt)
        cut_now = out.saw
        if cut_now and not self._cut_was_on:
            self.cuts += 1
        self._cut_was_on = cut_now
        return out

    def _step(self, i: Inputs, dt: float) -> Outputs:
        cfg = self.cfg
        in_air = not self._on_ground(i)
        if self._on_ground(i):
            self.climb_blocked = False
            self.forced_descent = False

        # SF4: e-stop has absolute priority. Clamp keeps (at least) its preload.
        if i.estop:
            self.state = State.ESTOP
            return Outputs(clamp_target=self._hold_target() if (in_air or self.clamp_requested) else 0.0)
        if self.state == State.ESTOP:
            if i.cmd == Cmd.RESET and i.comms_ok:
                self.state = State.CLAMPED if self._clamped(i) else State.GROUND
                if self.state == State.GROUND and in_air:
                    return self._fault("clamp below threshold after e-stop")
            return Outputs(clamp_target=self._hold_target() if (in_air or self.clamp_requested) else 0.0)

        # SF5: slip — the robot is moving relative to the trunk in a way the wheels did not command.
        if in_air and self.state not in (State.GROUND,):
            commanded = self.state in (State.CLIMBING, State.DESCENDING)
            if (not commanded and abs(i.imu_vel) > cfg.slip_tol) or \
               (commanded and abs(i.imu_vel - i.enc_vel) > cfg.slip_tol):
                self.slip = True
                return self._fault("slip")

        if i.comms_ok is False and self.state not in (State.GROUND, State.FAULT):
            return self._fault("radio link lost")
        if in_air and abs(i.tilt) > cfg.tilt_max and self.state != State.FAULT:
            return self._fault("tilt")

        # SF6: low battery forces descent.
        if in_air and i.battery_wh <= cfg.reserve(i.altitude):
            self.forced_descent = True

        s = self.state
        hold = Outputs(clamp_target=self._hold_target())

        if s == State.FAULT:
            if i.cmd == Cmd.RESET and i.comms_ok and abs(i.imu_vel) <= cfg.slip_tol:
                self.slip = False
                if self.fault_reason == "stall":
                    self.climb_blocked = True
                self.fault_reason = ""
                self.state = State.CLAMPED if (in_air or self.clamp_requested) else State.GROUND
            return hold

        if s == State.GROUND:
            if i.cmd == Cmd.CLAMP:
                self.clamp_requested = True
            elif i.cmd == Cmd.RELEASE and not in_air:
                self.clamp_requested = False
            if self.clamp_requested and self._clamped(i):
                self.state = State.CLAMPED
            # SF1: whatever the request, never open while off the ground.
            return Outputs(clamp_target=cfg.N0 if (self.clamp_requested or in_air) else 0.0)

        if s == State.CLAMPED:
            if not i.arm_stowed:
                # Arm is out (e.g. after a fault during harvest): only stowing is allowed.
                self.state = State.HARVEST
                return hold
            if self.forced_descent and not self._on_ground(i):
                if i.cmd != Cmd.STOP and self._move_ok(i):
                    self.state, self.stall_timer = State.DESCENDING, 0.0
                    return Outputs(drive=-1, clamp_target=self._hold_target())
                return hold
            if i.cmd == Cmd.CLIMB and self._climb_ok(i):
                self.state, self.stall_timer = State.CLIMBING, 0.0
                return Outputs(drive=+1, clamp_target=self._hold_target())
            if i.cmd == Cmd.DESCEND and in_air and self._move_ok(i):
                self.state, self.stall_timer = State.DESCENDING, 0.0
                return Outputs(drive=-1, clamp_target=self._hold_target())
            if (i.cmd == Cmd.DEPLOY_ARM and i.altitude >= cfg.min_harvest_alt and self._clamped(i)
                    and abs(i.imu_vel) <= cfg.slip_tol):
                self.state = State.HARVEST
                return Outputs(arm=+1, clamp_target=self._hold_target())
            if i.cmd == Cmd.RELEASE and self._on_ground(i) and i.arm_stowed:
                self.clamp_requested = False
                self.state = State.GROUND
                return Outputs(clamp_target=0.0)
            return hold

        if s in (State.CLIMBING, State.DESCENDING):
            up = s == State.CLIMBING
            ok = self._climb_ok(i) if up else (self._move_ok(i) and in_air)
            # SF8 hold-to-run: motion continues only while its button is held
            # (an automatic low-battery descent continues unless STOP is pressed).
            held = i.cmd == (Cmd.CLIMB if up else Cmd.DESCEND) or \
                (not up and self.forced_descent and i.cmd != Cmd.STOP)
            if not held or not ok:
                self.state, self.stall_timer = State.CLAMPED, 0.0
                return hold
            # SF7: stall detection
            if abs(i.enc_vel) < cfg.stall_vel:
                self.stall_timer += dt
                if self.stall_timer >= cfg.stall_time:
                    return self._fault("stall")
            else:
                self.stall_timer = 0.0
            return Outputs(drive=+1 if up else -1, clamp_target=self._hold_target())

        if s == State.HARVEST:
            stowing = i.cmd == Cmd.STOW_ARM or self.forced_descent or not self._clamped(i)
            if stowing:
                if i.arm_stowed:
                    self.state = State.CLAMPED
                    return hold
                return Outputs(arm=-1, clamp_target=self._hold_target())
            if not i.arm_deployed:
                if i.arm_stowed and i.cmd not in (Cmd.DEPLOY_ARM, Cmd.CUT, Cmd.NONE):
                    self.state = State.CLAMPED
                    return hold
                return Outputs(arm=+1, clamp_target=self._hold_target())
            # SF2: saw permissives, re-evaluated every scan (dead-man).
            saw = (i.cmd == Cmd.CUT and i.zone_clear and i.comms_ok and self._clamped(i)
                   and i.wind <= cfg.wind_max and abs(i.imu_vel) <= cfg.slip_tol)
            return Outputs(saw=saw, clamp_target=self._hold_target())

        raise AssertionError(f"unhandled state {s}")  # pragma: no cover
