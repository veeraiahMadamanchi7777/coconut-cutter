"""Inchworm robot: supervisory controller, plant model and mission executive.

Safety functions (target ISO 13849-1 PL d):

    IW1  Never both collars released while off the ground. A collar may be
         released only while the other one reports locked and the cylinder
         is braked. Release servos are spring-return: no power = locked.
    IW2  Cylinder moves only with exactly one collar released, the other
         locked, saw arm stowed, harvest tensioner slack, link up, no e-stop,
         tilt within limits.
    IW3  Saw only with: arm deployed, harvest tensioner engaged and tight,
         both collars locked, cylinder braked, drop zone clear, link up,
         dead-man CUT held, wind within limits, no e-stop.
    IW4  The harvest tensioner is never slackened while the arm is out.
    IW5  E-stop / link loss: brake the cylinder, drop both release servos
         (both collars lock), saw and arm stop.
    IW6  Slip: a collar that should be locked moves -> brake, lock all, fault.
    IW7  No climbing at the crown, in high wind, or without 2× the energy
         to come down; low battery forces a descent.
    IW8  Hold-to-run: releasing CLIMB/DESCEND finishes nothing: it brakes
         and re-locks the released collar immediately.
    IW9  Straps open only on the ground.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field, replace
from enum import Enum, auto
from typing import List, Optional

from .controller import Cmd
from .design import G
from .inchworm import ARM_STOW_X, InchwormDesign
from .tree import Tree


class IState(Enum):
    GROUND = auto()
    MOUNTED = auto()      # both collars locked, stationary
    CLIMBING = auto()
    DESCENDING = auto()
    HARVEST = auto()
    FAULT = auto()
    ESTOP = auto()


class Phase(Enum):
    REL_U = auto()
    MOVE_U = auto()
    LOCK_U = auto()
    REL_L = auto()
    MOVE_L = auto()
    LOCK_L = auto()


@dataclass(frozen=True)
class IInputs:
    cmd: Cmd = Cmd.NONE
    estop: bool = False
    comms_ok: bool = True
    upper_locked: bool = True
    lower_locked: bool = True
    upper_vel: float = 0.0
    lower_vel: float = 0.0
    act_pos: float = 0.0        # m, 0 = retracted
    tension_ok: bool = False
    altitude: float = 0.0       # m, lower collar
    tilt: float = 0.0
    battery_wh: float = 250.0
    arm_stowed: bool = True
    arm_deployed: bool = False
    zone_clear: bool = False
    at_crown: bool = False
    landed: bool = True
    wind: float = 0.0
    straps_closed: bool = False


@dataclass(frozen=True)
class IOutputs:
    act: int = 0                # -1 retract, 0 brake, +1 extend
    rel_upper: bool = False
    rel_lower: bool = False
    tension: bool = False
    saw: bool = False
    arm: int = 0
    straps_open: bool = False


@dataclass
class IConfig:
    stroke: float = 0.4
    slip_tol: float = 0.05
    tilt_max: float = 28.0
    wind_max: float = 10.0
    min_harvest_alt: float = 2.0
    landed_alt: float = 0.05
    end_tol: float = 0.005
    phase_timeout: float = 6.0
    reserve_wh_base: float = 2.0
    reserve_wh_per_m: float = 0.3

    def reserve(self, alt: float) -> float:
        return self.reserve_wh_base + self.reserve_wh_per_m * max(0.0, alt)


@dataclass
class IController:
    cfg: IConfig = field(default_factory=IConfig)
    state: IState = IState.GROUND
    phase: Phase = Phase.REL_U
    forced_descent: bool = False
    fault_reason: str = ""
    timer: float = 0.0
    cuts: int = 0
    steps: int = 0
    _saw_was_on: bool = False
    _tension_hold: bool = False   # latched while arm is (or may be) out

    def _on_ground(self, i: IInputs) -> bool:
        return i.landed and i.altitude <= self.cfg.landed_alt

    def _both_locked(self, i: IInputs) -> bool:
        return i.upper_locked and i.lower_locked

    def _fault(self, reason: str) -> IOutputs:
        self.state, self.fault_reason, self.timer = IState.FAULT, reason, 0.0
        return IOutputs(tension=self._tension_hold)

    def step(self, i: IInputs, dt: float) -> IOutputs:
        out = self._step(i, dt)
        if self._tension_hold and not out.tension:
            out = replace(out, tension=True)  # IW4 by construction, whatever branch ran
        if out.saw and not self._saw_was_on:
            self.cuts += 1
        self._saw_was_on = out.saw
        return out

    def _step(self, i: IInputs, dt: float) -> IOutputs:
        cfg = self.cfg
        on_ground = self._on_ground(i)
        in_air = not on_ground
        if on_ground:
            self.forced_descent = False
        if not i.arm_stowed:
            self._tension_hold = True  # IW4: whatever happens, keep clamping while the arm is out
        hold = IOutputs(tension=self._tension_hold)

        # IW5
        if i.estop:
            self.state = IState.ESTOP
            return hold
        if self.state == IState.ESTOP:
            if i.cmd == Cmd.RESET and i.comms_ok:
                self.state = IState.MOUNTED if i.straps_closed else IState.GROUND
            return hold
        if not i.comms_ok and self.state not in (IState.GROUND, IState.FAULT):
            return self._fault("radio link lost")

        # IW6: a collar that is supposed to be locked is moving.
        moving_u = self.state in (IState.CLIMBING, IState.DESCENDING) and self.phase == Phase.MOVE_U
        moving_l = self.state in (IState.CLIMBING, IState.DESCENDING) and self.phase == Phase.MOVE_L
        if in_air and self.state not in (IState.GROUND, IState.FAULT):
            if (not moving_u and abs(i.upper_vel) > cfg.slip_tol) or \
               (not moving_l and abs(i.lower_vel) > cfg.slip_tol):
                return self._fault("slip")
        if in_air and abs(i.tilt) > cfg.tilt_max and self.state != IState.FAULT:
            return self._fault("tilt")
        if in_air and i.battery_wh <= cfg.reserve(i.altitude):
            self.forced_descent = True

        s = self.state
        self.timer += dt

        if s == IState.FAULT:
            if i.cmd == Cmd.RESET and i.comms_ok and self._both_locked(i) \
                    and abs(i.upper_vel) <= cfg.slip_tol and abs(i.lower_vel) <= cfg.slip_tol:
                self.fault_reason = ""
                self.state = IState.MOUNTED if (in_air or i.straps_closed) else IState.GROUND
            return hold

        if s == IState.GROUND:
            if i.cmd == Cmd.CLAMP and i.straps_closed and self._both_locked(i):
                self.state = IState.MOUNTED
                return hold
            # IW9
            opening = i.cmd == Cmd.RELEASE and on_ground and i.arm_stowed
            return IOutputs(straps_open=opening, tension=self._tension_hold and in_air)

        if s == IState.MOUNTED:
            if not i.arm_stowed:
                self.state = IState.HARVEST
                return hold
            if self._tension_hold and i.arm_stowed and i.cmd not in (Cmd.DEPLOY_ARM, Cmd.CUT):
                self._tension_hold = False  # arm safely stowed: slacken the harvest strap
                return IOutputs()
            want_up = i.cmd == Cmd.CLIMB and not self.forced_descent
            want_down = (i.cmd == Cmd.DESCEND or (self.forced_descent and i.cmd != Cmd.STOP)) and in_air
            if (want_up and self._climb_ok(i)) or (want_down and self._move_ok(i)):
                self.state = IState.CLIMBING if want_up else IState.DESCENDING
                self.phase, self.timer = Phase.REL_U, 0.0
                return self._stepper(i, up=want_up)
            if i.cmd == Cmd.DEPLOY_ARM and i.altitude >= cfg.min_harvest_alt and self._both_locked(i):
                self._tension_hold = True
                if i.tension_ok:
                    self.state = IState.HARVEST
                    return IOutputs(tension=True, arm=+1)
                return IOutputs(tension=True)
            if i.cmd == Cmd.RELEASE and on_ground and i.arm_stowed and not self._tension_hold:
                self.state = IState.GROUND
                return IOutputs(straps_open=True)
            return hold

        if s in (IState.CLIMBING, IState.DESCENDING):
            up = s == IState.CLIMBING
            held = i.cmd == (Cmd.CLIMB if up else Cmd.DESCEND) or \
                (not up and self.forced_descent and i.cmd != Cmd.STOP)
            ok = self._climb_ok(i) if up else (self._move_ok(i) and (in_air or self.phase != Phase.REL_U))
            if not held or not ok:
                # IW8: brake and let the released collar's servo spring re-lock it.
                if self._both_locked(i):
                    self.state = IState.MOUNTED
                return IOutputs()
            if self.timer > cfg.phase_timeout:
                return self._fault(f"step timeout in {self.phase.name}")
            return self._stepper(i, up)

        if s == IState.HARVEST:
            stowing = i.cmd == Cmd.STOW_ARM or self.forced_descent or not self._both_locked(i)
            if stowing:
                if i.arm_stowed:
                    self.state = IState.MOUNTED
                    return IOutputs(tension=True)
                return IOutputs(tension=True, arm=-1)
            if not i.arm_deployed:
                return IOutputs(tension=True, arm=+1 if i.tension_ok else 0)
            saw = (i.cmd == Cmd.CUT and i.zone_clear and i.tension_ok and self._both_locked(i)
                   and i.wind <= cfg.wind_max)
            return IOutputs(tension=True, saw=saw)

        raise AssertionError(s)  # pragma: no cover

    # ------------------------------------------------------------ helpers
    def _move_ok(self, i: IInputs) -> bool:
        return (i.arm_stowed and not self._tension_hold and i.comms_ok and not i.estop
                and abs(i.tilt) <= self.cfg.tilt_max and i.straps_closed)

    def _climb_ok(self, i: IInputs) -> bool:
        return (self._move_ok(i) and not i.at_crown and i.wind <= self.cfg.wind_max
                and not self.forced_descent and i.battery_wh > self.cfg.reserve(i.altitude))

    def _stepper(self, i: IInputs, up: bool) -> IOutputs:
        """One inchworm step. Upper collar moves first in both directions."""
        cfg, p = self.cfg, self.phase
        end = cfg.stroke - cfg.end_tol
        # Upper collar moving: extend to climb, retract to descend. Lower: the opposite.
        u_dir, l_dir = (+1, -1) if up else (-1, +1)

        def at_end(d: int) -> bool:
            return i.act_pos >= end if d > 0 else i.act_pos <= cfg.end_tol

        def go(nxt: Phase) -> None:
            self.phase, self.timer = nxt, 0.0

        if p == Phase.REL_U:
            if not i.lower_locked:                  # IW1: other collar must hold
                return IOutputs()
            if not i.upper_locked:
                go(Phase.MOVE_U)
            return IOutputs(rel_upper=True)
        if p == Phase.MOVE_U:
            if not i.lower_locked:
                return self._fault("anchor lost during upper move")
            if at_end(u_dir):
                go(Phase.LOCK_U)
                return IOutputs()
            return IOutputs(rel_upper=True, act=u_dir)
        if p == Phase.LOCK_U:
            if i.upper_locked:
                go(Phase.REL_L)
            return IOutputs()
        if p == Phase.REL_L:
            if not i.upper_locked:
                return IOutputs()
            if not up and self._on_ground(i):     # already down: nothing to lower
                self.state = IState.MOUNTED
                return IOutputs()
            if not i.lower_locked:
                go(Phase.MOVE_L)
            return IOutputs(rel_lower=True)
        if p == Phase.MOVE_L:
            if not i.upper_locked:
                return self._fault("anchor lost during lower move")
            if at_end(l_dir) or (not up and i.landed):
                go(Phase.LOCK_L)
                return IOutputs()
            return IOutputs(rel_lower=True, act=l_dir)
        if p == Phase.LOCK_L:
            if i.lower_locked:
                self.steps += 1
                go(Phase.REL_U)
                if (up and i.at_crown) or (not up and i.landed):
                    self.state = IState.MOUNTED
            return IOutputs()
        raise AssertionError(p)  # pragma: no cover


def config_for(d: InchwormDesign) -> IConfig:
    per_m = d.descent_reserve(10.0) / 10.0
    return IConfig(stroke=d.stroke / 1000.0, wind_max=d.req.wind_max,
                   tilt_max=d.pop.lean.hi + 8.0, reserve_wh_base=2.0, reserve_wh_per_m=per_m)


# ============================================================================ plant
DT = 0.05
AIM_S = 2.0


@dataclass
class IPlant:
    d: InchwormDesign
    tree: Tree
    wet: bool = False
    rng: random.Random = field(default_factory=random.Random)
    z_l: float = 0.0
    act: float = 0.0
    straps_closed: bool = False
    rel_u_t: float = 0.0       # time the upper servo has been releasing (+) / locking (-)
    rel_l_t: float = 0.0
    tension: float = 0.0       # 0..1 harvest tensioner engagement
    arm: float = 0.0
    battery_wh: float = 0.0
    mu_override: Optional[float] = None
    upper_vel: float = 0.0
    lower_vel: float = 0.0
    energy_used: float = 0.0
    fell: bool = False

    def __post_init__(self) -> None:
        if not self.battery_wh:
            self.battery_wh = self.d.battery_wh * self.d.req.battery_dod

    @property
    def z_u(self) -> float:
        return self.z_l + self.act

    @property
    def mu(self) -> float:
        if self.mu_override is not None:
            return self.mu_override
        return self.tree.mu_dry * (self.d.pop.wet_factor if self.wet else 1.0)

    def _servo_locked(self, t: float) -> bool:
        return t <= -0.2       # locked once the spring has held it for 0.2 s

    def _holds(self, which: str) -> bool:
        """Does a locked collar hold its current load?"""
        d = self.d
        arm_x = ARM_STOW_X if self.arm <= 0 else -d.reach  # assume worst slew when deployed
        if which == "upper":
            active = d.N_active * self.tension
            return d.lock_ratio(self.mu, arm_x, active=active, lean=self.tree.lean, height=self.z_u) >= 1.0
        return d.lock_ratio(self.mu, 0.0, carried=d.weight, lean=self.tree.lean, height=self.z_u) >= 1.0

    def step(self, o: IOutputs, dt: float = DT) -> None:
        d = self.d
        if o.straps_open:
            self.straps_closed = False
        self.rel_u_t = (max(0.0, self.rel_u_t) + dt) if o.rel_upper else (min(0.0, self.rel_u_t) - dt)
        self.rel_l_t = (max(0.0, self.rel_l_t) + dt) if o.rel_lower else (min(0.0, self.rel_l_t) - dt)
        self.tension = min(1.0, self.tension + dt) if o.tension else max(0.0, self.tension - dt)
        self.arm = min(1.0, max(0.0, self.arm + o.arm * dt / 3.0))
        up_free = self.rel_u_t >= 0.15
        lo_free = self.rel_l_t >= 0.15
        z_l0, z_u0 = self.z_l, self.z_u
        p = d.req.electronics_w + (d.saw_w if o.saw else 0.0) + (30.0 if o.arm else 0.0)
        if self.straps_closed:
            if up_free and lo_free and self.z_l > 0:
                self.fell = True
                self.z_l = max(0.0, self.z_l - 1.0 * dt)
            elif o.act and up_free != lo_free:
                drag = 2 * d.pop.mu_dry.hi * d.req.strap_preload * 0.3
                load = (d.m_upper if up_free else d.m_lower) * G + drag + (d.tether_weight(self.z_u) if up_free else 0.0)
                going_up = (o.act > 0) == up_free
                v = d.speed_at(load) if going_up else d.speed_at(drag) * 0.8
                da = o.act * v * dt
                new_act = min(d.stroke / 1000.0, max(0.0, self.act + da))
                if lo_free:  # lower collar moves, upper fixed: z_u constant
                    z_u = self.z_u
                    self.act = new_act
                    self.z_l = max(0.0, z_u - self.act)
                    self.act = z_u - self.z_l
                else:
                    self.act = new_act
                p += load * abs(da) / dt / (d.req.eta_screw * d.req.eta_elec) * (1 if going_up else 0.25)
            # locked collars that cannot hold their load slide down
            if not up_free and not self._holds("upper") and not (not lo_free and self._holds("lower")):
                self.z_l = max(0.0, self.z_l - 0.3 * dt)
            elif not lo_free and up_free and not self._holds("lower"):
                self.z_l = max(0.0, self.z_l - 0.3 * dt)
        self.z_l = min(self.z_l, max(0.0, self.tree.height - self.act - 0.1))
        self.lower_vel = (self.z_l - z_l0) / dt
        self.upper_vel = (self.z_u - z_u0) / dt
        e = p * dt / 3600.0
        self.battery_wh -= e
        self.energy_used += e

    def sensors(self, cmd: Cmd, **kw) -> IInputs:
        locked_u = self.straps_closed and self._servo_locked(self.rel_u_t)
        locked_l = self.straps_closed and self._servo_locked(self.rel_l_t)
        base = dict(
            cmd=cmd, upper_locked=locked_u, lower_locked=locked_l, upper_vel=self.upper_vel,
            lower_vel=self.lower_vel, act_pos=self.act, tension_ok=self.tension >= 0.9,
            altitude=self.z_l, tilt=self.tree.lean, battery_wh=self.battery_wh,
            arm_stowed=self.arm <= 0, arm_deployed=self.arm >= 1, zone_clear=True,
            at_crown=self.z_u >= self.tree.height - 0.3, landed=self.z_l <= 0.02,
            straps_closed=self.straps_closed,
        )
        base.update(kw)
        return IInputs(**base)


@dataclass
class IMissionResult:
    ok: bool
    reason: str
    time_s: float
    energy_wh: float
    cuts: int
    steps: int
    faults: List[str]
    fell: bool


def run_mission(d: InchwormDesign, tree: Tree, peduncles: List[float], wet: bool = False,
                rng: Optional[random.Random] = None, plant: Optional[IPlant] = None,
                ctrl: Optional[IController] = None, max_time: float = 1500.0, disturb=None) -> IMissionResult:
    rng = rng or random.Random(0)
    p = plant or IPlant(d, tree, wet=wet, rng=rng)
    c = ctrl or IController(cfg=config_for(d))
    p.straps_closed = True              # crew buckles both straps; servos start locked
    p.rel_u_t = p.rel_l_t = -1.0
    faults: List[str] = []
    t, phase, todo, cut_left = 0.0, "mount", list(peduncles), 0.0
    out = IOutputs()
    extra: dict = {}

    def scan(cmd: Cmd) -> None:
        nonlocal out, t
        out = c.step(p.sensors(cmd, **extra), DT)
        p.step(out, DT)
        t += DT

    while t < max_time:
        if disturb:
            extra = disturb(t, p, c) or {}
        if c.state == IState.FAULT:
            faults.append(c.fault_reason)
            scan(Cmd.NONE)
            if extra.get("comms_ok", True):
                scan(Cmd.RESET)
                phase = "stow" if p.arm > 0 else "descend"
            continue
        if phase == "mount":
            scan(Cmd.CLAMP)
            if c.state == IState.MOUNTED:
                phase = "climb"
        elif phase == "climb":
            scan(Cmd.CLIMB)
            if c.state == IState.MOUNTED:
                if p.z_u >= tree.height - 0.3:
                    phase = "harvest" if todo else "descend"
                elif c.forced_descent:
                    phase = "descend"
        elif phase == "harvest":
            if c.state in (IState.MOUNTED, IState.HARVEST) and p.arm < 1.0:
                scan(Cmd.DEPLOY_ARM)
            elif c.state == IState.HARVEST:
                if cut_left <= 0:
                    if not todo:
                        phase = "stow"
                        continue
                    for _ in range(int(AIM_S / DT)):
                        scan(Cmd.NONE)
                    cut_left = d.cut_time(todo.pop(0)) + 1.0
                scan(Cmd.CUT)
                cut_left -= DT
            else:
                scan(Cmd.NONE)
        elif phase == "stow":
            scan(Cmd.STOW_ARM)
            if c.state == IState.MOUNTED and p.arm <= 0:
                phase = "descend"
        elif phase == "descend":
            if p.z_l <= 0.02 and c.state == IState.MOUNTED:
                phase = "release"
            else:
                scan(Cmd.DESCEND)
        elif phase == "release":
            scan(Cmd.RELEASE)
            if c.state == IState.GROUND and not p.straps_closed:
                done = len(peduncles) - len(todo)
                ok = done == len(peduncles) and not faults and not p.fell
                return IMissionResult(ok, "ok" if ok else "incomplete", t, p.energy_used, c.cuts, c.steps,
                                      faults, p.fell)
    return IMissionResult(False, "timeout", t, p.energy_used, c.cuts, c.steps, faults, p.fell)
