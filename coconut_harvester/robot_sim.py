"""Closed-loop simulation: controller + mission executive + robot/trunk physics.

The plant is deliberately independent of the controller: it only sees the
actuator outputs and produces sensor readings, so tests exercise the real
control code against physics (ridges, wet bark, slip, stall, battery drain).
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import List, Optional

from .controller import Cmd, Config, Controller, Inputs, Outputs, State
from .design import G, N_PER_TIER, N_WHEELS, Design
from .tree import Tree

DT = 0.05
AIM_S = 2.0   # s per bunch to line up the blade (closed-loop model; field value in fleet sim)


def controller_config(d: Design) -> Config:
    per_m = d.req.descent_reserve_sf * (d.energy_descend(1.0) + d.req.electronics_w / d.speed / 3600)
    return Config(N0=d.N0, wind_max=d.req.wind_max, tilt_max=d.pop.lean.hi + 8.0,
                  reserve_wh_base=2.0, reserve_wh_per_m=per_m)


@dataclass
class Plant:
    d: Design
    tree: Tree
    wet: bool = False
    rng: random.Random = field(default_factory=random.Random)
    z: float = 0.0             # true height of lower tier
    z_enc: float = 0.0         # odometry
    clamp: float = 0.0         # actual per-wheel normal force
    arm: float = 0.0           # 0 stowed .. 1 deployed
    battery_wh: float = 0.0
    wind: float = 0.0
    mu_override: Optional[float] = None
    enc_vel: float = 0.0
    imu_vel: float = 0.0
    energy_used: float = 0.0
    ridges: List[float] = field(default_factory=list)  # ridge heights (mm) every 0.5 m

    def __post_init__(self) -> None:
        if not self.battery_wh:
            self.battery_wh = self.d.battery_wh * self.d.req.battery_dod
        n = int(self.tree.height / 0.5) + 2
        self.ridges = [self.tree.ridge_h * self.rng.uniform(0.3, 1.0) for _ in range(n)]
        self.ridges[self.rng.randrange(n)] = self.tree.ridge_h  # the worst one is somewhere

    @property
    def mu(self) -> float:
        if self.mu_override is not None:
            return self.mu_override
        return self.tree.mu_dry * (self.d.pop.wet_factor if self.wet else 1.0)

    def _ridge_slope_here(self) -> float:
        """Slope seen by whichever tier is currently crossing a ridge."""
        L = self.tree.ridge_len / 1000.0
        worst = 0.0
        for zt in (self.z, self.z + self.d.H):
            k = int(zt / 0.5)
            if 0 <= k < len(self.ridges) and (zt - k * 0.5) < L:
                worst = max(worst, self.d.ridge_slope(self.ridges[k], self.tree.ridge_len))
        return worst

    def step(self, out: Outputs, dt: float = DT) -> None:
        d = self.d
        # clamp actuator: first-order towards target (tau 0.4 s)
        self.clamp += (out.clamp_target - self.clamp) * min(1.0, dt / 0.4)
        self.arm = min(1.0, max(0.0, self.arm + out.arm * dt / 3.0))
        W = d.weight * math.cos(math.radians(self.tree.lean))
        traction = self.mu * N_WHEELS * self.clamp
        p_elec = d.req.electronics_w + (d.saw_w if out.saw else 0.0) + (30.0 if out.arm else 0.0)

        if out.drive != 0 and self.clamp > 0:
            s = self._ridge_slope_here()
            rr = d.req.c_rr * N_WHEELS * self.clamp
            motor_max = N_WHEELS * d.motor[1] * 1.5 / d.r_wheel  # peak = 1.5x rated
            if out.drive > 0:
                demand = W + rr + N_PER_TIER * self.clamp * s
            else:
                demand = rr + N_PER_TIER * self.clamp * s - W  # gravity helps
            if demand > motor_max:
                self.enc_vel = self.imu_vel = 0.0  # stall
            elif demand > traction:
                self.enc_vel = out.drive * d.speed  # wheels spin, robot does not follow
                self.imu_vel = 0.0 if traction >= W else -0.3
            else:
                self.enc_vel = self.imu_vel = out.drive * d.speed
            if out.drive > 0:
                p_elec += max(demand, 0) * d.speed / (d.eta_worm * d.req.eta_elec)
            else:
                back = max(0.0, 1.0 / d.eta_worm - 2.0)
                p_elec += (d.weight * back + rr / d.eta_worm) * d.speed / d.req.eta_elec
        else:
            self.enc_vel = 0.0
            # holding: self-locking worms; slide only if friction cannot carry the weight
            self.imu_vel = -0.3 if (self.z > 0 and traction < W) else 0.0
        self.z = max(0.0, min(self.tree.height, self.z + self.imu_vel * dt))
        self.z_enc = max(0.0, min(self.tree.height, self.z_enc + self.enc_vel * dt))
        e = p_elec * dt / 3600.0
        self.battery_wh -= e
        self.energy_used += e

    def sensors(self, cmd: Cmd, **kw) -> Inputs:
        return Inputs(
            cmd=cmd, clamp_force=self.clamp, altitude=self.z_enc,
            enc_vel=self.enc_vel, imu_vel=self.imu_vel, tilt=self.tree.lean,
            battery_wh=self.battery_wh, arm_stowed=self.arm <= 0.0, arm_deployed=self.arm >= 1.0,
            bark_wet=self.wet, at_crown=self.z >= self.tree.height - 0.3,
            landed=self.z <= 0.02, wind=self.wind, **{"zone_clear": True, "comms_ok": True, **kw},
        )


@dataclass
class MissionResult:
    ok: bool
    reason: str
    time_s: float
    energy_wh: float
    cuts: int
    faults: List[str]


def run_mission(d: Design, tree: Tree, peduncles: List[float], wet: bool = False,
                rng: Optional[random.Random] = None, plant: Optional[Plant] = None,
                ctrl: Optional[Controller] = None, max_time: float = 900.0,
                disturb=None) -> MissionResult:
    """Mission executive: clamp, climb, cut each bunch, descend, release.

    ``disturb(t, plant, ctrl)`` may inject faults (rain, link loss, ...).
    Returns when the robot is back on the ground and released, or on timeout.
    """
    rng = rng or random.Random(0)
    p = plant or Plant(d, tree, wet=wet, rng=rng)
    c = ctrl or Controller(cfg=controller_config(d))
    faults: List[str] = []
    t = 0.0
    phase = "clamp"
    todo = list(peduncles)
    cut_left = 0.0
    out = Outputs()
    extra: dict = {}

    def scan(cmd: Cmd) -> None:
        nonlocal out, t
        out = c.step(p.sensors(cmd, **extra), DT)
        p.step(out, DT)
        t += DT

    while t < max_time:
        if disturb:
            extra = disturb(t, p, c) or {}
        if c.state == State.FAULT:
            faults.append(c.fault_reason)
            scan(Cmd.NONE)
            if extra.get("comms_ok", True):
                scan(Cmd.RESET)
                phase = "stow" if not p.arm <= 0 else "descend"
            continue
        if phase == "clamp":
            scan(Cmd.CLAMP)
            if c.state == State.CLAMPED:
                phase = "release" if wet else "climb"  # never climb wet bark
        elif phase == "climb":
            scan(Cmd.CLIMB)
            if c.state == State.CLAMPED:
                if p.z >= tree.height - 0.3:
                    phase = "harvest" if todo else "descend"
                elif c.climb_blocked or c.forced_descent or p.wet:
                    phase = "descend"
        elif phase == "harvest":
            if c.state in (State.CLAMPED, State.HARVEST) and p.arm < 1.0:
                scan(Cmd.DEPLOY_ARM)
            elif c.state == State.HARVEST:
                if cut_left <= 0:
                    if not todo:
                        phase = "stow"
                        continue
                    for _ in range(int(AIM_S / DT)):  # operator lines the blade up
                        scan(Cmd.NONE)
                    cut_left = d.cut_time(todo.pop(0)) + 1.0  # + spin-up / breakthrough
                scan(Cmd.CUT)
                cut_left -= DT
            else:
                scan(Cmd.NONE)
        elif phase == "stow":
            scan(Cmd.STOW_ARM)
            if c.state == State.CLAMPED and p.arm <= 0:
                phase = "descend"
        elif phase == "descend":
            if p.z <= 0.02 and c.state == State.CLAMPED:
                phase = "release"
            else:
                scan(Cmd.DESCEND if c.state == State.CLAMPED or c.state == State.DESCENDING else Cmd.NONE)
                if not p.arm <= 0 and c.state == State.HARVEST:
                    scan(Cmd.STOW_ARM)
        elif phase == "release":
            scan(Cmd.RELEASE)
            if c.state == State.GROUND and p.clamp < 1.0:
                done = len(peduncles) - len(todo)
                ok = not wet and done == len(peduncles) and not c.climb_blocked
                reason = "wet bark: not climbed" if wet else ("stall: tree flagged" if faults else "ok")
                return MissionResult(ok and not faults, reason, t, p.energy_used, c.cuts, faults)
    return MissionResult(False, "timeout", t, p.energy_used, c.cuts, faults)
