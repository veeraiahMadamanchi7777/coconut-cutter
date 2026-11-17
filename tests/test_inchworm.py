"""Reimagined concept: inchworm climber with self-locking collars."""
import math
import random
from dataclasses import replace

import pytest
from hypothesis import HealthCheck, given, settings, strategies as st

from coconut_harvester.controller import Cmd
from coconut_harvester.inchworm import (AL_TUBES, CYLINDERS, LEVERS, PAD_SPANS, STROKES, Requirements,
                                        optimize, size)
from coconut_harvester.inchworm_control import (DT, IConfig, IController, IInputs, IOutputs, IPlant, IState,
                                                config_for, run_mission)
from coconut_harvester.tree import Dist, Tree, TreePopulation


# ------------------------------------------------------------------ design

def test_every_check_passes_and_parts_are_catalogue(iw):
    assert iw.ok and len(iw.checks) >= 18
    assert iw.cylinder in CYLINDERS and iw.stroke in STROKES and iw.tube in AL_TUBES
    assert iw.lever in LEVERS and iw.pad_span in PAD_SPANS


def test_half_the_mass_of_the_wheel_robot(iw, design):
    assert iw.mass <= 25.0 and iw.mass < 0.6 * design.mass


def test_self_lock_physics_lever_rule(iw):
    """Grip grows with load: the lock ratio is independent of weight and equals 2μL/h."""
    mu = 0.4
    r1 = iw.lock_ratio(mu, 0.0, carried=100.0)
    r2 = iw.lock_ratio(mu, 0.0, carried=400.0)
    assert r1 == pytest.approx(r2)
    assert r1 == pytest.approx(2 * mu * iw.lever * 0.85 / iw.pad_span)


def test_saw_on_far_side_would_unlock_without_tensioner(iw):
    mu_wet = iw.pop.mu_dry.lo * iw.pop.wet_factor
    assert iw.lock_ratio(mu_wet, -iw.reach) < 1.0          # the finding that forced the tensioner
    assert iw.harvest_ratio(mu_wet) >= iw.req.sf_lock


def test_200k_trees_climbable_and_holdable_wet_or_dry(iw):
    for t in iw.pop.stream(200_000, seed=12):
        assert iw.can_climb(t, wet=True) and iw.can_climb(t, wet=False), t


def test_ridges_do_not_matter(iw):
    t = iw.pop.sample(random.Random(1))
    assert iw.can_climb(replace(t, ridge_h=9.9, ridge_len=30), wet=True)


def test_timing_and_energy_are_physical(iw):
    t = replace(iw.pop.sample(random.Random(2)), height=15.0)
    assert iw.climb_time(t) == pytest.approx(iw.steps(15.0) * iw._step_time(True))
    assert iw.speed >= iw.req.min_speed
    potential_wh = iw.mass * 9.81 * 15 / 3600
    assert iw.energy_climb(15) > potential_wh
    assert 0 < iw.energy_descend(15) < iw.energy_climb(15)


def test_impossible_envelope_is_reported():
    ice = TreePopulation(mu_dry=Dist(0.15, 0.02, 0.10, 0.20))
    with pytest.raises(RuntimeError):
        optimize(pop=ice)


@settings(max_examples=30, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(mu_lo=st.floats(0.4, 0.65), wet=st.floats(0.5, 0.8), height_hi=st.floats(15, 26),
       reach_hi=st.floats(0.5, 0.9), lean_hi=st.floats(10, 25))
def test_any_passing_design_really_holds_its_population(mu_lo, wet, height_hi, reach_hi, lean_hi):
    pop = TreePopulation(mu_dry=Dist(0.7, 0.07, mu_lo, 0.9), wet_factor=wet, height=Dist(12, 4, 4, height_hi),
                         bunch_radius=Dist(0.45, 0.08, 0.3, reach_hi), lean=Dist(5, 4, 0, lean_hi))
    try:
        d = optimize(Requirements(max_mass=35), pop)
    except RuntimeError:
        return
    rng = random.Random(3)
    for _ in range(2000):
        t = pop.sample(rng)
        assert d.can_climb(t, wet=True) and d.can_hold(t, wet=True)


# -------------------------------------------------------------- controller

CFG = IConfig(stroke=0.4)
AIR = IInputs(altitude=8.0, landed=False, straps_closed=True)


def test_climb_step_sequence_upper_first_and_never_both_released():
    c = IController(cfg=CFG)
    c.state = IState.MOUNTED
    o = c.step(replace(AIR, cmd=Cmd.CLIMB), DT)
    assert o.rel_upper and not o.rel_lower and o.act == 0
    o = c.step(replace(AIR, cmd=Cmd.CLIMB, upper_locked=False), DT)
    o = c.step(replace(AIR, cmd=Cmd.CLIMB, upper_locked=False, act_pos=0.1, upper_vel=0.2), DT)
    assert o.act == 1 and o.rel_upper and not o.rel_lower
    o = c.step(replace(AIR, cmd=Cmd.CLIMB, upper_locked=False, act_pos=0.4), DT)
    assert o == IOutputs()                       # LOCK_U: servo released -> spring locks
    o = c.step(replace(AIR, cmd=Cmd.CLIMB, act_pos=0.4), DT)
    o = c.step(replace(AIR, cmd=Cmd.CLIMB, act_pos=0.4), DT)
    assert o.rel_lower and not o.rel_upper


def test_releasing_climb_button_relocks_immediately():
    c = IController(cfg=CFG)
    c.state = IState.MOUNTED
    c.step(replace(AIR, cmd=Cmd.CLIMB), DT)
    c.step(replace(AIR, cmd=Cmd.CLIMB, upper_locked=False), DT)
    o = c.step(replace(AIR, cmd=Cmd.NONE, upper_locked=False, act_pos=0.2), DT)
    assert o == IOutputs()


def test_arm_needs_tensioner_tight_and_tensioner_holds_while_arm_out():
    c = IController(cfg=CFG)
    c.state = IState.MOUNTED
    o = c.step(replace(AIR, cmd=Cmd.DEPLOY_ARM), DT)
    assert o.tension and o.arm == 0
    o = c.step(replace(AIR, cmd=Cmd.DEPLOY_ARM, tension_ok=True), DT)
    assert o.tension and o.arm == 1 and c.state == IState.HARVEST
    out_ = replace(AIR, tension_ok=True, arm_stowed=False, arm_deployed=True)
    for bad in (dict(estop=True), dict(comms_ok=False), dict(cmd=Cmd.CLIMB), dict(cmd=Cmd.RELEASE)):
        assert c.step(replace(out_, **bad), DT).tension


def test_saw_permissives_each_individually_required():
    good = replace(AIR, cmd=Cmd.CUT, tension_ok=True, arm_stowed=False, arm_deployed=True, zone_clear=True)
    c = IController(cfg=CFG)
    c.state = IState.HARVEST
    assert c.step(good, DT).saw
    for bad in (dict(zone_clear=False), dict(tension_ok=False), dict(upper_locked=False), dict(lower_locked=False),
                dict(wind=15.0), dict(cmd=Cmd.NONE)):
        c = IController(cfg=CFG)
        c.state = IState.HARVEST
        assert not c.step(replace(good, **bad), DT).saw, bad


def test_estop_mid_step_locks_both_collars():
    c = IController(cfg=CFG)
    c.state = IState.CLIMBING
    o = c.step(replace(AIR, cmd=Cmd.CLIMB, estop=True, upper_locked=False), DT)
    assert o == IOutputs()


def test_straps_open_only_on_ground():
    c = IController(cfg=CFG)
    c.state = IState.MOUNTED
    assert not c.step(replace(AIR, cmd=Cmd.RELEASE), DT).straps_open
    g = IInputs(cmd=Cmd.RELEASE, straps_closed=True, landed=True, altitude=0.0)
    assert c.step(g, DT).straps_open


MISSION = ([Cmd.CLAMP] * 2 + [Cmd.CLIMB] * 140 + [Cmd.DEPLOY_ARM] * 6 + [Cmd.CUT] * 4 + [Cmd.NONE]
           + [Cmd.CUT] * 3 + [Cmd.STOW_ARM] * 6 + [Cmd.DESCEND] * 170 + [Cmd.RELEASE] * 2)
FAULT1 = st.sampled_from([
    {"estop": True}, {"comms_ok": False}, {"tilt": 40.0}, {"wind": 15.0}, {"zone_clear": False},
    {"battery_wh": 0.5}, {"battery_wh": 4.0}, {"upper_vel": -0.3}, {"lower_vel": -0.3}, {"at_crown": True},
    {"upper_locked": False}, {"lower_locked": False}, {"upper_locked": True}, {"lower_locked": True},
    {"tension_ok": False}, {"tension_ok": True}, {"arm_stowed": False}, {"arm_deployed": True},
    {"landed": True}, {"straps_closed": False},
])
HARVEST_AT = MISSION.index(Cmd.DEPLOY_ARM)
HARVEST_END = MISSION.index(Cmd.DESCEND)
# Half the injections land in the short harvest segment, where the saw permissives live.
POS = st.one_of(st.integers(0, len(MISSION) - 1), st.integers(HARVEST_AT, HARVEST_END))
PERTURB = st.lists(st.tuples(POS, st.one_of(st.none(), st.sampled_from(list(Cmd))),
                             FAULT1, st.integers(1, 6)), min_size=1, max_size=5)


class MiniPlant:
    """Plausible sensors for fuzzing: servo lag, cylinder travel, tensioner ramp, arm travel."""

    def __init__(self):
        self.alt, self.act, self.ru, self.rl, self.ten, self.arm, self.closed = 0.0, 0.0, -1, -1, 0.0, 0.0, False

    def apply(self, o: IOutputs):
        if o.straps_open:
            self.closed = False
        self.ru = (max(0, self.ru) + 1) if o.rel_upper else (min(0, self.ru) - 1)
        self.rl = (max(0, self.rl) + 1) if o.rel_lower else (min(0, self.rl) - 1)
        self.ten = min(1.0, self.ten + 0.5) if o.tension else max(0.0, self.ten - 0.5)
        self.arm = min(1.0, max(0.0, self.arm + 0.5 * o.arm))
        uv = lv = 0.0
        if o.act and (self.ru > 0) != (self.rl > 0):
            new = min(0.4, max(0.0, self.act + 0.1 * o.act))
            if self.rl > 0:
                self.alt = max(0.0, self.alt - (new - self.act))
                lv = -(new - self.act)
            else:
                uv = new - self.act
            self.act = new
        return uv, lv

    def inputs(self, cmd, uv, lv):
        return dict(cmd=cmd, upper_locked=self.closed and self.ru < 0, lower_locked=self.closed and self.rl < 0,
                    upper_vel=uv * 0.5, lower_vel=lv * 0.5, act_pos=self.act, tension_ok=self.ten >= 0.9,
                    altitude=self.alt, arm_stowed=self.arm <= 0, arm_deployed=self.arm >= 1, zone_clear=True,
                    landed=self.alt <= 0.02, straps_closed=self.closed, battery_wh=200.0, tilt=5.0, wind=3.0)


def check(c: IController, i: IInputs, o: IOutputs, before: IState):
    on_ground = i.landed and i.altitude <= CFG.landed_alt
    # IW1
    if o.rel_upper and o.rel_lower:
        assert on_ground
    if o.rel_upper:
        assert i.lower_locked or on_ground
    if o.rel_lower:
        assert i.upper_locked or on_ground
    # IW2
    if o.act:
        assert o.rel_upper != o.rel_lower
        assert (i.lower_locked if o.rel_upper else i.upper_locked)
        assert i.arm_stowed and not o.tension and i.comms_ok and not i.estop and abs(i.tilt) <= CFG.tilt_max
        assert not o.saw and o.arm == 0
    # IW3
    if o.saw:
        assert (i.arm_deployed and o.tension and i.tension_ok and i.upper_locked and i.lower_locked
                and not o.rel_upper and not o.rel_lower and o.act == 0 and i.zone_clear and i.comms_ok
                and not i.estop and i.cmd == Cmd.CUT and i.wind <= CFG.wind_max)
    # IW4
    if not i.arm_stowed:
        assert o.tension
    # IW5
    if i.estop or not i.comms_ok:
        assert o.act == 0 and not o.rel_upper and not o.rel_lower and not o.saw and o.arm == 0
    # IW7 + IW8: an upward move only while CLIMB is held and allowed
    if o.act and c.state == IState.CLIMBING:
        assert i.cmd == Cmd.CLIMB and not i.at_crown and i.wind <= CFG.wind_max
        assert i.battery_wh > CFG.reserve(i.altitude)
    if o.act and c.state == IState.DESCENDING:
        assert i.cmd == Cmd.DESCEND or (c.forced_descent and i.cmd != Cmd.STOP)
    # IW9
    if o.straps_open:
        assert on_ground and i.cmd == Cmd.RELEASE
    if before in (IState.FAULT, IState.ESTOP):
        assert o.act == 0 and not o.saw and not o.rel_upper and not o.rel_lower


@settings(max_examples=2500, deadline=None)
@given(PERTURB)
def test_safety_functions_under_faults_injected_into_a_real_mission(perturb):
    inject = {}
    for pos, cmd, fault, dur in perturb:
        for k in range(pos, min(len(MISSION), pos + dur)):
            inject[k] = (cmd, fault)
    c, m = IController(cfg=CFG), MiniPlant()
    m.closed = True
    uv = lv = 0.0
    for k, cmd in enumerate(MISSION):
        over, fault = inject.get(k, (None, {}))
        i = IInputs(**{**m.inputs(over or cmd, uv, lv), **fault})
        before = c.state
        o = c.step(i, DT)
        check(c, i, o, before)
        uv, lv = m.apply(o)


@settings(max_examples=2000, deadline=None)
@given(st.lists(st.builds(
    IInputs, cmd=st.sampled_from(list(Cmd)), estop=st.booleans(), comms_ok=st.booleans(),
    upper_locked=st.booleans(), lower_locked=st.booleans(), upper_vel=st.sampled_from([0.0, 0.3, -0.3]),
    lower_vel=st.sampled_from([0.0, -0.3]), act_pos=st.sampled_from([0.0, 0.2, 0.4]), tension_ok=st.booleans(),
    altitude=st.sampled_from([0.0, 1.0, 5.0, 15.0]), tilt=st.sampled_from([0.0, 35.0]),
    battery_wh=st.sampled_from([1.0, 200.0]), arm_stowed=st.booleans(), arm_deployed=st.booleans(),
    zone_clear=st.booleans(), at_crown=st.booleans(), landed=st.booleans(), wind=st.sampled_from([0.0, 15.0]),
    straps_closed=st.booleans()), min_size=1, max_size=100))
def test_safety_functions_hold_for_any_input_stream(seq):
    c = IController(cfg=CFG)
    for i in seq:
        before = c.state
        check(c, i, c.step(i, DT), before)


def test_scripted_mission_reaches_every_state():
    c, m = IController(cfg=CFG), MiniPlant()
    m.closed = True
    uv = lv = 0.0
    seen = set()
    for cmd in MISSION:
        o = c.step(IInputs(**m.inputs(cmd, uv, lv)), DT)
        uv, lv = m.apply(o)
        seen.add(c.state)
    assert {IState.MOUNTED, IState.CLIMBING, IState.HARVEST, IState.DESCENDING, IState.GROUND} <= seen
    assert c.cuts == 2 and c.steps > 4


# -------------------------------------------------------------- closed loop

def test_500_missions_wet_and_dry_complete_without_faults(iw):
    rng = random.Random(7)
    for n in range(500):
        t = iw.pop.sample(rng)
        peds = iw.pop.bunches(rng, 45)
        wet = n % 2 == 1
        r = run_mission(iw, t, peds, wet=wet, rng=rng)
        assert r.ok and not r.faults and not r.fell, (n, t, r)
        assert r.cuts == len(peds)
        model_t = iw.climb_time(t) + iw.descend_time(t)
        assert r.time_s <= model_t * 1.1 + sum(iw.cut_time(p) + 10 for p in peds) + 20  # model is conservative
        assert r.energy_wh <= iw.energy_tree(t, peds, r.time_s - model_t) * 1.1


def test_sap_patch_slip_is_caught_and_robot_never_falls(iw):
    t = replace(iw.pop.sample(random.Random(4)), height=14.0)
    p = IPlant(iw, t, rng=random.Random(4))

    def sap(time, plant, ctrl):
        plant.mu_override = 0.03 if 20.0 < time < 21.0 else None

    r = run_mission(iw, t, [], plant=p, disturb=sap)
    assert "slip" in r.faults and not r.fell and r.reason != "timeout"
    assert p.z_l <= 0.02  # landed


def test_link_loss_mid_step_locks_and_waits(iw):
    t = replace(iw.pop.sample(random.Random(5)), height=12.0)
    p = IPlant(iw, t, rng=random.Random(5))
    seen = []

    def radio(time, plant, ctrl):
        down = 15.0 < time < 18.0
        if down:
            seen.append((plant.z_l, plant.z_u))
        return {"comms_ok": False} if down else {}

    r = run_mission(iw, t, [50.0], plant=p, disturb=radio)
    assert "radio link lost" in r.faults and not r.fell and r.reason != "timeout"
    late = seen[10:]  # after the servos have re-locked
    assert max(z for z, _ in late) - min(z for z, _ in late) < 1e-6


def test_low_battery_forces_safe_return(iw):
    t = replace(iw.pop.sample(random.Random(6)), height=20.0)
    p = IPlant(iw, t, rng=random.Random(6), battery_wh=config_for(iw).reserve(0) + 3.0)
    r = run_mission(iw, t, [60.0, 60.0], plant=p)
    assert r.reason != "timeout" and not r.fell
    assert p.z_l <= 0.02 and p.battery_wh > 0
