"""Safety & function tests of the supervisory controller (no physics)."""
from dataclasses import replace

from hypothesis import given, settings, strategies as st

from coconut_harvester.controller import Cmd, Config, Controller, Inputs, Outputs, State

DT = 0.05
CFG = Config(N0=500.0)
GRIP = 500.0
AIR = Inputs(clamp_force=GRIP, altitude=10.0, landed=False, battery_wh=200.0)


def ctl(state=State.CLAMPED, **kw) -> Controller:
    c = Controller(cfg=CFG, **kw)
    c.state = state
    c.clamp_requested = True
    return c


# ------------------------------------------------------------------ functional

def test_clamp_then_climb_then_stop_at_crown():
    c = Controller(cfg=CFG)
    out = c.step(Inputs(cmd=Cmd.CLAMP), DT)
    assert out.clamp_target == CFG.N0 and c.state == State.GROUND
    c.step(Inputs(cmd=Cmd.NONE, clamp_force=GRIP), DT)
    assert c.state == State.CLAMPED
    out = c.step(Inputs(cmd=Cmd.CLIMB, clamp_force=GRIP, enc_vel=0.3, imu_vel=0.3), DT)
    assert out.drive == 1 and c.state == State.CLIMBING
    out = c.step(replace(AIR, cmd=Cmd.CLIMB, enc_vel=0.3, imu_vel=0.3, at_crown=True), DT)
    assert out.drive == 0 and c.state == State.CLAMPED


def test_harvest_cycle_and_cut_counting():
    c = ctl()
    assert c.step(replace(AIR, cmd=Cmd.DEPLOY_ARM), DT).arm == 1
    arm_out = replace(AIR, arm_stowed=False, arm_deployed=True)
    assert not c.step(replace(arm_out, cmd=Cmd.CUT), DT).saw          # zone not confirmed
    assert c.step(replace(arm_out, cmd=Cmd.CUT, zone_clear=True), DT).saw
    assert not c.step(replace(arm_out, cmd=Cmd.NONE, zone_clear=True), DT).saw  # dead-man released
    assert c.step(replace(arm_out, cmd=Cmd.CUT, zone_clear=True), DT).saw
    assert c.cuts == 2
    assert c.step(replace(arm_out, cmd=Cmd.STOW_ARM), DT).arm == -1
    c.step(replace(AIR, cmd=Cmd.STOW_ARM), DT)
    assert c.state == State.CLAMPED


def test_hold_to_run_releasing_the_button_stops_motion():
    for state, cmd in ((State.CLIMBING, Cmd.CLIMB), (State.DESCENDING, Cmd.DESCEND)):
        for other in (Cmd.NONE, Cmd.DEPLOY_ARM, Cmd.RELEASE, Cmd.CUT):
            c = ctl(state)
            assert c.step(replace(AIR, cmd=other), DT).drive == 0 and c.state == State.CLAMPED


def test_arm_never_deploys_near_the_ground():
    c = ctl()
    out = c.step(replace(AIR, altitude=1.0, cmd=Cmd.DEPLOY_ARM), DT)
    assert out.arm == 0 and c.state == State.CLAMPED


def test_wet_bark_wind_and_crown_block_climbing_but_not_descent():
    for bad in (dict(bark_wet=True), dict(wind=15.0), dict(at_crown=True)):
        c = ctl()
        assert c.step(replace(AIR, cmd=Cmd.CLIMB, **bad), DT).drive == 0
        assert c.step(replace(AIR, cmd=Cmd.DESCEND, **bad), DT).drive == -1


def test_low_battery_forces_descent_and_blocks_climb():
    c = ctl()
    low = replace(AIR, battery_wh=CFG.reserve(10.0) - 0.1)
    assert c.step(replace(low, cmd=Cmd.CLIMB), DT).drive == -1
    assert c.forced_descent


def test_slip_boosts_clamp_and_faults():
    c = ctl()
    out = c.step(replace(AIR, imu_vel=-0.3), DT)
    assert c.state == State.FAULT and c.fault_reason == "slip"
    assert out.clamp_target == CFG.N0 * CFG.slip_boost and out.drive == 0


def test_stall_faults_then_only_descent_allowed():
    c = ctl()
    c.step(replace(AIR, cmd=Cmd.CLIMB), DT)
    for _ in range(int(CFG.stall_time / DT) + 2):
        c.step(replace(AIR, cmd=Cmd.CLIMB, enc_vel=0.0, imu_vel=0.0), DT)
    assert c.state == State.FAULT and c.fault_reason == "stall"
    c.step(replace(AIR, cmd=Cmd.RESET), DT)
    assert c.step(replace(AIR, cmd=Cmd.CLIMB), DT).drive == 0
    assert c.step(replace(AIR, cmd=Cmd.DESCEND), DT).drive == -1


def test_link_loss_stops_saw_in_the_same_scan():
    c = ctl(State.HARVEST)
    arm_out = replace(AIR, arm_stowed=False, arm_deployed=True, zone_clear=True, cmd=Cmd.CUT)
    assert c.step(arm_out, DT).saw
    out = c.step(replace(arm_out, comms_ok=False), DT)
    assert out == Outputs(clamp_target=CFG.N0) and c.state == State.FAULT


def test_estop_holds_clamp_in_air_and_requires_reset():
    c = ctl(State.CLIMBING)
    out = c.step(replace(AIR, estop=True, cmd=Cmd.CLIMB), DT)
    assert out == Outputs(clamp_target=CFG.N0)
    assert c.step(replace(AIR, cmd=Cmd.CLIMB), DT) == Outputs(clamp_target=CFG.N0)
    c.step(replace(AIR, cmd=Cmd.RESET), DT)
    assert c.state == State.CLAMPED


def test_release_only_on_ground():
    c = ctl()
    assert c.step(replace(AIR, cmd=Cmd.RELEASE), DT).clamp_target == CFG.N0
    out = c.step(Inputs(cmd=Cmd.RELEASE, clamp_force=GRIP, landed=True, altitude=0.0), DT)
    assert out.clamp_target == 0.0 and c.state == State.GROUND


# --------------------------------------------------------------- safety fuzz

sensors = st.builds(
    Inputs,
    cmd=st.sampled_from(list(Cmd)),
    estop=st.booleans() | st.just(False),
    comms_ok=st.just(True) | st.booleans(),
    clamp_force=st.sampled_from([0.0, 200.0, 430.0, 500.0, 650.0]),
    altitude=st.sampled_from([0.0, 0.03, 0.5, 1.5, 3.0, 12.0, 20.0]),
    enc_vel=st.sampled_from([0.0, 0.01, 0.3, -0.3]),
    imu_vel=st.sampled_from([0.0, 0.01, 0.3, -0.3]),
    tilt=st.sampled_from([0.0, 10.0, 27.0, 35.0]),
    battery_wh=st.sampled_from([1.0, 5.0, 12.0, 200.0]),
    arm_stowed=st.booleans(), arm_deployed=st.booleans(),
    bark_wet=st.just(False) | st.booleans(), zone_clear=st.booleans(),
    at_crown=st.booleans(), landed=st.booleans(),
    wind=st.sampled_from([0.0, 5.0, 12.0]),
)


@settings(max_examples=3000, deadline=None)
@given(st.lists(sensors, min_size=1, max_size=120))
def test_safety_functions_hold_on_every_scan(seq):
    c = Controller(cfg=CFG)
    for i in seq:
        before = c.state
        out = c.step(i, DT)
        on_ground = i.landed and i.altitude <= CFG.landed_alt
        clamped = i.clamp_force >= CFG.clamp_ok_frac * CFG.N0
        # SF1
        if out.clamp_target == 0.0:
            assert on_ground
        # SF2
        if out.saw:
            assert (i.arm_deployed and i.zone_clear and i.comms_ok and not i.estop and i.cmd == Cmd.CUT
                    and out.drive == 0 and clamped and i.wind <= CFG.wind_max and out.arm == 0)
        # SF3
        if out.drive != 0:
            assert clamped and i.arm_stowed and not out.saw and i.comms_ok and not i.estop
            assert abs(i.tilt) <= CFG.tilt_max and out.arm == 0
        # SF6
        if out.drive == 1:
            assert not i.bark_wet and i.wind <= CFG.wind_max and not i.at_crown
            assert i.battery_wh > CFG.reserve(i.altitude)
        if out.drive == -1:
            assert not on_ground
        # SF4
        if i.estop or not i.comms_ok:
            assert out.drive == 0 and not out.saw and out.arm == 0
            if not on_ground:
                assert out.clamp_target >= CFG.N0
        # SF5: no motion out of FAULT/ESTOP without a reset
        if before in (State.FAULT, State.ESTOP) and out.drive != 0:
            raise AssertionError("moved straight out of a fault")
        if out.arm != 0:
            assert out.drive == 0


# ------------------------------------------------- model-based safety fuzz
# Random inputs rarely reach deep states (clamped, high up, arm out, cutting).
# Here a tiny plant keeps the sensors plausible and hypothesis injects faults
# on top, so every safety function is exercised mid-mission.

FAULTS = st.fixed_dictionaries({}, optional={
    "estop": st.just(True), "comms_ok": st.just(False), "clamp_force": st.sampled_from([0.0, 300.0]),
    "tilt": st.sampled_from([30.0, 40.0]), "wind": st.sampled_from([11.0, 15.0]),
    "bark_wet": st.just(True), "zone_clear": st.just(False), "battery_wh": st.sampled_from([0.5, 3.0, 8.0]),
    "imu_vel": st.sampled_from([-0.3, 0.3]), "at_crown": st.just(True),
    "arm_stowed": st.booleans(), "arm_deployed": st.booleans(),
})
# Single isolated faults: combinations often mask each other (an e-stop hides everything else).
FAULT1 = st.sampled_from([
    {"estop": True}, {"comms_ok": False}, {"clamp_force": 0.0}, {"clamp_force": 300.0}, {"tilt": 40.0},
    {"wind": 15.0}, {"bark_wet": True}, {"zone_clear": False}, {"battery_wh": 0.5}, {"battery_wh": 3.0},
    {"battery_wh": 8.0}, {"imu_vel": -0.3}, {"imu_vel": 0.3}, {"at_crown": True},
    {"arm_stowed": False}, {"arm_deployed": True}, {"arm_deployed": False}, {"landed": True},
])
STEP = st.tuples(st.sampled_from(list(Cmd)), st.one_of(st.just({}), st.just({}), FAULTS))


@settings(max_examples=1500, deadline=None)
@given(st.lists(STEP, min_size=5, max_size=300))
def test_safety_functions_hold_mid_mission(steps):
    c = Controller(cfg=CFG)
    alt, arm, clamp, out = 0.0, 0.0, 0.0, Outputs()
    for cmd, fault in steps:
        # plausible plant response to the previous outputs
        clamp += (out.clamp_target - clamp) * 0.6
        arm = min(1.0, max(0.0, arm + out.arm * 0.4))
        v = 0.4 * out.drive
        alt = max(0.0, alt + v * 1.0)
        base = dict(cmd=cmd, clamp_force=clamp, altitude=alt, enc_vel=v, imu_vel=v,
                    arm_stowed=arm <= 0.0, arm_deployed=arm >= 1.0, landed=alt <= 0.02,
                    zone_clear=True, battery_wh=200.0, tilt=5.0, wind=3.0)
        i = Inputs(**{**base, **fault})
        before = c.state
        out = c.step(i, DT)
        on_ground = i.landed and i.altitude <= CFG.landed_alt
        clamped = i.clamp_force >= CFG.clamp_ok_frac * CFG.N0
        if out.clamp_target == 0.0:
            assert on_ground
        if out.saw:
            assert (i.arm_deployed and i.zone_clear and i.comms_ok and not i.estop and i.cmd == Cmd.CUT
                    and clamped and i.wind <= CFG.wind_max and out.drive == 0)
        if out.drive != 0:
            assert clamped and i.arm_stowed and i.comms_ok and not i.estop and abs(i.tilt) <= CFG.tilt_max
        if out.drive == 1:
            assert not i.bark_wet and i.wind <= CFG.wind_max and not i.at_crown
            assert i.battery_wh > CFG.reserve(i.altitude)
        if i.estop or not i.comms_ok:
            assert out.drive == 0 and not out.saw and out.arm == 0
        if before in (State.FAULT, State.ESTOP):
            assert out.drive == 0 and not out.saw


MISSION = ([Cmd.CLAMP] * 4 + [Cmd.CLIMB] * 10 + [Cmd.DEPLOY_ARM] * 4 + [Cmd.CUT] * 5 + [Cmd.NONE]
           + [Cmd.CUT] * 3 + [Cmd.STOW_ARM] * 4 + [Cmd.DESCEND] * 12 + [Cmd.RELEASE] * 2)
PERTURB = st.lists(
    st.tuples(st.integers(0, len(MISSION) - 1), st.one_of(st.none(), st.sampled_from(list(Cmd))),
              st.one_of(FAULT1, FAULT1, FAULTS),
              st.integers(1, 4)),
    min_size=1, max_size=4)


def check_scan(c, i, out, before):
    on_ground = i.landed and i.altitude <= CFG.landed_alt
    # SF8 hold-to-run
    if out.drive == 1:
        assert i.cmd == Cmd.CLIMB
    if out.drive == -1:
        assert i.cmd == Cmd.DESCEND or (c.forced_descent and i.cmd != Cmd.STOP)
    clamped = i.clamp_force >= CFG.clamp_ok_frac * CFG.N0
    if out.clamp_target == 0.0:
        assert on_ground
    if out.saw:
        assert (i.arm_deployed and i.zone_clear and i.comms_ok and not i.estop and i.cmd == Cmd.CUT
                and clamped and i.wind <= CFG.wind_max and out.drive == 0)
    if out.drive != 0:
        assert clamped and i.arm_stowed and i.comms_ok and not i.estop and abs(i.tilt) <= CFG.tilt_max
    if out.drive == 1:
        assert not i.bark_wet and i.wind <= CFG.wind_max and not i.at_crown
        assert i.battery_wh > CFG.reserve(i.altitude)
    if i.estop or not i.comms_ok:
        assert out.drive == 0 and not out.saw and out.arm == 0
    if before in (State.FAULT, State.ESTOP):
        assert out.drive == 0 and not out.saw


@settings(max_examples=2500, deadline=None)
@given(PERTURB)
def test_safety_functions_under_faults_injected_into_a_real_mission(perturb):
    """A full harvest script; hypothesis injects faults / stray commands at arbitrary points."""
    inject = {}
    for pos, cmd, fault, dur in perturb:
        for k in range(pos, min(len(MISSION), pos + dur)):
            inject[k] = (cmd, fault)
    c = Controller(cfg=CFG)
    alt, arm, clamp, out = 0.0, 0.0, 0.0, Outputs()
    for k, cmd in enumerate(MISSION):
        clamp += (out.clamp_target - clamp) * 0.6
        arm = min(1.0, max(0.0, arm + out.arm * 0.4))
        v = 0.4 * out.drive
        alt = max(0.0, alt + v)
        over_cmd, fault = inject.get(k, (None, {}))
        base = dict(cmd=over_cmd or cmd, clamp_force=clamp, altitude=alt, enc_vel=v, imu_vel=v,
                    arm_stowed=arm <= 0.0, arm_deployed=arm >= 1.0, landed=alt <= 0.02,
                    zone_clear=True, battery_wh=200.0, tilt=5.0, wind=3.0)
        i = Inputs(**{**base, **fault})
        before = c.state
        out = c.step(i, DT)
        check_scan(c, i, out, before)


def test_scripted_mission_reaches_every_state_without_faults():
    c = Controller(cfg=CFG)
    alt, arm, clamp, out, seen = 0.0, 0.0, 0.0, Outputs(), set()
    for cmd in MISSION:
        clamp += (out.clamp_target - clamp) * 0.6
        arm = min(1.0, max(0.0, arm + out.arm * 0.4))
        v = 0.4 * out.drive
        alt = max(0.0, alt + v)
        out = c.step(Inputs(cmd=cmd, clamp_force=clamp, altitude=alt, enc_vel=v, imu_vel=v, arm_stowed=arm <= 0,
                            arm_deployed=arm >= 1, landed=alt <= 0.02, zone_clear=True), DT)
        seen.add(c.state)
    assert {State.GROUND, State.CLAMPED, State.CLIMBING, State.HARVEST, State.DESCENDING} <= seen
    assert c.cuts == 2 and c.state == State.GROUND
