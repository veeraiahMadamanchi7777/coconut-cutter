"""Safety & function tests for the station controller."""
import random
from dataclasses import replace

from hypothesis import given, settings, strategies as st

from coconut_cutter.coconut import Coconut
from coconut_cutter.controller import Config, Controller, Inputs, Outputs, State

DT = 0.01
OK = Inputs(nut_present=True)


def run(c: Controller, i: Inputs, seconds: float) -> Outputs:
    out = Outputs()
    for _ in range(max(1, round(seconds / DT))):
        out = c.step(i, DT)
    return out


# ---------------------------------------------------------------- functional

def test_normal_cycle():
    c = Controller()
    out = c.step(replace(OK, start=True), DT)
    assert c.state == State.EXTENDING and out.extend and not out.retract
    out = c.step(replace(OK, start=True, at_home=False), DT)
    assert out.extend
    c.step(replace(OK, at_home=False, at_depth=True), DT)
    assert c.state == State.DWELL
    out = run(c, replace(OK, at_home=False, at_depth=True), 0.2)
    assert c.state == State.RETRACTING and out.retract and not out.extend
    c.step(replace(OK, at_home=True), DT)
    assert c.state == State.IDLE and c.total_cuts == 1


def test_no_start_without_nut_or_away_from_home():
    c = Controller()
    assert not c.step(Inputs(nut_present=False, start=True), DT).extend
    c = Controller()
    assert not c.step(replace(OK, at_home=False, start=True), DT).extend


def test_anti_tie_down_start_must_be_released():
    c = Controller()
    held = replace(OK, start=True)
    c.step(held, DT)
    c.step(replace(held, at_home=False, at_depth=True), DT)
    run(c, replace(held, at_home=False, at_depth=True), 0.2)
    c.step(held, DT)
    assert c.state == State.IDLE
    # start still held from the previous cycle -> no new stroke
    for _ in range(50):
        assert not c.step(held, DT).extend
    c.step(OK, DT)  # release
    assert c.step(held, DT).extend  # re-press


def test_jam_timeout_faults_and_needs_reset():
    c = Controller()
    c.step(replace(OK, start=True), DT)
    out = run(c, replace(OK, at_home=False), c.cfg.extend_timeout + 0.1)
    assert c.state == State.FAULT and not out.extend and c.faults == 1
    assert not run(c, replace(OK, at_home=False), 1.0).retract   # no auto-recovery
    out = c.step(replace(OK, at_home=False, reset=True), DT)
    assert c.state == State.RETRACTING and out.retract


def test_guard_open_mid_stroke_stops_and_never_resumes_by_itself():
    c = Controller()
    c.step(replace(OK, start=True), DT)
    out = c.step(replace(OK, at_home=False, guard_closed=False), DT)
    assert c.state == State.STOPPED and not out.extend and not out.retract
    out = run(c, replace(OK, at_home=False), 2.0)  # guard closed again
    assert c.state == State.STOPPED and not out.extend
    c.step(replace(OK, at_home=False, reset=True), DT)
    assert c.state == State.RETRACTING


def test_estop_dumps_air_in_any_state():
    for prefix in (0, 1, 3):
        c = Controller()
        c.step(replace(OK, start=True), DT)
        run(c, replace(OK, at_home=False), DT * prefix)
        out = c.step(replace(OK, estop_ok=False), DT)
        assert out == Outputs(extend=False, retract=False, air_on=False)
        assert c.state == State.ESTOP
        # releasing the e-stop alone does nothing
        assert run(c, replace(OK, at_home=False), 0.5) == Outputs()
        c.step(replace(OK, at_home=False, reset=True), DT)
        assert c.state == State.RETRACTING


def test_low_pressure_faults():
    c = Controller()
    c.step(replace(OK, start=True), DT)
    out = c.step(replace(OK, at_home=False, pressure_ok=False), DT)
    assert c.state == State.FAULT and not out.extend


def test_blade_swap_lockout():
    c = Controller(cfg=Config(blade_interval=3))
    for _ in range(3):
        c.step(OK, DT)
        c.step(replace(OK, start=True), DT)
        c.step(replace(OK, at_home=False, at_depth=True), DT)
        run(c, replace(OK, at_home=False, at_depth=True), 0.2)
        c.step(OK, DT)
    assert c.state == State.BLADE_DUE
    c.step(OK, DT)
    assert not c.step(replace(OK, start=True), DT).extend
    c.step(replace(OK, reset=True, blade_swapped=True), DT)
    assert c.state == State.IDLE and c.cuts_since_swap == 0


# --------------------------------------------------------------- safety fuzz

inputs = st.builds(
    Inputs,
    guard_closed=st.booleans(), curtain_clear=st.booleans(), estop_ok=st.booleans(),
    pressure_ok=st.booleans(), nut_present=st.booleans(), at_home=st.booleans(),
    at_depth=st.booleans(), start=st.booleans(), reset=st.booleans(), blade_swapped=st.booleans(),
)
# Bias towards mostly-healthy sequences so deep states are actually reached.
healthy = st.builds(
    Inputs,
    guard_closed=st.just(True) | st.booleans(),
    curtain_clear=st.just(True) | st.booleans(), estop_ok=st.just(True) | st.booleans(),
    pressure_ok=st.just(True), nut_present=st.just(True) | st.booleans(),
    at_home=st.booleans(), at_depth=st.booleans(), start=st.booleans(),
    reset=st.booleans(), blade_swapped=st.booleans(),
)


@settings(max_examples=2000, deadline=None)
@given(st.lists(st.one_of(inputs, healthy), min_size=1, max_size=200),
       st.lists(st.floats(0.001, 0.5), min_size=1, max_size=200))
def test_safety_invariants_hold_for_any_input_sequence(seq, dts):
    c = Controller(cfg=Config(blade_interval=5))
    prev_start = False
    for k, i in enumerate(seq):
        before = c.state
        out = c.step(i, dts[k % len(dts)])
        # SF1/SF3: the punch only moves down when a hand cannot be in the zone
        if out.extend:
            assert i.guard_closed and i.curtain_clear and i.estop_ok and i.pressure_ok and i.nut_present
            assert out.air_on and not out.retract
        # retracting also needs the zone guarded
        if out.retract:
            assert i.guard_closed and i.curtain_clear and i.estop_ok
        # SF2: e-stop always dumps air
        if not i.estop_ok:
            assert out == Outputs()
        # SF4/SF5: a new downstroke only begins from IDLE on a fresh start edge
        if c.state == State.EXTENDING and before != State.EXTENDING:
            assert before == State.IDLE and i.start and not prev_start
        # SF5: no path from a stop/fault/e-stop back to motion without reset
        if before in (State.STOPPED, State.FAULT, State.ESTOP) and c.state not in (before, State.ESTOP):
            assert i.reset
        prev_start = i.start


# -------------------------------------------------- closed loop with a plant

class Cylinder:
    """Kinematic plant: moves at design speeds, stalls if the cut needs more force than available."""

    def __init__(self, design, nut: Coconut, dull: float):
        self.d, self.nut = design, nut
        self.x = 0.0
        self.travel = design.travel(nut)
        self.contact = self.travel - design.depth_cut
        self.stall = design.required_force(nut, dull) > design.force_available

    def step(self, out: Outputs, dt: float) -> None:
        if out.extend:
            nxt = self.x + self.d.kin.extend_speed * dt
            if self.stall and nxt > self.contact + self.nut.husk_top:
                nxt = self.x  # stalled in the shell
            self.x = min(nxt, self.travel)
        elif out.retract:
            self.x = max(0.0, self.x - self.d.kin.retract_speed * dt)

    def sensors(self, **kw) -> Inputs:
        return Inputs(nut_present=True, at_home=self.x <= 0.5,
                      at_depth=self.x >= self.travel - 1e-9, **kw)


def cycle(c: Controller, cyl: Cylinder, max_t: float = 5.0) -> float:
    t, out = 0.0, c.step(cyl.sensors(start=True), DT)
    while t < max_t:
        cyl.step(out, DT)
        t += DT
        out = c.step(cyl.sensors(), DT)
        if c.state in (State.IDLE, State.BLADE_DUE, State.FAULT):
            break
    return t


def test_closed_loop_10k_nuts_with_crown_swap_no_faults_and_cycle_time_matches_model(design):
    c = Controller(cfg=Config(blade_interval=design.req.blade_resharpen_cuts))
    rng = random.Random(1)
    for n in range(10_000):
        nut = design.pop.sample(rng)
        dull = 1 + (design.req.blade_dull_at_swap - 1) * (c.cuts_since_swap / c.cfg.blade_interval)
        cyl = Cylinder(design, nut, dull)
        t = cycle(c, cyl)
        assert c.state in (State.IDLE, State.BLADE_DUE), (n, c.state)
        if c.state == State.BLADE_DUE:  # operator swaps the crown and resets
            c.step(cyl.sensors(reset=True, blade_swapped=True), DT)
            assert c.state == State.IDLE
        # sampled control loop adds at most a few scans to the analytic machine time
        assert abs(t - (design.machine_time(nut) - 2 * design.kin.valve_response)) < 6 * DT
    assert c.faults == 0 and c.total_cuts == 10_000
    assert c.cuts_since_swap == 10_000 - design.req.blade_resharpen_cuts


def test_closed_loop_out_of_spec_nut_faults_instead_of_crushing(design):
    rock_hard = Coconut(180, 220, husk_top=20, shell=5, strength=9.0, mass=1.8)
    c = Controller()
    cyl = Cylinder(design, rock_hard, 1.0)
    assert cyl.stall
    cycle(c, cyl)
    assert c.state == State.FAULT and c.faults == 1
    # operator resets -> machine returns home and is usable again
    out = c.step(cyl.sensors(reset=True), DT)
    while c.state == State.RETRACTING:
        cyl.step(out, DT)
        out = c.step(cyl.sensors(), DT)
    assert c.state == State.IDLE and cyl.x == 0.0
