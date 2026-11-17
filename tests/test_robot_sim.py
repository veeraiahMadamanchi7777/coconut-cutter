"""Closed loop: real controller + mission executive + robot/trunk physics."""
import random
from dataclasses import replace

import pytest

from coconut_harvester.controller import Controller, State
from coconut_harvester.robot_sim import DT, Plant, controller_config, run_mission
from coconut_harvester.tree import Tree


def test_1000_harvest_missions_complete_without_faults(design):
    rng = random.Random(2026)
    for n in range(1000):
        tree = design.pop.sample(rng)
        peds = design.pop.bunches(rng, 45)
        r = run_mission(design, tree, peds, rng=rng)
        assert r.ok and not r.faults, (n, tree, r)
        assert r.cuts == len(peds)
        # analytic model used by the fleet sim agrees with the closed loop
        e = design.energy_tree(tree, peds, dwell_s=r.time_s - 2 * tree.height / design.speed)
        assert r.energy_wh == pytest.approx(e, rel=0.25)
        assert r.time_s >= 2 * tree.height / design.speed


def test_wet_bark_never_climbed(design):
    tree = design.pop.sample(random.Random(1))
    r = run_mission(design, tree, [50.0], wet=True)
    assert not r.ok and r.reason.startswith("wet") and r.cuts == 0


def test_rain_mid_climb_robot_holds_then_descends(design):
    tree = replace(design.pop.sample(random.Random(3)), height=18.0)
    peak = {"z": 0.0}

    def rain(t, p, c):
        if t > 15.0:
            p.wet = True
        peak["z"] = max(peak["z"], p.z)

    r = run_mission(design, tree, [55.0], disturb=rain)
    assert r.cuts == 0 and not r.ok
    assert 3.0 < peak["z"] < 18.0  # stopped part-way, came down under control


def test_sudden_slip_is_caught_and_held(design):
    tree = replace(design.pop.sample(random.Random(4)), height=15.0)
    p = Plant(design, tree, rng=random.Random(4))
    c = Controller(cfg=controller_config(design))
    state = {"z_at_slip": None}

    def oil_patch(t, plant, ctrl):
        if 10.0 < t < 10.5:
            plant.mu_override = 0.12          # e.g. sap / moss: wheels lose grip
            state["z_at_slip"] = state["z_at_slip"] or plant.z
        elif t >= 10.5:
            plant.mu_override = None

    r = run_mission(design, tree, [], plant=p, ctrl=c, disturb=oil_patch)
    assert "slip" in r.faults
    assert r.reason != "timeout"
    assert p.z == 0.0  # it still came down normally afterwards


def test_out_of_envelope_ridge_stalls_and_robot_comes_back(design):
    tree = Tree(height=14, d_base=300, d_top=250, lean=3, ridge_h=40, ridge_len=30, mu_dry=0.7, bunch_radius=0.4)
    r = run_mission(design, tree, [50.0])
    assert "stall" in r.faults and not r.ok and r.cuts == 0
    assert r.reason != "timeout"


def test_low_battery_forces_safe_return(design):
    tree = replace(design.pop.sample(random.Random(5)), height=20.0)
    p = Plant(design, tree, rng=random.Random(5), battery_wh=controller_config(design).reserve(0) + 6.0)
    r = run_mission(design, tree, [60.0, 60.0], plant=p)
    assert r.reason != "timeout"
    assert p.z == 0.0 and p.battery_wh > 0  # home with charge to spare


def test_link_loss_during_cut_stops_saw_and_robot_holds(design):
    tree = replace(design.pop.sample(random.Random(6)), height=10.0)
    p = Plant(design, tree, rng=random.Random(6))
    c = Controller(cfg=controller_config(design))
    log = []

    def radio(t, plant, ctrl):
        if not log and ctrl.state == State.HARVEST and plant.arm >= 1.0:
            log.append(plant.z)                 # link drops now, mid-harvest
        elif 0 < len(log) < 40:
            log.append(plant.z)
        lost = 0 < len(log) < 40
        return {"comms_ok": False} if lost else {}

    r = run_mission(design, tree, [55.0], plant=p, ctrl=c, disturb=radio)
    assert "radio link lost" in r.faults
    assert max(log[:40]) - min(log[:40]) < 1e-9   # robot did not move while link was down
    assert r.reason != "timeout"
