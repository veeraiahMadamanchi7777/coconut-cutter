# coconut-harvester

Design, safety controller and plantation-scale verification of a **coconut-palm climbing and
harvesting robot**. Two people clamp a ring robot around the trunk. It climbs to the crown, the
operator cuts ripe bunches with an arm-mounted circular saw using the mast camera, and the robot
comes back down. Nobody climbs the tree.

**Full report:** [`docs/DESIGN.md`](docs/DESIGN.md). It is generated from the model and includes
the drawing, mass breakdown, checks, fleet sizing and safety functions.

![robot](docs/robot.svg)

## Headline numbers

| | |
|---|---|
| Robot | 39.3 kg; 2 clamp tiers × 3 Ø150 PU wheels, 530 N preload each; 6 self-locking worm gearmotors |
| Climb | 0.39 m/s on trunks Ø143–420 mm, up to 22 m and 20° lean, over leaf-scar ridges |
| Saw | Ø235 mm TCT blade, 150 W BLDC, 0.7 m reach, cuts a 75 mm stalk in under 10 s |
| Energy | ~7 Wh per tree, so about 29 trees per 10 Ah pack; 3 hot-swap packs per robot |
| Plantation | **17 robots (+1 spare)** harvest 17,500 palms (100 ha) on a 45-day round, including monsoon |

## Layout

```
coconut_harvester/
  tree.py        palm population (fit to a field survey) + monsoon weather calendar
  design.py      grip/tipping/ridge physics, drive, clamp, saw, battery, mass roll-up, optimiser
  controller.py  supervisory controller with safety functions SF1-SF8
  robot_sim.py   closed loop: controller + mission executive + robot/trunk physics
  fleet.py       year-long plantation simulation, fleet sizing to a service level
  report.py      docs/DESIGN.md and docs/robot.svg
```

The design converges mass, grip force, motors and battery together, because each depends on the
others. `optimize()` then picks the lightest catalogue combination that passes every check.

## Run

```bash
pip install -e ".[test]"
pytest -q                      # 47 tests, ~2-3 min on 4 cores
pytest -q -s -m prodscale      # 1,000 ha estate (175k palms) + 20k closed-loop missions
coconut-harvester report       # regenerate docs/ (CI fails if it drifts)
coconut-harvester simulate --robots 17
```

## What the tests prove

- **Physics and design.** All 15 checks pass, including climb traction on the worst ridge, wet
  hold, wheel unloading at full reach and maximum lean, self-locking drives, cut time, battery
  reserve, carry mass and the belay tether. 200,000 sampled palms are all climbable when dry
  and holdable when wet. For random assumption sets, any design that reports `ok` really
  serves its population.
- **Safety.** SF1–SF8 are checked on every scan under three kinds of fuzz: random input streams,
  plausible-plant streams with injected faults, and full harvest scripts with faults injected
  at arbitrary points.
  - **Mutation testing:** each of 10 safety rules was deleted in turn, and the tests caught every one.
  - **Found by the fuzz:** the robot kept climbing after the operator released the button.
    It is now hold-to-run (SF8).
- **Closed loop.** 1,000 full harvests run with zero faults, and the energy and timing match the
  analytic model.
  - **Wet bark:** never climbed. Rain mid-climb makes the robot stop and come down.
  - **Disturbances:** the robot catches a sudden slip and holds. An out-of-envelope ridge stalls
    it and it returns. A low battery forces it home with charge to spare. A radio drop
    mid-cut stops the saw in the same scan.
- **Plantation scale.** 17 robots meet the round in 27 of 30 simulated weather years, and still
  meet the service level on 10 fresh years. 16 robots do not. A single battery pack per robot
  starves; three never do.

## Assumptions to replace before building

The tree population, bark friction (especially wet) and leaf-scar ridge geometry are
engineering estimates, and they drive the clamp force and the whole mass budget. Survey at
least 50 palms per estate, fit them with `TreePopulation.from_survey()`, measure stalk cutting
energy with the chosen blade, and regenerate. Then prototype tests: slip and pull-off on real
trunks, 1,000-climb endurance, belay drop test, rain exposure, and ISO 13849-2 validation of the
safety functions. Finally, calibrate the fleet model's mounting and aiming times on a pilot block.
