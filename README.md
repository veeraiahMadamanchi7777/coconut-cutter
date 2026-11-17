# coconut-harvester

Design, safety controllers and plantation-scale verification of a **coconut-palm climbing and
harvesting robot**. Nobody climbs the tree.

The repo holds two concepts, compared head to head in the same models:

- **v2, the reimagined design (recommended): inchworm with self-locking collars.** Two strap
  collars grip harder the more load they carry, and one electric cylinder walks them up the trunk.
- **v1: wheel ring.** Six driven wheels are clamped onto the trunk and climb on friction.

**Interactive 3D model:** [`docs/robot_3d.html`](docs/robot_3d.html) (open in a browser).

**Full report:** [`docs/DESIGN.md`](docs/DESIGN.md), generated from the models: drawings, checks,
comparison, sensitivity and safety.

![inchworm robot](docs/robot.svg)

## Why v2

| | Wheel ring (v1) | **Inchworm (v2)** |
|---|---|---|
| Mass | 39.3 kg | **19.0 kg** |
| Climbs wet bark | no | **yes** |
| Leaf-scar ridges matter | yes, they size the motors | no |
| Robots for a 100 ha / 17,500-palm estate | 17 | **7** |

Per tree the two are about equal: v2 climbs more slowly but straps on faster. **The fleet saving comes
entirely from working through the monsoon.** v1 loses about 10,000 robot-hours a year to wet bark.
Even under pessimistic assumptions v2 needs 9–12 robots. If it turned out not to climb wet bark
after all, it would need 17, the same as v1. So the first thing to measure is wet-bark self-lock
on real trunks.

## How v2 works

- **Collar:** a rubber V-pad on one side of the trunk and a steel-cored strap on the other, with a
  cantilever arm. Weight on the arm cocks the collar, giving normal force N = M/h. It holds when
  lever L ≥ h/(2μ), whatever the load.
- **Climbing step:**
  1. Release the upper collar.
  2. Extend: the upper collar rises.
  3. Lock the upper collar and release the lower one.
  4. Retract: the lower collar rises.
- **Fail-safe:** release servos are spring-return, so with no power both collars lock.
- **Harvest tensioner:** the physics model showed that with the saw swung to the far side, the
  collar would let go. A small self-locking strap tensioner clamps the upper collar before the arm
  may deploy.

## Layout

```
coconut_harvester/
  tree.py               palm population (fit to a survey) + monsoon weather
  inchworm.py           v2 physics & sizing: self-lock, tensioner, cylinder, arms, battery, mass
  inchworm_control.py   v2 controller (IW1-IW9), physics plant, mission executive
  design.py             v1 physics & sizing
  controller.py         v1 controller (SF1-SF8)
  robot_sim.py          v1 plant & mission executive
  fleet.py              year-long plantation sim, fleet sized to a service level (either concept)
  compare.py            v2 sensitivity study
  report.py             docs/DESIGN.md, docs/robot.svg, docs/wheel_robot.svg
```

## Run

```bash
pip install -e ".[test]"
pytest -q                      # 74 tests, ~3 min on 4 cores
pytest -q -s -m prodscale      # 1,000 ha estates + 20k (v1) / 5k (v2) closed-loop missions
coconut-harvester report       # regenerate docs/ (~5 min; CI fails if it drifts)
coconut-harvester simulate --robots 7 --concept inchworm
```

## What the tests prove (v2)

- **Design.** All 18 checks pass. 200,000 sampled palms are climbable and holdable on wet or dry bark.
  For random assumption sets, any design that reports `ok` really holds its population.
- **Safety.** IW1–IW9 are checked on every scan, under random input streams and under full harvest
  scripts with injected faults.
  - **Mutation testing:** 13 single-rule deletions; 12 are caught. The 13th is an equivalent mutant
    (defence in depth).
  - **Found by the fuzz:** a brake-and-relock path slackened the harvest tensioner while the arm was
    out. It is now enforced by construction.
- **Closed loop.** 500 missions, half on wet bark, finish with zero faults and zero falls.
  - **Sap patch:** the robot slips, faults and holds.
  - **Radio loss mid-step:** both collars lock and the robot doesn't move.
  - **Low battery:** the robot comes home with charge to spare.
- **Plantation.** 7 robots meet the harvest round in every one of 30 simulated weather years and
  in 10 fresh ones. 6 robots do not.

## Assumptions to replace before building

1. **Wet-bark self-lock rig:** measure collar slip load against lever on real trunks, wet and dry.
2. **Pilot block:** log how much of a rainy day is actually workable.
3. **Palm survey and stalk cutting energy:** fit `TreePopulation.from_survey()` and regenerate.
4. **Prototype tests:** collar and cylinder endurance, tensioner under saw vibration, belay drop
   test, and ISO 13849-2 validation of the safety functions.
