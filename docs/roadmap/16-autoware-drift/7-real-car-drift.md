# Phase 7: Real car, drift

**Status:** planned. **Needs:** phase 6 (grip laps on the car), phase 5 (vy that holds on
real laps) and phase 2 (the differential type and braking numbers).

[← Phase 6](6-real-car-grip.md) · [Overview](README.md)

## Goal

The planner's `fastest` style chooses a drift where it is faster, and the car holds the
planned slide on COSS within the 20° steering lock's limits. Expect mild slides: the lock
bounds the sideslip a drift can hold.

## Tasks

- **7.1 Open differential** (autoware-drift, if phase 2 found one): an axle model with an
  equal torque split, where the unloaded inner wheel limits drive; drift styles accept it.
  Behind the regression gate.
- **7.2 Rear-only braking** in the planner's brake model, if phase 2's report says the
  current model cannot express it.
- **7.3 Small-car drift data**: rebuild the corner library and drift maps for the small-car
  profile (the bundled ones are drift_mustang's), on the twin.
- **7.4 On the twin**: planned drifts on the COSS course; spin detection exercised (an MRM
  fault, the car stops inside the course).
- **7.5 On the car**: car A at a low cap, one corner first, with the procedure of phase 6;
  then the lap; then B and C.

## Exit

- A planned drift executed on COSS by car A: sideslip and yaw rate tracked against the plan,
  in the phase's report with the bag.
- Spin detection shown to stop the car on the twin.
