# Phase 2: Car facts

**Status:** planned. **Needs:** nothing. **Runs beside:** phase 1.

[← Phase 1](1-submodule-gate-benchmark.md) · [Overview](README.md) · [Phase 3 →](3-scale-and-course.md)

## Goal

One measured `sim_vehicle_spec` v2 file per car, so that the planner, the RC simulator in
race mode and the twin all read the same numbers, and no value the planner relies on at
the limit is a guess (spec, anti-goals).

## Where it lands

`src/param/autoware_individual_params/individual_params/config/<car>/autosdv_vehicle/vehicle_spec.yaml`
for `car_a`, `car_b`, `car_c`, plus `twin` (car A's, with the twin's photo-estimated mounts)
and `sim` (derived from car A's; read only by the race sim, never by the lab simulators).
Every value carries a provenance tag (`measured`, `estimate`, `literature`) as the twin's
spec does. These are new files: nothing existing in `individual_params` changes.

## Measurements, per car

| quantity | method | existing tool |
|---|---|---|
| mass | scale, with battery and sensors as raced | — |
| CoM, longitudinal | two scales under the axles | — |
| CoM height | tilt method (one axle raised, load change) | — |
| yaw inertia Iz | bifilar pendulum, or estimated from the mass layout and marked `estimate` | — |
| coast-down drag and rolling resistance | coast from several speeds on COSS paving | `just sim fit-coastdown` |
| tyre grip | constant-radius skidpad on COSS paving, speed raised until it slides | `autosdv_rc_sim.fit.skidpad_mu` |
| motor and ESC | PWM steps on the bench and on the ground | `just coach fit-motor` |
| servo rate | a step command on the bench, angle filmed or read from an IMU on the knuckle | — |
| steering ratio | already measured (A −249, B −166, C +132); restated, not re-measured | `actuator.param.yaml` |
| **differential type** | lift the rear, turn one wheel: same direction = locked/spool, opposite = open | — |
| braking | `brake_pwm` from a set of speeds, deceleration from the wheel speed and the IMU | — |

Brakes are rear-only through the ESC; each spec says so (`brakes.distribution` all rear,
`regen_*` zero).

## Tasks

- **2.1** A measurement procedure, `docs/guides/vehicle-spec-measurement.md`, one section
  per row above, with the recording commands and the fit that turns each recording into
  spec values.
- **2.2** Measure car A; write its spec; fix the twin's `autosdv_car_a.yaml`
  (`drive_axle: all` → `rear`, and the measured values) and re-run the twin's existing checks
  (`docs/guides/isaac-twin.md`), recording what changed.
- **2.3** Cars B and C.
- **2.4** A test in AutoSDV that loads each spec with autoware-drift's `Car.from_spec` (skipped
  when race_planner's dependencies are absent) and checks the provenance tags: the fields the
  planner uses at the limit (mass, CoM, tyre, motor, brakes) are `measured`.
- **2.5** Answer the spec's two modelling questions in a short report,
  `docs/reports/race-car-facts.md`: the differential type per car (if open, phase 3/7 need
  the open-differential model), and whether rear-only braking with the measured
  deceleration fits the planner's brake model.

## Exit

- Five spec files (A, B, C, twin, sim) committed; A, B, C's limit fields `measured`.
- The twin's spec is rear-drive and its checks re-run.
- The report answers both modelling questions.
- `autosdv_rc_sim/vehicle.yaml` and the `sim` instance's existing files are unchanged.
