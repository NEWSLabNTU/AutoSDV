# Phase 5: Cameras and estimator

**Status:** planned. **Needs:** phase 4 (the race stack on the twin, whose truth judges the
estimator). Hardware: three ZED X Mini and a GMSL2 capture card per car.

[← Phase 4](4-ros-nodes-in-simulation.md) · [Overview](README.md) · [Phase 6 →](6-real-car-grip.md)

## Goal

The race estimator gives the planner and controller a state with a real lateral velocity:
vy from rig_localization on three ZED X Mini cameras (front-left, front-right, rear),
fused with NDT's pose, the IMU, the wheel speed and the steering echo. Judged against the
twin's truth, then on recorded real laps.

## Hardware and sensor kit (AutoSDV)

- **5.1** Mount the three cameras on car A (then B, C); check what the capture card offers
  for hardware sync across them, and record it.
- **5.2** Three-ZED sensor suites under new names (one per lidar: VLP-16 for A, Robin-W
  for B and C). The ZED wrapper per camera, by serial number, depth off. The front-left
  camera keeps the `zed` topic and frame names, so ArUco localization and the perception
  presets keep working; the others are `front_right` and `rear`. Existing suites unchanged.
- **5.3** Calibrate the three extrinsics to `base_link` (rig's fusion trusts them) into each
  car's `sensor_kit_calibration.yaml` for the new suite; a car's preset moves to the new
  suite only once this is done, and the ArUco and perception mounts are re-checked then.
- **5.4** Decide where the front ZED's current consumers go (stay on front-left, or move),
  and write it down.

## rig_localization (generic changes, in its repository)

- **5.5** Submodule at `src/localization/external/rig_localization`, `branch = main`.
  `rigloc_pipeline` skips itself when `libcuvslam.so` is missing instead of failing the
  configure (today a `FATAL_ERROR`). The cuVSLAM fetch becomes a setup step; GTSAM via
  rosdep.
- **5.6** A parameter that drops the wheel factor's lateral row (`BodyVelocityFactor`
  measures (longitudinal, lateral, 0) with a 0.1 m/s lateral sigma; AutoSDV's lateral
  velocity is a hard-coded 0, which would pull vy to zero in a slide).
- **5.7** Metric scale: first read rig-odom-sim's history for why the golf cart moved from
  stereo to monocular, and check that reason against these cameras; then stereo (each ZED
  X Mini is a stereo pair) or, failing that, the wheel factor's gate tuned to reject
  wheelspin. Record the choice.
- **5.8** Camera slots: front-left as `zed`, front-right as `right`, rear as `rear`, no code
  change; front-left's IMU is the estimator's.
- **5.9** Small-car settings, and a build and run on the Orin (phase 1's benchmark repeated
  with real streams if it could not run then).

## AutoSDV integration

- **5.10** A `rigloc` localization preset (`twist_source:=rigloc`, `gyro_odom` stays the
  default) and a per-car rig overlay (cameras, topics, the wheel scale from
  `rigloc_wheel_calib` against NDT on COSS).
- **5.11** The twin's spec and Isaac rig gain the three cameras at the measured mounts.

## autoware-drift

- **5.12 The race estimator's filter**: a single-track filter whose process model is the
  spec's tyre model, measuring NDT pose (with covariance), rig's twist, IMU yaw rate and
  specific force, the driven-wheel speed and the steering command; `/race/state`
  (`nav_msgs/Odometry`, `twist.linear.y` filled). Python or C++: builder's call. Truth
  relay from phase 4 stays as a mode for simulation.

## Exit

- On the twin, driving laps with slides: the estimator's vy against Isaac's truth, error
  distribution reported; the phase's report sets the acceptance bound from the planner's
  sensitivity (how much vy error moves its off-plan and spin triggers).
- rig's start-up (10–34 s of motion) handled by a warm-up lap before the race.
- On car A, recorded hand-driven laps on COSS: the estimator runs on the bag and its vy is
  plausible against NDT's pose differences; the same for B and C once mounted.
- `just build` without cuVSLAM unchanged; `just launch` with an existing suite unchanged.
