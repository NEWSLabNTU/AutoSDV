# `steering_status` is the command, not a measurement

Date: 2026-07-30. Found while checking why `/vehicle/status/steering_status` is
identically 0.000 rad throughout `data/rosbags/outdoor_20251226_153115`, during
the CUDA NDT work (`docs/reports/cuda-ndt-coss-replay.md`).

## The zero in that bag is expected

Autonomy was never engaged for that recording:

| topic | value, whole bag |
|---|---|
| `/vehicle/status/control_mode` | `NOT_READY`, all 1568 messages |
| `/vehicle/engage` | `False`, all 1568 |
| `/vehicle/status/gear_status` | PARK (3593), NEUTRAL (1109), never DRIVE |
| `/vehicle/status/steering_status` | exactly 0.0, all 4710 |

`/control/command/control_cmd` is not in the bag at all. So the publisher had
nothing to report. **Every manually driven bag will read 0**, and no such bag can
be used to validate steering geometry or the lateral controller.

## The vehicle has no steering angle sensor

`autosdv_vehicle_interface/steering_status.py` subscribes
`/control/command/control_cmd`, clamps `lateral.steering_tire_angle` to
`max_steering_angle`, applies a first-order lag (`alpha = 1 / (response_time *
publish_rate) = 1/(0.1 * 30) = 1/3`) and publishes the result as a
`SteeringReport`. Its docstring says so plainly: "Since the vehicle lacks a
steering angle sensor, this node reports the commanded steering angle directly."

As a stand-in for a missing sensor that is a reasonable choice. The problem is
who believes it.

## Consumers treat it as a measurement

- `autoware_mpc_lateral_controller` takes the report as the controller's state
  (`calculateMPC(..., const SteeringReport & current_steer, ...)`). Feeding it
  the command means the lateral loop is closed on its own output: servo lag,
  deadband, saturation and linkage play are invisible to the controller, which
  models the actuator as ideal.
- `autoware_operation_mode_transition_manager` gates engagement on the command
  and the status agreeing. With an echo they cannot disagree, so that check
  passes unconditionally.

The lateral controller has therefore never had real feedback on this platform,
which is worth keeping in mind against the known steering complaints (see
CLAUDE.md, "Steering reversed").

## A real measurement is available from existing sensors

The kinematic bicycle relation gives the steering angle from yaw rate and speed:

    delta = atan(wheel_base * yaw_rate / speed)

With `wheel_base: 0.319`, the ZED IMU's yaw rate, and the wheel speed corrected
by the 0.5 factor from the hall-sensor scale error, over the drive segment of the
COSS bag:

| | |
|---|---|
| samples (v > 0.3 m/s) | 708 |
| mean | -0.003 rad, i.e. ~0 as expected for a near-straight drive |
| sd | 0.056 rad |
| range | -0.195 .. +0.294 rad |
| p5 / p50 / p95 | -5.3 deg / -0.1 deg / +3.5 deg |
| within the +/-0.349 rad actuator limit | 100 % |

Plausible, in range, and derived from sensors the vehicle already carries.

Limits, before anyone wires it in:

- **Undefined at low speed.** `yaw_rate / speed` blows up as speed falls, so it
  needs a fallback below ~0.3 m/s -- the same threshold the steering controller
  already uses for its own fallback mode.
- **It inherits the wheel-speed scale error.** That reads about 1.8x high today
  (`docs/reports/cuda-ndt-coss-replay.md`, finding 4), and speed enters the
  denominator, so the estimate comes out systematically small until the hall
  sensor is fixed. Fix that first.
- **It measures achieved path curvature, not tyre angle.** Linkage play and tyre
  slip do not show up; at 1 m/s on pavement slip is negligible, play is not.
- **It closes a loop through the IMU.** Feeding a gyro-derived estimate to MPC
  needs filtering; an unfiltered estimate may be worse than a clean lie.

## Options

1. **Add a real sensor.** A magnetic encoder (e.g. AS5600, a few dollars) on the
   steering linkage measures the actual angle and removes the guesswork. The
   only option that makes `steering_status` mean what its consumers assume.
2. **Publish the kinematic estimate** above the speed threshold and the command
   echo below it, low-pass filtered. Real feedback with no hardware change.
   Should land behind a parameter, defaulting off, and be validated against a
   controlled test (drive a known-radius circle and compare).
3. **Leave it and make the fiction visible.** Warn at startup that no sensor is
   present, and consider not publishing a confident 0.0 while disengaged, so a
   recording cannot be mistaken for measured data.

None is implemented. Option 2 is cheap but touches the lateral control loop, so
it wants a deliberate decision and a test drive rather than a quiet default.

## How to check this on the vehicle

```bash
# is the report just the command, delayed?
ros2 topic echo /control/command/control_cmd --field lateral.steering_tire_angle
ros2 topic echo /vehicle/status/steering_status --field steering_tire_angle

# what the bicycle model says at the same moment
ros2 topic echo /sensing/imu/imu_data --field angular_velocity.z
ros2 topic echo /vehicle/status/velocity_status --field longitudinal_velocity
```

Drive a steady circle of measured radius R: the true angle is
`atan(wheel_base / R)`. Compare all three.
