# Localization: open questions

Carried out of the 2026-07-28 CUDA NDT investigation
(`docs/reports/cuda-ndt-coss-replay.md`) and the work that followed. Everything
here is known, measured where it could be, and deliberately not yet fixed.

Ordered by what unblocks the most.

## 1. Wheel speed reads about 1.8x high

**Status:** measured in replay, not fixed. Vehicle-side.

The hall sensor integrates to 35.9 m over a drive the map says was about 20 m.
The EKF over-predicts between scans and NDT drags the pose back every frame;
replaying with the speed halved cut per-frame correction from 0.819 m to
0.282 m and visible scatter from 0.271 m to 0.096 m.

`autosdv_vehicle_interface/params/velocity_report.yaml` has
`wheel_diameter: 10.5` cm and `markers_per_rotation: 12`. A hall sensor
triggering on **both** edges of 12 markers yields 24 counts per revolution and
exactly doubles the reported speed; a wheel diameter wrong by 2x is not
plausible for a measured quantity.

**To settle it:** push the vehicle a tape-measured 10 m and integrate
`/vehicle/status/velocity_status`. If it reports ~18-20 m, set
`markers_per_rotation: 24` or debounce to one edge.

**Blocks:** every bag recorded so far carries the error, and the kinematic
steering estimate in item 4 has speed in its denominator.

## 2. Sensor extrinsics are inferred or unmeasured

**Status:** the LiDAR yaw is calibrated from replay; everything else is a
placeholder.

`vlp32c.yaw` is set to -0.2210 rad because with 0.0 the localised heading sat
12.66 deg (sd 2.72, n=88) off the vehicle's own direction of travel on straight
segments. Correcting it brought that to 0.07 deg and cut NDT's per-frame
correction from 0.127 m to 0.049 m, so the value is clearly better than zero --
but it is derived from one bag and absorbs whatever else is uncalibrated in that
axis.

Still untouched in `individual_params/.../autosdv_sensor_kit/sensor_kit_calibration.yaml`:

| frame | state |
|---|---|
| `vlp32c` x/y/z | zeros, unmeasured |
| `zedxm_camera_link` | "typical mounting" guesses |
| `imu_link` | z = -0.055, otherwise zeros |
| `gnss_base_link` | z = 0.055, commented `# random value` |
| `base_link -> sensor_kit_base_link` | identity |

**To settle it:** measure the mounts, or run a proper LiDAR-to-base_link
extrinsic calibration, and replace the inferred yaw with the measured one.

## 3. The ZED IMU transform is identity

**Status:** placeholder, and the vendor cannot supply it statically.

`zed_imu_tf.launch.xml` publishes `zedxm_left_camera_frame -> zedxm_imu_link` as
identity when the driver is not running, because zed_wrapper's URDF defines no
`imu_link` for any model. The driver knows the real per-unit extrinsic and
broadcasts it whenever it runs, so hardware is unaffected; only replay uses the
guess.

**To settle it:** with the driver up on the vehicle,

```bash
ros2 run tf2_ros tf2_echo zedxm_left_camera_frame zedxm_imu_link
```

and put the numbers in that launch file's args.

## 4. `steering_status` carries the command, not a measurement

**Status:** understood and documented
(`docs/reports/steering-status-has-no-feedback.md`), decision outstanding.

The vehicle has no steering angle sensor, so `steering_status.py` republishes
the commanded angle with a 100 ms lag. `autoware_mpc_lateral_controller` takes
that report as its controller state, so the lateral loop is closed on its own
output, and `operation_mode_transition_manager` gates engagement on a
command-versus-status agreement that cannot fail.

A real estimate is available from fitted sensors -- `atan(wheel_base * yaw_rate
/ speed)` landed at mean -0.003 rad over the COSS drive, entirely within the
+/-0.349 rad actuator limit -- but it needs a low-speed fallback, filtering
before it reaches MPC, and item 1 fixed first.

**Choice to make:** add an encoder on the linkage, publish the kinematic
estimate behind a parameter, or keep the echo and make the fiction visible.

## 5. No ground truth for COSS

**Status:** blocking any accuracy claim.

The replay establishes that NDT locks, tracks continuously, agrees with the gyro
on yaw to 0.35 deg, and puts heading within 0.07 deg of the direction of travel.
It cannot establish absolute position accuracy: this bag's GNSS is single point
(`status: 0`, no RTK, no RTCM in the bag) with ~20 m of scatter and an altitude
30-50 m off the map.

**To settle it:** an RTK-corrected run over the same route, or a survey of a few
map-frame landmarks the vehicle can be parked against.

## 6. The COSS map is mildly warped at its edges

**Status:** measured, not acted on.

Ground cells beyond 25 m, seen by at least three vehicle poses, show a residual
that is coherent across observers (median coherence 0.96, inter-pose spread
0.049 m) with median magnitude 0.237 m and p90 0.828 m. A displacement every
vantage point agrees on belongs to the map, not to the observation or to moving
foliage -- SLAM drift while mapping, at the decimetre scale.

Not currently limiting: the crop is 40 m and the ground fits to about 0.2 m
there. It would matter if the crop were widened past 60 m, where only 18 % of
returns land on any mapped cell at all.

Check it any time with `just demo map-quality`.

## 7. Perception: centerpoint fails to build its TRT engine

**Status:** noticed in passing, not investigated.

A demo run logged `Failed to setup TRT engine` for
`lidar_centerpoint`; the stack came up 83/84 composables and localization was
unaffected. Probably a stale or missing engine file, possibly GPU memory
pressure at the time.

## 8. Golf cart divergences (`~/repos/2026-golf-cart`)

Checked on 2026-07-31 against this repository's findings; only the
`transform_probability` defect applied and was fixed by bumping the submodule.
Two things were left for its owners:

- it ships `ndt.resolution: 4.0` while its own tuning study recommends 2.0, and
  our measurement says 4.0 costs real accuracy;
- `golfcart_sensor_kit_description/config/sensor_kit_calibration.yaml` and the
  `individual_params` copy have diverged (`vlp32c yaw` 0.0 vs -0.05, `falcon
  pitch` -1.5707963 vs -1.7707963). That may be deliberate under Autoware's
  nominal-versus-measured convention, but the description copy is the fallback
  whenever `config_dir` is not overridden.

Neither is verifiable without a golf cart bag.
