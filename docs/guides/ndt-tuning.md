# Tuning NDT localization: process, pitfalls, checklist

Written after a day spent making CUDA NDT work on the COSS map, where the
matcher looked broken and was not: it was being fed a frozen pose prior by a
missing TF, and the parameters everyone reaches for first would have been tuned
to compensate for that. Findings and numbers:
`docs/reports/cuda-ndt-coss-replay.md`.

The short version: **NDT is usually the last thing wrong.** It sits at the end
of a chain -- TF, IMU, twist, EKF, preprocessing, map -- and it fails loudly
when something upstream fails quietly. Tune it only after the chain is proven.

## The process

### 1. Get a repeatable replay before touching any parameter

A parameter study is worthless if runs are not comparable. Fix the bag, the
map, the initial pose, and the set of nodes; record every diagnostic topic; then
change exactly one thing per run.

```bash
just demo run                  # stack, seeded pose, replay, metrics
just demo report               # metrics for the most recent run
just demo compare a=<dir> b=<dir>
```

`just demo run` fixes the bag, the map, the initial pose and the node set for
you, which is most of what makes runs comparable. For a bare diagnostic replay
without the demo's workarounds:

```bash
scripts/testing/localization/run-ndt-replay.sh <label>
python3 scripts/testing/localization/summarize_ndt_run.py <run_dir>
```

Seed the initial pose deliberately rather than letting GNSS do it, or successive
runs start from different places and nothing is comparable. On the COSS bag,
GNSS auto-init landed 5 m apart run to run.

Disable perception, planning and control while working on localization. Fewer
moving parts, less GPU contention, faster runs.

### 2. Prove the prior chain before blaming the matcher

NDT does not estimate a pose from nothing. It refines the prior the EKF hands
it. If that prior is stale, the match degrades no matter what the matcher does.
Check, in order:

```bash
ros2 topic hz /sensing/imu/imu_data                          # IMU corrected?
ros2 topic hz /localization/twist_estimator/twist_with_covariance   # twist?
ros2 topic hz /localization/kinematic_state                  # EKF propagating?
ros2 run tf2_ros tf2_echo base_link <sensor_frame>           # every sensor TF
```

A zero-rate topic here is the bug. Read the *other* nodes' logs, not just the
matcher's: `play_log/latest/node/*/err`. The COSS failure announced itself as
15320 copies of `Please publish TF base_link to zedxm_imu_link` from
`imu_corrector`, a node nobody was looking at, while `ndt_scan_matcher` merely
reported low scores.

Sanity check that the EKF actually moves: compare its displacement over a drive
against the wheel odometry and against the map. A pose that travels 0.5 m while
the vehicle travels 20 m is the whole problem.

### 3. Verify what is actually running

- **Release build.** `-DCMAKE_BUILD_TYPE=Release` does not reach Rust packages.
  `just build` passes `--cargo-args --release`; if you invoke colcon by hand,
  pass it too. A debug `cuda_ndt_matcher` runs ~8x slower and drops scans.
- **The config that is loaded, not the one you edited.** Several
  `crop_box_filter_measurement_range.param.yaml` exist in this repo. Ask the
  running node:
  ```bash
  ros2 param get /localization/util/crop_box_filter_measurement_range max_x
  ```

### 4. Change one variable, and cross the variables you cannot separate

When two candidate causes are confounded, a 2x2 costs two extra runs and
settles it. Resolution 2.0-vs-4.0 crossed with crop 20-vs-60 m reversed the
conclusion drawn from either alone.

### 5. Validate against something independent of NDT

NDT's own scores cannot tell you whether NDT is right. Use references it does
not depend on:

| question | independent reference |
|---|---|
| is yaw *rotating* correctly? | integrate the gyro over the same window |
| is yaw *pointing* correctly? | course over ground on straight segments |
| is the distance right? | wheel odometry, and the map's own scale |
| does the pose fit the world? | scan-to-map nearest-neighbour distance |
| is the map trustworthy far out? | per-cell residual seen from several poses |

## Pitfalls

### NVTL gates convergence; it does not rank quality

`nearest_voxel_transformation_likelihood` is a mean per-point fit, compared
against `converged_param_nearest_voxel_transformation_likelihood` to decide
whether to publish. It is **not** an accuracy metric, and tuning to maximise it
inverts the answer:

- It scales with `ndt.resolution`. Coarser voxels raise NVTL while *lowering*
  accuracy -- on COSS, resolution 4.0 scored 4.84 against 2.0's 2.90 and had 5x
  the position scatter.
- Honest but imperfect far returns lower the mean while improving the pose.
  Widening the crop dropped NVTL 4.84 -> 4.65 and halved yaw error.

Rank instead on: per-frame position scatter, frame-to-frame yaw step,
`initial_to_result_distance`, and publish continuity.

### A low NVTL means a bad prior at least as often as a bad matcher

If NVTL sits near its threshold, look upstream before touching the matcher. On
the broken stack NVTL was 2.22 against a 2.3 gate and 74 % of frames were
rejected. Same resolution, healthy prior: 2.90 and 4 rejections.

Worse, the failure is self-reinforcing: rejected frames stop publishing, the
prior gets staler, the score falls further. A cliff in the published-pose rate
is the signature.

### `initial_to_result_distance` is the most informative single number

It is how far NDT had to move the prior. Split it by motion:

- **parked** near zero (0.02-0.05 m) means prior and matcher agree,
- **moving** much larger means something in the prediction path is wrong --
  twist scale, IMU, or an extrinsic.

On COSS, moving/parked ran 0.819 / 0.024 m. Halving the wheel-speed scale took
it to 0.282; fixing the LiDAR mounting yaw took it to 0.049.

### Frame-to-frame statistics hide slow drift

A yaw error that accumulates 30 deg over 10 s is only 0.3 deg per frame and
looks healthy in a p95-of-step metric. Always plot the absolute quantity over
time, bucketed, alongside an independent reference.

### Course over ground is only valid on straight segments

Comparing heading with the direction of travel is the way to catch a yaw
offset, but a finite chord lags the instantaneous heading through a turn, which
manufactures a bias that is not there. Restrict to |yaw rate| < ~2 deg/s. On
COSS this changed the estimate from a confounded -11.5 deg to a clean
-12.66 deg (sd 2.72).

### An uncalibrated mounting rotation looks exactly like a localization bug

If the LiDAR is yawed relative to `base_link` and the calibration says 0, NDT
places the *sensor* correctly and the trajectory looks right, but the reported
heading is off by the mounting error, constantly. It survives every score check
because the match itself is good.

Symptoms: heading differs from course over ground by a constant on straight
segments; yaw *changes* still agree with the gyro; `initial_to_result_distance`
is elevated while moving but fine at rest.

Check `sensor_kit_calibration.yaml` early, and be suspicious of a file full of
zeros or comments like `# random value`.

### A shared GPU makes every timing meaningless

CUDA NDT measured 67-83 ms per scan through a whole investigation, against the
~5 ms its package documents. The matcher was fine: another process held 26 GB of
the 32 GB card. On an idle GPU the same configuration runs at 2.6 ms parked and
4.1 ms driving.

The tell is that *only* the wall clock moves. Iterations and NVTL were identical
in both cases, because contention costs time, not convergence -- so a timing
regression with unchanged iteration counts points outside the algorithm.

Check before believing any number:

```bash
nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv
```

### Consumer GNSS is not ground truth

Before trusting `/sensing/gnss/pose`, check `nav_sat_fix.status` and whether
RTCM is flowing. The COSS bag's fix is `status: 0` with no NTRIP: 20 m of
horizontal scatter, altitude wrong by 30-50 m, and it disagreed with the actual
direction of travel. It is fine as a rough init seed and useless as a reference.

### A bag bakes in sensor errors

No parameter change can undo a wheel-speed scale error that is already recorded
in `/vehicle/status/velocity_status`. To test the hypothesis, republish a
rescaled copy and remap the bag's topic aside (`tmp/velocity_scaler.py` shows
the pattern) -- then fix the vehicle.

Check for dead signals too: COSS bags carry `steering_status` identically 0.

### The map's far field is not as good as its near field

Scan-to-map residual grows with range: on COSS, ground returns fit to 0.19 m at
10-20 m and 0.71 m at 40-60 m, and past 60 m the map simply ends. Vegetation
disagrees by ~0.5 m at *every* range. Before widening the measurement range,
measure whether the map deserves it.

To tell a warped map from moving vegetation: compare the residual for the same
map cell seen from several vehicle poses. Coherent across observers means the
map is displaced there; random means foliage.

## Checklist

Before tuning anything:

- [ ] Replay harness runs end to end and records diagnostics
- [ ] Initial pose seeded deliberately, identical across runs
- [ ] `just build` (release) -- confirm the installed binary is not `target/debug`
- [ ] IMU topic publishing, and `imu_corrector` log is clean
- [ ] Twist publishing; EKF displacement matches reality over a drive
- [ ] Every sensor TF resolves from `base_link`
- [ ] Params read back from the running node, not from the file you edited
- [ ] GNSS quality established before using it for anything
- [ ] GPU not shared with another process, if any timing is to be believed

Then, per run, record:

- [ ] published poses / aligned frames, and the worst publish gap
- [ ] `initial_to_result_distance`, split parked vs moving
- [ ] per-frame position scatter and yaw step
- [ ] NVTL -- as a gate check only, with its margin over the threshold
- [ ] iterations and `exe_time_ms` against the scan period

And validate before believing:

- [ ] yaw change vs integrated gyro, bucketed over time
- [ ] heading vs course over ground on straight segments only
- [ ] travelled distance vs wheel odometry and vs the map
- [ ] one changed variable per run, or a full cross of the confounded pair

## Parameters, and when to touch them

| parameter | reach for it when | do not use it to |
|---|---|---|
| `ndt.resolution` | the map's point density genuinely changed | buy NVTL margin |
| `converged_param_nearest_voxel_transformation_likelihood` | after re-measuring the score distribution at a new resolution | silence a symptom of a bad prior |
| crop box `min/max_x/y` | the map's far field is known good, and yaw is under-constrained | compensate for a mis-scaled twist |
| `max_iterations` | `exe_time_ms` fits the scan period with headroom | fix non-convergence caused by a stale prior |
| `initial_pose_estimation.particles_num` | Monte Carlo init is unreliable | improve tracking |

Values that are measurements, not tuning knobs -- get them right rather than
searching over them: sensor extrinsics, wheel diameter and encoder counts, IMU
frame, and the map itself.
