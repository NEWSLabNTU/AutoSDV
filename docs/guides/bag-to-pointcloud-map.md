# From a recording to a point cloud map (`just bag record` -> `just bag map`)

Drive the car through a space while it records, then build a point cloud map
of the space with GLIM (LiDAR-IMU mapping). The mapping step is Lab 3's
(`lab3_map`), run on a general recording instead of a Lab 3 one.

```
once per car   calibration, PTP time, GLIM          (section 1)
each map       sensors up, clock check              (section 2)
               just bag record      drive, Ctrl-C   (section 3)
               just bag map car_b                   (section 4)
               check the map                        (section 5)
```

## 1. Once per car

**The ZED's pose in the car's calibration.** GLIM needs the transform from the
LiDAR to the IMU, and the IMU is the ZED's. It is
`sensor_kit_base_link -> zedxm_camera_link` in
`individual_params/config/<car>/autosdv_sensor_kit/sensor_kit_calibration.yaml`.
Measure it; a placeholder makes the map wrong without any error. Car B's
(2026-11-06): x 0.072, y -0.009, z -0.14, roll 0.002, pitch 0, yaw 0.012. The
camera is 14 cm below the Robin-W.

**PTP time for the Robin-W.** The Robin-W stamps its clouds with its own clock,
which the Orin serves over PTP. Two services must both run:

```bash
systemctl is-active ptp4l phc2sys        # both: active
```

If `phc2sys` is not running, every cloud is stamped 37 s in the past: the gap
between TAI, which PTP uses, and UTC. GLIM then aborts (section 6). The repo's
unit has started reliably at boot since 6b80843. On a car set up before that,
reinstall it once:

```bash
./setup.sh --rerun robin-w-ptp
sudo systemctl start phc2sys
```

**GLIM.** `./setup.sh --run --only glim --yes` installs the build that matches
the CUDA toolkit (12.6 on JetPack 6.2).

## 2. Before each recording

Start the car's sensors, for example the full stack:

```bash
just launch vehicle_id:=car_b
```

Then check in a second terminal:

```bash
ros2 topic hz /sensing/lidar/concatenated/pointcloud     # about 10 Hz
ros2 topic hz /sensing/camera/zedxm/imu/data             # about 100 Hz
ros2 topic echo /sensing/lidar/concatenated/pointcloud --field header.stamp --once; date +%s.%N
```

The cloud's stamp and `date` must agree to within a fraction of a second. A gap
of about 37 s means `phc2sys` is down (section 1). Do not record until it is fixed.

## 3. Record

`scripts/rosbag/outdoor_topics.txt` lists what is recorded. GLIM needs these
four:

```
/sensing/camera/zedxm/imu/data
/sensing/lidar/concatenated/pointcloud
/tf_static
/tf
```

`/tf` is the only source of the ZED's camera -> IMU link: the driver broadcasts it
there and not on `/tf_static`. Then:

```bash
just bag record          # writes rosbags/outdoor_YYYYMMDD_HHMMSS/
```

Drive like this:

- **Stand still for about 8 s** at the start, and again at the end. GLIM
  estimates the IMU's bias and gravity from those seconds.
- **Drive slowly**, at walking pace or below (Lab 3 uses 0.8 m/s at most), and
  turn gently. Fast turns blur the scans.
- **Cover everything you want in the map**, and drive past each feature more
  than once. GLIM adds a keyframe every 0.15 m of travel, so a stop adds
  nothing.
- **End where you started.** Returning to a place lets GLIM close the loop and
  remove the drift.

Stop with **Ctrl-C** and wait for the recorder to exit. It writes
`metadata.yaml` on exit, and a recording without that file cannot be read.

## 4. Build the map

```bash
just bag map car_b                          # the newest rosbags/outdoor_*
just bag map car_b rosbags/outdoor_X        # a chosen recording
just bag map car_b rosbags/outdoor_X --cpu  # extra arguments go to lab3_map
```

The car argument selects GLIM's settings for its LiDAR (the car's preset:
`vlp16_zed` is vlp16, `robin_zed` is robin_w). Nothing in the recording
identifies the LiDAR.

Read the `T_lidar_imu` line it prints before trusting the result. A good one
names its source like this:

```
T_lidar_imu [...]  [bag /tf_static (base_link <- zedxm_imu_link); from /tf: zedxm_imu_link]
```

A `WARNING: T_lidar_imu is identity` means the recording had no path from the
LiDAR to the IMU, and the map is built with the IMU in the wrong place (section 6).

The output goes beside the recording, in `rosbags/outdoor_X_glim/`:

| file | what |
|---|---|
| `raw_map.pcd` | the map, in GLIM's frame (where the drive started), with intensity |
| `glim_report.yaml` | the inputs, the transform and its source, timings, the map's size |
| `glim_run.log`, `glim_log/` | GLIM's own output, for when it fails |

## 5. Check the map

The summary line gives the point count, the extent and the spacing, for
example `10,192 points in 1 submaps, 7.3 x 15.3 x 3.0 m, spacing 3.2 cm`.
Compare the extent with the space you drove. Then open `raw_map.pcd` in a point
cloud viewer such as CloudCompare, and look for:

- **Doubled walls**: the loop did not close. Record again, ending where you started.
- **A tilted or bent floor**: the IMU transform is wrong. Check the `T_lidar_imu`
  line and the car's ZED pose (section 1).
- **Smeared features**: the drive was too fast, or the turns too sharp.

For Lab 3 the next step moves the map into the board's frame:
`just lab3 anchor rosbags/outdoor_X_glim/raw_map.pcd MAP --car car_b`
(`docs/guides/lab3.md`).

## 6. When it fails

| symptom | cause | fix |
|---|---|---|
| `No outdoor recordings in rosbags/` | none yet; or made before bc4fd6c, which wrote them to `scripts/rosbags/` | pass the path, or `mv scripts/rosbags/outdoor_* rosbags/` |
| `... has no metadata.yaml` | the recorder did not exit cleanly | record again, and stop with Ctrl-C |
| `WARNING: T_lidar_imu is identity` | no `/tf` in the recording (the ZED's IMU link), or no `/tf_static` | add both to `outdoor_topics.txt` and record again; or pass `--T-lidar-imu "x y z qx qy qz qw"` |
| GLIM log: `large time difference between points and imu`, then `IndeterminantLinearSystemException` | the clouds' stamps are 37 s old: `phc2sys` is down | section 1, then record again. The stamps are in the recording, so it cannot be repaired |
| `unrecognized arguments` | a mistyped option to `lab3_map` | `ros2 run autosdv_lab3 lab3_map --help` |
| `sensor_suite is '...', not a LiDAR GLIM is set up for` | the car's preset names another sensor | GLIM is set up for the VLP16 and the Robin-W only |

A recording whose ZED pose was wrong in `/tf_static` can still be mapped:
compute the transform with the right pose and pass it with `--T-lidar-imu`.
