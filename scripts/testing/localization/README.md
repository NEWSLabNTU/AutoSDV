# Outdoor Localization Testing Tools

This directory contains scripts for testing and diagnosing GPS-based localization outdoors.

## Map Configuration

- **Map Location**: `data/COSS-map-planning/`
- **Map Origin**:
  - Latitude: 25.0201° (COSS campus, Taiwan)
  - Longitude: 121.5423°
  - Altitude: 25.0 m
- **Projection**: TransverseMercator (WGS84)

## Testing Scripts

### 1. GPS Monitor (`monitor_gps.py`)

Displays real-time GPS data and converted meter coordinates.

**Usage:**
```bash
source install/setup.bash
./scripts/testing/localization/monitor_gps.py
```

**Shows:**
- Raw GPS latitude/longitude/altitude
- Distance from map origin
- GPS accuracy (horizontal/vertical error)
- Converted meter coordinates (X, Y, Z in map frame)

### 2. Localization Monitor (`monitor_localization.py`)

Compares GPS pose with NDT localization in real-time.

**Usage:**
```bash
source install/setup.bash
./scripts/testing/localization/monitor_localization.py
```

**Shows:**
- GPS pose (from GNSS)
- NDT pose (from LiDAR localization)
- Position differences (2D distance, height, heading)
- Status assessment and warnings
- Fused kinematic state

### 3. Map Boundary Checker (`check_map_bounds.py`)

Checks if GPS position is within the mapped area.

**Usage:**
```bash
source install/setup.bash
./scripts/testing/localization/check_map_bounds.py
```

**Shows:**
- Point cloud map boundaries (X, Y, Z ranges)
- Current GPS position
- Distance to map boundaries
- Warning if position is outside mapped area

### 4. Launch All Monitors (`launch_monitors.sh`)

Starts all monitoring tools in a tmux session for easy viewing.

**Usage:**
```bash
./scripts/testing/localization/launch_monitors.sh
```

**Tmux Controls:**
- `Ctrl+B, 0/1/2` - Switch between windows
- `Ctrl+B, d` - Detach (monitors keep running)
- `tmux attach -t outdoor-test` - Re-attach
- Exit: Close each window with Ctrl+C, then exit tmux

## NDT diagnostics

Live tools, run while a replay or a drive is in progress:

| Script | What it answers |
|--------|-----------------|
| `check_ndt_activated.py` | is `ndt_scan_matcher` ACTIVATED, not merely alive? Exits 0/1, so a harness can gate on it |
| `ndt_quality_report.py` | pose quality over a window — scatter, yaw step, init-to-result, exe time. Deliberately not NVTL |
| `ndt_alignment_report.py` | how far the live scan actually sits from the map, point by point. `--map` defaults to the COSS map |
| `ndt_timeseries.py` | every NDT signal against time, to CSV and PNG. For "which signal moved first?" |
| `check_imu_velocity.py` | the two inputs the EKF prior is built from. `--raw-topic` follows `imu_source` |
| `capture_initial_pose.py` | save the settled pose for a named site to `data/initial_poses/<name>.yaml` |
| `set_initial_pose.py` | replay that pose, so a run needs no human at the RViz window |

Offline tools, run against a recorded run directory:

| Script | What it answers |
|--------|-----------------|
| `summarize_ndt_run.py` | one run's diagnostics, split into init and tracking phase |
| `compare_ndt_runs.py` | several runs side by side; `--row` for machine-readable output |
| `ndt_yaw_bias.py` | heading-minus-course bias on straight segments — a mounting calibration error |
| `tegrastats_summary.py` | GPU busy, rail power and CPU load from a tegrastats log |
| `export_ndt_frames.py` | dump frames for the offline GPU-vs-CPU benchmark |

**On NVTL.** `ndt_quality_report.py` reports pose quality rather than NVTL on
purpose. NVTL is a mean per-point fit that rises with `ndt.resolution` and rises
again when imperfect far returns are cropped away, so maximising it selects for
coarse voxels and narrow crop boxes whether or not the pose improves. See
`docs/research/localization/ndt_parameter_tuning_coss_map.md`, whose headline
conclusions reversed when re-measured against pose quality.

**Note on the pose pair.** `demo/scripts/seed_initialpose.py` predates these and
stays wired into `run-coss-ndt.sh`; it carries one hardcoded COSS pose.
`capture_initial_pose.py` generalises that to a named file per site, and records
which source the pose came from — the planning simulator's own ground truth is
the click unchanged, while a replay's is NDT agreeing with the map. Both seed;
only the second is a measurement.

Both scripts read `/localization/kinematic_state`, i.e. after NDT converged and
the EKF settled, not the raw click.

## Outdoor Testing Workflow

### Pre-Test Checklist
1. ✓ U-Blox GPS connected and recognized (`/dev/ttyACM0` or `/dev/ublox-gps`)
2. ✓ Vehicle is in the mapped area (COSS campus)
3. ✓ Clear sky view for GPS signal
4. ✓ AutoSDV system running (`make launch` or `make play`)

### Step 1: Check GPS Signal
```bash
./scripts/testing/localization/monitor_gps.py
```
- Wait for GPS fix (status should show "FIX" or "SBAS FIX")
- Check horizontal error < 5m (ideally < 2m)
- Verify coordinates are being converted to meters

### Step 2: Verify Position is in Map
```bash
./scripts/testing/localization/check_map_bounds.py
```
- Confirm GPS position is within map bounds
- Check height (Z) is reasonable
- If outside bounds, move vehicle to mapped area

### Step 3: Monitor Localization
```bash
./scripts/testing/localization/monitor_localization.py
```
- Watch GPS vs NDT difference
- Should converge to < 2m if localization is working
- Monitor height difference (should be < 1m)

### Step 4: Initialize and Test
1. Open RViz (if not already open)
2. Wait for NDT to converge (may take 10-30 seconds)
3. Use "2D Pose Estimate" in RViz to set initial pose if needed
4. Monitor localization stability while moving

## Troubleshooting

### GPS has no fix (0.0, 0.0)
- Move to area with clear sky view
- Wait 1-2 minutes for GPS to acquire satellites
- Check GPS antenna connection

### GPS position outside map bounds
- You are not in the mapped area
- Move vehicle to COSS campus area
- Or create a new map for your current location

### Large GPS vs NDT difference (> 5m)
- Check GPS accuracy (should be < 5m horizontal error)
- Verify map projection settings match
- NDT may not have converged yet (wait longer)
- Use manual pose initialization in RViz

### Height mismatch (Z difference > 1m)
- GPS altitude may use different reference (ellipsoid vs geoid)
- Point cloud map height may need adjustment
- Check `map_projector_info.yaml` altitude setting (currently 25.0m)

## Topics Reference

### GPS Topics
- `/sensing/gnss/ublox/nav_sat_fix` - Raw GPS (lat/lon/alt)
- `/sensing/gnss/pose` - GPS in meter coordinates
- `/sensing/gnss/pose_with_covariance` - GPS pose with uncertainty

### Localization Topics
- `/localization/pose_estimator/pose` - NDT localization
- `/localization/kinematic_state` - Fused localization (GPS + NDT + odometry)
- `/initialpose` - Manual pose initialization from RViz

### Map Topics
- `/map/pointcloud_map` - Point cloud map
- `/map/vector_map` - Lanelet2 vector map

## Additional Commands

### Check GPS device
```bash
ls -l /dev/ttyACM0
ls -l /dev/ublox-gps
```

### Monitor GPS raw messages
```bash
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix
```

### Monitor localization pose
```bash
ros2 topic echo /localization/kinematic_state
```

### Check NDT score
```bash
ros2 topic echo /localization/pose_estimator/transform_probability
```

## Notes

- The map is centered at COSS campus (Taiwan)
- Coordinates are in meters relative to map origin (25.0201°N, 121.5423°E)
- Height reference is WGS84 ellipsoid at 25.0m
- NDT localization requires being within ~50m of mapped area
