# AutoSDV Launch Package

This package provides the main launch system for AutoSDV, including launch files and Autoware configuration files with AutoSDV-specific customizations.

## Package Structure

```
autosdv_launch/
├── launch/             # Launch files for AutoSDV system
│   ├── autosdv.launch.yaml          # Main entry point
│   ├── sensing.launch.xml           # Sensor suite configuration
│   ├── localization.launch.xml      # Localization (NDT, EKF, Isaac SLAM)
│   ├── planning.launch.xml          # Path and motion planning
│   └── ...
├── config/             # Autoware configuration files
│   ├── localization/   # NDT, EKF parameters
│   ├── system/         # Diagnostics, MRM configuration
│   ├── planning/       # Planning parameters
│   ├── perception/     # Perception parameters
│   └── ...
├── rviz/              # RViz configuration files
└── autosdv_launch/    # Python package (if needed)
```

## Configuration Files Source

Base configuration copied from: autoware_launch (version 0.40.0)
Original location: autoware/src/launcher/autoware_launch/autoware_launch/config/
Date copied: 2025-12-28

## Configuration Customizations

All configuration files are in the `config/` directory. Below are AutoSDV-specific modifications from the Autoware defaults.

### Localization (config/localization/)

- **ekf_localizer.param.yaml**: Tuned for AR Tag + Isaac VSLAM sensor fusion
  - Custom process noise parameters for better fusion
  - Pose smoothing for AR tag corrections
  - Optimized for indoor/GPS-denied environments
  - Last modified: 2025-12-25

- **ndt_scan_matcher/ndt_scan_matcher.param.yaml**: Tuned for low-mounted LiDAR (30cm height)
  - Ground removal enabled (`no_ground_points.enable: true`)
  - Z-margin set to 0.2m for balanced ground filtering (empirically tuned)
  - Finer resolution (1.5m vs 2.0m default) for better yaw estimation
  - Increased max iterations (50 vs 30) for convergence
  - Lowered score threshold (1.8 vs 2.3) to allow more poses through
  - **Tuning history**: 0.4m (too permissive) → 0.0m (too aggressive, 2.5k pts) → 0.2m (balanced)
  - Last modified: 2025-12-28

- **ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range.param.yaml**: Tuned for low-mounted LiDAR
  - Balanced Z-range filtering (min: -0.25m, max: 10.0m)
  - Ground points at ~-0.3m, keeping points above -0.25m ensures sufficient points
  - **Tuning history**: -0.5m (too permissive) → -0.1m (too aggressive) → -0.25m (balanced)
  - Target: >5000 points/scan for reliable NDT convergence
  - Last modified: 2025-12-28

### System Diagnostics & MRM (config/system/)

- **diagnostics/localization.yaml**: Disabled localization accuracy check
  - **CRITICAL CHANGE**: Commented out `/autoware/localization/accuracy` check to prevent false MRM triggers
  - **Reason**: Default error ellipse thresholds (1.5m position, 0.3m lateral) too strict for VLP-32C
  - **Impact**: MRM emergency stops no longer triggered by localization uncertainty alone
  - Localization quality still monitored via `scan_matching_status` (NDT score threshold)
  - **Alternative (not implemented)**: Override `localization_error_monitor.param.yaml` with higher thresholds:
    - `error_ellipse_size: 3.0` (from 1.5m)
    - `error_ellipse_size_lateral_direction: 0.6` (from 0.3m)
  - **Recommendation**: Monitor `/localization/localization_error_monitor/debug/ellipse_marker` during testing
  - Last modified: 2025-12-28

**MRM Background:**
- MRM triggers emergency stop when autonomous mode becomes unavailable
- Autonomous mode requires ALL of: localization, perception, planning, control, vehicle, map
- Default behavior: Emergency stop with -2.5 m/s² braking (aggressive)
- During outdoor testing, brief localization uncertainty can trigger false emergency stops
- This change allows controlled operation with acceptable localization uncertainty

**Monitoring MRM Status:**
```bash
# Check autonomous mode availability
ros2 topic echo /system/operation_mode/availability

# Check MRM state
ros2 topic echo /system/fail_safe/mrm_state

# View all diagnostics
ros2 run rqt_robot_monitor rqt_robot_monitor
```

## Configuration Categories (config/ directory)

- **control/**: Vehicle control parameters (trajectory following, cmd gate, etc.)
- **localization/**: Localization parameters (NDT, EKF, pose initializer, etc.)
- **map/**: Map loading and projection parameters
- **perception/**: Object detection and tracking parameters
- **planning/**: Path and motion planning parameters
- **system/**: System monitoring and diagnostics
- **vehicle/**: Raw vehicle command converter parameters
- **simulator/**: Simulation-specific parameters

## Updating Configurations

When updating Autoware base version:
1. Review changes in upstream autoware_launch/config/
2. Merge relevant updates into this package's config/ directory
3. Preserve AutoSDV-specific customizations documented above
4. Test thoroughly with AutoSDV hardware
5. Document changes in this README

## Related Documentation

- **MRM Configuration Guide**: `docs/guides/mrm_configuration.md` - Detailed MRM system documentation
- **NDT Parameter Tuning**: `docs/research/localization/ndt_parameter_tuning_coss_map.md` - Empirical tuning results
- **Control Testing**: `docs/guides/control_testing.md` - Control system testing procedures
