# AutoSDV Launch Package

This package provides the main launch system for AutoSDV, including launch files and Autoware configuration files with AutoSDV-specific customizations.

## Package Structure

```
autosdv_launch/
├── launch/             # Launch files for AutoSDV system
│   ├── autosdv.launch.yaml          # Main entry point
│   ├── sensing.launch.xml           # Sensor suite configuration
│   ├── localization.launch.xml      # Localization (NDT, EKF, MCL)
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

- **ekf_localizer.param.yaml**: EKF tuning for pose/twist fusion
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

- **diagnostics/**: only the `-mcl` graphs here are loaded
  - `autosdv-mcl-main.yaml`, `localization-mcl.yaml` and `map-mcl.yaml` are used
    when `pose_source:=mcl`; every other pose source reads autoware_launch's
    graphs untouched.
  - The repository once carried edited copies of the stock graphs, including a
    `localization.yaml` with the accuracy check commented out. **Nothing loaded
    them**, so the accuracy check was live throughout; they were deleted on
    2026-09-22. See `docs/guides/mrm_configuration.md`.

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
