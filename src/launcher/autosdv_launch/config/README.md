# AutoSDV Configuration Files

This directory contains Autoware configuration files with AutoSDV-specific customizations.

## Source
Base configuration copied from: autoware_launch (version 0.40.0)
Original location: autoware/src/launcher/autoware_launch/autoware_launch/config/
Date copied: 2025-12-28

## Customizations

### Localization
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

## Updating Configurations

When updating Autoware base version:
1. Review changes in upstream autoware_launch/config/
2. Merge relevant updates into this directory
3. Preserve AutoSDV-specific customizations
4. Test thoroughly with AutoSDV hardware
5. Document changes in this README

## Configuration Categories

- **control/**: Vehicle control parameters (trajectory following, cmd gate, etc.)
- **localization/**: Localization parameters (NDT, EKF, pose initializer, etc.)
- **map/**: Map loading and projection parameters
- **perception/**: Object detection and tracking parameters
- **planning/**: Path and motion planning parameters
- **system/**: System monitoring and diagnostics
- **vehicle/**: Raw vehicle command converter parameters
- **simulator/**: Simulation-specific parameters
