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
