# AutoSDV Documentation

## 🔬 Research

Background research and technology surveys

- [Indoor Localization Methods](research/indoor_localization.md) - ROS 2 indoor localization solutions survey
- [NVIDIA Isaac ROS](research/nvidia_isaac_ros.md) - Isaac ROS Visual SLAM analysis
- [LiDAR Marker Localization](research/lidar_marker_localization.md) - LiDAR-based landmark localization
- [Robin-W LiDAR FOV](research/robin_w_fov.md) - Robin-W solid-state LiDAR field of view analysis

## 🏗️ Design

Architecture and integration design documents

- [Isaac VSLAM Integration](design/isaac_vslam_integration.md) - ✅ Complete (Isaac ROS Visual SLAM integration design)

## 📋 Roadmaps

Implementation status and timelines

- [Golf Cart Backport](roadmap/6-golfcart-backport.md) - 🚧 In Progress (7 of 10 phases done; CUDA pipeline and the setup rewrite remain)
- [Isaac VSLAM](roadmaps/isaac_vslam.md) - ✅ Complete
- [AR Tag Integration](roadmaps/ar_tag_integration/) - 🚧 In Progress
  - [Overview & Architecture](roadmaps/ar_tag_integration/README.md)
  - [Phase 1: Setup & Preparation](roadmaps/ar_tag_integration/phase_1_setup.md) - 🚧 In Progress
  - [Phase 2: AR Tag Map Creation](roadmaps/ar_tag_integration/phase_2_map_creation.md) - ⏸️ Pending
  - [Phase 3: AR Tag Localizer Integration](roadmaps/ar_tag_integration/phase_3_localizer.md) - ⏸️ Pending
  - [Phase 4: Isaac VSLAM Modification](roadmaps/ar_tag_integration/phase_4_vslam.md) - ⏸️ Pending
  - [Phase 5: EKF Fusion Configuration](roadmaps/ar_tag_integration/phase_5_fusion.md) - ⏸️ Pending
  - [Phase 6: Integration Testing](roadmaps/ar_tag_integration/phase_6_testing.md) - ⏸️ Pending
  - [Phase 7: Documentation & Deployment](roadmaps/ar_tag_integration/phase_7_deployment.md) - ⏸️ Pending

## 📖 Guides

Step-by-step tutorials and testing procedures

- [Control System Testing](guides/control_testing.md) - PID tuning and PlotJuggler integration
- [Simulation Testing](guides/simulation_testing.md) - Rosbag replay and CARLA simulation
- [Isaac VSLAM Standalone Testing](guides/isaac_vslam_testing.md) - Standalone Isaac ROS Visual SLAM test setup

---

**Quick Reference:** See [../CLAUDE.md](../CLAUDE.md) for project overview and common commands.
