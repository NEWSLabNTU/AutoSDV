# AutoSDV Documentation

## 🔬 Research

Background research and technology surveys

- [Indoor Localization Methods](research/indoor_localization.md) - ROS 2 indoor localization solutions survey (recommends Isaac, which was since retired)
- [NVIDIA Isaac ROS](research/nvidia_isaac_ros.md) - 🗄️ Retired (Isaac visual localization was removed from the project; kept as the record of what was evaluated)
- [LiDAR Marker Localization](research/lidar_marker_localization.md) - LiDAR-based landmark localization
- [Robin-W LiDAR FOV](research/robin_w_fov.md) - Robin-W solid-state LiDAR field of view analysis

## 🏗️ Design

Architecture and integration design documents


## 📋 Roadmaps

Implementation status and timelines

- [Book: Simulation Tutorials](roadmap/5-book_simulation_tutorials.md) - ✅ Complete (pages shipped; superseded by roadmap 10's tutorial)
- [Golf Cart Backport](roadmap/6-golfcart-backport.md) - ✅ Complete (all 10 phases)
- [Backport Verification](roadmap/7-backport-verification.md) - 📋 Proposed (what to run, in what order, to see it working)
- [Book Revision (v0.2 release)](roadmap/8-book-revision.md) - ✅ Phases 0-4 done, EN + zh-TW; **tag held** pending roadmap 10
- [Workshop: Laptop Onboarding](roadmap/9-workshop-laptop-onboarding.md) - 📝 Planned (2-hour class; blocked on roadmap 10)
- [Book: Tutorial Restructure](roadmap/10-book-tutorial-restructure.md) - 🚧 **Active** (concepts + tutorial written; rebased onto upstream; cuts book tag `0.2-1`)
- [Isaac VSLAM](roadmap/1-isaac_vslam.md) - 🗄️ Retired (shipped, then removed; see the note in the file)
- [Visual Global Localization](roadmap/2-visual_global_localization.md) - 🗄️ Retired (removed with Isaac; kept as the record)
- [AR Tag Integration](roadmap/3-ar_tag_integration/) - 🗄️ Retired (its phases 4-6 depend on Isaac VSLAM twist, and `pose_source:=artag` was never implemented; kept as the record)
  - [Overview & Architecture](roadmap/3-ar_tag_integration/README.md)
  - [Phase 1: Setup & Preparation](roadmap/3-ar_tag_integration/3.1-setup.md)
  - [Phase 2: AR Tag Map Creation](roadmap/3-ar_tag_integration/3.2-map_creation.md)
  - [Phase 3: AR Tag Localizer Integration](roadmap/3-ar_tag_integration/3.3-localizer.md)
  - [Phase 4: Isaac VSLAM Modification](roadmap/3-ar_tag_integration/3.4-vslam.md)
  - [Phase 5: EKF Fusion Configuration](roadmap/3-ar_tag_integration/3.5-fusion.md)
  - [Phase 6: Integration Testing](roadmap/3-ar_tag_integration/3.6-testing.md)
  - [Phase 7: Documentation & Deployment](roadmap/3-ar_tag_integration/3.7-deployment.md)

## 📖 Guides

Step-by-step tutorials and testing procedures

- [Control System Testing](guides/control_testing.md) - PID tuning and PlotJuggler integration
- [Simulation Testing](guides/simulation_testing.md) - Rosbag replay and CARLA simulation

---

**Quick Reference:** See [../CLAUDE.md](../CLAUDE.md) for project overview and common commands.
