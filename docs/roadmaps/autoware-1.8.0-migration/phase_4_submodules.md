# Phase 4: Submodule Migration (13 repos)

**Objective**: Migrate every `src/` submodule to jazzy / Autoware 1.8.0 using the Phase 1 audit results, then repin `.gitmodules` SHAs in AutoSDV.

**Status**: ⬜ Not started

**Depends on**: Phase 1 (audits), Phase 2 (1.8.0 debs available for build-testing)

## Strategy

- **Forks with hardware patches**: re-fork — create branch `autosdv-1.8.0` from the **upstream release tag** chosen in Phase 1 (never `master`/`main` HEAD), then cherry-pick only KEEP-classified commits.
- **NEWSLabNTU originals**: port in place on a `autosdv-1.8.0` branch.
- **Non-org repos**: prefer updated upstream; fork into the org only if patches are needed.
- After each repo migrates: build it against jazzy + localrepo 1.8.0, then update the pinned SHA in AutoSDV (`git submodule` pointer) — one commit per repo.

## Per-Submodule Plan

| #  | Submodule (path under `src/`)                   | Host                                       | Action                                                                                                       |
|----|-------------------------------------------------|--------------------------------------------|--------------------------------------------------------------------------------------------------------------|
| 1  | `sensor_component/external/zed-ros2-wrapper`    | NEWSLabNTU (stereolabs fork)               | Re-fork from tag **v5.4.0** (pairs with ZED SDK 5.4); cherry-pick KEEP commits                               |
| 2  | `calibration/CalibrationTools`                  | NEWSLabNTU (tier4 fork, `autosdv-2025.02`) | Re-fork onto jazzy-capable tier4 base (from branch tier4/universe); new branch `autosdv-1.8.0`; cherry-pick  |
| 3  | `sensor_component/external/seyond_ros_driver`   | NEWSLabNTU (Seyond-Inc fork)               | Sync to upstream release tag v1.0.3 (upstream supports jazzy); re-apply KEEP commits                         |
| 4  | `sensor_component/external/ros2_mpu9250_driver` | NEWSLabNTU (hiwad-aziz fork)               | Small driver: port directly on `autosdv-1.8.0`; verify jazzy build                                           |
| 5  | `sensor_component/external/ros-nmea-reader`     | NEWSLabNTU fork (upstream jerry73204)      | Create `autosdv-1.8.0` branch on the org fork; port to jazzy; repoint `.gitmodules` to the fork              |
| 6  | `sensor_component/external/autoware_zed`        | NEWSLabNTU original                        | Port in place: jazzy + Autoware 1.8.0 message APIs (currently humble-tested, pins autoware 0.45.1)           |
| 7  | `sensor_component/external/gnss_locator`        | NEWSLabNTU original                        | Port in place; verify jazzy build                                                                            |
| 8  | `sensor_kit/autosdv_sensor_kit_launch`          | NEWSLabNTU original                        | Port in place (launch files; check 1.8.0 launch arg compat)                                                  |
| 9  | `vehicle/autosdv_vehicle_launch`                | NEWSLabNTU original                        | Port in place (launch files)                                                                                 |
| 10 | `system/autosdv_runtime`                        | NEWSLabNTU original                        | Port in place; needs full build check                                                                        |
| 11 | `system/autosdv_system_monitor`                 | NEWSLabNTU original                        | Port in place; needs full build check                                                                        |
| 12 | `param/autoware_individual_params`              | NEWSLabNTU original                        | Port in place (param files; mostly trivial)                                                                  |
| 13 | `vehicle/external/autoware_manual_control`      | evshary (non-org)                          | Use updated upstream `main` (active as of 2026-06); verify jazzy build; fork into org only if patches needed |

## Work Items

- [ ] **4.1** Migrate repos 1–5 (forks; re-fork + cherry-pick per audit).
- [ ] **4.2** Migrate repos 6–12 (org originals; port in place).
- [ ] **4.3** Repo 13: verify `evshary/autoware_manual_control` main builds on jazzy + 1.8.0; pin the verified SHA.
- [ ] **4.4** Common jazzy porting checklist applied to each C++ repo:
  - `ament_target_dependencies` / rclcpp API drift (rclcpp 28+)
  - message API changes vs Autoware 1.8.0 (`autoware_*_msgs`, `tier4_*_msgs`)
  - gcc-13/noble warnings-as-errors fixes
  - `package.xml` rosdep keys valid on jazzy (`rosdep resolve` clean)
- [ ] **4.5** Update `.gitmodules` (ros-nmea-reader URL → NEWSLabNTU fork) and repin all 13 submodule SHAs in AutoSDV; `make checkout` clean on a fresh clone.

## Goal (exit criteria)

- [ ] All 13 submodules have an `autosdv-1.8.0` branch (or verified upstream pin for #13) that builds green against jazzy + localrepo 1.8.0.
- [ ] Zero un-audited fork commits left behind (Phase 1 table fully executed: every KEEP cherry-picked, every OBSOLETE dropped).
- [ ] `.gitmodules` + submodule SHAs updated in AutoSDV; fresh clone + `make checkout` + `make build` succeeds (amd64).
