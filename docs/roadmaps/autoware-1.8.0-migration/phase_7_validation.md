# Phase 7: Validation — Tests, Replay, Hardware Bringup

**Objective**: Prove functional parity with the 0.1.0-dev / Humble baseline: automated tests, rosbag replay, per-sensor bringup, and control tests all pass on 0.2.0-dev / Jazzy.

**Status**: ⬜ Not started

**Depends on**: Phase 6

## Work Items

### 7.1 Automated & simulation (AMD64)

- [ ] **7.1.1** `make test` green (all packages).
- [ ] **7.1.2** Rosbag replay: `logging_simulation.launch.yaml` + `scripts/play_rosbag.sh` with an existing Humble-era bag — verify message compat (rosbag2 format across humble→jazzy; convert bags if needed) and that localization/perception pipelines produce output.
- [ ] **7.1.3** Compare replay behavior against the 0.1.0-dev baseline (localization stability, perception detections) — no regressions.

### 7.2 Sensor bringup (📟 device-only, on JP7.2 Orin)

Per-sensor checklist — each sensor publishes correct data at expected rate:

- [ ] **7.2.1** Seyond Robin-W LiDAR (`seyond_ros_driver`)
- [ ] **7.2.2** Velodyne 32C LiDAR
- [ ] **7.2.3** Blickfeld Cube1 LiDAR (`ros2_blickfeld_driver`)
- [ ] **7.2.4** ZED camera (`zed-ros2-wrapper` v5.4 + `autoware_zed` conversion; verify SDK 5.4 runtime on CUDA 13)
- [ ] **7.2.5** MPU9250 IMU (`ros2_mpu9250_driver`)
- [ ] **7.2.6** GNSS + NTRIP (`gnss_locator`, `ros-nmea-reader`; `scripts/testing/ntrip/check_ntrip_setup.sh`)

### 7.3 Vehicle & control (📟 device-only)

- [ ] **7.3.1** `make test-control` (PID controller + speedometer, tmux session).
- [ ] **7.3.2** `make controller` keyboard manual control via `autoware_manual_control`.
- [ ] **7.3.3** Full-stack outdoor test: localization + perception + planning + control on the vehicle, per sensor preset (`lidar_only`, `camera_lidar_fusion`, `minimal`).

### 7.4 Release hygiene

- [ ] **7.4.1** Update docs: README, setup instructions, any humble/22.04 mentions.
- [ ] **7.4.2** Update this roadmap's status markers; record known issues.
- [ ] **7.4.3** Confirm `versions.yaml` = `0.2.0-dev` on the `develop` branch; migration branch merged per branch strategy.

## Goal (exit criteria)

- [ ] All automated tests and rosbag replay pass on AMD64 with no regressions vs the Humble baseline.
- [ ] 📟 Every sensor in 7.2 verified live on the JP7.2 Orin.
- [ ] 📟 Control tests pass; at least one full-stack outdoor run completed per active preset.
- [ ] Docs updated; AutoSDV `0.2.0-dev` is the working development version — **migration complete**.
