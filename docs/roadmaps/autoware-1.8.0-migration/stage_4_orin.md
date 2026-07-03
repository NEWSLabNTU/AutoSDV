# Stage 4: Jetson Orin Track — Flash, Bringup, Validation

**Objective**: All device work, deferred until everything buildable elsewhere is done: flash JetPack 7.2, install the staged artifacts, generate TensorRT engines, bring up every sensor, pass vehicle/control tests, and close the migration.

**Machine**: Jetson AGX Orin / Orin NX (+ the vehicle, for 4E). **The flash happens at the start of this stage — nothing earlier in the roadmap needs the device.**

**Status**: ⬜ Not started

**Depends on**: Stage 2 (migrated repo + amd64-validated stack), Stage 3 (jetpack7.2 deb + staged arm64 artifacts).

## 4A Flash + platform gate

- [ ] **4A.1** (old 1.2.1) Flash JetPack 7.2 (Jetson Linux r39.2) on the Orin; confirm boot, `nvidia-smi`/`tegrastats`. Acceptance: `nvidia-smi` shows driver ≥ the JP7.2 baseline, `nvcc --version` = 13.2.x, `dpkg -l | rg tensorrt` = 10.16.2.
- [ ] **4A.2** Record the exact `/etc/nv_tegra_release` string and fix the Stage 2B.8 detection regex if it guessed wrong.
- [ ] **4A.3** (old 1.2.4 verify-half) CUDA-13 spike verify-half: run the Stage 3A SM_87 binary on-device — confirms server-built CUDA 13 binaries execute on JP7.2.
- [ ] **4A.4** Confirm the ⏳ `nvidia_arm64` values in `versions.yaml` (Stage 2B.5) against the flashed device; correct if JP7.2 shipped different point versions.

## 4B Device setup

- [ ] **4B.1** (old 6.3.1 + old Phase 3 device criterion) `./setup.sh` on the JP7.2 Orin: r39 detection path selects and installs the `jetpack7.2` localrepo deb (checksum-verified); ZED SDK 5.4 Jetson variant (`ZED_SDK_Tegra_L4T39.2_v5.4.0.zstd.run`); blickfeld lib.
- [ ] **4B.2** (old 2.3.4) Source the installed autoware and launch a representative CUDA node (e.g. lidar centerpoint) to verify CUDA/TRT runtime linkage — no missing-symbol/driver errors. (Engine generation itself is 4C.2.)

## 4C Deploy + first launch

- [ ] **4C.1** (old 6.3.2) Deploy the AutoSDV workspace: server-built artifacts from Stage 3D or on-device `make build` — record build time for both; the 80-core server should win.
- [ ] **4C.2** (old 6.3.3) First `make launch`: TensorRT engine generation for perception models (SM_87) completes (first run takes 10–30 min); cache engines.
- [ ] **4C.3** (old 6.3.4) Verify CycloneDDS config (`cyclonedds.xml`) + sysctl tuning still applies on kernel 6.8.
- [ ] **4C.4** (old Phase 6 device criterion) `make launch` reaches a running Autoware stack (nodes up, no crash loops), TRT engines built and cached.

## 4D Sensor bringup

Per-sensor checklist — each sensor publishes correct data at expected rate:

- [ ] **4D.1** (old 7.2.1) Seyond Robin-W LiDAR (`seyond_ros_driver`)
- [ ] **4D.2** (old 7.2.2) Velodyne 32C LiDAR
- [ ] **4D.3** (old 7.2.3) Blickfeld Cube1 LiDAR (`ros2_blickfeld_driver`; closes Stage 2D.2's deferred point-cloud check)
- [ ] **4D.4** (old 7.2.4) ZED camera (`zed-ros2-wrapper` v5.4 + `autoware_zed` conversion; verify SDK 5.4 runtime on CUDA 13)
- [ ] **4D.5** (old 7.2.5) MPU9250 IMU (`ros2_mpu9250_driver`)
- [ ] **4D.6** (old 7.2.6) GNSS + NTRIP (`gnss_locator`, `ros-nmea-reader`; `scripts/testing/ntrip/check_ntrip_setup.sh`)
- [ ] **4D.7** (from Stage 1 §1.2.3 caveat) Isaac ROS on JP7.2/Orin: if ported in Stage 2D.3, verify the jazzy Isaac packages actually run on Orin (JP7 docs emphasize Thor; Orin on JP7.2 unconfirmed).

## 4E Vehicle & control

- [ ] **4E.1** (old 7.3.1) `make test-control` (PID controller + speedometer, tmux session).
- [ ] **4E.2** (old 7.3.2) `make controller` keyboard manual control via `autoware_manual_control`.
- [ ] **4E.3** (old 7.3.3) Full-stack outdoor test: localization + perception + planning + control on the vehicle, per sensor preset (`lidar_only`, `camera_lidar_fusion`, `minimal`).

## 4F Release hygiene

- [ ] **4F.1** (old 7.4.1) Update docs: README, setup instructions, any humble/22.04 mentions.
- [ ] **4F.2** (old 7.4.2) Update this roadmap's status markers; record known issues.
- [ ] **4F.3** (old 7.4.3) Confirm `versions.yaml` = `0.2.0-dev` on the `develop` branch; migration branch merged per branch strategy.

## Goal (exit criteria)

- [ ] JP7.2 flashed and platform-verified (nvcc 13.2.x, TRT 10.16.2); CUDA-13 spike binary runs on-device.
- [ ] `./setup.sh` completes on the Orin via the r39/jetpack7.2 path; CUDA/TRT node starts without linkage errors.
- [ ] `make launch` reaches a running Autoware stack; TRT engines built and cached; build/deploy procedure documented.
- [ ] Every sensor in 4D verified live.
- [ ] Control tests pass; at least one full-stack outdoor run completed per active preset.
- [ ] Docs updated; AutoSDV `0.2.0-dev` is the working development version — **migration complete**.
