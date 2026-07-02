# Autoware 1.8.0 / Ubuntu 24.04 Migration Roadmap

**Goal**: Migrate AutoSDV from Ubuntu 22.04 / ROS 2 Humble / Autoware 2025.02 to Ubuntu 24.04 / ROS 2 Jazzy / Autoware 1.8.0.

**Overall Status**: 📋 **Planning** (created 2026-07-02)

**End state**: AutoSDV version **`0.2.0-dev`** (MAJOR-worthy Autoware base upgrade, tracked on the development channel), running on:

- **Primary target**: Jetson AGX Orin / Orin NX on **JetPack 7.2**
- **Secondary target**: AMD64 desktop/server on Ubuntu 24.04

## Version Matrix

| Component         | Current (0.1.0-dev)             | Target (0.2.0-dev)                                          |
|-------------------|---------------------------------|-------------------------------------------------------------|
| Ubuntu            | 22.04 (jammy)                   | 24.04 (noble)                                               |
| ROS 2             | Humble                          | Jazzy                                                       |
| Autoware          | 2025.02 (`rosdebian/2025.02-1`) | 1.8.0 (`rosdebian/1.8.0-1`, to be produced)                 |
| autoware_core pin | —                               | 1.8.0 (1.9.0 bump is a separate follow-up)                  |
| JetPack / L4T     | 6.0 / 36.3                      | **7.2 / r39.2** (kernel 6.8, Ubuntu 24.04 rootfs)           |
| CUDA (Jetson)     | 12.2                            | **13.2.1**                                                  |
| TensorRT (Jetson) | 8.6.2.2                         | **10.16.2**                                                 |
| CUDA (AMD64)      | 12.3                            | **12.8**                                                    |
| TensorRT (AMD64)  | 8.6.1.6                         | **10.8.0.43-1+cuda12.8**                                    |
| ZED SDK           | (JP6-era)                       | **5.4** (CUDA 13 / TensorRT 10 variant; Ubuntu 24.04 build) |

Sources: JetPack 7.2 = Jetson Linux r39.2 / CUDA 13.2.1 / TensorRT 10.16.2 (NVIDIA JetPack 7.2 release; JP7.2 added Orin AGX / Orin NX support). AMD64 CUDA/TRT pins follow Autoware 1.8.0 `ansible/roles/cuda/defaults/main.yaml` (`cuda_version: 12.8`) and `ansible/roles/tensorrt/defaults/main.yaml` (`10.8.0.43-1+cuda12.8`).

## Build / Target Matrix

| Environment                                         | Role                    | Notes                                                                                                                                                                                                                                                                                                |
|-----------------------------------------------------|-------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| AMD64 machine (Ubuntu 24.04)                        | Build + test amd64      | Native jazzy build; full simulation testing                                                                                                                                                                                                                                                          |
| ARM64 server (80 cores / 512 GB, Ubuntu 22.04 host) | Build arm64             | **Always build inside arm64 Ubuntu 24.04 (noble/jazzy) containers — never on the bare 22.04 host** (glibc 2.35 vs 2.39, humble-only apt). CUDA 13.2 toolkit + TRT 10.16 inside container for CUDA packages. CUDA 13 unifies the arm64 target — verify Orin link-compatibility early (Phase 1 spike). |
| Jetson Orin (JetPack 7.2)                           | Final install + runtime | TensorRT engine generation is **on-device only** (SM_87). Sensor bringup on-device only.                                                                                                                                                                                                             |

## Phases

| Phase | File                                                         | Goal                                                             |
|-------|--------------------------------------------------------------|------------------------------------------------------------------|
| 1     | [phase_1_audit.md](phase_1_audit.md)                         | Fork-patch audits + environment gates verified                   |
| 2     | [phase_2_autoware_base.md](phase_2_autoware_base.md)         | `rosdebian/1.8.0-1` localrepo debs released (amd64 + jetpack7.2) |
| 3     | [phase_3_autosdv_config.md](phase_3_autosdv_config.md)       | AutoSDV repo config fully migrated to jazzy/24.04                |
| 4     | [phase_4_submodules.md](phase_4_submodules.md)               | All 13 submodules migrated and repinned                          |
| 5     | [phase_5_intree_packages.md](phase_5_intree_packages.md)     | In-tree/vendored packages build on jazzy                         |
| 6     | [phase_6_build_integration.md](phase_6_build_integration.md) | Green builds on all three environments                           |
| 7     | [phase_7_validation.md](phase_7_validation.md)               | Tests, replay, and hardware bringup validated                    |

## Risk Register

1. **Autoware universe CUDA/TRT packages have no official CUDA 13 support.** Autoware 1.8.0 aarch64 pin is JetPack-6-era (TRT `10.3.0.26-1+cuda12.5`). Porting universe TRT/CUDA code to CUDA 13.2 / TRT 10.16 is unquantified effort (see dusty-nv/jetson-containers#1661 for ecosystem-wide CUDA 13 breakage). Mitigation: keep patches as NEWSLabNTU/autoware fork commits; upstream later.
2. **ZED SDK 5.4** has the needed CUDA 13 / TRT 10 variant and an Ubuntu 24.04 build — but the JP7.2 / L4T r39.2 Jetson installer must be confirmed specifically (Phase 1 gate). `setup/scripts/install-zed-sdk.sh` must pin 5.4.
3. **Isaac ROS availability on jazzy / JetPack 7 unknown.** Current integration installs `ros-humble-isaac-ros-*` from a jammy apt repo. Isaac VSLAM stays gated/optional (its testing was already deferred, see [../isaac_vslam.md](../isaac_vslam.md)).
4. **Fork patch audits may reveal hardware patches with no upstream equivalent** — these carry forward as cherry-picks and add maintenance cost.
5. **The `jetpack7.2` localrepo deb variant is new build infrastructure** — the existing pipeline only knows amd64/arm64/jetpack6.0.
6. **x86 WSL2 development machines cannot validate Jetson or sensor work** — device-only items are flagged per phase.

## Conventions

- Every re-fork/sync starts from a **specific upstream release tag** (e.g. zed-ros2-wrapper `v5.4.0`), never `master`/`main` HEAD. Tags are chosen and recorded in Phase 1.
- New fork branches are named **`autosdv-1.8.0`**.
- Each phase file ends with a **Goal (exit criteria)** section; a phase is complete only when every criterion is checked.
