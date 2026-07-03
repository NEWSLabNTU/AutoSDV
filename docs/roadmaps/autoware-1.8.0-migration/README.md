# Autoware 1.8.0 / Ubuntu 24.04 Migration Roadmap

**Goal**: Migrate AutoSDV from Ubuntu 22.04 / ROS 2 Humble / Autoware 2025.02 to Ubuntu 24.04 / ROS 2 Jazzy / Autoware 1.8.0.

**Overall Status**: 🚧 **Stage 1 complete; Stage 2 ready to start** (created 2026-07-02; restructured by machine 2026-07-03)

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

## Stages — one machine at a time

The roadmap is grouped **by machine**, not by workstream: each stage runs to completion on one environment before the next machine is touched. The Jetson Orin is not flashed yet, so all device work is deferred to the final stage — **nothing before Stage 4 requires the device**.

| Stage | File                                               | Machine                             | Goal                                                                                                                       |
|-------|----------------------------------------------------|-------------------------------------|----------------------------------------------------------------------------------------------------------------------------|
| 1     | [stage_1_audit.md](stage_1_audit.md)               | any (done on ARM64 server)          | ✅ Fork-patch audits + desk-research gates — **complete**                                                                  |
| 2     | [stage_2_amd64.md](stage_2_amd64.md)               | WSL2 / AMD64 (native box at end)    | amd64 1.8.0 deb released; repo config, all submodules + in-tree packages migrated; full amd64 build/test/replay validation |
| 3     | [stage_3_arm64_server.md](stage_3_arm64_server.md) | ARM64 build server                  | CUDA-13 port; `jetpack7.2` deb built and attached to the release; arm64 workspace built; artifacts staged                  |
| 4     | [stage_4_orin.md](stage_4_orin.md)                 | Jetson Orin (📟 flash happens here) | Flash JP7.2; install, TRT engines, sensor bringup, vehicle tests — migration complete                                      |

Sequencing notes:

- **Release is incremental**: `rosdebian/1.8.0-1` is published in Stage 2A with the amd64 deb only; the `jetpack7.2` deb is attached to the same release in Stage 3C. This keeps all amd64 work unblocked by ARM64/CUDA-13 effort.
- Jetson-specific config edits (JP7.2 detection, checksums, Docker bases) are **authored in Stage 2B** in one sweep (marked ⏳) and **verified** in Stages 3/4.
- The two hardware/infra gates that used to sit in Phase 1's exit criteria moved to where they can actually run: CUDA-13 spike build-half → Stage 3A, JetPack flash + spike verify-half → Stage 4A.

### Old phase → new stage mapping

The roadmap was originally 7 workstream phases (each spanning several machines). Historical references (worklog, commit messages) decode as:

| Old phase                   | Now                                                                 |
|-----------------------------|---------------------------------------------------------------------|
| Phase 1 (audit)             | Stage 1 (re-scoped: device/infra gates 1.2.1 → 4A, 1.2.4 → 3A + 4A) |
| Phase 2 (autoware base)     | Stage 2A (amd64 + release) / Stage 3B–3C (jetpack7.2)               |
| Phase 3 (repo config)       | Stage 2B                                                            |
| Phase 4 (submodules)        | Stage 2C                                                            |
| Phase 5 (in-tree packages)  | Stage 2D                                                            |
| Phase 6 (build integration) | Stage 2E (amd64) / Stage 3D (arm64) / Stage 4B–4C (device)          |
| Phase 7 (validation)        | Stage 2E (replay) / Stage 4D–4F (device + hygiene)                  |

## Risk Register

1. **Autoware universe CUDA/TRT packages have no official CUDA 13 support.** Autoware 1.8.0 aarch64 pin is JetPack-6-era (TRT `10.3.0.26-1+cuda12.5`). Porting universe TRT/CUDA code to CUDA 13.2 / TRT 10.16 is unquantified effort (see dusty-nv/jetson-containers#1661 for ecosystem-wide CUDA 13 breakage). Mitigation: keep patches as NEWSLabNTU/autoware fork commits; upstream later. Confined to Stage 3.
2. **ZED SDK 5.4** has the needed CUDA 13 / TRT 10 variant and an Ubuntu 24.04 build — installer URL confirmed (Stage 1 §1.2.2), but it is a `.run` while `setup/scripts/install-zed-sdk.sh` installs a `.deb`; Stage 2B.10 must switch or rebuild.
3. **Isaac ROS on jazzy is supported, but Orin-on-JP7.2 is unconfirmed** (JP7 docs emphasize Thor). Isaac VSLAM stays gated/optional (its testing was already deferred, see [../isaac_vslam.md](../isaac_vslam.md)); on-device confirmation in Stage 4D.7.
4. **Fork patch audits revealed hardware patches with no upstream equivalent** — these carry forward as cherry-picks (Stage 2C) and add maintenance cost.
5. **The `jetpack7.2` localrepo deb variant is new build infrastructure** — the existing pipeline only knows amd64/arm64/jetpack6.0. Confined to Stage 3.
6. **Machines have non-overlapping capabilities** — x86 WSL2 cannot validate Jetson or sensor work; the ARM64 server has no CUDA-13-capable driver; only the Orin runs SM_87. This is why the roadmap is grouped by machine: each stage's exit criteria are checkable entirely on that stage's machine, and ⏳/📟 markers flag the few items verified later.

## Conventions

- Every re-fork/sync starts from a **specific upstream release tag** (e.g. zed-ros2-wrapper `v5.4.0`), never `master`/`main` HEAD. Tags were chosen and recorded in Stage 1. (Known exception: CalibrationTools has no jazzy tag — pin commit `d434e57` on `tier4/universe`.)
- New fork branches are named **`autosdv-1.8.0`**.
- The GitHub release `rosdebian/1.8.0-1` is **published incrementally**: amd64 deb in Stage 2A, jetpack7.2 deb attached in Stage 3C.
- Each stage file ends with a **Goal (exit criteria)** section; a stage is complete only when every criterion is checked. **Exit criteria never include work that needs a later stage's machine.**
