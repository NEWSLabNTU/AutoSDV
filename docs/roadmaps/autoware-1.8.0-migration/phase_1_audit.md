# Phase 1: Fork-Patch Audits & Environment Gates

**Objective**: Know exactly what hardware-fix commits each NEWSLabNTU fork carries, which are still needed, and confirm every external prerequisite (JetPack 7.2, ZED SDK 5.4, Isaac ROS, CUDA 13 cross-build) before any migration work starts.

**Status**: ⬜ Not started

## 1.1 Fork commit audits

Procedure per fork:

```bash
git clone <fork-url> && cd <repo>
git remote add upstream <upstream-url> && git fetch upstream --tags
# Find the upstream base the fork diverged from
git merge-base HEAD upstream/<default-branch>
# List org-only commits
git log --oneline <merge-base>..HEAD
```

Classify every commit into one of:

- **KEEP** — hardware fix still needed on jazzy; will be cherry-picked in Phase 4
- **OBSOLETE** — fixed upstream or no longer relevant (e.g. humble-only workaround)
- **UPSTREAMED** — already merged upstream; drop

Record the result as a table in this file (commit SHA, subject, classification, target upstream tag).

### Work Items

- [ ] **1.1.1** Audit `NEWSLabNTU/zed-ros2-wrapper` (fork of stereolabs/zed-ros2-wrapper, humble-era). Target upstream tag: **v5.4.0** (pairs with ZED SDK 5.4).
- [ ] **1.1.2** Audit `NEWSLabNTU/CalibrationTools` (fork of tier4/CalibrationTools, branch `autosdv-2025.02`, ~300 commits). Determine tier4 upstream jazzy state and pick target tag/branch.
- [ ] **1.1.3** Audit `NEWSLabNTU/seyond_ros_driver` (fork of Seyond-Inc/seyond_ros_driver; upstream already supports jazzy). Pick upstream release tag.
- [ ] **1.1.4** Audit `NEWSLabNTU/ros2_mpu9250_driver` (fork of hiwad-aziz/ros2_mpu9250_driver; small driver, no distro claim upstream).
- [ ] **1.1.5** Audit `NEWSLabNTU/ros-nmea-reader` fork (upstream jerry73204/ros-nmea-reader).
- [ ] **1.1.6** Audit blickfeld-scanner-lib **`v2.20.6-newslab1`** patch (installed by `setup/scripts/install-blickfeld.sh:29`): diff against upstream v2.20.6, decide whether the patch is still needed on noble/gcc-13.
- [ ] **1.1.7** For each fork, record the chosen upstream tag in the table below (never `master`/`main` HEAD).

### Audit results (fill in)

| Repo                  | Upstream tag chosen | KEEP commits | OBSOLETE | UPSTREAMED |
|-----------------------|---------------------|--------------|----------|------------|
| zed-ros2-wrapper      | v5.4.0              |              |          |            |
| CalibrationTools      |                     |              |          |            |
| seyond_ros_driver     |                     |              |          |            |
| ros2_mpu9250_driver   |                     |              |          |            |
| ros-nmea-reader       |                     |              |          |            |
| blickfeld-scanner-lib |                     |              |          |            |

## 1.2 Environment gates

- [ ] **1.2.1** 📟 *Device-only*: Flash JetPack 7.2 (Jetson Linux r39.2) on the actual Orin hardware; confirm boot, `nvidia-smi`/`tegrastats`, CUDA 13.2.1 and TensorRT 10.16.2 present.
- [ ] **1.2.2** Confirm ZED SDK **5.4** Jetson installer exists for JP7.2 / L4T r39.2 (CUDA 13 / TRT 10 variant); record exact download URL for `setup/scripts/install-zed-sdk.sh`.
- [ ] **1.2.3** Check Isaac ROS availability for jazzy / JetPack 7 (current integration uses `ros-humble-isaac-ros-*` from a jammy apt repo). If unavailable, keep `autosdv_isaac_slam_launch` gated (Phase 5).
- [ ] **1.2.4** CUDA 13 cross-build spike: on the ARM64 server, inside an arm64 noble container with CUDA 13.2 toolkit + TRT 10.16, build one CUDA package (e.g. a small autoware universe TRT node), copy to the Orin, and run it. Confirms server-built CUDA binaries are link-compatible with JetPack 7.2.
- [ ] **1.2.5** Confirm `autonomoustuff-public-jazzy.yaml` rosdep source exists (used by `setup/justfile:129-130`); find replacement if not.

## Goal (exit criteria)

- [ ] Every fork in the audit table has: chosen upstream tag + full commit classification (no unclassified commits).
- [ ] JetPack 7.2 confirmed running on target Orin hardware.
- [ ] ZED SDK 5.4 JP7.2 installer URL recorded.
- [ ] Isaac ROS jazzy decision recorded (available / gated).
- [ ] CUDA 13 cross-build spike passed (server-built binary runs on Orin) — or documented as failed with fallback (build CUDA packages on-device).
