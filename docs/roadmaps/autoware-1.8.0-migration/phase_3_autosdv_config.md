# Phase 3: AutoSDV Repo Configuration Migration

**Objective**: Update every Ubuntu 22.04 / Humble / Autoware 2025.02 coupling point in the AutoSDV repository to Ubuntu 24.04 / Jazzy / Autoware 1.8.0, and bump AutoSDV to `0.2.0-dev`.

**Status**: ⬜ Not started

**Depends on**: Phase 2 (release tag + checksums)

## Work Items

### 3.1 `versions.yaml` (single source of truth — update first)

- [ ] **3.1.1** `autosdv.version`: `0.1.0-dev` → **`0.2.0-dev`** (Autoware base upgrade = MAJOR-track bump per CLAUDE.md guidelines).
- [ ] **3.1.2** `autoware.version`: `2025.02` → `1.8.0`; `rosdebian_release`: `rosdebian/2025.02-1` → `rosdebian/1.8.0-1`; `package_version`: `2025.2-1` → `1.8.0-1` (match Phase 2 deb filename).
- [ ] **3.1.3** `ros.distro`: `humble` → `jazzy`.
- [ ] **3.1.4** `nvidia_amd64`: `cuda: 12.8`, `tensorrt: 10.8.0.43-1+cuda12.8` (+ matching cuDNN 9.x).
- [ ] **3.1.5** `nvidia_arm64`: `jetpack: 7.2`, `l4t: 39.2`, `cuda: 13.2`, `tensorrt: 10.16.2` (+ matching cuDNN from JP7.2).
- [ ] **3.1.6** `checksums`: replace `autoware_deb_jetpack60` with **`autoware_deb_jetpack72`**; fill amd64 + jetpack72 SHA256s from Phase 2.
- [ ] **3.1.7** Verify `scripts/version/get-version.sh` and `scripts/version/export-versions.sh` handle the renamed checksum key.

### 3.2 Setup scripts

- [ ] **3.2.1** `setup/scripts/install-autoware-debian.sh`: update JetPack detection — `R36 (release), REVISION: 3\.` regex (line 29) → JP7.2 pattern for `/etc/nv_tegra_release` (r39.2; confirm exact string on device); `jetpack6.0` deb filename/variable (lines 40-43) → `jetpack7.2`; `CHECKSUM_AUTOWARE_DEB_JETPACK60` → `CHECKSUM_AUTOWARE_DEB_JETPACK72`.
- [ ] **3.2.2** `setup/scripts/install-ros2.sh:7`: `ROSDISTRO="${ROSDISTRO:-humble}"` → `jazzy`.
- [ ] **3.2.3** `setup/scripts/install-zed-sdk.sh`: pin ZED SDK **5.4** (CUDA 13 / TRT 10 variant for Jetson; Ubuntu 24.04 build for amd64), URL from Phase 1.2.2.
- [ ] **3.2.4** `setup/scripts/install-blickfeld.sh`: bump/rebuild blickfeld-scanner-lib for noble per Phase 1.1.6 audit outcome.
- [ ] **3.2.5** `setup/justfile:147`: `ros-humble-plotjuggler-ros` → `ros-jazzy-plotjuggler-ros` (also the apt-mark hold name).
- [ ] **3.2.6** `setup/justfile:129-130`: `autonomoustuff-public-humble.yaml` → jazzy equivalent (per Phase 1.2.5).
- [ ] **3.2.7** `setup/justfile:201`: `source /opt/ros/humble/setup.sh` → jazzy; `rosdep update --rosdistro=jazzy`.

### 3.3 Hardcoded `/opt/ros/humble` paths (mechanical sweep)

Replace `source /opt/ros/humble/setup.bash` → `/opt/ros/jazzy/` in:

- [ ] **3.3.1** `.envrc:46`
- [ ] **3.3.2** `Makefile:74,82`
- [ ] **3.3.3** `scripts/play_rosbag.sh:5`
- [ ] **3.3.4** `scripts/record_localization.sh:5`
- [ ] **3.3.5** `scripts/build/package-deb.sh:9`
- [ ] **3.3.6** `scripts/build/make-deb.sh` (line 22 — hardcoded separately from its `$ROS_DISTRO` usage)
- [ ] **3.3.7** `scripts/testing/ntrip/check_ntrip_setup.sh:5`
- [ ] **3.3.8** Final sweep: `rg -n "ros/humble|ros-humble-|humble" --glob '!data/**' --glob '!docs/**'` returns no functional hits.

### 3.4 Docker & CI

- [ ] **3.4.1** `docker/Dockerfile:1`: base `nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel` → JP7.2-era base (`nvcr.io/nvidia/l4t-jetpack:r39.x` or the JP7.2 TensorRT devel image).
- [ ] **3.4.2** `docker/Dockerfile:4`: `ARG COMMIT_HASH=2025.02` → the new release branch/tag.
- [ ] **3.4.3** `docker/nvidia-l4t-apt-source.list`: `r36.3` → `r39.2`.
- [ ] **3.4.4** `.github/workflows/docker-build.yml:5`: branch trigger `[2025.02]` → new branch name.
- [ ] **3.4.5** Handle noble base-image `ubuntu` user (UID 1000) conflict if the container creates its own user.

## Goal (exit criteria)

- [ ] `./scripts/version/get-version.sh ros.distro` returns `jazzy`; `autosdv.version` returns `0.2.0-dev`.
- [ ] `rg "humble"` over the repo (excluding `data/`, historical docs, and this roadmap) returns zero functional references.
- [ ] `./setup.sh` completes on a clean Ubuntu 24.04 amd64 machine (installs jazzy + the 1.8.0 localrepo deb with checksum verification).
- [ ] 📟 *Device-only*: `./setup.sh` completes on the JP7.2 Orin, selecting the `jetpack7.2` deb via the r39 detection path.
