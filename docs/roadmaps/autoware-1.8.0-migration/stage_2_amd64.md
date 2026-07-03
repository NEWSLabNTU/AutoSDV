# Stage 2: AMD64 Track — Full Migration on One Machine

**Objective**: Complete every migration work item that needs no ARM64 server and no Jetson: build and release the amd64 Autoware 1.8.0 deb, migrate all AutoSDV repo config, migrate all 13 submodules and 4 in-tree packages, and validate the full stack on AMD64 (build, tests, simulation, rosbag replay).

**Machine**: the x86 WSL2 Ubuntu 24.04 development machine for everything except the final clean-machine check (2E.4), which runs once on a native Ubuntu 24.04 box.

**Status**: ⬜ Not started

**Depends on**: Stage 1 (audits — complete). No device or infra prerequisites.

## 2A Autoware base (amd64 deb + release)

Background:

- AutoSDV installs Autoware as a single `autoware-localrepo_<ver>_<arch>.deb` downloaded from `https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F<tag>/...` (see `setup/scripts/install-autoware-debian.sh:37-48`).
- Latest existing release: `rosdebian/2025.02-1` (amd64, arm64, jetpack6.0). No 1.8.0 and no jazzy release exists yet.
- Autoware 1.8.0 upstream already dual-supports humble/jazzy (`setup-dev-env.sh --ros-distro jazzy`; ansible maps Ubuntu 24.04 → jazzy). The official ROS buildfarm carries `ros-jazzy-autoware-core`, but **universe is not on the buildfarm** — hence the localrepo deb remains the delivery mechanism.
- Autoware 1.8.0 amd64 pins (from `ansible/roles/{cuda,tensorrt}/defaults/main.yaml`): CUDA 12.8; TRT `10.8.0.43-1+cuda12.8`.

Work items:

- [ ] **2A.1** (old 2.1.1) Sync NEWSLabNTU/autoware fork with upstream `autowarefoundation/autoware` tag **1.8.0**; create packaging branch (e.g. `rosdebian/1.8.0`).
- [ ] **2A.2** (old 2.1.2) Rebase the rosdebian packaging infrastructure (localrepo build scripts/CI) onto the 1.8.0 tree; audit packaging commits the same way as Stage 1 forks (KEEP / OBSOLETE).
- [ ] **2A.3** (old 2.1.3) Pin `autoware.repos` at the released 1.8.0 set (`autoware_core: 1.8.0` etc.); do **not** bump to 1.9.0 in this migration.
- [ ] **2A.4** (old 2.2.1) Build the full autoware workspace on Ubuntu 24.04 / jazzy (amd64) — natively or in `ghcr.io/autowarefoundation/autoware:universe-devel-cuda-jazzy`.
- [ ] **2A.5** (old 2.2.2) Package as `autoware-localrepo_1.8.0-1_amd64.deb`; record SHA256.
- [ ] **2A.6** (old 2.4.1, split) **Publish GitHub release `rosdebian/1.8.0-1` now, with the amd64 deb only.** The `jetpack7.2` deb is attached to this same release later in Stage 3C — do not block on it.
- [ ] **2A.7** Verify the amd64 deb: installs on clean Ubuntu 24.04, `source /opt/autoware/...` works, planning simulator launches.

## 2B AutoSDV repo config

Update every Ubuntu 22.04 / Humble / Autoware 2025.02 coupling point to Ubuntu 24.04 / Jazzy / Autoware 1.8.0, and bump AutoSDV to `0.2.0-dev`. Jetson-specific edits are **authored here in the same sweep but only verified in Stage 3/4** — marked ⏳ below.

### versions.yaml (single source of truth — update first)

- [ ] **2B.1** (old 3.1.1) `autosdv.version`: `0.1.0-dev` → **`0.2.0-dev`** (Autoware base upgrade = MAJOR-track bump per CLAUDE.md guidelines).
- [ ] **2B.2** (old 3.1.2) `autoware.version`: `2025.02` → `1.8.0`; `rosdebian_release`: `rosdebian/2025.02-1` → `rosdebian/1.8.0-1`; `package_version`: `2025.2-1` → `1.8.0-1` (match 2A.5 deb filename).
- [ ] **2B.3** (old 3.1.3) `ros.distro`: `humble` → `jazzy`.
- [ ] **2B.4** (old 3.1.4) `nvidia_amd64`: `cuda: 12.8`, `tensorrt: 10.8.0.43-1+cuda12.8` (+ matching cuDNN 9.x).
- [ ] **2B.5** (old 3.1.5) ⏳ `nvidia_arm64`: `jetpack: 7.2`, `l4t: 39.2`, `cuda: 13.2`, `tensorrt: 10.16.2` (+ matching cuDNN from JP7.2). *Verified on device in Stage 4.*
- [ ] **2B.6** (old 3.1.6) `checksums`: replace `autoware_deb_jetpack60` with **`autoware_deb_jetpack72`**; fill amd64 SHA256 from 2A.5. ⏳ jetpack72 value stays a **placeholder** until Stage 3C produces the deb.
- [ ] **2B.7** (old 3.1.7) Verify `scripts/version/get-version.sh` and `scripts/version/export-versions.sh` handle the renamed checksum key.

### Setup scripts

- [ ] **2B.8** (old 3.2.1) ⏳ `setup/scripts/install-autoware-debian.sh`: update JetPack detection — `R36 (release), REVISION: 3\.` regex (line 29) → JP7.2 pattern for `/etc/nv_tegra_release` (r39.2; confirm exact string on device in Stage 4); `jetpack6.0` deb filename/variable (lines 40-43) → `jetpack7.2`; `CHECKSUM_AUTOWARE_DEB_JETPACK60` → `CHECKSUM_AUTOWARE_DEB_JETPACK72`.
- [ ] **2B.9** (old 3.2.2) `setup/scripts/install-ros2.sh:7`: `ROSDISTRO="${ROSDISTRO:-humble}"` → `jazzy`.
- [ ] **2B.10** (old 3.2.3) `setup/scripts/install-zed-sdk.sh`: pin ZED SDK **5.4** (Ubuntu 24.04 build for amd64; ⏳ CUDA 13 / TRT 10 Jetson variant `ZED_SDK_Tegra_L4T39.2_v5.4.0.zstd.run`, URL from Stage 1 §1.2.2 — note it is a `.run`, current script installs a `.deb` from `jerry73204/zed-sdk-debian-package` pinned `4.2-1`; switch to the `.run` or rebuild a 5.4 deb).
- [ ] **2B.11** (old 3.2.4) `setup/scripts/install-blickfeld.sh`: bump/rebuild blickfeld-scanner-lib for noble per Stage 1 §1.1.6 audit outcome (packaging-only overlay; ⚠ `setup.py install` removed in noble's setuptools (PEP 517) → python-install step needs rework during the deb rebuild).
- [ ] **2B.12** (old 3.2.5) `setup/justfile:147`: `ros-humble-plotjuggler-ros` → `ros-jazzy-plotjuggler-ros` (also the apt-mark hold name).
- [ ] **2B.13** (old 3.2.6) `setup/justfile:129-130`: `autonomoustuff-public-humble.yaml` → jazzy equivalent. Per Stage 1 §1.2.5 the jazzy yaml is **404** — investigate whether pacmod/autonomoustuff deps are now in the ROS jazzy apt index or published under a different name (blocker carried from Stage 1).
- [ ] **2B.14** (old 3.2.7) `setup/justfile:201`: `source /opt/ros/humble/setup.sh` → jazzy; `rosdep update --rosdistro=jazzy`.

### Hardcoded `/opt/ros/humble` paths (mechanical sweep)

Replace `source /opt/ros/humble/setup.bash` → `/opt/ros/jazzy/` in:

- [ ] **2B.15** (old 3.3.1) `.envrc:46`
- [ ] **2B.16** (old 3.3.2) `Makefile:74,82`
- [ ] **2B.17** (old 3.3.3) `scripts/play_rosbag.sh:5`
- [ ] **2B.18** (old 3.3.4) `scripts/record_localization.sh:5`
- [ ] **2B.19** (old 3.3.5) `scripts/build/package-deb.sh:9`
- [ ] **2B.20** (old 3.3.6) `scripts/build/make-deb.sh` (line 22 — hardcoded separately from its `$ROS_DISTRO` usage)
- [ ] **2B.21** (old 3.3.7) `scripts/testing/ntrip/check_ntrip_setup.sh:5`
- [ ] **2B.22** (old 3.3.8) Final sweep: `rg -n "ros/humble|ros-humble-|humble" --glob '!data/**' --glob '!docs/**'` returns no functional hits.

### Docker & CI

- [ ] **2B.23** (old 3.4.1) ⏳ `docker/Dockerfile:1`: base `nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel` → JP7.2-era base (`nvcr.io/nvidia/l4t-jetpack:r39.x` or the JP7.2 TensorRT devel image). *Image build verified in Stage 3.*
- [ ] **2B.24** (old 3.4.2) `docker/Dockerfile:4`: `ARG COMMIT_HASH=2025.02` → the new release branch/tag.
- [ ] **2B.25** (old 3.4.3) ⏳ `docker/nvidia-l4t-apt-source.list`: `r36.3` → `r39.2`.
- [ ] **2B.26** (old 3.4.4) `.github/workflows/docker-build.yml:5`: branch trigger `[2025.02]` → new branch name.
- [ ] **2B.27** (old 3.4.5) Handle noble base-image `ubuntu` user (UID 1000) conflict if the container creates its own user.

## 2C Submodules (13 repos)

Migrate every `src/` submodule to jazzy / Autoware 1.8.0 using the Stage 1 audit results, then repin `.gitmodules` SHAs in AutoSDV. Builds against jazzy + the amd64 1.8.0 localrepo deb from 2A.

Strategy:

- **Forks with hardware patches**: re-fork — create branch `autosdv-1.8.0` from the **upstream release tag** chosen in Stage 1 (never `master`/`main` HEAD), then cherry-pick only KEEP-classified commits.
- **NEWSLabNTU originals**: port in place on a `autosdv-1.8.0` branch.
- **Non-org repos**: prefer updated upstream; fork into the org only if patches are needed.
- After each repo migrates: build it against jazzy + localrepo 1.8.0, then update the pinned SHA in AutoSDV (`git submodule` pointer) — one commit per repo.

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

Work items:

- [ ] **2C.1** (old 4.1) Migrate repos 1–5 (forks; re-fork + cherry-pick per Stage 1 audit).
- [ ] **2C.2** (old 4.2) Migrate repos 6–12 (org originals; port in place).
- [ ] **2C.3** (old 4.3) Repo 13: verify `evshary/autoware_manual_control` main builds on jazzy + 1.8.0; pin the verified SHA.
- [ ] **2C.4** (old 4.4) Common jazzy porting checklist applied to each C++ repo:
  - `ament_target_dependencies` / rclcpp API drift (rclcpp 28+)
  - message API changes vs Autoware 1.8.0 (`autoware_*_msgs`, `tier4_*_msgs`)
  - gcc-13/noble warnings-as-errors fixes
  - `package.xml` rosdep keys valid on jazzy (`rosdep resolve` clean)
- [ ] **2C.5** (old 4.5) Update `.gitmodules` (ros-nmea-reader URL → NEWSLabNTU fork) and repin all 13 submodule SHAs in AutoSDV; `make checkout` clean on a fresh clone. Reconcile Stage 1 discrepancies: CalibrationTools `.gitmodules` pin (branch `2025.02` @ `6286609` vs audit's `autosdv-2025.02` @ `005dd8c`; jazzy support lives on branch `tier4/universe` @ `d434e57` with **no release tag** — pin a commit, convention exception).

## 2D In-tree & vendored packages

| Package                     | Path                                                             | Risk                                                       |
|-----------------------------|------------------------------------------------------------------|------------------------------------------------------------|
| Blickfeld driver (vendored) | `src/sensor_component/external/ros2_blickfeld_driver_src-v1.5.5` | Medium — C++ driver + patched scanner lib                  |
| Isaac SLAM launch           | `src/localization/autosdv_isaac_slam_launch`                     | Gated — Isaac ROS jazzy availability (Stage 1 §1.2.3)      |
| Control test                | `src/vehicle/control_test`                                       | Low — Python; depends on `tier4_*_msgs`, `autoware_*_msgs` |
| Main launcher               | `src/launcher/autosdv_launch`                                    | Low — launch YAML; Autoware 1.8.0 launch args              |

Work items:

- [ ] **2D.1** (old 5.1.1) Rebuild blickfeld-scanner-lib on noble/gcc-13 per Stage 1 §1.1.6 audit (keep `-newslab1` patch only if still needed; check for newer upstream lib and driver than v1.5.5).
- [ ] **2D.2** (old 5.1.2) Build `ros2_blickfeld_driver` against jazzy; fix rclcpp 28+ / gcc-13 issues. (📟 old 5.1.3, point-cloud verification with a real Cube1 → Stage 4D.3.)
- [ ] **2D.3** (old 5.2.1) Isaac SLAM (gated): apply Stage 1 §1.2.3 decision — Isaac ROS supports jazzy; update apt repo + package names (`ros-humble-isaac-ros-*` → `ros-jazzy-isaac-ros-*`) and rebuild; if the jazzy packages turn out unavailable for the needed set, mark the package `COLCON_IGNORE` with a note and keep the humble instructions in [../isaac_vslam.md](../isaac_vslam.md) as historical.
- [ ] **2D.4** (old 5.2.2) If ported: verify `odometry_pose_bridge` unit tests still pass (6/6 on humble baseline).
- [ ] **2D.5** (old 5.3.1) `control_test`: verify `tier4_*_msgs` / `autoware_*_msgs` deps exist in the 1.8.0 localrepo; fix any renamed message fields; `make test-control` scripts unaffected.
- [ ] **2D.6** (old 5.3.2) `autosdv_launch`: cross-check `autosdv_autoware.launch.xml` and perception presets against Autoware 1.8.0 launch arguments (autoware_launch 1.8.0); update renamed/removed args and parameter files.

## 2E AMD64 integration + validation

- [ ] **2E.1** (old 6.1.1 part + old 6.1.2 + old 7.1.1) On the WSL2 machine: `make checkout`, `make build`, `make test` — zero errors, all packages green.
- [ ] **2E.2** (old 6.1.3 + old 7.1.2) Rosbag replay: `make launch` smoke with `logging_simulation.launch.yaml` (no hardware); `scripts/play_rosbag.sh` with an existing Humble-era bag — verify message compat (rosbag2 format across humble→jazzy; convert bags if needed) and that localization/perception pipelines produce output.
- [ ] **2E.3** (old 7.1.3) Compare replay behavior against the 0.1.0-dev baseline (localization stability, perception detections) — no regressions.
- [ ] **2E.4** (old 6.1.1 clean-machine part + old Phase 3 exit criterion) **Clean native Ubuntu 24.04 box**: `./setup.sh` completes from scratch (installs jazzy + the 1.8.0 localrepo deb with checksum verification + ZED SDK 5.4 + blickfeld lib), then `make checkout` + `make build` succeed.

## Goal (exit criteria)

- [ ] GitHub release `rosdebian/1.8.0-1` published with the amd64 deb; SHA256 in `versions.yaml`.
- [ ] `./scripts/version/get-version.sh ros.distro` returns `jazzy`; `autosdv.version` returns `0.2.0-dev`.
- [ ] `rg "humble"` over the repo (excluding `data/`, historical docs, and this roadmap) returns zero functional references.
- [ ] All 13 submodules have an `autosdv-1.8.0` branch (or verified upstream pin for #13) that builds green against jazzy + localrepo 1.8.0; zero un-audited fork commits left behind (Stage 1 table fully executed: every KEEP cherry-picked, every OBSOLETE dropped).
- [ ] All four in-tree packages build green on jazzy (amd64), or are explicitly gated with `COLCON_IGNORE` + documented reason (Isaac only); `colcon test` passes for `control_test` and (if ported) `autosdv_isaac_slam_launch`; `autosdv_launch` dry-launches without unknown-argument errors against Autoware 1.8.0.
- [ ] `make build` + `make test` green; rosbag replay passes with no regressions vs the Humble baseline.
- [ ] Clean native Ubuntu 24.04 machine: `./setup.sh` + fresh clone + `make checkout` + `make build` succeed.

No item in this stage requires the ARM64 server or the Jetson. ⏳-marked edits are authored here and verified in Stage 3/4.
