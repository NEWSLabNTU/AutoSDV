# Phase 1: Fork-Patch Audits & Environment Gates

**Objective**: Know exactly what hardware-fix commits each NEWSLabNTU fork carries, which are still needed, and confirm every external prerequisite (JetPack 7.2, ZED SDK 5.4, Isaac ROS, CUDA 13 cross-build) before any migration work starts.

**Status**: 🚧 In progress — §1.1 fork audits complete; §1.2 env gates 1.2.2/1.2.3/1.2.5 resolved, 1.2.1 + 1.2.4 device/infra-blocked (recorded below). Audited 2026-07-02 on the ARM64 build server.

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

- [x] **1.1.1** Audit `NEWSLabNTU/zed-ros2-wrapper` (fork of stereolabs/zed-ros2-wrapper, humble-era). Target upstream tag: **v5.4.0** (pairs with ZED SDK 5.4).
- [x] **1.1.2** Audit `NEWSLabNTU/CalibrationTools` (fork of tier4/CalibrationTools, branch `autosdv-2025.02`). Determine tier4 upstream jazzy state and pick target tag/branch. **Correction:** the "~300 commits" figure was measured against `tier4/main`; the true org divergence vs `tier4/universe` (the branch the fork actually tracks) is **3 commits**.
- [x] **1.1.3** Audit `NEWSLabNTU/seyond_ros_driver` (fork of Seyond-Inc/seyond_ros_driver; upstream already supports jazzy). Pick upstream release tag.
- [x] **1.1.4** Audit `NEWSLabNTU/ros2_mpu9250_driver` (fork of hiwad-aziz/ros2_mpu9250_driver; small driver, no distro claim upstream).
- [x] **1.1.5** Audit `NEWSLabNTU/ros-nmea-reader` fork (upstream jerry73204/ros-nmea-reader). Fork carries **0 commits** (identical to upstream). ⚠ Discrepancy: live `.gitmodules` pins `jerry73204/ros-nmea-reader` (the upstream), not the `NEWSLabNTU` fork the doc names — Phase 4 repoint decision.
- [x] **1.1.6** Audit blickfeld-scanner-lib **`v2.20.6-newslab1`** patch (installed by `setup/scripts/install-blickfeld.sh:29`): diff against upstream v2.20.6, decide whether the patch is still needed on noble/gcc-13.
- [x] **1.1.7** For each fork, record the chosen upstream tag in the table below (never `master`/`main` HEAD).

### Audit results

| Repo                  | Upstream target (to-state)                     | KEEP | OBSOLETE | UPSTREAMED | Total org commits |
|-----------------------|------------------------------------------------|------|----------|------------|-------------------|
| zed-ros2-wrapper      | **v5.4.0**                                     | 1    | 0        | 0          | 1                 |
| CalibrationTools      | `tier4/universe` @ **d434e57** (no jazzy tag)  | 2    | 1        | 0          | 3                 |
| seyond_ros_driver     | **v1.0.3**                                      | 7    | 0        | 0          | 7                 |
| ros2_mpu9250_driver   | upstream `main` @ **482aa09** (no tags)        | 3    | 0        | 0          | 3                 |
| ros-nmea-reader       | jerry73204 `main` @ **c584054** (no tags)      | 0    | 0        | 0          | 0                 |
| blickfeld-scanner-lib | upstream **v2.20.6** + rebuild deb on noble    | 1 (packaging) | 0 | 0        | 1 (deb overlay)   |

**Reproducibility record** (git objects immutable → re-runnable; no retained dir needed):

| Repo | Fork cloned | Source branch | Upstream base ref | merge-base SHA |
|------|-------------|---------------|-------------------|----------------|
| zed-ros2-wrapper | NEWSLabNTU/zed-ros2-wrapper | autosdv-2025.02 (@2d02559) | stereolabs `master` | `a66e227` |
| CalibrationTools | NEWSLabNTU/CalibrationTools | autosdv-2025.02 (@005dd8c) | tier4 `tier4/universe` | `2bfceec` |
| seyond_ros_driver | NEWSLabNTU/seyond_ros_driver | autosdv-2025.02 (@10c9599) | Seyond-Inc `main` | `ff961b0` |
| ros2_mpu9250_driver | NEWSLabNTU/ros2_mpu9250_driver | main (@e90ef97) | hiwad-aziz `main` | `482aa09` |
| ros-nmea-reader | NEWSLabNTU/ros-nmea-reader | main (@c584054) | jerry73204 `main` | `c584054` (==HEAD) |

Commands per fork: `git merge-base HEAD upstream/<base>` → `git rev-list --count <mb>..HEAD` → `git log --oneline <mb>..HEAD`; upstreamed-check via `git cherry -v <tag> HEAD` / `git tag --contains`.

### Commit classification

**zed-ros2-wrapper** (target v5.4.0; base `a66e227` is an ancestor of v5.4.0):

| SHA | Subject | Class | Notes |
|-----|---------|-------|-------|
| 2d02559 | Fix container name resolution | **KEEP** (confirmed vs v5.4.0) | v5.4.0's `zed_wrapper/launch/zed_camera.launch.py:474` still has the pre-patch `full_container_name = '/' + namespace_val + '/' + container_name_val`; the absolute-path (`container_name` starting with `/`) handling is absent upstream. Patch still applies. |

**CalibrationTools** (target `tier4/universe` @ d434e57 = "feat: jazzy support (#276)", 2026-02-20; no release tag exists):

| SHA | Subject | Class | Notes |
|-----|---------|-------|-------|
| 005dd8c | Update calibration_tools path in `calibration_tools_standalone.repos` | KEEP | AutoSDV-specific vendoring path. |
| 235de71 | Workaround Ceres compilation error | **KEEP** (confirmed vs d434e57) | Patch switches to `Ceres::ceres`, adds glog-conflict defs (`GLOG_NO_ABBREVIATED_SEVERITIES`) and `pybind11 MODULE`. d434e57 still uses old `${CERES_LIBRARIES}`, no `MODULE`, no glog defs → fix absent upstream; targets noble-era Ceres/glog. (Downgrade to OBSOLETE only if the Phase-4 jazzy build compiles clean without it.) |
| 150f466 | Remove ndt_omp dependency | **OBSOLETE** (confirmed vs d434e57) | d434e57 **vendors** ndt_omp (`vendor/ndt_omp` in `build_depends.repos` / `calibration_tools_standalone.repos`) and still `<depend>ndt_omp</depend>` + `#include <pclomp/ndt_omp.h>`. The org commit only commented out the depend — a humble rosdep workaround. Drop it; keep upstream's vendored ndt_omp in the imported `.repos`. |

**seyond_ros_driver** (target v1.0.3; `git cherry -v v1.0.3 HEAD` → all 7 marked `+`, i.e. none upstreamed):

| SHA | Subject | Class | Notes |
|-----|---------|-------|-------|
| 037766f | Use patched seyond_sdk from NEWSLab | KEEP | Vendored SDK. |
| 6e824b0 | Fix incorrect library names in CMakeLists | KEEP (re-verify) | Upstream reorganized build since fork base; re-check against v1.0.3. |
| b92c1e6 | Build seyond_sdk automatically | KEEP (re-verify) | " |
| 540ca30 | Fix syntax error in CMakeLists | KEEP (re-verify) | " |
| 3e12060 | Support PointXYZIRC format | KEEP | Autoware point spec; not upstream. |
| 8e99e38 | Follow Autoware preprocessor point spec (XYZIRC) | KEEP | Autoware-specific. |
| 10c9599 | Prevent RViz from starting in launch | KEEP | Deployment preference. |

**ros2_mpu9250_driver** (target upstream `main` @ 482aa09; upstream inactive, no tags):

| SHA | Subject | Class | Notes |
|-----|---------|-------|-------|
| 94e3f71 | Add libi2c-dev build dependency | KEEP | Build dep. |
| 5dea70c | Inhibit frequent error message | KEEP | Runtime log noise fix. |
| e90ef97 | Fix missing `#include <array>` | KEEP | Required for gcc-13 (noble) — do not drop. |

**ros-nmea-reader**: fork == upstream (`c584054`), 0 patches. User-confirmed upstream unchanged. Nothing to carry forward.

**blickfeld-scanner-lib v2.20.6-newslab1**: diff vs upstream `v2.20.6` is **packaging-only** — adds `debian/` (PKGBUILD, `fix_python_install_dir.patch`), `.ci/` release automation, and a 1-line `python/CMakeLists.txt` change appending `--prefix=$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}` to a `setup.py install`. **No C++/library change** — the library builds identically to upstream v2.20.6. → KEEP the deb-packaging overlay and rebuild amd64/arm64 debs on noble. ⚠ `setup.py install` is removed in noble's setuptools (PEP 517) → the python-install step needs rework during the deb rebuild.

## 1.2 Environment gates

- [ ] **1.2.1** 📟 *Device-only — DEFERRED (no Jetson attached to the audit host).* Flash JetPack 7.2 (Jetson Linux r39.2) on the actual Orin; confirm boot, `nvidia-smi`/`tegrastats`, CUDA 13.2.1 and TensorRT 10.16.2 present. **Owner: device operator.** Acceptance: `nvidia-smi` shows driver ≥ the JP7.2 baseline, `nvcc --version` = 13.2.x, `dpkg -l | rg tensorrt` = 10.16.2.
- [x] **1.2.2** ZED SDK **5.4** JP7.2 installer **confirmed to exist**. `https://download.stereolabs.com/zedsdk/5.4/l4t39.2/jetsons` (HTTP 206) redirects to the real file **`https://stereolabs.sfo2.cdn.digitaloceanspaces.com/zedsdk/5.4/ZED_SDK_Tegra_L4T39.2_v5.4.0.zstd.run`**. Note: this is a `.run` installer; current `setup/scripts/install-zed-sdk.sh` installs a `.deb` from `jerry73204/zed-sdk-debian-package` (pinned `4.2-1`) — Phase 2/4 must either switch to the `.run` or rebuild a 5.4 deb.
- [x] **1.2.3** Isaac ROS **now supports ROS 2 Jazzy** (all Isaac ROS packages tested on Jazzy; Jetson + x86_64+GPU) and JetPack 7 — the roadmap's "availability unknown" blocker is **resolved**. Caveat: JP7 docs emphasize Jetson **Thor** (JP7.0); Orin-AGX on **JP7.2** specifically must be confirmed on-device. **Decision:** keep `autosdv_isaac_slam_launch` gated through Phase 5 (migrate `ros-humble-isaac-ros-*` → jazzy packages), but it is now un-blockable rather than indefinitely deferred.
- [ ] **1.2.4** CUDA 13 cross-build spike — **BLOCKED, build-half could not run on the ARM64 build server** (infra, not a code problem). Root causes (generic): (a) no usable OCI container runtime for this account — rootless UID mapping is not configured, and the system daemon is not accessible; (b) the host GPU driver predates CUDA 13, so its CUDA **runtime** ceiling is below 13 — CUDA-13 binaries cannot execute here even if built. The arm64 noble CUDA-13 image itself is available (`nvidia/cuda:13.0.0-devel-ubuntu24.04`, multi-arch). **Unblock (infra):** enable a container runtime for the build account (rootless UID/GID mapping *or* daemon access) **and** provide a driver new enough for the CUDA-13 runtime; then re-run the build-half. The definitive verify-half (run on Orin, sm_87) remains device-blocked. **Owner: infra admin + device operator.** *(Host-specific diagnostics — account, driver build, UID-map state — recorded out-of-band, not in this repo.)*
- [x] **1.2.5** `autonomoustuff-public-jazzy.yaml` **does NOT exist** — `https://s3.amazonaws.com/autonomoustuff-repo/autonomoustuff-public-jazzy.yaml` → **HTTP 404** (the humble one → 200). `setup/justfile:129-130` has no jazzy rosdep source to point at. **Replacement TBD** — investigate whether pacmod/autonomoustuff deps are now in the ROS jazzy apt index or published under a different name; this is a Phase 2 blocker for the pacmod recipe.

## Goal (exit criteria)

- [x] Every fork in the audit table has: chosen upstream target + full commit classification (no unclassified commits). All verdicts confirmed against the **target** upstream (zed vs v5.4.0, seyond `git cherry` vs v1.0.3, CalibrationTools vs d434e57). One OBSOLETE found (CalibrationTools ndt_omp). *(seyond's 4 CMake commits stay KEEP but should be re-tried at the Phase-4 build since upstream reorganized its build; the CalibrationTools Ceres KEEP downgrades to OBSOLETE only if the jazzy build compiles clean without it.)*
- [ ] JetPack 7.2 confirmed running on target Orin hardware. — **deferred (device), see 1.2.1**
- [x] ZED SDK 5.4 JP7.2 installer URL recorded. — `ZED_SDK_Tegra_L4T39.2_v5.4.0.zstd.run`
- [x] Isaac ROS jazzy decision recorded — jazzy **supported**; kept gated to Phase 5; verify Orin/JP7.2 on-device.
- [ ] CUDA 13 cross-build spike passed — **blocked (infra + device), see 1.2.4**; not yet run. Fallback if the server build stays blocked: build CUDA packages on-device on the Orin.

### Open blockers carried out of Phase 1

1. **rosdep jazzy source (1.2.5)** — `autonomoustuff-public-jazzy.yaml` is 404; need a jazzy replacement before the `pacmod` setup recipe works. *(Phase 2)*
2. **CUDA-13 build spike (1.2.4)** — needs subuid/subgid (or docker group) + driver ≥580 on the ARM64 server. *(infra)*
3. **JetPack 7.2 flash (1.2.1)** — device task. *(operator)*

### Discrepancies (doc vs live repo — for Phase 4 reconciliation)

- **ros-nmea-reader**: `.gitmodules` pins `jerry73204/ros-nmea-reader` (upstream); doc names `NEWSLabNTU/ros-nmea-reader` (fork, currently identical). Decide which to track when repinning.
- **CalibrationTools**: `.gitmodules` pins branch `2025.02` @ `6286609`; doc/audit uses `autosdv-2025.02` @ `005dd8c`. Reconcile the pin. Also: no jazzy **release tag** upstream — jazzy support lives on branch `tier4/universe` @ `d434e57`, so Phase 4 must pin a commit (convention exception to the "tag not HEAD" rule).
