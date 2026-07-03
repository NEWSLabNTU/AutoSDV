# Phase 1 — Execution Worklog (redacted)

Traceable record of the Phase 1 audit: commands and migration-relevant results, so every value in `stage_1_audit.md` is reproducible. Run **2026-07-02** on the ARM64 build server.

**Redaction policy:** this file records only migration-relevant, non-identifying facts (git SHAs, branch/tag names, public URLs, generic outcomes). Host-identifying or security-sensitive specifics — account names, UIDs, exact driver builds, container UID-map state, raw system error strings, machine specs — are deliberately **excluded** and kept out-of-band.

## 1. Fork reachability
`git ls-remote` — all 6 targets reachable. `NEWSLabNTU/ros-nmea-reader` **exists** (branches `main`, `2026-golf`, both at `c584054`).

## 2. Clones + source branches
Disposable clones under the session scratchpad (not the repo). Per fork: `git clone` → `git checkout <source-branch>` → `git remote add upstream <up>` → `git fetch upstream --tags`.
Source heads: zed `autosdv-2025.02`@2d02559 · CalibrationTools `autosdv-2025.02`@005dd8c · seyond `autosdv-2025.02`@10c9599 · mpu9250 `main`@e90ef97 · nmea `main`@c584054.

## 3. merge-base + org-only commits
`git merge-base HEAD upstream/<base>` → `git rev-list --count <mb>..HEAD` → `git log --oneline <mb>..HEAD`.

CalibrationTools base selection (ahead-count per candidate upstream branch):
```
upstream/main           ahead=263  mb=e5a9117
upstream/tier4/universe ahead=3    mb=2bfceec   ← true base
upstream/humble         ahead=202  mb=9907d72
upstream/galactic       ahead=230  mb=8fc2c06
```
→ true divergence = 3 (doc's "~300" matched `main`, a wrong base).

| Fork                | upstream base  | merge-base | ahead |
|---------------------|----------------|------------|-------|
| zed-ros2-wrapper    | master         | a66e227    | 1     |
| CalibrationTools    | tier4/universe | 2bfceec    | 3     |
| seyond_ros_driver   | main           | ff961b0    | 7     |
| ros2_mpu9250_driver | main           | 482aa09    | 3     |
| ros-nmea-reader     | main           | c584054    | 0     |

Commit SHAs and per-commit KEEP/RE-VERIFY classification: see `stage_1_audit.md` → *Commit classification*.

## 4. Upstreamed-checks / target tags
```
git merge-base --is-ancestor a66e227 v5.4.0   → true  (zed base is in v5.4.0)
git log -1 upstream/tier4/universe            → d434e57 "feat: jazzy support (#276)" 2026-02-20
git tag -l (CalibrationTools)                 → only 2022-era tags → NO jazzy release tag
git cherry -v v1.0.3 HEAD (seyond)            → all 7 '+'  → none upstreamed by v1.0.3
git tag -l (seyond)                           → v1.0.0..v1.0.3
git tag -l (mpu9250)                          → none (upstream inactive)
```

## 5. blickfeld patch
`git diff --stat v2.20.6 v2.20.6-newslab1` → only `debian/`, `.ci/`, and 1 line in `python/CMakeLists.txt`
(appends `--prefix=$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}` to a `setup.py install`).
→ packaging-only, no library change; `setup.py install` deprecated on noble.

## 6. Environment gates
```
curl -sI  .../autonomoustuff-public-jazzy.yaml   → HTTP 404   (humble → 200)
curl -sIL .../zedsdk/5.4/l4t39.2/jetsons         → 206 → ZED_SDK_Tegra_L4T39.2_v5.4.0.zstd.run
        (control probe .../l4t99.9/jetsons       → 200 redirect to marketing page = not a real file)
```
Web: Isaac ROS docs — all packages tested on ROS 2 Jazzy; JP7 supported (Thor-focused) → jazzy blocker resolved.

## 7. CUDA-13 build-half spike
Outcome: **blocked (infra)** — no usable container runtime for the build account, and the host driver predates the CUDA-13 runtime. The arm64 noble CUDA-13 image exists. Details and remediation: `stage_1_audit.md` §1.2.4. Host-specific diagnostics kept out-of-band.

## 8. Closing the 3 RE-VERIFY commits (against the target tree, git+grep, no build)

```
# zed 2d02559 vs v5.4.0
git show v5.4.0:zed_wrapper/launch/zed_camera.launch.py | rg -n container
  → :474  full_container_name = '/' + namespace_val + '/' + container_name_val   (pre-patch pattern still present)
  ⇒ KEEP (fix absent in v5.4.0)

# CalibrationTools 150f466 (remove ndt_omp) vs d434e57
git grep -n ndt_omp d434e57
  → build_depends.repos / calibration_tools_standalone.repos: vendor/ndt_omp (tier4/ndt_omp)
  → mapping_based_calibrator/package.xml:26  <depend>ndt_omp</depend>
  → *.hpp  #include <pclomp/ndt_omp.h>
  ⇒ OBSOLETE (upstream vendors ndt_omp; the commented-out depend was a humble rosdep workaround)

# CalibrationTools 235de71 (Ceres workaround) vs d434e57
git show 235de71 → switches ${CERES_LIBRARIES}→Ceres::ceres, adds GLOG_NO_ABBREVIATED_SEVERITIES, pybind11 MODULE
git show d434e57:.../ceres_intrinsic_camera_calibrator/CMakeLists.txt → still ${CERES_LIBRARIES}, no MODULE, no glog defs
  ⇒ KEEP (fix absent upstream; noble-era Ceres/glog; downgrade to OBSOLETE only if Phase-4 build is clean)
```

Result: zed KEEP, CalibrationTools 235de71 KEEP, 150f466 **OBSOLETE**. No commit left at RE-VERIFY.

## Result
All findings folded into `stage_1_audit.md`. Scratch clones disposable; reproducibility = the SHAs + commands above. Only the two migration docs changed; nothing committed.
