# Backporting the golf cart work

**Goal**: bring the parts of `2026-golf-cart` that are not golf-cart-specific
back into AutoSDV — the CUDA point cloud pipeline, the `cuda_ndt_matcher`
bump, a driver QoS fix that is currently costing 60% of the Robin-W's points,
and the tooling and system-package work that accumulated on that side.

**Status**: Phase 6 done; the rest planned

**Source**: `~/repos/2026-golf-cart`, branch `main`

---

## Where the two repositories stand

`2026-golf-cart` was branched from AutoSDV and has diverged substantially. The
last shared commit is:

```
d2095873243c56e987e7affcfa93806ed63880e3  2026-02-02  Move scripts/*.sh to proper subdirs
```

Since then the golf cart side has 480 commits over 719 files
(+84,972 / −20,380). Most of that is golf-cart hardware — a CAN vehicle
interface, GMSL cameras, an Xsens CAN IMU, ArUco and reflective-board indoor
localization — and stays there. This roadmap covers only what is portable.

Both vehicles run an NVIDIA Jetson AGX Orin, which is why the CUDA and
profiling work transfers at all.

### Out of scope, deliberately

| Not ported | Reason |
|------------|--------|
| `golfcart_vehicle_launch` | no common ancestor with `autosdv_vehicle_launch`; golf cart is CAN, AutoSDV is PCA9685 PWM |
| `autoware_individual_params` | per-vehicle calibration |
| ArUco / reflective board localization | hardware AutoSDV does not have |
| `scripts/multi_machine/` | two-host Orin + Advantech deployment, not AutoSDV's shape |
| NDT parameter values | both sides are measured, on different maps and LiDARs — see phase 3 |

### AutoSDV-only work to protect

The launcher files being touched carry AutoSDV features the golf cart branch
never had. Any port that overwrites rather than merges will silently remove
them:

- `pose_source:=mcl` and `mcl_random_seed`
- `pose_source_package` pluggable estimator resolution
- Isaac visual localization (`pose_source:=isaac`, `pose_source:=visual`)
- the `use_sim_time` propagation block in
  `components/tier4_localization_component.launch.xml`

---

## Phase 1 — Seyond driver QoS fix

**One submodule bump, and the largest single win on this list.**

`seyond_ros_driver` publishes the point cloud with RELIABLE durability. The
publish runs on the SDK's single deliver worker (`worker_num = 1`, hardcoded in
`lidar_client.cpp`), which is also the thread that parses every incoming packet.
A reliable writer blocks that thread, the producer marks the backlog
non-preferred, and buffers are freed unparsed.

Measured on the golf cart's Orin over a 534 s run, from the driver's own
counters:

```
deliver queue#0  added=628,636  finished=251,406  dropped=377,005
                 blocked=0  active_time=507,917ms / elapsed=535,932ms
```

60% of the sensor discarded, rising to 65% by the end of the run. The SDK logs
"drop data in deliver stage" once per ten drops, so the log undercounts it by an
order of magnitude.

Nothing on that topic wants reliability: Autoware's concatenator subscribes with
`rclcpp::SensorDataQoS()`, rosbag2 adapts to the publisher, and RViz asks for
best effort. Offering RELIABLE is what created a reliable reader.

AutoSDV runs the Robin-W through this same driver, so AutoSDV is losing the same
points today.

**Tasks**:

1. `98dbcc8` sits on the fork's `golfcart` branch, which AutoSDV should not
   track. Cherry-pick it onto our own branch instead — and note that the
   rebase in 2.3 will replay it onto `v1.0.3`, so this is the same patch
   landing early rather than a second one.
2. Push the fork branch, **then** update the superproject pin. See CLAUDE.md,
   *Submodule Workflow*.

**Verification**: record a Robin-W bag before and after and compare point counts
per frame, or read the driver's own deliver-queue counters.

---

## Phase 2 — CUDA point cloud pipeline

AutoSDV has no CUDA sensing path at all. The golf cart has three independent
whole-stage switches:

| switch | selects | golf cart default |
|--------|---------|-------------------|
| `pointcloud_backend` | per-LiDAR preprocessing and concatenation | `cuda` |
| `localization_pointcloud_backend` | NDT input chain: crop, voxel, random downsample | `cpu` |
| `pose_source` | the scan matcher itself | `ndt` |

### 2.1 The constraint that governs the whole port

`cuda_blackboard` is **not a transport**. It is a process-local singleton
mapping a `UInt64` instance id to a device pointer. What crosses ROS is that
integer plus a negotiation handshake; the subscriber resolves the pointer *in
its own process*. A stage in a different process receives the id and finds
nothing behind it.

So every CUDA stage must load into the same `pointcloud_container`. That is a
requirement, not tidiness. Two consequences:

- Halves of one stage cannot mix. The CUDA concatenator needs the
  `pointcloud_before_sync/cuda` negotiation topic that only the CUDA
  preprocessor publishes. Autoware's own enum has no mixed mode for the same
  reason.
- `localization_pointcloud_backend:=cuda` alone still works — the blackboard
  subscriber has a compatible-topic fallback — but pays a host-to-device copy
  at its first stage.

Copy the golf cart's `docs/design/cuda-pipeline-data-flow.md` along with the
code; it is the map for wiring spread across a sensor kit launch file, a
localization launch file and a Rust package.

### 2.2 Which LiDARs actually benefit

**Only the VLP-32C.** The CUDA preprocessor fuses crop-self, distortion
correction and ring outlier filtering into one kernel sequence, and distortion
correction needs a per-point time offset. Nebula publishes `velodyne_points` in
the `PointXYZIRCAEDT` layout, which carries it.

`seyond_ros_driver` registers `PointXYZIRC`: x, y, z, intensity, return_type,
ring. No azimuth, no elevation, no distance, no per-point time. A cloud with no
per-point time cannot be deskewed by anything, CPU or GPU, so the Robin-W branch
is not merely un-accelerated, it is uncorrectable until the driver emits
`PointXYZIRCAEDT`. The golf cart routes its Seyond unit around the preprocessor
as a passthrough and lets the CUDA concatenator upload the plain `PointCloud2`.
The Blickfeld Cube1 needs the same treatment.

AutoSDV's default suite is `vlp32c_zed_imu`, so the default configuration does
benefit. `sensor_suite:=robin_zed` will not — until 2.3, which adds the missing
field to the driver and moves the Robin-W onto the accelerated path.

### 2.3 Seyond driver: rebase onto upstream 1.0.3, and add per-point time

The passthrough in 2.2 is a driver limitation, not a law. Fixing it is what
brings the Robin-W onto the accelerated path, so it belongs in this phase.

#### Where our fork stands

`NEWSLabNTU/seyond_ros_driver` carries 7 commits on top of upstream `v1.0.2`
(`ff961b0`):

| Commit | Change |
|--------|--------|
| `037766f` | point the nested `seyond_sdk` submodule at `NEWSLabNTU/inno-lidar-sdk` |
| `6e824b0` | fix incorrect library names in CMakeLists.txt |
| `b92c1e6` | build `seyond_sdk` automatically |
| `540ca30` | fix a CMakeLists.txt syntax error |
| `3e12060` | add `PointXYZIRC` |
| `8e99e38` | follow Autoware's preprocessor point spec for `PointXYZIRC` |
| `10c9599` | stop RViz starting from the launch file |

The golf cart branch adds two more, of which one is the phase 1 QoS fix
(`98dbcc8`) and the other is a comment rename (`85f2af8`, drop it).

The nested SDK fork is one commit deep: `73e456c` ("set minimum required CMake
version to 3.5") on top of the SDK commit upstream v1.0.2 pinned, `d4a8c40`.

#### What upstream 1.0.3 brings

`v1.0.3` is `e1ac1e5`, 9 commits past v1.0.2:

```
e1ac1e5 [feature]: support imu data publishing (#5)
41b1f20 [feat]: use frame start time (#6)
7ae9288 [fix]: update package.xml, del unused package
95d913e Added dependencies needed for building (#3)
48fd199 [chore]: update license, submodule
254373b [doc]: update readme
8c5ffd5 [fix]: scan_id set in PointXYZI
7d4c73e [chore]: del redundant parameters
6930e3e [feat]: support falcon ring_id
```

Three matter here:

- **`41b1f20` use frame start time.** The header stamp becomes the frame start.
  Autoware's per-point `time_stamp` is an offset from the header stamp, so this
  is the reference the offset needs, and it arrives for free.
- **`6930e3e` falcon ring_id** — upstream now sets the ring for the Falcon
  family, which our `PointXYZIRC` patch had to derive itself.
- **`48fd199` update submodule** — 1.0.3 pins SDK `c199dc7`, not the `d4a8c40`
  our SDK fork branched from, so the SDK fork rebases too.

#### The rebase

1. Rebase the nested SDK fork first: replay `73e456c` onto `c199dc7`, then check
   whether it is still needed — `c199dc7` may already carry a CMake minimum
   version fix, in which case the patch drops.
2. Rebase the driver's 7 commits plus `98dbcc8` onto `v1.0.3`. Expect
   `3e12060` and `8e99e38` to conflict with `6930e3e` (both touch the ring) and
   `037766f` to conflict with `48fd199` (both touch `.gitmodules` and the SDK
   pin). The three CMakeLists fixes may be answered by `95d913e`; verify before
   replaying them.
3. Push to `NEWSLabNTU/seyond_ros_driver` as **`autosdv-1.5.0`**.

The branch name follows this project's existing convention, where the suffix is
the Autoware release the patches are current for: the fork already carries
`autosdv-0.45.1` and `autosdv-2025.02`, and phase 9 settles that the current
target is 1.5.0. The old branches stay; they are the record of what worked
against those releases.

Nested submodule, so the lockstep rule applies twice over: push the SDK fork,
then the driver's SDK pin, then the AutoSDV superproject pin. See CLAUDE.md,
*Submodule Workflow*.

#### Adding per-point time

The driver already has the data. Upstream's own `seyond::PointXYZIT` carries a
`double timestamp` per point, and our `PointXYZIRC` path drops it. What is
missing is the Autoware layout, not the measurement.

Target layout, from `nebula_common/point_types.hpp`:

```cpp
struct PointXYZIRCAEDT
{
  float x; float y; float z;
  std::uint8_t  intensity;
  std::uint8_t  return_type;
  std::uint16_t channel;
  float azimuth; float elevation; float distance;
  std::uint32_t time_stamp;     // nanoseconds, offset from header.stamp
};
```

`ros2_driver_adapter.hpp` hand-builds the `PointCloud2` fields with compact
offsets, so this is an extension of an existing enumeration rather than new
machinery: six fields at `point_step` 16 become ten at 32.

Four things to decide while implementing, each a place the port can go quietly
wrong:

- **`time_stamp` is an offset, not an absolute.** Upstream's per-point value is
  absolute microseconds. The offset must be computed against the same origin
  the header carries, which is what `41b1f20` makes the frame start. Subtracting
  against the wrong origin yields a plausible-looking cloud that the distortion
  corrector shears.
- **Azimuth, elevation and distance are derived.** The SDK supplies none of the
  three; Nebula computes them in the driver from x, y, z, and so must this. That
  is three transcendentals per point on the CPU, before any GPU stage sees the
  cloud — measure it, because the point of the exercise is to save CPU.
- **`point_step` doubles.** 51,743 points went from 828 kB at step 16 to about
  1.66 MB at step 32. On the wire that interacts directly with phase 1: the
  driver's deliver thread is what was dropping 60% of the sensor, and this
  doubles what it has to hand to the publisher.
- **Keep `PointXYZIRC` selectable.** It is a compile-time typedef in
  `driver_lidar.h`; adding a third option rather than replacing the second keeps
  a fallback if the AEDT path misbehaves on the vehicle.

**Verification**: `ros2 topic echo --field fields` shows ten fields at the
offsets above; `CudaPointcloudPreprocessorNode` accepts the Robin-W cloud
without the layout error; and a bag recorded while driving shows the deskew
actually straightening structure, which a stationary bag cannot show.

**When this lands**, move the Robin-W from the passthrough list to the
preprocessed list in `pointcloud_preprocessor.launch.py`, and revisit 2.2.

### 2.4 Tasks

1. Port `src/sensing/golfcart_cuda_preprocessor` as
   `src/sensing/autosdv_cuda_preprocessor` (~19 files). Two CUDA nodes that
   upstream does not provide:
   - `cuda_crop_box_filter_node` — there is no standalone CUDA crop box
     upstream; the cropping in `CudaPointcloudPreprocessorNode` is fused with
     distortion correction.
   - `cuda_random_downsample_filter_node` — no CUDA version exists at all.

   Without both, the localization chain would pay a device-to-host copy before
   the one accelerated stage and a host-to-device copy after it.

   Two design decisions to preserve on the way across: non-finite points are
   dropped in **both** polarities (every comparison against NaN is false, so a
   `negative` implemented as `!inside` would hand NDT a NaN), and the crop box
   deliberately has no `output_frame` and does not transform — `input_frame` is
   an assertion, and a mismatched cloud is dropped with an error rather than
   cropped in the wrong frame.

2. Port the CUDA branch of `pointcloud_preprocessor.launch.py` into
   `autosdv_sensor_kit_launch` (golf cart: 326 lines, AutoSDV: 130). AutoSDV
   has one LiDAR where the golf cart has three, so the per-LiDAR table
   collapses, but keep the preprocessed/passthrough split from 2.2.

3. Add `pointcloud_backend` and `localization_pointcloud_backend` arguments to
   the top-level launch files.

4. Port `docs/design/cuda-pipeline-data-flow.md`.

**Dependencies**: already installed. `/opt/autoware/1.5.0/share` carries
`autoware_cuda_pointcloud_preprocessor`, `cuda_blackboard`, `autoware_cuda_utils`
and `autoware_cuda_dependency_meta`.

**Build**: the package is CUDA end to end with no CPU fallback, so its
`CMakeLists.txt` skips itself when no CUDA toolkit is found rather than
installing nodes that cannot load. `CMAKE_CUDA_ARCHITECTURES` defaults to
`87;86;89`; pass `-DCMAKE_CUDA_ARCHITECTURES=87` on the Orin.

**What is measured and what is not**:

| | |
|---|---|
| `pointcloud_backend:=cuda` | measured on the Orin: −23.7 points of container CPU, +33 points of GPU, +465 mW, equal throughput |
| `localization_pointcloud_backend:=cuda` | correctness verified, **speed never measured**. The CPU chain is ~19% of a core; the CUDA one has not been timed and could be slower |

Do not present the second as an optimisation until it is timed on AutoSDV
hardware.

**Verification**: `just launch pointcloud_backend:=cuda` reaches the same
concatenated cloud rate as `:=cpu`, and NDT converges on a COSS replay under
both.

---

## Phase 3 — cuda_ndt_matcher submodule bump

AutoSDV pins `e3f61f1`; the golf cart pins `db8c87b`, 22 commits ahead. AutoSDV
has **0** commits the golf cart lacks, so this is a fast-forward with no merge.

What arrives:

- `localization_pointcloud_backend` launch plumbing (needed by phase 2)
- GPU NVTL scoring, replacing a serial CPU loop
- the target grid is no longer repacked on every scan
- the aligned-scan overlay is built only when something subscribes
- three redundant scoring passes per frame removed
- per-frame degeneracy of the NDT registration is published
- a euler-convention bug in the GPU scorer, where poses were converted in the
  wrong convention for the scorer that read them
- an explicit CUDA floor for cubecl-cuda's cudarc, ending a 13.0 fallback

**Task**: bump `src/localization/cuda_ndt_matcher` to `db8c87b` or the current
`main`, then rebuild.

### NDT parameters are not part of this bump

The two repositories hold measured but conflicting tunings, on different maps
and different LiDARs:

| | AutoSDV (COSS map) | golf cart (CSIE-1, VLP-32C) |
|---|---|---|
| crop box | ±40 m | ±60 m |
| `ndt.resolution` | 4.0 | 2.0 |
| `max_iterations` | 15 | 30 |
| converged NVTL threshold | 2.2 | 1.3 |

Do not copy the values. Copy the reasoning, which is vehicle-independent and
which AutoSDV's own tuning notes should absorb:

**NVTL is a mean per-point likelihood, so it moves with how the input is
sampled, not only with how well the pose fits.** Widening the crop box from
±20 m to ±60 m spread the same 2000 sampled points over nine times the area:
measured NVTL fell from ~3.2 to a 1.95 median while the scan-to-map residual
*improved*, p50 0.210 → 0.144 m and p95 1.661 → 0.468 m. Left at the Autoware
default of 2.3, the matcher rejects poses that are demonstrably better, logs
"Score is below the threshold", and after `skipping_publish_num` consecutive
rejections stops publishing and deactivates — with the EKF dead-reckoning behind
it.

Whenever AutoSDV's crop box changes, the NVTL threshold has to be re-measured
with it.

### Two portable launch mechanics from the same file

Both vehicle-independent, both worth taking with the bump:

- **`ndt_param_file` argument**, so a tuning run can swap NDT parameters without
  editing the shipped file. Defaults to it, so nothing changes unless asked.
- **`gnss_enabled` defaulting to `use_gnss`.** Without this the stack keeps
  `gnss_enabled` at its upstream default of `true` even under `use_gnss:=false`,
  which leaves `pose_initializer` waiting on a GNSS pose that never arrives and
  launches `automatic_pose_initializer` to ask for it. Indoors that is a cold
  start that never completes. AutoSDV documents `use_gnss:=false` for indoor
  operation, so check whether it hits this.

---

## Phase 4 — Sensor kit: the xacro comment trap

The two sensor kits are separate repositories with no shared commits, so this is
re-application, not a cherry-pick. AutoSDV already has camera optical frames and
three recent ZED TF fixes, so only one item transfers, and it transfers as a
class of defect rather than a fix.

**Never write a colon followed by a space inside a comment in a `.xacro` file.**
Comments survive xacro expansion into the `robot_description` string, ROS 2
launch YAML-parses parameter values, and a colon-space makes `safe_load` fail:

```
ValueError: Failed to convert '<?xml version="1.0" ?> ...
using yaml rules: yaml.safe_load() failed
mapping values are not allowed here
```

The failure is remote from the cause. Nothing names a comment: the stack simply
never publishes `/localization/initialize` and the test harness reports that the
stack never came up. On the golf cart it blocked every NDT replay until found.

**Tasks**:

1. Audit `autosdv_sensor_kit_description/urdf/*.xacro` and the vehicle
   description for colon-space inside comments.
2. Add the warning comment at the top of the sensor kit xacro. Note that the
   golf cart's first attempt at that warning said no `word: word` in comments —
   which contains a colon-space and reproduced the bug exactly.

---

## Phase 5 — play_launch floor

AutoSDV pins no `--parser python` anywhere, so there is nothing to drop; what is
needed is the version floor that makes the default Rust parser safe.

`setup/scripts/install-play-launch.sh` has `REQUIRED_VERSION="0.9.0"`. Two
findings from the golf cart side:

- **0.8.2** rendered the pose initializer's array parameters as strings and
  `autoware_pose_initializer_node` died at startup, taking
  `/localization/initialize` with it. Fixed in **0.10.0** (`f78745da`).
- A 0.10.0 release build's **Rust parser** then died with
  `KeyError: 'rear_overhang'` — global parameters never reaching a `.launch.py`
  across the loader boundary (play_launch issue #0028). Fixed in **`8adc52ad`**,
  2026-09-11, ABI 4, which is *after* the 0.10.0 release.

With a build at or past `8adc52ad`, the Rust parser resolves every golf cart
entry point to the same node model as the Python one (indoor sim 83, NTU sim 81,
logging sim 130, planning sim 118, aruco sim 124 nodes).

**Tasks**:

1. Raise `REQUIRED_VERSION` to `0.10.0`, and the matching floor in
   `setup/justfile`'s status recipe.
2. Verify parser parity on AutoSDV's own entry points before relying on the Rust
   parser, using `play_launch dump` plus `context --tree` on
   `autosdv.launch.yaml`, `logging_simulation.launch.yaml` and the planning sim.
3. Record whether the `8adc52ad` fix has reached a release; until it has, the
   floor can only be documented, not enforced by a version check.

---

## Phase 6 — Tooling — DONE 2026-09-11

### 6.1 Localization diagnostics scripts

Seven scripts ported, into `scripts/testing/localization/` rather than a new
`scripts/localization/` — AutoSDV already had that directory with offline
counterparts (`summarize_ndt_run.py`, `compare_ndt_runs.py`, `ndt_yaw_bias.py`,
`tegrastats_summary.py`), and the two sets answer different halves of the same
question. The new ones are live tools:

| Script | What it answers |
|--------|-----------------|
| `check_ndt_activated.py` | is the matcher ACTIVATED, not merely alive? Exits 0/1 so a harness can gate |
| `ndt_quality_report.py` | pose quality over a window: scatter, yaw step, init-to-result, exe time |
| `ndt_alignment_report.py` | how far the live scan sits from the map, point by point |
| `ndt_timeseries.py` | every NDT signal against time, to CSV and PNG |
| `check_imu_velocity.py` | the two inputs the EKF prior is built from |
| `capture_initial_pose.py` | save a settled pose per named site |
| `set_initial_pose.py` | replay it, so a run needs no human at RViz |

Adaptations, each a place a straight copy would have been wrong:

- `ndt_alignment_report.py` hardcoded `/home/aeon/Downloads/2026-04_ntu_map/...`
  as its map; it now defaults to `data/COSS-map-planning/pointcloud_map.pcd`.
- `check_imu_velocity.py` hardcoded the golf cart's raw IMU topic. AutoSDV's
  depends on `imu_source` — `/sensing/camera/zedxm/imu/data` for the ZED,
  `/sensing/imu/mpu9250/imu_raw` for the MPU9250 — so it is now `--raw-topic`,
  defaulting to the ZED's.
- The pose pair wrote to `config/ntu_initial_poses/`, resolved three directories
  up. AutoSDV has no root `config/`, and one more directory level, so both now
  use `data/initial_poses/`.
- `ndt_timeseries.py` wrote to `/tmp`; it now resolves a relative `--out`
  against the repo root and defaults to `tmp/`, per CLAUDE.md.
- The Apache headers came off, matching the convention of every other script
  here; the repo's `LICENSE` covers them.

The existing `demo/scripts/seed_initialpose.py` stays as it is — it carries one
hardcoded COSS pose and is wired into `run-coss-ndt.sh`. The ported pair
generalises it to a named file per site and records which source the pose came
from: the planning simulator's is the click unchanged, a replay's is NDT
agreeing with the map. Both seed; only the second is a measurement.

Documented in `scripts/testing/localization/README.md`, whose stale
`scripts/localization-test/` paths were corrected while there.

### 6.2 Profiling scripts

Three scripts into a new `scripts/profiling/`, with a README:
`jetson_gpu_sampler.py`, `kernel_cpu_report.sh`, `perf_kernel.sh`.

The sampler earns its place on a fact worth repeating: **NVML does not exist on
Jetson**, so `play_launch`'s `gpu_*` columns are empty in every capture taken on
an Orin. The sampler reads sysfs instead, needs no root, and stamps ISO-8601 UTC
with milliseconds to match `system_stats.csv`, so a run joins on time.

`perf_kernel.sh` now writes to `./tmp/autosdv-perf.data`
(`AUTOSDV_PERF_DIR` overrides), and its dangling reference to a golf-cart-only
research doc is gone.

These are how phase 2 gets measured on AutoSDV hardware, which is why they came
before it.

### 6.3 Justfile module split

The root justfile went from 351 lines and 26 recipes to 9 recipes plus six
modules. New files under `just/`:

| Module | Recipes | Was |
|--------|---------|-----|
| `bag` | `record`, `play`, `download` | `bag-record`, `bag-play`, `download-data` |
| `control` | `basic`, `straight`, `circle` | `control-*` |
| `map` | `check`, `grid-from-pcd`, `grid-from-bag` | `map-*` |
| `sim` | `planning`, `logging`, `coss-park` | `launch-sim-planning`, `launch-sim-logging`, `sim-coss-park` |
| `tool` | `rviz`, `plotjuggler`, `controller`, `tui`, `zed` | `tool-*`, `launch-zed` |

`demo` stays where it was. The root keeps the daily verbs — `build`, `test`,
`clean`, `launch`, `checkout`, `setup` — plus `setup-autoware-data` and
`build-engines`, which are provisioning steps CLAUDE.md documents by name.

Four mechanics the split depends on, all of which bite silently:

- **`just --list --list-submodules`** is now the default recipe. Without
  `--list-submodules` each module collapses to one line and its recipes are
  invisible.
- **A module may not share a name with a recipe.** `mod launch` beside a
  `launch:` recipe is a hard error that kills the whole justfile. This is the
  real reason the daily verbs stay at the root.
- **`just <module>` runs the module's FIRST recipe**, so each file opens with a
  private `default` that only lists itself. Without it `just bag` would start
  recording.
- **`set working-directory := '..'`** in each module file. Otherwise recipes run
  with the cwd set to `just/` and every `./scripts/...` path breaks. Verified by
  running `just bag play` from `src/` and watching it read the repo root's
  `rosbags/`.
- A recipe calling a sibling needs the module name: `just map check ...`, not
  `just check ...`, because `just` resolves against the root justfile.

`just --list` descriptions come from the LAST comment line only, which turned
four long comment blocks into nonsense descriptions ("`# Drive. Installs
synology-dl via cargo…`"). Each now ends with a one-line description.

Renamed recipes were updated in `CLAUDE.md`, `docs/guides/simulation_testing.md`,
two design docs, two roadmaps, two `.typ` reports, `scripts/map/check_map.py`
and `map_sidecar.py` (which print these commands as hints), the map component
launch comment, and the COSS map's `autosdv_map.yaml`. `docs/reports/` and
`docs/superpowers/plans/` were deliberately left alone: they record what was run
at the time.

## Phase 7 — System packages, generic revisions only

Both packages were renamed on the golf cart side, so each cherry-pick needs the
rename undone.

### 7.1 autosdv_runtime

AutoSDV pins `0ebf59c`; the golf cart is 6 commits ahead. Three are generic:

| Commit | Change |
|--------|--------|
| `0d22f41` | template the workspace path into the systemd units at install time, instead of hardcoding it |
| `ae7c92a` | resolve the DDS profile through `env.sh` rather than a hardcoded URI |
| `40f43fe` | stop tracking colcon build artifacts (`.gitignore`) |

Skipped as golf-cart-specific: `57a6ebc` (the rename), `aacb0b4` and `2dec9b3`,
which delegate the service and the launcher to
`scripts/multi_machine/launch_unit_exec.sh` — a two-host entry point that is out
of scope by phase 0.

The generic half of `ae7c92a` is worth stating on its own: systemd user units
get none of the interactive shell's environment. direnv does not run, `~/.bashrc`
is not sourced, and `~/.local/bin` — where `play_launch` lives — is not on PATH.
Whatever AutoSDV's unit needs has to be set up explicitly inside it.

### 7.2 autosdv_system_monitor

AutoSDV pins `ff6d79b`; the golf cart is 8 commits ahead. Generic:

| Commit | Change |
|--------|--------|
| `acfeaa7` | mode availability strip, read from Autoware's diagnostic graph (with a graph fixture and a render test) |
| `36b3f81` | failing-path view — which leaf made which mode unavailable |
| `4651fa7` | fail-safe timeline, the order things happened in |
| `2b20654` | say which controls the refresh selector actually governs |
| `5bc3888` | drop the `/diagnostics_agg` row; nothing publishes it |
| `be07a3d` | **partial** — take the missing message type registrations, leave the golf cart topic names |

Skipped: `8d71947` (rename), `b68e135` and the topic-name half of `be07a3d`.

---

## Phase 8 — Cross-project fact ledgers

The golf cart keeps two documents AutoSDV has no equivalent of:

- `docs/known-config-defects.md` — things wrong in configuration rather than in
  hardware or code, which therefore stay wrong on every run until somebody edits
  a file.
- `docs/roadblocks.md` — what is blocked, and what was found and fixed.

Several entries are properties of any Autoware 1.5.0 install rather than of that
vehicle, and should be checked against AutoSDV:

- RViz asks for `rviz_plugins/MrmSummaryOverlayDisplay`, which does not exist in
  this Autoware release. It fails loudly at every startup. The install does ship
  `autoware_overlay_rviz_plugin/SignalDisplay` and
  `autoware_string_stamped_rviz_plugin/StringStampedOverlayDisplay`.
- `topic_state_monitor_initialpose3d` ships with all-zero thresholds.
- On the measured runs, **31.1% of all diagnostic reports were ERROR or STALE**,
  most of it configuration. A diagnostic graph that is permanently a third red
  trains everyone to ignore it.

One method from that document is worth adopting regardless: **read a config out
of git, not out of a submodule working tree.** One entry there was wrong for
exactly that reason. The guard is one line:

```bash
git submodule status --recursive | grep '^+'
```

**Task**: start `docs/known-config-defects.md` for AutoSDV, seeded by checking
the golf cart's entries against an AutoSDV run.

---

## Phase 9 — Pin Autoware at 1.5.0, and retire 2025.02

`versions.yaml` already says `autoware.version: "1.5.0"`, and the installed
distribution is the 1.5.0 apt localrepo. What remains is everything still naming
2025.02, which is now wrong in three different ways:

**Submodule branches.** Five forks track an `autosdv-2025.02` or `2025.02`
branch: `CalibrationTools`, `autoware_individual_params`, `autosdv_runtime`,
`autosdv_system_monitor`, `seyond_ros_driver`. `autosdv_vehicle_launch` is
already on `1.5.0`, which is the shape the rest should take. Rebase each onto its
current base and push an `autosdv-1.5.0` (or `1.5.0`) branch, then repin.
Phase 2.3 does this for the Seyond driver; phase 7 touches two more, so fold
the rename into those rather than doing it twice.

**Stale docs and scripts.** Done on 2026-09-11, ahead of the rest of the phase,
since it was doc-only:

- `scripts/version/get-version.sh` — the `autoware.version` example said
  `"2025.02"` and the `autosdv.version` example said `"1.0.0-dev"`; both now
  match `versions.yaml`.
- `docker/Dockerfile` — `ARG COMMIT_HASH` defaulted to the legacy `2025.02`
  branch, which still exists on the remote. Now `main`, the stable branch.
- Six docs carried `cd ~/repos/AutoSDV/2025.02`, from when the repo was checked
  out per Autoware release. There is no such directory now, so those were wrong
  independently of the version bump. One also said `git push origin 2025.02`,
  now `develop`.

Left alone: `README.md:79`, which links an F1EIGHTH `2025.02` tag and is
historical, and `src/calibration/CalibrationTools/calibration_tools_standalone.repos:5`
(`version: autosdv-2025.02`), which lives inside a submodule and therefore
belongs to that submodule's own branch rename below.

**`.gitmodules` tracking branches.** Done on 2026-09-11: 15 `branch` lines
added, alongside the two `particle_filter` and `range_libc` already had. Every
pin in this workspace turned out to sit on a named branch, so each fork now
declares the one it tracks. `autoware_manual_control` is the exception and stays
bare — it is `evshary`'s repository, not a fork of ours.

Worth recording because it was not obvious: `zed-ros2-wrapper`'s pin describes
as `humble-v5.0.0-225-g458c725` but lives on a branch called `ntust-workshop`.
The describe output names the upstream tag the patches sit on, not the branch
they are on, so nothing in this repo said where to commit a fix.

The branch names are still the pre-1.5.0 ones, so five of those lines change
again with the renames below. That is the point of writing them down first: a
rename with the branch recorded is a one-line edit, and a rename without it
strands the fork.

**Verification**: `grep -rn "2025\.02"` over the tree returns only the README's
historical F1EIGHTH link and anything under
`src/localization/cuda_ndt_matcher/docs/`, and
`git submodule status` shows no branch naming 2025.02.

---

## Phase 10 — Port the setup system

The golf cart's setup is finished and is a different design, not a patch on
ours. AutoSDV's `setup.sh` is the 586-line bash it replaced: a hand-written
`MENU_ITEMS` table with a cursor-driven renderer, re-forking `tput` and `cut` on
every keystroke.

What replaces it: a `curses` menu in the standard library — no venv, nothing to
bootstrap — over a declared step registry.

```
setup/setup.sh          launcher, ~40 lines, repo root carries a symlink
setup/main.py           CLI: --status --list --run --rerun --plain --profile --skip --yes
setup/<pkg>/registry.py every step, in the order they must run
setup/<pkg>/model.py    Step, Requires, Machine (host detection)
setup/<pkg>/menu.py     the curses UI
setup/<pkg>/runner.py   execution
setup/<pkg>/state.py    what ran, and what changed since
```

Three properties worth taking deliberately, because they are why the rewrite is
an improvement rather than a translation:

- **Profiles, not one flat list.** Steps declare which of `dev`, `vehicle`, `ci`
  they default to, so `./setup.sh --run --profile vehicle --yes` is a complete
  unattended install. AutoSDV's `--all` / `--minimal` pair is coarser than that.
- **Digests, not markers.** `Step.digest()` fingerprints the argv *and* the
  contents of any in-repo script it runs, so the UI can say "ran, but the script
  has changed since" — the case a marker file cannot express and which silently
  bites whenever an install script is edited. AutoSDV's marker directory has
  exactly this blind spot.
- **`run` is argv, executed without a shell**, with `_ros_bash()` as the one
  deliberate exception (ROS's setup hooks read deliberately-unset variables, so
  `set -u` has to come off around the sourcing).

### 10.1 Steps to carry over unchanged

`just`, `ros2`, `ros2-dev-tools`, Rust build support, developer tools, Python
dependencies (`play_launch`), GeographicLib + geoid data, Autoware Debian
packages, writable Autoware data directory, pre-compile TensorRT engines,
OpenCV consistency, workspace rosdep, kernel socket buffers for CycloneDDS,
multicast on loopback, u-blox udev rules, TurboVNC + VirtualGL.

### 10.2 Golf-cart steps to exclude

| Step | Why not |
|------|---------|
| CAN interfaces + LiDAR network profiles | golf cart is a CAN vehicle; AutoSDV is PCA9685 over I2C |
| OTOCAM GMSL kernel modules | no GMSL cameras |
| TIER IV camera udev + `usb_cam` | same |
| `linuxptp` (ptp4l + phc2sys) | two-host clock discipline |
| `chrony`: serve time / follow the master | same, and explicitly master/orin roles |
| `install-host-service.sh` | writes the two-host systemd drop-in |

### 10.3 AutoSDV steps to add

Absent from the golf cart registry, and needed here:

| Step | Why |
|------|-----|
| Isaac ROS Visual Localization | `pose_source:=visual` and `:=isaac` exist here; the golf cart dropped both, which is why its registry has no such step |
| Blickfeld Scanner Library | the Cube1 LiDAR driver; selecting it accepts the library's licence terms |
| ZED SDK | the ZED X Mini, and the default sensor suite uses a ZED |
| `play_launch` | golf cart folds this into "Python dependencies"; keep it as its own step, since phase 5 gives it a version floor of its own |
| `gdown` | still used here, by `scripts/2dlidar/download-sample-rosbag.sh` and `cuda_ndt_matcher/scripts/download_sample_data.sh` |
| `colcon-cargo-ros2` | without it colcon skips `cuda_ndt_matcher` and the build aborts |

### 10.4 Steps to drop while porting

- **`pacmod`** — an AutonomouStuff apt source added with `trusted=yes`, so
  signatures are not checked, and nothing under `src/` references it. The golf
  cart dropped it for the same reason; verified absent here too.
- **`nebula-driver`, `ublox-driver`** as separate apt steps — `rosdep install
  --from-paths src` already answers them. `ublox_gps` resolves to
  `ros-humble-ublox-gps`, which pulls `ublox-msgs` and `ublox-serialization`
  from one key. Nebula needs no step either: `autoware-full-1-5-0` pulls it
  through `autoware-ros-packages-1-5-0`.
- **`iceoryx`**, if any trace remains. The golf cart removed it project-wide:
  iceoryx caps publisher ports below what this stack opens, the cap is compiled
  in, and the failure is a hard abort at participant creation rather than a
  fallback to the network transport.

### 10.5 Keep

`./setup.sh status` must keep reading the machine rather than the markers. The
OpenCV, DDS and autoware-data rows check live state because a marker says a step
ran once while a reboot or a JetPack OTA can undo what it did. The digest
mechanism in 10.2 complements that; it does not replace it.

The two Autoware installer prompts stay folded in as sub-options, as they are
today — nothing may ask a question after the install starts.

**Verification**: `./setup.sh --list` on this machine, `--dry-run`/`--run
--profile ci --yes` in a container, and `./setup.sh status` against a machine
set up by the old script.

---

## Suggested order

1. **Phase 1** — one cherry-pick and one pin, largest win, no dependencies.
2. **Phase 3** — fast-forward, and phase 2 needs its launch plumbing.
3. **Phase 5** — cheap, and a working parser floor makes every later
   verification easier.
4. ~~**Phase 6** — tooling.~~ Done; 6.2's profiling scripts are what phase 2
   gets measured with.
5. **Phase 9** — do the branch renames before phase 7 repins the same forks.
6. **Phase 2** — the real work, including the driver rebase and per-point time.
7. **Phase 7** — mechanical, and inherits phase 9's branch names.
8. **Phase 10** — setup, independent of everything above; can run in parallel.
9. **Phases 4 and 8** — audits, done against a running stack.
