# Backporting the golf cart work

**Goal**: bring the parts of `2026-golf-cart` that are not golf-cart-specific
back into AutoSDV — the CUDA point cloud pipeline, the `cuda_ndt_matcher`
bump, a driver QoS fix that is currently costing 60% of the Robin-W's points,
and the tooling and system-package work that accumulated on that side.

**Status**: phases 1, 3, 4, 5, 6, 7 (fixes) and 9 done; phase 2's pipeline done, its driver work (2.3) open; 7.3, 8 and 10 open

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

## Phase 1 — Seyond driver QoS fix — DONE 2026-09-11

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

**What was done**: no cherry-pick was needed. The fork's `golfcart` branch
turned out to be our own `autosdv-2025.02` plus exactly one commit, the QoS fix
— `85f2af8`, which I had earlier mistaken for a golf-cart rename, is an
*AutoSDV* rename in a comment and was already on our branch. So the new
`autosdv-1.5.0` branch was cut from `origin/golfcart` directly, giving phase 1
and the Seyond half of phase 9 in one move, with no conflict.

Pushed to `NEWSLabNTU/seyond_ros_driver` as `autosdv-1.5.0`, then pinned, in
that order. `.gitmodules` records the branch.

**Still to verify on hardware**: record a Robin-W bag before and after and
compare point counts per frame, or read the driver's own deliver-queue
counters. Nothing here can be checked without the LiDAR.

---

## Phase 2 — CUDA point cloud pipeline — DONE 2026-09-11

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

### 2.3 Seyond driver: rebase onto v1.0.3, per-point time — DONE 2026-09-11

**The rebase.** Nine commits replayed onto `v1.0.3` (`e1ac1e5`), plus the nested
SDK fork rebased onto `c199dc7`, the SDK commit 1.0.3 pins. Pushed innermost
first: `inno-lidar-sdk` `autosdv-1.5.0` (`1c1bedc`), then `seyond_ros_driver`
`autosdv-1.5.0` (`2049a46`), then the pin here.

Two things the rebase turned up:

- The SDK patch is still needed, and was incomplete. Upstream still writes
  `cmake_minimum_required(VERSION 3.0)`, which CMake 3.27 and later refuse
  outright; 3.22 only warns, which is why this had not bitten. The root
  `CMakeLists.txt` is now covered as well as the three subdirectories.
- **Our QoS commit broke upstream's new IMU publisher.** v1.0.3 added `/iv_imu`
  sharing one `qos` object with the point cloud, and taking `SensorDataQoS()`
  for the cloud left the IMU referring to a variable that no longer existed — a
  compile error, so it surfaced immediately. The IMU keeps a reliable profile of
  its own: a small message at a low rate cannot stall the deliver thread the way
  the point cloud's writer did.

**Per-point time.** `PointXYZIRCAEDT` is now the default `POINT_TYPE`. The
layout matches `nebula_common/point_types.hpp` field for field, because Autoware
reads these clouds by offset: x0 y4 z8, intensity12, return_type13, channel14,
azimuth16, elevation20, distance24, time_stamp28, `point_step` 32.

| field | source |
|---|---|
| `time_stamp` | unsigned **nanoseconds after the header stamp**, which is the frame start — upstream's `41b1f20`. The point's absolute time is the packet start plus `ts_10us * 10 us`; the offset is that minus `frame_start_ts_`. |
| `azimuth`, `elevation` | computed here; the SDK point carries no angles |
| `distance` | the SDK's own `radius`, not recomputed from x/y/z |
| `channel` | `ring_id` when `enable_falcon_ring` is set, else `scan_id` |

The packet start is now kept unscaled in microseconds rather than differenced
through the existing seconds-since-epoch double, which is already carrying an
epoch. Getting the origin wrong does not fail loudly — it yields a plausible
cloud that the distortion corrector then shears — so the reasoning is written at
the site.

**Cost**: 32 bytes per point against 16, about 1.6 MB per frame at 50k points.
`-DPOINT_TYPE=PointXYZIRC` still builds and is one flag away.

**Verified here**: both layouts compile, and a synthetic cloud in exactly this
layout is accepted by `CudaPointcloudPreprocessorNode`, which processes it at the
full 10 Hz input rate and emits `PointXYZIRC`, with no errors. `robin-w` has
moved into `DESKEWABLE` in the kit's `pointcloud_preprocessor.launch.py`
accordingly — with the caveat that this needs a driver *built* at that pin, since
one built as `PointXYZIRC` publishes a cloud the preprocessor rejects.

**Not verified**: anything involving the sensor. The offsets are only as right as
the reasoning above until a bag off the Robin-W shows deskew straightening real
structure, which a stationary bag cannot show either.

### 2.4 Tasks — DONE 2026-09-11

**Done:**

1. The two filters live in **`github.com/NEWSLabNTU/cuda_pointcloud_filters`**,
   a standalone repository both vehicles submodule at
   `src/sensing/cuda_pointcloud_filters`, rather than as a copy per project.
   They were briefly copied in as `autosdv_cuda_preprocessor`; see "One package,
   one repository" below. Builds in 11 s, and its **12 unit tests run and pass
   on this machine's GPU** rather than skipping: the sm_120 RTX 5090 is not in
   `CMAKE_CUDA_ARCHITECTURES` and CUDA 12.3 cannot target it, but the driver
   JITs the compute_89 PTX.
2. The sensing CUDA branch in `autosdv_sensor_kit_launch`, plumbed through
   `lidar.launch.xml` and `sensing.launch.xml`.
3. The localization CUDA branch in
   `tier4_localization_launch/launch/util/util.launch.xml`, plumbed through
   `pose_twist_estimator.launch.xml`, `localization.launch.xml`, the localization
   component and the Autoware wrapper.
4. `pointcloud_backend` and `localization_pointcloud_backend` on both top-level
   launch files, defaulting to `cpu`.
5. `docs/design/cuda-pipeline-data-flow.md`, rewritten for this vehicle.

**Two findings that changed the design.**

*The concatenator cannot be used with one LiDAR.* Both the CPU and the CUDA
concatenator refuse a single input topic — `Only one topic given. Need at least
two topics to continue.` — and the duplicate-topic workaround the kit already
carried in its `use_single_lidar=false` branch was measured against a synthetic
10 Hz publisher at **1.7–2.4 Hz output**, logging `Reset the oldest collector`
every cycle and losing about 80% of frames. So the sensing chain ends at the
existing `PassThroughFilterComponent`, which also does the transform to
`base_link` that the CUDA preprocessor does not do. The cost is a device-to-host
copy there: the per-point work is on the GPU, the path is not GPU-resident.

*The AutoSDV localization chain does not go through `cuda_ndt_matcher_launch`'s
`util.launch.xml`.* AutoSDV reaches the matcher through `pose_source_package` →
`pose_estimator.launch.xml` only, and runs its own vendored
`tier4_localization_launch/util/util.launch.xml`, so the CUDA branch went there.

That let the first version of this work route *around* a problem rather than fix
it: `cuda_ndt_matcher_launch/util.launch.xml`, a file both vehicles submodule,
hardcoded `golfcart_cuda_preprocessor` and its plugin namespace. It now takes
`cuda_filters_package` and `cuda_filters_namespace` as arguments, defaulting to
`cuda_pointcloud_filters`, so the shared launch names no project.

### One package, one repository

The filters were the only pure duplicate between the two vehicles: 19 files, 14
byte-identical once the project name was normalised away, the rest differing
only by the rename. They are now
`github.com/NEWSLabNTU/cuda_pointcloud_filters`, public, submoduled by both
projects, and the package, namespace, README and licence text name no vehicle.

What is genuinely shared and already a submodule: `cuda_ndt_matcher`,
`seyond_ros_driver`, `CalibrationTools`, `autoware_individual_params`,
`gnss_locator`, `ros-nmea-reader`, `zed-ros2-wrapper`, `autoware_manual_control`.

What is duplicated and still could be unified, measured by files identical after
normalising the project name:

| Code | Files | Identical | Note |
|---|---|---|---|
| `control_test` | 16 | 10 | shared tool, drifting |
| `system_monitor` | 12 | 6 | one repo, renamed then diverged by four UI features |
| `runtime` | 24 | 4 | same origin, most diverged: unit names and env resolution now differ |
| diagnostics + profiling scripts | ~10 | 1 | copied in phase 6 |

`sensor_kit_launch`, `vehicle_launch` and the launchers are per-vehicle and
should stay separate; the launcher is 209 files with 29 identical.

**Not verified here**: nothing is launched. This machine has no full build, and
the two backends have never run together on a vehicle. Before trusting either
switch, run a COSS replay with `pointcloud_backend:=cuda` and confirm the
concatenated cloud rate matches `cpu`, then measure with `scripts/profiling/`.

## Phase 3 — cuda_ndt_matcher submodule bump — DONE 2026-09-11

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

**Done**: pinned at `db8c87b`, which is `origin/main`'s head, and the nested
`tests/rosbag_replay` submodule moved with it. Not rebuilt — this machine has
no `install/`, so the build is left to a machine that has one.

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
  Done.
- **`gnss_enabled` defaulting to `use_gnss`.** AutoSDV did hit this, and worse
  than the golf cart did: `pose_twist_estimator.launch.xml` declares
  `gnss_enabled` with an upstream default of `true`, and
  `localization.launch.xml` never forwarded it, so no caller could set it at
  all. `use_gnss` itself reached `autosdv_autoware.launch.xml` only through
  LaunchConfiguration leakage and was never declared there.

  Fixed across three files: `use_gnss` is declared and forwarded in the
  wrapper, the component derives `gnss_enabled` from it, and
  `localization.launch.xml` passes it down. Empty `use_gnss` means the sensor
  suite decides and every suite resolves it to `true`, so only an explicit
  `false` disables GNSS. Not launch-tested — no built workspace here — but the
  three files parse.

---

## Phase 4 — Sensor kit: the xacro comment trap — DONE 2026-09-11

The mechanism is real and reproduces on this stack. A comment carrying a colon
followed by a space survives xacro expansion into `robot_description`, ROS 2
launch YAML-parses parameter values, and `safe_load` fails on the whole URDF.
Reproduced with plain `ros2 launch` on Humble, on a two-line test file:

```
Failed to convert '<?xml version="1.0" ?> ... ' using yaml rules:
yaml.safe_load() failed
mapping values are not allowed here
  in "<unicode string>", line 7, column 16:
      <!-- measures: x forward -->
```

The error points at the comment, but nothing in a real run does: the stack
simply never comes up.

**Audit result.** Every `.xacro` and `.urdf` under `src/`, comments only:

- AutoSDV's own sensor kit, vehicle and param descriptions: **clean**, zero
  hits.
- `zed-ros2-wrapper`'s `zed_macro.urdf.xacro`: three hits, in one
  `Parameters:` block. **Inert**, and this was worth checking rather than
  assuming: expanding the real `zed_descr.urdf.xacro` — the file
  `zed_tf_only.launch.xml` and `zed_imu_only.launch.xml` actually pass through
  `$(command 'xacro ...')` into a parameter value — drops that comment, and the
  result parses. Comments inside a `xacro:macro` *definition* do not reach the
  output; ones in the instantiated body do. Left alone.

**Warning added** to `autosdv_sensor_kit_description/urdf/sensor_kit.xacro`,
phrased in words. The golf cart's first attempt said no `word: word` in
comments, which contains a colon-space and reproduced the bug inside the
warning; this one spells the pattern out instead, and was expanded and parsed to
prove it is safe.

**One bug found while there**: a stray `b` after the `config_dir` `xacro:arg`,
passing through expansion as character data in `robot_description`. Removed.

Pushed as `0c38d51` on the sensor kit's `main`, then pinned.

---

## Phase 5 — play_launch floor — DONE 2026-09-11, except the parity check

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

**Done**: the floor is `0.10.0` in `install-play-launch.sh`, the `setup/justfile`
status row, `setup/setup.sh`'s menu note and `setup/README.md`. The installer's
comment now records both reasons — the 0.9.0 startup governor and the 0.10.0
array-parameter fix — and warns that a plain `pip install play_launch` satisfies
the floor while still carrying play_launch issue #0028, because the fix
(`8adc52ad`) is later than the 0.10.0 release and there is no version to check
for. The machine here reads `play_launch 0.10.0 (>= 0.10.0)`.

**Left undone**: parser parity on AutoSDV's own entry points. `play_launch dump`
resolves a real launch tree, so it needs `autosdv_launch` installed, and this
machine has no `install/`. Run on a built machine:

```bash
play_launch dump launch autosdv_launch autosdv.launch.yaml -o tmp/rust.json
play_launch dump launch autosdv_launch autosdv.launch.yaml --parser python -o tmp/py.json
play_launch context tmp/rust.json --tree
```

and compare node counts, as the golf cart did for its five entry points.

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

## Phase 7 — System packages — fixes DONE 2026-09-11, features deferred

Both packages were renamed on the golf cart side, so nothing could be
cherry-picked; each commit was re-applied by hand against AutoSDV's names, and
both now live on a `1.5.0` branch (phase 9's rename, done in the same pass).

### 7.1 autosdv_runtime — done, `9fef4f2`

`0d22f41` and `ae7c92a` ported; `40f43fe` became three lines of `.gitignore`,
since nothing was actually tracked here.

The units now carry `@AUTOSDV_WORKSPACE@` and the installer fills it in from
`get_workspace_dir()`, with an unsubstituted placeholder raising rather than
becoming systemd's problem later. The hardcoded `%h/AutoSDV` matched no
checkout — this workspace is `~/repos/AutoSDV` — so `autosdv install && autosdv
start` would have failed on a path.

`ae7c92a` could not be taken literally: it routes the unit through
`scripts/env.sh` and `config/cyclonedds/<profile>.xml`, neither of which exists
here. The defect it fixes does exist, in AutoSDV's own shape, and turned out to
be worse: the launch script sourced `/opt/autoware/autoware-env`, which is not
where the Debian packages put it, so under `set -e` the unit died on its first
line. It now takes the newest `/opt/autoware/*/setup.bash` and exports the
workspace's own `cyclonedds.xml`, and `autosdv.service` no longer pins
`CYCLONEDDS_URI` to `/opt/autoware/cyclonedds.xml`.

Four more defects, all found by running `systemd-analyze verify` on the
rendered units rather than by reading them:

| Defect | Effect |
|--------|--------|
| `Environment=HOME=/home/%i` | `%i` is empty outside a template unit, so `HOME=/home/` |
| `User=%i` / `Group=%i` | fatal: "Invalid user/group name or numeric ID". A user unit runs as the user |
| `Requires=`/`After=autosdv@%i.service` | names a template unit that does not exist |
| `Documentation=github.com/AutoSDV/AutoSDV` | not the repository |

After the fixes `systemd-analyze verify` is clean on all four units apart from
the `ExecStart` path, which is absent only because this machine has no build.

### 7.2 autosdv_system_monitor — fixes done, `b4832b6`

`be07a3d`'s generic half and `5bc3888` ported. The same latent defect was here:
`config/monitor_topics.yaml` names `nmea_msgs/msg/Sentence`,
`rtcm_msgs/msg/Message` and `ublox_msgs/msg/RxmRTCM`, none of which the node's
type map had, so the loader skipped all three and the table showed NO DATA
whether or not anything published. All three packages were already
`exec_depend`s; each was verified to import on this install. The skip warning
now names the topic that will not be monitored.

`/diagnostics_agg` removed — Autoware 1.5.0 publishes `DiagGraphStruct` and
`DiagGraphStatus` through `autoware_diagnostic_graph_aggregator`, not an
aggregated `DiagnosticArray`.

Golf-cart topic names were **not** taken: AutoSDV's Velodyne genuinely publishes
`/sensing/lidar/velodyne_points`, unnamespaced, where the golf cart's sits under
`vlp32`.

### 7.3 Deferred: the four UI commits

`acfeaa7`, `36b3f81`, `4651fa7` and `2b20654` are a feature, not a fix, and do
not belong in a mechanical pass. Together they are ~2,200 lines including a
1,365-line captured graph fixture and a Node test, they build on each other
(`2b20654` only makes sense once the mode strip exists), and the mode strip
launches `rosbridge_server` from the monitor's launch file — a new runtime
dependency. AutoSDV's `monitor.html` is 499 lines against the golf cart's 1,072,
so this is a port onto a diverged template.

Worth doing, and worth scoping on its own. Two design facts to carry across when
it happens:

- The page must talk to rosbridge directly rather than through this node. The
  two graph topics **disagree on QoS** — struct is RELIABLE + TRANSIENT_LOCAL,
  status is BEST_EFFORT + VOLATILE — and one subscriber applying a single
  profile silently receives nothing on one of them.
- struct and status are joined **by array index**; `DiagNodeStatus` carries no
  path, so an off-by-one mislabels every chip while looking entirely plausible.
  The strip re-subscribes when the graph id changes for exactly this reason.

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

## Phase 9 — Pin Autoware at 1.5.0, and retire 2025.02 — DONE 2026-09-11

`versions.yaml` already says `autoware.version: "1.5.0"`, and the installed
distribution is the 1.5.0 apt localrepo. What remains is everything still naming
2025.02, which is now wrong in three different ways:

**Submodule branches.** Done. Every fork that named 2025.02 now has a branch
naming the release its patches are actually current for, pushed before the pin
was recorded:

| Submodule | Was | Now | Pin |
|-----------|-----|-----|-----|
| `seyond_ros_driver` | `autosdv-2025.02` | `autosdv-1.5.0` | `98dbcc8` (with the phase 1 QoS fix) |
| `CalibrationTools` | `2025.02` | `1.5.0` | `bc36609` |
| `autoware_individual_params` | `2025.02` | `1.5.0` | `94877fd`, unchanged |
| `autosdv_runtime` | `2025.02` | `1.5.0` | `9fef4f2` (phase 7) |
| `autosdv_system_monitor` | `2025.02` | `1.5.0` | `b4832b6` (phase 7) |

The old branches are kept; they are the record of what worked against 2025.02.
`CalibrationTools` needed one content change with the rename:
`calibration_tools_standalone.repos` pins a branch of the same fork, and still
named `autosdv-2025.02`.

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

Phases 1, 3, 4, 5, 6, 7 (fixes) and 9 are done. What is left:

1. **Phase 2** — the real work: the CUDA pipeline, the Seyond rebase onto
   upstream v1.0.3, and per-point time. Measure it with phase 6.2's profiling
   scripts.
2. **Phase 10** — the setup rewrite. Independent of everything else, so it can
   run in parallel.
3. **Phase 7.3** — the four system-monitor UI commits, scoped on their own.
4. **Phase 8** — the config-defect ledger, which needs a running stack.

Three things need a machine this one is not. The workspace here has no
`install/`, so: the cuda_ndt_matcher bump is unbuilt, the `gnss_enabled` and
`ndt_param_file` launch changes are unlaunched (they parse, nothing more), and
phase 5's parser parity check is unrun. The Seyond QoS fix needs the LiDAR.
