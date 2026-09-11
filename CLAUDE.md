# CLAUDE.md

Guidance for Claude Code when working with this repository.

## Project Overview

AutoSDV is a software-defined autonomous vehicle platform built on ROS 2 and Autoware. Supports multiple LiDAR configurations (Robin-W, Velodyne 32C, Blickfeld Cube1) for small-scale autonomous vehicles.

## Versioning

AutoSDV uses [Semantic Versioning](https://semver.org/): `MAJOR.MINOR.PATCH[-PRERELEASE]`

### Single Source of Truth: `versions.yaml`

All version information is centralized in `versions.yaml` at the repo root:
- **AutoSDV version**: Project version and release channel (stable/development)
- **Autoware version**: Pinned Autoware base version
- **ROS**: Distribution, installation type, RMW implementation
- **NVIDIA stack**: CUDA, cuDNN, TensorRT versions (AMD64 and ARM64/Jetson)
- **JetPack/L4T**: Jetson platform versions
- **Tool versions**: clang-format, etc.
- **Package checksums**: SHA256 for verification

### Version Helper Scripts

```bash
# Get a specific version value
./scripts/version/get-version.sh autosdv.version      # Returns "0.1.0-dev"
./scripts/version/get-version.sh autoware.version     # Returns "1.5.0"
./scripts/version/get-version.sh nvidia_amd64.cuda    # Returns "12.3"

# Export all versions as environment variables
source ./scripts/version/export-versions.sh
echo $AUTOSDV_VERSION    # 0.1.0-dev
echo $AUTOWARE_VERSION   # 1.5.0
echo $CUDA_VERSION_AMD64 # 12.3
```

### Version Bumping Guidelines

| Change Type | Version Bump |
|-------------|--------------|
| Breaking changes (vehicle interface, sensor configs, launch API) | MAJOR |
| Autoware base upgrade | MAJOR |
| New sensor/feature support | MINOR |
| New launch parameters | MINOR |
| Bug fixes, parameter tuning | PATCH |
| Documentation only | PATCH |

### Branch Strategy

| Branch | Version | Channel |
|--------|---------|---------|
| `main` | `X.Y.Z` (stable) | `stable` |
| `develop` | `X.Y.Z-dev` | `development` |
| `release/X.Y` | `X.Y.Z-rc.N` | `stable` |

`main` is the stable branch; day-to-day work goes to `develop`. Feature
branches start from `develop` and merge back into it; `develop` merges into
`main` for a release.

### Branch Protection (GitHub rulesets)

Both `main` and `develop` are protected, and the rules change how you merge:

| Rule | `main` | `develop` |
|------|--------|-----------|
| Pull request required | yes | no (direct push allowed) |
| Linear history required | yes | yes |
| Force push blocked | yes | yes |
| Branch deletion blocked | yes | yes |
| Allowed merge methods | squash, rebase | — (no merge commits either way) |

**Linear history means no merge commits on either branch.** Rebase instead of
merging when you bring in upstream changes:

```bash
# update a feature branch onto develop
git checkout feature/my-thing
git fetch origin
git rebase origin/develop

# update develop itself
git checkout develop
git pull --rebase origin develop
```

Set `git config pull.rebase true` in this repo so `git pull` never creates a
merge commit by accident.

**Merging to `main`** requires a pull request (approvals are not required, but
the PR itself is). Merge it with squash or rebase — the "Create a merge commit"
button is rejected by the ruleset:

```bash
git checkout -b release/X.Y develop     # or push develop and open the PR from it
gh pr create --base main --head develop --title "Release X.Y.Z"
gh pr merge --rebase                    # or --squash; never --merge
```

Prefer `--rebase` for a release PR so each commit from `develop` keeps its own
identity on `main`, and `--squash` for a single-topic PR that should land as one
commit.

**Merging to `develop`** does not need a PR, so a rebased feature branch can be
fast-forwarded in directly:

```bash
git checkout develop
git merge --ff-only feature/my-thing
git push origin develop
```

If the fast-forward is refused, `develop` moved on — rebase the feature branch
again rather than creating a merge commit. PRs into `develop` are still welcome
for review; merge them with squash or rebase for the same reason.

## Submodule Workflow

This workspace is mostly submodules, and two rules keep them from drifting.

### Lockstep: push the submodule first, then the pin

A superproject pin is a commit hash. A hash that exists only in a local
submodule checkout is a pin nobody else can resolve: their `git submodule
update` fails, and CI fails with it. So the order is never negotiable:

```bash
# 1. commit and push inside the submodule
cd src/sensor_component/external/seyond_ros_driver
git checkout autosdv-1.5.0        # never commit on a detached HEAD
git commit -am "..."
git push origin autosdv-1.5.0

# 2. only then record the new pin in the superproject
cd -
git add src/sensor_component/external/seyond_ros_driver
git commit -m "Bump the Seyond driver (...)"
git push origin develop
```

Nested submodules repeat this innermost-first. `seyond_ros_driver` contains
`seyond_sdk`, so a change reaching into the SDK is three pushes in order: SDK,
driver, superproject.

Before committing a pin, check that nothing is uncommitted underneath — a `+`
in this output means the working tree is ahead of the recorded pin:

```bash
git submodule status --recursive | grep '^+'
```

A related habit worth keeping: **read a config out of git, not out of a
submodule working tree.** A working tree can be ahead of, behind, or unrelated
to what the pin actually builds, so a conclusion drawn from it can be about code
that no one else has.

### Two kinds of submodule, two pinning conventions

**Upstream, unforked.** Pinned to a tag or a commit that names a stable release.
There is no branch to follow — following one would silently move the pin — so
these need no `branch` line in `.gitmodules`. Example: `zed-ros2-wrapper` at
`humble-v5.0.0`.

**Our forks** (`NEWSLabNTU/*`, `jerry73204/*`). These carry patches rebased onto
an upstream stable version or `main`, and the patch series lives on a tracking
branch. Record that branch in `.gitmodules`:

```
[submodule "src/localization/external/particle_filter"]
	path = src/localization/external/particle_filter
	url = https://github.com/NEWSLabNTU/particle_filter.git
	branch = autosdv
```

Without the `branch` line the fork's own branch structure is undiscoverable from
this repo: the pin still resolves, but nothing says which branch to commit to,
which branch to rebase, or which branch a rename would strand.

Branch naming follows the Autoware release the patches are current for —
`autosdv-1.5.0`, previously `autosdv-2025.02` and `autosdv-0.45.1`. Keep the old
branches after a rebase; they are the record of what worked against that
release.

When upstream releases, the fork is **rebased** onto the new tag rather than
merged, so the patch series stays a readable list of what we changed and why.

## Essential Commands

### Build & Run
```bash
./setup.sh              # Interactive setup (ROS 2, dependencies)
./setup.sh status       # Check installation status
just build              # Build all packages
just test               # Run tests
just launch             # Launch system (web UI: http://localhost:8081)
just launch ARGS="..."  # Launch with parameters
just clean              # Remove build artifacts
just checkout           # Update git submodules
just setup-autoware-data  # Writable model tree at data/autoware_data (TensorRT needs it)
just build-engines      # Pre-compile TensorRT engines (minutes; run on the target board)
just --list             # Show all available commands
```

### Demos

Scenarios that run end to end from one command, data preparation included.
They live in a justfile module (`demo/justfile`) so the top-level list stays short.

```bash
just demo               # list the demos
just demo check        # are the prerequisites in place?
just demo run          # COSS NDT replay: fetch data, launch, seed pose, replay, report
just demo stop         # stop the stack it leaves running for inspection
just demo report       # metrics for the most recent run
```

### Tools
```bash
just tool-rviz          # Launch RViz
just tool-plotjuggler   # PlotJuggler visualization
just tool-controller    # Keyboard manual control
just tool-tui           # Drive monitor TUI (pose, speed, states)
```

### Control Testing
```bash
just control-basic      # Launch vehicle control test
just control-straight   # Run 10m straight trajectory
just control-circle     # Run circular trajectory
```

### Maps
```bash
just map-check <map_dir> [pose_source]   # validate a map dir for a method
just map-grid-from-pcd <map_dir>         # prints z distribution + suggested band, refuses to guess
just map-grid-from-pcd <map_dir> --z-min A --z-max B   # writes the grid + autosdv_map.yaml
just map-grid-from-bag <bag> <map_dir>   # accumulate scans instead, for a site with no PCD
```
`pose_source:=mcl` needs an occupancy grid rather than a PCD; `map-check`
verifies the grid is in the lanelet2 map's frame, which is the failure class
that otherwise costs days. See `docs/design/map-handling-per-localization-method.md`.

### Rosbag
```bash
just bag-record         # Record outdoor sensor topics
just bag-play           # Play most recent recording
```

### Simulation
```bash
just launch-sim-planning  # Autoware planning simulator (no sensors needed)
just launch-sim-logging   # Logging simulation (rosbag replay)
just sim-coss-park        # Full COSS Park simulation scenario
just download-data        # Download test rosbag (~2.8 GB)
```
See `docs/guides/simulation_testing.md` for the full simulation guide
(planning sim, rosbag replay, CARLA integration).

### Manual Build
```bash
source install/setup.bash
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Or use justfile
just build

# Build specific package (must include all standard flags)
colcon build --base-paths src --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select <package_name>
```

**Important**: Always use `--base-paths src` and other standard flags from `just build` when running colcon commands manually.

## Architecture

### Core Structure
```
src/
├── launcher/autosdv_launch/
│   ├── launch/                     # Main launch files
│   │   ├── autosdv.launch.yaml     # Main launch (real hardware)
│   │   ├── logging_simulation.launch.yaml  # Simulation/replay mode
│   │   └── autosdv_autoware.launch.xml     # Autoware wrapper
│   └── config/                     # Configuration files
│       ├── perception/preset/      # Perception presets
│       │   ├── lidar_only_preset.yaml
│       │   ├── camera_lidar_fusion_preset.yaml
│       │   └── minimal_preset.yaml
│       ├── localization/preset/    # Localization presets
│       │   ├── default_preset.yaml
│       │   └── eagleye_preset.yaml
│       ├── control/                # Control parameters
│       ├── localization/           # Localization parameters (NDT, EKF, etc.)
│       └── perception/             # Perception parameters
├── localization/                   # Localization packages
├── param/autoware_individual_params/  # Sensor configs
├── sensor_kit/autosdv_sensor_kit_launch/
├── vehicle/autosdv_vehicle_launch/
└── sensor_component/external/      # Sensor driver submodules
data/
├── COSS-map-planning/              # Default map
├── models/                         # ML models (YOLOX, CenterPoint)
└── zed-sdk/                        # ZED calibration
scripts/
└── leodrive-bus-launch/            # Leo Drive Bus-ODD dataset tools (submodule)
```

### Key Files
- Main launch: `src/launcher/autosdv_launch/launch/autosdv.launch.yaml`
- Sensor calibration: `src/param/.../autosdv_sensor_kit/sensor_kit_calibration.yaml`
- Preset files: `src/launcher/autosdv_launch/config/{perception,localization}/preset/`
- Web UI: http://localhost:8081 (via play_launch)

### Isaac Visual Localization

Camera-only localization using NVIDIA Isaac ROS (cuVGL + cuVSLAM). Eliminates need for LiDAR NDT.
Supported on both x86_64 (Ampere+ GPU) and ARM64 (Jetson). Install via `./setup.sh isaac-ros`.

**Architecture:**
```
pose_source:=visual
       │
       ├── cuVGL (Global Localization)
       │   └── Initial pose from visual map keyframes
       │   └── Publishes: /visual_localization/.../pose
       │
       ├── cuVSLAM (Visual Odometry)
       │   └── Continuous tracking via stereo camera + IMU
       │   └── Publishes: /localization/pose_estimator/pose_with_covariance
       │
       └── Pose Initializer Bridge
           └── Calls /localization/initialize on first cuVGL pose
```

**Packages:**
```
src/localization/autoware_isaac_localization/  # Standalone repo (github.com/NEWSLabNTU/autoware_isaac_localization)
├── autoware_isaac_localization_launch/        # cuVSLAM + cuVGL launch files
│   ├── launch/visual_localization.launch.xml  # Entry point for pose_source:=visual
│   ├── launch/isaac_slam.launch.py            # cuVSLAM wrapper
│   └── launch/visual_global_localization.launch.py
└── autoware_isaac_pose_bridge/                # cuVGL → Autoware pose initializer bridge
```

**Usage:**
```bash
# Full visual localization (requires visual map)
just launch pose_source:=visual visual_map_dir:=/path/to/visual_map

# Visual odometry only (no global init, manual pose required)
just launch pose_source:=isaac
```

**Creating Visual Maps:**
```bash
# 1. Record rosbag with ZED stereo + IMU
./scripts/visual-map/record.sh ./data/visual_maps/my_location

# 2. Create map (generates cuvgl_map/, cuvslam_map/, occupancy_map/)
./scripts/visual-map/create-map.sh ./data/visual_maps/my_location_recording
```

**Roadmap:** See `docs/roadmaps/visual_global_localization.md`

### 2-D MCL Localization (`pose_source:=mcl`)

Localizes a single-plane LaserScan against a 2-D occupancy grid instead of a PCD.
Measured on the Autoware sample site against NDT ground truth, five seeds:
**mean 0.789 m, p95 2.075 m, mean |yaw| 0.0159 rad** with a 3-ring scan source.

**The scan contract.** MCL consumes ONE `sensor_msgs/LaserScan` in any frame TF
connects to `base_link`, and holds no scan geometry of its own. The sensor kit
owns scan production, because which physical plane to use depends on the sensor
and its mounting:

```bash
# the kit publishes the scan (production)
just launch pose_source:=mcl map_path:=data/my_site

# MCL synthesises one from a 3-D cloud (test scaffolding only)
just launch-sim-logging ARGS="pose_source:=mcl scan_source:=test_pointcloud"
```

`mcl_scan_normalizer` resolves the mounting offset itself: `particle_filter`
treats the scan as originating at the particle pose, so a laser frame 0.5 m
forward of `base_link` would otherwise bias every range by 0.5 m.

**Kit-side scan production** from a 3-D LiDAR, in `autosdv_sensor_kit_launch`:

```bash
just launch publish_scan:=true      # plus scan_ring, or ring_min/ring_max
```

Use a **small ring group, not a single ring**. Measured comparison
(`docs/reports/2dlidar-scan-source-comparison.md`):

| scan source | mean | seed spread | gate | mean \|yaw\| |
|---|---|---|---|---|
| slab, 0.30 m band | 0.821 m | 0.072 | 5/5 | 0.0339 rad |
| 1 ring | 0.992 m | 0.317 | 3/5 | 0.0321 rad |
| **3 rings (70-72)** | **0.789 m** | **0.037** | **5/5** | **0.0159 rad** |

A single ring is geometrically a perfect plane but too sparse on a 128-ring
spinner. Three adjacent VLS128 channels span 0.22 deg (0.23 m at 60 m), which is
*tighter* than the slab they beat, so this is not a fidelity trade.

**Spinning LiDARs only.** Ring extraction assumes constant-elevation rings, so it
applies to `vlp32c` and not to the kit's solid-state sensors: Robin-W and Cube1
have restricted fields of view and a channel index that is not a fixed
elevation, so no ring is a horizontal plane. A narrow FOV also constrains MCL
poorly against a 360 deg grid. Use those sensors on the 3-D NDT path, or with a
native 2-D LiDAR alongside.

**The ring is sensor-specific and must be measured**, not copied -- 0.11 deg
channel spacing is a property of that VLS128:

```bash
python3 scripts/sensor/inspect_rings.py <bag> --topic <cloud> --height <mounting_h>
```

It reports per-channel elevation, names the horizontal ring, and warns when a
ring points too far up to meet the ground (a low vehicle can otherwise be
configured with a ring aimed at the sky).

**Map:** needs `occupancy_grid.yaml` + `.pgm` rather than a PCD; build with
`just map-grid-from-pcd` or `map-grid-from-bag`, validate with `just map-check`.

Design: `docs/design/mcl-user-setup-ux.md`. Diagnostics: the normaliser reports a
missing scan, a missing TF, an all-non-finite scan, and an out-of-plane mount --
every scan-side failure in this project's history was previously silent.

### CUDA NDT Localization

CUDA-accelerated NDT scan matching for faster localization on NVIDIA GPUs. This package is **maintained by AutoSDV** (not upstream Autoware).

**Package location:**
```
src/localization/cuda_ndt_matcher/  # AutoSDV-maintained, can be modified directly
├── cuda_ndt_matcher/               # Core CUDA NDT implementation
└── cuda_ndt_matcher_launch/        # Launch files and config
```

**Usage:**
```bash
# Use CUDA NDT instead of standard NDT
just launch pose_source:=cuda_ndt

# In logging simulation
just launch-sim-logging pose_source:=cuda_ndt
```

**Performance:** 1.3-1.6x faster than standard NDT, 57% less CPU usage on Jetson platforms.

**Development notes:**
- This package can be freely modified for AutoSDV-specific optimizations
- Uses same input/output interfaces as standard Autoware NDT
- Config files: `src/launcher/autosdv_launch/config/localization/ndt_scan_matcher/`

### Leo Drive Bus-ODD Dataset

The `scripts/leodrive-bus-launch` submodule provides tools for the [Leo Drive Bus-ODD dataset](https://autowarefoundation.github.io/autoware-documentation/main/datasets/) - an Autoware dataset with camera streams for testing visual localization.

**Sensors in dataset:**
| Sensor | Model | Quantity |
|--------|-------|----------|
| LiDAR | Velodyne VLP16 | 1 (front) |
| LiDAR | Velodyne VLP32C | 2 (left, right) |
| Camera | Lucid Vision Triton 5.4MP | 3 |
| GNSS/INS | Applanix POS LV 120 | 1 |

**Usage:**
```bash
cd scripts/leodrive-bus-launch

# Full setup (download ~10.9GB + migrate to Autoware 1.5.0)
just setup

# Or step by step:
just setup-python    # Install rosbags Python package
just build           # Build ROS packages (applanix_msgs, sensor_kit, vehicle)
just download        # Download dataset
just migrate-all     # Migrate rosbags from autoware_auto_* to autoware_* msgs

# Play a migrated rosbag
just play data/all-sensors-bag1_migrated
```

**Packages included:**
- `leodrive_bus_sensor_kit_launch` - Sensor kit configuration
- `leodrive_bus_vehicle_launch` - Vehicle description (Isuzu bus)
- `applanix_msgs` - Applanix GNSS/INS message definitions

## Development

### Temporary Files
Write temp files to `./tmp/` (gitignored). Do NOT use system `/tmp/`.

### Build Notes
- With `--symlink-install`, edits to yaml/xml/py apply immediately (no rebuild needed)
- New files require rebuild to create symlinks
- First launch compiles TensorRT models (10-30 min) unless `just build-engines`
  was run first

### Autoware model data (`data_path`)

Autoware writes each compiled `.engine` next to the `.onnx` it built from. The
Debian package's tree at `/opt/autoware/1.5.0/data` is root-owned, so that write
fails, the engine is discarded, and the same models rebuild — and fail — on every
launch. `just setup-autoware-data` mirrors the tree into `data/autoware_data`
with symlinks (171 files, under a megabyte), and the top-level launch files
default `data_path` to it:

```yaml
default: "$(env AUTOSDV_DATA_PATH ./data/autoware_data)"
```

Override per launch with `data_path:=`, or globally with `AUTOSDV_DATA_PATH`.
Re-run after an Autoware upgrade — engines are tied to the TensorRT version and
the GPU, so they cannot be baked into an image built elsewhere.

### ROS 2 Launch Testing

**IMPORTANT**: When testing launch files, use `play_launch` instead of `ros2 launch`:

```bash
# PREFERRED: play_launch supports multi-stage kill (SIGINT → SIGTERM → SIGKILL)
play_launch launch autosdv_launch logging_simulation.launch.yaml

# If you must use ros2 launch directly, kill by process group (PGID) to avoid orphans:
ros2 launch autosdv_launch logging_simulation.launch.yaml &
LAUNCH_PID=$!
# ... do testing ...
kill -- -$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')  # Kill entire process group
```

**Killing play_launch by PGID** (for scripts/background processes):
```bash
# Start play_launch and capture PGID
setsid bash -c "play_launch launch autosdv_launch logging_simulation.launch.yaml" &
sleep 2
PLAY_PID=$(pgrep -f "play_launch.*logging_simulation" | head -1)
PGID=$(ps -o pgid= -p $PLAY_PID | tr -d " ")
echo $PGID > /tmp/sim_pgid.txt

# Later, kill by PGID (kills entire process group including child nodes)
kill -- -$(cat /tmp/sim_pgid.txt)

# Or directly if you have the PGID
kill -- -$PGID
```

**Why this matters**: Killing `ros2 launch` or `play_launch` with SIGKILL (`kill -9`) by PID only kills the parent process, leaving child processes (component_containers, nodes) as orphans. These orphan nodes continue running and consume resources. Always kill by PGID to terminate the entire process group.

**Cleaning up orphans** (if they occur):
```bash
# List orphan nodes
ros2 node list

# Kill all ROS-related processes
ps aux | grep -E "ros|component_container|autoware" | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

### Python Packages
Standard ROS 2 conventions: setup.py/setup.cfg, test files for copyright/flake8/pep257.

### Setup Script Architecture

The setup system (`setup/`) uses a two-layer design:

1. **`setup.sh`** - Wrapper that collects every choice upfront, in one
   navigable checkbox menu, before any installation begins
2. **`justfile`** - Recipe definitions that perform actual installations

```bash
./setup.sh                  # component menu, then install
./setup.sh --all            # everything, no questions
./setup.sh --minimal        # core only
./setup.sh --dry-run        # print the selection, install nothing (combines with --all/--minimal)
./setup.sh status           # what is installed
./setup.sh <recipe>         # one recipe, e.g. opencv, network-dds, ros2
```

**Adding new optional components:**

1. Add installation script to `setup/scripts/install-<name>.sh`
2. Add recipe to `setup/justfile`:
   ```just
   # Direct recipe (for manual invocation)
   my-component: _init
       @just _run my-component "{{scripts_dir}}/install-my-component.sh"

   # Conditional recipe (for interactive setup)
   _setup-my-component:
       #!/usr/bin/env bash
       if [[ "${INSTALL_MY_COMPONENT:-n}" == "y" ]]; then
           just my-component
       else
           printf "{{yellow}}⊘{{nc}} my-component skipped (user choice)\n"
       fi
   ```
3. Add `_setup-my-component` to the `setup:` recipe chain
4. Add one row to `MENU_ITEMS` in `setup.sh` — `key|default|indent|label|note`:
   ```bash
   "MY_COMPONENT|n|0|My Component|What it costs, and what breaks without it."
   ```
   `indent=1` makes it a sub-option of the row above: shown indented, greyed
   out when the parent is off, and forced to `n` in that case.
5. Export it in `export_choices()`, and add a row to the justfile `status` recipe

**Key pattern:** the menu is one table, choices are exported as env vars, and
justfile conditionals execute based on those vars. Nothing asks a question
after the install starts — the Autoware installer's own two prompts are folded
in as the `AUTOWARE_PREREQ_*` sub-options and passed to it as flags.

**Status reads the machine, not just markers.** The OpenCV, DDS and
autoware-data rows check the live state, because a marker says a step ran once
while a reboot or a JetPack OTA can undo what it did.

### Preset System

AutoSDV uses a **preset system** (following Autoware's pattern) to manage component-level configurations. Presets group related parameters for common use cases.

#### How Presets Work

**Preset files** are YAML launch files that define launch arguments:

```yaml
# config/perception/preset/lidar_only_preset.yaml
launch:
  - arg:
      name: perception_mode
      default: "lidar"
  - arg:
      name: use_traffic_light_recognition
      default: "false"
  # ... more args
```

**Main launch file** includes presets:

```yaml
# autosdv.launch.yaml
- arg:
    name: perception_preset
    default: "lidar_only"

- include:
    file: "$(find-pkg-share autosdv_launch)/config/perception/preset/$(var perception_preset)_preset.yaml"
```

**Benefits**:
- ✅ Select presets for convenience: `perception_preset:=camera_lidar_fusion`
- ✅ Override individual parameters for experimentation: `use_traffic_light_recognition:=true`
- ✅ Easy to extend: Add new preset file without modifying launch files

#### Creating Custom Presets

1. Copy existing preset: `cp lidar_only_preset.yaml custom_preset.yaml`
2. Modify parameter defaults in the new file
3. Use with: `just launch perception_preset:=custom`

**Note**: Preset files must use `<name>_preset.yaml` naming convention.

#### Available Presets

**Perception** (`config/perception/preset/`):
- `lidar_only` - LiDAR only, no camera features (default)
- `camera_lidar_fusion` - Camera + LiDAR with traffic light recognition
- `minimal` - Minimal features for development/debugging

**Localization** (`config/localization/preset/`):
- `default` - Gyro odometry twist estimation (default)
- `eagleye` - GNSS-based odometry (requires GNSS)

See `config/{perception,localization}/preset/README.md` for detailed documentation.

## Quick Reference

### Common Launch Parameters

#### Preset-Based Configuration (Recommended)
```bash
# Perception presets (controls perception mode and features)
perception_preset:=lidar_only           # Default: LiDAR only, no camera features
perception_preset:=camera_lidar_fusion  # Camera + LiDAR with traffic light recognition
perception_preset:=minimal              # Minimal features for development

# Localization presets (controls twist estimation)
localization_preset:=default            # Default: gyro_odom
localization_preset:=eagleye            # GNSS-based odometry (requires GNSS)

# Example: Use camera-lidar fusion
just launch perception_preset:=camera_lidar_fusion sensor_suite:=robin_zed
```

#### Sensor Configuration
```bash
# Sensor suites (predefined combinations)
sensor_suite:=robin_zed          # Robin-W + ZED + ZED IMU
sensor_suite:=vlp32c_zed         # Velodyne + ZED + ZED IMU
sensor_suite:=vlp32c_zed_imu     # Velodyne + ZED + ZED IMU + MPU9250

# Individual sensor overrides
lidar_model:=robin-w|vlp32c|cube1
camera_model:=zedxm|usb|none
imu_source:=mpu9250|zed
gnss_receiver:=garmin|ublox|septentrio
```

#### Localization (pose_source)
```bash
# pose_source options:
pose_source:=cuda_ndt  # Default: CUDA-accelerated NDT (1.3-1.6x faster, 57% less CPU on Jetson)
pose_source:=ndt       # Autoware NDT (OpenMP CPU, fallback)
# pose_source_package defaults to "auto" and is derived from pose_source
# (cuda_ndt -> cuda_ndt_matcher_launch, otherwise built-in NDT). Set it
# explicitly only to plug in a third-party estimator. See
# docs/design/localization-method-switching.md.
pose_source:=mcl       # 2-D Monte-Carlo localization against an occupancy grid
pose_source:=isaac     # cuVSLAM visual odometry only (relative tracking, manual init)
pose_source:=visual    # cuVGL + cuVSLAM (camera-only, auto init from visual map)

# For visual localization, specify map directory:
visual_map_dir:=/path/to/visual_map  # Contains cuvgl_map/, cuvslam_map/
```

#### System Features
```bash
# Localization
use_gnss:=false                  # Indoor operation (no GNSS)
use_ntrip:=true                  # RTK positioning (ublox only)
use_mapless_mode:=true           # Indoor operation without localization

# Perception
enable_zed_object_detection:=true  # ZED camera object detection
launch_perception:=false           # Disable entire perception module

# Advanced: Override preset-defined parameters
perception_mode:=lidar                      # Override preset perception mode
use_traffic_light_recognition:=true        # Override preset setting
use_detection_by_tracker:=false            # Override preset setting
use_image_segmentation_based_filter:=false # Override preset setting
use_pointcloud_map:=true                   # Override preset setting
twist_source:=gyro_odom|eagleye            # Override preset twist source
```

### Vehicle Interface (Quick Ref)

**Motor PWM** (PCA9685 I2C, channel 0):
- Range: 280-460, Init: 370 (neutral), Brake: 340
- Forward: 371-460, Reverse: 280-369
- Multi-mode controller: Emergency Brake, Full Stop, Deadband Hold, Active Control (PID)

**Steering PWM** (PCA9685 I2C, channel 1):
- Range: 350-450, Init: 400 (center)
- Max angle: 0.349 rad ≈ 20°
- Dual-mode controller: Fallback (v<0.3m/s), Normal (yaw rate feedback)

**Velocity Sensing**:
- Hall effect sensor (KY-003) on GPIO
- Parameters: `params/velocity_report.yaml`

**Actuator Parameters**: `src/vehicle/autosdv_vehicle_launch/autosdv_vehicle_interface/params/actuator.yaml`

## Documentation

### Main Documentation Book (MkDocs)
- **Framework**: MkDocs with Material theme (following Autoware conventions)
- **Setup**: `cd book && just setup` (installs dependencies)
- **Build**: `cd book && just build` (builds to `site/`)
- **Serve**: `cd book && just serve` (http://localhost:3000)
- **Source**: `book/src/` (Markdown files)
- **Config**: `book/mkdocs.yml`

**Features**:
- ✅ Material Design theme
- ✅ Mermaid diagram support
- ✅ Multi-language (English + 繁體中文)
- ✅ Math rendering (MathJax)
- ✅ Search, dark mode, mobile responsive

**Key Guides** (in book):
- **Sensor Integration**: `book/src/guides/sensor-integration/`
  - Simple usage guide, Robin-W walkthrough, sensor-specific details
- **Vehicle Control**: `book/src/guides/vehicle-control/`
  - Overview, hardware, control details, tuning & testing
  - Multi-mode controllers, PCA9685 I2C, hall effect sensor, PID tuning

### Legacy Guides (docs/)
| Guide | Description |
|-------|-------------|
| [docs/guides/sensor_configuration.md](docs/guides/sensor_configuration.md) | Sensor suites, NTRIP/RTK, localization |
| [docs/guides/ndt-tuning.md](docs/guides/ndt-tuning.md) | NDT localization tuning: process, pitfalls, checklist |
| [docs/reports/localization-open-questions.md](docs/reports/localization-open-questions.md) | Known-but-unfixed localization items, with how to settle each |
| [docs/guides/vehicle_calibration.md](docs/guides/vehicle_calibration.md) | PWM control, PID tuning, testing tools |
| [docs/guides/zed_camera.md](docs/guides/zed_camera.md) | ZED setup, troubleshooting |
| [docs/guides/lidar_integration.md](docs/guides/lidar_integration.md) | Robin-W, Velodyne, TensorRT |
| [docs/guides/control_testing.md](docs/guides/control_testing.md) | Control system testing procedures |
| [docs/guides/mrm_configuration.md](docs/guides/mrm_configuration.md) | MRM (emergency stop) configuration |
| [docs/guides/isaac_vslam_testing.md](docs/guides/isaac_vslam_testing.md) | Isaac SLAM testing |
| [docs/design/isaac_vslam_integration.md](docs/design/isaac_vslam_integration.md) | Isaac SLAM architecture |
| [docs/research/localization/ndt_parameter_tuning_coss_map.md](docs/research/localization/ndt_parameter_tuning_coss_map.md) | NDT tuning research |

## Known Issues

- **Steering reversed**: Left/right inverted in manual control
- **No steering feedback**: the vehicle has no steering angle sensor, so
  `/vehicle/status/steering_status` republishes the *command* (see
  `docs/reports/steering-status-has-no-feedback.md`). MPC consumes it as its
  controller state, so the lateral loop is closed on its own output. Reads 0
  in any bag recorded while disengaged.
- **Network monitor errors**: AWS Greengrass socket errors (non-critical, ignore)
- **ZED in VNC**: Requires TurboVNC with VirtualGL for hardware acceleration
- **Isaac ROS GXF libraries**: If `pose_source:=visual` or `pose_source:=isaac` fails with "libgxf_*.so not found", the GXF library paths are not in `LD_LIBRARY_PATH`. Re-source the setup files:
  ```bash
  source /opt/ros/humble/setup.bash
  source /opt/autoware/1.5.0/setup.bash
  source install/setup.bash
  ```
  GXF libraries are located at `/opt/ros/humble/share/*/gxf/lib/` and should be added by Isaac ROS environment hooks.

## Important Notes

- **Autoware 1.5.0**: Installed at `/opt/autoware/1.5.0/` via the setup script (autoware-localrepo)
- Source Autoware environment: `source /opt/autoware/1.5.0/setup.bash` (includes ROS 2)
- Source ROS only: `source /opt/ros/humble/setup.bash`
- Requires ROS 2 Humble, Ubuntu, NVIDIA GPU
- Uses colcon (not catkin)
- Logs: `play_log/latest/`
- Stop system: Ctrl+C
