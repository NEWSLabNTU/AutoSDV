# AutoSDV Development Environment Setup

A lightweight setup system using [just](https://github.com/casey/just) with checkpoint-based resume capability.

## Prerequisites

Install `just` command runner:

```bash
# Ubuntu/Debian
curl --proto '=https' --tlsv1.2 -sSf https://just.systems/install.sh | bash -s -- --to ~/.local/bin

# Or via cargo
cargo install just

# Or via apt (Ubuntu 22.04+)
sudo apt install just
```

## Quick Start

```bash
# From the repository root. Prompts for sudo once at the beginning.
./setup.sh

# If setup fails, simply re-run to resume from where it stopped
./setup.sh

# Run a specific recipe
./setup.sh status
./setup.sh ros2
```

## Choosing Components

`./setup.sh` opens a menu of optional components. Core (ROS 2, dev tools, Rust,
GeographicLib, Python deps) is always installed; everything else is a checkbox.

```
 ❯ [x] Autoware Debian packages
       ~2-3 GB. Skip to build from source instead.
   [ ]     └ let Autoware install ROS 2 Humble
   [x] Network configuration (DDS)
   [x]     └ kernel socket buffers
   [x]     └ multicast on lo (persistent)
```

`↑↓`/`jk` move, `SPACE` toggles, `a`/`n` select all or none, `ENTER` runs,
`q` quits. Indented entries are sub-options: they are ignored unless the entry
above them is selected, and shown greyed out when it is not.

The menu exists because the previous run of yes/no prompts could not be revised
once answered, and because the Autoware step went on to ask its own two
questions partway through the install. Those are the `└` entries under
Autoware now, collected up front and passed to that script as flags.

### Non-interactive

```bash
./setup.sh --all                    # everything, no questions (engines included)
./setup.sh --all --no-engines       # everything, but skip the minutes-long engine build
./setup.sh --all --no-isaac         # everything except Isaac ROS
./setup.sh --minimal                # core only
./setup.sh --dry-run                # pick components, print the selection, install nothing
./setup.sh --all --dry-run          # what --all would do
```

`--dry-run` combines with any of the others, and is the only way to exercise
the menu without running a multi-gigabyte install.

## Sudo Handling

The setup requires root privileges for many steps (apt, udev, etc.). The wrapper script `setup.sh` handles this:

1. **Password prompt once**: At the start, you'll be asked for your sudo password
2. **Keep-alive loop**: A background process refreshes sudo credentials every 50 seconds
3. **Auto-cleanup**: The keep-alive stops automatically when setup completes, fails, or is interrupted (Ctrl+C)

This means you can start `./setup.sh` and walk away - no need to babysit for password prompts.

**Note**: Always use `./setup.sh` instead of calling `just` directly to ensure proper sudo handling.

## Commands

| Command | Description |
|---------|-------------|
| `./setup.sh` | Run full setup (all steps) |
| `./setup.sh status` | Show which steps are completed |
| `./setup.sh <step>` | Run a specific step |
| `./setup.sh clean-markers` | Reset all checkpoints to force re-run |
| `./setup.sh clean-marker <name>` | Reset a specific step |

## Setup Steps

The setup runs these steps in order:

1. **ros2** - Install ROS 2 Humble
2. **ros2-dev-tools** - Install colcon, rosdep, pytest, flake8
3. **rust** - Install the Rust toolchain via rustup
4. **colcon-cargo-ros2** - Rust support for colcon (>= 0.5.1)
5. **gdown** - Install Google Drive downloader
6. **geographiclib** - Install GeographicLib tools and geoid data
7. **pacmod** - Add AutonomouStuff apt repository
8. **dev-tools** - Install git-lfs, pre-commit, Go, PlotJuggler
9. **blickfeld** - Install Blickfeld LiDAR SDK
10. **autoware-debian** - Install Autoware Debian packages
11. **autoware-data** - Build the writable Autoware model tree
    (then optionally **build-engines** - pre-compile the TensorRT engines)
12. **isaac-ros** - Isaac ROS Visual Localization (cuVSLAM + cuVGL)
13. **opencv** - Put OpenCV on one version, headers and runtime together
14. **python-deps** - Install AutoSDV Python dependencies
15. **ublox-udev** - Install u-blox GPS udev rules
16. **cyclonedds-sysctl** / **multicast-lo** - DDS network configuration
17. **ros-deps** - rosdep over the workspace

Order is not arbitrary in three places. `opencv` runs after `autoware-debian`
and `isaac-ros`, because both pull packages that depend on `libopencv-dev` and
it should correct one settled state rather than race apt. `autoware-data` runs
after `autoware-debian`, because it mirrors what that package installed. `rust`
runs before `colcon-cargo-ros2` and `python-deps`, which both want cargo on
PATH.

### The ones worth knowing about

**colcon-cargo-ros2** — `cuda_ndt_matcher` builds with `ament_cargo`. Without
this extension colcon does not process it at all: it reports the package as
"not processed", every dependent then fails looking for its `package.sh`, and
the build aborts naming that missing file rather than the missing extension.
The 0.5.1 floor matters — earlier releases import cleanly and still fail the
build.

**opencv** — JetPack ships NVIDIA's OpenCV 4.8.0 as `libopencv-dev`, which owns
`/usr/include/opencv4`, while every runtime library and every ROS deb on the
system is Ubuntu's 4.5.4. Local builds compile against one and link the other,
silently. It also costs the contrib modules, which is why `aruco` is missing.
`just opencv-check` reports the state without changing anything; the fix
refuses to run if anything is actually linked against 4.8.0.

**Network configuration (DDS)** — two halves, both required to run ROS here.
`cyclonedds-sysctl` raises `net.core.rmem_max` and the `net.ipv4.ipfrag_*`
limits (below 10 MB no `ros2` node can create a domain) and persists them to
`/etc/sysctl.d/99-cyclonedds-max.conf` — numbered 99 so it wins against the ZED
SDK's `60-zed-buffers.conf`, which sets a *lower* value. `multicast-lo`
installs a systemd unit that keeps the MULTICAST flag on `lo` across reboots;
`cyclonedds.xml` pins that interface, so without it every node dies at startup
with `selected interface "lo" is not multicast-capable`.

**autoware-data** — the packaged model tree at `/opt/autoware/*/data` is
root-owned, so TensorRT cannot write the `.engine` file it builds next to each
`.onnx`. Every engine is then discarded and rebuilt, and fails, on every
launch. This mirrors the tree into `data/autoware_data` with symlinks (171
files, under a megabyte) so engines can be cached. The top-level launch files
default `data_path` to it; override with `data_path:=` or `AUTOSDV_DATA_PATH`.

**TensorRT engines** (`just build-engines`, the sub-option under the data dir)
— Autoware compiles an `.onnx` into a `.engine` inside the *node's
constructor* the first time it runs, so skipping this does not save the work,
it just moves it into the first launch with perception down until it finishes.
Engines are specific to the TensorRT version and the GPU, so it must run on the
target board and be re-run after an Autoware or JetPack upgrade.

### Optional Steps

| Command | Description |
|---------|-------------|
| `./setup.sh download-artifacts` | Download ML model artifacts (~2GB) |
| `./setup.sh install-zed-sdk` | Install ZED camera SDK |
| `./setup.sh turbovnc-virtualgl` | TurboVNC + VirtualGL for VNC rendering |
| `./setup.sh opencv-check` | Report the OpenCV state, change nothing |
| `./setup.sh network-dds` | Both DDS network steps together |

From the repository root:

| Command | Description |
|---------|-------------|
| `just setup-autoware-data` | Rebuild the writable model tree (after an Autoware upgrade) |
| `just build-engines` | Compile the TensorRT engines ahead of the first launch |

## How Resume Works

Each completed step creates a marker file in `.markers/`. When you re-run setup:
- Completed steps are skipped (marker exists)
- Failed/incomplete steps are re-run
- You can force re-run with `just clean-marker <step>`

## Directory Structure

```
setup/
├── setup.sh              # Entry point (component menu + sudo keep-alive)
├── justfile              # Recipe definitions
├── README.md             # This file
├── .gitignore            # Ignores .markers/
├── .markers/             # Checkpoint files (auto-created)
├── scripts/              # Complex setup scripts
│   ├── install-ros2.sh
│   ├── install-ros2-dev-tools.sh
│   ├── install-colcon-cargo-ros2.sh
│   ├── install-autoware-debian.sh
│   ├── install-isaac-ros.sh
│   ├── install-opencv.sh
│   ├── install-blickfeld.sh
│   ├── install-zed-sdk.sh
│   ├── install-turbovnc-virtualgl.sh
│   ├── configure-cyclonedds-sysctl.sh
│   ├── configure-multicast-lo.sh
│   └── download-artifacts.sh
└── files/                # Static files
    ├── 99-ublox-gps.rules
    ├── 99-opencv-ubuntu.pref   # apt pin keeping OpenCV on Ubuntu's 4.5.4
    └── artifacts.yaml          # ML model download manifest

# ../setup.sh symlinks to setup/setup.sh, so it runs from the repo root.
# ../scripts/setup_autoware_data.sh builds the writable Autoware model tree.

# Root-level version configuration
versions.yaml             # Single source of truth for all versions
scripts/version/          # Version helper scripts
├── get-version.sh        # Get individual version values
└── export-versions.sh    # Export all versions as env vars
```

## Compared to Ansible

| Feature | Ansible | justfile |
|---------|---------|----------|
| Install overhead | ~100MB (Python, collections) | ~2MB (single binary) |
| Startup time | ~10-15 seconds | Instant |
| Resume from failure | Re-run all (idempotent) | Marker-based skip |
| Learning curve | High (YAML DSL) | Low (shell-like) |
| Single machine setup | Overkill | Perfect fit |

## Troubleshooting

### Check what's completed
```bash
./setup.sh status
```

Most rows read a marker file, but the OpenCV, DDS and autoware-data rows read
the machine instead. A marker says a step ran once; a reboot or a JetPack OTA
can undo what it did, and then the marker is a lie.

### Force re-run a specific step
```bash
./setup.sh clean-marker ros2
./setup.sh ros2
```

### Force re-run everything
```bash
./setup.sh clean-markers
./setup.sh
```

### View verbose output
The scripts output progress. For more detail, read the individual scripts in `scripts/`.
