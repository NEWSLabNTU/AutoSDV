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
cd scripts/setup

# Run full setup (will prompt for sudo password once at the beginning)
./setup.sh

# If setup fails, simply re-run to resume from where it stopped
./setup.sh

# Run specific recipe
./setup.sh status
./setup.sh ros2
```

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
3. **gdown** - Install Google Drive downloader
4. **geographiclib** - Install GeographicLib tools and geoid data
5. **pacmod** - Add AutonomouStuff apt repository
6. **dev-tools** - Install git-lfs, pre-commit, Go, PlotJuggler
7. **blickfeld** - Install Blickfeld LiDAR SDK
8. **autoware-debian** - Install Autoware Debian packages
9. **python-deps** - Install AutoSDV Python dependencies
10. **ublox-udev** - Install u-blox GPS udev rules

### Optional Steps

| Command | Description |
|---------|-------------|
| `./setup.sh download-artifacts` | Download ML model artifacts (~2GB) |
| `./setup.sh install-zed-sdk` | Install ZED camera SDK |

## How Resume Works

Each completed step creates a marker file in `.markers/`. When you re-run setup:
- Completed steps are skipped (marker exists)
- Failed/incomplete steps are re-run
- You can force re-run with `just clean-marker <step>`

## Directory Structure

```
scripts/setup/
├── setup.sh              # Entry point (handles sudo keep-alive)
├── justfile              # Recipe definitions
├── config.env            # Configuration variables
├── README.md             # This file
├── .gitignore            # Ignores .markers/
├── .markers/             # Checkpoint files (auto-created)
├── scripts/              # Complex setup scripts
│   ├── install-ros2.sh
│   ├── install-ros2-dev-tools.sh
│   ├── install-autoware-debian.sh
│   ├── install-zed-sdk.sh
│   └── download-artifacts.sh
└── files/                # Static files
    ├── 99-ublox-gps.rules
    └── artifacts.yaml    # ML model download manifest
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
