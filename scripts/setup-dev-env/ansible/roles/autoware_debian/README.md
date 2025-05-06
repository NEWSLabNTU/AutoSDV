# Role: autoware_debian

This role installs Autoware from NEWSLab NTU Debian packages version 2025.02.

## Inputs

| Variable                | Description                         | Default    |
|-------------------------|-------------------------------------|------------|
| `autoware_version`      | Autoware Debian package version     | `2025.2-1` |

## Supported Platforms

- Ubuntu 22.04 (x86_64)
- Generic ARM64 platforms running Ubuntu 22.04
- NVIDIA Jetson with JetPack 6.0 (L4T 36.3)

## What This Role Does

1. Detects the system architecture and platform specifics:
   - x86_64 (amd64) systems
   - ARM64 (aarch64) systems
   - Specifically identifies Jetson Linux 36.3 (JetPack 6.0) systems by checking `/etc/nv_tegra_release`

2. Downloads the appropriate Autoware local repository Debian package:
   - For x86_64: Uses the amd64 package
   - For Jetson with L4T 36.3: Uses the jetpack6.0 specific package
   - For other ARM64 systems: Uses the standard arm64 package

3. Installs the local repository, updates apt, installs Autoware, and configures the environment.

## Manual Installation

### For x86_64 (amd64):

```bash
wget -O autoware-localrepo.deb https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_amd64.deb
sudo dpkg -i autoware-localrepo.deb
```

### For Jetson with JetPack 6.0 (L4T 36.3):

```bash
wget -O autoware-localrepo.deb https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_jetpack6.0.deb
sudo dpkg -i autoware-localrepo.deb
```

### For other ARM64 platforms:

```bash
wget -O autoware-localrepo.deb https://github.com/NEWSLabNTU/autoware/releases/download/rosdebian%2F2025.02-1/autoware-localrepo_2025.2-1_arm64.deb
sudo dpkg -i autoware-localrepo.deb
```

### Common steps after installing the repository:

```bash
# Update apt repositories
sudo apt update

# Install Autoware
sudo apt install autoware-full

# Run Autoware setup
sudo autoware-setup

# Add to environment
echo "source /opt/autoware/autoware-env" >> ~/.bashrc
source ~/.bashrc
```

## Usage After Installation

To launch Autoware:

```bash
source /opt/autoware/autoware-env
ros2 launch autoware_launch planning_simulator.launch.xml \
    map_path=$HOME/autoware_map/sample-map-planning \
    vehicle_model=sample_vehicle \
    sensor_model=sample_sensor_kit
```

## Video Tutorial

For further information, you can watch the [Autoware installation tutorial video](https://youtu.be/2Ti5owIlKGg).