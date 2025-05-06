# AutoSDV Docker Environment

This directory contains Docker configuration files for building and running AutoSDV in a containerized NVIDIA Jetson Linux environment. The setup provides a consistent development and testing environment regardless of the host system.

## Overview

The Docker environment is configured to:

- Use NVIDIA L4T (Linux for Tegra) as the base image
- Include TensorRT for deep learning acceleration
- Configure necessary NVIDIA repositories and dependencies
- Set up a complete environment for AutoSDV development and deployment

## Requirements

It build script was tested on Ubuntu 22.04 operating system.

- Docker with NVIDIA container toolkit installed. You may read
  - the [installation guide](https://docs.docker.com/engine/install/ubuntu/) to install Docker engine, and
  - the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to install NVIDIA Container Toolkit.

- `rocker` for container management with GUI and NVIDIA support. You can read the installatoin instructions in the [README](https://github.com/osrf/rocker?tab=readme-ov-file#installation).
- QEMU for ARM64 emulation (if building on x86_64).

## Usage

### Initial Setup

Before building containers for the first time, run the bootstrap command to set up cross-architecture support:

```bash
make bootstrap
```

This installs required dependencies like QEMU and configures Docker to handle ARM64 images.

### Building the Image

Build the AutoSDV Docker image with:

```bash
make build
```

This creates a Docker image named `autosdv` configured for ARM64 architecture, suitable for Jetson devices.

### Running the Container

Launch an interactive shell in the container with:

```bash
make run
```

### Saving the Image

To save the built Docker image as a compressed file for transfer to other systems:

```bash
make save
```

This exports the image to `autosdv.tar.zstd` using zstd compression.

## Customization

To customize the Docker environment:

1. Modify the `Dockerfile` to add additional dependencies
2. Update version numbers in `nvidia-l4t-apt-source.list` if using a different L4T version
3. Edit the `Makefile` to adjust container runtime settings

## Integration with AutoSDV

Once inside the container, you can build and run AutoSDV using the following process inside the container:

```bash
# Build AutoSDV
cd /path/to/mounted/autosdv
make setup
make build

# Run AutoSDV
source install/setup.bash
ros2 launch autosdv_launch autosdv.launch.yaml
```
