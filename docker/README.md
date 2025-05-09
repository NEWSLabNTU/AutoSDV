# AutoSDV Docker Environment

This directory contains Docker configuration files for building and running AutoSDV in a containerized NVIDIA Jetson Linux environment. The setup provides a consistent development and testing environment regardless of the host system.

## Overview

The Docker environment is configured to:

- Use NVIDIA L4T (Linux for Tegra) as the base image
- Include TensorRT for deep learning acceleration
- Configure necessary NVIDIA repositories and dependencies
- Clone the AutoSDV repository and check out the **exact same commit** as your local repository
- Provide a ready-to-use environment that matches your current code state

## Requirements

The build script was tested on Ubuntu 22.04 operating system.

- Docker with NVIDIA container toolkit installed. You may read
  - the [installation guide](https://docs.docker.com/engine/install/ubuntu/) to install Docker engine, and
  - the [installation guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to install NVIDIA Container Toolkit.

- `rocker` for container management with GUI and NVIDIA support. You can read the installation instructions in the [README](https://github.com/osrf/rocker?tab=readme-ov-file#installation).
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

This creates a Docker image named `autosdv` configured for ARM64 architecture, suitable for Jetson devices. The image will:

1. Use the **current commit** of your local repository
2. Clone the repository and check out that same commit inside the container
3. Build with all necessary dependencies and artifacts
4. Create two tags:
   - `autosdv:<short-hash>` (e.g., `autosdv:a05519`)
   - `autosdv:<full-hash>` (e.g., `autosdv:a0551926248c75aac9411d53...")

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

This exports the image to `autosdv-<short-hash>.tar.zstd` using zstd compression.

## Other Commands

### Cleaning Up

To remove the Docker image:

```bash
make clean
```

This removes both the short hash tag and the full hash tag of the image.

## Customization

To customize the Docker environment:

1. Modify the `Dockerfile` to add additional dependencies
2. Update version numbers in `nvidia-l4t-apt-source.list` if using a different L4T version
3. Edit the `Makefile` to adjust container runtime settings

## Important Notes

- Each Docker image is tagged with both the short commit hash and the full commit hash
- Changes to your local repository will not automatically update existing Docker images
- Use `make build` after committing changes to create a new image with the updated code
- Remember to push your commits to the remote repository before building
