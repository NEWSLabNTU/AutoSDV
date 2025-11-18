FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
# Clean existing apt cache before updating
RUN rm -rf /var/lib/apt/lists/* && apt-get clean

# Accept commit hash and repository as build args
ARG COMMIT_HASH=2025.11-containerization
ARG REPO_URL=https://github.com/misuhsieh001/AutoSDV-containerization.git

# Update system packages
RUN apt update
RUN apt upgrade -y

# Configure the locale and timezone
RUN apt install -y locales
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen
RUN locale-gen
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Enable 'universe' repository
RUN apt install -y software-properties-common
RUN apt install -y apt-transport-https
RUN add-apt-repository universe

# Install APT packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
      gnupg2 curl ca-certificates cmake build-essential \
      python3-venv python3-pip sudo openssh-client openssh-server \
      git make libopencv-dev python3-opencv && \
    rm -rf /var/lib/apt/lists/*

# Remove cmake
# RUN rm -f /usr/local/bin/cmake /usr/local/bin/cpack /usr/local/bin/ctest
RUN rm -f /usr/local/bin/cpack /usr/local/bin/ctest || true

# Install NVIDIA l4t compiler
RUN apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc
COPY nvidia-l4t-apt-source.list /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
RUN mkdir -p /opt/nvidia/l4t-packages
RUN touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall
RUN apt update
RUN apt install -o DPkg::Options::="--force-confold" -y nvidia-l4t-dla-compiler

# Install dependencies for ZED SDK
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    lsb-release \
    zstd \
    udev \
    libusb-1.0-0 \
    libhidapi-libusb0 \
    libhidapi-dev

# Download ZED SDK 5.0 installer for JetPack 6.0 (L4T 36.3) with CUDA 12.2
# Using SDK 5.0 to match zed-ros2-wrapper version humble-v5.0.0
RUN wget --no-check-certificate -O ZED_SDK_Tegra_L4T36.3_v5.0.0.zstd.run \
    https://download.stereolabs.com/zedsdk/5.0/l4t36.3/jetsons && \
    echo "Downloaded ZED SDK 5.0 installer"

# Install ZED SDK with proper flags
# - silent: non-interactive installation
# - skip_cuda: skip CUDA installation (already installed)
# - skip_od_module: skip object detection module (optional, reduces size)
RUN chmod +x ZED_SDK_Tegra_L4T36.3_v5.0.0.zstd.run && \
    ./ZED_SDK_Tegra_L4T36.3_v5.0.0.zstd.run -- silent skip_cuda && \
    echo "ZED SDK 5.0 installation completed" && \
    rm -f ZED_SDK_Tegra_L4T36.3_v5.0.0.zstd.run

# Verify ZED SDK installation
RUN if [ -d /usr/local/zed ]; then \
        echo "ZED SDK successfully installed at /usr/local/zed"; \
        ls -la /usr/local/zed/; \
        find /usr/local/zed -name "*.cmake" | head -5; \
    else \
        echo "WARNING: ZED SDK installation may have failed - /usr/local/zed not found"; \
    fi

# Set environment variables for ZED SDK
ENV ZED_SDK_ROOT_DIR=/usr/local/zed
ENV CMAKE_PREFIX_PATH="/usr/local/zed:/opt/ros/${ROS_DISTRO}"
ENV LD_LIBRARY_PATH="/usr/local/zed/lib:${LD_LIBRARY_PATH}"

# Clone AutoSDV repository and checkout specific commit/branch
RUN git clone --recursive --branch ${COMMIT_HASH} ${REPO_URL} AutoSDV || \
    (git clone --recursive ${REPO_URL} AutoSDV && \
     cd AutoSDV && \
     git fetch --all && \
     git checkout ${COMMIT_HASH})

# Initialize nested submodules (specifically for seyond_ros_driver's inno-lidar-sdk)
RUN cd /AutoSDV && \
    git submodule update --init --recursive && \
    echo "All submodules initialized successfully"

# Force rebuild seyond SDK from source for ARM64 (remove precompiled x86-64 binaries)
RUN SDK_PATH="/AutoSDV/src/sensor_component/external/seyond_ros_driver/src/seyond_lidar_ros/src/seyond_sdk" && \
    if [ -d "$SDK_PATH" ]; then \
        echo "=== Seyond SDK found at $SDK_PATH ===" && \
        cd "$SDK_PATH" && \
        echo "Current directory: $(pwd)" && \
        ls -la lib/ && \
        echo "Removing precompiled x86-64 libraries..." && \
        rm -f lib/*.a lib/*.so* && \
        echo "Verifying libraries removed:" && \
        ls -la lib/ || echo "lib/ directory empty (good!)" && \
        echo "Building seyond SDK from source for ARM64..." && \
        cd build && \
        chmod +x build_unix.sh && \
        ./build_unix.sh && \
        echo "Build script completed, checking result:" && \
        cd .. && \
        file lib/*.a && \
        echo "Seyond SDK compiled successfully for ARM64"; \
    else \
        echo "WARNING: Seyond SDK path not found at $SDK_PATH"; \
        exit 1; \
    fi

# Fix git ownership issue for volume-mounted repository
RUN git config --global --add safe.directory '*'

# Run setup to install ROS 2 Humble and other dependencies
WORKDIR /AutoSDV
RUN /bin/bash -c "./scripts/setup-dev-env/setup-dev-env.sh -y"

# Install all required additional dependencies for AutoSDV
# Split into non-ROS and ROS packages for proper dependency resolution
RUN apt-get update && \
    apt-get install -y \
    # OpenCV with matching version
    libopencv-dev:arm64=4.5.4+dfsg-9ubuntu4 \
    libopencv-core4.5d:arm64 \
    python3-opencv \
    # I2C development libraries for MPU9250 driver
    libi2c-dev \
    i2c-tools \
    # Build tools
    wget && \
    rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages after sourcing ROS environment
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    apt-get update && \
    apt-get install -y \
    ros-humble-cv-bridge \
    ros-humble-backward-ros \
    ros-humble-robot-localization \
    ros-humble-point-cloud-transport \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-nmea-navsat-driver && \
    rm -rf /var/lib/apt/lists/*"

# Upgrade CMake to version 3.22+ (required for VTK/PCL compatibility)
RUN wget -q https://github.com/Kitware/CMake/releases/download/v3.22.1/cmake-3.22.1-linux-aarch64.sh && \
    chmod +x cmake-3.22.1-linux-aarch64.sh && \
    ./cmake-3.22.1-linux-aarch64.sh --skip-license --prefix=/usr/local && \
    rm cmake-3.22.1-linux-aarch64.sh && \
    cmake --version

# Create symlinks for ALL OpenCV libraries from aarch64 directory to /usr/lib
# This fixes CMake finding OpenCV libraries on ARM64 architecture
RUN if [ -d /usr/lib/aarch64-linux-gnu ]; then \
      echo "Creating OpenCV library symlinks..."; \
      for lib in /usr/lib/aarch64-linux-gnu/libopencv_*.so*; do \
        if [ -f "$lib" ]; then \
          ln -sf "$lib" "/usr/lib/$(basename $lib)"; \
        fi; \
      done; \
      echo "OpenCV symlinks created successfully"; \
      ls -la /usr/lib/libopencv_*.so* | head -10; \
    fi

# Clone zed-ros2-interfaces (provides zed_msgs package) if not already present
RUN if [ ! -d /AutoSDV/src/sensor_component/external/zed-ros2-interfaces ]; then \
        cd /AutoSDV/src/sensor_component/external && \
        git clone https://github.com/stereolabs/zed-ros2-interfaces.git && \
        echo "zed-ros2-interfaces cloned successfully"; \
    else \
        echo "zed-ros2-interfaces already exists, skipping clone"; \
    fi

# Auto-source ROS and workspace on shell startup
RUN echo "" >> /root/.bashrc && \
    echo "# Auto-source ROS 2 Humble and AutoSDV workspace" >> /root/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /AutoSDV/install/setup.bash ]; then" >> /root/.bashrc && \
    echo "    source /AutoSDV/install/setup.bash" >> /root/.bashrc && \
    echo "    echo '[AutoSDV] Workspace sourced successfully ✓'" >> /root/.bashrc && \
    echo "else" >> /root/.bashrc && \
    echo "    echo '[AutoSDV] Warning: Workspace not built yet. Run: make build'" >> /root/.bashrc && \
    echo "fi" >> /root/.bashrc && \
    echo "cd /AutoSDV" >> /root/.bashrc

# Default command
CMD ["/bin/bash"]
