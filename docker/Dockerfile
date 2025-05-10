FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel

# Accept commit hash as build arg
ARG COMMIT_HASH=2025.02

# Update system packages
RUN apt update -y
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
RUN apt install -y python3-venv
RUN apt install -y python3-pip
RUN apt install -y sudo
RUN apt install -y openssh-client
RUN apt install -y openssh-server
RUN apt install -y git
RUN apt install -y make

# Remove cmake
RUN rm -f /usr/local/bin/cmake /usr/local/bin/cpack /usr/local/bin/ctest

# Install NVIDIA l4t compiler
RUN apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc
COPY nvidia-l4t-apt-source.list /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
RUN mkdir -p /opt/nvidia/l4t-packages
RUN touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall
RUN apt update
RUN apt install -o DPkg::Options::="--force-confold" -y nvidia-l4t-dla-compiler

# Clone AutoSDV repository and checkout specific commit
RUN git clone --recursive https://github.com/NEWSLabNTU/AutoSDV.git && \
    cd AutoSDV && \
    git fetch --all && \
    (git checkout ${COMMIT_HASH} || \
    (echo "Error: Failed to checkout commit ${COMMIT_HASH}. Please ensure this commit is pushed to the remote repository." && exit 1))

# Run setup with non-interactive mode and download artifacts
WORKDIR /AutoSDV
RUN ./scripts/setup-dev-env/setup-dev-env.sh -y

WORKDIR /
RUN rm -rf /AutoSDV

# Default command
CMD ["/bin/bash"]
