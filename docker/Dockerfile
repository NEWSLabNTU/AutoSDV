FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel

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

# Remove cmake
RUN rm -f /usr/local/bin/cmake /usr/local/bin/cpack /usr/local/bin/ctest

# Install NVIDIA l4t compiler
RUN apt-key adv --fetch-key http://repo.download.nvidia.com/jetson/jetson-ota-public.asc
COPY nvidia-l4t-apt-source.list /etc/apt/sources.list.d/nvidia-l4t-apt-source.list
RUN mkdir -p /opt/nvidia/l4t-packages
RUN touch /opt/nvidia/l4t-packages/.nv-l4t-disable-boot-fw-update-in-preinstall
RUN apt update
RUN apt install -o DPkg::Options::="--force-confold" -y nvidia-l4t-dla-compiler

WORKDIR /
CMD ["/bin/bash"]
