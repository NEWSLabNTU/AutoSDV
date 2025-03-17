FROM nvcr.io/nvidia/l4t-tensorrt:r8.6.2-devel AS base

# Update system packages
RUN apt update -y
RUN apt upgrade -y

# Configure the locale and timezone
RUN apt install -y locales
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen
RUN locale-gen
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN ln -snf /usr/share/zoneinfo/Asia/Taipei /etc/localtime && echo Asia/Taipei > /etc/timezone

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

# Remove /usr/local CMake binaries 
RUN rm /usr/local/bin/cmake /usr/local/bin/cpack /usr/local/bin/ctest

# Create the 'jetson' user
RUN useradd -m -u 1000 jetson
RUN usermod -aG sudo jetson
RUN passwd --delete jetson

# Switch to the 'jetson' user
USER jetson
WORKDIR /home/jetson

# Set default process to bash
CMD ["/bin/bash"]


FROM base AS builder

# RUN git clone -b 2024.11 https://github.com/NEWSLabNTU/F1EIGHTH.git
USER root
RUN mkdir /repo
WORKDIR /repo
RUN chown jetson:jetson /repo
USER jetson

RUN mkdir scripts
COPY scripts/setup-dev-env /repo/scripts/setup-dev-env
RUN ./scripts/setup-dev-env/setup-dev-env.sh -y

RUN sudo apt install -y parallel fakeroot
RUN rosdep update
COPY scripts/generate-rosdep-commands.sh /repo/scripts/generate-rosdep-commands.sh
COPY scripts/package-deb.sh /repo/scripts/package-deb.sh
COPY scripts/make-deb.sh /repo/scripts/make-deb.sh
# COPY src /repo/src
# RUN . /opt/ros/humble/setup.sh && \
#     rosdep update --rosdistro=humble && \
#     rosdep install -y --from-paths src --ignore-src -r

USER root
RUN mkdir /mount
WORKDIR /mount
USER jetson

CMD ["/bin/bash"]

FROM builder AS packager

WORKDIR /repo

CMD ["sh", "-c", "./scripts/package-deb.sh ./build_deb"]
