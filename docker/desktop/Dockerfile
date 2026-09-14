# AutoSDV desktop image: the workshop environment.
#
# A student opens a browser at http://localhost:6080 and gets a desktop with
# RViz and a terminal, on Windows, macOS or Linux, with no ROS 2 and no Autoware
# installed on their laptop.
#
# Graphics are opportunistic. The container always runs its own X server, so the
# run command is identical everywhere; the entrypoint then picks the fastest
# renderer the host actually exposes (see entrypoint.sh). macOS can never expose
# a GPU to a container -- Hypervisor.framework has no vGPU -- so software
# rendering is the floor, and rviz/workshop.rviz exists to make that floor fast.
#
# ---------------------------------------------------------------------------
# Why an NVIDIA base image, on machines that may have no NVIDIA GPU
#
# setup.sh deliberately does not install CUDA or TensorRT: on the vehicle they
# come from JetPack, on a workstation from the host image. A clean container has
# neither, and the Autoware debs then refuse to resolve:
#
#   ros-humble-autoware-tensorrt-common-1-5-0 : Depends: libnvinfer10
#                                                        but it is not installable
#
# So the libraries must be present even where no GPU is. They are libraries, not
# a driver: nothing in Autoware links libcuda.so.1 (0 of 840 amd64 and 0 of 1053
# arm64 shipped libraries), so no driver is installed and none is needed.
#
# `devel` rather than `runtime`, because the workspace is BUILT here:
# cuda_ffi/build.rs panics without nvcc and the CUB headers. That costs roughly
# 3 GB in the shipped image.
#
# It stays `devel` even though the image could now be made smaller without it --
# cuda_ffi learned to build as a stub when no toolkit is present, so a `runtime`
# base would compile. That is a DELIBERATE decision, not an oversight: the image
# is meant to be usable for development later, and a student who wants to change
# CUDA code should find a toolchain rather than a stub. `pose_source:=cuda_ndt`
# is also the default, and shipping it stubbed would make the default silently
# not what it says. Do not swap this for `runtime` to save space.
#
# CUDA 12.8 matches what upstream Autoware selects for humble. The tag is
# published for linux/amd64 AND linux/arm64, which is what lets one Dockerfile
# serve Apple Silicon natively.
#
# The version lives in versions.yaml (container.desktop_base) and arrives as a
# build arg, so this line is not a second copy of it. build.sh passes it;
# the default here only keeps a bare `docker build` working.
#
# Measurements: docs/reports/gpu-less-simulation-and-rviz.md
# ---------------------------------------------------------------------------

ARG CUDA_IMAGE=nvidia/cuda:12.8.1-cudnn-devel-ubuntu22.04
FROM ${CUDA_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
ENV AUTOSDV_HOME=/opt/AutoSDV

# `just` lands in /root/.local/bin and rustup in /root/.cargo/bin, neither of
# which is on PATH in a non-interactive RUN. nvcc must be on PATH at build time
# too: ndt_cuda/build.rs records that cubecl-cuda silently assumes CUDA 13.0
# when nvcc is absent, and then compiles against the wrong API.
ENV CUDA_HOME=/usr/local/cuda
ENV PATH=/usr/local/cuda/bin:/root/.local/bin:/root/.cargo/bin:$PATH

# --- the middleware, and why it is set here ----------------------------------
# versions.yaml declares rmw_cyclonedds_cpp as this project's RMW, but nothing
# in the install path exports it: scripts/env.sh deliberately leaves the choice
# to the user, and Autoware's own setup.bash does not set it either (a comment
# in env.sh says it does -- measured here, it does not). A shell that sources
# the workspace and nothing else therefore runs ROS 2 Humble's default,
# rmw_fastrtps_cpp.
#
# That is not a neutral difference at this scale. The logging simulation brings
# up 125 participants, and FastDDS addresses participants through ports derived
# from the participant index, which runs out around 120 per host per domain.
# Past that point the STACK KEEPS WORKING -- every participant that joined
# early still talks -- but nothing new can join: a second terminal running
# `ros2 bag play` publishes into the void, and `ros2 topic list` reports two
# topics while 150 nodes are running. Nothing reports an error.
#
# That is the book's entire two-terminal logging-simulation tutorial, so it is
# not optional here. Measured on this image, same container, same stack:
#
#   rmw_fastrtps_cpp     2 topics,   0 nodes visible from a second shell
#   rmw_cyclonedds_cpp   605 topics, 150 nodes
#
# CycloneDDS is already installed; only the selection was missing. Bridge
# networking is enough -- this is not a networking problem and --network host
# does not fix it, which was checked.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# --- base tools -------------------------------------------------------------
# sudo is here because the setup scripts call it; we are already root, so it
# resolves to a no-op wrapper rather than a privilege change.
#
# Deliberately minimal. Anything a setup step needs belongs in that step, not
# here: a package added here to get the build moving would hide the same gap on
# a student's clean laptop, which is the machine this image exists to stand in
# for. Cython and python3-dev used to be here for exactly that reason; they now
# live in the range-libc step, where they are actually required.
#
# aria2 is the exception that earns its place, because the gap it hides is one
# the image itself creates. The Autoware deb is ~2 GB and the release host
# throttles per connection: measured on an AGX Orin, wget 383 KB/s against
# aria2c 2905 KB/s, so a build that falls back to a single stream spends hours
# where it should spend ten minutes -- one arm64 attempt crawled at 182 KB/s
# for an hour before this was noticed. install-autoware-debian.sh also installs
# aria2 when it is missing, which covers a bare laptop; having it here means the
# image never pays that apt round trip mid-build.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl wget aria2 gnupg lsb-release software-properties-common \
        git sudo locales tzdata build-essential \
        python3 python3-pip python3-venv \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# --- the installer, ahead of the workspace it installs for --------------------
# Copied rather than cloned so the image can be built from an unpushed working
# tree while this is being developed. Dockerfile.dockerignore keeps the context
# to source only (~690 MB of a ~30 GB tree). A published image builds from a tag.
#
# setup/ arrives WITHOUT src/, deliberately. Installing ROS 2 and Autoware takes
# about seventy minutes, and if that layer is keyed on the whole tree then every
# edit to a single CMakeLists -- which is what the failures at this stage
# actually are -- re-downloads and re-installs all of it before it can reach the
# line that changed. Six arm64 attempts paid that toll. Only two setup steps
# read src/ at all (ros-deps and range-libc), so they move below the full COPY
# and everything else keys on the installer alone.
COPY setup/ ${AUTOSDV_HOME}/setup/
COPY scripts/ ${AUTOSDV_HOME}/scripts/
COPY just/ ${AUTOSDV_HOME}/just/
COPY demo/ ${AUTOSDV_HOME}/demo/
COPY versions.yaml justfile ${AUTOSDV_HOME}/
WORKDIR ${AUTOSDV_HOME}

# Belt and braces against the worst build bug available here: setup.sh imports
# .markers/ as "already installed", so a host's markers arriving in the context
# make the image skip ROS 2 and Autoware entirely -- silently, and only visible
# much later as a broken image. Dockerfile.dockerignore excludes them; this
# makes certain.
RUN rm -rf setup/.markers setup/.state.json

# --- everything the repository already knows how to install -----------------
# setup.sh is the tested path and the one the book teaches, so the image uses it
# rather than a parallel list that would drift.
#
# Two steps are skipped because they cannot work inside a container:
#   cyclonedds-sysctl  net.core.rmem_max is not namespaced, so it can be set
#                      neither here nor with `docker run --sysctl` -- Docker
#                      refuses to start a container that asks for it. The value
#                      comes from the kernel underneath: the Linux host, or
#                      Docker Desktop's VM.
#   multicast-lo       installs a systemd unit; there is no systemd here. The
#                      entrypoint sets the loopback MULTICAST flag instead.
#
# The cleanup is part of THIS command rather than a RUN of its own, which is the
# only way a deletion reclaims anything: a `rm` in a later layer writes a
# whiteout and the bytes stay in the image regardless.
#
#   the localrepo      Autoware arrives as one 1.9 GB .deb that unpacks an apt
#                      pool to /opt/autoware/1.5.0/repo. After the 333 packages
#                      are installed, that pool is a second copy of them.
#                      Purged rather than rm'd so dpkg and apt stay consistent:
#                      it also takes the sources.list and preferences pin, and
#                      nothing depends on it (verified: 0 packages cascade).
#   the Windows blob   TensorRT ships libnvinfer_builder_resource_win.so, the
#                      resource for building engines that target Windows. 1.9 GB,
#                      dlopen'd only by a builder cross-compiling to Windows,
#                      which cannot happen here.
#   the build caches   ~/.cargo/registry is downloaded crate source and rustdoc
#                      is documentation. The toolchain itself stays, so a
#                      student can still rebuild a Rust package.
#
# The Autoware .deb lands in a CACHE MOUNT rather than in the image. Two things
# follow, both wanted:
#
#   * it is not a third copy of those 1.9 GB in the layer, so there is nothing
#     to delete afterwards -- a cache mount is never committed.
#   * a rebuild reuses the download. `COPY . ${AUTOSDV_HOME}` sits above this
#     step, so ANY source change invalidates it; without the cache each such
#     change re-downloads 2 GB before it can fail on the line that changed,
#     which is what made every arm64 attempt cost 35 minutes at best.
#
# install-autoware-debian.sh verifies the checksum of a file it finds already
# present and exits if it does not match, so a truncated cache cannot be
# installed silently.
#
# ros-deps and range-libc are skipped here and run below, after src/ arrives:
# they are the only two steps that read the workspace. `--only` runs them there
# by name, so this remains one list of steps rather than two.
#
# `./setup/setup.sh`, not `./setup.sh`: the repo root carries a SYMLINK to it,
# and the launcher derives the location of main.py with `readlink -f "$0"`.
# `COPY .` preserves that symlink, but naming it as an explicit COPY source
# dereferences it into a plain file, whose readlink then resolves to the repo
# root and looks for main.py there:
#
#   python3: can't open file '/opt/AutoSDV/main.py'
#
# The link arrives intact with the full COPY below; this layer calls the real
# file directly.
RUN --mount=type=cache,target=/opt/AutoSDV/data/autoware-debian,sharing=locked \
    ./setup/setup.sh --run --profile dev --yes \
        --skip cyclonedds-sysctl multicast-lo ros-deps range-libc \
    && apt-get purge -y autoware-localrepo-1-5-0 \
    && rm -f /opt/tensorrt/*/lib/libnvinfer_builder_resource_win.so.* \
    && rm -rf /root/.cargo/registry /root/.cargo/git \
    && rm -rf /root/.rustup/toolchains/*/share/doc \
    && rm -rf /var/lib/apt/lists/*

# --- the workspace ----------------------------------------------------------
# Everything below this line is re-run by a source change; everything above it
# is not.
COPY . ${AUTOSDV_HOME}

# The two setup steps that read src/: rosdep over the workspace's package.xml
# files, and the range_libc Cython extension built out of a submodule. Same
# registry, same order, named rather than re-described.
RUN ./setup.sh --run --only ros-deps range-libc --yes \
    && rm -rf /var/lib/apt/lists/*

# --- build the workspace ----------------------------------------------------
# Shipped prebuilt: a student reaches a running simulation in minutes instead of
# spending the first 40 of them watching colcon, and a laptop that would run out
# of memory while linking never has to.
RUN bash -c 'set -eo pipefail; \
      set +u; source /opt/autoware/1.5.0/setup.bash; set -u; \
      just build' \
    && rm -rf /root/.cargo/registry /root/.cargo/git

# `build/` and `src/` cannot be deleted, however tempting 1.5 GB is: `just build`
# passes --symlink-install, so the 25 MB install/ tree is 1071 symlinks pointing
# back into them. Removing either empties the workspace without any error.

# --- build-only weight inherited from the base image -------------------------
# CUDA's static archives (3.6 GB) and Nsight Compute (1.1 GB) come from the
# `devel` base and nothing loads them at run time. They are deleted here rather
# than left alone because build.sh FLATTENS the finished image; without that
# flatten this RUN reclaims nothing, since the bytes belong to a base layer.
# The glob is inside find rather than in the shell: the arm64 base is plain
# ubuntu:22.04 with no /usr/local/cuda-* at all, and an unmatched shell glob
# would hand find a literal path it then fails on, breaking the arm64 build.
RUN find /usr/local -maxdepth 1 -type d -name 'cuda-*' \
        -exec find {} -name '*.a' -delete \; \
    && rm -rf /opt/nvidia/nsight-compute

# --- graphics ---------------------------------------------------------------
# TurboVNC supplies the X server and the VNC transport; VirtualGL is used only
# when the host actually exposes a GPU. noVNC turns that into a browser tab, so
# the student installs no VNC client.
RUN ./setup/scripts/install-turbovnc-virtualgl.sh \
    && apt-get update && apt-get install -y --no-install-recommends \
        novnc websockify python3-numpy \
        openbox xterm dbus-x11 x11-xserver-utils mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# TurboVNC kills Xvnc when it cannot find a session desktop file for a window
# manager it recognises, and openbox ships none. This supplies the session.
COPY docker/desktop/rootfs/root/.vnc/xstartup.turbovnc /root/.vnc/xstartup.turbovnc

COPY docker/desktop/entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh /root/.vnc/xstartup.turbovnc

EXPOSE 6080 8081
ENV DISPLAY=:1
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
