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
# 3 GB in the shipped image. Splitting builder from runtime would reclaim it but
# is awkward: ROS 2 and Autoware pull hundreds of system libraries through apt,
# so a runtime stage would have to install them again rather than copy two
# directories. Left as a size optimisation, not a correctness one.
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

# --- base tools -------------------------------------------------------------
# sudo is here because the setup scripts call it; we are already root, so it
# resolves to a no-op wrapper rather than a privilege change.
#
# Deliberately minimal. Anything a setup step needs belongs in that step, not
# here: a package added here to get the build moving would hide the same gap on
# a student's clean laptop, which is the machine this image exists to stand in
# for. Cython and python3-dev used to be here for exactly that reason; they now
# live in the range-libc step, where they are actually required.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl wget gnupg lsb-release software-properties-common \
        git sudo locales tzdata build-essential \
        python3 python3-pip python3-venv \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# --- TensorRT ---------------------------------------------------------------
# Not shipped in the CUDA image, but the CUDA apt repo it already configures
# carries it, so this is three packages rather than a repository dance. Runtime
# libraries only: the Autoware debs link them; nothing here compiles against
# TensorRT.
RUN apt-get update && apt-get install -y --no-install-recommends \
        libnvinfer10 libnvinfer-plugin10 libnvonnxparsers10 \
    && rm -rf /var/lib/apt/lists/*

# --- the workspace ----------------------------------------------------------
# Copied rather than cloned so the image can be built from an unpushed working
# tree while this is being developed. Dockerfile.dockerignore keeps the context
# to source only (~690 MB of a ~30 GB tree). A published image builds from a tag.
COPY . ${AUTOSDV_HOME}
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
RUN ./setup.sh --run --profile dev --yes \
        --skip cyclonedds-sysctl multicast-lo \
    && rm -rf /var/lib/apt/lists/*

# --- build the workspace ----------------------------------------------------
# Shipped prebuilt: a student reaches a running simulation in minutes instead of
# spending the first 40 of them watching colcon, and a laptop that would run out
# of memory while linking never has to.
RUN bash -c 'set -eo pipefail; \
      set +u; source /opt/autoware/1.5.0/setup.bash; set -u; \
      just build'

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
