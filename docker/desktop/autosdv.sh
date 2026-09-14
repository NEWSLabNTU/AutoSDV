#!/usr/bin/env bash
# Start the AutoSDV desktop container, or open another shell in the one that is
# already running.  Linux and macOS.  Windows: use autosdv.ps1.
#
#   ./docker/desktop/autosdv.sh          first run: pull, start, open a shell
#   ./docker/desktop/autosdv.sh          again, anywhere: a SECOND shell in the
#                                        SAME container
#   ./docker/desktop/autosdv.sh --gpu    pass an NVIDIA GPU through to the
#                                        container (Linux with the NVIDIA
#                                        Container Toolkit; not macOS, and not
#                                        Docker Desktop on Windows)
#   ./docker/desktop/autosdv.sh --pull   check for a newer image first
#   ./docker/desktop/autosdv.sh --stop   stop and remove the container
#
# If the image was handed out as a file, `docker load` it first; this script
# then finds it locally and pulls nothing.
#
# The logging simulation needs two terminals -- one for the stack and one for
# the rosbag replay -- so running this twice is the normal case, not an error.
#
# Every shell it opens has ROS 2, Autoware and the workspace already sourced,
# because a shell without them has no `ros2` command at all and the error says
# only "command not found".

set -euo pipefail

NAME="${AUTOSDV_CONTAINER:-autosdv}"
IMAGE="${AUTOSDV_IMAGE:-jerry73204/autosdv:desktop}"
PORT="${AUTOSDV_PORT:-6080}"

# --gpu is consumed here rather than passed through: it changes how the
# container is CREATED, so it is meaningless on the runs that merely attach a
# second shell to one that already exists.
GPU=0
args=()
for a in "$@"; do
    case "$a" in
        --gpu) GPU=1 ;;
        *) args+=("$a") ;;
    esac
done
set -- "${args[@]+"${args[@]}"}"

# The repository, found from this script rather than from the caller's working
# directory, so the data mount is right no matter where it is invoked from.
#
# Resolved without `readlink -f`, which does not exist on macOS: BSD readlink
# has no -f, so the whole script would die on the first line that mattered, on
# exactly the platform this exists to serve. This walks symlinks by hand and
# works on bash 3.2, which is still what macOS ships.
_src="${BASH_SOURCE[0]}"
while [ -L "$_src" ]; do
    _dir="$(cd -P "$(dirname "$_src")" && pwd)"
    _src="$(readlink "$_src")"
    case "$_src" in /*) ;; *) _src="$_dir/$_src" ;; esac
done
REPO="$(cd -P "$(dirname "$_src")/../.." && pwd)"

say() { printf '  %s\n' "$*"; }

# Docker's own message for this is "Bind for 0.0.0.0:6080 failed: port is
# already allocated", which names neither what wanted the port nor how to move
# it. On a laptop that is usually another copy of this container, or whatever
# else the student happens to be running.
port_help() {
    cat >&2 <<EOF

  Could not start the container. If the message above mentions port ${PORT},
  something else already has it -- often an earlier AutoSDV container.

  See what it is:

      docker ps --filter publish=${PORT}

  Then either remove the old container:

      $0 --stop

  or use a different port:

      AUTOSDV_PORT=6081 $0

EOF
}


case "${1:-}" in
    --stop)
        docker rm -f "$NAME" >/dev/null 2>&1 && say "stopped and removed '$NAME'" \
            || say "no container named '$NAME'"
        exit 0
        ;;
    -h|--help)
        sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'
        exit 0
        ;;
esac

docker info >/dev/null 2>&1 || {
    cat >&2 <<EOF

  Docker is not running.

  Linux:  sudo systemctl start docker
  macOS:  open Docker Desktop and wait for the whale icon to settle

  Install: https://docs.docker.com/desktop/

EOF
    exit 1
}

# --- already up?  then this is just another terminal -------------------------
if [ "$(docker inspect -f '{{.State.Running}}' "$NAME" 2>/dev/null || echo false)" = "true" ]; then
    # Report the port the container was actually started on, not the default
    # this invocation happens to carry: a student who started it with
    # AUTOSDV_PORT=6081 and then runs the script plainly would otherwise be
    # sent to a URL that serves nothing.
    running_port="$(docker inspect \
        -f '{{with index .NetworkSettings.Ports "6080/tcp"}}{{(index . 0).HostPort}}{{end}}' \
        "$NAME" 2>/dev/null || true)"
    PORT="${running_port:-$PORT}"
    if [ "$GPU" = "1" ]; then
        cat >&2 <<EOF

  note: --gpu only takes effect when the container is created, and '$NAME' is
  already running. To switch it on, stop it first:

      $0 --stop
      $0 --gpu

EOF
    fi
    say "attaching another shell to '$NAME'"
    say "desktop: http://localhost:${PORT}/vnc.html?autoconnect=1&resize=remote"
    echo
    exec docker exec -it "$NAME" bash -c \
        'cd /opt/AutoSDV
         source /opt/autoware/1.5.0/setup.bash
         source install/setup.bash
         cat <<BANNER
  AutoSDV -- ROS 2, Autoware and the workspace are sourced.

  Two terminals: run this script again for the second one.
  Add --container-mode observable to play_launch: the default forks one
  process per composable node (126 processes, 4.6 GB) where observable keeps
  them in their containers (49 processes, 2.1 GB). On a memory-capped Docker
  Desktop VM the default gets nodes killed.

BANNER
         exec bash'
fi

# A container of that name exists but is stopped -- reuse rather than surprise
# the user with a name clash.
if docker inspect "$NAME" >/dev/null 2>&1; then
    say "starting the existing container '$NAME'"
    docker start "$NAME" >/dev/null || { port_help; exit 1; }
else
    if [ "${1:-}" = "--pull" ] || ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
        say "pulling ${IMAGE} -- this is several gigabytes, and only happens once"
        say "(if the image was handed out as a file, run 'docker load' on it first)"
        docker pull "$IMAGE"
    fi

    [ -d "${REPO}/data" ] || {
        echo "error: ${REPO}/data does not exist." >&2
        echo "       The map and rosbags live there and are not in the image." >&2
        echo "       Run this from a clone of the AutoSDV repository." >&2
        exit 1
    }

    gpu_args=()
    if [ "$GPU" = "1" ]; then
        # Fail here with an explanation rather than letting `docker run` emit
        # "could not select device driver with capabilities: [[gpu]]", which
        # names neither the missing piece nor where to get it.
        if ! docker info 2>/dev/null | grep -qi 'Runtimes:.*nvidia' \
           && ! command -v nvidia-ctk >/dev/null 2>&1; then
            cat >&2 <<EOF

  --gpu needs the NVIDIA Container Toolkit, which is not installed.

  It is Linux only: macOS cannot pass a GPU to a container at all, and
  Docker Desktop on Windows does it through WSL2 rather than this flag.

      https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

  Without it the container still works -- RViz renders in software, and the
  CPU paths (pose_source:=ndt, launch_perception:=false) are what the lab
  uses anyway.

EOF
            exit 1
        fi
        # The runtime in the image is CUDA 12.8, which needs a driver at least
        # as new as the forward-compat library it ships
        # (/usr/local/cuda/compat/libcuda.so.570.124.06). An older host driver
        # does not fail at startup -- the container comes up, the stack reaches
        # "Startup complete", and then individual CUDA nodes throw
        #
        #   cudaErrorInsufficientDriver (35): CUDA driver version is
        #   insufficient for CUDA runtime version
        #
        # from inside a composable-node constructor, which reads as an Autoware
        # problem rather than a driver one. Say it here instead.
        need="570.124.06"
        have="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ')"
        if [ -n "$have" ] && [ "$(printf '%s\n%s\n' "$need" "$have" | sort -V | head -1)" != "$need" ]; then
            cat >&2 <<EOF

  warning: this host's NVIDIA driver is ${have}, and the image's CUDA 12.8
  runtime needs ${need} or newer.

  The container will start and the stack will report "Startup complete", but
  CUDA nodes will fail one by one with

      cudaErrorInsufficientDriver (35)

  Either update the driver, or simply leave --gpu off: the CPU path
  (pose_source:=ndt launch_perception:=false) is what the lab uses and it
  needs no GPU at all.

EOF
        fi

        # NVIDIA_DRIVER_CAPABILITIES must include `graphics`, not just the
        # default `compute,utility`: without it the driver exposes CUDA but no
        # GL, and the entrypoint's VirtualGL path finds a GPU it cannot draw
        # with.
        gpu_args=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all -e NVIDIA_VISIBLE_DEVICES=all)
        say "passing through an NVIDIA GPU"
    fi

    say "starting '$NAME'"
    # -dit rather than -d: the image's command is bash, which exits immediately
    # without a terminal attached, and the container would stop with it.
    #
    # RMW_IMPLEMENTATION is set because the default, FastDDS, addresses
    # participants through ports derived from an index that runs out near 120
    # per host per domain -- and the logging simulation brings up 125. Past
    # that the stack keeps working while NOTHING NEW CAN JOIN: a second
    # terminal's `ros2 bag play` publishes into the void and `ros2 topic list`
    # shows two topics beside 150 running nodes, with no error anywhere.
    # Images built after this was found already set it; this keeps older ones
    # working and costs nothing.
    docker run -dit \
        --name "$NAME" \
        -p "${PORT}:6080" \
        -v "${REPO}/data:/opt/AutoSDV/data" \
        --shm-size=2gb \
        --cap-add=NET_ADMIN \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        ${gpu_args[@]+"${gpu_args[@]}"} \
        "$IMAGE" >/dev/null || {
            # A failed `docker run` still leaves the named container behind, and
            # the next invocation would take the "exists but stopped" path and
            # fail again in a way that no longer names the cause.
            docker rm -f "$NAME" >/dev/null 2>&1 || true
            port_help
            exit 1
        }
fi

say "desktop: http://localhost:${PORT}/vnc.html?autoconnect=1&resize=remote"
say "another terminal: run this script again"
say "stop: $0 --stop"
echo

exec docker exec -it "$NAME" bash -c \
    'cd /opt/AutoSDV
     source /opt/autoware/1.5.0/setup.bash
     source install/setup.bash
     cat <<BANNER
  AutoSDV -- ROS 2, Autoware and the workspace are sourced.

  Two terminals: run this script again for the second one.
  Add --container-mode observable to play_launch: the default forks one
  process per composable node (126 processes, 4.6 GB) where observable keeps
  them in their containers (49 processes, 2.1 GB). On a memory-capped Docker
  Desktop VM the default gets nodes killed.

BANNER
     exec bash'
