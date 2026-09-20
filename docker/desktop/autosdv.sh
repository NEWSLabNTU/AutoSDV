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
#   ./docker/desktop/autosdv.sh --workspace DIR
#                                        mount DIR at /workspace instead of the
#                                        repository this script lives in
#                                        (AUTOSDV_WORKSPACE does the same)
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
#
# Your checkout is mounted at /workspace, and lab work belongs there: it is the
# one directory whose contents survive `--stop`, and your own editor is already
# looking at it.

set -euo pipefail

NAME="${AUTOSDV_CONTAINER:-autosdv}"
IMAGE="${AUTOSDV_IMAGE:-jerry73204/autosdv:desktop}"
PORT="${AUTOSDV_PORT:-6080}"

# --gpu and --workspace are consumed here rather than passed through: both
# change how the container is CREATED, so they are meaningless on the runs that
# merely attach a second shell to one that already exists.
#
# A while loop rather than `for a in "$@"`, because --workspace takes a value
# and a for loop cannot shift past one.
GPU=0
WORKSPACE="${AUTOSDV_WORKSPACE:-}"
args=()
while [ $# -gt 0 ]; do
    case "$1" in
        --gpu) GPU=1 ;;
        --workspace)
            shift
            [ $# -gt 0 ] || { echo "error: --workspace needs a directory" >&2; exit 1; }
            WORKSPACE="$1"
            ;;
        --workspace=*) WORKSPACE="${1#--workspace=}" ;;
        *) args+=("$1") ;;
    esac
    shift
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

# What gets mounted at /workspace. The repository by default, because that is
# what a student cloned and what their editor has open.
#
# Resolved to an absolute path here: `docker run -v` takes a relative path as a
# VOLUME NAME rather than a directory, so `--workspace ../labs` would silently
# create an empty named volume called "..labs" instead of failing.
WORKSPACE="${WORKSPACE:-$REPO}"
if [ ! -d "$WORKSPACE" ]; then
    echo "error: --workspace ${WORKSPACE} is not a directory." >&2
    exit 1
fi
WORKSPACE="$(cd -P "$WORKSPACE" && pwd)"

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

# The shell every terminal gets: ROS 2, Autoware and the workspace already
# sourced. ONE copy, used by both the attach path and the create path -- they
# are the same shell, and keeping two literals in step by hand is how the
# banner ends up telling half the terminals something the other half does not
# know.
#
# `cd /opt/AutoSDV` is conditional because :base carries no prebuilt workspace
# (only :desktop does). Sourcing an install/setup.bash that is not there prints
# "No such file or directory" on every shell the student opens, which reads as
# a broken image rather than as a different image.
#
# And on that path NOTHING under /workspace is sourced, deliberately. A
# checkout that was ever built on the host has an install/setup.bash whose
# paths are the HOST's, so sourcing it greets every shell with
#   not found: "/opt/ros/humble/local_setup.bash"
# from inside a container where ROS is installed and fine. The student builds
# and sources their own workspace; the launcher does not guess at one.
ENTER='
if [ -f /opt/AutoSDV/install/setup.bash ]; then
    cd /opt/AutoSDV
    [ -f /opt/autoware/1.5.0/setup.bash ] && source /opt/autoware/1.5.0/setup.bash
    source install/setup.bash
    _sourced="ROS 2, Autoware and the workspace are sourced."
else
    cd /workspace 2>/dev/null || cd
    [ -f /opt/autoware/1.5.0/setup.bash ] && source /opt/autoware/1.5.0/setup.bash
    _sourced="ROS 2 and Autoware are sourced; build your own workspace here."
fi
cat <<BANNER
  AutoSDV -- $_sourced

  /workspace is your own checkout, mounted from the host: what you write there
  is on your laptop, and is the only thing that survives --stop.

  Two terminals: run this script again for the second one.
  Add --container-mode observable to play_launch: the default forks one
  process per composable node (126 processes, 4.6 GB) where observable keeps
  them in their containers (49 processes, 2.1 GB). On a memory-capped Docker
  Desktop VM the default gets nodes killed.

BANNER
exec bash'

# A MOUNT CANNOT BE ADDED TO A CONTAINER THAT ALREADY EXISTS -- not to a
# running one, and not to a stopped one either, because the mount list is fixed
# when the container is CREATED. A student who started theirs before
# /workspace existed -- yesterday, or with an older copy of this script --
# reaches it every day afterwards and finds no /workspace, with nothing to say
# why: `cd /workspace` reports only "No such file or directory", which reads as
# a mistake in the handout rather than as a stale container.
#
# Reported, not repaired. Recreating the container silently is the one thing
# not to do here: anything installed or left in $HOME inside it goes with it,
# and this script cannot know whether that matters.
warn_workspace_mount() {
    local mounted_ws
    mounted_ws="$(docker inspect \
        -f '{{range .Mounts}}{{if eq .Destination "/workspace"}}{{.Source}}{{end}}{{end}}' \
        "$NAME" 2>/dev/null || true)"
    if [ -z "$mounted_ws" ]; then
        cat >&2 <<EOF

  note: '$NAME' has no /workspace. It was created before this script mounted
  one, and a mount cannot be added to a container that already exists.

  Recreate it to get one:

      $0 --stop
      $0

  Your checkout on the host is untouched by that -- but anything saved INSIDE
  the container is removed with it, so copy that out first if it matters:

      docker cp $NAME:/root/something .

EOF
    elif [ "$mounted_ws" != "$WORKSPACE" ]; then
        cat >&2 <<EOF

  note: '$NAME' already has /workspace mounted from

      ${mounted_ws}

  and you asked for

      ${WORKSPACE}

  A mount cannot be changed on a container that already exists, so
  --workspace has no effect here. To switch:

      $0 --stop
      $0 --workspace ${WORKSPACE}

EOF
    fi
}

# `docker exec` does NOT go through the entrypoint, so the shell it opens runs
# as whatever user the image declares -- root -- however carefully the
# entrypoint reconciled HOST_UID and dropped to `autosdv` on startup. And every
# shell this script opens is a `docker exec`, including the second terminal the
# workshop's own two-terminal workflow needs. Left alone, a colcon build or a
# recorded bag from one of them is root-owned inside the student's own
# checkout, which is the exact symptom the UID matching exists to prevent:
# `git status` on the host then refuses with "detected dubious ownership".
#
# Asked, not assumed. An older image has no `autosdv` account reconciled to
# this host's UID, and `docker exec -u` against one gives a shell that greets
# the student with "I have no name!", cannot write $HOME, and fails at ~/.ros
# logging. Where the passwd entry exists we use it; where it does not, this is
# byte-identical to before.
EXEC_USER=()
set_exec_user() {
    if docker exec "$NAME" getent passwd "$(id -u)" >/dev/null 2>&1; then
        EXEC_USER=(-u "$(id -u):$(id -g)")
    fi
}

case "${1:-}" in
    --stop)
        docker rm -f "$NAME" >/dev/null 2>&1 && say "stopped and removed '$NAME'" \
            || say "no container named '$NAME'"
        exit 0
        ;;
    -h|--help)
        sed -n '2,31p' "$0" | sed 's/^# \{0,1\}//'
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

    warn_workspace_mount

    say "attaching another shell to '$NAME'"
    say "desktop: http://localhost:${PORT}/vnc.html?autoconnect=1&resize=remote"
    echo
    set_exec_user
    exec docker exec -it ${EXEC_USER[@]+"${EXEC_USER[@]}"} "$NAME" bash -c "$ENTER"
fi

# A container of that name exists but is stopped -- reuse rather than surprise
# the user with a name clash.
if docker inspect "$NAME" >/dev/null 2>&1; then
    say "starting the existing container '$NAME'"
    docker start "$NAME" >/dev/null || { port_help; exit 1; }
    warn_workspace_mount
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
    #
    # TWO mounts, and the data one is not redundant. /workspace is the
    # student's checkout, where their own work lives; the data mount stays
    # because :desktop's prebuilt workspace IS /opt/AutoSDV, and mounting the
    # checkout over it would shadow the prebuilt install/ and break the
    # `source install/setup.bash` every shell here does. So the map and the
    # rosbags are mounted into it by themselves, and the rest of /opt/AutoSDV
    # stays the image's.
    #
    # HOST_UID/HOST_GID are read by the entrypoint, which usermods the built-in
    # user to match before dropping privileges. Without them a Linux student's
    # build/, install/, log/ and recorded bags land root-owned inside their own
    # checkout and `git status` refuses with "detected dubious ownership".
    # macOS (VirtioFS) and Windows (drvfs) map ownership anyway, so there this
    # is harmlessly ignored.
    docker run -dit \
        --name "$NAME" \
        -p "${PORT}:6080" \
        -v "${REPO}/data:/opt/AutoSDV/data" \
        -v "${WORKSPACE}:/workspace" \
        -e HOST_UID="$(id -u)" \
        -e HOST_GID="$(id -g)" \
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
say "workspace: ${WORKSPACE} -> /workspace"
say "another terminal: run this script again"
say "stop: $0 --stop"
echo

set_exec_user
exec docker exec -it ${EXEC_USER[@]+"${EXEC_USER[@]}"} "$NAME" bash -c "$ENTER"
