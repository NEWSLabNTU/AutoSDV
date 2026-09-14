#!/usr/bin/env bash
# Start the AutoSDV desktop container, or open another shell in the one that is
# already running.  Linux and macOS.  Windows: use autosdv.ps1.
#
#   ./docker/desktop/autosdv.sh          first run: pull, start, open a shell
#   ./docker/desktop/autosdv.sh          again, anywhere: a SECOND shell in the
#                                        SAME container
#   ./docker/desktop/autosdv.sh --pull   check for a newer image first
#   ./docker/desktop/autosdv.sh --stop   stop and remove the container
#
#   AUTOSDV_SERVER=http://10.0.0.5:8000 ./docker/desktop/autosdv.sh
#                                        take the image from the classroom
#                                        server rather than Docker Hub. The
#                                        right architecture is chosen for you.
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


# Which image a machine needs is its PROCESSOR, not its operating system: an
# Apple Silicon Mac takes arm64 and an Intel Mac takes amd64, like Windows and
# like Linux on a PC. Nobody should have to know that about themselves, so ask
# Docker rather than the student.
#
# `docker version` and not `uname -m`: under Rosetta a shell reports x86_64 on
# an Apple Silicon machine whose Docker is arm64, and the student would load
# an emulated image that runs at a fraction of the speed for no visible reason.
detect_arch() {
    local a
    a="$(docker version --format '{{.Server.Arch}}' 2>/dev/null || true)"
    case "$a" in
        amd64|x86_64)  echo amd64 ;;
        arm64|aarch64) echo arm64 ;;
        *)
            echo "error: could not determine the Docker daemon's architecture (got '${a:-nothing}')." >&2
            echo "       Set AUTOSDV_ARCH=amd64 or AUTOSDV_ARCH=arm64 and run again." >&2
            exit 1
            ;;
    esac
}

# Fetch the image from a machine on the local network instead of Docker Hub.
# Fifty laptops pulling 14 GB each from the internet does not finish inside a
# class; one laptop serving a directory does.
load_from_server() {
    local arch file url tmp sum
    arch="${AUTOSDV_ARCH:-$(detect_arch)}"
    file="autosdv-desktop-${arch}.tar.gz"
    url="${AUTOSDV_SERVER%/}/${file}"
    tmp="${TMPDIR:-/tmp}/${file}"

    say "this machine needs the ${arch} image"
    say "fetching ${url}"
    say "(several gigabytes; it resumes if interrupted, so run this again)"
    echo

    # -C - resumes a partial file. On a shared classroom network a dropped
    # download is normal, and starting a 14 GB transfer again from zero is how
    # a session runs out of time.
    curl -fL -C - -o "$tmp" "$url" || {
        echo "error: download failed. Check AUTOSDV_SERVER=${AUTOSDV_SERVER}" >&2
        echo "       and that the server is reachable: curl -I ${url}" >&2
        exit 1
    }

    # Verify before loading. A truncated archive loads for many minutes and
    # then fails with a tar error that says nothing about the network.
    if curl -fsL -o "${tmp}.sha256" "${url}.sha256" 2>/dev/null; then
        say "verifying"
        if command -v sha256sum >/dev/null 2>&1; then
            sum="$(sha256sum "$tmp" | awk '{print $1}')"
        else
            sum="$(shasum -a 256 "$tmp" | awk '{print $1}')"    # macOS has no sha256sum
        fi
        if [ "$sum" != "$(awk '{print $1}' "${tmp}.sha256")" ]; then
            echo "error: checksum mismatch -- the download is incomplete or corrupt." >&2
            echo "       Delete ${tmp} and run this again." >&2
            exit 1
        fi
        say "checksum ok"
    else
        say "no checksum published alongside the image; skipping verification"
    fi

    say "loading into Docker (a few minutes, no progress output)"
    docker load -i "$tmp"
    rm -f "${tmp}.sha256"
    say "loaded. ${tmp} can be deleted, or kept to share with someone else."
    echo
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
        if [ -n "${AUTOSDV_SERVER:-}" ]; then
            load_from_server
        else
            say "pulling ${IMAGE} -- this is several gigabytes, and only happens once"
            docker pull "$IMAGE"
        fi
    fi

    [ -d "${REPO}/data" ] || {
        echo "error: ${REPO}/data does not exist." >&2
        echo "       The map and rosbags live there and are not in the image." >&2
        echo "       Run this from a clone of the AutoSDV repository." >&2
        exit 1
    }

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
