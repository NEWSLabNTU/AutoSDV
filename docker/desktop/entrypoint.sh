#!/usr/bin/env bash
# Start the desktop, then hand over to the command.
#
# The whole point of this file is that the student's run command is the same on
# every host. What differs between hosts is which renderer is reachable, and
# that is decided here rather than in the instructions.

set -euo pipefail

AUTOSDV_HOME="${AUTOSDV_HOME:-/opt/AutoSDV}"
WORKSPACE="${WORKSPACE:-/workspace}"
DISPLAY_NUM="${DISPLAY_NUM:-1}"
export DISPLAY=":${DISPLAY_NUM}"
GEOMETRY="${GEOMETRY:-1920x1080}"
NOVNC_PORT="${NOVNC_PORT:-6080}"

# The account everything ends up running as. Created at build time with UID
# 1000, which is what the overwhelming majority of single-user Linux laptops
# hand out first, so the common case needs no adjustment at all.
CONTAINER_USER="${CONTAINER_USER:-autosdv}"
HOST_UID="${HOST_UID:-1000}"
HOST_GID="${HOST_GID:-1000}"

say() { printf '  %s\n' "$*"; }

# --- who the container runs as ----------------------------------------------
# Everything the student creates through this container -- colcon's build/,
# install/ and log/, a recorded rosbag, a file their editor did not make -- is
# written into a bind mount that belongs to their own account on the host. A
# container that writes as root leaves those root-owned inside the student's own
# git checkout: their editor cannot save, `rm -rf build` needs sudo, and git
# refuses outright with
#
#   fatal: detected dubious ownership in repository at '/workspace'
#
# So the container user is bent to fit the host's UID/GID, not the other way
# round. This matters on Linux and is a harmless no-op on macOS (VirtioFS maps
# ownership to the calling user) and Windows (drvfs masks it), where PowerShell
# cannot report an id anyway and the launcher sends 1000.
#
# NOT `docker run --user`: a UID with no /etc/passwd entry gets `I have no
# name!` at the prompt, an unwritable $HOME, and no ~/.ros to log into.
#
# The MOUNT is never chowned. Matching the UID is precisely what makes a chown
# unnecessary, and rewriting the ownership of somebody's checkout from inside a
# container is not a thing this should ever do.
reconcile_user() {
    if [ "$(id -u)" -ne 0 ]; then
        say "user: $(id -un) (not root; leaving ids alone)"
        return
    fi
    if [ "$HOST_UID" = "0" ]; then
        # A root host, or a caller that asked for root outright.
        CONTAINER_USER=root
        say "user: root (HOST_UID=0)"
        return
    fi

    local cur_uid cur_gid
    cur_uid="$(id -u "$CONTAINER_USER" 2>/dev/null || echo "")"
    cur_gid="$(id -g "$CONTAINER_USER" 2>/dev/null || echo "")"
    if [ -z "$cur_uid" ]; then
        say "user: ${CONTAINER_USER} is missing from this image; staying root"
        CONTAINER_USER=root
        return
    fi

    if [ "$cur_gid" != "$HOST_GID" ]; then
        groupmod -o -g "$HOST_GID" "$CONTAINER_USER" 2>/dev/null || true
    fi
    if [ "$cur_uid" != "$HOST_UID" ]; then
        usermod -o -u "$HOST_UID" "$CONTAINER_USER" 2>/dev/null || true
    fi

    # Only $HOME, which is small and ours. usermod does not follow the account
    # when the uid moves underneath it.
    local home
    home="$(getent passwd "$CONTAINER_USER" | cut -d: -f6)"
    if [ -n "$home" ] && [ -d "$home" ]; then
        chown -R "$HOST_UID:$HOST_GID" "$home" 2>/dev/null || true
    fi

    say "user: ${CONTAINER_USER} (uid ${HOST_UID}, gid ${HOST_GID})"
}

# TurboVNC writes into $HOME/.vnc, so the session belongs to whoever will be
# using it rather than to root. The xstartup that the image ships lives in
# root's home because that is where the build could put it.
install_vnc_session() {
    local home
    home="$(getent passwd "$CONTAINER_USER" | cut -d: -f6)"
    [ -n "$home" ] || return 0
    if [ "$home" = "/root" ]; then
        return 0
    fi
    mkdir -p "$home/.vnc"
    cp /root/.vnc/xstartup.turbovnc "$home/.vnc/xstartup.turbovnc"
    chmod +x "$home/.vnc/xstartup.turbovnc"
    chown -R "$HOST_UID:$HOST_GID" "$home/.vnc"
    VNC_HOME="$home"
}

# --- the check that catches the failure nobody can diagnose -----------------
# net.core.rmem_max is not per-namespace, so it comes from the host. Below about
# 10 MB CycloneDDS cannot open its sockets and NOT ONE ROS 2 node starts -- with
# an error that says nothing about buffers.
check_dds_buffers() {
    local want=10485760 have
    # net.core.rmem_max is NOT namespaced, so inside a container it is neither
    # readable nor settable: /proc/sys/net/core holds only the namespaced
    # entries. `docker run --sysctl net.core.rmem_max=...` does not merely fail
    # to help, it stops the container starting at all:
    #   open /proc/sys/net/core/rmem_max: no such file or directory
    # The value that governs these sockets belongs to whatever kernel is
    # underneath -- the Linux host, or Docker Desktop's virtual machine.
    if [ ! -r /proc/sys/net/core/rmem_max ]; then
        say "DDS socket buffers: set by the host kernel, not visible from here"
        return
    fi
    have=$(cat /proc/sys/net/core/rmem_max)
    if [ "$have" -lt "$want" ]; then
        cat >&2 <<EOF

  ============================================================
  net.core.rmem_max is ${have}, which is below the ~10 MB
  CycloneDDS needs. No ROS 2 node will start.

  Fix it on the HOST, not here:

      sudo sysctl -w net.core.rmem_max=2147483647
      sudo sysctl -w net.core.wmem_max=2147483647

  (AutoSDV hosts: ./setup.sh --run --only cyclonedds-sysctl)
  ============================================================

EOF
    fi
}

# Nodes on one machine find each other over loopback multicast, and lo comes up
# without the MULTICAST flag. On the host a systemd unit fixes this; here we
# just set it, and carry on without if we lack NET_ADMIN.
enable_loopback_multicast() {
    if ip link set lo multicast on 2>/dev/null; then
        say "loopback multicast: on"
    else
        say "loopback multicast: could not set (no NET_ADMIN) -- add --cap-add=NET_ADMIN if nodes cannot see each other"
    fi
}

# --- pick a renderer --------------------------------------------------------
# Reports what it chose, so a student asking for help can say which path they
# are on rather than describing the symptom.
select_renderer() {
    if [ -e /dev/dxg ]; then
        # Windows, WSL2: Mesa's d3d12 driver forwards GL to the host GPU
        # through /dev/dxg. VirtualGL is not involved.
        export LIBGL_ALWAYS_SOFTWARE=0
        [ -d /usr/lib/wsl/lib ] && export LD_LIBRARY_PATH="/usr/lib/wsl/lib:${LD_LIBRARY_PATH:-}"
        RENDER_WRAPPER=()
        say "graphics: WSL2 GPU via Mesa d3d12"
    elif ls /dev/nvidia* >/dev/null 2>&1; then
        export LIBGL_ALWAYS_SOFTWARE=0
        RENDER_WRAPPER=(vglrun -d egl)
        say "graphics: NVIDIA GPU via VirtualGL"
    elif [ -d /dev/dri ]; then
        export LIBGL_ALWAYS_SOFTWARE=0
        RENDER_WRAPPER=(vglrun -d /dev/dri/renderD128)
        say "graphics: GPU via VirtualGL (/dev/dri)"
    else
        # macOS lands here always: Hypervisor.framework exposes no vGPU, so
        # there is nothing to accelerate onto. Measured at 31 fps with
        # rviz/workshop.rviz, which is why that config exists.
        export LIBGL_ALWAYS_SOFTWARE=1
        RENDER_WRAPPER=()
        say "graphics: software (llvmpipe) -- use rviz/workshop.rviz, not autosdv.rviz"
    fi
    # Make the choice available to anything started later in the session.
    printf '%s\n' "${RENDER_WRAPPER[@]:-}" > /run/autosdv-render-wrapper
    if [ ${#RENDER_WRAPPER[@]} -gt 0 ]; then
        printf '#!/usr/bin/env bash\nexec %s "$@"\n' "${RENDER_WRAPPER[*]}" > /usr/local/bin/gl
    else
        printf '#!/usr/bin/env bash\nexec "$@"\n' > /usr/local/bin/gl
    fi
    chmod +x /usr/local/bin/gl
}

# `docker exec` starts a NEW process that inherits nothing from here: not
# DISPLAY, and not the location of the Xauthority file Xvnc was given. It also
# lands as ROOT, whose $HOME is not where that file lives -- so the obvious
# command reports
#
#   No protocol specified
#   Error: unable to open display :9
#
# which reads as broken graphics and is an authority lookup that never happened.
# The container's own shells are fine; only a second window into it is affected,
# and that is exactly how anyone verifies the display from outside.
#
# So the values are written where any login shell picks them up. `docker exec
# <c> bash -lc '<anything X>'` then works as-is, for root and for the user
# alike, with the paths this run actually resolved rather than paths baked in
# at build time -- CONTAINER_USER and its home are both run-time answers.
publish_display() {
    local home auth
    home="$(getent passwd "$CONTAINER_USER" | cut -d: -f6)"
    auth="${home:-/root}/.Xauthority"
    cat > /etc/profile.d/autosdv-display.sh <<EOF
# Written by the AutoSDV entrypoint. The display this container is running.
export DISPLAY="${DISPLAY}"
export XAUTHORITY="${auth}"
EOF
    chmod 0644 /etc/profile.d/autosdv-display.sh

    # /etc/profile.d is read by LOGIN shells only, and the shell a student
    # actually gets is not one: autosdv.sh runs `docker exec ... bash -c`,
    # whose last act is `exec bash` -- interactive, non-login, which reads
    # /etc/bash.bashrc instead. Without this line the second terminal has the
    # image's build-time DISPLAY=:1 and no XAUTHORITY, which is right only
    # while DISPLAY_NUM is the default and wrong the moment anyone overrides
    # it.
    #
    # Still not covered, and not worth more machinery: `bash -c` without -i or
    # -l reads neither file. That is a shape only verification commands take,
    # and they can pass -l.
    if ! grep -q autosdv-display /etc/bash.bashrc 2>/dev/null; then
        printf '\n[ -f /etc/profile.d/autosdv-display.sh ] && . /etc/profile.d/autosdv-display.sh\n' \
            >> /etc/bash.bashrc
    fi
}

start_desktop() {
    # `as_user` is empty when we are already the right account, so a root-only
    # container (HOST_UID=0, or an image with no autosdv user) behaves exactly
    # as it did before any of this existed.
    local as_user=()
    if [ "$(id -u)" -eq 0 ] && [ "$CONTAINER_USER" != "root" ]; then
        as_user=(gosu "$CONTAINER_USER")
    fi
    local xstartup="${VNC_HOME:-/root}/.vnc/xstartup.turbovnc"

    "${as_user[@]}" /opt/TurboVNC/bin/vncserver -kill "$DISPLAY" >/dev/null 2>&1 || true
    rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}" 2>/dev/null || true

    # No -localhost argument: TurboVNC's is a boolean, so `-localhost no`
    # is parsed as a stray option and Xvnc dies with
    #   Fatal server error: Unrecognized option: no
    # Keeping Xvnc on localhost is also correct: websockify runs inside this
    # container and connects locally, and only 6080 is published.
    #
    # -xstartup is explicit because TurboVNC runs its OWN
    # /opt/TurboVNC/bin/xstartup.turbovnc otherwise, and that one kills Xvnc
    # when it cannot find a session file for a window manager it knows.
    "${as_user[@]}" /opt/TurboVNC/bin/vncserver "$DISPLAY" \
        -geometry "$GEOMETRY" -depth 24 \
        -SecurityTypes None \
        -xstartup "$xstartup" \
        >/var/log/vncserver.log 2>&1

    websockify -D --web=/usr/share/novnc "${NOVNC_PORT}" "localhost:590${DISPLAY_NUM}" \
        >/var/log/websockify.log 2>&1

    publish_display

    say "middleware: ${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (DEFAULT -- a second terminal will not see the stack)}"
    say "desktop: http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=remote"
}

echo
echo "  AutoSDV desktop"
echo "  ---------------"
reconcile_user
install_vnc_session
check_dds_buffers
enable_loopback_multicast
select_renderer
start_desktop
echo

# The environment the book teaches: Autoware first, then the workspace overlay.
#
# The overlay is conditional because the `base` target ships no built
# workspace -- that is the whole point of it. Sourcing a file that is not there
# under `set -e` would kill the container at startup with a message about a
# path, which tells a student nothing about which image they are running.
set +u
source /opt/autoware/1.5.0/setup.bash
if [ -f "${AUTOSDV_HOME}/install/setup.bash" ]; then
    source "${AUTOSDV_HOME}/install/setup.bash"
else
    say "workspace: none built into this image (base); yours is under ${WORKSPACE}"
fi
set -u

# Start where the work is. A bind-mounted checkout is the reason this container
# exists for anything but the workshop, so it wins when it is there.
if [ -d "${WORKSPACE}" ]; then
    cd "${WORKSPACE}"
else
    cd "${AUTOSDV_HOME}"
fi

# Sourcing happened as root, which costs nothing: what it produced is exported
# environment, and gosu carries the environment across.
if [ "$(id -u)" -eq 0 ] && [ "$CONTAINER_USER" != "root" ]; then
    exec gosu "$CONTAINER_USER" "$@"
fi
exec "$@"
