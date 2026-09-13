#!/usr/bin/env bash
# Start the desktop, then hand over to the command.
#
# The whole point of this file is that the student's run command is the same on
# every host. What differs between hosts is which renderer is reachable, and
# that is decided here rather than in the instructions.

set -euo pipefail

AUTOSDV_HOME="${AUTOSDV_HOME:-/opt/AutoSDV}"
DISPLAY_NUM="${DISPLAY_NUM:-1}"
export DISPLAY=":${DISPLAY_NUM}"
GEOMETRY="${GEOMETRY:-1920x1080}"
NOVNC_PORT="${NOVNC_PORT:-6080}"

say() { printf '  %s\n' "$*"; }

# --- the check that catches the failure nobody can diagnose -----------------
# net.core.rmem_max is not per-namespace, so it comes from the host. Below about
# 10 MB CycloneDDS cannot open its sockets and NOT ONE ROS 2 node starts -- with
# an error that says nothing about buffers.
check_dds_buffers() {
    local want=10485760 have
    have=$(cat /proc/sys/net/core/rmem_max 2>/dev/null || echo 0)
    if [ "$have" -lt "$want" ]; then
        cat >&2 <<EOF

  ============================================================
  net.core.rmem_max is ${have}, which is below the ~10 MB
  CycloneDDS needs. No ROS 2 node will start.

  This is a HOST setting; a container cannot change it. Re-run
  with:

      docker run --sysctl net.core.rmem_max=2147483647 \\
                 --sysctl net.core.wmem_max=2147483647 ...

  (docker compose: see compose.yaml, which sets both.)
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

start_desktop() {
    /opt/TurboVNC/bin/vncserver -kill "$DISPLAY" >/dev/null 2>&1 || true
    rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}" 2>/dev/null || true

    /opt/TurboVNC/bin/vncserver "$DISPLAY" \
        -geometry "$GEOMETRY" -depth 24 \
        -SecurityTypes None -localhost no \
        >/var/log/vncserver.log 2>&1

    openbox --sm-disable >/var/log/openbox.log 2>&1 &

    websockify -D --web=/usr/share/novnc "${NOVNC_PORT}" "localhost:590${DISPLAY_NUM}" \
        >/var/log/websockify.log 2>&1

    say "desktop: http://localhost:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=remote"
}

echo
echo "  AutoSDV desktop"
echo "  ---------------"
check_dds_buffers
enable_loopback_multicast
select_renderer
start_desktop
echo

# The environment the book teaches: Autoware first, then the workspace overlay.
set +u
source /opt/autoware/1.5.0/setup.bash
source "${AUTOSDV_HOME}/install/setup.bash"
set -u
cd "${AUTOSDV_HOME}"

exec "$@"
