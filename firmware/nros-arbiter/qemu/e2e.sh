#!/usr/bin/env bash
# One end-to-end run of the arbiter image in QEMU:
#
#   QEMU mps2-an521 uart1 <-> socat pty pair <-> rmw_zenohd (serial listener)
#                                                     ^
#                                  ROS 2 subscriber --+ (rmw_zenoh_cpp, tcp)
#
#   e2e.sh <zephyr.elf> <run dir> <seconds>
#
# The router listens on 127.0.0.1:7457, not ROS's 7447, so a router already
# running on this machine is neither used nor disturbed. Logs land in the run
# dir: qemu.log (the board's console), router.log, sub.log. Exits non-zero
# when the subscriber saw no message.
set -euo pipefail

ELF="$1"
RUN="$2"
SECS="$3"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_CFG=/opt/ros/humble/share/rmw_zenoh_cpp/config

die() { echo "e2e.sh: $*" >&2; exit 1; }
command -v socat >/dev/null || die "socat not found"
[ -d "$ROS_CFG" ] || die "no $ROS_CFG: install ros-humble-rmw-zenoh-cpp"
QEMU="$(ls -d "$HOME"/.nros/sdk/qemu/*/bin 2>/dev/null | sort -V | tail -1)/qemu-system-arm"
[ -x "$QEMU" ] || QEMU="$(command -v qemu-system-arm)" || die "qemu-system-arm not found"

mkdir -p "$RUN"
rm -f "$RUN"/router-pty "$RUN"/qemu-pty "$RUN"/*.log

# The stock rmw_zenoh configs with the endpoints changed: the router adds a
# serial listener on the pty, the subscriber connects to the router's port.
sed 's|"tcp/\[::\]:7447"|"tcp/127.0.0.1:7457", "serial/'"$RUN"'/router-pty#baudrate=115200"|' \
    "$ROS_CFG/DEFAULT_RMW_ZENOH_ROUTER_CONFIG.json5" >"$RUN/router.json5"
sed 's|"tcp/localhost:7447"|"tcp/127.0.0.1:7457"|' \
    "$ROS_CFG/DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5" >"$RUN/session.json5"

pids=()
cleanup() { kill "${pids[@]}" 2>/dev/null || true; }
trap cleanup EXIT

socat pty,raw,echo=0,link="$RUN/router-pty" pty,raw,echo=0,link="$RUN/qemu-pty" 2>"$RUN/socat.log" &
pids+=($!)
for _ in $(seq 50); do [ -e "$RUN/qemu-pty" ] && break; sleep 0.1; done
[ -e "$RUN/qemu-pty" ] || die "socat made no pty"

# ROS 2 Humble is Python 3.10: the system interpreter, never the Zephyr
# build's 3.12 venv that ws.sh puts on PATH. ROS's setup hooks read unset
# variables.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=0

# The router first: the board gives up after about 3 s without one.
ZENOH_ROUTER_CONFIG_URI="$RUN/router.json5" ros2 run rmw_zenoh_cpp rmw_zenohd >"$RUN/router.log" 2>&1 &
pids+=($!)
sleep 3

ZENOH_SESSION_CONFIG_URI="$RUN/session.json5" \
    /usr/bin/python3 "$HERE/sub.py" "$((SECS + 2))" >"$RUN/sub.log" 2>&1 &
sub=$!
pids+=($sub)

echo "e2e.sh: running $SECS s; console in $RUN/qemu.log"
timeout "$SECS" "$QEMU" -M mps2-an521 -nographic -kernel "$ELF" \
    -serial mon:stdio -serial "$(readlink -f "$RUN/qemu-pty")" -icount shift=auto \
    </dev/null >"$RUN/qemu.log" 2>&1 || true

wait "$sub" || true
echo "--- board"
grep -a 'arbiter:' "$RUN/qemu.log" | sed 's/\x1b\[[0-9;]*m//g' || true
echo "--- ROS 2 subscriber"
cat "$RUN/sub.log"
grep -q '^messages: [1-9]' "$RUN/sub.log" || die "the subscriber received nothing"
