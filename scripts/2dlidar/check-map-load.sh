#!/usr/bin/env bash
# scripts/2dlidar/check-map-load.sh — Phase 1 standalone gate:
# nav2 map_server must load the generated grid and publish /map once.
# Usage: check-map-load.sh [path/to/occupancy_grid.yaml]
set -euo pipefail
MAP_YAML="${1:-data/COSS-map-planning/occupancy_grid.yaml}"

# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
set -u

setsid ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$MAP_YAML" -p use_sim_time:=false &
SERVER_PID=$!
trap 'kill -- -"$SERVER_PID" 2>/dev/null || true' EXIT
sleep 3

# map_server is a lifecycle node: configure + activate.
# ros2 lifecycle set can race a slow-starting node, so retry a few times.
lifecycle_set_retry() {
    local transition="$1"
    local attempt
    for attempt in 1 2 3 4 5; do
        if ros2 lifecycle set /map_server "$transition"; then
            return 0
        fi
        sleep 2
    done
    echo "FAIL: could not set /map_server to $transition after retries"
    exit 1
}

lifecycle_set_retry configure
lifecycle_set_retry activate

# map_server publishes /map latched (transient_local); match QoS or echo hangs.
INFO=$(timeout 10 ros2 topic echo /map --once --field info \
    --qos-durability transient_local --qos-reliability reliable 2>/dev/null) || INFO=""
echo "$INFO"
WIDTH=$(echo "$INFO" | awk '/^width:/ {print $2}')
if [ -z "$WIDTH" ] || [ "$WIDTH" -le 0 ]; then
    echo "FAIL: /map not published or empty"; exit 1
fi
echo "PASS: map loads, width=${WIDTH} cells"
