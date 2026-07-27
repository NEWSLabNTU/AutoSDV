#!/usr/bin/env bash
# Bring the nav2 map_server lifecycle node (/map_server) through
# configure -> activate.
#
# map_server is a lifecycle node: it does nothing until driven through these
# transitions. A single `ros2 lifecycle set` attempt races the node's rclcpp
# init / service advertisement, so retry each transition with a short sleep
# between attempts -- same pattern as scripts/2dlidar/check-map-load.sh.
#
# Invoked from autosdv_map_component.launch.xml as a plain node/executable
# alongside the map_server node it brings up.
set -u

lifecycle_set_retry() {
    local transition="$1"
    local expect_state="$2"
    local attempt
    for attempt in 1 2 3 4 5 6 7 8 9 10; do
        # `ros2 lifecycle set` can report success at the CLI/service-call
        # level even when the transition callback itself failed (map_server
        # then sits in an error/broken state) -- so verify the resulting
        # state directly instead of trusting the command's exit code alone.
        ros2 lifecycle set /map_server "$transition" || true
        if ros2 lifecycle get /map_server 2>/dev/null | grep -q "^${expect_state} "; then
            return 0
        fi
        sleep 2
    done
    echo "map_server_lifecycle_bringup: FAIL could not bring /map_server to '$expect_state' via '$transition' after retries; last state: $(ros2 lifecycle get /map_server 2>&1)" >&2
    return 1
}

lifecycle_set_retry configure inactive || exit 1
lifecycle_set_retry activate active || exit 1
echo "map_server_lifecycle_bringup: /map_server configured and activated"
