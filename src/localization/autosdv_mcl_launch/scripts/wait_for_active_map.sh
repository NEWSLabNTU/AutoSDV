#!/usr/bin/env bash
# wait_for_active_map.sh -- Task 8 launch-order fix.
#
# particle_filter's own GetMap client (particle_filter.py's get_omap) only
# waits for the /map_server/map SERVICE to exist -- it does not check that
# nav2_map_server has actually reached the ACTIVE lifecycle state. Under the
# full stack, autosdv_map_component.launch.xml (Task 1) and this package's
# mcl_localization.launch.xml are included as launch-tree siblings with no
# ordering guarantee, so particle_filter routinely starts and calls GetMap
# before nav2_map_server's own configure+activate retry script
# (map_server_lifecycle_bringup.sh) finishes. A GetMap call against an
# inactive map_server returns an EMPTY OccupancyGrid (resolution 0.0), which
# particle_filter then divides by in get_omap() -> ZeroDivisionError,
# crashing the node -- confirmed empirically during Task 8 end-to-end
# verification.
#
# Used as this node's launch-prefix (see mcl_localization.launch.xml):
# polls /map_server/map with a GetMap request whose response resolution is
# nonzero as the "really active" signal, then execs the real command.
#
# BUG FOUND DURING TASK 8 END-TO-END DIAGNOSIS (2026-07-27): each poll
# attempt below had no bound of its own -- only the OUTER elapsed/TIMEOUT_S
# loop did, and that counter only advances after a call RETURNS. Against a
# large grid (confirmed with a 9788x8358-cell, 0.05 m/cell occupancy grid),
# a single `ros2 service call` can take far longer than expected to
# marshal/deserialize the GetMap response, or effectively hang; when that
# happens this loop never advances and the timeout can never fire, so
# particle_filter never starts at all -- confirmed empirically: the launch
# tree showed particle_filter's process running (via wait_for_active_map),
# but with a completely empty log (0 lines, not even the parameter
# deprecation warnings the real particle_filter binary always prints at
# startup) and zero messages ever on /pf/pose/odom or /pf/viz/inferred_pose.
# A single `timeout` per attempt below fixes it -- each individual call is
# now itself bounded, so a hang can no longer starve the outer loop.
set -euo pipefail

TIMEOUT_S="${MCL_MAP_WAIT_TIMEOUT_S:-60}"
PER_CALL_TIMEOUT_S="${MCL_MAP_WAIT_PER_CALL_TIMEOUT_S:-10}"
START_S="$SECONDS"  # bash builtin: real elapsed seconds since shell start
while true; do
    RESULT="$(timeout "$PER_CALL_TIMEOUT_S" ros2 service call /map_server/map nav_msgs/srv/GetMap "{}" 2>/dev/null)" || RESULT=""
    if echo "$RESULT" | grep -q "resolution=0.0" ; then
        RESULT=""  # inactive: empty map, resolution is exactly 0.0
    fi
    if [ -n "$RESULT" ] && echo "$RESULT" | grep -q "resolution="; then
        echo "wait_for_active_map: /map_server/map is serving a real map, starting $*"
        break
    fi
    if [ "$((SECONDS - START_S))" -ge "$TIMEOUT_S" ]; then
        echo "wait_for_active_map: timed out after ${TIMEOUT_S}s waiting for /map_server/map to become active; starting $* anyway"
        break
    fi
    sleep 1
done

exec "$@"
