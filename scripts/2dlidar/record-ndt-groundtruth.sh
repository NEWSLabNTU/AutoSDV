#!/usr/bin/env bash
# scripts/2dlidar/record-ndt-groundtruth.sh — Phase 3 Task 2 gate:
# bring up logging_simulation with standard NDT (pose_source:=ndt), replay the
# outdoor bag, and record /localization/kinematic_state (map-frame Odometry)
# as ground truth for later particle-filter comparison (Task 3/4).
#
# Env overrides: BAG, OUT, RATE, MAP_DIR, STARTUP_TIMEOUT.
#
# --- Hard-won facts (see .superpowers/sdd/p3-task-2-report.md for the full
# attempt-by-attempt log) ---
#
# 1. `pose_source:=ndt` does NOT select upstream Autoware's ndt_scan_matcher.
#    logging_simulation.launch.yaml defaults `pose_source_package` to
#    `cuda_ndt_matcher_launch` regardless of `pose_source`, so the node named
#    `ndt_scan_matcher` in the graph is actually AutoSDV's own Rust
#    `cuda_ndt_matcher` (src/localization/cuda_ndt_matcher). That's fine and
#    expected — this script targets that node.
#
# 2. `data/COSS-map-planning/` ships only a single, non-divided
#    pointcloud_map.pcd with no pointcloud_map_metadata.yaml. The map loader
#    (autoware_map_loader / pointcloud_map_loader) needs a metadata file to
#    serve partial/differential map queries even when whole-map load is also
#    enabled; without it, dynamic map loading is a permanent no-op (not a
#    timing flake). This script generates a minimal single-cell metadata file
#    covering the whole map if one isn't already present. NOTE: cell
#    coordinates in this metadata format must be YAML integers — floats
#    crash the loader with `YAML::TypedBadConversion<int>`.
#
# 3. gnss_poser cannot resolve the TF frame named in the bag's NavSatFix
#    messages ("gps") to base_link (the AutoSDV sensor_kit calibration only
#    defines "gnss_base_link"). Without that transform, GNSS pose is never
#    published and pose_initializer's automatic GNSS-seeded retry loop has
#    nothing to seed from. This script publishes a static identity TF
#    base_link -> gps for the duration of the run as a workaround (a few cm
#    of GNSS antenna offset is irrelevant for a rough NDT initial guess).
#
# 4. cuda_ndt_matcher's map subscription used the rclrs default QoS
#    (RELIABLE, VOLATILE). The pointcloud_map_loader publishes the whole map
#    exactly once, latched (TRANSIENT_LOCAL). A VOLATILE subscriber never
#    receives a TRANSIENT_LOCAL publisher's already-sent sample, so NDT align
#    failed forever with "No map loaded" regardless of startup ordering or
#    retries. Fixed upstream in
#    src/localization/cuda_ndt_matcher/src/cuda_ndt_matcher/src/node/init.rs
#    (map subscription now requests `.transient_local()`); that package must
#    be rebuilt (`colcon build --packages-select cuda_ndt_matcher`) before
#    this script can succeed. This script does NOT rebuild it for you.
#
# 5. Initial pose acquisition is fully automatic: autoware_pose_initializer
#    retries "Call align server" on its own (GNSS-seeded, AUTO method) once
#    the vehicle is detected stationary and GNSS is available — no manual
#    /initialpose publish needed, PROVIDED record+replay start soon
#    (seconds, not minutes) after the launch reports "Startup complete".
#    In testing, the first successful align landed ~100-150s after replay
#    start, close to (but independent of) the outdoor bag's own ~116s
#    stationary-to-moving transition.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_DIR"

BAG="${BAG:-$REPO_DIR/data/rosbags/outdoor_20251226_153115}"
OUT="${OUT:-$REPO_DIR/data/rosbags/phase3/ndt_gt}"
RATE="${RATE:-1.0}"
MAP_DIR="${MAP_DIR:-$REPO_DIR/data/COSS-map-planning}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-180}"
KINEMATIC_STATE_THRESHOLD=400

LAUNCH_LOG="$(mktemp -t record-ndt-groundtruth-launch.XXXXXX.log)"
RECORD_LOG="$(mktemp -t record-ndt-groundtruth-record.XXXXXX.log)"

LAUNCH_PID=""
GPS_TF_PID=""
RECORD_PID=""

cleanup() {
    if [ -n "$RECORD_PID" ]; then
        kill -INT "$RECORD_PID" 2>/dev/null || true
        # give rosbag2 a moment to flush and close the sqlite3 db cleanly
        sleep 3
    fi
    if [ -n "$GPS_TF_PID" ]; then
        kill -9 "$GPS_TF_PID" 2>/dev/null || true
    fi
    if [ -n "$LAUNCH_PID" ]; then
        LAUNCH_PGID="$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ')" || LAUNCH_PGID=""
        if [ -n "$LAUNCH_PGID" ]; then
            kill -- -"$LAUNCH_PGID" 2>/dev/null || true
            sleep 5
            kill -9 -- -"$LAUNCH_PGID" 2>/dev/null || true
        fi
    fi
}
trap cleanup EXIT

# --- Step 0: ensure the pointcloud map metadata file exists (see fact #2) ---
METADATA_PATH="$MAP_DIR/pointcloud_map_metadata.yaml"
if [ ! -f "$METADATA_PATH" ]; then
    echo "Generating missing $METADATA_PATH (single-cell, covers whole COSS map)"
    cat > "$METADATA_PATH" <<'EOF'
x_resolution: 300.0
y_resolution: 300.0
pointcloud_map.pcd: [-150, -150]
EOF
fi

# shellcheck disable=SC1091
set +u
source /opt/autoware/1.5.0/setup.bash
source "$REPO_DIR/install/setup.bash"
set -u

# --- Step 1: launch logging_simulation (localization-only: perception off) ---
setsid play_launch launch autosdv_launch logging_simulation.launch.yaml \
    rviz:=false pose_source:=ndt launch_perception:=false \
    > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

# --- Step 2: workaround for fact #3 (gnss_poser TF frame mismatch) ---
setsid ros2 run tf2_ros static_transform_publisher \
    --frame-id base_link --child-frame-id gps --x 0 --y 0 --z 0 \
    > /dev/null 2>&1 &
GPS_TF_PID=$!

echo "Waiting for launch startup to complete (timeout ${STARTUP_TIMEOUT}s)..."
elapsed=0
while ! grep -q "Startup complete" "$LAUNCH_LOG" 2>/dev/null; do
    if [ "$elapsed" -ge "$STARTUP_TIMEOUT" ]; then
        echo "FAIL: launch did not report startup completion within ${STARTUP_TIMEOUT}s"
        echo "See $LAUNCH_LOG"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
echo "Startup complete after ~${elapsed}s"

# --- Step 3: start recording immediately (fact #5: auto-init window matters) ---
rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"
setsid ros2 bag record -o "$OUT" \
    /localization/kinematic_state /tf /tf_static \
    > "$RECORD_LOG" 2>&1 &
RECORD_PID=$!
sleep 2

# --- Step 4: replay the outdoor bag (foreground; ~157s at rate 1.0) ---
echo "Replaying $BAG at rate $RATE..."
ros2 bag play "$BAG" --clock -r "$RATE"
echo "Replay finished."

# --- Step 5: stop recording cleanly ---
kill -INT "$RECORD_PID" 2>/dev/null || true
sleep 3
RECORD_PID=""

# --- Step 6: verify ---
INFO="$(ros2 bag info "$OUT" 2>&1)" || INFO=""
echo "$INFO"
KINEMATIC_COUNT="$(echo "$INFO" | awk -F'Count: ' '/\/localization\/kinematic_state/ {split($2,a," "); print a[1]}')"
KINEMATIC_COUNT="${KINEMATIC_COUNT:-0}"

if [ "$KINEMATIC_COUNT" -gt "$KINEMATIC_STATE_THRESHOLD" ]; then
    echo "PASS: kinematic_state count=$KINEMATIC_COUNT (> $KINEMATIC_STATE_THRESHOLD) -- $OUT"
    exit 0
else
    echo "FAIL: kinematic_state count=$KINEMATIC_COUNT (<= $KINEMATIC_STATE_THRESHOLD)"
    echo "Launch log: $LAUNCH_LOG"
    echo "Record log: $RECORD_LOG"
    exit 1
fi
