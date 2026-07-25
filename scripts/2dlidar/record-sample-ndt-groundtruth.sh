#!/usr/bin/env bash
# scripts/2dlidar/record-sample-ndt-groundtruth.sh — Phase 3b Task 2 gate:
# bring up the STOCK Autoware tutorial logging_simulator (not AutoSDV's
# logging_simulation.launch.yaml, not play_launch) against the sample
# map/vehicle/sensor kit, replay the migrated Autoware sample rosbag, and
# record NDT-localized ground truth (/localization/kinematic_state) plus the
# decoded top-LiDAR pointcloud, IMU, velocity, and TF for later
# particle-filter comparison (Task 3/4).
#
# Env overrides: BAG, OUT, RATE, MAP_DIR, STARTUP_TIMEOUT.
#
# --- Hard-won facts (see .superpowers/sdd/p3b-task-2-report.md for the full
# attempt-by-attempt log) ---
#
# 1. Use the STOCK `autoware_launch/logging_simulator.launch.xml`, sourcing
#    ONLY /opt/autoware/1.5.0/setup.bash (do NOT also source this repo's
#    install/setup.bash — AutoSDV's tier4_localization_launch override is
#    tuned for the AutoSDV/COSS map and can conflict with the stock sample
#    map/vehicle/sensor_kit used here).
#
# 2. `perception:=false planning:=false control:=false rviz:=false` are real
#    top-level args on logging_simulator.launch.xml (confirmed by reading
#    /opt/autoware/1.5.0/share/autoware_launch/launch/logging_simulator.launch.xml).
#    Localization + sensing + vehicle + map + system stay on. `use_sim_time`
#    is hardcoded to true inside that launch file (not an arg) — no need to
#    pass it.
#
# 3. logging_simulator.launch.xml has no "Startup complete" log marker (that
#    string is AutoSDV-specific, from autosdv.launch.yaml / play_launch).
#    Readiness is instead detected by polling `ros2 node list` for
#    `/localization/pose_estimator/ndt_scan_matcher`, which only appears once
#    localization bring-up has finished composing nodes.
#
# 4. The decoded top-LiDAR pointcloud topic is discovered at runtime (not
#    hardcoded) via `ros2 topic list -t`, matching `sensing/lidar/top` and
#    `sensor_msgs/msg/PointCloud2`, preferring a name containing
#    `pointcloud_raw_ex` (the un-filtered decoded cloud) if present. In
#    testing against the sample_sensor_kit this resolved to
#    `/sensing/lidar/top/pointcloud_raw_ex`. This topic is already advertised
#    right after launch startup (before any bag data flows), so discovery
#    happens before recording starts.
#
# 5. The migrated sample bag (data/sample-rosbag-replay/sample-rosbag-migrated)
#    already contains its own /clock (2941 msgs), so it is replayed WITHOUT
#    `--clock` — the bag supplies sim time itself. `use_sim_time:=true` comes
#    from the stock launch file by default.
#
# 6. GNSS auto-init worked out of the box with the stock sample_sensor_kit
#    (unlike AutoSDV's own outdoor-bag NDT ground truth capture, which needed
#    a manual base_link->gps static TF workaround for its own sensor_kit
#    calibration). autoware_pose_initializer's automatic GNSS-seeded retry
#    activated NDT ~4s after bag replay started, no manual /initialpose
#    publish needed. This script does not add the TF workaround; if a future
#    Autoware/sensor_kit revision breaks auto-init, add a static
#    base_link->gps identity TF publisher here (see record-ndt-groundtruth.sh
#    fact #3 for the pattern).
#
# 7. First attempt succeeded end-to-end at rate 0.5 (bag is only ~30s, so
#    replay is cheap — ~72s wall-clock old NDT ground truth at rate 0.5,
#    including robot_state_publisher/launch startup lead time before replay
#    starts). No fallback to rate 0.2 was needed.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_DIR"

BAG="${BAG:-$REPO_DIR/data/sample-rosbag-replay/sample-rosbag-migrated}"
OUT="${OUT:-$REPO_DIR/data/rosbags/phase3/sample_ndt_gt}"
RATE="${RATE:-0.5}"
MAP_DIR="${MAP_DIR:-$REPO_DIR/data/sample-rosbag-replay/sample-map-rosbag}"
STARTUP_TIMEOUT="${STARTUP_TIMEOUT:-180}"
KINEMATIC_STATE_THRESHOLD=200
POINTCLOUD_THRESHOLD=100

LAUNCH_LOG="$(mktemp -t record-sample-ndt-groundtruth-launch.XXXXXX.log)"
RECORD_LOG="$(mktemp -t record-sample-ndt-groundtruth-record.XXXXXX.log)"
PLAY_LOG="$(mktemp -t record-sample-ndt-groundtruth-play.XXXXXX.log)"

LAUNCH_PID=""
RECORD_PID=""

cleanup() {
    if [ -n "$RECORD_PID" ]; then
        kill -INT "$RECORD_PID" 2>/dev/null || true
        # give rosbag2 a moment to flush and close the sqlite3 db cleanly
        sleep 3
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

# shellcheck disable=SC1091
set +u
source /opt/autoware/1.5.0/setup.bash
set -u

# --- Step 1: launch the STOCK tutorial logging_simulator (localization-only) ---
setsid ros2 launch autoware_launch logging_simulator.launch.xml \
    map_path:="$MAP_DIR" vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit \
    perception:=false planning:=false control:=false rviz:=false \
    > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

echo "Waiting for localization bring-up (ndt_scan_matcher node, timeout ${STARTUP_TIMEOUT}s)..."
elapsed=0
while ! ros2 node list 2>/dev/null | grep -q "/localization/pose_estimator/ndt_scan_matcher"; do
    if [ "$elapsed" -ge "$STARTUP_TIMEOUT" ]; then
        echo "FAIL: ndt_scan_matcher node did not appear within ${STARTUP_TIMEOUT}s"
        echo "See $LAUNCH_LOG"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
echo "Localization bring-up ready after ~${elapsed}s"

# --- Step 2: discover the decoded top-LiDAR pointcloud topic (fact #4) ---
TOPIC_CANDIDATES="$(ros2 topic list -t 2>/dev/null | grep "sensing/lidar/top" | grep "sensor_msgs/msg/PointCloud2" | awk '{print $1}')"
POINTCLOUD_TOPIC="$(echo "$TOPIC_CANDIDATES" | grep "pointcloud_raw_ex" | head -1)"
if [ -z "$POINTCLOUD_TOPIC" ]; then
    POINTCLOUD_TOPIC="$(echo "$TOPIC_CANDIDATES" | head -1)"
fi
if [ -z "$POINTCLOUD_TOPIC" ]; then
    echo "FAIL: could not discover a decoded top-LiDAR PointCloud2 topic"
    echo "Candidates seen:"
    echo "$TOPIC_CANDIDATES"
    exit 1
fi
echo "Discovered top-LiDAR pointcloud topic: $POINTCLOUD_TOPIC"

# --- Step 3: start recording immediately (auto-init window matters, fact #6) ---
rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"
setsid ros2 bag record -o "$OUT" \
    /localization/kinematic_state "$POINTCLOUD_TOPIC" \
    /sensing/imu/tamagawa/imu_raw /vehicle/status/velocity_status \
    /tf /tf_static \
    > "$RECORD_LOG" 2>&1 &
RECORD_PID=$!
sleep 2

# --- Step 4: replay the migrated sample bag (foreground; bag supplies /clock, fact #5) ---
echo "Replaying $BAG at rate $RATE..."
ros2 bag play "$BAG" -r "$RATE" > "$PLAY_LOG" 2>&1
echo "Replay finished."

# --- Step 5: stop recording cleanly ---
kill -INT "$RECORD_PID" 2>/dev/null || true
sleep 3
RECORD_PID=""

# --- Step 6: verify ---
INFO="$(ros2 bag info "$OUT" 2>&1)" || INFO=""
echo "$INFO"
KINEMATIC_COUNT="$(echo "$INFO" | awk -F'Count: ' '/\/localization\/kinematic_state \|/ {split($2,a," "); print a[1]}')"
KINEMATIC_COUNT="${KINEMATIC_COUNT:-0}"
POINTCLOUD_COUNT="$(echo "$INFO" | awk -F'Count: ' -v t="$POINTCLOUD_TOPIC" '$0 ~ ("Topic: " t " \\|") {split($2,a," "); print a[1]}')"
POINTCLOUD_COUNT="${POINTCLOUD_COUNT:-0}"

if [ "$KINEMATIC_COUNT" -gt "$KINEMATIC_STATE_THRESHOLD" ] && [ "$POINTCLOUD_COUNT" -gt "$POINTCLOUD_THRESHOLD" ]; then
    echo "PASS: kinematic_state count=$KINEMATIC_COUNT (> $KINEMATIC_STATE_THRESHOLD), pointcloud ($POINTCLOUD_TOPIC) count=$POINTCLOUD_COUNT (> $POINTCLOUD_THRESHOLD) -- $OUT"
    exit 0
else
    echo "FAIL: kinematic_state count=$KINEMATIC_COUNT (threshold $KINEMATIC_STATE_THRESHOLD), pointcloud count=$POINTCLOUD_COUNT (threshold $POINTCLOUD_THRESHOLD)"
    echo "Launch log: $LAUNCH_LOG"
    echo "Record log: $RECORD_LOG"
    echo "Play log: $PLAY_LOG"
    exit 1
fi
