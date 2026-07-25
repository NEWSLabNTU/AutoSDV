#!/usr/bin/env bash
# scripts/2dlidar/run-particle-filter.sh — Phase 3 Task 3 gate:
# bring up the vendored particle_filter (MCL) against the COSS occupancy
# grid, feed it /scan (from pointcloud_to_laserscan) and /odom (from
# wheel_imu_odom.py), seed it with the first GT pose, replay the outdoor
# bag, and record /pf/viz/inferred_pose (+ /pf/pose/odom) for Task 4.
#
# Env overrides: BAG, OUT, RATE, MAP_YAML, GT_BAG, INITPOSE_DELAY.
#
# --- particle_filter.py source findings (src/localization/external/particle_filter) ---
#
# 1. Map acquisition: get_omap() calls the nav_msgs/GetMap service at
#    `/map_server/map` (rclpy.spin_until_future_complete inside __init__,
#    BEFORE any subscriptions are created) — NOT the /map topic. So
#    nav2_map_server just needs to be configured+activated (offers the
#    service regardless of QoS concerns) before particle_filter starts;
#    no topic QoS matching needed. Node name must be "map_server" (default
#    for `ros2 run nav2_map_server map_server`) since the service name is
#    hardcoded in particle_filter.py.
#
# 2. /initialpose: subscribed as geometry_msgs/PoseWithCovarianceStamped,
#    handled by clicked_pose -> initialize_particles_pose (seeds particles
#    with gaussian noise around the given x/y/yaw; ignores covariance
#    values and z/roll/pitch).
#
# 3. Frames: particle_filter does NOT use tf2 lookups at all — it only
#    broadcasts map->/laser itself (publish_tf) and never reads
#    msg.header.frame_id from the incoming LaserScan/Odometry. No static
#    TF (e.g. base_link->vlp32c) is needed for the filter to function.
#
# 4. CRITICAL param bug in the brief: visualize() (which publishes
#    /pf/viz/inferred_pose) starts with `if not self.DO_VIZ: return`. The
#    brief's suggested override `viz: 0` would silence inferred_pose
#    entirely. The vendored default is already `viz: 1` — this script does
#    NOT override it (kept explicit at 1 in the generated params file).
#    Inferred_pose is further gated on `pose_pub.get_subscription_count() >
#    0`, which `ros2 bag record` satisfies once it's subscribed.
#
# 5. update() (the MCL step, which sets inferred_pose) only runs once
#    lidar_initialized AND odom_initialized AND map_initialized are all
#    true, and is triggered from odomCB — so /odom must be flowing for any
#    inferred_pose output at all, not just /scan.
#
# 6. range_method: brief says use 'cddt' (CPU-only; 'rmgpu' is the vendored
#    default but requires a CUDA range_libc build which isn't present here
#    — GPU methods are unavailable/segfault). This script uses 'cddt'.
#
# --- Attempt 1 failure (documented) ---
# particle_filter's LaserScan subscription uses rclpy's default (queue-depth
# only) QoS, which resolves to RELIABLE/VOLATILE. pointcloud_to_laserscan's
# output publisher is hardcoded to rclcpp::SensorDataQoS (BEST_EFFORT).
# These are incompatible per ROS 2 QoS matching rules (a RELIABLE reader
# cannot receive from a BEST_EFFORT writer) — confirmed via `ros2 topic info
# /scan --verbose` (BEST_EFFORT) and the particle_filter log ("New publisher
# discovered on topic '/scan', offering incompatible QoS... Last
# incompatible policy: RELIABILITY"). pointcloud_to_laserscan does not wire
# QosOverridingOptions into its publisher (`-p
# qos_overrides.../publisher.reliability:=reliable` was tried against both
# the resolved and node-local topic names and had no effect — confirmed via
# `ros2 topic info /scan --verbose` still showing BEST_EFFORT). Result: 0
# scan messages ever reached particle_filter, lidar_initialized stayed
# False, update() never ran (see idea #5 above), and 0 inferred_pose
# messages were recorded despite /odom and /initialpose working correctly.
# Fix: a small QoS-bridging relay node (scan_qos_bridge, generated below)
# subscribes SensorDataQoS on /scan_raw and republishes RELIABLE on /scan.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_DIR"

BAG="${BAG:-$REPO_DIR/data/rosbags/outdoor_20251226_153115}"
OUT="${OUT:-$REPO_DIR/data/rosbags/phase3/pf_run}"
RATE="${RATE:-1.0}"
MAP_YAML="${MAP_YAML:-$REPO_DIR/data/COSS-map-planning/occupancy_grid.yaml}"
GT_BAG="${GT_BAG:-$REPO_DIR/data/rosbags/phase3/ndt_gt}"
INITPOSE_DELAY="${INITPOSE_DELAY:-5}"
INFERRED_POSE_THRESHOLD=200
PARAMS_FILE="$REPO_DIR/tmp/pf_params.yaml"

mkdir -p "$REPO_DIR/tmp"
MAP_LOG="$(mktemp "$REPO_DIR/tmp/run-pf-map.XXXXXX.log")"
SCAN_LOG="$(mktemp "$REPO_DIR/tmp/run-pf-scan.XXXXXX.log")"
BRIDGE_LOG="$(mktemp "$REPO_DIR/tmp/run-pf-bridge.XXXXXX.log")"
ODOM_LOG="$(mktemp "$REPO_DIR/tmp/run-pf-odom.XXXXXX.log")"
PF_LOG="$(mktemp "$REPO_DIR/tmp/run-pf-pf.XXXXXX.log")"
RECORD_LOG="$(mktemp "$REPO_DIR/tmp/run-pf-record.XXXXXX.log")"

MAP_PID=""
SCAN_PID=""
BRIDGE_PID=""
ODOM_PID=""
PF_PID=""
RECORD_PID=""
INITPOSE_PID=""

kill_pgid() {
    local pid="$1"
    [ -z "$pid" ] && return 0
    local pgid
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')" || pgid=""
    [ -n "$pgid" ] && kill -- -"$pgid" 2>/dev/null || true
}

cleanup() {
    if [ -n "$RECORD_PID" ]; then
        kill -INT "$RECORD_PID" 2>/dev/null || true
        sleep 3
    fi
    [ -n "$INITPOSE_PID" ] && kill -9 "$INITPOSE_PID" 2>/dev/null || true
    kill_pgid "$PF_PID"
    kill_pgid "$ODOM_PID"
    kill_pgid "$BRIDGE_PID"
    kill_pgid "$SCAN_PID"
    kill_pgid "$MAP_PID"
    sleep 2
    kill_pgid "$PF_PID"
    kill_pgid "$ODOM_PID"
    kill_pgid "$BRIDGE_PID"
    kill_pgid "$SCAN_PID"
    kill_pgid "$MAP_PID"
}
trap cleanup EXIT

# shellcheck disable=SC1091
set +u
source /opt/autoware/1.5.0/setup.bash
source "$REPO_DIR/install/setup.bash"
set -u

# --- Step 0: write pf_params.yaml (vendored config/localize.yaml + overrides) ---
mkdir -p "$REPO_DIR/tmp"
cat > "$PARAMS_FILE" <<'EOF'
particle_filter:
  ros__parameters:
    scan_topic: '/scan'
    odometry_topic: '/odom'
    angle_step: 18
    max_particles: 4000
    squash_factor: 2.2
    viz: 1
    max_viz_particles: 60
    range_method: 'cddt'
    theta_discretization: 112
    max_range: 30.0
    fine_timing: 0
    publish_odom: 1
    z_short: 0.01
    z_max: 0.07
    z_rand: 0.12
    z_hit: 0.75
    sigma_hit: 8.0
    motion_dispersion_x: 0.05
    motion_dispersion_y: 0.025
    motion_dispersion_theta: 0.25
    rangelib_variant: 2
EOF

# --- Step 1: nav2_map_server on the COSS grid (lifecycle configure+activate) ---
setsid ros2 run nav2_map_server map_server --ros-args \
    -p yaml_filename:="$MAP_YAML" -p use_sim_time:=true \
    > "$MAP_LOG" 2>&1 &
MAP_PID=$!
sleep 3

lifecycle_set_retry() {
    local node="$1" transition="$2"
    local attempt
    for attempt in 1 2 3 4 5; do
        if ros2 lifecycle set "$node" "$transition"; then
            return 0
        fi
        sleep 2
    done
    echo "FAIL: could not set $node to $transition after retries"
    exit 1
}
lifecycle_set_retry /map_server configure
lifecycle_set_retry /map_server activate

# --- Step 2: pointcloud_to_laserscan (Task 6 remaps, proven), output to
# /scan_raw — its publisher is hardcoded SensorDataQoS (BEST_EFFORT), which
# is incompatible with particle_filter's default RELIABLE subscription
# (Attempt 1 finding above). The QoS bridge in Step 2b fixes this.
setsid ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r /pointcloud_to_laserscan/input/pointcloud:=/sensing/lidar/velodyne_points \
    -r /pointcloud_to_laserscan/output/laserscan:=/scan_raw \
    -p min_height:=-0.15 -p max_height:=0.15 \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 \
    -p angle_increment:=0.0043 -p range_min:=0.1 -p range_max:=30.0 \
    -p use_sim_time:=true \
    > "$SCAN_LOG" 2>&1 &
SCAN_PID=$!

# --- Step 2b: QoS bridge /scan_raw (BEST_EFFORT) -> /scan (RELIABLE) ---
BRIDGE_SCRIPT="$REPO_DIR/tmp/scan_qos_bridge.py"
cat > "$BRIDGE_SCRIPT" <<'PYEOF'
"""Republish a BEST_EFFORT LaserScan as RELIABLE.

particle_filter's LaserScan subscription uses rclpy's default (depth-only)
QoS, which resolves to RELIABLE/VOLATILE. pointcloud_to_laserscan's output
publisher is hardcoded to rclcpp::SensorDataQoS (BEST_EFFORT). Those two
are incompatible per ROS 2 QoS matching rules, so this bridge sits between
them: subscribe SensorDataQoS on /scan_raw, republish RELIABLE on /scan.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = Node('scan_qos_bridge')
    sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=5)
    reliable_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10)
    pub = node.create_publisher(LaserScan, '/scan', reliable_qos)
    node.create_subscription(LaserScan, '/scan_raw', pub.publish, sensor_qos)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
PYEOF
setsid python3 "$BRIDGE_SCRIPT" --ros-args -p use_sim_time:=true \
    > "$BRIDGE_LOG" 2>&1 &
BRIDGE_PID=$!

# --- Step 3: wheel+IMU odometry ---
setsid python3 "$SCRIPT_DIR/wheel_imu_odom.py" --ros-args -p use_sim_time:=true \
    > "$ODOM_LOG" 2>&1 &
ODOM_PID=$!
sleep 2

# --- Step 4: particle_filter (blocks on GetMap service in __init__, so
# map_server must already be active) ---
setsid ros2 run particle_filter particle_filter --ros-args \
    --params-file "$PARAMS_FILE" -p use_sim_time:=true \
    > "$PF_LOG" 2>&1 &
PF_PID=$!

echo "Waiting for particle_filter to finish initializing..."
elapsed=0
while ! grep -q "Finished initializing" "$PF_LOG" 2>/dev/null; do
    if [ "$elapsed" -ge 60 ]; then
        echo "FAIL: particle_filter did not finish initializing within 60s"
        echo "See $PF_LOG"
        exit 1
    fi
    sleep 2
    elapsed=$((elapsed + 2))
done
echo "particle_filter ready after ~${elapsed}s"

# --- Step 5: extract the first GT pose for /initialpose ---
POSE_EXTRACT_SCRIPT="$REPO_DIR/tmp/extract_first_pose.py"
cat > "$POSE_EXTRACT_SCRIPT" <<'PYEOF'
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3")
converter_options = rosbag2_py.ConverterOptions("", "")
reader.open(storage_options, converter_options)
type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
target = "/localization/kinematic_state"
msg_type = get_message(type_map[target])

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == target:
        msg = deserialize_message(data, msg_type)
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        print(p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        break
PYEOF

POSE_LOG="$(mktemp "$REPO_DIR/tmp/run-pf-pose-extract.XXXXXX.log")"
POSE_OUT="$(python3 "$POSE_EXTRACT_SCRIPT" "$GT_BAG" 2>>"$POSE_LOG")" || POSE_OUT=""
read -r GT_X GT_Y GT_Z GT_QX GT_QY GT_QZ GT_QW <<< "$POSE_OUT"

if [ -z "${GT_X:-}" ]; then
    echo "FAIL: could not extract initial pose from $GT_BAG"
    echo "Pose extraction log: $POSE_LOG"
    exit 1
fi
echo "Initial pose from GT bag: x=$GT_X y=$GT_Y qz=$GT_QZ qw=$GT_QW"

# --- Step 6: start recording the PF pose bag ---
rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"
setsid ros2 bag record -o "$OUT" /pf/viz/inferred_pose /pf/pose/odom \
    > "$RECORD_LOG" 2>&1 &
RECORD_PID=$!
sleep 2

# --- Step 7: publish /initialpose a few seconds into replay (vehicle is
# stationary for the first ~116s of the outdoor bag, so timing is generous) ---
(
    sleep "$INITPOSE_DELAY"
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $GT_X, y: $GT_Y, z: 0.0}, \
        orientation: {x: $GT_QX, y: $GT_QY, z: $GT_QZ, w: $GT_QW}}, \
        covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, 0,0,0,0.068,0,0, 0,0,0,0,0.068,0, 0,0,0,0,0,0.068]}}" \
        > /dev/null 2>&1
) &
INITPOSE_PID=$!

# --- Step 8: replay the outdoor bag (foreground; ~157s at rate 1.0) ---
echo "Replaying $BAG at rate $RATE..."
ros2 bag play "$BAG" --clock -r "$RATE"
echo "Replay finished."

# --- Step 9: stop recording cleanly ---
kill -INT "$RECORD_PID" 2>/dev/null || true
sleep 3
RECORD_PID=""

# --- Step 10: verify ---
INFO="$(ros2 bag info "$OUT" 2>&1)" || INFO=""
echo "$INFO"
POSE_COUNT="$(echo "$INFO" | awk -F'Count: ' '/\/pf\/viz\/inferred_pose/ {split($2,a," "); print a[1]}')"
POSE_COUNT="${POSE_COUNT:-0}"

if [ "$POSE_COUNT" -gt "$INFERRED_POSE_THRESHOLD" ]; then
    echo "PASS: inferred_pose count=$POSE_COUNT (> $INFERRED_POSE_THRESHOLD) -- $OUT"
    exit 0
else
    echo "FAIL: inferred_pose count=$POSE_COUNT (<= $INFERRED_POSE_THRESHOLD)"
    echo "Map log: $MAP_LOG"
    echo "Scan log: $SCAN_LOG"
    echo "Bridge log: $BRIDGE_LOG"
    echo "Odom log: $ODOM_LOG"
    echo "PF log: $PF_LOG"
    echo "Record log: $RECORD_LOG"
    exit 1
fi
