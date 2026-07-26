#!/usr/bin/env bash
# scripts/2dlidar/run-amcl.sh — Phase 3c Lever 4 (arbitration): bring up
# nav2_amcl on the SAME map/scan/odom lineup the vendored particle_filter
# used (scripts/2dlidar/run-particle-filter.sh), so a PF-vs-AMCL comparison
# isolates "PF port defect" from "2D MCL limit at this site/speed".
#
# This is a fork of run-particle-filter.sh's node lineup (map_server,
# pointcloud_to_laserscan + QoS bridge, wheel_imu_odom) with the
# particle_filter node swapped for nav2_amcl. Differences from
# run-particle-filter.sh are called out inline below; see that script's
# header for the shared node-lineup rationale (frame-fix, QoS-bridge,
# double-literal normalization, etc.) which is not repeated here.
#
# Env overrides: BAG, OUT (alias OUT_BAG), RATE, MAP_YAML, GT_BAG,
# INITPOSE_DELAY, POINTCLOUD_TOPIC, VELOCITY_TOPIC, IMU_TOPIC, IMU_YAW_SIGN,
# SCAN_MIN_HEIGHT/SCAN_MAX_HEIGHT, SCAN_RANGE_MAX, AMCL_MIN_PARTICLES,
# AMCL_MAX_PARTICLES, AMCL_INIT_TIMEOUT_S.
#
# --- AMCL-specific differences from run-particle-filter.sh ---
#
# 1. AMCL is a nav2 lifecycle node (like map_server): configure then
#    activate, via the same lifecycle_set_retry helper used for map_server.
#
# 2. TF requirement (CRITICAL): unlike particle_filter (which does no tf2
#    lookups at all -- see run-particle-filter.sh notes), AMCL requires a
#    live odom -> base_link TF to advance particles between scans; it does
#    not subscribe to the /odom topic for that purpose. wheel_imu_odom.py
#    only ever published /odom as a TOPIC. Fixed by adding a `publish_tf`
#    parameter to wheel_imu_odom.py (default false, preserves the PF
#    script's behavior byte-for-byte); this script passes
#    `-p publish_tf:=true`. base_link -> velodyne_top still comes from the
#    bag's /tf_static (replayed below), which AMCL uses via tf2 to project
#    /scan into base_link.
#
# 3. TF COLLISION (CRITICAL, and the reason a naive rerun of the same replay
#    command would silently invalidate the whole comparison): the enriched
#    GT bag (data/rosbags/phase3/sample_ndt_gt) contains /tf recorded from
#    the ORIGINAL NDT run, which includes a map -> base_link transform.
#    AMCL broadcasts its OWN map -> odom transform (map -> odom -> base_link,
#    the standard nav2 tree). Replaying the bag's /tf verbatim would publish
#    a second, conflicting map -> base_link edge (via a different path than
#    AMCL's map -> odom -> base_link), corrupting the TF tree AMCL itself
#    relies on to project /scan into map frame -- a bug that would not show
#    up as a crash, just as silently-wrong (or effectively random) AMCL
#    poses. Fix: replay every topic in the bag EXCEPT /tf (allowlist below
#    keeps /tf_static, which only carries the static sensor-kit transforms
#    and does not conflict). Verified via `ros2 bag info` that the bag's
#    topic set is exactly: /vehicle/status/velocity_status,
#    /sensing/imu/tamagawa/imu_raw, /tf_static, /tf,
#    /sensing/lidar/top/pointcloud_raw_ex, /localization/kinematic_state.
#
# 4. Vehicle model approximation: the bag's vehicle is car-like (Ackermann),
#    but nav2_amcl only ships DifferentialMotionModel and OmniMotionModel
#    plugins -- there is no car-like/Ackermann motion model available in
#    stock nav2_amcl. This script uses DifferentialMotionModel (the closer
#    of the two available approximations for a forward-driving vehicle with
#    limited slip at these speeds); this is a known approximation, not a
#    perfect match, and is called out in the Lever 4 report rather than
#    hidden.
#
# 5. AMCL publishes geometry_msgs/PoseWithCovarianceStamped on /amcl_pose
#    (not PoseStamped like PF's /pf/viz/inferred_pose) plus
#    nav2_msgs/ParticleCloud on /particle_cloud; both are recorded. The
#    PASS gate below checks /amcl_pose count, mirroring PF's
#    /pf/viz/inferred_pose gate.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_DIR"

BAG="${BAG:-$REPO_DIR/data/rosbags/phase3/sample_ndt_gt}"
OUT="${OUT_BAG:-${OUT:-$REPO_DIR/data/rosbags/phase3/amcl_run}}"
RATE="${RATE:-1.0}"
MAP_YAML="${MAP_YAML:-$REPO_DIR/data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml}"
GT_BAG="${GT_BAG:-$REPO_DIR/data/rosbags/phase3/sample_ndt_gt}"
INITPOSE_DELAY="${INITPOSE_DELAY:-5}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/sensing/lidar/top/pointcloud_raw_ex}"
VELOCITY_TOPIC="${VELOCITY_TOPIC:-/vehicle/status/velocity_status}"
IMU_TOPIC="${IMU_TOPIC:-/sensing/imu/tamagawa/imu_raw}"
IMU_YAW_SIGN="${IMU_YAW_SIGN:--1.0}"

# ROS 2 rejects integer literals for double-typed parameters -- see
# run-particle-filter.sh's Lever-3 postmortem (SCAN_RANGE_MAX=60 crashed
# pointcloud_to_laserscan's range_max). Normalize every double-typed env
# override the same way here.
to_float() { printf '%.6f' "$1"; }
SCAN_MIN_HEIGHT="$(to_float "${SCAN_MIN_HEIGHT:-1.91611}")"
SCAN_MAX_HEIGHT="$(to_float "${SCAN_MAX_HEIGHT:-2.21611}")"
SCAN_RANGE_MAX="$(to_float "${SCAN_RANGE_MAX:-60.0}")"
AMCL_LASER_MAX_RANGE="$(to_float "${AMCL_LASER_MAX_RANGE:-60.0}")"
AMCL_MIN_PARTICLES="${AMCL_MIN_PARTICLES:-500}"
AMCL_MAX_PARTICLES="${AMCL_MAX_PARTICLES:-4000}"
AMCL_INIT_TIMEOUT_S="${AMCL_INIT_TIMEOUT_S:-60}"

AMCL_POSE_THRESHOLD=200
PARAMS_FILE="$REPO_DIR/tmp/amcl_params.yaml"

mkdir -p "$REPO_DIR/tmp"
MAP_LOG="$(mktemp "$REPO_DIR/tmp/run-amcl-map.XXXXXX.log")"
SCAN_LOG="$(mktemp "$REPO_DIR/tmp/run-amcl-scan.XXXXXX.log")"
BRIDGE_LOG="$(mktemp "$REPO_DIR/tmp/run-amcl-bridge.XXXXXX.log")"
ODOM_LOG="$(mktemp "$REPO_DIR/tmp/run-amcl-odom.XXXXXX.log")"
AMCL_LOG="$(mktemp "$REPO_DIR/tmp/run-amcl-amcl.XXXXXX.log")"
RECORD_LOG="$(mktemp "$REPO_DIR/tmp/run-amcl-record.XXXXXX.log")"

MAP_PID=""
SCAN_PID=""
BRIDGE_PID=""
ODOM_PID=""
AMCL_PID=""
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
    kill_pgid "$AMCL_PID"
    kill_pgid "$ODOM_PID"
    kill_pgid "$BRIDGE_PID"
    kill_pgid "$SCAN_PID"
    kill_pgid "$MAP_PID"
    sleep 2
    kill_pgid "$AMCL_PID"
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

# --- Step 0: write amcl_params.yaml ---
cat > "$PARAMS_FILE" <<EOF
amcl:
  ros__parameters:
    use_sim_time: true
    scan_topic: 'scan'
    odom_frame_id: 'odom'
    base_frame_id: 'base_link'
    global_frame_id: 'map'
    tf_broadcast: true
    robot_model_type: 'nav2_amcl::DifferentialMotionModel'
    min_particles: $AMCL_MIN_PARTICLES
    max_particles: $AMCL_MAX_PARTICLES
    laser_max_range: $AMCL_LASER_MAX_RANGE
    laser_min_range: -1.0
    laser_max_beams: 60
    laser_model_type: 'likelihood_field'
    laser_likelihood_max_dist: 2.0
    laser_z_hit: 0.5
    laser_z_rand: 0.5
    laser_z_short: 0.05
    laser_z_max: 0.05
    laser_sigma_hit: 0.2
    laser_lambda_short: 0.1
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    update_min_d: 0.1
    update_min_a: 0.1
    resample_interval: 1
    transform_tolerance: 1.0
    recovery_alpha_slow: 0.0
    recovery_alpha_fast: 0.0
    pf_err: 0.05
    pf_z: 0.99
    save_pose_rate: 0.5
    always_reset_initial_pose: false
    first_map_only: false
    set_initial_pose: false
EOF

# --- Step 1: nav2_map_server on the map grid (lifecycle configure+activate) ---
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

# --- Step 2: pointcloud_to_laserscan -> /scan_raw (BEST_EFFORT), same
# frame-fix (target_frame:=base_link) and z-band shift as
# run-particle-filter.sh -- see that script's header for the derivation.
setsid ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r /pointcloud_to_laserscan/input/pointcloud:="$POINTCLOUD_TOPIC" \
    -r /pointcloud_to_laserscan/output/laserscan:=/scan_raw \
    -p target_frame:=base_link \
    -p min_height:="$SCAN_MIN_HEIGHT" -p max_height:="$SCAN_MAX_HEIGHT" \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 \
    -p angle_increment:=0.0043 -p range_min:=0.1 -p range_max:="$SCAN_RANGE_MAX" \
    -p use_sim_time:=true \
    > "$SCAN_LOG" 2>&1 &
SCAN_PID=$!

sleep 2
if ! kill -0 "$SCAN_PID" 2>/dev/null; then
    echo "FAIL: scan converter did not start (see scan log)"
    echo "Scan log: $SCAN_LOG"
    tail -n 20 "$SCAN_LOG" || true
    exit 1
fi

# --- Step 2b: QoS bridge /scan_raw (BEST_EFFORT) -> /scan (RELIABLE).
# AMCL's LaserScan subscription (like particle_filter's) resolves to
# RELIABLE/VOLATILE by default and is incompatible with
# pointcloud_to_laserscan's hardcoded SensorDataQoS publisher -- see
# run-particle-filter.sh Attempt-1 finding.
BRIDGE_SCRIPT="$REPO_DIR/tmp/scan_qos_bridge_amcl.py"
cat > "$BRIDGE_SCRIPT" <<'PYEOF'
"""Republish a BEST_EFFORT LaserScan as RELIABLE (see run-amcl.sh Step 2b)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = Node('scan_qos_bridge_amcl')
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

# --- Step 3: wheel+IMU odometry, WITH TF broadcast (publish_tf:=true) --
# AMCL-specific: AMCL needs a live odom -> base_link TF (see header note 2).
setsid python3 "$SCRIPT_DIR/wheel_imu_odom.py" --ros-args -p use_sim_time:=true \
    -p velocity_topic:="$VELOCITY_TOPIC" -p imu_topic:="$IMU_TOPIC" \
    -p imu_yaw_sign:="$IMU_YAW_SIGN" -p publish_tf:=true \
    > "$ODOM_LOG" 2>&1 &
ODOM_PID=$!
sleep 2

# --- Step 4: nav2_amcl (lifecycle node: configure then activate) ---
setsid ros2 run nav2_amcl amcl --ros-args \
    --params-file "$PARAMS_FILE" -p use_sim_time:=true \
    > "$AMCL_LOG" 2>&1 &
AMCL_PID=$!
sleep 3
lifecycle_set_retry /amcl configure
lifecycle_set_retry /amcl activate
echo "amcl active"

# --- Step 5: extract the first GT pose for /initialpose (same helper as
# run-particle-filter.sh) ---
POSE_EXTRACT_SCRIPT="$REPO_DIR/tmp/extract_first_pose_amcl.py"
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

POSE_LOG="$(mktemp "$REPO_DIR/tmp/run-amcl-pose-extract.XXXXXX.log")"
POSE_OUT="$(python3 "$POSE_EXTRACT_SCRIPT" "$GT_BAG" 2>>"$POSE_LOG")" || POSE_OUT=""
read -r GT_X GT_Y GT_Z GT_QX GT_QY GT_QZ GT_QW <<< "$POSE_OUT"

if [ -z "${GT_X:-}" ]; then
    echo "FAIL: could not extract initial pose from $GT_BAG"
    echo "Pose extraction log: $POSE_LOG"
    exit 1
fi
echo "Initial pose from GT bag: x=$GT_X y=$GT_Y qz=$GT_QZ qw=$GT_QW"

# --- Step 6: start recording the AMCL pose bag ---
rm -rf "$OUT"
mkdir -p "$(dirname "$OUT")"
setsid ros2 bag record -o "$OUT" /amcl_pose /particle_cloud \
    > "$RECORD_LOG" 2>&1 &
RECORD_PID=$!
sleep 2

# --- Step 7: publish /initialpose a few seconds into replay ---
(
    sleep "$INITPOSE_DELAY"
    ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
        "{header: {frame_id: 'map'}, pose: {pose: {position: {x: $GT_X, y: $GT_Y, z: 0.0}, \
        orientation: {x: $GT_QX, y: $GT_QY, z: $GT_QZ, w: $GT_QW}}, \
        covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, 0,0,0,0.068,0,0, 0,0,0,0,0.068,0, 0,0,0,0,0,0.068]}}" \
        > /dev/null 2>&1
) &
INITPOSE_PID=$!

# --- Step 8: replay the bag EXCLUDING /tf (header note 3: the bag's /tf is
# the NDT run's map -> base_link, which would collide with AMCL's own
# map -> odom broadcast). /tf_static (sensor-kit static transforms) is kept.
echo "Replaying $BAG at rate $RATE (excluding /tf to avoid AMCL TF collision)..."
ros2 bag play "$BAG" --clock -r "$RATE" \
    --topics "$POINTCLOUD_TOPIC" "$VELOCITY_TOPIC" "$IMU_TOPIC" /tf_static /localization/kinematic_state
echo "Replay finished."

# --- Step 9: stop recording cleanly ---
kill -INT "$RECORD_PID" 2>/dev/null || true
sleep 3
RECORD_PID=""

# --- Step 10: verify ---
INFO="$(ros2 bag info "$OUT" 2>&1)" || INFO=""
echo "$INFO"
POSE_COUNT="$(echo "$INFO" | awk -F'Count: ' '/\/amcl_pose/ {split($2,a," "); print a[1]}')"
POSE_COUNT="${POSE_COUNT:-0}"

if [ "$POSE_COUNT" -gt "$AMCL_POSE_THRESHOLD" ]; then
    echo "PASS: amcl_pose count=$POSE_COUNT (> $AMCL_POSE_THRESHOLD) -- $OUT"
    exit 0
else
    echo "FAIL: amcl_pose count=$POSE_COUNT (<= $AMCL_POSE_THRESHOLD)"
    echo "Map log: $MAP_LOG"
    echo "Scan log: $SCAN_LOG"
    echo "Bridge log: $BRIDGE_LOG"
    echo "Odom log: $ODOM_LOG"
    echo "AMCL log: $AMCL_LOG"
    echo "Record log: $RECORD_LOG"
    exit 1
fi
