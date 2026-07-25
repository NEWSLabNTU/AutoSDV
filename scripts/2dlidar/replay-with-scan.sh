#!/usr/bin/env bash
# scripts/2dlidar/replay-with-scan.sh — Phase 2 gate:
# replay a rosbag and synthesize /scan from the 3D pointcloud.
# Env overrides: BAG, POINTCLOUD_TOPIC, RATE.
#
# Default bag: AutoSDV's own outdoor recording (data/rosbags/outdoor_20251226_153115),
# which publishes /sensing/lidar/velodyne_points (sensor_msgs/PointCloud2, ~10 Hz).
#
# Alternative: the Autoware sample-rosbag (data/sample-rosbag-replay/sample-rosbag,
# from Task 5) contains NO PointCloud2 topics — only raw velodyne_msgs/VelodyneScan
# packet topics (e.g. /sensing/lidar/top/velodyne_packets). To use it here you must
# first decode packets to points, e.g.:
#   ros2 run velodyne_pointcloud velodyne_transform_node --ros-args \
#       -p calibration:=<vlp16.yaml> -r velodyne_packets:=/sensing/lidar/top/velodyne_packets
# (available at /opt/ros/humble; requires a VLP-16 calibration yaml) then point this
# script at POINTCLOUD_TOPIC:=/velodyne_points. Automating that decode step is out of
# scope here — comment only.
# Note: Autoware 1.5.0's pointcloud_to_laserscan package differs from the
# upstream ros-perception one: topics are input/pointcloud and output/laserscan
# (not cloud_in/scan), the input subscription is created lazily (only once a
# subscriber attaches to the remapped output topic), and remap rules must use
# fully-qualified names (/pointcloud_to_laserscan/...) — relative remaps were
# silently ignored in testing.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BAG="${BAG:-$SCRIPT_DIR/../../data/rosbags/outdoor_20251226_153115}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/sensing/lidar/velodyne_points}"
RATE="${RATE:-0.5}"

# shellcheck disable=SC1091
set +u
source /opt/autoware/1.5.0/setup.bash
set -u

setsid ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r /pointcloud_to_laserscan/input/pointcloud:="$POINTCLOUD_TOPIC" \
    -r /pointcloud_to_laserscan/output/laserscan:=/scan \
    -p min_height:=-0.15 -p max_height:=0.15 \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 \
    -p angle_increment:=0.0043 -p range_min:=0.1 -p range_max:=30.0 \
    -p use_sim_time:=true &
CONV_PID=$!
PLAY_PID=""
trap 'kill -- -"$CONV_PID" 2>/dev/null || true; [ -n "$PLAY_PID" ] && kill -- -"$PLAY_PID" 2>/dev/null || true' EXIT
sleep 2

setsid ros2 bag play "$BAG" --clock -r "$RATE" &
PLAY_PID=$!
sleep 5

echo "--- /scan rate (expect ~10 Hz x replay rate) ---"
# `ros2 topic hz` runs until killed, so `timeout` always ends it with a
# nonzero exit — that alone is not failure. Check its captured output for an
# actual rate reading instead of trusting the exit code.
HZ_OUT=$(timeout 15 ros2 topic hz /scan --window 20 2>&1) || true
echo "$HZ_OUT"
if ! echo "$HZ_OUT" | grep -q "average rate"; then
    echo "FAIL: no /scan"; exit 1
fi
echo "--- sample message ---"
HEADER=$(timeout 10 ros2 topic echo /scan --once --field header 2>/dev/null) || HEADER=""
if [ -z "$HEADER" ]; then
    echo "FAIL: could not read /scan header"; exit 1
fi
echo "$HEADER"
echo "PASS: /scan synthesized from replay"
