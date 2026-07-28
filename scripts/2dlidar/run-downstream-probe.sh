#!/usr/bin/env bash
# Drive planning and control on a live pose source, end to end.
#
# The accuracy matrix (run-mcl-e2e-matrix.sh) scores kinematic_state against NDT
# ground truth and stops there. This exercises what comes after: initialise, set
# a route, engage, and record whether a trajectory and control commands actually
# flow, plus the vehicle's cross-track distance from its own plan.
#
# Configuration this encodes, each of which was a separate day's debugging:
#
#   * the RE-STAMPED sample bag. The upstream bag's storage timestamps run 328.9
#     days ahead of its message header stamps, so `--clock` produces a clock
#     inconsistent with the data and every TF lookup at a message stamp fails.
#     See scripts/rosbag/restamp_bag.py.
#   * sample_vehicle + sample_sensor_kit. The pairing rule is COSS bags with the
#     AutoSDV vehicle, the Autoware sample bag with the sample pair. Mismatching
#     is silent: the obstacle crop box spans ground-2.5 .. vehicle_height, so a
#     0.262 m vehicle against 2.5 m data discards every point above ~26 cm.
#   * gnss_receiver:=ublox, required by sample_sensor_kit's gnss.launch.xml,
#     which defines navsatfix_topic_name only for ublox or septentrio.
#   * for mcl, the kit's ring adapter on ONE sensor. The concatenated cloud is
#     3-D fusion for the NDT path; the 2-D path takes a single sensor. Rings
#     70-72 are the measured horizontal group for this VLS128.
#
# Usage:
#   METHOD=mcl scripts/2dlidar/run-downstream-probe.sh
#   METHOD=ndt scripts/2dlidar/run-downstream-probe.sh
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_DIR"

METHOD="${METHOD:-mcl}"
GT_BAG="$REPO_DIR/data/rosbags/phase3/sample_ndt_gt"
# The bag actually replayed. The Autoware sample rosbag carries RAW
# velodyne_packets from three LiDARs plus vehicle status; with sample_sensor_kit
# the sensing pipeline decodes and concatenates them into
# /sensing/lidar/concatenated/pointcloud, which is what localization and
# perception expect by default. Replaying the derived sample_ndt_gt recording
# instead fed the stack a single decoded cloud and starved both.
# -migrated carries autoware_vehicle_msgs; the original has the pre-1.5.0
# autoware_auto_vehicle_msgs types this Autoware cannot deserialize.
# -restamped: the upstream sample bag's storage timestamps run 328.9 days ahead
# of its message header stamps, so `--clock` produced a sim clock inconsistent
# with the data and every TF lookup at a message stamp failed. See
# scripts/rosbag/restamp_bag.py.
PLAY_BAG="${PLAY_BAG:-$REPO_DIR/data/sample-rosbag-replay/sample-rosbag-restamped}"
MAP_PATH="$REPO_DIR/data/sample-rosbag-replay/sample-map-rosbag"
GRID="occupancy_grid_scanaccum_mh1r05.yaml"
LOG_DIR="$REPO_DIR/tmp/downstream"
mkdir -p "$LOG_DIR"

set +u
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash
set -u

echo "=== $METHOD: launching $(date -Iseconds) ==="
setsid bash -c "
    set +u; source '$REPO_DIR/install/setup.bash'; set -u
    exec play_launch launch --web-addr 0.0.0.0:8081 \
        autosdv_launch logging_simulation.launch.yaml rviz:=false \
        pose_source:=$METHOD map_path:='$MAP_PATH' \
        occupancy_grid_file:='$GRID' mcl_random_seed:=1 use_gnss:=false \
        vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit \
        gnss_receiver:=ublox \
        scan_source:=external
" > "$LOG_DIR/launch_${METHOD}.log" 2>&1 &
sleep 8
LAUNCH_PID="$(pgrep -f 'play_launch launch.*logging_simulation' | head -1 || true)"
PGID="$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')"
echo "pgid=$PGID"

teardown() {
    kill -- "-$PGID" 2>/dev/null || true
    sleep 5
    pkill -9 -f "play_launch|component_container|particle_filter" 2>/dev/null || true
    # The scan adapter is launched by THIS script, outside the stack's process
    # group, so the kill above never reached it. A leaked scan_ring_filter or
    # scan_from_ring then collides with the next run's copy, and Autoware's
    # duplicated_node_checker fails the system branch -- which blocks autonomous
    # mode with no hint that the cause is a previous run.
    pkill -9 -f "scan_from_ring.launch.xml" 2>/dev/null || true
    pkill -9 -f "passthrough_filter_uint16_node" 2>/dev/null || true
    pkill -9 -f "pointcloud_to_laserscan_node" 2>/dev/null || true
    sleep 3
}
trap teardown EXIT

# Wait for the pose estimator to be ready. mcl logs its own readiness; for the
# NDT paths, wait for the planning stack's trajectory follower to exist.
if [ "$METHOD" = "mcl" ]; then
    for _ in $(seq 1 240); do
        PF_ERR="$(ls -t "$REPO_DIR"/play_log/*/node/particle_filter/err 2>/dev/null | head -1 || true)"
        [ -n "$PF_ERR" ] && grep -q "Finished initializing" "$PF_ERR" 2>/dev/null && break
        sleep 1
    done
else
    sleep 90
fi
echo "stack up, waiting for ADAPI"
for _ in $(seq 1 60); do
    ros2 service list 2>/dev/null | grep -q "/api/routing/set_route_points" && break
    sleep 2
done

# Replay in the background: the probe needs the bag's sensor data to localize,
# and the bag's own kinematic_state is excluded for the same reason as the
# accuracy matrix.
# The concatenated cloud is 3-D LiDAR fusion for the NDT path; the 2-D path
# takes ONE sensor. sample_sensor_kit ships no scan producer, so AutoSDV's ring
# adapter is launched alongside here, standing in for a kit that would ship it.
# ring=71 is the VLS128's horizontal channel, measured with
# scripts/sensor/inspect_rings.py -- not guessed.
if [ "$METHOD" = "mcl" ]; then
    setsid ros2 launch autosdv_sensor_kit_launch scan_from_ring.launch.xml \
        input_topic:=/sensing/lidar/top/pointcloud_raw_ex \
        ring_min:=70 ring_max:=72 \
        output_topic:=/scan_raw use_sim_time:=true \
        > "$LOG_DIR/scan_from_ring_${METHOD}.log" 2>&1 &
    sleep 5
fi

# Everything except the bag's own /clock, which `--clock` republishes. This
# ROS 2 version offers no --exclude, only an allowlist, so the eleven topics are
# named explicitly.
setsid timeout 260 ros2 bag play "$PLAY_BAG" --clock -r 0.5 \
    --topics /sensing/gnss/ublox/fix_velocity /sensing/gnss/ublox/nav_sat_fix \
             /sensing/gnss/ublox/navpvt /sensing/imu/tamagawa/imu_raw \
             /sensing/lidar/left/velodyne_packets \
             /sensing/lidar/right/velodyne_packets \
             /sensing/lidar/top/velodyne_packets \
             /vehicle/status/control_mode /vehicle/status/gear_status \
             /vehicle/status/steering_status /vehicle/status/velocity_status \
    > "$LOG_DIR/bagplay_${METHOD}.log" 2>&1 &
sleep 5

python3 "$REPO_DIR/scripts/2dlidar/downstream_probe.py" "$LOG_DIR/report_${METHOD}.json" \
    2>&1 | tee "$LOG_DIR/probe_${METHOD}.log"
echo "=== $METHOD done ==="
