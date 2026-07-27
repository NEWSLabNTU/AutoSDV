#!/usr/bin/env bash
# End-to-end accuracy matrix for pose_source:=mcl, scored on the fused
# /localization/kinematic_state against an NDT ground-truth bag.
#
# This replaces the ad-hoc Phase 5 Task 8 driver (tmp/t8_e2e_run.sh plus
# tmp/t8_seed_matrix*.sh), which carried two measurement defects that this
# script exists to prevent. Both were found by re-examining the recorded
# bags, not by the runs failing:
#
#   1. The seed was never applied. The old driver used $SEED only to name
#      output files; particle_filter ran with the shipped random_seed: -1,
#      i.e. nondeterministic. A table of five rows labelled "seed 61..65"
#      therefore reported five nondeterministic repeats, and its spread was
#      not seed spread. Fixed by passing mcl_random_seed:= into the launch
#      (plumbed for this purpose) and ASSERTING the readback below.
#
#   2. One stack was reused across all five replays. particle_filter and
#      ekf_localizer keep their converged pose between replays, so every run
#      after the first began believing it was at the previous run's finish
#      line -- 116 m from the new run's start on the sample site -- until the
#      new seed landed ~5 s in. Those ~24 paired poses were recorded as a
#      116 m "startup transient" and were the sole reason the mean gate
#      failed. The first run of each matrix was clean, which is exactly why
#      seed 61 passed and 62-65 did not. Fixed by launching and tearing down
#      a fresh stack per seed.
#
# Everything else follows the Phase 3/4 convention: oracle seed pose taken
# from the GT bag's first kinematic_state message, /localization/initialize
# (ADAPI, method DIRECT) to reach pose_initializer plus a direct /initialpose
# publish to reach particle_filter (which is outside the pose_initializer
# contract -- see docs/design/localization-method-switching.md), and the
# GT bag's own kinematic_state excluded from replay so it cannot be mistaken
# for the live EKF's output.
#
# Usage:
#   scripts/2dlidar/run-mcl-e2e-matrix.sh
#   SEEDS="1 2 3" scripts/2dlidar/run-mcl-e2e-matrix.sh
#
# Environment:
#   SEEDS                 seeds to run (default "1 2 3 4 5")
#   GT_BAG                NDT ground-truth bag (default sample_ndt_gt)
#   MAP_PATH              map directory for pose_source:=mcl
#   OCCUPANCY_GRID_FILE   grid metadata file within MAP_PATH
#   OUT_DIR               where per-seed bags land
#   RESULTS               jsonl results path
#   REPLAY_S              replay duration cap (default 75)
#   MIN_PF_POSES          dead-run gate (default 50)
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_DIR"

SEEDS="${SEEDS:-1 2 3 4 5}"
GT_BAG="${GT_BAG:-$REPO_DIR/data/rosbags/phase3/sample_ndt_gt}"
MAP_PATH="${MAP_PATH:-$REPO_DIR/data/sample-rosbag-replay/sample-map-rosbag}"
OCCUPANCY_GRID_FILE="${OCCUPANCY_GRID_FILE:-occupancy_grid_scanaccum_mh1r05.yaml}"
OUT_DIR="${OUT_DIR:-$REPO_DIR/data/rosbags/phase5-fresh}"
RESULTS="${RESULTS:-$OUT_DIR/results.jsonl}"
REPLAY_S="${REPLAY_S:-75}"
MIN_PF_POSES="${MIN_PF_POSES:-50}"
LAUNCH_TIMEOUT_S="${LAUNCH_TIMEOUT_S:-240}"

RELAY_TOPIC=/localization/pose_estimator/pose_with_covariance
LOG_DIR="$REPO_DIR/tmp/mcl-e2e"
mkdir -p "$OUT_DIR" "$LOG_DIR"

set +u
source /opt/autoware/1.5.0/setup.bash
source install/setup.bash
set -u

: > "$RESULTS"

# Oracle seed pose: the GT bag's first kinematic_state message.
read -r GT_X GT_Y GT_Z GT_QX GT_QY GT_QZ GT_QW <<< "$(python3 - "$GT_BAG" <<'PYEOF'
import sys
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
target = "/localization/kinematic_state"
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == target:
        msg = deserialize_message(data, get_message(types[topic]))
        p, o = msg.pose.pose.position, msg.pose.pose.orientation
        print(p.x, p.y, p.z, o.x, o.y, o.z, o.w)
        break
PYEOF
)"
echo "oracle seed pose: x=$GT_X y=$GT_Y qz=$GT_QZ qw=$GT_QW"

COV="[0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0.25,0,0,0, 0,0,0,0.068,0,0, 0,0,0,0,0.068,0, 0,0,0,0,0,0.068]"
POSE="{position: {x: $GT_X, y: $GT_Y, z: 0.0}, orientation: {x: $GT_QX, y: $GT_QY, z: $GT_QZ, w: $GT_QW}}"

teardown() {
    local pgid="$1"
    [ -n "$pgid" ] && kill -- "-$pgid" 2>/dev/null || true
    # Killing the launcher by PID alone orphans component containers, which
    # then hold the node names the next seed needs (see CLAUDE.md).
    for _ in $(seq 1 20); do
        pgrep -f "component_container|particle_filter|play_launch" >/dev/null || break
        sleep 1
    done
    pkill -9 -f "play_launch|component_container|particle_filter" 2>/dev/null || true
    sleep 3
}

fail_seed() {
    printf '{"seed": %s, "error": "%s"}\n' "$1" "$2" >> "$RESULTS"
    echo "seed $1: FAILED -- $2"
}

for SEED in $SEEDS; do
    echo "=== seed $SEED starting $(date -Iseconds) ==="
    OUT="$OUT_DIR/mcl_e2e_s${SEED}"
    LAUNCH_LOG="$LOG_DIR/launch_s${SEED}.log"
    rm -rf "$OUT"

    # --- fresh stack (defect 2) ---
    setsid bash -c "
        set +u; source '$REPO_DIR/install/setup.bash'; set -u
        exec play_launch launch --web-addr 0.0.0.0:8081 \
            autosdv_launch logging_simulation.launch.yaml rviz:=false \
            pose_source:=mcl map_path:='$MAP_PATH' \
            occupancy_grid_file:='$OCCUPANCY_GRID_FILE' \
            mcl_random_seed:=$SEED use_gnss:=false
    " > "$LAUNCH_LOG" 2>&1 &
    sleep 8
    LAUNCH_PID="$(pgrep -f 'play_launch launch.*logging_simulation' | head -1 || true)"
    if [ -z "$LAUNCH_PID" ]; then
        fail_seed "$SEED" "launch did not start"
        continue
    fi
    PGID="$(ps -o pgid= -p "$LAUNCH_PID" | tr -d ' ')"
    echo "seed $SEED: launch pgid=$PGID"

    # Ready when particle_filter reports it is waiting on messages.
    PF_ERR=""
    READY=0
    for _ in $(seq 1 "$LAUNCH_TIMEOUT_S"); do
        PF_ERR="$(ls -t "$REPO_DIR"/play_log/*/node/particle_filter/err 2>/dev/null | head -1 || true)"
        if [ -n "$PF_ERR" ] && grep -q "Finished initializing" "$PF_ERR" 2>/dev/null; then
            READY=1; break
        fi
        sleep 1
    done
    if [ "$READY" -ne 1 ]; then
        teardown "$PGID"
        fail_seed "$SEED" "particle_filter never finished initializing"
        continue
    fi

    # --- assert the seed was actually applied (defect 1) ---
    if ! grep -q "Seeded numpy RNG with random_seed=$SEED" "$PF_ERR"; then
        teardown "$PGID"
        fail_seed "$SEED" "random_seed readback mismatch (mcl_random_seed did not reach the node)"
        continue
    fi
    echo "seed $SEED: random_seed=$SEED confirmed on the running node"

    # --- record, seed, replay ---
    setsid ros2 bag record -o "$OUT" \
        /localization/kinematic_state "$RELAY_TOPIC" \
        /pf/viz/inferred_pose /pf/pose/odom /clock \
        > "$LOG_DIR/record_s${SEED}.log" 2>&1 &
    RECORD_PID=$!
    sleep 2

    (
        sleep 5
        # ADAPI is the real entry point: publishing /initialpose3d directly
        # bypasses pose_initializer and leaves ekf_localizer deactivated.
        ros2 service call /localization/initialize \
            autoware_localization_msgs/srv/InitializeLocalization \
            "{pose_with_covariance: [{header: {frame_id: 'map'}, pose: {pose: $POSE, covariance: $COV}}], method: 1}" \
            > "$LOG_DIR/initialize_s${SEED}.log" 2>&1
        # particle_filter subscribes /initialpose, not /initialpose3d, so the
        # ADAPI call above does not reach it.
        ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
            "{header: {frame_id: 'map'}, pose: {pose: $POSE, covariance: $COV}}" \
            > /dev/null 2>&1
    ) &
    SEED_PID=$!

    # The GT bag was recorded from a real NDT run and carries its own
    # /localization/kinematic_state; replaying it would mix ground truth into
    # the topic the live EKF publishes on.
    setsid timeout "$REPLAY_S" ros2 bag play "$GT_BAG" --clock -r 1.0 \
        --topics /vehicle/status/velocity_status /sensing/imu/tamagawa/imu_raw \
                 /sensing/lidar/top/pointcloud_raw_ex /tf /tf_static \
        > "$LOG_DIR/bagplay_s${SEED}.log" 2>&1 || true

    kill -INT "$RECORD_PID" 2>/dev/null || true
    sleep 3
    wait "$SEED_PID" 2>/dev/null || true

    INFO="$(ros2 bag info "$OUT" 2>&1)" || INFO=""
    teardown "$PGID"

    # --- dead-run gate: a frozen EKF pose still scores as a real number ---
    PF_POSES="$(echo "$INFO" | awk -F'Count: ' -v t="$RELAY_TOPIC" \
        '$0 ~ t {split($2,a," "); print a[1]}')"
    PF_POSES="${PF_POSES:-0}"
    if [ "$PF_POSES" -lt "$MIN_PF_POSES" ]; then
        fail_seed "$SEED" "dead run: only $PF_POSES relay poses (< $MIN_PF_POSES)"
        continue
    fi

    # --- score ---
    REPORT="$OUT_DIR/mcl_e2e_s${SEED}_report.md"
    python3 scripts/2dlidar/compare_poses.py "$GT_BAG" "$OUT" \
        --pf-topic /localization/kinematic_state --pf-type Odometry \
        --no-motion-window --gt-time-source bag --out "$REPORT" \
        > "$LOG_DIR/compare_s${SEED}.log" 2>&1 \
        || echo "seed $SEED: compare_poses.py exited nonzero (a FAIL verdict is a result, not an error)"

    if [ ! -f "$REPORT" ]; then
        fail_seed "$SEED" "no report"
        continue
    fi

    read -r PAIRS TMEAN TP95 TMAX YAW <<< "$(python3 - "$REPORT" <<'PYCODE'
import re, sys
text = open(sys.argv[1]).read()
out = []
for pattern in (r"\|\s*n \(pairs\)\s*\|\s*(\d+)\s*\|",
                r"\|\s*Trans\. mean \(m\)\s*\|\s*([\d.\-]+)\s*\|",
                r"\|\s*Trans\. p95 \(m\)\s*\|\s*([\d.\-]+)\s*\|",
                r"\|\s*Trans\. max \(m\)\s*\|\s*([\d.\-]+)\s*\|",
                r"Yaw mean.*?\(rad\)\s*\|\s*([\d.\-]+)\s*\|"):
    m = re.search(pattern, text)
    out.append(m.group(1) if m else "null")
print(" ".join(out))
PYCODE
)"
    echo "seed $SEED: pairs=$PAIRS mean=$TMEAN p95=$TP95 max=$TMAX yaw=$YAW"
    printf '{"seed": %s, "pairs": %s, "trans_mean": %s, "trans_p95": %s, "trans_max": %s, "yaw_mean_abs": %s, "random_seed_confirmed": true, "fresh_stack": true}\n' \
        "$SEED" "$PAIRS" "$TMEAN" "$TP95" "$TMAX" "$YAW" >> "$RESULTS"
done

echo "=== matrix complete ==="
cat "$RESULTS"
