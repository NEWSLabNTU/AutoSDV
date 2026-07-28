#!/usr/bin/env bash
# Replay the COSS rosbag through a pose estimator and record everything needed
# to tell an init failure from a tracking failure.
#
# The bag is parked for its first 115.7 s (NDT init) and drives for the last
# 41.3 s (tracking), so both phases are covered in one run. Captured:
#   - every NDT diagnostic topic (TP, NVTL, iteration_num, exe_time,
#     initial_to_result_distance, ...) into a rosbag
#   - the full launch stdout/stderr (node warnings, skipping_publish, etc.)
#   - /diagnostics, GNSS, EKF and kinematic_state for cross-checking
#
# Per-node stdout/stderr lands in play_log/latest/node/<name>/ as usual.
# Analyse a run with summarize_ndt_run.py in this directory. Findings from the
# first use of this harness: docs/reports/cuda-ndt-coss-replay.md
#
# Usage: scripts/testing/localization/run-ndt-replay.sh [label]
#        POSE_SOURCE=ndt scripts/testing/localization/run-ndt-replay.sh builtin
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$REPO"

LABEL="${1:-run}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$REPO/tmp/ndt-replay/${LABEL}_${STAMP}"
mkdir -p "$OUT"

BAG_IN="$REPO/data/rosbags/outdoor_20251226_153115"
MAP="$REPO/data/COSS-map-planning"
BAG_DURATION=157

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

# Per-run debug JSONL (only produced if the node was built --features debug-output)
export NDT_DEBUG=1
export NDT_DEBUG_FILE="$OUT/ndt_cuda_debug.jsonl"

echo "[harness] output dir: $OUT"
nvidia-smi --query-gpu=memory.total,memory.used,memory.free --format=csv > "$OUT/gpu_before.txt"

# --- 1. launch localization-only stack ------------------------------------
setsid bash -c "play_launch launch --web-addr 0.0.0.0:8081 \
    autosdv_launch logging_simulation.launch.yaml \
    pose_source:=${POSE_SOURCE:-cuda_ndt} \
    map_path:=$MAP \
    rviz:=false \
    launch_perception:=false \
    launch_planning:=false \
    launch_control:=false" > "$OUT/launch.log" 2>&1 &

sleep 5
PLAY_PID=$(pgrep -f "play_launch.*logging_simulation" | head -1)
if [[ -z "$PLAY_PID" ]]; then
    echo "[harness] play_launch failed to start; see $OUT/launch.log" >&2
    exit 1
fi
PGID=$(ps -o pgid= -p "$PLAY_PID" | tr -d ' ')
echo "$PGID" > "$OUT/pgid.txt"
echo "[harness] play_launch pgid=$PGID"

cleanup() {
    echo "[harness] stopping pgid=$PGID"
    kill -- "-$PGID" 2>/dev/null || true
    sleep 5
    kill -9 -- "-$PGID" 2>/dev/null || true
}
trap cleanup EXIT

# --- 2. wait for the NDT node, then for the map to load -------------------
echo "[harness] waiting for ndt_scan_matcher node..."
for _ in $(seq 1 120); do
    if ros2 node list 2>/dev/null | grep -q "ndt_scan_matcher"; then break; fi
    sleep 1
done
ros2 node list > "$OUT/nodes.txt" 2>&1 || true
grep -q ndt_scan_matcher "$OUT/nodes.txt" || {
    echo "[harness] ndt_scan_matcher never appeared" >&2; exit 1; }

echo "[harness] node up; waiting 25 s for PCD map load"
sleep 25

# --- 3. record diagnostics, then replay the bag ---------------------------
setsid ros2 bag record -o "$OUT/diagnostics_bag" \
    --regex "(/localization/.*|/diagnostics|/sensing/gnss/.*|/vehicle/status/velocity_status|/tf|/tf_static)" \
    > "$OUT/record.log" 2>&1 &
REC_PID=$!
sleep 3

echo "[harness] playing bag ($BAG_DURATION s)"
ros2 bag play "$BAG_IN" --clock -r 1.0 > "$OUT/play.log" 2>&1

sleep 5
nvidia-smi --query-gpu=memory.total,memory.used,memory.free --format=csv > "$OUT/gpu_after.txt"

# stop recorder first so the bag closes cleanly
REC_PGID=$(ps -o pgid= -p "$REC_PID" 2>/dev/null | tr -d ' ' || true)
[[ -n "$REC_PGID" ]] && kill -INT -- "-$REC_PGID" 2>/dev/null || true
sleep 5

echo "[harness] done -> $OUT"
