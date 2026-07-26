#!/usr/bin/env bash
# scripts/2dlidar/run-seed-matrix.sh — Phase 3e Task 5: multi-seed
# end-to-end evaluation.
#
# Runs the sample-site particle_filter replay for a 2-configuration x
# 5-seed matrix, SEQUENTIALLY (concurrent ROS graphs on one machine
# cross-talk), and appends one machine-readable JSON line per cell to
# data/rosbags/phase3/seedmatrix/results.jsonl:
#   {"config": ..., "seed": ..., "pairs": ..., "trans_mean": ...,
#    "trans_p95": ..., "trans_max": ..., "yaw_mean_abs": ...}
#
# Configurations:
#   upstream — Phase 3c Lever-2 tuned baseline (no Phase 3e fixes)
#   fixed    — Phase 3c Lever-2 tuning + all three Phase 3e fixes
#              (sensor_model_variant=normalized_short, skip_nonfinite_beams,
#              update_on_new_scan_only)
#
# Idempotent: a cell whose result line already exists in results.jsonl is
# skipped, so an interrupted matrix resumes where it left off. Run once,
# unattended:
#   setsid bash scripts/2dlidar/run-seed-matrix.sh > tmp/seedmatrix.log 2>&1 &
# then poll data/rosbags/phase3/seedmatrix/results.jsonl (wc -l) rather
# than waiting on the background job.
#
# Phase 4 Task 1: MATRIX_CONFIGS (default "upstream fixed", unchanged) and
# MATRIX_OUT_SUBDIR (default "seedmatrix", unchanged) let a caller run a
# separate matrix -- e.g. CONFIGS="fixed" with INITPOSE_SOURCE=gnss -- into
# its own results.jsonl/output dir without touching the Phase 3e results.
# INITPOSE_SOURCE itself is just forwarded to run-particle-filter.sh
# (exported below); unset (the default) is byte-identical to every Phase 3e
# invocation of this script.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$REPO_DIR"

MATRIX_OUT_SUBDIR="${MATRIX_OUT_SUBDIR:-seedmatrix}"
OUT_DIR="$REPO_DIR/data/rosbags/phase3/$MATRIX_OUT_SUBDIR"
RESULTS="$OUT_DIR/results.jsonl"
mkdir -p "$OUT_DIR" "$REPO_DIR/tmp"

SEEDS="1 2 3 4 5"
CONFIGS="${MATRIX_CONFIGS:-upstream fixed}"
export INITPOSE_SOURCE="${INITPOSE_SOURCE:-gt_bag}"

# --- Sample-site env, common to both configurations (per task-5-brief) ---
export BAG="${BAG:-$REPO_DIR/data/rosbags/phase3/sample_ndt_gt}"
export GT_BAG="${GT_BAG:-$REPO_DIR/data/rosbags/phase3/sample_ndt_gt}"
export POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/sensing/lidar/top/pointcloud_raw_ex}"
export VELOCITY_TOPIC="${VELOCITY_TOPIC:-/vehicle/status/velocity_status}"
export IMU_TOPIC="${IMU_TOPIC:-/sensing/imu/tamagawa/imu_raw}"
export IMU_YAW_SIGN="${IMU_YAW_SIGN:--1.0}"
export MAP_YAML="${MAP_YAML:-$REPO_DIR/data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml}"
export SCAN_MIN_HEIGHT="${SCAN_MIN_HEIGHT:-1.91611}"
export SCAN_MAX_HEIGHT="${SCAN_MAX_HEIGHT:-2.21611}"
export SCAN_RANGE_MAX="${SCAN_RANGE_MAX:-60.0}"
export PF_MAX_RANGE="${PF_MAX_RANGE:-60.0}"
export PF_SQUASH="${PF_SQUASH:-3.0}"
export PF_DISP_THETA="${PF_DISP_THETA:-0.1}"

RUN_PF="$SCRIPT_DIR/run-particle-filter.sh"

# --- Defense-in-depth process supervision (run-particle-filter.sh already
# cleans up its own children via its own EXIT trap; this adds a second
# layer so an interrupted *driver* doesn't orphan a whole run-in-progress) ---
CURRENT_PID=""
kill_pgid() {
    local pid="$1"
    [ -z "$pid" ] && return 0
    local pgid
    pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')" || pgid=""
    [ -n "$pgid" ] && kill -- -"$pgid" 2>/dev/null || true
}
driver_cleanup() {
    if [ -n "$CURRENT_PID" ]; then
        kill_pgid "$CURRENT_PID"
        sleep 2
        kill_pgid "$CURRENT_PID"
    fi
}
trap driver_cleanup EXIT INT TERM

# --- ROS environment (sourced once, guarded per repo convention) ---
# shellcheck disable=SC1091
set +u
source /opt/autoware/1.5.0/setup.bash
source "$REPO_DIR/install/setup.bash"
set -u

result_exists() {
    local config="$1" seed="$2"
    [ -f "$RESULTS" ] || return 1
    jq -e --arg c "$config" --argjson s "$seed" \
        'select(.config == $c and .seed == $s)' "$RESULTS" >/dev/null 2>&1
}

append_error_result() {
    local config="$1" seed="$2" reason="$3"
    jq -nc --arg c "$config" --argjson s "$seed" --arg r "$reason" \
        '{config: $c, seed: $s, pairs: 0, trans_mean: null, trans_p95: null,
          trans_max: null, yaw_mean_abs: null, error: $r}' >> "$RESULTS"
}

CELL_NUM=0
CELL_TOTAL=$(( $(echo "$CONFIGS" | wc -w) * $(echo "$SEEDS" | wc -w) ))

for CONFIG in $CONFIGS; do
    for SEED in $SEEDS; do
        CELL_NUM=$((CELL_NUM + 1))
        PROGRESS="[cell $CELL_NUM/$CELL_TOTAL config=$CONFIG seed=$SEED]"

        if result_exists "$CONFIG" "$SEED"; then
            echo "$PROGRESS already in $RESULTS -- skipping"
            continue
        fi

        echo "$PROGRESS starting at $(date -Iseconds)"

        OUT_BAG="$OUT_DIR/${CONFIG}_s${SEED}"
        RUN_LOG="$REPO_DIR/tmp/seedmatrix_${CONFIG}_s${SEED}_run.log"
        REPORT="$OUT_DIR/${CONFIG}_s${SEED}_report.md"

        # --- per-configuration env overrides ---
        if [ "$CONFIG" = "fixed" ]; then
            export PF_SENSOR_MODEL_VARIANT="normalized_short"
            export PF_SKIP_NONFINITE="true"
            export PF_UPDATE_ON_SCAN_ONLY="true"
        else
            export PF_SENSOR_MODEL_VARIANT="upstream"
            export PF_SKIP_NONFINITE="false"
            export PF_UPDATE_ON_SCAN_ONLY="false"
        fi
        export PF_RANDOM_SEED="$SEED"
        export OUT_BAG="$OUT_BAG"

        # --- run the replay (setsid so we can PGID-kill the whole tree if
        # the driver itself is interrupted mid-run; bounded by `timeout` as
        # a hang safety net -- expected ~110s per cell) ---
        RUN_RC=0
        setsid timeout 300 bash "$RUN_PF" > "$RUN_LOG" 2>&1 &
        CURRENT_PID=$!
        wait "$CURRENT_PID" || RUN_RC=$?
        CURRENT_PID=""

        if [ "$RUN_RC" -ne 0 ]; then
            echo "$PROGRESS run-particle-filter.sh exited $RUN_RC (see $RUN_LOG) -- attempting comparison anyway"
        fi

        if [ ! -f "${OUT_BAG}/metadata.yaml" ]; then
            echo "$PROGRESS no output bag at $OUT_BAG -- recording error result"
            append_error_result "$CONFIG" "$SEED" "no_output_bag rc=$RUN_RC"
            continue
        fi

        # --- compare against GT (literal invocation per task-5-brief) ---
        COMPARE_LOG="$REPO_DIR/tmp/seedmatrix_${CONFIG}_s${SEED}_compare.log"
        COMPARE_RC=0
        python3 "$SCRIPT_DIR/compare_poses.py" "$GT_BAG" "$OUT_BAG" \
            --no-motion-window --gt-time-source bag --out "$REPORT" \
            > "$COMPARE_LOG" 2>&1 || COMPARE_RC=$?

        if [ ! -f "$REPORT" ]; then
            echo "$PROGRESS compare_poses.py produced no report (rc=$COMPARE_RC, see $COMPARE_LOG)"
            append_error_result "$CONFIG" "$SEED" "compare_failed rc=$COMPARE_RC"
            continue
        fi

        # --- parse the "Full-Overlap Statistics" table out of the report
        # (avoids a second, costly bag read just to get numbers already
        # computed by compare_poses.py) ---
        PARSED="$(python3 - "$REPORT" <<'PYCODE'
import re
import sys

text = open(sys.argv[1]).read()

vals = []


def grab(pattern):
    m = re.search(pattern, text)
    vals.append(m.group(1) if m else "NaN")

grab(r"\|\s*n \(pairs\)\s*\|\s*(\d+)\s*\|")
grab(r"\|\s*Trans\. mean \(m\)\s*\|\s*([\d.\-]+)\s*\|")
grab(r"\|\s*Trans\. max \(m\)\s*\|\s*([\d.\-]+)\s*\|")
grab(r"\|\s*Trans\. p95 \(m\)\s*\|\s*([\d.\-]+)\s*\|")
grab(r"Yaw mean.*?\(rad\)\s*\|\s*([\d.\-]+)\s*\|")

# one whitespace-separated line: bash `read` consumes a single line only
print(" ".join(vals))
PYCODE
)"
        read -r PAIRS TRANS_MEAN TRANS_MAX TRANS_P95 YAW_MEAN_ABS <<< "$PARSED"

        if [[ "$PARSED" == *NaN* ]] || [ -z "${YAW_MEAN_ABS:-}" ]; then
            echo "$PROGRESS could not parse stats table out of $REPORT"
            append_error_result "$CONFIG" "$SEED" "unparseable_report"
            continue
        fi

        jq -nc --arg c "$CONFIG" --argjson s "$SEED" \
            --argjson pairs "$PAIRS" --argjson tm "$TRANS_MEAN" \
            --argjson tp95 "$TRANS_P95" --argjson tmax "$TRANS_MAX" \
            --argjson yaw "$YAW_MEAN_ABS" \
            '{config: $c, seed: $s, pairs: $pairs, trans_mean: $tm,
              trans_p95: $tp95, trans_max: $tmax, yaw_mean_abs: $yaw}' \
            >> "$RESULTS" || {
            echo "$PROGRESS jq append failed -- recording error result"
            append_error_result "$CONFIG" "$SEED" "append_failed"
            continue
        }

        echo "$PROGRESS done: pairs=$PAIRS trans_mean=$TRANS_MEAN trans_p95=$TRANS_P95 yaw_mean_abs=$YAW_MEAN_ABS"
    done
done

echo "Seed matrix complete: $CELL_TOTAL cells, results in $RESULTS"
