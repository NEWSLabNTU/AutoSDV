#!/usr/bin/env bash
# scripts/2dlidar/run-particle-filter.sh — Phase 3 Task 3 gate:
# bring up the vendored particle_filter (MCL) against the COSS occupancy
# grid, feed it /scan (from pointcloud_to_laserscan) and /odom (from
# wheel_imu_odom.py), seed it with the first GT pose, replay the outdoor
# bag, and record /pf/viz/inferred_pose (+ /pf/pose/odom) for Task 4.
#
# Env overrides: BAG, OUT (alias OUT_BAG), RATE, MAP_YAML, GT_BAG,
# INITPOSE_DELAY, POINTCLOUD_TOPIC, VELOCITY_TOPIC, IMU_TOPIC, IMU_YAW_SIGN
# (default 1.0; set -1.0 to compensate for IMUs whose z-rate is inverted
# relative to map-frame yaw, e.g. the sample_sensor_kit Tamagawa unit --
# see wheel_imu_odom.py docstring), SCAN_MIN_HEIGHT/SCAN_MAX_HEIGHT
# (defaults -0.15/0.15, unchanged for COSS -- see Step 2 frame-fix note
# below for why the sample-site sensor_kit needs different values),
# PF_MAX_RANGE/PF_SQUASH/PF_DISP_X/PF_DISP_Y/PF_DISP_THETA/SCAN_RANGE_MAX
# (PF tuning passthroughs, defaults preserve the untuned vendored values
# -- see the tuning note above the env-var block below), PF_USE_ESS_GATE
# (default false)/PF_ESS_RATIO (default 0.5) (Phase 3c Lever 3
# effective-sample-size resampling gate -- see the note above the
# env-var block below), PF_INIT_TIMEOUT_S
# (default 60; raise for large/fine-resolution grids whose CDDT precompute
# takes longer), PF_LIKELIHOOD_FIELD (default false)/PF_LF_WINDOW_M
# (default 40.0)/PF_LF_RES_M (default 0.5)/PF_LF_PERIOD_S (default
# 1.0)/PF_LF_LOG_FLOOR (default 20.0) (Phase 3d Task 4 live
# likelihood-field debug grid on /pf/debug/likelihood_field -- see the
# passthrough block below), PF_SENSOR_MODEL_VARIANT (default "upstream")/
# PF_LAMBDA_SHORT (default 1.0, 1/pixel) (Phase 3e Task 2 sensor-model
# p_short normalisation -- see the passthrough block below), PF_SKIP_NONFINITE
# (default false) (Phase 3e Task 3: drop non-finite/no-return observed beams
# from sensor-model evaluation entirely instead of letting them fall into the
# max-range table bucket -- see the passthrough block below).
#
# NOTE on PF_LF_LOG_FLOOR: the default of 20.0 nats is far narrower than
# this filter's actual likelihood dynamic range, measured at 47-50 nats
# between the true pose and the field's argmax (Phase 3d Task 5b). At the
# default, nearly every cell -- including the true pose -- clips to 0 and
# the overlay looks uniformly black with a few isolated hot pixels. That
# is a display artifact, not underflow. Raise it (e.g. 120.0) to see the
# field's structure. See docs/reports/2dlidar-phase3d-instrumentation.md.
#
# All default to
# the COSS outdoor-bag values below, so an unmodified invocation is
# byte-identical to the original COSS run.
#
# --- Frame-fix note (sample-site second root cause) ---
# pointcloud_to_laserscan's default target_frame is "" (empty), which
# means "keep the input cloud's own frame" -- it flattens the cloud
# in SENSOR frame (e.g. velodyne_top), not robot/base_link frame.
# particle_filter assumes /scan is already in the robot's frame (it does
# no tf2 lookups at all -- see the particle_filter source-finding notes
# above). For sample_sensor_kit, sensor_kit_base_link -> velodyne_top_base_link
# has yaw=1.575 rad (+90 deg, from
# /opt/autoware/1.5.0/share/sample_sensor_kit_description/config/sensor_kit_calibration.yaml),
# so a /scan built in sensor frame is rotated ~90 deg from base_link,
# which PF's sensor model then scores against particles expressed in
# base_link/map frame -- producing a large systematic yaw error
# independent of anything odometry-related. Fix: set
# `target_frame:=base_link` on pointcloud_to_laserscan so it transforms
# the cloud into base_link (via tf2, using /tf_static from the bag) BEFORE
# flattening. This also means the z-band filter (min_height/max_height)
# is now measured in base_link frame, not sensor frame, so its defaults
# must shift by the sensor's height above base_link (~2.0-2.1 m for
# sample_sensor_kit's velodyne_top -- see SCAN_MIN_HEIGHT/SCAN_MAX_HEIGHT
# above). COSS callers are unaffected (defaults unchanged, COSS's sensor
# kit was not audited for this issue and is out of scope here).
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
OUT="${OUT_BAG:-${OUT:-$REPO_DIR/data/rosbags/phase3/pf_run}}"
RATE="${RATE:-1.0}"
MAP_YAML="${MAP_YAML:-$REPO_DIR/data/COSS-map-planning/occupancy_grid.yaml}"
GT_BAG="${GT_BAG:-$REPO_DIR/data/rosbags/phase3/ndt_gt}"
INITPOSE_DELAY="${INITPOSE_DELAY:-5}"
POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/sensing/lidar/velodyne_points}"
VELOCITY_TOPIC="${VELOCITY_TOPIC:-/vehicle/status/velocity_status}"
IMU_TOPIC="${IMU_TOPIC:-/sensing/camera/zedxm/imu/data}"
IMU_YAW_SIGN="${IMU_YAW_SIGN:-1.0}"
# ROS 2 rejects integer literals for double-typed parameters (rclcpp
# raises InvalidParameterTypeException, e.g. "range_max" on
# pointcloud_to_laserscan) -- normalize every double-typed env override
# below to decimal form with printf so a caller passing e.g.
# SCAN_RANGE_MAX=60 (no decimal) can't crash the node it's forwarded to.
to_float() { printf '%.6f' "$1"; }
SCAN_MIN_HEIGHT="$(to_float "${SCAN_MIN_HEIGHT:--0.15}")"
SCAN_MAX_HEIGHT="$(to_float "${SCAN_MAX_HEIGHT:-0.15}")"
# PF tuning passthroughs (defaults preserve the untuned vendored values --
# see PF_* below and pf_params.yaml generation). Motivated by the
# longitudinal-corridor-aliasing diagnosis: PF tracks well through curves
# but diverges on long straights, where down-corridor beams don't reach
# far enough to disambiguate position along the corridor axis, and a
# peaky sensor-model likelihood lets resampling collapse onto an aliased
# (wrong) hypothesis. PF_MAX_RANGE/SCAN_RANGE_MAX raise the beam range so
# down-corridor structure (mapped from later scans, in the
# scan-accumulated grid) becomes visible; PF_SQUASH flattens the
# likelihood (fewer overconfident resampling collapses); PF_DISP_*
# lowers motion-model noise to trust the now-validated wheel+IMU
# odometry prior more.
PF_MAX_RANGE="$(to_float "${PF_MAX_RANGE:-30.0}")"
PF_SQUASH="$(to_float "${PF_SQUASH:-2.2}")"
PF_DISP_X="$(to_float "${PF_DISP_X:-0.05}")"
PF_DISP_Y="$(to_float "${PF_DISP_Y:-0.025}")"
PF_DISP_THETA="$(to_float "${PF_DISP_THETA:-0.25}")"
SCAN_RANGE_MAX="$(to_float "${SCAN_RANGE_MAX:-30.0}")"
# Phase 3c Lever 3: effective-sample-size (ESS) resampling gate
# (particle_filter submodule, branch autosdv). Resample only when
# N_eff = 1/sum(w^2) < PF_ESS_RATIO * max_particles, instead of every
# update -- aims to stop the particle filter from collapsing onto an
# aliased (wrong) hypothesis on featureless corridor segments via a
# resampling step driven by a peaked-but-wrong likelihood. Default
# (false) preserves upstream behavior (always resample).
PF_USE_ESS_GATE="${PF_USE_ESS_GATE:-false}"
PF_ESS_RATIO="$(to_float "${PF_ESS_RATIO:-0.5}")"
# particle_filter's CDDT range-method precompute cost scales with grid
# cell count (theta_discretization x width x height); a finer-resolution
# grid (e.g. Phase 3c Lever 2's 0.05 m grid, ~4x the cells of the 0.1 m
# grid) can take noticeably longer than the original 60s timeout allowed
# for. Override PF_INIT_TIMEOUT_S for such runs rather than lowering
# fidelity to fit the old timeout.
PF_INIT_TIMEOUT_S="${PF_INIT_TIMEOUT_S:-60}"
# Phase 3d: per-update diagnostics passthrough (particle_filter submodule,
# branch autosdv). Defaults preserve today's behavior exactly -- no JSONL
# file, no /pf/debug/* topics, zero measurable overhead.
# PF_DIAG_ENABLE: append one JSON record per update (per PF_DIAG_EVERY) to
#   PF_DIAG_PATH (auto-generated under ./tmp if empty).
# PF_DIAG_TOPICS: publish the same per-update scalars as std_msgs/Float32
#   on /pf/debug/{n_eff,weight_entropy,pose_cov_trace,update_hz,
#   frac_clamped,frac_short} for live PlotJuggler inspection.
PF_DIAG_ENABLE="${PF_DIAG_ENABLE:-false}"
PF_DIAG_PATH="${PF_DIAG_PATH:-}"
PF_DIAG_EVERY="${PF_DIAG_EVERY:-1}"
PF_DIAG_BEAM_ARRAYS="${PF_DIAG_BEAM_ARRAYS:-false}"
PF_DIAG_TOPICS="${PF_DIAG_TOPICS:-false}"
# Phase 3d Task 4: live likelihood-field debug grid passthrough
# (particle_filter submodule, branch autosdv). Disabled by default (no
# publisher, no extra raycasts). When enabled, publishes a coarse
# pose-grid likelihood surface centred on the inferred pose as
# nav_msgs/OccupancyGrid on /pf/debug/likelihood_field, at most once
# every PF_LF_PERIOD_S seconds -- see build_likelihood_field() in the
# fork for the encoding (log-weight, max-subtracted, floor-clipped,
# mapped to 0..100).
PF_LIKELIHOOD_FIELD="${PF_LIKELIHOOD_FIELD:-false}"
PF_LF_WINDOW_M="$(to_float "${PF_LF_WINDOW_M:-40.0}")"
PF_LF_RES_M="$(to_float "${PF_LF_RES_M:-0.5}")"
PF_LF_PERIOD_S="$(to_float "${PF_LF_PERIOD_S:-1.0}")"
PF_LF_LOG_FLOOR="$(to_float "${PF_LF_LOG_FLOOR:-20.0}")"
# Phase 3e Task 2: sensor-model variant passthrough (particle_filter
# submodule, branch autosdv, particle_filter/sensor_model.py). Default
# ("upstream") preserves today's (unnormalised p_short) table exactly.
# PF_SENSOR_MODEL_VARIANT=normalized_short switches to the per-column-
# normalised short-reading component (docs/research/localization/
# 2d_mcl_algorithm.md sec 5.1); PF_LAMBDA_SHORT (1/pixel) only affects
# that variant -- see sensor_model.py's module docstring for why its
# value is resolution-dependent.
PF_SENSOR_MODEL_VARIANT="${PF_SENSOR_MODEL_VARIANT:-upstream}"
PF_LAMBDA_SHORT="$(to_float "${PF_LAMBDA_SHORT:-1.0}")"
# Phase 3e Task 3: skip_nonfinite_beams passthrough (particle_filter
# submodule, branch autosdv, particle_filter/particle_filter.py). Default
# (false) preserves today's behavior exactly -- non-finite (no-return)
# observed beams fall into the max-range sensor-model bucket, unchanged.
# PF_SKIP_NONFINITE=true drops non-finite beams (and their predicted-range
# columns) from the sensor-model evaluation entirely -- see
# docs/research/localization/2d_mcl_algorithm.md sec 5.2. PF_MIN_FINITE_BEAMS
# (default 10) (Phase 3e Task 3 robustness guard) skips the correction
# entirely when fewer than this many finite beams survive masking -- only
# reachable when PF_SKIP_NONFINITE=true. PF_UPDATE_ON_SCAN_ONLY (default
# false) (Phase 3e Task 4: docs/research/localization/2d_mcl_algorithm.md
# sec 5.3) gates the MCL correction to run once per scan instead of once per
# odometry message (odom arrives ~2x scan rate, so the default double-counts
# each scan into the likelihood); odometry deltas accumulate across the
# skipped odom callbacks via an exact rotation composition. NOTE: pose/tf
# publishing only happens on a correction, so enabling this also drops the
# publish rate from odom rate to scan rate.
PF_SKIP_NONFINITE="${PF_SKIP_NONFINITE:-false}"
# Phase 3e Task 4 passthrough (particle_filter submodule, branch autosdv,
# particle_filter/particle_filter.py). odomCB fires at odom rate (~20 Hz)
# while scans arrive at ~10 Hz, so upstream's "correct on every odomCB"
# double-counts each scan into the likelihood (see
# docs/research/localization/2d_mcl_algorithm.md sec 5.3).
# PF_UPDATE_ON_SCAN_ONLY=true gates the correction to run only once per
# not-yet-consumed scan; odometry deltas accumulate (exact rotation
# composition, not approximated) across the skipped odomCB calls -- see
# compose_odometry_delta()/should_run_correction() in the fork. NOTE:
# pose/tf publishing only happens on a correction, so this also drops the
# publish rate from odom rate to scan rate -- see the Task 4 report.
# Default (false) preserves today's behavior exactly.
PF_UPDATE_ON_SCAN_ONLY="${PF_UPDATE_ON_SCAN_ONLY:-false}"
# Phase 3e Task 3 robustness guard (flagged during Task 3 review):
# PF_MIN_FINITE_BEAMS is the minimum surviving finite-beam count (after
# skip_nonfinite_beams masking) required to run the sensor-model
# evaluation; below it, the correction is skipped for that update (uniform
# weights) instead of letting a handful of beams dominate. Only reachable
# when PF_SKIP_NONFINITE=true -- inert (cannot trigger, defaults
# unaffected) when skip_nonfinite_beams is false.
PF_MIN_FINITE_BEAMS="${PF_MIN_FINITE_BEAMS:-10}"
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
# NOTE: heredoc is unquoted (variable substitution) so PF_* env overrides
# above reach the tuned parameters; all other values stay literal (no `$`
# in them) so this is safe.
mkdir -p "$REPO_DIR/tmp"
cat > "$PARAMS_FILE" <<EOF
particle_filter:
  ros__parameters:
    scan_topic: '/scan'
    odometry_topic: '/odom'
    angle_step: 18
    max_particles: 4000
    squash_factor: $PF_SQUASH
    viz: 1
    max_viz_particles: 60
    range_method: 'cddt'
    theta_discretization: 112
    max_range: $PF_MAX_RANGE
    fine_timing: 0
    publish_odom: 1
    z_short: 0.01
    z_max: 0.07
    z_rand: 0.12
    z_hit: 0.75
    sigma_hit: 8.0
    sensor_model_variant: '$PF_SENSOR_MODEL_VARIANT'
    sensor_model_lambda_short: $PF_LAMBDA_SHORT
    skip_nonfinite_beams: $PF_SKIP_NONFINITE
    min_finite_beams: $PF_MIN_FINITE_BEAMS
    update_on_new_scan_only: $PF_UPDATE_ON_SCAN_ONLY
    motion_dispersion_x: $PF_DISP_X
    motion_dispersion_y: $PF_DISP_Y
    motion_dispersion_theta: $PF_DISP_THETA
    rangelib_variant: 2
    use_ess_gate: $PF_USE_ESS_GATE
    ess_threshold_ratio: $PF_ESS_RATIO
    diag_enable: $PF_DIAG_ENABLE
    diag_path: '$PF_DIAG_PATH'
    diag_every: $PF_DIAG_EVERY
    diag_beam_arrays: $PF_DIAG_BEAM_ARRAYS
    diag_topics: $PF_DIAG_TOPICS
    likelihood_field_enable: $PF_LIKELIHOOD_FIELD
    lf_window_m: $PF_LF_WINDOW_M
    lf_res_m: $PF_LF_RES_M
    lf_period_s: $PF_LF_PERIOD_S
    lf_log_floor: $PF_LF_LOG_FLOOR
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
    -r /pointcloud_to_laserscan/input/pointcloud:="$POINTCLOUD_TOPIC" \
    -r /pointcloud_to_laserscan/output/laserscan:=/scan_raw \
    -p target_frame:=base_link \
    -p min_height:="$SCAN_MIN_HEIGHT" -p max_height:="$SCAN_MAX_HEIGHT" \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 \
    -p angle_increment:=0.0043 -p range_min:=0.1 -p range_max:="$SCAN_RANGE_MAX" \
    -p use_sim_time:=true \
    > "$SCAN_LOG" 2>&1 &
SCAN_PID=$!

# --- Step 2a: verify pointcloud_to_laserscan actually stayed up. A bad
# param (e.g. an int literal for a double-typed param like range_max --
# rclcpp raises InvalidParameterTypeException and the node process exits)
# otherwise fails silently here: no /scan ever appears, particle_filter
# still starts and idles waiting on lidar_initialized, and the whole run
# looks like an ordinary (if bad) result -- 0 inferred_pose messages --
# rather than the void/infra failure it actually is. Fail loud instead.
sleep 2
if ! kill -0 "$SCAN_PID" 2>/dev/null; then
    echo "FAIL: scan converter did not start (see scan log)"
    echo "Scan log: $SCAN_LOG"
    tail -n 20 "$SCAN_LOG" || true
    exit 1
fi

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
    -p velocity_topic:="$VELOCITY_TOPIC" -p imu_topic:="$IMU_TOPIC" \
    -p imu_yaw_sign:="$IMU_YAW_SIGN" \
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
    if [ "$elapsed" -ge "$PF_INIT_TIMEOUT_S" ]; then
        echo "FAIL: particle_filter did not finish initializing within ${PF_INIT_TIMEOUT_S}s"
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
