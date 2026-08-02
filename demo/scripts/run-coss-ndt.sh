#!/usr/bin/env bash
# Drive the whole COSS NDT replay demo: bring the stack up, seed the initial
# pose, replay the bag, record diagnostics, and leave the stack running so the
# result can be inspected in RViz.
#
# Normally invoked as `just demo run`; runnable directly for debugging.
#
# Environment:
#   LABEL       run label, becomes part of the output directory name
#   RVIZ        true|false (default: true when DISPLAY is set)
#   SCALE       wheel-speed correction, "" disables the scaler (default 0.5)
#   SEED_POSE   true|false, publish the recorded initial pose (default true)
#   KEEP_UP     true|false, leave the stack running at the end (default true)
#   RATE        rosbag playback rate (default 1.0)
#
# Interrupting the demo (Ctrl-C) always tears everything down, KEEP_UP or not.
# The children are deliberately setsid'd so the stack can outlive a *successful*
# run; that also means no signal reaches them by way of the terminal, so this
# script has to kill them itself. Everything it starts is registered in
# CLEANUP_PGIDS as it goes.
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

LABEL="${LABEL:-coss-ndt}"
RVIZ="${RVIZ:-$([[ -n "${DISPLAY:-}" ]] && echo true || echo false)}"
SCALE="${SCALE-0.5}"
SEED_POSE="${SEED_POSE:-true}"
KEEP_UP="${KEEP_UP:-true}"
RATE="${RATE:-1.0}"

BAG="$REPO/data/rosbags/outdoor_20251226_153115"
MAP="$REPO/data/COSS-map-planning"
OUT="$REPO/tmp/demo-runs/${LABEL}_$(date +%Y%m%d_%H%M%S)"
DEMO="$REPO/demo/scripts"

BAG_DURATION=157      # the recording is 157 s: parked 115.7 s, then a 41 s drive
MAP_LOAD_WAIT_MAX=240 # ceiling on the wait for the 4.9 M-point PCD
SEED_DELAY=8          # into playback, so scans and /clock are flowing

say() { printf '\033[1;36m[demo]\033[0m %s\n' "$*"; }

# ---- teardown ------------------------------------------------------------
STACK_PGID=""          # the launch, kept alive on a clean finish if KEEP_UP
CLEANUP_PGIDS=()       # helpers, never outlive this script
COMPLETED=false

register() { [[ -n "${1:-}" ]] && CLEANUP_PGIDS+=("$1"); }

pgid_of() { ps -o pgid= -p "$1" 2>/dev/null | tr -d ' '; }

kill_pgid() {
    local pgid="$1" sig="${2:-TERM}"
    [[ -n "$pgid" ]] && kill "-$sig" -- "-$pgid" 2>/dev/null
    return 0
}

stop_helpers() {
    for pgid in "${CLEANUP_PGIDS[@]:-}"; do kill_pgid "$pgid" INT; done
    sleep 2
    for pgid in "${CLEANUP_PGIDS[@]:-}"; do kill_pgid "$pgid" KILL; done
    pkill -f "$DEMO/velocity_scaler.py" 2>/dev/null
    return 0
}

stop_stack() {
    [[ -z "$STACK_PGID" ]] && return 0
    kill_pgid "$STACK_PGID" TERM
    sleep 5
    kill_pgid "$STACK_PGID" KILL
    # play_launch's own wrapper regularly survives a group signal
    for pid in $(pgrep -f "play_launch.*logging_simulation" 2>/dev/null); do
        kill -9 "$pid" 2>/dev/null
    done
    pkill -9 -f component_container 2>/dev/null
    pkill -9 -f rviz2 2>/dev/null
    return 0
}

on_interrupt() {
    trap '' INT TERM      # a second Ctrl-C must not cut the teardown short
    echo
    say "interrupted, tearing down"
    stop_helpers
    stop_stack
    say "stopped"
    exit 130
}

on_exit() {
    $COMPLETED && return 0
    stop_helpers
    stop_stack
}

trap on_interrupt INT TERM
trap on_exit EXIT

mkdir -p "$OUT"

set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u

# ---- a previous stack would fight this one for topics --------------------
for pid in $(pgrep -f "play_launch.*logging_simulation" 2>/dev/null); do
    kill_pgid "$(pgid_of "$pid")" TERM
done
sleep 5
pkill -9 -f component_container 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
pkill -f velocity_scaler.py 2>/dev/null
sleep 2

say "output   $OUT"
say "rviz=$RVIZ  scale=${SCALE:-off}  seed_pose=$SEED_POSE  rate=$RATE"
command -v nvidia-smi >/dev/null && \
    nvidia-smi --query-gpu=memory.total,memory.free --format=csv > "$OUT/gpu_before.txt"

# ---- 1. stack ------------------------------------------------------------
# use_gnss:=false because this bag's fix is single point with ~20 m of scatter;
# letting it auto-initialise puts the vehicle somewhere different every run.
setsid bash -c "play_launch launch --web-addr 0.0.0.0:8081 \
    autosdv_launch logging_simulation.launch.yaml \
    pose_source:=cuda_ndt \
    map_path:=$MAP \
    use_gnss:=false \
    rviz:=$RVIZ" > "$OUT/launch.log" 2>&1 &

sleep 5
PLAY_PID=$(pgrep -f "play_launch.*logging_simulation" | head -1)
if [[ -z "$PLAY_PID" ]]; then
    say "play_launch did not start; see $OUT/launch.log"
    exit 1
fi
STACK_PGID=$(pgid_of "$PLAY_PID")
echo "$STACK_PGID" > "$OUT/pgid.txt"
echo "$STACK_PGID" > "$REPO/tmp/demo-runs/PGID"
say "stack pgid=$STACK_PGID  (just demo stop)"

# Readiness comes from the matcher's own log, not from `ros2 node list`: with
# 120+ nodes the daemon's discovery is slow and partial, and it reported 24 of
# them while the stack was perfectly healthy. The log line below is emitted once
# the PCD is voxelised, which is exactly the thing worth waiting for, so this
# also replaces a fixed sleep with the real condition.
NDT_LOG="$REPO/play_log/latest/node/ndt_scan_matcher/err"
say "waiting for NDT to load the map (up to ${MAP_LOAD_WAIT_MAX}s)"
for _ in $(seq 1 "$MAP_LOAD_WAIT_MAX"); do
    [[ -f "$NDT_LOG" ]] && grep -q "NDT target updated with map" "$NDT_LOG" && break
    sleep 1
done
if ! { [[ -f "$NDT_LOG" ]] && grep -q "NDT target updated with map" "$NDT_LOG"; }; then
    say "NDT never loaded the map; see $NDT_LOG and $OUT/launch.log"
    exit 1
fi
grep -m1 "Target grid created" "$NDT_LOG" | sed 's/^/[demo] /'
ros2 node list > "$OUT/nodes.txt" 2>&1 || true   # a snapshot for the record, not a gate
sleep 3

# ---- 2. helpers ----------------------------------------------------------
PLAY_REMAP=()
if [[ -n "$SCALE" ]]; then
    SCALE="$SCALE" setsid python3 "$DEMO/velocity_scaler.py" > "$OUT/scaler.log" 2>&1 &
    register "$(pgid_of $!)"
    PLAY_REMAP=(--remap /vehicle/status/velocity_status:=/vehicle/status/velocity_status_raw)
    sleep 2
fi

setsid ros2 bag record -o "$OUT/bag" \
    --regex "(/localization/.*|/initialpose.*|/vehicle/status/.*|/diagnostics|/tf|/tf_static)" \
    > "$OUT/record.log" 2>&1 &
REC_PGID=$(pgid_of $!)
register "$REC_PGID"
sleep 3

if [[ "$SEED_POSE" == "true" ]]; then
    ( sleep "$SEED_DELAY"; python3 "$DEMO/seed_initialpose.py" > "$OUT/seed.log" 2>&1 ) &
    register "$(pgid_of $!)"
    say "initial pose will be seeded ${SEED_DELAY}s into playback"
else
    say "no pose seeding: set it yourself with RViz's 2D Pose Estimate"
fi

# ---- 3. replay -----------------------------------------------------------
# Foreground, so Ctrl-C reaches it directly and the traps above do the rest.
say "replaying ${BAG_DURATION}s: parked ~115s, then a 41s drive"
ros2 bag play "$BAG" --clock -r "$RATE" "${PLAY_REMAP[@]}" > "$OUT/play.log" 2>&1

sleep 3
kill_pgid "$REC_PGID" INT       # let rosbag2 close the file cleanly
sleep 6
pkill -f "$DEMO/velocity_scaler.py" 2>/dev/null

command -v nvidia-smi >/dev/null && \
    nvidia-smi --query-gpu=memory.total,memory.free --format=csv > "$OUT/gpu_after.txt"

# ---- 4. verdict ----------------------------------------------------------
IMU_ERRS=$(grep -c "Please publish TF" play_log/latest/node/imu_corrector_node/err 2>/dev/null || echo 0)
say "imu_corrector TF errors: $IMU_ERRS  (must be 0)"
say "run: $OUT"

python3 "$REPO/scripts/testing/localization/summarize_ndt_run.py" "$OUT" \
    2>/dev/null | tee "$OUT/summary.txt" | grep -vE "^\[INFO|^\[WARN" || true

# Only now, with a recording on disk, is this run worth pointing LATEST at.
echo "$OUT" > "$REPO/tmp/demo-runs/LATEST"
COMPLETED=true
if [[ "$KEEP_UP" == "true" ]]; then
    say "stack still running (pgid=$STACK_PGID) -- inspect in RViz, then: just demo stop"
else
    stop_stack
    say "stack stopped"
fi
