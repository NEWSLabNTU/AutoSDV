#!/usr/bin/env bash
# Race-stack timing benchmark: L1/L2 live loop (bench_live), acados tracker, GPU load (Jetson).
#
#   scripts/race/bench.sh LABEL [--cpus LIST]
#
# Writes tmp/race-bench/LABEL/ (RACE_BENCH_OUT overrides tmp/race-bench). Starts no load itself:
# bring the stack up first, and let LABEL name that load (x86-idle, orin-stack-up, ...).
# Then: scripts/race/bench_report.py DIR... -o F.md
#
# --cpus pins every timed process: bench_live to the first CPU (its seed workers to the rest), and
# acados' record and bench to the first CPU, so the acados column is comparable with L2's.
# load.json gets the start load before anything runs and the end load on exit, failed runs too.
set -euo pipefail

usage() { echo "usage: $0 LABEL [--cpus LIST]" >&2; exit 2; }
[ $# -ge 1 ] || usage
LABEL=$1; shift
CPUS=""
while [ $# -gt 0 ]; do
    case $1 in
        --cpus) [ $# -ge 2 ] || usage; CPUS=$2; shift 2 ;;
        *) usage ;;
    esac
done

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
RACE_ENV=${AUTOSDV_RACE_VENV:-$REPO/.venv-race}/race-env.sh
[ -f "$RACE_ENV" ] || { echo "bench: $RACE_ENV missing; run ./setup.sh --run --only race-stack" >&2; exit 1; }
# shellcheck disable=SC1090
source "$RACE_ENV"   # exports AUTOSDV_RACE_VENV, ACADOS_SOURCE_DIR, SWB (no LD_LIBRARY_PATH, on purpose)
PY=$AUTOSDV_RACE_VENV/bin/python
STUDIES=$REPO/src/planning/external/autoware-drift/src/planning/race_planner/studies
[ -f "$STUDIES/bench_live.py" ] || { echo "bench: submodule autoware-drift not checked out (just checkout)" >&2; exit 1; }
unset RP_DET

# bench_live and acados_tracker read the car and the surfaces from a scenario-world-bridge checkout.
SWB_FILES="src/sim_vehicle_spec/vehicles/drift_mustang.yaml src/swb_isaac_world/config/surface_classes.yaml"
swb_help() {
    echo "bench: $1" >&2
    echo "  SWB must name a scenario-world-bridge checkout with its src/sim_vehicle_ecu and" >&2
    echo "  src/sim_vehicle_spec submodules (and src/swb_isaac_world, for surface_classes.yaml), e.g.:" >&2
    echo "    git clone https://github.com/NEWSLabNTU/scenario-world-bridge.git ~/repos/scenario-world-bridge" >&2
    echo "    git -C ~/repos/scenario-world-bridge submodule update --init src/sim_vehicle_ecu src/sim_vehicle_spec src/swb_isaac_world" >&2
    echo "    export SWB=~/repos/scenario-world-bridge" >&2
    echo "  (exported before ./setup.sh --run --only race-stack, race-env.sh keeps it)" >&2
    exit 1
}
[ -n "${SWB:-}" ] || swb_help "SWB is not set"
[ -d "$SWB" ] || swb_help "SWB=$SWB: no such directory"
for f in $SWB_FILES; do
    [ -f "$SWB/$f" ] || swb_help "SWB=$SWB: missing $f"
done

case ":${LD_LIBRARY_PATH:-}:" in
    *":${ACADOS_SOURCE_DIR:-/nonexistent}/lib:"*)
        echo "bench: WARNING ${ACADOS_SOURCE_DIR}/lib is on LD_LIBRARY_PATH; acados' libhpipm then" \
             "replaces CasADi's (install-race-stack.sh, 'Why no LD_LIBRARY_PATH')" >&2 ;;
esac

OUT=${RACE_BENCH_OUT:-$REPO/tmp/race-bench}/$LABEL
mkdir -p "$OUT"
load1() { cut -d' ' -f1 /proc/loadavg; }
LOAD_START=$(load1)
printf '{"start": %s, "end": null}\n' "$LOAD_START" > "$OUT/load.json"

GPU_PID=""
cleanup() {
    local rc=$?
    printf '{"start": %s, "end": %s, "exit": %s}\n' "$LOAD_START" "$(load1)" "$rc" > "$OUT/load.json"
    if [ -n "$GPU_PID" ]; then kill "$GPU_PID" 2>/dev/null || true; fi
}
trap cleanup EXIT

# The GPU sampler: Tegra's by default; RACE_GPU_SAMPLER names another (any host).
SAMPLER=${RACE_GPU_SAMPLER:-}
if [ -z "$SAMPLER" ] && [ -f /etc/nv_tegra_release ]; then
    SAMPLER=$REPO/scripts/profiling/jetson_gpu_sampler.py
fi
if [ -n "$SAMPLER" ]; then
    "$SAMPLER" -o "$OUT/gpu.csv" &
    GPU_PID=$!
    sleep 2
    if ! kill -0 "$GPU_PID" 2>/dev/null; then
        echo "bench: WARNING ******** the GPU sampler ($SAMPLER) died at startup:" \
             "this run records NO GPU load ********" >&2
        GPU_PID=""
    fi
fi

# the course A,B of Task 4 plus the zones course (~80 s warm)
COURSES=c:asphalt:25:90:8,c:gravel:15:90:8,zones
run() { env -u PYTHONPATH "$PY" "$@"; }
# acados runs on the control loop's core, as bench_live's own process does
PIN=()
if [ -n "$CPUS" ]; then PIN=(taskset -c "${CPUS%%,*}"); fi
run_pinned() { "${PIN[@]}" env -u PYTHONPATH "$PY" "$@"; }

run "$STUDIES/bench_live.py" run --courses "$COURSES" --arm sched --cpus "$CPUS" \
    --label "$LABEL" --out "$OUT"
# separate processes: CasADi's and acados' HPIPM share symbol names
ACADOS_COURSE=c:gravel:15:90:8
run_pinned "$STUDIES/acados_tracker.py" record --course "$ACADOS_COURSE" --out "$OUT" --label "$LABEL"
run_pinned "$STUDIES/acados_tracker.py" bench  --course "$ACADOS_COURSE" --out "$OUT" --label "$LABEL"

echo "bench: $OUT"
