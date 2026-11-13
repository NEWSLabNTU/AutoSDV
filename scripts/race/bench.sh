#!/usr/bin/env bash
# Race-stack timing benchmark: L1/L2 live loop (bench_live), acados tracker, GPU load (Jetson).
#
#   scripts/race/bench.sh LABEL [--cpus LIST]
#
# Writes tmp/race-bench/LABEL/. Starts no load itself: bring the stack up first, and let LABEL
# name that load (x86-idle, orin-stack-up, ...). Then: scripts/race/bench_report.py DIR... -o F.md
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

OUT=$REPO/tmp/race-bench/$LABEL
mkdir -p "$OUT"
GPU_PID=""
cleanup() { [ -n "$GPU_PID" ] && kill "$GPU_PID" 2>/dev/null || true; }
trap cleanup EXIT

load1() { cut -d' ' -f1 /proc/loadavg; }
LOAD_START=$(load1)

if [ -f /etc/nv_tegra_release ]; then
    "$REPO/scripts/profiling/jetson_gpu_sampler.py" -o "$OUT/gpu.csv" &
    GPU_PID=$!
fi

# the course A,B of Task 4 plus the zones course (~80 s warm)
COURSES=c:asphalt:25:90:8,c:gravel:15:90:8,zones
run() { env -u PYTHONPATH "$PY" "$@"; }

run "$STUDIES/bench_live.py" run --courses "$COURSES" --arm sched --cpus "$CPUS" \
    --label "$LABEL" --out "$OUT"
# separate processes: CasADi's and acados' HPIPM share symbol names
ACADOS_COURSE=c:gravel:15:90:8
run "$STUDIES/acados_tracker.py" record --course "$ACADOS_COURSE" --out "$OUT" --label "$LABEL"
run "$STUDIES/acados_tracker.py" bench  --course "$ACADOS_COURSE" --out "$OUT" --label "$LABEL"

printf '{"start": %s, "end": %s}\n' "$LOAD_START" "$(load1)" > "$OUT/load.json"
echo "bench: $OUT"
