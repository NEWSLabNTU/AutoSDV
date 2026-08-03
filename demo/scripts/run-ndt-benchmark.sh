#!/usr/bin/env bash
# Does the GPU actually earn its place in cuda_ndt_matcher?
#
# Replays the same bag through three matchers and compares them:
#
#   gpu       cuda_ndt_matcher as shipped, CubeCL/CUDA pipeline
#   cpu       cuda_ndt_matcher with NDT_USE_GPU=0 -- the same Rust algorithm
#             and the same parameters, no GPU
#   autoware  Autoware's autoware_ndt_scan_matcher, OpenMP over CPU threads
#
# gpu-vs-cpu isolates what the GPU contributes. cpu-vs-autoware puts that in
# context against the reference implementation.
#
# The two arms do NOT take identical steps, so read the equivalence line in the
# report before reading any timing. The CPU arm follows the reference: unit step
# direction, More-Thuente bounded by ndt.step_size, convergence on the step
# actually applied. The GPU arm applies its line-search alpha to an
# unnormalised Newton step (ndt_graph_kernels.cu:699), so it moves further per
# iteration and ndt.step_size is inert whenever use_line_search is true. Expect
# the GPU to converge in fewer iterations for that reason alone -- it is a real
# algorithmic difference, not only a hardware one.
#
# Logging is off by default: the always-published diagnostics already carry
# exe_time, iteration count and scores per scan, and play_launch samples per-node
# CPU, memory and GPU utilisation into the run directory. Set PROFILE=1 to also
# request the per-iteration JSONL, which only produces anything if the matcher
# was built with --features debug-output (`just build-cuda-debug-iterations`).
#
# Environment:
#   CONFIGS   space-separated subset of "gpu cpu autoware" (default: all three)
#   REPEATS   runs per configuration (default 1)
#   PROFILE   1 to request per-iteration JSONL as well (default off)
#   RVIZ      passed through; default false, since nobody watches a benchmark
set -uo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO"

CONFIGS="${CONFIGS:-gpu cpu autoware}"
REPEATS="${REPEATS:-1}"
PROFILE="${PROFILE:-0}"
export RVIZ="${RVIZ:-false}"

STAMP="$(date +%Y%m%d_%H%M%S)"
BENCH="$REPO/tmp/demo-runs/bench_$STAMP"
mkdir -p "$BENCH"

say() { printf '\033[1;33m[bench]\033[0m %s\n' "$*"; }

on_interrupt() {
    trap '' INT TERM
    echo
    say "interrupted; the run in flight tears itself down"
    exit 130
}
trap on_interrupt INT TERM

say "configurations: $CONFIGS   repeats: $REPEATS   profile: $PROFILE"
say "collecting into $BENCH"
# Is the GPU ours alone? A shared GPU makes every timing here meaningless, and
# the tell is that only wall clock moves while iterations and NVTL stay put.
#
# Tegra needs its own answer: nvidia-smi exists on Jetson but its query fields
# are unimplemented, so --query-compute-apps returns the literal "[N/A], [N/A]"
# rather than nothing. Reading that as a process list made every Jetson run
# print a contention warning it had not earned. Ask devfreq instead.
if [[ -e /etc/nv_tegra_release ]]; then
    gpu_dev=/sys/class/devfreq/17000000.gpu
    load=$(cat "$gpu_dev/device/load" 2>/dev/null || echo unknown)   # per mille
    freq=$(cat "$gpu_dev/cur_freq" 2>/dev/null || echo unknown)
    fmin=$(cat "$gpu_dev/min_freq" 2>/dev/null || echo 0)
    fmax=$(cat "$gpu_dev/max_freq" 2>/dev/null || echo 1)
    say "Tegra GPU before: load=${load}/1000 freq=$((freq / 1000000))MHz range=$((fmin / 1000000))-$((fmax / 1000000))MHz"
    [[ "$load" =~ ^[0-9]+$ && "$load" -gt 50 ]] &&
        say "WARNING GPU is already ${load}/1000 busy, timings will be contended"
    # jetson_clocks pins by raising the frequency floor to the ceiling, and
    # leaves the governor name alone -- so min == max is the test for "pinned",
    # not governor == userspace.
    [[ "$fmin" == "$fmax" ]] &&
        say "clocks pinned at $((fmax / 1000000))MHz" ||
        say "NOTE GPU clock floats $((fmin / 1000000))-$((fmax / 1000000))MHz; run 'sudo jetson_clocks' or the numbers measure the governor"
elif command -v nvidia-smi >/dev/null; then
    say "GPU before: $(nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader)"
    others=$(nvidia-smi --query-compute-apps=pid,used_memory --format=csv,noheader)
    [[ -n "$others" ]] && say "WARNING other GPU processes are running, timings will be contended:" && echo "$others"
fi

for rep in $(seq 1 "$REPEATS"); do
    for cfg in $CONFIGS; do
        label="bench-${cfg}-${rep}"
        case "$cfg" in
            gpu)      export POSE_SOURCE=cuda_ndt NDT_USE_GPU=1 ;;
            cpu)      export POSE_SOURCE=cuda_ndt NDT_USE_GPU=0 ;;
            autoware) export POSE_SOURCE=ndt      NDT_USE_GPU=1 ;;
            *) say "unknown configuration '$cfg'"; exit 1 ;;
        esac
        if [[ "$PROFILE" == "1" ]]; then
            export NDT_DEBUG_FILE="$REPO/tmp/demo-runs/${label}_profile.jsonl"
        else
            unset NDT_DEBUG_FILE NDT_DEBUG
        fi

        say "=== $cfg, run $rep of $REPEATS"
        LABEL="$label" KEEP_UP=false "$REPO/demo/scripts/run-coss-ndt.sh" \
            > "$BENCH/${label}.log" 2>&1
        rc=$?
        run_dir=$(ls -dt "$REPO"/tmp/demo-runs/${label}_*/ 2>/dev/null | head -1)
        if [[ $rc -ne 0 || -z "$run_dir" || ! -f "${run_dir}bag/metadata.yaml" ]]; then
            say "$cfg run $rep FAILED (rc=$rc); see $BENCH/${label}.log"
            continue
        fi
        echo "${cfg}	${run_dir%/}" >> "$BENCH/runs.tsv"
        say "$cfg run $rep -> ${run_dir%/}"
        sleep 5   # let the machine settle between runs
    done
done

if [[ ! -f "$BENCH/runs.tsv" ]]; then
    say "no run completed; nothing to report"
    exit 1
fi

say "building the report"
set +u
source /opt/ros/humble/setup.bash
source "$REPO/install/setup.bash"
set -u
python3 "$REPO/scripts/testing/localization/ndt_benchmark_report.py" "$BENCH/runs.tsv" \
    2>/dev/null | tee "$BENCH/report.txt"
say "report: $BENCH/report.txt"
