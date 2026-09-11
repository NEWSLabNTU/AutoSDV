#!/usr/bin/env bash
# System-wide kernel+user CPU profile, resolved to symbols.
#
# Two traps this script exists to absorb:
#
#  1. /usr/bin/perf is Ubuntu's dispatch WRAPPER. It looks for a perf built for
#     `uname -r`, which on the Orin is 5.15.148-tegra, finds only
#     linux-tools-5.15.0-177, and refuses with "WARNING: perf not found for
#     kernel 5.15.148". The ABI is identical across a point release, so we call
#     the real binary under /usr/lib/linux-tools/*/perf directly.
#  2. Kernel symbols come out as raw addresses unless perf report can read
#     /proc/kallsyms, which kptr_restrict=1 allows only for root. So `record`
#     AND `report` both run under sudo, and report needs --force because the
#     perf.data it wrote is no longer owned by the invoking user.
#
# Usage: perf_kernel.sh [seconds]   (default 20)
set -euo pipefail
SECS="${1:-20}"
# Temp output goes under the repo's ./tmp, not the system /tmp -- see CLAUDE.md.
OUT_DIR="${AUTOSDV_PERF_DIR:-$(git rev-parse --show-toplevel 2>/dev/null || pwd)/tmp}"
mkdir -p "$OUT_DIR"
OUT="$OUT_DIR/autosdv-perf.data"

PERF=$(ls /usr/lib/linux-tools/*/perf 2>/dev/null | head -1)
if [ -z "$PERF" ]; then
  echo "no perf binary. install: sudo apt install linux-tools-generic" >&2
  exit 1
fi

# cpu-clock is a software event: works even where the Orin's PMU is not exposed.
echo "recording ${SECS}s system-wide -> $OUT"
sudo "$PERF" record -a -e cpu-clock -F 299 -g -o "$OUT" -- sleep "$SECS" 2>&1 | tail -2

echo
echo "══ CPU by process/thread name ══"
echo "   Cyclone's own threads are tev / recv / recvMC / recvUC / gc / dq.*."
echo "   Their total is DDS overhead, not Autoware work."
sudo "$PERF" report -i "$OUT" --force --no-children --sort=comm --stdio \
  --percent-limit 0.4 2>/dev/null | grep -E '^\s+[0-9]+\.[0-9]+%' | cut -c1-60

echo
echo "══ hottest symbols with call chains ══"
sudo "$PERF" report -i "$OUT" --force --no-children --sort=symbol --stdio \
  --percent-limit 0.5 2>/dev/null | grep -E '%|--' | head -60 | cut -c1-110

echo
echo "raw profile kept at $OUT"
echo "browse it:  sudo $PERF report -i $OUT --force"
