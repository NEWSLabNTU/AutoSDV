# Profiling

Where the CPU, the GPU and the power go while the stack is up. Ported from the
golf cart, which runs the same AGX Orin; see
`docs/roadmap/6-golfcart-backport.md`, phase 6.

| Script | What it does | Root? |
|--------|--------------|-------|
| `jetson_gpu_sampler.py` | samples GPU load, clock, rail power and temperature to CSV | no |
| `kernel_cpu_report.sh` | rates, not counters: CPU split, softirq, UDP, per-interface packets, DDS cost centres | no |
| `perf_kernel.sh` | system-wide `perf` profile resolved to symbols | yes (sudo) |

## Why a GPU sampler exists at all

`play_launch`'s resource monitor reads GPU state through NVML, and **NVML does
not exist on Jetson** — there is no `libnvidia-ml` on the board. Every `gpu_*`
column in every capture taken on an Orin is empty as a result. Everything needed
is in sysfs, readable without root:

```
/sys/devices/platform/gpu.0/load              GPU busy, per-mille
/sys/class/devfreq/17000000.gpu/cur_freq      GPU clock, Hz
/sys/class/hwmon/hwmon*/  (name == ina3221)   VDD_GPU_SOC, VDD_CPU_CV, VIN_SYS_5V0
/sys/class/thermal/thermal_zone*/             per-zone temperature
```

The timestamp column is ISO-8601 UTC with milliseconds, matching `play_launch`'s
`system_stats.csv`, so a sampler run beside a capture joins on time.

These paths are Orin-specific. On another Tegra the devfreq node and the ina3221
rail names differ.

`scripts/testing/localization/tegrastats_summary.py` answers the same question
from a `tegrastats` log instead, and is the right tool when a run already
recorded one.

## Reading kernel_cpu_report.sh

Two lines carry most of the signal:

- **NET_RX dominating the softirq rate** means a DDS datagram storm, not
  Autoware work.
- **`RcvbufErrors` above zero** means drops, and drops on a RELIABLE writer mean
  a retransmit spiral. The kernel socket buffer step in `./setup.sh` exists for
  this.

CycloneDDS's own threads are `tev`, `recv`, `recvMC`, `recvUC`, `gc` and `dq.*`,
seven per participant. Their total is DDS overhead. Each composable node running
as its own process is another participant, which is what the container-mode
choice trades against observability.

## perf on Jetson

`/usr/bin/perf` is Ubuntu's dispatch wrapper: it looks for a perf built for
`uname -r` — on the Orin `5.15.148-tegra` — finds only `linux-tools-5.15.0-177`,
and refuses. The ABI is identical across a point release, so `perf_kernel.sh`
calls the real binary under `/usr/lib/linux-tools/*/perf` directly.

Both `record` and `report` run under sudo, because kernel symbols come out as
raw addresses unless `perf report` can read `/proc/kallsyms`, which
`kptr_restrict=1` allows only for root. `report` also needs `--force`, since the
`perf.data` it wrote is no longer owned by the invoking user.

Output goes to `./tmp/autosdv-perf.data`; override the directory with
`AUTOSDV_PERF_DIR`.
