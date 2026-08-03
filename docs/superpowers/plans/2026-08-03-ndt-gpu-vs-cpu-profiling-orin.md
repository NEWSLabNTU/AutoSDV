# NDT Profiling Phase — Does the GPU Earn Its Place, on AGX Orin

> **Handover.** Written on the desktop (RTX 5090) after building the benchmark
> harness and running it once. The measurement itself moves to AGX Orin, which is
> the deployment target and where the answer actually matters. Everything below
> is either already committed or a known blocker; nothing here is speculative
> unless it says so.

**Goal:** answer whether `cuda_ndt_matcher`'s GPU pipeline is worth its
complexity on Orin, against two baselines — the same Rust algorithm on CPU, and
Autoware's OpenMP `autoware_ndt_scan_matcher` — with numbers that survive
scrutiny.

**Owner repos, all pushable, ff-merge to main:**
`NEWSLabNTU/AutoSDV` (develop), `NEWSLabNTU/cuda_ndt_matcher` (main),
`NEWSLabNTU/autoware_core` on branch `cuda_ndt` (the patched Autoware NDT, a
submodule at `src/localization/cuda_ndt_matcher/tests/comparison/autoware_core`).

## Read first

- `docs/guides/ndt-tuning.md` — the pitfalls. Non-optional: three of them were
  discovered the expensive way and two of them invert the answer if ignored.
- `docs/reports/cuda-ndt-coss-replay.md` — why the stack is configured as it is,
  and the contended-GPU episode that this phase must not repeat.
- `demo/README.md` §Benchmarking — how to drive the harness.

## What already exists

```bash
just demo bench                       # gpu, cpu, autoware; ~15 min
just demo bench "gpu cpu" 3           # subset, three repeats
just demo bench-report <bench_dir>/runs.tsv
```

| piece | where |
|---|---|
| matrix driver | `demo/scripts/run-ndt-benchmark.sh` |
| single run, parameterised | `demo/scripts/run-coss-ndt.sh` (`POSE_SOURCE`, `NDT_USE_GPU`, `NDT_DEBUG_FILE`) |
| report | `scripts/testing/localization/ndt_benchmark_report.py` |

Configurations: `gpu` = as shipped; `cpu` = `NDT_USE_GPU=0`, which is meant to be
the same Rust algorithm without the GPU; `autoware` = `pose_source:=ndt`.

The report deliberately covers three axes, because speed alone misleads:
**speed** (exe_time, and what fraction blew the 100 ms scan budget), **cost**
(CPU/GPU/RSS from the samples `play_launch` already takes), and **equivalence**
(iterations and NVTL — if the configurations did not converge alike, the timing
comparison is void and the report says so).

Logging is **off by default**: the always-published diagnostics carry exe_time,
iterations and scores per scan, so a benchmark costs nothing extra. `PROFILE=1`
additionally requests per-iteration JSONL, which needs a matcher built with
`just build-cuda-debug-iterations` inside the cuda_ndt_matcher submodule.

## Corrected: the CPU arm is not a different algorithm, it is starved

The first draft of this doc called `NDT_USE_GPU=0` "not the same algorithm",
because the first full matrix looked like this (desktop, **GPU contended by an
unrelated process at 85 %**):

| config | aligns | mean ms | p95 | >budget | cpu % | iters | NVTL | poses |
|---|---|---|---|---|---|---|---|---|
| gpu | 1252 | 13.07 | 23.23 | 0.0 % | 49.8 | 2.40 | 2.792 | 1246 |
| cpu | 1128 | 69.78 | 173.88 | 22.6 % | 93.1 | 13.03 | 1.979 | **0** |
| autoware | 1381 | 4.62 | 7.89 | 0.0 % | 48.7 | 4.31 | 4.594 | 1376 |

That reading was wrong. Given the same input the arms agree exactly
(`cuda_ndt_matcher@717e9e1`, three new tests in `ndt_cuda`):

- **alignment**: same scene, known transform, from identity -- 6 iterations
  each, score 5582.38 each, final poses 1.3 mm apart;
- **NVTL**: same grid, same points, same pose -- identical to four decimals.

The older parity test could not have caught a difference: it aligned 100
Gaussian points that all fall in a single voxel, a scene that constrains no
pose, with a score-ratio tolerance of 0.5-2.0 and half a metre of position
slack.

**What actually happens in the replay** is the collapse this project has now
seen twice. The CPU arm is roughly 5x slower, so it blows the 100 ms scan budget
22.6 % of the time and drops 29 % of scans (1542 in, 1100 aligned, against the
GPU's 1567 in, 1250 aligned). A dropped scan means a staler prior; a stale prior
scores worse; a worse score fails the 2.3 gate -- 1128 rejections out of ~1128
alignments -- so nothing is published; the EKF then dead-reckons into a worse
prior still. Same self-reinforcing loop as the missing IMU transform, triggered
by throughput instead of by a missing input.

### Consequence for the method

**A real-time replay cannot compare the two arms.** It starves the slower one
and measures a collapse rather than a speed ratio, and the collapse is
non-linear: an arm 2x too slow does not score 2x worse, it stops publishing.

Task 1 is therefore not "fix the CPU path" but **build an offline comparison**:
record the (scan, initial guess) pairs from one good run, then feed the identical
sequence to each arm outside the ROS loop and time the alignments. Same inputs by
construction, no starvation, no feedback. `ndt_cuda` is a library, so this can be
a Rust bench or a small binary in the submodule rather than anything ROS-shaped.

The in-stack matrix stays useful for a different question -- can this arm hold
10 Hz on this hardware, and what does it cost in CPU and power -- which on Orin
matters as much as the per-alignment time. Keep both, and do not quote the
in-stack numbers as a speed ratio.

## Orin specifics

- **Thermals and clocks decide the answer.** Fix them before measuring and record
  what was set: `sudo nvpmodel -q`, `sudo jetson_clocks --show`. A benchmark at
  an unpinned clock measures the governor. Run the matrix at least twice and
  check the second is not systematically slower (thermal drift).
- **Unified memory.** Orin's GPU and CPU share physical memory, so the
  host/device copies that dominate discrete-GPU overhead behave differently. A
  GPU win on the 5090 does not transfer, in either direction.
- **`cudarc` must stay on the `cuda-12050` feature** even though JetPack ships
  CUDA 12.6: Tegra's driver lacks `cuEventElapsedTime_v2` and `cusolverDnXgeev`,
  and a higher feature panics on the missing symbol. See the submodule's
  CLAUDE.md.
- **`CUDA_ARCH=87`** for Orin.
- **Sampling.** `tegrastats` gives GPU, CPU-per-core and power;
  `scripts/analyze_tegrastats_cpu.py` and `analyze_resource_usage.py` in the
  submodule already parse it. `play_launch`'s per-node `metrics.csv` also carries
  `gpu_utilization_percent` and `gpu_power_milliwatts`, and the harness copies
  the matcher's into each run directory.
- **Power is a first-class metric here**, unlike on the desktop. The submodule's
  earlier figures claim 34.8 Hz vs Autoware's 26.5 Hz and 57 % less NDT CPU at
  equal power on Orin — worth confirming or retiring, since they predate every
  fix from the July investigation.

## Data and build on the Orin

```bash
just demo check                 # prerequisites
just demo prepare               # fetches the 2.8 GB bag, builds
```

`just build` is required, not plain colcon: it passes `--cargo-args --release`,
without which the matcher is a debug binary about 8x slower — the single easiest
way to produce a meaningless benchmark. `just demo check` warns if the installed
binary looks like a debug build.

## Method

1. Build the offline comparison described above and take the GPU-vs-CPU ratio
   from it. The in-stack matrix answers "does it hold 10 Hz, at what cost",
   not "how much faster".
2. Fix clocks and power mode; record them in the report.
3. `just demo bench "gpu cpu autoware" 3`. Confirm no other process is on the GPU
   first — `nvidia-smi --query-compute-apps=...`, or `tegrastats` on Orin.
4. Check the equivalence line before reading any timing. If it says the
   configurations did different work, the timings are not comparable; fix that
   first.
5. Compare against the desktop numbers above, and against the submodule's
   `docs/performance/autoware-comparison.md`.

## Acceptance

A result is finished when it states, with numbers:

- GPU against CPU from the **offline** harness, where identical inputs make
  matched convergence structural rather than something to check afterwards;
- from the in-stack matrix, separately: whether each arm holds 10 Hz, the
  fraction of scans dropped, and what happens to published poses when it cannot;
- GPU against Autoware OpenMP, on exe_time, on the fraction of scans that blew
  the budget, and on published poses;
- CPU cost and power for each, since on Orin the point of offloading is to free
  CPU for perception rather than to win milliseconds;
- the clock and power mode the numbers were taken at, and evidence the GPU was
  otherwise idle;
- a verdict on whether the CUDA path is worth its maintenance on this platform.

Write it to `docs/reports/ndt-gpu-vs-cpu-orin.md`, and update the submodule's
`docs/performance/autoware-comparison.md` if the old figures no longer hold.

## Traps that already cost time here

- **NVTL is a convergence gate, not a quality score.** It scales with
  `ndt.resolution` and rises when far returns are excluded. Tuning to maximise it
  selects coarse voxels and narrow crops regardless of accuracy — it inverted the
  conclusion twice in July.
- **A shared GPU makes every timing meaningless**, and the tell is that only wall
  clock moves while iterations and NVTL stay put.
- **`ros2 node list` is not a readiness signal** at this stack size; it reported
  24 of 127 nodes on a healthy system. The harness waits on the matcher's own log
  line instead.
- **A matcher that misses the scan budget looks fast**: it drops scans. Always
  read the alignment and pose counts next to the mean.

## Out of scope

The vehicle-side items in `docs/reports/localization-open-questions.md` — wheel
speed scale, extrinsics, steering feedback — are unrelated to this phase and
should not be pulled in.
