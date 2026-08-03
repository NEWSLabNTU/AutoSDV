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

## The offline harness exists, and it found the real defect

```bash
just demo bench-offline            # export frames from the latest run, then compare
just demo bench-nvtl-probe         # NVTL from each arm at identical poses
```

`scripts/testing/localization/export_ndt_frames.py` writes the (scan, initial
guess) pairs the stack actually used, plus the map, into a flat dump;
`ndt_cuda/examples/offline_bench.rs` replays that identical sequence through
each arm outside ROS. Warmup frames are discarded so lazy GPU setup does not
land on whichever arm runs first, and `--repeats` keeps the fastest run.

First result, 300 frames of the COSS bag, **desktop GPU still contended**:

| arm | mean ms | p50 | p95 | iters | score | NVTL |
|---|---|---|---|---|---|---|
| gpu | 15.530 | 17.463 | 20.385 | 1.59 | 9492.9 | 2.822 |
| cpu | 12.763 | 6.326 | 46.545 | 2.63 | 9495.5 | 1.921 |

**Alignment agrees**: mean score 9492.9 against 9495.5, worst pose difference
7.6 cm over 300 frames. **NVTL does not**, and `NVTL_PROBE` shows it at
identical poses, so it is the scoring rather than the alignment that preceded
it:

```
 frame   gpu nvtl   cpu nvtl   cpu/gpu
     0     2.7873     1.9199    0.6888
     3     2.8225     1.9360    0.6859
     5     2.8071     1.8977    0.6760
```

**That is what empties the stack in CPU mode.** 1.92 sits under the 2.3
convergence gate, so every frame is rejected and nothing is published -- which
the earlier in-stack matrix showed as 1128 rejections out of ~1128 alignments.
Scan drops make it worse but are not the cause.

### Resolved: the GPU scored at the wrong rotation (`cuda_ndt_matcher@13a4e36`)

`evaluate_nvtl_gpu` converted its `Isometry3` to a pose vector and back to a
matrix. That is not a round trip: the pose-vector helpers use nalgebra's euler
convention (R = Rz·Ry·Rx), `pose_to_transform_matrix` composes Autoware's
(R = Rx·Ry·Rz), and the two agree only when at most one angle is non-zero. The
COSS bag drives at yaw ~175 deg with a couple of degrees of roll and pitch, so
the GPU scored a different rotation than the CPU did for the same argument --
NVTL 2.79 against a true 1.92, about 1.45x.

Zeroing roll and pitch made the arms agree to four decimals, which is what
identified the conversion rather than the kernel. Fixed by converting straight
from the isometry; two tests pin it, one that the direct conversion is exact at
any orientation and one that the detour is *not*.

**The gate moved with it.** 2.3 was calibrated against the inflated value and
would now reject every frame. It is 1.4, chosen from a healthy run's
distribution (tracking p50 1.85, p5 1.48, p1 1.34), rejecting 0.7% of frames
against the 0.4% the old pair rejected. Verified in the stack: 1423 poses,
no rejections, worst gap 0.200 s.

Be aware the gate does not separate a bad prior from hard geometry on this map.
The frozen-EKF failure of 2026-07-28 scored about 1.43-1.63 true, overlapping
healthy tracking. It rejects non-converged alignments, nothing more.

### Desktop answer, both defects fixed, idle GPU

400 frames of identical input, three repeats, `cuda_ndt_matcher@324df7c`. The
card was genuinely idle this time (0 % utilisation, 15 MiB) and the CPU repeats
came out 11.13 / 11.01 / 11.23 ms, so there is no thermal drift in these.

| arm | mean ms | p50 | p90 | p99 | max | iters mean/max | score | NVTL |
|---|---|---|---|---|---|---|---|---|
| gpu | **2.197** | 2.37 | 2.48 | 3.05 | 3.65 | 1.62 / 4 | 9778.5 | 2.821 |
| cpu | 11.009 | 6.33 | 28.13 | 94.87 | 163.91 | 2.44 / 19 | 9780.5 | 2.821 |

**2.67x at the median, 5.01x at the mean.** Quote both: the mean gap is the
CPU's tail, not its typical case. Six frames of 400 needed 9-19 CPU iterations
and took 54-164 ms, while the GPU never exceeded 3.65 ms or 4 iterations.

For real time the flatness matters more than the ratio. The CPU arm blows the
100 ms scan budget on about 1 % of frames; the GPU's p99 is 3.05 ms, a factor of
30 inside it.

Equivalence is now what it should be: NVTL identical at 2.821, scores 0.02 %
apart, poses a median 6.0 mm apart. The harness still warns, because 6 frames
exceed its 5 cm threshold (worst 6.7 cm) -- and those are the same hard frames
where the CPU spends 19 iterations and the GPU stops at 4. **Worth a look on
Orin**: it suggests the GPU's convergence test gives up earlier than the CPU's,
which would mean part of the speed is bought with slightly less converged poses.
The scores say the cost is small (the CPU's is marginally better) but it is not
nothing.

### Why the GPU stops earlier: a different convergence test (investigated, left alone)

The two arms compare different quantities against `trans_epsilon`:

| arm | tests | where |
|---|---|---|
| gpu | the **applied** step, after line search | `ndt_graph_kernels.cu:765` |
| cpu | the **raw Newton step**, before line search | `solver.rs`, before `step_dir` |

Autoware compares the applied step: `multigrid_ndt_omp_impl.hpp` reassigns
`delta_p_norm` to the `computeStepLengthMT` result at line 346 and tests that at
line 381. **So the GPU matches the reference and the CPU is the deviation** --
which is the answer to why the GPU gives up in <=4 iterations where the CPU
takes up to 19 on the same frame.

Aligning the CPU to the letter of it was tried and measured strictly worse:

| variant | mean ms | iters | score | worst pose diff |
|---|---|---|---|---|
| as shipped | **11.00** | 2.44 | 9780.5 | 0.067 m |
| converge on applied step | 19.23 | 2.65 | 9779.5 | 0.083 m |
| + line search seeded with the Newton norm (Autoware's `step_init`) | 39.60 | 5.78 | 9779.4 | 0.083 m |

Two to four times slower, no better score, no better agreement with the GPU.
Both changes were reverted.

The mechanism is worth knowing before anyone tries again: the CPU's check sits
*before* the line search, so a frame whose Newton step is already tiny returns
without paying for a More-Thuente search -- several full derivative evaluations
over 2000 points. Autoware's ordering applies the step first and cannot skip
that. At ~2.4 iterations per frame, one skipped line search is most of the
runtime. The CPU arm reaches the same answer more cheaply by a different route.

What it costs: the GPU's earlier stop leaves its score 0.02 % below the CPU's
(9778.5 against 9780.5) and its pose a median 6 mm away, with 6 frames of 400
past 5 cm. Small, real, and not obviously worth GPU time to close -- decide that
on Orin with power and latency in hand, since it is the production path there.

### Also fixed: the pose vector meant two different rotations (`324df7c`)

The same convention split ran deeper than the scoring path. The boundary
converters used nalgebra's euler order while every consumer of a pose vector --
`pose_to_transform_matrix`, the GPU angular derivatives, and
`derivatives/cpu.rs` -- assumes Autoware's XYZ. NDT therefore misread its own
initial guess, optimised in one parameterisation, and had the answer converted
back by the same wrong inverse. The error was common to both arms, so they
agreed with each other while both drifted from the caller's isometry, and the
offline harness could not see it.

Converters moved to XYZ, being the minority. Over 1422 frames of the COSS bag:

| | before | after |
|---|---|---|
| initial-to-result | 0.070 m | **0.025 m** |
| iterations | 3.25 | **2.14** |
| exe_time | 3.19 ms | **2.72 ms** |
| NVTL | 2.00 | **2.80** |

Heading against course over ground is unchanged at +0.31 deg, so the `vlp32c`
mounting yaw calibrated on 2026-08-02 was not absorbing this and stands.

**The gate is now 2.0**, from a healthy run's distribution (mean 2.80, min 2.29).
It passed through 1.6 and 1.4 while these two defects were in flight; anything
quoting those is stale. Verified: 1415 poses, no rejections, worst gap 0.115 s.

### What this means for the phase

The two defects are the reason the desktop numbers moved so much, and both were
found by comparing arms rather than by reading code. Take the Orin measurement
on `cuda_ndt_matcher` at `324df7c` or later; anything earlier is measuring a
matcher that mis-scores and mis-orients.

Nothing in that area is known-open now. If a future comparison shows the arms
disagreeing, the tests in `optimization/types.rs` and `derivatives/gpu.rs` are
the first things to run: they pin the conventions in both directions.

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

1. Take the GPU-vs-CPU ratio from the offline harness (`just demo bench-offline`).
   The in-stack matrix answers "does it hold 10 Hz, at what cost", not "how much
   faster". The desktop ratio is 5.78x; Orin is the question.
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
