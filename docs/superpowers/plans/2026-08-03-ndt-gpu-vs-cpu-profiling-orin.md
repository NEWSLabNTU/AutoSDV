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

## Where this stands (2026-08-04) — **phase complete**

Both platforms are measured. The verdict is **keep the CUDA path on Orin**, and
the margin there is wider than on the desktop, not narrower.

Full Orin write-up: `docs/reports/ndt-gpu-vs-cpu-orin.md`.

| | desktop (RTX 5090) | **Orin, pinned** |
|---|---|---|
| GPU vs CPU, identical input, offline | 2.67x median, 5.09x mean (2.16 / 11.00 ms) | **3.9x median, 8.8x mean** (5.20 / 45.74 ms) |
| arms equivalent? | yes: NVTL 2.821 both, scores 0.02 % apart | yes: **NVTL 2.819 both**, scores 0.02 % apart |
| GPU vs Autoware, in-stack | — | **2.5x** (8.39 ms against 20.69) |
| NDT CPU, in-stack | — | **23.3 % against 119 %** |
| poses published | — | **100 % of alignments**; Autoware 23-100 % |
| power | — | **roughly neutral**: +208 mW GPU rail, −325 mW CPU rail |
| fixed on the way | GPU NVTL at the wrong rotation (`13a4e36`); pose vector meant two conventions (`324df7c`) | three harness faults + GPU line-search step bound (`f0df671`) |
| investigated, deliberately unchanged | the arms' differing convergence tests -- see below | same; **closing the GPU's earlier stop is not worth it on Orin** (see below) |

**The GPU advantage widens on Orin because its CPU is the weaker part.** Against
the desktop, the GPU arm is 2.4x slower (2.16 → 5.20 ms) but the CPU arm is 4.2x
slower (11.00 → 45.74 ms). Unified memory did not erode the win; the CPU deficit
enlarged it.

**Do not read the in-stack `cpu` row as an algorithm comparison.** In real time
the CPU arm blows the 100 ms budget on 52-93 % of scans and falls into the
self-reinforcing collapse below. The offline row is the fair one.

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

## How the answer was reached, in order

Kept because each step corrected the previous one, and the corrections are the
useful part.

**First matrix, in-stack, contended GPU.** `NDT_USE_GPU=0` published nothing:
1128 rejections out of ~1128 alignments, 13.03 iterations against the GPU's
2.40, NVTL 1.979 against 2.792. Read at the time as "not the same algorithm".

**Wrong.** Parity tests on identical input showed the arms agree exactly -- 6
iterations each, the same score, poses 1.3 mm apart. The old parity test could
not have seen otherwise: 100 Gaussian points in a single voxel, a scene that
constrains no pose, with a score-ratio tolerance of 0.5-2.0.

**Second reading: starvation.** The CPU arm is slower, blew the 100 ms budget
22.6 % of the time and dropped 29 % of scans, and a stale prior scores worse,
fails the gate and publishes nothing -- the same self-reinforcing loop as the
missing IMU transform. True, and still not the main cause.

**The actual cause**, found by the offline harness: the GPU scored NVTL at a
rotation mangled by a euler round trip and read ~1.45x high, so the 2.3 gate was
calibrated against an inflated number and the correctly-scoring CPU arm fell
under it. Fixed in `13a4e36`, with a second convention defect in the pose vector
itself fixed in `324df7c`.

### Consequence for the method

**A real-time replay cannot compare the two arms.** It starves the slower one
and measures a collapse rather than a speed ratio, and the collapse is
non-linear: an arm 2x too slow does not score 2x worse, it stops publishing.

That is why the offline comparison exists (it does now, see below): record the
(scan, initial guess) pairs from one good run, then feed the identical sequence
to each arm outside the ROS loop and time the alignments. Same inputs by
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
would have rejected every frame once the score was correct, so it went to 1.4 --
and then to **2.0** after the second defect below lifted honest scores from 2.00
to 2.80. 2.0 is the current value; 2.3, 1.6 and 1.4 all belong to intermediate
states and should not be quoted.

Be aware the gate does not separate a bad prior from hard geometry on this map.
The frozen-EKF failure of 2026-07-28 scored about 1.43-1.63 true, overlapping
healthy tracking. It rejects non-converged alignments, nothing more.

### Desktop baseline: what Orin gets compared against

Final figures, `cuda_ndt_matcher@324df7c`, idle card (0 % utilisation, 15 MiB),
400 frames of identical recorded input, repeats taken. Two independent runs
either side of the convergence experiment agreed to within 2 %
(2.164/11.003 ms and 2.197/11.009 ms), so these are stable, not a lucky sample.

**Configuration under test** — reproduce it before comparing anything:

| | |
|---|---|
| `ndt.resolution` | 2.0 |
| `converged_param_nearest_voxel_transformation_likelihood` | 2.0 |
| measurement-range crop | +/-40 m |
| points into NDT | 2000 (random downsample) |
| map | COSS, 4.9 M points, 9219 voxels |

**Alignment cost, identical (scan, initial guess) pairs:**

| arm | mean ms | p50 | p90 | p95 | p99 | max | iters mean/max |
|---|---|---|---|---|---|---|---|
| gpu | **2.16** | 2.35 | 2.48 | 2.93 | 3.05 | 3.72 | 1.62 / 4 |
| cpu | 11.00 | 6.26 | 28.13 | 31.18 | 94.87 | 162.88 | 2.44 / 19 |

**2.67x at the median, 5.09x at the mean.** Quote both. The mean gap is the
CPU's tail: six frames of 400 needed 9-19 iterations and 54-164 ms, while the
GPU never exceeded 3.72 ms or 4 iterations.

For real time the flatness matters more than the ratio. The CPU arm blows the
100 ms scan budget on about 1 % of frames; the GPU's p99 is 3.05 ms, a factor of
30 inside it. That tail is exactly what made the in-stack CPU run collapse
earlier in this phase, so treat p99 as the number that decides deployability and
the mean as the number that decides efficiency.

**Equivalence**, which makes the timings comparable at all:

| | |
|---|---|
| NVTL | 2.821 both arms, mean abs diff 0.0016 |
| score | 9778.5 gpu against 9780.5 cpu, 0.02 % |
| pose | median 6.0 mm apart, p95 35.8 mm, worst 66.8 mm |
| frames past the harness's 5 cm warning | 6 of 400 |

**In-stack, for the same build** (`just demo run-headless`, full ROS pipeline
rather than the offline harness): 1415 poses published, no score rejections,
worst publish gap 0.115 s, per-frame correction 0.025 m, 2.7 ms per scan,
heading within 0.31 deg of the direction of travel.

### In-stack three-way, clean conditions (the deployment comparison)

Same build, idle card, full ROS pipeline rather than the offline harness. This
is the comparison that decides what to run on the vehicle, because it includes
the cost of keeping up rather than the cost of one alignment.

| config | aligns | mean ms | p95 | max | >budget | cpu % | gpu % | rss MB | iters | NVTL | poses |
|---|---|---|---|---|---|---|---|---|---|---|---|
| gpu | 1415 | **2.71** | 4.28 | 8.0 | 0.0 % | **17.1** | 11.4 | 863 | 2.14 | 2.802 | **1415** |
| cpu | 1136 | 16.65 | 44.86 | 413.6 | 1.4 % | 55.9 | 4.4 | 747 | 3.17 | 2.802 | 1134 |
| autoware | 1390 | 4.14 | 6.90 | 25.8 | 0.0 % | 43.1 | n/a | **290** | 4.43 | 4.592 | 1369 |

**GPU against Autoware's OpenMP matcher: 1.53x on time, and 2.5x less CPU**
(17.1 % against 43.1 %). Both hold 10 Hz comfortably on this desktop; the CPU
arm does not, dropping to 1136 alignments and 1.4 % of frames over budget with
a 413 ms worst case.

The CPU-time difference is the one to carry to Orin. Freeing 26 points of a CPU
core matters more on a Jetson than 1.4 ms of latency, and it is the claim the
submodule's older figures made (57 % less NDT CPU) -- broadly reproduced here at
60 %, though those figures predate every fix in this phase and should be
re-taken rather than cited.

Two caveats on that table:

- **Autoware's NVTL of 4.592 is not comparable** to the other two. It runs at
  `resolution: 4.0` from its own config; NVTL scales with voxel size. Its
  iteration count is likewise its own tuning, not a like-for-like.
- **RSS: 863 MB for the GPU arm against 290 MB for Autoware.** Three times the
  memory, on a platform where the GPU and CPU share it. Worth watching on Orin
  alongside perception's own footprint.

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

## Orin results (2026-08-04, `cuda_ndt_matcher@cec3e68`)

Conditions: MAXN (`nvpmodel` mode 0), GPU pinned min = cur = max = 1300.5 MHz,
CPU pinned at 2 201 600 kHz with all 12 cores online, GPU load 0/1000 and no
other compute process at start. **Pinning is not optional** — the same
configuration measures 14.9 ms unpinned and 7.9 ms pinned.

`jetson_clocks` pins by raising the frequency floor to the ceiling and leaves
the governor named `nvhost_podgov`, so **`min_freq == max_freq` is the test for
"pinned"**, not the governor name. The harness checks it that way now.

### Offline, identical input (the fair GPU-vs-CPU number)

`just demo bench-offline <run_dir> 300`, 300 frames, 4.9 M map points:

| arm | mean ms | p50 | p95 | max | iters | score | NVTL |
|---|---|---|---|---|---|---|---|
| gpu | **5.20** | 5.25 | 6.23 | 11.3 | 1.72 | 9726.0 | 2.819 |
| cpu | 45.74 | 20.41 | 140.7 | 415.5 | 2.90 | 9727.8 | 2.819 |

**8.8x on the mean, 3.9x on the median.** The gap between those two is the CPU
arm's tail: p95 140 ms against the GPU's 6.2. Equivalence is structural here —
NVTL agrees to three decimals and the scores are 0.02 % apart.

One caveat the harness prints and this doc should not bury: worst pose
difference over 300 frames is **0.0913 m**, against the desktop's median 6 mm.
It is a small number of frames, but it is past the 5 cm the harness warns at,
and nobody has looked at which frames they are.

### In-stack, three-way (does it hold 10 Hz, and at what cost)

Three repeats each, `just demo bench "gpu cpu autoware" 3`,
`tmp/demo-runs/bench_20260804_053700`:

| config | mean ms | >budget | NDT CPU % | RSS MB | iters | NVTL | poses / aligns |
|---|---|---|---|---|---|---|---|
| gpu | **8.39** | 0.0 % | **23.3** | 626-740 | 2.2-3.0 | 2.80 | **100 %** |
| cpu | 142-761 | 52-93 % | 102.6 | 701-766 | 6.8-24.8 | 1.97-2.79 | 0.5-98 % |
| autoware | 19.81-43.53 | 0.0-0.2 % | 116-178 | 208-241 | 4.2-12.5 | 4.60 | 23-100 % |

Medians: GPU **2.5x** faster than Autoware on exe_time, and **5.1x** cheaper on
CPU. Neither the GPU nor Autoware misses the scan budget, so the speed is
margin, not recovered scans — **the CPU column is the result that matters on
this platform.**

Power, per-run `tegrastats` windows, Autoware's degraded run excluded:

| | GPU | Autoware |
|---|---|---|
| GPU busy | 5.9 % | 1.35 % |
| VDD_GPU_SOC | 5819 mW | 5611 mW |
| VDD_CPU_CV | **6107 mW** | 6432 mW |

**Roughly neutral**: +208 mW on the GPU rail, −325 mW on the CPU rail. The
submodule's "equal power" claim stands. An earlier draft of the Orin report
said it did not, having measured +506 mW before the euler fixes; that is
withdrawn.

### Answered: closing the GPU's earlier stop is not worth it here

The desktop work priced the GPU's earlier convergence at 0.02 % of score and a
median 6 mm of pose, and left the decision to this phase. **Leave it.** The GPU
arm already publishes 100 % of alignments at 23 % of a core, and 6 mm is far
inside the spread between repeats. Spending GPU time to close it buys nothing
measurable and costs the thing Orin is short of.

### What the CPU arm's in-stack row actually measures

Not the algorithm — the collapse. ~5x slower, blows the budget on 52-93 % of
scans, then: dropped scans → stale prior → worse score → gate rejection →
nothing published → the EKF dead-reckons into a worse prior still. The three
runs catch it at three depths, and the shallowest (416 alignments, 407 poses,
NVTL 2.787, 6.84 iterations) is the proof it converges fine when it keeps up.

It is also **single-threaded**: no `rayon` or `par_iter` in the CPU derivative
path and `num_threads` is not plumbed into the Rust code at all, so
`ndt.num_threads: 4` is inert. Measured `cpu %` confirms it — 102.6 %, one
saturated core, against Autoware's 116-178 %.

### Three harness faults, each of which produced an empty result that looked fine

Worth knowing before trusting any earlier Orin measurement:

- **Component name collision.** Localization's `voxel_grid_downsample_filter`
  shares a base name with perception's, and a ROS 2 container *silently* drops
  the loser. NDT received no points while the filters either side of it ran at
  10 Hz. The submodule's `CLAUDE.md` documented this fix as applied; it was not.
- **Stale-log readiness race.** The gate grepped `play_log/latest` before
  `play_launch` repointed it, matched the *previous* run's marker, passed in ~1 s,
  and the pose was seeded ~50 s before NDT existed. Every metric came back `n=0`
  on a run that reported success.
- **tegrastats outliving its run**, so power averaged across configurations.

And one platform trap: on Tegra `nvidia-smi --query-compute-apps` returns the
literal `[N/A], [N/A]`, which the contention check read as a process list.
play_launch's `gpu_utilization_percent` / `gpu_power_milliwatts` are `nan` there
too — NVML is not implemented on Jetson — hence
`scripts/testing/localization/tegrastats_summary.py`.

**After any submodule update, rebuild before benchmarking.** `--symlink-install`
does not symlink files that did not exist at build time, so a launch file added
by someone else fails every run with a `FileNotFoundError` that looks nothing
like a benchmark problem. This cost a full 9-run matrix.

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

## Method (as executed — kept for anyone repeating this on another platform)

1. Take the GPU-vs-CPU ratio from the offline harness (`just demo bench-offline`).
   The in-stack matrix answers "does it hold 10 Hz, at what cost", not "how much
   faster". Desktop is 2.67x median / 5.09x mean; Orin measured 3.9x / 8.8x.
2. Fix clocks and power mode; record them in the report.
3. `just demo bench "gpu cpu autoware" 3`. Confirm no other process is on the GPU
   first — `nvidia-smi --query-compute-apps=...`, or `tegrastats` on Orin.
4. Check the equivalence line before reading any timing. If it says the
   configurations did different work, the timings are not comparable; fix that
   first.
5. Compare against the desktop numbers above, and against the submodule's
   `docs/performance/autoware-comparison.md`.

## Acceptance — met

All six items are answered in `docs/reports/ndt-gpu-vs-cpu-orin.md`, and the
submodule's `docs/performance/autoware-comparison.md` carries a superseded
banner with the corrected figures. Restated here so the criteria stay legible:

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
- **The harness can pass on the previous run's log.** `play_log/latest` only
  names this run tens of seconds in. Anything read before then belongs to the
  run before, and a gate that accepts it fires instantly on a stack that does
  not exist yet.
- **A documented fix is not a fix.** The component name collision was written up
  in the submodule's CLAUDE.md, complete with the rename that resolved it, while
  neither copy of `util.launch.xml` had it. Check the code, not the note.
- **On Jetson, `nvidia-smi` answers questions it cannot answer**, returning
  `[N/A]` rather than failing. Treat any GPU figure from NVML on Tegra as absent,
  not as zero.

## Left open

1. ~~The 9 cm worst-case pose disagreement between the arms offline.~~
   **Answered.** It is the flat basin, not an error: median 6.2 mm, 10 of 300
   frames past 5 cm, and it tracks the iteration gap from the known
   convergence-test difference — 9 of those 10 have the CPU taking at least one
   extra iteration. **Max |ΔNVTL| over all 300 frames is 0.0148**, and on the
   worst frames the arms agree on NVTL to three decimals. Both land on an
   equally good score and stop at different points along an objective that is
   flat near its optimum. Pose difference is the wrong equivalence metric there;
   score is the right one. See the Orin report.
2. **One Autoware run in three degraded** to 43.5 ms, 12.5 iterations and 287
   poses, with NVTL unchanged at 4.599 — so not the gate. Unexplained, and the
   reason the comparison above uses medians.
3. **Inert parameters.** `ndt.num_threads` is ignored entirely; `ndt.step_size`
   was ignored on the GPU line-search path until `f0df671`. A parameter that
   silently does nothing is worse than one that is absent.
4. **The CPU arm's status.** It is a correct implementation that cannot hold
   real time on this platform. Either make it offline-only and say so, or give
   it the parallelism `num_threads` already promises.

## Out of scope

The vehicle-side items in `docs/reports/localization-open-questions.md` — wheel
speed scale, extrinsics, steering feedback — are unrelated to this phase and
should not be pulled in.
