# Does the GPU earn its place in cuda_ndt_matcher, on AGX Orin

**Verdict: yes, and mainly for the CPU it frees.** On the deployment target the
CUDA path runs one alignment in ~8.4 ms against Autoware OpenMP's ~20.7 ms, and
publishes a pose for **every** alignment. The number that matters most is
**23 % of a core against 119 %**: NDT stops being the largest single CPU
consumer in the stack, at roughly neutral power.

Measured 2026-08-04 on the AGX Orin Developer Kit (64 GB) at pinned clocks,
replaying `outdoor_20251226_153115` against `data/COSS-map-planning`, via
`just demo bench "gpu cpu autoware" 3`.

---

## What had to be fixed before any of this could be measured

Five faults, each of which independently produced a plausible-looking but
worthless result. Three were in the harness, two in the matcher.

| fault | symptom | why it was invisible |
|---|---|---|
| Component name collision | NDT received **no points at all** | localization's `voxel_grid_downsample_filter` shares a base name with perception's, and a ROS 2 container *silently* drops the loser. The crop box either side of it ran at 10 Hz, so the pipeline looked healthy. |
| Stale-log readiness race | pose seeded ~50 s **before NDT existed** | the gate grepped `play_log/latest/.../err` before `play_launch` repointed the symlink, matched the *previous* run's marker, and passed in ~1 s. Every metric came back `n=0` on a run reporting success. |
| tegrastats outliving its run | power averaged across configurations | started under `setsid`, whose parent exits at once, so `pgid_of $!` raced and the sampler was never registered for teardown. |
| **Euler convention in `evaluate_nvtl_gpu`** | **GPU NVTL inflated ~1.45x** | it round-tripped through a pose vector (`Rz·Ry·Rx`) while `pose_to_transform_matrix` composes Autoware's (`Rx·Ry·Rz`). 2.79 reported against a true 1.92. |
| **Euler convention in the pose-vector converters** | NDT misread every initial guess | the same mismatch at the boundary, affecting **both** arms — which is why they agreed with each other while both drifted from the caller's isometry. |

The last two were found by a parallel investigation (`717e9e1`..`324df7c`) and
are the reason the numbers below differ from every earlier measurement on this
platform. Fixing the converters alone moved initial-to-result 0.070 → 0.025 m
and iterations 3.25 → 2.14.

The first is still the one to remember: the submodule's `CLAUDE.md` documented
that collision *and its fix* as already applied. It was not applied, in either
copy of `util.launch.xml`. A documented fix is not a fix.

## Conditions

Clocks pinned before measuring, via `sudo jetson_clocks`:

| | setting |
|---|---|
| power mode | MAXN (`nvpmodel` mode 0) |
| GPU | `17000000.gpu` min = cur = max = **1300.5 MHz** |
| CPU | min = cur = max = **2 201 600 kHz**, cores 0-11 online |
| GPU at start | load **0/1000**, no other compute processes |

Pinning is not optional: the same configuration measured **14.9 ms** unpinned
and **7.9 ms** pinned. `jetson_clocks` pins by raising the frequency floor to
the ceiling and leaves the governor named `nvhost_podgov`/`schedutil`, so
**`min_freq == max_freq` is the test for "pinned"**, not the governor name —
the harness checks it that way now.

## Results

Three repeats per configuration, `tmp/demo-runs/bench_20260804_053700`.
Scan budget 100 ms at 10 Hz.

| config | aligns | mean ms | p50 | p95 | >budget | NDT CPU % | RSS MB | iters | NVTL | poses |
|---|---|---|---|---|---|---|---|---|---|---|
| gpu | 1351 | 9.10 | 8.37 | 14.81 | 0.0 % | 23.1 | 740 | 2.21 | 2.804 | **1351** |
| gpu | 1347 | 8.39 | 7.98 | 12.15 | 0.0 % | 23.3 | 675 | 2.97 | 2.797 | **1347** |
| gpu | 1355 | 7.84 | 7.53 | 11.12 | 0.0 % | 23.4 | 626 | 2.25 | 2.804 | **1355** |
| cpu | 40 | 760.85 | 909.26 | 1117.78 | 92.5 % | 103.0 | 766 | 24.77 | 2.004 | 2 |
| cpu | 54 | 414.91 | 342.96 | 900.37 | 90.7 % | 102.5 | 701 | 19.02 | 1.966 | 9 |
| cpu | 416 | 142.23 | 105.16 | 415.28 | 52.4 % | 102.3 | 741 | 6.84 | 2.787 | 407 |
| autoware | 1244 | 19.81 | 19.68 | 30.74 | 0.1 % | 116.3 | 213 | 4.23 | 4.596 | 1244 |
| autoware | 1231 | 20.69 | 20.13 | 36.08 | 0.0 % | 119.0 | 208 | 4.59 | 4.596 | 1189 |
| autoware | 1231 | 43.53 | 47.94 | 58.76 | 0.2 % | 177.6 | 241 | 12.54 | 4.599 | 287 |

**GPU against Autoware OpenMP**, on medians of three:

| | GPU | Autoware | ratio |
|---|---|---|---|
| exe_time, median | **8.39 ms** | 20.69 ms | **2.5x** |
| p95 | 11.1-14.8 ms | 30.7-58.8 ms | ~2.6x |
| scans over the 100 ms budget | 0.0 % | 0.0-0.2 % | — |
| NDT CPU | **23.3 %** | 119 % | **5.1x less** |
| RSS | 626-740 MB | 208-241 MB | **3.1x more** |
| poses published | **100 % of alignments** | 23-100 % | — |

Do not quote the harness's own "5.55x" headline: it compares the worst Autoware
run against the best GPU run. On medians it is **2.5x**.

The GPU arm now publishes a pose for **every single alignment** in all three
runs — 1351/1351, 1347/1347, 1355/1355. Before the euler fixes it dropped
roughly 30 %. Its NVTL is also stable at 2.797-2.804, where it previously
alternated between two clusters; that bimodality was the euler defect, not a
mystery of the pose prior, and it is gone.

### Power and GPU occupancy

Per-run `tegrastats` windows at 1 s, first 30 samples dropped so map load and
warm-up stay out of the mean. Autoware run 3 is excluded as degraded.

| | GPU | Autoware | delta |
|---|---|---|---|
| GPU busy | **5.9 %** | 1.35 % | +4.5 pt |
| VDD_GPU_SOC | 5819 mW | 5611 mW | **+208 mW** |
| VDD_CPU_CV | **6107 mW** | 6432 mW | **−325 mW** |
| VIN_SYS_5V0 | 5681 mW | 5658 mW | +23 mW |

**Power is roughly neutral, and if anything slightly favours the GPU**: it pays
~208 mW on the GPU/SOC rail and recovers ~325 mW on the CPU rail. An earlier
draft of this report, measured before the euler fixes, found +506 mW on the GPU
rail and concluded the submodule's "equal power" claim was wrong. On the
corrected code that conclusion does not hold and has been withdrawn — "equal
power" is a fair summary.

GPU occupancy of **5.9 %** means the offload leaves essentially the whole GPU
free for perception, which was the point.

## The CPU arm is the same algorithm, starved of time

`NDT_USE_GPU=0` is **not** a broken or reduced implementation. Given identical
input the two arms agree: the offline harness (`ndt_cuda/examples/offline_bench.rs`)
feeds both the same (scan, initial guess) pairs and gets 6 iterations each,
score 5582.38 each, final poses 1.3 mm apart; NVTL identical to four decimals.
This run's equivalence check agrees — **NVTL differs by 0.017, "same result"**.

What kills it in a real-time replay is throughput, and the failure is
self-reinforcing:

> ~5x slower → blows the 100 ms budget on 52-93 % of scans → drops scans →
> the prior goes stale → a stale prior scores worse → a worse score fails the
> convergence gate → nothing is published → the EKF dead-reckons into a worse
> prior still.

The three runs above catch it at different depths of that spiral: two collapse
almost completely (40 and 54 alignments, 2 and 9 poses, NVTL ~1.98 below the
2.0 gate), while the third partially escapes — **416 alignments, 407 poses,
NVTL 2.787, 6.84 iterations**. That third run is the proof: when the CPU arm
keeps up well enough, it converges and publishes like the GPU does.

It remains unusable for production on this platform, and it remains
**single-threaded** — there is no `rayon` or `par_iter` in the CPU derivative
path and `num_threads` is not plumbed into the Rust code at all, so
`ndt.num_threads: 4` in `cuda_scan_matcher.param.yaml` is inert. The measured
`cpu %` confirms it: 102-103 %, one saturated core, against Autoware's 116-178 %.

**A real-time replay cannot be used to compare the two arms.** Use the offline
harness. The `cpu` row above measures the collapse, not the algorithm.

Its convergence test **differs from the reference on purpose**. The CPU arm
compares the raw Newton step against `trans_epsilon` before clamping, where
Autoware compares the step actually applied, after scaling
(`multigrid_ndt_omp_impl.hpp:346,381`) — so on this point the GPU matches the
reference and the CPU is the deviation.

This branch initially "fixed" that, on the strength of reading the reference.
It was wrong to. The desktop investigation had already made exactly that change
and measured it **strictly worse** — converging on the applied step took the
CPU arm from 11.0 ms to 19.2, and additionally seeding More-Thuente with the
Newton magnitude took it to 39.6 ms and 5.78 iterations, for a score that moved
0.01 % and agreement with the GPU that got slightly *worse*. The reason is
ordering: the CPU's check sits before the line search, so a frame whose Newton
step is already tiny returns without paying for a More-Thuente search over
~2000 points, and at ~2.4 iterations per frame that saving is most of the
runtime. The arm reaches the same answer more cheaply by a different route.

The change has been withdrawn and the arm left as it was. What the GPU's
earlier stop costs is recorded on the desktop side: 0.02 % of score and a
median 6 mm of pose, 6 frames in 400 past 5 cm.

## The GPU's step deviation is real, and inert

The GPU arm deviated from the reference in the opposite direction: it applied
its line-search α to an **unnormalised** Newton step with candidates decaying
from 1.0. That makes α dimensionless and means **`ndt.step_size` had no effect
at all** whenever `use_line_search: true` — which it is.

Corrected, and then measured. **It changes nothing on this workload:**

| build | init iters | track iters |
|---|---|---|
| as-shipped | 1.738 / 1.804 / 1.725 | 3.88 / 8.58 / 8.62 |
| aligned | 1.735 / 1.746 / 1.732 | 8.38 / 9.35 / 4.13 |

Both produce the same clusters; neither separates. The arithmetic explains it:
the old ladder is `0.618^k · |δ|`, the new one `0.618^k · min(|δ|, step_size)`,
and these are **identical whenever |δ| ≤ step_size**. With a seeded pose and an
EKF prior the Newton step stays well inside 0.1 m, so the bound never binds.

The correction is kept because the code was wrong, not because it was costing
anything here. It would bite on a bad prior, a recovery, or Monte Carlo
initialisation from far away, where the unbounded form can overshoot in one
step. An earlier draft claimed the GPU's low iteration count was an artefact of
this deviation; **that is refuted by the table above** and has been withdrawn.

## Remaining instability, in Autoware's arm

The GPU arm is now stable across repeats (exe_time 7.84-9.10 ms, NVTL within
0.007, 100 % publish rate). Autoware's is not: one run in three degraded to
**43.53 ms, 12.54 iterations, 177.6 % CPU and 287 poses**, against ~20 ms and
~1200 poses for the other two. NVTL stayed at 4.596-4.599 throughout, so the
gate was not the cause.

This is unexplained and single-run, so it should not be read as a defect in
Autoware's matcher without further work — but it is why the comparison above
uses medians, and why a single run of either matcher is not evidence.

## NVTL is not comparable across implementations

GPU sits at ~2.80, Autoware at ~4.60. This is **not** a quality gap. NVTL
scales with `ndt.resolution` and with how far returns are cropped, and the two
implementations do not compute it over the same neighbourhood. Both clear the
2.0 convergence gate. Per `docs/guides/ndt-tuning.md`, NVTL gates convergence —
it does not rank quality, and it inverted the conclusion twice in July. Compare
NVTL only between runs of the *same* matcher.

## Verdict

**Keep the CUDA path on Orin.** On this evidence it is worth its maintenance:

- it removes NDT as a CPU bottleneck — 23 % of a core against 119 %;
- it leaves the GPU ~94 % idle, so it does not compete with perception;
- it publishes a pose for every alignment, where Autoware dropped to 23 % of
  alignments in one run of three;
- it is ~2.5x faster per alignment, though neither matcher misses the budget
  here, so that is margin rather than recovered scans;
- power is roughly neutral: +208 mW on the GPU rail, −325 mW on the CPU rail.

The costs are **3x the resident memory** (626-740 MB against 208-241 MB) and a
CUDA/CubeCL/cudarc toolchain with a Tegra-specific pin (`cuda-12050`,
`CUDA_ARCH=87`) that has already cost this project several days.

The `NDT_USE_GPU=0` arm should be kept only as an offline reference, and
labelled as one. As a runtime option it is a trap: it looks like a CPU control
for the GPU path, and in real time it measures a feedback collapse instead.

## Follow-ups, in priority order

1. **Explain the degraded Autoware run** — 43.5 ms and 287 poses out of 1231
   alignments, once in three.
2. **Decide the CPU arm's fate**: make it offline-only, or give it the
   parallelism its `num_threads` parameter already promises.
3. **Surface inert parameters.** `ndt.num_threads` is ignored entirely, and
   `ndt.step_size` was ignored on the GPU line-search path until this branch. A
   parameter that silently does nothing is worse than one that is absent.
4. **Decide whether to close the GPU's earlier stop.** The desktop work left
   this to the Orin measurement, priced at 0.02 % of score and a median 6 mm of
   pose. On these numbers it is not worth spending GPU time on: the GPU arm
   already publishes 100 % of alignments at 23 % CPU, and 6 mm is far inside
   the variation between repeats.

## Reproducing

```bash
sudo jetson_clocks && sudo jetson_clocks --show   # pin first, or you measure the governor
just demo bench "gpu cpu autoware" 3
just demo bench-report <bench_dir>/runs.tsv
python3 scripts/testing/localization/tegrastats_summary.py <run_dir>/tegrastats.log
```

Raw runs: `tmp/demo-runs/bench_20260804_053700`.

On a fresh machine: `just build` aborts on `zed_components` without the ZED
SDK, so `--continue-on-error` is needed, and `zed_wrapper` must exist at all
because `sensor_kit.xacro:4` includes its xacro unconditionally. Autoware 1.5.0
must be sourced explicitly — `.bashrc` points at `/opt/autoware/autoware-env`,
which does not exist. And after any submodule update, **rebuild before
benchmarking**: `--symlink-install` does not symlink files that did not exist at
build time, and a launch file added by someone else will fail every run with a
`FileNotFoundError` that looks nothing like a benchmark problem.
