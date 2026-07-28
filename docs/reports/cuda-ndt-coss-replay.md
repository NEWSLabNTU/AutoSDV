# CUDA NDT on the COSS map: why it did not hold a fix

Date: 2026-07-28. Bag: `data/rosbags/outdoor_20251226_153115` (157 s, VLP32C +
ublox + ZED IMU). Map: `data/COSS-map-planning` (4.9 M points, 130 x 75 x 21.6 m,
a campus park: grass, scattered trees, buildings around the edge).
Stack: `logging_simulation.launch.yaml`. Harness:
`scripts/testing/localization/run-ndt-replay.sh`, analysis
`scripts/testing/localization/summarize_ndt_run.py`.

The bag is parked for its first 115.7 s and drives for the last 41.3 s at up to
1.58 m/s, covering about 20 m of ground. "init" below means t < 115.7 s and
"track" means t >= 115.7 s.

**Headline: one defect explains the localization failure** -- nothing published
the IMU transform in logging simulation, so the EKF never propagated and NDT was
handed a stale prior on every frame. Three further defects were found alongside
it (a wrong diagnostic formula, a debug binary shipped by `just build`, and a
wheel speed roughly 1.8x too high). Two tuning changes that looked like fixes
were measured and rejected; see "What did not turn out to be the problem".

## What the runs showed

| metric | as shipped | built-in NDT | + IMU TF | all fixes, release |
|---|---|---|---|---|
| published poses | 330 | 571 | 883 | **1425 / 1425** |
| NVTL init / track | 2.24 / 2.18 | 4.59 / 4.38 | 2.33 / 2.10 | 2.90 (res 2.0) |
| frames rejected on score | 995 | 0 | 446 | **4** |
| transform_probability | 0.000 | 10.65 | 0.000 | **correct** |
| iterations, init | 7.6 | 11.4 | 2.9 | 2.4 |
| init to result distance, init | 0.294 m | 0.522 m | 0.061 m | **0.070 m** |
| exe_time mean | 83 ms | 11 ms | 73 ms | **11 ms** |
| worst publish gap | 15.0 s | 9.7 s | 3.3 s | **0.115 s** |

The bag's ublox fix is not usable as ground truth: `status: 0` (single point, no
RTK -- `/sensing/gnss/ntrip/rtcm` carries 0 messages), reported altitude swings
32 m to 69 m against a map ground plane at z = 9.3 m, horizontal scatter is
+/-20 m, and it disagrees about the direction of travel (GNSS moves in x, the
vehicle in y). See "Initial pose".

## 1. Nothing published the IMU transform in logging simulation (root cause)

`imu_corrector` logged `Please publish TF base_link to zedxm_imu_link` 15320
times, once per IMU message, and `gyro_odometer` published **zero** twist
messages. The recorded TF tree has no `zedxm_left_camera_frame ->
zedxm_imu_link` edge.

For `sensor_suite:=vlp32c_zed_imu` the suite sets `camera_model=none`, which
skips `camera.launch.xml`'s ZED group and with it `zed_tf_only.launch.xml` --
the file written for exactly this case, but gated on `camera_model=='zedxm'`.
The other publisher, `zed_imu_only.launch.xml`, only starts when `launch_driver`
is true, and logging simulation runs with `launch_sensing_driver:=false`.

With no twist the EKF does not propagate: it moved 0.5 m while the vehicle moved
~20 m. NDT is then handed that frozen pose as its initial guess on every frame,
the guess falls further behind reality, the match degrades, NVTL decays
(2.36 -> 2.07), frames start failing the 2.3 convergence gate, publishing stops,
and the prior gets staler still. Everything downstream of this -- the low NVTL,
the rejected frames, the 15 s publish gaps -- is that loop.

Fixed in `autosdv_sensor_kit_launch/launch/imu.launch.xml`. Publishing the
transform alone lifts published poses from 330 to 883 and drops init iterations
from 7.6 to 2.9.

## 2. `transform_probability` used the wrong formula

`cuda_ndt_matcher/src/node/processing.rs` recomputed it as `exp(-score/10)`
rather than Autoware's `total_score / num_source_points`. NDT scores here are
large and positive, so the exponential underflowed and the published diagnostic
was identically 0.000 in every run. The solver already computes the correct
value (`ndt_cuda/src/optimization/solver.rs`, surfaced as
`result.transform_probability`); it was being discarded. Inert while
`converged_param_type: 1` gates on NVTL, fatal with `converged_param_type: 0`,
which would have rejected every frame. Now reads 10.96, against built-in NDT's
10.65 on the same data.

## 3. `just build` installed a debug binary

`--cmake-args -DCMAKE_BUILD_TYPE=Release` does not reach Rust packages, and the
top-level build never passed `--cargo-args --release`, so
`install/cuda_ndt_matcher` was `target/debug/` (190 MB, versus 9.9 MB for
release). This is why the matcher took 83 ms per scan against the ~5 ms the
package documents. Only the package's own justfile built release, so nothing
that ran through `just launch` ever used it.

## 4. The wheel speed is roughly 1.8x too high

Over the drive the hall sensor integrates to **35.9 m** while the map says the
vehicle covered about **20 m**. The EKF therefore predicts too far between
scans, NDT drags the pose back on every frame, and the result is visible
lurching -- "goes off the rail a bit, then snaps back".

Replaying with the speed halved (`tmp/velocity_scaler.py`, since the wrong
numbers are baked into the bag and no parameter change can undo them):

| | stock | scale 0.5 |
|---|---|---|
| wheel distance fed to EKF | 35.9 m | 17.9 m |
| NDT distance (1 Hz, scatter-suppressed) | 22.3 m | 20.6 m |
| per-frame position scatter | 0.271 m | **0.096 m** |
| init to result distance, moving | 0.819 m | **0.282 m** |
| init to result distance, parked | 0.024 m | 0.028 m |
| NVTL moving | 4.827 | 4.835 |

NVTL is unchanged, so the map matching was never at fault -- only the prediction
handed to it. At rest, prediction and result already agree to 2 cm.

`params/velocity_report.yaml` has `wheel_diameter: 10.5` cm and
`markers_per_rotation: 12`. A hall sensor triggering on **both** edges of 12
markers yields 24 counts per revolution and exactly doubles the reported speed;
a wheel diameter wrong by 2x is not plausible for a measured quantity. **This is
a vehicle-side fix and is not yet made.** Confirm by pushing the vehicle a
tape-measured 10 m and integrating `/vehicle/status/velocity_status`.

Related: `steering_status` is identically 0.000 rad for the whole bag, so
nothing in this recording can be checked against steering.

## What did not turn out to be the problem

Both of these looked like fixes mid-investigation and were reverted after
measurement. Recorded here because both are easy to re-derive and wrong.

### Voxel resolution (`resolution: 2.0` vs `4.0`)

Raising it to 4.0 restored publishing on the *broken* stack, so it looked like a
fix. With the IMU transform fixed, a 2x2 over resolution and crop -- everything
else held constant, 2000 points into NDT in every run -- shows 2.0 is better
everywhere:

| | res 2.0 | res 4.0 |
|---|---|---|
| crop +/-20 m | scatter **0.020 m**, yaw p95 1.12 deg, i2r 0.154, NVTL 2.898 | scatter 0.106 m, yaw p95 2.03 deg, i2r 0.282, NVTL 4.837 |
| crop +/-60 m | scatter **0.014 m**, yaw p95 1.18 deg, i2r 0.131, NVTL 2.722 | scatter 0.032 m, yaw p95 1.12 deg, i2r 0.151, NVTL 4.653 |

Coarser voxels localise less precisely: 5x the scatter at +/-20 m. They *look*
better only through NVTL, which scales with voxel size and therefore buys margin
over the 2.3 gate without buying accuracy. **NVTL gates convergence; it does not
rank quality.** Tuning on it inverts the answer.

Corollary for diagnosis: an NVTL sitting near the gate points at the pose prior,
not at the voxel size. On the broken stack NVTL was 2.22; with the same
resolution and a healthy prior it is 2.90.

### Measurement-range crop (the "wider range makes NDT worse" effect)

Field experience had been that widening the crop degraded NDT, so it was tuned
to +/-20 m. On the fixed stack at res 4.0, widening helps a lot (scatter 0.106
-> 0.032, yaw p95 2.03 -> 1.12 deg, iterations 5.0 -> 3.6). At the shipped
res 2.0 the effect is much smaller, because the pose is already precise -- the
large crop sensitivity was an artefact of the coarse resolution:

| crop, at res 2.0 | scatter | yaw med | yaw p95 | i2r mean | NVTL | iters |
|---|---|---|---|---|---|---|
| +/-20 m | 0.020 m | 0.30 deg | 1.12 deg | 0.154 | 2.898 | 5.0 |
| **+/-40 m** | **0.012 m** | 0.32 deg | **1.07 deg** | **0.129** | 2.752 | 4.1 |
| +/-60 m | 0.014 m | 0.33 deg | 1.18 deg | 0.131 | 2.722 | 4.0 |

+/-40 m is the best of the three on scatter, yaw p95 and prediction correction,
and is now the default. It keeps 0.45 of NVTL margin over the 2.3 gate and
stays inside the well-mapped area: past 60 m only 18 % of returns have any map
to match against, and the map's ground is sound to 40 m (see "Map quality").

So the direction of the original finding was real, but its cause was the
resolution, not the range.

## Initial pose

No COSS initial pose exists anywhere in this repository -- the only concrete
`user_defined_initial_pose` is the Autoware sample-map value in
`ndt_replay_simulation.launch.xml`, disabled by default, and every recorded run
under `play_log/` shows `enable: false`. COSS has always leaned on GNSS
auto-init, which given the fix quality above lands up to 20 m off and with the
wrong heading; successive runs from the same bag initialised 5 m apart and
produced net displacements between 19 m and 34 m.

The operator set the pose by hand in RViz on 2026-07-28 and confirmed the point
cloud fits the map:

```
/initialpose    x=-1.839  y=-8.280  qz=0.9992  qw=0.0397   (yaw ~175.4 deg)
/initialpose3d  x= 0.117  y=-8.350  z=9.312  qz=0.9907  qw=0.1359  (after NDT align)
```

`tmp/seed_initialpose.py` republishes it so a replay does not need a human.
Worth promoting into a launch default once a second operator confirms it.

## Map quality

Scan-to-map nearest-neighbour distance, 8 observer poses spread along the drive,
split by height above the per-cell ground:

| shell | ground <0.5 m | low veg 0.5-2 m | high >2 m |
|---|---|---|---|
| 0-10 m | 0.255 | 0.440 | 0.648 |
| 10-20 m | 0.194 | 0.467 | 0.256 |
| 20-30 m | 0.245 | 0.564 | 0.460 |
| 30-40 m | 0.307 | 0.508 | 0.303 |
| 40-60 m | 0.713 | 0.616 | 0.640 |

Ground geometry is sound to 40 m. Vegetation disagrees by 0.44-0.62 m at *every*
range, near field included -- unsurprising for a park, and another reason not to
read quality off raw NVTL.

For the 56 ground cells beyond 25 m seen by at least 3 different poses, the
residual is coherent across observers (median coherence 0.96, inter-pose spread
0.049 m) with median magnitude 0.237 m and p90 0.828 m. A displacement every
vantage point agrees on is a property of the map, not of the observation or of
moving foliage: the map is mildly warped at its edges, at the decimetre scale
rather than the metre scale. Not a limiting factor at the current crop.

Beyond 60 m the map simply ends: 17.9 % of returns at 60-80 m and 1.9 % beyond
80 m fall on any mapped cell.

## Where it stands

On the fixed stack, with the operator's initial pose: NDT publishes every frame
for the whole bag (worst gap 0.115 s), per-frame scatter 0.020 m, frame-to-frame
yaw step median 0.30 deg, and NDT agrees with the gyro on total yaw over the
drive to **0.35 deg** (-4.88 vs -4.53). Absolute accuracy remains unestablished:
this bag has no trustworthy reference.

Still open:

- The wheel-speed scale (finding 4) has to be fixed on the vehicle.
- **Speed.** 11 ms mean per scan after the release fix. Another process held
  26 GB of this machine's 32 GB GPU throughout the session, so every timing here
  is contended and should be re-measured on an idle GPU before being compared
  with the 199.7 Hz the package documents for an RTX 5090.
- A ground-truth trajectory for COSS, without which "how accurate" cannot be
  answered.

## Reproducing

```bash
scripts/testing/localization/run-ndt-replay.sh verify        # cuda_ndt
POSE_SOURCE=ndt scripts/testing/localization/run-ndt-replay.sh builtin
python3 scripts/testing/localization/summarize_ndt_run.py <run_dir>
```

The one-off harnesses behind findings 4, the 2x2 and the map analysis live in
`tmp/` (`coss-scaled-replay.sh`, `crop-sweep.sh`, `velocity_scaler.py`,
`seed_initialpose.py`, `map_warp_check.py`, `residual_by_range.py`,
`compare_runs.py`); promote them if this becomes routine work.
