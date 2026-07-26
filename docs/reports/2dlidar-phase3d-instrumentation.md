# Phase 3d: MCL instrumentation and the underflow-vs-misspecification verdict

Phase 3d built the observation apparatus for the divergence diagnosed across
Phases 3b/3c: five tasks producing a diagnostics module, live debug topics,
an offline chart suite, a live likelihood-field render, and (this report's
subject) two sensor-model chart families plus a from-scratch, log-space
"frozen scan" likelihood field. Fixing anything is explicitly out of scope
(that is Phase 3e); this document ties the six artifacts together, states
what each one shows and which defect from
[`2d_mcl_algorithm.md`](../research/localization/2d_mcl_algorithm.md) §5 it
makes visible, and settles the one question the brief called out as
deciding Phase 3e's priorities.

All work here is against one real run (`data/rosbags/phase3/sample_ndt_gt`,
the sample Autoware site) and one PF configuration. Every number in this
report is **n=1** — see [§7](#7-the-n1-caveat) before treating any of it as
more than "this is what happened on this run."

---

## 1. Task 1 — per-update diagnostics (JSONL)

**What it is:** `particle_filter/diagnostics.py` (fork, branch `autosdv`) adds
`effective_sample_size`, `weight_entropy`, `pose_covariance`,
`beam_categories`, and a `DiagnosticsRecorder` that appends one JSON record
per MCL update (or every `diag_every`-th) to a JSONL file when
`diag_enable:=true`. Schema (see `tmp/mcl_diag_t1.jsonl` for a real
example): `iter`, `stamp_scan`, `stamp_wall`, `dt_update`, `action_dx/dy/
dtheta`, `n_eff`, `weight_entropy`, `weight_max`, `pose_x/y/theta`,
`cov_xx/yy/xy`, `resampled`, `frac_hit/short/long/clamped/nonfinite`,
`t_propose/motion/sensor/norm`.

**Validated run:** 860 records total, 828 with beam-category fractions
populated (the first ~32 updates run before the first scan arrives, so
`frac_*` and `t_sensor` are not yet meaningful — `lidar_initialized` gates
the sensor step, not the odometry-driven `update()` call).

**What it shows / §5 link:** this is the raw material for everything below
it. On its own, one record already demonstrates §5.3 (`update()` fires from
`odomCB`, so `dt_update` is far shorter than the ~100 ms scan period) and
§5.5/§5.2 (`frac_nonfinite` ~0.30, `frac_clamped` ~0.00 — no-return beams
are common and land in the max-range bucket, not literally clamped from
over-range since `PF_MAX_RANGE=SCAN_RANGE_MAX=60` here).

**Regenerate:**
```bash
PF_DIAG_ENABLE=true PF_DIAG_PATH=./tmp/mcl_diag.jsonl PF_DIAG_TOPICS=true \
BAG=data/rosbags/phase3/sample_ndt_gt GT_BAG=data/rosbags/phase3/sample_ndt_gt \
MAP_YAML=data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
SCAN_MIN_HEIGHT=1.91611 SCAN_MAX_HEIGHT=2.21611 SCAN_RANGE_MAX=60.0 \
PF_MAX_RANGE=60.0 IMU_TOPIC=/sensing/imu/tamagawa/imu_raw IMU_YAW_SIGN=-1.0 \
bash scripts/2dlidar/run-particle-filter.sh
```

---

## 2. Task 2 — live scalar topics + script passthrough

**What it is:** `diag_topics:=true` publishes the same six scalars
(`n_eff`, `weight_entropy`, `pose_cov_trace`, `update_hz`, `frac_clamped`,
`frac_short`) as `std_msgs/Float32` on `/pf/debug/*`, for live PlotJuggler
inspection during a run instead of post-hoc from JSONL.
`run-particle-filter.sh`'s `PF_DIAG_*` env passthrough writes both the
JSONL and topic flags into `pf_params.yaml`; both default off, so an
unmodified invocation is byte-identical to pre-Phase-3d behaviour (verified
by Task 2's steps).

**What it shows / §5 link:** same data as Task 1, live rather than
offline — useful for watching `n_eff` collapse in real time while tuning,
but the actual divergence analysis in this report uses the JSONL (Task 1)
and the chart suite (Task 3), since those are reproducible artifacts.

**Regenerate:** add `PF_DIAG_TOPICS=true` to any `run-particle-filter.sh`
invocation and `ros2 topic echo /pf/debug/n_eff` (etc.) while it runs.

---

## 3. Task 3 — offline six-panel diagnostics chart

**What it is:** `scripts/2dlidar/plot_mcl_diagnostics.py` reads a Task 1
JSONL and renders `2dlidar-phase3d-mcl-diagnostics.png`, six panels sharing
a time axis:

1. translational error vs GT (when `--gt-bag` timestamps overlap)
2. `N_eff`, the `ess_threshold_ratio * N` line, resample-event rug
3. weight entropy and `weight_max`
4. pose-covariance trace and its xy eigenvalue ratio
5. stacked beam-category fractions (hit/short/long/clamped/nonfinite)
6. per-stage timing and update rate

**What it shows / §5 link:** panel 2 is the direct N_eff evidence for
degeneracy (n_eff min ~1.1-1.7 of N=4000, i.e. the entire particle set is
occasionally carried by ~1 particle); panel 4's eigenvalue ratio is the
§5.4 ridge signature (beam non-independence sharpens the likelihood along
the corridor axis, which shows up as anisotropic covariance, not elevated
trace); panel 5 is the §5.2/§5.5 evidence in aggregate — mid-run beam
categories hit ~0.40 / short ~0.01 / long ~0.27 / clamped ~0.00 / nonfinite
~0.30, i.e. roughly a third of every beam vote is pose-independent
(no-return); panel 6 shows the median 65 ms update of which sensor
evaluation is 15 ms — the 328,000-raycast step from §3.2, not the
dominant cost.

**Regenerate:**
```bash
python3 scripts/2dlidar/plot_mcl_diagnostics.py \
    tmp/mcl_diag.jsonl --gt-bag data/rosbags/phase3/sample_ndt_gt \
    --out docs/reports/assets/2dlidar-phase3d-mcl-diagnostics.png
```

---

## 4. Task 4 — live likelihood field + RViz

**What it is:** `particle_filter.py`'s `build_likelihood_field()`
(`likelihood_field_enable:=true`) evaluates an 81×81 pose grid (default
`lf_window_m=40`, `lf_res_m=0.5`) centred on the current inferred pose,
heading fixed at the inferred theta, through the **same runtime path MCL
itself uses** — `calc_range_repeat_angles` + `eval_sensor_model`
(`range_libc/includes/RangeLib.h:533`), which is a **raw float64 product**
over the ~82-beam per-particle weight:

```cpp
weight = 1.0;
for (j = 0; j < rays_per_particle; ++j) {
    ...
    weight *= sensor_model[(int)r][(int)d];
}
```

The result is published as an `OccupancyGrid` on
`/pf/debug/likelihood_field` (log-weight, max-subtracted, clipped at
`-lf_log_floor` = 20 nats, linearly mapped to 0..100), visualised via
`scripts/2dlidar/rviz/mcl_debug.rviz`.

**What it shows:** the two committed renders
(`2dlidar-phase3d-likelihood-field-{tracking,divergence}.png`) are the
reason Part B of this task exists. At t=9.8s (tracking well, ~0.4 m error)
and t=23.3s (approaching divergence), the raw-weight grid is at the
`-20`-nat display floor (black) essentially everywhere **including at the
true pose**, with the single visible maximum sitting ~19-20 m away from
truth, at the same map location in both frames despite the vehicle having
moved and the true error being small at t=9.8s. Read at face value this
looks exactly like §5.4 (beam non-independence sharpens the likelihood
into an aliased ridge/spike far from truth) — but the raw product's
float64 accumulation is *also* a plausible source of exactly this look
(any beam-count product this small underflows silently), which is why the
brief made settling which explanation actually holds Part B's priority.
**§4 below is that answer.**

**Regenerate:**
```bash
PF_LIKELIHOOD_FIELD=true PF_LF_WINDOW_M=40.0 PF_LF_RES_M=0.5 PF_LF_PERIOD_S=1.0 \
PF_LF_LOG_FLOOR=20.0 PF_DIAG_TOPICS=true \
BAG=data/rosbags/phase3/sample_ndt_gt GT_BAG=data/rosbags/phase3/sample_ndt_gt \
MAP_YAML=data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
SCAN_MIN_HEIGHT=1.91611 SCAN_MAX_HEIGHT=2.21611 SCAN_RANGE_MAX=60.0 PF_MAX_RANGE=60.0 \
PF_SQUASH=3.0 PF_DISP_THETA=0.1 IMU_TOPIC=/sensing/imu/tamagawa/imu_raw IMU_YAW_SIGN=-1.0 \
bash scripts/2dlidar/run-particle-filter.sh
# then: RViz with scripts/2dlidar/rviz/mcl_debug.rviz, or record
# /pf/debug/likelihood_field and render offline.
```

---

## 5. Part A — sensor-model charts

**What it is:** `scripts/2dlidar/plot_sensor_model.py` rebuilds the
`(table_width, table_width)` sensor-model lookup table with the *same*
four-component-mixture arithmetic as `precompute_sensor_model`
(`particle_filter.py:459-506`; cross-checked against a literal nested-loop
port in `test_plot_sensor_model.py`), and renders:

1. **`sensor-model-columns`** — normalised `P(r|d)` for `d = 10/20/50 m`.
   Shows the Gaussian hit peak at `r=d` and the short-reading ramp shoulder
   for `r<d` widening and flattening the peak as `d` grows.
2. **`sensor-model-mixture-mass`** — the §5.1 effective-mixture-weight
   table as a stacked chart: effective `z_hit` falls from 25.4% (`d=10m`)
   to 15.2% (`d=20m`) to 6.8% (`d=50m`), against a *configured* `z_hit` of
   75%, because `z_short`'s ramp is unnormalised before mixing.
3. **`sensor-model-noreturn-ratio`** — `P(no-return)/P(perfect match)` vs
   `d`; crosses 1.0 well before `d=20m` (1.87× at exactly 20 m, per the
   doc's §4 worked example) — the §5.2 inversion, a beam that sees nothing
   outscores one that matches exactly.
4. **`sensor-model-resolution-coupling`** — (2) and (3) at 0.05 m and
   0.10 m map resolution side by side: `sigma_hit` is fixed in *pixels*
   (8.0 px), so halving the map's cell size halves the metric matching
   width and reshapes both curves — the resolution/model coupling flagged
   at the end of §5.1.

**§5 link:** directly §5.1 and §5.2; the resolution-coupling panel is why
every Phase 3c lever comparison that also changed map resolution
(Lever 2) had two effects confounded in one result.

**Regenerate:**
```bash
python3 scripts/2dlidar/plot_sensor_model.py \
    --resolution 0.05 --max-range 60.0 --out-dir docs/reports/assets
```

---

## 6. Part B — frozen-scan log-space likelihood field, and the verdict

### 6.1 What it is

`plot_sensor_model.py --frozen-scan <gt-bag> --map <yaml> --time <rel_s>
--label <name>` reconstructs one scan **exactly** as
`run-particle-filter.sh`'s pipeline would have (pointcloud_to_laserscan's
z-band filter and azimuth-binning loop, replicated beam-for-beam in
`build_scan_ranges` against points transformed sensor→base_link via the
bag's `/tf_static` chain — full 3D, not the planar approximation the Part
A scan-accumulation grid uses, since the z-band boundary itself is what's
being filtered on; then particle_filter's `angle_step=18` decimation via
`decimate_scan`), evaluates a fine pose grid (default 60 m / 0.1 m = 601×601
poses, heading fixed at the GT yaw) against the map through `range_libc`
directly (`PyOMap` + `PyCDDTCast`, the same classes `get_omap()` uses), and
accumulates the per-beam sensor-model lookups as a **sum of logs**
(`frozen_field_grid`) — never a product. A sum of ~82 finite log-terms
cannot silently underflow to zero the way `eval_sensor_model`'s float64
product can; that's the entire point of Part B existing as a *second,
independent* evaluation path alongside Task 4's live one.

Two frames were rendered, matched to Task 4's live-render timestamps by
using `sample_ndt_gt`'s own bag-recording start as the `--time` origin
(confirmed to reproduce Task 4's captions to the recorded second: `--time
9.82` → scan stamp rel `10.009s`≈"t=9.8s"; `--time 23.35` → `23.282s`≈"t=23.3s"):

```bash
bash -c 'source /opt/autoware/1.5.0/setup.bash && python3 \
    scripts/2dlidar/plot_sensor_model.py \
    --frozen-scan data/rosbags/phase3/sample_ndt_gt \
    --map data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
    --time 9.82 --label tracking --out-dir docs/reports/assets'

bash -c 'source /opt/autoware/1.5.0/setup.bash && python3 \
    scripts/2dlidar/plot_sensor_model.py \
    --frozen-scan data/rosbags/phase3/sample_ndt_gt \
    --map data/sample-rosbag-replay/sample-map-rosbag/occupancy_grid_scanaccum_mh1r05.yaml \
    --time 23.35 --label divergence --out-dir docs/reports/assets'
```

`--window-m`/`--field-res-m` default to 60.0/0.1 (the brief's suggested
values); `--min-height`/`--max-height` default to 1.91611/2.21611 (the
sample-site z-band); `--z-hit`/`--z-short`/`--z-max`/`--z-rand`/
`--sigma-px` default to the production values (0.75/0.01/0.07/0.12/8.0);
map resolution (0.05 m) is read from the `.yaml`, not passed separately.

### 6.2 THE VERDICT: (A), decisively — not (B)

The brief posed two competing explanations for Task 4's renders (raw-weight
grid at the -20-nat display floor everywhere including at the true pose,
maximum ~19-20 m away at the window edge, in both the tracking and
divergence frames):

> (A) the measurement model is genuinely mis-specified, so its global
> maximum is far from truth; (B) float64 underflow, with the surviving
> bright pixels an artifact of which poses happened not to underflow.

**Frozen-scan numbers (log-space, underflow-free, 601×601=361,201 poses,
60 m window):**

| | tracking (t=9.82s) | divergence (t=23.35s) |
|---|---|---|
| log-likelihood at GT pose | -566.04 nats | -546.49 nats |
| log-likelihood at grid argmax | -518.52 nats | -495.97 nats |
| **GT→argmax gap** | **47.53 nats** | **50.53 nats** |
| argmax distance from GT | 29.39 m | 30.37 m |
| worst pose in the entire grid | -688.26 nats (122.22 nats below GT) | -669.22 nats (122.72 nats below GT) |
| raw float64 product at GT | 1.48e-246 | 4.58e-238 |
| raw float64 product at argmax | 6.46e-226 | 4.02e-216 |
| raw float64 product, worst pose in grid | 1.24e-299 | 2.31e-291 |
| **grid poses whose raw product underflows to exactly 0.0** | **0 / 361,201 (0.0000%)** | **0 / 361,201 (0.0000%)** |

**This settles it as (A), not (B).** Not one pose in either 361,201-pose
grid — including the single worst pose in the entire 60 m window, at
~1.2-2.3e-299 — actually reaches float64's underflow floor (denormal
minimum ≈4.9e-324; normal minimum ≈2.2e-308). Hypothesis B predicted the
bright survivors in Task 4's renders were "an artifact of which poses
happened not to underflow" — but computed exactly, in log space, over a
window that is a strict superset of Task 4's 40 m grid, **nothing
underflows anywhere in it**. The mechanism in B does not fire on this data.

What *is* real: a 47.5-50.5 nat gap between the true pose and the grid's
best-scoring pose (`exp(47.5) ≈ 4×10^20`, `exp(50.5) ≈ 7×10^21` — the true
pose is that many times less likely than the best decoy under this exact
model, in exact arithmetic, no floating-point involved). The two rendered
fields (`2dlidar-phase3d-frozen-field-{tracking,divergence}.png`) show
*why*: the underlying surface has real spatial structure — raycast-shadow
texture from actual map geometry (the corridor curve, the parked-vehicle
blobs, wall edges), not underflow noise (which would look like uniform
black with scattered, uncorrelated bright single pixels). The GT pose sits
in a comparatively dark stretch; a specific patch of open ground ~30 m to
the northeast scores better under the beam model on both frames — the same
location both times, which is itself further evidence this is a stable
property of the map/model pairing, not a per-frame numerical accident.

**Why Task 4's live renders still look like near-total blackout, then:**
the 47.5-50.5 nat GT→argmax gap this report measures is more than double
Task 4's `lf_log_floor=20.0` nats visualisation clip. Any pose scoring
worse than 20 nats below the *local* grid's own maximum is display-floored
to black regardless of its actual (non-zero, non-underflowed) value — so a
40 m/0.5 m grid whose true dynamic range is 47+ nats will show flat black
almost everywhere by construction of the colour mapping, with detail only
near the peak. That is a visualisation-resolution artifact of Task 4's
fixed 20-nat floor, not a numerical-underflow artifact of its raw-product
math. Both readings agree the model places its maximum ~19-30 m from
truth; Part B additionally proves that placement is a genuine property of
the (mis-specified) model, computed exactly, not a float64 accident.

**Quantified: A vs B contribution.**
- **(A) mis-specification: the entire effect.** The GT→argmax gap and its
  location are reproduced identically in exact log-space arithmetic; there
  is no underflow anywhere in the grid to attribute any part of it to.
- **(B) underflow: zero measured contribution to *this* effect.** It does
  not occur in this window at this scan. (Underflow is not fictional in
  general — `eval_sensor_model`'s raw product for the full 82-beam,
  4000-particle MCL step, or for a wider/finer frozen grid than tested
  here, could still hit it on other frames; it simply isn't what produced
  Task 4's renders.)
- The only real contribution "B-shaped" reasoning had was in the
  *visualisation*: Task 4's fixed `-20`-nat floor makes a genuine-but-only
  47-50-nat-wide mis-specification gap look like total collapse. That is a
  rendering-parameter artifact, not a floating-point one — the fix (if it
  mattered) would be raising `lf_log_floor`, not touching the math.

### 6.3 §5 link

Directly §5.4 (beam non-independence: 82 beams along/across a corridor
sharpen the likelihood surface's real spatial texture into confident,
occasionally wrong, peaks) and §5.1/§5.2 (the mixture that produces each
per-beam term in the first place). The frozen field is the clearest visual
evidence in this phase for §5.4: it is not a diffuse blob around the truth
tapering off with distance, it is a textured surface with a competing
strong mode tens of metres away.

---

## 7. The n=1 caveat

Every number in this report — Task 1's diagnostics, Task 3's chart, Task
4's live renders, and Part B's frozen fields — comes from **one run
against one bag** (`data/rosbags/phase3/sample_ndt_gt`) with **one PF
configuration**. Phase 3c's variance investigation (Lever 3) already
established that this pipeline's run-to-run variance is large enough that
a single seed cannot be treated as characteristic of the method; the
`docs/superpowers/plans/2026-07-26-2dlidar-phase-3d-instrumentation.md`
brief's own scope note repeats this explicitly: fixing any §5 defect is
Phase 3e's job, and Phase 3e's evaluations must use **N ≥ 5 seeds**, not
one. Concretely for Part B: the 47.5/50.5 nat GT→argmax gaps, the specific
~30 m-northeast decoy location, and the "0 underflow" result are this
scan's numbers, on this map, at these two moments; they establish *that
mis-specification alone reproduces Task 4's headline symptom on real data*
(the priority question this task existed to settle), not a distribution
of how large that gap typically is, or whether some other frame in some
other run would in fact reach the underflow floor.

---

## 8. What Phase 3e should fix first

Ordered by the evidence this phase collected, strongest first:

1. **§5.1, unnormalised `z_short`.** Part A's mixture-mass chart shows the
   configured 75% `z_hit` degrades to an *effective* 6.8-25.4% across the
   10-50 m range actually seen in this data (median scan range 19.7 m,
   p90 49.3 m per `2d_mcl_algorithm.md` §1.1) — the model spends most of
   its mass on a term (`z_short`'s ramp) that was never meant to dominate.
   This is upstream of §5.2 and §5.4 both: fixing the normalisation
   changes every beam's contribution to the sum Part B measured, and is a
   contained, one-function change (`precompute_sensor_model`'s short-ramp
   term).
2. **§5.2, no-return beams outscoring matches.** The no-return ratio chart
   (Part A, panel 3) crosses break-even below `d=20m` and reaches 1.87× at
   exactly 20 m — with `frac_nonfinite` measured at ~0.30 mid-run (Task 1),
   this is not an edge case, it is roughly a third of every beam vote
   actively rewarding wrong hypotheses that predict empty scans. Directly
   implicated in Part B's finding: some fraction of the 47.5-50.5 nat gap
   is beams near the argmax decoy scoring well specifically *because*
   they returned nothing there.
3. **§5.4, beam correlation / the ridge.** Part B is now the strongest
   available evidence for this: the frozen field's texture is real map
   structure, sharpened into a confident wrong peak by treating 82
   correlated beams as independent votes. Likely downstream of fixing (1)
   and (2) rather than a prerequisite — re-run Part B's frozen-scan render
   after each of the above to see whether the ~30 m decoy mode shrinks,
   moves, or persists, using it as a regression check on the fix.
4. **§5.3, double-counting via `odomCB`-triggered updates.** Real (Task 1's
   `dt_update` distribution shows updates far more frequent than the
   ~100 ms scan period), but orthogonal to the argmax-location question
   Part B settled — lower priority unless (1)-(3) alone don't close the
   Phase 3c divergence gap.
5. **§5.5, `max_range` clamping discards long beams.** Already partially
   mitigated in current runs (`PF_MAX_RANGE=SCAN_RANGE_MAX=60m`, up from
   the original 30 m); revisit only if raising it further remains cheap
   relative to `precompute_sensor_model`'s `O((range/res)^2)` startup cost.

All four remaining defects should be evaluated with **N ≥ 5 seeds** per
§7, using Task 3's chart suite (for N_eff/error-over-time) and Part B's
frozen field (for argmax-location regression) as the before/after
instruments this phase built for exactly that purpose.
